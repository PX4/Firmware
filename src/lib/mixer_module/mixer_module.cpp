/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "mixer_module.hpp"

#include <lib/mixer/MultirotorMixer/MultirotorMixer.hpp>

#include <uORB/Publication.hpp>
#include <px4_platform_common/log.h>

using namespace time_literals;

static constexpr unsigned n_act = actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS;
static constexpr unsigned n_out = output_control_s::NUM_OUTPUT_CONTROL_GROUPS;


MixingOutput::MixingOutput(uint8_t max_num_outputs, OutputModuleInterface &interface,
			   SchedulingPolicy scheduling_policy,
			   bool support_esc_calibration, bool ramp_up)
	: ModuleParams(&interface),
	  _control_subs{
	{&interface, ORB_ID(actuator_controls_0)},
	{&interface, ORB_ID(actuator_controls_1)},
	{&interface, ORB_ID(actuator_controls_2)},
	{&interface, ORB_ID(actuator_controls_3)},
	{&interface, ORB_ID(actuator_controls_4)},
	{&interface, ORB_ID(actuator_controls_5)},
	{&interface, ORB_ID(output_control_ca)},
	{&interface, ORB_ID(output_control_mavlink)},
	{&interface, ORB_ID(output_control_mc)},
	{&interface, ORB_ID(output_control_fw)},
	{&interface, ORB_ID(output_control_gimbal)},

},
_output_module_prefix(interface.get_param_prefix()),
_scheduling_policy(scheduling_policy),
_support_esc_calibration(support_esc_calibration),
_max_num_outputs(max_num_outputs < MAX_ACTUATORS ? max_num_outputs : MAX_ACTUATORS),
_interface(interface),
_control_latency_perf(perf_alloc(PC_ELAPSED, "control latency"))
{
	output_limit_init(&_output_limit);
	_output_limit.ramp_up = ramp_up;

	/* Safely initialize armed flags */
	_armed.armed = false;
	_armed.prearmed = false;
	_armed.ready_to_arm = false;
	_armed.lockdown = false;
	_armed.force_failsafe = false;
	_armed.in_esc_calibration_mode = false;

	px4_sem_init(&_lock, 0, 1);

	// Enforce the existence of the test_motor topic, so we won't miss initial publications
	test_motor_s test{};
	uORB::Publication<test_motor_s> test_motor_pub{ORB_ID(test_motor)};
	test_motor_pub.publish(test);
	_motor_test.test_motor_sub.subscribe();

	// Initialize all control inputs to NaN
	for (auto &control : _output_controls) {
		for (unsigned i = 0; i < output_control_s::MAX_ACTUATORS; i++) {
			control.value[i] = NAN;
		}
	}
}

MixingOutput::~MixingOutput()
{
	perf_free(_control_latency_perf);
	delete _mixers;
	px4_sem_destroy(&_lock);
}

void MixingOutput::printStatus() const
{
	perf_print_counter(_control_latency_perf);
	PX4_INFO("Switched to rate_ctrl work queue: %i", (int)_wq_switched);
	PX4_INFO("Mixer loaded: %s", _mixers ? "yes" : "no");
	PX4_INFO("Lockdown: %d, Manual Lockdown: %d", _armed.lockdown, _armed.manual_lockdown);
	PX4_INFO("Driver instance: %i", _driver_instance);
	PX4_INFO("Output module prefix: '%s'", _output_module_prefix);
	PX4_INFO("Groups required: %d (0x%08x), Groups subscribed: 0x%08x", _groups_required, _groups_required,
		 _groups_subscribed);

	PX4_INFO("Channel Configuration:");

	for (unsigned i = 0; i < _max_num_outputs; i++) {
		int reordered_i = reorderedMotorIndex(i);
		PX4_INFO("Channel %i: input: %f, value: %i, failsafe: %d, disarmed: %d, min: %d, max: %d, Function: %d", reordered_i,
			 (double)_current_inputs[i], _current_output_value[i],
			 _failsafe_value[reordered_i], _disarmed_value[reordered_i], _min_value[reordered_i], _max_value[reordered_i],
			 _assigned_function[i]);
	}
}

void MixingOutput::updateParams()
{
	ModuleParams::updateParams();

	// update mixer if we have one
	if (_mixers) {
		if (_param_mot_slew_max.get() <= FLT_EPSILON) {
			_mixers->set_max_delta_out_once(0.f);
		}

		_mixers->set_thrust_factor(_param_thr_mdl_fac.get());
		_mixers->set_airmode((Mixer::Airmode)_param_mc_airmode.get());
	}

	/** Update Function Mappings */

	updateParamValues("FUNC", _assigned_function);
	updateFailsafeValues();
	updateDisarmedValues();
	updateMinValues();
	updateMaxValues();
	updateTrimValues();
	updateReverseMask();

	// Determine the output_control groups required to subscribe to

	// First, clear all bits associated with the output_control groups
	_groups_required &= (1 << n_act) - 1;

	// Next, use the "_FUNCx" parameters to determine what groups are required
	for (unsigned i = 0; i < _max_num_outputs; i++) {
		int32_t func = _assigned_function[i];

		if (func >= output_control_s::FUNCTION_CA0 &&
		    func <= output_control_s::FUNCTION_CA15) {

			_groups_required |= (1 << (n_act + 0));

		} else if (func >= output_control_s::FUNCTION_MAVLINK_SERVO0 &&
			   func <= output_control_s::FUNCTION_MAVLINK_SERVO7) {

			_groups_required |= (1 << (n_act + 1));

		} else if (func >= output_control_s::FUNCTION_MC_MOTOR1 &&
			   func <= output_control_s::FUNCTION_MC_MOTOR8) {

			_groups_required |= (1 << (n_act + 2));

		} else if (func >= output_control_s::FUNCTION_AILERON1 &&
			   func <= output_control_s::FUNCTION_ENGINE2) {

			_groups_required |= (1 << (n_act + 3));

		} else if (func >= output_control_s::FUNCTION_GIMBAL_ROLL &&
			   func <= output_control_s::FUNCTION_CAMERA_ZOOM) {

			_groups_required |= (1 << (n_act + 4));

		}
	}
}

bool MixingOutput::updateSubscriptions(bool allow_wq_switch, bool limit_callbacks_to_primary)
{
	if (_groups_subscribed == _groups_required) {
		return false;
	}

	// must be locked to potentially change WorkQueue
	lock();

	if (_scheduling_policy == SchedulingPolicy::Auto) {
		// first clear everything
		unregister();
		_interface.ScheduleClear();

		// if subscribed to control group 0 or 1, or control allocator, then move to the rate_ctrl WQ
		const bool sub_group_0  = (_groups_required & (1 << 0));
		const bool sub_group_1  = (_groups_required & (1 << 1));
		const bool sub_group_ca = (_groups_required & (1 << n_act));

		if (allow_wq_switch && !_wq_switched && (sub_group_0 || sub_group_1 || sub_group_ca)) {
			if (_interface.ChangeWorkQeue(px4::wq_configurations::rate_ctrl)) {
				// let the new WQ handle the subscribe update
				_wq_switched = true;
				_interface.ScheduleNow();
				unlock();
				return false;
			}
		}

		bool sub_group_0_callback_registered  = false;
		bool sub_group_1_callback_registered  = false;
		bool sub_group_ca_callback_registered = false;

		// register callback to all required actuator control groups
		for (unsigned i = 0; i < n_act + n_out; i++) {

			if (limit_callbacks_to_primary) {
				// don't register additional callbacks if actuator_controls_0 or actuator_controls_1 are already registered
				if ((i > 1) && (sub_group_0_callback_registered || sub_group_1_callback_registered
						|| sub_group_ca_callback_registered)) {
					break;
				}
			}

			if (_groups_required & (1 << i)) {
				if (_control_subs[i].registerCallback()) {
					PX4_DEBUG("subscribed to [actuator/output]_controls_%d", i);

					if (limit_callbacks_to_primary) {
						if (i == 0) {
							sub_group_0_callback_registered = true;

						} else if (i == 1) {
							sub_group_1_callback_registered = true;

						} else if (i == n_act) {
							sub_group_ca_callback_registered = true;
						}
					}

				} else {
					PX4_ERR("[actuator/output]_controls_%d register callback failed!", i);
				}
			}
		}

		// if nothing required keep periodic schedule (so the module can update other things)
		if (_groups_required == 0) {
			// TODO: this might need to be configurable depending on the module
			_interface.ScheduleOnInterval(100_ms);
		}
	}

	_groups_subscribed = _groups_required;
	setMaxTopicUpdateRate(_max_topic_update_interval_us);

	PX4_DEBUG("_groups_required 0x%08x", _groups_required);
	PX4_DEBUG("_groups_subscribed 0x%08x", _groups_subscribed);

	unlock();

	return true;
}

void MixingOutput::setMaxTopicUpdateRate(unsigned max_topic_update_interval_us)
{
	_max_topic_update_interval_us = max_topic_update_interval_us;

	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (_groups_subscribed & (1 << i)) {
			_control_subs[i].set_interval_us(_max_topic_update_interval_us);
		}
	}
}

void MixingOutput::setAllMinValues(uint16_t value)
{
	for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
		_min_value[i] = value;
	}
}

void MixingOutput::setAllMaxValues(uint16_t value)
{
	for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
		_max_value[i] = value;
	}
}

void MixingOutput::setAllFailsafeValues(uint16_t value)
{
	for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
		_failsafe_value[i] = value;
	}
}

void MixingOutput::setAllDisarmedValues(uint16_t value)
{
	for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
		_disarmed_value[i] = value;
	}
}

void MixingOutput::unregister()
{
	for (auto &control_sub : _control_subs) {
		control_sub.unregisterCallback();
	}
}

void MixingOutput::updateOutputSlewrateMultirotorMixer()
{
	if (!_mixers) {
		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	const float dt = math::constrain((now - _time_last_dt_update_multicopter) / 1e6f, 0.0001f, 0.02f);
	_time_last_dt_update_multicopter = now;

	// maximum value the outputs of the multirotor mixer are allowed to change in this cycle
	// factor 2 is needed because actuator outputs are in the range [-1,1]
	const float delta_out_max = 2.0f * 1000.0f * dt / (_max_value[0] - _min_value[0]) / _param_mot_slew_max.get();
	_mixers->set_max_delta_out_once(delta_out_max);
}

void MixingOutput::updateOutputSlewrateSimplemixer()
{
	if (!_mixers) {
		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	const float dt = math::constrain((now - _time_last_dt_update_simple_mixer) / 1e6f, 0.0001f, 0.02f);
	_time_last_dt_update_simple_mixer = now;

	// set dt for slew rate limiter in SimpleMixer (is reset internally after usig it, so needs to be set on every update)
	_mixers->set_dt_once(dt);
}


unsigned MixingOutput::motorTest()
{
	test_motor_s test_motor;
	bool had_update = false;

	while (_motor_test.test_motor_sub.update(&test_motor)) {
		if (test_motor.driver_instance != _driver_instance ||
		    test_motor.timestamp == 0 ||
		    hrt_elapsed_time(&test_motor.timestamp) > 100_ms) {
			continue;
		}

		bool in_test_mode = test_motor.action == test_motor_s::ACTION_RUN;

		if (in_test_mode != _motor_test.in_test_mode) {
			// reset all outputs to disarmed on state change
			for (int i = 0; i < MAX_ACTUATORS; ++i) {
				_current_output_value[i] = _disarmed_value[i];
			}
		}

		if (in_test_mode) {
			int idx = test_motor.motor_number;

			if (idx < MAX_ACTUATORS) {
				if (test_motor.value < 0.f) {
					_current_output_value[reorderedMotorIndex(idx)] = _disarmed_value[idx];

				} else {
					_current_output_value[reorderedMotorIndex(idx)] =
						math::constrain<uint16_t>(_min_value[idx] + (uint16_t)((_max_value[idx] - _min_value[idx]) * test_motor.value),
									  _min_value[idx], _max_value[idx]);
				}
			}

			if (test_motor.timeout_ms > 0) {
				_motor_test.timeout = test_motor.timestamp + test_motor.timeout_ms * 1000;

			} else {
				_motor_test.timeout = 0;
			}
		}

		_motor_test.in_test_mode = in_test_mode;
		had_update = true;
	}

	// check for timeouts
	if (_motor_test.timeout != 0 && hrt_absolute_time() > _motor_test.timeout) {
		_motor_test.in_test_mode = false;
		_motor_test.timeout = 0;

		for (int i = 0; i < MAX_ACTUATORS; ++i) {
			_current_output_value[i] = _disarmed_value[i];
		}

		had_update = true;
	}

	return (_motor_test.in_test_mode || had_update) ? _max_num_outputs : 0;
}

bool MixingOutput::update()
{
	if (!_mixers) {
		handleCommands();
		// do nothing until we have a valid mixer
		// return false;
	}

	// check arming state
	if (_armed_sub.update(&_armed)) {
		_armed.in_esc_calibration_mode &= _support_esc_calibration;

		if (_ignore_lockdown) {
			_armed.lockdown = false;
		}

		/* Update the armed status and check that we're not locked down.
		 * We also need to arm throttle for the ESC calibration. */
		_throttle_armed = (_armed.armed && !_armed.lockdown) || _armed.in_esc_calibration_mode;

		if (_armed.armed) {
			_motor_test.in_test_mode = false;
		}
	}

	// check for motor test
	if (!_armed.armed && !_armed.manual_lockdown) {
		unsigned num_motor_test = motorTest();

		if (num_motor_test > 0) {
			if (_interface.updateOutputs(false, _current_output_value, num_motor_test, 1)) {
				actuator_outputs_s actuator_outputs{};
				setAndPublishActuatorOutputs(num_motor_test, actuator_outputs);
			}

			handleCommands();
			return true;
		}
	}

	if (_param_mot_slew_max.get() > FLT_EPSILON) {
		updateOutputSlewrateMultirotorMixer();
	}

	updateOutputSlewrateSimplemixer(); // update dt for output slew rate in simple mixer

	unsigned n_updates = 0;

	/* get controls for required topics */
	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (_groups_subscribed & (1 << i)) {
			if (_control_subs[i].copy(&_mixer_controls[i])) {
				n_updates++;
			}

			/* During ESC calibration, we overwrite the throttle value. */
			if (i == 0 && _armed.in_esc_calibration_mode) {

				/* Set all controls to 0 */
				memset(&_mixer_controls[i], 0, sizeof(_mixer_controls[i]));

				/* except thrust; set that to maximum. */
				_mixer_controls[i].control[actuator_controls_s::INDEX_THROTTLE] = 1.0f;

				/* Switch off the output limit ramp for the calibration. */
				_output_limit.state = OUTPUT_LIMIT_STATE_ON;
			}
		}
	}

	/* Do mixing (From mixer files) */

	unsigned mixed_num_outputs = 0;

	if (_mixers) {
		mixed_num_outputs = _mixers->mix(_mixer_outputs, _max_num_outputs);
	}

	/* Get output controls for required topics */

	for (unsigned i = 0; i < output_control_s::NUM_OUTPUT_CONTROL_GROUPS; i++) {
		const unsigned grp = n_act + i;

		if (_groups_required & (1 << grp)) {
			if (_control_subs[n_act + i].update(&_output_controls[i])) {
				n_updates++;
			}
		}
	}

	/* Map the current control function to the correct output(s) */
	for (unsigned i = 0; i < _max_num_outputs; i++) {
		const int32_t func = _assigned_function[i];
		float val = NAN;

		if (func == output_control_s::FUNCTION_MIXER && _mixers) {
			val = _mixer_outputs[i];

		} else if (func >= output_control_s::FUNCTION_CA0 &&
			   func <= output_control_s::FUNCTION_CA15) {

			val = _output_controls[0].value[func - output_control_s::FUNCTION_CA0];

		} else if (func >= output_control_s::FUNCTION_MAVLINK_SERVO0 &&
			   func <= output_control_s::FUNCTION_MAVLINK_SERVO7) {

			val = _output_controls[1].value[func - output_control_s::FUNCTION_MAVLINK_SERVO0];

		} else if (func >= output_control_s::FUNCTION_MC_MOTOR1 &&
			   func <= output_control_s::FUNCTION_MC_MOTOR8) {

			val = _output_controls[2].value[func - output_control_s::FUNCTION_MC_MOTOR1];

		} else if (func >= output_control_s::FUNCTION_AILERON1 &&
			   func <= output_control_s::FUNCTION_ENGINE2) {

			val = _output_controls[3].value[func - output_control_s::FUNCTION_AILERON1];

		} else if (func >= output_control_s::FUNCTION_GIMBAL_ROLL &&
			   func <= output_control_s::FUNCTION_CAMERA_ZOOM) {

			val = _output_controls[4].value[func - output_control_s::FUNCTION_GIMBAL_ROLL];
		}

		if (PX4_ISFINITE(val)) {
			_current_inputs[i] = val;
			mixed_num_outputs = (i >= mixed_num_outputs) ? (i + 1) : mixed_num_outputs;
		}
	}

	/// TODO: Not highly necessary, but if controlling a MAVLink servo, gimbal, etc. and not
	///       controlling any motors, set _output_limit.state = OUTPUT_LIMIT_STATE_ON

	/* the output limit call takes care of out of band errors, NaN and constrains */
	output_limit_calc(_throttle_armed, armNoThrottle(), _max_num_outputs, _reverse_output_mask,
			  _disarmed_value, _min_value, _max_value, _current_inputs, _current_output_value, &_output_limit);

	/* overwrite outputs in case of force_failsafe with _failsafe_value values */
	if (_armed.force_failsafe) {
		for (size_t i = 0; i < mixed_num_outputs; i++) {
			_current_output_value[i] = _failsafe_value[i];
		}
	}

	bool stop_motors = mixed_num_outputs == 0 || !_throttle_armed;

	/* overwrite outputs in case of lockdown or parachute triggering with disarmed values */
	if (_armed.lockdown || _armed.manual_lockdown) {
		for (size_t i = 0; i < mixed_num_outputs; i++) {
			_current_output_value[i] = _disarmed_value[i];
		}

		stop_motors = true;
	}

	/* apply _param_mot_ordering */
	reorderOutputs(_current_output_value);

	/* Zero out any outputs which do not have a function assigned */
	for (unsigned i = 0; i < _max_num_outputs; i++) {
		if (_assigned_function[i] == output_control_s::FUNCTION_NONE) {
			_current_output_value[i] = 0;
		}
	}

	/* now return the outputs to the driver */
	if (_interface.updateOutputs(stop_motors, _current_output_value, mixed_num_outputs, n_updates)) {
		actuator_outputs_s actuator_outputs{};
		setAndPublishActuatorOutputs(mixed_num_outputs, actuator_outputs);

		publishMixerStatus(actuator_outputs);
		updateLatencyPerfCounter(actuator_outputs);
	}

	handleCommands();

	return true;
}

void
MixingOutput::setAndPublishActuatorOutputs(unsigned num_outputs, actuator_outputs_s &actuator_outputs)
{
	actuator_outputs.noutputs = num_outputs;

	for (size_t i = 0; i < num_outputs; ++i) {
		actuator_outputs.output[i] = _current_output_value[i];
	}

	actuator_outputs.timestamp = hrt_absolute_time();
	_outputs_pub.publish(actuator_outputs);
}

void
MixingOutput::publishMixerStatus(const actuator_outputs_s &actuator_outputs)
{
	if (!_mixers) {
		return;
	}

	MultirotorMixer::saturation_status saturation_status;
	saturation_status.value = _mixers->get_saturation_status();

	if (saturation_status.flags.valid) {
		multirotor_motor_limits_s motor_limits;
		motor_limits.timestamp = actuator_outputs.timestamp;
		motor_limits.saturation_status = saturation_status.value;

		_to_mixer_status.publish(motor_limits);
	}
}

void
MixingOutput::updateLatencyPerfCounter(const actuator_outputs_s &actuator_outputs)
{
	// use first valid timestamp_sample for latency tracking
	for (int i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		const bool required = _groups_required & (1 << i);
		const hrt_abstime &timestamp_sample = _mixer_controls[i].timestamp_sample;

		if (required && (timestamp_sample > 0)) {
			perf_set_elapsed(_control_latency_perf, actuator_outputs.timestamp - timestamp_sample);
			break;
		}
	}
}

void
MixingOutput::reorderOutputs(uint16_t values[MAX_ACTUATORS])
{
	if (MAX_ACTUATORS < 4) {
		return;
	}

	if ((MotorOrdering)_param_mot_ordering.get() == MotorOrdering::Betaflight) {
		/*
		 * Betaflight default motor ordering:
		 * 4     2
		 *    ^
		 * 3     1
		 */
		const uint16_t value_tmp[4] = {values[0], values[1], values[2], values[3] };
		values[0] = value_tmp[3];
		values[1] = value_tmp[0];
		values[2] = value_tmp[1];
		values[3] = value_tmp[2];
	}

	/* else: PX4, no need to reorder
	 * 3     1
	 *    ^
	 * 2     4
	 */
}

int MixingOutput::reorderedMotorIndex(int index) const
{
	if ((MotorOrdering)_param_mot_ordering.get() == MotorOrdering::Betaflight) {
		switch (index) {
		case 0: return 1;

		case 1: return 2;

		case 2: return 3;

		case 3: return 0;
		}
	}

	return index;
}

int MixingOutput::controlCallback(uintptr_t handle, uint8_t control_group, uint8_t control_index, float &input)
{
	const MixingOutput *output = (const MixingOutput *)handle;

	input = output->_mixer_controls[control_group].control[control_index];

	/* limit control input */
	input = math::constrain(input, -1.f, 1.f);

	/* motor spinup phase - lock throttle to zero */
	if (output->_output_limit.state == OUTPUT_LIMIT_STATE_RAMP) {
		if (((control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE ||
		      control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE_ALTERNATE) &&
		     control_index == actuator_controls_s::INDEX_THROTTLE) ||
		    (control_group == actuator_controls_s::GROUP_INDEX_ALLOCATED_PART1 ||
		     control_group == actuator_controls_s::GROUP_INDEX_ALLOCATED_PART2)) {
			/* limit the throttle output to zero during motor spinup,
			 * as the motors cannot follow any demand yet
			 */
			input = 0.0f;
		}
	}

	/* throttle not arming - mark throttle input as invalid */
	if (output->armNoThrottle() && !output->_armed.in_esc_calibration_mode) {
		if (((control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE ||
		      control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE_ALTERNATE) &&
		     control_index == actuator_controls_s::INDEX_THROTTLE) ||
		    (control_group == actuator_controls_s::GROUP_INDEX_ALLOCATED_PART1 ||
		     control_group == actuator_controls_s::GROUP_INDEX_ALLOCATED_PART2)) {
			/* set the throttle to an invalid value */
			input = NAN;
		}
	}

	return 0;
}

void MixingOutput::resetMixer()
{
	if (_mixers != nullptr) {
		delete _mixers;
		_mixers = nullptr;
		_groups_required = 0;
	}

	_interface.mixerChanged();
}

int MixingOutput::loadMixer(const char *buf, unsigned len)
{
	if (_mixers == nullptr) {
		_mixers = new MixerGroup();
	}

	if (_mixers == nullptr) {
		_groups_required = 0;
		return -ENOMEM;
	}

	int ret = _mixers->load_from_buf(controlCallback, (uintptr_t)this, buf, len);

	if (ret != 0) {
		PX4_ERR("mixer load failed with %d", ret);
		delete _mixers;
		_mixers = nullptr;
		_groups_required = 0;
		return ret;
	}

	_mixers->groups_required(_groups_required);
	PX4_DEBUG("loaded mixers \n%s\n", buf);

	updateParams();
	_interface.mixerChanged();
	return ret;
}

void MixingOutput::handleCommands()
{
	if ((Command::Type)_command.command.load() == Command::Type::None) {
		return;
	}

	switch ((Command::Type)_command.command.load()) {
	case Command::Type::loadMixer:
		_command.result = loadMixer(_command.mixer_buf, _command.mixer_buf_length);
		break;

	case Command::Type::resetMixer:
		resetMixer();
		_command.result = 0;
		break;

	default:
		break;
	}

	// mark as done
	_command.command.store((int)Command::Type::None);
}

void MixingOutput::resetMixerThreadSafe()
{
	if ((Command::Type)_command.command.load() != Command::Type::None) {
		// Cannot happen, because we expect only one other thread to call this.
		// But as a safety precaution we return here.
		PX4_ERR("Command not None");
		return;
	}

	lock();

	_command.command.store((int)Command::Type::resetMixer);

	_interface.ScheduleNow();

	unlock();

	// wait until processed
	while ((Command::Type)_command.command.load() != Command::Type::None) {
		usleep(1000);
	}

}

int MixingOutput::loadMixerThreadSafe(const char *buf, unsigned len)
{
	if ((Command::Type)_command.command.load() != Command::Type::None) {
		// Cannot happen, because we expect only one other thread to call this.
		// But as a safety precaution we return here.
		PX4_ERR("Command not None");
		return -1;
	}

	lock();

	_command.mixer_buf = buf;
	_command.mixer_buf_length = len;
	_command.command.store((int)Command::Type::loadMixer);

	_interface.ScheduleNow();

	unlock();

	// wait until processed
	while ((Command::Type)_command.command.load() != Command::Type::None) {
		usleep(1000);
	}

	return _command.result;
}


void MixingOutput::updateParamValues(const char *type, uint16_t values[MAX_ACTUATORS])
{
	for (unsigned i = 0; i < _max_num_outputs; i++) {
		char pname[16];

		/* fill the struct from parameters */
		sprintf(pname, "%s_%s%d", _output_module_prefix, type, i + 1);
		param_t param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t pval = 0;
			param_get(param_h, &pval);
			values[i] = (uint16_t)pval;
			PX4_DEBUG("%s: %d", pname, values[i]);
		}
	}
}

void MixingOutput::updateFailsafeValues()
{
	int32_t default_fail = 0;
	char pname[16];
	sprintf(pname, "%s_FAIL", _output_module_prefix);
	param_t def_param_h = param_find(pname);

	if (def_param_h != PARAM_INVALID) {
		param_get(def_param_h, &default_fail);
	}

	for (unsigned i = 0; i < _max_num_outputs; i++) {
		_failsafe_value[i] = default_fail;

		/* fill the struct from parameters */
		sprintf(pname, "%s_FAIL%d", _output_module_prefix, i + 1);
		param_t param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t pval = 0;
			param_get(param_h, &pval);

			if (pval < 0) {
				// In the default case of -1, use default value
				pval = default_fail;
			}

			_failsafe_value[i] = (uint16_t)pval;
			PX4_DEBUG("%s: %d", pname, _failsafe_value[i]);
		}
	}
}

void MixingOutput::updateDisarmedValues()
{
	int32_t default_dis = 0;
	char pname[16];
	sprintf(pname, "%s_DIS", _output_module_prefix);
	param_t def_param_h = param_find(pname);

	if (def_param_h != PARAM_INVALID) {
		param_get(def_param_h, &default_dis);
	}

	for (unsigned i = 0; i < _max_num_outputs; i++) {
		_disarmed_value[i] = default_dis;

		/* fill the struct from parameters */
		sprintf(pname, "%s_DIS%d", _output_module_prefix, i + 1);
		param_t param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t pval = 0;
			param_get(param_h, &pval);

			if (pval < 0) {
				// In the default case of -1, use default value
				pval = default_dis;
			}

			_disarmed_value[i] = (uint16_t)pval;
			PX4_DEBUG("%s: %d", pname, _disarmed_value[i]);
		}
	}
}

void MixingOutput::updateMinValues()
{
	int32_t default_min = 0;
	char pname[16];
	sprintf(pname, "%s_MIN", _output_module_prefix);
	param_t def_param_h = param_find(pname);

	if (def_param_h != PARAM_INVALID) {
		param_get(def_param_h, &default_min);
	}

	for (unsigned i = 0; i < _max_num_outputs; i++) {
		_min_value[i] = default_min;

		/* fill the struct from parameters */
		sprintf(pname, "%s_MIN%d", _output_module_prefix, i + 1);
		param_t param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t pval = 0;
			param_get(param_h, &pval);

			if (pval < 0) {
				// In the default case of -1, use default value
				pval = default_min;
			}

			_min_value[i] = (uint16_t)pval;
			PX4_DEBUG("%s: %d", pname, _min_value[i]);
		}
	}
}

void MixingOutput::updateMaxValues()
{
	int32_t default_max = 2000;
	char pname[16];
	sprintf(pname, "%s_MAX", _output_module_prefix);
	param_t def_param_h = param_find(pname);

	if (def_param_h == PARAM_INVALID) {
		param_get(def_param_h, &default_max);
	}

	for (unsigned i = 0; i < _max_num_outputs; i++) {
		_max_value[i] = default_max;

		/* fill the struct from parameters */
		sprintf(pname, "%s_MAX%d", _output_module_prefix, i + 1);
		param_t param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t pval = 0;
			param_get(param_h, &pval);

			if (pval < 0) {
				// In the default case of -1, use default value
				pval = default_max;
			}

			_max_value[i] = (uint16_t)pval;
			PX4_DEBUG("%s: %d", pname, _max_value[i]);
		}
	}
}

void MixingOutput::updateTrimValues()
{
	for (unsigned i = 0; i < _max_num_outputs; i++) {
		_trim_value[i] = 0;

		/* fill the struct from parameters */
		char pname[16];
		sprintf(pname, "%s_TRIM%d", _output_module_prefix, i + 1);
		param_t param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			float pval = 0.0f;
			param_get(param_h, &pval);
			_trim_value[i] = (int16_t)(10000 * pval);
			PX4_DEBUG("%s: %d", pname, _trim_value[i]);
		}
	}
}

void MixingOutput::updateReverseMask()
{
	_reverse_output_mask = 0;

	for (unsigned i = 0; i < _max_num_outputs; i++) {
		char pname[16];

		/* fill the channel reverse mask from parameters */
		sprintf(pname, "%s_REV%d", _output_module_prefix, i + 1);
		param_t param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t ival = 0;
			param_get(param_h, &ival);
			_reverse_output_mask |= ((int16_t)(ival != 0)) << i;
		}
	}
}
