/****************************************************************************
 *
 *   Copyright (c) 2012-2021 PX4 Development Team. All rights reserved.
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

#include "PWMOut.hpp"

pthread_mutex_t pwm_out_module_mutex = PTHREAD_MUTEX_INITIALIZER;
static px4::atomic<PWMOut *> _objects[PWM_OUT_MAX_INSTANCES] {};

static bool is_running()
{
	for (auto &obj : _objects) {
		if (obj.load() != nullptr) {
			return true;
		}
	}

	return false;
}

PWMOut::PWMOut(int instance, uint8_t output_base) :
	CDev((instance == 0) ? PX4FMU_DEVICE_PATH : PX4FMU_DEVICE_PATH"1"),
	OutputModuleInterface((instance == 0) ? MODULE_NAME"0" : MODULE_NAME"1", px4::wq_configurations::hp_default),
	_instance(instance),
	_output_base(output_base),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_interval_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": interval"))
{
	_mixing_output.setAllMinValues(PWM_DEFAULT_MIN);
	_mixing_output.setAllMaxValues(PWM_DEFAULT_MAX);
}

PWMOut::~PWMOut()
{
	/* make sure servos are off */
	up_pwm_servo_deinit(); // TODO: review for multi

	/* clean up the alternate device node */
	unregister_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH, _class_instance);

	perf_free(_cycle_perf);
	perf_free(_interval_perf);
}

int PWMOut::init()
{
	/* do regular cdev init */
	int ret = CDev::init();

	if (ret != OK) {
		return ret;
	}

	// XXX best would be to register / de-register the device depending on modes

	/* try to claim the generic PWM output device node as well - it's OK if we fail at this */
	_class_instance = register_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH);

	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		/* lets not be too verbose */
	} else if (_class_instance < 0) {
		PX4_ERR("FAILED registering class device");
	}

	_mixing_output.setDriverInstance(_class_instance);

	/* force a reset of the update rate */
	_current_update_rate = 0;

	// Getting initial parameter values
	update_params();

	ScheduleNow();

	return 0;
}

int PWMOut::set_mode(Mode mode)
{
	unsigned old_mask = _pwm_mask;

	/*
	 * Configure for PWM output.
	 *
	 * Note that regardless of the configured mode, the task is always
	 * listening and mixing; the mode just selects which of the channels
	 * are presented on the output pins.
	 */
	switch (mode) {
	case MODE_1PWM:
		/* default output rates */
		_pwm_default_rate = 50;
		_pwm_alt_rate = 50;
		_pwm_alt_rate_channels = 0;
		_pwm_mask = 0b0000'0000'0000'0001 << _output_base;
		_pwm_initialized = false;
		_num_outputs = 1;
		_mixing_output.setMaxNumOutputs(_num_outputs);
		update_params();
		break;

#if defined(BOARD_HAS_CAPTURE)

	case MODE_2PWM2CAP:	// v1 multi-port with flow control lines as PWM
		up_input_capture_set(2, Rising, 0, NULL, NULL);
		up_input_capture_set(3, Rising, 0, NULL, NULL);
		PX4_DEBUG("MODE_2PWM2CAP");
#endif

	/* FALLTHROUGH */

	case MODE_2PWM:	// v1 multi-port with flow control lines as PWM
		PX4_DEBUG("MODE_2PWM");

		/* default output rates */
		_pwm_default_rate = 50;
		_pwm_alt_rate = 50;
		_pwm_alt_rate_channels = 0;
		_pwm_mask = 0b0000'0000'0000'0011 << _output_base;
		_pwm_initialized = false;
		_num_outputs = 2;
		_mixing_output.setMaxNumOutputs(_num_outputs);
		update_params();

		break;

#if defined(BOARD_HAS_CAPTURE)

	case MODE_3PWM1CAP:	// v1 multi-port with flow control lines as PWM
		PX4_DEBUG("MODE_3PWM1CAP");
		up_input_capture_set(3, Rising, 0, NULL, NULL);
#endif

	/* FALLTHROUGH */

	case MODE_3PWM:	// v1 multi-port with flow control lines as PWM
		PX4_DEBUG("MODE_3PWM");

		/* default output rates */
		_pwm_default_rate = 50;
		_pwm_alt_rate = 50;
		_pwm_alt_rate_channels = 0;
		_pwm_mask = 0b0000'0000'0000'0111 << _output_base;
		_pwm_initialized = false;
		_num_outputs = 3;
		_mixing_output.setMaxNumOutputs(_num_outputs);
		update_params();

		break;

#if defined(BOARD_HAS_CAPTURE)

	case MODE_4PWM1CAP:
		PX4_DEBUG("MODE_4PWM1CAP");
		up_input_capture_set(4, Rising, 0, NULL, NULL);
#endif

	/* FALLTHROUGH */

	case MODE_4PWM: // v1 or v2 multi-port as 4 PWM outs
		PX4_DEBUG("MODE_4PWM");

		/* default output rates */
		_pwm_default_rate = 50;
		_pwm_alt_rate = 50;
		_pwm_alt_rate_channels = 0;
		_pwm_mask = 0b0000'0000'0000'1111 << _output_base;
		_pwm_initialized = false;
		_num_outputs = 4;
		_mixing_output.setMaxNumOutputs(_num_outputs);
		update_params();

		break;

#if defined(BOARD_HAS_CAPTURE)

	case MODE_4PWM2CAP:
		PX4_DEBUG("MODE_4PWM2CAP");
		up_input_capture_set(5, Rising, 0, NULL, NULL);

		/* default output rates */
		_pwm_default_rate = 400;
		_pwm_alt_rate = 50;
		_pwm_alt_rate_channels = 0;
		_pwm_mask = 0b0000'0000'0000'1111 << _output_base;
		_pwm_initialized = false;
		_num_outputs = 4;
		_mixing_output.setMaxNumOutputs(_num_outputs);
		update_params();

		break;
#endif

#if defined(BOARD_HAS_CAPTURE)

	case MODE_5PWM1CAP:
		PX4_DEBUG("MODE_5PWM1CAP");
		up_input_capture_set(5, Rising, 0, NULL, NULL);
#endif

	/* FALLTHROUGH */

	case MODE_5PWM: // v1 or v2 multi-port as 5 PWM outs
		PX4_DEBUG("MODE_5PWM");

		/* default output rates */
		_pwm_default_rate = 50;
		_pwm_alt_rate = 50;
		_pwm_alt_rate_channels = 0;
		_pwm_mask = 0b0000'0000'0001'1111 << _output_base;
		_pwm_initialized = false;
		_num_outputs = 5;
		_mixing_output.setMaxNumOutputs(_num_outputs);
		update_params();

		break;

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 6

	case MODE_6PWM:
		PX4_DEBUG("MODE_6PWM");

		/* default output rates */
		_pwm_default_rate = 50;
		_pwm_alt_rate = 50;
		_pwm_alt_rate_channels = 0;
		_pwm_mask = 0b0000'0000'0011'1111 << _output_base;
		_pwm_initialized = false;
		_num_outputs = 6;
		_mixing_output.setMaxNumOutputs(_num_outputs);
		update_params();

		break;
#endif

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 8

	case MODE_8PWM:
		PX4_DEBUG("MODE_8PWM");
		/* default output rates */
		_pwm_default_rate = 50;
		_pwm_alt_rate = 50;
		_pwm_alt_rate_channels = 0;
		_pwm_mask = 0b0000'0000'1111'1111 << _output_base;
		_pwm_initialized = false;
		_num_outputs = 8;
		_mixing_output.setMaxNumOutputs(_num_outputs);
		update_params();

		break;
#endif

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 12

	case MODE_12PWM:
		PX4_DEBUG("MODE_12PWM");
		/* default output rates */
		_pwm_default_rate = 50;
		_pwm_alt_rate = 50;
		_pwm_alt_rate_channels = 0;
		_pwm_mask = 0b0000'1111'1111'1111 << _output_base;
		_pwm_initialized = false;
		_num_outputs = 12;
		_mixing_output.setMaxNumOutputs(_num_outputs);
		update_params();

		break;
#endif

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 14

	case MODE_14PWM:
		PX4_DEBUG("MODE_14PWM");
		/* default output rates */
		_pwm_default_rate = 50;
		_pwm_alt_rate = 50;
		_pwm_alt_rate_channels = 0;
		_pwm_mask = 0b0011'1111'1111'1111 << _output_base;
		_pwm_initialized = false;
		_num_outputs = 14;
		_mixing_output.setMaxNumOutputs(_num_outputs);
		update_params();

		break;
#endif

	case MODE_NONE:
		PX4_DEBUG("MODE_NONE");

		_pwm_default_rate = 10;	/* artificially reduced output rate */
		_pwm_alt_rate = 10;
		_pwm_alt_rate_channels = 0;
		_pwm_mask = 0x0;
		_pwm_initialized = false;
		_num_outputs = 0;
		_mixing_output.setMaxNumOutputs(_num_outputs);
		update_params();

		if (old_mask != _pwm_mask) {
			/* disable servo outputs - no need to set rates */
			up_pwm_servo_deinit(); // TODO: review for multi
		}

		break;

	default:
		return -EINVAL;
	}

	_mode = mode;
	return OK;
}

/* When set_pwm_rate is called from either of the 2 IOCTLs:
 *
 * PWM_SERVO_SET_UPDATE_RATE        - Sets the "alternate" channel's rate to the callers's rate specified
 *                                    and the non "alternate" channels to the _pwm_default_rate.
 *
 *                                    rate_map     = _pwm_alt_rate_channels
 *                                    default_rate = _pwm_default_rate
 *                                    alt_rate     = arg of IOCTL (see rates)
 *
 * PWM_SERVO_SET_SELECT_UPDATE_RATE - The caller's specified rate map selects the "alternate" channels
 *                                    to be set to the alt rate. (_pwm_alt_rate)
 *                                    All other channels are set to the default rate. (_pwm_default_rate)
 *
 *                                    rate_map     = arg of IOCTL
 *                                    default_rate = _pwm_default_rate
 *                                    alt_rate     = _pwm_alt_rate

 *  rate_map                        - A mask of 1's for the channels to be set to the
 *                                    alternate rate.
 *                                    N.B. All channels is a given group must be set
 *                                    to the same rate/mode. (default or alt)
 * rates:
 *   alt_rate, default_rate           For PWM is 25 or 400Hz
 *                                    For Oneshot there is no rate, 0 is therefore used
 *                                    to  select Oneshot mode
 */
int PWMOut::set_pwm_rate(uint32_t rate_map, unsigned default_rate, unsigned alt_rate)
{
	PX4_DEBUG("pwm_out%u set_pwm_rate %x %u %u", _instance, rate_map, default_rate, alt_rate);

	for (unsigned pass = 0; pass < 2; pass++) {

		/* We should note that group is iterated over from 0 to FMU_MAX_ACTUATORS.
		 * This allows for the ideal worlds situation: 1 channel per group
		 * configuration.
		 *
		 * This is typically not what HW supports. A group represents a timer
		 * and channels belongs to a timer.
		 * Therefore all channels in a group are dependent on the timer's
		 * common settings and can not be independent in terms of count frequency
		 * (granularity of pulse width) and rate (period of repetition).
		 *
		 * To say it another way, all channels in a group moust have the same
		 * rate and mode. (See rates above.)
		 */

		for (unsigned group = 0; group < FMU_MAX_ACTUATORS; group++) {

			// get the channel mask for this rate group
			uint32_t mask = up_pwm_servo_get_rate_group(group);

			if (mask == 0) {
				continue;
			}

			// all channels in the group must be either default or alt-rate
			uint32_t alt = rate_map & mask;

			if (pass == 0) {
				// preflight
				if ((alt != 0) && (alt != mask)) {
					PX4_WARN("rate group %u mask %x bad overlap %x", group, mask, alt);
					// not a legal map, bail
					return -EINVAL;
				}

			} else {
				// set it - errors here are unexpected
				if (alt != 0) {
					if (up_pwm_servo_set_rate_group_update(group, alt_rate) != OK) {
						PX4_WARN("rate group set alt failed");
						return -EINVAL;
					}

				} else {
					if (up_pwm_servo_set_rate_group_update(group, default_rate) != OK) {
						PX4_WARN("rate group set default failed");
						return -EINVAL;
					}
				}
			}
		}
	}

	_pwm_alt_rate_channels = rate_map;
	_pwm_default_rate = default_rate;
	_pwm_alt_rate = alt_rate;

	// minimum rate for backup schedule
	unsigned backup_schedule_rate_hz = math::min(_pwm_default_rate, _pwm_alt_rate);

	if (backup_schedule_rate_hz == 0) {
		// OneShot rate is 0
		backup_schedule_rate_hz = 50;
	}

	// constrain reasonably (1 to 50 Hz)
	backup_schedule_rate_hz = math::constrain(backup_schedule_rate_hz, 1u, 50u);

	_backup_schedule_interval_us = roundf(1e6f / backup_schedule_rate_hz);

	_current_update_rate = 0; // force update

	return OK;
}

int PWMOut::set_i2c_bus_clock(unsigned bus, unsigned clock_hz)
{
	return device::I2C::set_bus_clock(bus, clock_hz);
}

void PWMOut::update_current_rate()
{
	/*
	* Adjust actuator topic update rate to keep up with
	* the highest servo update rate configured.
	*
	* We always mix at max rate; some channels may update slower.
	*/
	int max_rate = (_pwm_default_rate > _pwm_alt_rate) ? _pwm_default_rate : _pwm_alt_rate;

	// oneshot
	if ((_pwm_default_rate == 0) || (_pwm_alt_rate == 0)) {
		max_rate = 2000;

	} else {
		// run up to twice PWM rate to reduce end-to-end latency
		//  actual pulse width only updated for next period regardless of output module
		max_rate *= 2;
	}

	// max interval 0.5 - 100 ms (10 - 2000Hz)
	const int update_interval_in_us = math::constrain(1000000 / max_rate, 500, 100000);

	if (_current_update_rate != max_rate) {
		PX4_INFO("instance: %d, max rate: %d, default: %d, alt: %d", _instance, max_rate, _pwm_default_rate, _pwm_alt_rate);
	}

	_current_update_rate = max_rate;
	_mixing_output.setMaxTopicUpdateRate(update_interval_in_us);
}

int PWMOut::task_spawn(int argc, char *argv[])
{
	for (unsigned instance = 0; instance < (sizeof(_objects) / sizeof(_objects[0])); instance++) {

		if (instance < PWM_OUT_MAX_INSTANCES) {
			uint8_t base = instance * 8;  // TODO: configurable
			PWMOut *dev = new PWMOut(instance, base);

			if (dev) {
				_objects[instance].store(dev);

				if (dev->init() != PX4_OK) {
					PX4_ERR("%d - init failed", instance);
					delete dev;
					_objects[instance].store(nullptr);
					return PX4_ERROR;
				}

			} else {
				PX4_ERR("alloc failed");
			}

		} else {
			// This hardware platform does not support
			// this many devices, set the storage to
			// a sane default
			_objects[instance].store(nullptr);
		}
	}

	return PX4_OK;
}

void PWMOut::capture_trampoline(void *context, uint32_t chan_index,
				hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow)
{
	PWMOut *dev = static_cast<PWMOut *>(context);
	dev->capture_callback(chan_index, edge_time, edge_state, overflow);
}

void PWMOut::capture_callback(uint32_t chan_index,
			      hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow)
{
	fprintf(stdout, "FMU: Capture chan:%d time:%lld state:%d overflow:%d\n", chan_index, edge_time, edge_state, overflow);
}

void PWMOut::update_pwm_out_state(bool on)
{
	if (on && !_pwm_initialized && _pwm_mask != 0) {

		// Collect all PWM masks from all instances
		uint32_t pwm_mask_new = 0;
		// Collect the PWM alt rate channels across all instances
		uint32_t pwm_alt_rate_channels_new = 0;

		// Collect the minimum default rate
		unsigned default_rate_min = 0;
		// Collect the maximum alternative rate (400 Hz or DSHOT outputs)
		unsigned alt_rate_max = 0;

		for (int i = 0; i < PWM_OUT_MAX_INSTANCES; i++) {
			if (_objects[i].load()) {
				pwm_mask_new |= _objects[i].load()->get_pwm_mask();
				pwm_alt_rate_channels_new |= _objects[i].load()->get_alt_rate_channels();

				if (_objects[i].load()->get_alt_rate() > alt_rate_max) {
					alt_rate_max = _objects[i].load()->get_alt_rate();
				}

				if (_objects[i].load()->get_default_rate() < default_rate_min) {
					default_rate_min = _objects[i].load()->get_default_rate();
				}
			}
		}

		// Initialize the PWM output state for all instances
		// this is re-done once per instance, but harmless
		up_pwm_servo_init(pwm_mask_new);

		// Set rate is not affecting non-masked channels, so can be called
		// individually
		set_pwm_rate(pwm_alt_rate_channels_new, default_rate_min, alt_rate_max);
		_pwm_initialized = true;
	}

	up_pwm_servo_arm(on);
}

bool PWMOut::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated)
{
	if (_test_mode) {
		return false;
	}

	/* output to the servos */
	if (_pwm_initialized) {
		for (size_t i = 0; i < math::min(_num_outputs, num_outputs); i++) {
			up_pwm_servo_set(_output_base + i, outputs[i]);
		}
	}

	/* Trigger all timer's channels in Oneshot mode to fire
	 * the oneshots with updated values.
	 */
	if (num_control_groups_updated > 0) {
		up_pwm_update(); // TODO: review for multi
	}

	return true;
}

void PWMOut::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		//exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	// push backup schedule
	ScheduleDelayed(_backup_schedule_interval_us);

	if (_new_mode_request.load() != MODE_NO_REQUEST) {
		set_mode(_new_mode_request.load());
		_new_mode_request.store(MODE_NO_REQUEST);
	}

	_mixing_output.update();

	/* update PWM status if armed or if disarmed PWM values are set */
	bool pwm_on = _mixing_output.armed().armed || (_num_disarmed_set > 0) || _mixing_output.armed().in_esc_calibration_mode;

	if (_pwm_on != pwm_on) {
		_pwm_on = pwm_on;
		update_pwm_out_state(pwm_on);
	}

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		update_params();
	}

	if (_current_update_rate == 0) {
		update_current_rate();
	}

	// check at end of cycle (updateSubscriptions() can potentially change to a different WorkQueue thread)
	_mixing_output.updateSubscriptions(true, true);

	perf_end(_cycle_perf);
}

void PWMOut::update_params()
{
	updateParams();

	int32_t pwm_min_default = PWM_DEFAULT_MIN;
	int32_t pwm_max_default = PWM_DEFAULT_MAX;
	int32_t pwm_disarmed_default = 0;
	int32_t pwm_rate_default = 50;

	const char *prefix;

	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		prefix = "PWM_MAIN";

		param_get(param_find("PWM_MAIN_MIN"), &pwm_min_default);
		param_get(param_find("PWM_MAIN_MAX"), &pwm_max_default);
		param_get(param_find("PWM_MAIN_DISARM"), &pwm_disarmed_default);
		param_get(param_find("PWM_MAIN_RATE"), &pwm_rate_default);

	} else if (_class_instance == CLASS_DEVICE_SECONDARY) {
		prefix = "PWM_AUX";

		param_get(param_find("PWM_AUX_MIN"), &pwm_min_default);
		param_get(param_find("PWM_AUX_MAX"), &pwm_max_default);
		param_get(param_find("PWM_AUX_DISARM"), &pwm_disarmed_default);
		param_get(param_find("PWM_AUX_RATE"), &pwm_rate_default);

	} else if (_class_instance == CLASS_DEVICE_TERTIARY) {
		prefix = "PWM_EXTRA";

		param_get(param_find("PWM_EXTRA_MIN"), &pwm_min_default);
		param_get(param_find("PWM_EXTRA_MAX"), &pwm_max_default);
		param_get(param_find("PWM_EXTRA_DISARM"), &pwm_disarmed_default);
		param_get(param_find("PWM_EXTRA_RATE"), &pwm_rate_default);

	} else {
		PX4_ERR("invalid class instance %d", _class_instance);
		return;
	}

	// update the counter
	// this is needed to decide if disarmed PWM output should be turned on or not
	int num_disarmed_set = 0;

	char str[17];

	for (unsigned i = 0; i < _num_outputs; i++) {
		// PWM_MAIN_MINx
		{
			sprintf(str, "%s_MIN%u", prefix, i + 1);
			int32_t pwm_min = -1;

			if (param_get(param_find(str), &pwm_min) == PX4_OK) {
				if (pwm_min >= 0) {
					_mixing_output.minValue(i) = math::constrain(pwm_min, PWM_LOWEST_MIN, PWM_HIGHEST_MIN);

					if (pwm_min != _mixing_output.minValue(i)) {
						int32_t pwm_min_new = _mixing_output.minValue(i);
						param_set(param_find(str), &pwm_min_new);
					}

				} else {
					_mixing_output.minValue(i) = pwm_min_default;
				}
			}
		}

		// PWM_MAIN_MAXx
		{
			sprintf(str, "%s_MAX%u", prefix, i + 1);
			int32_t pwm_max = -1;

			if (param_get(param_find(str), &pwm_max) == PX4_OK) {
				if (pwm_max >= 0) {
					_mixing_output.maxValue(i) = math::constrain(pwm_max, PWM_LOWEST_MAX, PWM_HIGHEST_MAX);

					if (pwm_max != _mixing_output.maxValue(i)) {
						int32_t pwm_max_new = _mixing_output.maxValue(i);
						param_set(param_find(str), &pwm_max_new);
					}

				} else {
					_mixing_output.maxValue(i) = pwm_max_default;
				}
			}
		}

		// PWM_MAIN_FAILx
		{
			sprintf(str, "%s_FAIL%u", prefix, i + 1);
			int32_t pwm_failsafe = -1;

			if (param_get(param_find(str), &pwm_failsafe) == PX4_OK) {
				if (pwm_failsafe >= 0) {
					_mixing_output.failsafeValue(i) = math::constrain(pwm_failsafe, 0, PWM_HIGHEST_MAX);

					if (pwm_failsafe != _mixing_output.failsafeValue(i)) {
						int32_t pwm_fail_new = _mixing_output.failsafeValue(i);
						param_set(param_find(str), &pwm_fail_new);
					}
				}
			}
		}

		// PWM_MAIN_DISx
		{
			sprintf(str, "%s_DIS%u", prefix, i + 1);
			int32_t pwm_dis = -1;

			if (param_get(param_find(str), &pwm_dis) == PX4_OK) {
				if (pwm_dis >= 0) {
					_mixing_output.disarmedValue(i) = math::constrain(pwm_dis, 0, PWM_HIGHEST_MAX);

					if (pwm_dis != _mixing_output.disarmedValue(i)) {
						int32_t pwm_dis_new = _mixing_output.disarmedValue(i);
						param_set(param_find(str), &pwm_dis_new);
					}

				} else {
					_mixing_output.disarmedValue(i) = pwm_disarmed_default;
				}
			}

			if (_mixing_output.disarmedValue(i) > 0) {
				num_disarmed_set++;
			}
		}

		// PWM_MAIN_REVx
		{
			sprintf(str, "%s_REV%u", prefix, i + 1);
			int32_t pwm_rev = 0;

			if (param_get(param_find(str), &pwm_rev) == PX4_OK) {
				uint16_t &reverse_pwm_mask = _mixing_output.reverseOutputMask();

				if (pwm_rev >= 1) {
					reverse_pwm_mask = reverse_pwm_mask | (1 << i);

				} else {
					reverse_pwm_mask = reverse_pwm_mask & ~(1 << i);
				}
			}
		}
	}

	if (_mixing_output.mixers()) {
		int16_t values[FMU_MAX_ACTUATORS] {};

		for (unsigned i = 0; i < _num_outputs; i++) {
			sprintf(str, "%s_TRIM%u", prefix, i + 1);

			float pval = 0.0f;
			param_get(param_find(str), &pval);
			values[i] = roundf(10000 * pval);
		}

		// copy the trim values to the mixer offsets
		_mixing_output.mixers()->set_trims(values, _num_outputs);
	}

	_num_disarmed_set = num_disarmed_set;
}

int PWMOut::ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret;

	/* try it as a Capture ioctl next */
	ret = capture_ioctl(filp, cmd, arg);

	if (ret != -ENOTTY) {
		return ret;
	}

	/* if we are in valid PWM mode, try it as a PWM ioctl as well */
	switch (_mode) {
	case MODE_1PWM:
	case MODE_2PWM:
	case MODE_3PWM:
	case MODE_4PWM:
	case MODE_5PWM:
	case MODE_2PWM2CAP:
	case MODE_3PWM1CAP:
	case MODE_4PWM1CAP:
	case MODE_4PWM2CAP:
	case MODE_5PWM1CAP:
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 6
	case MODE_6PWM:
#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 8
	case MODE_8PWM:
#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 12
	case MODE_12PWM:
#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 14
	case MODE_14PWM:
#endif
		ret = pwm_ioctl(filp, cmd, arg);
		break;

	default:
		PX4_DEBUG("pwm_out%u, not in a PWM mode", _instance);
		break;
	}

	/* if nobody wants it, let CDev have it */
	if (ret == -ENOTTY) {
		ret = CDev::ioctl(filp, cmd, arg);
	}

	return ret;
}

int PWMOut::pwm_ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret = OK;

	PX4_DEBUG("pwm_out%u: ioctl cmd: %d, arg: %ld", _instance, cmd, arg);

	lock();

	switch (cmd) {
	case PWM_SERVO_ARM:
		update_pwm_out_state(true);
		break;

	case PWM_SERVO_SET_ARM_OK:
	case PWM_SERVO_CLEAR_ARM_OK:
	case PWM_SERVO_SET_FORCE_SAFETY_OFF:
	case PWM_SERVO_SET_FORCE_SAFETY_ON:
		break;

	case PWM_SERVO_DISARM:

		/* Ignore disarm if disarmed PWM is set already. */
		if (_num_disarmed_set == 0) {
			update_pwm_out_state(false);
		}

		break;

	case PWM_SERVO_GET_DEFAULT_UPDATE_RATE:
		*(uint32_t *)arg = _pwm_default_rate;
		break;

	case PWM_SERVO_SET_UPDATE_RATE:
		ret = set_pwm_rate(_pwm_alt_rate_channels, _pwm_default_rate, arg);
		break;

	case PWM_SERVO_GET_UPDATE_RATE:
		*(uint32_t *)arg = _pwm_alt_rate;
		break;

	case PWM_SERVO_SET_SELECT_UPDATE_RATE:
		ret = set_pwm_rate(arg, _pwm_default_rate, _pwm_alt_rate);
		break;

	case PWM_SERVO_GET_SELECT_UPDATE_RATE:
		*(uint32_t *)arg = _pwm_alt_rate_channels;
		break;

	case PWM_SERVO_SET_FAILSAFE_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > FMU_MAX_ACTUATORS) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (pwm->values[i] == 0) {
					/* ignore 0 */
				} else if (pwm->values[i] > PWM_HIGHEST_MAX) {
					_mixing_output.failsafeValue(i) = PWM_HIGHEST_MAX;

				}

#if PWM_LOWEST_MIN > 0

				else if (pwm->values[i] < PWM_LOWEST_MIN) {
					_mixing_output.failsafeValue(i) = PWM_LOWEST_MIN;

				}

#endif

				else {
					_mixing_output.failsafeValue(i) = pwm->values[i];
				}
			}

			break;
		}

	case PWM_SERVO_GET_FAILSAFE_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < FMU_MAX_ACTUATORS; i++) {
				pwm->values[i] = _mixing_output.failsafeValue(i);
			}

			pwm->channel_count = FMU_MAX_ACTUATORS;
			break;
		}

	case PWM_SERVO_SET_DISARMED_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > FMU_MAX_ACTUATORS) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (pwm->values[i] == 0) {
					/* ignore 0 */
				} else if (pwm->values[i] > PWM_HIGHEST_MAX) {
					_mixing_output.disarmedValue(i) = PWM_HIGHEST_MAX;
				}

#if PWM_LOWEST_MIN > 0

				else if (pwm->values[i] < PWM_LOWEST_MIN) {
					_mixing_output.disarmedValue(i) = PWM_LOWEST_MIN;
				}

#endif

				else {
					_mixing_output.disarmedValue(i) = pwm->values[i];
				}
			}

			/*
			 * update the counter
			 * this is needed to decide if disarmed PWM output should be turned on or not
			 */
			_num_disarmed_set = 0;

			for (unsigned i = 0; i < FMU_MAX_ACTUATORS; i++) {
				if (_mixing_output.disarmedValue(i) > 0) {
					_num_disarmed_set++;
				}
			}

			break;
		}

	case PWM_SERVO_GET_DISARMED_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < FMU_MAX_ACTUATORS; i++) {
				pwm->values[i] = _mixing_output.disarmedValue(i);
			}

			pwm->channel_count = FMU_MAX_ACTUATORS;
			break;
		}

	case PWM_SERVO_SET_MIN_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > FMU_MAX_ACTUATORS) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (pwm->values[i] == 0) {
					/* ignore 0 */
				} else if (pwm->values[i] > PWM_HIGHEST_MIN) {
					_mixing_output.minValue(i) = PWM_HIGHEST_MIN;

				}

#if PWM_LOWEST_MIN > 0

				else if (pwm->values[i] < PWM_LOWEST_MIN) {
					_mixing_output.minValue(i) = PWM_LOWEST_MIN;
				}

#endif

				else {
					_mixing_output.minValue(i) = pwm->values[i];
				}
			}

			break;
		}

	case PWM_SERVO_GET_MIN_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < FMU_MAX_ACTUATORS; i++) {
				pwm->values[i] = _mixing_output.minValue(i);
			}

			pwm->channel_count = FMU_MAX_ACTUATORS;
			arg = (unsigned long)&pwm;
			break;
		}

	case PWM_SERVO_SET_MAX_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > FMU_MAX_ACTUATORS) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (pwm->values[i] == 0) {
					/* ignore 0 */
				} else if (pwm->values[i] < PWM_LOWEST_MAX) {
					_mixing_output.maxValue(i) = PWM_LOWEST_MAX;

				} else if (pwm->values[i] > PWM_HIGHEST_MAX) {
					_mixing_output.maxValue(i) = PWM_HIGHEST_MAX;

				} else {
					_mixing_output.maxValue(i) = pwm->values[i];
				}
			}

			break;
		}

	case PWM_SERVO_GET_MAX_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < FMU_MAX_ACTUATORS; i++) {
				pwm->values[i] = _mixing_output.maxValue(i);
			}

			pwm->channel_count = FMU_MAX_ACTUATORS;
			arg = (unsigned long)&pwm;
		}
		break;

	case PWM_SERVO_GET_TRIM_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			if (_mixing_output.mixers() == nullptr) {
				memset(pwm, 0, sizeof(pwm_output_values));
				PX4_WARN("warning: trim values not valid - no mixer loaded");

			} else {

				pwm->channel_count = _mixing_output.mixers()->get_trims((int16_t *)pwm->values);
			}
		}
		break;

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 14

	case PWM_SERVO_SET(13):
	case PWM_SERVO_SET(12):
	case PWM_SERVO_SET(11):
	case PWM_SERVO_SET(10):
	case PWM_SERVO_SET(9):
	case PWM_SERVO_SET(8):
		if (_mode < MODE_14PWM) {
			ret = -EINVAL;
			break;
		}

#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 8

	/* FALLTHROUGH */
	case PWM_SERVO_SET(7):

	/* FALLTHROUGH */
	case PWM_SERVO_SET(6):
		if (_mode < MODE_8PWM) {
			ret = -EINVAL;
			break;
		}

#endif

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 6

	/* FALLTHROUGH */
	case PWM_SERVO_SET(5):
		if (_mode < MODE_6PWM) {
			ret = -EINVAL;
			break;
		}

#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 5

	/* FALLTHROUGH */
	case PWM_SERVO_SET(4):
		if (_mode < MODE_5PWM) {
			ret = -EINVAL;
			break;
		}

#endif

	/* FALLTHROUGH */
	case PWM_SERVO_SET(3):
		if (_mode < MODE_4PWM) {
			ret = -EINVAL;
			break;
		}

	/* FALLTHROUGH */
	case PWM_SERVO_SET(2):
		if (_mode < MODE_3PWM) {
			ret = -EINVAL;
			break;
		}

	/* FALLTHROUGH */
	case PWM_SERVO_SET(1):
	case PWM_SERVO_SET(0):
		if (arg <= 2100) {
			up_pwm_servo_set(cmd - PWM_SERVO_SET(0 + _output_base), arg);

		} else {
			ret = -EINVAL;
		}

		break;

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 14

	case PWM_SERVO_GET(13):
	case PWM_SERVO_GET(12):
	case PWM_SERVO_GET(11):
	case PWM_SERVO_GET(10):
	case PWM_SERVO_GET(9):
	case PWM_SERVO_GET(8):
		if (_mode < MODE_14PWM) {
			ret = -EINVAL;
			break;
		}

#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 8

	/* FALLTHROUGH */
	case PWM_SERVO_GET(7):
	case PWM_SERVO_GET(6):
		if (_mode < MODE_8PWM) {
			ret = -EINVAL;
			break;
		}

#endif

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 6

	/* FALLTHROUGH */
	case PWM_SERVO_GET(5):
		if (_mode < MODE_6PWM) {
			ret = -EINVAL;
			break;
		}

#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 5

	/* FALLTHROUGH */
	case PWM_SERVO_GET(4):
		if (_mode < MODE_5PWM) {
			ret = -EINVAL;
			break;
		}

#endif

	/* FALLTHROUGH */
	case PWM_SERVO_GET(3):
		if (_mode < MODE_4PWM) {
			ret = -EINVAL;
			break;
		}

	/* FALLTHROUGH */
	case PWM_SERVO_GET(2):
		if (_mode < MODE_3PWM) {
			ret = -EINVAL;
			break;
		}

	/* FALLTHROUGH */
	case PWM_SERVO_GET(1):
	case PWM_SERVO_GET(0):
		*(servo_position_t *)arg = up_pwm_servo_get(cmd - PWM_SERVO_GET(0 + _output_base));
		break;

	case PWM_SERVO_GET_RATEGROUP(0):
	case PWM_SERVO_GET_RATEGROUP(1):
	case PWM_SERVO_GET_RATEGROUP(2):
	case PWM_SERVO_GET_RATEGROUP(3):
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 5
	case PWM_SERVO_GET_RATEGROUP(4):
#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 6
	case PWM_SERVO_GET_RATEGROUP(5):
#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 8
	case PWM_SERVO_GET_RATEGROUP(6):
	case PWM_SERVO_GET_RATEGROUP(7):
#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 14
	case PWM_SERVO_GET_RATEGROUP(8):
	case PWM_SERVO_GET_RATEGROUP(9):
	case PWM_SERVO_GET_RATEGROUP(10):
	case PWM_SERVO_GET_RATEGROUP(11):
	case PWM_SERVO_GET_RATEGROUP(12):
	case PWM_SERVO_GET_RATEGROUP(13):
#endif
		*(uint32_t *)arg = up_pwm_servo_get_rate_group(cmd - PWM_SERVO_GET_RATEGROUP(0 + _output_base));
		break;

	case PWM_SERVO_GET_COUNT:
		switch (_mode) {

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 14

		case MODE_14PWM:
			*(unsigned *)arg = 14;
			break;
#endif

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 12

		case MODE_12PWM:
			*(unsigned *)arg = 12;
			break;
#endif

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 8

		case MODE_8PWM:
			*(unsigned *)arg = 8;
			break;
#endif

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 6

		case MODE_6PWM:
			*(unsigned *)arg = 6;
			break;
#endif

		case MODE_5PWM:
		case MODE_5PWM1CAP:
			*(unsigned *)arg = 5;
			break;

		case MODE_4PWM:
		case MODE_4PWM1CAP:
		case MODE_4PWM2CAP:
			*(unsigned *)arg = 4;
			break;

		case MODE_3PWM:
		case MODE_3PWM1CAP:
			*(unsigned *)arg = 3;
			break;

		case MODE_2PWM:
		case MODE_2PWM2CAP:
			*(unsigned *)arg = 2;
			break;

		case MODE_1PWM:
			*(unsigned *)arg = 1;
			break;

		default:
			ret = -EINVAL;
			break;
		}

		break;

	case PWM_SERVO_SET_MODE: {
			switch (arg) {
			case PWM_SERVO_MODE_NONE:
				ret = set_mode(MODE_NONE);
				break;

			case PWM_SERVO_MODE_1PWM:
				ret = set_mode(MODE_1PWM);
				break;

			case PWM_SERVO_MODE_2PWM:
				ret = set_mode(MODE_2PWM);
				break;

			case PWM_SERVO_MODE_2PWM2CAP:
				ret = set_mode(MODE_2PWM2CAP);
				break;

			case PWM_SERVO_MODE_3PWM:
				ret = set_mode(MODE_3PWM);
				break;

			case PWM_SERVO_MODE_3PWM1CAP:
				ret = set_mode(MODE_3PWM1CAP);
				break;

			case PWM_SERVO_MODE_4PWM:
				ret = set_mode(MODE_4PWM);
				break;

			case PWM_SERVO_MODE_4PWM1CAP:
				ret = set_mode(MODE_4PWM1CAP);
				break;

			case PWM_SERVO_MODE_4PWM2CAP:
				ret = set_mode(MODE_4PWM2CAP);
				break;

			case PWM_SERVO_MODE_5PWM:
				ret = set_mode(MODE_5PWM);
				break;

			case PWM_SERVO_MODE_5PWM1CAP:
				ret = set_mode(MODE_5PWM1CAP);
				break;

			case PWM_SERVO_MODE_6PWM:
				ret = set_mode(MODE_6PWM);
				break;

			case PWM_SERVO_MODE_8PWM:
				ret = set_mode(MODE_8PWM);
				break;

			case PWM_SERVO_MODE_12PWM:
				ret = set_mode(MODE_12PWM);
				break;

			case PWM_SERVO_MODE_14PWM:
				ret = set_mode(MODE_14PWM);
				break;

			case PWM_SERVO_MODE_4CAP:
				ret = set_mode(MODE_4CAP);
				break;

			case PWM_SERVO_MODE_5CAP:
				ret = set_mode(MODE_5CAP);
				break;

			case PWM_SERVO_MODE_6CAP:
				ret = set_mode(MODE_6CAP);
				break;

			case PWM_SERVO_ENTER_TEST_MODE:
				_test_mode = true;
				break;

			case PWM_SERVO_EXIT_TEST_MODE:
				_test_mode = false;
				break;

			default:
				ret = -EINVAL;
			}

			break;
		}

	case MIXERIOCRESET:
		_mixing_output.resetMixerThreadSafe();

		break;

	case MIXERIOCLOADBUF: {
			const char *buf = (const char *)arg;
			unsigned buflen = strlen(buf);
			ret = _mixing_output.loadMixerThreadSafe(buf, buflen);
			update_params();

			break;
		}

	default:
		ret = -ENOTTY;
		break;
	}

	unlock();

	return ret;
}

void PWMOut::sensor_reset(int ms)
{
	if (ms < 1) {
		ms = 1;
	}

	board_spi_reset(ms, 0xffff);
}

void PWMOut::peripheral_reset(int ms)
{
	if (ms < 1) {
		ms = 10;
	}

	board_peripheral_reset(ms);
}

int PWMOut::capture_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int	ret = -EINVAL;

#if defined(BOARD_HAS_CAPTURE)

	lock();

	input_capture_config_t *pconfig = 0;

	input_capture_stats_t *stats = (input_capture_stats_t *)arg;

	if (_mode == MODE_3PWM1CAP || _mode == MODE_2PWM2CAP ||
	    _mode == MODE_4PWM1CAP || _mode == MODE_5PWM1CAP ||
	    _mode == MODE_4PWM2CAP) {

		pconfig = (input_capture_config_t *)arg;
	}

	switch (cmd) {

	case INPUT_CAP_SET:
		if (pconfig) {
			ret =  up_input_capture_set(pconfig->channel, pconfig->edge, pconfig->filter,
						    pconfig->callback, pconfig->context);
		}

		break;

	case INPUT_CAP_SET_CALLBACK:
		if (pconfig) {
			ret =  up_input_capture_set_callback(pconfig->channel, pconfig->callback, pconfig->context);
		}

		break;

	case INPUT_CAP_GET_CALLBACK:
		if (pconfig) {
			ret =  up_input_capture_get_callback(pconfig->channel, &pconfig->callback, &pconfig->context);
		}

		break;

	case INPUT_CAP_GET_STATS:
		if (arg) {
			ret =  up_input_capture_get_stats(stats->chan_in_edges_out, stats, false);
		}

		break;

	case INPUT_CAP_GET_CLR_STATS:
		if (arg) {
			ret =  up_input_capture_get_stats(stats->chan_in_edges_out, stats, true);
		}

		break;

	case INPUT_CAP_SET_EDGE:
		if (pconfig) {
			ret =  up_input_capture_set_trigger(pconfig->channel, pconfig->edge);
		}

		break;

	case INPUT_CAP_GET_EDGE:
		if (pconfig) {
			ret =  up_input_capture_get_trigger(pconfig->channel, &pconfig->edge);
		}

		break;

	case INPUT_CAP_SET_FILTER:
		if (pconfig) {
			ret =  up_input_capture_set_filter(pconfig->channel, pconfig->filter);
		}

		break;

	case INPUT_CAP_GET_FILTER:
		if (pconfig) {
			ret =  up_input_capture_get_filter(pconfig->channel, &pconfig->filter);
		}

		break;

	case INPUT_CAP_GET_COUNT:
		ret = OK;

		switch (_mode) {
		case MODE_5PWM1CAP:
		case MODE_4PWM1CAP:
		case MODE_3PWM1CAP:
			*(unsigned *)arg = 1;
			break;

		case MODE_2PWM2CAP:
		case MODE_4PWM2CAP:
			*(unsigned *)arg = 2;
			break;

		default:
			ret = -EINVAL;
			break;
		}

		break;

	case INPUT_CAP_SET_COUNT:
		ret = OK;

		switch (_mode) {
		case MODE_3PWM1CAP:
			set_mode(MODE_3PWM1CAP);
			break;

		case MODE_2PWM2CAP:
			set_mode(MODE_2PWM2CAP);
			break;

		case MODE_4PWM1CAP:
			set_mode(MODE_4PWM1CAP);
			break;

		case MODE_4PWM2CAP:
			set_mode(MODE_4PWM2CAP);
			break;

		case MODE_5PWM1CAP:
			set_mode(MODE_5PWM1CAP);
			break;

		default:
			ret = -EINVAL;
			break;
		}

		break;

	default:
		ret = -ENOTTY;
		break;
	}

	unlock();

#else
	ret = -ENOTTY;
#endif
	return ret;
}

int PWMOut::fmu_new_mode(PortMode new_mode)
{
	if (!is_running()) {
		return -1;
	}

	PWMOut::Mode pwm_mode0 = PWMOut::MODE_NONE;
	PWMOut::Mode pwm_mode1 = PWMOut::MODE_NONE;

	switch (new_mode) {
	case PORT_FULL_GPIO:
	case PORT_MODE_UNSET:
		break;

	case PORT_FULL_PWM:

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM == 4
		/* select 4-pin PWM mode */
		pwm_mode0 = PWMOut::MODE_4PWM;
#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM == 5
		pwm_mode0 = PWMOut::MODE_5PWM;
#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM == 6
		pwm_mode0 = PWMOut::MODE_6PWM;
#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM == 8
		pwm_mode0 = PWMOut::MODE_8PWM;
#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM == 12
		//pwm_mode0 = PWMOut::MODE_12PWM;
		pwm_mode0 = PWMOut::MODE_8PWM;
		pwm_mode1 = PWMOut::MODE_4PWM;
#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM == 14
		//pwm_mode0 = PWMOut::MODE_14PWM;
		pwm_mode0 = PWMOut::MODE_8PWM;
		pwm_mode1 = PWMOut::MODE_6PWM;
#endif
		break;

	case PORT_PWM1:
		/* select 2-pin PWM mode */
		pwm_mode0 = PWMOut::MODE_1PWM;
		break;

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 14

	case PORT_PWM14:
		/* select 14-pin PWM mode */
		//pwm_mode0 = PWMOut::MODE_14PWM;
		pwm_mode0 = PWMOut::MODE_8PWM;
		pwm_mode1 = PWMOut::MODE_6PWM;
		break;
#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 12

	case PORT_PWM12:
		/* select 12-pin PWM mode */
		//pwm_mode0 = PWMOut::MODE_12PWM;
		pwm_mode0 = PWMOut::MODE_8PWM;
		pwm_mode1 = PWMOut::MODE_4PWM;
		break;
#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 8

	case PORT_PWM8:
		/* select 8-pin PWM mode */
		pwm_mode0 = PWMOut::MODE_8PWM;
		break;
#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 6

	case PORT_PWM6:
		/* select 6-pin PWM mode */
		pwm_mode0 = PWMOut::MODE_6PWM;
		break;
#endif

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 5

	case PORT_PWM5:
		/* select 5-pin PWM mode */
		pwm_mode0 = PWMOut::MODE_5PWM;
		break;


#  if defined(BOARD_HAS_CAPTURE)

	case PORT_PWM5CAP1:
		/* select 5-pin PWM mode 1 capture */
		pwm_mode0 = PWMOut::MODE_5PWM1CAP;
		break;

#  endif
#endif

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 4

	case PORT_PWM4:
		/* select 4-pin PWM mode */
		pwm_mode0 = PWMOut::MODE_4PWM;
		break;


#  if defined(BOARD_HAS_CAPTURE)

	case PORT_PWM4CAP1:
		/* select 4-pin PWM mode 1 capture */
		pwm_mode0 = PWMOut::MODE_4PWM1CAP;
		break;

	case PORT_PWM4CAP2:
		/* select 4-pin PWM mode 2 capture */
		pwm_mode0 = PWMOut::MODE_4PWM2CAP;
		break;

#  endif

	case PORT_PWM3:
		/* select 3-pin PWM mode */
		pwm_mode0 = PWMOut::MODE_3PWM;
		break;

#  if defined(BOARD_HAS_CAPTURE)

	case PORT_PWM3CAP1:
		/* select 3-pin PWM mode 1 capture */
		pwm_mode0 = PWMOut::MODE_3PWM1CAP;
		break;
#  endif

	case PORT_PWM2:
		/* select 2-pin PWM mode */
		pwm_mode0 = PWMOut::MODE_2PWM;
		break;

#  if defined(BOARD_HAS_CAPTURE)

	case PORT_PWM2CAP2:
		/* select 2-pin PWM mode 2 capture */
		pwm_mode0 = PWMOut::MODE_2PWM2CAP;
		break;

#  endif
#endif

	default:
		return -1;
	}

	if (PWM_OUT_MAX_INSTANCES > 0) {
		PWMOut *pwm0 = _objects[0].load(); // TODO: get_instance();

		if (pwm0 && pwm_mode0 != pwm0->get_mode()) {
			pwm0->request_mode(pwm_mode0);
		}
	}

	if (PWM_OUT_MAX_INSTANCES > 1) {
		PWMOut *pwm1 = _objects[1].load(); // TODO: get_instance();

		if (pwm1 && pwm_mode1 != pwm1->get_mode()) {
			pwm1->request_mode(pwm_mode1);
		}
	}

	return OK;
}

void PWMOut::request_mode(Mode new_mode)
{
	if (_new_mode_request.load() != MODE_NO_REQUEST) {
		PX4_ERR("already being set"); // not expected to happen
		return;
	}

	_new_mode_request.store(new_mode);
	ScheduleNow();
	// wait until processed
	int max_time = 1000;

	while (_new_mode_request.load() != MODE_NO_REQUEST && max_time-- > 0) {
		px4_usleep(1000);
	}
}

namespace
{

int fmu_new_i2c_speed(unsigned bus, unsigned clock_hz)
{
	return PWMOut::set_i2c_bus_clock(bus, clock_hz);
}

} // namespace

int PWMOut::test()
{
	int	 fd;
	unsigned servo_count = 0;
	unsigned capture_count = 0;
	unsigned pwm_value = 1000;
	int	 direction = 1;
	int  ret;
	int   rv = -1;
	uint32_t rate_limit = 0;
	struct input_capture_t {
		bool valid;
		input_capture_config_t  chan;
	} capture_conf[INPUT_CAPTURE_MAX_CHANNELS];

	fd = ::open(PX4FMU_DEVICE_PATH, O_RDWR);

	if (fd < 0) {
		PX4_ERR("open fail");
		return -1;
	}

	if (::ioctl(fd, PWM_SERVO_SET_MODE, PWM_SERVO_ENTER_TEST_MODE) < 0) {
		PX4_ERR("Failed to Enter pwm test mode");
		goto err_out_no_test;
	}

	if (::ioctl(fd, PWM_SERVO_ARM, 0) < 0) {
		PX4_ERR("servo arm failed");
		goto err_out;
	}

	if (::ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count) != 0) {
		PX4_ERR("Unable to get servo count");
		goto err_out;
	}

	if (::ioctl(fd, INPUT_CAP_GET_COUNT, (unsigned long)&capture_count) != 0) {
		PX4_INFO("Not in a capture mode");
	}

	PX4_INFO("Testing %u servos and %u input captures", (unsigned)servo_count, capture_count);
	memset(capture_conf, 0, sizeof(capture_conf));

	if (capture_count != 0) {
		for (unsigned i = 0; i < capture_count; i++) {
			// Map to channel number
			capture_conf[i].chan.channel = i + servo_count;

			/* Save handler */
			if (::ioctl(fd, INPUT_CAP_GET_CALLBACK, (unsigned long)&capture_conf[i].chan.channel) != 0) {
				PX4_ERR("Unable to get capture callback for chan %u\n", capture_conf[i].chan.channel);
				goto err_out;

			} else {
				input_capture_config_t conf = capture_conf[i].chan;
				conf.callback = &PWMOut::capture_trampoline;
				conf.context = _objects[0].load(); // TODO PWMOut::get_instance();

				if (::ioctl(fd, INPUT_CAP_SET_CALLBACK, (unsigned long)&conf) == 0) {
					capture_conf[i].valid = true;

				} else {
					PX4_ERR("Unable to set capture callback for chan %u\n", capture_conf[i].chan.channel);
					goto err_out;
				}
			}

		}
	}

	struct pollfd fds;

	fds.fd = 0; /* stdin */

	fds.events = POLLIN;

	PX4_INFO("Press CTRL-C or 'c' to abort.");

	for (;;) {
		/* sweep all servos between 1000..2000 */
		servo_position_t servos[servo_count];

		for (unsigned i = 0; i < servo_count; i++) {
			servos[i] = pwm_value;
		}

		for (unsigned i = 0; i < servo_count;	i++) {
			if (::ioctl(fd, PWM_SERVO_SET(i), servos[i]) < 0) {
				PX4_ERR("servo %u set failed", i);
				goto err_out;
			}
		}

		if (direction > 0) {
			if (pwm_value < 2000) {
				pwm_value++;

			} else {
				direction = -1;
			}

		} else {
			if (pwm_value > 1000) {
				pwm_value--;

			} else {
				direction = 1;
			}
		}

		/* readback servo values */
		for (unsigned i = 0; i < servo_count; i++) {
			servo_position_t value;

			if (::ioctl(fd, PWM_SERVO_GET(i), (unsigned long)&value)) {
				PX4_ERR("error reading PWM servo %d", i);
				goto err_out;
			}

			if (value != servos[i]) {
				PX4_ERR("servo %d readback error, got %u expected %u", i, value, servos[i]);
				goto err_out;
			}
		}

		if (capture_count != 0 && (++rate_limit % 500 == 0)) {
			for (unsigned i = 0; i < capture_count; i++) {
				if (capture_conf[i].valid) {
					input_capture_stats_t stats;
					stats.chan_in_edges_out = capture_conf[i].chan.channel;

					if (::ioctl(fd, INPUT_CAP_GET_STATS, (unsigned long)&stats) != 0) {
						PX4_ERR("Unable to get stats for chan %u\n", capture_conf[i].chan.channel);
						goto err_out;

					} else {
						fprintf(stdout, "FMU: Status chan:%u edges: %d last time:%lld last state:%d overflows:%d lantency:%u\n",
							capture_conf[i].chan.channel,
							stats.chan_in_edges_out,
							stats.last_time,
							stats.last_edge,
							stats.overflows,
							stats.latnecy);
					}
				}
			}

		}

		/* Check if user wants to quit */
		char c;
		ret = ::poll(&fds, 1, 0);

		if (ret > 0) {

			::read(0, &c, 1);

			if (c == 0x03 || c == 0x63 || c == 'q') {
				PX4_INFO("User abort");
				break;
			}
		}
	}

	if (capture_count != 0) {
		for (unsigned i = 0; i < capture_count; i++) {
			// Map to channel number
			if (capture_conf[i].valid) {
				/* Save handler */
				if (::ioctl(fd, INPUT_CAP_SET_CALLBACK, (unsigned long)&capture_conf[i].chan) != 0) {
					PX4_ERR("Unable to set capture callback for chan %u\n", capture_conf[i].chan.channel);
					goto err_out;
				}
			}
		}
	}

	rv = 0;

err_out:

	if (::ioctl(fd, PWM_SERVO_SET_MODE, PWM_SERVO_EXIT_TEST_MODE) < 0) {
		PX4_ERR("Failed to Exit pwm test mode");
	}

err_out_no_test:
	::close(fd);
	return rv;
}

int PWMOut::custom_command(int argc, char *argv[])
{
	PortMode new_mode = PORT_MODE_UNSET;
	const char *verb = argv[0];

	/* does not operate on a FMU instance */
	if (!strcmp(verb, "i2c")) {
		if (argc > 2) {
			int bus = strtol(argv[1], 0, 0);
			int clock_hz = strtol(argv[2], 0, 0);
			int ret = fmu_new_i2c_speed(bus, clock_hz);

			if (ret) {
				PX4_ERR("setting I2C clock failed");
			}

			return ret;
		}

		return print_usage("not enough arguments");
	}

	if (!strcmp(verb, "sensor_reset")) {
		if (argc > 1) {
			int reset_time = strtol(argv[1], nullptr, 0);
			sensor_reset(reset_time);

		} else {
			sensor_reset(0);
			PX4_INFO("reset default time");
		}

		return 0;
	}

	if (!strcmp(verb, "peripheral_reset")) {
		if (argc > 2) {
			int reset_time = strtol(argv[2], 0, 0);
			peripheral_reset(reset_time);

		} else {
			peripheral_reset(0);
			PX4_INFO("reset default time");
		}

		return 0;
	}


	/* start pwm_out if not running */
	if (!is_running()) {

		int ret = PWMOut::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	/*
	 * Mode switches.
	 */
	if (!strcmp(verb, "mode_gpio")) {
		new_mode = PORT_FULL_GPIO;

	} else if (!strcmp(verb, "mode_pwm")) {
		new_mode = PORT_FULL_PWM;

		// mode: defines which outputs to drive (others may be used by other tasks such as camera capture)
#if defined(BOARD_HAS_PWM)

	} else if (!strcmp(verb, "mode_pwm1")) {
		new_mode = PORT_PWM1;
#endif

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 6

	} else if (!strcmp(verb, "mode_pwm6")) {
		new_mode = PORT_PWM6;

#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 5

	} else if (!strcmp(verb, "mode_pwm5")) {
		new_mode = PORT_PWM5;

#  if defined(BOARD_HAS_CAPTURE)

	} else if (!strcmp(verb, "mode_pwm5cap1")) {
		new_mode = PORT_PWM5CAP1;
#  endif

	} else if (!strcmp(verb, "mode_pwm4")) {
		new_mode = PORT_PWM4;

#  if defined(BOARD_HAS_CAPTURE)

	} else if (!strcmp(verb, "mode_pwm4cap1")) {
		new_mode = PORT_PWM4CAP1;

	} else if (!strcmp(verb, "mode_pwm4cap2")) {
		new_mode = PORT_PWM4CAP2;
#  endif

	} else if (!strcmp(verb, "mode_pwm3")) {
		new_mode = PORT_PWM3;

#  if defined(BOARD_HAS_CAPTURE)

	} else if (!strcmp(verb, "mode_pwm3cap1")) {
		new_mode = PORT_PWM3CAP1;
#  endif

	} else if (!strcmp(verb, "mode_pwm2")) {
		new_mode = PORT_PWM2;

#  if defined(BOARD_HAS_CAPTURE)

	} else if (!strcmp(verb, "mode_pwm2cap2")) {
		new_mode = PORT_PWM2CAP2;
#  endif
#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 8

	} else if (!strcmp(verb, "mode_pwm8")) {
		new_mode = PORT_PWM8;
#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 12

	} else if (!strcmp(verb, "mode_pwm12")) {
		new_mode = PORT_PWM12;
#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 14

	} else if (!strcmp(verb, "mode_pwm14")) {
		new_mode = PORT_PWM14;
#endif

	}

	/* was a new mode set? */
	if (new_mode != PORT_MODE_UNSET) {
		/* switch modes */
		return PWMOut::fmu_new_mode(new_mode);
	}

	if (!strcmp(verb, "test")) {
		return test();
	}

	return print_usage("unknown command");
}

int PWMOut::print_status()
{
	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		PX4_INFO("%d - PWM_MAIN 0x%04X", _instance, _pwm_mask);

	} else if (_class_instance == CLASS_DEVICE_SECONDARY) {
		PX4_INFO("%d - PWM_AUX 0x%04X", _instance, _pwm_mask);

	} else if (_class_instance == CLASS_DEVICE_TERTIARY) {
		PX4_INFO("%d - PWM_EXTRA 0x%04X", _instance, _pwm_mask);
	}

	PX4_INFO("%d - Max update rate: %i Hz", _instance, _current_update_rate);

	const char *mode_str = nullptr;

	switch (_mode) {
	case MODE_NONE: mode_str = "no pwm"; break;

	case MODE_1PWM: mode_str = "pwm1"; break;

	case MODE_2PWM: mode_str = "pwm2"; break;

	case MODE_2PWM2CAP: mode_str = "pwm2cap2"; break;

	case MODE_3PWM: mode_str = "pwm3"; break;

	case MODE_3PWM1CAP: mode_str = "pwm3cap1"; break;

	case MODE_4PWM: mode_str = "pwm4"; break;

	case MODE_4PWM1CAP: mode_str = "pwm4cap1"; break;

	case MODE_4PWM2CAP: mode_str = "pwm4cap2"; break;

	case MODE_5PWM: mode_str = "pwm5"; break;

	case MODE_5PWM1CAP: mode_str = "pwm5cap1"; break;

	case MODE_6PWM: mode_str = "pwm6"; break;

	case MODE_8PWM: mode_str = "pwm8"; break;

	case MODE_12PWM: mode_str = "pwm12"; break;

	case MODE_14PWM: mode_str = "pwm14"; break;

	case MODE_4CAP: mode_str = "cap4"; break;

	case MODE_5CAP: mode_str = "cap5"; break;

	case MODE_6CAP: mode_str = "cap6"; break;

	default:
		break;
	}

	if (mode_str) {
		PX4_INFO("%d - PWM Mode: %s", _instance, mode_str);
	}

	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);
	_mixing_output.printStatus();

	return 0;
}

int PWMOut::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module is responsible for driving the output and reading the input pins. For boards without a separate IO chip
(eg. Pixracer), it uses the main channels. On boards with an IO chip (eg. Pixhawk), it uses the AUX channels, and the
px4io driver is used for main ones.

It listens on the actuator_controls topics, does the mixing and writes the PWM outputs.

The module is configured via mode_* commands. This defines which of the first N pins the driver should occupy.
By using mode_pwm4 for example, pins 5 and 6 can be used by the camera trigger driver or by a PWM rangefinder
driver. Alternatively, pwm_out can be started in one of the capture modes, and then drivers can register a capture
callback with ioctl calls.

### Implementation
By default the module runs on a work queue with a callback on the uORB actuator_controls topic.

### Examples
It is typically started with:
$ pwm_out mode_pwm
To drive all available pins.

Capture input (rising and falling edges) and print on the console: start pwm_out in one of the capture modes:
$ pwm_out mode_pwm3cap1
This will enable capturing on the 4th pin. Then do:
$ pwm_out test

Use the `pwm` command for further configurations (PWM rate, levels, ...), and the `mixer` command to load
mixer files.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("pwm_out", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task (without any mode set, use any of the mode_* cmds)");

	PRINT_MODULE_USAGE_PARAM_COMMENT("All of the mode_* commands will start pwm_out if not running already");

	PRINT_MODULE_USAGE_COMMAND("mode_gpio");
#if defined(BOARD_HAS_PWM)
	PRINT_MODULE_USAGE_COMMAND_DESCR("mode_pwm", "Select all available pins as PWM");
# if BOARD_HAS_PWM >= 14
	PRINT_MODULE_USAGE_COMMAND("mode_pwm14");
# endif
# if BOARD_HAS_PWM >= 12
	PRINT_MODULE_USAGE_COMMAND("mode_pwm12");
# endif
# if BOARD_HAS_PWM >= 8
	PRINT_MODULE_USAGE_COMMAND("mode_pwm8");
# endif
# if BOARD_HAS_PWM >= 6
	PRINT_MODULE_USAGE_COMMAND("mode_pwm6");
	PRINT_MODULE_USAGE_COMMAND("mode_pwm5");
	PRINT_MODULE_USAGE_COMMAND("mode_pwm5cap1");
	PRINT_MODULE_USAGE_COMMAND("mode_pwm4");
	PRINT_MODULE_USAGE_COMMAND("mode_pwm4cap1");
	PRINT_MODULE_USAGE_COMMAND("mode_pwm4cap2");
	PRINT_MODULE_USAGE_COMMAND("mode_pwm3");
	PRINT_MODULE_USAGE_COMMAND("mode_pwm3cap1");
	PRINT_MODULE_USAGE_COMMAND("mode_pwm2");
	PRINT_MODULE_USAGE_COMMAND("mode_pwm2cap2");
# endif
	PRINT_MODULE_USAGE_COMMAND("mode_pwm1");
#endif

	PRINT_MODULE_USAGE_COMMAND_DESCR("sensor_reset", "Do a sensor reset (SPI bus)");
	PRINT_MODULE_USAGE_ARG("<ms>", "Delay time in ms between reset and re-enabling", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("peripheral_reset", "Reset board peripherals");
	PRINT_MODULE_USAGE_ARG("<ms>", "Delay time in ms between reset and re-enabling", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("i2c", "Configure I2C clock rate");
	PRINT_MODULE_USAGE_ARG("<bus_id> <rate>", "Specify the bus id (>=0) and rate in Hz", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Test inputs and outputs");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int pwm_out_main(int argc, char *argv[])
{
	if (argc <= 1 || strcmp(argv[1], "-h") == 0) {
		return PWMOut::print_usage();
	}

	if (strcmp(argv[1], "start") == 0) {

		if (is_running()) {
			return 0;
		}

		int ret = 0;

		PWMOut::lock_module();

		ret = PWMOut::task_spawn(argc - 1, argv + 1);

		if (ret < 0) {
			PX4_ERR("start failed (%i)", ret);
		}

		PWMOut::unlock_module();
		return ret;

	} else if (strcmp(argv[1], "status") == 0) {
		if (PWMOut::trylock_module()) {

			unsigned count = 0;

			for (int i = 0; i < PWM_OUT_MAX_INSTANCES; i++) {
				if (_objects[i].load()) {
					PX4_INFO_RAW("\n");
					_objects[i].load()->print_status();
					count++;
				}
			}

			PWMOut::unlock_module();

			if (count == 0) {
				PX4_INFO("not running");
				return 1;
			}

		} else {
			PX4_WARN("module locked, try again later");
		}

		return 0;

	} else if (strcmp(argv[1], "stop") == 0) {
		PWMOut::lock_module();

		if (argc > 2) {
			int instance = atoi(argv[2]);

			if (instance >= 0 && instance < PWM_OUT_MAX_INSTANCES) {
				PX4_INFO("stopping instance %d", instance);
				PWMOut *inst = _objects[instance].load();

				if (inst) {
					inst->request_stop();
					px4_usleep(20000); // 20 ms
					delete inst;
					_objects[instance].store(nullptr);
				}
			} else {
				PX4_ERR("invalid instance %d", instance);
			}

		} else {
			// otherwise stop everything
			bool was_running = false;

			for (int i = 0; i < PWM_OUT_MAX_INSTANCES; i++) {
				PWMOut *inst = _objects[i].load();

				if (inst) {
					PX4_INFO("stopping pwm_out instance %d", i);
					was_running = true;
					inst->request_stop();
					px4_usleep(20000); // 20 ms
					delete inst;
					_objects[i].store(nullptr);
				}
			}

			if (!was_running) {
				PX4_WARN("not running");
			}
		}

		PWMOut::unlock_module();
		return PX4_OK;
	}

	PWMOut::lock_module(); // Lock here, as the method could access _object.
	int ret = PWMOut::custom_command(argc - 1, argv + 1);
	PWMOut::unlock_module();

	return ret;
}
