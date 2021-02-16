/****************************************************************************
 *
 *   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
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

#include "DShot.h"

char DShot::_telemetry_device[] {};
px4::atomic_bool DShot::_request_telemetry_init{false};

DShot::DShot() :
	CDev("/dev/dshot"),
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default)
{
	_mixing_output.setAllDisarmedValues(DSHOT_DISARM_VALUE);
	_mixing_output.setAllMinValues(DSHOT_MIN_THROTTLE);
	_mixing_output.setAllMaxValues(DSHOT_MAX_THROTTLE);

}

DShot::~DShot()
{
	// make sure outputs are off
	up_dshot_arm(false);

	// clean up the alternate device node
	unregister_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH, _class_instance);

	perf_free(_cycle_perf);
	delete _telemetry;
}

int DShot::init()
{
	// do regular cdev init
	int ret = CDev::init();

	if (ret != OK) {
		return ret;
	}

	// try to claim the generic PWM output device node as well - it's OK if we fail at this
	_class_instance = register_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH);

	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		// lets not be too verbose
	} else if (_class_instance < 0) {
		PX4_ERR("FAILED registering class device");
	}

	_mixing_output.setDriverInstance(_class_instance);

	// Getting initial parameter values
	update_params();

	ScheduleNow();

	return OK;
}

int DShot::set_mode(const Mode mode)
{
	unsigned old_mask = _output_mask;

	/*
	 * Configure for output.
	 *
	 * Note that regardless of the configured mode, the task is always
	 * listening and mixing; the mode just selects which of the channels
	 * are presented on the output pins.
	 */
	switch (mode) {
	case MODE_1PWM:
		// default output rates
		_output_mask = 0x1;
		_outputs_initialized = false;
		_num_outputs = 1;
		break;

#if defined(BOARD_HAS_CAPTURE)

	case MODE_2PWM2CAP:	// v1 multi-port with flow control lines as PWM
		up_input_capture_set(2, Rising, 0, NULL, NULL);
		up_input_capture_set(3, Rising, 0, NULL, NULL);
		PX4_DEBUG("MODE_2PWM2CAP");
#endif

	// FALLTHROUGH

	case MODE_2PWM:	// v1 multi-port with flow control lines as PWM
		PX4_DEBUG("MODE_2PWM");

		// default output rates
		_output_mask = 0x3;
		_outputs_initialized = false;
		_num_outputs = 2;

		break;

#if defined(BOARD_HAS_CAPTURE)

	case MODE_3PWM1CAP:	// v1 multi-port with flow control lines as PWM
		PX4_DEBUG("MODE_3PWM1CAP");
		up_input_capture_set(3, Rising, 0, NULL, NULL);
#endif

	// FALLTHROUGH

	case MODE_3PWM:	// v1 multi-port with flow control lines as PWM
		PX4_DEBUG("MODE_3PWM");

		// default output rates
		_output_mask = 0x7;
		_outputs_initialized = false;
		_num_outputs = 3;

		break;

#if defined(BOARD_HAS_CAPTURE)

	case MODE_4PWM1CAP:
		PX4_DEBUG("MODE_4PWM1CAP");
		up_input_capture_set(4, Rising, 0, NULL, NULL);
#endif

	// FALLTHROUGH

	case MODE_4PWM: // v1 or v2 multi-port as 4 PWM outs
		PX4_DEBUG("MODE_4PWM");

		// default output rates
		_output_mask = 0xf;
		_outputs_initialized = false;
		_num_outputs = 4;

		break;

#if defined(BOARD_HAS_CAPTURE)

	case MODE_4PWM2CAP:
		PX4_DEBUG("MODE_4PWM2CAP");
		up_input_capture_set(5, Rising, 0, NULL, NULL);

		// default output rates
		_output_mask = 0x0f;
		_outputs_initialized = false;
		_num_outputs = 4;

		break;
#endif

#if defined(BOARD_HAS_CAPTURE)

	case MODE_5PWM1CAP:
		PX4_DEBUG("MODE_5PWM1CAP");
		up_input_capture_set(5, Rising, 0, NULL, NULL);
#endif

	// FALLTHROUGH

	case MODE_5PWM: // v1 or v2 multi-port as 5 PWM outs
		PX4_DEBUG("MODE_5PWM");

		// default output rates
		_output_mask = 0x1f;
		_outputs_initialized = false;
		_num_outputs = 5;

		break;

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 6

	case MODE_6PWM:
		PX4_DEBUG("MODE_6PWM");

		// default output rates
		_output_mask = 0x3f;
		_outputs_initialized = false;
		_num_outputs = 6;

		break;
#endif

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 8

	case MODE_8PWM: // AeroCore PWMs as 8 PWM outs
		PX4_DEBUG("MODE_8PWM");
		// default output rates
		_output_mask = 0xff;
		_outputs_initialized = false;
		_num_outputs = 8;

		break;
#endif

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 12

	case MODE_12PWM:
		PX4_DEBUG("MODE_12PWM");
		// default output rates
		_output_mask = 0xfff;
		_outputs_initialized = false;
		_num_outputs = 12;

		break;
#endif

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 14

	case MODE_14PWM:
		PX4_DEBUG("MODE_14PWM");
		// default output rates
		_output_mask = 0x3fff;
		_outputs_initialized = false;
		_num_outputs = 14;

		break;
#endif

	case MODE_NONE:
		PX4_DEBUG("MODE_NONE");
		_output_mask = 0x0;
		_outputs_initialized = false;
		_num_outputs = 0;

		if (old_mask != _output_mask) {
			// disable motor outputs
			enable_dshot_outputs(false);
		}

		break;

	default:
		return -EINVAL;
	}

	_mode = mode;
	return OK;
}

int DShot::task_spawn(int argc, char *argv[])
{
	DShot *instance = new DShot();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

void DShot::capture_trampoline(void *context, const uint32_t channel_index, const hrt_abstime edge_time,
			       const uint32_t edge_state, const uint32_t overflow)
{
	DShot *dev = static_cast<DShot *>(context);
	dev->capture_callback(channel_index, edge_time, edge_state, overflow);
}

void DShot::capture_callback(const uint32_t channel_index, const hrt_abstime edge_time,
			     const uint32_t edge_state, const uint32_t overflow)
{
	fprintf(stdout, "DShot: Capture chan:%d time:%lld state:%d overflow:%d\n", channel_index, edge_time, edge_state,
		overflow);
}

void DShot::enable_dshot_outputs(const bool enabled)
{
	if (enabled && !_outputs_initialized && _output_mask != 0) {
		DShotConfig config = (DShotConfig)_param_dshot_config.get();

		unsigned int dshot_frequency = DSHOT600;

		switch (config) {
		case DShotConfig::DShot150:
			dshot_frequency = DSHOT150;
			break;

		case DShotConfig::DShot300:
			dshot_frequency = DSHOT300;
			break;

		case DShotConfig::DShot600:
			dshot_frequency = DSHOT600;
			break;

		case DShotConfig::DShot1200:
			dshot_frequency = DSHOT1200;
			break;

		default:
			break;
		}

		int ret = up_dshot_init(_output_mask, dshot_frequency);

		if (ret != 0) {
			PX4_ERR("up_dshot_init failed (%i)", ret);
			return;
		}

		_outputs_initialized = true;
	}

	if (_outputs_initialized) {
		up_dshot_arm(enabled);
		_outputs_on = enabled;
	}
}

void DShot::update_telemetry_num_motors()
{
	if (!_telemetry) {
		return;
	}

	int motor_count = 0;

	if (_mixing_output.mixers()) {
		motor_count = _mixing_output.mixers()->get_multirotor_count();
	}

	_telemetry->handler.setNumMotors(motor_count);
}

void DShot::init_telemetry(const char *device)
{
	if (!_telemetry) {
		_telemetry = new Telemetry{};

		if (!_telemetry) {
			PX4_ERR("alloc failed");
			return;
		}
	}

	int ret = _telemetry->handler.init(device);

	if (ret != 0) {
		PX4_ERR("telemetry init failed (%i)", ret);
	}

	update_telemetry_num_motors();
}

void DShot::handle_new_telemetry_data(const int motor_index, const DShotTelemetry::EscData &data)
{
	// fill in new motor data
	esc_status_s &esc_status = _telemetry->esc_status_pub.get();

	if (motor_index < esc_status_s::CONNECTED_ESC_MAX) {
		esc_status.esc_online_flags |= 1 << motor_index;

		esc_status.esc[motor_index].timestamp       = data.time;
		esc_status.esc[motor_index].esc_rpm         = (static_cast<int>(data.erpm) * 100) / (_param_mot_pole_count.get() / 2);
		esc_status.esc[motor_index].esc_voltage     = static_cast<float>(data.voltage) * 0.01f;
		esc_status.esc[motor_index].esc_current     = static_cast<float>(data.current) * 0.01f;
		esc_status.esc[motor_index].esc_temperature = data.temperature;
		// TODO: accumulate consumption and use for battery estimation
	}

	// publish when motor index wraps (which is robust against motor timeouts)
	if (motor_index <= _telemetry->last_motor_index) {
		esc_status.timestamp = hrt_absolute_time();
		esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_DSHOT;
		esc_status.esc_count = _telemetry->handler.numMotors();
		++esc_status.counter;
		// FIXME: mark all ESC's as online, otherwise commander complains even for a single dropout
		esc_status.esc_online_flags = (1 << esc_status.esc_count) - 1;
		esc_status.esc_armed_flags = (1 << esc_status.esc_count) - 1;

		_telemetry->esc_status_pub.update();

		// reset esc data (in case a motor times out, so we won't send stale data)
		memset(&esc_status.esc, 0, sizeof(_telemetry->esc_status_pub.get().esc));
		esc_status.esc_online_flags = 0;
	}

	_telemetry->last_motor_index = motor_index;
}

int DShot::send_command_thread_safe(const dshot_command_t command, const int num_repetitions, const int motor_index)
{
	Command cmd{};
	cmd.command = command;

	if (motor_index == -1) {
		cmd.motor_mask = 0xff;

	} else {
		cmd.motor_mask = 1 << _mixing_output.reorderedMotorIndex(motor_index);
	}

	cmd.num_repetitions = num_repetitions;
	_new_command.store(&cmd);

	// wait until main thread processed it
	while (_new_command.load()) {
		px4_usleep(1000);
	}

	return 0;
}

void DShot::retrieve_and_print_esc_info_thread_safe(const int motor_index)
{
	if (_request_esc_info.load() != nullptr) {
		// already in progress (not expected to ever happen)
		return;
	}

	DShotTelemetry::OutputBuffer output_buffer{};
	output_buffer.motor_index = motor_index;

	// start the request
	_request_esc_info.store(&output_buffer);

	// wait until processed
	int max_time = 1000;

	while (_request_esc_info.load() != nullptr && max_time-- > 0) {
		px4_usleep(1000);
	}

	_request_esc_info.store(nullptr); // just in case we time out...

	if (output_buffer.buf_pos == 0) {
		PX4_ERR("No data received. If telemetry is setup correctly, try again");
		return;
	}

	DShotTelemetry::decodeAndPrintEscInfoPacket(output_buffer);
}

int DShot::request_esc_info()
{
	_telemetry->handler.redirectOutput(*_request_esc_info.load());
	_waiting_for_esc_info = true;

	int motor_index = _mixing_output.reorderedMotorIndex(_request_esc_info.load()->motor_index);

	_current_command.motor_mask = 1 << motor_index;
	_current_command.num_repetitions = 1;
	_current_command.command = DShot_cmd_esc_info;

	PX4_DEBUG("Requesting ESC info for motor %i", motor_index);
	return motor_index;
}

void DShot::mixerChanged()
{
	update_telemetry_num_motors();
}

bool DShot::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			  unsigned num_outputs, unsigned num_control_groups_updated)
{
	if (!_outputs_on) {
		return false;
	}

	int requested_telemetry_index = -1;

	if (_telemetry) {
		// check for an ESC info request. We only process it when we're not expecting other telemetry data
		if (_request_esc_info.load() != nullptr && !_waiting_for_esc_info && stop_motors
		    && !_telemetry->handler.expectingData() && !_current_command.valid()) {
			requested_telemetry_index = request_esc_info();

		} else {
			requested_telemetry_index = _mixing_output.reorderedMotorIndex(_telemetry->handler.getRequestMotorIndex());
		}
	}

	if (stop_motors) {

		// when motors are stopped we check if we have other commands to send
		for (int i = 0; i < (int)num_outputs; i++) {
			if (_current_command.valid() && (_current_command.motor_mask & (1 << i))) {
				// for some reason we need to always request telemetry when sending a command
				up_dshot_motor_command(i, _current_command.command, true);

			} else {
				up_dshot_motor_command(i, DShot_cmd_motor_stop, i == requested_telemetry_index);
			}
		}

		if (_current_command.valid()) {
			--_current_command.num_repetitions;
		}

	} else {
		for (int i = 0; i < (int)num_outputs; i++) {
			if (outputs[i] == DSHOT_DISARM_VALUE) {
				up_dshot_motor_command(i, DShot_cmd_motor_stop, i == requested_telemetry_index);

			} else {
				up_dshot_motor_data_set(i, math::min(outputs[i], static_cast<uint16_t>(DSHOT_MAX_THROTTLE)),
							i == requested_telemetry_index);
			}
		}

		// clear commands when motors are running
		_current_command.clear();
	}

	if (stop_motors || num_control_groups_updated > 0) {
		up_dshot_trigger();
	}

	return true;
}

void DShot::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	_mixing_output.update();

	// update output status if armed or if mixer is loaded
	bool outputs_on = _mixing_output.armed().armed || _mixing_output.mixers();

	if (_outputs_on != outputs_on) {
		enable_dshot_outputs(outputs_on);
	}

	if (_telemetry) {
		int telem_update = _telemetry->handler.update();

		// Are we waiting for ESC info?
		if (_waiting_for_esc_info) {
			if (telem_update != -1) {
				_request_esc_info.store(nullptr);
				_waiting_for_esc_info = false;
			}

		} else if (telem_update >= 0) {
			handle_new_telemetry_data(telem_update, _telemetry->handler.latestESCData());
		}
	}

	if (_parameter_update_sub.updated()) {
		update_params();
	}

	// telemetry device update request?
	if (_request_telemetry_init.load()) {
		init_telemetry(_telemetry_device);
		_request_telemetry_init.store(false);
	}

	// new command?
	if (!_current_command.valid()) {
		Command *new_command = _new_command.load();

		if (new_command) {
			_current_command = *new_command;
			_new_command.store(nullptr);
		}
	}

	// check at end of cycle (updateSubscriptions() can potentially change to a different WorkQueue thread)
	_mixing_output.updateSubscriptions(true);

	perf_end(_cycle_perf);
}

void DShot::update_params()
{
	parameter_update_s pupdate;
	_parameter_update_sub.copy(&pupdate);

	updateParams();

	// we use a minimum value of 1, since 0 is for disarmed
	_mixing_output.setAllMinValues(math::constrain(static_cast<int>((_param_dshot_min.get() *
				       static_cast<float>(DSHOT_MAX_THROTTLE))),
				       DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE));
}

int DShot::ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret;

	// try it as a Capture ioctl next
	ret = capture_ioctl(filp, cmd, arg);

	if (ret != -ENOTTY) {
		return ret;
	}

	// if we are in valid PWM mode, try it as a PWM ioctl as well
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
		PX4_DEBUG("not in a PWM mode");
		break;
	}

	// if nobody wants it, let CDev have it
	if (ret == -ENOTTY) {
		ret = CDev::ioctl(filp, cmd, arg);
	}

	return ret;
}

int DShot::pwm_ioctl(file *filp, const int cmd, const unsigned long arg)
{
	int ret = OK;

	PX4_DEBUG("dshot ioctl cmd: %d, arg: %ld", cmd, arg);

	lock();

	switch (cmd) {
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

			case PWM_SERVO_MODE_4CAP:
				ret = set_mode(MODE_4CAP);
				break;

			case PWM_SERVO_MODE_5CAP:
				ret = set_mode(MODE_5CAP);
				break;

			case PWM_SERVO_MODE_6CAP:
				ret = set_mode(MODE_6CAP);
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

			break;
		}

	default:
		ret = -ENOTTY;
		break;
	}

	unlock();

	return ret;
}

int DShot::capture_ioctl(file *filp, const int cmd, const unsigned long arg)
{
	int ret = -EINVAL;

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

int DShot::module_new_mode(const PortMode new_mode)
{
	if (!is_running()) {
		return -1;
	}

	DShot::Mode mode;

	mode = DShot::MODE_NONE;

	switch (new_mode) {
	case PORT_FULL_GPIO:
	case PORT_MODE_UNSET:
		break;

	case PORT_FULL_PWM:

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM == 4
		// select 4-pin PWM mode
		mode = DShot::MODE_4PWM;
#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM == 5
		mode = DShot::MODE_5PWM;
#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM == 6
		mode = DShot::MODE_6PWM;
#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM == 8
		mode = DShot::MODE_8PWM;
#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM == 12
		mode = DShot::MODE_12PWM;
#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM == 14
		mode = DShot::MODE_14PWM;
#endif
		break;

	case PORT_PWM1:
		// select 2-pin PWM mode
		mode = DShot::MODE_1PWM;
		break;

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 8

	case PORT_PWM8:
		// select 8-pin PWM mode
		mode = DShot::MODE_8PWM;
		break;
#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 6

	case PORT_PWM6:
		// select 6-pin PWM mode
		mode = DShot::MODE_6PWM;
		break;
#endif

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 5

	case PORT_PWM5:
		// select 5-pin PWM mode
		mode = DShot::MODE_5PWM;
		break;


#  if defined(BOARD_HAS_CAPTURE)

	case PORT_PWM5CAP1:
		// select 5-pin PWM mode 1 capture
		mode = DShot::MODE_5PWM1CAP;
		break;

#  endif
#endif

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 4

	case PORT_PWM4:
		// select 4-pin PWM mode
		mode = DShot::MODE_4PWM;
		break;


#  if defined(BOARD_HAS_CAPTURE)

	case PORT_PWM4CAP1:
		// select 4-pin PWM mode 1 capture
		mode = DShot::MODE_4PWM1CAP;
		break;

	case PORT_PWM4CAP2:
		// select 4-pin PWM mode 2 capture
		mode = DShot::MODE_4PWM2CAP;
		break;

#  endif

	case PORT_PWM3:
		// select 3-pin PWM mode
		mode = DShot::MODE_3PWM;
		break;

#  if defined(BOARD_HAS_CAPTURE)

	case PORT_PWM3CAP1:
		// select 3-pin PWM mode 1 capture
		mode = DShot::MODE_3PWM1CAP;
		break;
#  endif

	case PORT_PWM2:
		// select 2-pin PWM mode
		mode = DShot::MODE_2PWM;
		break;

#  if defined(BOARD_HAS_CAPTURE)

	case PORT_PWM2CAP2:
		// select 2-pin PWM mode 2 capture
		mode = DShot::MODE_2PWM2CAP;
		break;

#  endif
#endif

	default:
		return -1;
	}

	DShot *object = get_instance();

	if (mode != object->get_mode()) {
		// (re)set the output mode
		object->set_mode(mode);
	}

	return OK;
}

int DShot::custom_command(int argc, char *argv[])
{
	PortMode new_mode = PORT_MODE_UNSET;
	const char *verb = argv[0];

	if (!strcmp(verb, "telemetry")) {
		if (argc > 1) {
			// telemetry can be requested before the module is started
			strncpy(_telemetry_device, argv[1], sizeof(_telemetry_device) - 1);
			_telemetry_device[sizeof(_telemetry_device) - 1] = '\0';
			_request_telemetry_init.store(true);
		}

		return 0;
	}

	int motor_index = -1; // select motor index, default: -1=all
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "m:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'm':
			motor_index = strtol(myoptarg, nullptr, 10) - 1;
			break;

		default:
			return print_usage("unrecognized flag");
		}
	}

	struct VerbCommand {
		const char *name;
		dshot_command_t command;
		int num_repetitions;
	};

	constexpr VerbCommand commands[] = {
		{"reverse", DShot_cmd_spin_direction_reversed, 10},
		{"normal", DShot_cmd_spin_direction_normal, 10},
		{"save", DShot_cmd_save_settings, 10},
		{"3d_on", DShot_cmd_3d_mode_on, 10},
		{"3d_off", DShot_cmd_3d_mode_off, 10},
		{"beep1", DShot_cmd_beacon1, 1},
		{"beep2", DShot_cmd_beacon2, 1},
		{"beep3", DShot_cmd_beacon3, 1},
		{"beep4", DShot_cmd_beacon4, 1},
		{"beep5", DShot_cmd_beacon5, 1},
	};

	for (unsigned i = 0; i < sizeof(commands) / sizeof(commands[0]); ++i) {
		if (!strcmp(verb, commands[i].name)) {
			if (!is_running()) {
				PX4_ERR("module not running");
				return -1;
			}

			return get_instance()->send_command_thread_safe(commands[i].command, commands[i].num_repetitions, motor_index);
		}
	}

	if (!strcmp(verb, "esc_info")) {
		if (!is_running()) {
			PX4_ERR("module not running");
			return -1;
		}

		if (motor_index == -1) {
			PX4_ERR("No motor index specified");
			return -1;
		}

		if (!get_instance()->telemetry_enabled()) {
			PX4_ERR("Telemetry is not enabled, but required to get ESC info");
			return -1;
		}

		get_instance()->retrieve_and_print_esc_info_thread_safe(motor_index);
		return 0;
	}


	if (!is_running()) {
		int ret = DShot::task_spawn(argc, argv);

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

	// was a new mode set?
	if (new_mode != PORT_MODE_UNSET) {

		// switch modes
		return DShot::module_new_mode(new_mode);
	}

	return print_usage("unknown command");
}

int DShot::print_status()
{
	const char *mode_str = nullptr;

	switch (_mode) {

	case MODE_NONE: mode_str = "no outputs"; break;

	case MODE_1PWM: mode_str = "outputs1"; break;

	case MODE_2PWM: mode_str = "outputs2"; break;

	case MODE_2PWM2CAP: mode_str = "outputs2cap2"; break;

	case MODE_3PWM: mode_str = "outputs3"; break;

	case MODE_3PWM1CAP: mode_str = "outputs3cap1"; break;

	case MODE_4PWM: mode_str = "outputs4"; break;

	case MODE_4PWM1CAP: mode_str = "outputs4cap1"; break;

	case MODE_4PWM2CAP: mode_str = "outputs4cap2"; break;

	case MODE_5PWM: mode_str = "outputs5"; break;

	case MODE_5PWM1CAP: mode_str = "outputs5cap1"; break;

	case MODE_6PWM: mode_str = "outputs6"; break;

	case MODE_8PWM: mode_str = "outputs8"; break;

	case MODE_4CAP: mode_str = "cap4"; break;

	case MODE_5CAP: mode_str = "cap5"; break;

	case MODE_6CAP: mode_str = "cap6"; break;

	default:
		break;
	}

	if (mode_str) {
		PX4_INFO("Mode: %s", mode_str);
	}

	PX4_INFO("Outputs initialized: %s", _outputs_initialized ? "yes" : "no");
	PX4_INFO("Outputs on: %s", _outputs_on ? "yes" : "no");
	perf_print_counter(_cycle_perf);
	_mixing_output.printStatus();

	if (_telemetry) {
		PX4_INFO("telemetry on: %s", _telemetry_device);
		_telemetry->handler.printStatus();
	}

	return 0;
}

int DShot::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This is the DShot output driver. It is similar to the fmu driver, and can be used as drop-in replacement
to use DShot as ESC communication protocol instead of PWM.

It supports:
- DShot150, DShot300, DShot600, DShot1200
- telemetry via separate UART and publishing as esc_status message
- sending DShot commands via CLI

### Examples
Permanently reverse motor 1:
$ dshot reverse -m 1
$ dshot save -m 1
After saving, the reversed direction will be regarded as the normal one. So to reverse again repeat the same commands.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("dshot", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task (without any mode set, use any of the mode_* cmds)");

	PRINT_MODULE_USAGE_PARAM_COMMENT("All of the mode_* commands will start the module if not running already");

	PRINT_MODULE_USAGE_COMMAND("mode_gpio");
	PRINT_MODULE_USAGE_COMMAND_DESCR("mode_pwm", "Select all available pins as PWM");
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 14
	PRINT_MODULE_USAGE_COMMAND("mode_pwm14");
#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 12
	PRINT_MODULE_USAGE_COMMAND("mode_pwm12");
#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 8
	PRINT_MODULE_USAGE_COMMAND("mode_pwm8");
#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 6
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
#endif
#if defined(BOARD_HAS_PWM)
	PRINT_MODULE_USAGE_COMMAND("mode_pwm1");
#endif

	PRINT_MODULE_USAGE_COMMAND_DESCR("telemetry", "Enable Telemetry on a UART");
	PRINT_MODULE_USAGE_ARG("<device>", "UART device", false);

	// DShot commands
	PRINT_MODULE_USAGE_COMMAND_DESCR("reverse", "Reverse motor direction");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("normal", "Normal motor direction");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("save", "Save current settings");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("3d_on", "Enable 3D mode");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("3d_off", "Disable 3D mode");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("beep1", "Send Beep pattern 1");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("beep2", "Send Beep pattern 2");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("beep3", "Send Beep pattern 3");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("beep4", "Send Beep pattern 4");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("beep5", "Send Beep pattern 5");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("esc_info", "Request ESC information");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based)", false);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int dshot_main(int argc, char *argv[])
{
	return DShot::main(argc, argv);
}
