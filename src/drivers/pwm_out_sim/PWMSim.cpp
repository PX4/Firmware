/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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

#include "PWMSim.hpp"

#include <mathlib/mathlib.h>
#include <px4_platform_common/getopt.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>

PWMSim::PWMSim(bool hil_mode_enabled) :
	CDev(PWM_OUTPUT0_DEVICE_PATH),
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default)
{
	_mixing_output.setAllDisarmedValues(PWM_SIM_DISARMED_MAGIC);
	_mixing_output.setAllFailsafeValues(PWM_SIM_FAILSAFE_MAGIC);
	_mixing_output.setAllMinValues(PWM_SIM_PWM_MIN_MAGIC);
	_mixing_output.setAllMaxValues(PWM_SIM_PWM_MAX_MAGIC);

	_mixing_output.setIgnoreLockdown(hil_mode_enabled);

	CDev::init();
}

void
PWMSim::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		exit_and_cleanup();
		return;
	}

	_mixing_output.update();

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);
		updateParams();

		_mixing_output.updateParams();
	}

	// check at end of cycle (updateSubscriptions() can potentially change to a different WorkQueue thread)
	_mixing_output.updateSubscriptions(true);
}

bool
PWMSim::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs,
		      unsigned num_control_groups_updated)
{
	// Nothing to do, as we are only interested in the actuator_outputs topic publication.
	// That should only be published once we receive actuator_controls (important for lock-step to work correctly)
	return num_control_groups_updated > 0;
}

int
PWMSim::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	int ret = OK;

	lock();

	switch (cmd) {
	case PWM_SERVO_ARM:
		break;

	case PWM_SERVO_DISARM:
		break;

	case PWM_SERVO_SET_MIN_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (i < OutputModuleInterface::MAX_ACTUATORS) {
					_mixing_output.minValue(i) = pwm->values[i];
				}
			}

			break;
		}

	case PWM_SERVO_SET_MAX_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (i < OutputModuleInterface::MAX_ACTUATORS) {
					_mixing_output.maxValue(i) = pwm->values[i];
				}
			}

			break;
		}

	case PWM_SERVO_SET_UPDATE_RATE:
		// PWMSim does not limit the update rate
		break;

	case PWM_SERVO_SET_SELECT_UPDATE_RATE:
		break;

	case PWM_SERVO_GET_DEFAULT_UPDATE_RATE:
		*(uint32_t *)arg = 9999;
		break;

	case PWM_SERVO_GET_UPDATE_RATE:
		*(uint32_t *)arg = 9999;
		break;

	case PWM_SERVO_GET_SELECT_UPDATE_RATE:
		*(uint32_t *)arg = 0;
		break;

	case PWM_SERVO_GET_FAILSAFE_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < OutputModuleInterface::MAX_ACTUATORS; i++) {
				pwm->values[i] = _mixing_output.failsafeValue(i);
			}

			pwm->channel_count = OutputModuleInterface::MAX_ACTUATORS;
			break;
		}

	case PWM_SERVO_GET_DISARMED_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < OutputModuleInterface::MAX_ACTUATORS; i++) {
				pwm->values[i] = _mixing_output.disarmedValue(i);
			}

			pwm->channel_count = OutputModuleInterface::MAX_ACTUATORS;
			break;
		}

	case PWM_SERVO_GET_MIN_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < OutputModuleInterface::MAX_ACTUATORS; i++) {
				pwm->values[i] = _mixing_output.minValue(i);
			}

			pwm->channel_count = OutputModuleInterface::MAX_ACTUATORS;
			break;
		}

	case PWM_SERVO_GET_TRIM_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < OutputModuleInterface::MAX_ACTUATORS; i++) {
				pwm->values[i] = (_mixing_output.maxValue(i) + _mixing_output.minValue(i)) / 2;
			}

			pwm->channel_count = OutputModuleInterface::MAX_ACTUATORS;
			break;
		}

	case PWM_SERVO_GET_MAX_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < OutputModuleInterface::MAX_ACTUATORS; i++) {
				pwm->values[i] = _mixing_output.maxValue(i);
			}

			pwm->channel_count = OutputModuleInterface::MAX_ACTUATORS;
			break;
		}

	case PWM_SERVO_GET_RATEGROUP(0) ... PWM_SERVO_GET_RATEGROUP(PWM_OUTPUT_MAX_CHANNELS - 1): {
			// no restrictions on output grouping
			unsigned channel = cmd - PWM_SERVO_GET_RATEGROUP(0);

			*(uint32_t *)arg = (1 << channel);
			break;
		}

	case PWM_SERVO_GET_COUNT:
		*(unsigned *)arg = OutputModuleInterface::MAX_ACTUATORS;
		break;

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

int
PWMSim::task_spawn(int argc, char *argv[])
{
	bool hil_mode = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "m:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'm':
			hil_mode = strcmp(myoptarg, "hil") == 0;
			break;

		default:
			return print_usage("unrecognized flag");
		}
	}

	PWMSim *instance = new PWMSim(hil_mode);

	if (!instance) {
		PX4_ERR("alloc failed");
		return -1;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;
	instance->ScheduleNow();
	return 0;
}

int PWMSim::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int PWMSim::print_status()
{
	_mixing_output.printStatus();
	return 0;
}

int PWMSim::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for simulated PWM outputs.

Its only function is to take `actuator_control` uORB messages,
mix them with any loaded mixer and output the result to the
`actuator_output` uORB topic.

It is used in SITL and HITL.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("pwm_out_sim", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the module");
	PRINT_MODULE_USAGE_PARAM_STRING('m', "sim", "hil|sim", "Mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int pwm_out_sim_main(int argc, char *argv[])
{
	return PWMSim::main(argc, argv);
}
