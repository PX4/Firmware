/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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

/**
 * @file pca9685/main.cpp
 * A cross-platform driver and wrapper for pca9685.
 * Designed to support all control-groups by binding to correct mixer files
 * @author SalimTerryLi <lhf2613@gmail.com>
 */

#include <px4_log.h>
#include <drivers/device/device.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <px4_platform_common/module.h>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_mixer.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/getopt.h>

#include "PCA9685.h"

#define PCA9685_DEFAULT_IICBUS  1
#define PCA9685_DEFAULT_ADDRESS (0x40)

using namespace drv_pca9685_pwm;
using namespace time_literals;

class PCA9685Wrapper : public cdev::CDev, public ModuleBase<PCA9685Wrapper>, public OutputModuleInterface
{
public:

	PCA9685Wrapper(int schd_rate_limit = 400);
	~PCA9685Wrapper() override ;

	int init() override;

	int ioctl(cdev::file_t *filep, int cmd, unsigned long arg) override;

	void mixerChanged() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);
	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool updateOutputs(bool stop_motors, uint16_t *outputs, unsigned num_outputs,
			   unsigned num_control_groups_updated) override;

	PCA9685Wrapper(const PCA9685Wrapper &) = delete;
	PCA9685Wrapper operator=(const PCA9685Wrapper &) = delete;

	int print_status() override;

private:
	perf_counter_t	_cycle_perf;

	int		_class_instance{-1};

	enum class STATE : uint8_t {
		INIT,
		WAIT_FOR_OSC,
		RUNNING
	};
	STATE _state{STATE::INIT};
	// used to compare and cancel unecessary scheduling changes caused by parameter update
	int32_t _last_fetched_Freq = -1;
	// If this value is above zero, then change freq and scheduling in running state.
	float _targetFreq = -1.0f;


	void Run() override;

protected:
	void updateParams() override;

	void updatePWMParams();

	void updatePWMParamTrim();

	int _schd_rate_limit = 400;

	PCA9685 *pca9685 = nullptr; // driver handle.

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	MixingOutput _mixing_output{PCA9685_PWM_CHANNEL_COUNT, *this, MixingOutput::SchedulingPolicy::Disabled, true};
};

PCA9685Wrapper::PCA9685Wrapper(int schd_rate_limit) :
	CDev(nullptr),
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_schd_rate_limit(schd_rate_limit)
{
	_mixing_output.setAllMinValues(PWM_DEFAULT_MIN);
	_mixing_output.setAllMaxValues(PWM_DEFAULT_MAX);
}

PCA9685Wrapper::~PCA9685Wrapper()
{
	if (pca9685 != nullptr) { // normally this should not be called.
		PX4_DEBUG("Destruction of PCA9685Wrapper without pwmDevice unloaded!");
		pca9685->Stop(); // force stop
		delete pca9685;
		pca9685 = nullptr;
	}

	perf_free(_cycle_perf);
}

int PCA9685Wrapper::init()
{
	int ret = CDev::init();

	if (ret != PX4_OK) {
		return ret;
	}

	ret = pca9685->init();

	if (ret != PX4_OK) {
		return ret;
	}

	_class_instance = register_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH);

	this->ChangeWorkQeue(px4::device_bus_to_wq(pca9685->get_device_id()));

	PX4_INFO("running on I2C bus %d address 0x%.2x", pca9685->get_device_bus(), pca9685->get_device_address());

	ScheduleNow();

	return PX4_OK;
}

void PCA9685Wrapper::updateParams()
{
	updatePWMParams();
	ModuleParams::updateParams();
}

void PCA9685Wrapper::updatePWMParams()
{
	// update pwm params
	const char *pname_format_pwm_ch_max[2] = {"PWM_MAIN_MAX%d", "PWM_AUX_MAX%d"};
	const char *pname_format_pwm_ch_min[2] = {"PWM_MAIN_MIN%d", "PWM_AUX_MIN%d"};
	const char *pname_format_pwm_ch_fail[2] = {"PWM_MAIN_FAIL%d", "PWM_AUX_FAIL%d"};
	const char *pname_format_pwm_ch_dis[2] = {"PWM_MAIN_DIS%d", "PWM_AUX_DIS%d"};
	const char *pname_format_pwm_ch_rev[2] = {"PWM_MAIN_REV%d", "PWM_AUX_REV%d"};

	int32_t default_pwm_max = PWM_DEFAULT_MAX,
		default_pwm_min = PWM_DEFAULT_MIN,
		default_pwm_fail = PWM_DEFAULT_MIN,
		default_pwm_dis = PWM_MOTOR_OFF;

	param_t param_h = param_find("PWM_MAIN_MAX");

	if (param_h != PARAM_INVALID) {
		param_get(param_h, &default_pwm_max);

	} else {
		PX4_DEBUG("PARAM_INVALID: %s", "PWM_MAIN_MAX");
	}

	param_h = param_find("PWM_MAIN_MIN");

	if (param_h != PARAM_INVALID) {
		param_get(param_h, &default_pwm_min);

	} else {
		PX4_DEBUG("PARAM_INVALID: %s", "PWM_MAIN_MIN");
	}

	param_h = param_find("PWM_MAIN_RATE");

	if (param_h != PARAM_INVALID) {
		int32_t pval = 0;
		param_get(param_h, &pval);

		if (_last_fetched_Freq != pval) {
			_last_fetched_Freq = pval;
			_targetFreq = (float)pval;  // update only if changed
		}

	} else {
		PX4_DEBUG("PARAM_INVALID: %s", "PWM_MAIN_RATE");
	}

	for (int i = 0; i < PCA9685_PWM_CHANNEL_COUNT; i++) {
		char pname[16];
		uint8_t param_group, param_index;

		if (i <= 7) {	// Main channel
			param_group = 0;
			param_index = i + 1;

		} else {	// AUX
			param_group = 1;
			param_index = i - 8 + 1;
		}

		sprintf(pname, pname_format_pwm_ch_max[param_group], param_index);
		param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t pval = 0;
			param_get(param_h, &pval);

			if (pval != -1) {
				_mixing_output.maxValue(i) = pval;

			} else {
				_mixing_output.maxValue(i) = default_pwm_max;
			}

		} else {
			PX4_DEBUG("PARAM_INVALID: %s", pname);
		}

		sprintf(pname, pname_format_pwm_ch_min[param_group], param_index);
		param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t pval = 0;
			param_get(param_h, &pval);

			if (pval != -1) {
				_mixing_output.minValue(i) = pval;

			} else {
				_mixing_output.minValue(i) = default_pwm_min;
			}

		} else {
			PX4_DEBUG("PARAM_INVALID: %s", pname);
		}

		sprintf(pname, pname_format_pwm_ch_fail[param_group], param_index);
		param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t pval = 0;
			param_get(param_h, &pval);

			if (pval != -1) {
				_mixing_output.failsafeValue(i) = pval;

			} else {
				_mixing_output.failsafeValue(i) = default_pwm_fail;
			}

		} else {
			PX4_DEBUG("PARAM_INVALID: %s", pname);
		}

		sprintf(pname, pname_format_pwm_ch_dis[param_group], param_index);
		param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t pval = 0;
			param_get(param_h, &pval);

			if (pval != -1) {
				_mixing_output.disarmedValue(i) = pval;

			} else {
				_mixing_output.disarmedValue(i) = default_pwm_dis;
			}

		} else {
			PX4_DEBUG("PARAM_INVALID: %s", pname);
		}

		sprintf(pname, pname_format_pwm_ch_rev[param_group], param_index);
		param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			uint16_t &reverse_pwm_mask = _mixing_output.reverseOutputMask();
			int32_t pval = 0;
			param_get(param_h, &pval);
			reverse_pwm_mask &= (0xfffe << i);  // clear this bit
			reverse_pwm_mask |= (((uint16_t)(pval != 0)) << i); // set to new val

		} else {
			PX4_DEBUG("PARAM_INVALID: %s", pname);
		}
	}

	if (_mixing_output.mixers()) { // only update trims if mixer loaded
		updatePWMParamTrim();
	}
}

void PCA9685Wrapper::updatePWMParamTrim()
{
	const char *pname_format_pwm_ch_trim[2] = {"PWM_MAIN_TRIM%d", "PWM_AUX_TRIM%d"};

	int16_t trim_values[PCA9685_PWM_CHANNEL_COUNT] = {};

	for (int i = 0; i < PCA9685_PWM_CHANNEL_COUNT; i++) {
		char pname[16];

		uint8_t param_group, param_index;

		if (i <= 7) {	// Main channel
			param_group = 0;
			param_index = i + 1;

		} else {	// AUX
			param_group = 1;
			param_index = i - 8 + 1;
		}

		sprintf(pname, pname_format_pwm_ch_trim[param_group], param_index);
		param_t param_h = param_find(pname);
		int32_t val;

		if (param_h != PARAM_INVALID) {
			param_get(param_h, &val);
			trim_values[i] = (int16_t)val;

		} else {
			PX4_DEBUG("PARAM_INVALID: %s", pname);
		}
	}

	unsigned n_out = _mixing_output.mixers()->set_trims(trim_values, PCA9685_PWM_CHANNEL_COUNT);
	PX4_DEBUG("set %d trims", n_out);
}

bool PCA9685Wrapper::updateOutputs(bool stop_motors, uint16_t *outputs, unsigned num_outputs,
				   unsigned num_control_groups_updated)
{
	return pca9685->updatePWM(outputs, num_outputs) == 0 ? true : false;
}

void PCA9685Wrapper::Run()
{
	if (should_exit()) {
		PX4_INFO("PCA9685 stopping.");
		ScheduleClear();
		_mixing_output.unregister();
		unregister_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH, _class_instance);

		pca9685->Stop();
		delete pca9685;
		pca9685 = nullptr;

		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	switch (_state) {
	case STATE::INIT:
		pca9685->initReg();
		updatePWMParams();  // target frequency fetched, immediately apply it

		if (_targetFreq > 0.0f) {
			if (pca9685->setFreq(_targetFreq) != PX4_OK) {
				PX4_ERR("failed to set pwm frequency to %.2f, fall back to 50Hz", (double)_targetFreq);
				pca9685->setFreq(50.0f);	// this should not fail
			}

			_targetFreq = -1.0f;

		} else {
			// should not happen
			PX4_ERR("INIT failed: invalid initial frequency settings");
		}

		pca9685->startOscillator();
		_state = STATE::WAIT_FOR_OSC;
		ScheduleDelayed(500);
		break;

	case STATE::WAIT_FOR_OSC: {
			pca9685->triggerRestart();  // start actual outputting
			_state = STATE::RUNNING;
			float schedule_rate = pca9685->getFrequency();

			if (_schd_rate_limit < pca9685->getFrequency()) {
				schedule_rate = _schd_rate_limit;
			}

			ScheduleOnInterval(1000000 / schedule_rate, 1000000 / schedule_rate);
		}
		break;

	case STATE::RUNNING:
		_mixing_output.update();

		// check for parameter updates
		if (_parameter_update_sub.updated()) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			updateParams();
		}

		_mixing_output.updateSubscriptions(false);

		if (_targetFreq > 0.0f) { // check if frequency should be changed
			ScheduleClear();
			pca9685->disableAllOutput();
			pca9685->stopOscillator();

			if (pca9685->setFreq(_targetFreq) != PX4_OK) {
				PX4_ERR("failed to set pwm frequency, fall back to 50Hz");
				pca9685->setFreq(50.0f);	// this should not fail
			}

			_targetFreq = -1.0f;
			pca9685->startOscillator();
			_state = STATE::WAIT_FOR_OSC;
			ScheduleDelayed(500);
		}

		break;
	}

	perf_end(_cycle_perf);
}

// TODO
int PCA9685Wrapper::ioctl(cdev::file_t *filep, int cmd, unsigned long arg)
{
	int ret = OK;

	lock();

	switch (cmd) {
	case MIXERIOCRESET:
		_mixing_output.resetMixerThreadSafe();

		break;

	case MIXERIOCLOADBUF: {
			const char *buf = (const char *)arg;
			unsigned buflen = strlen(buf);

			ret = _mixing_output.loadMixerThreadSafe(buf, buflen);

			break;
		}

	case PWM_SERVO_GET_COUNT:
		*(unsigned *)arg = PCA9685_PWM_CHANNEL_COUNT;

		break;

	case PWM_SERVO_SET_ARM_OK:
	case PWM_SERVO_SET_FORCE_SAFETY_OFF:
	case PWM_SERVO_CLEAR_ARM_OK:
	case PWM_SERVO_SET_FORCE_SAFETY_ON:
	case PWM_SERVO_ARM:
	case PWM_SERVO_DISARM:
		break;

	default:
		ret = -ENOTTY;
		break;
	}

	unlock();

	if (ret == -ENOTTY) {
		ret = CDev::ioctl(filep, cmd, arg);
	}

	return ret;
}

int PCA9685Wrapper::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module is responsible for generate pwm pulse with PCA9685 chip.

It listens on the actuator_controls topics, does the mixing and writes the PWM outputs.

### Implementation
This module depends on ModuleBase and OutputModuleInterface.
IIC communication is based on CDev::I2C

### Examples
It is typically started with:
$ pca9685_pwm_out start -a 64 -b 1

Use the `mixer` command to load mixer files.
`mixer load /dev/pwm_outputX etc/mixers/quad_x.main.mix`
The number X can be acquired by executing
`pca9685_pwm_out status` when this driver is running.
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("pca9685_pwm_out", "driver");
    PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task");
    PRINT_MODULE_USAGE_PARAM_INT('a',64,0,255,"device address on this bus",true);
	PRINT_MODULE_USAGE_PARAM_INT('b',1,0,255,"bus that pca9685 is connected to",true);
	PRINT_MODULE_USAGE_PARAM_INT('r',400,50,400,"schedule rate limit",true);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int PCA9685Wrapper::print_status() {
    int ret =  ModuleBase::print_status();
    PX4_INFO("PCA9685 @I2C Bus %d, address 0x%.2x, true frequency %.5f",
            pca9685->get_device_bus(),
            pca9685->get_device_address(),
             (double)(pca9685->getFrequency()));
    PX4_INFO("CDev path: %s%d", PWM_OUTPUT_BASE_DEVICE_PATH, this->_class_instance);

    return ret;
}

int PCA9685Wrapper::custom_command(int argc, char **argv) { // only for test use
    return PX4_OK;
}

int PCA9685Wrapper::task_spawn(int argc, char **argv) {

	int ch;
	int address=PCA9685_DEFAULT_ADDRESS;
	int iicbus=PCA9685_DEFAULT_IICBUS;
	int schd_rate_limit=400;

	int myoptind = 1;
	const char *myoptarg = nullptr;
	while ((ch = px4_getopt(argc, argv, "a:b:r:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
			case 'a':
				address = atoi(myoptarg);
				break;

			case 'b':
				iicbus = atoi(myoptarg);
				break;

			case 'r':
				schd_rate_limit = atoi(myoptarg);
				break;

			case '?':
				PX4_WARN("Unsupported args");
				return PX4_ERROR;

			default:
				break;
		}
	}

    auto *instance = new PCA9685Wrapper(schd_rate_limit);

    if (instance) {
        _object.store(instance);
        _task_id = task_id_is_work_queue;

        instance->pca9685 = new PCA9685(iicbus, address);
        if(instance->pca9685==nullptr){
            PX4_ERR("alloc failed");
            goto driverInstanceAllocFailed;
        }

        if (instance->init() == PX4_OK) {
            return PX4_OK;
        } else {
            PX4_ERR("driver init failed");
            delete instance->pca9685;
            instance->pca9685=nullptr;
        }
    } else {
        PX4_ERR("alloc failed");
	    return PX4_ERROR;
    }

    driverInstanceAllocFailed:
    delete instance;
    _object.store(nullptr);
    _task_id = -1;

    return PX4_ERROR;
}

void PCA9685Wrapper::mixerChanged() {
    OutputModuleInterface::mixerChanged();
    if (_mixing_output.mixers()) { // only update trims if mixer loaded
        updatePWMParamTrim();
    }
    _mixing_output.updateSubscriptions(false);
}

extern "C" __EXPORT int pca9685_pwm_out_main(int argc, char *argv[]);

int pca9685_pwm_out_main(int argc, char *argv[]){
	return PCA9685Wrapper::main(argc, argv);
}
