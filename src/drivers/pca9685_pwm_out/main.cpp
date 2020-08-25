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

#define PCA9685_MAX_OUTPUT_CHANNELS   16
#define PCA9685_DEVICE_BASE_PATH	"/dev/pca9685"

#define PCA9685_DEFAULT_IICBUS  1
#define PCA9685_DEFAULT_ADDRESS (0x40)

using namespace drv_pca9685_pwm;

class PWMDriverWrapper : public cdev::CDev, public ModuleBase<PWMDriverWrapper>, public OutputModuleInterface
{
public:

	PWMDriverWrapper();
	~PWMDriverWrapper() override ;

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

	PWMDriverWrapper(const PWMDriverWrapper &) = delete;
	PWMDriverWrapper operator=(const PWMDriverWrapper &) = delete;

	int print_status() override;

private:
	perf_counter_t	_cycle_perf;

	void Run() override;

protected:
	void updateParams() override;

	void updatePWMParams();

	void updatePWMParamTrim();

	PCA9685 *pca9685 = nullptr; // driver handle.

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)}; // param handle

	MixingOutput _mixing_output{PCA9685_MAX_OUTPUT_CHANNELS, *this, MixingOutput::SchedulingPolicy::Auto, true};

private:

};

PWMDriverWrapper::PWMDriverWrapper() :
	CDev(PCA9685_DEVICE_BASE_PATH),
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_mixing_output.setAllMinValues(PWM_DEFAULT_MIN);
	_mixing_output.setAllMaxValues(PWM_DEFAULT_MAX);
}

PWMDriverWrapper::~PWMDriverWrapper()
{
	if (pca9685 != nullptr) { // normally this should not be called.
		PX4_DEBUG("Destruction of PWMDriverWrapper without pwmDevice unloaded!");
		pca9685->Stop(); // force stop
		delete pca9685;
		pca9685 = nullptr;
	}

	perf_free(_cycle_perf);
}

int PWMDriverWrapper::init()
{
	int ret = CDev::init();

	if (ret != PX4_OK) {
		return ret;
	}

	ret = pca9685->Start();

	if (ret != PX4_OK) {
		return ret;
	}

	updatePWMParams();

	if (!_mixing_output.updateSubscriptions(true)) {  // change to proper wq
		ScheduleNow();
	}

	return PX4_OK;
}

void PWMDriverWrapper::updateParams()
{
	updatePWMParams();
	ModuleParams::updateParams();
}

void PWMDriverWrapper::updatePWMParams()
{
	// update pwm params
	const char *pname_format_main_max = "PWM_MAIN_MAX%d";
	const char *pname_format_main_min = "PWM_MAIN_MIN%d";
	const char *pname_format_main_fail = "PWM_MAIN_FAIL%d";
	const char *pname_format_main_dis = "PWM_MAIN_DIS%d";
	const char *pname_format_main_rev = "PWM_MAIN_REV%d";
	const char *pname_format_aux_max = "PWM_AUX_MAX%d";
	const char *pname_format_aux_min = "PWM_AUX_MIN%d";
	const char *pname_format_aux_fail = "PWM_AUX_FAIL%d";
	const char *pname_format_aux_dis = "PWM_AUX_DIS%d";
	const char *pname_format_aux_rev = "PWM_AUX_REV%d";

	param_t param_h = param_find("PWM_MAX");

	if (param_h != PARAM_INVALID) {
		int32_t pval = 0;
		param_get(param_h, &pval);
		_mixing_output.setAllMaxValues(pval);

	} else {
		PX4_DEBUG("PARAM_INVALID: %s", "PWM_MAX");
	}

	param_h = param_find("PWM_MIN");

	if (param_h != PARAM_INVALID) {
		int32_t pval = 0;
		param_get(param_h, &pval);
		_mixing_output.setAllMinValues(pval);

	} else {
		PX4_DEBUG("PARAM_INVALID: %s", "PWM_MIN");
	}

	param_h = param_find("PWM_RATE");

	if (param_h != PARAM_INVALID) {
		int32_t pval = 0;
		param_get(param_h, &pval);

		if (pca9685->setFreq(pval) != PX4_OK) {
			PX4_DEBUG("failed to set pwm frequency");
		}

	} else {
		PX4_DEBUG("PARAM_INVALID: %s", "PWM_RATE");
	}

	for (int i = 0; i < 8; i++) {
		char pname[16];

		sprintf(pname, pname_format_main_max, i + 1);
		param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t pval = 0;
			param_get(param_h, &pval);

			if (pval != -1) {
				_mixing_output.maxValue(i) = pval;
			}

		} else {
			PX4_DEBUG("PARAM_INVALID: %s", pname);
		}

		sprintf(pname, pname_format_main_min, i + 1);
		param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t pval = 0;
			param_get(param_h, &pval);

			if (pval != -1) {
				_mixing_output.minValue(i) = pval;
			}

		} else {
			PX4_DEBUG("PARAM_INVALID: %s", pname);
		}

		sprintf(pname, pname_format_main_fail, i + 1);
		param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t pval = 0;
			param_get(param_h, &pval);

			if (pval != -1) {
				_mixing_output.failsafeValue(i) = pval;
			}

		} else {
			PX4_DEBUG("PARAM_INVALID: %s", pname);
		}

		sprintf(pname, pname_format_main_dis, i + 1);
		param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t pval = 0;
			param_get(param_h, &pval);

			if (pval != -1) {
				_mixing_output.disarmedValue(i) = pval;
			}

		} else {
			PX4_DEBUG("PARAM_INVALID: %s", pname);
		}

		sprintf(pname, pname_format_main_rev, i + 1);
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

		sprintf(pname, pname_format_aux_max, i + 1);
		param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t pval = 0;
			param_get(param_h, &pval);

			if (pval != -1) {
				_mixing_output.maxValue(i + 8) = pval;
			}

		} else {
			PX4_DEBUG("PARAM_INVALID: %s", pname);
		}

		sprintf(pname, pname_format_aux_min, i + 1);
		param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t pval = 0;
			param_get(param_h, &pval);

			if (pval != -1) {
				_mixing_output.minValue(i + 8) = pval;
			}

		} else {
			PX4_DEBUG("PARAM_INVALID: %s", pname);
		}

		sprintf(pname, pname_format_aux_fail, i + 1);
		param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t pval = 0;
			param_get(param_h, &pval);

			if (pval != -1) {
				_mixing_output.failsafeValue(i + 8) = pval;
			}

		} else {
			PX4_DEBUG("PARAM_INVALID: %s", pname);
		}

		sprintf(pname, pname_format_aux_dis, i + 1);
		param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t pval = 0;
			param_get(param_h, &pval);

			if (pval != -1) {
				_mixing_output.disarmedValue(i + 8) = pval;
			}

		} else {
			PX4_DEBUG("PARAM_INVALID: %s", pname);
		}

		sprintf(pname, pname_format_aux_rev, i + 1);
		param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			uint16_t &reverse_pwm_mask = _mixing_output.reverseOutputMask();
			int32_t pval = 0;
			param_get(param_h, &pval);
			reverse_pwm_mask &= (0xfffe << (i + 8)); // clear this bit
			reverse_pwm_mask |= (((uint16_t)(pval != 0)) << (i + 8)); // set to new val

		} else {
			PX4_DEBUG("PARAM_INVALID: %s", pname);
		}
	}

	if (_mixing_output.mixers()) { // only update trims if mixer loaded
		updatePWMParamTrim();
	}
}

void PWMDriverWrapper::updatePWMParamTrim()
{
	const char *pname_format_main_trim = "PWM_MAIN_TRIM%d";
	const char *pname_format_aux_trim = "PWM_AUX_TRIM%d";

	int16_t trim_values[PCA9685_MAX_OUTPUT_CHANNELS] = {};

	for (int i = 0; i < 8; i++) {
		char pname[16];

		sprintf(pname, pname_format_main_trim, i + 1);
		param_t param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			float pval = 0.0f;
			param_get(param_h, &pval);
			trim_values[i] = (int16_t)(10000 * pval);

		} else {
			PX4_DEBUG("PARAM_INVALID: %s", pname);
		}

		sprintf(pname, pname_format_aux_trim, i + 1);
		param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			float pval = 0.0f;
			param_get(param_h, &pval);
			trim_values[i + 8] = (int16_t)(10000 * pval);

		} else {
			PX4_DEBUG("PARAM_INVALID: %s", pname);
		}
	}

	unsigned n_out = _mixing_output.mixers()->set_trims(trim_values, PCA9685_MAX_OUTPUT_CHANNELS);
	PX4_DEBUG("set %d trims", n_out);
}

bool PWMDriverWrapper::updateOutputs(bool stop_motors, uint16_t *outputs, unsigned num_outputs,
				     unsigned num_control_groups_updated)
{
	/*char buf[1024]="PWM:";
	for(uint i=0;i<num_outputs;++i){
	    sprintf(buf,"%s %.4d",buf,outputs[i]);
	}
	PX4_INFO("%s",buf);*/
	return pca9685->updatePWM(outputs, num_outputs);;
}

void PWMDriverWrapper::Run()
{
	hrt_abstime timestamp = hrt_absolute_time();

	if (should_exit()) {
		PX4_INFO("PCA9685 stopping.");
		ScheduleClear();
		_mixing_output.unregister();

		pca9685->Stop();
		delete pca9685;
		pca9685 = nullptr;

		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	_mixing_output.update();

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}

	perf_end(_cycle_perf);

	int nextDelay = 1000000 / pca9685->getFrequency() - (hrt_absolute_time() - timestamp);

	if (nextDelay < 0) {
		PX4_DEBUG("PCA9685: can not follow up. %d us");
		ScheduleNow();

	} else {
		ScheduleDelayed((uint32_t)nextDelay);
	}
}

// TODO
int PWMDriverWrapper::ioctl(cdev::file_t *filep, int cmd, unsigned long arg)
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

int PWMDriverWrapper::print_usage(const char *reason)
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
`mixer load /dev/pca9685 ROMFS/px4fmu_common/mixers/quad_x.main.mix`
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("pca9685_pwm_out", "driver");
    PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task");
    PRINT_MODULE_USAGE_PARAM_INT('a',64,0,255,"device address on this bus",true);
    PRINT_MODULE_USAGE_PARAM_INT('b',1,0,255,"bus that pca9685 is connected to",true);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int PWMDriverWrapper::print_status() {
    int ret =  ModuleBase::print_status();
    PX4_INFO("PCA9685 @I2C Bus %d, address 0x%.2x, true frequency %.5f",
            pca9685->get_device_bus(),
            pca9685->get_device_address(),
             (double)(pca9685->getFrequency()));

    return ret;
}

int PWMDriverWrapper::custom_command(int argc, char **argv) { // only for test use
    return PX4_OK;
}

int PWMDriverWrapper::task_spawn(int argc, char **argv) {

    auto *instance = new PWMDriverWrapper();

    if (instance) {
        _object.store(instance);
        _task_id = task_id_is_work_queue;

        int ch;
        int address=PCA9685_DEFAULT_ADDRESS;
        int iicbus=PCA9685_DEFAULT_IICBUS;

        int myoptind = 1;
        const char *myoptarg = nullptr;
        while ((ch = px4_getopt(argc, argv, "a:b:", &myoptind, &myoptarg)) != EOF) {
            switch (ch) {
                case 'a':
                    address = atoi(myoptarg);
                    break;

                case 'b':
                    iicbus = atoi(myoptarg);
                    break;

                case '?':
                    PX4_WARN("Unsupported args");
                    goto driverInstanceAllocFailed;

                default:
                    break;
            }
        }

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
    }

    driverInstanceAllocFailed:
    delete instance;
    _object.store(nullptr);
    _task_id = -1;

    return PX4_ERROR;
}

void PWMDriverWrapper::mixerChanged() {
    OutputModuleInterface::mixerChanged();
    if (_mixing_output.mixers()) { // only update trims if mixer loaded
        updatePWMParamTrim();
    }
    if(!_mixing_output.updateSubscriptions(true)) {   // change to proper wq
        ScheduleNow();
    }
}

extern "C" __EXPORT int pca9685_pwm_out_main(int argc, char *argv[]);

int pca9685_pwm_out_main(int argc, char *argv[]){
	return PWMDriverWrapper::main(argc, argv);
}