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
 * @file fmu.cpp
 *
 * Driver/configurator for the PX4 FMU
 */

#include <float.h>
#include <math.h>

#include <board_config.h>
#include <drivers/device/device.h>
#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_input_capture.h>
#include <drivers/drv_mixer.h>
#include <drivers/drv_pwm_output.h>
#include <lib/cdev/CDev.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/parameter_update.h>

using namespace time_literals;

/** Mode given via CLI */
enum PortMode {
	PORT_MODE_UNSET = 0,
	PORT_FULL_GPIO,
	PORT_FULL_PWM,
	PORT_PWM8,
	PORT_PWM6,
	PORT_PWM5,
	PORT_PWM4,
	PORT_PWM3,
	PORT_PWM2,
	PORT_PWM1,
	PORT_PWM3CAP1,
	PORT_PWM4CAP1,
	PORT_PWM4CAP2,
	PORT_PWM5CAP1,
	PORT_PWM2CAP2,
	PORT_CAPTURE,
};

#if !defined(BOARD_HAS_PWM)
#  error "board_config.h needs to define BOARD_HAS_PWM"
#endif

#define PX4FMU_DEVICE_PATH	"/dev/px4fmu"


class PX4FMU : public cdev::CDev, public ModuleBase<PX4FMU>, public OutputModuleInterface
{
public:
	enum Mode {
		MODE_NONE,
		MODE_1PWM,
		MODE_2PWM,
		MODE_2PWM2CAP,
		MODE_3PWM,
		MODE_3PWM1CAP,
		MODE_4PWM,
		MODE_4PWM1CAP,
		MODE_4PWM2CAP,
		MODE_5PWM,
		MODE_5PWM1CAP,
		MODE_6PWM,
		MODE_8PWM,
		MODE_14PWM,
		MODE_4CAP,
		MODE_5CAP,
		MODE_6CAP,
	};
	PX4FMU();
	virtual ~PX4FMU();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	void Run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	/** change the FMU mode of the running module */
	static int fmu_new_mode(PortMode new_mode);

	static int test();

	static int fake(int argc, char *argv[]);

	virtual int	ioctl(file *filp, int cmd, unsigned long arg);

	virtual int	init();

	int		set_mode(Mode mode);
	Mode		get_mode() { return _mode; }

	static int	set_i2c_bus_clock(unsigned bus, unsigned clock_hz);

	static void	capture_trampoline(void *context, uint32_t chan_index,
					   hrt_abstime edge_time, uint32_t edge_state,
					   uint32_t overflow);

	void update_pwm_trims();

	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

private:
	static constexpr int FMU_MAX_ACTUATORS = DIRECT_PWM_OUTPUT_CHANNELS;
	static_assert(FMU_MAX_ACTUATORS <= MAX_ACTUATORS, "Increase MAX_ACTUATORS if this fails");

	MixingOutput _mixing_output{FMU_MAX_ACTUATORS, *this, MixingOutput::SchedulingPolicy::Auto, true};

	Mode		_mode{MODE_NONE};

	unsigned	_pwm_default_rate{50};
	unsigned	_pwm_alt_rate{50};
	uint32_t	_pwm_alt_rate_channels{0};

	unsigned	_current_update_rate{0};

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};


	unsigned	_num_outputs{0};
	int		_class_instance{-1};

	bool		_pwm_on{false};
	uint32_t	_pwm_mask{0};
	bool		_pwm_initialized{false};
	bool		_test_mode{false};

	unsigned	_num_disarmed_set{0};

	perf_counter_t	_cycle_perf;

	void		capture_callback(uint32_t chan_index,
					 hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow);
	void		update_current_rate();
	int			set_pwm_rate(unsigned rate_map, unsigned default_rate, unsigned alt_rate);
	int			pwm_ioctl(file *filp, int cmd, unsigned long arg);
	void		update_pwm_rev_mask();
	void		update_pwm_out_state(bool on);

	void		update_params();

	static void		sensor_reset(int ms);
	static void		peripheral_reset(int ms);

	int		capture_ioctl(file *filp, int cmd, unsigned long arg);

	PX4FMU(const PX4FMU &) = delete;
	PX4FMU operator=(const PX4FMU &) = delete;

};

PX4FMU::PX4FMU() :
	CDev(PX4FMU_DEVICE_PATH),
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_mixing_output.setAllMinValues(PWM_DEFAULT_MIN);
	_mixing_output.setAllMaxValues(PWM_DEFAULT_MAX);

}

PX4FMU::~PX4FMU()
{
	/* make sure servos are off */
	up_pwm_servo_deinit();

	/* clean up the alternate device node */
	unregister_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH, _class_instance);

	perf_free(_cycle_perf);
}

int
PX4FMU::init()
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

int
PX4FMU::set_mode(Mode mode)
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
		_pwm_mask = 0x1;
		_pwm_initialized = false;
		_num_outputs = 1;
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
		_pwm_mask = 0x3;
		_pwm_initialized = false;
		_num_outputs = 2;

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
		_pwm_mask = 0x7;
		_pwm_initialized = false;
		_num_outputs = 3;

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
		_pwm_mask = 0xf;
		_pwm_initialized = false;
		_num_outputs = 4;

		break;

#if defined(BOARD_HAS_CAPTURE)

	case MODE_4PWM2CAP:
		PX4_DEBUG("MODE_4PWM2CAP");
		up_input_capture_set(5, Rising, 0, NULL, NULL);

		/* default output rates */
		_pwm_default_rate = 400;
		_pwm_alt_rate = 50;
		_pwm_alt_rate_channels = 0;
		_pwm_mask = 0x0f;
		_pwm_initialized = false;
		_num_outputs = 4;

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
		_pwm_mask = 0x1f;
		_pwm_initialized = false;
		_num_outputs = 4;

		break;

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 6

	case MODE_6PWM:
		PX4_DEBUG("MODE_6PWM");

		/* default output rates */
		_pwm_default_rate = 50;
		_pwm_alt_rate = 50;
		_pwm_alt_rate_channels = 0;
		_pwm_mask = 0x3f;
		_pwm_initialized = false;
		_num_outputs = 6;

		break;
#endif

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 8

	case MODE_8PWM: // AeroCore PWMs as 8 PWM outs
		PX4_DEBUG("MODE_8PWM");
		/* default output rates */
		_pwm_default_rate = 50;
		_pwm_alt_rate = 50;
		_pwm_alt_rate_channels = 0;
		_pwm_mask = 0xff;
		_pwm_initialized = false;
		_num_outputs = 8;

		break;
#endif

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 14

	case MODE_14PWM:
		PX4_DEBUG("MODE_14PWM");
		/* default output rates */
		_pwm_default_rate = 50;
		_pwm_alt_rate = 50;
		_pwm_alt_rate_channels = 0;
		_pwm_mask = 0x3fff;
		_pwm_initialized = false;
		_num_outputs = 14;

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

		if (old_mask != _pwm_mask) {
			/* disable servo outputs - no need to set rates */
			up_pwm_servo_deinit();
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
int
PX4FMU::set_pwm_rate(uint32_t rate_map, unsigned default_rate, unsigned alt_rate)
{
	PX4_DEBUG("set_pwm_rate %x %u %u", rate_map, default_rate, alt_rate);

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

	_current_update_rate = 0; // force update

	return OK;
}

int
PX4FMU::set_i2c_bus_clock(unsigned bus, unsigned clock_hz)
{
	return device::I2C::set_bus_clock(bus, clock_hz);
}

void
PX4FMU::update_current_rate()
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
	}

	// max interval 0.5 - 100 ms (10 - 2000Hz)
	const int update_interval_in_us = math::constrain(1000000 / max_rate, 500, 100000);

	_current_update_rate = max_rate;
	_mixing_output.setMaxTopicUpdateRate(update_interval_in_us);
}

void
PX4FMU::update_pwm_rev_mask()
{
	uint16_t &reverse_pwm_mask = _mixing_output.reverseOutputMask();
	reverse_pwm_mask = 0;

	const char *pname_format;

	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		pname_format = "PWM_MAIN_REV%d";

	} else if (_class_instance == CLASS_DEVICE_SECONDARY) {
		pname_format = "PWM_AUX_REV%d";

	} else {
		PX4_ERR("PWM REV only for MAIN and AUX");
		return;
	}

	for (unsigned i = 0; i < FMU_MAX_ACTUATORS; i++) {
		char pname[16];

		/* fill the channel reverse mask from parameters */
		sprintf(pname, pname_format, i + 1);
		param_t param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t ival = 0;
			param_get(param_h, &ival);
			reverse_pwm_mask |= ((int16_t)(ival != 0)) << i;
		}
	}
}

void
PX4FMU::update_pwm_trims()
{
	PX4_DEBUG("update_pwm_trims");

	if (!_mixing_output.mixers()) {
		return;
	}

	int16_t values[FMU_MAX_ACTUATORS] = {};

	const char *pname_format;

	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		pname_format = "PWM_MAIN_TRIM%d";

	} else if (_class_instance == CLASS_DEVICE_SECONDARY) {
		pname_format = "PWM_AUX_TRIM%d";

	} else {
		PX4_ERR("PWM TRIM only for MAIN and AUX");
		return;
	}

	for (unsigned i = 0; i < FMU_MAX_ACTUATORS; i++) {
		char pname[16];

		/* fill the struct from parameters */
		sprintf(pname, pname_format, i + 1);
		param_t param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			float pval = 0.0f;
			param_get(param_h, &pval);
			values[i] = (int16_t)(10000 * pval);
			PX4_DEBUG("%s: %d", pname, values[i]);
		}
	}

	/* copy the trim values to the mixer offsets */
	unsigned n_out = _mixing_output.mixers()->set_trims(values, FMU_MAX_ACTUATORS);
	PX4_DEBUG("set %d trims", n_out);
}

int
PX4FMU::task_spawn(int argc, char *argv[])
{
	PX4FMU *instance = new PX4FMU();

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

void
PX4FMU::capture_trampoline(void *context, uint32_t chan_index,
			   hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow)
{
	PX4FMU *dev = static_cast<PX4FMU *>(context);
	dev->capture_callback(chan_index, edge_time, edge_state, overflow);
}

void
PX4FMU::capture_callback(uint32_t chan_index,
			 hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow)
{
	fprintf(stdout, "FMU: Capture chan:%d time:%lld state:%d overflow:%d\n", chan_index, edge_time, edge_state, overflow);
}

void
PX4FMU::update_pwm_out_state(bool on)
{
	if (on && !_pwm_initialized && _pwm_mask != 0) {
		up_pwm_servo_init(_pwm_mask);
		set_pwm_rate(_pwm_alt_rate_channels, _pwm_default_rate, _pwm_alt_rate);
		_pwm_initialized = true;
	}

	up_pwm_servo_arm(on);
}


bool PX4FMU::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated)
{
	if (_test_mode) {
		return false;
	}

	/* output to the servos */
	if (_pwm_initialized) {
		for (size_t i = 0; i < num_outputs; i++) {
			up_pwm_servo_set(i, outputs[i]);
		}
	}

	/* Trigger all timer's channels in Oneshot mode to fire
	 * the oneshots with updated values.
	 */
	if (num_control_groups_updated > 0) {
		up_pwm_update();
	}

	return true;
}

void
PX4FMU::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

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
	_mixing_output.updateSubscriptions(true);

	perf_end(_cycle_perf);
}

void PX4FMU::update_params()
{
	update_pwm_rev_mask();
	update_pwm_trims();

	updateParams();
}

int
PX4FMU::ioctl(file *filp, int cmd, unsigned long arg)
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
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 14
	case MODE_14PWM:
#endif
		ret = pwm_ioctl(filp, cmd, arg);
		break;

	default:
		PX4_DEBUG("not in a PWM mode");
		break;
	}

	/* if nobody wants it, let CDev have it */
	if (ret == -ENOTTY) {
		ret = CDev::ioctl(filp, cmd, arg);
	}

	return ret;
}

int
PX4FMU::pwm_ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret = OK;

	PX4_DEBUG("fmu ioctl cmd: %d, arg: %ld", cmd, arg);

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
			break;
		}

	case PWM_SERVO_SET_TRIM_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > FMU_MAX_ACTUATORS) {
				PX4_DEBUG("error: too many trim values: %d", pwm->channel_count);
				ret = -EINVAL;
				break;
			}

			if (_mixing_output.mixers() == nullptr) {
				PX4_ERR("error: no mixer loaded");
				ret = -EIO;
				break;
			}

			/* copy the trim values to the mixer offsets */
			_mixing_output.mixers()->set_trims((int16_t *)pwm->values, pwm->channel_count);
			PX4_DEBUG("set_trims: %d, %d, %d, %d", pwm->values[0], pwm->values[1], pwm->values[2], pwm->values[3]);

			break;
		}

	case PWM_SERVO_GET_TRIM_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			if (_mixing_output.mixers() == nullptr) {
				memset(pwm, 0, sizeof(pwm_output_values));
				PX4_WARN("warning: trim values not valid - no mixer loaded");

			} else {

				pwm->channel_count = _mixing_output.mixers()->get_trims((int16_t *)pwm->values);
			}

			break;
		}

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
			up_pwm_servo_set(cmd - PWM_SERVO_SET(0), arg);

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
		*(servo_position_t *)arg = up_pwm_servo_get(cmd - PWM_SERVO_GET(0));
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
		*(uint32_t *)arg = up_pwm_servo_get_rate_group(cmd - PWM_SERVO_GET_RATEGROUP(0));
		break;

	case PWM_SERVO_GET_COUNT:
		switch (_mode) {

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 14

		case MODE_14PWM:
			*(unsigned *)arg = 14;
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

	case PWM_SERVO_SET_COUNT: {
			/* change the number of outputs that are enabled for
			 * PWM. This is used to change the split between GPIO
			 * and PWM under control of the flight config
			 * parameters.
			 */
			switch (arg) {
			case 0:
				set_mode(MODE_NONE);
				break;

			case 1:
				set_mode(MODE_1PWM);
				break;

			case 2:
				set_mode(MODE_2PWM);
				break;

			case 3:
				set_mode(MODE_3PWM);
				break;

			case 4:
				set_mode(MODE_4PWM);
				break;

			case 5:
				set_mode(MODE_5PWM);
				break;

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >=6

			case 6:
				set_mode(MODE_6PWM);
				break;
#endif

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >=8

			case 8:
				set_mode(MODE_8PWM);
				break;
#endif

			default:
				ret = -EINVAL;
				break;
			}

			break;
		}

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
			update_pwm_trims();

			break;
		}

	default:
		ret = -ENOTTY;
		break;
	}

	unlock();

	return ret;
}

void
PX4FMU::sensor_reset(int ms)
{
	if (ms < 1) {
		ms = 1;
	}

	board_spi_reset(ms, 0xffff);
}

void
PX4FMU::peripheral_reset(int ms)
{
	if (ms < 1) {
		ms = 10;
	}

	board_peripheral_reset(ms);
}

int
PX4FMU::capture_ioctl(struct file *filp, int cmd, unsigned long arg)
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

int
PX4FMU::fmu_new_mode(PortMode new_mode)
{
	if (!is_running()) {
		return -1;
	}

	PX4FMU::Mode servo_mode;

	servo_mode = PX4FMU::MODE_NONE;

	switch (new_mode) {
	case PORT_FULL_GPIO:
	case PORT_MODE_UNSET:
		break;

	case PORT_FULL_PWM:

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM == 4
		/* select 4-pin PWM mode */
		servo_mode = PX4FMU::MODE_4PWM;
#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM == 5
		servo_mode = PX4FMU::MODE_5PWM;
#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM == 6
		servo_mode = PX4FMU::MODE_6PWM;
#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM == 8
		servo_mode = PX4FMU::MODE_8PWM;
#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM == 14
		servo_mode = PX4FMU::MODE_14PWM;
#endif
		break;

	case PORT_PWM1:
		/* select 2-pin PWM mode */
		servo_mode = PX4FMU::MODE_1PWM;
		break;

#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 8

	case PORT_PWM8:
		/* select 8-pin PWM mode */
		servo_mode = PX4FMU::MODE_8PWM;
		break;
#endif
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 6

	case PORT_PWM6:
		/* select 6-pin PWM mode */
		servo_mode = PX4FMU::MODE_6PWM;
		break;

	case PORT_PWM5:
		/* select 5-pin PWM mode */
		servo_mode = PX4FMU::MODE_5PWM;
		break;


#  if defined(BOARD_HAS_CAPTURE)

	case PORT_PWM5CAP1:
		/* select 5-pin PWM mode 1 capture */
		servo_mode = PX4FMU::MODE_5PWM1CAP;
		break;

#  endif

	case PORT_PWM4:
		/* select 4-pin PWM mode */
		servo_mode = PX4FMU::MODE_4PWM;
		break;


#  if defined(BOARD_HAS_CAPTURE)

	case PORT_PWM4CAP1:
		/* select 4-pin PWM mode 1 capture */
		servo_mode = PX4FMU::MODE_4PWM1CAP;
		break;

	case PORT_PWM4CAP2:
		/* select 4-pin PWM mode 2 capture */
		servo_mode = PX4FMU::MODE_4PWM2CAP;
		break;

#  endif

	case PORT_PWM3:
		/* select 3-pin PWM mode */
		servo_mode = PX4FMU::MODE_3PWM;
		break;

#  if defined(BOARD_HAS_CAPTURE)

	case PORT_PWM3CAP1:
		/* select 3-pin PWM mode 1 capture */
		servo_mode = PX4FMU::MODE_3PWM1CAP;
		break;
#  endif

	case PORT_PWM2:
		/* select 2-pin PWM mode */
		servo_mode = PX4FMU::MODE_2PWM;
		break;

#  if defined(BOARD_HAS_CAPTURE)

	case PORT_PWM2CAP2:
		/* select 2-pin PWM mode 2 capture */
		servo_mode = PX4FMU::MODE_2PWM2CAP;
		break;

#  endif
#endif

	default:
		return -1;
	}

	PX4FMU *object = get_instance();

	if (servo_mode != object->get_mode()) {
		/* (re)set the PWM output mode */
		object->set_mode(servo_mode);
	}

	return OK;
}


namespace
{

int fmu_new_i2c_speed(unsigned bus, unsigned clock_hz)
{
	return PX4FMU::set_i2c_bus_clock(bus, clock_hz);
}

} // namespace

int
PX4FMU::test()
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
				conf.callback = &PX4FMU::capture_trampoline;
				conf.context = PX4FMU::get_instance();

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

int
PX4FMU::fake(int argc, char *argv[])
{
	if (argc < 5) {
		print_usage("not enough arguments");
		return -1;
	}

	actuator_controls_s ac;

	ac.control[0] = strtol(argv[1], 0, 0) / 100.0f;

	ac.control[1] = strtol(argv[2], 0, 0) / 100.0f;

	ac.control[2] = strtol(argv[3], 0, 0) / 100.0f;

	ac.control[3] = strtol(argv[4], 0, 0) / 100.0f;

	orb_advert_t handle = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &ac);

	if (handle == nullptr) {
		PX4_ERR("advertise failed");
		return -1;
	}

	orb_unadvertise(handle);

	actuator_armed_s aa;

	aa.armed = true;
	aa.lockdown = false;

	handle = orb_advertise(ORB_ID(actuator_armed), &aa);

	if (handle == nullptr) {
		PX4_ERR("advertise failed 2");
		return -1;
	}

	orb_unadvertise(handle);
	return 0;
}

int PX4FMU::custom_command(int argc, char *argv[])
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


	/* start the FMU if not running */
	if (!is_running()) {
		int ret = PX4FMU::task_spawn(argc, argv);

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
	}

	/* was a new mode set? */
	if (new_mode != PORT_MODE_UNSET) {

		/* switch modes */
		return PX4FMU::fmu_new_mode(new_mode);
	}

	if (!strcmp(verb, "test")) {
		return test();
	}

	if (!strcmp(verb, "fake")) {
		return fake(argc - 1, argv + 1);
	}

	return print_usage("unknown command");
}

int PX4FMU::print_status()
{
	PX4_INFO("Max update rate: %i Hz", _current_update_rate);

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

	case MODE_4CAP: mode_str = "cap4"; break;

	case MODE_5CAP: mode_str = "cap5"; break;

	case MODE_6CAP: mode_str = "cap6"; break;

	default:
		break;
	}

	if (mode_str) {
		PX4_INFO("PWM Mode: %s", mode_str);
	}

	perf_print_counter(_cycle_perf);
	_mixing_output.printStatus();

	return 0;
}

int PX4FMU::print_usage(const char *reason)
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
driver. Alternatively, the fmu can be started in one of the capture modes, and then drivers can register a capture
callback with ioctl calls.

### Implementation
By default the module runs on a work queue with a callback on the uORB actuator_controls topic.

### Examples
It is typically started with:
$ fmu mode_pwm
To drive all available pins.

Capture input (rising and falling edges) and print on the console: start the fmu in one of the capture modes:
$ fmu mode_pwm3cap1
This will enable capturing on the 4th pin. Then do:
$ fmu test

Use the `pwm` command for further configurations (PWM rate, levels, ...), and the `mixer` command to load
mixer files.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fmu", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task (without any mode set, use any of the mode_* cmds)");

	PRINT_MODULE_USAGE_PARAM_COMMENT("All of the mode_* commands will start the fmu if not running already");

	PRINT_MODULE_USAGE_COMMAND("mode_gpio");
	PRINT_MODULE_USAGE_COMMAND_DESCR("mode_pwm", "Select all available pins as PWM");
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

	PRINT_MODULE_USAGE_COMMAND_DESCR("sensor_reset", "Do a sensor reset (SPI bus)");
	PRINT_MODULE_USAGE_ARG("<ms>", "Delay time in ms between reset and re-enabling", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("peripheral_reset", "Reset board peripherals");
	PRINT_MODULE_USAGE_ARG("<ms>", "Delay time in ms between reset and re-enabling", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("i2c", "Configure I2C clock rate");
	PRINT_MODULE_USAGE_ARG("<bus_id> <rate>", "Specify the bus id (>=0) and rate in Hz", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Test inputs and outputs");
	PRINT_MODULE_USAGE_COMMAND_DESCR("fake", "Arm and send an actuator controls command");
	PRINT_MODULE_USAGE_ARG("<roll> <pitch> <yaw> <thrust>", "Control values in range [-100, 100]", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fmu_main(int argc, char *argv[])
{
	return PX4FMU::main(argc, argv);
}
