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
#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_log.h>
#include <px4_module.h>
#include <circuit_breaker/circuit_breaker.h>
#include <lib/cdev/CDev.hpp>
#include <lib/mixer/mixer.h>
#include <parameters/param.h>
#include <perf/perf_counter.h>
#include <pwm_limit/pwm_limit.h>
#include <uORB/uORB.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/safety.h>

#define SCHEDULE_INTERVAL	2000	/**< The schedule interval in usec (500 Hz) */

static constexpr uint8_t CYCLE_COUNT = 10; /* safety switch must be held for 1 second to activate */

/*
 * Define the various LED flash sequences for each system state.
 */
#define LED_PATTERN_FMU_OK_TO_ARM 		0x0003		/**< slow blinking			*/
#define LED_PATTERN_FMU_REFUSE_TO_ARM 		0x5555		/**< fast blinking			*/
#define LED_PATTERN_IO_ARMED 			0x5050		/**< long off, then double blink 	*/
#define LED_PATTERN_FMU_ARMED 			0x5500		/**< long off, then quad blink 		*/
#define LED_PATTERN_IO_FMU_ARMED 		0xffff		/**< constantly on			*/

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

class PX4FMU : public cdev::CDev, public ModuleBase<PX4FMU>
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
	PX4FMU(bool run_as_task);
	virtual ~PX4FMU();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static PX4FMU *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/**
	 * run the main loop: if running as task, continuously iterate, otherwise execute only one single cycle
	 */
	void cycle();

	/** @see ModuleBase::print_status() */
	int print_status() override;

	/** change the FMU mode of the running module */
	static int fmu_new_mode(PortMode new_mode);

	static int test();

	static int fake(int argc, char *argv[]);

	virtual int	ioctl(file *filp, int cmd, unsigned long arg);
	virtual ssize_t	write(file *filp, const char *buffer, size_t len);

	virtual int	init();

	int		set_mode(Mode mode);
	Mode		get_mode() { return _mode; }

	static int	set_i2c_bus_clock(unsigned bus, unsigned clock_hz);

	static void	capture_trampoline(void *context, uint32_t chan_index,
					   hrt_abstime edge_time, uint32_t edge_state,
					   uint32_t overflow);

	void update_pwm_trims();

private:

	enum class MotorOrdering : int32_t {
		PX4 = 0,
		Betaflight = 1
	};

	hrt_abstime _cycle_timestamp = 0;
	hrt_abstime _last_safety_check = 0;
	hrt_abstime _time_last_mix = 0;

	static constexpr unsigned _MAX_ACTUATORS = DIRECT_PWM_OUTPUT_CHANNELS;

	Mode		_mode;
	unsigned	_pwm_default_rate;
	unsigned	_pwm_alt_rate;
	uint32_t	_pwm_alt_rate_channels;
	unsigned	_current_update_rate;
	bool 		_run_as_task;
	static struct work_s	_work;

	int		_armed_sub;
	int		_param_sub;
	int		_safety_sub;

	orb_advert_t	_outputs_pub;
	unsigned	_num_outputs;
	int		_class_instance;

	bool		_throttle_armed;
	bool		_pwm_on;
	uint32_t	_pwm_mask;
	bool		_pwm_initialized;
	bool		_test_mode;

	MixerGroup	*_mixers;

	uint32_t	_groups_required;
	uint32_t	_groups_subscribed;
	int		_control_subs[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS] {-1, -1, -1, -1};
	actuator_controls_s _controls[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS] {};
	orb_id_t	_control_topics[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS] {};
	pollfd	_poll_fds[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS] {};
	unsigned	_poll_fds_num;

	static pwm_limit_t	_pwm_limit;
	static actuator_armed_s	_armed;
	uint16_t	_failsafe_pwm[_MAX_ACTUATORS] {};
	uint16_t	_disarmed_pwm[_MAX_ACTUATORS] {};
	uint16_t	_min_pwm[_MAX_ACTUATORS] {};
	uint16_t	_max_pwm[_MAX_ACTUATORS] {};
	uint16_t	_reverse_pwm_mask;
	unsigned	_num_failsafe_set;
	unsigned	_num_disarmed_set;
	bool		_safety_off;			///< State of the safety button from the subscribed safety topic
	bool		_safety_btn_off;		///< State of the safety button read from the HW button
	bool		_safety_disabled;
	int8_t		_calibration_phase; // 0 - low pwm, 1 - high pwm
	orb_advert_t		_to_safety;
	orb_advert_t      _to_mixer_status; 	///< mixer status flags

	float _mot_t_max;	///< maximum rise time for motor (slew rate limiting)
	float _thr_mdl_fac;	///< thrust to pwm modelling factor
	Mixer::Airmode _airmode;	///< multicopter air-mode
	MotorOrdering _motor_ordering;

	perf_counter_t	_perf_control_latency;

	static bool	arm_nothrottle()
	{
		return ((_armed.prearmed && !_armed.armed) || _armed.in_esc_calibration_mode);
	}

	static void	cycle_trampoline(void *arg);
	int 		start();

	static int	control_callback(uintptr_t handle,
					 uint8_t control_group,
					 uint8_t control_index,
					 float &input);
	void		capture_callback(uint32_t chan_index,
					 hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow);
	void		subscribe();
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

	void safety_check_button(void);
	void flash_safety_button(void);

	/**
	 * Reorder PWM outputs according to _motor_ordering
	 * @param values PWM values to reorder
	 */
	inline void reorder_outputs(uint16_t values[_MAX_ACTUATORS]);
};

pwm_limit_t		PX4FMU::_pwm_limit;
actuator_armed_s	PX4FMU::_armed = {};
work_s	PX4FMU::_work = {};

PX4FMU::PX4FMU(bool run_as_task) :
	CDev(PX4FMU_DEVICE_PATH),
	_mode(MODE_NONE),
	_pwm_default_rate(50),
	_pwm_alt_rate(50),
	_pwm_alt_rate_channels(0),
	_current_update_rate(0),
	_run_as_task(run_as_task),
	_armed_sub(-1),
	_param_sub(-1),
	_safety_sub(-1),
	_outputs_pub(nullptr),
	_num_outputs(0),
	_class_instance(0),
	_throttle_armed(false),
	_pwm_on(false),
	_pwm_mask(0),
	_pwm_initialized(false),
	_test_mode(false),
	_mixers(nullptr),
	_groups_required(0),
	_groups_subscribed(0),
	_poll_fds_num(0),
	_reverse_pwm_mask(0),
	_num_failsafe_set(0),
	_num_disarmed_set(0),
	_safety_off(false),
	_safety_btn_off(false),
	_safety_disabled(false),
	_calibration_phase(-1),
	_to_safety(nullptr),
	_to_mixer_status(nullptr),
	_mot_t_max(0.0f),
	_thr_mdl_fac(0.0f),
	_airmode(Mixer::Airmode::disabled),
	_motor_ordering(MotorOrdering::PX4),
	_perf_control_latency(perf_alloc(PC_ELAPSED, "fmu control latency"))
{
	for (unsigned i = 0; i < _MAX_ACTUATORS; i++) {
		_min_pwm[i] = PWM_DEFAULT_MIN;
		_max_pwm[i] = PWM_DEFAULT_MAX;
	}

	_control_topics[0] = ORB_ID(actuator_controls_0);
	_control_topics[1] = ORB_ID(actuator_controls_1);
	_control_topics[2] = ORB_ID(actuator_controls_2);
	_control_topics[3] = ORB_ID(actuator_controls_3);

	for (int i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; ++i) {
		_control_subs[i] = -1;
	}

	memset(_controls, 0, sizeof(_controls));
	memset(_poll_fds, 0, sizeof(_poll_fds));

	// Safely initialize armed flags.
	_armed.armed = false;
	_armed.prearmed = false;
	_armed.ready_to_arm = false;
	_armed.lockdown = false;
	_armed.force_failsafe = false;
	_armed.in_esc_calibration_mode = false;

	// If there is no safety button, disable it on boot.
#ifndef GPIO_BTN_SAFETY
	_safety_off = true;
	_safety_btn_off = true;
#endif
}

PX4FMU::~PX4FMU()
{
	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (_control_subs[i] >= 0) {
			orb_unsubscribe(_control_subs[i]);
		}
	}

	orb_unsubscribe(_armed_sub);
	orb_unsubscribe(_param_sub);
	orb_unsubscribe(_safety_sub);

	orb_unadvertise(_outputs_pub);
	orb_unadvertise(_to_safety);
	orb_unadvertise(_to_mixer_status);

	/* make sure servos are off */
	up_pwm_servo_deinit();

	/* note - someone else is responsible for restoring the GPIO config */

	/* clean up the alternate device node */
	unregister_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH, _class_instance);

	perf_free(_perf_control_latency);
}

int
PX4FMU::init()
{
	int ret;

	/* do regular cdev init */
	ret = CDev::init();

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

	_safety_disabled = circuit_breaker_enabled("CBRK_IO_SAFETY", CBRK_IO_SAFETY_KEY);

	if (_safety_disabled) {
		_safety_off = true;
		_safety_btn_off = true;
	}

	/* force a reset of the update rate */
	_current_update_rate = 0;

	_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	_param_sub = orb_subscribe(ORB_ID(parameter_update));
	_safety_sub = orb_subscribe(ORB_ID(safety));

	/* initialize PWM limit lib */
	pwm_limit_init(&_pwm_limit);

	// Getting initial parameter values
	update_params();

	return 0;
}

void
PX4FMU::safety_check_button(void)
{
#ifdef GPIO_BTN_SAFETY

	if (!PX4_MFT_HW_SUPPORTED(PX4_MFT_PX4IO)) {

		static int counter = 0;
		/*
		 * Debounce the safety button, change state if it has been held for long enough.
		 *
		 */
		bool safety_button_pressed = px4_arch_gpioread(GPIO_BTN_SAFETY);

		/*
		 * Keep pressed for a while to arm.
		 *
		 * Note that the counting sequence has to be same length
		 * for arming / disarming in order to end up as proper
		 * state machine, keep ARM_COUNTER_THRESHOLD the same
		 * length in all cases of the if/else struct below.
		 */
		if (safety_button_pressed && !_safety_btn_off) {

			if (counter < CYCLE_COUNT) {
				counter++;

			} else if (counter == CYCLE_COUNT) {
				/* switch to armed state */
				_safety_btn_off = true;
				counter++;
			}

		} else if (safety_button_pressed && _safety_btn_off) {

			if (counter < CYCLE_COUNT) {
				counter++;

			} else if (counter == CYCLE_COUNT) {
				/* change to disarmed state and notify the FMU */
				_safety_btn_off = false;
				counter++;
			}

		} else {
			counter = 0;
		}
	}

#endif
}

void
PX4FMU::flash_safety_button()
{
#if defined(GPIO_BTN_SAFETY) &&  defined(GPIO_LED_SAFETY)

	if (!PX4_MFT_HW_SUPPORTED(PX4_MFT_PX4IO)) {
		/* Select the appropriate LED flash pattern depending on the current arm state */
		uint16_t pattern = LED_PATTERN_FMU_REFUSE_TO_ARM;

		/* cycle the blink state machine at 10Hz */
		static int blink_counter = 0;

		if (_safety_btn_off) {
			if (_armed.armed) {
				pattern = LED_PATTERN_IO_FMU_ARMED;

			} else {
				pattern = LED_PATTERN_IO_ARMED;
			}

		} else if (_armed.armed) {
			pattern = LED_PATTERN_FMU_ARMED;

		} else {
			pattern = LED_PATTERN_FMU_OK_TO_ARM;

		}

		/* Turn the LED on if we have a 1 at the current bit position */
		px4_arch_gpiowrite(GPIO_LED_SAFETY, !(pattern & (1 << blink_counter++)));

		if (blink_counter > 15) {
			blink_counter = 0;
		}
	}

#endif
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

		/* We should note that group is iterated over from 0 to _MAX_ACTUATORS.
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

		for (unsigned group = 0; group < _MAX_ACTUATORS; group++) {

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

	return OK;
}

int
PX4FMU::set_i2c_bus_clock(unsigned bus, unsigned clock_hz)
{
	return device::I2C::set_bus_clock(bus, clock_hz);
}

void
PX4FMU::subscribe()
{
	/* subscribe/unsubscribe to required actuator control groups */
	uint32_t sub_groups = _groups_required & ~_groups_subscribed;
	uint32_t unsub_groups = _groups_subscribed & ~_groups_required;
	_poll_fds_num = 0;

	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (sub_groups & (1 << i)) {
			PX4_DEBUG("subscribe to actuator_controls_%d", i);
			_control_subs[i] = orb_subscribe(_control_topics[i]);
		}

		if (unsub_groups & (1 << i)) {
			PX4_DEBUG("unsubscribe from actuator_controls_%d", i);
			orb_unsubscribe(_control_subs[i]);
			_control_subs[i] = -1;
		}

		if (_control_subs[i] >= 0) {
			_poll_fds[_poll_fds_num].fd = _control_subs[i];
			_poll_fds[_poll_fds_num].events = POLLIN;
			_poll_fds_num++;
		}
	}
}

void
PX4FMU::update_pwm_rev_mask()
{
	_reverse_pwm_mask = 0;

	const char *pname_format;

	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		pname_format = "PWM_MAIN_REV%d";

	} else if (_class_instance == CLASS_DEVICE_SECONDARY) {
		pname_format = "PWM_AUX_REV%d";

	} else {
		PX4_ERR("PWM REV only for MAIN and AUX");
		return;
	}

	for (unsigned i = 0; i < _MAX_ACTUATORS; i++) {
		char pname[16];

		/* fill the channel reverse mask from parameters */
		sprintf(pname, pname_format, i + 1);
		param_t param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t ival = 0;
			param_get(param_h, &ival);
			_reverse_pwm_mask |= ((int16_t)(ival != 0)) << i;
		}
	}
}

void
PX4FMU::update_pwm_trims()
{
	PX4_DEBUG("update_pwm_trims");

	if (_mixers != nullptr) {

		int16_t values[_MAX_ACTUATORS] = {};

		const char *pname_format;

		if (_class_instance == CLASS_DEVICE_PRIMARY) {
			pname_format = "PWM_MAIN_TRIM%d";

		} else if (_class_instance == CLASS_DEVICE_SECONDARY) {
			pname_format = "PWM_AUX_TRIM%d";

		} else {
			PX4_ERR("PWM TRIM only for MAIN and AUX");
			return;
		}

		for (unsigned i = 0; i < _MAX_ACTUATORS; i++) {
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
		unsigned n_out = _mixers->set_trims(values, _MAX_ACTUATORS);
		PX4_DEBUG("set %d trims", n_out);
	}
}

int
PX4FMU::task_spawn(int argc, char *argv[])
{
	bool run_as_task = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "t", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 't':
			run_as_task = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return -1;
	}


	if (!run_as_task) {

		/* schedule a cycle to start things */
		int ret = work_queue(HPWORK, &_work, (worker_t)&PX4FMU::cycle_trampoline, nullptr, 0);

		if (ret < 0) {
			return ret;
		}

		_task_id = task_id_is_work_queue;

	} else {

		/* start the IO interface task */

		_task_id = px4_task_spawn_cmd("fmu",
					      SCHED_DEFAULT,
					      SCHED_PRIORITY_ACTUATOR_OUTPUTS,
					      1340,
					      (px4_main_t)&run_trampoline,
					      nullptr);

		if (_task_id < 0) {
			_task_id = -1;
			return -errno;
		}
	}

	// wait until task is up & running (the mode_* commands depend on it)
	if (wait_until_running() < 0) {
		_task_id = -1;
		return -1;
	}

	return PX4_OK;
}

void
PX4FMU::cycle_trampoline(void *arg)
{
	PX4FMU *dev = reinterpret_cast<PX4FMU *>(arg);

	// check if the trampoline is called for the first time
	if (!dev) {
		dev = new PX4FMU(false);

		if (!dev) {
			PX4_ERR("alloc failed");
			return;
		}

		if (dev->init() != 0) {
			PX4_ERR("init failed");
			delete dev;
			return;
		}

		_object.store(dev);
	}

	dev->cycle();
}

void
PX4FMU::capture_trampoline(void *context, uint32_t chan_index,
			   hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow)
{
	PX4FMU *dev = reinterpret_cast<PX4FMU *>(context);
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

void
PX4FMU::run()
{
	if (init() != 0) {
		PX4_ERR("init failed");
		exit_and_cleanup();
		return;
	}

	cycle();
}

void
PX4FMU::cycle()
{
	while (true) {

		if (_groups_subscribed != _groups_required) {
			subscribe();
			_groups_subscribed = _groups_required;
			/* force setting update rate */
			_current_update_rate = 0;
		}

		int poll_timeout = 10;

		if (!_run_as_task) {
			/*
			 * Adjust actuator topic update rate to keep up with
			 * the highest servo update rate configured.
			 *
			 * We always mix at max rate; some channels may update slower.
			 */
			unsigned max_rate = (_pwm_default_rate > _pwm_alt_rate) ? _pwm_default_rate : _pwm_alt_rate;

			if (_current_update_rate != max_rate) {
				_current_update_rate = max_rate;
				int update_rate_in_ms = int(1000 / _current_update_rate);

				/* reject faster than 500 Hz updates */
				if (update_rate_in_ms < 2) {
					update_rate_in_ms = 2;
				}

				/* reject slower than 10 Hz updates */
				if (update_rate_in_ms > 100) {
					update_rate_in_ms = 100;
				}

				PX4_DEBUG("adjusted actuator update interval to %ums", update_rate_in_ms);

				for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
					if (_control_subs[i] >= 0) {
						orb_set_interval(_control_subs[i], update_rate_in_ms);
					}
				}

				// set to current max rate, even if we are actually checking slower/faster
				_current_update_rate = max_rate;
			}

			/* check if anything updated */
			poll_timeout = 0;
		}

		/* wait for an update */
		unsigned n_updates = 0;
		int ret = px4_poll(_poll_fds, _poll_fds_num, poll_timeout);

		/* this would be bad... */
		if (ret < 0) {
			PX4_DEBUG("poll error %d", errno);

		} else if (ret == 0) {
			/* timeout: no control data, switch to failsafe values */
			//			PX4_WARN("no PWM: failsafe");

		} else {
			if (_mixers != nullptr) {
				/* get controls for required topics */
				unsigned poll_id = 0;

				for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
					if (_control_subs[i] >= 0) {

						if (_poll_fds[poll_id].revents & POLLIN) {
							if (i == 0) {
								n_updates++;
							}

							orb_copy(_control_topics[i], _control_subs[i], &_controls[i]);
						}

						poll_id++;
					}
				}
			}
		} // poll_fds

		/* run the mixers on every cycle */
		{
			if (_mixers != nullptr) {

				if (_mot_t_max > FLT_EPSILON) {
					hrt_abstime now = hrt_absolute_time();
					float dt = (now - _time_last_mix) / 1e6f;
					_time_last_mix = now;

					if (dt < 0.0001f) {
						dt = 0.0001f;

					} else if (dt > 0.02f) {
						dt = 0.02f;
					}

					// maximum value the outputs of the multirotor mixer are allowed to change in this cycle
					// factor 2 is needed because actuator outputs are in the range [-1,1]
					const float delta_out_max = 2.0f * 1000.0f * dt / (_max_pwm[0] - _min_pwm[0]) / _mot_t_max;
					_mixers->set_max_delta_out_once(delta_out_max);
				}

				_mixers->set_thrust_factor(_thr_mdl_fac);
				_mixers->set_airmode(_airmode);

				/* do mixing */
				float outputs[_MAX_ACTUATORS];
				const unsigned mixed_num_outputs = _mixers->mix(outputs, _num_outputs);

				/* the PWM limit call takes care of out of band errors, NaN and constrains */
				uint16_t pwm_limited[_MAX_ACTUATORS];

				pwm_limit_calc(_throttle_armed, arm_nothrottle(), mixed_num_outputs, _reverse_pwm_mask,
					       _disarmed_pwm, _min_pwm, _max_pwm, outputs, pwm_limited, &_pwm_limit);

				/* During ESC calibration, we overwrite the PWM values. */
				if (_armed.in_esc_calibration_mode) {
					/* For each channel, set PWM to maximum or minimum depending on the calibration phase. */
					for (size_t i = 0; i < mixed_num_outputs; i++) {
						if (_calibration_phase == 1) {
							pwm_limited[i] = _max_pwm[i];

						} else if (_calibration_phase == 0) {
							// set PWM a bit smaller than MIN PWM to ensure the motors are spinning at MIN PWM
							pwm_limited[i] = _min_pwm[i] - 25;

							if (pwm_limited[i] < _disarmed_pwm[i]) {
								pwm_limited[i] = _disarmed_pwm[i];
							}
						}
					}
				}

				/* overwrite outputs in case of force_failsafe with _failsafe_pwm PWM values */
				if (_armed.force_failsafe) {
					for (size_t i = 0; i < mixed_num_outputs; i++) {
						pwm_limited[i] = _failsafe_pwm[i];
					}
				}

				/* overwrite outputs in case of lockdown or parachute triggering with disarmed PWM values */
				if (_armed.lockdown || _armed.manual_lockdown) {
					for (size_t i = 0; i < mixed_num_outputs; i++) {
						pwm_limited[i] = _disarmed_pwm[i];
					}
				}

				/* apply _motor_ordering */
				reorder_outputs(pwm_limited);

				/* output to the servos */
				if (_pwm_initialized && !_test_mode) {
					for (size_t i = 0; i < mixed_num_outputs; i++) {
						up_pwm_servo_set(i, pwm_limited[i]);
					}
				}

				/* Trigger all timer's channels in Oneshot mode to fire
				 * the oneshots with updated values.
				 */
				if (n_updates > 0 && !_test_mode) {
					up_pwm_update();
				}

				actuator_outputs_s actuator_outputs = {};
				actuator_outputs.timestamp = hrt_absolute_time();
				actuator_outputs.noutputs = mixed_num_outputs;

				// zero unused outputs
				for (size_t i = 0; i < mixed_num_outputs; ++i) {
					actuator_outputs.output[i] = pwm_limited[i];
				}

				orb_publish_auto(ORB_ID(actuator_outputs), &_outputs_pub, &actuator_outputs, &_class_instance, ORB_PRIO_DEFAULT);

				/* publish mixer status */
				MultirotorMixer::saturation_status saturation_status;
				saturation_status.value = _mixers->get_saturation_status();

				if (saturation_status.flags.valid) {
					multirotor_motor_limits_s motor_limits;
					motor_limits.timestamp = hrt_absolute_time();
					motor_limits.saturation_status = saturation_status.value;

					orb_publish_auto(ORB_ID(multirotor_motor_limits), &_to_mixer_status, &motor_limits, &_class_instance, ORB_PRIO_DEFAULT);
				}

				// use first valid timestamp_sample for latency tracking
				for (int i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
					const bool required = _groups_required & (1 << i);
					const hrt_abstime &timestamp_sample = _controls[i].timestamp_sample;

					if (required && (timestamp_sample > 0)) {
						perf_set_elapsed(_perf_control_latency, actuator_outputs.timestamp - timestamp_sample);
						break;
					}
				}
			}
		}

		_cycle_timestamp = hrt_absolute_time();

#ifdef GPIO_BTN_SAFETY

		if (!PX4_MFT_HW_SUPPORTED(PX4_MFT_PX4IO)) {

			if (_cycle_timestamp - _last_safety_check >= (unsigned int)1e5) {
				_last_safety_check = _cycle_timestamp;

				/**
				 * Get and handle the safety status at 10Hz
				 */
				struct safety_s safety = {};

				if (!_safety_disabled) {
					/* read safety switch input and control safety switch LED at 10Hz */
					safety_check_button();
				}

				/* Make the safety button flash anyway, no matter if it's used or not. */
				flash_safety_button();

				safety.timestamp = hrt_absolute_time();
				safety.safety_switch_available = true;
				safety.safety_off = _safety_btn_off;

				/* lazily publish the safety status */
				if (_to_safety != nullptr) {
					orb_publish(ORB_ID(safety), _to_safety, &safety);

				} else {
					int instance;
					_to_safety = orb_advertise_multi(ORB_ID(safety), &safety, &instance, ORB_PRIO_DEFAULT);
				}
			}
		}

#endif

		/* check safety button state */
		bool updated = false;
		orb_check(_safety_sub, &updated);

		if (updated) {
			safety_s safety;

			if (orb_copy(ORB_ID(safety), _safety_sub, &safety) == 0) {
				_safety_off = !safety.safety_switch_available || safety.safety_off;
			}
		}

		/* check arming state */
		orb_check(_armed_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);

			/* Update the armed status and check that we're not locked down.
			 * We also need to arm throttle for the ESC calibration. */
			_throttle_armed = (_safety_off && _armed.armed && !_armed.lockdown) ||
					  (_safety_off && _armed.in_esc_calibration_mode);
		}

		/* update PWM status if armed or if disarmed PWM values are set */
		bool pwm_on = _armed.armed || _num_disarmed_set > 0 || _armed.in_esc_calibration_mode;

		if (_pwm_on != pwm_on) {
			_pwm_on = pwm_on;

			update_pwm_out_state(pwm_on);
		}

		orb_check(_param_sub, &updated);

		if (updated) {
			this->update_params();
		}

		if (_run_as_task) {
			if (should_exit()) {
				break;
			}

		} else {
			if (should_exit()) {
				exit_and_cleanup();

			} else {
				/* schedule next cycle */
				work_queue(HPWORK, &_work, (worker_t)&PX4FMU::cycle_trampoline, this, USEC2TICK(SCHEDULE_INTERVAL));
			}

			break;
		}
	}
}

void PX4FMU::update_params()
{
	parameter_update_s pupdate;
	orb_copy(ORB_ID(parameter_update), _param_sub, &pupdate);

	update_pwm_rev_mask();
	update_pwm_trims();

	param_t param_handle;

	// maximum motor slew rate parameter
	param_handle = param_find("MOT_SLEW_MAX");

	if (param_handle != PARAM_INVALID) {
		param_get(param_handle, &_mot_t_max);
	}

	// thrust to pwm modelling factor
	param_handle = param_find("THR_MDL_FAC");

	if (param_handle != PARAM_INVALID) {
		param_get(param_handle, &_thr_mdl_fac);
	}

	// multicopter air-mode
	param_handle = param_find("MC_AIRMODE");

	if (param_handle != PARAM_INVALID) {
		param_get(param_handle, &_airmode);
	}

	// motor ordering
	param_handle = param_find("MOT_ORDERING");

	if (param_handle != PARAM_INVALID) {
		param_get(param_handle, (int32_t *)&_motor_ordering);
	}
}


int
PX4FMU::control_callback(uintptr_t handle,
			 uint8_t control_group,
			 uint8_t control_index,
			 float &input)
{
	const actuator_controls_s *controls = (actuator_controls_s *)handle;

	input = controls[control_group].control[control_index];

	/* limit control input */
	if (input > 1.0f) {
		input = 1.0f;

	} else if (input < -1.0f) {
		input = -1.0f;
	}

	/* motor spinup phase - lock throttle to zero */
	if (_pwm_limit.state == PWM_LIMIT_STATE_RAMP) {
		if ((control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE ||
		     control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE_ALTERNATE) &&
		    control_index == actuator_controls_s::INDEX_THROTTLE) {
			/* limit the throttle output to zero during motor spinup,
			 * as the motors cannot follow any demand yet
			 */
			input = 0.0f;
		}
	}

	/* throttle not arming - mark throttle input as invalid */
	if (arm_nothrottle() && !_armed.in_esc_calibration_mode) {
		if ((control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE ||
		     control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE_ALTERNATE) &&
		    control_index == actuator_controls_s::INDEX_THROTTLE) {
			/* set the throttle to an invalid value */
			input = NAN;
		}
	}

	return 0;
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
		break;

	case PWM_SERVO_SET_FORCE_SAFETY_OFF:
		/* force safety switch off */
		_safety_btn_off = true;
		break;

	case PWM_SERVO_SET_FORCE_SAFETY_ON:
		/* force safety switch on */
		_safety_btn_off = false;
		break;

	case PWM_SERVO_DISARM:

		/* Ignore disarm if disarmed PWM is set already. */
		if (_num_disarmed_set == 0) {
			update_pwm_out_state(false);
		}

		break;

	case PWM_SERVO_SET_CAL_LOW:
		_calibration_phase = 0;
		break;

	case PWM_SERVO_SET_CAL_HIGH:
		_calibration_phase = 1;
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
			if (pwm->channel_count > _MAX_ACTUATORS) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (pwm->values[i] == 0) {
					/* ignore 0 */
				} else if (pwm->values[i] > PWM_HIGHEST_MAX) {
					_failsafe_pwm[i] = PWM_HIGHEST_MAX;

				}

#if PWM_LOWEST_MIN > 0

				else if (pwm->values[i] < PWM_LOWEST_MIN) {
					_failsafe_pwm[i] = PWM_LOWEST_MIN;

				}

#endif

				else {
					_failsafe_pwm[i] = pwm->values[i];
				}
			}

			/*
			 * update the counter
			 * this is needed to decide if disarmed PWM output should be turned on or not
			 */
			_num_failsafe_set = 0;

			for (unsigned i = 0; i < _MAX_ACTUATORS; i++) {
				if (_failsafe_pwm[i] > 0) {
					_num_failsafe_set++;
				}
			}

			break;
		}

	case PWM_SERVO_GET_FAILSAFE_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _MAX_ACTUATORS; i++) {
				pwm->values[i] = _failsafe_pwm[i];
			}

			pwm->channel_count = _MAX_ACTUATORS;
			break;
		}

	case PWM_SERVO_SET_DISARMED_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > _MAX_ACTUATORS) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (pwm->values[i] == 0) {
					/* ignore 0 */
				} else if (pwm->values[i] > PWM_HIGHEST_MAX) {
					_disarmed_pwm[i] = PWM_HIGHEST_MAX;
				}

#if PWM_LOWEST_MIN > 0

				else if (pwm->values[i] < PWM_LOWEST_MIN) {
					_disarmed_pwm[i] = PWM_LOWEST_MIN;
				}

#endif

				else {
					_disarmed_pwm[i] = pwm->values[i];
				}
			}

			/*
			 * update the counter
			 * this is needed to decide if disarmed PWM output should be turned on or not
			 */
			_num_disarmed_set = 0;

			for (unsigned i = 0; i < _MAX_ACTUATORS; i++) {
				if (_disarmed_pwm[i] > 0) {
					_num_disarmed_set++;
				}
			}

			break;
		}

	case PWM_SERVO_GET_DISARMED_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _MAX_ACTUATORS; i++) {
				pwm->values[i] = _disarmed_pwm[i];
			}

			pwm->channel_count = _MAX_ACTUATORS;
			break;
		}

	case PWM_SERVO_SET_MIN_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > _MAX_ACTUATORS) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (pwm->values[i] == 0) {
					/* ignore 0 */
				} else if (pwm->values[i] > PWM_HIGHEST_MIN) {
					_min_pwm[i] = PWM_HIGHEST_MIN;

				}

#if PWM_LOWEST_MIN > 0

				else if (pwm->values[i] < PWM_LOWEST_MIN) {
					_min_pwm[i] = PWM_LOWEST_MIN;
				}

#endif

				else {
					_min_pwm[i] = pwm->values[i];
				}
			}

			break;
		}

	case PWM_SERVO_GET_MIN_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _MAX_ACTUATORS; i++) {
				pwm->values[i] = _min_pwm[i];
			}

			pwm->channel_count = _MAX_ACTUATORS;
			arg = (unsigned long)&pwm;
			break;
		}

	case PWM_SERVO_SET_MAX_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > _MAX_ACTUATORS) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (pwm->values[i] == 0) {
					/* ignore 0 */
				} else if (pwm->values[i] < PWM_LOWEST_MAX) {
					_max_pwm[i] = PWM_LOWEST_MAX;

				} else if (pwm->values[i] > PWM_HIGHEST_MAX) {
					_max_pwm[i] = PWM_HIGHEST_MAX;

				} else {
					_max_pwm[i] = pwm->values[i];
				}
			}

			break;
		}

	case PWM_SERVO_GET_MAX_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _MAX_ACTUATORS; i++) {
				pwm->values[i] = _max_pwm[i];
			}

			pwm->channel_count = _MAX_ACTUATORS;
			arg = (unsigned long)&pwm;
			break;
		}

	case PWM_SERVO_SET_TRIM_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > _MAX_ACTUATORS) {
				PX4_DEBUG("error: too many trim values: %d", pwm->channel_count);
				ret = -EINVAL;
				break;
			}

			if (_mixers == nullptr) {
				PX4_ERR("error: no mixer loaded");
				ret = -EIO;
				break;
			}

			/* copy the trim values to the mixer offsets */
			_mixers->set_trims((int16_t *)pwm->values, pwm->channel_count);
			PX4_DEBUG("set_trims: %d, %d, %d, %d", pwm->values[0], pwm->values[1], pwm->values[2], pwm->values[3]);

			break;
		}

	case PWM_SERVO_GET_TRIM_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			if (_mixers == nullptr) {
				memset(pwm, 0, sizeof(pwm_output_values));
				PX4_WARN("warning: trim values not valid - no mixer loaded");

			} else {

				pwm->channel_count = _mixers->get_trims((int16_t *)pwm->values);
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

	/* FALLTHROUGH */
	case PWM_SERVO_SET(4):
		if (_mode < MODE_5PWM1CAP) {
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

	case PWM_SERVO_GET(4):
		if (_mode < MODE_5PWM1CAP) {
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
#if defined(BOARD_HAS_PWM) && BOARD_HAS_PWM >= 6
	case PWM_SERVO_GET_RATEGROUP(4):
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
	case MIXERIOCGETOUTPUTCOUNT:
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
		if (_mixers != nullptr) {
			delete _mixers;
			_mixers = nullptr;
			_groups_required = 0;
		}

		break;

	case MIXERIOCLOADBUF: {
			const char *buf = (const char *)arg;
			unsigned buflen = strnlen(buf, 1024);

			if (_mixers == nullptr) {
				_mixers = new MixerGroup(control_callback, (uintptr_t)_controls);
			}

			if (_mixers == nullptr) {
				_groups_required = 0;
				ret = -ENOMEM;

			} else {

				ret = _mixers->load_from_buf(buf, buflen);

				if (ret != 0) {
					PX4_DEBUG("mixer load failed with %d", ret);
					delete _mixers;
					_mixers = nullptr;
					_groups_required = 0;
					ret = -EINVAL;

				} else {

					_mixers->groups_required(_groups_required);
					PX4_DEBUG("loaded mixers \n%s\n", buf);
					update_pwm_trims();
				}
			}

			break;
		}

	default:
		ret = -ENOTTY;
		break;
	}

	unlock();

	return ret;
}

/*
  this implements PWM output via a write() method, for compatibility
  with px4io
 */
ssize_t
PX4FMU::write(file *filp, const char *buffer, size_t len)
{
	unsigned count = len / 2;
	uint16_t values[_MAX_ACTUATORS];

#if BOARD_HAS_PWM == 0
	return 0;
#endif

	if (count > BOARD_HAS_PWM) {
		// we have at most BOARD_HAS_PWM outputs
		count = BOARD_HAS_PWM;
	}

	if (count > _MAX_ACTUATORS) {
		count = _MAX_ACTUATORS;
	}

	// allow for misaligned values
	memcpy(values, buffer, count * 2);

	for (unsigned i = count; i < _num_outputs; ++i) {
		values[i] = PWM_IGNORE_THIS_CHANNEL;
	}

	reorder_outputs(values);

	for (unsigned i = 0; i < _num_outputs; i++) {
		if (values[i] != PWM_IGNORE_THIS_CHANNEL) {
			up_pwm_servo_set(i, values[i]);
		}
	}

	return count * 2;
}

void
PX4FMU::reorder_outputs(uint16_t values[_MAX_ACTUATORS])
{
	if (_MAX_ACTUATORS < 4) {
		return;
	}

	if (_motor_ordering == MotorOrdering::Betaflight) {
		/*
		 * Betaflight default motor ordering:
		 * 4     2
		 *    ^
		 * 3     1
		 */
		const uint16_t pwm_tmp[4] = {values[0], values[1], values[2], values[3] };
		values[0] = pwm_tmp[3];
		values[1] = pwm_tmp[0];
		values[2] = pwm_tmp[1];
		values[3] = pwm_tmp[2];
	}

	/* else: PX4, no need to reorder
	 * 3     1
	 *    ^
	 * 2     4
	 */
}

void
PX4FMU::sensor_reset(int ms)
{
	if (ms < 1) {
		ms = 1;
	}

	board_spi_reset(ms);
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

		if (direction == 1) {
			// use ioctl interface for one direction
			for (unsigned i = 0; i < servo_count;	i++) {
				if (::ioctl(fd, PWM_SERVO_SET(i), servos[i]) < 0) {
					PX4_ERR("servo %u set failed", i);
					goto err_out;
				}
			}

		} else {
			// and use write interface for the other direction
			ret = ::write(fd, servos, sizeof(servos));

			if (ret != (int)sizeof(servos)) {
				PX4_ERR("error writing PWM servo data, wrote %u got %d", sizeof(servos), ret);
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

PX4FMU *PX4FMU::instantiate(int argc, char *argv[])
{
	// No arguments to parse. We also know that we should run as task
	return new PX4FMU(true);
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
By default the module runs on the work queue, to reduce RAM usage. It can also be run in its own thread,
specified via start flag -t, to reduce latency.
When running on the work queue, it schedules at a fixed frequency, and the pwm rate limits the update rate of
the actuator_controls topics. In case of running in its own thread, the module polls on the actuator_controls topic.
Additionally the pwm rate defines the lower-level IO timer rates.

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
	PRINT_MODULE_USAGE_PARAM_FLAG('t', "Run as separate task instead of the work queue", true);

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

int PX4FMU::print_status()
{
	PX4_INFO("Running %s", (_run_as_task ? "as task" : "on work queue"));

	if (!_run_as_task) {
		PX4_INFO("Max update rate: %i Hz", _current_update_rate);
	}

#ifdef GPIO_BTN_SAFETY
	if (!PX4_MFT_HW_SUPPORTED(PX4_MFT_PX4IO)) {
		PX4_INFO("Safety State (from button): %s", _safety_btn_off ? "off" : "on");
	}
#endif

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

	return 0;
}

extern "C" __EXPORT int fmu_main(int argc, char *argv[]);

int
fmu_main(int argc, char *argv[])
{
	return PX4FMU::main(argc, argv);
}
