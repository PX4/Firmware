/****************************************************************************
 *
 *   Copyright (c) 2012-2015, 2017 PX4 Development Team. All rights reserved.
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
 * @file PWM servo output interface.
 *
 * Servo values can be set with the PWM_SERVO_SET ioctl, by writing a
 * pwm_output_values structure to the device
 * Writing a value of 0 to a channel suppresses any output for that
 * channel.
 */

#pragma once

#include <px4_platform_common/defines.h>

#include <stdint.h>
#include <sys/ioctl.h>
#include <board_config.h>

#include "drv_orb_dev.h"

__BEGIN_DECLS

/**
 * Path for the default PWM output device.
 *
 * Note that on systems with more than one PWM output path (e.g.
 * PX4FMU with PX4IO connected) there may be other devices that
 * respond to this protocol.
 */
#define PWM_OUTPUT_BASE_DEVICE_PATH "/dev/pwm_output"
#define PWM_OUTPUT0_DEVICE_PATH	"/dev/pwm_output0"
#define PWM_OUTPUT1_DEVICE_PATH	"/dev/pwm_output1"

#define PWM_OUTPUT_MAX_CHANNELS 16

struct pwm_output_values {
	uint32_t channel_count;
	uint16_t values[PWM_OUTPUT_MAX_CHANNELS];
};

/* Use defaults unless the board override the defaults by providing
 * PX4_PWM_ALTERNATE_RANGES and a replacement set of
 * constants
 */
#if !defined(PX4_PWM_ALTERNATE_RANGES)

/**
 * Lowest minimum PWM in us
 */
#define PWM_LOWEST_MIN 90

/**
 * Default value for a shutdown motor
 */
#define PWM_MOTOR_OFF	900

/**
 * Default minimum PWM in us
 */
#define PWM_DEFAULT_MIN 1000

/**
 * Highest PWM allowed as the minimum PWM
 */
#define PWM_HIGHEST_MIN 1600

/**
 * Highest maximum PWM in us
 */
#define PWM_HIGHEST_MAX 2150

/**
 * Default maximum PWM in us
 */
#define PWM_DEFAULT_MAX 2000

/**
 * Default trim PWM in us
 */
#define PWM_DEFAULT_TRIM 0

/**
 * Lowest PWM allowed as the maximum PWM
 */
#define PWM_LOWEST_MAX 200

#endif // not PX4_PWM_ALTERNATE_RANGES

/**
 * Do not output a channel with this value
 */
#define PWM_IGNORE_THIS_CHANNEL UINT16_MAX

/**
 * Servo output signal type, value is actual servo output pulse
 * width in microseconds.
 */
typedef uint16_t	servo_position_t;

/**
 * RC config values for a channel
 *
 * This allows for PX4IO_PAGE_RC_CONFIG values to be set without a
 * param_get() dependency
 */
struct pwm_output_rc_config {
	uint8_t channel;
	uint16_t rc_min;
	uint16_t rc_trim;
	uint16_t rc_max;
	uint16_t rc_dz;
	uint16_t rc_assignment;
	bool     rc_reverse;
};

/*
 * ioctl() definitions
 *
 * Note that ioctls and ORB updates should not be mixed, as the
 * behaviour of the system in this case is not defined.
 */
#define _PWM_SERVO_BASE		0x2a00

/** arm all servo outputs handle by this driver */
#define PWM_SERVO_ARM		_PX4_IOC(_PWM_SERVO_BASE, 0)

/** disarm all servo outputs (stop generating pulses) */
#define PWM_SERVO_DISARM	_PX4_IOC(_PWM_SERVO_BASE, 1)

/** get default servo update rate */
#define PWM_SERVO_GET_DEFAULT_UPDATE_RATE _PX4_IOC(_PWM_SERVO_BASE, 2)

/** set alternate servo update rate */
#define PWM_SERVO_SET_UPDATE_RATE _PX4_IOC(_PWM_SERVO_BASE, 3)

/** get alternate servo update rate */
#define PWM_SERVO_GET_UPDATE_RATE _PX4_IOC(_PWM_SERVO_BASE, 4)

/** get the number of servos in *(unsigned *)arg */
#define PWM_SERVO_GET_COUNT	_PX4_IOC(_PWM_SERVO_BASE, 5)

/** selects servo update rates, one bit per servo. 0 = default (50Hz), 1 = alternate */
#define PWM_SERVO_SET_SELECT_UPDATE_RATE _PX4_IOC(_PWM_SERVO_BASE, 6)

/** check the selected update rates */
#define PWM_SERVO_GET_SELECT_UPDATE_RATE _PX4_IOC(_PWM_SERVO_BASE, 7)

/** set the 'ARM ok' bit, which activates the safety switch */
#define PWM_SERVO_SET_ARM_OK	_PX4_IOC(_PWM_SERVO_BASE, 8)

/** clear the 'ARM ok' bit, which deactivates the safety switch */
#define PWM_SERVO_CLEAR_ARM_OK	_PX4_IOC(_PWM_SERVO_BASE, 9)

/** start DSM bind */
#define DSM_BIND_START	_PX4_IOC(_PWM_SERVO_BASE, 10)

/** power up DSM receiver */
#define DSM_BIND_POWER_UP _PX4_IOC(_PWM_SERVO_BASE, 11)

/** set the PWM value for failsafe */
#define PWM_SERVO_SET_FAILSAFE_PWM	_PX4_IOC(_PWM_SERVO_BASE, 12)

/** get the PWM value for failsafe */
#define PWM_SERVO_GET_FAILSAFE_PWM	_PX4_IOC(_PWM_SERVO_BASE, 13)

/** set the PWM value when disarmed - should be no PWM (zero) by default */
#define PWM_SERVO_SET_DISARMED_PWM	_PX4_IOC(_PWM_SERVO_BASE, 14)

/** get the PWM value when disarmed */
#define PWM_SERVO_GET_DISARMED_PWM	_PX4_IOC(_PWM_SERVO_BASE, 15)

/** set the minimum PWM value the output will send */
#define PWM_SERVO_SET_MIN_PWM	_PX4_IOC(_PWM_SERVO_BASE, 16)

/** get the minimum PWM value the output will send */
#define PWM_SERVO_GET_MIN_PWM	_PX4_IOC(_PWM_SERVO_BASE, 17)

/** set the maximum PWM value the output will send */
#define PWM_SERVO_SET_MAX_PWM	_PX4_IOC(_PWM_SERVO_BASE, 18)

/** get the maximum PWM value the output will send */
#define PWM_SERVO_GET_MAX_PWM	_PX4_IOC(_PWM_SERVO_BASE, 19)

/** set the TRIM value the output will send */
#define PWM_SERVO_SET_TRIM_PWM	_PX4_IOC(_PWM_SERVO_BASE, 20)

/** get the TRIM value the output will send */
#define PWM_SERVO_GET_TRIM_PWM	_PX4_IOC(_PWM_SERVO_BASE, 21)

/** set the number of servos in (unsigned)arg - allows change of
 * split between servos and GPIO */
#define PWM_SERVO_SET_COUNT	_PX4_IOC(_PWM_SERVO_BASE, 22)

/** set the lockdown override flag to enable outputs in HIL */
#define PWM_SERVO_SET_DISABLE_LOCKDOWN		_PX4_IOC(_PWM_SERVO_BASE, 23)

/** get the lockdown override flag to enable outputs in HIL */
#define PWM_SERVO_GET_DISABLE_LOCKDOWN		_PX4_IOC(_PWM_SERVO_BASE, 24)

/** force safety switch off (to disable use of safety switch) */
#define PWM_SERVO_SET_FORCE_SAFETY_OFF		_PX4_IOC(_PWM_SERVO_BASE, 25)

/** force failsafe mode (failsafe values are set immediately even if failsafe condition not met) */
#define PWM_SERVO_SET_FORCE_FAILSAFE		_PX4_IOC(_PWM_SERVO_BASE, 26)

/** make failsafe non-recoverable (termination) if it occurs */
#define PWM_SERVO_SET_TERMINATION_FAILSAFE	_PX4_IOC(_PWM_SERVO_BASE, 27)

/** force safety switch on (to enable use of safety switch) */
#define PWM_SERVO_SET_FORCE_SAFETY_ON		_PX4_IOC(_PWM_SERVO_BASE, 28)

/** setup OVERRIDE_IMMEDIATE behaviour on FMU fail */
#define PWM_SERVO_SET_OVERRIDE_IMMEDIATE	_PX4_IOC(_PWM_SERVO_BASE, 32)

/** set SBUS output frame rate in Hz */
#define PWM_SERVO_SET_SBUS_RATE			_PX4_IOC(_PWM_SERVO_BASE, 33)

/** set auxillary output mode. These correspond to enum Mode in px4fmu/fmu.cpp */
#define PWM_SERVO_MODE_NONE         0
#define PWM_SERVO_MODE_1PWM         1
#define PWM_SERVO_MODE_2PWM         2
#define PWM_SERVO_MODE_2PWM2CAP     3
#define PWM_SERVO_MODE_3PWM         4
#define PWM_SERVO_MODE_3PWM1CAP     5
#define PWM_SERVO_MODE_4PWM         6
#define PWM_SERVO_MODE_4PWM1CAP     7
#define PWM_SERVO_MODE_4PWM2CAP     8
#define PWM_SERVO_MODE_5PWM         9
#define PWM_SERVO_MODE_5PWM1CAP    10
#define PWM_SERVO_MODE_6PWM        11
#define PWM_SERVO_MODE_8PWM        12
#define PWM_SERVO_MODE_14PWM       13
#define PWM_SERVO_MODE_4CAP        14
#define PWM_SERVO_MODE_5CAP        15
#define PWM_SERVO_MODE_6CAP        16
#define PWM_SERVO_ENTER_TEST_MODE  17
#define PWM_SERVO_EXIT_TEST_MODE   18
#define PWM_SERVO_SET_MODE         _PX4_IOC(_PWM_SERVO_BASE, 34)

/*
 *
 *
 * WARNING WARNING WARNING! DO NOT EXCEED 47 IN IOC INDICES HERE!
 *
 *
 */

/** set a single servo to a specific value */
#define PWM_SERVO_SET(_servo)	_PX4_IOC(_PWM_SERVO_BASE, 0x30 + _servo)

/** get a single specific servo value */
#define PWM_SERVO_GET(_servo)	_PX4_IOC(_PWM_SERVO_BASE, 0x50 + _servo)

/** get the _n'th rate group's channels; *(uint32_t *)arg returns a bitmap of channels
 *  whose update rates must be the same.
 */
#define PWM_SERVO_GET_RATEGROUP(_n) _PX4_IOC(_PWM_SERVO_BASE, 0x70 + _n)

/** specific rates for configuring the timer for OneShot or PWM */
#define	PWM_RATE_ONESHOT			0u
#define	PWM_RATE_LOWER_LIMIT		1u
#define	PWM_RATE_UPPER_LIMIT		10000u

/** Dshot PWM frequency */
#define DSHOT1200					1200000u	//Hz
#define DSHOT600					600000u		//Hz
#define DSHOT300					300000u		//Hz
#define DSHOT150					150000u		//Hz

#define DSHOT_MAX_THROTTLE			1999

typedef enum {
	DShot_cmd_motor_stop = 0,
	DShot_cmd_beacon1,
	DShot_cmd_beacon2,
	DShot_cmd_beacon3,
	DShot_cmd_beacon4,
	DShot_cmd_beacon5,
	DShot_cmd_esc_info, // V2 includes settings
	DShot_cmd_spin_direction_1,
	DShot_cmd_spin_direction_2,
	DShot_cmd_3d_mode_off,
	DShot_cmd_3d_mode_on,
	DShot_cmd_settings_request, // Currently not implemented
	DShot_cmd_save_settings,
	DShot_cmd_spin_direction_normal = 20,
	DShot_cmd_spin_direction_reversed = 21,
	DShot_cmd_led0_on, // BLHeli32 only
	DShot_cmd_led1_on, // BLHeli32 only
	DShot_cmd_led2_on, // BLHeli32 only
	DShot_cmd_led3_on, // BLHeli32 only
	DShot_cmd_led0_off, // BLHeli32 only
	DShot_cmd_led1_off, // BLHeli32 only
	DShot_cmd_led2_off, // BLHeli32 only
	DShot_cmd_led4_off, // BLHeli32 only
	DShot_cmd_audio_stream_mode_on_off = 30, // KISS audio Stream mode on/off
	DShot_cmd_silent_mode_on_off = 31, // KISS silent Mode on/off
	DShot_cmd_signal_line_telemeetry_disable = 32,
	DShot_cmd_signal_line_continuous_erpm_telemetry = 33,
	DShot_cmd_MAX = 47,
	DShot_cmd_MIN_throttle = 48
				 // >47 are throttle values
} dshot_command_t;


/*
 * Low-level PWM output interface.
 *
 * This is the low-level API to the platform-specific PWM driver.
 */

/**
 * Intialise the PWM servo outputs using the specified configuration.
 *
 * @param channel_mask	Bitmask of channels (LSB = channel 0) to enable.
 *			This allows some of the channels to remain configured
 *			as GPIOs or as another function.
 * @return		OK on success.
 */
__EXPORT extern int	up_pwm_servo_init(uint32_t channel_mask);

/**
 * De-initialise the PWM servo outputs.
 */
__EXPORT extern void	up_pwm_servo_deinit(void);

/**
 * Arm or disarm servo outputs.
 *
 * When disarmed, servos output no pulse.
 *
 * @bug This function should, but does not, guarantee that any pulse
 *      currently in progress is cleanly completed.
 *
 * @param armed		If true, outputs are armed; if false they
 *			are disarmed.
 */
__EXPORT extern void	up_pwm_servo_arm(bool armed);

/**
 * Set the servo update rate for all rate groups.
 *
 * @param rate		The update rate in Hz to set.
 * @return		OK on success, -ERANGE if an unsupported update rate is set.
 */
__EXPORT extern int	up_pwm_servo_set_rate(unsigned rate);

/**
 * Get a bitmap of output channels assigned to a given rate group.
 *
 * @param group		The rate group to query. Rate groups are assigned contiguously
 *			starting from zero.
 * @return		A bitmap of channels assigned to the rate group, or zero if
 *			the group number has no channels.
 */
__EXPORT extern uint32_t up_pwm_servo_get_rate_group(unsigned group);

/**
 * Set the update rate for a given rate group.
 *
 * @param group		The rate group whose update rate will be changed.
 * @param rate		The update rate in Hz.
 * @return		OK if the group was adjusted, -ERANGE if an unsupported update rate is set.
 */
__EXPORT extern int	up_pwm_servo_set_rate_group_update(unsigned group, unsigned rate);

/**
 * Trigger all timer's channels in Oneshot mode to fire
 * the oneshot with updated values.
 * Nothing is done if not in oneshot mode.
 *
 */
__EXPORT extern void up_pwm_update(void);

/**
 * Set the current output value for a channel.
 *
 * @param channel	The channel to set.
 * @param value		The output pulse width in microseconds.
 */
__EXPORT extern int	up_pwm_servo_set(unsigned channel, servo_position_t value);

/**
 * Get the current output value for a channel.
 *
 * @param channel	The channel to read.
 * @return		The output pulse width in microseconds, or zero if
 *			outputs are not armed or not configured.
 */
__EXPORT extern servo_position_t up_pwm_servo_get(unsigned channel);

/**
 * Intialise the Dshot outputs using the specified configuration.
 *
 * @param	channel_mask	Bitmask of channels (LSB = channel 0) to enable.
 *			This allows some of the channels to remain configured
 *			as GPIOs or as another function.
 * @param	dshot_pwm_freq is frequency of DSHOT signal. Usually DSHOT1200, DSHOT600, DSHOT300 or DSHOT150
 * @return	OK on success.
 */
__EXPORT extern int up_dshot_init(uint32_t channel_mask, unsigned dshot_pwm_freq);

/**
 * Set the current dshot throttle value for a channel (motor).
 *
 * @param channel	The channel to set.
 * @param throttle	The output dshot throttle value in [0, 1999 = DSHOT_MAX_THROTTLE].
 * @param telemetry If true, request telemetry from that motor
 */
__EXPORT extern void up_dshot_motor_data_set(unsigned channel, uint16_t throttle, bool telemetry);

/**
 * Send DShot command to a channel (motor).
 *
 * @param channel	The channel to set.
 * @param command	dshot_command_t
 * @param telemetry If true, request telemetry from that motor
 */
__EXPORT extern void up_dshot_motor_command(unsigned channel, uint16_t command, bool telemetry);

/**
 * Trigger dshot data transfer.
 */
__EXPORT extern void up_dshot_trigger(void);

/**
 * Arm or disarm dshot outputs (This will enable/disable complete timer for safety purpose.).
 *
 * When disarmed, dshot output no pulse.
 *
 * @param armed		If true, outputs are armed; if false they
 *			are disarmed.
 */
__EXPORT extern int up_dshot_arm(bool armed);

__END_DECLS
