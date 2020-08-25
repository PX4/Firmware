/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file motor_ramp.cpp
 *
 * @author Andreas Antener <andreas@uaventure.com>
 * @author Roman Bapst <bapstroman@gmail.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>


#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>

#include "systemlib/err.h"
#include "uORB/topics/actuator_controls.h"

enum RampState {
	RAMP_INIT,
	RAMP_MIN,
	RAMP_RAMP,
	RAMP_WAIT
};

enum Mode {
	RAMP,
	SINE,
	SQUARE
};

static bool _thread_should_exit = false;		/**< motor_ramp exit flag */
static bool _thread_running = false;		/**< motor_ramp status flag */
static int _motor_ramp_task;				/**< Handle of motor_ramp task / thread */
static float _ramp_time;
static int _min_pwm;
static int _max_pwm;
static Mode _mode;
static const char *_mode_c;
static const char *_pwm_output_dev = "/dev/pwm_output0";

/**
 * motor_ramp management function.
 */
extern "C" __EXPORT int motor_ramp_main(int argc, char *argv[]);

/**
 * Mainloop of motor_ramp.
 */
int motor_ramp_thread_main(int argc, char *argv[]);

bool min_pwm_valid(int pwm_value);

bool max_pwm_valid(int pwm_value);

int set_min_pwm(int fd, unsigned long max_channels, int pwm_value);

int set_out(int fd, unsigned long max_channels, float output);

int prepare(int fd, unsigned long *max_channels);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Application to test motor ramp up.

Before starting, make sure to stop any running attitude controller:
$ mc_rate_control stop
$ fw_att_control stop

When starting, a background task is started, runs for several seconds (as specified), then exits.

### Example
$ motor_ramp sine -a 1100 -r 0.5
)DESCR_STR");


	PRINT_MODULE_USAGE_NAME_SIMPLE("motor_ramp", "command");
	PRINT_MODULE_USAGE_ARG("ramp|sine|square", "mode", false);
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/pwm_output0", nullptr, "Pwm output device", true);
	PRINT_MODULE_USAGE_PARAM_INT('a', 0, 900, 1500, "Select minimum pwm duty cycle in usec", false);
	PRINT_MODULE_USAGE_PARAM_INT('b', 2000, 901, 2100, "Select maximum pwm duty cycle in usec", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('r', 1.0f, 0.0f, 65536.0f, "Select motor ramp duration in sec", true);

	PRINT_MODULE_USAGE_PARAM_COMMENT("WARNING: motors will ramp up to full speed!");

}

/**
 * The motor_ramp app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int motor_ramp_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	bool error_flag = false;
	bool set_pwm_min = false;
	_max_pwm = 2000;
	_ramp_time = 1.0f;

	if (_thread_running) {
		PX4_WARN("motor_ramp already running\n");
		/* this is not an error */
		return 0;
	}

	if (argc < 4) {
		usage("missing parameters");
		return 1;
	}

	while ((ch = px4_getopt(argc, argv, "d:a:b:r:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {

			case 'd':
				if(!strcmp(myoptarg, "/dev/pwm_output0") || !strcmp(myoptarg, "/dev/pwm_output1")){
					_pwm_output_dev = myoptarg;
				} else {
					usage("pwm output device not found");
					error_flag = true;
				}
				break;

			case 'a':
				_min_pwm = atoi(myoptarg);

				if (!min_pwm_valid(_min_pwm)) {
					usage("min PWM not in range");
					error_flag = true;
				} else {
					set_pwm_min = true;
				}

				break;

			case 'b':
				_max_pwm = atoi(myoptarg);

				if (!max_pwm_valid(_max_pwm)) {
					usage("max PWM not in range");
					error_flag = true;
				}

				break;

			case 'r':
				_ramp_time = atof(myoptarg);

				if (_ramp_time <= 0) {
					usage("ramp time must be greater than 0");
					error_flag = true;
				}

				break;

			default:
				PX4_WARN("unrecognized flag");
				error_flag = true;
				break;
		}
	}

	_thread_should_exit = false;

	if(!set_pwm_min){
		PX4_WARN("pwm_min not set. use -a flag");
		error_flag = true;
	}


	if (!strcmp(argv[myoptind], "ramp")) {
		_mode = RAMP;

	} else if (!strcmp(argv[myoptind], "sine")) {
		_mode = SINE;

	} else if (!strcmp(argv[myoptind], "square")) {
		_mode = SQUARE;

	} else {
		usage("selected mode not valid");
		error_flag = true;
	}

	_mode_c = myoptarg;

	if(error_flag){
		return 1;
	}

	_motor_ramp_task = px4_task_spawn_cmd("motor_ramp",
					      SCHED_DEFAULT,
					      SCHED_PRIORITY_DEFAULT + 40,
					      2000,
					      motor_ramp_thread_main,
					      (argv) ? (char *const *)&argv[2] : (char *const *)nullptr);

	return 0;
}

bool min_pwm_valid(int pwm_value)
{
	return pwm_value >= 900 && pwm_value <= 1500;
}

bool max_pwm_valid(int pwm_value)
{
	return pwm_value <= 2100 && pwm_value > _min_pwm;
}

int set_min_pwm(int fd, unsigned long max_channels, int pwm_value)
{
	int ret;

	struct pwm_output_values pwm_values {};

	pwm_values.channel_count = max_channels;

	for (unsigned i = 0; i < max_channels; i++) {
		pwm_values.values[i] = pwm_value;
	}

	ret = px4_ioctl(fd, PWM_SERVO_SET_MIN_PWM, (long unsigned int)&pwm_values);

	if (ret != OK) {
		PX4_ERR("failed setting min values");
		return 1;
	}

	return 0;
}

int set_out(int fd, unsigned long max_channels, float output)
{
	int ret;
	int pwm = (_max_pwm - _min_pwm) * output + _min_pwm;

	for (unsigned i = 0; i < max_channels; i++) {

		ret = ioctl(fd, PWM_SERVO_SET(i), pwm);

		if (ret != OK) {
			PX4_ERR("PWM_SERVO_SET(%d), value: %d", i, pwm);
			return 1;
		}
	}

	return 0;
}

int prepare(int fd, unsigned long *max_channels)
{
	/* make sure no other source is publishing control values now */
	struct actuator_controls_s actuators;
	int act_sub = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);

	/* clear changed flag */
	orb_copy(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, act_sub, &actuators);

	/* wait 50 ms */
	px4_usleep(50000);

	/* now expect nothing changed on that topic */
	bool orb_updated;
	orb_check(act_sub, &orb_updated);

	if (orb_updated) {
		PX4_ERR("ABORTING! Attitude control still active. Please ensure to shut down all controllers:\n"
			"\tmc_rate_control stop\n"
			"\tfw_att_control stop\n");
		return 1;
	}

	/* get number of channels available on the device */
	if (px4_ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)max_channels) != OK) {
		PX4_ERR("PWM_SERVO_GET_COUNT");
		return 1;
	}

	/* tell IO/FMU that its ok to disable its safety with the switch */
	if (px4_ioctl(fd, PWM_SERVO_SET_ARM_OK, 0) != OK) {
		PX4_ERR("PWM_SERVO_SET_ARM_OK");
		return 1;
	}

	/* tell IO/FMU that the system is armed (it will output values if safety is off) */
	if (px4_ioctl(fd, PWM_SERVO_ARM, 0) != OK) {
		PX4_ERR("PWM_SERVO_ARM");
		return 1;
	}

	/* tell IO to switch off safety without using the safety switch */
	if (px4_ioctl(fd, PWM_SERVO_SET_FORCE_SAFETY_OFF, 0) != OK) {
		PX4_ERR("PWM_SERVO_SET_FORCE_SAFETY_OFF");
		return 1;
	}

	return 0;
}

int motor_ramp_thread_main(int argc, char *argv[])
{
	_thread_running = true;

	unsigned long max_channels = 0;
	static struct pwm_output_values last_spos;
	static struct pwm_output_values last_min;
	static unsigned servo_count;

	int fd = px4_open(_pwm_output_dev, 0);

	if (fd < 0) {
		PX4_ERR("can't open %s", _pwm_output_dev);
		_thread_running = false;
		return 1;
	}

	/* get the number of servo channels */
	if (px4_ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count) < 0) {
			PX4_ERR("PWM_SERVO_GET_COUNT");
			px4_close(fd);
			_thread_running = false;
			return 1;

	}

	/* get current servo values */
	for (unsigned i = 0; i < servo_count; i++) {

		if (px4_ioctl(fd, PWM_SERVO_GET(i), (unsigned long)&last_spos.values[i]) < 0) {
			PX4_ERR("PWM_SERVO_GET(%d)", i);
			px4_close(fd);
			_thread_running = false;
			return 1;
		}
	}

	/* get current pwm min */
	if (px4_ioctl(fd, PWM_SERVO_GET_MIN_PWM, (long unsigned int)&last_min) < 0) {
		PX4_ERR("failed getting pwm min values");
		px4_close(fd);
		_thread_running = false;
		return 1;
	}

	if (px4_ioctl(fd, PWM_SERVO_SET_MODE, PWM_SERVO_ENTER_TEST_MODE) < 0) {
		PX4_ERR("Failed to Enter pwm test mode");
		px4_close(fd);
		_thread_running = false;
		return 1;
	}

	if (prepare(fd, &max_channels) != OK) {
		_thread_should_exit = true;
	}

	set_out(fd, max_channels, 0.0f);

	float dt = 0.001f; // prevent division with 0
	float timer = 0.0f;
	hrt_abstime start = 0;
	hrt_abstime last_run = 0;

	enum RampState ramp_state = RAMP_INIT;
	float output = 0.0f;

	while (!_thread_should_exit) {

		if (last_run > 0) {
			dt = hrt_elapsed_time(&last_run) * 1e-6;

		} else {
			start = hrt_absolute_time();
		}

		last_run = hrt_absolute_time();
		timer = hrt_elapsed_time(&start) * 1e-6;

		switch (ramp_state) {
		case RAMP_INIT: {
				PX4_INFO("setting pwm min: %d", _min_pwm);
				set_min_pwm(fd, max_channels, _min_pwm);
				ramp_state = RAMP_MIN;
				break;
			}

		case RAMP_MIN: {
				if (timer > 3.0f) {
					PX4_INFO("starting %s: %.2f sec", _mode_c, (double)_ramp_time);
					start = hrt_absolute_time();
					ramp_state = RAMP_RAMP;
				}

				set_out(fd, max_channels, output);
				break;
			}

		case RAMP_RAMP: {
				if (_mode == RAMP) {
					output += 1000.0f * dt / (_max_pwm - _min_pwm) / _ramp_time;

				} else if (_mode == SINE) {
					// sine outpout with period T = _ramp_time and magnitude between [0,1]
					// phase shift makes sure that it starts at zero when timer is zero
					output = 0.5f * (1.0f + sinf(M_TWOPI_F * timer / _ramp_time - M_PI_2_F));

				} else if (_mode == SQUARE) {
					output = fmodf(timer, _ramp_time) > (_ramp_time * 0.5f) ? 1.0f : 0.0f;
				}

				if ((output > 1.0f && _mode == RAMP) || (timer > 3.0f * _ramp_time)) {
					// for ramp mode we set output to 1, for others we just leave it as is
					output = _mode != RAMP ? output : 1.0f;
					start = hrt_absolute_time();
					ramp_state = RAMP_WAIT;
					PX4_INFO("%s finished, waiting", _mode_c);
				}

				set_out(fd, max_channels, output);
				break;
			}

		case RAMP_WAIT: {
				if (timer > 0.5f) {
					_thread_should_exit = true;
					PX4_INFO("stopping");
					break;
				}

				set_out(fd, max_channels, output);
				break;
			}
		}

		// rate limit
		px4_usleep(2000);
	}

	if (fd >= 0) {
		/* set current pwm min */
		if (px4_ioctl(fd, PWM_SERVO_SET_MIN_PWM, (long unsigned int)&last_min) < 0) {
			PX4_ERR("failed setting pwm min values");
			px4_close(fd);
			_thread_running = false;
			return 1;
		}

		/* set previous servo values */
		for (unsigned i = 0; i < servo_count; i++) {

			if (px4_ioctl(fd, PWM_SERVO_SET(i), (unsigned long)last_spos.values[i]) < 0) {
				PX4_ERR("PWM_SERVO_SET(%d)", i);
				px4_close(fd);
				_thread_running = false;
				return 1;
			}
		}

		if (px4_ioctl(fd, PWM_SERVO_SET_MODE, PWM_SERVO_EXIT_TEST_MODE) < 0) {
			PX4_ERR("Failed to Exit pwm test mode");
			px4_close(fd);
			_thread_running = false;
			return 1;
		}

		px4_close(fd);
	}

	_thread_running = false;

	return 0;
}
