/****************************************************************************
 *
 *  Copyright (C) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file test_ppm_loopback.c
 * Tests the PWM outputs and PPM input
 */

#include <px4_platform_common/time.h>
#include <px4_platform_common/px4_config.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include <drivers/drv_pwm_output.h>
#include <drivers/drv_rc_input.h>
#include <uORB/topics/rc_channels.h>
#include <systemlib/err.h>

#include "tests_main.h"

#include <math.h>
#include <float.h>

int test_ppm_loopback(int argc, char *argv[])
{

	int _rc_sub = orb_subscribe(ORB_ID(input_rc));

	int servo_fd, result;
	servo_position_t pos;

	servo_fd = open(PWM_OUTPUT0_DEVICE_PATH, O_RDWR);

	if (servo_fd < 0) {
		printf("failed opening /dev/pwm_servo\n");
	}

	printf("Servo readback, pairs of values should match defaults\n");

	unsigned servo_count;
	result = ioctl(servo_fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count);

	if (result != OK) {
		warnx("PWM_SERVO_GET_COUNT");

		(void)close(servo_fd);
		return ERROR;
	}

	for (unsigned i = 0; i < servo_count; i++) {
		result = ioctl(servo_fd, PWM_SERVO_GET(i), (unsigned long)&pos);

		if (result < 0) {
			printf("failed reading channel %u\n", i);
		}

		//printf("%u: %u %u\n", i, pos, data[i]);

	}

	// /* tell safety that its ok to disable it with the switch */
	// result = ioctl(servo_fd, PWM_SERVO_SET_ARM_OK, 0);
	// if (result != OK)
	// 	warnx("FAIL: PWM_SERVO_SET_ARM_OK");
	//  tell output device that the system is armed (it will output values if safety is off)
	// result = ioctl(servo_fd, PWM_SERVO_ARM, 0);
	// if (result != OK)
	// 	warnx("FAIL: PWM_SERVO_ARM");

	int pwm_values[] = {1200, 1300, 1900, 1700, 1500, 1250, 1800, 1400};


	for (unsigned i = 0; (i < servo_count) && (i < sizeof(pwm_values) / sizeof(pwm_values[0])); i++) {
		result = ioctl(servo_fd, PWM_SERVO_SET(i), pwm_values[i]);

		if (result) {
			(void)close(servo_fd);
			return ERROR;

		} else {
			warnx("channel %d set to %d", i, pwm_values[i]);
		}
	}

	warnx("servo count: %d", servo_count);

	struct pwm_output_values pwm_out = {.values = {0}, .channel_count = 0};

	for (unsigned i = 0; (i < servo_count) && (i < sizeof(pwm_values) / sizeof(pwm_values[0])); i++) {
		pwm_out.values[i] = pwm_values[i];
		//warnx("channel %d: disarmed PWM: %d", i+1, pwm_values[i]);
		pwm_out.channel_count++;
	}

	result = ioctl(servo_fd, PWM_SERVO_SET_DISARMED_PWM, (long unsigned int)&pwm_out);

	/* give driver 10 ms to propagate */

	/* read low-level values from FMU or IO RC inputs (PPM, Spektrum, S.Bus) */
	struct input_rc_s rc_input;
	orb_copy(ORB_ID(input_rc), _rc_sub, &rc_input);
	px4_usleep(100000);

	/* open PPM input and expect values close to the output values */

	bool rc_updated;
	orb_check(_rc_sub, &rc_updated);

	if (rc_updated) {

		orb_copy(ORB_ID(input_rc), _rc_sub, &rc_input);

		// int ppm_fd = open(RC_INPUT_DEVICE_PATH, O_RDONLY);



		// struct input_rc_s rc;
		// result = read(ppm_fd, &rc, sizeof(rc));

		// if (result != sizeof(rc)) {
		// 	warnx("Error reading RC output");
		// 	(void)close(servo_fd);
		// 	(void)close(ppm_fd);
		// 	return ERROR;
		// }

		/* go and check values */
		for (unsigned i = 0; (i < servo_count) && (i < sizeof(pwm_values) / sizeof(pwm_values[0])); i++) {
			if (abs(rc_input.values[i] - pwm_values[i]) > 10) {
				warnx("comparison fail: RC: %d, expected: %d", rc_input.values[i], pwm_values[i]);
				(void)close(servo_fd);
				return ERROR;
			}
		}

	} else {
		warnx("failed reading RC input data");
		(void)close(servo_fd);
		return ERROR;
	}

	close(servo_fd);

	warnx("PPM LOOPBACK TEST PASSED SUCCESSFULLY!");

	return 0;
}
