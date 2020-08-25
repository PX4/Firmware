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
 * @file test_servo.c
 * Tests the servo outputs
 */

#include <px4_platform_common/time.h>
#include <px4_platform_common/px4_config.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>

#include <drivers/drv_pwm_output.h>
#include <systemlib/err.h>

#include "tests_main.h"

int test_servo(int argc, char *argv[])
{
	int fd;
	int result = 0;
	servo_position_t data[PWM_OUTPUT_MAX_CHANNELS];
	servo_position_t pos;

	fd = open(PWM_OUTPUT0_DEVICE_PATH, O_RDWR);

	if (fd < 0) {
		printf("failed opening /dev/pwm_servo\n");
		goto out;
	}

	result = read(fd, &data, sizeof(data));

	if (result != sizeof(data)) {
		printf("failed bulk-reading channel values\n");
		goto out;
	}

	printf("Servo readback, pairs of values should match defaults\n");

	unsigned servo_count;
	result = ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count);

	if (result != OK) {
		warnx("PWM_SERVO_GET_COUNT");
		goto out;
	}

	for (unsigned i = 0; i < servo_count; i++) {
		result = ioctl(fd, PWM_SERVO_GET(i), (unsigned long)&pos);

		if (result < 0) {
			printf("failed reading channel %u\n", i);
			goto out;
		}

		printf("%u: %u %u\n", i, pos, data[i]);

	}

	/* tell safety that its ok to disable it with the switch */
	result = ioctl(fd, PWM_SERVO_SET_ARM_OK, 0);

	if (result != OK) {
		warnx("FAIL: PWM_SERVO_SET_ARM_OK");
		goto out;
	}

	/* tell output device that the system is armed (it will output values if safety is off) */
	result = ioctl(fd, PWM_SERVO_ARM, 0);

	if (result != OK) {
		warnx("FAIL: PWM_SERVO_ARM");
		goto out;
	}

	px4_usleep(5000000);
	printf("Advancing channel 0 to 1500\n");
	result = ioctl(fd, PWM_SERVO_SET(0), 1500);
	printf("Advancing channel 1 to 1800\n");
	result = ioctl(fd, PWM_SERVO_SET(1), 1800);
out:

	if (fd >= 0) {
		close(fd);
	}

	return result;
}
