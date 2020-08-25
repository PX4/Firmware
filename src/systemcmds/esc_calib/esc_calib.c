/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Julian Oes <joes@student.ethz.ch>
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
 * @file esc_calib.c
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/mount.h>
#include <sys/ioctl.h>
#include <sys/stat.h>



#include "drivers/drv_pwm_output.h"

#include <uORB/topics/actuator_controls.h>

static void	usage(const char *reason);
__EXPORT int	esc_calib_main(int argc, char *argv[]);

static void
usage(const char *reason)
{
	if (reason != NULL) {
		PX4_ERR("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION("Tool for ESC calibration\n"
				 "\n"
				 "Calibration procedure (running the command will guide you through it):\n"
				 "- Remove props, power off the ESC's\n"
				 "- Stop attitude and rate controllers: mc_rate_control stop, fw_att_control stop\n"
				 "- Make sure safety is off\n"
				 "- Run this command\n"
				);

	PRINT_MODULE_USAGE_NAME_SIMPLE("esc_calib", "command");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/pwm_output0", "<file:dev>", "Select PWM output device", true);
	PRINT_MODULE_USAGE_PARAM_INT('l', 1000, 0, 3000, "Low PWM value in us", true);
	PRINT_MODULE_USAGE_PARAM_INT('h', 2000, 0, 3000, "High PWM value in us", true);
	PRINT_MODULE_USAGE_PARAM_STRING('c', NULL, NULL, "select channels in the form: 1234 (1 digit per channel, 1=first)",
					true);
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 4096, "Select channels via bitmask (eg. 0xF, 3)", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('a', "Select all channels", true);
}

int
esc_calib_main(int argc, char *argv[])
{
	const char *dev = PWM_OUTPUT0_DEVICE_PATH;
	char *ep;
	int ch;
	int ret;
	char c;

	unsigned max_channels = 0;

	uint32_t set_mask = 0;
	unsigned long channels;
	unsigned single_ch = 0;

	uint16_t pwm_high = PWM_DEFAULT_MAX;
	uint16_t pwm_low = PWM_DEFAULT_MIN;

	struct pollfd fds;
	fds.fd = 0; /* stdin */
	fds.events = POLLIN;

	if (argc < 2) {
		usage("no channels provided");
		return 1;
	}

	int arg_consumed = 0;

	int myoptind = 1;
	const char *myoptarg = NULL;

	while ((ch = px4_getopt(argc, argv, "d:c:m:al:h:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {

		case 'd':
			dev = myoptarg;
			arg_consumed += 2;
			break;

		case 'c':
			/* Read in channels supplied as one int and convert to mask: 1234 -> 0xF */
			channels = strtoul(myoptarg, &ep, 0);

			while ((single_ch = channels % 10)) {

				set_mask |= 1 << (single_ch - 1);
				channels /= 10;
			}

			break;

		case 'm':
			/* Read in mask directly */
			set_mask = strtoul(myoptarg, &ep, 0);

			if (*ep != '\0') {
				usage("bad set_mask value");
				return 1;
			}

			break;

		case 'a':

			/* Choose all channels */
			for (unsigned i = 0; i < PWM_OUTPUT_MAX_CHANNELS; i++) {
				set_mask |= 1 << i;
			}

			break;

		case 'l':
			/* Read in custom low value */
			pwm_low = strtoul(myoptarg, &ep, 0);

			if (*ep != '\0'
#if PWM_LOWEST_MIN > 0
			    || pwm_low < PWM_LOWEST_MIN
#endif
			    || pwm_low > PWM_HIGHEST_MIN) {
				usage("low PWM invalid");
				return 1;
			}

			break;

		case 'h':
			/* Read in custom high value */
			pwm_high = strtoul(myoptarg, &ep, 0);

			if (*ep != '\0' || pwm_high > PWM_HIGHEST_MAX || pwm_high < PWM_LOWEST_MAX) {
				usage("high PWM invalid");
				return 1;
			}

			break;

		default:
			usage(NULL);
			return 1;
		}
	}

	if (set_mask == 0) {
		usage("no channels chosen");
		return 1;
	}

	if (pwm_low > pwm_high) {
		usage("low pwm is higher than high pwm");
		return 1;
	}

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

	printf("\nATTENTION, please remove or fix propellers before starting calibration!\n"
	       "\n"
	       "Make sure\n"
	       "\t - that the ESCs are not powered\n"
	       "\t - that safety is off (two short blinks)\n"
	       "\t - that the controllers are stopped\n"
	       "\n"
	       "Do you want to start calibration now: y or n?\n");

	/* wait for user input */
	while (1) {

		ret = poll(&fds, 1, 0);

		if (ret > 0) {
			if (read(0, &c, 1) <= 0) {
				printf("ESC calibration read error\n");
				return 0;
			}

			if (c == 'y' || c == 'Y') {
				break;

			} else if (c == 0x03 || c == 0x63 || c == 'q') {
				printf("ESC calibration exited\n");
				return 0;

			} else if (c == 'n' || c == 'N') {
				printf("ESC calibration aborted\n");
				return 0;

			} else {
				printf("Unknown input, ESC calibration aborted\n");
				return 0;
			}
		}

		/* rate limit to ~ 20 Hz */
		px4_usleep(50000);
	}

	/* open for ioctl only */
	int fd = open(dev, 0);

	if (fd < 0) {
		PX4_ERR("can't open %s", dev);
		return 1;
	}

	/* get number of channels available on the device */
	ret = ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&max_channels);

	if (ret != OK) {
		PX4_ERR("PWM_SERVO_GET_COUNT");
		goto cleanup;
	}

	/* tell IO/FMU that its ok to disable its safety with the switch */
	ret = ioctl(fd, PWM_SERVO_SET_ARM_OK, 0);

	if (ret != OK) {
		PX4_ERR("PWM_SERVO_SET_ARM_OK");
		goto cleanup;
	}

	/* tell IO/FMU that the system is armed (it will output values if safety is off) */
	ret = ioctl(fd, PWM_SERVO_ARM, 0);

	if (ret != OK) {
		PX4_ERR("PWM_SERVO_ARM");
		goto cleanup;
	}

	printf("Outputs armed");


	/* wait for user confirmation */
	printf("\nHigh PWM set: %d\n"
	       "\n"
	       "Connect battery now and hit ENTER after the ESCs confirm the first calibration step\n"
	       "\n", pwm_high);
	fflush(stdout);

	while (1) {
		/* set max PWM */
		for (unsigned i = 0; i < max_channels; i++) {

			if (set_mask & 1 << i) {
				ret = ioctl(fd, PWM_SERVO_SET(i), pwm_high);

				if (ret != OK) {
					PX4_ERR("PWM_SERVO_SET(%d), value: %d", i, pwm_high);
					goto cleanup;
				}
			}
		}

		ret = poll(&fds, 1, 0);

		if (ret > 0) {
			if (read(0, &c, 1) <= 0) {
				printf("ESC calibration read error\n");
				goto done;
			}

			if (c == 13) {
				break;

			} else if (c == 0x03 || c == 0x63 || c == 'q') {
				printf("ESC calibration exited");
				goto done;
			}
		}

		/* rate limit to ~ 20 Hz */
		px4_usleep(50000);
	}

	printf("Low PWM set: %d\n"
	       "\n"
	       "Hit ENTER when finished\n"
	       "\n", pwm_low);

	while (1) {

		/* set disarmed PWM */
		for (unsigned i = 0; i < max_channels; i++) {
			if (set_mask & 1 << i) {
				ret = ioctl(fd, PWM_SERVO_SET(i), pwm_low);

				if (ret != OK) {
					PX4_ERR("PWM_SERVO_SET(%d), value: %d", i, pwm_low);
					goto cleanup;
				}
			}
		}

		ret = poll(&fds, 1, 0);

		if (ret > 0) {
			if (read(0, &c, 1) <= 0) {
				printf("ESC calibration read error\n");
				goto done;
			}

			if (c == 13) {
				break;

			} else if (c == 0x03 || c == 0x63 || c == 'q') {
				printf("ESC calibration exited\n");
				goto done;
			}
		}

		/* rate limit to ~ 20 Hz */
		px4_usleep(50000);
	}

	/* disarm */
	ret = ioctl(fd, PWM_SERVO_DISARM, 0);

	if (ret != OK) {
		PX4_ERR("PWM_SERVO_DISARM");
		goto cleanup;
	}

	printf("Outputs disarmed");

	printf("ESC calibration finished\n");

done:
	close(fd);
	return 0;
cleanup:
	close(fd);
	return 1;
}
