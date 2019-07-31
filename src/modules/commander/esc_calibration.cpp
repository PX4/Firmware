/****************************************************************************
 *
 *   Copyright (c) 2015-2017 PX4 Development Team. All rights reserved.
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
 * @file esc_calibration.cpp
 *
 * Definition of esc calibration
 *
 * @author Roman Bapst <roman@px4.io>
 */

#include "esc_calibration.h"
#include "calibration_messages.h"
#include "calibration_routines.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <systemlib/err.h>
#include <fcntl.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_time.h>
#include "drivers/drv_pwm_output.h"
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/uORB.h>
#include <drivers/drv_hrt.h>
#include <systemlib/mavlink_log.h>

using namespace time_literals;

int do_esc_calibration(orb_advert_t *mavlink_log_pub, struct actuator_armed_s *armed)
{
	int	return_code = PX4_OK;

#if defined(__PX4_POSIX_OCPOC) || defined(__PX4_POSIX_BBBLUE)
	hrt_abstime timeout_start = 0;
	hrt_abstime timeout_wait = 60_s;
	armed->in_esc_calibration_mode = true;
	calibration_log_info(mavlink_log_pub, CAL_QGC_DONE_MSG, "begin esc");
	timeout_start = hrt_absolute_time();

	while (true) {
		if (hrt_absolute_time() - timeout_start > timeout_wait) {
			break;

		} else {
			px4_usleep(50000);
		}
	}

	armed->in_esc_calibration_mode = false;
	calibration_log_info(mavlink_log_pub, CAL_QGC_DONE_MSG, "end esc");

	if (return_code == OK) {
		calibration_log_info(mavlink_log_pub, CAL_QGC_DONE_MSG, "esc");
	}

	return return_code;

#else
	int	fd = -1;

	struct	battery_status_s battery = {};
	int	batt_sub = -1;
	bool	batt_updated = false;
	bool	batt_connected = true;	// for safety resons assume battery is connected, will be cleared below if not the case

	hrt_abstime battery_connect_wait_timeout = 20_s;
	hrt_abstime pwm_high_timeout = 5_s;
	hrt_abstime pwm_low_timeout = 5_s;
	hrt_abstime timeout_start = 0;

	calibration_log_info(mavlink_log_pub, CAL_QGC_STARTED_MSG, "esc");

	batt_sub = orb_subscribe(ORB_ID(battery_status));

	if (batt_sub < 0) {
		calibration_log_critical(mavlink_log_pub, CAL_QGC_FAILED_MSG, "Subscribe to battery");
		goto Error;
	}

	// Make sure battery is disconnected
	if (orb_copy(ORB_ID(battery_status), batt_sub, &battery) == PX4_OK) {

		// battery is not connected if the connected flag is not set and we have a recent battery measurement
		if (!battery.connected && (hrt_absolute_time() - battery.timestamp < 500_ms)) {
			batt_connected = false;
		}
	}

	if (batt_connected) {
		calibration_log_critical(mavlink_log_pub, CAL_QGC_FAILED_MSG, "Disconnect battery and try again");
		goto Error;
	}

	armed->in_esc_calibration_mode = true;

	fd = px4_open(PWM_OUTPUT0_DEVICE_PATH, 0);

	if (fd < 0) {
		calibration_log_critical(mavlink_log_pub, CAL_QGC_FAILED_MSG, "Can't open PWM device");
		goto Error;
	}

	/* tell IO/FMU that its ok to disable its safety with the switch */
	if (px4_ioctl(fd, PWM_SERVO_SET_ARM_OK, 0) != PX4_OK) {
		calibration_log_critical(mavlink_log_pub, CAL_QGC_FAILED_MSG, "Unable to disable safety switch");
		goto Error;
	}

	/* tell IO/FMU that the system is armed (it will output values if safety is off) */
	if (px4_ioctl(fd, PWM_SERVO_ARM, 0) != PX4_OK) {
		calibration_log_critical(mavlink_log_pub, CAL_QGC_FAILED_MSG, "Unable to arm system");
		goto Error;
	}

	/* tell IO to switch off safety without using the safety switch */
	if (px4_ioctl(fd, PWM_SERVO_SET_FORCE_SAFETY_OFF, 0) != PX4_OK) {
		calibration_log_critical(mavlink_log_pub, CAL_QGC_FAILED_MSG, "Unable to force safety off");
		goto Error;
	}

	/* tell IO to switch to the calibration high pwm phase */
	if (px4_ioctl(fd, PWM_SERVO_SET_CAL_HIGH, 0) != PX4_OK) {
		calibration_log_critical(mavlink_log_pub, CAL_QGC_FAILED_MSG, "Unable to switch to to calibration high pwm phase");
		goto Error;
	}

	calibration_log_info(mavlink_log_pub, "[cal] Connect battery now");

	timeout_start = hrt_absolute_time();

	while (true) {
		// We are either waiting for the user to connect the battery. Or we are waiting to let the PWM
		// sit high.
		hrt_abstime timeout_wait = batt_connected ? pwm_high_timeout : battery_connect_wait_timeout;

		if (hrt_elapsed_time(&timeout_start) > timeout_wait) {
			if (!batt_connected) {
				calibration_log_critical(mavlink_log_pub, CAL_QGC_FAILED_MSG, "Timeout waiting for battery");
				goto Error;
			}

			// PWM was high long enough
			break;
		}

		if (!batt_connected) {
			orb_check(batt_sub, &batt_updated);

			if (batt_updated) {
				orb_copy(ORB_ID(battery_status), batt_sub, &battery);

				if (battery.connected) {
					// Battery is connected, signal to user and start waiting again
					batt_connected = true;
					timeout_start = hrt_absolute_time();
					calibration_log_info(mavlink_log_pub, "[cal] Battery connected");
				}
			}
		}

		px4_usleep(50_ms);
	}

	/* tell IO to switch to the calibration low pwm phase */
	if (px4_ioctl(fd, PWM_SERVO_SET_CAL_LOW, 0) != PX4_OK) {
		calibration_log_critical(mavlink_log_pub, CAL_QGC_FAILED_MSG, "Unable to switch to to calibration low pwm phase");
		goto Error;
	}

	timeout_start = hrt_absolute_time();

	while (true) {
		// We are waiting to let the PWM sit low.

		if (hrt_absolute_time() - timeout_start > pwm_low_timeout) {
			if (!batt_connected) {
				calibration_log_critical(mavlink_log_pub, CAL_QGC_FAILED_MSG, "Timeout waiting for battery");
				goto Error;
			}

			// PWM was low long enough
			break;
		}

		px4_usleep(50_ms);
	}

Out:

	if (batt_sub != -1) {
		orb_unsubscribe(batt_sub);
	}

	if (fd != -1) {
		if (px4_ioctl(fd, PWM_SERVO_SET_FORCE_SAFETY_ON, 0) != PX4_OK) {
			calibration_log_info(mavlink_log_pub, CAL_QGC_FAILED_MSG, "Safety switch still off");
		}

		if (px4_ioctl(fd, PWM_SERVO_DISARM, 0) != PX4_OK) {
			calibration_log_info(mavlink_log_pub, CAL_QGC_FAILED_MSG, "Servos still armed");
		}

		if (px4_ioctl(fd, PWM_SERVO_CLEAR_ARM_OK, 0) != PX4_OK) {
			calibration_log_info(mavlink_log_pub, CAL_QGC_FAILED_MSG, "Safety switch still deactivated");
		}

		px4_close(fd);
	}

	armed->in_esc_calibration_mode = false;

	if (return_code == PX4_OK) {
		calibration_log_info(mavlink_log_pub, CAL_QGC_DONE_MSG, "esc");
	}

	return return_code;

Error:
	return_code = PX4_ERROR;
	goto Out;
#endif
}
