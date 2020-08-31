/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "../PreFlightCheck.hpp"

#include <drivers/drv_hrt.h>
#include <HealthFlags.h>
#include <math.h>
#include <px4_defines.h>
#include <lib/sensor_calibration/Utilities.hpp>
#include <lib/systemlib/mavlink_log.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/subsystem_info.h>

using namespace time_literals;

bool PreFlightCheck::accelerometerCheck(orb_advert_t *mavlink_log_pub, vehicle_status_s &status, const uint8_t instance,
					const bool optional, const bool dynamic, int32_t &device_id, const bool report_fail)
{
	const bool exists = (orb_exists(ORB_ID(sensor_accel), instance) == PX4_OK);
	bool calibration_valid = false;
	bool accel_valid = true;

	if (exists) {

		uORB::SubscriptionData<sensor_accel_s> accel{ORB_ID(sensor_accel), instance};

		accel_valid = (hrt_elapsed_time(&accel.get().timestamp) < 1_s);

		if (!accel_valid) {
			if (report_fail) {
				mavlink_log_critical(mavlink_log_pub, "Preflight Fail: no valid data from Accel #%u", instance);
			}
		}

		device_id = accel.get().device_id;

		calibration_valid = (calibration::FindCalibrationIndex("ACC", device_id) >= 0);

		if (!calibration_valid) {
			if (report_fail) {
				mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Accel #%u uncalibrated", instance);
			}

		} else {

			if (dynamic) {
				const float accel_magnitude = sqrtf(accel.get().x * accel.get().x
								    + accel.get().y * accel.get().y
								    + accel.get().z * accel.get().z);

				if (accel_magnitude < 4.0f || accel_magnitude > 15.0f /* m/s^2 */) {
					if (report_fail) {
						mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Accel Range, hold still on arming");
					}

					/* this is frickin' fatal */
					accel_valid = false;
				}
			}
		}

	} else {
		if (!optional && report_fail) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Accel Sensor #%u missing", instance);
		}
	}

	const bool success = calibration_valid && accel_valid;

	if (instance == 0) {
		set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_ACC, exists, !optional, success, status);

	} else if (instance == 1) {
		set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_ACC2, exists, !optional, success, status);
	}

	return success;
}
