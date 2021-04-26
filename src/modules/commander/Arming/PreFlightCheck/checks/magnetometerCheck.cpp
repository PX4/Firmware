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
#include <px4_defines.h>
#include <lib/sensor_calibration/Utilities.hpp>
#include <lib/systemlib/mavlink_log.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/sensor_mag.h>

using namespace time_literals;

bool PreFlightCheck::magnetometerCheck(orb_advert_t *mavlink_log_pub, vehicle_status_s &status, const uint8_t instance,
				       const bool optional, int32_t &device_id, const bool report_fail)
{
	const bool exists = (orb_exists(ORB_ID(sensor_mag), instance) == PX4_OK);
	bool calibration_valid = false;
	bool valid = false;
	bool is_mag_fault = false;

	if (exists) {

		uORB::SubscriptionData<sensor_mag_s> magnetometer{ORB_ID(sensor_mag), instance};

		valid = (magnetometer.get().device_id != 0) && (magnetometer.get().timestamp != 0);

		if (!valid) {
			if (report_fail) {
				mavlink_log_critical(mavlink_log_pub, "Preflight Fail: no valid data from Compass %u", instance);
			}
		}

		device_id = magnetometer.get().device_id;

		calibration_valid = (calibration::FindCalibrationIndex("MAG", device_id) >= 0);

		if (!calibration_valid) {
			if (report_fail) {
				mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Compass %u uncalibrated", instance);
			}
		}

		for (uint8_t i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
			uORB::SubscriptionData<estimator_status_s> estimator_status_sub{ORB_ID(estimator_status), i};

			if (estimator_status_sub.get().mag_device_id == static_cast<uint32_t>(device_id)) {
				if (estimator_status_sub.get().control_mode_flags & (1 << estimator_status_s::CS_MAG_FAULT)) {
					is_mag_fault = true;
					break;
				}
			}
		}

		if (is_mag_fault && report_fail) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Compass #%u fault", instance);
		}

	} else {
		if (!optional && report_fail) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Compass Sensor %u missing", instance);
		}
	}

	const bool success = calibration_valid && valid && !is_mag_fault;

	if (instance == 0) {
		set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_MAG, exists, !optional, success, status);

	} else if (instance == 1) {
		set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_MAG2, exists, !optional, success, status);
	}

	return success;
}
