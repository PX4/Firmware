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

#include <HealthFlags.h>

#include <lib/parameters/param.h>
#include <systemlib/mavlink_log.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_preflight_imu.h>
#include <uORB/topics/subsystem_info.h>

bool PreFlightCheck::imuConsistencyCheck(orb_advert_t *mavlink_log_pub, vehicle_status_s &status,
		const bool report_status)
{
	float test_limit = 1.0f; // pass limit re-used for each test

	// Get sensor_preflight data if available and exit with a fail recorded if not
	uORB::SubscriptionData<sensor_preflight_imu_s> sensors_sub{ORB_ID(sensor_preflight_imu)};
	sensors_sub.update();
	const sensor_preflight_imu_s &sensors = sensors_sub.get();

	// Use the difference between IMU's to detect a bad calibration.
	// If a single IMU is fitted, the value being checked will be zero so this check will always pass.
	param_get(param_find("COM_ARM_IMU_ACC"), &test_limit);

	if (sensors.accel_inconsistency_m_s_s > test_limit) {
		if (report_status) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Accels inconsistent - Check Cal");
			set_health_flags_healthy(subsystem_info_s::SUBSYSTEM_TYPE_ACC, false, status);
			set_health_flags_healthy(subsystem_info_s::SUBSYSTEM_TYPE_ACC2, false, status);
		}

		return false;

	} else if (sensors.accel_inconsistency_m_s_s > test_limit * 0.8f) {
		if (report_status) {
			mavlink_log_info(mavlink_log_pub, "Preflight Advice: Accels inconsistent - Check Cal");
		}
	}

	// Fail if gyro difference greater than 5 deg/sec and notify if greater than 2.5 deg/sec
	param_get(param_find("COM_ARM_IMU_GYR"), &test_limit);

	if (sensors.gyro_inconsistency_rad_s > test_limit) {
		if (report_status) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Gyros inconsistent - Check Cal");
			set_health_flags_healthy(subsystem_info_s::SUBSYSTEM_TYPE_GYRO, false, status);
			set_health_flags_healthy(subsystem_info_s::SUBSYSTEM_TYPE_GYRO2, false, status);
		}

		return false;

	} else if (sensors.gyro_inconsistency_rad_s > test_limit * 0.5f) {
		if (report_status) {
			mavlink_log_info(mavlink_log_pub, "Preflight Advice: Gyros inconsistent - Check Cal");
		}
	}

	return true;
}
