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
#include <mathlib/mathlib.h>
#include <systemlib/mavlink_log.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_preflight_mag.h>

// return false if the magnetomer measurements are inconsistent
bool PreFlightCheck::magConsistencyCheck(orb_advert_t *mavlink_log_pub, vehicle_status_s &status,
		const bool report_status)
{
	bool pass = false; // flag for result of checks

	// get the sensor preflight data
	uORB::SubscriptionData<sensor_preflight_mag_s> sensors_sub{ORB_ID(sensor_preflight_mag)};
	sensors_sub.update();
	const sensor_preflight_mag_s &sensors = sensors_sub.get();

	if (sensors.timestamp == 0) {
		// can happen if not advertised (yet)
		pass = true;
	}

	// Use the difference between sensors to detect a bad calibration, orientation or magnetic interference.
	// If a single sensor is fitted, the value being checked will be zero so this check will always pass.
	int32_t angle_difference_limit_deg = 90;
	param_get(param_find("COM_ARM_MAG_ANG"), &angle_difference_limit_deg);

	pass = pass || angle_difference_limit_deg < 0; // disabled, pass check
	pass = pass || sensors.mag_inconsistency_angle < math::radians<float>(angle_difference_limit_deg);

	if (!pass && report_status) {
		mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Compasses %d° inconsistent",
				     static_cast<int>(math::degrees<float>(sensors.mag_inconsistency_angle)));
		mavlink_log_critical(mavlink_log_pub, "Please check orientations and recalibrate");
		set_health_flags_healthy(subsystem_info_s::SUBSYSTEM_TYPE_MAG, false, status);
		set_health_flags_healthy(subsystem_info_s::SUBSYSTEM_TYPE_MAG2, false, status);
	}

	return pass;
}
