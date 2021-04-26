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

/**
 * @file PreFlightCheck.hpp
 *
 * Check if flight is safely possible
 * if not prevent it and inform the user.
 */

#pragma once

#include <uORB/topics/safety.h>
#include <uORB/topics/vehicle_status_flags.h>

#include <uORB/topics/vehicle_status.h>
#include <drivers/drv_hrt.h>

class PreFlightCheck
{
public:
	PreFlightCheck() = default;
	~PreFlightCheck() = default;

	/**
	* Runs a preflight check on all sensors to see if they are properly calibrated and healthy
	*
	* The function won't fail the test if optional sensors are not found, however,
	* it will fail the test if optional sensors are found but not in working condition.
	*
	* @param mavlink_log_pub
	*   Mavlink output orb handle reference for feedback when a sensor fails
	* @param checkMag
	*   true if the magneteometer should be checked
	* @param checkAcc
	*   true if the accelerometers should be checked
	* @param checkGyro
	*   true if the gyroscopes should be checked
	* @param checkBaro
	*   true if the barometer should be checked
	* @param checkAirspeed
	*   true if the airspeed sensor should be checked
	* @param checkRC
	*   true if the Remote Controller should be checked
	* @param checkGNSS
	*   true if the GNSS receiver should be checked
	* @param checkPower
	*   true if the system power should be checked
	**/
	static bool preflightCheck(orb_advert_t *mavlink_log_pub, vehicle_status_s &status,
				   vehicle_status_flags_s &status_flags, bool reportFailures, const bool prearm,
				   const hrt_abstime &time_since_boot);

	struct arm_requirements_t {
		bool arm_authorization = false;
		bool esc_check = false;
		bool global_position = false;
		bool mission = false;
	};

	static bool preArmCheck(orb_advert_t *mavlink_log_pub, const vehicle_status_flags_s &status_flags,
				const safety_s &safety, const arm_requirements_t &arm_requirements, vehicle_status_s &status,
				bool report_fail = true);

private:
	static bool magnetometerCheck(orb_advert_t *mavlink_log_pub, vehicle_status_s &status, const uint8_t instance,
				      const bool optional, int32_t &device_id, const bool report_fail);
	static bool magConsistencyCheck(orb_advert_t *mavlink_log_pub, vehicle_status_s &status, const bool report_status);
	static bool accelerometerCheck(orb_advert_t *mavlink_log_pub, vehicle_status_s &status, const uint8_t instance,
				       const bool optional, int32_t &device_id, const bool report_fail);
	static bool gyroCheck(orb_advert_t *mavlink_log_pub, vehicle_status_s &status, const uint8_t instance,
			      const bool optional, int32_t &device_id, const bool report_fail);
	static bool baroCheck(orb_advert_t *mavlink_log_pub, vehicle_status_s &status, const uint8_t instance,
			      const bool optional, int32_t &device_id, const bool report_fail);
	static bool imuConsistencyCheck(orb_advert_t *mavlink_log_pub, vehicle_status_s &status, const bool report_status);
	static bool airspeedCheck(orb_advert_t *mavlink_log_pub, vehicle_status_s &status, const bool optional,
				  const bool report_fail, const bool prearm, const bool max_airspeed_check_en, const float arming_max_airspeed_allowed);
	static int rcCalibrationCheck(orb_advert_t *mavlink_log_pub, bool report_fail, bool isVTOL);
	static bool powerCheck(orb_advert_t *mavlink_log_pub, const vehicle_status_s &status, const bool report_fail,
			       const bool prearm);
	static bool ekf2Check(orb_advert_t *mavlink_log_pub, vehicle_status_s &vehicle_status, const bool optional,
			      const bool report_fail);

	static bool ekf2CheckSensorBias(orb_advert_t *mavlink_log_pub, const bool report_fail);

	static bool failureDetectorCheck(orb_advert_t *mavlink_log_pub, const vehicle_status_s &status, const bool report_fail,
					 const bool prearm);

	static bool manualControlCheck(orb_advert_t *mavlink_log_pub, const bool report_fail);
	static bool airframeCheck(orb_advert_t *mavlink_log_pub, const vehicle_status_s &status);
	static bool cpuResourceCheck(orb_advert_t *mavlink_log_pub, const bool report_fail);
	static bool sdcardCheck(orb_advert_t *mavlink_log_pub, bool &sd_card_detected_once, const bool report_fail);
};
