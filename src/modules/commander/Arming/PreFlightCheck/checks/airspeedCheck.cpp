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
#include <drivers/drv_hrt.h>
#include <math.h>
#include <systemlib/mavlink_log.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/airspeed_validated.h>

using namespace time_literals;

bool PreFlightCheck::airspeedCheck(orb_advert_t *mavlink_log_pub, vehicle_status_s &status, const bool optional,
				   const bool report_fail, const bool prearm, const bool max_airspeed_check_en, const float arming_max_airspeed_allowed)
{
	bool present = true;
	bool success = true;

	uORB::SubscriptionData<airspeed_validated_s> airspeed_validated_sub{ORB_ID(airspeed_validated)};
	airspeed_validated_sub.update();
	const airspeed_validated_s &airspeed_validated = airspeed_validated_sub.get();

	/*
	 * Check if Airspeed Selector is up and running.
	 */
	if (hrt_elapsed_time(&airspeed_validated.timestamp) > 1_s) {
		if (report_fail && !optional) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Airspeed Selector module down.");
		}

		present = false;
		success = false;
		goto out;
	}

	/*
	 * Check if airspeed is declared valid or not by airspeed selector.
	 */
	if (!PX4_ISFINITE(airspeed_validated.calibrated_airspeed_m_s)) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Airspeed invalid.");
		}

		present = true;
		success = false;
		goto out;
	}

	/*
	 * Check if airspeed is higher than maximally accepted while the vehicle is landed / not flying
	 * Negative and positive offsets are considered. Do not check anymore while arming because pitot cover
	 * might have been removed.
	 */
	if (max_airspeed_check_en && fabsf(airspeed_validated.calibrated_airspeed_m_s) > arming_max_airspeed_allowed
	    && prearm) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: check Airspeed Cal or pitot");
		}

		present = true;
		success = false;
		goto out;
	}

out:
	set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_DIFFPRESSURE, present, !optional, success, status);

	return success;
}
