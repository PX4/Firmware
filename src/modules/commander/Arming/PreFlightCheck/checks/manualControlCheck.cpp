/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include <systemlib/mavlink_log.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/manual_control_switches.h>

using namespace time_literals;

bool PreFlightCheck::manualControlCheck(orb_advert_t *mavlink_log_pub, const bool report_fail)
{
	bool success = true;

	uORB::SubscriptionData<manual_control_switches_s> manual_control_switches_sub{ORB_ID(manual_control_switches)};
	const manual_control_switches_s &manual_control_switches = manual_control_switches_sub.get();

	if (manual_control_switches.timestamp != 0) {

		// check action switches
		if (manual_control_switches.return_switch == manual_control_switches_s::SWITCH_POS_ON) {
			success = false;

			if (report_fail) {
				mavlink_log_critical(mavlink_log_pub, "Failure: RTL switch engaged");
			}
		}

		if (manual_control_switches.kill_switch == manual_control_switches_s::SWITCH_POS_ON) {
			success = false;

			if (report_fail) {
				mavlink_log_critical(mavlink_log_pub, "Failure: Kill switch engaged");
			}
		}

		if (manual_control_switches.gear_switch == manual_control_switches_s::SWITCH_POS_ON) {
			success = false;

			if (report_fail) {
				mavlink_log_critical(mavlink_log_pub, "Failure: Landing gear switch set in UP position");
			}
		}

	}

	return success;
}
