/****************************************************************************
 *
 *   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
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
#include <systemlib/mavlink_log.h>
#include <lib/parameters/param.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/system_power.h>

using namespace time_literals;

unsigned int countSetBits(unsigned int n)
{
	unsigned int count = 0;

	while (n) {
		count += n & 1;
		n >>= 1;
	}

	return count;
}

bool PreFlightCheck::powerCheck(orb_advert_t *mavlink_log_pub, const vehicle_status_s &status, const bool report_fail,
				const bool prearm)
{
	bool success = true;

	if (!prearm) {
		// Ignore power check after arming.
		return true;
	}

	uORB::SubscriptionData<system_power_s> system_power_sub{ORB_ID(system_power)};
	system_power_sub.update();
	const system_power_s &system_power = system_power_sub.get();

	if (system_power.timestamp != 0) {
		int32_t required_power_module_count = 0;
		param_get(param_find("COM_POWER_COUNT"), &required_power_module_count);

		// Check avionics rail voltages (if USB isn't connected)
		if (!system_power.usb_connected) {
			float avionics_power_rail_voltage = system_power.voltage5v_v;

			if (avionics_power_rail_voltage < 4.5f) {
				success = false;

				if (report_fail) {
					mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Avionics Power low: %6.2f Volt",
							     (double)avionics_power_rail_voltage);
				}

			} else if (avionics_power_rail_voltage < 4.8f) {
				if (report_fail) {
					mavlink_log_critical(mavlink_log_pub, "CAUTION: Avionics Power low: %6.2f Volt", (double)avionics_power_rail_voltage);
				}

			} else if (avionics_power_rail_voltage > 5.4f) {
				if (report_fail) {
					mavlink_log_critical(mavlink_log_pub, "CAUTION: Avionics Power high: %6.2f Volt", (double)avionics_power_rail_voltage);
				}
			}


			const int power_module_count = countSetBits(system_power.brick_valid);

			if (power_module_count < required_power_module_count) {
				success = false;

				if (report_fail) {
					mavlink_log_critical(mavlink_log_pub, "Power redundancy not met: %d instead of %d",
							     power_module_count, required_power_module_count);
				}
			}
		}

	} else {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "system power unavailable");
		}

		success = false;
	}

	return success;
}
