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

#include <systemlib/mavlink_log.h>

bool PreFlightCheck::failureDetectorCheck(orb_advert_t *mavlink_log_pub, const vehicle_status_s &status,
		const bool report_fail, const bool prearm)
{
	// Ignore failure detector check after arming
	if (!prearm) {
		return true;
	}

	if (status.failure_detector_status != vehicle_status_s::FAILURE_NONE) {
		if (report_fail) {
			if (status.failure_detector_status & vehicle_status_s::FAILURE_ROLL) {
				mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Roll failure detected");
			}

			if (status.failure_detector_status & vehicle_status_s::FAILURE_PITCH) {
				mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Pitch failure detected");
			}

			if (status.failure_detector_status & vehicle_status_s::FAILURE_ALT) {
				mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Altitude failure detected");
			}

			if (status.failure_detector_status & vehicle_status_s::FAILURE_EXT) {
				mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Parachute failure detected");
			}

			if (status.failure_detector_status & vehicle_status_s::FAILURE_ARM_ESC) {
				mavlink_log_critical(mavlink_log_pub, "Preflight Fail: ESC failure detected");
			}
		}

		return false;
	}

	return true;
}
