/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file VtolLandDetector.cpp
 * Land detection algorithm for VTOL
 *
 * @author Roman Bapst <bapstroma@gmail.com>
 * @author Julian Oes <julian@oes.ch>
 */

#include <drivers/drv_hrt.h>
#include <matrix/math.hpp>

#include "VtolLandDetector.h"

namespace land_detector
{

void VtolLandDetector::_update_topics()
{
	MulticopterLandDetector::_update_topics();
}

bool VtolLandDetector::_get_maybe_landed_state()
{
	// If in Fixed-wing mode, only trigger if disarmed
	if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
		return !_armed;
	}

	return MulticopterLandDetector::_get_maybe_landed_state();
}

bool VtolLandDetector::_get_landed_state()
{
	// If in Fixed-wing mode, only trigger if disarmed
	if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
		return !_armed;
	}

	// this is returned from the mutlicopter land detector
	bool landed = MulticopterLandDetector::_get_landed_state();

	// for vtol we additionally consider airspeed
	airspeed_validated_s airspeed_validated{};
	_airspeed_validated_sub.copy(&airspeed_validated);

	if (hrt_elapsed_time(&airspeed_validated.timestamp) < 1_s && PX4_ISFINITE(airspeed_validated.true_airspeed_m_s)) {

		_airspeed_filtered = 0.95f * _airspeed_filtered + 0.05f * airspeed_validated.true_airspeed_m_s;

	} else {
		// if airspeed does not update, set it to zero and rely on multicopter land detector
		_airspeed_filtered = 0.0f;
	}

	// only consider airspeed if we have been in air before to avoid false
	// detections in the case of wind on the ground
	if (_was_in_air && (_airspeed_filtered > _param_lndfw_airspd_max.get())) {
		landed = false;
	}

	_was_in_air = !landed;

	return landed;
}

bool VtolLandDetector::_get_freefall_state()
{
	// true if falling or in a parabolic flight (low gravity)
	bool free_fall_detected = MulticopterLandDetector::_get_freefall_state();

	// only return a positive free fall detected if not in fixed-wing mode
	return _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_FIXED_WING && free_fall_detected;
}

} // namespace land_detector
