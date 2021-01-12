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

/**
 * @file StickYaw.cpp
 */

#include "StickYaw.hpp"

#include <px4_platform_common/defines.h>

void StickYaw::generateYawSetpoint(float &yawspeed_setpoint, float &yaw_setpoint, const float desired_yawspeed,
				   const float yaw, const float deltatime)
{
	_yawspeed_slew_rate.setSlewRate(2.f * M_PI_F);
	yawspeed_setpoint = _yawspeed_slew_rate.update(desired_yawspeed, deltatime);
	yaw_setpoint = updateYawLock(yaw, yawspeed_setpoint, yaw_setpoint);
}

float StickYaw::updateYawLock(const float yaw, const float yawspeed_setpoint, const float yaw_setpoint)
{
	// Yaw-lock depends on desired yawspeed input. If not locked, yaw_sp is set to NAN.
	if (fabsf(yawspeed_setpoint) > FLT_EPSILON) {
		// no fixed heading when rotating around yaw by stick
		return NAN;

	} else {
		// break down and hold the current heading when no more rotation commanded
		if (!PX4_ISFINITE(yaw_setpoint)) {
			return yaw;

		} else {
			return yaw_setpoint;
		}
	}
}
