/****************************************************************************
 *
 *   Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
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
 * @file StraightLine.cpp
 */

#include "StraightLine.hpp"
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

using namespace matrix;

void StraightLine::generateSetpoints(matrix::Vector3f &position_setpoint, matrix::Vector3f &velocity_setpoint)
{
	if (isEndReached()) {
		// Vehicle has reached target, lock position
		position_setpoint = _end;
		velocity_setpoint.setNaN();
		return;
	}

	Vector3f start_to_end = _end - _start;
	float distance_start_to_end = start_to_end.norm();

	// capture progress as ratio between 0 and 1 of entire distance
	Vector3f vehicle_to_end = _end - _position;
	float distance_vehicle_to_end = vehicle_to_end.norm();
	float distance_from_start = Vector3f(_start - _position).norm();
	float progress = distance_from_start / (distance_from_start + distance_vehicle_to_end);

	float distance_from_boundary = 0.f;

	// calculate the distance to the closer boundary
	if (progress < 0.5f) {
		distance_from_boundary = (2 * progress) * distance_start_to_end;

	} else {
		distance_from_boundary = (2 * (1 - progress)) * distance_start_to_end;
	}

	// ramp velocity based on the distance to the boundary
	float velocity = 0.5f + (distance_from_boundary / 4.f);
	velocity = math::min(velocity, _speed);
	velocity_setpoint = vehicle_to_end.unit_or_zero() * velocity;

	// check if we plan to go against the line direction which indicates we reached the goal
	if (start_to_end * vehicle_to_end < 0) {
		_end_reached = true;
	}
}

void StraightLine::setLineFromTo(const Vector3f &start, const Vector3f &end)
{
	if (PX4_ISFINITE(start.norm_squared()) && PX4_ISFINITE(end.norm_squared())) {
		_start = start;
		_end = end;
		_end_reached = false;
	}
}
