/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskManualPositionHeadless.hpp
 *
 * Flight task for manual controlled headless position.
 *
 * @author Inhwan Wee <inhwan.wee@gmail.com>
 */

#pragma once

#include <lib/collision_prevention/CollisionPrevention.hpp>
#include "FlightTaskManualPosition.hpp"

class FlightTaskManualPositionHeadless : public FlightTaskManualPosition
{
public:
	FlightTaskManualPositionHeadless();
	virtual ~FlightTaskManualPositionHeadless() = default;
	bool activate(const vehicle_local_position_setpoint_s &last_setpoint) override;

protected:
	void _scaleSticks() override;
	/**
	 * rotates vector into local frame
	 */
	void _rotateIntoHeadingFrame(matrix::Vector2f &vec);

private:
	float _computeVelXYGroundDist();
	float _velocity_scale{0.0f}; //scales the stick input to velocity
	float _reference_yaw{0.0f};

	CollisionPrevention _collision_prevention;	/**< collision avoidance setpoint amendment */
};
