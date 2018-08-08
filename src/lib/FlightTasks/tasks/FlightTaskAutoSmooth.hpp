/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskAutoSmooth.hpp
 *
 * Flight task for smooth, autonomous, gps driven mode.
 *
 */

#pragma once

#include "FlightTaskAutoMapper.hpp"
#include "lib/bezier/BezierQuad.hpp"
#include "Utility/StraightLine.hpp"

class FlightTaskAutoSmooth : public FlightTaskAutoMapper
{
public:
	FlightTaskAutoSmooth();
	virtual ~FlightTaskAutoSmooth() = default;
	void _generateSetpoints() override;

protected:
	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTaskAutoMapper,
					(ParamFloat<px4::params::NAV_ACC_RAD>) NAV_ACC_RAD, // acceptance radius at which waypoints are updated
					(ParamFloat<px4::params::MPC_CRUISE_90>) MPC_CRUISE_90 // speed at corner when angle is 90 degrees
				       )

private:

	bezier::BezierQuad_f _bezier;
	StraightLine _line;
	matrix::Vector3f _pt_0; /**< Bezier starting point */
	matrix::Vector3f _pt_1; /**< Bezier end point */

	bool _lock_position = false;
	bool _control_points_update = true;
	bool _pt_0_reached_once = false;

	void _update_control_points(); /**< Update Bezier control points */
};
