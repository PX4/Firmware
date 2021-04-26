/****************************************************************************
 *
 *   Copyright (c) 2017-2019 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskOrbit.hpp
 *
 * Flight task for orbiting in a circle around a target position
 *
 * @author Matthias Grob <maetugr@gmail.com>
 */

#pragma once

#include "FlightTaskManualAltitudeSmoothVel.hpp"
#include <uORB/Publication.hpp>
#include <uORB/topics/orbit_status.h>
#include "StraightLine.hpp"
#include <lib/slew_rate/SlewRateYaw.hpp>

class FlightTaskOrbit : public FlightTaskManualAltitudeSmoothVel
{
public:
	FlightTaskOrbit();
	virtual ~FlightTaskOrbit() = default;

	bool applyCommandParameters(const vehicle_command_s &command) override;
	bool activate(const vehicle_local_position_setpoint_s &last_setpoint) override;
	bool update() override;

	/**
	 * Check the feasibility of orbit parameters with respect to
	 * centripetal acceleration a = v^2 / r
	 * @param r desired radius
	 * @param v desired velocity
	 * @param a maximal allowed acceleration
	 * @return true on success, false if value not accepted
	 */
	bool checkAcceleration(float r, float v, float a);

protected:
	/**
	 * Send out telemetry information for the log and MAVLink.
	 * @return true on success, false on error
	 */
	bool sendTelemetry();

	/**
	 * Change the radius of the circle.
	 * @param r desired new radius
	 * @return true on success, false if value not accepted
	 */
	bool setRadius(const float r);

	/**
	 * Change the velocity of the vehicle on the circle.
	 * @param v desired new velocity
	 * @return true on success, false if value not accepted
	 */
	bool setVelocity(const float v);

private:
	/** generates setpoints to smoothly reach the closest point on the circle when starting from far away */
	void generate_circle_approach_setpoints(const matrix::Vector2f &center_to_position);
	/** generates xy setpoints to make the vehicle orbit */
	void generate_circle_setpoints(const matrix::Vector2f &center_to_position);
	/** generates yaw setpoints to control the vehicle's heading */
	void generate_circle_yaw_setpoints(const matrix::Vector2f &center_to_position);

	float _r = 0.f; /**< radius with which to orbit the target */
	float _v = 0.f; /**< clockwise tangential velocity for orbiting in m/s */
	matrix::Vector2f _center; /**< local frame coordinates of the center point */

	bool _in_circle_approach = false;
	StraightLine _circle_approach_line;

	// TODO: create/use parameters for limits
	const float _radius_min = 1.f;
	const float _radius_max = 100.f;
	const float _velocity_max = 10.f;
	const float _acceleration_max = 2.f;

	/** yaw behaviour during the orbit flight according to MAVLink's ORBIT_YAW_BEHAVIOUR enum */
	int _yaw_behaviour = orbit_status_s::ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER;
	float _initial_heading = 0.f; /**< the heading of the drone when the orbit command was issued */
	SlewRateYaw<float> _slew_rate_yaw;

	uORB::Publication<orbit_status_s> _orbit_status_pub{ORB_ID(orbit_status)};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MPC_XY_CRUISE>) _param_mpc_xy_cruise, /**< cruise speed for circle approach */
		(ParamFloat<px4::params::MPC_YAWRAUTO_MAX>) _param_mpc_yawrauto_max
	)
};
