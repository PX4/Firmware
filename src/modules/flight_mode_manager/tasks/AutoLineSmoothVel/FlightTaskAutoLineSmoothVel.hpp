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
 * @file FlightTaskAutoLineSmoothVel.hpp
 *
 * Flight task for autonomous, gps driven mode. The vehicle flies
 * along a straight line in between waypoints.
 */

#pragma once

#include "FlightTaskAutoMapper.hpp"
#include <motion_planning/VelocitySmoothing.hpp>

class FlightTaskAutoLineSmoothVel : public FlightTaskAutoMapper
{
public:
	FlightTaskAutoLineSmoothVel() = default;
	virtual ~FlightTaskAutoLineSmoothVel() = default;

	bool activate(const trajectory_setpoint_s &last_setpoint) override;
	void reActivate() override;

protected:

	/** Reset position or velocity setpoints in case of EKF reset event */
	void _ekfResetHandlerPositionXY() override;
	void _ekfResetHandlerVelocityXY() override;
	void _ekfResetHandlerPositionZ() override;
	void _ekfResetHandlerVelocityZ() override;
	void _ekfResetHandlerHeading(float delta_psi) override;

	void _generateSetpoints() override; /**< Generate setpoints along line. */
	void _generateHeading();
	void _updateTurningCheck();
	bool _generateHeadingAlongTraj(); /**< Generates heading along trajectory. */

	static float _constrainOneSide(float val, float constraint); /**< Constrain val between INF and constraint */

	static float _constrainAbs(float val, float max); /** Constrain the value -max <= val <= max */

	float _getMaxXYSpeed() const;
	float _getMaxZSpeed() const;

	matrix::Vector3f getCrossingPoint() const;
	bool isTargetModified() const;
	matrix::Vector2f getL1Point() const;

	float _max_speed_prev{};
	bool _is_turning{false};

	void _prepareSetpoints(); /**< Generate velocity target points for the trajectory generator. */
	void _updateTrajConstraints();
	void _generateTrajectory();

	/** determines when to trigger a takeoff (ignored in flight) */
	bool _checkTakeoff() override { return _want_takeoff; };
	bool _want_takeoff{false};

	VelocitySmoothing _trajectory[3]; ///< Trajectories in x, y and z directions

	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTaskAutoMapper,
					(ParamFloat<px4::params::MIS_YAW_ERR>) _param_mis_yaw_err, // yaw-error threshold
					(ParamFloat<px4::params::MPC_ACC_HOR>) _param_mpc_acc_hor, // acceleration in flight
					(ParamFloat<px4::params::MPC_ACC_UP_MAX>) _param_mpc_acc_up_max,
					(ParamFloat<px4::params::MPC_ACC_DOWN_MAX>) _param_mpc_acc_down_max,
					(ParamFloat<px4::params::MPC_JERK_AUTO>) _param_mpc_jerk_auto,
					(ParamFloat<px4::params::MPC_XY_TRAJ_P>) _param_mpc_xy_traj_p,
					(ParamFloat<px4::params::MPC_XY_ERR_MAX>) _param_mpc_xy_err_max
				       );
};
