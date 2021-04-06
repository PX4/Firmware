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
 * @file FlightManualAltitude.hpp
 *
 * Flight task for manual controlled altitude.
 */

#pragma once

#include "FlightTask.hpp"
#include "Sticks.hpp"
#include <lib/ecl/AlphaFilter/AlphaFilter.hpp>
#include <uORB/Subscription.hpp>

class FlightTaskManualAltitude : public FlightTask
{
public:
	FlightTaskManualAltitude();
	virtual ~FlightTaskManualAltitude() = default;
	bool activate(const trajectory_setpoint_s &last_setpoint) override;
	bool updateInitialize() override;
	bool update() override;

protected:
	void _updateHeadingSetpoints(); /**< sets yaw or yaw speed */
	void _ekfResetHandlerHeading(float delta_psi) override; /**< adjust heading setpoint in case of EKF reset event */
	virtual void _updateSetpoints(); /**< updates all setpoints */
	virtual void _scaleSticks(); /**< scales sticks to velocity in z */
	bool _checkTakeoff() override;
	void _updateConstraintsFromEstimator();

	/**
	 * rotates vector into local frame
	 */
	void _rotateIntoHeadingFrame(matrix::Vector2f &vec);

	/**
	 *  Check and sets for position lock.
	 *  If sticks are at center position, the vehicle
	 *  will exit velocity control and enter position control.
	 */
	void _updateAltitudeLock();

	Sticks _sticks;
	bool _sticks_data_required = true; ///< let inherited task-class define if it depends on stick data

	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTask,
					(ParamFloat<px4::params::MPC_HOLD_MAX_Z>) _param_mpc_hold_max_z,
					(ParamInt<px4::params::MPC_ALT_MODE>) _param_mpc_alt_mode,
					(ParamFloat<px4::params::MPC_HOLD_MAX_XY>) _param_mpc_hold_max_xy,
					(ParamFloat<px4::params::MPC_Z_P>) _param_mpc_z_p, /**< position controller altitude propotional gain */
					(ParamFloat<px4::params::MPC_MAN_Y_MAX>) _param_mpc_man_y_max, /**< scaling factor from stick to yaw rate */
					(ParamFloat<px4::params::MPC_MAN_Y_TAU>) _param_mpc_man_y_tau,
					(ParamFloat<px4::params::MPC_MAN_TILT_MAX>) _param_mpc_man_tilt_max, /**< maximum tilt allowed for manual flight */
					(ParamFloat<px4::params::MPC_LAND_ALT1>) _param_mpc_land_alt1, /**< altitude at which to start downwards slowdown */
					(ParamFloat<px4::params::MPC_LAND_ALT2>) _param_mpc_land_alt2, /**< altitude below wich to land with land speed */
					(ParamFloat<px4::params::MPC_LAND_SPEED>)
					_param_mpc_land_speed, /**< desired downwards speed when approaching the ground */
					(ParamFloat<px4::params::MPC_TKO_SPEED>)
					_param_mpc_tko_speed, /**< desired upwards speed when still close to the ground */
					(ParamFloat<px4::params::MC_MAN_TILT_TAU>) _param_mc_man_tilt_tau
				       )
private:
	bool _isYawInput();
	void _unlockYaw();
	void _lockYaw();

	/**
	 * Filter between stick input and yaw setpoint
	 *
	 * @param yawspeed_target yaw setpoint desired by the stick
	 * @return filtered value from independent filter state
	 */
	float _applyYawspeedFilter(float yawspeed_target);

	/**
	 * Terrain following.
	 * During terrain following, the position setpoint is adjusted
	 * such that the distance to ground is kept constant.
	 * @param apply_brake is true if user wants to break
	 * @param stopped is true if vehicle has stopped moving in D-direction
	 */
	void _terrainFollowing(bool apply_brake, bool stopped);

	/**
	 * Minimum Altitude during range sensor operation.
	 * If a range sensor is used for altitude estimates, for
	 * best operation a minimum altitude is required. The minimum
	 * altitude is only enforced during altitude lock.
	 */
	void _respectMinAltitude();

	void _respectMaxAltitude();

	/**
	 * Sets downwards velocity constraint based on the distance to ground.
	 * To ensure a slowdown to land speed before hitting the ground.
	 */
	void _respectGroundSlowdown();

	void setGearAccordingToSwitch();

	float _yawspeed_filter_state{}; /**< state of low-pass filter in rad/s */
	uint8_t _reset_counter = 0; /**< counter for estimator resets in z-direction */
	float _max_speed_up = 10.0f;
	float _max_speed_down = 1.0f;
	bool _terrain_follow{false}; /**< true when the vehicle is following the terrain height */
	bool _terrain_hold{false}; /**< true when vehicle is controlling height above a static ground position */

	float _min_distance_to_ground{-INFINITY}; /**< min distance to ground constraint */
	float _max_distance_to_ground{INFINITY};  /**< max distance to ground constraint */

	/**
	 * Distance to ground during terrain following.
	 * If user does not demand velocity change in D-direction and the vehcile
	 * is in terrain-following mode, then height to ground will be locked to
	 * _dist_to_ground_lock.
	 */
	float _dist_to_ground_lock = NAN;

	AlphaFilter<matrix::Vector2f> _man_input_filter;
};
