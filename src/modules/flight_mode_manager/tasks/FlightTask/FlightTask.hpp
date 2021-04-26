/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file FlightTask.hpp
 *
 * Abstract base class for different advanced flight tasks like orbit, follow me, ...
 *
 * @author Matthias Grob <maetugr@gmail.com>
 */

#pragma once

#include <px4_platform_common/module_params.h>
#include <drivers/drv_hrt.h>
#include <matrix/matrix/math.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/landing_gear.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_constraints.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_trajectory_waypoint.h>
#include <uORB/topics/home_position.h>
#include <lib/ecl/geo/geo.h>
#include <lib/weather_vane/WeatherVane.hpp>

struct ekf_reset_counters_s {
	uint8_t xy;
	uint8_t vxy;
	uint8_t z;
	uint8_t vz;
	uint8_t heading;
};

class FlightTask : public ModuleParams
{
public:
	FlightTask() :
		ModuleParams(nullptr)
	{
		_resetSetpoints();
		_constraints = empty_constraints;
	}

	virtual ~FlightTask() = default;

	/**
	 * Call once on the event where you switch to the task
	 * @param last_setpoint last output of the previous task
	 * @return true on success, false on error
	 */
	virtual bool activate(const vehicle_local_position_setpoint_s &last_setpoint);

	/**
	 * Call this to reset an active Flight Task
	 */
	virtual void reActivate();

	/**
	 * To be called to adopt parameters from an arrived vehicle command
	 * @param command received command message containing the parameters
	 * @return true if accepted, false if declined
	 */
	virtual bool applyCommandParameters(const vehicle_command_s &command) { return false; }

	/**
	 * Call before activate() or update()
	 * to initialize time and input data
	 * @return true on success, false on error
	 */
	virtual bool updateInitialize();

	/**
	 * To be called regularly in the control loop cycle to execute the task
	 * @return true on success, false on error
	 */
	virtual bool update();

	/**
	 * Call after update()
	 * to constrain the generated setpoints in order to comply
	 * with the constraints of the current mode
	 * @return true on success, false on error
	 */
	virtual bool updateFinalize() { return true; };

	/**
	 * Get the output data
	 * @return task output setpoints that get executed by the positon controller
	 */
	const vehicle_local_position_setpoint_s getPositionSetpoint();

	const ekf_reset_counters_s getResetCounters() const { return _reset_counters; }
	void setResetCounters(const ekf_reset_counters_s &counters) { _reset_counters = counters; }

	/**
	 * Get vehicle constraints.
	 * The constraints can vary with task.
	 * @return constraints
	 */
	const vehicle_constraints_s &getConstraints() { return _constraints; }

	/**
	 * Get landing gear position.
	 * The constraints can vary with task.
	 * @return landing gear
	 */
	const landing_gear_s &getGear() { return _gear; }

	/**
	 * Get avoidance desired waypoint
	 * @return desired waypoints
	 */
	const vehicle_trajectory_waypoint_s &getAvoidanceWaypoint() { return _desired_waypoint; }

	/**
	 * Empty setpoint.
	 * All setpoints are set to NAN.
	 */
	static const vehicle_local_position_setpoint_s empty_setpoint;

	/**
	 * Empty constraints.
	 * All constraints are set to NAN.
	 */
	static const vehicle_constraints_s empty_constraints;

	/**
	 * default landing gear state
	 */
	static const landing_gear_s empty_landing_gear_default_keep;

	/**
	 * Call this whenever a parameter update notification is received (parameter_update uORB message)
	 */
	void handleParameterUpdate()
	{
		updateParams();
	}

	/**
	 * Sets an external yaw handler which can be used by any flight task to implement a different yaw control strategy.
	 * This method does nothing, each flighttask which wants to use the yaw handler needs to override this method.
	 */
	virtual void setYawHandler(WeatherVane *ext_yaw_handler) {}

	void updateVelocityControllerFeedback(const matrix::Vector3f &vel_sp,
					      const matrix::Vector3f &acc_sp)
	{
		_velocity_setpoint_feedback = vel_sp;
		_acceleration_setpoint_feedback = acc_sp;
	}

protected:
	uORB::SubscriptionData<vehicle_local_position_s> _sub_vehicle_local_position{ORB_ID(vehicle_local_position)};
	uORB::SubscriptionData<home_position_s> _sub_home_position{ORB_ID(home_position)};

	/** Reset all setpoints to NAN */
	void _resetSetpoints();

	/** Check and update local position */
	void _evaluateVehicleLocalPosition();

	void _evaluateDistanceToGround();

	/** Set constraints to default values */
	virtual void _setDefaultConstraints();

	/** Determine when to trigger a takeoff (ignored in flight) */
	virtual bool _checkTakeoff();

	/**
	 * Monitor the EKF reset counters and
	 * call the appropriate handling functions in case of a reset event
	 * TODO: add the delta values to all the handlers
	 */
	void _checkEkfResetCounters();
	virtual void _ekfResetHandlerPositionXY() {};
	virtual void _ekfResetHandlerVelocityXY() {};
	virtual void _ekfResetHandlerPositionZ() {};
	virtual void _ekfResetHandlerVelocityZ() {};
	virtual void _ekfResetHandlerHeading(float delta_psi) {};

	map_projection_reference_s _global_local_proj_ref{};
	float                      _global_local_alt0{NAN};

	/* Time abstraction */
	static constexpr uint64_t _timeout = 500000; /**< maximal time in us before a loop or data times out */

	float _time{}; /**< passed time in seconds since the task was activated */
	float _deltatime{}; /**< passed time in seconds since the task was last updated */

	hrt_abstime _time_stamp_activate{}; /**< time stamp when task was activated */
	hrt_abstime _time_stamp_current{}; /**< time stamp at the beginning of the current task update */
	hrt_abstime _time_stamp_last{}; /**< time stamp when task was last updated */

	/* Current vehicle state */
	matrix::Vector3f _position; /**< current vehicle position */
	matrix::Vector3f _velocity; /**< current vehicle velocity */

	float _yaw{}; /**< current vehicle yaw heading */
	float _dist_to_bottom{}; /**< current height above ground level */
	float _dist_to_ground{}; /**< equals _dist_to_bottom if valid, height above home otherwise */

	/**
	 * Setpoints which the position controller has to execute.
	 * Setpoints that are set to NAN are not controlled. Not all setpoints can be set at the same time.
	 * If more than one type of setpoint is set, then order of control is a as follow: position, velocity,
	 * acceleration, thrust. The exception is _position_setpoint together with _velocity_setpoint, where the
	 * _velocity_setpoint and _acceleration_setpoint are used as feedforward.
	 * _jerk_setpoint does not executed but just serves as internal state.
	 */
	matrix::Vector3f _position_setpoint;
	matrix::Vector3f _velocity_setpoint;
	matrix::Vector3f _acceleration_setpoint;
	matrix::Vector3f _jerk_setpoint;

	float _yaw_setpoint{};
	float _yawspeed_setpoint{};

	matrix::Vector3f _velocity_setpoint_feedback;
	matrix::Vector3f _acceleration_setpoint_feedback;

	ekf_reset_counters_s _reset_counters{}; ///< Counters for estimator local position resets

	/**
	 * Vehicle constraints.
	 * The constraints can vary with tasks.
	 */
	vehicle_constraints_s _constraints{};

	landing_gear_s _gear{};

	/**
	 * Desired waypoints.
	 * Goals set by the FCU to be sent to the obstacle avoidance system.
	 */
	vehicle_trajectory_waypoint_s _desired_waypoint{};

	DEFINE_PARAMETERS_CUSTOM_PARENT(ModuleParams,
					(ParamFloat<px4::params::MPC_XY_VEL_MAX>) _param_mpc_xy_vel_max,
					(ParamFloat<px4::params::MPC_Z_VEL_MAX_DN>) _param_mpc_z_vel_max_dn,
					(ParamFloat<px4::params::MPC_Z_VEL_MAX_UP>) _param_mpc_z_vel_max_up
				       )
};
