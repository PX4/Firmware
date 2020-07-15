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

/**
 * @file AngularVelocityControl.hpp
 *
 * PID 3 axis angular velocity control.
 */

#pragma once

#include <matrix/matrix/math.hpp>

#include <lib/mixer/MultirotorMixer/MultirotorMixer.hpp>

class AngularVelocityControl
{
public:
	AngularVelocityControl() = default;
	~AngularVelocityControl() = default;

	/**
	 * Set the control gains
	 * @param P 3D vector of proportional gains for body x,y,z axis
	 * @param I 3D vector of integral gains
	 * @param D 3D vector of derivative gains
	 */
	void setGains(const matrix::Vector3f &P, const matrix::Vector3f &I, const matrix::Vector3f &D);

	/**
	 * Set the mximum absolute value of the integrator for all axes
	 * @param integrator_limit limit value for all axes x, y, z
	 */
	void setIntegratorLimit(const matrix::Vector3f &integrator_limit) { _lim_int = integrator_limit; };

	/**
	 * Set direct angular velocity setpoint to torque feed forward gain
	 * @see _gain_ff
	 * @param FF 3D vector of feed forward gains for body x,y,z axis
	 */
	void setFeedForwardGain(const matrix::Vector3f &FF) { _gain_ff = FF; };

	/**
	 * Set inertia matrix
	 * @see _inertia
	 * @param inertia inertia matrix
	 */
	void setInertiaMatrix(const matrix::Matrix3f &inertia) { _inertia = inertia; };

	/**
	 * Set saturation status
	 * @see _saturation_positive
	 * @see _saturation_negative
	 * @param saturation_positive positive saturation
	 * @param saturation_negative negative saturation
	 */
	void setSaturationStatus(const matrix::Vector<bool, 3> &saturation_positive,
				 const matrix::Vector<bool, 3> &saturation_negative);

	/**
	 * Run one control loop cycle calculation
	 * @param angular_velocity estimation of the current vehicle angular velocity
	 * @param angular_velocity_sp desired vehicle angular velocity setpoint
	 * @param angular_acceleration estimation of the current vehicle angular acceleration
	 * @param dt elapsed time since last update
	 * @param landed whether the vehicle is on the ground
	 */
	void update(const matrix::Vector3f &angular_velocity, const matrix::Vector3f &angular_velocity_sp,
		    const matrix::Vector3f &angular_acceleration, const float dt, const bool landed);

	/**
	 * Get the desired angular acceleration
	 * @see _angular_accel_sp
	 */
	const matrix::Vector3f &getAngularAccelerationSetpoint() {return _angular_accel_sp;};

	/**
	 * Get the torque vector to apply to the vehicle
	 * @see _torque_sp
	 */
	const matrix::Vector3f &getTorqueSetpoint() {return _torque_sp;};

	/**
	 * Get the integral term
	 * @see _torque_sp
	 */
	const matrix::Vector3f &getIntegral() {return _angular_velocity_int;};

	/**
	 * Set the integral term to 0 to prevent windup
	 * @see _angular_velocity_int
	 */
	void resetIntegral() { _angular_velocity_int.zero(); }

	/**
	 * ReSet the controller state
	 */
	void reset();

private:
	void updateIntegral(matrix::Vector3f &angular_velocity_error, const float dt);

	// Gains
	matrix::Vector3f _gain_p; ///< proportional gain for all axes x, y, z
	matrix::Vector3f _gain_i; ///< integral gain
	matrix::Vector3f _gain_d; ///< derivative gain
	matrix::Vector3f _lim_int; ///< integrator term maximum absolute value
	matrix::Vector3f _gain_ff; ///< direct angular velocity to torque feed forward gain
	matrix::Matrix3f _inertia{matrix::eye<float, 3>()}; ///< inertia matrix

	// States
	matrix::Vector3f _angular_velocity_int;
	matrix::Vector<bool, 3> _saturation_positive;
	matrix::Vector<bool, 3> _saturation_negative;

	// Output
	matrix::Vector3f _angular_accel_sp; 	//< Angular acceleration setpoint computed using P and D gains
	matrix::Vector3f _torque_sp;		//< Torque setpoint to apply to the vehicle
};
