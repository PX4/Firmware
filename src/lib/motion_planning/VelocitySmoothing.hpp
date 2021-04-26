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

#pragma once

struct Trajectory {
	float j; //< jerk
	float a; //< acceleration
	float v; //< velocity
	float x; //< position
};

/**
 * @class VelocitySmoothing
 *
 * TODO: document the algorithm
 *    |T1| T2 |T3|
 *     ___
 *   __| |____   __ Jerk
 *            |_|
 *        ___
 *       /   \	 Acceleration
 *   ___/     \___
 *             ___
 *           ;"
 *          /
 *         / 	 Velocity
 *        ;
 *   ----"
 */
class VelocitySmoothing
{
public:
	VelocitySmoothing(float initial_accel = 0.f, float initial_vel = 0.f, float initial_pos = 0.f);
	~VelocitySmoothing() = default;

	/**
	 * Reset the state.
	 * @param accel Current acceleration
	 * @param vel Current velocity
	 * @param pos Current position
	 */
	void reset(float accel, float vel, float pos);

	/**
	 * Compute T1, T2, T3 depending on the current state and velocity setpoint. This should be called on every cycle
	 * and before updateTraj().
	 * @param vel_setpoint velocity setpoint input
	 */
	void updateDurations(float vel_setpoint);

	/**
	 * Generate the trajectory (acceleration, velocity and position) by integrating the current jerk
	 * @param dt integration period
	 * @param time_stretch (optional) used to scale the integration period. This can be used to slow down
	 * or fast-forward the trajectory
	 */
	void updateTraj(float dt, float time_stretch = 1.f);

	/**
	 * Getters and setters
	 */
	float getMaxJerk() const { return _max_jerk; }
	void setMaxJerk(float max_jerk) { _max_jerk = max_jerk; }

	float getMaxAccel() const { return _max_accel; }
	void setMaxAccel(float max_accel) { _max_accel = max_accel; }

	float getMaxVel() const { return _max_vel; }
	void setMaxVel(float max_vel) { _max_vel = max_vel; }

	float getCurrentJerk() const { return _state.j; }
	void setCurrentAcceleration(const float accel) { _state.a = _state_init.a = accel; }
	float getCurrentAcceleration() const { return _state.a; }
	void setCurrentVelocity(const float vel) { _state.v = _state_init.v = vel; }
	float getCurrentVelocity() const { return _state.v; }
	void setCurrentPosition(const float pos) { _state.x = _state_init.x = pos; }
	float getCurrentPosition() const { return _state.x; }

	float getVelSp() const { return _vel_sp; }

	float getT1() const { return _T1; }
	float getT2() const { return _T2; }
	float getT3() const { return _T3; }
	float getTotalTime() const { return _T1 + _T2 + _T3; }

	/**
	 * Synchronize several trajectories to have the same total time. This is required to generate
	 * straight lines.
	 * The resulting total time is the one of the longest trajectory.
	 * @param traj an array of VelocitySmoothing objects
	 * @param n_traj the number of trajectories to be synchronized
	 */
	static void timeSynchronization(VelocitySmoothing *traj, int n_traj);

private:

	/**
	 * Compute T1, T2, T3 depending on the current state and velocity setpoint.
	 * Minimize the total time of the trajectory
	 */
	void updateDurationsMinimizeTotalTime();

	/**
	 * Compute T1, T2, T3 depending on the current state and velocity setpoint.
	 * @param T123 desired total time of the trajectory
	 */
	void updateDurationsGivenTotalTime(float T123);

	/**
	 * Compute the direction of the jerk to be applied in order to drive the current state
	 * to the desired one
	 */
	int computeDirection();

	/**
	 * Compute the velocity at which the trajectory will be if the maximum jerk is applied
	 * during the time required to cancel the current acceleration
	 */
	float computeVelAtZeroAcc();

	/**
	 * Compute increasing acceleration time
	 */
	inline float computeT1(float a0, float v3, float j_max, float a_max);

	/**
	 * Compute increasing acceleration time using total time constraint
	 */
	inline float computeT1(float T123, float a0, float v3, float j_max, float a_max);

	/**
	 * Saturate T1 in order to respect the maximum acceleration constraint
	 */
	inline float saturateT1ForAccel(float a0, float j_max, float T1, float a_max);

	/**
	 * Compute constant acceleration time
	 */
	inline float computeT2(float T1, float T3, float a0, float v3, float j_max);

	/**
	 * Compute constant acceleration time using total time constraint
	 */
	inline float computeT2(float T123, float T1, float T3);

	/**
	 * Compute decreasing acceleration time
	 */
	inline float computeT3(float T1, float a0, float j_max);

	/**
	 * Compute the jerk, acceleration, velocity and position
	 * of a jerk-driven polynomial trajectory at a given time t
	 * @param j jerk
	 * @param a0 initial acceleration at t = 0
	 * @param v0 initial velocity
	 * @param x0 initial postion
	 * @param t current time
	 * @param d direction
	 */
	inline Trajectory evaluatePoly(float j, float a0, float v0, float x0, float t, int d);

	/* Input */
	float _vel_sp{0.0f};

	/* Constraints */
	float _max_jerk = 22.f;
	float _max_accel = 8.f;
	float _max_vel = 6.f;

	/* State (previous setpoints) */
	Trajectory _state{};
	int _direction{0};

	/* Initial conditions */
	Trajectory _state_init{};

	/* Duration of each phase */
	float _T1 = 0.f; ///< Increasing acceleration [s]
	float _T2 = 0.f; ///< Constant acceleration [s]
	float _T3 = 0.f; ///< Decreasing acceleration [s]

	float _local_time = 0.f; ///< Current local time
};
