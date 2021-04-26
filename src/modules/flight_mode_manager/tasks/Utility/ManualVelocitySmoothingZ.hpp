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
 * @file ManualVelocitySmoothingZ.hpp
 *
 */

#pragma once

#include <motion_planning/VelocitySmoothing.hpp>

class ManualVelocitySmoothingZ final
{
public:
	ManualVelocitySmoothingZ() = default;
	~ManualVelocitySmoothingZ() = default;

	void reset(float accel, float vel, float pos);
	void update(float dt, float velocity_target);

	void setVelSpFeedback(const float fb) { _velocity_setpoint_feedback = fb; }

	void setMaxJerk(const float max_jerk)
	{
		_trajectory.setMaxJerk(max_jerk);
	}
	void setMaxAccelUp(const float max_accel_up)
	{
		_max_accel_up = max_accel_up;
	}
	void setMaxAccelDown(const float max_accel_down)
	{
		_max_accel_down = max_accel_down;
	}
	void setMaxVelUp(const float max_vel_up)
	{
		_max_vel_up = max_vel_up;
	}
	void setMaxVelDown(const float max_vel_down)
	{
		_max_vel_down = max_vel_down;
	}

	float getCurrentJerk() const { return _state.j; }
	float getCurrentAcceleration() const { return _state.a; }
	void setCurrentVelocity(const float vel)
	{
		_state.v = vel;
		_trajectory.setCurrentVelocity(vel);
	}
	float getCurrentVelocity() const { return _state.v; }
	void setCurrentPosition(const float pos)
	{
		_state.x = pos;
		_trajectory.setCurrentPosition(pos);
		_position_estimate = pos;

		if (_position_lock_active) {
			_position_setpoint_locked = pos;
		}
	}
	float getCurrentPosition() const { return _position_setpoint_locked; }
	void setCurrentPositionEstimate(float pos) { _position_estimate = pos; }

private:
	void resetPositionLock();
	void updateTrajectories(float dt);
	void updateTrajConstraints(float vel_target);
	void checkPositionLock(float velocity_target);
	void updateTrajDurations(float velocity_target);

	VelocitySmoothing _trajectory; ///< Trajectory in z direction

	bool _position_lock_active{false};

	float _position_setpoint_locked{};

	float _velocity_setpoint_feedback{};
	float _position_estimate{};

	struct {
		float j;
		float a;
		float v;
		float x;
	} _state{};

	float _max_accel_up{0.f};
	float _max_accel_down{0.f};
	float _max_vel_up{0.f};
	float _max_vel_down{0.f};
};
