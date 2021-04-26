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

#include "VelocitySmoothing.hpp"

#include <cstdio>
#include <float.h>
#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>

using matrix::sign;

VelocitySmoothing::VelocitySmoothing(float initial_accel, float initial_vel, float initial_pos)
{
	reset(initial_accel, initial_vel, initial_pos);
}

void VelocitySmoothing::reset(float accel, float vel, float pos)
{
	_state.j = 0.f;
	_state.a = accel;
	_state.v = vel;
	_state.x = pos;

	_state_init = _state;
}

float VelocitySmoothing::saturateT1ForAccel(float a0, float j_max, float T1, float a_max)
{
	/* Check maximum acceleration, saturate and recompute T1 if needed */
	float accel_T1 = a0 + j_max * T1;
	float T1_new = T1;

	if (accel_T1 > a_max) {
		T1_new = (a_max - a0) / j_max;

	} else if (accel_T1 < -a_max) {
		T1_new = (-a_max - a0) / j_max;
	}

	return T1_new;
}

float VelocitySmoothing::computeT1(float a0, float v3, float j_max, float a_max)
{
	float delta = 2.f * a0 * a0 + 4.f * j_max * v3;

	if (delta < 0.f) {
		// Solution is not real
		return 0.f;
	}

	float sqrt_delta = sqrtf(delta);
	float T1_plus = (-a0 + 0.5f * sqrt_delta) / j_max;
	float T1_minus = (-a0 - 0.5f * sqrt_delta) / j_max;

	float T3_plus = a0 / j_max + T1_plus;
	float T3_minus = a0 / j_max + T1_minus;

	float T1 = 0.f;

	if (T1_plus >= 0.f && T3_plus >= 0.f) {
		T1 = T1_plus;

	} else if (T1_minus >= 0.f && T3_minus >= 0.f) {
		T1 = T1_minus;
	}

	T1 = saturateT1ForAccel(a0, j_max, T1, a_max);

	return math::max(T1, 0.f);
}

float VelocitySmoothing::computeT1(float T123, float a0, float v3, float j_max, float a_max)
{
	float a = -j_max;
	float b = j_max * T123 - a0;
	float delta = T123 * T123 * j_max * j_max + 2.f * T123 * a0 * j_max - a0 * a0 - 4.f * j_max * v3;

	if (delta < 0.f) {
		// Solution is not real
		return 0.f;
	}

	float sqrt_delta = sqrtf(delta);
	float denominator_inv = 1.f / (2.f * a);
	float T1_plus = math::max((-b + sqrt_delta) * denominator_inv, 0.f);
	float T1_minus = math::max((-b - sqrt_delta) * denominator_inv, 0.f);

	float T3_plus = a0 / j_max + T1_plus;
	float T3_minus = a0 / j_max + T1_minus;

	float T13_plus = T1_plus + T3_plus;
	float T13_minus = T1_minus + T3_minus;

	float T1 = 0.f;

	if (T13_plus > T123) {
		T1 = T1_minus;

	} else if (T13_minus > T123) {
		T1 = T1_plus;
	}

	T1 = saturateT1ForAccel(a0, j_max, T1, a_max);

	return T1;
}


float VelocitySmoothing::computeT2(float T1, float T3, float a0, float v3, float j_max)
{
	float T2 = 0.f;

	float den = a0 + j_max * T1;

	if (math::abs_t(den) > FLT_EPSILON) {
		T2 = (-0.5f * T1 * T1 * j_max - T1 * T3 * j_max - T1 * a0 + 0.5f * T3 * T3 * j_max - T3 * a0 + v3) / den;
	}

	return math::max(T2, 0.f);
}

float VelocitySmoothing::computeT2(float T123, float T1, float T3)
{
	float T2 = T123 - T1 - T3;
	return math::max(T2, 0.f);
}

float VelocitySmoothing::computeT3(float T1, float a0, float j_max)
{
	float T3 = a0 / j_max + T1;
	return math::max(T3, 0.f);
}

void VelocitySmoothing::updateDurations(float vel_setpoint)
{
	_vel_sp = math::constrain(vel_setpoint, -_max_vel, _max_vel);
	_local_time = 0.f;
	_state_init = _state;

	_direction = computeDirection();

	if (_direction != 0) {
		updateDurationsMinimizeTotalTime();

	} else {
		_T1 = _T2 = _T3 = 0.f;
	}
}

int VelocitySmoothing::computeDirection()
{
	// Compute the velocity at which the trajectory will be
	// when the acceleration will be zero
	float vel_zero_acc = computeVelAtZeroAcc();

	/* Depending of the direction, start accelerating positively or negatively */
	int direction = sign(_vel_sp - vel_zero_acc);

	if (direction == 0) {
		// If by braking immediately the velocity is exactly
		// the require one with zero acceleration, then brake
		direction = sign(_state.a);
	}

	return direction;
}

float VelocitySmoothing::computeVelAtZeroAcc()
{
	float vel_zero_acc = _state.v;

	if (fabsf(_state.a) > FLT_EPSILON) {
		float j_zero_acc = -sign(_state.a) * _max_jerk; // Required jerk to reduce the acceleration
		float t_zero_acc = -_state.a / j_zero_acc; // Required time to cancel the current acceleration
		vel_zero_acc = _state.v + _state.a * t_zero_acc + 0.5f * j_zero_acc * t_zero_acc * t_zero_acc;
	}

	return vel_zero_acc;
}

void VelocitySmoothing::updateDurationsMinimizeTotalTime()
{
	float jerk_max_T1 = _direction * _max_jerk;
	float delta_v = _vel_sp - _state.v;

	// compute increasing acceleration time
	_T1 = computeT1(_state.a, delta_v, jerk_max_T1, _max_accel);

	// compute decreasing acceleration time
	_T3 = computeT3(_T1, _state.a, jerk_max_T1);

	// compute constant acceleration time
	_T2 = computeT2(_T1, _T3, _state.a, delta_v, jerk_max_T1);
}

Trajectory VelocitySmoothing::evaluatePoly(float j, float a0, float v0, float x0, float t, int d)
{
	Trajectory traj;
	float jt = d * j;
	float t2 = t * t;
	float t3 = t2 * t;

	traj.j = jt;
	traj.a = a0 + jt * t;
	traj.v = v0 + a0 * t + 0.5f * jt * t2;
	traj.x = x0 + v0 * t + 0.5f * a0 * t2 + 1.f / 6.f * jt * t3;

	return traj;
}

void VelocitySmoothing::updateTraj(float dt, float time_stretch)
{
	_local_time += dt * time_stretch;
	float t_remain = _local_time;

	float t1 = math::min(t_remain, _T1);
	_state = evaluatePoly(_max_jerk, _state_init.a, _state_init.v, _state_init.x, t1, _direction);
	t_remain -= t1;

	if (t_remain > 0.f) {
		float t2 = math::min(t_remain, _T2);
		_state = evaluatePoly(0.f, _state.a, _state.v, _state.x, t2, 0.f);
		t_remain -= t2;
	}

	if (t_remain > 0.f) {
		float t3 = math::min(t_remain, _T3);
		_state = evaluatePoly(_max_jerk, _state.a, _state.v, _state.x, t3, -_direction);
		t_remain -= t3;
	}

	if (t_remain > 0.f) {
		_state = evaluatePoly(0.f, 0.f, _state.v, _state.x, t_remain, 0.f);
	}
}

void VelocitySmoothing::timeSynchronization(VelocitySmoothing *traj, int n_traj)
{
	float desired_time = 0.f;
	int longest_traj_index = 0;

	for (int i = 0; i < n_traj; i++) {
		const float T123 = traj[i].getTotalTime();

		if (T123 > desired_time) {
			desired_time = T123;
			longest_traj_index = i;
		}
	}

	if (desired_time > FLT_EPSILON) {
		for (int i = 0; i < n_traj; i++) {
			if (i != longest_traj_index) {
				traj[i].updateDurationsGivenTotalTime(desired_time);
			}
		}
	}
}

void VelocitySmoothing::updateDurationsGivenTotalTime(float T123)
{
	float jerk_max_T1 = _direction * _max_jerk;
	float delta_v = _vel_sp - _state.v;

	// compute increasing acceleration time
	_T1 = computeT1(T123, _state.a, delta_v, jerk_max_T1, _max_accel);

	// compute decreasing acceleration time
	_T3 = computeT3(_T1, _state.a, jerk_max_T1);

	// compute constant acceleration time
	_T2 = computeT2(T123, _T1, _T3);
}
