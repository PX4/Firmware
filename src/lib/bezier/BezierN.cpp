/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file BezierN.cpp
 * Bezier function
 *
 * @author Julian Kent <julian@auterion.com>
 */

#include <bezier/BezierN.hpp>
#include <matrix/Dual.hpp>

namespace
{

/*
 * Generic in-place bezier implementation. Leaves result in first element.
 *
 */
template <typename Scalar, size_t D>
void calculateBezier(matrix::Vector<Scalar, D> *positions, int N, Scalar t, Scalar one_minus_t)
{
	for (int bezier_order = 1; bezier_order < N; bezier_order++) {
		for (int i = 0; i < N - bezier_order; i++) {
			positions[i] = positions[i] * one_minus_t + positions[i + 1] * t;
		}
	}
}
}

namespace bezier
{

bool calculateBezierPosVel(const matrix::Vector3f *positions, int N, float t,
			   matrix::Vector3f &position, matrix::Vector3f &velocity)
{
	if (positions == nullptr || N == 0 || t < 0 || t > 1) {
		return false;
	}

	using Df = matrix::Dual<float, 1>;
	using Vector3Df = matrix::Vector3<Df>;

	Vector3Df intermediates[N];

	for (int i = 0; i < N; i++) {
		for (int j = 0; j < 3; j++) {
			intermediates[i](j) = positions[i](j);
		}
	}

	Df dual_t(t, 0); // derivative with respect to time
	calculateBezier(intermediates, N, dual_t, Df(1) - dual_t);

	position = matrix::collectReals(intermediates[0]);
	velocity = matrix::collectDerivatives(intermediates[0]);

	return true;
}

bool calculateBezierPosVelAcc(const matrix::Vector3f *positions, int N, float t,
			      matrix::Vector3f &position, matrix::Vector3f &velocity, matrix::Vector3f &acceleration)
{
	if (positions == nullptr || N == 0 || t < 0 || t > 1) {
		return false;
	}

	using Df = matrix::Dual<float, 1>;
	using DDf = matrix::Dual<Df, 1>;
	using Vector3DDf = matrix::Vector3<DDf>;

	Vector3DDf intermediates[N];

	for (int i = 0; i < N; i++) {
		for (int j = 0; j < 3; j++) {
			intermediates[i](j) = Df(positions[i](j));
		}
	}

	DDf dual_t(Df(t, 0), 0); // 1st and 2nd derivative with respect to time
	calculateBezier(intermediates, N, dual_t, Df(1) - dual_t);

	position = matrix::collectReals(matrix::collectReals(intermediates[0]));
	velocity = matrix::collectReals(matrix::collectDerivatives(intermediates[0]));
	acceleration = matrix::collectDerivatives(matrix::collectDerivatives(intermediates[0]));

	return true;
}

bool calculateBezierYaw(const float *setpoints, int N, float t, float &yaw_setpoint, float &yaw_vel_setpoint)
{
	if (setpoints == nullptr || N == 0 || t < 0 || t > 1) {
		return false;
	}


	using Df = matrix::Dual<float, 1>;
	using Vector1Df = matrix::Vector<Df, 1>;

	Vector1Df intermediates[N];

	// all yaw setpoints are wrapped relative to the starting yaw
	const float offset = setpoints[0];

	for (int i = 0; i < N; i++) {
		intermediates[i](0) = matrix::wrap_pi(setpoints[i] - offset);
	}

	Df dual_t (t, 0); // derivative with respect to time
	calculateBezier(intermediates, N, dual_t, Df(1) - dual_t);

	Df result = intermediates[0](0);
	yaw_setpoint = matrix::wrap_pi(result.value + offset);
	yaw_vel_setpoint = result.derivative(0);

	return true;
}

bool calculateT(int64_t start_time, int64_t end_time, int64_t now, float &T)
{
	if (now < start_time || end_time < now) {
		return false;
	}

	int64_t total_duration = end_time - start_time;
	int64_t elapsed_duration = now - start_time;

	T = (float) elapsed_duration / (float) total_duration;

	return true;
}

}
