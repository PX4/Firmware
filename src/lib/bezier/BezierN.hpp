

/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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
 * @file BerzierN.hpp
 *
 * @author Julian Kent <julian@auterion.com>
 *
 * N-order Bezier library designed for time-aware trajectory tracking
 */

#pragma once
#include <matrix/math.hpp>

namespace bezier
{

/*
 * Calculates the location and velocity with respect to T on a given bezier curve of any order.
 *
 */
bool calculateBezierPosVel(const matrix::Vector3f *positions, int N, float t,
			   matrix::Vector3f &position, matrix::Vector3f &velocity);

/*
 * Calculates the position, velocity and acceleration with respect to T on a given bezier curve of any order.
 *
 */
bool calculateBezierPosVelAcc(const matrix::Vector3f *positions, int N, float t,
			      matrix::Vector3f &position, matrix::Vector3f &velocity, matrix::Vector3f &acceleration);

/*
 * Calculates the position and velocity of yaw with respect to t on a bezier curve.
 * All yaw setpoints are wrapped relative to the starting yaw.
 *
 */
bool calculateBezierYaw(const float *setpoints, int N, float t, float &yaw_setpoint, float &yaw_vel_setpoint);

/*
 * Calculates the fraction between the begin and end time which can be used for fast bezier curve lookups
 */
bool calculateT(int64_t start_time, int64_t end_time, int64_t now, float &T);

}
