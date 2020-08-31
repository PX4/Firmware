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

/**
 * @file StraightLine.hpp
 *
 * lib to return setpoints on a straight line
 *
 * @author Christoph Tobler <christoph@px4.io>
 */

#pragma once

#include <matrix/matrix/math.hpp>

class StraightLine
{
public:
	StraightLine(const matrix::Vector3f &pos) : _position(pos) {};
	~StraightLine() = default;

	// setter functions
	void setLineFromTo(const matrix::Vector3f &start, const matrix::Vector3f &end);
	void setSpeed(const float &speed) { _speed = speed; };

	/**
	 * Generate setpoints on a straight line according to parameters
	 *
	 * @param position_setpoint: reference to the 3D vector with the position setpoint to update
	 * @param velocity_setpoint: reference to the 3D vector with the velocity setpoint to update
	 */
	void generateSetpoints(matrix::Vector3f &position_setpoint, matrix::Vector3f &velocity_setpoint);

	/**
	 * Check if the end was reached
	 *
	 * @return false when on the way from start to end, true when end was reached
	 */
	bool isEndReached() const { return _end_reached; }

	void reset() { _end_reached = true; }

private:
	const matrix::Vector3f &_position; /**< vehicle position (dependency injection) */

	matrix::Vector3f _start; /**< Start point of the straight line */
	matrix::Vector3f _end; /**< End point of the straight line */
	float _speed = 1.f; /**< desired speed between accelerating and decelerating */

	bool _end_reached = true; /**< Flag to lock further movement when end is reached */
};
