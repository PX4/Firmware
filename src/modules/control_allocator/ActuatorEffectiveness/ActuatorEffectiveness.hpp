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
 * @file ActuatorEffectiveness.hpp
 *
 * Interface for Actuator Effectiveness
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#pragma once

#include <ControlAllocation/ControlAllocation.hpp>

#include <matrix/matrix/math.hpp>
#include <uORB/topics/vehicle_actuator_setpoint.h>

class ActuatorEffectiveness
{
public:
	ActuatorEffectiveness() = default;
	virtual ~ActuatorEffectiveness() = default;

	static constexpr uint8_t NUM_ACTUATORS = ControlAllocation::NUM_ACTUATORS;
	static constexpr uint8_t NUM_AXES = ControlAllocation::NUM_AXES;

	enum class FlightPhase {
		HOVER_FLIGHT = 0,
		FORWARD_FLIGHT = 1,
		TRANSITION_HF_TO_FF = 2,
		TRANSITION_FF_TO_HF = 3
	};

	/**
	 * Set the current flight phase
	 *
	 * @param Flight phase
	 */
	virtual void setFlightPhase(const FlightPhase &flight_phase)
	{
		_flight_phase = flight_phase;
	}

	/**
	 * Get the control effectiveness matrix if updated
	 *
	 * @return true if updated and matrix is set
	 */
	virtual bool getEffectivenessMatrix(matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &matrix) = 0;

	/**
	 * Get the actuator trims
	 *
	 * @return Actuator trims
	 */
	const matrix::Vector<float, NUM_ACTUATORS> &getActuatorTrim() const
	{
		return _trim;
	}

	/**
	 * Get the current flight phase
	 *
	 * @return Flight phase
	 */
	const FlightPhase &getFlightPhase() const
	{
		return _flight_phase;
	}

	/**
	 * Get the number of actuators
	 */
	virtual int numActuators() const = 0;

protected:
	matrix::Vector<float, NUM_ACTUATORS> _trim;			///< Actuator trim
	FlightPhase _flight_phase{FlightPhase::HOVER_FLIGHT};		///< Current flight phase
};
