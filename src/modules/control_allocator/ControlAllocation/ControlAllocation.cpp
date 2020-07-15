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
 * @file ControlAllocation.cpp
 *
 * Interface for Control Allocation Algorithms
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include "ControlAllocation.hpp"

const matrix::Vector<float, ControlAllocation::NUM_ACTUATORS> &
ControlAllocation::getActuatorSetpoint() const
{
	return _actuator_sp;
}

void
ControlAllocation::setControlSetpoint(const matrix::Vector<float, ControlAllocation::NUM_AXES> &control)
{
	_control_sp = control;
}

const matrix::Vector<float, ControlAllocation::NUM_AXES> &
ControlAllocation::getControlSetpoint() const
{
	return _control_sp;
}

const matrix::Vector<float, ControlAllocation::NUM_AXES> &
ControlAllocation::getAllocatedControl() const
{
	return _control_allocated;
}

void
ControlAllocation::setEffectivenessMatrix(
	const matrix::Matrix<float, ControlAllocation::NUM_AXES, ControlAllocation::NUM_ACTUATORS> &effectiveness,
	const matrix::Vector<float, ControlAllocation::NUM_ACTUATORS> &actuator_trim)
{
	_effectiveness = effectiveness;
	_actuator_trim = clipActuatorSetpoint(actuator_trim);
	_control_trim = _effectiveness * _actuator_trim;
}

const matrix::Matrix<float, ControlAllocation::NUM_AXES, ControlAllocation::NUM_ACTUATORS> &
ControlAllocation::getEffectivenessMatrix() const
{
	return _effectiveness;
}

void
ControlAllocation::setActuatorMin(const matrix::Vector<float, ControlAllocation::NUM_ACTUATORS>
				  &actuator_min)
{
	_actuator_min = actuator_min;
}

const matrix::Vector<float, ControlAllocation::NUM_ACTUATORS> &
ControlAllocation::getActuatorMin() const
{
	return _actuator_min;
}

void
ControlAllocation::setActuatorMax(const matrix::Vector<float, ControlAllocation::NUM_ACTUATORS>
				  &actuator_max)
{
	_actuator_max = actuator_max;
}

const matrix::Vector<float, ControlAllocation::NUM_ACTUATORS> &
ControlAllocation::getActuatorMax() const
{
	return _actuator_max;
}

void
ControlAllocation::setActuatorSetpoint(
	const matrix::Vector<float, ControlAllocation::NUM_ACTUATORS> &actuator_sp)
{
	// Set actuator setpoint
	_actuator_sp = actuator_sp;

	// Clip
	_actuator_sp = clipActuatorSetpoint(_actuator_sp);

	// Compute achieved control
	_control_allocated = _effectiveness * _actuator_sp;

}

matrix::Vector<float, ControlAllocation::NUM_ACTUATORS>
ControlAllocation::clipActuatorSetpoint(const matrix::Vector<float, ControlAllocation::NUM_ACTUATORS> &actuator) const
{
	matrix::Vector<float, ControlAllocation::NUM_ACTUATORS> actuator_clipped;

	for (size_t i = 0; i < ControlAllocation::NUM_ACTUATORS; i++) {
		if (_actuator_max(i) < _actuator_min(i)) {
			actuator_clipped(i) = _actuator_trim(i);

		} else if (actuator_clipped(i) < _actuator_min(i)) {
			actuator_clipped(i) = _actuator_min(i);

		} else if (actuator_clipped(i) > _actuator_max(i)) {
			actuator_clipped(i) = _actuator_max(i);

		} else {
			actuator_clipped(i) = actuator(i);
		}
	}

	return actuator_clipped;
}

matrix::Vector<float, ControlAllocation::NUM_ACTUATORS>
ControlAllocation::normalizeActuatorSetpoint(const matrix::Vector<float, ControlAllocation::NUM_ACTUATORS> &actuator)
const
{
	matrix::Vector<float, ControlAllocation::NUM_ACTUATORS> actuator_normalized;

	for (size_t i = 0; i < ControlAllocation::NUM_ACTUATORS; i++) {
		if (_actuator_min(i) < _actuator_max(i)) {
			actuator_normalized(i) = -1.0f + 2.0f * (actuator(i) - _actuator_min(i)) / (_actuator_max(i) - _actuator_min(i));

		} else {
			actuator_normalized(i) = -1.0f + 2.0f * (_actuator_trim(i) - _actuator_min(i)) / (_actuator_max(i) - _actuator_min(i));
		}
	}

	return actuator_normalized;
}
