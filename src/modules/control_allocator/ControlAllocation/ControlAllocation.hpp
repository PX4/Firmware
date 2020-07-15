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
 * @file ControlAllocation.hpp
 *
 * Interface for Control Allocation Algorithms
 *
 * Implementers of this interface are expected to update the members
 * of this base class in the `allocate` method.
 *
 * Example usage:
 * ```
 * [...]
 * // Initialization
 * ControlAllocationMethodImpl alloc();
 * alloc.setEffectivenessMatrix(effectiveness, actuator_trim);
 * alloc.setActuatorMin(actuator_min);
 * alloc.setActuatorMin(actuator_max);
 *
 * while (1) {
 * 	[...]
 *
 * 	// Set control setpoint, allocate actuator setpoint, retrieve actuator setpoint
 * 	alloc.setControlSetpoint(control_sp);
 * 	alloc.allocate();
 * 	actuator_sp = alloc.getActuatorSetpoint();
 *
 * 	// Check if the control setpoint was fully allocated
 *	unallocated_control = control_sp - alloc.getAllocatedControl()
 *
 *	[...]
 * }
 * ```
 *
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <uORB/topics/vehicle_actuator_setpoint.h>

class ControlAllocation
{
public:
	ControlAllocation() = default;
	virtual ~ControlAllocation() = default;

	static constexpr uint8_t NUM_ACTUATORS = 16;
	static constexpr uint8_t NUM_AXES = 6;

	typedef matrix::Vector<float, NUM_ACTUATORS> ActuatorVector;

	enum ControlAxis {
		ROLL = 0,
		PITCH,
		YAW,
		THRUST_X,
		THRUST_Y,
		THRUST_Z
	};

	/**
	 * Allocate control setpoint to actuators
	 *
	 * @param control_setpoint  Desired control setpoint vector (input)
	 */
	virtual void allocate() = 0;

	/**
	 * Set the control effectiveness matrix
	 *
	 * @param B Effectiveness matrix
	 */
	virtual void setEffectivenessMatrix(const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &effectiveness,
					    const matrix::Vector<float, NUM_ACTUATORS> &actuator_trim);

	/**
	 * Get the allocated actuator vector
	 *
	 * @return Actuator vector
	 */
	const matrix::Vector<float, NUM_ACTUATORS> &getActuatorSetpoint() const;

	/**
	 * Set the desired control vector
	 *
	 * @param Control vector
	 */
	void setControlSetpoint(const matrix::Vector<float, NUM_AXES> &control);

	/**
	 * Set the desired control vector
	 *
	 * @param Control vector
	 */
	const matrix::Vector<float, NUM_AXES> &getControlSetpoint() const;

	/**
	 * Get the allocated control vector
	 *
	 * @return Control vector
	 */
	const matrix::Vector<float, NUM_AXES> &getAllocatedControl() const;

	/**
	 * Get the control effectiveness matrix
	 *
	 * @return Effectiveness matrix
	 */
	const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &getEffectivenessMatrix() const;

	/**
	 * Set the minimum actuator values
	 *
	 * @param actuator_min Minimum actuator values
	 */
	void setActuatorMin(const matrix::Vector<float, NUM_ACTUATORS> &actuator_min);

	/**
	 * Get the minimum actuator values
	 *
	 * @return Minimum actuator values
	 */
	const matrix::Vector<float, NUM_ACTUATORS> &getActuatorMin() const;

	/**
	 * Set the maximum actuator values
	 *
	 * @param actuator_max Maximum actuator values
	 */
	void setActuatorMax(const matrix::Vector<float, NUM_ACTUATORS> &actuator_max);

	/**
	 * Get the maximum actuator values
	 *
	 * @return Maximum actuator values
	 */
	const matrix::Vector<float, NUM_ACTUATORS> &getActuatorMax() const;

	/**
	 * Set the current actuator setpoint.
	 *
	 * Use this when a new allocation method is started to initialize it properly.
	 * In most cases, it is not needed to call this method before `allocate()`.
	 * Indeed the previous actuator setpoint is expected to be stored during calls to `allocate()`.
	 *
	 * @param actuator_sp Actuator setpoint
	 */
	void setActuatorSetpoint(const matrix::Vector<float, NUM_ACTUATORS> &actuator_sp);

	/**
	 * Clip the actuator setpoint between minimum and maximum values.
	 *
	 * The output is in the range [min; max]
	 *
	 * @param actuator Actuator vector to clip
	 *
	 * @return Clipped actuator setpoint
	 */
	matrix::Vector<float, NUM_ACTUATORS> clipActuatorSetpoint(const matrix::Vector<float, NUM_ACTUATORS> &actuator) const;

	/**
	 * Normalize the actuator setpoint between minimum and maximum values.
	 *
	 * The output is in the range [-1; +1]
	 *
	 * @param actuator Actuator vector to normalize
	 *
	 * @return Clipped actuator setpoint
	 */
	matrix::Vector<float, NUM_ACTUATORS> normalizeActuatorSetpoint(const matrix::Vector<float, NUM_ACTUATORS> &actuator)
	const;

protected:
	matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> _effectiveness;  //< Effectiveness matrix
	matrix::Vector<float, NUM_ACTUATORS> _actuator_trim; 	//< Neutral actuator values
	matrix::Vector<float, NUM_ACTUATORS> _actuator_min; 	//< Minimum actuator values
	matrix::Vector<float, NUM_ACTUATORS> _actuator_max; 	//< Maximum actuator values
	matrix::Vector<float, NUM_ACTUATORS> _actuator_sp;  	//< Actuator setpoint
	matrix::Vector<float, NUM_AXES> _control_sp;   		//< Control setpoint
	matrix::Vector<float, NUM_AXES> _control_allocated;  	//< Allocated control
	matrix::Vector<float, NUM_AXES> _control_trim;  	//< Control at trim actuator values
};
