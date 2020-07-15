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
 * @file ControlAllocationSequentialDesaturation.hpp
 *
 * Control Allocation Algorithm which sequentially modifies control demands in order to
 * eliminate the saturation of the actuator setpoint vector.
 *
 *
 * @author Roman Bapst <bapstroman@gmail.com>
 */

#pragma once

#include "ControlAllocationPseudoInverse.hpp"

class ControlAllocationSequentialDesaturation: public ControlAllocationPseudoInverse
{
public:

	ControlAllocationSequentialDesaturation() = default;
	virtual ~ControlAllocationSequentialDesaturation() = default;

	void allocate() override;

private:

	/**
	 * List of control axis used for desaturating the actuator vector. The desaturation logic will sequentially
	 * go through this list and if needed apply corrections to the demand of the corresponding axis in order to desaturate
	 * the actuator vector.
	 */
	ControlAxis _axis_prio_increasing [NUM_AXES] = {ControlAxis::YAW, ControlAxis::THRUST_Y, ControlAxis::THRUST_X, ControlAxis::THRUST_Z, ControlAxis::PITCH, ControlAxis::ROLL};

	/**
	 * Desaturate actuator setpoint vector.
	 *
	 * @return Desaturated actuator setpoint vector.
	 */
	void desaturateActuators(ActuatorVector &actuator_sp, const ControlAxis &axis);

	/**
	 * Get desaturation vector.
	 *
	 * @param 	axis 			Control axis
	 * @return 	ActuatorVector 	Column of the pseudo-inverse matrix corresponding to the given control axis.
	 */
	ActuatorVector getDesaturationVector(const ControlAxis &axis);

	/**
	 * Compute desaturation gain.
	 *
	 * @param 	desaturation_vector Column of the pseudo-inverse matrix corresponding to a given control axis.
	 * @param 	Actuator setpoint vector.
	 * @return 	Gain which eliminates the saturation of the highest saturated actuator.
	 */
	float computeDesaturationGain(const ActuatorVector &desaturation_vector, const ActuatorVector &actuator_sp);
};
