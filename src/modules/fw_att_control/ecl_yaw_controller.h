/****************************************************************************
 *
 *   Copyright (c) 2013-2016 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file ecl_yaw_controller.h
 * Definition of a simple orthogonal coordinated turn yaw PID controller.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 *
 * Acknowledgements:
 *
 *   The control design is based on a design
 *   by Paul Riseborough and Andrew Tridgell, 2013,
 *   which in turn is based on initial work of
 *   Jonathan Challinger, 2012.
 */
#ifndef ECL_YAW_CONTROLLER_H
#define ECL_YAW_CONTROLLER_H

#include "ecl_controller.h"

class ECL_YawController :
	public ECL_Controller
{
public:
	ECL_YawController() = default;
	~ECL_YawController() = default;

	float control_attitude(const struct ECL_ControlData &ctl_data) override;
	float control_euler_rate(const struct ECL_ControlData &ctl_data) override;
	float control_bodyrate(const struct ECL_ControlData &ctl_data) override;

	/* Additional setters */
	void set_coordinated_min_speed(float coordinated_min_speed)
	{
		_coordinated_min_speed = coordinated_min_speed;
	}

	void set_coordinated_method(int32_t coordinated_method)
	{
		_coordinated_method = coordinated_method;
	}

	enum {
		COORD_METHOD_OPEN = 0,
		COORD_METHOD_CLOSEACC = 1
	};

protected:
	float _coordinated_min_speed{1.0f};
	float _max_rate{0.0f};

	int32_t _coordinated_method{COORD_METHOD_OPEN};

	float control_attitude_impl_openloop(const struct ECL_ControlData &ctl_data);

	float control_attitude_impl_accclosedloop(const struct ECL_ControlData &ctl_data);

};

#endif // ECL_YAW_CONTROLLER_H
