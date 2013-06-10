/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Tobias Naegeli <naegelit@student.ethz.ch>
 *           Lorenz Meier <lm@inf.ethz.ch>
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

/*
 * @file multirotor_pos_control_flow_params.h
 * 
 * Parameters for position controller
 */

#include <systemlib/param/param.h>

struct multirotor_position_control_flow_params {
	float pos_p;
	float pos_d;
	float height_p;
	float height_i;
	float height_d;
	float height_sp;
	float thrust_feedforward;
	float limit_pitch;
	float limit_roll;
	float limit_thrust_int;
	float limit_thrust_lower;
	float trim_roll;
	float trim_pitch;
};

struct multirotor_position_control_flow_param_handles {
	param_t pos_p;
	param_t pos_d;
	param_t height_p;
	param_t height_i;
	param_t height_d;
	param_t height_sp;
	param_t thrust_feedforward;
	param_t limit_pitch;
	param_t limit_roll;
	param_t limit_thrust_int;
	param_t limit_thrust_lower;
	param_t trim_roll;
	param_t trim_pitch;
};

/**
 * Initialize all parameter handles and values
 *
 */
int parameters_init(struct multirotor_position_control_flow_param_handles *h);

/**
 * Update all parameters
 *
 */
int parameters_update(const struct multirotor_position_control_flow_param_handles *h, struct multirotor_position_control_flow_params *p);
