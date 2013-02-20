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
 * @file multirotor_pos_control_flow_params.c
 * 
 * Parameters for EKF filter
 */

#include "multirotor_pos_control_flow_params.h"

/* Extended Kalman Filter covariances */

/* controller parameters */
PARAM_DEFINE_FLOAT(MFPC_POS_P, 0.1f);
PARAM_DEFINE_FLOAT(MFPC_POS_D, 0.05f);
PARAM_DEFINE_FLOAT(MFPC_H_P, 0.06f);
PARAM_DEFINE_FLOAT(MFPC_H_I, 0.00001f);
PARAM_DEFINE_FLOAT(MFPC_H_D, 0.01f);
PARAM_DEFINE_FLOAT(MFPC_H_SP, -1.0f);
PARAM_DEFINE_FLOAT(MFPC_T_FFWD, 0.66f);

int parameters_init(struct multirotor_position_control_flow_param_handles *h)
{
	/* PID parameters */
	h->pos_p	 	=	param_find("MFPC_POS_P");
	h->pos_d 		=	param_find("MFPC_POS_D");
	h->height_p 	=	param_find("MFPC_H_P");
	h->height_i 	=	param_find("MFPC_H_I");
	h->height_d 	=	param_find("MFPC_H_D");
	h->height_sp 	=	param_find("MFPC_H_SP");
	h->thrust_feedforward=	param_find("MFPC_T_FFWD");

	return OK;
}

int parameters_update(const struct multirotor_position_control_flow_param_handles *h, struct multirotor_position_control_flow_params *p)
{
	param_get(h->pos_p, &(p->pos_p));
	param_get(h->pos_d, &(p->pos_d));
	param_get(h->height_p, &(p->height_p));
	param_get(h->height_i, &(p->height_i));
	param_get(h->height_d, &(p->height_d));
	param_get(h->height_sp, &(p->height_sp));
	param_get(h->thrust_feedforward, &(p->thrust_feedforward));

	return OK;
}
