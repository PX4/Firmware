/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
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
 * @file rotation.cpp
 *
 * Vector rotation library
 */

#include <px4_platform_common/defines.h>
#include "math.h"
#include "rotation.h"

__EXPORT matrix::Dcmf
get_rot_matrix(enum Rotation rot)
{
	return matrix::Dcmf{matrix::Eulerf{
			math::radians((float)rot_lookup[rot].roll),
			math::radians((float)rot_lookup[rot].pitch),
			math::radians((float)rot_lookup[rot].yaw)}};
}

__EXPORT matrix::Quatf
get_rot_quaternion(enum Rotation rot)
{
	return matrix::Quatf{matrix::Eulerf{
			math::radians((float)rot_lookup[rot].roll),
			math::radians((float)rot_lookup[rot].pitch),
			math::radians((float)rot_lookup[rot].yaw)}};
}

__EXPORT void
rotate_3f(enum Rotation rot, float &x, float &y, float &z)
{
	float tmp;

	switch (rot) {
	case ROTATION_NONE:
	case ROTATION_MAX:
		return;

	case ROTATION_YAW_45: {
			tmp = M_SQRT1_2_F * (x - y);
			y   = M_SQRT1_2_F * (x + y);
			x = tmp;
			return;
		}

	case ROTATION_YAW_90: {
			tmp = x; x = -y; y = tmp;
			return;
		}

	case ROTATION_YAW_135: {
			tmp = -M_SQRT1_2_F * (x + y);
			y   =  M_SQRT1_2_F * (x - y);
			x = tmp;
			return;
		}

	case ROTATION_YAW_180:
		x = -x; y = -y;
		return;

	case ROTATION_YAW_225: {
			tmp = M_SQRT1_2_F * (y - x);
			y   = -M_SQRT1_2_F * (x + y);
			x = tmp;
			return;
		}

	case ROTATION_YAW_270: {
			tmp = x; x = y; y = -tmp;
			return;
		}

	case ROTATION_YAW_315: {
			tmp = M_SQRT1_2_F * (x + y);
			y   = M_SQRT1_2_F * (y - x);
			x = tmp;
			return;
		}

	case ROTATION_ROLL_180: {
			y = -y; z = -z;
			return;
		}

	case ROTATION_ROLL_180_YAW_45: {
			tmp = M_SQRT1_2_F * (x + y);
			y   = M_SQRT1_2_F * (x - y);
			x = tmp; z = -z;
			return;
		}

	case ROTATION_ROLL_180_YAW_90:

	// FALLTHROUGH
	case ROTATION_PITCH_180_YAW_270: {
			tmp = x;
			x = y;
			y = tmp;
			z = -z;
			return;
		}

	case ROTATION_ROLL_180_YAW_135: {
			tmp = M_SQRT1_2_F * (y - x);
			y   = M_SQRT1_2_F * (y + x);
			x = tmp; z = -z;
			return;
		}

	case ROTATION_PITCH_180: {
			x = -x; z = -z;
			return;
		}

	case ROTATION_ROLL_180_YAW_225: {
			tmp = -M_SQRT1_2_F * (x + y);
			y   =  M_SQRT1_2_F * (y - x);
			x = tmp; z = -z;
			return;
		}

	case ROTATION_ROLL_180_YAW_270:

	// FALLTHROUGH
	case ROTATION_PITCH_180_YAW_90: {
			tmp = x;
			x = -y;
			y = -tmp;
			z = -z;
			return;
		}

	case ROTATION_ROLL_180_YAW_315: {
			tmp =  M_SQRT1_2_F * (x - y);
			y   = -M_SQRT1_2_F * (x + y);
			x = tmp; z = -z;
			return;
		}

	case ROTATION_ROLL_90: {
			tmp = z; z = y; y = -tmp;
			return;
		}

	case ROTATION_ROLL_90_YAW_45: {
			tmp = z; z = y; y = -tmp;
			tmp = M_SQRT1_2_F * (x - y);
			y   = M_SQRT1_2_F * (x + y);
			x = tmp;
			return;
		}

	case ROTATION_ROLL_90_YAW_90: {
			tmp = x;
			x = z;
			z = y;
			y = tmp;
			return;
		}

	case ROTATION_ROLL_90_YAW_135: {
			tmp = z; z = y; y = -tmp;
			tmp = -M_SQRT1_2_F * (x + y);
			y   =  M_SQRT1_2_F * (x - y);
			x = tmp;
			return;
		}

	case ROTATION_ROLL_270: {
			tmp = z; z = -y; y = tmp;
			return;
		}

	case ROTATION_ROLL_270_YAW_45: {
			tmp = z; z = -y; y = tmp;
			tmp = M_SQRT1_2_F * (x - y);
			y   = M_SQRT1_2_F * (x + y);
			x = tmp;
			return;
		}

	case ROTATION_ROLL_270_YAW_90: {
			tmp = x;
			x = -z;
			z = -y;
			y = tmp;
			return;
		}

	case ROTATION_ROLL_270_YAW_135: {
			tmp = z; z = -y; y = tmp;
			tmp = -M_SQRT1_2_F * (x + y);
			y   =  M_SQRT1_2_F * (x - y);
			x = tmp;
			return;
		}

	case ROTATION_PITCH_90: {
			tmp = z; z = -x; x = tmp;
			return;
		}

	case ROTATION_PITCH_270: {
			tmp = z; z = x; x = -tmp;
			return;
		}

	case ROTATION_ROLL_180_PITCH_270: {
			tmp = z; z = x; x = tmp;
			y = -y;
			return;
		}

	case ROTATION_PITCH_9_YAW_180: {
			const float tmpx = x;
			const float tmpy = y;
			const float tmpz = z;
			x = -0.987688f * tmpx +  0.000000f * tmpy + -0.156434f * tmpz;
			y =  0.000000f * tmpx + -1.000000f * tmpy +  0.000000f * tmpz;
			z = -0.156434f * tmpx +  0.000000f * tmpy +  0.987688f * tmpz;
			return;
		}

	case ROTATION_PITCH_45: {
			tmp = M_SQRT1_2_F * x + M_SQRT1_2_F * z;
			z = M_SQRT1_2_F * z - M_SQRT1_2_F * x;
			x = tmp;
			return;
		}

	case ROTATION_PITCH_315: {
			tmp = M_SQRT1_2_F * x - M_SQRT1_2_F * z;
			z = M_SQRT1_2_F * z + M_SQRT1_2_F * x;
			x = tmp;
			return;
		}

	case ROTATION_ROLL_90_YAW_270: {
			tmp = x;
			x = -z;
			z = y;
			y = -tmp;
			return;
		}

	case ROTATION_ROLL_90_PITCH_90: {
			tmp = x;
			x = y;
			y = -z;
			z = -tmp;
			return;
		}

	case ROTATION_ROLL_180_PITCH_90:

	// FALLTHROUGH
	case ROTATION_PITCH_90_YAW_180: {
			tmp = x;
			x = -z;
			y = -y;
			z = -tmp;
			return;
		}

	case ROTATION_ROLL_270_PITCH_90: {
			tmp = x;
			x = -y;
			y = z;
			z = -tmp;
			return;
		}

	case ROTATION_ROLL_90_PITCH_180:

	// FALLTHROUGH
	case ROTATION_ROLL_270_YAW_180: {
			tmp = y;
			x = -x;
			y = -z;
			z = -tmp;
			return;
		}

	case ROTATION_ROLL_270_PITCH_180: {
			tmp = y;
			x = -x;
			y = z;
			z = tmp;
			return;
		}

	case ROTATION_ROLL_90_PITCH_270: {
			tmp = x;
			x = -y;
			y = -z;
			z = tmp;
			return;
		}

	case ROTATION_ROLL_270_PITCH_270: {
			tmp = x;
			x = y;
			y = z;
			z = tmp;
			return;
		}

	case ROTATION_ROLL_90_PITCH_180_YAW_90:

	// FALLTHROUGH
	case ROTATION_ROLL_270_YAW_270: {
			tmp = x;
			x = z;
			z = -y;
			y = -tmp;
			return;
		}

	case ROTATION_ROLL_90_PITCH_68_YAW_293: {
			const float tmpx = x;
			const float tmpy = y;
			const float tmpz = z;
			x = 0.146371f * tmpx + 0.362280f * tmpy - 0.920505f * tmpz;
			y = -0.344827f * tmpx - 0.853477f * tmpy - 0.390731f * tmpz;
			z = -0.927184f * tmpx + 0.374607f * tmpy;
			return;
		}

	case ROTATION_ROLL_90_PITCH_315: {
			const float tmpx = x;
			const float tmpy = y;
			const float tmpz = z;
			x = 0.707107f * tmpx - 0.707107f * tmpy;
			y = -tmpz;
			z = 0.707107f * tmpx + 0.707107f * tmpy;
			return;
		}

	}
}
