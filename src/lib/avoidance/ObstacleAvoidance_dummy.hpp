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
 * @file ObstacleAvoidance_dummy.hpp
 * This is a dummy class to reduce flash space for when obstacle avoidance is not required
 *
 * @author Julian Kent
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <commander/px4_custom_mode.h>


#include <matrix/matrix/math.hpp>


class ObstacleAvoidance
{
public:
	ObstacleAvoidance(void *) {} // takes void* argument to be compatible with ModuleParams constructor


	void injectAvoidanceSetpoints(matrix::Vector3f &pos_sp, matrix::Vector3f &vel_sp, float &yaw_sp,
				      float &yaw_speed_sp)
	{
		notify_dummy();
	};

	void updateAvoidanceDesiredWaypoints(const matrix::Vector3f &curr_wp, const float curr_yaw,
					     const float curr_yawspeed,
					     const matrix::Vector3f &next_wp, const float next_yaw, const float next_yawspeed, const bool ext_yaw_active,
					     const int wp_type)
	{
		notify_dummy();
	};

	void updateAvoidanceDesiredSetpoints(const matrix::Vector3f &pos_sp, const matrix::Vector3f &vel_sp,
					     const int type)
	{
		notify_dummy();
	}


	void checkAvoidanceProgress(const matrix::Vector3f &pos, const matrix::Vector3f &prev_wp,
				    float target_acceptance_radius, const matrix::Vector2f &closest_pt)
	{
		notify_dummy();
	};

protected:

	bool _logged_error = false;
	void notify_dummy()
	{
		if (!_logged_error) {
			PX4_ERR("Dummy avoidance library called!");
			_logged_error = true;
		}
	}
};
