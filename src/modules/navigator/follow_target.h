/***************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file followme.cpp
 *
 * Helper class to track and follow a given position
 *
 * @author Jimmy Johnson <catch22@fastmail.net>
 */

#pragma once

#include "navigator_mode.h"
#include "mission_block.h"

#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/follow_target.h>

class FollowTarget : public MissionBlock, public ModuleParams
{

public:
	FollowTarget(Navigator *navigator, NavigatorCore &navigator_core);
	~FollowTarget() = default;

	void on_inactive() override;
	void on_activation() override;
	void on_active() override;

private:

	static constexpr int TARGET_TIMEOUT_MS = 2500;
	static constexpr int TARGET_ACCEPTANCE_RADIUS_M = 5;
	static constexpr int INTERPOLATION_PNTS = 20;
	static constexpr float FF_K = .25F;
	static constexpr float OFFSET_M = 8;

	enum FollowTargetState {
		TRACK_POSITION,
		TRACK_VELOCITY,
		SET_WAIT_FOR_TARGET_POSITION,
		WAIT_FOR_TARGET_POSITION
	};

	enum {
		FOLLOW_FROM_RIGHT,
		FOLLOW_FROM_BEHIND,
		FOLLOW_FROM_FRONT,
		FOLLOW_FROM_LEFT
	};

	static constexpr float _follow_position_matricies[4][9] = {
		{ 1.0F, -1.0F, 0.0F,  1.0F,  1.0F, 0.0F, 0.0F, 0.0F, 1.0F}, // follow right
		{-1.0F,  0.0F, 0.0F,  0.0F, -1.0F, 0.0F, 0.0F, 0.0F, 1.0F}, // follow behind
		{ 1.0F,  0.0F, 0.0F,  0.0F,  1.0F, 0.0F, 0.0F, 0.0F, 1.0F}, // follow front
		{ 1.0F,  1.0F, 0.0F, -1.0F,  1.0F, 0.0F, 0.0F, 0.0F, 1.0F}  // follow left side
	};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::NAV_MIN_FT_HT>) _param_nav_min_ft_ht,
		(ParamFloat<px4::params::NAV_FT_DST>) _param_nav_ft_dst,
		(ParamInt<px4::params::NAV_FT_FS>) _param_nav_ft_fs,
		(ParamFloat<px4::params::NAV_FT_RS>) _param_nav_ft_rs
	)

	FollowTargetState _follow_target_state{SET_WAIT_FOR_TARGET_POSITION};
	int _follow_target_position{FOLLOW_FROM_BEHIND};

	uORB::Subscription _follow_target_sub{ORB_ID(follow_target)};
	float _step_time_in_ms{0.0f};
	float _follow_offset{OFFSET_M};

	uint64_t _target_updates{0};
	uint64_t _last_update_time{0};

	matrix::Vector3f _current_vel;
	matrix::Vector3f _step_vel;
	matrix::Vector3f _est_target_vel;
	matrix::Vector3f _target_distance;
	matrix::Vector3f _target_position_offset;
	matrix::Vector3f _target_position_delta;
	matrix::Vector3f _filtered_target_position_delta;

	follow_target_s _current_target_motion{};
	follow_target_s _previous_target_motion{};

	float _yaw_rate{0.0f};
	float _responsiveness{0.0f};
	float _yaw_angle{0.0f};

	// Mavlink defined motion reporting capabilities
	enum {
		POS = 0,
		VEL = 1,
		ACCEL = 2,
		ATT_RATES = 3
	};

	matrix::Dcmf _rot_matrix;

	void track_target_position();
	void track_target_velocity();
	bool target_velocity_valid();
	bool target_position_valid();
	void reset_target_validity();
	void update_position_sp(bool velocity_valid, bool position_valid, float yaw_rate);
	void update_target_motion();
	void update_target_velocity();

	/**
	 * Set follow_target item
	 */
	void set_follow_target_item(struct mission_item_s *item, float min_clearance, follow_target_s &target, float yaw);
};
