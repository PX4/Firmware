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

#pragma once

#include <mavsdk/mavsdk.h>
#include <mavsdk/geometry.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include "catch2/catch.hpp"
#include <chrono>
#include <memory>
#include <thread>

extern std::string connection_url;

using namespace mavsdk;
using namespace mavsdk::geometry;

class AutopilotTester
{
public:
	struct MissionOptions {
		double leg_length_m {20.0};
		double relative_altitude_m {10.0};
		bool rtl_at_end {false};
		bool fly_through {false};
	};

	void connect(const std::string uri);
	void wait_until_ready();
	void wait_until_ready_local_position_only();
	void store_home();
	void check_home_within(float acceptance_radius_m);
	void set_takeoff_altitude(const float altitude_m);
	void arm();
	void takeoff();
	void land();
	void transition_to_fixedwing();
	void transition_to_multicopter();
	void wait_until_disarmed(std::chrono::seconds timeout_duration = std::chrono::seconds(90));
	void wait_until_hovering();
	void prepare_square_mission(MissionOptions mission_options);
	void prepare_straight_mission(MissionOptions mission_options);
	void execute_mission();
	void execute_rtl();
	void offboard_goto(const Offboard::PositionNedYaw &target, float acceptance_radius_m = 0.3f,
			   std::chrono::seconds timeout_duration = std::chrono::seconds(60));
	void offboard_land();
	void request_ground_truth();
	void check_mission_item_speed_above(int item_index, float min_speed_m_s);
	void check_tracks_mission(float corridor_radius_m = 1.0f);


private:
	mavsdk::geometry::CoordinateTransformation get_coordinate_transformation();
	mavsdk::Mission::MissionItem create_mission_item(
		const mavsdk::geometry::CoordinateTransformation::LocalCoordinate &local_coordinate,
		const MissionOptions &mission_options,
		const mavsdk::geometry::CoordinateTransformation &ct);

	bool ground_truth_horizontal_position_close_to(const Telemetry::GroundTruth &target_pos, float acceptance_radius_m);
	bool estimated_position_close_to(const Offboard::PositionNedYaw &target_pos, float acceptance_radius_m);
	bool estimated_horizontal_position_close_to(const Offboard::PositionNedYaw &target_pos, float acceptance_radius_m);

	mavsdk::Mavsdk _mavsdk{};
	std::unique_ptr<mavsdk::Telemetry> _telemetry{};
	std::unique_ptr<mavsdk::Action> _action{};
	std::unique_ptr<mavsdk::Mission> _mission{};
	std::unique_ptr<mavsdk::Offboard> _offboard{};

	Telemetry::GroundTruth _home{NAN, NAN, NAN};
};

template<typename Rep, typename Period>
bool poll_condition_with_timeout(
	std::function<bool()> fun, std::chrono::duration<Rep, Period> duration)
{
	// We need millisecond resolution for sleeping.
	const std::chrono::milliseconds duration_ms(duration);

	unsigned iteration = 0;

	while (!fun()) {
		std::this_thread::sleep_for(duration_ms / 100);

		if (iteration++ >= 100) {
			return false;
		}
	}

	return true;
}

inline float sq(float x) { return x * x; };
