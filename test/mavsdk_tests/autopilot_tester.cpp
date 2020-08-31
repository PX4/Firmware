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

#include "autopilot_tester.h"
#include <iostream>
#include <future>

std::string connection_url {"udp://"};

namespace
{
std::array<float, 3> get_local_mission_item(const Mission::MissionItem &item, const CoordinateTransformation &ct)
{
	using GlobalCoordinate = mavsdk::geometry::CoordinateTransformation::GlobalCoordinate;
	GlobalCoordinate global;
	global.latitude_deg = item.latitude_deg;
	global.longitude_deg = item.longitude_deg;
	auto local = ct.local_from_global(global);
	return {static_cast<float>(local.north_m), static_cast<float>(local.east_m), -item.relative_altitude_m};
}

float norm(const std::array<float, 3> &vec)
{
	return std::sqrt(sq(vec[0]) + sq(vec[1]) + sq(vec[2]));
}

float dot(const std::array<float, 3> &vec1, const std::array<float, 3> &vec2)
{
	return vec1[0] * vec2[0] + vec1[1] * vec2[1] + vec1[2] * vec2[2];
}

std::array<float, 3> diff(const std::array<float, 3> &vec1, const std::array<float, 3> &vec2)
{
	return {vec1[0] - vec2[0], vec1[1] - vec2[1], vec1[2] - vec2[2]};
}

std::array<float, 3> normalized(const std::array<float, 3> &vec)
{
	float n = norm(vec);

	if (n > 1e-6f) {
		return {vec[0] / n, vec[1] / n, vec[2] / n};

	} else {
		return {0, 0, 0};
	}
}

float point_to_line_distance(const std::array<float, 3> &point, const std::array<float, 3> &line_start,
			     const std::array<float, 3> &line_end)
{
	std::array<float, 3> norm_dir = normalized(diff(line_end, line_start));
	float t = dot(norm_dir, diff(point, line_start));

	// closest_on_line = line_start + t * norm_dir;
	std::array<float, 3> closest_on_line { line_start[0] + t *norm_dir[0], line_start[1] + t *norm_dir[1], line_start[2] + t *norm_dir[2]};

	return norm(diff(closest_on_line, point));
}

}

void AutopilotTester::connect(const std::string uri)
{
	ConnectionResult ret = _mavsdk.add_any_connection(uri);
	REQUIRE(ret == ConnectionResult::Success);

	std::cout << "Waiting for system connect" << std::endl;
	REQUIRE(poll_condition_with_timeout(
	[this]() { return _mavsdk.is_connected(); }, std::chrono::seconds(25)));

	auto &system = _mavsdk.system();

	_telemetry.reset(new Telemetry(system));
	_action.reset(new Action(system));
	_mission.reset(new Mission(system));
	_offboard.reset(new Offboard(system));
}

void AutopilotTester::wait_until_ready()
{
	std::cout << "Waiting for system to be ready" << std::endl;
	CHECK(poll_condition_with_timeout(
	[this]() { return _telemetry->health_all_ok(); }, std::chrono::seconds(20)));

	// FIXME: workaround to prevent race between PX4 switching to Hold mode
	// and us trying to arm and take off. If PX4 is not in Hold mode yet,
	// our arming presumably triggers a failsafe in manual mode.
	std::this_thread::sleep_for(std::chrono::seconds(1));
}

void AutopilotTester::wait_until_ready_local_position_only()
{
	std::cout << "Waiting for system to be ready" << std::endl;
	CHECK(poll_condition_with_timeout(
	[this]() {
		return
			(_telemetry->health().is_gyrometer_calibration_ok &&
			 _telemetry->health().is_accelerometer_calibration_ok &&
			 _telemetry->health().is_magnetometer_calibration_ok &&
			 _telemetry->health().is_level_calibration_ok &&
			 _telemetry->health().is_local_position_ok);
	}, std::chrono::seconds(20)));
}

void AutopilotTester::store_home()
{
	request_ground_truth();
	std::cout << "Waiting to get home position" << std::endl;
	CHECK(poll_condition_with_timeout(
	[this]() {
		_home = _telemetry->ground_truth();
		return std::isfinite(_home.latitude_deg) && std::isfinite(_home.longitude_deg);
	}, std::chrono::seconds(10)));
}

void AutopilotTester::check_home_within(float acceptance_radius_m)
{
	CHECK(ground_truth_horizontal_position_close_to(_home, acceptance_radius_m));
}

void AutopilotTester::set_takeoff_altitude(const float altitude_m)
{
	CHECK(Action::Result::Success == _action->set_takeoff_altitude(altitude_m));
	const auto result = _action->get_takeoff_altitude();
	CHECK(result.first == Action::Result::Success);
	CHECK(result.second == Approx(altitude_m));
}

void AutopilotTester::arm()
{
	const auto result = _action->arm();
	REQUIRE(result == Action::Result::Success);
}

void AutopilotTester::takeoff()
{
	const auto result = _action->takeoff();
	REQUIRE(result == Action::Result::Success);
}

void AutopilotTester::land()
{
	const auto result = _action->land();
	REQUIRE(result == Action::Result::Success);
}

void AutopilotTester::transition_to_fixedwing()
{
	const auto result = _action->transition_to_fixedwing();
	REQUIRE(result == Action::Result::Success);
}

void AutopilotTester::transition_to_multicopter()
{
	const auto result = _action->transition_to_multicopter();
	REQUIRE(result == Action::Result::Success);
}

void AutopilotTester::wait_until_disarmed(std::chrono::seconds timeout_duration)
{
	REQUIRE(poll_condition_with_timeout(
	[this]() { return !_telemetry->armed(); }, timeout_duration));
}

void AutopilotTester::wait_until_hovering()
{
	REQUIRE(poll_condition_with_timeout(
	[this]() { return _telemetry->landed_state() == Telemetry::LandedState::InAir; }, std::chrono::seconds(20)));
}

void AutopilotTester::prepare_square_mission(MissionOptions mission_options)
{
	const auto ct = get_coordinate_transformation();

	Mission::MissionPlan mission_plan {};
	mission_plan.mission_items.push_back(create_mission_item({mission_options.leg_length_m, 0.}, mission_options, ct));
	mission_plan.mission_items.push_back(create_mission_item({mission_options.leg_length_m, mission_options.leg_length_m},
					     mission_options, ct));
	mission_plan.mission_items.push_back(create_mission_item({0., mission_options.leg_length_m}, mission_options, ct));

	_mission->set_return_to_launch_after_mission(mission_options.rtl_at_end);

	std::promise<void> prom;
	auto fut = prom.get_future();

	_mission->upload_mission_async(mission_plan, [&prom](Mission::Result result) {
		REQUIRE(Mission::Result::Success == result);
		prom.set_value();
	});

	REQUIRE(fut.wait_for(std::chrono::seconds(2)) == std::future_status::ready);
}

void AutopilotTester::prepare_straight_mission(MissionOptions mission_options)
{
	const auto ct = get_coordinate_transformation();

	Mission::MissionPlan mission_plan {};
	mission_plan.mission_items.push_back(create_mission_item({0, 0.}, mission_options, ct));
	mission_plan.mission_items.push_back(create_mission_item({mission_options.leg_length_m, 0}, mission_options, ct));
	mission_plan.mission_items.push_back(create_mission_item({2 * mission_options.leg_length_m, 0}, mission_options, ct));
	mission_plan.mission_items.push_back(create_mission_item({3 * mission_options.leg_length_m, 0}, mission_options, ct));
	mission_plan.mission_items.push_back(create_mission_item({4 * mission_options.leg_length_m, 0}, mission_options, ct));

	_mission->set_return_to_launch_after_mission(mission_options.rtl_at_end);

	std::promise<void> prom;
	auto fut = prom.get_future();

	_mission->upload_mission_async(mission_plan, [&prom](Mission::Result result) {
		REQUIRE(Mission::Result::Success == result);
		prom.set_value();
	});

	REQUIRE(fut.wait_for(std::chrono::seconds(2)) == std::future_status::ready);
}

void AutopilotTester::execute_mission()
{
	std::promise<void> prom;
	auto fut = prom.get_future();

	_mission->start_mission_async([&prom](Mission::Result result) {
		REQUIRE(Mission::Result::Success == result);
		prom.set_value();
	});

	// TODO: Adapt time limit based on mission size, flight speed, sim speed factor, etc.

	REQUIRE(poll_condition_with_timeout(
	[this]() {
		auto result = _mission->is_mission_finished();
		return result.first == Mission::Result::Success && result.second;
	}, std::chrono::seconds(60)));

	REQUIRE(fut.wait_for(std::chrono::seconds(1)) == std::future_status::ready);
}

CoordinateTransformation AutopilotTester::get_coordinate_transformation()
{
	const auto home = _telemetry->home();
	CHECK(std::isfinite(home.latitude_deg));
	CHECK(std::isfinite(home.longitude_deg));
	return CoordinateTransformation({home.latitude_deg, home.longitude_deg});
}

Mission::MissionItem  AutopilotTester::create_mission_item(
	const CoordinateTransformation::LocalCoordinate &local_coordinate,
	const MissionOptions &mission_options,
	const CoordinateTransformation &ct)
{
	auto mission_item = Mission::MissionItem{};
	const auto pos_north = ct.global_from_local(local_coordinate);
	mission_item.latitude_deg = pos_north.latitude_deg;
	mission_item.longitude_deg = pos_north.longitude_deg;
	mission_item.relative_altitude_m = mission_options.relative_altitude_m;
	mission_item.is_fly_through = mission_options.fly_through;
	return mission_item;
}

void AutopilotTester::execute_rtl()
{
	REQUIRE(Action::Result::Success == _action->return_to_launch());
}

void AutopilotTester::offboard_goto(const Offboard::PositionNedYaw &target, float acceptance_radius_m,
				    std::chrono::seconds timeout_duration)
{
	_offboard->set_position_ned(target);
	REQUIRE(_offboard->start() == Offboard::Result::Success);
	CHECK(poll_condition_with_timeout(
	[ = ]() { return estimated_position_close_to(target, acceptance_radius_m); }, timeout_duration));
	std::cout << "Target position reached" << std::endl;
}

void AutopilotTester::check_mission_item_speed_above(int item_index, float min_speed_m_s)
{

	_telemetry->set_rate_velocity_ned(10);
	_telemetry->subscribe_velocity_ned([item_index, min_speed_m_s, this](Telemetry::VelocityNed velocity) {
		float horizontal = std::hypot(velocity.north_m_s, velocity.east_m_s);
		auto progress = _mission->mission_progress();

		if (progress.current == item_index) {
			CHECK(horizontal > min_speed_m_s);
		}
	});
}

void AutopilotTester::check_tracks_mission(float corridor_radius_m)
{
	auto mission = _mission->download_mission();
	CHECK(mission.first == Mission::Result::Success);

	std::vector<Mission::MissionItem> mission_items = mission.second.mission_items;
	auto ct = get_coordinate_transformation();

	_telemetry->set_rate_position_velocity_ned(5);
	_telemetry->subscribe_position_velocity_ned([ct, mission_items, corridor_radius_m,
	    this](Telemetry::PositionVelocityNed position_velocity_ned) {
		auto progress = _mission->mission_progress();

		if (progress.current > 0 && progress.current < progress.total) {
			// Get shortest distance of current position to 3D line between previous and next waypoint

			std::array<float, 3> current { position_velocity_ned.position.north_m,
						       position_velocity_ned.position.east_m,
						       position_velocity_ned.position.down_m };
			std::array<float, 3> wp_prev = get_local_mission_item(mission_items[progress.current - 1], ct);
			std::array<float, 3> wp_next = get_local_mission_item(mission_items[progress.current], ct);

			float distance_to_trajectory = point_to_line_distance(current, wp_prev, wp_next);

			CHECK(distance_to_trajectory < corridor_radius_m);
		}
	});
}


void AutopilotTester::offboard_land()
{
	Offboard::VelocityNedYaw land_velocity;
	land_velocity.north_m_s = 0.0f;
	land_velocity.east_m_s = 0.0f;
	land_velocity.down_m_s = 1.0f;
	land_velocity.yaw_deg = 0.0f;
	_offboard->set_velocity_ned(land_velocity);
}

bool AutopilotTester::estimated_position_close_to(const Offboard::PositionNedYaw &target_pos, float acceptance_radius_m)
{
	Telemetry::PositionNed est_pos = _telemetry->position_velocity_ned().position;
	return sq(est_pos.north_m - target_pos.north_m) +
	       sq(est_pos.east_m - target_pos.east_m) +
	       sq(est_pos.down_m - target_pos.down_m)  < sq(acceptance_radius_m);
}

bool AutopilotTester::estimated_horizontal_position_close_to(const Offboard::PositionNedYaw &target_pos,
		float acceptance_radius_m)
{
	Telemetry::PositionNed est_pos = _telemetry->position_velocity_ned().position;
	return sq(est_pos.north_m - target_pos.north_m) +
	       sq(est_pos.east_m - target_pos.east_m) < sq(acceptance_radius_m);
}

void AutopilotTester::request_ground_truth()
{
	CHECK(_telemetry->set_rate_ground_truth(15) == Telemetry::Result::Success);
}

bool AutopilotTester::ground_truth_horizontal_position_close_to(const Telemetry::GroundTruth &target_pos,
		float acceptance_radius_m)
{
	CHECK(std::isfinite(target_pos.latitude_deg));
	CHECK(std::isfinite(target_pos.longitude_deg));
	using GlobalCoordinate = CoordinateTransformation::GlobalCoordinate;
	using LocalCoordinate = CoordinateTransformation::LocalCoordinate;
	CoordinateTransformation ct(GlobalCoordinate{target_pos.latitude_deg, target_pos.longitude_deg});

	Telemetry::GroundTruth current_pos = _telemetry->ground_truth();
	CHECK(std::isfinite(current_pos.latitude_deg));
	CHECK(std::isfinite(current_pos.longitude_deg));
	GlobalCoordinate global_current;
	global_current.latitude_deg = current_pos.latitude_deg;
	global_current.longitude_deg = current_pos.longitude_deg;
	LocalCoordinate local_pos = ct.local_from_global(global_current);
	const double distance = sqrt(sq(local_pos.north_m) + sq(local_pos.east_m));
	const bool pass = distance < acceptance_radius_m;

	if (!pass) {
		std::cout << "target_pos.lat: " << target_pos.latitude_deg << std::endl;
		std::cout << "target_pos.lon: " << target_pos.longitude_deg << std::endl;
		std::cout << "current.lat: " << current_pos.latitude_deg << std::endl;
		std::cout << "current.lon: " << current_pos.longitude_deg << std::endl;
		std::cout << "Distance: " << distance << std::endl;
		std::cout << "Acceptance radius: " << acceptance_radius_m << std::endl;
	}

	return pass;
}
