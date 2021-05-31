/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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
#include <gtest/gtest.h>
#include "FakeNavigator.h"
#include "takeoff.h"
#include <dataman/dataman_mocks.hpp>


TEST(NavigatorModeTakeoffTest, TestTakeoff)
{
	FakeNavigator fake_navigator;

	NavigatorCore &navigator_core = fake_navigator.getCore();
	Takeoff takeoff(&fake_navigator, navigator_core);
	fake_navigator.params_update();

	vehicle_status_s status = {};
	status.timestamp = 1e6;
	status.is_vtol = true;
	status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
	status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	vehicle_global_position_s global_pos = {};
	global_pos.timestamp = 1e6;
	global_pos.lat = 40;
	global_pos.lon = 3;
	global_pos.alt = 300;

	vehicle_land_detected_s landed = {};
	landed.landed = true;
	landed.timestamp = 1e6;

	home_position_s home = {};
	home.timestamp = 1e6;
	home.lat = global_pos.lat;
	home.lon = global_pos.lon;
	home.alt = global_pos.alt;
	home.valid_alt = true;
	home.valid_hpos = true;
	home.valid_lpos = true;

	vehicle_local_position_s local_pos = {};
	local_pos.timestamp = 1e6;
	local_pos.x = -0.2f;
	local_pos.y = 0.1f;
	local_pos.z = 0.1f;

	local_pos.vx = local_pos.vy = local_pos.vz = 0;
	local_pos.heading = -0.54f;

	navigator_core.updateGlobalPosition(global_pos);
	navigator_core.updateHomePosition(home);
	navigator_core.updateLandedState(landed);
	navigator_core.updateLocalPosition(local_pos);
	navigator_core.updateVehicleStatus(status);

	takeoff.on_activation();

	position_setpoint_s current = fake_navigator.get_position_setpoint_triplet()->current;

	ASSERT_EQ(current.lat, navigator_core.getLatRad());
	ASSERT_EQ(current.lon, navigator_core.getLonRad());
	ASSERT_EQ(current.type, 3); // position_setpoint_s::TYPE_TAKEOFF
	ASSERT_EQ(current.valid, true);
	ASSERT_EQ(current.alt_valid, true);
	ASSERT_EQ(current.alt, global_pos.alt + navigator_core.getRelativeTakeoffMinAltitudeMeter());
	ASSERT_EQ(current.yaw, navigator_core.getTrueHeadingRad());
	ASSERT_EQ(current.yaw_valid, true);
	ASSERT_EQ(current.yawspeed_valid, false);
	ASSERT_EQ(current.loiter_radius, navigator_core.getLoiterRadiusMeter());
	ASSERT_EQ(current.loiter_direction, 1);
	ASSERT_EQ(current.acceptance_radius, navigator_core.getHorAcceptanceRadiusMeter());
	ASSERT_EQ(current.cruising_speed, -1.0f);

	uint64_t t_first = current.timestamp;

	landed.landed = false;
	navigator_core.updateLandedState(landed);
	global_pos.alt += navigator_core.getRelativeTakeoffMinAltitudeMeter();
	navigator_core.updateGlobalPosition(global_pos);

	takeoff.on_active();
	current = fake_navigator.get_position_setpoint_triplet()->current;

	ASSERT_NE(t_first, current.timestamp);
	ASSERT_EQ(current.type, 2);




}
