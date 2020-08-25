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

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <iostream>
#include <string>
#include "autopilot_tester.h"


TEST_CASE("Offboard takeoff and land", "[multicopter][offboard][nogps]")
{
	AutopilotTester tester;
	Offboard::PositionNEDYaw takeoff_position {0.0f, 0.0f, -2.0f, 0.0f};
	tester.connect(connection_url);
	tester.wait_until_ready_local_position_only();
	tester.store_home();
	tester.arm();
	tester.offboard_goto(takeoff_position, 0.5f);
	tester.offboard_land();
	tester.wait_until_disarmed();
	tester.check_home_within(0.5f);
}

TEST_CASE("Offboard position control", "[multicopter][offboard][nogps]")
{
	AutopilotTester tester;
	Offboard::PositionNEDYaw takeoff_position {0.0f, 0.0f, -2.0f, 0.0f};
	Offboard::PositionNEDYaw setpoint_1 {0.0f, 5.0f, -2.0f, 180.0f};
	Offboard::PositionNEDYaw setpoint_2 {5.0f, 5.0f, -4.0f, 180.0f};
	Offboard::PositionNEDYaw setpoint_3 {5.0f, 0.0f, -4.0f, 90.0f};
	tester.connect(connection_url);
	tester.wait_until_ready_local_position_only();
	tester.store_home();
	tester.arm();
	tester.offboard_goto(takeoff_position, 0.5f);
	tester.offboard_goto(setpoint_1, 1.0f);
	tester.offboard_goto(setpoint_2, 1.0f);
	tester.offboard_goto(setpoint_3, 1.0f);
	tester.offboard_goto(takeoff_position, 0.2f);
	tester.offboard_land();
	tester.wait_until_disarmed();
	tester.check_home_within(1.0f);
}
