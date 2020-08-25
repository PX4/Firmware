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


TEST_CASE("Takeoff and Land", "[multicopter][vtol]")
{
	AutopilotTester tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.arm();
	tester.takeoff();
	tester.wait_until_hovering();
	tester.land();
	tester.wait_until_disarmed();
}

TEST_CASE("Fly square Multicopter Missions", "[multicopter][vtol]")
{
	AutopilotTester tester;
	tester.connect(connection_url);
	tester.wait_until_ready();

	SECTION("Mission including RTL") {
		AutopilotTester::MissionOptions mission_options;
		mission_options.rtl_at_end = true;
		tester.prepare_square_mission(mission_options);
		tester.arm();
		tester.execute_mission();
		tester.wait_until_disarmed();
	}

	SECTION("Mission with manual RTL") {
		AutopilotTester::MissionOptions mission_options;
		mission_options.rtl_at_end = false;
		tester.prepare_square_mission(mission_options);
		tester.arm();
		tester.execute_mission();
		tester.wait_until_hovering();
		tester.execute_rtl();
		tester.wait_until_disarmed();
	}
}
