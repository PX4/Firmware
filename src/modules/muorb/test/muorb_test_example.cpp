/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
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
 * @file hello_example.cpp
 * Example for Linux
 *
 * @author Mark Charlebois <charlebm@gmail.com>
 */

#include "muorb_test_example.h"
#include <px4_platform_common/log.h>
#include <unistd.h>
#include <stdio.h>
#include <px4_platform_common/defines.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>

px4::AppState MuorbTestExample::appState;

int MuorbTestExample::main()
{
	int rc;
	appState.setRunning(true);
	rc = PingPongTest();
	appState.setRunning(false);
	return rc;
}

int  MuorbTestExample::DefaultTest()
{
	int i = 0;

	uORB::Subscription sub_vc{ORB_ID(vehicle_command)};
	uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
	uORB::Publication<esc_status_s> pub_id{ORB_ID(esc_status)};
	pub_id.publish(m_esc_status);

	while (!appState.exitRequested() && i < 100) {

		PX4_DEBUG("[%d]  Doing work...", i);

		if (!pub_id.publish(m_esc_status)) {
			PX4_ERR("[%d]Error publishing the esc status message for iter", i);
			break;
		}

		if (sub_vc.updated()) {
			PX4_DEBUG("[%d]Vechicle Status is updated... reading new value", i);

			if (!sub_vc.copy(&m_vc)) {
				PX4_ERR("[%d]Error calling orb copy for vechivle status... ", i);
				break;
			}

			if (!vcmd_pub.publish(m_vc)) {
				PX4_ERR("[%d]Error publishing the vechile command message", i);
				break;
			}

		} else {
			PX4_ERR("[%d]Error checking the updated status for vechile command... ", i);
			PX4_DEBUG("[%d] VC topic is not updated", i);
			break;
		}

		++i;
	}

	return 0;
}

int MuorbTestExample::PingPongTest()
{
	int i = 0;
	uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
	uORB::Subscription sub_esc_status{ORB_ID(esc_status)};

	while (!appState.exitRequested()) {

		PX4_INFO("[%d]  Doing work...", i);

		if (sub_esc_status.updated()) {
			PX4_INFO("[%d]ESC status is updated... reading new value", i);

			if (!sub_esc_status.copy(&m_esc_status)) {
				PX4_ERR("[%d]Error calling orb copy for esc status... ", i);
				break;
			}

			if (!vcmd_pub.publish(m_vc)) {
				PX4_ERR("[%d]Error publishing the vechile command message", i);
				break;
			}

		} else {
			PX4_INFO("[%d] esc status topic is not updated", i);
		}

		// sleep for 1 sec.
		usleep(1000000);

		++i;
	}

	return 0;
}
