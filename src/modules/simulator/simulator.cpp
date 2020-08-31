/****************************************************************************
 *
 *   Copyright (c) 2015 Mark Charlebois. All rights reserved.
 *   Copyright (c) 2016-2019 PX4 Development Team. All rights reserved.
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
 * @file simulator.cpp
 *
 * This module interfaces via MAVLink to a software in the loop simulator (SITL)
 * such as jMAVSim or Gazebo.
 */

#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <systemlib/err.h>
#include <drivers/drv_board_led.h>

#include "simulator.h"

static px4_task_t g_sim_task = -1;

Simulator *Simulator::_instance = nullptr;

void Simulator::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}
}

int Simulator::start(int argc, char *argv[])
{
	_instance = new Simulator();

	if (_instance) {
		if (argc == 4 && strcmp(argv[2], "-u") == 0) {
			_instance->set_ip(InternetProtocol::UDP);
			_instance->set_port(atoi(argv[3]));
		}

		if (argc == 4 && strcmp(argv[2], "-c") == 0) {
			_instance->set_ip(InternetProtocol::TCP);
			_instance->set_port(atoi(argv[3]));
		}

		_instance->run();

		return 0;

	} else {
		PX4_WARN("Simulator creation failed");
		return 1;
	}
}

static void usage()
{
	PX4_INFO("Usage: simulator {start -[spt] [-u udp_port / -c tcp_port] |stop|status}");
	PX4_INFO("Start simulator:     simulator start");
	PX4_INFO("Connect using UDP: simulator start -u udp_port");
	PX4_INFO("Connect using TCP: simulator start -c tcp_port");
}

__BEGIN_DECLS
extern int simulator_main(int argc, char *argv[]);
__END_DECLS


int simulator_main(int argc, char *argv[])
{
	if (argc > 1 && strcmp(argv[1], "start") == 0) {

		if (g_sim_task >= 0) {
			PX4_WARN("Simulator already started");
			return 0;
		}

		g_sim_task = px4_task_spawn_cmd("simulator",
						SCHED_DEFAULT,
						SCHED_PRIORITY_MAX,
						1500,
						Simulator::start,
						argv);

#if defined(ENABLE_LOCKSTEP_SCHEDULER)

		// We want to prevent the rest of the startup script from running until time
		// is initialized by the HIL_SENSOR messages from the simulator.
		while (true) {
			if (Simulator::getInstance() && Simulator::getInstance()->has_initialized()) {
				break;
			}

			system_usleep(100);
		}

#endif

	} else if (argc == 2 && strcmp(argv[1], "stop") == 0) {
		if (g_sim_task < 0) {
			PX4_WARN("Simulator not running");
			return 1;

		} else {
			px4_task_delete(g_sim_task);
			g_sim_task = -1;
		}

	} else if (argc == 2 && strcmp(argv[1], "status") == 0) {
		if (g_sim_task < 0) {
			PX4_WARN("Simulator not running");
			return 1;

		} else {
			PX4_INFO("running");
		}

	} else {
		usage();
		return 1;
	}

	return 0;
}
