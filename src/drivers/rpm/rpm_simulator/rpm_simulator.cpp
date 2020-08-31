/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *   Copyright (c) 2020 ThunderFly s.r.o. All rights reserved.
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
 * @file rpm_simulator.c
 * Simple app for publishing RPM messages with custom value.
 *
 * Usage: rpm_simulator <rpm_value>
 *  rpm_simulator 344.2
 *
 * @author ThunderFly s.r.o., Roman Dvorak <dvorakroman@thunderfly.cz>
 */

#include <px4_platform_common/px4_config.h>
#include <drivers/drv_hrt.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/rpm.h>

extern "C" __EXPORT int rpm_simulator_main(int argc, char *argv[]);
int rpm_simulator_main(int argc, char *argv[])
{

	// check input
	if (argc != 2) {
		PX4_INFO("Usage: rpm_simulator <published RPM>");
		PX4_INFO("Exit. Without publishing any message.");
		return 0;
	}

	rpm_s rpm{};

	uORB::Publication<rpm_s> rpm_pub{ORB_ID(rpm)};
	uint64_t timestamp_us = hrt_absolute_time();
	float frequency = atof(argv[1]);

	// prpepare RPM data message
	rpm.timestamp = timestamp_us;
	rpm.indicated_frequency_rpm = frequency;
	rpm.estimated_accurancy_rpm = frequency / 100.0f;

	// Publish data and let the user know what was published
	rpm_pub.publish(rpm);
	print_message(rpm);

	return 0;
}
