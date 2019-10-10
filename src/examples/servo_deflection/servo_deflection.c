/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file servo_deflection.c
 * Minimal application example of servo deflection for PX4 autopilot
 * @author Richard Kirby <rkirby@kspresearch.com>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/actuator_direct.h>

__EXPORT int servo_deflection_main(int argc, char *argv[]);

int servo_deflection_main(int argc, char *argv[])
{
	if (argc < 5) {
		PX4_INFO("useage: <actuator_0 value> <actuator_1 value> <actuator_2 value> <actuator_3 value>\n\r");
		return 1;
	}

	const float actuator_value[] = {atof(argv[1]), atof(argv[2]), atof(argv[3]), atof(argv[4])};

	/* advertise actuator direct topic */
	struct actuator_direct_s act_out;
	memset(&act_out, 0, sizeof(act_out));
	orb_advert_t act_pub = orb_advertise(ORB_ID(actuator_direct), &act_out);

	/*reserve first four values for ESCs, this will be handled correctly by mixer*/
	act_out.values[0] = 0.0;
	act_out.values[1] = 0.0;
	act_out.values[2] = 0.0;
	act_out.values[3] = 0.0;

	/*next 4 values I'm using as for servo motors, value must be between -1 and 1*/
	act_out.values[4] = actuator_value[0];
	act_out.values[5] = actuator_value[1];
	act_out.values[6] = actuator_value[2];
	act_out.values[7] = actuator_value[3];

	act_out.nvalues = 8;

	orb_publish(ORB_ID(actuator_direct), act_pub, &act_out);

	PX4_INFO("uORB actuator message sent");

	return 0;
}
