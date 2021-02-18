/****************************************************************************
 *
 *  Copyright (C) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file test_rc.c
 * Tests RC input.
 */

#include <px4_platform_common/px4_config.h>

#include <sys/types.h>

#include <stdio.h>
#include <poll.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include <drivers/drv_pwm_output.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_hrt.h>
#include <systemlib/err.h>

#include "tests_main.h"

#include <math.h>
#include <float.h>

int test_rc(int argc, char *argv[])
{
	int input_rc_sub = orb_subscribe(ORB_ID(input_rc));

	/* read low-level values from FMU or IO RC inputs (PPM, Spektrum, S.Bus) */
	struct input_rc_s rc_input;
	struct input_rc_s rc_last;
	orb_copy(ORB_ID(input_rc), input_rc_sub, &rc_input);
	px4_usleep(100000);

	/* open PPM input and expect values close to the output values */

	bool rc_updated;
	orb_check(input_rc_sub, &rc_updated);

	PX4_INFO("Reading PPM values - press any key to abort");
	PX4_INFO("This test guarantees: 10 Hz update rates, no glitches (channel values), no channel count changes.");

	if (rc_updated) {

		/* copy initial set */
		for (unsigned i = 0; i < rc_input.channel_count; i++) {
			rc_last.values[i] = rc_input.values[i];
		}

		rc_last.channel_count = rc_input.channel_count;

		/* poll descriptor */
		struct pollfd fds[2];
		fds[0].fd = input_rc_sub;
		fds[0].events = POLLIN;
		fds[1].fd = 0;
		fds[1].events = POLLIN;

		while (true) {

			int ret = poll(fds, 2, 200);

			if (ret > 0) {

				if (fds[0].revents & POLLIN) {

					orb_copy(ORB_ID(input_rc), input_rc_sub, &rc_input);

					/* go and check values */
					for (unsigned i = 0; i < rc_input.channel_count; i++) {
						if (abs(rc_input.values[i] - rc_last.values[i]) > 20) {
							PX4_ERR("comparison fail: RC: %d, expected: %d", rc_input.values[i], rc_last.values[i]);
							(void)orb_unsubscribe(input_rc_sub);
							return ERROR;
						}

						rc_last.values[i] = rc_input.values[i];
					}

					if (rc_last.channel_count != rc_input.channel_count) {
						PX4_ERR("channel count mismatch: last: %d, now: %d", rc_last.channel_count, rc_input.channel_count);
						(void)orb_unsubscribe(input_rc_sub);
						return ERROR;
					}

					if (hrt_absolute_time() - rc_input.timestamp_last_signal > 100000) {
						PX4_ERR("TIMEOUT, less than 10 Hz updates");
						(void)orb_unsubscribe(input_rc_sub);
						return ERROR;
					}

				} else {
					/* key pressed, bye bye */
					return 0;
				}

			}
		}

	} else {
		PX4_ERR("failed reading RC input data");
		return ERROR;
	}

	PX4_INFO("PPM CONTINUITY TEST PASSED SUCCESSFULLY!");

	return 0;
}
