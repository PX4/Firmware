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
 * @file test_jig_voltages.c
 * Tests for jig voltages.
 */

#include <px4_platform_common/defines.h>
#include <unistd.h>
#include "tests_main.h"
#include <drivers/drv_adc.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/adc_report.h>

int test_jig_voltages(int argc, char *argv[])
{
	uORB::Subscription	_adc_sub{ORB_ID(adc_report)};
	adc_report_s adc;

	px4_usleep(50000);	// sleep 50ms and wait for adc report

	if (_adc_sub.update(&adc)) {
		PX4_INFO_RAW("DeviceID: %d\n", adc.device_id);
		PX4_INFO_RAW("Resolution: %d\n", adc.resolution);
		PX4_INFO_RAW("Voltage Reference: %f\n", adc.v_ref);

		unsigned channels = 0;

		for (int i = 0; i < PX4_MAX_ADC_CHANNELS; ++i) {
			PX4_INFO_RAW("%d: %d  ", adc.channel_id[i], adc.raw_data[i]);

			if (adc.channel_id[i] != -1) {
				++channels;
			}
		}

		PX4_INFO_RAW("\n");

		PX4_INFO("\t ADC operational.\n");

		/* Expected values */
		int16_t expected_min[] = {2800, 2800, 1800,  800};
		int16_t expected_max[] = {3100, 3100, 2100, 1100};
		const char *check_res[channels];

		if (channels < 4) {
			PX4_ERR("not all four test channels available, aborting.");
			return 1;

		} else {
			int ret = OK;

			/* Check values */
			check_res[0] = (expected_min[0] < adc.raw_data[0] && expected_max[0] > adc.raw_data[0]) ? "OK" : "FAIL";
			check_res[1] = (expected_min[1] < adc.raw_data[1] && expected_max[1] > adc.raw_data[1]) ? "OK" : "FAIL";
			check_res[2] = (expected_min[2] < adc.raw_data[2] && expected_max[2] > adc.raw_data[2]) ? "OK" : "FAIL";
			check_res[3] = (expected_min[3] < adc.raw_data[3] && expected_max[3] > adc.raw_data[3]) ? "OK" : "FAIL";

			/* Accumulate result */
			ret += (expected_min[0] > adc.raw_data[0] || expected_max[0] < adc.raw_data[0]) ? 1 : 0;
			ret += (expected_min[1] > adc.raw_data[1] || expected_max[1] < adc.raw_data[1]) ? 1 : 0;
			ret += (expected_min[2] > adc.raw_data[2] || expected_max[2] < adc.raw_data[2]) ? 1 : 0;
			ret += (expected_min[3] > adc.raw_data[3] || expected_max[3] < adc.raw_data[3]) ? 1 : 0;

			PX4_INFO("Sample:");
			PX4_INFO("channel: %d value: %d (allowed min: %d, allowed max: %d), result: %s",
				 adc.channel_id[0], (int)(adc.raw_data[0]), expected_min[0], expected_max[0], check_res[0]);
			PX4_INFO("channel: %d value: %d (allowed min: %d, allowed max: %d), result: %s",
				 adc.channel_id[1], (int)(adc.raw_data[1]), expected_min[1], expected_max[1], check_res[1]);
			PX4_INFO("channel: %d value: %d (allowed min: %d, allowed max: %d), result: %s",
				 adc.channel_id[2], (int)(adc.raw_data[2]), expected_min[2], expected_max[2], check_res[2]);
			PX4_INFO("channel: %d value: %d (allowed min: %d, allowed max: %d), result: %s",
				 adc.channel_id[3], (int)(adc.raw_data[3]), expected_min[3], expected_max[3], check_res[3]);

			if (ret != OK) {
				PX4_ERR("\t JIG voltages test FAILED. Some channels where out of allowed range. Check supply voltages.");
				return ret;
			}
		}

		PX4_INFO("JIG voltages test successful.");

		return OK;

	} else {
		return 1;
	}
}
