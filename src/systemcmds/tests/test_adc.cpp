/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file test_adc.c
 * Test for the analog to digital converter.
 */

#include <px4_platform_common/time.h>
#include <px4_platform_common/log.h>
#include "tests_main.h"
#include <drivers/drv_adc.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/adc_report.h>

int test_adc(int argc, char *argv[])
{
	uORB::Subscription	_adc_sub{ORB_ID(adc_report)};
	adc_report_s adc;

	px4_usleep(50000);	// sleep 50ms and wait for adc report

	if (_adc_sub.update(&adc)) {
		PX4_INFO_RAW("DeviceID: %d\n", adc.device_id);
		PX4_INFO_RAW("Resolution: %d\n", adc.resolution);
		PX4_INFO_RAW("Voltage Reference: %f\n", adc.v_ref);

		for (int i = 0; i < PX4_MAX_ADC_CHANNELS; ++i) {
			PX4_INFO_RAW("%d: %d  ", adc.channel_id[i], adc.raw_data[i]);
		}

		PX4_INFO_RAW("\n");

		PX4_INFO_RAW("\t ADC test successful.\n");

		return OK;

	} else {
		return 1;
	}
}
