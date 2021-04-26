/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#ifndef MODULE_NAME
#define MODULE_NAME "bbblue_pwm_out"
#endif

#include <fcntl.h>
#include <errno.h>
#include <px4_platform_common/log.h>

#include <robotcontrol.h>
#include <board_config.h>

#include "board_pwm_out.h"

using namespace pwm_out;

BBBlueRcPWMOut::BBBlueRcPWMOut(int max_num_outputs) : _num_outputs(max_num_outputs)
{
	if (_num_outputs > MAX_NUM_PWM) {
		PX4_WARN("number of outputs too large. Setting to %i", MAX_NUM_PWM);
		_num_outputs = MAX_NUM_PWM;
	}
}

BBBlueRcPWMOut::~BBBlueRcPWMOut()
{
	rc_cleaning();
}

int BBBlueRcPWMOut::init()
{
	rc_init();

	return 0;
}

int BBBlueRcPWMOut::send_output_pwm(const uint16_t *pwm, int num_outputs)
{
	if (num_outputs > _num_outputs) {
		num_outputs = _num_outputs;
	}

	int ret = 0;

	// pwm[ch] is duty_cycle in us
	for (int ch = 0; ch < num_outputs; ++ch) {
		ret += rc_servo_send_pulse_us(ch + 1, pwm[ch]); // converts to 1-based channel #
	}

	return ret;
}

