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


#include "tune_publisher.h"
#include "string.h"
#include <px4_platform_common/log.h>

void TunePublisher::set_tune_string(const char *tune, const hrt_abstime &now)
{
	// The tune string needs to be 0 terminated.
	const unsigned tune_len = strlen(tune);

	// We don't expect the tune string to be longer than what can come in via MAVLink including 0 termination.
	if (tune_len >= MAX_TUNE_LEN) {
		PX4_ERR("Tune string too long.");
		return;
	}

	strncpy(_tune_buffer, tune, MAX_TUNE_LEN);

	_tunes.set_string(_tune_buffer, tune_control_s::VOLUME_LEVEL_DEFAULT);

	_next_publish_time = now;
}


void TunePublisher::publish_next_tune(const hrt_abstime now)
{
	if (_next_publish_time == 0) {
		// Nothing to play.
		return;
	}

	if (now < _next_publish_time) {
		// Too early, try again later.
		return;
	}

	unsigned frequency;
	unsigned duration;
	unsigned silence;
	uint8_t volume;

	if (_tunes.get_next_note(frequency, duration, silence, volume) > 0) {
		tune_control_s tune_control {};
		tune_control.tune_id = 0;
		tune_control.volume = tune_control_s::VOLUME_LEVEL_DEFAULT;

		tune_control.tune_id = 0;
		tune_control.frequency = static_cast<uint16_t>(frequency);
		tune_control.duration = static_cast<uint32_t>(duration);
		tune_control.silence = static_cast<uint32_t>(silence);
		tune_control.volume = static_cast<uint8_t>(volume);
		tune_control.timestamp = now;
		_tune_control_pub.publish(tune_control);

		_next_publish_time = now + duration + silence;

	} else {
		// We're done, let's reset.
		_next_publish_time = 0;
	}
}
