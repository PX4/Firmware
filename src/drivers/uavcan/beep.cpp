/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file beep.cpp
 *
 * @author CUAVcaijie <caijie@cuav.net>
 */

#include "beep.hpp"

#include <lib/circuit_breaker/circuit_breaker.h>

UavcanBeep::UavcanBeep(uavcan::INode &node) :
	_beep_pub(node),
	_timer(node)
{
}

int UavcanBeep::init()
{
	// don't initialize if CBRK_BUZZER circuit breaker is enabled.
	if (circuit_breaker_enabled("CBRK_BUZZER", CBRK_BUZZER_KEY)) {
		return 0;
	}

	/*
	 * Setup timer and call back function for periodic updates
	 */
	if (!_timer.isRunning()) {
		_timer.setCallback(TimerCbBinder(this, &UavcanBeep::periodic_update));
		_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(1000 / MAX_RATE_HZ));
	}

	return 0;
}

void UavcanBeep::periodic_update(const uavcan::TimerEvent &)
{
	if (_tune_control_sub.updated()) {
		_tune_control_sub.copy(&_tune);

		if (_tune.timestamp > 0) {
			Tunes::ControlResult result = _tunes.set_control(_tune);
			_play_tone = (result == Tunes::ControlResult::Success) || (result == Tunes::ControlResult::AlreadyPlaying);
		}
	}

	const hrt_abstime timestamp_now = hrt_absolute_time();

	if ((timestamp_now - _interval_timestamp <= _duration) || !_play_tone) {
		return;
	}

	_interval_timestamp = timestamp_now;

	if (_silence_length > 0) {
		_duration = _silence_length;
		_silence_length = 0;

	} else if (_play_tone) {
		Tunes::Status parse_ret_val = _tunes.get_next_note(_frequency, _duration, _silence_length);

		if (parse_ret_val == Tunes::Status::Continue) {
			// Continue playing.
			_play_tone = true;

			if (_frequency > 0) {
				// Start playing the note.
				uavcan::equipment::indication::BeepCommand cmd{};
				cmd.frequency = _frequency;
				cmd.duration = _duration / 1000000.f;
				_beep_pub.broadcast(cmd);
			}

		} else {
			_play_tone = false;
		}
	}
}
