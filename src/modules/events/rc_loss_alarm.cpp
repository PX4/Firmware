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

/**
 * @file rc_loss_alarm.cpp
 *
 */

#include "rc_loss_alarm.h"

#include <px4_platform_common/defines.h>

#include <drivers/drv_hrt.h>
#include <stdint.h>

#include <tunes/tune_definition.h>

namespace events
{
namespace rc_loss
{

void RC_Loss_Alarm::process()
{
	vehicle_status_s status{};

	if (!_vehicle_status_sub.update(&status)) {
		return;
	}

	if (!_was_armed &&
	    status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {

		_was_armed = true;	// Once true, impossible to go back to false
	}

	if (!_had_rc && !status.rc_signal_lost) {

		_had_rc = true;
	}

	if (_was_armed && _had_rc && status.rc_signal_lost &&
	    status.arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
		play_tune();
		_alarm_playing = true;

	} else if (_alarm_playing) {
		stop_tune();
		_alarm_playing = false;
	}
}

void RC_Loss_Alarm::play_tune()
{
	tune_control_s tune_control{};
	tune_control.tune_id = tune_control_s::TUNE_ID_ERROR;
	tune_control.tune_override = true;
	tune_control.volume = tune_control_s::VOLUME_LEVEL_MAX;
	tune_control.timestamp = hrt_absolute_time();
	_tune_control_pub.publish(tune_control);
}

void RC_Loss_Alarm::stop_tune()
{
	tune_control_s tune_control{};
	tune_control.tune_override = true;
	tune_control.timestamp = hrt_absolute_time();
	_tune_control_pub.publish(tune_control);
}

} /* namespace rc_loss */
} /* namespace events */
