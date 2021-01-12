/****************************************************************************
 *
 *   Copyright (C) 2013-2020 PX4 Development Team. All rights reserved.
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
 * @file ToneAlarm.cpp
 */

#include "ToneAlarm.h"

#include <px4_platform_common/time.h>
#include <uORB/Publication.hpp>

using namespace time_literals;

ToneAlarm::ToneAlarm() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	// ensure ORB_ID(tune_control) is advertised with correct queue depth
	orb_advertise_queue(ORB_ID(tune_control), nullptr, tune_control_s::ORB_QUEUE_LENGTH);
}

ToneAlarm::~ToneAlarm()
{
	ToneAlarmInterface::stop_note();
}

bool ToneAlarm::Init()
{
	// NOTE: Implement hardware specific detail in the ToneAlarmInterface class implementation.
	ToneAlarmInterface::init();

	_tune_control_sub.set_interval_us(10_ms);

	if (!_tune_control_sub.registerCallback()) {
		PX4_ERR("tune_control callback registration failed!");
		return PX4_ERROR;
	}

	ScheduleNow();

	return true;
}

void ToneAlarm::InterruptStopNote(void *arg)
{
	ToneAlarmInterface::stop_note();
}

void ToneAlarm::Run()
{
	// Check if circuit breaker is enabled.
	if (!_circuit_break_initialized) {
		if (circuit_breaker_enabled("CBRK_BUZZER", CBRK_BUZZER_KEY)) {
			request_stop();
		}

		_circuit_break_initialized = true;
	}

	if (should_exit()) {
		_tune_control_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// Check for next tune_control when not currently playing
	if (_tune_control_sub.updated()) {
		tune_control_s tune_control;

		if (_tune_control_sub.copy(&tune_control)) {
			if (tune_control.timestamp > 0) {
				Tunes::ControlResult tune_result = _tunes.set_control(tune_control);

				switch (tune_result) {
				case Tunes::ControlResult::Success:
					PX4_DEBUG("new tune %d", tune_control.tune_id);

					if (tune_control.tune_override) {
						// clear existing
						ToneAlarmInterface::stop_note();
						_next_note_time = 0;
						hrt_cancel(&_hrt_call);
					}

					_play_tone = true;

#if (!defined(TONE_ALARM_TIMER) && !defined(GPIO_TONE_ALARM_GPIO)) || defined(DEBUG_BUILD)

					switch (tune_control.tune_id) {
					case tune_control_s::TUNE_ID_STARTUP:
						PX4_INFO("startup tune");
						break;

					case tune_control_s::TUNE_ID_ERROR:
						PX4_ERR("error tune");
						break;

					case tune_control_s::TUNE_ID_NOTIFY_POSITIVE:
						PX4_INFO("notify positive");
						break;

					case tune_control_s::TUNE_ID_NOTIFY_NEUTRAL:
						PX4_INFO("notify neutral");
						break;

					case tune_control_s::TUNE_ID_NOTIFY_NEGATIVE:
						PX4_INFO("notify negative");
						break;

					case tune_control_s::TUNE_ID_ARMING_WARNING:
						PX4_INFO("arming warning");
						break;

					case tune_control_s::TUNE_ID_BATTERY_WARNING_SLOW:
						PX4_INFO("battery warning (slow)");
						break;

					case tune_control_s::TUNE_ID_BATTERY_WARNING_FAST:
						PX4_INFO("battery warning (fast)");
						break;

					case tune_control_s::TUNE_ID_ARMING_FAILURE:
						PX4_ERR("arming failure");
						break;

					case tune_control_s::TUNE_ID_SINGLE_BEEP:
						PX4_INFO("beep");
						break;

					case tune_control_s::TUNE_ID_HOME_SET:
						PX4_INFO("home set");
						break;
					}

#endif // (!TONE_ALARM_TIMER && !GPIO_TONE_ALARM_GPIO) || DEBUG_BUILD

					break;

				case Tunes::ControlResult::WouldInterrupt:
					// otherwise re-publish tune to process next
					PX4_DEBUG("tune already playing, requeing tune: %d", tune_control.tune_id);
					{
						uORB::Publication<tune_control_s> tune_control_pub{ORB_ID(tune_control)};
						tune_control.timestamp = hrt_absolute_time();
						tune_control_pub.publish(tune_control);
					}

					break;


				case Tunes::ControlResult::InvalidTune:
					PX4_WARN("Invalid tune: %d", tune_control.tune_id);
					break;

				case Tunes::ControlResult::AlreadyPlaying:
					// Do nothing
					break;
				}
			}
		}
	}

	unsigned int frequency = 0;
	unsigned int duration = 0;
	unsigned int silence_length = 0;

	// Does an inter-note silence occur?
	if ((_next_note_time != 0) && (hrt_absolute_time() < _next_note_time)) {
		PX4_DEBUG("inter-note silence");
		ScheduleAt(_next_note_time);

	} else if (_play_tone && (_tunes.get_next_note(frequency, duration, silence_length) == Tunes::Status::Continue)) {
		PX4_DEBUG("Play frequency: %d, duration: %d us, silence: %d us", frequency, duration, silence_length);

		if (frequency > 0) {
			// Start playing the note.
			const hrt_abstime time_started = ToneAlarmInterface::start_note(frequency);

			if (time_started > 0) {
				// schedule stop with HRT
				hrt_call_at(&_hrt_call, time_started + duration, (hrt_callout)&InterruptStopNote, this);
				_next_note_time = time_started + duration + silence_length;

				// schedule next note
				ScheduleAt(_next_note_time);
			}

		} else {
			// A frequency of 0 corresponds to ToneAlarmInterface::stop_note()
			_next_note_time = hrt_absolute_time() + duration + silence_length;
			ToneAlarmInterface::stop_note();
			ScheduleAt(_next_note_time);
		}

	} else {
		PX4_DEBUG("stopping");
		ToneAlarmInterface::stop_note();
		_play_tone = false;
		_next_note_time = 0;
	}

	// if done playing and a new tune_control is still available re-schedule
	if (!Scheduled() && _tune_control_sub.updated()) {
		ScheduleDelayed(_tunes.get_maximum_update_interval());
	}
}

int ToneAlarm::task_spawn(int argc, char *argv[])
{
	ToneAlarm *instance = new ToneAlarm();

	if (!instance) {
		PX4_ERR("alloc failed");
		return -1;
	}

	if (!instance->Init()) {
		delete instance;
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	return PX4_OK;
}

int ToneAlarm::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module is responsible for the tone alarm.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("tone_alarm", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int tone_alarm_main(int argc, char *argv[])
{
	return ToneAlarm::main(argc, argv);
}
