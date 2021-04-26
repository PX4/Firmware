/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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

#include <drivers/drv_tone_alarm.h>
#include "SafetyButton.hpp"

#ifndef GPIO_BTN_SAFETY
#error "board needs to define a safety button gpio pin to use this module"
#endif

using namespace time_literals;

static constexpr uint8_t CYCLE_COUNT{30}; /* safety switch must be held for 1 second to activate */

// Define the various LED flash sequences for each system state.
enum class LED_PATTERN : uint16_t {
	FMU_OK_TO_ARM		= 0x0003,	/**< slow blinking */
	FMU_REFUSE_TO_ARM	= 0x5555,	/**< fast blinking */
	IO_ARMED		= 0x5050,	/**< long off, then double blink */
	FMU_ARMED		= 0x5500,	/**< long off, then quad blink */
	IO_FMU_ARMED		= 0xffff,	/**< constantly on */
};

SafetyButton::SafetyButton() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	_safety_disabled = circuit_breaker_enabled("CBRK_IO_SAFETY", CBRK_IO_SAFETY_KEY);

	if (_safety_disabled) {
		_safety_btn_off = true;
	}
}

SafetyButton::~SafetyButton()
{
	ScheduleClear();
}

void
SafetyButton::CheckButton()
{
	const bool safety_button_pressed = px4_arch_gpioread(GPIO_BTN_SAFETY);

	/* Keep safety button pressed for one second to turn off safety
	 *
	 * Note that safety cannot be turned on again by button because a button
	 * hardware problem could accidentally disable it in flight.
	 */
	if (safety_button_pressed && !_safety_btn_off) {

		if (_button_counter <= CYCLE_COUNT) {
			_button_counter++;
		}

		if (_button_counter == CYCLE_COUNT) {
			// switch safety off -> ready to arm state
			_safety_btn_off = true;
		}

	} else {
		_button_counter = 0;
	}

	CheckPairingRequest(safety_button_pressed);

	_safety_btn_prev_sate = safety_button_pressed;
}

void
SafetyButton::CheckPairingRequest(bool button_pressed)
{
	// Need to press the button 3 times within 2 seconds
	const hrt_abstime now = hrt_absolute_time();

	if (now - _pairing_start > 2_s) {
		// reset state
		_pairing_start = 0;
		_pairing_button_counter = 0;
	}

	if (!_safety_btn_prev_sate && button_pressed) {
		if (_pairing_start == 0) {
			_pairing_start = now;
		}

		++_pairing_button_counter;
	}

	if (_pairing_button_counter == 3) {
		vehicle_command_s vcmd{};
		vcmd.command = vehicle_command_s::VEHICLE_CMD_START_RX_PAIR;
		vcmd.param1 = 10.f; // GCS pairing request handled by a companion.
		vcmd.timestamp = hrt_absolute_time();
		_to_command.publish(vcmd);
		PX4_DEBUG("Sending GCS pairing request");

		led_control_s led_control{};
		led_control.led_mask = 0xff;
		led_control.mode = led_control_s::MODE_BLINK_FAST;
		led_control.color = led_control_s::COLOR_GREEN;
		led_control.num_blinks = 1;
		led_control.priority = 0;
		led_control.timestamp = hrt_absolute_time();
		_to_led_control.publish(led_control);

		tune_control_s tune_control{};
		tune_control.tune_id = tune_control_s::TUNE_ID_NOTIFY_POSITIVE;
		tune_control.volume = tune_control_s::VOLUME_LEVEL_DEFAULT;
		tune_control.timestamp = hrt_absolute_time();
		_to_tune_control.publish(tune_control);

		// reset state
		_pairing_start = 0;
		_pairing_button_counter = 0;
	}
}

void
SafetyButton::FlashButton()
{
#if defined(GPIO_LED_SAFETY)
	actuator_armed_s armed;

	if (_armed_sub.copy(&armed)) {
		// Select the appropriate LED flash pattern depending on the current arm state
		LED_PATTERN pattern = LED_PATTERN::FMU_REFUSE_TO_ARM;

		// cycle the blink state machine
		if (_safety_btn_off) {
			if (armed.armed) {
				pattern = LED_PATTERN::IO_FMU_ARMED;

			} else {
				pattern = LED_PATTERN::IO_ARMED;
			}

		} else if (armed.armed) {
			pattern = LED_PATTERN::FMU_ARMED;

		} else {
			pattern = LED_PATTERN::FMU_OK_TO_ARM;
		}

		// Turn the LED on if we have a 1 at the current bit position
		px4_arch_gpiowrite(GPIO_LED_SAFETY, !((uint16_t)pattern & (1 << (_blink_counter++ / 3))));

		if (_blink_counter > 45) {
			_blink_counter = 0;
		}
	}

#endif // GPIO_LED_SAFETY
}

void
SafetyButton::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	CheckButton();

	// control safety switch LED & publish safety topic
	if (!PX4_MFT_HW_SUPPORTED(PX4_MFT_PX4IO)) {
		FlashButton();

		const bool safety_off = _safety_btn_off || _safety_disabled;

		// publish immediately on change, otherwise at 1 Hz
		if ((hrt_elapsed_time(&_safety.timestamp) >= 1_s)
		    || (_safety.safety_off != safety_off)) {

			_safety.safety_switch_available = true;
			_safety.safety_off = safety_off;
			_safety.timestamp = hrt_absolute_time();

			_to_safety.publish(_safety);
		}
	}
}

int
SafetyButton::task_spawn(int argc, char *argv[])
{
	SafetyButton *instance = new SafetyButton();

	if (!instance) {
		PX4_ERR("alloc failed");
		return -1;
	}

	int ret = instance->Start();

	if (ret != PX4_OK) {
		delete instance;
		return ret;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	return ret;
}

int
SafetyButton::Start()
{
	ScheduleOnInterval(33_ms); // run at 30 Hz

	return PX4_OK;
}

int
SafetyButton::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int
SafetyButton::print_status()
{
	PX4_INFO("Safety Disabled: %s", _safety_disabled ? "yes" : "no");
	PX4_INFO("Safety State (from button): %s", _safety_btn_off ? "off" : "on");

	return 0;
}

int
SafetyButton::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module is responsible for the safety button.
Pressing the safety button 3 times quickly will trigger a GCS pairing request.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("safety_button", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int safety_button_main(int argc, char *argv[]);

int
safety_button_main(int argc, char *argv[])
{
	return SafetyButton::main(argc, argv);
}
