/****************************************************************************
 *
 *   Copyright (c) 2015, 2016 Airmind Development Team. All rights reserved.
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
 * 3. Neither the name Airmind nor the names of its contributors may be
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
 * @file rgbled_pwm.cpp
 *
 * Driver for the onboard RGB LED controller by PWM.
 * this driver is based the PX4 led driver
 *
 */

#include <string.h>

#include <lib/led/led.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

class RGBLED_PWM : public px4::ScheduledWorkItem
{
public:
	RGBLED_PWM();
	virtual ~RGBLED_PWM();

	int			init();
	int			status();

private:

	uint8_t			_r{0};
	uint8_t			_g{0};
	uint8_t			_b{0};

	volatile bool		_running{false};
	volatile bool		_should_run{true};

	LedController		_led_controller;

	void			Run() override;

	int			send_led_rgb();
	int			get(bool &on, bool &powersave, uint8_t &r, uint8_t &g, uint8_t &b);
};

extern int led_pwm_servo_set(unsigned channel, uint8_t  value);
extern int led_pwm_servo_set_ex(uint8_t r, uint8_t g, uint8_t b, uint8_t led_count);
extern unsigned led_pwm_servo_get(unsigned channel);
extern int led_pwm_servo_init(void);
extern void led_pwm_servo_deinit(void);


/* for now, we only support one RGBLED */
namespace
{
RGBLED_PWM *g_rgbled = nullptr;
}

RGBLED_PWM::RGBLED_PWM() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

RGBLED_PWM::~RGBLED_PWM()
{
	_should_run = false;
	int counter = 0;

	while (_running && ++counter < 10) {
		px4_usleep(100000);
	}
}

int
RGBLED_PWM::init()
{
	/* switch off LED on start */
	led_pwm_servo_init();
	send_led_rgb();

	_running = true;

	// kick off work queue
	ScheduleNow();

	return OK;
}

int
RGBLED_PWM::status()
{
	bool on, powersave;
	uint8_t r, g, b;

	int ret = get(on, powersave, r, g, b);

	if (ret == OK) {
		/* we don't care about power-save mode */
		PX4_INFO("state: %s", on ? "ON" : "OFF");
		PX4_INFO("red: %u, green: %u, blue: %u", (unsigned)r, (unsigned)g, (unsigned)b);

	} else {
		PX4_WARN("failed to read led");
	}

	return ret;
}

/**
 * Main loop function
 */
void
RGBLED_PWM::Run()
{
	if (!_should_run) {
		_running = false;
		return;
	}

	LedControlData led_control_data;

	if (_led_controller.update(led_control_data) == 1) {
		uint8_t brightness = led_control_data.leds[0].brightness;

		switch (led_control_data.leds[0].color) {
		case led_control_s::COLOR_RED:
			_r = brightness; _g = 0; _b = 0;
			break;

		case led_control_s::COLOR_GREEN:
			_r = 0; _g = brightness; _b = 0;
			break;

		case led_control_s::COLOR_BLUE:
			_r = 0; _g = 0; _b = brightness;
			break;

		case led_control_s::COLOR_AMBER: //make it the same as yellow
		case led_control_s::COLOR_YELLOW:
			_r = brightness; _g = brightness; _b = 0;
			break;

		case led_control_s::COLOR_PURPLE:
			_r = brightness; _g = 0; _b = brightness;
			break;

		case led_control_s::COLOR_CYAN:
			_r = 0; _g = brightness; _b = brightness;
			break;

		case led_control_s::COLOR_WHITE:
			_r = brightness; _g = brightness; _b = brightness;
			break;

		default: // led_control_s::COLOR_OFF
			_r = 0; _g = 0; _b = 0;
			break;
		}

		send_led_rgb();
	}

	/* re-queue ourselves to run again later */
	ScheduleDelayed(_led_controller.maximum_update_interval());
}

/**
 * Send RGB PWM settings to LED driver according to current color and brightness
 */
int
RGBLED_PWM::send_led_rgb()
{
#if defined(BOARD_HAS_LED_PWM)
	led_pwm_servo_set(0, _r);
	led_pwm_servo_set(1, _g);
	led_pwm_servo_set(2, _b);
#endif

#if defined(BOARD_HAS_UI_LED_PWM)
	led_pwm_servo_set(3, _r);
	led_pwm_servo_set(4, _g);
	led_pwm_servo_set(5, _b);
#endif
	return (OK);
}

int
RGBLED_PWM::get(bool &on, bool &powersave, uint8_t &r, uint8_t &g, uint8_t &b)
{
	powersave = OK;
	on = _r > 0 || _g > 0 || _b > 0;
	r = _r;
	g = _g;
	b = _b;
	return OK;
}

static void
rgbled_usage()
{
	PX4_INFO("missing command: try 'start', 'status', 'stop'");
}

extern "C" __EXPORT int
rgbled_pwm_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	/* jump over start/off/etc and look at options first */
	while ((ch = px4_getopt(argc, argv, "a:b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'a':
			break;

		case 'b':
			break;

		default:
			rgbled_usage();
			return 1;
		}
	}

	if (myoptind >= argc) {
		rgbled_usage();
		return 1;
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		if (g_rgbled != nullptr) {
			PX4_WARN("already started");
			return 1;
		}

		if (g_rgbled == nullptr) {
			g_rgbled = new RGBLED_PWM();

			if (g_rgbled == nullptr) {
				PX4_WARN("alloc failed");
				return 1;
			}

			if (OK != g_rgbled->init()) {
				delete g_rgbled;
				g_rgbled = nullptr;
				PX4_ERR("init failed");
				return 1;
			}
		}

		return 0;
	}

	/* need the driver past this point */
	if (g_rgbled == nullptr) {
		PX4_WARN("not started");
		rgbled_usage();
		return 1;
	}

	if (!strcmp(verb, "status")) {
		g_rgbled->status();
		return 0;
	}

	if (!strcmp(verb, "stop")) {
		delete g_rgbled;
		g_rgbled = nullptr;
		return 0;
	}

	rgbled_usage();
	return 1;
}
