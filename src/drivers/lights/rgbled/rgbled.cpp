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

/**
 * @file rgbled.cpp
 *
 * Driver for the onboard RGB LED controller (TCA62724FMG) connected via I2C.
 *
 * @author Julian Oes <julian@px4.io>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <string.h>

#include <drivers/device/i2c.h>
#include <lib/led/led.h>
#include <lib/parameters/param.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>

using namespace time_literals;

#define ADDR			0x55	/**< I2C adress of TCA62724FMG */
#define SUB_ADDR_START		0x01	/**< write everything (with auto-increment) */
#define SUB_ADDR_PWM0		0x81	/**< blue     (without auto-increment) */
#define SUB_ADDR_PWM1		0x82	/**< green    (without auto-increment) */
#define SUB_ADDR_PWM2		0x83	/**< red      (without auto-increment) */
#define SUB_ADDR_SETTINGS	0x84	/**< settings (without auto-increment)*/

#define SETTING_NOT_POWERSAVE	0x01	/**< power-save mode not off */
#define SETTING_ENABLE   	0x02	/**< on */


class RGBLED : public device::I2C, public I2CSPIDriver<RGBLED>
{
public:
	RGBLED(const I2CSPIDriverConfig &config);
	virtual ~RGBLED() = default;

	static void print_usage();

	int		init() override;
	int		probe() override;

	void			RunImpl();

protected:
	void			print_status() override;
private:

	float			_brightness{1.0f};
	float			_max_brightness{1.0f};

	uint8_t			_r{0};
	uint8_t			_g{0};
	uint8_t			_b{0};
	bool			_leds_enabled{true};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	LedController		_led_controller;

	int			send_led_enable(bool enable);
	int			send_led_rgb();
	int			get(bool &on, bool &powersave, uint8_t &r, uint8_t &g, uint8_t &b);
	void			update_params();
};

RGBLED::RGBLED(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config)
{
}

int
RGBLED::init()
{
	int ret = I2C::init();

	if (ret != OK) {
		return ret;
	}

	/* switch off LED on start */
	send_led_enable(false);
	send_led_rgb();

	update_params();

	// kick off work queue
	ScheduleNow();

	return OK;
}

int
RGBLED::probe()
{
	int ret;
	bool on, powersave;
	uint8_t r, g, b;

	if ((ret = get(on, powersave, r, g, b)) != OK ||
	    (ret = send_led_enable(false) != OK) ||
	    (ret = send_led_enable(false) != OK)) {
		return ret;
	}

	_retries = 1;

	return ret;
}

void
RGBLED::print_status()
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
}

void
RGBLED::RunImpl()
{
	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		update_params();

		// Immediately update to change brightness
		send_led_rgb();
	}

	LedControlData led_control_data;

	if (_led_controller.update(led_control_data) == 1) {
		switch (led_control_data.leds[0].color) {
		case led_control_s::COLOR_RED:
			_r = 255; _g = 0; _b = 0;
			send_led_enable(true);
			break;

		case led_control_s::COLOR_GREEN:
			_r = 0; _g = 255; _b = 0;
			send_led_enable(true);
			break;

		case led_control_s::COLOR_BLUE:
			_r = 0; _g = 0; _b = 255;
			send_led_enable(true);
			break;

		case led_control_s::COLOR_AMBER: //make it the same as yellow
		case led_control_s::COLOR_YELLOW:
			_r = 255; _g = 255; _b = 0;
			send_led_enable(true);
			break;

		case led_control_s::COLOR_PURPLE:
			_r = 255; _g = 0; _b = 255;
			send_led_enable(true);
			break;

		case led_control_s::COLOR_CYAN:
			_r = 0; _g = 255; _b = 255;
			send_led_enable(true);
			break;

		case led_control_s::COLOR_WHITE:
			_r = 255; _g = 255; _b = 255;
			send_led_enable(true);
			break;

		default: // led_control_s::COLOR_OFF
			_r = 0; _g = 0; _b = 0;
			send_led_enable(false);
			break;
		}

		_brightness = (float)led_control_data.leds[0].brightness / 255.f;

		send_led_rgb();
	}

	/* re-queue ourselves to run again later */
	ScheduleDelayed(_led_controller.maximum_update_interval());
}

/**
 * Sent ENABLE flag to LED driver
 */
int
RGBLED::send_led_enable(bool enable)
{
	if (_leds_enabled && enable) {
		// already enabled
		return 0;
	}

	_leds_enabled = enable;
	uint8_t settings_byte = 0;

	if (enable) {
		settings_byte |= SETTING_ENABLE;
	}

	settings_byte |= SETTING_NOT_POWERSAVE;

	const uint8_t msg[2] = { SUB_ADDR_SETTINGS, settings_byte};

	return transfer(msg, sizeof(msg), nullptr, 0);
}

/**
 * Send RGB PWM settings to LED driver according to current color and brightness
 */
int
RGBLED::send_led_rgb()
{
	/* To scale from 0..255 -> 0..15 shift right by 4 bits */
	const uint8_t msg[6] = {
		SUB_ADDR_PWM0, static_cast<uint8_t>((_b >> 4) * _brightness * _max_brightness + 0.5f),
		SUB_ADDR_PWM1, static_cast<uint8_t>((_g >> 4) * _brightness * _max_brightness + 0.5f),
		SUB_ADDR_PWM2, static_cast<uint8_t>((_r >> 4) * _brightness * _max_brightness + 0.5f)
	};
	return transfer(msg, sizeof(msg), nullptr, 0);
}

int
RGBLED::get(bool &on, bool &powersave, uint8_t &r, uint8_t &g, uint8_t &b)
{
	uint8_t result[2] = {0, 0};
	int ret;

	ret = transfer(nullptr, 0, &result[0], 2);

	if (ret == OK) {
		on = ((result[0] >> 4) & SETTING_ENABLE);
		powersave = !((result[0] >> 4) & SETTING_NOT_POWERSAVE);
		/* XXX check, looks wrong */
		r = (result[0] & 0x0f) << 4;
		g = (result[1] & 0xf0);
		b = (result[1] & 0x0f) << 4;
	}

	return ret;
}

void
RGBLED::update_params()
{
	int32_t maxbrt = 15;
	param_get(param_find("LED_RGB_MAXBRT"), &maxbrt);
	maxbrt = maxbrt > 15 ? 15 : maxbrt;
	maxbrt = maxbrt <  0 ?  0 : maxbrt;

	// A minimum of 2 "on" steps is required for breathe effect
	if (maxbrt == 1) {
		maxbrt = 2;
	}

	_max_brightness = maxbrt / 15.0f;
}

void
RGBLED::print_usage()
{
	PRINT_MODULE_USAGE_NAME("rgbled", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x55);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int rgbled_main(int argc, char *argv[])
{
	using ThisDriver = RGBLED;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 100000;
	cli.i2c_address = ADDR;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_LED_DEVTYPE_RGBLED);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
