/****************************************************************************
 *
 *   Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
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
 * @file rgbled_ncp5623c.cpp
 *
 * Driver for the onboard RGB LED controller (NCP5623C) connected via I2C.
 *
 * @author CUAVcaijie <caijie@cuav.net>
 */

#include <string.h>

#include <drivers/device/i2c.h>
#include <lib/led/led.h>
#include <lib/parameters/param.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>

using namespace time_literals;

#define ADDR			0x39	/**< I2C adress of NCP5623C */

#define NCP5623_LED_CURRENT	0x20	/**< Current register */
#define NCP5623_LED_PWM0	0x40	/**< pwm0 register */
#define NCP5623_LED_PWM1	0x60	/**< pwm1 register */
#define NCP5623_LED_PWM2	0x80	/**< pwm2 register */

#define NCP5623_LED_BRIGHT	0x1f	/**< full brightness */
#define NCP5623_LED_OFF		0x00	/**< off */


class RGBLED_NPC5623C : public device::I2C, public I2CSPIDriver<RGBLED_NPC5623C>
{
public:
	RGBLED_NPC5623C(I2CSPIBusOption bus_option, const int bus, int bus_frequency, const int address);
	virtual ~RGBLED_NPC5623C() = default;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	int		init() override;
	int		probe() override;

	void			RunImpl();

private:

	float			_brightness{1.0f};
	float			_max_brightness{1.0f};

	uint8_t			_r{0};
	uint8_t			_g{0};
	uint8_t			_b{0};
	volatile bool		_running{false};
	volatile bool		_should_run{true};
	bool			_leds_enabled{true};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	LedController		_led_controller;

	int			send_led_rgb();
	void			update_params();

	int			write(uint8_t reg, uint8_t data);
};

RGBLED_NPC5623C::RGBLED_NPC5623C(I2CSPIBusOption bus_option, const int bus, int bus_frequency, const int address) :
	I2C(DRV_LED_DEVTYPE_RGBLED_NCP5623C, MODULE_NAME, bus, address, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus, address)
{
}

int
RGBLED_NPC5623C::write(uint8_t reg, uint8_t data)
{
	uint8_t msg[1] = { 0x00 };
	msg[0] = ((reg & 0xe0) | (data & 0x1f));

	int ret = transfer(&msg[0], 1, nullptr, 0);

	return ret;
}

int
RGBLED_NPC5623C::init()
{
	int ret = I2C::init();

	if (ret != OK) {
		return ret;
	}

	update_params();

	_running = true;

	ScheduleNow();

	return OK;
}

int
RGBLED_NPC5623C::probe()
{
	_retries = 4;

	return write(NCP5623_LED_CURRENT, 0x00);
}

void
RGBLED_NPC5623C::RunImpl()
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
			_r = NCP5623_LED_BRIGHT; _g = 0; _b = 0;
			break;

		case led_control_s::COLOR_GREEN:
			_r = 0; _g = NCP5623_LED_BRIGHT; _b = 0;
			break;

		case led_control_s::COLOR_BLUE:
			_r = 0; _g = 0; _b = NCP5623_LED_BRIGHT;
			break;

		case led_control_s::COLOR_AMBER: //make it the same as yellow
		case led_control_s::COLOR_YELLOW:
			_r = NCP5623_LED_BRIGHT; _g = NCP5623_LED_BRIGHT; _b = 0;
			break;

		case led_control_s::COLOR_PURPLE:
			_r = NCP5623_LED_BRIGHT; _g = 0; _b = NCP5623_LED_BRIGHT;
			break;

		case led_control_s::COLOR_CYAN:
			_r = 0; _g = NCP5623_LED_BRIGHT; _b = NCP5623_LED_BRIGHT;
			break;

		case led_control_s::COLOR_WHITE:
			_r = NCP5623_LED_BRIGHT; _g = NCP5623_LED_BRIGHT; _b = NCP5623_LED_BRIGHT;
			break;

		default: // led_control_s::COLOR_OFF
			_r = 0; _g = 0; _b = 0;
			break;
		}

		_brightness = (float)led_control_data.leds[0].brightness / 255.f;
		send_led_rgb();

	}

	/* re-queue ourselves to run again later */
	ScheduleDelayed(_led_controller.maximum_update_interval());
}

/**
 * Send RGB PWM settings to LED driver according to current color and brightness
 */
int
RGBLED_NPC5623C::send_led_rgb()
{

	uint8_t msg[7] = {0x20, 0x70, 0x40, 0x70, 0x60, 0x70, 0x80};
	uint8_t brightness = 0x1f * _max_brightness;

	msg[0] = NCP5623_LED_CURRENT | (brightness & 0x1f);
	msg[2] = NCP5623_LED_PWM0 | (uint8_t(_r * _brightness) & 0x1f);
	msg[4] = NCP5623_LED_PWM1 | (uint8_t(_g * _brightness) & 0x1f);
	msg[6] = NCP5623_LED_PWM2 | (uint8_t(_b * _brightness) & 0x1f);

	return transfer(&msg[0], 7, nullptr, 0);
}

void
RGBLED_NPC5623C::update_params()
{
	int32_t maxbrt = 31;
	param_get(param_find("LED_RGB1_MAXBRT"), &maxbrt);
	maxbrt = maxbrt > 31 ? 31 : maxbrt;
	maxbrt = maxbrt <  0 ?  0 : maxbrt;

	if (maxbrt == 0) {
		maxbrt = 1;
	}

	_max_brightness = maxbrt / 31.0f;
}

void
RGBLED_NPC5623C::print_usage()
{
	PRINT_MODULE_USAGE_NAME("rgbled", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x39);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

I2CSPIDriverBase *RGBLED_NPC5623C::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
		int runtime_instance)
{
	RGBLED_NPC5623C *instance = new RGBLED_NPC5623C(iterator.configuredBusOption(), iterator.bus(), cli.bus_frequency,
			cli.i2c_address);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (instance->init() != PX4_OK) {
		delete instance;
		return nullptr;
	}

	return instance;
}

extern "C" __EXPORT int rgbled_ncp5623c_main(int argc, char *argv[])
{
	using ThisDriver = RGBLED_NPC5623C;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 100000;
	cli.i2c_address = ADDR;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli,
				     DRV_LED_DEVTYPE_RGBLED_NCP5623C);

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
