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

#include <drivers/drv_sensor.h>
#include <px4_platform/gpio/mcp23009.hpp>
#include "mcp23009_registers.hpp"

using namespace Microchip_MCP23009;

const struct gpio_operations_s MCP23009::gpio_ops = {
go_read : MCP23009::go_read,
go_write : MCP23009::go_write,
go_attach : nullptr,
go_enable : nullptr,
go_setpintype : MCP23009::go_setpintype,
};

MCP23009::MCP23009(int bus, int address, int first_minor, int bus_frequency) :
	I2C(DRV_GPIO_DEVTYPE_MCP23009, "MCP23009", bus, address, bus_frequency),
	_first_minor(first_minor)
{
}

MCP23009::~MCP23009()
{
	/* set all as input & unregister */
	for (int i = 0; i < num_gpios; ++i) {
		go_setpintype(i, GPIO_INPUT_PIN);
		gpio_pin_unregister(&_gpio[i].gpio, _first_minor + i);
	}
}

int MCP23009::go_read(struct gpio_dev_s *dev, bool *value)
{
	mcp23009_gpio_dev_s *gpio = (struct mcp23009_gpio_dev_s *)dev;
	return gpio->obj->go_read(gpio->id, value);
}

int MCP23009::go_write(struct gpio_dev_s *dev, bool value)
{
	mcp23009_gpio_dev_s *gpio = (struct mcp23009_gpio_dev_s *)dev;
	return gpio->obj->go_write(gpio->id, value);
}

int MCP23009::go_setpintype(struct gpio_dev_s *dev, enum gpio_pintype_e pintype)
{
	mcp23009_gpio_dev_s *gpio = (struct mcp23009_gpio_dev_s *)dev;
	return gpio->obj->go_setpintype(gpio->id, pintype);
}


int MCP23009::read_reg(Register address, uint8_t &data)
{
	return transfer((uint8_t *)&address, 1, &data, 1);
}

int MCP23009::write_reg(Register address, uint8_t value)
{
	uint8_t data[2] = {(uint8_t)address, value};
	return transfer(data, sizeof(data), nullptr, 0);
}

int MCP23009::init(uint8_t direction, uint8_t intital, uint8_t pull_up)
{
	/* do I2C init (and probe) first */
	int ret = I2C::init();

	if (ret != PX4_OK) {
		return ret;
	}

	/* Use this state as the out puts */

	ret = write_reg(Register::OLAT, intital);
	ret |= write_reg(Register::IODIR, direction);
	ret |= write_reg(Register::GPPU, pull_up);

	if (ret != PX4_OK) {
		return ret;
	}

	/* register the pins */
	for (int i = 0; i < num_gpios; ++i) {
		_gpio[i].gpio.gp_pintype = GPIO_INPUT_PIN;
		_gpio[i].gpio.gp_ops = &gpio_ops;
		_gpio[i].id = i;
		_gpio[i].obj = this;
		ret = gpio_pin_register(&_gpio[i].gpio, _first_minor + i);

		if (ret != PX4_OK) {
			return ret;
		}
	}

	return ret;
}

int MCP23009::probe()
{
	// no whoami, try to read IOCON
	uint8_t data;
	return read_reg(Register::IOCON, data);
}

int MCP23009::go_read(int id, bool *value)
{

	uint8_t data;
	int ret = read_reg(Register::GPIO, data);

	if (ret != 0) {
		return ret;
	}

	*value = data & (1 << id);
	return 0;
}

int MCP23009::go_write(int id, bool value)
{
	uint8_t data;
	int ret = read_reg(Register::GPIO, data);

	if (ret != 0) {
		return ret;
	}

	if (value) {
		data |= (1 << id);

	} else {
		data &= ~(1 << id);
	}

	return write_reg(Register::GPIO, data);
}

int MCP23009::go_setpintype(int id, enum gpio_pintype_e pintype)
{
	uint8_t direction;
	int ret = read_reg(Register::IODIR, direction);

	if (ret != 0) {
		return ret;
	}

	uint8_t pullup;
	ret = read_reg(Register::GPPU, pullup);

	if (ret != 0) {
		return ret;
	}

	switch (pintype) {
	case GPIO_INPUT_PIN:
		direction |= (1 << id);
		pullup &= ~(1 << id);
		break;

	case GPIO_INPUT_PIN_PULLUP:
		direction |= (1 << id);
		pullup |= (1 << id);
		break;

	case GPIO_OUTPUT_PIN:
		direction &= ~(1 << id);
		break;

	default:
		return -EINVAL;
	}

	_gpio[id].gpio.gp_pintype = pintype;

	ret = write_reg(Register::GPPU, pullup);

	if (ret != 0) {
		return ret;
	}

	return write_reg(Register::IODIR, direction);
}
