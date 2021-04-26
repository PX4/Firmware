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

#pragma once

#include <drivers/device/i2c.h>

#include <nuttx/ioexpander/gpio.h>

using namespace time_literals;

namespace Microchip_MCP23009
{
enum class Register : uint8_t;
}

class MCP23009 : public device::I2C
{
public:
	MCP23009(int bus, int address, int first_minor = 0, int bus_frequency = 400000);
	virtual ~MCP23009();

	int init(uint8_t direction, uint8_t intital = 0, uint8_t pull_up = 0);

protected:
	int probe() override;

private:
	static constexpr int num_gpios = 8;
	static const gpio_operations_s	gpio_ops;

	struct mcp23009_gpio_dev_s {
		struct gpio_dev_s gpio;
		uint8_t id;
		MCP23009 *obj;
	};

	static int go_read(struct gpio_dev_s *dev, bool *value);
	static int go_write(struct gpio_dev_s *dev, bool value);
	static int go_setpintype(struct gpio_dev_s *dev, enum gpio_pintype_e pintype);

	int go_read(int id, bool *value);
	int go_write(int id, bool value);
	int go_setpintype(int id, enum gpio_pintype_e pintype);

	int read_reg(Microchip_MCP23009::Register address, uint8_t &data);
	int write_reg(Microchip_MCP23009::Register address, uint8_t data);

	const int _first_minor;
	mcp23009_gpio_dev_s _gpio[num_gpios] {};
};
