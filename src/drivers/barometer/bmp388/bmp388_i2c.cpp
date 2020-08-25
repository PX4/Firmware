/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file bmp388_i2c.cpp
 *
 * I2C interface for BMP388
 */

#include <drivers/device/i2c.h>

#include "bmp388.h"

class BMP388_I2C: public device::I2C, public IBMP388
{
public:
	BMP388_I2C(uint8_t bus, uint32_t device, int bus_frequency);
	virtual ~BMP388_I2C() = default;

	int init();

	uint8_t get_reg(uint8_t addr);
	int get_reg_buf(uint8_t addr, uint8_t *buf, uint8_t len);
	int set_reg(uint8_t value, uint8_t addr);
	calibration_s *get_calibration(uint8_t addr);

	uint32_t get_device_id() const override { return device::I2C::get_device_id(); }

	uint8_t get_device_address() const override { return device::I2C::get_device_address(); }

private:
	struct calibration_s _cal;
};

IBMP388 *bmp388_i2c_interface(uint8_t busnum, uint32_t device, int bus_frequency)
{
	return new BMP388_I2C(busnum, device, bus_frequency);
}

BMP388_I2C::BMP388_I2C(uint8_t bus, uint32_t device, int bus_frequency) :
	I2C("BMP388_I2C", nullptr, bus, device, bus_frequency)
{
}

int BMP388_I2C::init()
{
	return I2C::init();
}

uint8_t BMP388_I2C::get_reg(uint8_t addr)
{
	uint8_t cmd[2] = { (uint8_t)(addr), 0};
	transfer(&cmd[0], 1, &cmd[1], 1);

	return cmd[1];
}

int BMP388_I2C::get_reg_buf(uint8_t addr, uint8_t *buf, uint8_t len)
{
	const uint8_t cmd = (uint8_t)(addr);
	return transfer(&cmd, sizeof(cmd), buf, len);
}

int BMP388_I2C::set_reg(uint8_t value, uint8_t addr)
{
	uint8_t cmd[2] = { (uint8_t)(addr), value};
	return transfer(cmd, sizeof(cmd), nullptr, 0);
}

calibration_s *BMP388_I2C::get_calibration(uint8_t addr)
{
	const uint8_t cmd = (uint8_t)(addr);

	if (transfer(&cmd, sizeof(cmd), (uint8_t *)&_cal, sizeof(struct calibration_s)) == OK) {
		return &(_cal);

	} else {
		return nullptr;
	}
}
