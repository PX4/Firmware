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

/**
 * @file lis2mdl_i2c.cpp
 *
 * I2C interface for LIS2MDL
 */

#include <px4_platform_common/px4_config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

#include <drivers/device/i2c.h>
#include <drivers/drv_device.h>

#include "board_config.h"
#include "lis2mdl.h"

#define LIS2MDLL_ADDRESS        0x1e

class LIS2MDL_I2C : public device::I2C
{
public:
	LIS2MDL_I2C(int bus, int bus_frequency);
	virtual ~LIS2MDL_I2C() = default;

	virtual int     read(unsigned address, void *data, unsigned count);
	virtual int     write(unsigned address, void *data, unsigned count);

protected:
	virtual int     probe();

};

device::Device *
LIS2MDL_I2C_interface(int bus, int bus_frequency);

device::Device *
LIS2MDL_I2C_interface(int bus, int bus_frequency)
{
	return new LIS2MDL_I2C(bus, bus_frequency);
}

LIS2MDL_I2C::LIS2MDL_I2C(int bus, int bus_frequency) :
	I2C(DRV_MAG_DEVTYPE_LIS2MDL, "LIS2MDL_I2C", bus, LIS2MDLL_ADDRESS, bus_frequency)
{
}

int
LIS2MDL_I2C::probe()
{
	uint8_t data = 0;

	_retries = 10;

	if (read(ADDR_WHO_AM_I, &data, 1)) {
		DEVICE_DEBUG("read_reg fail");
		return -EIO;
	}

	_retries = 2;

	if (data != ID_WHO_AM_I) {
		DEVICE_DEBUG("LIS2MDL bad ID: %02x", data);
		return -EIO;
	}

	return OK;
}

int
LIS2MDL_I2C::read(unsigned address, void *data, unsigned count)
{
	uint8_t cmd = address;
	return transfer(&cmd, 1, (uint8_t *)data, count);
}

int
LIS2MDL_I2C::write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address;
	memcpy(&buf[1], data, count);

	return transfer(&buf[0], count + 1, nullptr, 0);
}
