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
 * @file rm3100_i2c.cpp
 *
 * I2C interface for RM3100
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

#include "board_config.h"
#include "rm3100.h"

class RM3100_I2C : public device::I2C
{
public:
	RM3100_I2C(int bus, int bus_frequency);
	~RM3100_I2C() override = default;

	int     read(unsigned address, void *data, unsigned count) override;
	int     write(unsigned address, void *data, unsigned count) override;

protected:
	int     probe() override;
};

device::Device *
RM3100_I2C_interface(int bus, int bus_frequency);

device::Device *
RM3100_I2C_interface(int bus, int bus_frequency)
{
	return new RM3100_I2C(bus, bus_frequency);
}

RM3100_I2C::RM3100_I2C(int bus, int bus_frequency) :
	I2C(DRV_MAG_DEVTYPE_RM3100, MODULE_NAME, bus, RM3100_ADDRESS, bus_frequency)
{
}

int RM3100_I2C::probe()
{
	uint8_t data = 0;

	if (read(ADDR_REVID, &data, 1)) {
		DEVICE_DEBUG("RM3100 read_reg fail");
		return -EIO;
	}

	if (data != RM3100_REVID) {
		DEVICE_DEBUG("RM3100 bad ID: %02x", data);
		return -EIO;
	}

	_retries = 1;

	return OK;
}

int RM3100_I2C::read(unsigned address, void *data, unsigned count)
{
	uint8_t cmd = address;
	int ret;

	/* We need a first transfer where we write the register to read */
	ret = transfer(&cmd, 1, nullptr, 0);

	if (ret != OK) {
		return ret;
	}

	/* Now we read the previously selected register */
	ret = transfer(nullptr, 0, (uint8_t *)data, count);

	return ret;
}

int RM3100_I2C::write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address;
	memcpy(&buf[1], data, count);

	return transfer(&buf[0], count + 1, nullptr, 0);
}
