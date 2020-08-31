/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file mag_i2c.cpp
 *
 * I2C interface for AK8963
 */

#include <px4_platform_common/px4_config.h>
#include <drivers/device/i2c.h>
#include <drivers/drv_device.h>

#include "mpu9250.h"
#include "MPU9250_mag.h"

#ifdef USE_I2C

device::Device *AK8963_I2C_interface(int bus, int bus_frequency);

class AK8963_I2C : public device::I2C
{
public:
	AK8963_I2C(int bus, int bus_frequency);
	~AK8963_I2C() override = default;

	int	read(unsigned address, void *data, unsigned count) override;
	int	write(unsigned address, void *data, unsigned count) override;

protected:
	int	probe() override;

};

device::Device *
AK8963_I2C_interface(int bus, int bus_frequency)
{
	return new AK8963_I2C(bus, bus_frequency);
}

AK8963_I2C::AK8963_I2C(int bus, int bus_frequency) :
	I2C(DRV_MAG_DEVTYPE_AK8963, "AK8963_I2C", bus, AK8963_I2C_ADDR, bus_frequency)
{
}

int
AK8963_I2C::write(unsigned reg_speed, void *data, unsigned count)
{
	uint8_t cmd[2] {};

	if (sizeof(cmd) < (count + 1)) {
		return -EIO;
	}

	cmd[0] = MPU9250_REG(reg_speed);
	cmd[1] = *(uint8_t *)data;
	return transfer(&cmd[0], count + 1, nullptr, 0);
}

int
AK8963_I2C::read(unsigned reg_speed, void *data, unsigned count)
{
	uint8_t cmd = MPU9250_REG(reg_speed);
	return transfer(&cmd, 1, (uint8_t *)data, count);
}

int
AK8963_I2C::probe()
{
	uint8_t whoami = 0;
	uint8_t expected = AK8963_DEVICE_ID;

	if (PX4_OK != read(AK8963REG_WIA, &whoami, 1)) {
		return -EIO;
	}

	if (whoami != expected) {
		return -EIO;
	}

	return OK;
}

#endif
