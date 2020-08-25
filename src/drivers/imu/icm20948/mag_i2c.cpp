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
 * I2C interface for AK09916
 */

#include "icm20948.h"
#include "ICM20948_mag.h"

#include <drivers/device/i2c.h>

#ifdef USE_I2C

device::Device *AK09916_I2C_interface(int bus);

class AK09916_I2C : public device::I2C
{
public:
	AK09916_I2C(int bus);
	~AK09916_I2C() override = default;

	int	read(unsigned address, void *data, unsigned count) override;
	int	write(unsigned address, void *data, unsigned count) override;

protected:
	int	probe() override;

};

device::Device *
AK09916_I2C_interface(int bus)
{
	return new AK09916_I2C(bus);
}

AK09916_I2C::AK09916_I2C(int bus) :
	I2C("AK09916_I2C", nullptr, bus, AK09916_I2C_ADDR, 400000)
{
	_device_id.devid_s.devtype = DRV_IMU_DEVTYPE_ICM20948;
}

int
AK09916_I2C::write(unsigned reg_speed, void *data, unsigned count)
{
	uint8_t cmd[2] {};

	if (sizeof(cmd) < (count + 1)) {
		return -EIO;
	}

	cmd[0] = ICM20948_REG(reg_speed);
	cmd[1] = *(uint8_t *)data;
	return transfer(&cmd[0], count + 1, nullptr, 0);
}

int
AK09916_I2C::read(unsigned reg_speed, void *data, unsigned count)
{
	uint8_t cmd = ICM20948_REG(reg_speed);
	return transfer(&cmd, 1, (uint8_t *)data, count);
}

int
AK09916_I2C::probe()
{
	uint8_t whoami = 0;
	uint8_t expected = AK09916_DEVICE_ID;

	if (PX4_OK != read(AK09916REG_WIA, &whoami, 1)) {
		return -EIO;
	}

	if (whoami != expected) {
		return -EIO;
	}

	return OK;
}

#endif // USE_I2C
