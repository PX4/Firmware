/****************************************************************************
 *
 *   Copyright (c) 2016-2019 PX4 Development Team. All rights reserved.
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
 * @file mpu9250_i2c.cpp
 *
 * I2C interface for MPU9250
 */

#include <drivers/device/i2c.h>

#include "mpu9250.h"

#ifdef USE_I2C

device::Device *MPU9250_I2C_interface(int bus, uint32_t address, int bus_frequency);

class MPU9250_I2C : public device::I2C
{
public:
	MPU9250_I2C(int bus, uint32_t address, int bus_frequency);
	~MPU9250_I2C() override = default;

	int	read(unsigned address, void *data, unsigned count) override;
	int	write(unsigned address, void *data, unsigned count) override;

protected:
	virtual int	probe() override;

private:

};

device::Device *
MPU9250_I2C_interface(int bus, uint32_t address, int bus_frequency)
{
	return new MPU9250_I2C(bus, address, bus_frequency);
}

MPU9250_I2C::MPU9250_I2C(int bus, uint32_t address, int bus_frequency) :
	I2C(DRV_IMU_DEVTYPE_MPU9250, MODULE_NAME, bus, address, bus_frequency)
{
}

int
MPU9250_I2C::write(unsigned reg_speed, void *data, unsigned count)
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
MPU9250_I2C::read(unsigned reg_speed, void *data, unsigned count)
{
	/* We want to avoid copying the data of MPUReport: So if the caller
	 * supplies a buffer not MPUReport in size, it is assume to be a reg or
	 * reg 16 read
	 * Since MPUReport has a cmd at front, we must return the data
	 * after that. Foe anthing else we must return it
	 */
	uint32_t offset = count < sizeof(MPUReport) ? 0 : offsetof(MPUReport, ACCEL_XOUT_H);
	uint8_t cmd = MPU9250_REG(reg_speed);
	return transfer(&cmd, 1, &((uint8_t *)data)[offset], count - offset);
}

int
MPU9250_I2C::probe()
{
	uint8_t whoami = 0;

	// Try first for mpu9250/6500
	read(MPUREG_WHOAMI, &whoami, 1);

	if (whoami == MPU_WHOAMI_9250 || whoami == MPU_WHOAMI_6500) {
		return PX4_OK;
	}

	return -ENODEV;
}

#endif /* USE_I2C */
