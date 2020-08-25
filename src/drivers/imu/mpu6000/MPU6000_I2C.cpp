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
 * @file mpu6000_i2c.cpp
 *
 * I2C interface for MPU6000 /MPU6050
 */

#include <drivers/device/i2c.h>

#include "MPU6000.hpp"

#ifdef PX4_I2C_MPU6050_ADDR

device::Device *MPU6000_I2C_interface(int bus, uint32_t devid, int device_type, bool external_bus, int bus_frequency);

class MPU6000_I2C : public device::I2C
{
public:
	MPU6000_I2C(int bus, int device_type, int bus_frequency);
	~MPU6000_I2C() override = default;

	int	read(unsigned address, void *data, unsigned count) override;
	int	write(unsigned address, void *data, unsigned count) override;

protected:
	int	probe() override;

private:
	int _device_type;

};


device::Device *
MPU6000_I2C_interface(int bus, uint32_t devid, int device_type, bool external_bus, int bus_frequency)
{
	return new MPU6000_I2C(bus, device_type, bus_frequency);
}

MPU6000_I2C::MPU6000_I2C(int bus, int device_type, int bus_frequency) :
	I2C("MPU6000_I2C", nullptr, bus, PX4_I2C_MPU6050_ADDR, bus_frequency),
	_device_type(device_type)
{
}

int
MPU6000_I2C::write(unsigned reg_speed, void *data, unsigned count)
{
	uint8_t cmd[MPU_MAX_WRITE_BUFFER_SIZE];

	if (sizeof(cmd) < (count + 1)) {
		return -EIO;
	}

	cmd[0] = MPU6000_REG(reg_speed);
	cmd[1] = *(uint8_t *)data;
	return transfer(&cmd[0], count + 1, nullptr, 0);
}

int
MPU6000_I2C::read(unsigned reg_speed, void *data, unsigned count)
{
	/* We want to avoid copying the data of MPUReport: So if the caller
	 * supplies a buffer not MPUReport in size, it is assume to be a reg or
	 * reg 16 read
	 * Since MPUReport has a cmd at front, we must return the data
	 * after that. Foe anthing else we must return it
	 */
	uint32_t offset = count < sizeof(MPUReport) ? 0 : offsetof(MPUReport, status);
	uint8_t cmd = MPU6000_REG(reg_speed);
	int ret = transfer(&cmd, 1, &((uint8_t *)data)[offset], count);
	return ret == OK ? count : ret;
}

int
MPU6000_I2C::probe()
{
	uint8_t whoami = 0;
	uint8_t expected = _device_type == MPU_DEVICE_TYPE_MPU6000 ? MPU_WHOAMI_6000 : ICM_WHOAMI_20608;
	return (read(MPUREG_WHOAMI, &whoami, 1) > 0 && (whoami == expected)) ? 0 : -EIO;

}
#else

device::Device *
MPU6000_I2C_interface(int bus, uint32_t devid, int device_type, bool external_bus, int bus_frequency)
{
	return nullptr;
}

#endif /* USE_I2C */
