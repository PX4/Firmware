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
 * @file mpu9250_spi.cpp
 *
 * Driver for the Invensense MPU9250 connected via SPI.
 *
 * @author Andrew Tridgell
 * @author Pat Hickey
 * @author David sidrane
 */

#include <drivers/device/spi.h>
#include "mpu9250.h"

#define DIR_READ			0x80
#define DIR_WRITE			0x00

/*
 * The MPU9250 can only handle high SPI bus speeds of 20Mhz on the sensor and
 * interrupt status registers. All other registers have a maximum 1MHz
 * SPI speed
 *
 * The Actual Value will be rounded down by the spi driver.
 * for a 168Mhz CPU this will be 10.5 Mhz and for a 180 Mhz CPU
 * it will be 11.250 Mhz
 */
#define MPU9250_LOW_SPI_BUS_SPEED	1000*1000
#define MPU9250_HIGH_SPI_BUS_SPEED	20*1000*1000

device::Device *MPU9250_SPI_interface(int bus, uint32_t cs, int bus_frequency, spi_mode_e spi_mode);

class MPU9250_SPI : public device::SPI
{
public:
	MPU9250_SPI(int bus, uint32_t device, int bus_frequency, spi_mode_e spi_mode);
	~MPU9250_SPI() override = default;

	int	read(unsigned address, void *data, unsigned count) override;
	int	write(unsigned address, void *data, unsigned count) override;

protected:
	int probe() override;

private:
	/* Helper to set the desired speed and isolate the register on return */
	void set_bus_frequency(unsigned &reg_speed_reg_out);

	const int _high_bus_speed;
};

device::Device *
MPU9250_SPI_interface(int bus, uint32_t cs, int bus_frequency, spi_mode_e spi_mode)
{
	return new MPU9250_SPI(bus, cs, bus_frequency, spi_mode);
}

MPU9250_SPI::MPU9250_SPI(int bus, uint32_t device, int bus_frequency, spi_mode_e spi_mode) :
	SPI(DRV_IMU_DEVTYPE_MPU9250, MODULE_NAME, bus, device, spi_mode, MPU9250_LOW_SPI_BUS_SPEED),
	_high_bus_speed(bus_frequency)
{
}

void
MPU9250_SPI::set_bus_frequency(unsigned &reg_speed)
{
	/* Set the desired speed */
	set_frequency(MPU9250_IS_HIGH_SPEED(reg_speed) ? _high_bus_speed : MPU9250_LOW_SPI_BUS_SPEED);

	/* Isolate the register on return */
	reg_speed = MPU9250_REG(reg_speed);
}

int
MPU9250_SPI::write(unsigned reg_speed, void *data, unsigned count)
{
	uint8_t cmd[2] {};

	if (sizeof(cmd) < (count + 1)) {
		return -EIO;
	}

	/* Set the desired speed and isolate the register */
	set_bus_frequency(reg_speed);

	cmd[0] = reg_speed | DIR_WRITE;
	cmd[1] = *(uint8_t *)data;

	return transfer(&cmd[0], &cmd[0], count + 1);
}

int
MPU9250_SPI::read(unsigned reg_speed, void *data, unsigned count)
{
	/* We want to avoid copying the data of MPUReport: So if the caller
	 * supplies a buffer not MPUReport in size, it is assume to be a reg or reg 16 read
	 * and we need to provied the buffer large enough for the callers data
	 * and our command.
	 */
	uint8_t cmd[3] {};

	uint8_t *pbuff  =  count < sizeof(MPUReport) ? cmd : (uint8_t *) data ;

	if (count < sizeof(MPUReport))  {
		/* add command */
		count++;
	}

	set_bus_frequency(reg_speed);

	/* Set command */
	pbuff[0] = reg_speed | DIR_READ ;

	/* Transfer the command and get the data */
	int ret = transfer(pbuff, pbuff, count);

	if (ret == OK && pbuff == &cmd[0]) {
		/* Adjust the count back */
		count--;

		/* Return the data */
		memcpy(data, &cmd[1], count);
	}

	return ret;
}

int
MPU9250_SPI::probe()
{
	uint8_t whoami = 0;

	int ret = read(MPUREG_WHOAMI, &whoami, 1);

	if (ret != OK) {
		return -EIO;
	}

	switch (whoami) {
	case MPU_WHOAMI_9250:
	case MPU_WHOAMI_6500:
		ret = 0;
		break;

	default:
		PX4_WARN("probe failed! %u", whoami);
		ret = -EIO;
	}

	return ret;
}
