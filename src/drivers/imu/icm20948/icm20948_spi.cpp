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
 * @file icm20948_spi.cpp
 *
 * Driver for the Invensense ICM20948 connected via SPI.
 *
 * @author Andrew Tridgell
 * @author Pat Hickey
 * @author David sidrane
 */

#include <drivers/device/spi.h>
#include "icm20948.h"

#define DIR_READ			0x80
#define DIR_WRITE			0x00

/*
 * The ICM20948 can only handle high SPI bus speeds of 20Mhz on the sensor and
 * interrupt status registers. All other registers have a maximum 1MHz
 * SPI speed
 *
 * The Actual Value will be rounded down by the spi driver.
 * for a 168Mhz CPU this will be 10.5 Mhz and for a 180 Mhz CPU
 * it will be 11.250 Mhz
 */
#define ICM20948_LOW_SPI_BUS_SPEED	1000*1000
#define ICM20948_HIGH_SPI_BUS_SPEED	20*1000*1000

device::Device *ICM20948_SPI_interface(int bus, uint32_t cs);

class ICM20948_SPI : public device::SPI
{
public:
	ICM20948_SPI(int bus, uint32_t device);
	~ICM20948_SPI() override = default;

	int	read(unsigned address, void *data, unsigned count) override;
	int	write(unsigned address, void *data, unsigned count) override;

protected:
	int probe() override;

private:

	/* Helper to set the desired speed and isolate the register on return */
	void set_bus_frequency(unsigned &reg_speed_reg_out);
};

device::Device *
ICM20948_SPI_interface(int bus, uint32_t cs)
{
	device::Device *interface = nullptr;

	interface = new ICM20948_SPI(bus, cs);

	return interface;
}

ICM20948_SPI::ICM20948_SPI(int bus, uint32_t device) :
	SPI("ICM20948", nullptr, bus, device, SPIDEV_MODE3, ICM20948_LOW_SPI_BUS_SPEED)
{
	_device_id.devid_s.devtype = DRV_IMU_DEVTYPE_ICM20948;
}

void
ICM20948_SPI::set_bus_frequency(unsigned &reg_speed)
{
	/* Set the desired speed */
	set_frequency(ICM20948_IS_HIGH_SPEED(reg_speed) ? ICM20948_HIGH_SPI_BUS_SPEED : ICM20948_LOW_SPI_BUS_SPEED);

	/* Isoolate the register on return */
	reg_speed = ICM20948_REG(reg_speed);
}

int
ICM20948_SPI::write(unsigned reg_speed, void *data, unsigned count)
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
ICM20948_SPI::read(unsigned reg_speed, void *data, unsigned count)
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
ICM20948_SPI::probe()
{
	uint8_t whoami = 0;

	int ret = read(MPUREG_WHOAMI, &whoami, 1);

	if (ret != OK) {
		return -EIO;
	}

	switch (whoami) {
	case ICM_WHOAMI_20948:
		ret = 0;
		break;

	default:
		PX4_WARN("probe failed! %u", whoami);
		ret = -EIO;
	}

	return ret;
}
