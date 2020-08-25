/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file mpu6000_spi.cpp
 *
 * Driver for the Invensense MPU6000 connected via SPI.
 *
 * @author Andrew Tridgell
 * @author Pat Hickey
 * @author David sidrane
 */

#include <drivers/device/spi.h>

#include "MPU6000.hpp"

#define DIR_READ			0x80
#define DIR_WRITE			0x00


/* The MPU6000 can only handle high SPI bus speeds of 20Mhz on the sensor and
* interrupt status registers. All other registers have a maximum 1MHz
* SPI speed
*
* The ICM parts are not rated as high.
*
* The Actual Value will be rounded down by the spi driver.
*          168 Mhz CPU         180 Mhz CPU
* Selected ------------actual---------------
* 20 Mhz   10.5     Mhz         11.250   Mhz
* 10 Mhz   5.250    Mhz         5.625    Mhz
* 8  Mhz   5.250    Mhz         5.625    Mhz
* 1  Mhz   0.703125 Mhz         0.65625  Mhz
*
*/
#define MPU6000_LOW_SPI_BUS_SPEED    1000*1000
#define MPU6000_HIGH_SPI_BUS_SPEED   20*1000*1000
#define ICM20608_HIGH_SPI_BUS_SPEED  8*1000*1000
#define ICM20689_HIGH_SPI_BUS_SPEED  8*1000*1000
#define ICM20602_HIGH_SPI_BUS_SPEED 10*1000*1000
#define UNKNOWN_HIGH_SPI_BUS_SPEED   8*1000*1000 // Use the minimum


device::Device *MPU6000_SPI_interface(int bus, uint32_t devid, int device_type, bool external_bus, int bus_frequency,
				      spi_mode_e spi_mode);


class MPU6000_SPI : public device::SPI
{
public:
	MPU6000_SPI(int bus, uint32_t device, int device_type, int bus_frequency, spi_mode_e spi_mode);
	~MPU6000_SPI() override = default;

	int	read(unsigned address, void *data, unsigned count) override;
	int	write(unsigned address, void *data, unsigned count) override;

protected:
	int probe() override;

private:

	int _device_type;
	/* Helper to set the desired speed and isolate the register on return */

	int _max_frequency;
	void set_bus_frequency(unsigned &reg_speed_reg_out);
};

device::Device *
MPU6000_SPI_interface(int bus, uint32_t devid, int device_type, bool external_bus, int bus_frequency,
		      spi_mode_e spi_mode)
{
	return new MPU6000_SPI(bus, devid, device_type, bus_frequency, spi_mode);
}

MPU6000_SPI::MPU6000_SPI(int bus, uint32_t device, int device_type, int bus_frequency, spi_mode_e spi_mode) :
	SPI("MPU6000", nullptr, bus, device, spi_mode, bus_frequency),
	_device_type(device_type)
{
}

void
MPU6000_SPI::set_bus_frequency(unsigned &reg_speed)
{
	/* Set the desired speed */
	set_frequency(MPU6000_IS_HIGH_SPEED(reg_speed) ? _max_frequency : MPU6000_LOW_SPI_BUS_SPEED);

	/* Isolate the register on return */
	reg_speed = MPU6000_REG(reg_speed);
}

int
MPU6000_SPI::write(unsigned reg_speed, void *data, unsigned count)
{
	uint8_t cmd[MPU_MAX_WRITE_BUFFER_SIZE];

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
MPU6000_SPI::read(unsigned reg_speed, void *data, unsigned count)
{
	/* We want to avoid copying the data of MPUReport: So if the caller
	 * supplies a buffer not MPUReport in size, it is assume to be a reg or reg 16 read
	 * and we need to provied the buffer large enough for the callers data
	 * and our command.
	 */
	uint8_t cmd[3] = {0, 0, 0};

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

	return ret == OK ? count : ret;
}

int
MPU6000_SPI::probe()
{
	uint8_t whoami = 0;
	uint8_t expected = MPU_WHOAMI_6000;
	_max_frequency = UNKNOWN_HIGH_SPI_BUS_SPEED;

	switch (_device_type) {

	default:
	case MPU_DEVICE_TYPE_MPU6000:
		_max_frequency = MPU6000_HIGH_SPI_BUS_SPEED;
		break;

	case MPU_DEVICE_TYPE_ICM20602:
		expected = ICM_WHOAMI_20602;
		_max_frequency = ICM20602_HIGH_SPI_BUS_SPEED;
		break;

	case MPU_DEVICE_TYPE_ICM20608:
		expected = ICM_WHOAMI_20608;
		_max_frequency = ICM20608_HIGH_SPI_BUS_SPEED;
		break;

	case MPU_DEVICE_TYPE_ICM20689:
		expected = ICM_WHOAMI_20689;
		_max_frequency = ICM20689_HIGH_SPI_BUS_SPEED;
		break;
	}

	return (read(MPUREG_WHOAMI, &whoami, 1) > 0 && (whoami == expected)) ? 0 : -EIO;
}

