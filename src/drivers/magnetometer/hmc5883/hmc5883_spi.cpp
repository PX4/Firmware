/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file HMC5883_SPI.cpp
 *
 * SPI interface for HMC5983
 */

#include <px4_platform_common/px4_config.h>
#include <drivers/device/spi.h>
#include <drivers/drv_device.h>

#include "hmc5883.h"

/* SPI protocol address bits */
#define DIR_READ			(1<<7)
#define DIR_WRITE			(0<<7)
#define ADDR_INCREMENT			(1<<6)

#define HMC_MAX_SEND_LEN		4
#define HMC_MAX_RCV_LEN			8

device::Device *HMC5883_SPI_interface(int bus, uint32_t devid, int bus_frequency, spi_mode_e spi_mode);

class HMC5883_SPI : public device::SPI
{
public:
	HMC5883_SPI(int bus, uint32_t device, int bus_frequency, spi_mode_e spi_mode);
	virtual ~HMC5883_SPI() = default;

	virtual int	init();
	virtual int	read(unsigned address, void *data, unsigned count);
	virtual int	write(unsigned address, void *data, unsigned count);
};

device::Device *
HMC5883_SPI_interface(int bus, uint32_t devid, int bus_frequency, spi_mode_e spi_mode)
{
	return new HMC5883_SPI(bus, devid, bus_frequency, spi_mode);
}

HMC5883_SPI::HMC5883_SPI(int bus, uint32_t device, int bus_frequency, spi_mode_e spi_mode) :
	SPI(DRV_MAG_DEVTYPE_HMC5883, MODULE_NAME, bus, device, spi_mode, bus_frequency)
{
}

int HMC5883_SPI::init()
{
	int ret;

	ret = SPI::init();

	if (ret != OK) {
		DEVICE_DEBUG("SPI init failed");
		return -EIO;
	}

	// read WHO_AM_I value
	uint8_t data[3] = {0, 0, 0};

	if (read(ADDR_ID_A, &data[0], 1) ||
	    read(ADDR_ID_B, &data[1], 1) ||
	    read(ADDR_ID_C, &data[2], 1)) {
		DEVICE_DEBUG("read_reg fail");
	}

	if ((data[0] != ID_A_WHO_AM_I) ||
	    (data[1] != ID_B_WHO_AM_I) ||
	    (data[2] != ID_C_WHO_AM_I)) {
		DEVICE_DEBUG("ID byte mismatch (%02x,%02x,%02x)", data[0], data[1], data[2]);
		return -EIO;
	}

	return OK;
}

int HMC5883_SPI::write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address | DIR_WRITE;
	memcpy(&buf[1], data, count);

	return transfer(&buf[0], &buf[0], count + 1);
}

int HMC5883_SPI::read(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address | DIR_READ | ADDR_INCREMENT;

	int ret = transfer(&buf[0], &buf[0], count + 1);
	memcpy(data, &buf[1], count);
	return ret;
}
