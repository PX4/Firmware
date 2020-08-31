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
 * @file LPS22HB_SPI.cpp
 *
 * SPI interface for LPS22HB
 */

#include "LPS22HB.hpp"

#include <drivers/device/spi.h>

/* SPI protocol address bits */
#define DIR_READ			(1<<7)
#define DIR_WRITE			(0<<7)

device::Device *LPS22HB_SPI_interface(int bus, uint32_t devid, int bus_frequency, spi_mode_e spi_mode);

class LPS22HB_SPI : public device::SPI
{
public:
	LPS22HB_SPI(int bus, uint32_t device, int bus_frequency, spi_mode_e spi_mode);
	~LPS22HB_SPI() override = default;

	int	read(unsigned address, void *data, unsigned count) override;
	int	write(unsigned address, void *data, unsigned count) override;

protected:
	int	probe() override;
};

device::Device *
LPS22HB_SPI_interface(int bus, uint32_t devid, int bus_frequency, spi_mode_e spi_mode)
{
	return new LPS22HB_SPI(bus, devid, bus_frequency, spi_mode);
}

LPS22HB_SPI::LPS22HB_SPI(int bus, uint32_t device, int bus_frequency, spi_mode_e spi_mode) :
	SPI(DRV_BARO_DEVTYPE_LPS22HB, MODULE_NAME, bus, device, spi_mode, bus_frequency)
{
}

int LPS22HB_SPI::probe()
{
	// read WHO_AM_I value
	uint8_t id = 0;

	if (read(WHO_AM_I, &id, 1)) {
		DEVICE_DEBUG("read_reg fail");
		return -EIO;
	}

	if (id != LPS22HB_ID_WHO_AM_I) {
		DEVICE_DEBUG("ID byte mismatch (%02x != %02x)", LPS22HB_ID_WHO_AM_I, id);
		return -EIO;
	}

	return PX4_OK;
}

int LPS22HB_SPI::write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address | DIR_WRITE;
	memcpy(&buf[1], data, count);

	return transfer(&buf[0], &buf[0], count + 1);
}

int LPS22HB_SPI::read(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address | DIR_READ;

	int ret = transfer(&buf[0], &buf[0], count + 1);
	memcpy(data, &buf[1], count);
	return ret;
}
