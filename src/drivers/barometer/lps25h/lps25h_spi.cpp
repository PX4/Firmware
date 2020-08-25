/****************************************************************************
 *
 *   Copyright (c) 2016-2020 PX4 Development Team. All rights reserved.
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
 * @file LPS25H_SPI.cpp
 *
 * SPI interface for LPS25H
 */

#include "lps25h.h"

#include <drivers/device/spi.h>

/* SPI protocol address bits */
#define DIR_READ			(1<<7)
#define DIR_WRITE			(0<<7)

device::Device *LPS25H_SPI_interface(int bus, uint32_t devid, int bus_frequency, spi_mode_e spi_mode);

class LPS25H_SPI : public device::SPI
{
public:
	LPS25H_SPI(int bus, uint32_t device, int bus_frequency, spi_mode_e spi_mode);
	~LPS25H_SPI() override = default;

	int	init() override;
	int	read(unsigned address, void *data, unsigned count) override;
	int	write(unsigned address, void *data, unsigned count) override;

};

device::Device *LPS25H_SPI_interface(int bus, uint32_t devid, int bus_frequency, spi_mode_e spi_mode)
{
	return new LPS25H_SPI(bus, devid, bus_frequency, spi_mode);
}

LPS25H_SPI::LPS25H_SPI(int bus, uint32_t device, int bus_frequency, spi_mode_e spi_mode) :
	SPI("LPS25H_SPI", nullptr, bus, device, spi_mode, bus_frequency)
{
	set_device_type(DRV_BARO_DEVTYPE_LPS25H);
}

int LPS25H_SPI::init()
{
	int ret = SPI::init();

	if (ret != OK) {
		DEVICE_DEBUG("SPI init failed");
		return -EIO;
	}

	// read WHO_AM_I value
	uint8_t id;

	if (read(ADDR_WHO_AM_I, &id, 1)) {
		DEVICE_DEBUG("read_reg fail");
		return -EIO;
	}

	if (id != ID_WHO_AM_I) {
		DEVICE_DEBUG("ID byte mismatch (%02x != %02x)", ID_WHO_AM_I, id);
		return -EIO;
	}

	return OK;
}

int LPS25H_SPI::write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address | DIR_WRITE;
	memcpy(&buf[1], data, count);

	return transfer(&buf[0], &buf[0], count + 1);
}

int LPS25H_SPI::read(unsigned address, void *data, unsigned count)
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
