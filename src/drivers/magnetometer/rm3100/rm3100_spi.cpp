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
 * @file rm3100_spi.cpp
 *
 * SPI interface for RM3100
 */

#include <px4_platform_common/px4_config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

#include <drivers/device/spi.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_device.h>

#include "board_config.h"
#include "rm3100.h"

#if defined(PX4_SPIDEV_RM) || defined (PX4_SPIDEV_RM_EXT)

/* SPI protocol address bits */
#define DIR_READ        (1<<7)
#define DIR_WRITE       (0<<7)

class RM3100_SPI : public device::SPI
{
public:
	RM3100_SPI(int bus, uint32_t device);
	virtual ~RM3100_SPI() = default;

	virtual int     init();
	virtual int     ioctl(unsigned operation, unsigned &arg);
	virtual int     read(unsigned address, void *data, unsigned count);
	virtual int     write(unsigned address, void *data, unsigned count);
};

device::Device *
RM3100_SPI_interface(int bus);

device::Device *
RM3100_SPI_interface(int bus)
{
#ifdef PX4_SPIDEV_RM_EXT
	return new RM3100_SPI(bus, PX4_SPIDEV_RM_EXT);
#else
	return new RM3100_SPI(bus, PX4_SPIDEV_RM);
#endif
}

RM3100_SPI::RM3100_SPI(int bus, uint32_t device) :
	SPI("RM3100_SPI", nullptr, bus, device, SPIDEV_MODE3, 1 * 1000 * 1000 /* */)
{
	_device_id.devid_s.devtype = DRV_MAG_DEVTYPE_RM3100;
}

int
RM3100_SPI::init()
{
	int ret;

	ret = SPI::init();

	if (ret != OK) {
		DEVICE_DEBUG("SPI init failed");
		return -EIO;
	}

	// Read REV_ID value
	uint8_t data = 0;

	if (read(ADDR_REVID, &data, 1)) {
		DEVICE_DEBUG("RM3100 read_reg fail");
	}

	if (data != RM3100_REVID) {
		DEVICE_DEBUG("RM3100 ID: %02x", data);
		return -EIO;
	}

	return OK;
}

int
RM3100_SPI::ioctl(unsigned operation, unsigned &arg)
{
	int ret;

	switch (operation) {

	case MAGIOCGEXTERNAL:
		return external();

	case DEVIOCGDEVICEID:
		return CDev::ioctl(nullptr, operation, arg);

	default: {
			ret = -EINVAL;
		}
	}

	return ret;
}

int
RM3100_SPI::read(unsigned address, void *data, unsigned count)
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

int
RM3100_SPI::write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address | DIR_WRITE;
	memcpy(&buf[1], data, count);

	return transfer(&buf[0], &buf[0], count + 1);
}

#endif /* PX4_SPIDEV_RM || PX4_SPIDEV_RM_EXT */
