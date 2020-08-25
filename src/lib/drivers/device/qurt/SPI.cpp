/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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
 * @file SPI.cpp
 *
 * Base class for devices connected via SPI.
 *
 */

#include "SPI.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "dev_fs_lib_spi.h"

#include <px4_platform_common/px4_config.h>

namespace device
{

SPI::SPI(const char *name, const char *devname, int bus, uint32_t device, enum spi_mode_e mode, uint32_t frequency) :
	CDev(name, devname),
	_device(device),
	_mode(mode),
	_frequency(frequency)
{
	DEVICE_DEBUG("SPI::SPI name = %s devname = %s", name, devname);

	// fill in _device_id fields for a SPI device
	_device_id.devid_s.bus_type = DeviceBusType_SPI;
	_device_id.devid_s.bus = bus;
	_device_id.devid_s.address = (uint8_t)device;
	// devtype needs to be filled in by the driver
	_device_id.devid_s.devtype = 0;
}

SPI::~SPI()
{
	if (_fd >= 0) {
		::close(_fd);
		_fd = -1;
	}
}

int
SPI::init()
{
	// Open the actual SPI device
	char dev_path[16];
	snprintf(dev_path, sizeof(dev_path), DEV_FS_SPI_DEVICE_TYPE_STRING"%lu", PX4_SPI_DEV_ID(_device));
	DEVICE_DEBUG("%s", dev_path);
	_fd = ::open(dev_path, O_RDWR);

	if (_fd < 0) {
		PX4_ERR("could not open %s", dev_path);
		return PX4_ERROR;
	}

	/* call the probe function to check whether the device is present */
	int ret = probe();

	if (ret != OK) {
		DEVICE_DEBUG("probe failed");
		return ret;
	}

	/* do base class init, which will create the device node, etc. */
	ret = CDev::init();

	if (ret != OK) {
		DEVICE_DEBUG("cdev init failed");
		return ret;
	}

	/* tell the workd where we are */
	DEVICE_LOG("on SPI bus %d at %d (%u KHz)", get_device_bus(), PX4_SPI_DEV_ID(_device), _frequency / 1000);

	return PX4_OK;
}

int
SPI::transfer(uint8_t *send, uint8_t *recv, unsigned len)
{
	if ((send == nullptr) && (recv == nullptr)) {
		return -EINVAL;
	}

	// set bus frequency
	dspal_spi_ioctl_set_bus_frequency bus_freq{};

	bus_freq.bus_frequency_in_hz = _frequency;

	if (::ioctl(_fd, SPI_IOCTL_SET_BUS_FREQUENCY_IN_HZ, &bus_freq) < 0) {
		PX4_ERR("setting bus frequency failed");
		return PX4_ERROR;
	}

	// set bus mode
	// dspal_spi_ioctl_set_spi_mode bus_mode{};
	// bus_mode.eClockPolarity = SPI_CLOCK_IDLE_HIGH;
	// bus_mode.eShiftMode = SPI_OUTPUT_FIRST;

	// if (::ioctl(spi_fildes, SPI_IOCTL_SET_SPI_MODE, &bus_mode) < 0) {
	// 	PX4_ERR("setting mode failed");
	// 	return PX4_ERROR;
	// }

	// transfer data
	dspal_spi_ioctl_read_write ioctl_write_read{};

	ioctl_write_read.read_buffer = send;
	ioctl_write_read.read_buffer_length = len;
	ioctl_write_read.write_buffer = recv;
	ioctl_write_read.write_buffer_length = len;

	int result = ::ioctl(_fd, SPI_IOCTL_RDWR, &ioctl_write_read);

	if (result < 0) {
		PX4_ERR("transfer error %d", result);
		return PX4_ERROR;
	}

	return PX4_OK;
}

int
SPI::transferhword(uint16_t *send, uint16_t *recv, unsigned len)
{
	if ((send == nullptr) && (recv == nullptr)) {
		return -EINVAL;
	}

	// set bus frequency
	dspal_spi_ioctl_set_bus_frequency bus_freq{};
	bus_freq.bus_frequency_in_hz = _frequency;

	if (::ioctl(_fd, SPI_IOCTL_SET_BUS_FREQUENCY_IN_HZ, &bus_freq) < 0) {
		PX4_ERR("setting bus frequency failed");
		return PX4_ERROR;
	}

	// set bus mode
	// dspal_spi_ioctl_set_spi_mode bus_mode{};
	// bus_mode.eClockPolarity = SPI_CLOCK_IDLE_HIGH;
	// bus_mode.eShiftMode = SPI_OUTPUT_FIRST;

	// if (::ioctl(spi_fildes, SPI_IOCTL_SET_SPI_MODE, &bus_mode) < 0) {
	// 	PX4_ERR("setting mode failed");
	// 	return PX4_ERROR;
	// }

	// transfer data
	dspal_spi_ioctl_read_write ioctl_write_read{};

	ioctl_write_read.read_buffer = send;
	ioctl_write_read.read_buffer_length = len * 2;
	ioctl_write_read.write_buffer = recv;
	ioctl_write_read.write_buffer_length = len * 2;

	int result = ::ioctl(_fd, SPI_IOCTL_RDWR, &ioctl_write_read);

	if (result < 0) {
		PX4_ERR("transfer error %d", result);
		return PX4_ERROR;
	}

	return PX4_OK;
}

} // namespace device
