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
 * @file I2C.cpp
 *
 * Base class for devices attached via the I2C bus.
 *
 * @todo Bus frequency changes; currently we do nothing with the value
 *       that is supplied.  Should we just depend on the bus knowing?
 */

#include "I2C.hpp"

#include <dev_fs_lib_i2c.h>

namespace device
{

I2C::I2C(const char *name, const char *devname, const int bus, const uint16_t address, const uint32_t frequency) :
	CDev(name, devname),
	_frequency(frequency)
{
	DEVICE_DEBUG("I2C::I2C name = %s devname = %s", name, devname);
	// fill in _device_id fields for a I2C device
	_device_id.devid_s.bus_type = DeviceBusType_I2C;
	_device_id.devid_s.bus = bus;
	_device_id.devid_s.address = address;
	// devtype needs to be filled in by the driver
	_device_id.devid_s.devtype = 0;
}

I2C::~I2C()
{
	if (_fd >= 0) {
		::close(_fd);
		_fd = -1;
	}
}

int
I2C::init()
{
	int ret = PX4_ERROR;

	// Open the actual I2C device
	char dev_path[16] {};
	snprintf(dev_path, sizeof(dev_path), DEV_FS_I2C_DEVICE_TYPE_STRING"%i", get_device_bus());
	_fd = ::open(dev_path, O_RDWR);

	if (_fd < 0) {
		DEVICE_DEBUG("failed to init I2C");
		ret = -ENOENT;
		goto out;
	}

	// call the probe function to check whether the device is present
	ret = probe();

	if (ret != OK) {
		DEVICE_DEBUG("probe failed");
		goto out;
	}

	// do base class init, which will create device node, etc
	ret = CDev::init();

	if (ret != OK) {
		DEVICE_DEBUG("cdev init failed");
		goto out;
	}

	// tell the world where we are
	DEVICE_LOG("on I2C bus %d at 0x%02x", get_device_bus(), get_device_address());

out:

	if ((ret != OK) && !(_fd < 0)) {
		::close(_fd);
		_fd = -1;
	}

	return ret;
}

int
I2C::transfer(const uint8_t *send, const unsigned send_len, uint8_t *recv, const unsigned recv_len)
{
	int ret = PX4_ERROR;
	unsigned retry_count = 0;

	if (_fd < 0) {
		PX4_ERR("I2C device not opened");
		return PX4_ERROR;
	}

	do {
		DEVICE_DEBUG("transfer out %p/%u  in %p/%u", send, send_len, recv, recv_len);

		dspal_i2c_ioctl_slave_config slave_config{};
		slave_config.slave_address = get_device_address();
		slave_config.bus_frequency_in_khz = _frequency / 1000;
		slave_config.byte_transer_timeout_in_usecs = 10000; // 10 ms
		int ret_config = ::ioctl(_fd, I2C_IOCTL_SLAVE, &slave_config);

		if (ret_config < 0) {
			DEVICE_DEBUG("Could not set slave config, result: %d", ret_config);
		}


		dspal_i2c_ioctl_combined_write_read ioctl_write_read{};

		ioctl_write_read.write_buf = (uint8_t *)send;
		ioctl_write_read.write_buf_len = send_len;
		ioctl_write_read.read_buf = recv;
		ioctl_write_read.read_buf_len = recv_len;

		int bytes_read = ::ioctl(_fd, I2C_IOCTL_RDWR, &ioctl_write_read);

		if (bytes_read != (int)recv_len) {
			DEVICE_DEBUG("I2C transfer failed, bytes read %d", bytes_read);
			ret = PX4_ERROR;

		} else {
			// success
			ret = PX4_OK;
			break;
		}

	} while (retry_count++ < _retries);

	return ret;
}

} // namespace device
