/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file QMC5883_I2C.cpp
 *
 * I2C interface for QMC5883
 */

#include <px4_platform_common/px4_config.h>
#include <drivers/device/i2c.h>
#include <drivers/drv_device.h>

#include "qmc5883.h"

device::Device *QMC5883_I2C_interface(int bus, int bus_frequency, int i2c_address);

class QMC5883_I2C : public device::I2C
{
public:
	QMC5883_I2C(int bus, int bus_frequency, int i2c_address);
	virtual ~QMC5883_I2C() = default;

	virtual int	read(unsigned address, void *data, unsigned count);
	virtual int	write(unsigned address, void *data, unsigned count);

protected:
	virtual int	probe();
};

device::Device *
QMC5883_I2C_interface(int bus, int bus_frequency, int i2c_address)
{
	return new QMC5883_I2C(bus, bus_frequency, i2c_address);
}

QMC5883_I2C::QMC5883_I2C(int bus, int bus_frequency, int i2c_address) :
	I2C(DRV_MAG_DEVTYPE_QMC5883, MODULE_NAME, bus, i2c_address, bus_frequency)
{
}

int QMC5883_I2C::probe()
{
	uint8_t data[2] = {0, 0};

	// must read registers 0x00 once or reset to read ID registers reliably
	read(0x00, &data[0], 1);
	read(0x00, &data[0], 1);
	read(0x00, &data[0], 1);

	_retries = 10;

	bool read_valid = false;
	bool id_valid = false;

	for (unsigned i = 0; i < _retries; i++) {
		//attempt read
		if (!read(ADDR_ID_A, &data[0], 1) &&
		    !read(ADDR_ID_B, &data[1], 1)) {
			read_valid = true;
		}

		if (read_valid && data[0] == ID_A_WHO_AM_I &&
		    data[1] == ID_B_WHO_AM_I) {
			id_valid = true;
		}

		if (read_valid && id_valid) {
			return OK;
		}

		// wait 100 usec
		usleep(100);
	}

	if (!read_valid) {
		DEVICE_DEBUG("read_reg fail");
	}

	if (!id_valid) {
		DEVICE_DEBUG("ID byte mismatch (%02x,%02x)", data[0], data[1]);
	}

	return -EIO;
}

int QMC5883_I2C::write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address;
	memcpy(&buf[1], data, count);

	return transfer(&buf[0], count + 1, nullptr, 0);
}

int QMC5883_I2C::read(unsigned address, void *data, unsigned count)
{
	uint8_t cmd = address;
	return transfer(&cmd, 1, (uint8_t *)data, count);
}
