/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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
 * @file mpc2520_i2c.cpp
 *
 * I2C interface for MPC2520
 */

#include <px4_platform_common/defines.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>

#include <arch/board/board.h>

#include <drivers/device/i2c.h>

#include "mpc2520.h"

#include "board_config.h"


#define MPC2520_ADDRESS_1		0x76	/* address select pins pulled high (PX4FMU series v1.6+) */
#define MPC2520_ADDRESS_2		0x77    /* address select pins pulled low (PX4FMU prototypes) */

device::Device *MPC2520_i2c_interface(mpc2520::prom_s &prom_buf, uint8_t busnum);

class MPC2520_I2C : public device::I2C
{
public:
	MPC2520_I2C(uint8_t bus, mpc2520::prom_s &prom_buf);
	virtual ~MPC2520_I2C();

	virtual int	init();
	virtual int	write(unsigned offset, void *data, unsigned count);
	virtual int	read(unsigned offset, void *data, unsigned count);
	virtual int	ioctl(unsigned operation, unsigned &arg);

protected:
	virtual int	probe();

private:
	mpc2520::prom_s	&_prom;

	int		_probe_address(uint8_t address);

	/**
	 * Send a reset command to the MPC2520.
	 *
	 * This is required after any bus reset.
	 */
	int		_reset();

	/**
	 * Send a measure command to the MPC2520.
	 *
	 * @param addr		Which address to use for the measure operation.
	 */
	int		_measure(unsigned addr);

	/**
	 * Read the MPC2520 PROM
	 *
	 * @return		PX4_OK if the PROM reads successfully.
	 */
	int		_read_prom();

};

device::Device *
MPC2520_i2c_interface(mpc2520::prom_s &prom_buf, uint8_t busnum)
{
	return new MPC2520_I2C(busnum, prom_buf);
}

MPC2520_I2C::MPC2520_I2C(uint8_t bus, mpc2520::prom_s &prom) :
	I2C(DRV_BARO_DEVTYPE_MPC2520, MODULE_NAME, bus, MPC2520_ADDRESS_1, 400000),
	_prom(prom)
{
}

MPC2520_I2C::~MPC2520_I2C()
{
}

int
MPC2520_I2C::init()
{
	/* this will call probe(), and thereby _probe_address */
	return I2C::init();
}

int
MPC2520_I2C::write(unsigned offset, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = offset;
	memcpy(&buf[1], data, count);

	return transfer(&buf[0], count + 1, nullptr, 0);
}

int
MPC2520_I2C::read(unsigned offset, void *data, unsigned count)
{
	uint8_t cmd = offset;
	return transfer(&cmd, 1, (uint8_t *)data, count);
}

int
MPC2520_I2C::ioctl(unsigned operation, unsigned &arg)
{
	int ret;

	switch (operation) {
	case IOCTL_RESET:
		ret = _reset();
		break;

	case IOCTL_MEASURE:
		ret = _measure(arg);
		break;

	default:
		ret = EINVAL;
	}

	return ret;
}

int
MPC2520_I2C::probe()
{
	_retries = 10;

	if ((PX4_OK == _probe_address(MPC2520_ADDRESS_1)) ||
	    (PX4_OK == _probe_address(MPC2520_ADDRESS_2))) {
		/*
		 * Disable retries; we may enable them selectively in some cases,
		 * but the device gets confused if we retry some of the commands.
		 */
		_retries = 0;
		return PX4_OK;
	}

	return -EIO;
}

int
MPC2520_I2C::_probe_address(uint8_t address)
{
	/* select the address we are going to try */
	set_device_address(address);

	/* send reset command */
	if (PX4_OK != _reset()) {
		return -EIO;
	}

	/* read PROM */
	if (PX4_OK != _read_prom()) {
		return -EIO;
	}

	return PX4_OK;
}

int
MPC2520_I2C::_reset()
{
	unsigned	old_retrycount = _retries;
	uint8_t		cmd = ADDR_RESET_CMD;
	int		result;

	/* bump the retry count */
	_retries = 10;
	result = transfer(&cmd, 1, nullptr, 0);
	_retries = old_retrycount;

	return result;
}

int
MPC2520_I2C::_measure(unsigned addr)
{
	/*
	 * Disable retries on this command; we can't know whether failure
	 * means the device did or did not see the command.
	 */
	_retries = 0;

	uint8_t cmd = addr;
	return transfer(&cmd, 1, nullptr, 0);
}

int
MPC2520_I2C::_read_prom()
{
	uint8_t	prom_buf[3];
	uint8_t cmd;

	/*
	 * Wait for PROM contents to be in the device (2.8 ms) in the case we are
	 * called immediately after reset.
	 */
	usleep(3000);

	cmd = 0x10;
	transfer(&cmd, 1, &prom_buf[0], 2);
	_prom.c0 = (int16_t)prom_buf[0] << 4 | prom_buf[1] >> 4;
	_prom.c0 = (_prom.c0 & 0x0800) ? (0xF000 | _prom.c0) : _prom.c0;
	//printf("c0=%d********\n", _prom.c0);

	cmd = 0x11;
	transfer(&cmd, 1, &prom_buf[0], 2);
	_prom.c1 = (int16_t)(prom_buf[0] & 0x0F) << 8 | prom_buf[1];
	_prom.c1 = (_prom.c1 & 0x0800) ? (0xF000 | _prom.c1) : _prom.c1;

	cmd = 0x13;
	transfer(&cmd, 1, &prom_buf[0], 3);
	_prom.c00 = (int32_t)prom_buf[0] << 12 | (int32_t)prom_buf[1] << 4 | (int32_t)prom_buf[2] >> 4;
	_prom.c00 = (_prom.c00 & 0x080000) ? (0xFFF00000 | _prom.c00) : _prom.c00;

	cmd = 0x15;
	transfer(&cmd, 1, &prom_buf[0], 3);
	_prom.c10 = (int32_t)prom_buf[0] << 16 | (int32_t)prom_buf[1] << 8 | prom_buf[2];
	_prom.c10 = (_prom.c10 & 0x080000) ? (0xFFF00000 | _prom.c10) : _prom.c10;

	cmd = 0x18;
	transfer(&cmd, 1, &prom_buf[0], 2);
	_prom.c01 = (int16_t)prom_buf[0] << 8 | prom_buf[1];

	cmd = 0x1A;
	transfer(&cmd, 1, &prom_buf[0], 2);
	_prom.c11 = (int16_t)prom_buf[0] << 8 | prom_buf[1];

	cmd = 0x1C;
	transfer(&cmd, 1, &prom_buf[0], 2);
	_prom.c20 = (int16_t)prom_buf[0] << 8 | prom_buf[1];

	cmd = 0x1E;
	transfer(&cmd, 1, &prom_buf[0], 2);
	_prom.c21 = (int16_t)prom_buf[0] << 8 | prom_buf[1];

	cmd = 0x20;
	transfer(&cmd, 1, &prom_buf[0], 2);
	_prom.c30 = (int16_t)prom_buf[0] << 8 | prom_buf[1];

	return 0;
}
