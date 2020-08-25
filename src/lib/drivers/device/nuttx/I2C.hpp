/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file i2c.h
 *
 * Base class for devices connected via I2C.
 */

#ifndef _DEVICE_I2C_H
#define _DEVICE_I2C_H

#include "../CDev.hpp"
#include <px4_platform_common/i2c.h>

#include <nuttx/i2c/i2c_master.h>

#if !defined(CONFIG_I2C)
#  error I2C support requires CONFIG_I2C
#endif

namespace device __EXPORT
{

/**
 * Abstract class for character device on I2C
 */
class __EXPORT I2C : public CDev
{

public:

	// no copy, assignment, move, move assignment
	I2C(const I2C &) = delete;
	I2C &operator=(const I2C &) = delete;
	I2C(I2C &&) = delete;
	I2C &operator=(I2C &&) = delete;

	virtual int	init() override;

	static int	set_bus_clock(unsigned bus, unsigned clock_hz);

protected:
	/**
	 * The number of times a read or write operation will be retried on
	 * error.
	 */
	uint8_t		_retries{0};

	/**
	 * @ Constructor
	 *
	 * @param name		Driver name
	 * @param devname	Device node name
	 * @param bus		I2C bus on which the device lives
	 * @param address	I2C bus address, or zero if set_address will be used
	 * @param frequency	I2C bus frequency for the device (currently not used)
	 */
	I2C(const char *name, const char *devname, const int bus, const uint16_t address, const uint32_t frequency);
	virtual ~I2C();

	/**
	 * Check for the presence of the device on the bus.
	 */
	virtual int	probe() { return PX4_OK; }

	/**
	 * Perform an I2C transaction to the device.
	 *
	 * At least one of send_len and recv_len must be non-zero.
	 *
	 * @param send		Pointer to bytes to send.
	 * @param send_len	Number of bytes to send.
	 * @param recv		Pointer to buffer for bytes received.
	 * @param recv_len	Number of bytes to receive.
	 * @return		OK if the transfer was successful, -errno
	 *			otherwise.
	 */
	int		transfer(const uint8_t *send, const unsigned send_len, uint8_t *recv, const unsigned recv_len);

	bool	external() const override { return px4_i2c_bus_external(_device_id.devid_s.bus); }

private:
	static unsigned	int	_bus_clocks[PX4_NUMBER_I2C_BUSES];

	const uint32_t		_frequency;
	i2c_master_s		*_dev{nullptr};

};

} // namespace device

#endif /* _DEVICE_I2C_H */
