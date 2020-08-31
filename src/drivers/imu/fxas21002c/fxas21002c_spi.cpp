/****************************************************************************
 *
 * Copyright (c) 2016-2020 PX4 Development Team. All rights reserved.
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
 * @file fxas21002c_spi.cpp
 *
 * Driver for the NXP FXAS21002C connected via SPI.
 *
 * @author Robert Fu
 */
#include <drivers/device/spi.h>
#include "FXAS21002C.hpp"

device::Device *FXAS21002C_SPI_interface(int bus, uint32_t chip_select, int bus_frequency,
		spi_mode_e spi_mode);

class FXAS21002C_SPI : public device::SPI
{
public:
	FXAS21002C_SPI(int bus, uint32_t chip_select, int bus_frequency, spi_mode_e spi_mode);
	~FXAS21002C_SPI() override = default;

	/**
	 * Read directly from the device.
	 *
	 * The actual size of each unit quantity is device-specific.
	 *
	 * @param reg	The register address at which to start reading
	 * @param data	The buffer into which the read values should be placed.
	 * @param count	The number of items to read.
	 * @return		The number of items read on success, negative errno otherwise.
	 */
	int	read(unsigned reg, void *data, unsigned count) override;

	/**
	 * Write directly to the device.
	 *
	 * The actual size of each unit quantity is device-specific.
	 *
	 * @param reg	The register address at which to start writing.
	 * @param data	The buffer from which values should be read.
	 * @param count	The number of items to write.
	 * @return		The number of items written on success, negative errno otherwise.
	 */
	int	write(unsigned reg, void *data, unsigned count) override;

	/**
	 * Read a register from the device.
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	uint8_t read_reg(unsigned reg) override;

	/**
	 * Write a register to the device.
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 * @return		OK on success, negative errno otherwise.
	 */
	int write_reg(unsigned reg, uint8_t value) override;

protected:
	int probe() override;
};

device::Device *
FXAS21002C_SPI_interface(int bus, uint32_t chip_select, int bus_frequency, spi_mode_e spi_mode)
{
	return new FXAS21002C_SPI(bus, chip_select, bus_frequency, spi_mode);
}

FXAS21002C_SPI::FXAS21002C_SPI(int bus, uint32_t chip_select, int bus_frequency, spi_mode_e spi_mode) :
	SPI(DRV_GYR_DEVTYPE_FXAS2100C, MODULE_NAME, bus, chip_select, spi_mode, bus_frequency)
{
}

int
FXAS21002C_SPI::probe()
{
	uint8_t whoami = read_reg(FXAS21002C_WHO_AM_I);
	bool success = (whoami == WHO_AM_I);

	DEVICE_DEBUG("FXAS21002C_SPI::probe: %s, whoami: 0x%02x", (success ? "Succeeded" : "failed"), whoami);
	return success ? OK : -EIO;
}

/**
 * Read directly from the device.
 *
 * The actual size of each unit quantity is device-specific.
 *
 * @param reg	The register address at which to start reading
 * @param data	The buffer into which the read values should be placed.
 * @param count	The number of items to read.
 * @return		The number of items read on success, negative errno otherwise.
 */
int FXAS21002C_SPI::read(unsigned reg, void *data, unsigned count)
{
	/* Same as in mpu9250_spi.cpp:
	 * We want to avoid copying the data of RawGyroReport: So if the caller
	 * supplies a buffer not RawGyroReport in size, it is assume to be a reg or reg 16 read
	 * and we need to provied the buffer large enough for the callers data
	 * and our command.
	 */
	uint8_t cmd[3] {};

	uint8_t *pBuf  =  count < sizeof(RawGyroReport) ? cmd : (uint8_t *) data ;

	if (count < sizeof(RawGyroReport))  {
		/* add command */
		count++;
	}

	/* Set command */
	pBuf[0] = DIR_READ(reg);

	/* Transfer the command and get the data */
	int ret = transfer(pBuf, pBuf, count);

	if (ret == OK && pBuf == &cmd[0]) {
		/* Adjust the count back */
		count--;

		/* Return the data */
		memcpy(data, &cmd[1], count);
	}

	return ret;
}

/**
 * Write directly to the device.
 *
 * The actual size of each unit quantity is device-specific.
 *
 * @param reg	The register address at which to start writing.
 * @param data	The buffer from which values should be read.
 * @param count	The number of items to write.
 * @return		The number of items written on success, negative errno otherwise.
 */
int FXAS21002C_SPI::write(unsigned reg, void *data, unsigned count)
{
	uint8_t cmd[2] {};

	if (sizeof(cmd) < (count + 1)) {
		// same as in mpu9250_spi.cpp
		// This condition means only supportting the case of count == 1
		// so this API is the same as write_reg
		return -EIO;
	}

	cmd[0] = DIR_WRITE(reg);
	cmd[1] = *(uint8_t *)data;

	return transfer(cmd, nullptr, sizeof(cmd));
}

/**
 * Read a register from the device.
 *
 * @param		The register to read.
 * @return		The value that was read.
 */
uint8_t FXAS21002C_SPI::read_reg(unsigned reg)
{
	uint8_t cmd[2];

	cmd[0] = DIR_READ(reg);
	cmd[1] = 0;

	transfer(cmd, cmd, sizeof(cmd));

	return cmd[1];
}

/**
 * Write a register to the device.
 *
 * @param reg		The register to write.
 * @param value		The new value to write.
 * @return		OK on success, negative errno otherwise.
 */
int FXAS21002C_SPI::write_reg(unsigned reg, uint8_t value)
{
	uint8_t cmd[2];

	cmd[0] = DIR_WRITE(reg);
	cmd[1] = value;

	return transfer(cmd, nullptr, sizeof(cmd));
}
