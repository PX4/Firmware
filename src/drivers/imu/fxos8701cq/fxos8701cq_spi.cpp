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
 * @file fxos8701cq_spi.cpp
 *
 * Driver for the NXP FXOS8701CQ connected via SPI.
 *
 * @author Robert Fu
 */

#include "FXOS8701CQ.hpp"

device::Device *FXOS8701CQ_SPI_interface(int bus, uint32_t chip_select, int bus_frequency, spi_mode_e spi_mode);

class FXOS8701CQ_SPI : public device::SPI
{
public:
	FXOS8701CQ_SPI(int bus, uint32_t chip_select, int bus_frequency, spi_mode_e spi_mode);
	~FXOS8701CQ_SPI() override = default;

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
	 * Read a register from FXOS8701CQ
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	uint8_t read_reg(unsigned reg) override;

	/**
	 * Write a register FXOS8701CQ.
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
FXOS8701CQ_SPI_interface(int bus, uint32_t chip_select, int bus_frequency, spi_mode_e spi_mode)
{
	return new FXOS8701CQ_SPI(bus, chip_select, bus_frequency, spi_mode);
}

FXOS8701CQ_SPI::FXOS8701CQ_SPI(int bus, uint32_t chip_select, int bus_frequency, spi_mode_e spi_mode) :
	SPI(DRV_ACC_DEVTYPE_FXOS8701C, MODULE_NAME, bus, chip_select, spi_mode, bus_frequency)
{
}

int
FXOS8701CQ_SPI::probe()
{
	uint8_t whoami = read_reg(FXOS8701CQ_WHOAMI);
	bool success = (whoami == FXOS8700CQ_WHOAMI_VAL) || (whoami == FXOS8701CQ_WHOAMI_VAL);

	DEVICE_DEBUG("FXAS21002C_SPI::probe: %s, whoami: 0x%02x", (success ? "Succeeded" : "failed"), whoami);
	return success ? OK : -EIO;
}

uint8_t
FXOS8701CQ_SPI::read_reg(unsigned reg)
{
	uint8_t cmd[3];

	cmd[0] = DIR_READ(reg);
	cmd[1] = ADDR_7(reg);
	cmd[2] = 0;

	transfer(cmd, cmd, sizeof(cmd));

	return cmd[2];
}

int
FXOS8701CQ_SPI::write_reg(unsigned reg, uint8_t value)
{
	uint8_t cmd[3];

	cmd[0] = DIR_WRITE(reg);
	cmd[1] = ADDR_7(reg);
	cmd[2] = value;

	return transfer(cmd, nullptr, sizeof(cmd));
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
int FXOS8701CQ_SPI::read(unsigned reg, void *data, unsigned count)
{
	/* Same as in mpu9250_spi.cpp:
	 * We want to avoid copying the data of RawAccelMagReport: So if the caller
	 * supplies a buffer not RawAccelMagReport in size, it is assume to be a reg or reg 16 read
	 * and we need to provied the buffer large enough for the callers data
	 * and our command.
	 */
	uint8_t cmd[4] {};

	uint8_t *pBuf  =  count < sizeof(RawAccelMagReport) ? cmd : (uint8_t *) data ;

	if (count < sizeof(RawAccelMagReport))  {
		/* add command */
		count += 2;
	}

	/* Set command */
	pBuf[0] = DIR_READ(reg);
	pBuf[1] = ADDR_7(reg);

	/* Transfer the command and get the data */
	int ret = transfer(pBuf, pBuf, count);

	if (ret == OK && pBuf == &cmd[0]) {
		/* Adjust the count back */
		count -= 2;

		/* Return the data */
		memcpy(data, &cmd[2], count);
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
int FXOS8701CQ_SPI::write(unsigned reg, void *data, unsigned count)
{
	uint8_t cmd[3] {};

	if (sizeof(cmd) < (count + 1)) {
		// same as in mpu9250_spi.cpp
		// This condition means only supportting the case of count == 1
		// so this API is the same as write_reg
		return -EIO;
	}

	cmd[0] = DIR_WRITE(reg);
	cmd[1] = ADDR_7(reg);
	cmd[2] = *(uint8_t *)data;

	return transfer(cmd, nullptr, sizeof(cmd));
}

