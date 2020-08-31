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
 * @file fxos8701cq_i2c.cpp
 *
 * Driver for the NXP FXOS8701CQ connected via I2C.
 *
 * @author Robert Fu
 */

#include "FXOS8701CQ.hpp"

device::Device *FXOS8701CQ_I2C_interface(int bus, int bus_frequency, int i2c_address);

class FXOS8701CQ_I2C : public device::I2C
{
public:
	FXOS8701CQ_I2C(int bus, int bus_frequency, int i2c_address);
	~FXOS8701CQ_I2C() override = default;

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
	 * Write a register in the device.
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
FXOS8701CQ_I2C_interface(int bus, int bus_frequency, int i2c_address)
{
	return new FXOS8701CQ_I2C(bus, bus_frequency, i2c_address);
}

FXOS8701CQ_I2C::FXOS8701CQ_I2C(int bus, int bus_frequency, int i2c_address) :
	I2C(DRV_ACC_DEVTYPE_FXOS8701C, MODULE_NAME, bus, i2c_address, bus_frequency)
{
}

int
FXOS8701CQ_I2C::probe()
{
	uint8_t whoami = read_reg(FXOS8701CQ_WHOAMI);
	bool success = (whoami == FXOS8700CQ_WHOAMI_VAL) || (whoami == FXOS8701CQ_WHOAMI_VAL);

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
int FXOS8701CQ_I2C::read(unsigned reg, void *data, unsigned count)
{
	/* Same as in mpu9250_i2c.cpp:
	 * We want to avoid copying the data of RawAccelMagReport: So if the caller
	 * supplies a buffer not RawAccelMagReport in size, it is assume to be a reg or
	 * reg 16 read
	 * Since RawAccelMagReport has a cmd at front, we must return the data
	 * after that. Foe anthing else we must return it
	 */
	uint32_t offset = count < sizeof(RawAccelMagReport) ? 0 : offsetof(RawAccelMagReport, status);
	uint8_t cmd = FXOS8701CQ_REG(reg);

	return transfer(&cmd, 1, &((uint8_t *)data)[offset], count - offset);
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
int FXOS8701CQ_I2C::write(unsigned reg, void *data, unsigned count)
{
	uint8_t cmd[2] {};

	if (sizeof(cmd) < (count + 1)) {
		// same as in mpu9250_i2c.cpp
		// This condition means only supportting the case of count == 1
		// so this API is the same as write_reg
		return -EIO;
	}

	cmd[0] = FXOS8701CQ_REG(reg);
	cmd[1] = *(uint8_t *)data;

	return transfer(cmd, sizeof(cmd), nullptr, 0);
}

/**
 * Read a register from the device.
 *
 * @param		The register to read.
 * @return		The value that was read.
 */
uint8_t FXOS8701CQ_I2C::read_reg(unsigned reg)
{
	uint8_t cmd[1];
	uint8_t data[1];

	cmd[0] = reg & 0x00FF;

	transfer(cmd, 1, data, 1);

	return data[0];
}

/**
 * Write a register in the device.
 *
 * @param reg		The register to write.
 * @param value		The new value to write.
 * @return		OK on success, negative errno otherwise.
 */
int FXOS8701CQ_I2C::write_reg(unsigned reg, uint8_t value)
{
	uint8_t cmd[2];

	cmd[0] = reg & 0x00FF;
	cmd[1] = value;

	return transfer(cmd, 2, nullptr, 0);
}
