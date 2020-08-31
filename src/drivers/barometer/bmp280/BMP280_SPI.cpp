/****************************************************************************
 *
 *   Copyright (c) 2016-2019 PX4 Development Team. All rights reserved.
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
 * @file bmp280_spi.cpp
 *
 * SPI interface for BMP280
 */

#include "bmp280.h"

#include <px4_platform_common/px4_config.h>
#include <drivers/device/spi.h>

/* SPI protocol address bits */
#define DIR_READ			(1<<7)  //for set
#define DIR_WRITE			~(1<<7) //for clear

#pragma pack(push,1)
struct spi_data_s {
	uint8_t addr;
	struct bmp280::data_s data;
};

struct spi_calibration_s {
	uint8_t addr;
	struct bmp280::calibration_s cal;
};
#pragma pack(pop)

class BMP280_SPI: public device::SPI, public bmp280::IBMP280
{
public:
	BMP280_SPI(uint8_t bus, uint32_t device, int bus_frequency, spi_mode_e spi_mode);
	virtual ~BMP280_SPI() override = default;

	int init() override { return SPI::init(); }

	uint8_t	get_reg(uint8_t addr) override;
	int	set_reg(uint8_t value, uint8_t addr) override;

	bmp280::data_s		*get_data(uint8_t addr) override;
	bmp280::calibration_s	*get_calibration(uint8_t addr) override;

	uint32_t get_device_id() const override { return device::SPI::get_device_id(); }

	uint8_t get_device_address() const override { return device::SPI::get_device_address(); }
private:
	spi_calibration_s	_cal{};
	spi_data_s		_data{};
};

bmp280::IBMP280 *
bmp280_spi_interface(uint8_t busnum, uint32_t device, int bus_frequency, spi_mode_e spi_mode)
{
	return new BMP280_SPI(busnum, device, bus_frequency, spi_mode);
}

BMP280_SPI::BMP280_SPI(uint8_t bus, uint32_t device, int bus_frequency, spi_mode_e spi_mode) :
	SPI(DRV_BARO_DEVTYPE_BMP280, MODULE_NAME, bus, device, spi_mode, bus_frequency)
{
}

uint8_t
BMP280_SPI::get_reg(uint8_t addr)
{
	uint8_t cmd[2] = { (uint8_t)(addr | DIR_READ), 0}; // set MSB bit
	transfer(&cmd[0], &cmd[0], 2);

	return cmd[1];
}

int
BMP280_SPI::set_reg(uint8_t value, uint8_t addr)
{
	uint8_t cmd[2] = { (uint8_t)(addr & DIR_WRITE), value}; // clear MSB bit
	return transfer(&cmd[0], nullptr, 2);
}

bmp280::data_s *
BMP280_SPI::get_data(uint8_t addr)
{
	_data.addr = (uint8_t)(addr | DIR_READ); // set MSB bit

	if (transfer((uint8_t *)&_data, (uint8_t *)&_data, sizeof(spi_data_s)) == OK) {
		return &(_data.data);

	} else {
		return nullptr;
	}
}

bmp280::calibration_s *
BMP280_SPI::get_calibration(uint8_t addr)
{
	_cal.addr = addr | DIR_READ;

	if (transfer((uint8_t *)&_cal, (uint8_t *)&_cal, sizeof(spi_calibration_s)) == OK) {
		return &(_cal.cal);

	} else {
		return nullptr;
	}
}
