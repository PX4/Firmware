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
 * @file tcbp001ta_spi.cpp
 *
 * SPI interface for TCBP001TA.
 *
 * @author Xiaowei Zhao <xiaowei_zhao1013@163.com>
 * @author Stone White <stone@thone.io>
 *
 */

#include "defines.h"

#include <px4_platform_common/px4_config.h>
#include <drivers/device/spi.h>


/* SPI protocol address bits */
#define DIR_READ			(1<<7)  //for set
#define DIR_WRITE			~(1<<7) //for clear

#pragma pack(push,1)
struct spi_data_s {
	uint8_t addr;
	struct tcbp001ta::data_s data;
};

struct spi_calibration_s {
	uint8_t addr;
	struct tcbp001ta::calibration_s cal;
};
#pragma pack(pop)

class TCBP001TA_SPI: public device::SPI, public tcbp001ta::ITCBP001TA
{
public:
	TCBP001TA_SPI(uint8_t bus, uint32_t device);
	virtual ~TCBP001TA_SPI() override = default;

	int init() override { return SPI::init(); }

	uint8_t	get_reg(uint8_t addr) override;
	int	set_reg(uint8_t value, uint8_t addr) override;

	tcbp001ta::data_s		*get_data(uint8_t addr) override;
	tcbp001ta::calibration_s	*get_calibration(uint8_t addr) override;

	uint32_t get_device_id() const override { return device::SPI::get_device_id(); }

private:
	spi_calibration_s	_cal{};
	spi_data_s		_data{};
};

tcbp001ta::ITCBP001TA *
tcbp001ta_spi_interface(uint8_t busnum, uint32_t device)
{
	return new TCBP001TA_SPI(busnum, device);
}

TCBP001TA_SPI::TCBP001TA_SPI(uint8_t bus, uint32_t device) :
	SPI("TCBP001TA_SPI", nullptr, bus, device, SPIDEV_MODE3, 10 * 1000 * 1000)
{
}

uint8_t
TCBP001TA_SPI::get_reg(uint8_t addr)
{
	uint8_t cmd[2] = { (uint8_t)(addr | DIR_READ), 0}; // set MSB bit
	transfer(&cmd[0], &cmd[0], 2);

	return cmd[1];
}

int
TCBP001TA_SPI::set_reg(uint8_t value, uint8_t addr)
{
	uint8_t cmd[2] = { (uint8_t)(addr & DIR_WRITE), value}; // clear MSB bit
	return transfer(&cmd[0], nullptr, 2);
}

tcbp001ta::data_s *
TCBP001TA_SPI::get_data(uint8_t addr)
{
	_data.addr = (uint8_t)(addr | DIR_READ); // set MSB bit
	// PX4_INFO("addr %02x", addr);
	// PX4_INFO("addr %02x", _data.addr);

	if (transfer((uint8_t *)&_data, (uint8_t *)&_data, sizeof(spi_data_s)) == OK) {
		// PX4_INFO("t xlsb data %d", _data.data.t_xlsb);
		// PX4_INFO("t msb data %d", _data.data.t_msb);
		// PX4_INFO("t lsb data %d", _data.data.t_lsb);
		// PX4_INFO("p xlsb data %d", _data.data.p_xlsb);
		// PX4_INFO("p msb data %d", _data.data.p_msb);
		// PX4_INFO("p lsb data %d", _data.data.p_lsb);
		return &(_data.data);

	} else {
		return nullptr;
	}
}

tcbp001ta::calibration_s *
TCBP001TA_SPI::get_calibration(uint8_t addr)
{
	_cal.addr = addr | DIR_READ;

	if (transfer((uint8_t *)&_cal, (uint8_t *)&_cal, sizeof(spi_calibration_s)) == OK) {
		return &(_cal.cal);

	} else {
		return nullptr;
	}
}
