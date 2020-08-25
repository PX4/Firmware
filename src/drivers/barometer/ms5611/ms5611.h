/****************************************************************************
 *
 *   Copyright (C) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file ms5611.h
 *
 * Shared defines for the ms5611 driver.
 */

#pragma once

#include <string.h>

#include <drivers/device/i2c.h>
#include <drivers/device/device.h>
#include <drivers/device/spi.h>
#include <lib/cdev/CDev.hpp>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <systemlib/err.h>
#include <uORB/uORB.h>

#include "board_config.h"

#define ADDR_RESET_CMD			0x1E	/* write to this address to reset chip */
#define ADDR_PROM_SETUP			0xA0	/* address of 8x 2 bytes factory and calibration data */

/* interface ioctls */
#define IOCTL_RESET			2
#define IOCTL_MEASURE			3

namespace ms5611
{

/**
 * Calibration PROM as reported by the device.
 */
#pragma pack(push,1)
struct prom_s {
	uint16_t factory_setup;
	uint16_t c1_pressure_sens;
	uint16_t c2_pressure_offset;
	uint16_t c3_temp_coeff_pres_sens;
	uint16_t c4_temp_coeff_pres_offset;
	uint16_t c5_reference_temp;
	uint16_t c6_temp_coeff_temp;
	uint16_t serial_and_crc;
};

/**
 * Grody hack for crc4()
 */
union prom_u {
	uint16_t c[8];
	prom_s s;
};
#pragma pack(pop)

extern bool crc4(uint16_t *n_prom);

} /* namespace */

/* interface factories */
extern device::Device *MS5611_spi_interface(ms5611::prom_u &prom_buf, uint32_t devid, uint8_t busnum, int bus_frequency,
		spi_mode_e spi_mode);
extern device::Device *MS5611_i2c_interface(ms5611::prom_u &prom_buf, uint32_t devid, uint8_t busnum,
		int bus_frequency);

typedef device::Device *(*MS5611_constructor)(ms5611::prom_u &prom_buf, uint32_t devid, uint8_t busnum);
