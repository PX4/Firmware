/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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

#include <px4_arch/spi_hw_description.h>
#include <drivers/drv_sensor.h>
#include <nuttx/spi/spi.h>

constexpr px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {
	initSPIBus(SPI::Bus::SPI1, {
		initSPIDevice(DRV_IMU_DEVTYPE_ICM20689, SPI::CS{GPIO::PortG, GPIO::Pin6}, SPI::DRDY{GPIO::PortJ, GPIO::Pin0}),
	}, {GPIO::PortE, GPIO::Pin3}),
	initSPIBus(SPI::Bus::SPI2, {
		initSPIDevice(SPIDEV_FLASH(0), SPI::CS{GPIO::PortF, GPIO::Pin5}),
		initSPIDevice(DRV_MAG_DEVTYPE_RM3100, SPI::CS{GPIO::PortF, GPIO::Pin2}, SPI::DRDY{GPIO::PortE, GPIO::Pin4}),
	}),
	initSPIBus(SPI::Bus::SPI4, {
		initSPIDevice(DRV_ACC_DEVTYPE_BMI088, SPI::CS{GPIO::PortF, GPIO::Pin3}, SPI::DRDY{GPIO::PortB, GPIO::Pin15}),
		initSPIDevice(DRV_GYR_DEVTYPE_BMI088, SPI::CS{GPIO::PortF, GPIO::Pin4}, SPI::DRDY{GPIO::PortB, GPIO::Pin14}),
		initSPIDevice(DRV_BARO_DEVTYPE_MS5611, SPI::CS{GPIO::PortG, GPIO::Pin10}),
	}),
	initSPIBusExternal(SPI::Bus::SPI5, {
		initSPIConfigExternal(SPI::CS{GPIO::PortI, GPIO::Pin4}),
		initSPIConfigExternal(SPI::CS{GPIO::PortI, GPIO::Pin10}),
		initSPIConfigExternal(SPI::CS{GPIO::PortI, GPIO::Pin13}),
	}),
	initSPIBus(SPI::Bus::SPI6, {
		initSPIDevice(DRV_IMU_DEVTYPE_ICM20649, SPI::CS{GPIO::PortI, GPIO::Pin12}, SPI::DRDY{GPIO::PortH, GPIO::Pin5}),
		initSPIDevice(DRV_BARO_DEVTYPE_MS5611, SPI::CS{GPIO::PortI, GPIO::Pin8}),
	}),
};

static constexpr bool unused = validateSPIConfig(px4_spi_buses);
