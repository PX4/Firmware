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

#include <nuttx/spi/spi.h>
#include <px4_arch/spi_hw_description.h>
#include <drivers/drv_sensor.h>

constexpr px4_spi_bus_all_hw_t px4_spi_buses_all_hw[BOARD_NUM_SPI_CFG_HW_VERSIONS] = {
	initSPIHWVersion(HW_VER_FMUV2, {
		initSPIBus(SPI::Bus::SPI1, {
			initSPIDevice(DRV_IMU_DEVTYPE_MPU6000, SPI::CS{GPIO::PortC, GPIO::Pin2}, SPI::DRDY{GPIO::PortD, GPIO::Pin15}),
			initSPIDevice(DRV_IMU_DEVTYPE_MPU9250, SPI::CS{GPIO::PortC, GPIO::Pin2}, SPI::DRDY{GPIO::PortD, GPIO::Pin15}),
			initSPIDevice(DRV_GYR_DEVTYPE_L3GD20, SPI::CS{GPIO::PortC, GPIO::Pin13}, SPI::DRDY{GPIO::PortB, GPIO::Pin0}),
			initSPIDevice(DRV_ACC_DEVTYPE_LSM303D, SPI::CS{GPIO::PortC, GPIO::Pin15}),
			initSPIDevice(DRV_BARO_DEVTYPE_MS5611, SPI::CS{GPIO::PortD, GPIO::Pin7}),
			initSPIDevice(DRV_BARO_DEVTYPE_MS5607, SPI::CS{GPIO::PortD, GPIO::Pin7}),
		}, {GPIO::PortE, GPIO::Pin3}),
		initSPIBus(SPI::Bus::SPI2, {
			initSPIDevice(SPIDEV_FLASH(0), SPI::CS{GPIO::PortD, GPIO::Pin10})
		}),
		initSPIBusExternal(SPI::Bus::SPI4, {
			initSPIConfigExternal(SPI::CS{GPIO::PortC, GPIO::Pin14}),
			initSPIConfigExternal(SPI::CS{GPIO::PortE, GPIO::Pin4}),
		}),
	}),

	initSPIHWVersion(HW_VER_FMUV3, {
		initSPIBus(SPI::Bus::SPI1, {
			initSPIDevice(DRV_IMU_DEVTYPE_MPU6000, SPI::CS{GPIO::PortC, GPIO::Pin2}, SPI::DRDY{GPIO::PortD, GPIO::Pin15}),
			initSPIDevice(DRV_IMU_DEVTYPE_MPU9250, SPI::CS{GPIO::PortC, GPIO::Pin2}, SPI::DRDY{GPIO::PortD, GPIO::Pin15}),
			initSPIDevice(DRV_MAG_DEVTYPE_HMC5883, SPI::CS{GPIO::PortC, GPIO::Pin1}), // HMC5983
			initSPIDevice(DRV_BARO_DEVTYPE_MS5611, SPI::CS{GPIO::PortD, GPIO::Pin7}),
			initSPIDevice(DRV_BARO_DEVTYPE_MS5607, SPI::CS{GPIO::PortD, GPIO::Pin7}),
		}, {GPIO::PortE, GPIO::Pin3}),
		initSPIBus(SPI::Bus::SPI2, {
			initSPIDevice(SPIDEV_FLASH(0), SPI::CS{GPIO::PortD, GPIO::Pin10})
		}),
		initSPIBus(SPI::Bus::SPI4, {
			initSPIDevice(DRV_IMU_DEVTYPE_MPU6000, SPI::CS{GPIO::PortE, GPIO::Pin4}),
			initSPIDevice(DRV_IMU_DEVTYPE_MPU9250, SPI::CS{GPIO::PortE, GPIO::Pin4}),
			initSPIDevice(DRV_GYR_DEVTYPE_L3GD20, SPI::CS{GPIO::PortC, GPIO::Pin13}),
			initSPIDevice(DRV_BARO_DEVTYPE_MS5611, SPI::CS{GPIO::PortC, GPIO::Pin14}),
			initSPIDevice(DRV_ACC_DEVTYPE_LSM303D, SPI::CS{GPIO::PortC, GPIO::Pin15}),
		}),
	}),

	initSPIHWVersion(HW_VER_FMUV2MINI, {
		initSPIBus(SPI::Bus::SPI1, {
			initSPIDevice(DRV_IMU_DEVTYPE_ICM20608, SPI::CS{GPIO::PortC, GPIO::Pin15}, SPI::DRDY{GPIO::PortC, GPIO::Pin14}),
			initSPIDevice(DRV_BARO_DEVTYPE_MS5611, SPI::CS{GPIO::PortD, GPIO::Pin7}),
		}, {GPIO::PortE, GPIO::Pin3}),
		initSPIBus(SPI::Bus::SPI2, {
			initSPIDevice(SPIDEV_FLASH(0), SPI::CS{GPIO::PortD, GPIO::Pin10})
		}),
		initSPIBusExternal(SPI::Bus::SPI4, { // unused, but we must at least define it here
		}),
	}),

	// HW_VER_FMUV2X: treat as HW_VER_FMUV2
};
static constexpr bool unused = validateSPIConfig(px4_spi_buses_all_hw);

