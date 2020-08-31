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

/* Due to inconsistent use of chip select and data ready signal on
 * different board that use this build, we are using different
 * versions.
 *
 *  --------------- SPI1 -------------------- SPI4 --------------     Incompatibilities    ---------
 *  FMUv2:                                                        FmuV3 Cube        PixhawkMini
 *   Power Domain:  VDD_3V3_SENSORS_EN        nVDD_5V_PERIPH_EN    V3V:SPI1&SPI4    V3V:SPI1 No SPI4
 *   PA5            SPI_INT_SCK
 *   PA6            SPI_INT_MISO
 *   PA7            SPI_INT_MOSI
 *   PB0            GYRO_DRDY                                      SPI4:EXTERN_DRDY        NC
 *   PB1            MAG_DRDY                                       +SPI4:nEXTERN_CS        NC
 *   PB4            ACCEL_DRDY                                     NC                      NC
 *   PC1            Spare ADC ( NC )                               +SPI1:SPI_INT_MAG_!CS
 *   PC2            nMPU_CS                                        @MPU6000|MPU9250        @MPU9250
 *   PC13           nGYRO_CS                                       SPI4:nGYRO_EXT_CS       NC
 *   PC14                                     GPIO_EXT_1           nBARO_EXT_CS            -20608_DRDY
 *   PC15           nACCEL_MAG_CS                                  SPI4:nACCEL_MAG_EXT_CS  20608_CS
 *   PD7            nBARO_CS
 *   PD15           nMPU_DRDY                                      @MPU6000|MPU9250        @MPU9250
 *   PE2                                      SPI_EXT_SCK                                  NC
 *   PE4                                      nSPI_EXT_NSS         SPI4:nMPU_EXT_CS        NC
 *   PE5                                      SPI_EXT_MISO                                 NC
 *   PE6                                      SPI_EXT_MOSI                                 NC
 *
 *   Notes: Prefixed with @ Does not effect board control
 *          Prefixed with + Input used as Output
 *          Prefixed with - Output used as Input
 *          Prefixed with SPIn: Bus changed
 *
 *  The board API provides for mechanism to perform a SPI bus reset.
 *  To facilitate a SPI bus reset
 *
 *    1) All the pins: SPIn, CD, DRDY associated with the SPI bus are turned to inputs
 *       with outputs driven low. (OFFIng)
 *    2) The power domain of that bus is turned off.
 *    3) A usleep it done for ms.
 *    4) The power domain of that bus is turned back on.
 *    5) The SPIn pins are re-initialized.
 *    6) The SPI CS, DRDY pins are re-initialized.
 *
 * To insure the complete net is de-energized and is not bing back fed, it is important to
 * note the all signals in the net list of the parts/bus.
 *
 * I.E. Not OFFIng PC1 on V3 would leave that pin back feeding the HMC part. As would not
 * OFFIng PE4, not associated with SPI1 on V2, but would back feed an MPUxxxx on V3
 *
 */

/*
 *
 *  --------------- SPI1 -------------------- SPI4 --------------     Incompatibilities    ---------
 *  FMUv3 Cube:                                                       FmuV2            PixhawkMini
 *   Power Domain:  VDD_3V3_SENSORS_EN         NA			      V3V:SPI V5:SPI4   V3V:SPI1 No SPI4
 *   PA5            SPI_INT_SCK
 *   PA6            SPI_INT_MISO
 *   PA7            SPI_INT_MOSI
 *   PB0                                      EXTERN_DRDY          SPI1:GYRO_DRDY          NC
 *   PB1            MAG_DRDY                  nEXTERN_CS           -SPI1:MAG_DRDY          NC
 *   PB4                                          NC               SPI1:ACCEL_DRDY         NC
 *   PC1            SPI_INT_MAG_!CS                                -ADC1_IN11              NC
 *   PC2            nMPU_CS                                        @MPU6000             @MPU9250
 *   PC13                                     nGYRO_EXT_CS         SPI1:nGYRO_CS           NC
 *   PC14                                     nBARO_EXT_CS         GPIO_EXT_1            -20608_DRDY
 *   PC15                                     nACCEL_MAG_EXT_CS    SPI1:nACCEL_MAG_CS     20608_CS
 *   PD7            nBARO_CS
 *   PD15           nMPU_DRDY                                      @MPU6000              @MPU9250
 *   PE2                                      SPI_EXT_SCK                                  NC
 *   PE4                                      MPU_EXT_CS           SPI4:nSPI_EXT_NSS       NC
 *   PE5                                      SPI_EXT_MISO                                 NC
 *   PE6                                      SPI_EXT_MOSI                                 NC
 *
 *
 *   Notes: Prefixed with @ Does not effect board control
 *          Prefixed with + Input used as Output
 *          Prefixed with - Output used as Input
 *          Prefixed with SPIn: Bus changed
 *
 */

/*----------------------------------------------------------*/
/*
 *  --------------- SPI1 -------------------- SPI4 --------------      Incompatibilities    ---------
 *  FMUv2 Pixhawk Mini                                                FmuV2               FmuV3 Cube
 *   Power Domain:  VDD_3V3_SENSORS_EN        NA                  V3V:SPI V5:SPI4        V3V:SPI1&SPI4
 *   PA5            SPI_INT_SCK
 *   PA6            SPI_INT_MISO
 *   PA7            SPI_INT_MOSI
 *   PB0            NC                                             SPI1:GYRO_DRDY      SPI4:EXTERN_DRDY
 *   PB1            NC                                             -SPI1:MAG_DRDY      +SPI4:nEXTERN_CS
 *   PB4            NC                                             SPI1:ACCEL_DRDY     NC
 *   PC1            Spare ADC ( NC )                                                   +SPI1:SPI_INT_MAG_!CS
 *   PC2            nMPU_CS                                        @MPU6000            @MPU6000|MPU9250
 *   PC13           NC                                             SPI1:nGYRO_CS       SPI4:nGYRO_EXT_CS
 *   PC14           20608_DRDY                                     +GPIO_EXT_1         nBARO_EXT_CS
 *   PC15           20608_CS                                       nACCEL_MAG_CS       SPI4:nACCEL_MAG_EXT_CS
 *   PD7            nBARO_CS
 *   PD15           nMPU_DRDY                                      @MPU6000            @MPU6000|MPU9250
 *   PE2                                      NC                   SPI_EXT_SCK         SPI_EXT_SCK
 *   PE4                                      NC                   SPI4:nSPI_EXT_NSS   SPI4:nMPU_EXT_CS
 *   PE5                                      NC                   SPI_EXT_MISO        SPI_EXT_MISO
 *   PE6                                      NC                   SPI_EXT_MOSI        SPI_EXT_MOSI
 *
 *   Notes: Prefixed with @ Does not effect board control
 *          Prefixed with + Input used as Output
 *          Prefixed with - Output used as Input
 *          Prefixed with SPIn: Bus changed
 *
 */

constexpr px4_spi_bus_all_hw_t px4_spi_buses_all_hw[BOARD_NUM_SPI_CFG_HW_VERSIONS] = {
	initSPIHWVersion(HW_VER_FMUV2, {
		initSPIBus(SPI::Bus::SPI1, {
			initSPIDevice(DRV_IMU_DEVTYPE_MPU6000, SPI::CS{GPIO::PortC, GPIO::Pin2}, SPI::DRDY{GPIO::PortD, GPIO::Pin15}),
			initSPIDevice(DRV_IMU_DEVTYPE_MPU9250, SPI::CS{GPIO::PortC, GPIO::Pin2}, SPI::DRDY{GPIO::PortD, GPIO::Pin15}),
			initSPIDevice(DRV_GYR_DEVTYPE_L3GD20, SPI::CS{GPIO::PortC, GPIO::Pin13}, SPI::DRDY{GPIO::PortB, GPIO::Pin0}),
			initSPIDevice(DRV_IMU_DEVTYPE_LSM303D, SPI::CS{GPIO::PortC, GPIO::Pin15}),
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
			initSPIDevice(DRV_IMU_DEVTYPE_LSM303D, SPI::CS{GPIO::PortC, GPIO::Pin15}),
		}),
	}),

	initSPIHWVersion(HW_VER_FMUV2MINI, {
		initSPIBus(SPI::Bus::SPI1, {
			initSPIDevice(DRV_IMU_DEVTYPE_ICM20608G, SPI::CS{GPIO::PortC, GPIO::Pin15}, SPI::DRDY{GPIO::PortC, GPIO::Pin14}),
			initSPIDevice(DRV_BARO_DEVTYPE_MS5611, SPI::CS{GPIO::PortD, GPIO::Pin7}),
			initSPIDevice(DRV_DEVTYPE_UNUSED, SPI::CS{GPIO::PortC, GPIO::Pin2}, SPI::DRDY{GPIO::PortD, GPIO::Pin15}), // unused MPU9250
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
