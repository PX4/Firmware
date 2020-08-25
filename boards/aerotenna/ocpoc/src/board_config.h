/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file board_config.h
 *
 * Aerotenna Ocpoc internal definitions
 */

#pragma once

#define BOARD_OVERRIDE_UUID "OCPOC00000000000" // must be of length 16
#define PX4_SOC_ARCH_ID     PX4_SOC_ARCH_ID_OCPOC

#define BOARD_BATTERY1_V_DIV   (10.177939394f)

#define BOARD_HAS_NO_RESET
#define BOARD_HAS_NO_BOOTLOADER

#define BOARD_MAX_LEDS 1 // Number of external LED's this board has


// I2C
#define PX4_I2C_BUS_EXPANSION  2 // i2c-2: Air Data Probe or I2C Splitter
#define PX4_I2C_BUS_EXPANSION1 4 // i2c-4: GPS/Compass #1
#define PX4_I2C_BUS_EXPANSION2 5 // i2c-5: GPS/Compass #2
#define PX4_I2C_BUS_EXPANSION3 3 // i2c-3: GPS/Compass #3

#define PX4_NUMBER_I2C_BUSES   4

#define PX4_I2C_BUS_LED 1

// SPI
#include <drivers/drv_sensor.h>
#define PX4_SPI_BUS_SENSORS    1
#define PX4_SPIDEV_MPU         PX4_MK_SPI_SEL(0, DRV_IMU_DEVTYPE_MPU9250) // spidev1.0 - mpu9250
#define PX4_SPIDEV_BARO        PX4_MK_SPI_SEL(0, DRV_BARO_DEVTYPE_MS5611) // spidev1.1 - ms5611
//#define PX4_SPIDEV_MPU2      PX4_MK_SPI_SEL(0, 2) // TODO: where is the 2nd mpu9250?

#define PX4_SPI_BUS_BARO PX4_SPI_BUS_SENSORS


// ADC channels:
#define ADC_CHANNELS (1 << 8)

#define ADC_BATTERY_VOLTAGE_CHANNEL  8
#define ADC_BATTERY_CURRENT_CHANNEL  ((uint8_t)(-1))


#include <system_config.h>
#include <px4_platform_common/board_common.h>
