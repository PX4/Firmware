/****************************************************************************
 *
 *   Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
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
 * NXP fmukrt1062-v1 internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include "imxrt_gpio.h"
#include "imxrt_iomuxc.h"
#include "hardware/imxrt_pinmux.h"

#include <arch/board/board.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

/* PX4IO connection configuration */

#if 0 // There is no PX4IO Support on first out
// This requires serial DMA driver
#define BOARD_USES_PX4IO_VERSION       2
#define PX4IO_SERIAL_DEVICE            "/dev/ttyS6"
#define PX4IO_SERIAL_TX_GPIO           GPIO_LPUART8_TX_2
#define PX4IO_SERIAL_RX_GPIO           GPIO_LPUART8_RX_2
#define PX4IO_SERIAL_BASE              IMXRT_LPUART8_BASE
#define PX4IO_SERIAL_VECTOR            IMXRT_IRQ_LPUART8
#define PX4IO_SERIAL_TX_DMAMAP
#define PX4IO_SERIAL_RX_DMAMAP
#define PX4IO_SERIAL_RCC_REG
#define PX4IO_SERIAL_RCC_EN
#define PX4IO_SERIAL_CLOCK
#define PX4IO_SERIAL_BITRATE           1500000               /* 1.5Mbps -> max rate for IO */
#endif

/* Configuration ************************************************************************************/

/* FMURT1062 GPIOs ***********************************************************************************/
/* LEDs */
/* An RGB LED is connected through GPIO as shown below:
 */
#define LED_IOMUX (IOMUX_OPENDRAIN | IOMUX_PULL_NONE | IOMUX_DRIVE_33OHM | IOMUX_SPEED_MEDIUM | IOMUX_SLEW_SLOW)
#define GPIO_nLED_RED   /* GPIO_B0_00 QTIMER1_TIMER0 GPIO2_IO0  */ (GPIO_PORT2 | GPIO_PIN0  | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | LED_IOMUX)
#define GPIO_nLED_GREEN /* GPIO_B0_01 QTIMER1_TIMER1 GPIO2_IO1  */ (GPIO_PORT2 | GPIO_PIN1  | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | LED_IOMUX)
#define GPIO_nLED_BLUE  /* GPIO_B1_08 QTIMER1_TIMER3 GPIO2_IO24 */ (GPIO_PORT2 | GPIO_PIN24 | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | LED_IOMUX)

#define BOARD_HAS_CONTROL_STATUS_LEDS   1
#define BOARD_OVERLOAD_LED              LED_RED
#define BOARD_ARMED_STATE_LED           LED_BLUE

/* SENSORS are on SPI1, 3
 * MEMORY is on bus SPI2
 * MS5611 is on bus SPI4
 */

#define PX4_SPI_BUS_SENSORS             1
#define PX4_SPI_BUS_MEMORY              2
#define PX4_SPI_BUS_BARO                3
#define PX4_SPI_BUS_EXTERNAL1           4

/*
 *  Define the ability to shut off off the sensor signals
 *  by changing the signals to inputs
 */

#define _PIN_OFF(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT | IOMUX_PULL_DOWN_100K | IOMUX_CMOS_INPUT))

/*  Define the Chip Selects, Data Ready and Control signals per SPI bus */

#define CS_IOMUX  (IOMUX_CMOS_OUTPUT | IOMUX_PULL_UP_100K | IOMUX_DRIVE_33OHM | IOMUX_SPEED_LOW | IOMUX_SLEW_FAST)
#define OUT_IOMUX (IOMUX_CMOS_OUTPUT | IOMUX_PULL_UP_100K | IOMUX_DRIVE_50OHM | IOMUX_SPEED_MEDIUM | IOMUX_SLEW_FAST)

/* SPI 1 CS */
#define GPIO_SPI1_CS1_ICM20689     /* GPIO_EMC_40 GPIO3_IO26 */ (GPIO_PORT3 | GPIO_PIN26 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | CS_IOMUX)
//      GPIO_SPI1_CS2_ICM20602     is not wired from CPU
#define GPIO_SPI1_CS3_BMI055_GYRO  /* GPIO_B1_10 GPIO2_IO26 */  (GPIO_PORT2 | GPIO_PIN26 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | CS_IOMUX)
#define GPIO_SPI1_CS4_BMI055_ACCEL /* GPIO_B1_15 GPIO2_IO31 */  (GPIO_PORT2 | GPIO_PIN31 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | CS_IOMUX)

#define SPI1_CS5_AUX_MEM           /* GPIO_SD_B1_00 GPIO3_IO00 */ (GPIO_PORT3 | GPIO_PIN0 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | CS_IOMUX)

/*  Define the SPI1 Data Ready interrupts */

#define DRDY_IOMUX (IOMUX_SCHMITT_TRIGGER | IOMUX_PULL_UP_47K | IOMUX_DRIVE_HIZ)

// todo add IRQ-ness!

#define GPIO_SPI1_DRDY1_ICM20689     /* GPIO_EMC_15 GPIO4_IO15 */   (GPIO_PORT4 | GPIO_PIN15 | GPIO_INPUT | DRDY_IOMUX)
#define GPIO_SPI1_DRDY2_BMI055_GYRO  /* GPIO_EMC_16 GPIO4_IO16 */   (GPIO_PORT4 | GPIO_PIN16 | GPIO_INPUT | DRDY_IOMUX)
#define GPIO_SPI1_DRDY3_BMI055_ACCEL /* GPIO_EMC_37 GPIO3_IO23 */   (GPIO_PORT3 | GPIO_PIN23 | GPIO_INPUT | DRDY_IOMUX)
#define SPI1_DRDY4_ICM20602          /* GPIO_EMC_12 GPIO4_IO12 N.B . The ICM20602 CS is not wired */ (GPIO_PORT4 | GPIO_PIN12 | GPIO_INPUT | DRDY_IOMUX)

/* SPI1 off */

#define _GPIO_LPSPI1_SCK   /* GPIO_EMC_27  GPIO4_IO27 */  (GPIO_PORT4 | GPIO_PIN27 | CS_IOMUX)
#define _GPIO_LPSPI1_MISO  /* GPIO_EMC_29  GPIO4_IO29 */  (GPIO_PORT4 | GPIO_PIN29 | CS_IOMUX)
#define _GPIO_LPSPI1_MOSI  /* GPIO_EMC_28  GPIO4_IO28 */  (GPIO_PORT4 | GPIO_PIN28 | CS_IOMUX)

#define GPIO_SPI1_SCK_OFF   _PIN_OFF(_GPIO_LPSPI1_SCK)
#define GPIO_SPI1_MISO_OFF  _PIN_OFF(_GPIO_LPSPI1_MISO)
#define GPIO_SPI1_MOSI_OFF  _PIN_OFF(_GPIO_LPSPI1_MOSI)

#define GPIO_DRDY_OFF_SPI1_DRDY1_ICM20689    _PIN_OFF(GPIO_SPI1_DRDY1_ICM20689)
#define GPIO_DRDY_OFF_SPI1_DRDY2_BMI055_GYRO _PIN_OFF(GPIO_SPI1_DRDY2_BMI055_GYRO)
#define GPIO_DRDY_OFF_SPI1_DRDY3_BMI055_ACC  _PIN_OFF(GPIO_SPI1_DRDY3_BMI055_ACCEL)
#define GPIO_DRDY_OFF_SPI1_DRDY4_ICM20602    _PIN_OFF(GPIO_SPI1_DRDY4_ICM20602)

/* SPI 2 CS */

#define GPIO_SPI2_CS_FRAM     /* GPIO_EMC_34 G GPIO3_IO20 */ (GPIO_PORT3 | GPIO_PIN20 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | CS_IOMUX)

/* SPI 3 CS */

#define GPIO_SPI3_CS1_MS5611 /* GPIO_EMC_14 GPIO4_IO14 */ (GPIO_PORT4 | GPIO_PIN14 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | CS_IOMUX)

#define _GPIO_LPSPI3_SCK   /* GPIO_AD_B1_15 GPIO1_IO27 */  (GPIO_PORT1 | GPIO_PIN31 | CS_IOMUX)
#define _GPIO_LPSPI3_MISO  /* GPIO_AD_B1_13 GPIO1_IO27 */  (GPIO_PORT1 | GPIO_PIN29 | CS_IOMUX)
#define _GPIO_LPSPI3_MOSI  /* GPIO_AD_B1_14 GPIO1_IO27 */  (GPIO_PORT1 | GPIO_PIN30 | CS_IOMUX)

#define GPIO_SPI3_SCK_OFF   _PIN_OFF(_GPIO_LPSPI3_SCK)
#define GPIO_SPI3_MISO_OFF  _PIN_OFF(_GPIO_LPSPI3_MISO)
#define GPIO_SPI3_MOSI_OFF  _PIN_OFF(_GPIO_LPSPI3_MOSI)

/* SPI 4 CS */

#define SPI4_CS1_EXTERNAL1    /* GPIO_EMC_07 GPIO4_IO07 */  (GPIO_PORT4 | GPIO_PIN7  | GPIO_OUTPUT | GPIO_OUTPUT_ONE | CS_IOMUX)
#define SPI4_CS2_EXTERNAL1    /* GPIO_B1_14  GPIO2_IO30 */  (GPIO_PORT2 | GPIO_PIN30 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | CS_IOMUX)
#define SPI4_CS3_EXTERNAL1    /* GPIO_EMC_11 GPIO4_IO011 */ (GPIO_PORT4 | GPIO_PIN11 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | CS_IOMUX)

/*  Define the SPI4 Data Ready and Control signals */

#define GPIO_SPI4_DRDY7_EXTERNAL1   /* GPIO_EMC_35 GPIO3_IO21*/ (GPIO_PORT3 | GPIO_PIN21 | GPIO_INPUT  | DRDY_IOMUX)
#define GPIO_nSPI4_RESET_EXTERNAL1  /* GPIO_B1_00 GPIO2_IO16 */ (GPIO_PORT2 | GPIO_PIN16 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | OUT_IOMUX)
#define GPIO_SPI4_SYNC_EXTERNAL1    /* GPIO_EMC_05 GPIO4_IO5  */(GPIO_PORT4 | GPIO_PIN5  | GPIO_OUTPUT | GPIO_OUTPUT_ONE | OUT_IOMUX)

#define GPIO_DRDY_OFF_SPI4_DRDY7_EXTERNAL1   _PIN_OFF(GPIO_SPI4_DRDY7_EXTERNAL1)
#define GPIO_nSPI4_RESET_EXTERNAL1_OFF       _PIN_OFF(GPIO_nSPI4_RESET_EXTERNAL1)
#define GPIO_SPI4_SYNC_EXTERNAL1_OFF         _PIN_OFF(GPIO_SPI4_SYNC_EXTERNAL1)


/* v BEGIN Legacy SPI defines TODO: fix this with enumeration */

#define PX4_SPI_BUS_RAMTRON  PX4_SPI_BUS_MEMORY
#define PX4_SPIDEV_BMA 0
#define PX4_SPIDEV_BMI 0

/* ^ END Legacy SPI defines TODO: fix this with enumeration */
#include <drivers/drv_sensor.h>

#define PX4_SPIDEV_ICM_20689        PX4_MK_SPI_SEL(0,DRV_IMU_DEVTYPE_ICM20689)
#define PX4_SPIDEV_BMI055_GYR       PX4_MK_SPI_SEL(0,DRV_GYR_DEVTYPE_BMI055)
#define PX4_SPIDEV_BMI055_ACC       PX4_MK_SPI_SEL(0,DRV_ACC_DEVTYPE_BMI055)
#define PX4_SPIDEV_AUX_MEM          PX4_MK_SPI_SEL(0,DRV_DEVTYPE_UNUSED)

#define PX4_SENSOR_BUS_CS_GPIO      {GPIO_SPI1_CS1_ICM20689, GPIO_SPI1_CS3_BMI055_GYRO, GPIO_SPI1_CS4_BMI055_ACCEL, SPI1_CS5_AUX_MEM}
#define PX4_SENSORS_BUS_FIRST_CS    PX4_SPIDEV_ICM_20689
#define PX4_SENSORS_BUS_LAST_CS     PX4_SPIDEV_AUX_MEM

#define PX4_SPIDEV_MEMORY           SPIDEV_FLASH(0)
#define PX4_MEMORY_BUS_CS_GPIO      {GPIO_SPI2_CS_FRAM}
#define PX4_MEMORY_BUS_FIRST_CS     PX4_SPIDEV_MEMORY
#define PX4_MEMORY_BUS_LAST_CS      PX4_SPIDEV_MEMORY

#define PX4_SPIDEV_BARO             PX4_MK_SPI_SEL(0,DRV_BARO_DEVTYPE_MS5611)
#define PX4_BARO_BUS_CS_GPIO        {GPIO_SPI3_CS1_MS5611}
#define PX4_BARO_BUS_FIRST_CS       PX4_SPIDEV_BARO
#define PX4_BARO_BUS_LAST_CS        PX4_SPIDEV_BARO

#define PX4_SPIDEV_EXTERNAL1_1      PX4_MK_SPI_SEL(0,0)
#define PX4_SPIDEV_EXTERNAL1_2      PX4_MK_SPI_SEL(0,1)
#define PX4_SPIDEV_EXTERNAL1_3      PX4_MK_SPI_SEL(0,2)
#define PX4_EXTERNAL1_BUS_CS_GPIO   {SPI4_CS1_EXTERNAL1, SPI4_CS2_EXTERNAL1, SPI4_CS3_EXTERNAL1}
#define PX4_EXTERNAL1_BUS_FIRST_CS  PX4_SPIDEV_EXTERNAL1_1
#define PX4_EXTERNAL1_BUS_LAST_CS   PX4_SPIDEV_EXTERNAL1_3

/* I2C busses */

#define PX4_I2C_BUS_EXPANSION       1
#define PX4_I2C_BUS_EXPANSION1      2
#define PX4_I2C_BUS_ONBOARD         3
#define PX4_I2C_BUS_LED             PX4_I2C_BUS_EXPANSION

#define BOARD_NUMBER_I2C_BUSES      3
#define BOARD_I2C_BUS_CLOCK_INIT    {100000, 100000, 100000}


#define ADC_IOMUX (IOMUX_CMOS_INPUT | IOMUX_PULL_NONE | IOMUX_DRIVE_HIZ)

#define ADC1_CH(n)                  (n)
#define ADC1_GPIO(n, p)             (GPIO_PORT1 | GPIO_PIN##p | ADC_IOMUX) //

/* Define GPIO pins used as ADC N.B. Channel numbers are for reference, */

#define PX4_ADC_GPIO  \
	/* BATTERY1_VOLTAGE       GPIO_AD_B1_11 GPIO1 Pin 27 */  ADC1_GPIO(0,  27),  \
	/* BATTERY1_CURRENT       GPIO_AD_B0_12 GPIO1 Pin 12 */  ADC1_GPIO(1,  12),  \
	/* BATTERY2_VOLTAGE       GPIO_AD_B0_13 GPIO1 Pin 13 */  ADC1_GPIO(2,  13),  \
	/* BATTERY2_CURRENT       GPIO_AD_B0_14 GPIO1 Pin 14 */  ADC1_GPIO(3,  14),  \
	/* SPARE_2_CHANNEL        GPIO_AD_B0_15 GPIO1 Pin 15 */  ADC1_GPIO(4,  15),  \
	/* HW_VER_SENSE           GPIO_AD_B1_04 GPIO1 Pin 20 */  ADC1_GPIO(9,  20),  \
	/* SCALED_V5              GPIO_AD_B1_05 GPIO1 Pin 21 */  ADC1_GPIO(10, 21), \
	/* SCALED_VDD_3V3_SENSORS GPIO_AD_B1_06 GPIO1 Pin 22 */  ADC1_GPIO(11, 22), \
	/* HW_REV_SENSE           GPIO_AD_B1_08 GPIO1 Pin 24 */  ADC1_GPIO(13, 24), \
	/* SPARE_1                GPIO_AD_B1_09 GPIO1 Pin 25 */  ADC1_GPIO(14, 25), \
	/* RSSI_IN                GPIO_AD_B1_10 GPIO1 Pin 26 */  ADC1_GPIO(15, 26)

/* Define Channel numbers must match above GPIO pin IN(n)*/

#define ADC_BATTERY1_VOLTAGE_CHANNEL        /* GPIO_AD_B1_11 GPIO1 Pin 27 */  ADC1_CH(0)
#define ADC_BATTERY1_CURRENT_CHANNEL        /* GPIO_AD_B0_12 GPIO1 Pin 12 */  ADC1_CH(1)
#define ADC_BATTERY2_VOLTAGE_CHANNEL        /* GPIO_AD_B0_13 GPIO1 Pin 13 */  ADC1_CH(2)
#define ADC_BATTERY2_CURRENT_CHANNEL        /* GPIO_AD_B0_14 GPIO1 Pin 14 */  ADC1_CH(3)
#define ADC1_SPARE_2_CHANNEL                /* GPIO_AD_B0_15 GPIO1 Pin 15 */  ADC1_CH(4)
#define ADC_HW_VER_SENSE_CHANNEL            /* GPIO_AD_B1_04 GPIO1 Pin 20 */  ADC1_CH(9)
#define ADC_SCALED_V5_CHANNEL               /* GPIO_AD_B1_05 GPIO1 Pin 21 */  ADC1_CH(10)
#define ADC_SCALED_VDD_3V3_SENSORS_CHANNEL  /* GPIO_AD_B1_06 GPIO1 Pin 22 */  ADC1_CH(11)
#define ADC_HW_REV_SENSE_CHANNEL            /* GPIO_AD_B1_08 GPIO1 Pin 24 */  ADC1_CH(13)
#define ADC1_SPARE_1_CHANNEL                /* GPIO_AD_B1_09 GPIO1 Pin 25 */  ADC1_CH(14)
#define ADC_RSSI_IN_CHANNEL                 /* GPIO_AD_B1_10 GPIO1 Pin 26 */  ADC1_CH(15)

#define ADC_CHANNELS \
	((1 << ADC_BATTERY1_VOLTAGE_CHANNEL)       | \
	 (1 << ADC_BATTERY1_CURRENT_CHANNEL)       | \
	 (1 << ADC_BATTERY2_VOLTAGE_CHANNEL)       | \
	 (1 << ADC_BATTERY2_CURRENT_CHANNEL)       | \
	 (1 << ADC1_SPARE_2_CHANNEL)               | \
	 (1 << ADC_RSSI_IN_CHANNEL)                | \
	 (1 << ADC_SCALED_V5_CHANNEL)              | \
	 (1 << ADC_SCALED_VDD_3V3_SENSORS_CHANNEL) | \
	 (1 << ADC_HW_VER_SENSE_CHANNEL)           | \
	 (1 << ADC_HW_REV_SENSE_CHANNEL)           | \
	 (1 << ADC1_SPARE_1_CHANNEL))

/* Define Battery 1 Voltage Divider and A per V */

#define BOARD_BATTERY1_V_DIV         (10.1097f)     /* measured with the provided PM board */
#define BOARD_BATTERY1_A_PER_V       (15.391030303f)

/* HW has to large of R termination on ADC todo:change when HW value is chosen */

#define BOARD_ADC_OPEN_CIRCUIT_V     (5.6f)

/* HW Version and Revision drive signals Default to 1 to detect */

#define BOARD_HAS_HW_VERSIONING

#define HW_IOMUX (IOMUX_CMOS_OUTPUT | IOMUX_PULL_NONE | IOMUX_DRIVE_33OHM | IOMUX_SPEED_MAX | IOMUX_SLEW_FAST)

#define GPIO_HW_VER_REV_DRIVE /* GPIO_AD_B0_01 GPIO1_IO01   */  (GPIO_PORT1 | GPIO_PIN1 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | HW_IOMUX)
#define GPIO_HW_REV_SENSE     /* GPIO_AD_B1_08 GPIO1 Pin 24 */  ADC1_GPIO(13, 24)
#define GPIO_HW_VER_SENSE     /* GPIO_AD_B1_04 GPIO1 Pin 20 */  ADC1_GPIO(9,  20)
#define HW_INFO_INIT          {'V','5','x', 'x',0}
#define HW_INFO_INIT_VER      2
#define HW_INFO_INIT_REV      3
/* CAN Silence
 *
 * Silent mode control \ ESC Mux select
 */

#define SILENT_IOMUX (IOMUX_CMOS_OUTPUT | IOMUX_PULL_NONE | IOMUX_DRIVE_33OHM | IOMUX_SPEED_MAX | IOMUX_SLEW_FAST)
#define GPIO_CAN1_SILENT_S0  /* GPIO_AD_B0_10	GPIO1_IO10 */ (GPIO_PORT1 | GPIO_PIN10 | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | SILENT_IOMUX)
#define GPIO_CAN2_SILENT_S1  /* GPIO_EMC_06	GPIO4_IO06   */ (GPIO_PORT4 | GPIO_PIN6  | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | SILENT_IOMUX)
#define GPIO_CAN3_SILENT_S2  /* GPIO_EMC_09	GPIO4_IO09   */ (GPIO_PORT4 | GPIO_PIN9  | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | SILENT_IOMUX)

/* HEATER
 * PWM in future
 */
#define HEATER_IOMUX (IOMUX_CMOS_OUTPUT | IOMUX_PULL_NONE | IOMUX_DRIVE_50OHM | IOMUX_SPEED_MEDIUM | IOMUX_SLEW_FAST)
#define GPIO_HEATER_OUTPUT   /* GPIO_B1_09 QTIMER2_TIMER3 GPIO2_IO25 */ (GPIO_QTIMER2_TIMER3_1 | HEATER_IOMUX)

/* PWM Capture
 *
 * 2  PWM Capture inputs are supported
 */
#define DIRECT_PWM_CAPTURE_CHANNELS  2
#define CAP_IOMUX (IOMUX_CMOS_OUTPUT | IOMUX_PULL_NONE | IOMUX_DRIVE_50OHM | IOMUX_SPEED_MEDIUM | IOMUX_SLEW_FAST)
#define PIN_FLEXPWM2_PWMB0  /* P2:7  PWM2 B0 FMU_CAP1 */ (CAP_IOMUX | GPIO_FLEXPWM2_PWMB00_2)
#define PIN_FLEXPWM2_PWMB3  /* P3:3  PWM2 A1 FMU_CAP2 */ (CAP_IOMUX | GPIO_FLEXPWM2_PWMB03_3)

#define nARMED_INPUT_IOMUX  (IOMUX_CMOS_INPUT |  IOMUX_PULL_UP_22K | IOMUX_DRIVE_HIZ)
#define nARMED_OUTPUT_IOMUX (IOMUX_CMOS_OUTPUT | IOMUX_PULL_KEEP | IOMUX_DRIVE_33OHM  | IOMUX_SPEED_MEDIUM | IOMUX_SLEW_FAST)

#define GPIO_nARMED_INIT     /* GPIO_SD_B1_01 GPIO3_IO1 */ (GPIO_PORT3 | GPIO_PIN1 | GPIO_INPUT | nARMED_INPUT_IOMUX)
#define GPIO_nARMED          /* GPIO_SD_B1_01 GPIO3_IO1 */ (GPIO_PORT3 | GPIO_PIN1 | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | nARMED_OUTPUT_IOMUX)

#define BOARD_INDICATE_ARMED_STATE(on_armed)  px4_arch_configgpio((on_armed) ? GPIO_nARMED : GPIO_nARMED_INIT)

/* PWM
 */

#define DIRECT_PWM_OUTPUT_CHANNELS  8

// Input Capture not supported on MVP

#define DIRECT_INPUT_TIMER_CHANNELS  0

#define BOARD_HAS_LED_PWM              1
#define BOARD_LED_PWM_DRIVE_ACTIVE_LOW 1

/*  UI LEDs are driven by timer 4 the pins have no alternates
 *
 *  nUI_LED_RED   GPIO_B0_10 GPIO2_IO10 QTIMER4_TIMER1
 *  nUI_LED_GREEN GPIO_B0_11 GPIO2_IO11 QTIMER4_TIMER2
 *  nUI_LED_BLUE  GPIO_B1_11 GPIO2_IO27 QTIMER4_TIMER3
 */


/* Power supply control and monitoring GPIOs */

#define GENERAL_INPUT_IOMUX  (IOMUX_CMOS_INPUT |  IOMUX_PULL_UP_47K | IOMUX_DRIVE_HIZ)
#define GENERAL_OUTPUT_IOMUX (IOMUX_CMOS_OUTPUT | IOMUX_PULL_KEEP | IOMUX_DRIVE_33OHM  | IOMUX_SPEED_MEDIUM | IOMUX_SLEW_FAST)

#define GPIO_nPOWER_IN_A                /* GPIO_B0_12	GPIO2_IO12 */ (GPIO_PORT2 | GPIO_PIN12 | GPIO_INPUT | GENERAL_INPUT_IOMUX)
#define GPIO_nPOWER_IN_B                /* GPIO_B0_13	GPIO2_IO13 */ (GPIO_PORT2 | GPIO_PIN13 | GPIO_INPUT | GENERAL_INPUT_IOMUX)
#define GPIO_nPOWER_IN_C                /* GPIO_B0_14	GPIO2_IO14 */ (GPIO_PORT2 | GPIO_PIN14 | GPIO_INPUT | GENERAL_INPUT_IOMUX)

#define GPIO_nVDD_BRICK1_VALID          GPIO_nPOWER_IN_A /* Brick 1 Is Chosen */
#define GPIO_nVDD_BRICK2_VALID          GPIO_nPOWER_IN_B /* Brick 2 Is Chosen  */
#define BOARD_NUMBER_BRICKS             2
#define GPIO_nVDD_USB_VALID             GPIO_nPOWER_IN_C /* USB     Is Chosen */

#define OC_INPUT_IOMUX  (IOMUX_CMOS_INPUT |  IOMUX_PULL_NONE | IOMUX_DRIVE_HIZ)

#define GPIO_nVDD_5V_PERIPH_EN          /* GPIO_B1_04    GPIO2_IO20 */ (GPIO_PORT2 | GPIO_PIN20 | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GENERAL_OUTPUT_IOMUX)
#define GPIO_nVDD_5V_PERIPH_OC          /* GPIO_B1_03    GPIO2_IO19 */ (GPIO_PORT2 | GPIO_PIN19 | GPIO_INPUT  | OC_INPUT_IOMUX)
#define GPIO_nVDD_5V_HIPOWER_EN         /* GPIO_B1_02    GPIO2_IO18 */ (GPIO_PORT2 | GPIO_PIN18 | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GENERAL_OUTPUT_IOMUX)
#define GPIO_nVDD_5V_HIPOWER_OC         /* GPIO_B1_01    GPIO2_IO17 */ (GPIO_PORT2 | GPIO_PIN17 | GPIO_INPUT  | OC_INPUT_IOMUX)
#define GPIO_VDD_3V3_SENSORS_EN         /* GPIO_EMC_41   GPIO3_IO27 */ (GPIO_PORT3 | GPIO_PIN27 | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GENERAL_OUTPUT_IOMUX)
#define GPIO_VDD_3V3_SPEKTRUM_POWER_EN  /* GPIO_AD_B0_00 GPIO1_IO00 */ (GPIO_PORT1 | GPIO_PIN0  | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GENERAL_OUTPUT_IOMUX)
#define GPIO_VDD_5V_RC_EN               /* GPIO_AD_B0_08 GPIO1_IO08 */ (GPIO_PORT1 | GPIO_PIN8  | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GENERAL_OUTPUT_IOMUX)
#define GPIO_VDD_5V_WIFI_EN             /* PMIC_STBY_REQ GPIO5_IO02 */ (GPIO_PORT5 | GPIO_PIN2  | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GENERAL_OUTPUT_IOMUX)
#define GPIO_VDD_3V3_SD_CARD_EN         /* GPIO_EMC_13   GPIO4_IO13 */ (GPIO_PORT4 | GPIO_PIN13 | GPIO_OUTPUT | GPIO_OUTPUT_ZERO |GENERAL_OUTPUT_IOMUX)

/* Define True logic Power Control in arch agnostic form */

#define VDD_5V_PERIPH_EN(on_true)          px4_arch_gpiowrite(GPIO_nVDD_5V_PERIPH_EN, !(on_true))
#define VDD_5V_HIPOWER_EN(on_true)         px4_arch_gpiowrite(GPIO_nVDD_5V_HIPOWER_EN, !(on_true))
#define VDD_3V3_SENSORS_EN(on_true)        px4_arch_gpiowrite(GPIO_VDD_3V3_SENSORS_EN, (on_true))
#define VDD_3V3_SPEKTRUM_POWER_EN(on_true) px4_arch_gpiowrite(GPIO_VDD_3V3_SPEKTRUM_POWER_EN, (on_true))
#define READ_VDD_3V3_SPEKTRUM_POWER_EN()   px4_arch_gpioread(GPIO_VDD_3V3_SPEKTRUM_POWER_EN)
#define VDD_5V_RC_EN(on_true)              px4_arch_gpiowrite(GPIO_VDD_5V_RC_EN, (on_true))
#define VDD_5V_WIFI_EN(on_true)            px4_arch_gpiowrite(GPIO_VDD_5V_WIFI_EN, (on_true))
#define VDD_3V3_SD_CARD_EN(on_true)        px4_arch_gpiowrite(GPIO_VDD_3V3_SD_CARD_EN, (on_true))

/* Tone alarm output */

#define TONE_ALARM_TIMER        2  /* GPT 2 */
#define TONE_ALARM_CHANNEL      3  /* GPIO_AD_B1_07 GPT2_COMPARE3 */

#define GPIO_BUZZER_1           /* GPIO_AD_B1_07  GPIO1_IO23  */ (GPIO_PORT1 | GPIO_PIN23  | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GENERAL_OUTPUT_IOMUX)

#define GPIO_TONE_ALARM_IDLE    GPIO_BUZZER_1
#define GPIO_TONE_ALARM         (GPIO_GPT2_COMPARE3_2 | GENERAL_OUTPUT_IOMUX)

/* USB OTG FS
 *
 * VBUS_VALID is detected in USB_ANALOG_USB1_VBUS_DETECT_STAT
 */

/* High-resolution timer */
#define HRT_TIMER               1  /* use GPT1 for the HRT */
#define HRT_TIMER_CHANNEL       1  /* use capture/compare channel 1 */

#define HRT_PPM_CHANNEL         /* GPIO_B1_06 GPT1_CAPTURE2 */  2  /* use capture/compare channel 2 */
#define GPIO_PPM_IN             /* GPIO_B1_06 GPT1_CAPTURE2 */ (GPIO_GPT1_CAPTURE2_2 | GENERAL_INPUT_IOMUX)

#define RC_SERIAL_PORT          "/dev/ttyS5"

/* PWM input driver. Use FMU AUX5 pins attached to GPIO_EMC_33 GPIO3_IO19 FLEXPWM3_PWMA2 */

#define PWMIN_TIMER            /* FLEXPWM3_PWMA2 */  3
#define PWMIN_TIMER_CHANNEL    /* FLEXPWM3_PWMA2 */  2
#define GPIO_PWM_IN            /* GPIO_EMC_33 GPIO3_IO19 */ (GPIO_FLEXPWM3_PWMA02_1 | GENERAL_INPUT_IOMUX)

/* Shared pins Both FMU and PX4IO control/monitor
 * FMU Initializes these pins to passive input until it is known
 * if we have and PX4IO on board
 */

#define GPIO_RSSI_IN                       /* GPIO_AD_B1_10 GPIO1_IO26 */ (GPIO_PORT1 | GPIO_PIN26 | GPIO_INPUT | ADC_IOMUX)
#define GPIO_RSSI_IN_INIT                  /* GPIO_AD_B1_10 GPIO1_IO26 */ 0 /* Using 0 will Leave as ADC RSSI_IN */

/* Safety Switch is HW version dependent on having an PX4IO
 * So we init to a benign state with the _INIT definition
 * and provide the the non _INIT one for the driver to make a run time
 * decision to use it.
 */
#define SAFETY_INIT_IOMUX (IOMUX_CMOS_INPUT  | IOMUX_PULL_NONE | IOMUX_DRIVE_HIZ)
#define SAFETY_IOMUX      (IOMUX_CMOS_OUTPUT | IOMUX_PULL_NONE | IOMUX_DRIVE_33OHM  | IOMUX_SPEED_MEDIUM | IOMUX_SLEW_SLOW)
#define SAFETY_SW_IOMUX   (IOMUX_CMOS_INPUT  | IOMUX_PULL_UP_22K | IOMUX_DRIVE_HIZ)

#define GPIO_nSAFETY_SWITCH_LED_OUT_INIT   /* GPIO_B0_15 GPIO2_IO15 */ (GPIO_PORT2 | GPIO_PIN15 | GPIO_INPUT  | SAFETY_INIT_IOMUX)
#define GPIO_nSAFETY_SWITCH_LED_OUT        /* GPIO_B0_15 GPIO2_IO15 */ (GPIO_PORT2 | GPIO_PIN15 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | SAFETY_IOMUX)

/* Enable the FMU to control it if there is no px4io fixme:This should be BOARD_SAFETY_LED(__ontrue) */
#define GPIO_LED_SAFETY GPIO_nSAFETY_SWITCH_LED_OUT
#define GPIO_SAFETY_SWITCH_IN              /* GPIO_AD_B1_12 GPIO1_IO28 */ (GPIO_PORT1 | GPIO_PIN28 | GPIO_INPUT | SAFETY_SW_IOMUX)
/* Enable the FMU to use the switch it if there is no px4io fixme:This should be BOARD_SAFTY_BUTTON() */
#define GPIO_BTN_SAFETY GPIO_SAFETY_SWITCH_IN /* Enable the FMU to control it if there is no px4io */

/*
 * FMUv5 has a separate RC_IN
 *
 * GPIO PPM_IN on GPIO_EMC_23 GPIO4 Pin 23 GPT1_CAPTURE2
 *   Inversion is possible in the UART and can drive GPIO PPM_IN as an output
 */

#define GPIO_PPM_IN_AS_OUT             /* GPIO_B1_06 GPIO2_IO23 GPT1_CAPTURE2 GPT1_CAPTURE2 */ (GPIO_PORT2 | GPIO_PIN23 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | GENERAL_OUTPUT_IOMUX)

#define SDIO_SLOTNO                    0  /* Only one slot */
#define SDIO_MINOR                     0

/* SD card bringup does not work if performed on the IDLE thread because it
 * will cause waiting.  Use either:
 *
 *  CONFIG_LIB_BOARDCTL=y, OR
 *  CONFIG_BOARD_INITIALIZE=y && CONFIG_BOARD_INITTHREAD=y
 */

#if defined(CONFIG_BOARD_INITIALIZE) && !defined(CONFIG_LIB_BOARDCTL) && \
   !defined(CONFIG_BOARD_INITTHREAD)
#  warning SDIO initialization cannot be perfomed on the IDLE thread
#endif

/* By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */

#define BOARD_ADC_USB_VALID     (!px4_arch_gpioread(GPIO_nVDD_USB_VALID))
#define BOARD_ADC_USB_CONNECTED (board_read_VBUS_state() == 0)

/* FMUv5 never powers odd the Servo rail */

#define BOARD_ADC_SERVO_VALID     (1)

#define BOARD_ADC_BRICK1_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK1_VALID))
#define BOARD_ADC_BRICK2_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK2_VALID))

#define BOARD_ADC_PERIPH_5V_OC  (!px4_arch_gpioread(GPIO_nVDD_5V_PERIPH_OC))
#define BOARD_ADC_HIPOWER_5V_OC (!px4_arch_gpioread(GPIO_nVDD_5V_HIPOWER_OC))

#define BOARD_HAS_PWM  DIRECT_PWM_OUTPUT_CHANNELS

/* This board provides a DMA pool and APIs */

#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */

#define BOARD_HAS_ON_RESET 1

#define PX4_GPIO_INIT_LIST { \
		GPIO_nARMED_INIT,                 \
		PX4_ADC_GPIO,                     \
		GPIO_HW_VER_REV_DRIVE,            \
		GPIO_FLEXCAN1_TX,                 \
		GPIO_FLEXCAN1_RX,                 \
		GPIO_FLEXCAN2_TX,                 \
		GPIO_FLEXCAN2_RX,                 \
		GPIO_FLEXCAN3_TX,                 \
		GPIO_FLEXCAN3_RX,                 \
		GPIO_CAN1_SILENT_S0,              \
		GPIO_CAN2_SILENT_S1,              \
		GPIO_CAN3_SILENT_S2,              \
		GPIO_HEATER_OUTPUT,               \
		GPIO_nPOWER_IN_A,                 \
		GPIO_nPOWER_IN_B,                 \
		GPIO_nPOWER_IN_C,                 \
		GPIO_nVDD_5V_PERIPH_EN,           \
		GPIO_nVDD_5V_PERIPH_OC,           \
		GPIO_nVDD_5V_HIPOWER_EN,          \
		GPIO_nVDD_5V_HIPOWER_OC,          \
		GPIO_VDD_3V3_SENSORS_EN,          \
		GPIO_VDD_3V3_SD_CARD_EN,          \
		GPIO_VDD_3V3_SPEKTRUM_POWER_EN,   \
		GPIO_VDD_5V_RC_EN,                \
		GPIO_VDD_5V_WIFI_EN,              \
		GPIO_TONE_ALARM_IDLE,             \
		GPIO_RSSI_IN_INIT,                \
		GPIO_nSAFETY_SWITCH_LED_OUT_INIT, \
		GPIO_nSPI4_RESET_EXTERNAL1,       \
		GPIO_SPI4_SYNC_EXTERNAL1,         \
		GPIO_SAFETY_SWITCH_IN             \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER
__BEGIN_DECLS

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************
 * Name: fmurt1062_usdhc_initialize
 *
 * Description:
 *   Initialize SDIO-based MMC/SD card support
 *
 ****************************************************************************/

int fmurt1062_usdhc_initialize(void);

/****************************************************************************************************
 * Name: imxrt_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ****************************************************************************************************/

extern void imxrt_spidev_initialize(void);

/************************************************************************************
 * Name: imxrt_spi_bus_initialize
 *
 * Description:
 *   Called to configure SPI Buses.
 *
 ************************************************************************************/

extern int imxrt1062_spi_bus_initialize(void);

/************************************************************************************
 * Name: imxrt_usb_initialize
 *
 * Description:
 *   Called to configure USB.
 *
 ************************************************************************************/

extern int imxrt_usb_initialize(void);

extern void imxrt_usbinitialize(void);

extern void board_peripheral_reset(int ms);

extern void fmurt1062_timer_initialize(void);
/****************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization for NSH.
 *
 *   CONFIG_NSH_ARCHINIT=y :
 *     Called from the NSH library
 *
 *   CONFIG_BOARD_INITIALIZE=y, CONFIG_NSH_LIBRARY=y, &&
 *   CONFIG_NSH_ARCHINIT=n :
 *     Called from board_initialize().
 *
 ****************************************************************************/

#ifdef CONFIG_NSH_LIBRARY
int nsh_archinitialize(void);
#endif

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
