/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * PX4FMU-v5 internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <stm32_gpio.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

/* PX4IO connection configuration */

#define BOARD_USES_PX4IO_VERSION       2
#define PX4IO_SERIAL_DEVICE            "/dev/ttyS6"
#define PX4IO_SERIAL_TX_GPIO           GPIO_UART8_TX
#define PX4IO_SERIAL_RX_GPIO           GPIO_UART8_RX
#define PX4IO_SERIAL_BASE              STM32_UART8_BASE
#define PX4IO_SERIAL_VECTOR            STM32_IRQ_UART8
#define PX4IO_SERIAL_TX_DMAMAP         DMAMAP_UART8_TX
#define PX4IO_SERIAL_RX_DMAMAP         DMAMAP_UART8_RX
#define PX4IO_SERIAL_RCC_REG           STM32_RCC_APB1ENR
#define PX4IO_SERIAL_RCC_EN            RCC_APB1ENR_UART8EN
#define PX4IO_SERIAL_CLOCK             STM32_PCLK1_FREQUENCY
#define PX4IO_SERIAL_BITRATE           1500000               /* 1.5Mbps -> max rate for IO */

/* Configuration ************************************************************************************/

#define BOARD_HAS_LTC4417

#if defined(BOARD_HAS_LTC4417)
#  define BOARD_HAS_LTC44XX_VALIDS      2 // No LTC or N Bricks
#  define BOARD_HAS_USB_VALID           1 // LTC Has No USB valid
#  define BOARD_HAS_NBAT_V              2 // Only one Vbat to ADC
#  define BOARD_HAS_NBAT_I              2 // No Ibat ADC
#else
#  define BOARD_HAS_LTC44XX_VALIDS      0 // No LTC or N Bricks
#  define BOARD_HAS_USB_VALID           0 // LTC Has No USB valid
#  define BOARD_HAS_NBAT_V              1 // Only one Vbat to ADC
#  define BOARD_HAS_NBAT_I              0 // No Ibat ADC
#endif

/* PX4FMU GPIOs ***********************************************************************************/

/* LEDs are driven with push open drain to support Anode to 5V or 3.3V */

#define GPIO_nLED_RED        /* PB1 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN1)
#define GPIO_nLED_GREEN      /* PC6 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN6)
#define GPIO_nLED_BLUE       /* PC7 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN7)

#define BOARD_HAS_CONTROL_STATUS_LEDS      1
#define BOARD_OVERLOAD_LED     LED_RED
#define BOARD_ARMED_STATE_LED  LED_BLUE

/* SENSORS are on SPI1, 5, 6
 * MEMORY is on bus SPI2
 * MS5611 is on bus SPI4
 */

#define PX4_SPI_BUS_SENSORS   1
#define PX4_SPI_BUS_MEMORY    2
#define PX4_SPI_BUS_BARO      4
#define PX4_SPI_BUS_EXTERNAL1 5
#define PX4_SPI_BUS_EXTERNAL2 6

/*  Define the Chip Selects, Data Ready and Control signals per SPI bus */

/* SPI 1 CS */

#define GPIO_SPI1_CS1_ICM20689    /* PF2 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN2)
#define GPIO_SPI1_CS2_ICM20602    /* PF3 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN3)
#define GPIO_SPI1_CS3_BMI055_GYRO /* PF4 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN4)
#define GPIO_SPI1_CS4_BMI055_ACC  /* PG10 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN10)
#define GPIO_SPI1_CS5_AUX_MEM     /* PH5  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTH|GPIO_PIN5)


/*  Define the SPI1 Data Ready interrupts */

#define GPIO_SPI1_DRDY1_ICM20689    /* PB4 */   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTB|GPIO_PIN4)
#define GPIO_SPI1_DRDY2_BMI055_GYRO /* PB14 */  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTB|GPIO_PIN14)
#define GPIO_SPI1_DRDY3_BMI055_ACC  /* PB15 */  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTB|GPIO_PIN15)
#define GPIO_SPI1_DRDY4_ICM20602    /* PC5  */  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN5)
#define GPIO_SPI1_DRDY5_BMI055_GYRO /* PC13 */  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN13)
#define GPIO_SPI1_DRDY6_BMI055_ACC  /* PD10 */  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN10)

/* SPI1 off */

#define GPIO_SPI1_SCK_OFF  _PIN_OFF(GPIO_SPI1_SCK)
#define GPIO_SPI1_MISO_OFF  _PIN_OFF(GPIO_SPI1_MISO)
#define GPIO_SPI1_MOSI_OFF  _PIN_OFF(GPIO_SPI1_MOSI)

#define GPIO_DRDY_OFF_SPI1_DRDY1_ICM20689    _PIN_OFF(GPIO_SPI1_DRDY1_ICM20689)
#define GPIO_DRDY_OFF_SPI1_DRDY2_BMI055_GYRO _PIN_OFF(GPIO_SPI1_DRDY2_BMI055_GYRO)
#define GPIO_DRDY_OFF_SPI1_DRDY3_BMI055_ACC  _PIN_OFF(GPIO_SPI1_DRDY3_BMI055_ACC)
#define GPIO_DRDY_OFF_SPI1_DRDY4_ICM20602    _PIN_OFF(GPIO_SPI1_DRDY4_ICM20602)
#define GPIO_DRDY_OFF_SPI1_DRDY5_BMI055_GYRO _PIN_OFF(GPIO_SPI1_DRDY5_BMI055_GYRO)
#define GPIO_DRDY_OFF_SPI1_DRDY6_BMI055_ACC  _PIN_OFF(GPIO_SPI1_DRDY6_BMI055_ACC)

/* SPI 2 CS */

#define GPIO_SPI2_CS_FRAM         /* PF5 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN5)

/* SPI 4 CS */

#define GPIO_SPI4_CS1_MS5611      /* PF10 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN10)
#define GPIO_SPI4_CS2             /* PF11 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN11)

/* SPI 5 CS */

#define SPI5_CS1_EXTERNAL1     /* PI4  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN4)
#define SPI5_CS2_EXTERNAL1     /* PI10 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN10)
#define SPI5_CS3_EXTERNAL1     /* PI11 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN11)

/*  Define the SPI1 Data Ready and Control signals */

#define GPIO_SPI5_DRDY7_EXTERNAL1   /* PD15 */  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN15)
#define GPIO_nSPI5_RESET_EXTERNAL1  /* PB10 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN10)
#define GPIO_SPI5_SYNC_EXTERNAL1    /* PH15 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|PORTH|PIN15)

/* SPI 6 */

#define SPI6_CS1_EXTERNAL2     /* PI6  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN6)
#define SPI6_CS2_EXTERNAL2     /* PI7  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN7)
#define SPI6_CS3_EXTERNAL2     /* PI8  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN8)


/*
 *  Define the ability to shut off off the sensor signals
 *  by changing the signals to inputs
 */

#define _PIN_OFF(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz))


#define GPIO_DRDY_OFF_SPI5_DRDY7_EXTERNAL1   _PIN_OFF(GPIO_SPI5_DRDY7_EXTERNAL1)


/* v BEGIN Legacy SPI defines TODO: fix this with enumeration */
#define PX4_SPI_BUS_RAMTRON  PX4_SPI_BUS_MEMORY
/* ^ END Legacy SPI defines TODO: fix this with enumeration */
#include <drivers/drv_sensor.h>

#define PX4_SPIDEV_ICM_20689        PX4_MK_SPI_SEL(0,DRV_IMU_DEVTYPE_ICM20689)
#define PX4_SPIDEV_ICM_20602        PX4_MK_SPI_SEL(0,DRV_IMU_DEVTYPE_ICM20602)
#define PX4_SPIDEV_BMI055_GYR       PX4_MK_SPI_SEL(0,DRV_GYR_DEVTYPE_BMI055)
#define PX4_SPIDEV_BMI055_ACC       PX4_MK_SPI_SEL(0,DRV_ACC_DEVTYPE_BMI055)
#define PX4_SPIDEV_AUX_MEM          PX4_MK_SPI_SEL(0,4)
#define PX4_SENSOR_BUS_CS_GPIO      {GPIO_SPI1_CS1_ICM20689, GPIO_SPI1_CS2_ICM20602, GPIO_SPI1_CS3_BMI055_GYRO, GPIO_SPI1_CS4_BMI055_ACC, GPIO_SPI1_CS5_AUX_MEM}

#define PX4_SPIDEV_MEMORY           SPIDEV_FLASH(0)
#define PX4_MEMORY_BUS_CS_GPIO      {GPIO_SPI2_CS_FRAM}

#define PX4_SPIDEV_BARO             PX4_MK_SPI_SEL(0,DRV_BARO_DEVTYPE_MS5611)
#define PX4_SPIDEV_SPI4_CS2         PX4_MK_SPI_SEL(PX4_SPI_BUS_BARO,1)
#define PX4_BARO_BUS_CS_GPIO        {GPIO_SPI4_CS1_MS5611, GPIO_SPI4_CS2}

#define PX4_SPIDEV_EXTERNAL1_1      PX4_MK_SPI_SEL(PX4_SPI_BUS_EXTERNAL1,0)
#define PX4_SPIDEV_EXTERNAL1_2      PX4_MK_SPI_SEL(PX4_SPI_BUS_EXTERNAL1,1)
#define PX4_SPIDEV_EXTERNAL1_3      PX4_MK_SPI_SEL(PX4_SPI_BUS_EXTERNAL1,2)
#define PX4_EXTERNAL1_BUS_CS_GPIO   {SPI5_CS1_EXTERNAL1, SPI5_CS2_EXTERNAL1, SPI5_CS3_EXTERNAL1}

#define PX4_SPIDEV_EXTERNAL2_1      PX4_MK_SPI_SEL(PX4_SPI_BUS_EXTERNAL2,0)
#define PX4_SPIDEV_EXTERNAL2_2      PX4_MK_SPI_SEL(PX4_SPI_BUS_EXTERNAL2,1)
#define PX4_SPIDEV_EXTERNAL2_3      PX4_MK_SPI_SEL(PX4_SPI_BUS_EXTERNAL2,2)
#define PX4_EXTERNAL2_BUS_CS_GPIO   {SPI6_CS1_EXTERNAL2, SPI6_CS2_EXTERNAL2, SPI6_CS3_EXTERNAL2}


/* I2C busses */

#define PX4_I2C_BUS_EXPANSION       1
#define PX4_I2C_BUS_EXPANSION1      2
#define PX4_I2C_BUS_ONBOARD         3
#define PX4_I2C_BUS_EXPANSION2      4
#define PX4_I2C_BUS_LED             PX4_I2C_BUS_EXPANSION

#define BOARD_NUMBER_I2C_BUSES      4
#define BOARD_I2C_BUS_CLOCK_INIT    {100000, 100000, 100000, 100000}

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that
 * can be used by the Px4 Firmware in the adc driver
 */

/* ADC defines to be used in sensors.cpp to read from a particular channel */

#define ADC1_CH(n)                  (n)
#define ADC1_GPIO(n)                GPIO_ADC1_IN##n

/* Define GPIO pins used as ADC N.B. Channel numbers must match below */

#define PX4_ADC_GPIO  \
	/* PA0 */  ADC1_GPIO(0),  \
	/* PA1 */  ADC1_GPIO(1),  \
	/* PA2 */  ADC1_GPIO(2),  \
	/* PA3 */  ADC1_GPIO(3),  \
	/* PA4 */  ADC1_GPIO(4),  \
	/* PB8 */  ADC1_GPIO(8),  \
	/* PC0 */  ADC1_GPIO(10), \
	/* PC1 */  ADC1_GPIO(11), \
	/* PC2 */  ADC1_GPIO(12), \
	/* PC3 */  ADC1_GPIO(13), \
	/* PC4 */  ADC1_GPIO(14)

/* Define Channel numbers must match above GPIO pin IN(n)*/

#define ADC_BATTERY1_VOLTAGE_CHANNEL        /* PA0 */  ADC1_CH(0)
#define ADC_BATTERY1_CURRENT_CHANNEL        /* PA1 */  ADC1_CH(1)
#define ADC_BATTERY2_VOLTAGE_CHANNEL        /* PA2 */  ADC1_CH(2)
#define ADC_BATTERY2_CURRENT_CHANNEL        /* PA3 */  ADC1_CH(3)
#define ADC1_SPARE_2_CHANNEL                /* PA4 */  ADC1_CH(4)
#define ADC_RSSI_IN_CHANNEL                 /* PB0 */  ADC1_CH(8)
#define ADC_SCALED_V5_CHANNEL               /* PC0 */  ADC1_CH(10)
#define ADC_SCALED_VDD_3V3_SENSORS_CHANNEL  /* PC1 */  ADC1_CH(11)
#define ADC_HW_VER_SENSE_CHANNEL            /* PC2 */  ADC1_CH(12)
#define ADC_HW_REV_SENSE_CHANNEL            /* PC3 */  ADC1_CH(13)
#define ADC1_SPARE_1_CHANNEL                /* PC4 */  ADC1_CH(14)

#if BOARD_HAS_NBAT_V == 2 && BOARD_HAS_NBAT_I == 2
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
#elif BOARD_HAS_NBAT_V == 1 && BOARD_HAS_NBAT_I == 1
#define ADC_CHANNELS \
	((1 << ADC_BATTERY1_VOLTAGE_CHANNEL)       | \
	 (1 << ADC_BATTERY1_CURRENT_CHANNEL)       | \
	 (1 << ADC1_SPARE_2_CHANNEL)               | \
	 (1 << ADC_RSSI_IN_CHANNEL)                | \
	 (1 << ADC_SCALED_V5_CHANNEL)              | \
	 (1 << ADC_SCALED_VDD_3V3_SENSORS_CHANNEL) | \
	 (1 << ADC_HW_VER_SENSE_CHANNEL)           | \
	 (1 << ADC_HW_REV_SENSE_CHANNEL)           | \
	 (1 << ADC1_SPARE_1_CHANNEL))
#elif BOARD_HAS_NBAT_V == 1 && BOARD_HAS_NBAT_I == 0
#define ADC_CHANNELS \
	((1 << ADC_BATTERY1_VOLTAGE_CHANNEL)       | \
	 (1 << ADC1_SPARE_2_CHANNEL)               | \
	 (1 << ADC_RSSI_IN_CHANNEL)                | \
	 (1 << ADC_SCALED_V5_CHANNEL)              | \
	 (1 << ADC_SCALED_VDD_3V3_SENSORS_CHANNEL) | \
	 (1 << ADC_HW_VER_SENSE_CHANNEL)           | \
	 (1 << ADC_HW_REV_SENSE_CHANNEL)           | \
	 (1 << ADC1_SPARE_1_CHANNEL))
#endif

/* Define Battery 1 Voltage Divider and A per V
 */

#define BOARD_BATTERY1_V_DIV         (18.1f)     /* measured with the provided PM board */
#define BOARD_BATTERY1_A_PER_V       (36.367515152f)

/* HW has to large of R termination on ADC todo:change when HW value is chosen */

#define BOARD_ADC_OPEN_CIRCUIT_V     (5.6f)

/* HW Version and Revision drive signals Default to 1 to detect */

#define BOARD_HAS_HW_VERSIONING

#define GPIO_HW_REV_DRIVE    /* PH14  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTH|GPIO_PIN14)
#define GPIO_HW_REV_SENSE    /* PC3   */ ADC1_GPIO(13)
#define GPIO_HW_VER_DRIVE    /* PG0   */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN0)
#define GPIO_HW_VER_SENSE    /* PC2   */ ADC1_GPIO(12)
#define HW_INFO_INIT         {'V','5','x', 'x',0}
#define HW_INFO_INIT_VER     2
#define HW_INFO_INIT_REV     3
/* CAN Silence
 *
 * Silent mode control \ ESC Mux select
 */

#define GPIO_CAN1_SILENT_S0  /* PH2  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN2)
#define GPIO_CAN2_SILENT_S1  /* PH3  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN3)
#define GPIO_CAN3_SILENT_S2  /* PH4  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN4)

#define UAVCAN_NUM_IFACES_RUNTIME 1

/* HEATER
 * PWM in future
 */
#define GPIO_HEATER_OUTPUT   /* PA7  T14CH1 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN7)

/* PWM Capture
 *
 * 3  PWM Capture inputs are configured.
 *
 * Pins:
 *
 * FMU_CAP1 : PA5  : TIM2_CH1
 * FMU_CAP2 : PB3  : TIM2_CH2
 * FMU_CAP3 : PB11 : TIM2_CH4
 */
#define GPIO_TIM2_CH1_IN     /* PA5   T22C1  FMU_CAP1 */ GPIO_TIM2_CH1IN_3
#define GPIO_TIM2_CH2_IN     /* PB3   T22C2  FMU_CAP2 */ GPIO_TIM2_CH2IN_2
#define GPIO_TIM2_CH4_IN     /* PB11  T22C4  FMU_CAP3 */ GPIO_TIM2_CH4IN_2

#define DIRECT_PWM_CAPTURE_CHANNELS  3

/* PI0 is nARMED
 *  The GPIO will be set as input while not armed HW will have external HW Pull UP.
 *  While armed it shall be configured at a GPIO OUT set LOW
 */
#define GPIO_nARMED_INIT     /* PI0 */  (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTI|GPIO_PIN0)
#define GPIO_nARMED          /* PI0 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTI|GPIO_PIN0)

#define BOARD_INDICATE_ARMED_STATE(on_armed)  px4_arch_configgpio((on_armed) ? GPIO_nARMED : GPIO_nARMED_INIT)

/* PWM
 */
#define DIRECT_PWM_OUTPUT_CHANNELS  8
#define DIRECT_INPUT_TIMER_CHANNELS  8

#define BOARD_HAS_LED_PWM              1
#define BOARD_LED_PWM_DRIVE_ACTIVE_LOW 1

#define BOARD_HAS_UI_LED_PWM            1

#define BOARD_UI_LED_PWM_DRIVE_ACTIVE_LOW 1


/* Power supply control and monitoring GPIOs */

#define GPIO_nPOWER_IN_A                /* PG1  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN1)
#define GPIO_nPOWER_IN_B                /* PG2  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN2)
#define GPIO_nPOWER_IN_C                /* PG3  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN3)

#define GPIO_nVDD_BRICK1_VALID          GPIO_nPOWER_IN_A /* Brick 1 Is Chosen */
#define GPIO_nVDD_BRICK2_VALID          GPIO_nPOWER_IN_B /* Brick 2 Is Chosen  */
#define BOARD_NUMBER_BRICKS             2
#define GPIO_nVDD_USB_VALID             GPIO_nPOWER_IN_C /* USB     Is Chosen */

#define GPIO_nVDD_5V_PERIPH_EN          /* PG4  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN4)
#define GPIO_nVDD_5V_PERIPH_OC          /* PE15 */ (GPIO_INPUT |GPIO_FLOAT|GPIO_PORTE|GPIO_PIN15)
#define GPIO_nVDD_5V_HIPOWER_EN         /* PF12 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN12)
#define GPIO_nVDD_5V_HIPOWER_OC         /* PG13 */ (GPIO_INPUT |GPIO_FLOAT|GPIO_PORTF|GPIO_PIN13)
#define GPIO_VDD_3V3_SPEKTRUM_POWER_EN  /* PE4  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN4)
#define GPIO_VDD_5V_RC_EN               /* PG5  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN5)
#define GPIO_VDD_5V_WIFI_EN             /* PG6  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN5)
#define GPIO_VDD_3V3_SD_CARD_EN         /* PG7  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN7)


/* Define True logic Power Control in arch agnostic form */

#define VDD_5V_PERIPH_EN(on_true)          px4_arch_gpiowrite(GPIO_nVDD_5V_PERIPH_EN, !(on_true))
#define VDD_5V_HIPOWER_EN(on_true)         px4_arch_gpiowrite(GPIO_nVDD_5V_HIPOWER_EN, !(on_true))
#define VDD_3V3_SPEKTRUM_POWER_EN(on_true) px4_arch_gpiowrite(GPIO_VDD_3V3_SPEKTRUM_POWER_EN, (on_true))
#define READ_VDD_3V3_SPEKTRUM_POWER_EN()   px4_arch_gpioread(GPIO_VDD_3V3_SPEKTRUM_POWER_EN)
#define VDD_5V_RC_EN(on_true)              px4_arch_gpiowrite(GPIO_VDD_5V_RC_EN, (on_true))
#define VDD_5V_WIFI_EN(on_true)            px4_arch_gpiowrite(GPIO_VDD_5V_WIFI_EN, (on_true))
#define VDD_3V3_SD_CARD_EN(on_true)        px4_arch_gpiowrite(GPIO_VDD_3V3_SD_CARD_EN, (on_true))

/* Tone alarm output */

#define TONE_ALARM_TIMER        9  /* timer 9 */
#define TONE_ALARM_CHANNEL      1  /* PE5 TIM9_CH1 */

#define GPIO_BUZZER_1           /* PE5 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN5)

#define GPIO_TONE_ALARM_IDLE    GPIO_BUZZER_1
#define GPIO_TONE_ALARM         GPIO_TIM9_CH1OUT_2

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS         /* PA9 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_100MHz|GPIO_PORTA|GPIO_PIN9)

/* High-resolution timer */
#define HRT_TIMER               8  /* use timer8 for the HRT */
#define HRT_TIMER_CHANNEL       3  /* use capture/compare channel 3 */

#define HRT_PPM_CHANNEL         /* T8C1 */  1  /* use capture/compare channel 1 */
#define GPIO_PPM_IN             /* PI5 T8C1 */ GPIO_TIM8_CH1IN_2

/* RC Serial port */

#define RC_SERIAL_PORT                     "/dev/ttyS4"
#define RC_SERIAL_SINGLEWIRE

/* Input Capture Channels. */
#define INPUT_CAP1_TIMER                  2
#define INPUT_CAP1_CHANNEL     /* T4C1 */ 1
#define GPIO_INPUT_CAP1        /*  PA5 */ GPIO_TIM2_CH1_IN

#define INPUT_CAP2_TIMER                  2
#define INPUT_CAP2_CHANNEL     /* T4C2 */ 2
#define GPIO_INPUT_CAP2        /*  PB3 */ GPIO_TIM2_CH2_IN

#define INPUT_CAP3_TIMER                  2
#define INPUT_CAP3_CHANNEL     /* T4C4 */ 4
#define GPIO_INPUT_CAP3        /* PB11 */ GPIO_TIM2_CH4_IN

/* PWM input driver. Use FMU AUX5 pins attached to timer4 channel 2 */
#define PWMIN_TIMER                       4
#define PWMIN_TIMER_CHANNEL    /* T4C2 */ 2
#define GPIO_PWM_IN            /* PD13 */ GPIO_TIM4_CH2IN_2

/* Shared pins Both FMU and PX4IO control/monitor
 * FMU Initializes these pins to passive input until it is known
 * if we have and PX4IO on board
 */

#define GPIO_RSSI_IN                       /* PB0  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN0)
#define GPIO_RSSI_IN_INIT                  /* PB0  */ 0 /* Leave as ADC RSSI_IN */

/* Safety Switch is HW version dependent on having an PX4IO
 * So we init to a benign state with the _INIT definition
 * and provide the the non _INIT one for the driver to make a run time
 * decision to use it.
 */
#define GPIO_nSAFETY_SWITCH_LED_OUT_INIT   /* PE12 */ (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTE|GPIO_PIN12)
#define GPIO_nSAFETY_SWITCH_LED_OUT        /* PE12 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN12)

/* Enable the FMU to control it if there is no px4io fixme:This should be BOARD_SAFETY_LED(__ontrue) */
#define GPIO_LED_SAFETY GPIO_nSAFETY_SWITCH_LED_OUT
#define GPIO_SAFETY_SWITCH_IN              /* PE10 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN10)
/* Enable the FMU to use the switch it if there is no px4io fixme:This should be BOARD_SAFTY_BUTTON() */
#define GPIO_BTN_SAFETY GPIO_SAFETY_SWITCH_IN /* Enable the FMU to control it if there is no px4io */

/* Power switch controls ******************************************************/

#define SPEKTRUM_POWER(_on_true)           VDD_3V3_SPEKTRUM_POWER_EN(_on_true)

/*
 * FMUv5 has a separate RC_IN
 *
 * GPIO PPM_IN on PI5 T8CH1
 * SPEKTRUM_RX (it's TX or RX in Bind) on UART6 PG9 (NOT FMUv5 test HW ONLY)
 *   In version is possible in the UART
 * and can drive  GPIO PPM_IN as an output
 */

#define GPIO_PPM_IN_AS_OUT             (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN5)
#define SPEKTRUM_RX_AS_GPIO_OUTPUT()   px4_arch_configgpio(GPIO_PPM_IN_AS_OUT)
#define SPEKTRUM_RX_AS_UART()          /* Can be left as uart */
#define SPEKTRUM_OUT(_one_true)        px4_arch_gpiowrite(GPIO_PPM_IN_AS_OUT, (_one_true))

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
#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_OTGFS_VBUS))

#if BOARD_HAS_USB_VALID == 1
#  define BOARD_ADC_USB_VALID     (!px4_arch_gpioread(GPIO_nVDD_USB_VALID))
#else
#  define BOARD_ADC_USB_VALID     BOARD_ADC_USB_CONNECTED
#endif

/* FMUv5 never powers odd the Servo rail */

#define BOARD_ADC_SERVO_VALID     (1)

#if !defined(BOARD_HAS_LTC44XX_VALIDS) || BOARD_HAS_LTC44XX_VALIDS == 0
#  define BOARD_ADC_BRICK1_VALID  (1)
#  define BOARD_ADC_BRICK2_VALID  (0)
#elif BOARD_HAS_LTC44XX_VALIDS == 1
#  define BOARD_ADC_BRICK1_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK1_VALID))
#  define BOARD_ADC_BRICK2_VALID  (0)
#elif BOARD_HAS_LTC44XX_VALIDS == 2
#  define BOARD_ADC_BRICK1_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK1_VALID))
#  define BOARD_ADC_BRICK2_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK2_VALID))
#elif BOARD_HAS_LTC44XX_VALIDS == 3
#  define BOARD_ADC_BRICK1_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK1_VALID))
#  define BOARD_ADC_BRICK2_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK2_VALID))
#  define BOARD_ADC_BRICK3_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK3_VALID))
#elif BOARD_HAS_LTC44XX_VALIDS == 4
#  define BOARD_ADC_BRICK1_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK1_VALID))
#  define BOARD_ADC_BRICK2_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK2_VALID))
#  define BOARD_ADC_BRICK3_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK3_VALID))
#  define BOARD_ADC_BRICK4_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK4_VALID))
#else
#  error Unsupported BOARD_HAS_LTC44XX_VALIDS value
#endif

#define BOARD_ADC_PERIPH_5V_OC  (!px4_arch_gpioread(GPIO_nVDD_5V_PERIPH_OC))
#define BOARD_ADC_HIPOWER_5V_OC (!px4_arch_gpioread(GPIO_nVDD_5V_HIPOWER_OC))

#define BOARD_HAS_PWM  DIRECT_PWM_OUTPUT_CHANNELS

/* This board provides a DMA pool and APIs */

#define BOARD_DMA_ALLOC_POOL_SIZE (5120 + 1024 + 1024)	// 5120 fat + 1024 + 1024 spi

/* This board provides the board_on_reset interface */

#define BOARD_HAS_ON_RESET 1

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,                     \
		GPIO_HW_REV_DRIVE,                \
		GPIO_HW_VER_DRIVE,                \
		GPIO_CAN1_TX,                     \
		GPIO_CAN1_RX,                     \
		GPIO_CAN2_TX,                     \
		GPIO_CAN2_RX,                     \
		GPIO_CAN3_TX,                     \
		GPIO_CAN3_RX,                     \
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
		GPIO_VDD_3V3_SPEKTRUM_POWER_EN,   \
		GPIO_VDD_5V_RC_EN,                \
		GPIO_VDD_5V_WIFI_EN,              \
		GPIO_VDD_3V3_SD_CARD_EN,          \
		GPIO_TONE_ALARM_IDLE,             \
		GPIO_RSSI_IN_INIT,                \
		GPIO_nSAFETY_SWITCH_LED_OUT_INIT, \
		GPIO_SAFETY_SWITCH_IN,            \
		GPIO_nARMED_INIT                  \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER

#define BOARD_NUM_IO_TIMERS 5

#define BOARD_DSHOT_MOTOR_ASSIGNMENT {3, 2, 1, 0, 4, 5, 6, 7};

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
 * Name: stm32_sdio_initialize
 *
 * Description:
 *   Initialize SDIO-based MMC/SD card support
 *
 ****************************************************************************/

int stm32_sdio_initialize(void);

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

extern void stm32_usbinitialize(void);

extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
