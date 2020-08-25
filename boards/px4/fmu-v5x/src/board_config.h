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
#define PX4IO_SERIAL_DEVICE            "/dev/ttyS5"
#define PX4IO_SERIAL_TX_GPIO           GPIO_USART6_TX
#define PX4IO_SERIAL_RX_GPIO           GPIO_USART6_RX
#define PX4IO_SERIAL_BASE              STM32_USART6_BASE
#define PX4IO_SERIAL_VECTOR            STM32_IRQ_USART6
#define PX4IO_SERIAL_TX_DMAMAP         DMAMAP_USART6_TX
#define PX4IO_SERIAL_RX_DMAMAP         DMAMAP_USART6_RX
#define PX4IO_SERIAL_RCC_REG           STM32_RCC_APB2ENR
#define PX4IO_SERIAL_RCC_EN            RCC_APB2ENR_USART6EN
#define PX4IO_SERIAL_CLOCK             STM32_PCLK2_FREQUENCY
#define PX4IO_SERIAL_BITRATE           1500000               /* 1.5Mbps -> max rate for IO */

/* Configuration ************************************************************************************/

#define BOARD_HAS_LTC44XX_VALIDS      2 //  N Bricks
#define BOARD_HAS_USB_VALID           1 // LTC Has USB valid
#define BOARD_HAS_NBAT_V              2d // 2 Digital Voltage
#define BOARD_HAS_NBAT_I              2d // 2 Digital Current

/* PX4FMU GPIOs ***********************************************************************************/

/* Trace Clock and D0-D3 are available on the debug connector and adjacent pads
 *
 * TRACECLK PE2  - Dedicated  - Debug Connector Pin 7
 * TRACED0  PE3  - nLED_RED   - Debug Connector Pin 8
 * TRACED1  PE4  - nLED_GREEN - Test Point next to Debug Connector
 * TRACED2  PE5  - nLED_BLUE  - Test Point next to Debug Connector
 * TRACED3  PC12 - nARMED     - Test Point next to Debug Connector

 */
#undef TRACE_PINS

/* LEDs are driven with push open drain to support Anode to 5V or 3.3V or used as TRACE0-2 */

#if !defined(TRACE_PINS)
#  define GPIO_nLED_RED        /* PE3 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3)
#  define GPIO_nLED_GREEN      /* PE4 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN4)
#  define GPIO_nLED_BLUE       /* PE5 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN5)

#  define BOARD_HAS_CONTROL_STATUS_LEDS      1
#  define BOARD_OVERLOAD_LED     LED_RED
#  define BOARD_ARMED_STATE_LED  LED_BLUE
#endif

/* SPI
 *
 * SPI1 is sensors1
 *  ICM-20602
 *    CS        PI9
 *    DRDY      PF2
 *
 * SPI2 is sensors2
 *  ISM330
 *    CS        PH5
 *    DRDY      PH12
 *
 * SPI3 is sensors3
 *   BMI088
 *    CS ACCL   PI4
 *    CS GYRO   PI8
 *    DRDY ACCL PI6
 *    DRDY GYRO PI7
 *
 * SPI5 is FRAM
 *  FM25V02A
 *    CS        PG7
 *
 * SPI6 is EXTERNAL1
 *
 *    CS1       PI10
 *    CS2       PA15
 *    DRDY1     PD11
 *    DRDY2     PD12
 *
 */

#define PX4_SPI_BUS_SENSORS1   1
#define PX4_SPI_BUS_SENSORS2   2
#define PX4_SPI_BUS_SENSORS3   3
#define PX4_SPI_BUS_SENSORS4   4

#define PX4_SPI_BUS_MAG        PX4_SPI_BUS_SENSORS4
#define PX4_SPI_BUS_MEMORY     5
#define PX4_SPI_BUS_EXTERNAL1  6

/*  Define the Chip Selects, Data Ready and Control signals per SPI bus */

/* SPI 1 CS */

#define GPIO_SPI1_nCS1_ICM20602     /* PI9 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN9)

/*  Define the SPI1 Data Ready interrupts */

#define GPIO_SPI1_DRDY1_ICM20602    /* PF2  */  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTF|GPIO_PIN2)

/*  SPI1 off */

#define GPIO_SPI1_SCK_OFF   _PIN_OFF(GPIO_SPI1_SCK)
#define GPIO_SPI1_MISO_OFF  _PIN_OFF(GPIO_SPI1_MISO)
#define GPIO_SPI1_MOSI_OFF  _PIN_OFF(GPIO_SPI1_MOSI)

#define GPIO_DRDY_OFF_SPI1_DRDY1_ICM20602    _PIN_OFF(GPIO_SPI1_DRDY1_ICM20602)

/* SPI 2 CS */

#define GPIO_SPI2_nCS1_ISM330       /* PH5 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTH|GPIO_PIN5)

/*  Define the SPI2 Data Ready interrupts */

#define GPIO_SPI2_DRDY1_ISM330      /* PH12  */  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTH|GPIO_PIN12)

/*  SPI2 off */

#define GPIO_SPI2_SCK_OFF   _PIN_OFF(GPIO_SPI2_SCK)
#define GPIO_SPI2_MISO_OFF  _PIN_OFF(GPIO_SPI2_MISO)
#define GPIO_SPI2_MOSI_OFF  _PIN_OFF(GPIO_SPI2_MOSI)

#define GPIO_DRDY_OFF_SPI2_DRDY1_ISM330    _PIN_OFF(GPIO_SPI2_DRDY1_ISM330)

/* SPI 3 CS */

#define GPIO_SPI3_nCS1_BMI088_ACCEL /* PI4 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN4)
#define GPIO_SPI3_nCS2_BMI088_GYRO  /* PI8 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN8)

/*  Define the SPI3 Data Ready interrupts */
#define GPIO_SYNC                   /* PH10 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_100MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN10)

#define GPIO_SPI3_DRDY1_BMI088_INT1_ACCEL /* PI6  */  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTI|GPIO_PIN6)
#define GPIO_SPI3_DRDY2_BMI088_INT3_GYRO  /* PI7  */  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTI|GPIO_PIN7)

/*  SPI3 off */

#define GPIO_SPI3_SCK_OFF   _PIN_OFF(GPIO_SPI3_SCK)
#define GPIO_SPI3_MISO_OFF  _PIN_OFF(GPIO_SPI3_MISO)
#define GPIO_SPI3_MOSI_OFF  _PIN_OFF(GPIO_SPI3_MOSI)

#define GPIO_DRDY_OFF_SPI3_DRDY1_BMI088    _PIN_OFF(GPIO_SPI3_DRDY1_BMI088_INT1_ACCEL)
#define GPIO_DRDY_OFF_SPI3_DRDY2_BMI088    _PIN_OFF(GPIO_SPI3_DRDY2_BMI088_INT3_GYRO)

/* SPI 5 CS */

#define GPIO_SPI5_nCS1_FRAM          /* PG7  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN7)

/* SPI 6 */

#define GPIO_SPI6_nCS1_EXTERNAL1    /* PI10  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN10)
#define GPIO_SPI6_nCS2_EXTERNAL1    /* PA15  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN15)

/*  Define the SPI6 Data Ready interrupts */

#define GPIO_SPI6_DRDY1_EXTERNAL1   /* PD11  */  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN11)
#define GPIO_SPI6_DRDY2_EXTERNAL1   /* PD12  */  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN12)
#define SPI6_nRESET_EXTERNAL1       /* PF10  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTF|GPIO_PIN10)

/*  SPI6 off */

#define GPIO_SPI6_SCK_OFF   _PIN_OFF(GPIO_SPI6_SCK)
#define GPIO_SPI6_MISO_OFF  _PIN_OFF(GPIO_SPI6_MISO)
#define GPIO_SPI6_MOSI_OFF  _PIN_OFF(GPIO_SPI6_MOSI)

#define GPIO_DRDY_OFF_SPI6_DRDY1    _PIN_OFF(GPIO_SPI6_DRDY1_EXTERNAL1)
#define GPIO_DRDY_OFF_SPI6_DRDY2    _PIN_OFF(GPIO_SPI6_DRDY2_EXTERNAL1)

/*
 *  Define the ability to shut off off the sensor signals
 *  by changing the signals to inputs
 */

#define _PIN_OFF(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz))
#define PX4_SPI_BUS_RAMTRON  PX4_SPI_BUS_MEMORY

#include <drivers/drv_sensor.h>
#define PX4_SPIDEV_ICM_20602        PX4_MK_SPI_SEL(0,DRV_IMU_DEVTYPE_ICM20602)
#define PX4_SENSORS1_BUS_CS_GPIO    {GPIO_SPI1_nCS1_ICM20602}

#define PX4_SPIDEV_ISM330           PX4_MK_SPI_SEL(0,DRV_IMU_DEVTYPE_ST_ISM330DLC)
#define PX4_SENSORS2_BUS_CS_GPIO    {GPIO_SPI2_nCS1_ISM330}

#define PX4_SPIDEV_BMI088_GYR       PX4_MK_SPI_SEL(0,DRV_GYR_DEVTYPE_BMI088)
#define PX4_SPIDEV_BMI088_ACC       PX4_MK_SPI_SEL(0,DRV_ACC_DEVTYPE_BMI088)
#define PX4_SENSORS3_BUS_CS_GPIO    {GPIO_SPI3_nCS2_BMI088_GYRO, GPIO_SPI3_nCS1_BMI088_ACCEL}

#define PX4_SPIDEV_MEMORY           SPIDEV_FLASH(0)
#define PX4_MEMORY_BUS_CS_GPIO      {GPIO_SPI5_nCS1_FRAM}

#define PX4_SPIDEV_EXTERNAL1_1      PX4_MK_SPI_SEL(0,0)
#define PX4_SPIDEV_EXTERNAL1_2      PX4_MK_SPI_SEL(0,1)
#define PX4_EXTERNAL1_BUS_CS_GPIO   {GPIO_SPI6_nCS1_EXTERNAL1, GPIO_SPI6_nCS2_EXTERNAL1}


/* I2C busses */

#define PX4_I2C_BUS_EXPANSION       1
#define PX4_I2C_BUS_EXPANSION1      2
#define PX4_I2C_BUS_EXPANSION2      3
#define PX4_I2C_BUS_ONBOARD         4
#define PX4_I2C_BUS_LED             PX4_I2C_BUS_EXPANSION

/* Devices on the onboard buses.
 *
 * Note that these are unshifted addresses.
 */
#define PX4_I2C_OBDEV_A71CH         0x49

#define BOARD_NUMBER_I2C_BUSES      4
#define BOARD_I2C_BUS_CLOCK_INIT    {100000, 100000, 100000, 100000}

#define GPIO_I2C4_DRDY1_BMP388      /* PG5  */  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTG|GPIO_PIN5)
#define GPIO_SE050_nRST             /* PG6  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN6)

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that
 * can be used by the Px4 Firmware in the adc driver
 */

/* ADC defines to be used in sensors.cpp to read from a particular channel */

#define ADC1_CH(n)                  (n)
#define ADC1_GPIO(n)                GPIO_ADC1_IN##n
#define ADC3_CH(n)                  (n)
#define ADC3_GPIO(n)                GPIO_ADC3_IN##n

/* Define GPIO pins used as ADC N.B. Channel numbers must match below */

#define PX4_ADC_GPIO  \
	/* PA0 */  ADC1_GPIO(0),  \
	/* PA4 */  ADC1_GPIO(4),  \
	/* PB0 */  ADC1_GPIO(8),  \
	/* PB1 */  ADC1_GPIO(9),  \
	/* PC0 */  ADC1_GPIO(10), \
	/* PC2 */  ADC1_GPIO(12), \
	/* PC3 */  ADC1_GPIO(13), \
	/* PF4 */  ADC3_GPIO(14), \
	/* PF5 */  ADC3_GPIO(15)

/* Define Channel numbers must match above GPIO pin IN(n)*/

#define ADC_SCALED_VDD_3V3_SENSORS1_CHANNEL     /* PA0 */  ADC1_CH(0)
#define ADC_SCALED_VDD_3V3_SENSORS2_CHANNEL     /* PA4 */  ADC1_CH(4)
#define ADC_SCALED_VDD_3V3_SENSORS3_CHANNEL     /* PB0 */  ADC1_CH(8)
#define ADC_SCALED_V5_CHANNEL                   /* PB1 */  ADC1_CH(9)
#define ADC_ADC1_6V6_CHANNEL                    /* PC0 */  ADC1_CH(10)
#define ADC_SCALED_VDD_3V3_SENSORS4_CHANNEL     /* PC2 */  ADC1_CH(12)
#define ADC_ADC1_3V3_CHANNEL                    /* PC3 */  ADC1_CH(13)

#define ADC_HW_VER_SENSE_CHANNEL                /* PF4 */  ADC3_CH(14)
#define ADC_HW_REV_SENSE_CHANNEL                /* PF5 */  ADC3_CH(15)

#define ADC_CHANNELS \
	((1 << ADC_SCALED_VDD_3V3_SENSORS1_CHANNEL) | \
	 (1 << ADC_SCALED_VDD_3V3_SENSORS2_CHANNEL) | \
	 (1 << ADC_SCALED_VDD_3V3_SENSORS3_CHANNEL) | \
	 (1 << ADC_SCALED_V5_CHANNEL)               | \
	 (1 << ADC_ADC1_6V6_CHANNEL)                | \
	 (1 << ADC_SCALED_VDD_3V3_SENSORS4_CHANNEL) | \
	 (1 << ADC_ADC1_3V3_CHANNEL))

/* HW has to large of R termination on ADC todo:change when HW value is chosen */

#define HW_REV_VER_ADC_BASE STM32_ADC3_BASE

#define SYSTEM_ADC_BASE STM32_ADC1_BASE



#define BOARD_ADC_OPEN_CIRCUIT_V     (5.6f)

/* HW Version and Revision drive signals Default to 1 to detect */

#define BOARD_HAS_HW_VERSIONING

#define GPIO_HW_VER_REV_DRIVE  /* PG0 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN0)
#define GPIO_HW_REV_SENSE      /* PF5 */  ADC3_GPIO(15)
#define GPIO_HW_VER_SENSE      /* PF4 */  ADC3_GPIO(14)
#define HW_INFO_INIT           {'V','5','X','x', 'x',0}
#define HW_INFO_INIT_VER       3 /* Offset in above string of the VER */
#define HW_INFO_INIT_REV       4 /* Offset in above string of the REV */

/* HEATER
 * PWM in future
 */
#define GPIO_HEATER_OUTPUT   /* PB10  T2CH3 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN10)

/* PWM Capture
 *
 * 1  PWM Capture inputs are configured.
 *
 * Pins:
 *
 * FMU_CAP1 : PI0  : TIM5_CH4
 */

#define GPIO_TIM5_CH4IN      /* PI0  T5C4   FMU_CAP1 */ GPIO_TIM5_CH4IN_2
#define GPIO_TIM5_CH4OUT     /* PI0  T5C4   FMU_CAP1 */ GPIO_TIM5_CH4OUT_2

#define DIRECT_PWM_CAPTURE_CHANNELS  1

/* PC12 is nARMED
 *  The GPIO will be set as input while not armed HW will have external HW Pull UP.
 *  While armed it shall be configured at a GPIO OUT set LOW
 */
#define GPIO_nARMED_INIT     /* PC12 */  (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN12)
#define GPIO_nARMED          /* PC12 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN12)

#if !defined(TRACE_PINS)
#  define BOARD_INDICATE_ARMED_STATE(on_armed)  px4_arch_configgpio((on_armed) ? GPIO_nARMED : GPIO_nARMED_INIT)
#endif
/* PWM
 */
#define DIRECT_PWM_OUTPUT_CHANNELS  8
#define DIRECT_INPUT_TIMER_CHANNELS  8

#define BOARD_DSHOT_MOTOR_ASSIGNMENT {3, 2, 1, 0, 4, 5, 6, 7};

/* Power supply control and monitoring GPIOs */

#define GPIO_nPOWER_IN_A                /* PG1  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN1)
#define GPIO_nPOWER_IN_B                /* PG2  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN2)
#define GPIO_nPOWER_IN_C                /* PG3  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN3)

#define GPIO_nVDD_BRICK1_VALID          GPIO_nPOWER_IN_A /* Brick 1 Is Chosen */
#define GPIO_nVDD_BRICK2_VALID          GPIO_nPOWER_IN_B /* Brick 2 Is Chosen  */
#define BOARD_NUMBER_BRICKS             2
#define BOARD_NUMBER_DIGITAL_BRICKS     2
#define GPIO_nVDD_USB_VALID             GPIO_nPOWER_IN_C /* USB     Is Chosen */

#define GPIO_VDD_5V_PERIPH_nEN          /* PG4  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN4)
#define GPIO_VDD_5V_PERIPH_nOC          /* PE15 */ (GPIO_INPUT |GPIO_FLOAT|GPIO_PORTE|GPIO_PIN15)
#define GPIO_VDD_5V_HIPOWER_nEN         /* PF12 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN12)
#define GPIO_VDD_5V_HIPOWER_nOC         /* PF13 */ (GPIO_INPUT |GPIO_FLOAT|GPIO_PORTF|GPIO_PIN13)
#define GPIO_VDD_3V3_SENSORS4_EN        /* PG8  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN8)
#define GPIO_VDD_3V3_SPEKTRUM_POWER_EN  /* PH2  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN2)
#define GPIO_VDD_3V3_SD_CARD_EN         /* PC13 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN13)

/* Spare GPIO */

#define GPIO_PH11                       /* PH11  */  (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTH|GPIO_PIN1)

/* ETHERNET GPIO */

#define GPIO_ETH_POWER_EN              /* PG15 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN15)

/* NFC GPIO */

#define GPIO_NFC_GPIO                   /* PH3 */ (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTH|GPIO_PIN3)


/* Define True logic Power Control in arch agnostic form */

#define VDD_5V_PERIPH_EN(on_true)          px4_arch_gpiowrite(GPIO_VDD_5V_PERIPH_nEN, !(on_true))
#define VDD_5V_HIPOWER_EN(on_true)         px4_arch_gpiowrite(GPIO_VDD_5V_HIPOWER_nEN, !(on_true))
#define VDD_3V3_SENSORS4_EN(on_true)       px4_arch_gpiowrite(GPIO_VDD_3V3_SENSORS4_EN, (on_true))
#define VDD_3V3_SPEKTRUM_POWER_EN(on_true) px4_arch_gpiowrite(GPIO_VDD_3V3_SPEKTRUM_POWER_EN, (on_true))
#define READ_VDD_3V3_SPEKTRUM_POWER_EN()   px4_arch_gpioread(GPIO_VDD_3V3_SPEKTRUM_POWER_EN)
#define VDD_3V3_SD_CARD_EN(on_true)        px4_arch_gpiowrite(GPIO_VDD_3V3_SD_CARD_EN, (on_true))
#define VDD_3V3_ETH_POWER_EN(on_true)      px4_arch_gpiowrite(GPIO_ETH_POWER_EN, (on_true))
#define SE050_RESET(reset_true)            px4_arch_gpiowrite(GPIO_SE050_nRST, !(reset_true))


/* Tone alarm output */

#define TONE_ALARM_TIMER        14  /* Timer 14 */
#define TONE_ALARM_CHANNEL      1  /* PF9 GPIO_TIM14_CH1OUT_2 */

#define GPIO_BUZZER_1           /* PF9 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTF|GPIO_PIN9)

#define GPIO_TONE_ALARM_IDLE    GPIO_BUZZER_1
#define GPIO_TONE_ALARM         GPIO_TIM14_CH1OUT_2

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

#define RC_SERIAL_PORT                     "/dev/ttyS5"
#define RC_SERIAL_SINGLEWIRE

/* Input Capture Channels. */
#define INPUT_CAP1_TIMER                  5
#define INPUT_CAP1_CHANNEL     /* T5C4 */ 4
#define GPIO_INPUT_CAP1        /*  PI0 */ GPIO_TIM5_CH4_IN

/* PWM input driver. Use FMU AUX5 pins attached to timer4 channel 2 */
#define PWMIN_TIMER                       4
#define PWMIN_TIMER_CHANNEL    /* T4C2 */ 2
#define GPIO_PWM_IN            /* PD13 */ GPIO_TIM4_CH2IN_2

/* Safety Switch is HW version dependent on having an PX4IO
 * So we init to a benign state with the _INIT definition
 * and provide the the non _INIT one for the driver to make a run time
 * decision to use it.
 */
#define GPIO_nSAFETY_SWITCH_LED_OUT_INIT   /* PD10 */ (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTD|GPIO_PIN10)
#define GPIO_nSAFETY_SWITCH_LED_OUT        /* PD10 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN10)

/* Enable the FMU to control it if there is no px4io fixme:This should be BOARD_SAFETY_LED(__ontrue) */
#define GPIO_LED_SAFETY GPIO_nSAFETY_SWITCH_LED_OUT
#define GPIO_SAFETY_SWITCH_IN              /* PH4 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTH|GPIO_PIN4)
/* Enable the FMU to use the switch it if there is no px4io fixme:This should be BOARD_SAFTY_BUTTON() */
#define GPIO_BTN_SAFETY GPIO_SAFETY_SWITCH_IN /* Enable the FMU to control it if there is no px4io */

/* Power switch controls ******************************************************/

#define SPEKTRUM_POWER(_on_true)           VDD_3V3_SPEKTRUM_POWER_EN(_on_true)

/*
 * FMUv5X has a separate RC_IN
 *
 * GPIO PPM_IN on PI5 T8CH1
 * SPEKTRUM_RX (it's TX or RX in Bind) on UART6 PC7
 *   Inversion is possible in the UART and can drive  GPIO PPM_IN as an output
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
#define BOARD_ADC_USB_VALID     (!px4_arch_gpioread(GPIO_nVDD_USB_VALID))

/* FMUv5X never powers off the Servo rail */

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

#define BOARD_ADC_PERIPH_5V_OC  (!px4_arch_gpioread(GPIO_VDD_5V_PERIPH_nOC))
#define BOARD_ADC_HIPOWER_5V_OC (!px4_arch_gpioread(GPIO_VDD_5V_HIPOWER_nOC))

#define BOARD_HAS_PWM  DIRECT_PWM_OUTPUT_CHANNELS

/* This board provides a DMA pool and APIs */

#define BOARD_DMA_ALLOC_POOL_SIZE (5120 + 4096 + 1024 + 1024) 	// 5120 fat + 4096 + 1024 + 1024 spi

/* This board provides the board_on_reset interface */

#define BOARD_HAS_ON_RESET 1

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,                     \
		GPIO_HW_VER_REV_DRIVE,            \
		GPIO_CAN1_TX,                     \
		GPIO_CAN1_RX,                     \
		GPIO_CAN2_TX,                     \
		GPIO_CAN2_RX,                     \
		GPIO_HEATER_OUTPUT,               \
		GPIO_nPOWER_IN_A,                 \
		GPIO_nPOWER_IN_B,                 \
		GPIO_nPOWER_IN_C,                 \
		GPIO_VDD_5V_PERIPH_nEN,           \
		GPIO_VDD_5V_PERIPH_nOC,           \
		GPIO_VDD_5V_HIPOWER_nEN,          \
		GPIO_VDD_5V_HIPOWER_nOC,          \
		GPIO_VDD_3V3_SENSORS4_EN,         \
		GPIO_VDD_3V3_SPEKTRUM_POWER_EN,   \
		GPIO_VDD_3V3_SD_CARD_EN,          \
		GPIO_PH11,                        \
		GPIO_SYNC,                        \
		SPI6_nRESET_EXTERNAL1,            \
		GPIO_ETH_POWER_EN,                \
		GPIO_NFC_GPIO,                    \
		GPIO_TONE_ALARM_IDLE,             \
		GPIO_nSAFETY_SWITCH_LED_OUT_INIT, \
		GPIO_SAFETY_SWITCH_IN,            \
		GPIO_SE050_nRST,                  \
		GPIO_nARMED_INIT                  \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER

#define BOARD_NUM_IO_TIMERS 5

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
