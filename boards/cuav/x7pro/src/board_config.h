/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * Board internal definitions
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <stm32_gpio.h>

/* LEDs */
#define GPIO_nLED_RED        /* PI5 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN5)
#define GPIO_nLED_GREEN      /* PI6 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN6)
#define GPIO_nLED_BLUE       /* PI7 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN7)

#define BOARD_HAS_LED_PWM              1

/* ADC channels */
#define PX4_ADC_GPIO  \
	/* PA0  */  GPIO_ADC1_INP16,   \
	/* PA1  */  GPIO_ADC1_INP17,   \
	/* PA2  */  GPIO_ADC12_INP14,  \
	/* PF11 */  GPIO_ADC1_INP2,    \
	/* PC4  */  GPIO_ADC12_INP4,   \
	/* PA4  */  GPIO_ADC12_INP18,  \
	/* PF12 */  GPIO_ADC1_INP6,    \
	/* PC5  */  GPIO_ADC12_INP8,   \
	/* PC1  */  GPIO_ADC123_INP11, \
	/* PC2  */  GPIO_ADC123_INP12, \
	/* PC3  */  GPIO_ADC12_INP13

/* Define Channel numbers must match above GPIO pins */
#define ADC_BATTERY1_VOLTAGE_CHANNEL        16 /* PA0  */
#define ADC_BATTERY1_CURRENT_CHANNEL        17 /* PA1  */
#define ADC_BATTERY2_VOLTAGE_CHANNEL        14 /* PA2  */
#define ADC_BATTERY2_CURRENT_CHANNEL         2 /* PF11 */
#define ADC1_6V6_IN_CHANNEL                  4 /* PC4  */   // SPARE1_ADC1: ADC6.6
#define ADC1_3V3_IN_CHANNEL                 18 /* PA4  */   // SPARE2_ADC1: ADC3.3
#define ADC_RSSI_IN_CHANNEL                  6 /* PF12 */
#define ADC_SCALED_V5_CHANNEL                8 /* PC5  */   // VDD_5V_SENS: Motherboard 5V voltage detection
#define ADC_SCALED_VDD_3V3_SENSORS_CHANNEL  11 /* PC1  */   // SCALED_V3V3: Sensor power detection
#define ADC_HW_VER_SENSE_CHANNEL            12 /* PC2  */
#define ADC_HW_REV_SENSE_CHANNEL            13 /* PC3  */

#define ADC_CHANNELS \
	((1 << ADC_BATTERY1_VOLTAGE_CHANNEL)       | \
	 (1 << ADC_BATTERY1_CURRENT_CHANNEL)       | \
	 (1 << ADC_BATTERY2_VOLTAGE_CHANNEL)       | \
	 (1 << ADC_BATTERY2_CURRENT_CHANNEL)       | \
	 (1 << ADC1_6V6_IN_CHANNEL)                | \
	 (1 << ADC1_3V3_IN_CHANNEL)                | \
	 (1 << ADC_RSSI_IN_CHANNEL)                | \
	 (1 << ADC_SCALED_V5_CHANNEL)              | \
	 (1 << ADC_SCALED_VDD_3V3_SENSORS_CHANNEL) | \
	 (1 << ADC_HW_VER_SENSE_CHANNEL)           | \
	 (1 << ADC_HW_REV_SENSE_CHANNEL))

/* HW has to large of R termination on ADC todo:change when HW value is chosen */
#define BOARD_ADC_OPEN_CIRCUIT_V     (5.6f)

#define GPIO_HW_REV_DRIVE    /* PH14  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTH|GPIO_PIN14)
#define GPIO_HW_REV_SENSE    /* PC3   */ GPIO_ADC12_INP13
#define GPIO_HW_VER_DRIVE    /* PH14  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN3)
#define GPIO_HW_VER_SENSE    /* PC2   */ GPIO_ADC123_INP12

/* CAN Silence Silent mode control */
#define GPIO_CAN1_SILENT_S0  /* PH2  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN2)
#define GPIO_CAN2_SILENT_S1  /* PH3  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN3)

/* HEATER */
#define GPIO_HEATER_OUTPUT   /* PA8 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN8)

/* PWM */
#define DIRECT_PWM_OUTPUT_CHANNELS   14
#define BOARD_NUM_IO_TIMERS           4

/* Power supply control and monitoring GPIOs */
#define BOARD_NUMBER_BRICKS             2

#define GPIO_nPOWER_IN_ADC              /* PG1  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN1)
#define GPIO_nPOWER_IN_CAN              /* PG2  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN2)
#define GPIO_nPOWER_IN_C                /* PG0  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN0)

#define GPIO_nVDD_BRICK1_VALID          GPIO_nPOWER_IN_CAN /* Brick 1 is Chosen */
#define GPIO_nVDD_BRICK2_VALID          GPIO_nPOWER_IN_ADC /* Brick 2 is Chosen  */
#define GPIO_nVDD_USB_VALID             GPIO_nPOWER_IN_C   /* USB     is Chosen */

#define GPIO_VDD_5V_HIPOWER_EN          /* PD11 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN11)
#define GPIO_nVDD_5V_PERIPH_EN          /* PG4  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN4)
#define GPIO_VDD_5V_RC_EN               /* PG5  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN5)
#define GPIO_VDD_3V3_SD_CARD_EN         /* PG7  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN7)

#define GPIO_VDD_5V_HIPOWER_OC          /* PJ3 */  (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTJ|GPIO_PIN3)
#define GPIO_nVDD_5V_PERIPH_OC          /* PJ4 */  (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTJ|GPIO_PIN4)

/* Power switch controls ******************************************************/
#define VDD_5V_PERIPH_EN(on_true)          px4_arch_gpiowrite(GPIO_nVDD_5V_PERIPH_EN, !(on_true))
#define VDD_5V_HIPOWER_EN(on_true)         px4_arch_gpiowrite(GPIO_VDD_5V_HIPOWER_EN, (on_true))
#define VDD_3V3_SD_CARD_EN(on_true)        px4_arch_gpiowrite(GPIO_VDD_3V3_SD_CARD_EN, (on_true))

#define SPEKTRUM_POWER(on_true)            px4_arch_gpiowrite(GPIO_VDD_5V_RC_EN, (on_true))
#define READ_SPEKTRUM_POWER()              px4_arch_gpioread(GPIO_VDD_5V_RC_EN)

/* Tone alarm output */
#define TONE_ALARM_TIMER        15  /* timer 15 */
#define TONE_ALARM_CHANNEL      1  /* PE5 TIM15_CH1 */

#define GPIO_TONE_ALARM_IDLE    /* PE5 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN5) // ALARM
#define GPIO_TONE_ALARM         GPIO_TIM15_CH1OUT_2

/* USB
 *  OTG FS: PA9  OTG_FS_VBUS VBUS sensing
 *  HS USB EN: PH15
 */
#define GPIO_OTGFS_VBUS         /* PA9 */  (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_100MHz|GPIO_PORTA|GPIO_PIN9)
#define GPIO_HS_USB_EN          /* PH15 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN15)

/* High-resolution timer */
#define HRT_TIMER               3  /* use timer3 for the HRT */
#define HRT_TIMER_CHANNEL       3  /* use capture/compare channel 3 */


#define HRT_PPM_CHANNEL         /* T3C1 */  1  /* use capture/compare channel 1 */
#define GPIO_PPM_IN             /* PB4 T3C1 */ GPIO_TIM3_CH1IN_2
#define RC_SERIAL_PORT          "/dev/ttyS5"
#define RC_SERIAL_SINGLEWIRE

/**
 * GPIO PPM_IN on PB4 T3C1
 * SPEKTRUM_RX (it's TX or RX in Bind) on UART8 PE0
 *   Inversion is possible in the UART and can drive GPIO_PPM_IN as an output
 */
#define GPIO_PPM_IN_AS_OUT           (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN4)
#define SPEKTRUM_RX_AS_GPIO_OUTPUT() px4_arch_configgpio(GPIO_PPM_IN_AS_OUT)
#define SPEKTRUM_RX_AS_UART()       /* Can be left as uart */
#define SPEKTRUM_OUT(_one_true)      px4_arch_gpiowrite(GPIO_PPM_IN_AS_OUT, (_one_true))

/* RSSI_IN */
#define GPIO_RSSI_IN                   /* PB0  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN0)

/* Safety Switch is only on FMU */
#define FMU_LED_AMBER                 /* PE12 */ (GPIO_OUTPUT|GPIO_PULLDOWN|GPIO_PORTE|GPIO_PIN12) // FMU_LED_AMBER
#define GPIO_BTN_SAFETY               /* PE10 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTE|GPIO_PIN10) // SAFETY_SW

#define GPIO_LED_SAFETY FMU_LED_AMBER

/* By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */
#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_OTGFS_VBUS))
#define BOARD_ADC_USB_VALID     (!px4_arch_gpioread(GPIO_nVDD_USB_VALID))
#define BOARD_ADC_BRICK1_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK1_VALID))
#define BOARD_ADC_BRICK2_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK2_VALID))
#define BOARD_ADC_PERIPH_5V_OC  (!px4_arch_gpioread(GPIO_nVDD_5V_PERIPH_OC))
#define BOARD_ADC_HIPOWER_5V_OC (!px4_arch_gpioread(GPIO_VDD_5V_HIPOWER_OC))


/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */
#define BOARD_HAS_ON_RESET 1

#define BOARD_HAS_PWM  DIRECT_PWM_OUTPUT_CHANNELS

#define BOARD_ENABLE_CONSOLE_BUFFER

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,                     \
		GPIO_HW_REV_DRIVE,                \
		GPIO_HW_VER_DRIVE,                \
		GPIO_CAN1_TX,                     \
		GPIO_CAN1_RX,                     \
		GPIO_CAN2_TX,                     \
		GPIO_CAN2_RX,                     \
		GPIO_CAN1_SILENT_S0,              \
		GPIO_CAN2_SILENT_S1,              \
		GPIO_HEATER_OUTPUT,               \
		GPIO_nPOWER_IN_CAN,               \
		GPIO_nPOWER_IN_ADC,               \
		GPIO_nPOWER_IN_C,                 \
		GPIO_nVDD_5V_PERIPH_EN,           \
		GPIO_nVDD_5V_PERIPH_OC,           \
		GPIO_VDD_5V_HIPOWER_EN,           \
		GPIO_VDD_5V_HIPOWER_OC,          \
		GPIO_VDD_5V_RC_EN,                \
		PX4_GPIO_PIN_OFF(GPIO_SDMMC1_D0), \
		PX4_GPIO_PIN_OFF(GPIO_SDMMC1_D1), \
		PX4_GPIO_PIN_OFF(GPIO_SDMMC1_D2), \
		PX4_GPIO_PIN_OFF(GPIO_SDMMC1_D3), \
		PX4_GPIO_PIN_OFF(GPIO_SDMMC1_CMD),\
		GPIO_VDD_3V3_SD_CARD_EN,          \
		GPIO_TONE_ALARM_IDLE,             \
		GPIO_OTGFS_VBUS,                  \
		PX4_GPIO_PIN_OFF(GPIO_HS_USB_EN), \
		GPIO_RSSI_IN,                     \
		FMU_LED_AMBER,                    \
		GPIO_BTN_SAFETY,                  \
	}

__BEGIN_DECLS
#ifndef __ASSEMBLY__

extern void stm32_spiinitialize(void);
extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>
#endif /* __ASSEMBLY__ */
__END_DECLS
