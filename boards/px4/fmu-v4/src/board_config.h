/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * PX4FMUv4 internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/
/* Configuration ************************************************************************************/

/* PX4FMU GPIOs ***********************************************************************************/
/* LEDs */
#define GPIO_LED1                    (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN11)
#define GPIO_LED2                    (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN1)
#define GPIO_LED3                    (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN3)

#define GPIO_LED_RED                 GPIO_LED1
#define GPIO_LED_GREEN               GPIO_LED2
#define GPIO_LED_BLUE                GPIO_LED3

#define BOARD_HAS_CONTROL_STATUS_LEDS 1
#define BOARD_OVERLOAD_LED     LED_RED
#define BOARD_ARMED_LED        LED_BLUE
#define BOARD_ARMED_STATE_LED  LED_GREEN

#ifdef CONFIG_STM32_SPI4
#  define BOARD_HAS_BUS_MANIFEST 1 // We support a bus manifest because spi 4 is optional
#endif /* CONFIG_STM32_SPI4 */

/**
 * ADC channels:
 * These are the channel numbers of the ADCs of the microcontroller that can be used by the Px4 Firmware in the adc driver.
 */
#define ADC_CHANNELS (1 << 2) | (1 << 3) | (1 << 4) | (1 << 10) | (1 << 11) | (1 << 12) | (1 << 13) | (1 << 14)

/* ADC defines to be used in sensors.cpp to read from a particular channel. */
#define ADC_BATTERY_VOLTAGE_CHANNEL  2
#define ADC_BATTERY_CURRENT_CHANNEL  3
#define ADC_5V_RAIL_SENSE            4
#define ADC_RC_RSSI_CHANNEL          11

/* Define Battery 1 Voltage Divider and A per V. */
#define BOARD_BATTERY1_V_DIV         (13.653333333f)
#define BOARD_BATTERY1_A_PER_V       (36.367515152f)


/* Power supply control and monitoring GPIOs. */
#define GPIO_VDD_BRICK_VALID         (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN5)
#define GPIO_VDD_USB_VALID           (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN0)

/* Tone alarm output. */
#define TONE_ALARM_TIMER             2    /* timer 2 */
#define TONE_ALARM_CHANNEL           1    /* channel 1 */
#define GPIO_TONE_ALARM_IDLE         (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN15)
#define GPIO_TONE_ALARM              (GPIO_ALT|GPIO_AF1|GPIO_SPEED_2MHz|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN15)

/**
 * PWM:
 *
 * Six PWM outputs are configured.
 */
#define DIRECT_PWM_OUTPUT_CHANNELS   6
#define DIRECT_INPUT_TIMER_CHANNELS  6

/**
 * USB OTG FS:
 * PA9  OTG_FS_VBUS VBUS sensing.
 */
#define GPIO_OTGFS_VBUS              (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9)

/* High-resolution timer */
#define HRT_TIMER                    3  /* use timer 3 for the HRT */
#define HRT_TIMER_CHANNEL            4  /* use capture/compare channel 4 */

#define HRT_PPM_CHANNEL              3  /* use capture/compare channel 3 */
#define GPIO_PPM_IN                  (GPIO_ALT|GPIO_AF2|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN0)

/* RC Serial port */

#define RC_SERIAL_PORT               "/dev/ttyS4"

/* PWM input driver. Use FMU AUX5 pins attached to timer4 channel 2. */
#define PWMIN_TIMER                  4
#define PWMIN_TIMER_CHANNEL          2
#define GPIO_PWM_IN                  GPIO_TIM4_CH2IN_2

#define GPIO_RSSI_IN                 (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN1)
#define GPIO_LED_SAFETY              (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN3)
#define GPIO_BTN_SAFETY              (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN4)
#define GPIO_PERIPH_3V3_EN           (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN5)

/* For R12, this signal is active high. */
#define GPIO_SBUS_INV                (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN13)
#define RC_INVERT_INPUT(_invert_true) px4_arch_gpiowrite(GPIO_SBUS_INV, _invert_true)

#define GPIO_SPEKTRUM_PWR_EN         (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN4)

#define GPIO_8266_GPIO0              (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN2)
#define GPIO_8266_GPIO2              (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN4)
#define GPIO_8266_PD                 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN5)
#define GPIO_8266_RST                (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN6)

/* Heater pins */
#define GPIO_HEATER_INPUT            (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTC|GPIO_PIN6)
#define GPIO_HEATER_OUTPUT           (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN6)

/* Power switch controls */

#define SPEKTRUM_POWER(_on_true)     px4_arch_gpiowrite(GPIO_SPEKTRUM_PWR_EN, (!_on_true))

/**
 * FMUv4 has separate RC_IN
 *
 * GPIO PPM_IN on PB0 T3C3
 * SPEKTRUM_RX (it's TX or RX in Bind) on UART6 PC7
 * Inversion is possible via the 74LVC2G86 controlled by the FMU
 * The FMU can drive  GPIO PPM_IN as an output
 */

#define GPIO_PPM_IN_AS_OUT           (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN0)
#define SPEKTRUM_RX_AS_GPIO_OUTPUT() px4_arch_configgpio(GPIO_PPM_IN_AS_OUT)
#define SPEKTRUM_RX_AS_UART()       /* Can be left as uart */
#define SPEKTRUM_OUT(_one_true)      px4_arch_gpiowrite(GPIO_PPM_IN_AS_OUT, (_one_true))

/**
 * By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */
#define BOARD_ADC_USB_CONNECTED      (px4_arch_gpioread(GPIO_OTGFS_VBUS))
#define BOARD_ADC_BRICK_VALID        (px4_arch_gpioread(GPIO_VDD_BRICK_VALID))
#define BOARD_ADC_USB_VALID          (px4_arch_gpioread(GPIO_VDD_USB_VALID))
#define BOARD_ADC_SERVO_VALID        (1)
#define BOARD_ADC_PERIPH_5V_OC       (0)
#define BOARD_ADC_HIPOWER_5V_OC      (0)

#define BOARD_HAS_PWM    DIRECT_PWM_OUTPUT_CHANNELS

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

#define BOARD_HAS_ON_RESET 1

#define BOARD_DSHOT_MOTOR_ASSIGNMENT {3, 2, 1, 0, 4, 5};

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
