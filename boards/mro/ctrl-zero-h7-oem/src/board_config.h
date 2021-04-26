/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

/* LEDs are driven with push open drain to support Anode to 5V or 3.3V */
#define GPIO_nLED_RED   /* PB11 */ (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN11)
#define GPIO_nLED_GREEN /* PB1  */ (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN1)
#define GPIO_nLED_BLUE  /* PB3  */ (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN3)

#define BOARD_HAS_CONTROL_STATUS_LEDS      1
#define BOARD_OVERLOAD_LED     LED_RED
#define BOARD_ARMED_STATE_LED  LED_BLUE

/* ADC channels */
#define PX4_ADC_GPIO  \
	/* PA2 */  GPIO_ADC12_INP14,  \
	/* PA3 */  GPIO_ADC12_INP15,  \
	/* PA4 */  GPIO_ADC12_INP18,  \
	/* PC1 */  GPIO_ADC123_INP11

/* Define Channel numbers must match above GPIO pins */
#define ADC_BATTERY_VOLTAGE_CHANNEL       14 /* PA2 BATT_VOLT_SENS */
#define ADC_BATTERY_CURRENT_CHANNEL       15 /* PA3 BATT_CURRENT_SENS */
#define ADC_SCALED_V5_CHANNEL             18 /* PA4 VDD_5V_SENS */
#define ADC_RC_RSSI_CHANNEL               11 /* PC1 */

#define ADC_CHANNELS \
	((1 << ADC_BATTERY_VOLTAGE_CHANNEL)       | \
	 (1 << ADC_BATTERY_CURRENT_CHANNEL)       | \
	 (1 << ADC_SCALED_V5_CHANNEL)             | \
	 (1 << ADC_RC_RSSI_CHANNEL))

/* HW has to large of R termination on ADC todo:change when HW value is chosen */
#define BOARD_ADC_OPEN_CIRCUIT_V     (5.6f)

/* CAN Silence: Silent mode control */
#define GPIO_CAN1_SILENT_S0  /* PF5  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTF|GPIO_PIN5)
#define GPIO_CAN2_SILENT_S0  /* PF5  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTF|GPIO_PIN5)

/* PWM */
#define DIRECT_PWM_OUTPUT_CHANNELS  8
#define DIRECT_INPUT_TIMER_CHANNELS  8

/* Power supply control and monitoring GPIOs */
#define GPIO_nPOWER_IN_A                /* PB5 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN5)

#define GPIO_VDD_BRICK1_VALID          GPIO_nPOWER_IN_A /* Brick 1 Is Chosen */
#define BOARD_NUMBER_BRICKS             1

#define GPIO_VDD_3V3_SPEKTRUM_POWER_EN  /* PE4  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN4)

/* Define True logic Power Control in arch agnostic form */
#define VDD_3V3_SPEKTRUM_POWER_EN(on_true) px4_arch_gpiowrite(GPIO_VDD_3V3_SPEKTRUM_POWER_EN, (!on_true))
#define READ_VDD_3V3_SPEKTRUM_POWER_EN()   (px4_arch_gpioread(GPIO_VDD_3V3_SPEKTRUM_POWER_EN) == 0)

/* Tone alarm output */
#define TONE_ALARM_TIMER        2  /* timer 2 */
#define TONE_ALARM_CHANNEL      1  /* PA15 TIM2_CH1 */

#define GPIO_BUZZER_1           /* PA15 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN15)

#define GPIO_TONE_ALARM_IDLE    GPIO_BUZZER_1
#define GPIO_TONE_ALARM         GPIO_TIM2_CH1OUT_2

/* USB OTG FS */
#define GPIO_OTGFS_VBUS         /* PA9 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_100MHz|GPIO_PORTA|GPIO_PIN9)

/* High-resolution timer */
#define HRT_TIMER               3  /* use timer3 for the HRT */
#define HRT_TIMER_CHANNEL       1 /* use capture/compare channel 1 */

#define HRT_PPM_CHANNEL         /* T3C3 */  3  /* use capture/compare channel 3 */
#define GPIO_PPM_IN             /* PB0 T3C3 */ GPIO_TIM3_CH3IN_1

/* RC Serial port */
#define RC_SERIAL_PORT          "/dev/ttyS3"

#define GPIO_RSSI_IN            /* PC1  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN1)

/* Safety Switch: Enable the FMU to control it if there is no px4io fixme:This should be BOARD_SAFETY_LED(__ontrue) */
#define GPIO_SAFETY_SWITCH_IN              /* PC4 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN4)
/* Enable the FMU to use the switch it if there is no px4io fixme:This should be BOARD_SAFTY_BUTTON() */
#define GPIO_BTN_SAFETY GPIO_SAFETY_SWITCH_IN /* Enable the FMU to control it if there is no px4io */

/* Power switch controls ******************************************************/
#define SPEKTRUM_POWER(_on_true)           VDD_3V3_SPEKTRUM_POWER_EN(_on_true)

/*
 * Board has a separate RC_IN
 *
 * GPIO PPM_IN on PB0 T3CH3
 * SPEKTRUM_RX (it's TX or RX in Bind) on UART6 PC7
 *   Inversion is possible in the UART and can drive GPIO_PPM_IN as an output
 */
#define GPIO_PPM_IN_AS_OUT             (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN0)
#define SPEKTRUM_RX_AS_GPIO_OUTPUT()   px4_arch_configgpio(GPIO_PPM_IN_AS_OUT)
#define SPEKTRUM_RX_AS_UART()          /* Can be left as uart */
#define SPEKTRUM_OUT(_one_true)        px4_arch_gpiowrite(GPIO_PPM_IN_AS_OUT, (_one_true))

/* By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */
#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_OTGFS_VBUS))
#define BOARD_ADC_USB_VALID     BOARD_ADC_USB_CONNECTED
#define BOARD_ADC_SERVO_VALID   (1) /* never powers off the Servo rail */
#define BOARD_ADC_BRICK_VALID   (px4_arch_gpioread(GPIO_VDD_BRICK1_VALID))

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */
#define BOARD_HAS_ON_RESET 1

#define BOARD_HAS_STATIC_MANIFEST 1

#define BOARD_HAS_PWM  DIRECT_PWM_OUTPUT_CHANNELS

#define BOARD_NUM_IO_TIMERS 3

#define BOARD_DSHOT_MOTOR_ASSIGNMENT {3, 2, 1, 0, 4, 5, 6, 7};

#define BOARD_ENABLE_CONSOLE_BUFFER

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,                     \
		GPIO_CAN1_TX,                     \
		GPIO_CAN1_RX,                     \
		GPIO_CAN2_TX,                     \
		GPIO_CAN2_RX,                     \
		GPIO_CAN1_SILENT_S0,              \
		GPIO_CAN2_SILENT_S0,              \
		GPIO_nPOWER_IN_A,                 \
		GPIO_VDD_3V3_SPEKTRUM_POWER_EN,   \
		GPIO_TONE_ALARM_IDLE,             \
		GPIO_SAFETY_SWITCH_IN,            \
		GPIO_OTGFS_VBUS,                  \
	}

__BEGIN_DECLS
#ifndef __ASSEMBLY__

extern void stm32_spiinitialize(void);
extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>
#endif /* __ASSEMBLY__ */
__END_DECLS
