/************************************************************************************
 * configs/aerofc-v1/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2012-2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           David Sidrane <david_s5@nscdg.com>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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
 ************************************************************************************/

#pragma once

/************************************************************************************
 * Included Files
 ************************************************************************************/
#include "board_dma_map.h"

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

#include "stm32_rcc.h"
#include "stm32.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* The TAP V1 uses a 16MHz crystal connected to the HSE.
 *
 * This is the canonical configuration:
 *   System Clock source           : PLL (HSE)
 *   SYSCLK(Hz)                    : 168000000    Determined by PLL configuration
 *   HCLK(Hz)                      : 168000000    (STM32_RCC_CFGR_HPRE)
 *   AHB Prescaler                 : 1            (STM32_RCC_CFGR_HPRE)
 *   APB1 Prescaler                : 4            (STM32_RCC_CFGR_PPRE1)
 *   APB2 Prescaler                : 2            (STM32_RCC_CFGR_PPRE2)
 *   HSE Frequency(Hz)             : 16000000     (STM32_BOARD_XTAL)
 *   PLLM                          : 8            (STM32_PLLCFG_PLLM)
 *   PLLN                          : 168          (STM32_PLLCFG_PLLN)
 *   PLLP                          : 2            (STM32_PLLCFG_PLLP)
 *   PLLQ                          : 7            (STM32_PLLCFG_PLLQ)
 *   Main regulator output voltage : Scale1 mode  Needed for high speed SYSCLK
 *   Flash Latency(WS)             : 5
 *   Prefetch Buffer               : OFF
 *   Instruction cache             : ON
 *   Data cache                    : ON
 *   Require 48MHz for USB OTG FS, : Enabled
 *   SDIO and RNG clock
 */

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * HSE - On-board crystal frequency is 16MHz
 * LSE - not installed
 */

#define STM32_BOARD_XTAL        16000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL

/* Main PLL Configuration.
 *
 * PLL source is HSE
 * PLL_VCO = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 *         = (16,000,000 / 8) * 168
 *         = 336,000,000
 * SYSCLK  = PLL_VCO / PLLP
 *         = 336,000,000 / 2 = 168,000,000
 * USB OTG FS, SDIO and RNG Clock
 *         =  PLL_VCO / PLLQ
 *         =  336,000,000 / 7
 *         = 48,000,000
 */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(8)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(168)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(7)

#define STM32_SYSCLK_FREQUENCY  168000000ul

/* AHB clock (HCLK) is SYSCLK (168MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK  /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY  /* same as above, to satisfy compiler */

/* APB1 clock (PCLK1) is HCLK/4 (42MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd4     /* PCLK1 = HCLK / 4 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/4)

/* Timers driven from APB1 will be twice PCLK1 */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN  (2*STM32_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK/2 (84MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLKd2     /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers driven from APB2 will be twice PCLK2 */

#define STM32_APB2_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM9_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM10_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM11_CLKIN  (2*STM32_PCLK2_FREQUENCY)

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,8-11 are on APB2, others on APB1
 */

#define BOARD_TIM1_FREQUENCY    STM32_APB2_TIM1_CLKIN
#define BOARD_TIM2_FREQUENCY    STM32_APB1_TIM2_CLKIN
#define BOARD_TIM3_FREQUENCY    STM32_APB1_TIM3_CLKIN
#define BOARD_TIM4_FREQUENCY    STM32_APB1_TIM4_CLKIN
#define BOARD_TIM5_FREQUENCY    STM32_APB1_TIM5_CLKIN
#define BOARD_TIM6_FREQUENCY    STM32_APB1_TIM6_CLKIN
#define BOARD_TIM7_FREQUENCY    STM32_APB1_TIM7_CLKIN
#define BOARD_TIM8_FREQUENCY    STM32_APB2_TIM8_CLKIN
#define BOARD_TIM9_FREQUENCY    STM32_APB2_TIM9_CLKIN
#define BOARD_TIM10_FREQUENCY   STM32_APB2_TIM10_CLKIN
#define BOARD_TIM11_FREQUENCY   STM32_APB2_TIM11_CLKIN
#define BOARD_TIM12_FREQUENCY   STM32_APB1_TIM12_CLKIN
#define BOARD_TIM13_FREQUENCY   STM32_APB1_TIM13_CLKIN
#define BOARD_TIM14_FREQUENCY   STM32_APB1_TIM14_CLKIN

/* LED definitions ******************************************************************/
/* If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in any
 * way.  The following definitions are used to access individual LEDs.
 */

/* LED index values for use with stm32_setled() */
#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_NLEDS       2

#define BOARD_LED_BLUE    BOARD_LED1
#define BOARD_LED_RED     BOARD_LED2

/* LED bits for use with stm32_setleds() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the 2 LEDs on board
 * the aerofc-v1.  The following definitions describe how NuttX controls
 * the LEDs:
 */

#define LED_STARTED       0  /* BLUE */
#define LED_HEAPALLOCATE  1  /* LED2 */
#define LED_IRQSENABLED   2  /* BLUE */
#define LED_STACKCREATED  3  /* BLUE + RED  */
#define LED_INIRQ         4  /* BLUE */
#define LED_SIGNAL        5  /* RED  */
#define LED_ASSERTION     6  /* BLUE + RED  */
#define LED_PANIC         7  /* BLUE + RED  */

/* Alternate function pin selections ************************************************/

/*
 * USARTs and UARTs
 *
 *
 * Peripheral   Port     Signal Name               CONN
 * USART1_TX    PB6     GPS_USART1_TX            JP1-15,16
 * USART1_RX    PB7     GPS_USART1_RX            JP1-13,14
 * USART2_TX    PA2     GB_USART2_TX             JP2-19,20
 * USART2_RX    PA3     GB_USART2_RX             JP2-21,22
 * USART3_TX    PC10    RF2_USART3_TX            J3-2
 * USART3_RX    PC11    RF2_USART3_RX            J3-1
 * USART6_TX    PC6     RF_USART6_TX             JP2-15,16
 * USART6_RX    PC7     RF_USART6_RX             JP2-17,18
*/

// ESC
#define GPIO_USART1_TX	GPIO_USART1_TX_2
#define GPIO_USART1_RX	GPIO_USART1_RX_2

// Companion
#define GPIO_USART2_TX	GPIO_USART2_TX_1
#define GPIO_USART2_RX	GPIO_USART2_RX_1
#define GPIO_USART2_CTS	0 // unused
#define GPIO_USART2_RTS	0 // unused

// GPS
#define GPIO_UART7_TX	GPIO_UART7_TX_1
#define GPIO_UART7_RX	GPIO_UART7_RX_1
#define GPIO_UART7_CTS	0 // unused
#define GPIO_UART7_RTS	0 // unused

// RC
#define GPIO_UART4_TX	GPIO_UART4_TX_1
#define GPIO_UART4_RX	GPIO_UART4_RX_1
#define GPIO_UART4_CTS	0 // unused
#define GPIO_UART4_RTS	0 // unused

// Mavlink
// UART5

// Serial console
#define GPIO_USART6_TX	GPIO_USART6_TX_1
#define GPIO_USART6_RX	GPIO_USART6_RX_1

/*
 * I2C
 *
 * Peripheral   Port     Signal Name               CONN
 * I2C1_SDA     PB9     I2C1_SDA                  J2-4,9,16,21 mpu6050, U4 MS6507
 * I2C1_SDL     PB8     I2C1_SCL                  J2-3,10,15,22 mpu6050, U4 MS6507
 * I2C2_SDA     PB11    Sonar Echo/I2C_SDA        JP2-31,32
 * I2C2_SDL     PB10    Sonar Trig/I2C_SCL        JP2-29,30
 * I2C3_SDA     PC9     COMPASS_I2C3_SDA          JP1-27,28
 * I2C3_SDL     PA8     COMPASS_I2C3_SCL          JP1-25,26
 *
 * The optional _GPIO configurations allow the I2C driver to manually
 * reset the bus to clear stuck slaves.  They match the pin configuration,
 * but are normally-high GPIOs.
 */
#define GPIO_I2C1_SDA GPIO_I2C1_SDA_2
#define GPIO_I2C1_SCL GPIO_I2C1_SCL_2

#define GPIO_I2C2_SDA GPIO_I2C2_SDA_1
#define GPIO_I2C2_SCL GPIO_I2C2_SCL_1

#define GPIO_I2C3_SDA		GPIO_I2C3_SDA_1
#define GPIO_I2C3_SCL		GPIO_I2C3_SCL_1

/*
 * SPI
 *
 * Peripheral   Port     Signal Name               CONN
 * SPI2_NSS       PB12    SD_SPI2_NSS               SD-2 CS
 * TBD
 *
 */

#define GPIO_SPI1_NSS   (GPIO_SPI1_NSS_2  | GPIO_SPEED_50MHz)
#define GPIO_SPI1_SCK   (GPIO_SPI1_SCK_1  | GPIO_SPEED_50MHz)
#define GPIO_SPI1_MISO  (GPIO_SPI1_MISO_1 | GPIO_SPEED_50MHz)
#define GPIO_SPI1_MOSI  (GPIO_SPI1_MOSI_1 | GPIO_SPEED_50MHz)

/* The following Pin Mapping is just for completeness */
/*
 * JTAG
 *
 * We will only enable SW-DP, JTAG-DP will be disabled
 *
 * Function     Port     Signal Name               CONN
 * SWDIO        PA13    DAT                       J10-3,J7
 * SWCLK        PA14    CLK                       J10-4,J8
 *
 */

/*
 *  BOOT
 *
 * Function     Port     Signal Name               CONN
 * BOOT0        NA       BOOT0              GND via 10 K
 * BOOT1        PB2      BOOT1              V3.3 - 10 K
 *
 *   As jumpered the device can only boot from FLASH.
 *
 *   It can be booted to:
 *
 * 	   SRAM if BOOT0 is pulled High with 1K.
 *     System memory if:
 *       BOOT0 is pulled High with 1K and
 *       BOOT1 is pulled Low with  1K
*/

/*
 * Timer PWM
 *
 * Peripheral   Port     Signal Name               CONN
 * TIM3_CH1     PA6     LED_R                     JP2-23,24
 * TIM3_CH2     PA7     LED_G                     JP2-25,26
 * TIM3_CH3     PB0     LED_B                     JP2-27,28
 * TIM3_CH4     PB1     nPWM_1 AUX1(Landing Gear) JP1-21,22
 */

/*
 * GPIO
 *
 * Port     Signal Name               CONN
 * PA4     POWER                     JP1-23,            - Must be held High to run w/o USB
 * PB4     TEMP_CONT                 J2-2,11,14,23      - Gyro Heater
 * PC0     VOLTAGE                   JP2-13,14          - 1.84 @16.66  1.67 @15.12 Scale 0.1105
 * PC1     KEY_AD                    JP1-31,32          - Low when Power button is depressed
 * PC2     SD_SW                     SD-9 SW            - Card Present
 * PC3     PCON_RADIO                JP1-29,30
 * PC13    S2                        U8-9 74HCT151
 * PC14    S1                        U8-10 74HCT151
 * PC15    S0                        U8-11 74HCT151
 */

/*
 * USB
 *
 * Port     Signal Name               CONN
 * PA9     OTG_FS_VBUS               J1-1
 * PA10    OTG_FS_ID                 J1-4
 * PA11    OTG_FS_DM                 J1-2
 * PA12    OTG_FS_DP                 J1-3
 */

/*
 * UNUSED PINS - In an idle world - these would have been tied to pads to
 * facilitate debugging probs.
 * Port
 * PA15
 * PB3
 * PB5
 * PC8
*/

/* Board provides GPIO or other Hardware for signaling to timing analyzer */

#if defined(CONFIG_BOARD_USE_PROBES)
# define PROBE_N(n) (1<<((n)-1))
# define PROBE_1	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN15)
# define PROBE_2	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN3)
# define PROBE_3	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN5)
# define PROBE_4	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN8)

# define PROBE_INIT(mask) \
	do { \
		if ((mask)& PROBE_N(1)) { stm32_configgpio(PROBE_1); } \
		if ((mask)& PROBE_N(2)) { stm32_configgpio(PROBE_2); } \
		if ((mask)& PROBE_N(3)) { stm32_configgpio(PROBE_3); } \
		if ((mask)& PROBE_N(4)) { stm32_configgpio(PROBE_4); } \
	} while(0)

# define PROBE(n,s)  do {stm32_gpiowrite(PROBE_##n,(s));}while(0)
# define PROBE_MARK(n) PROBE(n,false);PROBE(n,true)
#else
# define PROBE_INIT(mask)
# define PROBE(n,s)
# define PROBE_MARK(n)
#endif

