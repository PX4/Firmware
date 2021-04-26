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
 * PX4FMUv4 internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/* BUTTON *************************************************************************** */
#define BUTTON_SAFETY         /* PB3  */  (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTB|GPIO_PIN3|GPIO_EXTI)

#define GPIO_RGB_S            /* PB0  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN0)
#define BOARD_SRGBLED_PORT    STM32_GPIOB_ODR
#define BOARD_SRGBLED_BIT     0

/* CAN Silence
 *
 * Silent mode control \ ESC Mux select
 */
#define GPIO_CAN1_SILENT_S0  /* PB5  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN5)

#define GPIO_LED_SAFETY              (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN1)
#define GPIO_BTN_SAFETY              (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTB|GPIO_PIN3)

/* Tone alarm output. */
#define TONE_ALARM_TIMER             2    /* timer 2 */
#define TONE_ALARM_CHANNEL           3    /* channel 1 */
#define GPIO_TONE_ALARM_IDLE         (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN10)
#define GPIO_TONE_ALARM              (GPIO_ALT|GPIO_AF1|GPIO_SPEED_2MHz|GPIO_PUSHPULL|GPIO_PORTB|GPIO_PIN10)

/* High-resolution timer */
#define HRT_TIMER                    3  /* use timer 3 for the HRT */
#define HRT_TIMER_CHANNEL            4  /* use capture/compare channel 4 */

#define FLASH_BASED_PARAMS

__BEGIN_DECLS
#define BOARD_HAS_N_S_RGB_LED       8  /* Uses 8 SK6812 digital led chip */
#define BOARD_MAX_LEDS              BOARD_HAS_N_S_RGB_LED

/* USE_S_RGB_LED_DMAis passed in from the *.cmake file
 * Bootloader is not timming sensitive and can use the SW version as a
 * size savings
 * The Application can not as it needs DMA to maintain reall time.
 */

#if defined(USE_S_RGB_LED_DMA)
#  define S_RGB_LED_DMA              DMAMAP_TIM1_CH2
#  define S_RGB_LED_TIMER                1   /* timer 1    */
#  define S_RGB_LED_CHANNEL              2   /* channel 2  */
#  define S_RGB_LED_CHANNELN             1   /* channel 2N */
#  define S_RGB_LED_TIM_GPIO             GPIO_TIM1_CH2N_1
#endif
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

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
