/****************************************************************************
 *
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************/

/**
 * @file board_config.h
 *
 * NXP ucans32k146 internal definitions
 */

#pragma once

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <px4_platform_common/px4_config.h>

#include <nuttx/compiler.h>

#include <stdint.h>

__BEGIN_DECLS

/* these headers are not C++ safe */
#include "hardware/s32k1xx_pinmux.h"
#include "s32k1xx_periphclocks.h"
#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* UCANS32K146 GPIOs **************************************************/

/* LEDs.  The UCANS32K146 has one RGB LED:
 *
 *   RedLED   PTD15 (FTM0 CH0)
 *   GreenLED PTD16 (FTM0 CH1)
 *   BlueLED  PTD0  (FTM0 CH2)
 *
 * An output of '1' illuminates the LED.
 */

#define GPIO_LED_R     (PIN_PTD15 | GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO)
#define GPIO_LED_G     (PIN_PTD16 | GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO)
#define GPIO_LED_B     (PIN_PTD0  | GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO)

/* Buttons.  The UCANS32K146 supports one button:
 *
 *   SW3  PTC14
 */

#define GPIO_SW3       (PIN_PTC14 | PIN_INT_BOTH)

/* SPI chip selects */

/* Count of peripheral clock user configurations */

#define NUM_OF_PERIPHERAL_CLOCKS_0 18


/* High-resolution timer */
#define HRT_TIMER              5  /* FTM timer for the HRT */
#define HRT_TIMER_CHANNEL      0  /* Use capture/compare channel 0 */

/* Timer I/O PWM and capture
 *
 * ?? PWM outputs are configured.
 * ?? Timer inputs are configured.
 *
 * Pins:
 *      Defined in board.h
 */

#define DIRECT_PWM_OUTPUT_CHANNELS  1

#define BOARD_HAS_PWM	DIRECT_PWM_OUTPUT_CHANNELS

#define BOARD_HAS_LED_PWM              1
#define BOARD_LED_PWM_DRIVE_ACTIVE_LOW 1
#define BOARD_HAS_SHARED_PWM_TIMERS    1

#define LED_TIM0_CH0OUT   /* PTD15  RGB_R */ PIN_FTM0_CH0_3
#define LED_TIM0_CH1OUT   /* PTD16  RGB_G */ PIN_FTM0_CH1_3
#define LED_TIM0_CH2OUT   /* PTD0   RGB_B */ PIN_FTM0_CH2_3

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* User peripheral configuration structure 0 */

extern const struct peripheral_clock_config_s g_peripheral_clockconfig0[];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int s32k1xx_bringup(void);

/****************************************************************************
 * Name: s32k1xx_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the UCANS32K146
 *   board.
 *
 ****************************************************************************/

#ifdef CONFIG_S32K1XX_LPSPI
void s32k1xx_spidev_initialize(void);
#endif

/****************************************************************************************************
 * Name: board_spi_reset board_peripheral_reset
 *
 * Description:
 *   Called to reset SPI and the perferal bus
 *
 ****************************************************************************************************/
void board_peripheral_reset(int ms);

/************************************************************************************
 * Name: ucans32k_timer_initialize
 *
 * Description:
 *   Called to configure the FTM to provide 1 Mhz
 *
 ************************************************************************************/

void ucans32k_timer_initialize(void);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
