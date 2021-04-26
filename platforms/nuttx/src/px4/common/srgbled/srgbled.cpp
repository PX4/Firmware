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
* @file srgbled.cpp
* Author: David.Sidrane@Nscdg.com*
*  This is a driver for the the neopixel class of serial RGB LEDs.
*
*  It is arch dependent on ARM for the DWT and the GPIO is done in arch
*  independent manner.
*
*  To use this driver the board must define:

*  BOARD_HAS_N_S_RGB_LED  - the number of LEDs
*  BOARD_SRGBLED_BIT      - the bit number it is connected to. 0-n (not a mask)
        and
*  BOARD_SRGBLED_PORT     - The address of the port's "data out" register
*                           the LED is connected to.
*       OR
*  BOARD_SRGBLED_SET_PORT - for an arch with set/clear GPIO. The address of the
*                           port's "SET Bit" register
*  BOARD_SRGBLED_CLEAR    - for an arch with set/clear GPIO.  The address of the
*                           port's "Clear Bit" register
*
*  N.B. This version is small but will disable interrupts for 30 us per LED!
*  With an 8 LED's this is 240uS and not tolerable for systems with sensors.
*  It is however useful in bootloaders.
*
*  The DMA version of this driver should be used for systems that can not have
*  interrupts for long times.
*
*/

#include <px4_platform_common/px4_config.h>
#include <drivers/drv_neopixel.h>
#include <board_config.h>
#include <dwt.h>
#include <nvic.h>

#define REG(_addr)       (*(volatile uint32_t *)(_addr))
#define rDEMCR           REG(NVIC_DEMCR)
#define rDWT_CTRL        REG(DWT_CTRL)
#define rDWT_CNT         REG(DWT_CYCCNT)

#if defined(BOARD_SRGBLED_PORT) && defined(BOARD_SRGBLED_BIT)
#  define PORT             REG(BOARD_SRGBLED_PORT)
#  define D0               ((PORT) &= ~(1 << BOARD_SRGBLED_BIT));
#  define D1               ((PORT) |= (1 << BOARD_SRGBLED_BIT));
#elif defined(BOARD_SRGBLED_SET_PORT) && defined(BOARD_SRGBLED_CLEAR_PORT)  && defined(BOARD_SRGBLED_BIT)
#  define PORT             REG(BOARD_SRGBLED_PORT)
#  define D0               ((BOARD_SRGBLED_CLEAR_PORT) |= (1 << BOARD_SRGBLED_BIT));
#  define D1               ((BOARD_SRGBLED_SET_PORT)   |= (1 << BOARD_SRGBLED_BIT));
#else
# error BOARD_SRGBLED_[]{SET|CLEAR}_]PORT and BOARD_SRGBLED_BIT needs to be defined.
#endif

#define DWT_DEADLINE(t)  rDWT_CNT + (t)
#define DWT_WAIT(v, D)   while((rDWT_CNT - (v)) < (D)){}

#define T0H              (STM32_SYSCLK_FREQUENCY/3333333)
#define T1H              (STM32_SYSCLK_FREQUENCY/1666666)
#define TW               (STM32_SYSCLK_FREQUENCY/850000)

#define COLOR_PER_LED   3  // There is a R G B in each package.
#define BITS_PER_COLOR  8  // Each LED has 8 bits of luminosity
#define BITS_PER_PACKAGE (BITS_PER_COLOR * COLOR_PER_LED)

#if defined(BOARD_HAS_N_S_RGB_LED) && !defined(S_RGB_LED_DMA)


int neopixel_write_no_dma(uint8_t r, uint8_t g, uint8_t b, uint8_t led_count)
{
	neopixel::NeoLEDData::led_data_t data;
	data.grb[2] = g;
	data.grb[1] = r;
	data.grb[0] = b;
	rDEMCR    |= NVIC_DEMCR_TRCENA;
	rDWT_CTRL |= DWT_CTRL_CYCCNTENA_MASK;
	irqstate_t state = px4_enter_critical_section();

	while (led_count--) {
		uint32_t  deadline = DWT_DEADLINE(TW);

		for (uint32_t mask = 1 << (BITS_PER_PACKAGE - 1);  mask != 0;  mask >>= 1) {
			DWT_WAIT(deadline, TW);
			deadline = rDWT_CNT;
			D1;
			DWT_WAIT(deadline, data.l & mask ? T1H : T0H);
			D0;
		}

		DWT_WAIT(deadline, TW);
	}

	px4_leave_critical_section(state);
	return 0;
}
#endif // BOARD_HAS_SRGBLED
