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
* @file sled_dma.cpp
* Author: David.Sidrane@Nscdg.com
*
*  This is a DMA based driver for the the neopixel class of serial RGB LEDs.
*
*  It is arch dependent on timers ans DMA
*  To use this driver a board must define:
*
*  To use this driver the board must define:

*  BOARD_HAS_N_S_RGB_LED      - the number of LEDs
*  S_RGB_LED_DMA              - the DMAMAP Channel
*  S_RGB_LED_TIMER            - The Timer the LED(s) are connected to
*  S_RGB_LED_CHANNEL          - The Timer' channel the LED(s) are connected to
*  S_RGB_LED_CHANNELN         - If the Channel is the complamentry
*  S_RGB_LED_TIM_GPIO         - The GPIO pinmap for the connection
*
*/

#include <px4_platform_common/px4_config.h>
#include <board_config.h>

#include <px4_platform_common/micro_hal.h>
#include <systemlib/px4_macros.h>
#include <drivers/drv_neopixel.h>

#include <stm32_dma.h>
#include <stm32_tim.h>
#include <dwt.h>
#include <nvic.h>


#if defined(BOARD_HAS_N_S_RGB_LED) && defined(S_RGB_LED_DMA)
#if !defined(S_RGB_LED_TIMER)
#  error  S_RGB_LED_TIMER must be defined with S_RGB_LED_DMA
#endif

/* Check that Serial LED and HRT timers are different */

#if defined(HRT_TIMER)
# if S_RGB_LED_TIMER == HRT_TIMER
#   error S_RGB_LED_TIMER and HRT_TIMER must use different timers.
# endif
#endif

/* Check that Serial LED and Tone Alarm timers are different */

#if defined(TONE_ALARM_TIMER)
# if S_RGB_LED_TIMER == TONE_ALARM_TIMER
#   error S_RGB_LED_TIMER and TONE_ALARM_TIMER must use different timers.
# endif
#endif

/* The H7 has a 2 RCC_APB1ENR registers RCC_APB1LENR and RCC_APB1HENR
 * We simply map the RCC_APB1LENR back to STM32_RCC_APB1ENR as well as
 * the bits
 */

#if !defined(STM32_RCC_APB1ENR) && defined(STM32_RCC_APB1LENR)
# define STM32_RCC_APB1ENR STM32_RCC_APB1LENR

# define RCC_APB1ENR_TIM2EN  RCC_APB1LENR_TIM2EN
# define RCC_APB1ENR_TIM3EN  RCC_APB1LENR_TIM3EN
# define RCC_APB1ENR_TIM4EN  RCC_APB1LENR_TIM4EN
# define RCC_APB1ENR_TIM5EN  RCC_APB1LENR_TIM5EN
# define RCC_APB1ENR_TIM6EN  RCC_APB1LENR_TIM6EN
# define RCC_APB1ENR_TIM7EN  RCC_APB1LENR_TIM7EN
# define RCC_APB1ENR_TIM12EN RCC_APB1LENR_TIM12EN
# define RCC_APB1ENR_TIM13EN RCC_APB1LENR_TIM13EN
# define RCC_APB1ENR_TIM14EN RCC_APB1LENR_TIM14EN
#endif

/* Serial LED Timer configuration */

#if S_RGB_LED_TIMER == 1
# define S_RGB_LED_BASE                STM32_TIM1_BASE
# define S_RGB_LED_CLOCK               STM32_APB2_TIM1_CLKIN
# define S_RGB_LED_CLOCK_ENABLE        RCC_APB2ENR_TIM1EN
# define S_RGB_LED_CLOCK_POWER_REG     STM32_RCC_APB2ENR
# if defined(CONFIG_STM32_TIM1)
#  error Must not set CONFIG_STM32_TIM1 when S_RGB_LED_TIMER is 1
# endif
#elif S_RGB_LED_TIMER == 2
# define S_RGB_LED_BASE                STM32_TIM2_BASE
# define S_RGB_LED_CLOCK               STM32_APB1_TIM2_CLKIN
# define S_RGB_LED_CLOCK_ENABLE        RCC_APB1ENR_TIM2EN
# define S_RGB_LED_CLOCK_POWER_REG     STM32_RCC_APB1ENR
# if defined(CONFIG_STM32_TIM2)
#  error Must not set CONFIG_STM32_TIM2 when S_RGB_LED_TIMER is 2
# endif
#elif S_RGB_LED_TIMER == 3
# define S_RGB_LED_BASE                STM32_TIM3_BASE
# define S_RGB_LED_CLOCK               STM32_APB1_TIM3_CLKIN
# define S_RGB_LED_CLOCK_ENABLE        RCC_APB1ENR_TIM3EN
# define S_RGB_LED_CLOCK_POWER_REG     STM32_RCC_APB1ENR
# if defined(CONFIG_STM32_TIM3)
#  error Must not set CONFIG_STM32_TIM3 when S_RGB_LED_TIMER is 3
# endif
#elif S_RGB_LED_TIMER == 4
# define S_RGB_LED_BASE                STM32_TIM4_BASE
# define S_RGB_LED_CLOCK               STM32_APB1_TIM4_CLKIN
# define S_RGB_LED_CLOCK_ENABLE        RCC_APB1ENR_TIM4EN
# define S_RGB_LED_CLOCK_POWER_REG     STM32_RCC_APB1ENR
# if defined(CONFIG_STM32_TIM4)
#  error Must not set CONFIG_STM32_TIM4 when S_RGB_LED_TIMER is 4
# endif
#elif S_RGB_LED_TIMER == 5
# define S_RGB_LED_BASE                STM32_TIM5_BASE
# define S_RGB_LED_CLOCK               STM32_APB1_TIM5_CLKIN
# define S_RGB_LED_CLOCK_ENABLE        RCC_APB1ENR_TIM5EN
# define S_RGB_LED_CLOCK_POWER_REG     STM32_RCC_APB1ENR
# if defined(CONFIG_STM32_TIM5)
#  error Must not set CONFIG_STM32_TIM5 when S_RGB_LED_TIMER is 5
# endif
#elif S_RGB_LED_TIMER == 8
# define S_RGB_LED_BASE                STM32_TIM8_BASE
# define S_RGB_LED_CLOCK               STM32_APB2_TIM8_CLKIN
# define S_RGB_LED_CLOCK_ENABLE        RCC_APB2ENR_TIM8EN
# define S_RGB_LED_CLOCK_POWER_REG     STM32_RCC_APB2ENR
# if defined(CONFIG_STM32_TIM8)
#  error Must not set CONFIG_STM32_TIM8 when S_RGB_LED_TIMER is 8
# endif
#elif S_RGB_LED_TIMER == 9
# define S_RGB_LED_BASE                STM32_TIM9_BASE
# define S_RGB_LED_CLOCK               STM32_APB2_TIM9_CLKIN
# define S_RGB_LED_CLOCK_ENABLE        RCC_APB2ENR_TIM9EN
# define S_RGB_LED_CLOCK_POWER_REG     STM32_RCC_APB2ENR
# if defined(CONFIG_STM32_TIM9)
#  error Must not set CONFIG_STM32_TIM9 when S_RGB_LED_TIMER is 9
# endif
#elif S_RGB_LED_TIMER == 10
# define S_RGB_LED_BASE                STM32_TIM10_BASE
# define S_RGB_LED_CLOCK               STM32_APB2_TIM10_CLKIN
# define S_RGB_LED_CLOCK_ENABLE        RCC_APB2ENR_TIM10EN
# define S_RGB_LED_CLOCK_POWER_REG     STM32_RCC_APB2ENR
# if defined(CONFIG_STM32_TIM10)
#  error Must not set CONFIG_STM32_TIM10 when S_RGB_LED_TIMER is 10
# endif
#elif S_RGB_LED_TIMER == 11
# define S_RGB_LED_BASE                STM32_TIM11_BASE
# define S_RGB_LED_CLOCK               STM32_APB2_TIM11_CLKIN
# define S_RGB_LED_CLOCK_ENABLE        RCC_APB2ENR_TIM11EN
# define S_RGB_LED_CLOCK_POWER_REG     STM32_RCC_APB2ENR
# if defined(CONFIG_STM32_TIM11)
#  error Must not set CONFIG_STM32_TIM11 when S_RGB_LED_TIMER is 11
# endif
#elif S_RGB_LED_TIMER == 12
# define S_RGB_LED_BASE                STM32_TIM12_BASE
# define S_RGB_LED_CLOCK               STM32_APB1_TIM12_CLKIN
# define S_RGB_LED_CLOCK_ENABLE        RCC_APB1ENR_TIM12EN
# define S_RGB_LED_CLOCK_POWER_REG     STM32_RCC_APB1ENR
# if defined(CONFIG_STM32_TIM12)
#  error Must not set CONFIG_STM32_TIM12 when S_RGB_LED_TIMER is 12
# endif
#elif S_RGB_LED_TIMER == 13
# define S_RGB_LED_BASE                STM32_TIM13_BASE
# define S_RGB_LED_CLOCK               STM32_APB1_TIM13_CLKIN
# define S_RGB_LED_CLOCK_ENABLE        RCC_APB1ENR_TIM13EN
# define S_RGB_LED_CLOCK_POWER_REG     STM32_RCC_APB1ENR
# if defined(CONFIG_STM32_TIM13)
#  error Must not set CONFIG_STM32_TIM13 when S_RGB_LED_TIMER is 13
# endif
#elif S_RGB_LED_TIMER == 14
# define S_RGB_LED_BASE                STM32_TIM14_BASE
# define S_RGB_LED_CLOCK               STM32_APB1_TIM14_CLKIN
# define S_RGB_LED_CLOCK_ENABLE        RCC_APB1ENR_TIM14EN
# define S_RGB_LED_CLOCK_POWER_REG     STM32_RCC_APB1ENR
# if defined(CONFIG_STM32_TIM14)
#  error Must not set CONFIG_STM32_TIM14 when S_RGB_LED_TIMER is 14
# endif
#elif S_RGB_LED_TIMER == 15
# define S_RGB_LED_BASE                STM32_TIM15_BASE
# define S_RGB_LED_CLOCK               STM32_APB2_TIM15_CLKIN
# define S_RGB_LED_CLOCK_ENABLE        RCC_APB2ENR_TIM15EN
# define S_RGB_LED_CLOCK_POWER_REG     STM32_RCC_APB2ENR
# if defined(CONFIG_STM32_TIM15)
#  error Must not set CONFIG_STM32_TIM15 when S_RGB_LED_TIMER is 15
# endif
#elif S_RGB_LED_TIMER == 16
# define S_RGB_LED_BASE                STM32_TIM16_BASE
# define S_RGB_LED_CLOCK               STM32_APB2_TIM16_CLKIN
# define S_RGB_LED_CLOCK_ENABLE        RCC_APB2ENR_TIM16EN
# define S_RGB_LED_CLOCK_POWER_REG     STM32_RCC_APB2ENR
# if defined(CONFIG_STM32_TIM16)
#  error Must not set CONFIG_STM32_TIM16 when S_RGB_LED_TIMER is 16
# endif
#elif S_RGB_LED_TIMER == 17
# define S_RGB_LED_BASE                STM32_TIM17_BASE
# define S_RGB_LED_CLOCK               STM32_APB2_TIM17_CLKIN
# define S_RGB_LED_CLOCK_ENABLE        RCC_APB2ENR_TIM17EN
# define S_RGB_LED_CLOCK_POWER_REG     STM32_RCC_APB2ENR
# if defined(CONFIG_STM32_TIM17)
#  error Must not set CONFIG_STM32_TIM16 when S_RGB_LED_TIMER is 17
# endif
#else
# error Must set S_RGB_LED_TIMER to one of the timers between 1 and 17 (inclusive) to use this driver.
#endif // S_RGB_LED_TIMER

// Was the N complementary timer output was used

#if defined(S_RGB_LED_CHANNELN)
#  define CCER_CCnxE GTIM_CCER_CC1NE
#else
#  define CCER_CCnxE GTIM_CCER_CC1E
#endif

#if S_RGB_LED_CHANNEL == 1
# define SLED_CCER      (CCER_CCnxE << 0) |
# define SLED_CCMR1     GTIM_CCMR1_OC1PE | (GTIM_CCMR_MODE_PWM1 << GTIM_CCMR1_OC1M_SHIFT)
# define SLED_CCMR2     0
# define SLED_rCCR      rCCR1
#elif S_RGB_LED_CHANNEL == 2
# define SLED_CCER      (CCER_CCnxE << 4)
# define SLED_CCMR1     GTIM_CCMR1_OC2PE | (GTIM_CCMR_MODE_PWM1 << GTIM_CCMR1_OC2M_SHIFT)
# define SLED_CCMR2     0
# define SLED_rCCR      rCCR2
#elif S_RGB_LED_CHANNEL == 3
# define SLED_CCER      (CCER_CCnxE << 8)
# define SLED_CCMR1     0
# define SLED_CCMR2     GTIM_CCMR1_OC1PE | (GTIM_CCMR_MODE_PWM1 << GTIM_CCMR1_OC1M_SHIFT)
# define SLED_rCCR      rCCR3
#elif S_RGB_LED_CHANNEL == 4
# define SLED_CCER      (CCER_CCnxE << 12)
# define SLED_CCMR1     0
# define SLED_CCMR2     GTIM_CCMR1_OC2PE | (GTIM_CCMR_MODE_PWM1 << GTIM_CCMR1_OC2M_SHIFT)
# define SLED_rCCR      rCCR4
#else
# error Must set S_RGB_LED_CHANNEL to a value between 1 and 4 to use this driver.
#endif // S_RGB_LED_CHANNEL

/* Timer register accessors. */

#define _TIM_REG(_reg)       (S_RGB_LED_BASE + (_reg))
#define TIM_REG(_reg)       (*(volatile uint32_t *)(_TIM_REG(_reg)))

#if S_RGB_LED_TIMER == 1 || S_RGB_LED_TIMER == 8 // Note: If using TIM1 or TIM8, then you are using the ADVANCED timers and NOT the GENERAL TIMERS, therefore different registers
# define rARR           TIM_REG(STM32_ATIM_ARR_OFFSET)
# define rBDTR          TIM_REG(STM32_ATIM_BDTR_OFFSET)
# define rCCER          TIM_REG(STM32_ATIM_CCER_OFFSET)
# define rCCMR1         TIM_REG(STM32_ATIM_CCMR1_OFFSET)
# define rCCMR2         TIM_REG(STM32_ATIM_CCMR2_OFFSET)
# define rCCR1          TIM_REG(STM32_ATIM_CCR1_OFFSET)
# define rCCR2          TIM_REG(STM32_ATIM_CCR2_OFFSET)
# define rCCR3          TIM_REG(STM32_ATIM_CCR3_OFFSET)
# define rCCR4          TIM_REG(STM32_ATIM_CCR4_OFFSET)
# define rCNT           TIM_REG(STM32_ATIM_CNT_OFFSET)
# define rCR1           TIM_REG(STM32_ATIM_CR1_OFFSET)
# define rCR2           TIM_REG(STM32_ATIM_CR2_OFFSET)
# define rDCR           TIM_REG(STM32_ATIM_DCR_OFFSET)
# define rDIER          TIM_REG(STM32_ATIM_DIER_OFFSET)
# define rDMAR          TIM_REG(STM32_ATIM_DMAR_OFFSET)
# define rEGR           TIM_REG(STM32_ATIM_EGR_OFFSET)
# define rPSC           TIM_REG(STM32_ATIM_PSC_OFFSET)
# define rRCR           TIM_REG(STM32_ATIM_RCR_OFFSET)
# define rSMCR          TIM_REG(STM32_ATIM_SMCR_OFFSET)
# define rSR            TIM_REG(STM32_ATIM_SR_OFFSET)
#else
# define rARR           TIM_REG(STM32_GTIM_ARR_OFFSET)
#if S_RGB_LED_TIMER >= 15 && S_RGB_LED_TIMER <= 17 // Note: If using TIM15 - TIM17 it has a BDTR
#   define rBDTR          TIM_REG(STM32_ATIM_BDTR_OFFSET)
#endif
# define rCCER          TIM_REG(STM32_GTIM_CCER_OFFSET)
# define rCCMR1         TIM_REG(STM32_GTIM_CCMR1_OFFSET)
# define rCCMR2         TIM_REG(STM32_GTIM_CCMR2_OFFSET)
# define rCCR1          TIM_REG(STM32_GTIM_CCR1_OFFSET)
# define rCCR2          TIM_REG(STM32_GTIM_CCR2_OFFSET)
# define rCCR3          TIM_REG(STM32_GTIM_CCR3_OFFSET)
# define rCCR4          TIM_REG(STM32_GTIM_CCR4_OFFSET)
# define rCNT           TIM_REG(STM32_GTIM_CNT_OFFSET)
# define rCR1           TIM_REG(STM32_GTIM_CR1_OFFSET)
# define rCR2           TIM_REG(STM32_GTIM_CR2_OFFSET)
# define rDCR           TIM_REG(STM32_GTIM_DCR_OFFSET)
# define rDIER          TIM_REG(STM32_GTIM_DIER_OFFSET)
# define rDMAR          TIM_REG(STM32_GTIM_DMAR_OFFSET)
# define rEGR           TIM_REG(STM32_GTIM_EGR_OFFSET)
# define rPSC           TIM_REG(STM32_GTIM_PSC_OFFSET)
# define rRCR           TIM_REG(STM32_GTIM_RCR_OFFSET)
# define rSMCR          TIM_REG(STM32_GTIM_SMCR_OFFSET)
# define rSR            TIM_REG(STM32_GTIM_SR_OFFSET)
#endif

// The DMA control word

#define SLED_DMA_SCR (DMA_SCR_PRIHI | DMA_SCR_MSIZE_16BITS | DMA_SCR_PSIZE_16BITS | DMA_SCR_MINC | \
		      DMA_SCR_DIR_M2P | DMA_SCR_TCIE | DMA_SCR_TEIE | DMA_SCR_DMEIE)

#define DIVISOR          1000000000ll // working in nS
// Timing from data sheet Total bit time is 1200 nS
// one is ----____, (600/600) a zero is ---_____ (300/900)

#define T0H              ((300ll  * S_RGB_LED_CLOCK) / DIVISOR)
#define T1H              ((600ll  * S_RGB_LED_CLOCK) / DIVISOR)
#define TW               ((1200ll * S_RGB_LED_CLOCK) / DIVISOR)

#define COLOR_PER_LED   3  // There is a R G B in each package.
#define BITS_PER_COLOR  8  // Each LED has 8 bits of luminosity
#define BITS_PER_PACKAGE (BITS_PER_COLOR * COLOR_PER_LED)


// The DMA handle used by the driver.
static DMA_HANDLE  dma_handle;

// DMA Call cack stops timer.

void dma_callback(DMA_HANDLE handle, uint8_t status, void *arg)
{
	rCR1 &= ~GTIM_CR1_CEN;  // Stop the Timer
}

// The led pwm API


/* Begin 2 words be bit: high time, low time repeated the number of bits per color
 * times colors per LED package * times number of led packages
 *  [hi][lo]:{8 * 3 * 8} [ffff] Last DMA will set the output low
 *  Output =  ctr < ccr ? 1 : 0;
*/
uint16_t bits[(BITS_PER_COLOR * COLOR_PER_LED * BOARD_HAS_N_S_RGB_LED) + 1] __attribute__((aligned(sizeof(uint16_t))));

extern int neopixel_write(neopixel::NeoLEDData *led_data, int number_of_packages)
{
	uint32_t mask;

	// DMA in progress let it finish

	while (rCR1 & GTIM_CR1_CEN) {
		usleep(1);
	}

	// For the bits times to DMA

	for (uint32_t i = 0, leds = 0; i < arraySize(bits) - 1; i++) {
		mask = 1 << ((BITS_PER_PACKAGE - 1) - (i % BITS_PER_PACKAGE));
		bits[i] = led_data[leds].data.l & mask ? T1H : T0H;

		if (mask & 1) {
			leds++;
		}
	}

	// Set up the DMA Operations

	stm32_dmasetup(dma_handle,
		       _TIM_REG(STM32_GTIM_DMAR_OFFSET),
		       (uint32_t) bits,
		       arraySize(bits),
		       SLED_DMA_SCR);

	// atomic operations
	irqstate_t flags = px4_enter_critical_section();

	// Prep the timer for update, start with the first bit.
	SLED_rCCR = bits[0];
	rEGR |= GTIM_EGR_UG;
	rCR1 |= GTIM_CR1_CEN;  // Start the Timer

	// And away we go
	stm32_dmastart(dma_handle, dma_callback, NULL, false);
	px4_leave_critical_section(flags);

	return 0;
}

// DMA/TIMER setup

int neopixel_init(neopixel::NeoLEDData *led_data, int number_of_packages)
{
	// Array has a trailing last value of 0 out put off
	if (number_of_packages > BOARD_HAS_N_S_RGB_LED) {
		return -1;
	}

	memset(bits, 0, sizeof(bits));

	/* Enable the timer clock before we try to talk to it */

	modifyreg32(S_RGB_LED_CLOCK_POWER_REG, 0, S_RGB_LED_CLOCK_ENABLE);

	/* disable and configure the timer */

	rCR1   = 0;
	rCR2   = 0;
	rCCER  &= SLED_CCER; // Unlock CCMR* registers.
	rCCER  = SLED_CCER;
	rCCMR1 = SLED_CCMR1;
	rCCMR2 = SLED_CCMR2;
	rDCR   = 0;
	rDIER  = 0;
	rSMCR  = 0;

#if defined(rBDTR)

	/* master output enable = on */

	rBDTR = ATIM_BDTR_MOE;
#endif

	// Timing is based on the clock for the timer not divided at all.

	rPSC  = 0;

	/* Set the bit time */

	rARR = TW;

	// Set up DMA for 1 transefer to SLED_rCCR register
	rDCR   = ATIM_DCR_DBL(1) | (((uint32_t)&SLED_rCCR) - S_RGB_LED_BASE) >> 2;

	// DMA on Compar events

	rDIER |= ATIM_DIER_CC1DE << (S_RGB_LED_CHANNEL - 1);

	SLED_rCCR = 0;

	// Down count, edge aligned PWN 1 wiht auto reload

	rCR1 = GTIM_CR1_ARPE | GTIM_CR1_EDGE | GTIM_CR1_DIR;

	// Set up the GPIO

	stm32_configgpio(S_RGB_LED_TIM_GPIO);

	// Get a DMA channel.

	dma_handle = stm32_dmachannel(S_RGB_LED_DMA);

	return (dma_handle == nullptr) ? -1 : 0;
}

int neopixel_deinit()
{
	if (dma_handle != nullptr) {
		stm32_dmafree(dma_handle);
	}

	return OK;
}
#endif // defined(BOARD_HAS_N_S_RGB_LED) && defined(S_RGB_LED_DMA)
