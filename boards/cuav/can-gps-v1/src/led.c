/****************************************************************************
 *
 *   Copyright (C) 2015 PX4 Development Team. All rights reserved.
 *   Author: David Sidrane<david_s5@nscdg.com>
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
#include <px4_config.h>

#include <stdint.h>
#include <stdbool.h>

#include <arch/board/board.h>

#include <hardware/stm32_tim.h>

#include "led.h"

#define TMR_BASE        STM32_TIM3_BASE
#define TMR_FREQUENCY   STM32_APB1_TIM3_CLKIN
#define TMR_REG(o)      (TMR_BASE+(o))

void rgb_led(int r, int g, int b, int freqs)
{
	long fosc = TMR_FREQUENCY;
	long prescale = 2048;
	long p1s = fosc / prescale;
	long p0p5s  = p1s / 2;
	uint16_t val;
	static bool once = 0;

	if (!once) {
		once = 1;

		/* Enabel Clock to Block */
		modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM3EN);

		/* Reload */
		val = getreg16(TMR_REG(STM32_BTIM_EGR_OFFSET));
		val |= ATIM_EGR_UG;
		putreg16(val, TMR_REG(STM32_BTIM_EGR_OFFSET));

		/* Set Prescaler STM32_TIM_SETCLOCK */
		putreg16(prescale, TMR_REG(STM32_BTIM_PSC_OFFSET));

		/* Enable STM32_TIM_SETMODE*/
		putreg16(ATIM_CR1_CEN | ATIM_CR1_ARPE, TMR_REG(STM32_BTIM_CR1_OFFSET));


		putreg16((ATIM_CCMR_MODE_PWM1 << ATIM_CCMR1_OC1M_SHIFT) | ATIM_CCMR1_OC1PE |
			 (ATIM_CCMR_MODE_PWM1 << ATIM_CCMR1_OC2M_SHIFT) | ATIM_CCMR1_OC2PE, TMR_REG(STM32_GTIM_CCMR1_OFFSET));
		putreg16((ATIM_CCMR_MODE_PWM1 << ATIM_CCMR2_OC3M_SHIFT) | ATIM_CCMR2_OC3PE, TMR_REG(STM32_GTIM_CCMR2_OFFSET));
		putreg16(ATIM_CCER_CC3E | ATIM_CCER_CC3P |
			 ATIM_CCER_CC2E | ATIM_CCER_CC2P |
			 ATIM_CCER_CC1E | ATIM_CCER_CC1P, TMR_REG(STM32_GTIM_CCER_OFFSET));

		// TODO: verify
		stm32_configgpio(GPIO_TIM3_CH1OUT_1);
		stm32_configgpio(GPIO_TIM3_CH2OUT_1);
		stm32_configgpio(GPIO_TIM3_CH3OUT_1);
	}

	long p  = freqs == 0 ? p1s : p1s / freqs;
	putreg32(p, TMR_REG(STM32_BTIM_ARR_OFFSET));

	p  = freqs == 0 ? p1s + 1 : p0p5s / freqs;

	putreg32((r * p) / 255, TMR_REG(STM32_GTIM_CCR1_OFFSET));
	putreg32((g * p) / 255, TMR_REG(STM32_GTIM_CCR2_OFFSET));
	putreg32((b * p) / 255, TMR_REG(STM32_GTIM_CCR3_OFFSET));

	val = getreg16(TMR_REG(STM32_BTIM_CR1_OFFSET));

	if (freqs == 0) {
		val &= ~ATIM_CR1_CEN;

	} else {
		val |= ATIM_CR1_CEN;
	}

	putreg16(val, TMR_REG(STM32_BTIM_CR1_OFFSET));
}
