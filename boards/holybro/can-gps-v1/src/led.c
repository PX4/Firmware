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
 * @file led.c
 *
 * LED backend.
 */

#include <px4_platform_common/px4_config.h>

#include <stdbool.h>

#include "chip.h"
#include "stm32_gpio.h"
#include "board_config.h"

#include <nuttx/board.h>
#include <arch/board/board.h>

// TODO:This needs a complete rewrite We need to change the HW
// Swap PB0/PB2 and use timer for all LEDS

#define TMR_BASE        STM32_TIM1_BASE
#define TMR_FREQUENCY   STM32_APB2_TIM1_CLKIN
#define TMR_REG(o)      (TMR_BASE+(o))

static  uint8_t _off[] =  {0, 0, 0};
static  uint8_t _rgb[] =  {0, 0, 0};

/*
 * Ideally we'd be able to get these from arm_internal.h,
 * but since we want to be able to disable the NuttX use
 * of leds for system indication at will and there is no
 * separate switch, we need to build independent of the
 * CONFIG_ARCH_LEDS configuration switch.
 */
__BEGIN_DECLS
extern void led_init(void);
extern void led_on(int led);
extern void led_off(int led);
extern void led_toggle(int led);
__END_DECLS

#define LED_RED     1
#define LED_BLUE    0
#define LED_GREEN   3

#define xlat(p) (p)
static uint32_t g_ledmap[] = {
	GPIO_MCU_NLED_BLUE,                 // Indexed by LED_BLUE
	GPIO_MCU_NLED_RED,                  // Indexed by LED_RED, LED_AMBER
	0,                                  // Indexed by LED_SAFETY (defaulted to an input)
	GPIO_MCU_NLED_GREEN                 // Indexed by LED_GREEN
};

__EXPORT void led_init(void)
{
	for (size_t l = 0; l < (sizeof(g_ledmap) / sizeof(g_ledmap[0])); l++) {
		if (g_ledmap[l] != 0) {
			stm32_configgpio(g_ledmap[l]);
		}
	}
}

static void phy_set_led(int led, bool state)
{
	/* Drive Low to switch on */

	if (g_ledmap[led] != 0) {
		stm32_gpiowrite(g_ledmap[led], !state);
	}
}

static bool phy_get_led(int led)
{
	/* If Low it is on */
	if (g_ledmap[led] != 0) {
		return !stm32_gpioread(g_ledmap[led]);
	}

	return false;
}

__EXPORT void led_on(int led)
{
	phy_set_led(xlat(led), true);
}

__EXPORT void led_off(int led)
{
	phy_set_led(xlat(led), false);
}

__EXPORT void led_toggle(int led)
{
	phy_set_led(xlat(led), !phy_get_led(xlat(led)));
}


static void setled(uint8_t *rgb)
{
	phy_set_led(LED_RED,   _rgb[0]);
	phy_set_led(LED_GREEN, _rgb[1]);
	phy_set_led(LED_BLUE,  _rgb[2]);
}


static int timerInterrupt(int irq, void *context, void *arg)
{
	putreg16(~getreg16(TMR_REG(STM32_GTIM_SR_OFFSET)), TMR_REG(STM32_GTIM_SR_OFFSET));

	static int d2 = 1;
	setled((d2++ & 1) ? _rgb : _off);
	return 0;
}

void rgb_led(int r, int g, int b, int freqs)
{
	long fosc = TMR_FREQUENCY;
	long prescale = 1536;
	long p1s = fosc / prescale;
	long p0p5s  = p1s / 2;
	uint16_t val;
	static bool once = 0;

	if (!once) {
		once = 1;
		setled(_off);

		/* Enable Clock to Block */

		modifyreg32(STM32_RCC_APB2ENR, 0, RCC_APB2ENR_TIM1EN);

		/* Reload */
		val = getreg16(TMR_REG(STM32_BTIM_EGR_OFFSET));
		val |= ATIM_EGR_UG;
		putreg16(val, TMR_REG(STM32_BTIM_EGR_OFFSET));

		/* Set Prescaler STM32_TIM_SETCLOCK */
		putreg16(prescale, TMR_REG(STM32_BTIM_PSC_OFFSET));

		/* Enable STM32_TIM_SETMODE*/
		putreg16(ATIM_CR1_CEN | ATIM_CR1_ARPE, TMR_REG(STM32_BTIM_CR1_OFFSET));

		putreg32(p0p5s + 1, TMR_REG(STM32_BTIM_ARR_OFFSET));


		irq_attach(STM32_IRQ_TIM1CC, timerInterrupt, NULL);
		up_enable_irq(STM32_IRQ_TIM1CC);
		putreg16(GTIM_DIER_CC1IE, TMR_REG(STM32_GTIM_DIER_OFFSET));
	}

	long p  = freqs == 0 ? p1s + 1 : p0p5s / freqs;
	putreg32(p + 1, TMR_REG(STM32_BTIM_ARR_OFFSET));
	putreg32(p, TMR_REG(STM32_GTIM_CCR1_OFFSET));
	_rgb[0] = g;
	_rgb[1] = r;
	_rgb[2] = b;
	setled(_rgb);

	val = getreg16(TMR_REG(STM32_BTIM_CR1_OFFSET));

	if (freqs == 0) {
		val &= ~ATIM_CR1_CEN;

	} else {
		val |= ATIM_CR1_CEN;
	}

	putreg16(val, TMR_REG(STM32_BTIM_CR1_OFFSET));
}
