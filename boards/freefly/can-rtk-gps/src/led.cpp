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
#include "led.h"

#include <string.h>

#include <hardware/stm32_tim.h>
#include <drivers/device/i2c.h>
#include <lib/led/led.h>

__BEGIN_DECLS
#include "cxx_init.h"
void board_autoled_on(int led);
void board_autoled_off(int led);
__END_DECLS

using namespace time_literals;
#define MODULE_NAME "LED"

#define ADDR                0x39  /**< I2C adress of NCP5623C */

#define NCP5623_LED_CURRENT 0x20  /**< Current register */
#define NCP5623_LED_PWM0    0x40  /**< pwm0 register */
#define NCP5623_LED_PWM1    0x60  /**< pwm1 register */
#define NCP5623_LED_PWM2    0x80  /**< pwm2 register */

#define NCP5623_LED_BRIGHT  0x1f  /**< full brightness */
#define NCP5623_LED_OFF     0x00  /**< off */


class RGBLED_NCP5623C : public device::I2C
{
public:
	RGBLED_NCP5623C(const int bus, int bus_frequency, const int address);
	virtual ~RGBLED_NCP5623C() = default;

	int   init() override;
	int   probe() override;

	int   send_led_rgb(uint8_t red, uint8_t green, uint8_t blue);


private:
	float     _brightness{1.0f};
	float     _max_brightness{1.0f};

	int     write(uint8_t reg, uint8_t data);
};

RGBLED_NCP5623C::RGBLED_NCP5623C(const int bus, int bus_frequency, const int address) :
	I2C(DRV_LED_DEVTYPE_RGBLED_NCP5623C, MODULE_NAME, bus, address, bus_frequency)
{
}

int
RGBLED_NCP5623C::write(uint8_t reg, uint8_t data)
{
	uint8_t msg[1] = { 0x00 };
	msg[0] = ((reg & 0xe0) | (data & 0x1f));

	int ret = transfer(&msg[0], 1, nullptr, 0);

	return ret;
}

int
RGBLED_NCP5623C::init()
{
	int ret = I2C::init();

	if (ret != OK) {
		return ret;
	}

	return OK;
}

int
RGBLED_NCP5623C::probe()
{
	_retries = 1;

	return write(NCP5623_LED_CURRENT, 0x00);
}


/**
 * Send RGB PWM settings to LED driver according to current color and brightness
 */
int
RGBLED_NCP5623C::send_led_rgb(uint8_t red, uint8_t green, uint8_t blue)
{

	uint8_t msg[7] = {0x20, 0x70, 0x40, 0x70, 0x60, 0x70, 0x80};
	uint8_t brightness = 0x1f * _max_brightness;

	msg[0] = NCP5623_LED_CURRENT | (brightness & 0x1f);
	msg[2] = NCP5623_LED_PWM0 | (uint8_t(red * _brightness) & 0x1f);
	msg[4] = NCP5623_LED_PWM1 | (uint8_t(green * _brightness) & 0x1f);
	msg[6] = NCP5623_LED_PWM2 | (uint8_t(blue * _brightness) & 0x1f);

	return transfer(&msg[0], 7, nullptr, 0);
}

static RGBLED_NCP5623C instance(1, 100000, ADDR);

#define TMR_BASE        STM32_TIM1_BASE
#define TMR_FREQUENCY   STM32_APB2_TIM1_CLKIN
#define TMR_REG(o)      (TMR_BASE+(o))

static  uint8_t _rgb[] = {0, 0, 0};

static int timerInterrupt(int irq, void *context, void *arg)
{
	putreg16(~getreg16(TMR_REG(STM32_GTIM_SR_OFFSET)), TMR_REG(STM32_GTIM_SR_OFFSET));

	static int d2 = 1;
	(d2++ & 1) ? instance.send_led_rgb(0, 0, 0) : instance.send_led_rgb(_rgb[0], _rgb[1], _rgb[2]);

	return 0;
}


void rgb_led(int r, int g, int b, int freqs)
{
	long fosc = TMR_FREQUENCY;
	long prescale = 1536;
	long p1s = fosc / prescale;
	long p0p5s  = p1s / 2;
	uint16_t val;
	static bool once = false;

	if (!once) {
		cxx_initialize();

		if (instance.init() != PX4_OK) {
			return;
		}

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
		once = true;
	}

	long p  = freqs == 0 ? p1s + 1 : p0p5s / freqs;
	putreg32(p + 1, TMR_REG(STM32_BTIM_ARR_OFFSET));
	putreg32(p, TMR_REG(STM32_GTIM_CCR1_OFFSET));
	_rgb[0] = r;
	_rgb[1] = g;
	_rgb[2] = b;

	val = getreg16(TMR_REG(STM32_BTIM_CR1_OFFSET));

	if (freqs == 0) {
		val &= ~ATIM_CR1_CEN;

	} else {
		val |= ATIM_CR1_CEN;
	}

	putreg16(val, TMR_REG(STM32_BTIM_CR1_OFFSET));
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

__EXPORT void board_autoled_on(int led)
{
	if (led == LED_ASSERTION || led == LED_PANIC) {
		stm32_gpiowrite(GPIO_LED_SAFETY, true);
	}
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

__EXPORT void board_autoled_off(int led)
{
	if (led == LED_ASSERTION || led == LED_PANIC) {
		stm32_gpiowrite(GPIO_LED_SAFETY, false);
	}

}
