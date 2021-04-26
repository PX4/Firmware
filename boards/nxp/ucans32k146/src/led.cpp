/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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
#include <drivers/drv_pwm_output.h>

#include <px4_arch/io_timer.h>

#include "s32k1xx_pin.h"
#include "hardware/s32k1xx_ftm.h"

#define REG(offset) (*(volatile uint32_t *)(S32K1XX_FTM0_BASE + (offset)))

#define rSC()         REG(S32K1XX_FTM_SC_OFFSET)
#define rCNT()        REG(S32K1XX_FTM_CNT_OFFSET)
#define rMOD()        REG(S32K1XX_FTM_MOD_OFFSET)

#define FTM_SRC_CLOCK_FREQ  8000000
#define LED_PWM_FREQ        1000000

extern int led_pwm_servo_set(unsigned channel, uint8_t  value);
extern int led_pwm_servo_init(void);

void rgb_led(int r, int g, int b, int freqs)
{
	long fosc = FTM_SRC_CLOCK_FREQ;
	long prescale = 128;
	long p1s = fosc / prescale;
	long p0p5s  = p1s / 2;
	long p  = freqs == 0 ? p1s : p1s / freqs;

	static bool once = false;

	if (!once) {
		once = true;
		led_pwm_servo_init();
	}

	irqstate_t flags = px4_enter_critical_section();
	uint32_t save = rSC();
	rSC() = save & ~(FTM_SC_CLKS_MASK);
	rMOD() = p;
	rSC() = save | FTP_SC_PS_DIV128;
	px4_leave_critical_section(flags);

	p  = freqs == 0 ? p1s + 1 : p0p5s / freqs;

	led_pwm_servo_set(0, r * p);
	led_pwm_servo_set(1, g * p);
	led_pwm_servo_set(2, b * p);

	if (freqs == 0) {
		rSC()    |= FTM_SC_CLKS_DIS;

	} else {
		rSC()    &= ~FTM_SC_CLKS_DIS;
	}
}
