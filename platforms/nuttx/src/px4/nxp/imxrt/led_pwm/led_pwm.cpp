/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name Airmind nor the names of its contributors may be
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
* @file drv_led_pwm.cpp
*
*
*/

#include <px4_platform_common/px4_config.h>
#include <systemlib/px4_macros.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdbool.h>

#include <assert.h>
#include <debug.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

#include <arch/board/board.h>
#include <drivers/drv_pwm_output.h>

#include <px4_arch/io_timer.h>

#include <chip.h>
#include "hardware/imxrt_tmr.h"

int led_pwm_servo_set(unsigned channel, uint8_t  cvalue)
{
	return 0;
}
int led_pwm_servo_init(void)
{
	return 0;

}

#if 0 && defined(BOARD_HAS_LED_PWM) || defined(BOARD_HAS_UI_LED_PWM)

#define FTM_SRC_CLOCK_FREQ 16000000
#define LED_PWM_FREQ        1000000

#if (BOARD_LED_PWM_RATE)
#  define LED_PWM_RATE BOARD_LED_PWM_RATE
#else
#  define LED_PWM_RATE 50
#endif

#define _REG(_addr)	(*(volatile uint32_t *)(_addr))
#define _REG32(_base, _reg)	(*(volatile uint32_t *)(_base + _reg))
#define REG(_tmr, _reg)		_REG32(led_pwm_timers[_tmr].base, _reg)


/* Timer register accessors */

#define rSC(_tmr)         REG(_tmr,KINETIS_FTM_SC_OFFSET)
#define rCNT(_tmr)        REG(_tmr,KINETIS_FTM_CNT_OFFSET)
#define rMOD(_tmr)        REG(_tmr,KINETIS_FTM_MOD_OFFSET)
#define rC0SC(_tmr)       REG(_tmr,KINETIS_FTM_C0SC_OFFSET)
#define rC0V(_tmr)        REG(_tmr,KINETIS_FTM_C0V_OFFSET)
#define rC1SC(_tmr)       REG(_tmr,KINETIS_FTM_C1SC_OFFSET)
#define rC1V(_tmr)        REG(_tmr,KINETIS_FTM_C1V_OFFSET)
#define rC2SC(_tmr)       REG(_tmr,KINETIS_FTM_C2SC_OFFSET)
#define rC2V(_tmr)        REG(_tmr,KINETIS_FTM_C2V_OFFSET)
#define rC3SC(_tmr)       REG(_tmr,KINETIS_FTM_C3SC_OFFSET)
#define rC3V(_tmr)        REG(_tmr,KINETIS_FTM_C3V_OFFSET)
#define rC4SC(_tmr)       REG(_tmr,KINETIS_FTM_C4SC_OFFSET)
#define rC4V(_tmr)        REG(_tmr,KINETIS_FTM_C4V_OFFSET)
#define rC5SC(_tmr)       REG(_tmr,KINETIS_FTM_C5SC_OFFSET)
#define rC5V(_tmr)        REG(_tmr,KINETIS_FTM_C5V_OFFSET)
#define rC6SC(_tmr)       REG(_tmr,KINETIS_FTM_C6SC_OFFSET)
#define rC6V(_tmr)        REG(_tmr,KINETIS_FTM_C6V_OFFSET)
#define rC7SC(_tmr)       REG(_tmr,KINETIS_FTM_C7SC_OFFSET)
#define rC7V(_tmr)        REG(_tmr,KINETIS_FTM_C7V_OFFSET)

#define rCNTIN(_tmr)      REG(_tmr,KINETIS_FTM_CNTIN_OFFSET)
#define rSTATUS(_tmr)     REG(_tmr,KINETIS_FTM_STATUS_OFFSET)
#define rMODE(_tmr)       REG(_tmr,KINETIS_FTM_MODE_OFFSET)
#define rSYNC(_tmr)       REG(_tmr,KINETIS_FTM_SYNC_OFFSET)
#define rOUTINIT(_tmr)    REG(_tmr,KINETIS_FTM_OUTINIT_OFFSET)
#define rOUTMASK(_tmr)    REG(_tmr,KINETIS_FTM_OUTMASK_OFFSET)
#define rCOMBINE(_tmr)    REG(_tmr,KINETIS_FTM_COMBINE_OFFSET)
#define rDEADTIME(_tmr)   REG(_tmr,KINETIS_FTM_DEADTIME_OFFSET)
#define rEXTTRIG(_tmr)    REG(_tmr,KINETIS_FTM_EXTTRIG_OFFSET)
#define rPOL(_tmr)        REG(_tmr,KINETIS_FTM_POL_OFFSET)
#define rFMS(_tmr)        REG(_tmr,KINETIS_FTM_FMS_OFFSET)
#define rFILTER(_tmr)     REG(_tmr,KINETIS_FTM_FILTER_OFFSET)
#define rFLTCTRL(_tmr)    REG(_tmr,KINETIS_FTM_FLTCTRL_OFFSET)
#define rQDCTRL(_tmr)     REG(_tmr,KINETIS_FTM_QDCTRL_OFFSET)
#define rCONF(_tmr)       REG(_tmr,KINETIS_FTM_CONF_OFFSET)
#define rFLTPOL(_tmr)     REG(_tmr,KINETIS_FTM_FLTPOL_OFFSET)
#define rSYNCONF(_tmr)    REG(_tmr,KINETIS_FTM_SYNCONF_OFFSET)
#define rINVCTRL(_tmr)    REG(_tmr,KINETIS_FTM_INVCTRL_OFFSET)
#define rSWOCTRL(_tmr)    REG(_tmr,KINETIS_FTM_SWOCTRL_OFFSET)
#define rPWMLOAD(_tmr)    REG(_tmr,KINETIS_FTM_PWMLOAD_OFFSET)

#define CnSC_RESET          (FTM_CSC_CHF|FTM_CSC_CHIE|FTM_CSC_MSB|FTM_CSC_MSA|FTM_CSC_ELSB|FTM_CSC_ELSA|FTM_CSC_DMA)
#define CnSC_CAPTURE_INIT   (FTM_CSC_CHIE|FTM_CSC_ELSB|FTM_CSC_ELSA) // Both

#if defined(BOARD_LED_PWM_DRIVE_ACTIVE_LOW)
#define CnSC_PWMOUT_INIT    (FTM_CSC_MSB|FTM_CSC_ELSA)
#else
#define CnSC_PWMOUT_INIT    (FTM_CSC_MSB|FTM_CSC_ELSB)
#endif

#define FTM_SYNC (FTM_SYNC_SWSYNC)

static void             led_pwm_timer_init(unsigned timer);
static void             led_pwm_timer_set_rate(unsigned timer, unsigned rate);
static void             led_pwm_channel_init(unsigned channel);

int led_pwm_servo_set(unsigned channel, uint8_t  value);
unsigned led_pwm_servo_get(unsigned channel);
int led_pwm_servo_init(void);
void led_pwm_servo_deinit(void);
void led_pwm_servo_arm(bool armed);
unsigned led_pwm_timer_get_period(unsigned timer);

static void led_pwm_timer_set_rate(unsigned timer, unsigned rate)
{

	irqstate_t flags = px4_enter_critical_section();

	uint32_t save = rSC(timer);
	rSC(timer) = save & ~(FTM_SC_CLKS_MASK);

	/* configure the timer to update at the desired rate */
	rMOD(timer) = (LED_PWM_FREQ / rate) - 1;
	rSC(timer) = save;

	px4_leave_critical_section(flags);
}

static inline uint32_t div2psc(int div)
{
	return 31 - __builtin_clz(div);
}

static inline void led_pwm_timer_set_PWM_mode(unsigned timer)
{
	irqstate_t flags = px4_enter_critical_section();
	rSC(timer) &= ~(FTM_SC_CLKS_MASK | FTM_SC_PS_MASK);
	rSC(timer) |= (FTM_SC_CLKS_EXTCLK | div2psc(FTM_SRC_CLOCK_FREQ / LED_PWM_FREQ));
	px4_leave_critical_section(flags);
}


static void
led_pwm_timer_init(unsigned timer)
{
	/* valid Timer */

	if (led_pwm_timers[timer].base != 0) {

		/* enable the timer clock before we try to talk to it */

		uint32_t regval = _REG(led_pwm_timers[timer].clock_register);
		regval |= led_pwm_timers[timer].clock_bit;
		_REG(led_pwm_timers[timer].clock_register) = regval;

		/* disable and configure the timer */

		rSC(timer)    = FTM_SC_CLKS_NONE;
		rCNT(timer)   = 0;

		rMODE(timer) = 0;
		rSYNCONF(timer)   = (FTM_SYNCONF_SYNCMODE | FTM_SYNCONF_SWWRBUF | FTM_SYNCONF_SWRSTCNT);

		/* Set to run in debug mode */

		rCONF(timer)   |= FTM_CONF_BDMMODE_MASK;

		/* enable the timer */

		led_pwm_timer_set_PWM_mode(timer);

		/*
		 * Note we do the Standard PWM Out init here
		 * default to updating at LED_PWM_RATE
		 */

		led_pwm_timer_set_rate(timer, LED_PWM_RATE);
	}
}
unsigned
led_pwm_timer_get_period(unsigned timer)
{
	// MOD is a 16 bit reg
	unsigned mod = rMOD(timer);

	if (mod == 0) {
		return 1 << 16;
	}

	return (uint16_t)(mod + 1);
}


static void
led_pwm_channel_init(unsigned channel)
{
	/* Only initialize used channels */

	if (led_pwm_channels[channel].timer_channel) {
		unsigned timer = led_pwm_channels[channel].timer_index;

		irqstate_t flags = px4_enter_critical_section();

		/* configure the GPIO first */

		px4_arch_configgpio(led_pwm_channels[channel].gpio_out);

		/* configure the channel */

		uint32_t chan = led_pwm_channels[channel].timer_channel - 1;

		uint16_t rvalue = REG(timer, KINETIS_FTM_CSC_OFFSET(chan));
		rvalue &= ~CnSC_RESET;
		rvalue |=  CnSC_PWMOUT_INIT;
		REG(timer, KINETIS_FTM_CSC_OFFSET(chan)) = rvalue;
		REG(timer, KINETIS_FTM_CV_OFFSET(0)) = 0;
		px4_leave_critical_section(flags);
	}
}

int
led_pwm_servo_set(unsigned channel, uint8_t  cvalue)
{
	if (channel >= arraySize(led_pwm_channels)) {
		return -1;
	}

	unsigned timer = led_pwm_channels[channel].timer_index;

	/* test timer for validity */
	if ((led_pwm_timers[timer].base == 0) ||
	    (led_pwm_channels[channel].gpio_out == 0)) {
		return -1;
	}

	unsigned period = led_pwm_timer_get_period(timer);

	unsigned value = (unsigned)cvalue * period / 255;

	/* configure the channel */
	if (value > 0) {
		value--;
	}

	REG(timer, KINETIS_FTM_CV_OFFSET(led_pwm_channels[channel].timer_channel - 1)) = value;

	return 0;
}
unsigned
led_pwm_servo_get(unsigned channel)
{
	if (channel >= 3) {
		return 0;
	}

	unsigned timer = led_pwm_channels[channel].timer_index;
	servo_position_t value = 0;

	/* test timer for validity */
	if ((led_pwm_timers[timer].base == 0) ||
	    (led_pwm_channels[channel].timer_channel == 0)) {
		return value;
	}

	value = REG(timer, KINETIS_FTM_CV_OFFSET(led_pwm_channels[channel].timer_channel - 1));
	unsigned period = led_pwm_timer_get_period(timer);
	return ((value + 1) * 255 / period);
}
int
led_pwm_servo_init(void)
{
	/* do basic timer initialisation first */
	for (unsigned i = 0; i < arraySize(led_pwm_timers); i++) {
		led_pwm_timer_init(i);
	}

	/* now init channels */
	for (unsigned i = 0; i < arraySize(led_pwm_channels); i++) {
		led_pwm_channel_init(i);
	}

	led_pwm_servo_arm(true);
	return OK;
}

void
led_pwm_servo_deinit(void)
{
	/* disable the timers */
	led_pwm_servo_arm(false);
}

void
led_pwm_servo_arm(bool armed)
{
	/* iterate timers and arm/disarm appropriately */
	for (unsigned i = 0; i < arraySize(led_pwm_timers); i++) {
		if (led_pwm_timers[i].base != 0) {
			if (armed) {
				/* force an update to preload all registers */
				led_pwm_timer_set_PWM_mode(i);

			} else {
				/* disable and configure the timer */

				rSC(i)    = FTM_SC_CLKS_NONE;
				rCNT(i)   = 0;
			}
		}
	}
}

#endif // BOARD_HAS_LED_PWM || BOARD_HAS_UI_LED_PWM
