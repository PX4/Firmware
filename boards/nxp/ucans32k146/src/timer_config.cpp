/****************************************************************************
 *
 *   Copyright (C) 2016, 2018 PX4 Development Team. All rights reserved.
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

/*
 * @file timer_config.c
 *
 * Configuration data for the S32K1XX pwm_servo, input capture and pwm input driver.
 *
 * Note that these arrays must always be fully-sized.
 */

// TODO:Stubbed out for now
#include <stdint.h>

#include "hardware/s32k1xx_pcc.h"
#include "hardware/s32k1xx_ftm.h"
#include "s32k1xx_pin.h"

#include <drivers/drv_pwm_output.h>
#include <px4_arch/io_timer_hw_description.h>

#include "board_config.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Register accessors */

#define _REG(_addr)	(*(volatile uint32_t *)(_addr))

/* Timer register accessors */

#define REG(t, _reg)	_REG(S32K1XX_FTM##t##_BASE + (_reg))

#define rSC(t)         REG(t,S32K1XX_FTM_SC_OFFSET)
#define rCNT(t)        REG(t,S32K1XX_FTM_CNT_OFFSET)
#define rMOD(t)        REG(t,S32K1XX_FTM_MOD_OFFSET)
#define rC0SC(t)       REG(t,S32K1XX_FTM_C0SC_OFFSET)
#define rC0V(t)        REG(t,S32K1XX_FTM_C0V_OFFSET)
#define rC1SC(t)       REG(t,S32K1XX_FTM_C1SC_OFFSET)
#define rC1V(t)        REG(t,S32K1XX_FTM_C1V_OFFSET)
#define rC2SC(t)       REG(t,S32K1XX_FTM_C2SC_OFFSET)
#define rC2V(t)        REG(t,S32K1XX_FTM_C2V_OFFSET)
#define rC3SC(t)       REG(t,S32K1XX_FTM_C3SC_OFFSET)
#define rC3V(t)        REG(t,S32K1XX_FTM_C3V_OFFSET)
#define rC4SC(t)       REG(t,S32K1XX_FTM_C4SC_OFFSET)
#define rC4V(t)        REG(t,S32K1XX_FTM_C4V_OFFSET)
#define rC5SC(t)       REG(t,S32K1XX_FTM_C5SC_OFFSET)
#define rC5V(t)        REG(t,S32K1XX_FTM_C5V_OFFSET)
#define rC6SC(t)       REG(t,S32K1XX_FTM_C6SC_OFFSET)
#define rC6V(t)        REG(t,S32K1XX_FTM_C6V_OFFSET)
#define rC7SC(t)       REG(t,S32K1XX_FTM_C7SC_OFFSET)
#define rC7V(t)        REG(t,S32K1XX_FTM_C7V_OFFSET)

#define rCNTIN(t)      REG(t,S32K1XX_FTM_CNTIN_OFFSET)
#define rSTATUS(t)     REG(t,S32K1XX_FTM_STATUS_OFFSET)
#define rMODE(t)       REG(t,S32K1XX_FTM_MODE_OFFSET)
#define rSYNC(t)       REG(t,S32K1XX_FTM_SYNC_OFFSET)
#define rOUTINIT(t)    REG(t,S32K1XX_FTM_OUTINIT_OFFSET)
#define rOUTMASK(t)    REG(t,S32K1XX_FTM_OUTMASK_OFFSET)
#define rCOMBINE(t)    REG(t,S32K1XX_FTM_COMBINE_OFFSET)
#define rDEADTIME(t)   REG(t,S32K1XX_FTM_DEADTIME_OFFSET)
#define rEXTTRIG(t)    REG(t,S32K1XX_FTM_EXTTRIG_OFFSET)
#define rPOL(t)        REG(t,S32K1XX_FTM_POL_OFFSET)
#define rFMS(t)        REG(t,S32K1XX_FTM_FMS_OFFSET)
#define rFILTER(t)     REG(t,S32K1XX_FTM_FILTER_OFFSET)
#define rFLTCTRL(t)    REG(t,S32K1XX_FTM_FLTCTRL_OFFSET)
#define rQDCTRL(t)     REG(t,S32K1XX_FTM_QDCTRL_OFFSET)
#define rCONF(t)       REG(t,S32K1XX_FTM_CONF_OFFSET)
#define rFLTPOL(t)     REG(t,S32K1XX_FTM_FLTPOL_OFFSET)
#define rSYNCONF(t)    REG(t,S32K1XX_FTM_SYNCONF_OFFSET)
#define rINVCTRL(t)    REG(t,S32K1XX_FTM_INVCTRL_OFFSET)
#define rSWOCTRL(t)    REG(t,S32K1XX_FTM_SWOCTRL_OFFSET)
#define rPWMLOAD(t)    REG(t,S32K1XX_FTM_PWMLOAD_OFFSET)

constexpr io_timers_t io_timers[MAX_IO_TIMERS] = {
	initIOTimer(Timer::FTM2),
};

constexpr timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
	initIOTimerChannel(io_timers, {Timer::FTM2, Timer::Channel1}, {GPIO::PortA, GPIO::Pin0}),
};

constexpr io_timers_channel_mapping_t io_timers_channel_mapping =
	initIOTimerChannelMapping(io_timers, timer_io_channels);

const struct io_timers_t led_pwm_timers[MAX_LED_TIMERS] = {
	{
		.base = S32K1XX_FTM0_BASE,
		.clock_register = S32K1XX_PCC_FTM0,
		.clock_bit = PCC_CGC,
		//.vectorno =  0,
	},
};

const struct timer_io_channels_t led_pwm_channels[MAX_TIMER_LED_CHANNELS] = {
	{
		.gpio_out = LED_TIM0_CH0OUT, // RGB_R
		.gpio_in  = 0,
		.timer_index = 0,
		.timer_channel = 1,
	},
	{
		.gpio_out = LED_TIM0_CH1OUT, // RGB_G
		.gpio_in  = 0,
		.timer_index = 0,
		.timer_channel = 2,
	},
	{
		.gpio_out = LED_TIM0_CH2OUT, // RGB_B
		.gpio_in  = 0,
		.timer_index = 0,
		.timer_channel = 3,
	},
};

void ucans32k_timer_initialize(void)
{
	/* Select external clock (set to SOSC in periphclocks.c) as source for FTM0, and FTM2 */

	uint32_t regval = _REG(S32K1XX_FTM0_SC);
	regval &= ~(FTM_SC_CLKS_MASK);
	regval |= FTM_SC_CLKS_FTM;
	_REG(S32K1XX_FTM0_SC) = regval;

	regval = _REG(S32K1XX_FTM2_SC);
	regval &= ~(FTM_SC_CLKS_MASK);
	regval |= FTM_SC_CLKS_FTM;
	_REG(S32K1XX_FTM2_SC) = regval;

	/* Enabled System Clock Gating Control for FTM0, and FTM2 */

	regval = _REG(S32K1XX_PCC_FTM0);
	regval |= PCC_CGC;
	_REG(S32K1XX_PCC_FTM0) = regval;

	regval = _REG(S32K1XX_PCC_FTM2);
	regval |= PCC_CGC;
	_REG(S32K1XX_PCC_FTM2) = regval;
}
