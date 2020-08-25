/****************************************************************************
 *
 *   Copyright (C) 2016, 2018-2019 PX4 Development Team. All rights reserved.
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

// TODO:Stubbed out for now
#include <stdint.h>

#include <chip.h>
#include "hardware/imxrt_tmr.h"
#include "hardware/imxrt_flexpwm.h"
#include "imxrt_gpio.h"
#include "imxrt_iomuxc.h"
#include "hardware/imxrt_pinmux.h"
#include "imxrt_xbar.h"
#include "imxrt_periphclks.h"

#include <drivers/drv_pwm_output.h>
#include <px4_arch/io_timer_hw_description.h>

#include "board_config.h"

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

/* Register accessors */

#define _REG(_addr) (*(volatile uint16_t *)(_addr))

/* QTimer3 register accessors */

#define REG(_reg) _REG(IMXRT_QTIMER3_BASE + IMXRT_TMR_OFFSET(IMXRT_TMR_CH0,(_reg)))

#define rCOMP1        REG(IMXRT_TMR_COMP1_OFFSET)
#define rCOMP2        REG(IMXRT_TMR_COMP2_OFFSET)
#define rCAPT         REG(IMXRT_TMR_CAPT_OFFSET)
#define rLOAD         REG(IMXRT_TMR_LOAD_OFFSET)
#define rHOLD         REG(IMXRT_TMR_HOLD_OFFSET)
#define rCNTR         REG(IMXRT_TMR_CNTR_OFFSET)
#define rCTRL         REG(IMXRT_TMR_CTRL_OFFSET)
#define rSCTRL        REG(IMXRT_TMR_SCTRL_OFFSET)
#define rCMPLD1       REG(IMXRT_TMR_CMPLD1_OFFSET)
#define rCMPLD2       REG(IMXRT_TMR_CMPLD2_OFFSET)
#define rCSCTRL       REG(IMXRT_TMR_CSCTRL_OFFSET)
#define rFILT         REG(IMXRT_TMR_FILT_OFFSET)
#define rDMA          REG(IMXRT_TMR_DMA_OFFSET)
#define rENBL         REG(IMXRT_TMR_ENBL_OFFSET)

constexpr io_timers_t io_timers[MAX_IO_TIMERS] = {
	initIOPWM(PWM::FlexPWM2),
	initIOPWM(PWM::FlexPWM3),
	initIOPWM(PWM::FlexPWM4),
};

constexpr timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
	initIOTimerChannel(io_timers, {PWM::PWM2_PWM_A, PWM::Submodule0}, IOMUX::Pad::GPIO_B0_06),
	initIOTimerChannel(io_timers, {PWM::PWM2_PWM_A, PWM::Submodule1}, IOMUX::Pad::GPIO_EMC_08),
	initIOTimerChannel(io_timers, {PWM::PWM2_PWM_A, PWM::Submodule2}, IOMUX::Pad::GPIO_EMC_10),
	initIOTimerChannel(io_timers, {PWM::PWM2_PWM_A, PWM::Submodule3}, IOMUX::Pad::GPIO_AD_B0_09),
	initIOTimerChannel(io_timers, {PWM::PWM3_PWM_A, PWM::Submodule2}, IOMUX::Pad::GPIO_EMC_33),
	initIOTimerChannel(io_timers, {PWM::PWM3_PWM_B, PWM::Submodule0}, IOMUX::Pad::GPIO_EMC_30),
	initIOTimerChannel(io_timers, {PWM::PWM4_PWM_A, PWM::Submodule2}, IOMUX::Pad::GPIO_EMC_04),
	initIOTimerChannel(io_timers, {PWM::PWM4_PWM_B, PWM::Submodule0}, IOMUX::Pad::GPIO_EMC_01),
};

constexpr io_timers_channel_mapping_t io_timers_channel_mapping =
	initIOTimerChannelMapping(io_timers, timer_io_channels);

constexpr io_timers_t led_pwm_timers[MAX_LED_TIMERS] = {
};

constexpr timer_io_channels_t led_pwm_channels[MAX_TIMER_LED_CHANNELS] = {
};


void fmurt1062_timer_initialize(void)
{
	/* We must configure Qtimer 3 as the IPG divide by to yield 16 Mhz
	 * and deliver that clock to the eFlexPWM234 via XBAR
	 *
	 * IPG    = 144 Mhz
	 * 16Mhz  = 144 / 9
	 * COMP 1 = 5, COMP2 = 4
	 *
	 * */

	/* Enable Block Clocks for Qtimer and XBAR1 */

	imxrt_clockall_timer3();
	imxrt_clockall_xbar1();

	/* Disable Timer */

	rCTRL = 0;
	rCOMP1 = 5 - 1; // N - 1
	rCOMP2 = 4 - 1;

	rCAPT = 0;
	rLOAD = 0;
	rCNTR = 0;

	rSCTRL = TMR_SCTRL_OEN;

	rCMPLD1 = 0;
	rCMPLD2 = 0;
	rCSCTRL = 0;
	rFILT   = 0;
	rDMA    = 0;

	/* Count rising edges of primary source,
	 * Prescaler is /1
	 * Count UP until compare, then re-initialize. a successful compare occurs when the counter reaches a COMP1 value.
	 * Toggle OFLAG output using alternating compare registers
	 */
	rCTRL   = (TMR_CTRL_CM_MODE1 | TMR_CTRL_PCS_DIV1 | TMR_CTRL_LENGTH | TMR_CTRL_OUTMODE_TOG_ALT);

	/* QTIMER3_TIMER0  -> Flexpwm234ExtClk  */

	imxrt_xbar_connect(IMXRT_XBARA1_OUT_FLEXPWM234_EXT_CLK_SEL_OFFSET, IMXRT_XBARA1_IN_QTIMER3_TMR0_OUT);
}
