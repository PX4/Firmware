/****************************************************************************
 *
 *   Copyright (C) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file ToneAlarmInterface.cpp
 */

#include <drivers/device/device.h>
#include <drivers/drv_tone_alarm.h>
#include <px4_platform_common/defines.h>
#include <systemlib/px4_macros.h>

#include "chip.h"
#include "hardware/imxrt_gpt.h"
#include "imxrt_periphclks.h"

#define CAT3_(A, B, C)    A##B##C
#define CAT3(A, B, C)     CAT3_(A, B, C)

#define CAT2_(A, B)    A##B
#define CAT2(A, B)     CAT2_(A, B)

/* Check that tone alarm and HRT timers are different */
#if defined(TONE_ALARM_TIMER)  && defined(HRT_TIMER)
# if TONE_ALARM_TIMER == HRT_TIMER
#   error TONE_ALARM_TIMER and HRT_TIMER must use different timers.
# endif
#endif

/*
* Period of the free-running counter, in microseconds.
*/
#define TONE_ALARM_COUNTER_PERIOD  4294967296

/* Tone Alarm configuration */

#define TONE_ALARM_TIMER_CLOCK  BOARD_GPT_FREQUENCY                       /* The input clock frequency to the GPT block */
#define TONE_ALARM_TIMER_BASE   CAT3(IMXRT_GPT, TONE_ALARM_TIMER,_BASE)   /* The Base address of the GPT */
#define TONE_ALARM_TIMER_VECTOR CAT(IMXRT_IRQ_GPT, TONE_ALARM_TIMER)      /* The GPT Interrupt vector */

#if TONE_ALARM_TIMER == 1
#  define TONE_ALARM_CLOCK_ALL()  imxrt_clockall_gpt_bus()         /* The Clock Gating macro for this GPT */
#elif TONE_ALARM_TIMER == 2
#  define TONE_ALARM_CLOCK_ALL()  imxrt_clockall_gpt2_bus()        /* The Clock Gating macro for this GPT */
#endif

#if TONE_ALARM_TIMER == 1 && defined(CONFIG_IMXRT_GPT1)
#  error must not set CONFIG_IMXRT_GPT1=y and TONE_ALARM_TIMER=1
#elif   TONE_ALARM_TIMER == 2 && defined(CONFIG_IMXRT_GPT2)
#  error must not set CONFIG_IMXRT_GPT2=y and TONE_ALARM_TIMER=2
#endif


# define TONE_ALARM_TIMER_FREQ    1000000

/*
* Tone Alarm clock must be a multiple of 1MHz greater than 1MHz
*/
#if (TONE_ALARM_TIMER_CLOCK % TONE_ALARM_TIMER_FREQ) != 0
#  error TONE_ALARM_TIMER_CLOCK must be a multiple of 1MHz
#endif
#if TONE_ALARM_TIMER_CLOCK <= TONE_ALARM_TIMER_FREQ
# error TONE_ALARM_TIMER_CLOCK must be greater than 1MHz
#endif

#if (TONE_ALARM_TIMER_CHANNEL > 1) || (TONE_ALARM_TIMER_CHANNEL > 3)
#  error TONE_ALARM_CHANNEL must be a value between 1 and 3
#endif


/* Register accessors */

#define _REG(_addr) (*(volatile uint32_t *)(_addr))

/* Timer register accessors */

#define REG(_reg) _REG(TONE_ALARM_TIMER_BASE + (_reg))

#define rCR         REG(IMXRT_GPT_CR_OFFSET)
#define rPR         REG(IMXRT_GPT_PR_OFFSET)
#define rSR         REG(IMXRT_GPT_SR_OFFSET)
#define rIR         REG(IMXRT_GPT_IR_OFFSET)
#define rOCR1       REG(IMXRT_GPT_OCR1_OFFSET)
#define rOCR2       REG(IMXRT_GPT_OCR2_OFFSET)
#define rOCR3       REG(IMXRT_GPT_OCR3_OFFSET)
#define rICR1       REG(IMXRT_GPT_ICR1_OFFSET)
#define rICR2       REG(IMXRT_GPT_ICR2_OFFSET)
#define rCNT        REG(IMXRT_GPT_CNT_OFFSET)

/*
* Specific registers and bits used by Tone Alarm sub-functions
*/

#define rOCR         CAT2(rOCR, TONE_ALARM_CHANNEL)               /* GPT Output Compare Register used by HRT */
#define rSTATUS      CAT2(GPT_SR_OF, TONE_ALARM_CHANNEL)          /* OF Output Compare Flag */
#define CR_OM        CAT3(GPT_CR_OM, TONE_ALARM_CHANNEL,_TOGGLE)  /* Output Compare mode */


#define CBRK_BUZZER_KEY 782097

namespace ToneAlarmInterface
{

void init()
{
#if defined(TONE_ALARM_TIMER)
	/* configure the GPIO to the idle state */
	px4_arch_configgpio(GPIO_TONE_ALARM_IDLE);

	/* Enable the Module clock */

	TONE_ALARM_CLOCK_ALL();

	rCR = 0;


	/* disable and configure the timer */

	/* Use Restart mode. The FFR bit determines the behavior of the GPT
	 * when a compare event in _channel 1_ occurs.
	 */
	rCR =  GPT_CR_ENMOD | GPT_CR_DBGEN | GPT_CR_WAITEN |
	       GPT_CR_IM2_DIS | GPT_CR_IM1_DIS |
	       CR_OM | GPT_CR_CLKSRC_IPG;

	/* CLKSRC field is divided by [PRESCALER + 1] */

	rPR = (TONE_ALARM_TIMER_CLOCK / TONE_ALARM_TIMER_FREQ) - 1;

	/* enable the timer and output toggle */

	rCR |=  GPT_CR_EN;
#endif /* TONE_ALARM_TIMER */
}

void start_note(unsigned frequency)
{
#if defined(TONE_ALARM_TIMER)
	float period = 0.5f / frequency;

	// and the divisor, rounded to the nearest integer
	unsigned divisor = (period * TONE_ALARM_TIMER_FREQ) + 0.5f;

	rCR  &= ~GPT_CR_EN;
	rOCR  = divisor;   // load new toggle period

	/* We must always load and run Channel 1 in parallel
	 * with the Timer out channel used to drive the tone.
	 * The will to initiate the reset in Restart mode.
	 */
	rOCR1 = divisor;
	rCR  |= GPT_CR_EN;

	// configure the GPIO to enable timer output
	px4_arch_configgpio(GPIO_TONE_ALARM);
#endif /* TONE_ALARM_TIMER */
}

void stop_note()
{
#if defined(TONE_ALARM_TIMER)
	/* stop the current note */

	rCR &= ~GPT_CR_EN;

	/*
	 * Make sure the GPIO is not driving the speaker.
	 */
	px4_arch_configgpio(GPIO_TONE_ALARM_IDLE);
#endif /* TONE_ALARM_TIMER */
}

} /* namespace ToneAlarmInterface */
