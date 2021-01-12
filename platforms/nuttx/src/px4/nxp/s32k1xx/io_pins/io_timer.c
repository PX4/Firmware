/****************************************************************************
 *
 *   Copyright (C) 2017 PX4 Development Team. All rights reserved.
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
 * @file io_timer.c
 *
 * Servo driver supporting PWM servos connected to S32K1XX FlexTimer blocks.
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

#include "s32k1xx_pin.h"
#include "hardware/s32k1xx_pcc.h"
#include "hardware/s32k1xx_ftm.h"


static int io_timer_handler0(int irq, void *context, void *arg);
static int io_timer_handler1(int irq, void *context, void *arg);
static int io_timer_handler2(int irq, void *context, void *arg);
static int io_timer_handler3(int irq, void *context, void *arg);
static int io_timer_handler4(int irq, void *context, void *arg);
static int io_timer_handler5(int irq, void *context, void *arg);
static int io_timer_handler6(int irq, void *context, void *arg);
static int io_timer_handler7(int irq, void *context, void *arg);

/* The FTM pre-scalers are limited to divide by 2^n where n={1-7}.
 *
 * All FTM blocks have their clock sources set to the system oscillator
 * which should generate an 8 MHz clock.
 */
#if !defined(BOARD_PWM_FREQ)
#define BOARD_PWM_FREQ 1000000
#endif

#if !defined(BOARD_ONESHOT_FREQ)
#define BOARD_ONESHOT_FREQ 8000000
#endif

#define FTM_SRC_CLOCK_FREQ 8000000

#define MAX_CHANNELS_PER_TIMER 8

#define _REG(_addr)		(*(volatile uint32_t *)(_addr))
#define _REG32(_base, _reg)	(*(volatile uint32_t *)(_base + _reg))
#define REG(_tmr, _reg)		_REG32(io_timers[_tmr].base, _reg)


/* Timer register accessors */

#define rSC(_tmr)         REG(_tmr, S32K1XX_FTM_SC_OFFSET)
#define rCNT(_tmr)        REG(_tmr, S32K1XX_FTM_CNT_OFFSET)
#define rMOD(_tmr)        REG(_tmr, S32K1XX_FTM_MOD_OFFSET)
#define rC0SC(_tmr)       REG(_tmr, S32K1XX_FTM_C0SC_OFFSET)
#define rC0V(_tmr)        REG(_tmr, S32K1XX_FTM_C0V_OFFSET)
#define rC1SC(_tmr)       REG(_tmr, S32K1XX_FTM_C1SC_OFFSET)
#define rC1V(_tmr)        REG(_tmr, S32K1XX_FTM_C1V_OFFSET)
#define rC2SC(_tmr)       REG(_tmr, S32K1XX_FTM_C2SC_OFFSET)
#define rC2V(_tmr)        REG(_tmr, S32K1XX_FTM_C2V_OFFSET)
#define rC3SC(_tmr)       REG(_tmr, S32K1XX_FTM_C3SC_OFFSET)
#define rC3V(_tmr)        REG(_tmr, S32K1XX_FTM_C3V_OFFSET)
#define rC4SC(_tmr)       REG(_tmr, S32K1XX_FTM_C4SC_OFFSET)
#define rC4V(_tmr)        REG(_tmr, S32K1XX_FTM_C4V_OFFSET)
#define rC5SC(_tmr)       REG(_tmr, S32K1XX_FTM_C5SC_OFFSET)
#define rC5V(_tmr)        REG(_tmr, S32K1XX_FTM_C5V_OFFSET)
#define rC6SC(_tmr)       REG(_tmr, S32K1XX_FTM_C6SC_OFFSET)
#define rC6V(_tmr)        REG(_tmr, S32K1XX_FTM_C6V_OFFSET)
#define rC7SC(_tmr)       REG(_tmr, S32K1XX_FTM_C7SC_OFFSET)
#define rC7V(_tmr)        REG(_tmr, S32K1XX_FTM_C7V_OFFSET)

#define rCNTIN(_tmr)      REG(_tmr, S32K1XX_FTM_CNTIN_OFFSET)
#define rSTATUS(_tmr)     REG(_tmr, S32K1XX_FTM_STATUS_OFFSET)
#define rMODE(_tmr)       REG(_tmr, S32K1XX_FTM_MODE_OFFSET)
#define rSYNC(_tmr)       REG(_tmr, S32K1XX_FTM_SYNC_OFFSET)
#define rOUTINIT(_tmr)    REG(_tmr, S32K1XX_FTM_OUTINIT_OFFSET)
#define rOUTMASK(_tmr)    REG(_tmr, S32K1XX_FTM_OUTMASK_OFFSET)
#define rCOMBINE(_tmr)    REG(_tmr, S32K1XX_FTM_COMBINE_OFFSET)
#define rDEADTIME(_tmr)   REG(_tmr, S32K1XX_FTM_DEADTIME_OFFSET)
#define rEXTTRIG(_tmr)    REG(_tmr, S32K1XX_FTM_EXTTRIG_OFFSET)
#define rPOL(_tmr)        REG(_tmr, S32K1XX_FTM_POL_OFFSET)
#define rFMS(_tmr)        REG(_tmr, S32K1XX_FTM_FMS_OFFSET)
#define rFILTER(_tmr)     REG(_tmr, S32K1XX_FTM_FILTER_OFFSET)
#define rFLTCTRL(_tmr)    REG(_tmr, S32K1XX_FTM_FLTCTRL_OFFSET)
#define rQDCTRL(_tmr)     REG(_tmr, S32K1XX_FTM_QDCTRL_OFFSET)
#define rCONF(_tmr)       REG(_tmr, S32K1XX_FTM_CONF_OFFSET)
#define rFLTPOL(_tmr)     REG(_tmr, S32K1XX_FTM_FLTPOL_OFFSET)
#define rSYNCONF(_tmr)    REG(_tmr, S32K1XX_FTM_SYNCONF_OFFSET)
#define rINVCTRL(_tmr)    REG(_tmr, S32K1XX_FTM_INVCTRL_OFFSET)
#define rSWOCTRL(_tmr)    REG(_tmr, S32K1XX_FTM_SWOCTRL_OFFSET)
#define rPWMLOAD(_tmr)    REG(_tmr, S32K1XX_FTM_PWMLOAD_OFFSET)

#define CnSC_RESET          (FTM_CNSC_CHF | FTM_CNSC_CHIE | FTM_CNSC_MSB | FTM_CNSC_MSA | FTM_CNSC_ELSB | FTM_CNSC_ELSA | FTM_CNSC_DMA)
#define CnSC_CAPTURE_INIT   (FTM_CNSC_CHIE | FTM_CNSC_ELSB | FTM_CNSC_ELSA) // Both

#if defined(BOARD_PWM_DRIVE_ACTIVE_LOW)
#define CnSC_PWMOUT_INIT    (FTM_CNSC_MSB | FTM_CNSC_ELSA)
#else
#define CnSC_PWMOUT_INIT    (FTM_CNSC_MSB | FTM_CNSC_ELSB)
#endif

#define FTM_SYNC (FTM_SYNC_SWSYNC)

#define CnSC_PWMIN_INIT 0 // TBD

//									   NotUsed     PWMOut  PWMIn   Capture OneShot Trigger
io_timer_channel_allocation_t channel_allocations[IOTimerChanModeSize] = { UINT16_MAX, 0,      0,      0,      0,      0 };

typedef uint8_t io_timer_allocation_t; /* big enough to hold MAX_IO_TIMERS */

static io_timer_allocation_t once = 0;

typedef struct channel_stat_t {
	uint32_t 			isr_cout;
	uint32_t 			overflows;
} channel_stat_t;

static channel_stat_t io_timer_channel_stats[MAX_TIMER_IO_CHANNELS];

static struct channel_handler_entry {
	channel_handler_t callback;
	void			  *context;
} channel_handlers[MAX_TIMER_IO_CHANNELS];


static int io_timer_handler(uint16_t timer_index)
{
	/* Read the count at the time of the interrupt */

	uint16_t count = rCNT(timer_index);

	/* Read the HRT at the time of the interrupt */

	hrt_abstime now = hrt_absolute_time();

	const io_timers_t *tmr = &io_timers[timer_index];

	/* What is pending */

	uint32_t statusr = rSTATUS(timer_index);

	/* Acknowledge all that are pending */

	rSTATUS(timer_index) = 0;

	/* Iterate over the timer_io_channels table */

	uint32_t first_channel_index = io_timers_channel_mapping.element[timer_index].first_channel_index;
	uint32_t last_channel_index = first_channel_index + io_timers_channel_mapping.element[timer_index].channel_count;

	for (unsigned chan_index = first_channel_index; chan_index < last_channel_index; chan_index++) {

		uint16_t chan = 1 << chan_index;

		if (statusr & chan) {

			io_timer_channel_stats[chan_index].isr_cout++;

			/* Call the client to read the rCnV etc and clear the CHnF */

			if (channel_handlers[chan_index].callback) {
				channel_handlers[chan_index].callback(channel_handlers[chan_index].context, tmr,
								      chan_index, &timer_io_channels[chan_index],
								      now, count, _REG32(tmr->base, S32K1XX_FTM_CNV_OFFSET(chan_index)));
			}
		}

		/* Did it set again during call out? */

		if (rSTATUS(timer_index) & chan) {

			/* Error - we had a second edge before we serviced the first */

			io_timer_channel_stats[chan_index].overflows++;
		}
	}

	return 0;
}

int io_timer_handler0(int irq, void *context, void *arg)
{
	return io_timer_handler(0);
}

int io_timer_handler1(int irq, void *context, void *arg)
{
	return io_timer_handler(1);
}

int io_timer_handler2(int irq, void *context, void *arg)
{
	return io_timer_handler(2);
}

int io_timer_handler3(int irq, void *context, void *arg)
{
	return io_timer_handler(3);
}

int io_timer_handler4(int irq, void *context, void *arg)
{
	return io_timer_handler(4);
}
int io_timer_handler5(int irq, void *context, void *arg)
{
	return io_timer_handler(5);
}
int io_timer_handler6(int irq, void *context, void *arg)
{
	return io_timer_handler(6);
}
int io_timer_handler7(int irq, void *context, void *arg)
{
	return io_timer_handler(7);
}

static inline int validate_timer_index(unsigned timer)
{
	return (timer < MAX_IO_TIMERS && io_timers[timer].base != 0) ? 0 : -EINVAL;
}

static inline int is_timer_uninitalized(unsigned timer)
{
	int rv = 0;

	if (once & 1 << timer) {
		rv = -EBUSY;
	}

	return rv;
}

static inline void set_timer_initalized(unsigned timer)
{
	once |= 1 << timer;
}

static inline void set_timer_deinitalized(unsigned timer)
{
	once &= ~(1 << timer);
}

static inline int channels_timer(unsigned channel)
{
	return timer_io_channels[channel].timer_index;
}

static uint32_t get_timer_channels(unsigned timer)
{
	uint32_t channels = 0;
	static uint32_t channels_cache[MAX_IO_TIMERS] = {0};

	if (validate_timer_index(timer) < 0) {
		return channels;

	} else {
		if (channels_cache[timer] == 0) {

			/* Gather the channel bits that belong to the timer */

			uint32_t first_channel_index = io_timers_channel_mapping.element[timer].first_channel_index;
			uint32_t last_channel_index = first_channel_index + io_timers_channel_mapping.element[timer].channel_count;

			for (unsigned chan_index = first_channel_index; chan_index < last_channel_index; chan_index++) {
				channels |= 1 << chan_index;
			}

			/* cache them */

			channels_cache[timer] = channels;
		}
	}

	return channels_cache[timer];
}

static inline int is_channels_timer_uninitalized(unsigned channel)
{
	return is_timer_uninitalized(channels_timer(channel));
}

int io_timer_is_channel_free(unsigned channel)
{
	int rv = io_timer_validate_channel_index(channel);

	if (rv == 0) {
		if (0 == (channel_allocations[IOTimerChanMode_NotUsed] & (1 << channel))) {
			rv = -EBUSY;
		}
	}

	return rv;
}

int io_timer_validate_channel_index(unsigned channel)
{
	int rv = -EINVAL;

	if (channel < MAX_TIMER_IO_CHANNELS && timer_io_channels[channel].timer_channel != 0) {

		unsigned timer = timer_io_channels[channel].timer_index;

		/* test timer for validity */

		if ((io_timers[timer].base != 0) &&
		    (timer_io_channels[channel].gpio_out != 0) &&
		    (timer_io_channels[channel].gpio_in != 0)) {
			rv = 0;
		}
	}

	return rv;
}

uint32_t io_timer_channel_get_gpio_output(unsigned channel)
{
	if (io_timer_validate_channel_index(channel) != 0) {
		return 0;
	}

	return (timer_io_channels[channel].gpio_out & ~(_PIN_MODE_MASK | _PIN_OPTIONS_MASK)) | GPIO_HIGHDRIVE;
}

uint32_t io_timer_channel_get_as_pwm_input(unsigned channel)
{
	if (io_timer_validate_channel_index(channel) != 0) {
		return 0;
	}

	return timer_io_channels[channel].gpio_in;
}


int io_timer_get_mode_channels(io_timer_channel_mode_t mode)
{
	if (mode < IOTimerChanModeSize) {
		return channel_allocations[mode];
	}

	return 0;
}

int io_timer_get_channel_mode(unsigned channel)
{
	io_timer_channel_allocation_t bit = 1 << channel;

	for (int mode = IOTimerChanMode_NotUsed; mode < IOTimerChanModeSize; mode++) {
		if (bit & channel_allocations[mode]) {
			return mode;
		}
	}

	return -1;
}

static int reallocate_channel_resources(uint32_t channels, io_timer_channel_mode_t mode,
					io_timer_channel_mode_t new_mode)
{
	/* If caller mode is not based on current setting adjust it */

	if ((channels & channel_allocations[IOTimerChanMode_NotUsed]) == channels) {
		mode = IOTimerChanMode_NotUsed;
	}

	/* Remove old set of channels from original */

	channel_allocations[mode] &= ~channels;

	/* Will this change ?*/

	uint32_t before = channel_allocations[new_mode] & channels;

	/* add in the new set */

	channel_allocations[new_mode] |= channels;

	/* Indicate a mode change */

	return before ^ channels;
}

static inline int allocate_channel_resource(unsigned channel, io_timer_channel_mode_t mode)
{
	int rv = io_timer_is_channel_free(channel);

	if (rv == 0) {
		io_timer_channel_allocation_t bit = 1 << channel;
		channel_allocations[IOTimerChanMode_NotUsed] &= ~bit;
		channel_allocations[mode] |= bit;
	}

	return rv;
}


static inline int free_channel_resource(unsigned channel)
{
	int mode = io_timer_get_channel_mode(channel);

	if (mode > IOTimerChanMode_NotUsed) {
		io_timer_channel_allocation_t bit = 1 << channel;
		channel_allocations[mode] &= ~bit;
		channel_allocations[IOTimerChanMode_NotUsed] |= bit;
	}

	return mode;
}

int io_timer_free_channel(unsigned channel)
{
	if (io_timer_validate_channel_index(channel) != 0) {
		return -EINVAL;
	}

	int mode = io_timer_get_channel_mode(channel);

	if (mode > IOTimerChanMode_NotUsed) {
		io_timer_set_enable(false, mode, 1 << channel);
		free_channel_resource(channel);

	}

	return 0;
}


static int allocate_channel(unsigned channel, io_timer_channel_mode_t mode)
{
	int rv = -EINVAL;

	if (mode != IOTimerChanMode_NotUsed) {
		rv = io_timer_validate_channel_index(channel);

		if (rv == 0) {
			rv = allocate_channel_resource(channel, mode);
		}
	}

	return rv;
}

static int timer_set_rate(unsigned timer, unsigned rate)
{

	irqstate_t flags = px4_enter_critical_section();

	uint32_t save = rSC(timer);
	rSC(timer) = save & ~(FTM_SC_CLKS_MASK);

	/* configure the timer to update at the desired rate */
	rMOD(timer) = (BOARD_PWM_FREQ / rate) - 1;
	rSC(timer) = save;

	/* generate an update event; reloads the counter and all registers */
	rSYNC(timer) = FTM_SYNC;

	px4_leave_critical_section(flags);

	return 0;
}

static inline uint32_t div2psc(int div)
{
	return 31 - __builtin_clz(div);
}

static inline void io_timer_set_oneshot_mode(unsigned timer)
{
	/* Ideally, we would want per channel One pulse mode in HW
	 * Alas The FTM anly support onshot capture)
	 * todo:We can do this in an ISR later
	 * But since we do not have that
	 * We try to get the longest rate we can.
	 *  On 16 bit timers this is 4.68 Ms.
	 */

	irqstate_t flags = px4_enter_critical_section();
	rSC(timer) &= ~(FTM_SC_CLKS_MASK | FTM_SC_PS_MASK);
	rMOD(timer) = 0xffff;
	rSC(timer) |= (FTM_SC_CLKS_EXTCLK | div2psc(FTM_SRC_CLOCK_FREQ / BOARD_ONESHOT_FREQ));
	px4_leave_critical_section(flags);
}

static inline void io_timer_set_PWM_mode(unsigned timer)
{
	irqstate_t flags = px4_enter_critical_section();
	rSC(timer) &= ~(FTM_SC_CLKS_MASK | FTM_SC_PS_MASK);
	rSC(timer) |= (FTM_SC_CLKS_EXTCLK | div2psc(FTM_SRC_CLOCK_FREQ / BOARD_PWM_FREQ));
	px4_leave_critical_section(flags);
}

void io_timer_trigger(void)
{
	int oneshots = io_timer_get_mode_channels(IOTimerChanMode_OneShot);
	uint32_t action_cache[MAX_IO_TIMERS] = {0};
	int actions = 0;

	/* Pre-calculate the list of timers to Trigger */

	for (int timer = 0; timer < MAX_IO_TIMERS; timer++) {
		if (validate_timer_index(timer) == 0) {
			int channels = get_timer_channels(timer);

			if (oneshots & channels) {
				action_cache[actions++] = io_timers[timer].base;
			}
		}
	}

	/* Now do them all wit the shortest delay in between */

	irqstate_t flags = px4_enter_critical_section();

	for (actions = 0; actions < MAX_IO_TIMERS && action_cache[actions] != 0; actions++) {
		_REG32(action_cache[actions], S32K1XX_FTM_SYNC_OFFSET) |= FTM_SYNC;
	}

	px4_leave_critical_section(flags);
}

int io_timer_init_timer(unsigned timer)
{
	/* Do this only once per timer */

	int rv = is_timer_uninitalized(timer);

	if (rv == 0) {

		irqstate_t flags = px4_enter_critical_section();

		set_timer_initalized(timer);

		/* enable the timer clock before we try to talk to it */

		uint32_t regval = _REG(io_timers[timer].clock_register);
		regval |= io_timers[timer].clock_bit;
		_REG(io_timers[timer].clock_register) = regval;

		/* disable and configure the timer */

		rSC(timer)      = FTM_SC_CLKS_DIS;
		rCNT(timer)     = 0;

		rMODE(timer)    = 0;
		rSYNCONF(timer) = (FTM_SYNCONF_SYNCMODE | FTM_SYNCONF_SWWRBUF | FTM_SYNCONF_SWRSTCNT);

		/* Set to run in debug mode */

		rCONF(timer)   |= FTM_CONF_BDMMODE_MASK;

		/* enable the timer */

		io_timer_set_PWM_mode(timer);

		/*
		 * Note we do the Standard PWM Out init here
		 * default to updating at 50Hz
		 */

		timer_set_rate(timer, 50);

		/*
		 * Note that the timer is left disabled with IRQ subs installed
		 * and active but DEIR bits are not set.
		 */
		xcpt_t handler;

		switch (timer) {
		case 0: handler = io_timer_handler0; break;

		case 1: handler = io_timer_handler1; break;

		case 2: handler = io_timer_handler2; break;

		case 3: handler = io_timer_handler3; break;

		case 4: handler = io_timer_handler4; break;

		case 5: handler = io_timer_handler5; break;

		case 6: handler = io_timer_handler6; break;

		case 7: handler = io_timer_handler7; break;

		default:
			handler = NULL;
			rv = -EINVAL;
			break;
		}

		if (handler) {
			irq_attach(io_timers[timer].vectorno, handler, NULL);
			up_enable_irq(io_timers[timer].vectorno);
#if defined(CONFIG_ARCH_CHIP_S32K14X) /* Quick (and not very nice) workaround for the multiple interrupts per FTM on S32K14X */
			irq_attach(io_timers[timer].vectorno + 1, handler, NULL);
			up_enable_irq(io_timers[timer].vectorno + 1);
			irq_attach(io_timers[timer].vectorno + 2, handler, NULL);
			up_enable_irq(io_timers[timer].vectorno + 2);
			irq_attach(io_timers[timer].vectorno + 3, handler, NULL);
			up_enable_irq(io_timers[timer].vectorno + 3);
#endif
		}

		px4_leave_critical_section(flags);
	}

	return rv;
}


int io_timer_set_rate(unsigned timer, unsigned rate)
{
	int rv = EBUSY;

	/* Get the channel bits that belong to the timer */

	uint32_t channels = get_timer_channels(timer);

	/* Check that all channels are either in PWM or Oneshot */

	if ((channels & (channel_allocations[IOTimerChanMode_PWMOut] |
			 channel_allocations[IOTimerChanMode_OneShot] |
			 channel_allocations[IOTimerChanMode_NotUsed])) ==
	    channels) {

		/* Change only a timer that is owned by pwm or one shot */

		/* Request to use OneShot ?*/

		if (rate == 0) {

			/* Request to use OneShot
			 *
			 * We are here because ALL these channels were either PWM or Oneshot
			 * Now they need to be Oneshot
			 */

			/* Did the allocation change */
			if (reallocate_channel_resources(channels, IOTimerChanMode_PWMOut, IOTimerChanMode_OneShot)) {
				io_timer_set_oneshot_mode(timer);
			}

		} else {

			/* Request to use PWM
			 *
			 * We are here because  ALL these channels were either PWM or Oneshot
			 * Now they need to be PWM
			 */

			if (reallocate_channel_resources(channels, IOTimerChanMode_OneShot, IOTimerChanMode_PWMOut)) {
				io_timer_set_PWM_mode(timer);
			}

			timer_set_rate(timer, rate);
		}

		rv = OK;
	}

	return rv;
}

int io_timer_channel_init(unsigned channel, io_timer_channel_mode_t mode,
			  channel_handler_t channel_handler, void *context)
{
	uint32_t gpio = timer_io_channels[channel].gpio_in;
	uint32_t clearbits = CnSC_RESET;
	uint32_t setbits = CnSC_CAPTURE_INIT;

	/* figure out the GPIO config first */

	switch (mode) {

	case IOTimerChanMode_OneShot:
	case IOTimerChanMode_PWMOut:
	case IOTimerChanMode_Trigger:
		setbits = CnSC_PWMOUT_INIT;
		gpio = 0;
		break;

	case IOTimerChanMode_PWMIn:
	case IOTimerChanMode_Capture:
		break;

	case IOTimerChanMode_NotUsed:
		setbits = 0;
		break;

	default:
		return -EINVAL;
	}

	int rv = allocate_channel(channel, mode);

	/* Valid channel should now be reserved in new mode */

	if (rv >= 0) {

		/* Blindly try to initialize the timer - it will only do it once */

		io_timer_init_timer(channels_timer(channel));

		irqstate_t flags = px4_enter_critical_section();

		/* Set up IO */
		if (gpio) {
			px4_arch_configgpio(gpio);
		}


		unsigned timer = channels_timer(channel);

		/* configure the channel */

		uint32_t chan = timer_io_channels[channel].timer_channel - 1;

		uint16_t rvalue = REG(timer, S32K1XX_FTM_CNSC_OFFSET(chan));
		rvalue &= ~clearbits;
		rvalue |=  setbits;
		REG(timer, S32K1XX_FTM_CNSC_OFFSET(chan)) = rvalue;

		//if (gpio){ TODO: NEEDED?
		REG(timer, S32K1XX_FTM_CNV_OFFSET(0)) = 0;

		channel_handlers[channel].callback = channel_handler;
		channel_handlers[channel].context = context;
		px4_leave_critical_section(flags);
	}

	return rv;
}

int io_timer_set_enable(bool state, io_timer_channel_mode_t mode, io_timer_channel_allocation_t masks)
{
	typedef struct action_cache_rp_t {
		uint32_t cnsc_offset;
		uint32_t cnsc_value;
		uint32_t gpio;
	} action_cache_rp_t;
	struct action_cache_t {
		uint32_t base;
		uint32_t index;
		uint32_t mask;
		action_cache_rp_t cnsc[MAX_CHANNELS_PER_TIMER];
	} action_cache[MAX_IO_TIMERS];

	memset(action_cache, 0, sizeof(action_cache));

	uint32_t bits =  state ? CnSC_PWMOUT_INIT : 0;

	switch (mode) {
	case IOTimerChanMode_NotUsed:
	case IOTimerChanMode_PWMOut:
	case IOTimerChanMode_OneShot:
	case IOTimerChanMode_Trigger:
		break;

	case IOTimerChanMode_PWMIn:
	case IOTimerChanMode_Capture:
		if (state) {
			bits |= FTM_CNSC_CHIE;
		}

		break;

	default:
		return -EINVAL;
	}

	/* Was the request for all channels in this mode ?*/

	if (masks == IO_TIMER_ALL_MODES_CHANNELS) {

		/* Yes - we provide them */

		masks = channel_allocations[mode];

	} else {

		/* No - caller provided mask */

		/* Only allow the channels in that mode to be affected */

		masks &= channel_allocations[mode];

	}

	/* Pre calculate all the changes */

	for (int chan_index = 0; masks != 0 && chan_index < MAX_TIMER_IO_CHANNELS; chan_index++) {
		if (masks & (1 << chan_index)) {
			masks &= ~(1 << chan_index);

			if (io_timer_validate_channel_index(chan_index) == 0) {
				uint32_t chan = timer_io_channels[chan_index].timer_channel - 1;
				uint32_t timer = channels_timer(chan_index);
				action_cache[timer].base  = io_timers[timer].base;
				action_cache[timer].cnsc[action_cache[timer].index].cnsc_offset = io_timers[timer].base + S32K1XX_FTM_CNSC_OFFSET(chan);
				action_cache[timer].cnsc[action_cache[timer].index].cnsc_value = bits;

				if ((state &&
				     (mode == IOTimerChanMode_PWMOut ||
				      mode == IOTimerChanMode_OneShot ||
				      mode == IOTimerChanMode_Trigger))) {
					action_cache[timer].cnsc[action_cache[timer].index].gpio = timer_io_channels[chan_index].gpio_out;
					action_cache[timer].mask |= 1 << chan;
				}

				action_cache[timer].index++;
			}
		}
	}

	irqstate_t flags = px4_enter_critical_section();


	for (unsigned actions = 0; actions < arraySize(action_cache); actions++) {
		uint32_t any_on = 0;

		if (action_cache[actions].base != 0) {
			for (unsigned int index = 0; index < action_cache[actions].index; index++) {

				if (action_cache[actions].cnsc[index].gpio) {
					px4_arch_configgpio(action_cache[actions].cnsc[index].gpio);
				}

				_REG(action_cache[actions].cnsc[index].cnsc_offset) = action_cache[actions].cnsc[index].cnsc_value;
				any_on |= action_cache[actions].cnsc[index].cnsc_value;
			}

			/* Any On ?*/

			/* Assume not */
			uint32_t regval = _REG32(action_cache[actions].base, S32K1XX_FTM_SC_OFFSET);
			regval &= ~(FTM_SC_CLKS_MASK);

			if (any_on != 0) {

				/* force an update to preload all registers */
				_REG32(action_cache[actions].base, S32K1XX_FTM_SYNC_OFFSET) |= FTM_SYNC;

				/* arm requires the timer be enabled */
				regval |= (FTM_SC_CLKS_EXTCLK);

				regval |= action_cache[actions].mask << FTM_SC_PWMEN_SHIFT;

			}

			_REG32(action_cache[actions].base, S32K1XX_FTM_SC_OFFSET) = regval;
		}
	}

	px4_leave_critical_section(flags);

	return 0;
}

int io_timer_set_ccr(unsigned channel, uint16_t value)
{
	int rv = io_timer_validate_channel_index(channel);
	int mode = io_timer_get_channel_mode(channel);

	if (rv == 0) {
		if ((mode != IOTimerChanMode_PWMOut) &&
		    (mode != IOTimerChanMode_OneShot) &&
		    (mode != IOTimerChanMode_Trigger)) {

			rv = -EIO;

		} else {

			/* configure the channel */

			/* todo:This is a HACK!
			 * Was getting:
			 * servo 0 readback error, got 1001 expected 1000
			 * None of the SW syncs seamed update the CV
			 * So we stop the counter to get an immediate update.
			 */
			uint32_t timer = channels_timer(channel);
			irqstate_t flags = px4_enter_critical_section();
			uint32_t save = rSC(timer);
			rSC(timer) = save & ~(FTM_SC_CLKS_MASK);
			REG(timer, S32K1XX_FTM_CNV_OFFSET(timer_io_channels[channel].timer_channel - 1)) = value;
			rSC(timer) = save;
			px4_leave_critical_section(flags);
		}
	}

	return rv;
}

uint16_t io_channel_get_ccr(unsigned channel)
{
	uint16_t value = 0;

	if (io_timer_validate_channel_index(channel) == 0) {
		int mode = io_timer_get_channel_mode(channel);

		if ((mode == IOTimerChanMode_PWMOut) ||
		    (mode == IOTimerChanMode_OneShot) ||
		    (mode == IOTimerChanMode_Trigger)) {
			value = REG(channels_timer(channel), S32K1XX_FTM_CNV_OFFSET(timer_io_channels[channel].timer_channel - 1));
		}
	}

	return value;
}

uint32_t io_timer_get_group(unsigned timer)
{
	return get_timer_channels(timer);

}
