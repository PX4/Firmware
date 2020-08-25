/****************************************************************************
 *
 *   Copyright (C) 2012, 2017 PX4 Development Team. All rights reserved.
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
 * Servo driver supporting PWM servos connected to STM32 timer blocks.
 */

#include <px4_platform_common/px4_config.h>
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
#include <px4_arch/dshot.h>

#include <stm32_gpio.h>
#include <stm32_tim.h>


static int io_timer_handler0(int irq, void *context, void *arg);
static int io_timer_handler1(int irq, void *context, void *arg);
static int io_timer_handler2(int irq, void *context, void *arg);
static int io_timer_handler3(int irq, void *context, void *arg);
static int io_timer_handler4(int irq, void *context, void *arg);
static int io_timer_handler5(int irq, void *context, void *arg);
static int io_timer_handler6(int irq, void *context, void *arg);
static int io_timer_handler7(int irq, void *context, void *arg);

#if defined(HAVE_GTIM_CCXNP)
#define HW_GTIM_CCER_CC1NP GTIM_CCER_CC1NP
#else
#  define HW_GTIM_CCER_CC1NP    0
#endif

#define arraySize(a) (sizeof((a))/sizeof(((a)[0])))

/* If the timer clock source provided as clock_freq is the STM32_APBx_TIMx_CLKIN
 * then configure the timer to free-run at 1MHz.
 * Otherwise, other frequencies are attainable by adjusting .clock_freq accordingly.
 * For instance .clock_freq = 1000000 would set the prescaler to 1.
 * We also allow for overrides here but all timer register usage need to be
 * taken into account
 */
#if !defined(BOARD_PWM_FREQ)
#define BOARD_PWM_FREQ 1000000
#endif

#if !defined(BOARD_ONESHOT_FREQ)
#define BOARD_ONESHOT_FREQ 8000000
#endif

#define MAX_CHANNELS_PER_TIMER 4

#define _REG32(_base, _reg)	(*(volatile uint32_t *)(_base + _reg))
#define REG(_tmr, _reg)		_REG32(io_timers[_tmr].base, _reg)

#define rCR1(_tmr)    	REG(_tmr, STM32_GTIM_CR1_OFFSET)
#define rCR2(_tmr)    	REG(_tmr, STM32_GTIM_CR2_OFFSET)
#define rSMCR(_tmr)   	REG(_tmr, STM32_GTIM_SMCR_OFFSET)
#define rDIER(_tmr)   	REG(_tmr, STM32_GTIM_DIER_OFFSET)
#define rSR(_tmr)     	REG(_tmr, STM32_GTIM_SR_OFFSET)
#define rEGR(_tmr)    	REG(_tmr, STM32_GTIM_EGR_OFFSET)
#define rCCMR1(_tmr)  	REG(_tmr, STM32_GTIM_CCMR1_OFFSET)
#define rCCMR2(_tmr)  	REG(_tmr, STM32_GTIM_CCMR2_OFFSET)
#define rCCER(_tmr)   	REG(_tmr, STM32_GTIM_CCER_OFFSET)
#define rCNT(_tmr)    	REG(_tmr, STM32_GTIM_CNT_OFFSET)
#define rPSC(_tmr)    	REG(_tmr, STM32_GTIM_PSC_OFFSET)
#define rARR(_tmr)    	REG(_tmr, STM32_GTIM_ARR_OFFSET)
#define rCCR1(_tmr)   	REG(_tmr, STM32_GTIM_CCR1_OFFSET)
#define rCCR2(_tmr)   	REG(_tmr, STM32_GTIM_CCR2_OFFSET)
#define rCCR3(_tmr)   	REG(_tmr, STM32_GTIM_CCR3_OFFSET)
#define rCCR4(_tmr)   	REG(_tmr, STM32_GTIM_CCR4_OFFSET)
#define rDCR(_tmr)    	REG(_tmr, STM32_GTIM_DCR_OFFSET)
#define rDMAR(_tmr)   	REG(_tmr, STM32_GTIM_DMAR_OFFSET)
#define rBDTR(_tmr)		REG(_tmr, STM32_ATIM_BDTR_OFFSET)

#define GTIM_SR_CCIF (GTIM_SR_CC4IF|GTIM_SR_CC3IF|GTIM_SR_CC2IF|GTIM_SR_CC1IF)
#define GTIM_SR_CCOF (GTIM_SR_CC4OF|GTIM_SR_CC3OF|GTIM_SR_CC2OF|GTIM_SR_CC1OF)

#define CCMR_C1_RESET 			0x00ff
#define CCMR_C1_NUM_BITS 		8
#define CCER_C1_NUM_BITS 		4

#define CCMR_C1_CAPTURE_INIT (GTIM_CCMR_CCS_CCIN1  << GTIM_CCMR1_CC1S_SHIFT) | \
	(GTIM_CCMR_ICPSC_NOPSC << GTIM_CCMR1_IC1PSC_SHIFT) | \
	(GTIM_CCMR_ICF_NOFILT << GTIM_CCMR1_IC1F_SHIFT)

#define CCMR_C1_PWMOUT_INIT (GTIM_CCMR_MODE_PWM1 << GTIM_CCMR1_OC1M_SHIFT) | GTIM_CCMR1_OC1PE

#define CCMR_C1_PWMIN_INIT 0 // TBD

#if defined(BOARD_PWM_DRIVE_ACTIVE_LOW)
#define CCER_C1_INIT  (GTIM_CCER_CC1P | GTIM_CCER_CC1E)
#else
#define CCER_C1_INIT  GTIM_CCER_CC1E
#endif

/* The transfer is done to 1 register starting from TIMx_CR1 + TIMx_DCR.DBA   */
#define TIM_DMABURSTLENGTH_1TRANSFER	0x00000000U
/* The transfer is done to 2 registers starting from TIMx_CR1 + TIMx_DCR.DBA  */
#define TIM_DMABURSTLENGTH_2TRANSFERS	0x00000100U
/* The transfer is done to 3 registers starting from TIMx_CR1 + TIMx_DCR.DBA  */
#define TIM_DMABURSTLENGTH_3TRANSFERS	0x00000200U
/* The transfer is done to 4 registers starting from TIMx_CR1 + TIMx_DCR.DBA  */
#define TIM_DMABURSTLENGTH_4TRANSFERS	0x00000300U

//												 				  NotUsed   PWMOut  PWMIn Capture OneShot Trigger Dshot
io_timer_channel_allocation_t channel_allocations[IOTimerChanModeSize] = { UINT16_MAX,   0,  0,  0, 0, 0, 0 };

typedef uint8_t io_timer_allocation_t; /* big enough to hold MAX_IO_TIMERS */

static io_timer_allocation_t once = 0;

#if defined(BOARD_HAS_CAPTURE)

/* Stats and handlers are only useful for Capture */

typedef struct channel_stat_t {
	uint32_t 			isr_cout;
	uint32_t 			overflows;
} channel_stat_t;

static channel_stat_t io_timer_channel_stats[MAX_TIMER_IO_CHANNELS];

static struct channel_handler_entry {
	channel_handler_t callback;
	void			  *context;
} channel_handlers[MAX_TIMER_IO_CHANNELS];
#endif // defined(BOARD_HAS_CAPTURE)


static int io_timer_handler(uint16_t timer_index)
{
#if defined(BOARD_HAS_CAPTURE)
	/* Read the count at the time of the interrupt */

	uint16_t count = rCNT(timer_index);

	/* Read the HRT at the time of the interrupt */

	hrt_abstime now = hrt_absolute_time();

	const io_timers_t *tmr = &io_timers[timer_index];

	/* What is pending and enabled? */

	uint16_t statusr = rSR(timer_index);
	uint16_t enabled = rDIER(timer_index) & GTIM_SR_CCIF;

	/* Iterate over the timer_io_channels table */

	uint32_t first_channel_index = io_timers_channel_mapping.element[timer_index].first_channel_index;
	uint32_t last_channel_index = first_channel_index + io_timers_channel_mapping.element[timer_index].channel_count;

	for (unsigned chan_index = first_channel_index; chan_index < last_channel_index; chan_index++) {

		uint16_t masks = timer_io_channels[chan_index].masks;

		/* Do we have an enabled channel */

		if (enabled & masks) {


			if (statusr & masks & GTIM_SR_CCIF) {

				io_timer_channel_stats[chan_index].isr_cout++;

				/* Call the client to read the CCxR etc and clear the CCxIF */

				if (channel_handlers[chan_index].callback) {
					channel_handlers[chan_index].callback(channel_handlers[chan_index].context, tmr,
									      chan_index, &timer_io_channels[chan_index],
									      now, count);
				}
			}

			if (statusr & masks & GTIM_SR_CCOF) {

				/* Error we has a second edge before we cleared CCxR */

				io_timer_channel_stats[chan_index].overflows++;
			}
		}
	}

	/* Clear all the SR bits for interrupt enabled channels only */

	rSR(timer_index) = ~(statusr & (enabled | enabled << 8));
#endif // defined(BOARD_HAS_CAPTURE)
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

#ifdef CONFIG_STM32_STM32F10XX
	return (timer_io_channels[channel].gpio_out & (GPIO_PORT_MASK | GPIO_PIN_MASK)) |
	       (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz | GPIO_OUTPUT_CLEAR);
#else
	return (timer_io_channels[channel].gpio_out & (GPIO_PORT_MASK | GPIO_PIN_MASK)) |
	       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR);
#endif
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
	/* configure the timer to update at the desired rate */

	rARR(timer) = (BOARD_PWM_FREQ / rate) - 1;

	return 0;
}



static inline void io_timer_set_oneshot_mode(unsigned timer)
{
	/* Ideally, we would want per channel One pulse mode in HW
	 * Alas OPE stops the Timer not the channel
	 * todo:We can do this in an ISR later
	 * But since we do not have that
	 * We try to get the longest rate we can.
	 *  On 16 bit timers this is 8.1 Ms.
	 *  On 32 but timers this is 536870.912
	 */

	rARR(timer) = 0xffffffff;
	rPSC(timer) = (io_timers[timer].clock_freq / BOARD_ONESHOT_FREQ) - 1;
	rEGR(timer) = GTIM_EGR_UG;
}

void io_timer_update_dma_req(uint8_t timer, bool enable)
{
	if (enable) {
		rDIER(timer) |= ATIM_DIER_UDE;

	} else {
		rDIER(timer) &= ~ATIM_DIER_UDE;
	}
}

int io_timer_set_dshot_mode(uint8_t timer, unsigned dshot_pwm_freq, uint8_t dma_burst_length)
{
	int ret_val = OK;
	uint32_t tim_dma_burst_length;

	if (1u == dma_burst_length) {
		tim_dma_burst_length = TIM_DMABURSTLENGTH_1TRANSFER;

	} else if (2u == dma_burst_length) {
		tim_dma_burst_length = TIM_DMABURSTLENGTH_2TRANSFERS;

	} else if (3u == dma_burst_length) {
		tim_dma_burst_length = TIM_DMABURSTLENGTH_3TRANSFERS;

	} else if (4u == dma_burst_length) {
		tim_dma_burst_length = TIM_DMABURSTLENGTH_4TRANSFERS;

	} else {
		ret_val = ERROR;
	}

	if (OK == ret_val) {
		rARR(timer)  = DSHOT_MOTOR_PWM_BIT_WIDTH;
		rPSC(timer)  = ((int)(io_timers[timer].clock_freq / dshot_pwm_freq) / DSHOT_MOTOR_PWM_BIT_WIDTH) - 1;
		rEGR(timer)  = ATIM_EGR_UG;

		// find the lowest channel index for the timer (they are not necesarily in ascending order)
		unsigned lowest_timer_channel = 4;
		uint32_t first_channel_index = io_timers_channel_mapping.element[timer].first_channel_index;
		uint32_t last_channel_index = first_channel_index + io_timers_channel_mapping.element[timer].channel_count;

		for (unsigned chan_index = first_channel_index; chan_index < last_channel_index; chan_index++) {
			if (timer_io_channels[chan_index].timer_channel < lowest_timer_channel) {
				lowest_timer_channel = timer_io_channels[chan_index].timer_channel;
			}
		}

		uint32_t start_ccr_register = 0;

		switch (lowest_timer_channel) {
		case 1: start_ccr_register = TIM_DMABASE_CCR1; break;

		case 2: start_ccr_register = TIM_DMABASE_CCR2; break;

		case 3: start_ccr_register = TIM_DMABASE_CCR3; break;

		case 4: start_ccr_register = TIM_DMABASE_CCR4; break;
		}

		rDCR(timer)  = start_ccr_register | tim_dma_burst_length;
	}

	return ret_val;
}

static inline void io_timer_set_PWM_mode(unsigned timer)
{
	rPSC(timer) = (io_timers[timer].clock_freq / BOARD_PWM_FREQ) - 1;
}

void io_timer_trigger(void)
{
	int oneshots = io_timer_get_mode_channels(IOTimerChanMode_OneShot);

	if (oneshots != 0) {
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

		/* Now do them all with the shortest delay in between */

		irqstate_t flags = px4_enter_critical_section();

		for (actions = 0; actions < MAX_IO_TIMERS && action_cache[actions] != 0; actions++) {
			_REG32(action_cache[actions], STM32_GTIM_EGR_OFFSET) |= GTIM_EGR_UG;
		}

		px4_leave_critical_section(flags);
	}
}

int io_timer_init_timer(unsigned timer)
{
	/* Do this only once per timer */

	int rv = is_timer_uninitalized(timer);

	if (rv == 0) {

		irqstate_t flags = px4_enter_critical_section();

		set_timer_initalized(timer);

		/* enable the timer clock before we try to talk to it */

		modifyreg32(io_timers[timer].clock_register, 0, io_timers[timer].clock_bit);

		/* disable and configure the timer */
		rCR1(timer) = 0;
		rCR2(timer) = 0;
		rSMCR(timer) = 0;
		rDIER(timer) = 0;
		rCCER(timer) = 0;
		rCCMR1(timer) = 0;
		rCCMR2(timer) = 0;
		rCCR1(timer) = 0;
		rCCR2(timer) = 0;
		rCCR3(timer) = 0;
		rCCR4(timer) = 0;
		rCCER(timer) = 0;
		rDCR(timer) = 0;

		if ((io_timers[timer].base == STM32_TIM1_BASE) || (io_timers[timer].base == STM32_TIM8_BASE)) {

			/* master output enable = on */

			rBDTR(timer) = ATIM_BDTR_MOE;
		}

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

	/* Check that all channels are either in PWM, Oneshot or Dshot*/

	if ((channels & (channel_allocations[IOTimerChanMode_Dshot] |
			 channel_allocations[IOTimerChanMode_PWMOut] |
			 channel_allocations[IOTimerChanMode_OneShot] |
			 channel_allocations[IOTimerChanMode_NotUsed])) ==
	    channels) {

		/* Change only a timer that is owned by pwm or one shot */

		/* Request to use OneShot ?*/

		if (PWM_RATE_ONESHOT == rate) {

			/* Request to use OneShot
			 *
			 * We are here because ALL these channels were either PWM or Dshot
			 * Now they need to be Oneshot
			 */

			int changePWMOut = reallocate_channel_resources(channels, IOTimerChanMode_PWMOut, IOTimerChanMode_OneShot);
			int changeDshot = reallocate_channel_resources(channels, IOTimerChanMode_Dshot, IOTimerChanMode_OneShot);
			int changedChannels = changePWMOut | changeDshot;

			/* Did the allocation change */
			if (changedChannels) {
				io_timer_set_oneshot_mode(timer);
			}

		} else {

			/* Request to use PWM
			 *
			 * We are here because  ALL these channels were either Oneshot or Dshot
			 * Now they need to be PWM
			 */

			int changeOneShot = reallocate_channel_resources(channels, IOTimerChanMode_OneShot, IOTimerChanMode_PWMOut);
			int changeDshot = reallocate_channel_resources(channels, IOTimerChanMode_Dshot, IOTimerChanMode_PWMOut);

			if (changeOneShot && changeDshot) {
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

	uint32_t gpio = 0;
	uint32_t clearbits = CCMR_C1_RESET;
	uint32_t setbits = CCMR_C1_CAPTURE_INIT;
	uint32_t ccer_setbits = CCER_C1_INIT;
	uint32_t dier_setbits = GTIM_DIER_CC1IE;

	/* figure out the GPIO config first */

	switch (mode) {

	case IOTimerChanMode_OneShot:
	case IOTimerChanMode_PWMOut:
	case IOTimerChanMode_Trigger:
	case IOTimerChanMode_Dshot:
		ccer_setbits = 0;
		dier_setbits = 0;
		setbits = CCMR_C1_PWMOUT_INIT;
		break;

	case IOTimerChanMode_PWMIn:
		setbits = CCMR_C1_PWMIN_INIT;
		gpio = timer_io_channels[channel].gpio_in;
		break;

#if defined(BOARD_HAS_CAPTURE)

	case IOTimerChanMode_Capture:
		setbits = CCMR_C1_CAPTURE_INIT;
		gpio = timer_io_channels[channel].gpio_in;
		break;
#endif

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

		uint32_t shifts = timer_io_channels[channel].timer_channel - 1;

		/* Map shifts timer channel 1-4 to 0-3 */

		uint32_t ccmr_offset = STM32_GTIM_CCMR1_OFFSET + ((shifts >> 1)  * sizeof(uint32_t));
		uint32_t ccr_offset = STM32_GTIM_CCR1_OFFSET + (shifts * sizeof(uint32_t));

		clearbits <<= (shifts & 1) * CCMR_C1_NUM_BITS;
		setbits <<= (shifts & 1) * CCMR_C1_NUM_BITS;

		uint16_t rvalue = REG(timer, ccmr_offset);
		rvalue &= ~clearbits;
		rvalue |=  setbits;
		REG(timer, ccmr_offset) = rvalue;

		/*
		 * The beauty here is that per DocID018909 Rev 8 18.3.5 Input capture mode
		 * As soon as CCxS (in SSMRx becomes different from 00, the channel is configured
		 * in input and the TIMx_CCR1 register becomes read-only.
		 * so the next line does nothing in capture mode and initializes an PWM out to
		 * 0
		 */

		REG(timer, ccr_offset) = 0;

		/* on PWM Out ccer_setbits is 0 */

		clearbits = (GTIM_CCER_CC1E | GTIM_CCER_CC1P | HW_GTIM_CCER_CC1NP) << (shifts * CCER_C1_NUM_BITS);
		setbits  = ccer_setbits << (shifts * CCER_C1_NUM_BITS);
		rvalue = rCCER(timer);
		rvalue &= ~clearbits;
		rvalue |=  setbits;
		rCCER(timer) = rvalue;

#if !defined(BOARD_HAS_CAPTURE)
		UNUSED(dier_setbits);
#else
		channel_handlers[channel].callback = channel_handler;
		channel_handlers[channel].context = context;
		rDIER(timer) |= dier_setbits << shifts;
#endif
		px4_leave_critical_section(flags);
	}

	return rv;
}

int io_timer_set_enable(bool state, io_timer_channel_mode_t mode, io_timer_channel_allocation_t masks)
{

	struct action_cache_t {
		uint32_t ccer_clearbits;
		uint32_t ccer_setbits;
		uint32_t dier_setbits;
		uint32_t dier_clearbits;
		uint32_t base;
		uint32_t gpio[MAX_CHANNELS_PER_TIMER];
	} action_cache[MAX_IO_TIMERS];
	memset(action_cache, 0, sizeof(action_cache));

	uint32_t dier_bit = state ? GTIM_DIER_CC1IE : 0;
	uint32_t ccer_bit =  state ? CCER_C1_INIT : 0;
	uint32_t cr1_bit  = 0;

	switch (mode) {
	case IOTimerChanMode_NotUsed:
	case IOTimerChanMode_OneShot:
	case IOTimerChanMode_PWMOut:
	case IOTimerChanMode_Trigger:
		dier_bit = 0;
		cr1_bit  = GTIM_CR1_CEN | GTIM_CR1_ARPE;
		break;

	case IOTimerChanMode_Dshot:
		dier_bit = 0;
		cr1_bit  = state ? GTIM_CR1_CEN : 0;
		break;

	case IOTimerChanMode_PWMIn:
	case IOTimerChanMode_Capture:
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
			uint32_t shifts = timer_io_channels[chan_index].timer_channel - 1;
			uint32_t timer = channels_timer(chan_index);
			action_cache[timer].base  = io_timers[timer].base;
			action_cache[timer].ccer_clearbits |= CCER_C1_INIT << (shifts * CCER_C1_NUM_BITS);
			action_cache[timer].ccer_setbits   |= ccer_bit  << (shifts * CCER_C1_NUM_BITS);
			action_cache[timer].dier_clearbits |= GTIM_DIER_CC1IE  << shifts;
			action_cache[timer].dier_setbits   |= dier_bit << shifts;

			if ((state &&
			     (mode == IOTimerChanMode_PWMOut ||
			      mode == IOTimerChanMode_OneShot ||
			      mode == IOTimerChanMode_Dshot ||
			      mode == IOTimerChanMode_Trigger))) {
				action_cache[timer].gpio[shifts] = timer_io_channels[chan_index].gpio_out;
			}
		}
	}

	irqstate_t flags = px4_enter_critical_section();


	for (unsigned actions = 0; actions < arraySize(action_cache); actions++) {
		if (action_cache[actions].base != 0) {
			uint32_t rvalue = _REG32(action_cache[actions].base, STM32_GTIM_CCER_OFFSET);
			rvalue &= ~action_cache[actions].ccer_clearbits;
			rvalue |= action_cache[actions].ccer_setbits;
			_REG32(action_cache[actions].base, STM32_GTIM_CCER_OFFSET) = rvalue;
			uint32_t after = rvalue & (GTIM_CCER_CC1E | GTIM_CCER_CC2E | GTIM_CCER_CC3E | GTIM_CCER_CC4E);

			rvalue = _REG32(action_cache[actions].base, STM32_GTIM_DIER_OFFSET);
			rvalue &= ~action_cache[actions].dier_clearbits;
			rvalue |= action_cache[actions].dier_setbits;
			_REG32(action_cache[actions].base, STM32_GTIM_DIER_OFFSET) = rvalue;

			/* Any On ?*/

			if (after != 0) {

				/* force an update to preload all registers */
				rEGR(actions) = GTIM_EGR_UG;

				for (unsigned chan = 0; chan < arraySize(action_cache[actions].gpio); chan++) {
					if (action_cache[actions].gpio[chan]) {
						px4_arch_configgpio(action_cache[actions].gpio[chan]);
						action_cache[actions].gpio[chan] = 0;
					}
				}

				/* arm requires the timer be enabled */
				rCR1(actions) |= cr1_bit;

			} else 	{

				rCR1(actions) = 0;
			}
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
		    (mode != IOTimerChanMode_Dshot) &&
		    (mode != IOTimerChanMode_Trigger)) {

			rv = -EIO;

		} else {

			/* configure the channel */

			REG(channels_timer(channel), timer_io_channels[channel].ccr_offset) = value;
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
			value = REG(channels_timer(channel), timer_io_channels[channel].ccr_offset);
		}
	}

	return value;
}

uint32_t io_timer_get_group(unsigned timer)
{
	return get_timer_channels(timer);

}
