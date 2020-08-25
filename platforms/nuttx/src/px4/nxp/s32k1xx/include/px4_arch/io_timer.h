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
 * @file drv_io_timer.h
 *
 * stm32-specific PWM output data.
 */
#include <px4_platform_common/px4_config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <drivers/drv_hrt.h>

#pragma once
__BEGIN_DECLS
/* configuration limits */
#ifdef BOARD_NUM_IO_TIMERS
#define MAX_IO_TIMERS			BOARD_NUM_IO_TIMERS
#else
#define MAX_IO_TIMERS			2
#endif
#define MAX_TIMER_IO_CHANNELS	16

#define MAX_LED_TIMERS			2
#define MAX_TIMER_LED_CHANNELS	6

#define IO_TIMER_ALL_MODES_CHANNELS 0

typedef enum io_timer_channel_mode_t {
	IOTimerChanMode_NotUsed = 0,
	IOTimerChanMode_PWMOut  = 1,
	IOTimerChanMode_PWMIn   = 2,
	IOTimerChanMode_Capture = 3,
	IOTimerChanMode_OneShot = 4,
	IOTimerChanMode_Trigger = 5,
	IOTimerChanModeSize
} io_timer_channel_mode_t;

typedef uint16_t io_timer_channel_allocation_t; /* big enough to hold MAX_TIMER_IO_CHANNELS */

/* array of timers dedicated to PWM in and out and capture use
 *** Note that the clock_freq is set to the source in the clock tree that
 *** feeds this specific timer. This can differs by Timer!
 *** In PWM  mode the timer's prescaler is set to achieve a counter frequency of 1MHz
 *** In OneShot mode the timer's prescaler is set to achieve a counter frequency of 8MHz
 *** Other prescaler rates can be achieved by fore instance by setting the clock_freq = 1Mhz
 *** the resulting PSC will be one and the timer will count at it's clock frequency.
 */
typedef struct io_timers_t {
	uint32_t	base;                /* Base address of the timer */
	uint32_t	clock_register;      /* SIM_SCGCn */
	uint32_t	clock_bit;           /* SIM_SCGCn bit pos */
	uint32_t	vectorno;            /* IRQ number */
} io_timers_t;

typedef struct io_timers_channel_mapping_element_t {
	uint32_t first_channel_index;
	uint32_t channel_count;
} io_timers_channel_mapping_element_t;

/* mapping for each io_timers to timer_io_channels */
typedef struct io_timers_channel_mapping_t {
	io_timers_channel_mapping_element_t element[MAX_IO_TIMERS];
} io_timers_channel_mapping_t;

/* array of channels in logical order */
typedef struct timer_io_channels_t {
	uint32_t	gpio_out;            /* The timer channel GPIO for PWM */
	uint32_t	gpio_in;             /* The timer channel GPIO for Capture */
	uint8_t		timer_index;         /* 0 based index in the io_timers_t table */
	uint8_t		timer_channel;       /* 1 based channel index GPIO_FTMt_CHcIN = c+1) */
} timer_io_channels_t;


typedef void (*channel_handler_t)(void *context, const io_timers_t *timer, uint32_t chan_index,
				  const timer_io_channels_t *chan,
				  hrt_abstime isrs_time, uint16_t isrs_rcnt,
				  uint16_t capture);


/* supplied by board-specific code */
__EXPORT extern const io_timers_t io_timers[MAX_IO_TIMERS];
__EXPORT extern const io_timers_channel_mapping_t io_timers_channel_mapping;
__EXPORT extern const timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS];

__EXPORT extern const io_timers_t led_pwm_timers[MAX_LED_TIMERS];
__EXPORT extern const timer_io_channels_t led_pwm_channels[MAX_TIMER_LED_CHANNELS];

__EXPORT extern io_timer_channel_allocation_t allocations[IOTimerChanModeSize];

__EXPORT int io_timer_channel_init(unsigned channel, io_timer_channel_mode_t mode,
				   channel_handler_t channel_handler, void *context);

__EXPORT int io_timer_init_timer(unsigned timer);

__EXPORT int io_timer_set_rate(unsigned timer, unsigned rate);
__EXPORT int io_timer_set_enable(bool state, io_timer_channel_mode_t mode,
				 io_timer_channel_allocation_t masks);
__EXPORT int io_timer_set_rate(unsigned timer, unsigned rate);
__EXPORT uint16_t io_channel_get_ccr(unsigned channel);
__EXPORT int io_timer_set_ccr(unsigned channel, uint16_t value);
__EXPORT uint32_t io_timer_get_group(unsigned timer);
__EXPORT int io_timer_validate_channel_index(unsigned channel);
__EXPORT int io_timer_is_channel_free(unsigned channel);
__EXPORT int io_timer_free_channel(unsigned channel);
__EXPORT int io_timer_get_channel_mode(unsigned channel);
__EXPORT int io_timer_get_mode_channels(io_timer_channel_mode_t mode);
__EXPORT extern void io_timer_trigger(void);
/**
 * Returns the pin configuration for a specific channel, to be used as GPIO output.
 * 0 is returned if the channel is not valid.
 */
__EXPORT uint32_t io_timer_channel_get_gpio_output(unsigned channel);
/**
 * Returns the pin configuration for a specific channel, to be used as PWM input.
 * 0 is returned if the channel is not valid.
 */
__EXPORT uint32_t io_timer_channel_get_as_pwm_input(unsigned channel);


__END_DECLS

