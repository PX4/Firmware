/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *       Author: Ben Dyer <ben_dyer@mac.com>
 *               Pavel Kirienko <pavel.kirienko@zubax.com>
 *               David Sidrane <david_s5@nscdg.com>
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
#pragma once

/*
 * We support two classes of timer interfaces. The first one is for structured
 * timers that have an API for life cycle management and use. (timer_xx)
 * The Second type of methods are for interfacing to a high resolution
 * counter with fast access and are provided via an in line API (timer_hrt)
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include <px4_arch/micro_hal.h>
#include "nvic.h"


/* Types for timer access */
typedef uint8_t bl_timer_id;             /* A timer handle */
typedef uint32_t time_ms_t;              /* A timer value */
typedef volatile time_ms_t *time_ref_t;  /* A pointer to the internal
                                         counter in the structure of a timer
                                         used to do a time out test value */

typedef uint32_t time_hrt_cycles_t;     /* A timer value type of the hrt */


/*
 *  Timers
 *
 *  There are 3 modes of operation for the timers.
 *  All modes support a call back on expiration.
 *
 */
typedef enum {
	/*  Specifies a one-shot timer. After notification timer is discarded. */
	modeOneShot         = 1,
	/*  Specifies a repeating timer. */
	modeRepeating       = 2,
	/* Specifies a persistent start / stop timer. */
	modeTimeout         = 3,
	/* Or'ed in to start the timer when allocated */
	modeStarted         = 0x40
} bl_timer_modes_t;


/* The call back function signature type */

typedef void (*bl_timer_ontimeout)(bl_timer_id id, void *context);

/*
 * A helper type for timer allocation to setup a callback
 * There is a null_cb object (see below) that can be used to
 * a bl_timer_cb_t.
 *
 * Typical usage is:
 *
 *  void my_process(bl_timer_id id, void *context) {
 *  ...
 *  };
 *
 *  bl_timer_cb_t mycallback = null_cb;
 *  mycallback.cb = my_process;
 *  bl_timer_id mytimer = timer_allocate(modeRepeating|modeStarted, 100, &mycallback);
 */

typedef struct {
	void *context;
	bl_timer_ontimeout cb;

} bl_timer_cb_t;


extern const bl_timer_cb_t null_cb;

/****************************************************************************
 * Name: timer_init
 *
 * Description:
 *   Called early in os_start to initialize the data associated with
 *   the timers
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
void timer_init(void);

/****************************************************************************
 * Name: timer_allocate
 *
 * Description:
 *   Is used to allocate a timer. Allocation does not involve memory
 *   allocation as the data for the timer are compile time generated.
 *   See OPT_BL_NUMBER_TIMERS
 *
 *   There is an inherent priority to the timers in that the first timer
 *   allocated is the first timer run per tick.
 *
 *   There are 3 modes of operation for the timers. All modes support an
 *   optional call back on expiration.
 *
 *     modeOneShot   - Specifies a one-shot timer. After notification timer
 *                     is resource is freed.
 *     modeRepeating - Specifies a repeating timer that will reload and
 *                     call an optional.
 *     modeTimeout   - Specifies a persistent start / stop timer.
 *
 *     modeStarted   - Or'ed in to start the timer when allocated
 *
 *
 * Input Parameters:
 *   mode       - One of bl_timer_modes_t with the Optional modeStarted
 *   msfromnow  - The reload and initial value for the timer in Ms.
 *   fc         - A pointer or NULL (0). If it is non null it can be any
 *                of the following:
 *
 *                 a) A bl_timer_cb_t populated on the users stack or
 *                 in the data segment. The values are copied into the
 *                 internal data structure of the timer and therefore do
 *                 not have to persist after the call to timer_allocate
 *
 *                 b) The address of null_cb. This is identical to passing
 *                 null for the value of fc.
 *
 * Returned Value:
 *    On success a value from 0 - OPT_BL_NUMBER_TIMERS-1 that is
 *    the bl_timer_id for subsequent timer operations
 *    -1 on failure. This indicates there are no free timers.
 *
 ****************************************************************************/
bl_timer_id timer_allocate(bl_timer_modes_t mode, time_ms_t msfromnow, bl_timer_cb_t *fc);

/****************************************************************************
 * Name: timer_free
 *
 * Description:
 *   Is used to free a timer. Freeing a timer does not involve memory
 *   deallocation as the data for the timer are compile time generated.
 *   See OPT_BL_NUMBER_TIMERS
 *
 * Input Parameters:
 *   id - Returned from timer_allocate;
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/
void timer_free(bl_timer_id id);

/****************************************************************************
 * Name: timer_start
 *
 * Description:
 *   Is used to Start a timer. The reload value is copied to the counter.
 *   And the running bit it set. There is no problem in Starting a running
 *   timer. But it will restart the timeout.
 *
 * Input Parameters:
 *   id - Returned from timer_allocate;
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/
void timer_start(bl_timer_id id);

/****************************************************************************
 * Name: timer_restart
 *
 * Description:
 *   Is used to re start a timer with a new reload count. The reload value
 *   is copied to the counter and the running bit it set. There is no
 *   problem in restarting a running timer.
 *
 * Input Parameters:
 *   id - Returned from timer_allocate;
 *   ms - Is a time_ms_t and the new reload value to use.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/
void timer_restart(bl_timer_id id, time_ms_t ms);

/****************************************************************************
 * Name: timer_stop
 *
 * Description:
 *   Is used to stop a timer. It is Ok to stop a stopped timer.
 *
 * Input Parameters:
 *   id - Returned from timer_allocate;
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/
void timer_stop(bl_timer_id id);

/****************************************************************************
 * Name: timer_expired
 *
 * Description:
 *   Test if a timer that was configured as a modeTimeout timer is expired.
 *   To be expired the time has to be running and have a count of 0.
 *
 * Input Parameters:
 *   id - Returned from timer_allocate;
 *
 * Returned Value:
 *   No Zero if the timer is expired otherwise zero.
 *
 ****************************************************************************/
int timer_expired(bl_timer_id id);

/****************************************************************************
 * Name: timer_ref
 *
 * Description:
 *   Returns an time_ref_t that is a reference (pointer) to the internal counter
 *   of the timer selected by id. It should only be used with calls to
 *   timer_ref_expired.
 *
 * Input Parameters:
 *   id - Returned from timer_allocate;
 *
 * Returned Value:
 *   An internal reference that should be treated as opaque by the caller and
 *   should only be used with calls to timer_ref_expired.
 *   There is no reference counting on the reference and therefore does not
 *   require any operation to free it.
 *
 ****************************************************************************/
time_ref_t timer_ref(bl_timer_id id);

/****************************************************************************
 * Name: timer_ref_expired
 *
 * Description:
 *   Test if a timer, that was configured as a modeTimeout timer is expired
 *   based on the reference provided.
 *
 * Input Parameters:
 *   ref - Returned timer_ref;
 *
 * Returned Value:
 *   Non Zero if the timer is expired otherwise zero.
 *
 ****************************************************************************/
static inline int timer_ref_expired(time_ref_t ref)
{
	return *ref == 0;
}

/****************************************************************************
 * Name: timer_tic
 *
 * Description:
 *   Returns the system tic counter that counts in units of
 *   (CONFIG_USEC_PER_TICK/1000). By default 10 Ms.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
time_ms_t timer_tic(void);

/****************************************************************************
 * Name: timer_hrt_read
 *
 * Description:
 *   Returns the hardware SysTic counter that counts in units of
 *  STM32_HCLK_FREQUENCY. This file defines TIMER_HRT_CYCLES_PER_US
 *  and TIMER_HRT_CYCLES_PER_MS that should be used to define times.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current value of the HW counter in the type of time_hrt_cycles_t.
 *
 ****************************************************************************/
static inline time_hrt_cycles_t timer_hrt_read(void)
{
	return getreg32(NVIC_SYSTICK_CURRENT);
}

/****************************************************************************
 * Name: timer_hrt_clear_wrap
 *
 * Description:
 *   Clears the wrap flag by reading the timer
 *
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
static inline void timer_hrt_clear_wrap(void)
{
	(void)timer_hrt_read();
}
/****************************************************************************
 * Name: timer_hrt_wrap
 *
 * Description:
 *   Returns true if SysTic  counted to 0 since last time it was
 *   read.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Returns true  if timer counted to 0 since last time this was read.
 *
 ****************************************************************************/
static inline bool timer_hrt_wrap(void)
{
	uint32_t rv = getreg32(NVIC_SYSTICK_CTRL);
	return ((rv  & NVIC_SYSTICK_CTRL_COUNTFLAG) ? true : false);
}

/****************************************************************************
 * Name: timer_hrt_max
 *
 * Description:
 *   Returns the hardware SysTic reload value +1
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current SysTic reload of the HW counter in the type of
 *   time_hrt_cycles_t.
 *
 ****************************************************************************/
static inline time_hrt_cycles_t timer_hrt_max(void)
{
	return getreg32(NVIC_SYSTICK_RELOAD) + 1;
}

/****************************************************************************
 * Name: timer_hrt_elapsed
 *
 * Description:
 *   Returns the difference between 2 time values, taking into account
 *   the way the timer wrap.
 *
 * Input Parameters:
 *   begin - Beginning timer count.
 *   end   - Ending timer count.
 *
 * Returned Value:
 *   The difference from begin to end
 *
 ****************************************************************************/
static inline time_hrt_cycles_t timer_hrt_elapsed(time_hrt_cycles_t begin, time_hrt_cycles_t end)
{
	/* It is a down count from NVIC_SYSTICK_RELOAD */

	time_hrt_cycles_t elapsed = begin - end;
	time_hrt_cycles_t reload = timer_hrt_max();

	/* Did it wrap */
	if (elapsed > reload) {
		elapsed +=  reload;
	}

	return elapsed;
}
