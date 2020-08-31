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

#include <nuttx/config.h>

// Turn off Probes in this module
#undef CONFIG_BOARD_USE_PROBES

#include <boot_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/arch.h>
#include <arch/irq.h>
#include <arch/board/board.h>

#include "px4_macros.h"
#include "timer.h"
#include "nvic.h"

typedef enum {
	OneShot         = modeOneShot,
	Repeating       = modeRepeating,
	Timeout         = modeTimeout,

	modeMsk         = 0x3,
	running         = modeStarted,
	inuse           = 0x80,

} bl_timer_ctl_t;

typedef struct {
	bl_timer_cb_t         usr;
	time_ms_t             count;
	time_ms_t             reload;
	bl_timer_ctl_t        ctl;
} bl_timer_t;

static time_ms_t sys_tic;
static bl_timer_t timers[OPT_BL_NUMBER_TIMERS];

/* Use to initialize  */
const bl_timer_cb_t null_cb = { 0, 0 };

/* We use the linker --wrap ability to wrap the NuttX stm32 call out to
 * the sceduler's sched_process_timer and service it here. Thus replacing
 * the NuttX scheduler with are timer driven scheduling.
 */
void __wrap_sched_process_timer(void);

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
time_ms_t timer_tic(void)
{
	return sys_tic;
}

/****************************************************************************
 * Name: sched_process_timer
 *
 * Description:
 *   Called by Nuttx on the ISR of the SysTic. This function run the list of
 *   timers. It deducts that amount of the time of a system tick from the
 *   any timers that are in use and running.
 *
 *   Depending on the mode of the timer, the appropriate actions is taken on
 *   expiration.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
__EXPORT void __wrap_sched_process_timer(void)
{
	//PROBE(1, true);
	//PROBE(1, false);

	/* Increment the per-tick system counter */
	sys_tic++;

	/* todo:May need a X tick here is threads run long */
	time_ms_t ms_elapsed = (CONFIG_USEC_PER_TICK / 1000);

	/* Walk the time list from High to low and */
	bl_timer_id t;

	for (t =  arraySize(timers) - 1; (int8_t) t >= 0; t--) {

		/* Timer in use and running */
		if ((timers[t].ctl & (inuse | running)) == (inuse | running)) {

			/* Is it NOT already expired nor about to expire ?*/
			if (timers[t].count != 0) {

				/* Is it off in future */
				if (timers[t].count > ms_elapsed) {

					/* Just remove the amount attributed to the tick */

					timers[t].count -= ms_elapsed;
					continue;
				}

				/* it has expired now or less than a tick ago */

				/* Mark it expired */

				timers[t].count = 0;

				/* Now perform action based on mode */
				switch (timers[t].ctl & ~(inuse | running)) {

				case OneShot: {
						bl_timer_cb_t user = timers[t].usr;
						memset(&timers[t], 0, sizeof(timers[t]));

						if (user.cb) {
							user.cb(t, user.context);
						}
					}
					break;

				case Repeating:
					timers[t].count = timers[t].reload;

				/* FALLTHROUGH */
				/* to callback */
				case Timeout:
					if (timers[t].usr.cb) {
						timers[t].usr.cb(t, timers[t].usr.context);
					}

					break;

				default:
					break;
				}
			}
		}
	}
}

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
bl_timer_id timer_allocate(bl_timer_modes_t mode, time_ms_t msfromnow, bl_timer_cb_t *fc)
{
	bl_timer_id t;
	irqstate_t s = enter_critical_section();

	for (t = arraySize(timers) - 1; (int8_t)t >= 0; t--) {

		if ((timers[t].ctl & inuse) == 0) {

			timers[t].reload = msfromnow;
			timers[t].count = msfromnow;
			timers[t].usr = fc ? *fc : null_cb;
			timers[t].ctl = (mode & (modeMsk | running)) | (inuse);
			break;
		}
	}

	leave_critical_section(s);
	return t;
}

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
void timer_free(bl_timer_id id)
{
	DEBUGASSERT(id >= 0 && id < arraySize(timers));
	irqstate_t s = enter_critical_section();
	memset(&timers[id], 0, sizeof(timers[id]));
	leave_critical_section(s);
}

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
void timer_start(bl_timer_id id)
{
	DEBUGASSERT(id >= 0 && id < arraySize(timers) && (timers[id].ctl & inuse));
	irqstate_t s = enter_critical_section();
	timers[id].count = timers[id].reload;
	timers[id].ctl |= running;
	leave_critical_section(s);
}

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
void timer_stop(bl_timer_id id)
{
	DEBUGASSERT(id >= 0 && id < arraySize(timers) && (timers[id].ctl & inuse));
	irqstate_t s = enter_critical_section();
	timers[id].ctl &= ~running;
	leave_critical_section(s);

}

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
 *   Non Zero if the timer is expired otherwise zero.
 *
 ****************************************************************************/
int timer_expired(bl_timer_id id)
{
	DEBUGASSERT(id >= 0 && id < arraySize(timers) && (timers[id].ctl & inuse));
	irqstate_t s = enter_critical_section();
	int rv = ((timers[id].ctl & running) && timers[id].count == 0);
	leave_critical_section(s);
	return rv;
}

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
void timer_restart(bl_timer_id id, time_ms_t ms)
{
	DEBUGASSERT(id >= 0 && id < arraySize(timers) && (timers[id].ctl & inuse));
	irqstate_t s = enter_critical_section();
	timers[id].count = timers[id].reload = ms;
	timers[id].ctl |= running;
	leave_critical_section(s);
}

/****************************************************************************
 * Name: timer_ref
 *
 * Description:
 *   Returns an time_ref_t that is a reference (ponter) to the internal counter
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
 *************************************************************************/
time_ref_t timer_ref(bl_timer_id id)
{
	DEBUGASSERT(id >= 0 && id < arraySize(timers) && (timers[id].ctl & inuse));
	return (time_ref_t) &timers[id].count;
}

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
__EXPORT void timer_init(void)
{
	/* For system timing probing see bord.h and
	 * CONFIG_BOARD_USE_PROBES
	 */
	//PROBE_INIT(7);
	//PROBE(1, true);
	//PROBE(2, true);
	//PROBE(3, true);
	//PROBE(1, false);
	//PROBE(2, false);
	//PROBE(3, false);
	/* This is the lowlevel IO if needed to instrument timing
	 * with the smallest impact
	 * *((uint32_t *)0x40011010) = 0x100; // PROBE(3,true);
	 *  *((uint32_t *)0x40011014) = 0x100; // PROBE(3,false);
	 */

	/* Initialize timer data */

	sys_tic = 0;
	memset(timers, 0, sizeof(timers));
}
