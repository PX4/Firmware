/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file drv_hrt.h
 *
 * High-resolution timer with callouts and timekeeping.
 */

#pragma once

#include <sys/ioctl.h>
#include <sys/types.h>
#include <stdbool.h>
#include <inttypes.h>

#include <px4_platform_common/time.h>
#include <queue.h>

__BEGIN_DECLS

/**
 * Absolute time, in microsecond units.
 *
 * Absolute time is measured from some arbitrary epoch shortly after
 * system startup.  It should never wrap or go backwards.
 */
typedef uint64_t	hrt_abstime;

/**
 * Callout function type.
 *
 * Note that callouts run in the timer interrupt context, so
 * they are serialised with respect to each other, and must not
 * block.
 */
typedef void	(* hrt_callout)(void *arg);

/**
 * Callout record.
 */
typedef struct hrt_call {
	struct sq_entry_s	link;

	hrt_abstime		deadline;
	hrt_abstime		period;
	hrt_callout		callout;
	void			*arg;
} *hrt_call_t;


#define LATENCY_BUCKET_COUNT 8
extern const uint16_t latency_bucket_count;
extern const uint16_t latency_buckets[LATENCY_BUCKET_COUNT];
extern uint32_t latency_counters[LATENCY_BUCKET_COUNT + 1];

/**
 * Get absolute time in [us] (does not wrap).
 */
__EXPORT extern hrt_abstime hrt_absolute_time(void);

/**
 * Convert a timespec to absolute time.
 */
__EXPORT extern hrt_abstime ts_to_abstime(const struct timespec *ts);

/**
 * Convert absolute time to a timespec.
 */
__EXPORT extern void	abstime_to_ts(struct timespec *ts, hrt_abstime abstime);

/**
 * Compute the delta between a timestamp taken in the past
 * and now.
 *
 * This function is not interrupt save.
 */
static inline hrt_abstime hrt_elapsed_time(const hrt_abstime *then)
{
	return hrt_absolute_time() - *then;
}

/**
 * Compute the delta between a timestamp taken in the past
 * and now.
 *
 * This function is safe to use even if the timestamp is updated
 * by an interrupt during execution.
 */
__EXPORT extern hrt_abstime hrt_elapsed_time_atomic(const volatile hrt_abstime *then);

/**
 * Store the absolute time in an interrupt-safe fashion.
 *
 * This function ensures that the timestamp cannot be seen half-written by an interrupt handler.
 */
__EXPORT extern void hrt_store_absolute_time(volatile hrt_abstime *time);

#ifdef __PX4_QURT
/**
 * Set a time offset to hrt_absolute_time on the DSP.
 * @param time_diff_us: time difference of the DSP clock to Linux clock.
 *   This param is positive if the Linux clock is ahead of the DSP one.
 */
__EXPORT extern int hrt_set_absolute_time_offset(int32_t time_diff_us);
#endif

/**
 * Call callout(arg) after delay has elapsed.
 *
 * If callout is NULL, this can be used to implement a timeout by testing the call
 * with hrt_called().
 */
__EXPORT extern void	hrt_call_after(struct hrt_call *entry, hrt_abstime delay, hrt_callout callout, void *arg);

/**
 * Call callout(arg) at absolute time calltime.
 */
__EXPORT extern void	hrt_call_at(struct hrt_call *entry, hrt_abstime calltime, hrt_callout callout, void *arg);

/**
 * Call callout(arg) after delay, and then after every interval.
 *
 * Note thet the interval is timed between scheduled, not actual, call times, so the call rate may
 * jitter but should not drift.
 */
__EXPORT extern void	hrt_call_every(struct hrt_call *entry, hrt_abstime delay, hrt_abstime interval,
				       hrt_callout callout, void *arg);

/**
 * If this returns true, the entry has been invoked and removed from the callout list,
 * or it has never been entered.
 *
 * Always returns false for repeating callouts.
 */
__EXPORT extern bool	hrt_called(struct hrt_call *entry);

/**
 * Remove the entry from the callout list.
 */
__EXPORT extern void	hrt_cancel(struct hrt_call *entry);

/**
 * Initialise a hrt_call structure
 */
__EXPORT extern void	hrt_call_init(struct hrt_call *entry);

/*
 * delay a hrt_call_every() periodic call by the given number of
 * microseconds. This should be called from within the callout to
 * cause the callout to be re-scheduled for a later time. The periodic
 * callouts will then continue from that new base time at the
 * previously specified period.
 */
__EXPORT extern void	hrt_call_delay(struct hrt_call *entry, hrt_abstime delay);

/*
 * Initialise the HRT.
 */
__EXPORT extern void	hrt_init(void);

#ifdef __PX4_POSIX

__EXPORT extern hrt_abstime hrt_absolute_time_offset(void);

#endif
#if defined(ENABLE_LOCKSTEP_SCHEDULER)

__EXPORT extern int px4_lockstep_register_component(void);
__EXPORT extern void px4_lockstep_unregister_component(int component);
__EXPORT extern void px4_lockstep_progress(int component);
__EXPORT extern void px4_lockstep_wait_for_components(void);

#else
static inline int px4_lockstep_register_component(void) { return 0; }
static inline void px4_lockstep_unregister_component(int component) { }
static inline void px4_lockstep_progress(int component) { }
static inline void px4_lockstep_wait_for_components(void) { }
#endif /* defined(ENABLE_LOCKSTEP_SCHEDULER) */

__END_DECLS



#ifdef	__cplusplus

namespace time_literals
{

// User-defined integer literals for different time units.
// The base unit is hrt_abstime in microseconds

constexpr hrt_abstime operator "" _s(unsigned long long seconds)
{
	return hrt_abstime(seconds * 1000000ULL);
}

constexpr hrt_abstime operator "" _ms(unsigned long long milliseconds)
{
	return hrt_abstime(milliseconds * 1000ULL);
}

constexpr hrt_abstime operator "" _us(unsigned long long microseconds)
{
	return hrt_abstime(microseconds);
}

} /* namespace time_literals */


#endif /* __cplusplus */
