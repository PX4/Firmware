/****************************************************************************
 *
 *   Copyright (C) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file perf_counter.h
 * Performance measuring tools.
 */

#ifndef _SYSTEMLIB_PERF_COUNTER_H
#define _SYSTEMLIB_PERF_COUNTER_H value

#include <stdint.h>
#include <px4_platform_common/defines.h>

/**
 * Counter types.
 */
enum perf_counter_type {
	PC_COUNT,		/**< count the number of times an event occurs */
	PC_ELAPSED,		/**< measure the time elapsed performing an event */
	PC_INTERVAL		/**< measure the interval between instances of an event */
};

struct perf_ctr_header;
typedef struct perf_ctr_header	*perf_counter_t;

__BEGIN_DECLS

/**
 * Create a new local counter.
 *
 * @param type			The type of the new counter.
 * @param name			The counter name.
 * @return			Handle for the new counter, or NULL if a counter
 *				could not be allocated.
 */
#ifndef perf_alloc	// perf_alloc might be defined to be NULL in src/modules/px4iofirmware/px4io.h
__EXPORT extern perf_counter_t	perf_alloc(enum perf_counter_type type, const char *name);
#endif

/**
 * Get the reference to an existing counter or create a new one if it does not exist.
 *
 * @param type			The type of the counter.
 * @param name			The counter name.
 * @return			Handle for the counter, or NULL if a counter
 *				could not be allocated.
 */
__EXPORT extern perf_counter_t	perf_alloc_once(enum perf_counter_type type, const char *name);

/**
 * Free a counter.
 *
 * @param handle		The performance counter's handle.
 */
__EXPORT extern void		perf_free(perf_counter_t handle);

/**
 * Count a performance event.
 *
 * This call only affects counters that take single events; PC_COUNT, PC_INTERVAL etc.
 *
 * @param handle		The handle returned from perf_alloc.
 */
__EXPORT extern void		perf_count(perf_counter_t handle);

/**
 * Begin a performance event.
 *
 * This call applies to counters that operate over ranges of time; PC_ELAPSED etc.
 *
 * @param handle		The handle returned from perf_alloc.
 */
__EXPORT extern void		perf_begin(perf_counter_t handle);

/**
 * End a performance event.
 *
 * This call applies to counters that operate over ranges of time; PC_ELAPSED etc.
 * If a call is made without a corresponding perf_begin call, or if perf_cancel
 * has been called subsequently, no change is made to the counter.
 *
 * @param handle		The handle returned from perf_alloc.
 */
__EXPORT extern void		perf_end(perf_counter_t handle);

/**
 * Register a measurement
 *
 * This call applies to counters that operate over ranges of time; PC_ELAPSED etc.
 * If a call is made without a corresponding perf_begin call. It sets the
 * value provided as argument as a new measurement.
 *
 * @param handle		The handle returned from perf_alloc.
 * @param elapsed		The time elapsed. Negative values lead to incrementing the overrun counter.
 */
__EXPORT extern void		perf_set_elapsed(perf_counter_t handle, int64_t elapsed);

/**
 * Register a measurement
 *
 * This call applies to counters that operate over ranges of time; PC_ELAPSED etc.
 * If a call is made without a corresponding perf_begin call. It sets the
 * value provided as argument as a new measurement.
 *
 * @param handle		The handle returned from perf_alloc.
 * @param time			The time for the interval.
 */
__EXPORT extern void		perf_count_interval(perf_counter_t handle, uint64_t time);

/**
 * Set a counter
 *
 * This call applies to counters of type PC_COUNT. It (re-)sets the count.
 *
 * @param handle		The handle returned from perf_alloc.
 * @param count			The counter value to be set.
 */
__EXPORT extern void		perf_set_count(perf_counter_t handle, uint64_t count);

/**
 * Cancel a performance event.
 *
 * This call applies to counters that operate over ranges of time; PC_ELAPSED etc.
 * It reverts the effect of a previous perf_begin.
 *
 * @param handle		The handle returned from perf_alloc.
 */
__EXPORT extern void		perf_cancel(perf_counter_t handle);

/**
 * Reset a performance counter.
 *
 * This call resets performance counter to initial state
 *
 * @param handle		The handle returned from perf_alloc.
 */
__EXPORT extern void		perf_reset(perf_counter_t handle);

/**
 * Print one performance counter to stdout
 *
 * @param handle		The counter to print.
 */
__EXPORT extern void		perf_print_counter(perf_counter_t handle);

/**
 * Print one performance counter to a fd.
 *
 * @param fd			File descriptor to print to - e.g. 0 for stdout
 * @param handle		The counter to print.
 */
__EXPORT extern void		perf_print_counter_fd(int fd, perf_counter_t handle);

/**
 * Print one performance counter to a buffer.
 *
 * @param buffer			buffer to write to
 * @param length			buffer length
 * @param handle			The counter to print.
 * @param return			number of bytes written
 */
__EXPORT extern int		perf_print_counter_buffer(char *buffer, int length, perf_counter_t handle);

/**
 * Print all of the performance counters.
 *
 * @param fd			File descriptor to print to - e.g. 0 for stdout
 */
__EXPORT extern void		perf_print_all(int fd);


typedef void (*perf_callback)(perf_counter_t handle, void *user);

/**
 * Iterate over all performance counters using a callback.
 *
 * Caution: This will aquire the mutex, so do not call any other perf_* method
 * that aquire the mutex as well from the callback (If this is needed, configure
 * the mutex to be reentrant).
 *
 * @param cb callback method
 * @param user custom argument for the callback
 */
__EXPORT extern void	perf_iterate_all(perf_callback cb, void *user);

/**
 * Print hrt latency counters.
 *
 * @param fd			File descriptor to print to - e.g. 0 for stdout
 */
__EXPORT extern void		perf_print_latency(int fd);

/**
 * Reset all of the performance counters.
 */
__EXPORT extern void		perf_reset_all(void);

/**
 * Return current event_count
 *
 * @param handle		The counter returned from perf_alloc.
 * @return			event_count
 */
__EXPORT extern uint64_t	perf_event_count(perf_counter_t handle);

/**
 * Return current mean
 *
 * @param handle		The handle returned from perf_alloc.
 * @param return		mean
 */
__EXPORT extern float		perf_mean(perf_counter_t handle);

__END_DECLS

#endif
