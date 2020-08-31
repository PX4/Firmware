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
 * @file perf_counter.c
 *
 * @brief Performance measuring tools.
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/queue.h>
#include <drivers/drv_hrt.h>
#include <math.h>
#include <pthread.h>
#include <systemlib/err.h>

#include "perf_counter.h"

#ifdef __PX4_QURT
// There is presumably no dprintf on QURT. Therefore use the usual output to mini-dm.
#define dprintf(_fd, _text, ...) ((_fd) == 1 ? PX4_INFO((_text), ##__VA_ARGS__) : (void)(_fd))
#endif

/**
 * Header common to all counters.
 */
struct perf_ctr_header {
	sq_entry_t		link;	/**< list linkage */
	enum perf_counter_type	type;	/**< counter type */
	const char		*name;	/**< counter name */
};

/**
 * PC_EVENT counter.
 */
struct perf_ctr_count : public perf_ctr_header {
	uint64_t		event_count{0};
};

/**
 * PC_ELAPSED counter.
 */
struct perf_ctr_elapsed : public perf_ctr_header {
	uint64_t		event_count{0};
	uint64_t		time_start{0};
	uint64_t		time_total{0};
	uint32_t		time_least{0};
	uint32_t		time_most{0};
	float			mean{0.0f};
	float			M2{0.0f};
};

/**
 * PC_INTERVAL counter.
 */
struct perf_ctr_interval : public perf_ctr_header {
	uint64_t		event_count{0};
	uint64_t		time_event{0};
	uint64_t		time_first{0};
	uint64_t		time_last{0};
	uint32_t		time_least{0};
	uint32_t		time_most{0};
	float			mean{0.0f};
	float			M2{0.0f};
};

/**
 * List of all known counters.
 */
static sq_queue_t	perf_counters = { nullptr, nullptr };

/**
 * mutex protecting access to the perf_counters linked list (which is read from & written to by different threads)
 */
pthread_mutex_t perf_counters_mutex = PTHREAD_MUTEX_INITIALIZER;
// FIXME: the mutex does **not** protect against access to/from the perf
// counter's data. It can still happen that a counter is updated while it is
// printed. This can lead to inconsistent output, or completely bogus values
// (especially the 64bit values which are in general not atomically updated).
// The same holds for shared perf counters (perf_alloc_once), that can be updated
// concurrently (this affects the 'ctrl_latency' counter).


perf_counter_t
perf_alloc(enum perf_counter_type type, const char *name)
{
	perf_counter_t ctr = nullptr;

	switch (type) {
	case PC_COUNT:
		ctr = new perf_ctr_count();
		break;

	case PC_ELAPSED:
		ctr = new perf_ctr_elapsed();
		break;

	case PC_INTERVAL:
		ctr = new perf_ctr_interval();
		break;

	default:
		break;
	}

	if (ctr != nullptr) {
		ctr->type = type;
		ctr->name = name;
		pthread_mutex_lock(&perf_counters_mutex);
		sq_addfirst(&ctr->link, &perf_counters);
		pthread_mutex_unlock(&perf_counters_mutex);
	}

	return ctr;
}

perf_counter_t
perf_alloc_once(enum perf_counter_type type, const char *name)
{
	pthread_mutex_lock(&perf_counters_mutex);
	perf_counter_t handle = (perf_counter_t)sq_peek(&perf_counters);

	while (handle != nullptr) {
		if (!strcmp(handle->name, name)) {
			if (type == handle->type) {
				/* they are the same counter */
				pthread_mutex_unlock(&perf_counters_mutex);
				return handle;

			} else {
				/* same name but different type, assuming this is an error and not intended */
				pthread_mutex_unlock(&perf_counters_mutex);
				return nullptr;
			}
		}

		handle = (perf_counter_t)sq_next(&handle->link);
	}

	pthread_mutex_unlock(&perf_counters_mutex);

	/* if the execution reaches here, no existing counter of that name was found */
	return perf_alloc(type, name);
}

void
perf_free(perf_counter_t handle)
{
	if (handle == nullptr) {
		return;
	}

	pthread_mutex_lock(&perf_counters_mutex);
	sq_rem(&handle->link, &perf_counters);
	pthread_mutex_unlock(&perf_counters_mutex);

	delete handle;
}

void
perf_count(perf_counter_t handle)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_COUNT:
		((struct perf_ctr_count *)handle)->event_count++;
		break;

	case PC_INTERVAL:
		perf_count_interval(handle, hrt_absolute_time());
		break;

	default:
		break;
	}
}

void
perf_begin(perf_counter_t handle)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_ELAPSED:
		((struct perf_ctr_elapsed *)handle)->time_start = hrt_absolute_time();
		break;

	default:
		break;
	}
}

void
perf_end(perf_counter_t handle)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;

			if (pce->time_start != 0) {
				perf_set_elapsed(handle, hrt_elapsed_time(&pce->time_start));
			}
		}
		break;

	default:
		break;
	}
}

void
perf_set_elapsed(perf_counter_t handle, int64_t elapsed)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;

			if (elapsed >= 0) {
				pce->event_count++;
				pce->time_total += elapsed;

				if ((pce->time_least > (uint32_t)elapsed) || (pce->time_least == 0)) {
					pce->time_least = elapsed;
				}

				if (pce->time_most < (uint32_t)elapsed) {
					pce->time_most = elapsed;
				}

				// maintain mean and variance of the elapsed time in seconds
				// Knuth/Welford recursive mean and variance of update intervals (via Wikipedia)
				float dt = elapsed / 1e6f;
				float delta_intvl = dt - pce->mean;
				pce->mean += delta_intvl / pce->event_count;
				pce->M2 += delta_intvl * (dt - pce->mean);

				pce->time_start = 0;
			}
		}
		break;

	default:
		break;
	}
}

void
perf_count_interval(perf_counter_t handle, hrt_abstime now)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_INTERVAL: {
			struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;

			switch (pci->event_count) {
			case 0:
				pci->time_first = now;
				break;

			case 1:
				pci->time_least = (uint32_t)(now - pci->time_last);
				pci->time_most = (uint32_t)(now - pci->time_last);
				pci->mean = pci->time_least / 1e6f;
				pci->M2 = 0;
				break;

			default: {
					hrt_abstime interval = now - pci->time_last;

					if ((uint32_t)interval < pci->time_least) {
						pci->time_least = (uint32_t)interval;
					}

					if ((uint32_t)interval > pci->time_most) {
						pci->time_most = (uint32_t)interval;
					}

					// maintain mean and variance of interval in seconds
					// Knuth/Welford recursive mean and variance of update intervals (via Wikipedia)
					float dt = interval / 1e6f;
					float delta_intvl = dt - pci->mean;
					pci->mean += delta_intvl / pci->event_count;
					pci->M2 += delta_intvl * (dt - pci->mean);
					break;
				}
			}

			pci->time_last = now;
			pci->event_count++;
			break;
		}

	default:
		break;
	}
}

void
perf_set_count(perf_counter_t handle, uint64_t count)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_COUNT: {
			((struct perf_ctr_count *)handle)->event_count = count;
		}
		break;

	default:
		break;
	}

}

void
perf_cancel(perf_counter_t handle)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;

			pce->time_start = 0;
		}
		break;

	default:
		break;
	}
}

void
perf_reset(perf_counter_t handle)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_COUNT:
		((struct perf_ctr_count *)handle)->event_count = 0;
		break;

	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;
			pce->event_count = 0;
			pce->time_start = 0;
			pce->time_total = 0;
			pce->time_least = 0;
			pce->time_most = 0;
			break;
		}

	case PC_INTERVAL: {
			struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;
			pci->event_count = 0;
			pci->time_event = 0;
			pci->time_first = 0;
			pci->time_last = 0;
			pci->time_least = 0;
			pci->time_most = 0;
			break;
		}
	}
}

void
perf_print_counter(perf_counter_t handle)
{
	if (handle == nullptr) {
		return;
	}

	perf_print_counter_fd(1, handle);
}

void
perf_print_counter_fd(int fd, perf_counter_t handle)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_COUNT:
		dprintf(fd, "%s: %llu events\n",
			handle->name,
			(unsigned long long)((struct perf_ctr_count *)handle)->event_count);
		break;

	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;
			float rms = sqrtf(pce->M2 / (pce->event_count - 1));
			dprintf(fd, "%s: %llu events, %lluus elapsed, %.2fus avg, min %lluus max %lluus %5.3fus rms\n",
				handle->name,
				(unsigned long long)pce->event_count,
				(unsigned long long)pce->time_total,
				(pce->event_count == 0) ? 0 : (double)pce->time_total / (double)pce->event_count,
				(unsigned long long)pce->time_least,
				(unsigned long long)pce->time_most,
				(double)(1e6f * rms));
			break;
		}

	case PC_INTERVAL: {
			struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;
			float rms = sqrtf(pci->M2 / (pci->event_count - 1));

			dprintf(fd, "%s: %llu events, %.2fus avg, min %lluus max %lluus %5.3fus rms\n",
				handle->name,
				(unsigned long long)pci->event_count,
				(pci->event_count == 0) ? 0 : (double)(pci->time_last - pci->time_first) / (double)pci->event_count,
				(unsigned long long)pci->time_least,
				(unsigned long long)pci->time_most,
				(double)(1e6f * rms));
			break;
		}

	default:
		break;
	}
}


int
perf_print_counter_buffer(char *buffer, int length, perf_counter_t handle)
{
	int num_written = 0;

	if (handle == nullptr) {
		return 0;
	}

	switch (handle->type) {
	case PC_COUNT:
		num_written = snprintf(buffer, length, "%s: %llu events",
				       handle->name,
				       (unsigned long long)((struct perf_ctr_count *)handle)->event_count);
		break;

	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;
			float rms = sqrtf(pce->M2 / (pce->event_count - 1));
			num_written = snprintf(buffer, length, "%s: %llu events, %lluus elapsed, %.2fus avg, min %lluus max %lluus %5.3fus rms",
					       handle->name,
					       (unsigned long long)pce->event_count,
					       (unsigned long long)pce->time_total,
					       (pce->event_count == 0) ? 0 : (double)pce->time_total / (double)pce->event_count,
					       (unsigned long long)pce->time_least,
					       (unsigned long long)pce->time_most,
					       (double)(1e6f * rms));
			break;
		}

	case PC_INTERVAL: {
			struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;
			float rms = sqrtf(pci->M2 / (pci->event_count - 1));

			num_written = snprintf(buffer, length, "%s: %llu events, %.2f avg, min %lluus max %lluus %5.3fus rms",
					       handle->name,
					       (unsigned long long)pci->event_count,
					       (pci->event_count == 0) ? 0 : (double)(pci->time_last - pci->time_first) / (double)pci->event_count,
					       (unsigned long long)pci->time_least,
					       (unsigned long long)pci->time_most,
					       (double)(1e6f * rms));
			break;
		}

	default:
		break;
	}

	buffer[length - 1] = 0; // ensure 0-termination
	return num_written;
}

uint64_t
perf_event_count(perf_counter_t handle)
{
	if (handle == nullptr) {
		return 0;
	}

	switch (handle->type) {
	case PC_COUNT:
		return ((struct perf_ctr_count *)handle)->event_count;

	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;
			return pce->event_count;
		}

	case PC_INTERVAL: {
			struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;
			return pci->event_count;
		}

	default:
		break;
	}

	return 0;
}

float
perf_mean(perf_counter_t handle)
{
	if (handle == nullptr) {
		return 0;
	}

	switch (handle->type) {
	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;
			return pce->mean;
		}

	case PC_INTERVAL: {
			struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;
			return pci->mean;
		}

	default:
		break;
	}

	return 0.0f;
}

void
perf_iterate_all(perf_callback cb, void *user)
{
	pthread_mutex_lock(&perf_counters_mutex);
	perf_counter_t handle = (perf_counter_t)sq_peek(&perf_counters);

	while (handle != nullptr) {
		cb(handle, user);
		handle = (perf_counter_t)sq_next(&handle->link);
	}

	pthread_mutex_unlock(&perf_counters_mutex);
}

void
perf_print_all(int fd)
{
	pthread_mutex_lock(&perf_counters_mutex);
	perf_counter_t handle = (perf_counter_t)sq_peek(&perf_counters);

	while (handle != nullptr) {
		perf_print_counter_fd(fd, handle);
		handle = (perf_counter_t)sq_next(&handle->link);
	}

	pthread_mutex_unlock(&perf_counters_mutex);
}

void
perf_print_latency(int fd)
{
	dprintf(fd, "bucket [us] : events\n");

	for (int i = 0; i < latency_bucket_count; i++) {
		dprintf(fd, "       %4i : %li\n", latency_buckets[i], (long int)latency_counters[i]);
	}

	// print the overflow bucket value
	dprintf(fd, " >%4i : %i\n", latency_buckets[latency_bucket_count - 1], latency_counters[latency_bucket_count]);
}

void
perf_reset_all(void)
{
	pthread_mutex_lock(&perf_counters_mutex);
	perf_counter_t handle = (perf_counter_t)sq_peek(&perf_counters);

	while (handle != nullptr) {
		perf_reset(handle);
		handle = (perf_counter_t)sq_next(&handle->link);
	}

	pthread_mutex_unlock(&perf_counters_mutex);

	for (int i = 0; i <= latency_bucket_count; i++) {
		latency_counters[i] = 0;
	}
}
