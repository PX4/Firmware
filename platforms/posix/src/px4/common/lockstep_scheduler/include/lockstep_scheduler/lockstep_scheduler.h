/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include <cstdint>
#include <mutex>
#include <vector>
#include <memory>
#include <atomic>
#include <pthread.h>

#include "lockstep_components.h"

class LockstepScheduler
{
public:
	~LockstepScheduler();

	void set_absolute_time(uint64_t time_us);
	inline uint64_t get_absolute_time() const { return _time_us; }
	int cond_timedwait(pthread_cond_t *cond, pthread_mutex_t *lock, uint64_t time_us);
	int usleep_until(uint64_t timed_us);

	LockstepComponents &components() { return _components; }

private:
	struct TimedWait {
		~TimedWait()
		{
			if (!done) {
				// This can only happen when a thread gets canceled (e.g. via pthread_cancel), and since
				// pthread_cond_wait is a cancellation point, the rest of LockstepScheduler::cond_timedwait afterwards
				// might not be executed. Which means the mutex will not be unlocked either, so we unlock to avoid
				// a dead-lock in LockstepScheduler::set_absolute_time().
				// This destructor gets called as part of thread-local storage cleanup.
				// This is really only a work-around for non-proper thread stopping. Note that we also assume,
				// that we can still access the mutex.
				if (passed_lock) {
					pthread_mutex_unlock(passed_lock);
				}

				done = true;
			}

			// If a thread quickly exits after a cond_timedwait(), the
			// thread_local object can still be in the linked list. In that case
			// we need to wait until it's removed.
			while (!removed) {
				system_usleep(5000);
			}
		}

		pthread_cond_t *passed_cond{nullptr};
		pthread_mutex_t *passed_lock{nullptr};
		uint64_t time_us{0};
		bool timeout{false};
		std::atomic<bool> done{false};
		std::atomic<bool> removed{true};

		TimedWait *next{nullptr}; ///< linked list
	};

	LockstepComponents _components;

	std::atomic<uint64_t> _time_us{0};

	TimedWait *_timed_waits{nullptr}; ///< head of linked list
	std::mutex _timed_waits_mutex;
	std::atomic<bool> _setting_time{false}; ///< true if set_absolute_time() is currently being executed
};
