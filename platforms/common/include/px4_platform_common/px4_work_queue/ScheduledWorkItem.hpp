/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "WorkItem.hpp"

#include <drivers/drv_hrt.h>

namespace px4
{

class ScheduledWorkItem : public WorkItem
{
public:

	bool Scheduled() { return !hrt_called(&_call); }

	/**
	 * Schedule next run with a delay in microseconds.
	 *
	 * @param delay_us		The delay in microseconds.
	 */
	void ScheduleDelayed(uint32_t delay_us);

	/**
	 * Schedule repeating run with optional delay.
	 *
	 * @param interval_us		The interval in microseconds.
	 * @param delay_us		The delay (optional) in microseconds.
	 */
	void ScheduleOnInterval(uint32_t interval_us, uint32_t delay_us = 0);

	/**
	 * Schedule next run at a specific time.
	 *
	 * @param time_us		The time in microseconds.
	 */
	void ScheduleAt(hrt_abstime time_us);

	/**
	 * Clear any scheduled work.
	 */
	void ScheduleClear();

protected:

	ScheduledWorkItem(const char *name, const wq_config_t &config) : WorkItem(name, config) {}
	virtual ~ScheduledWorkItem() override;

	virtual void print_run_status() override;

private:

	virtual void Run() override = 0;

	static void	schedule_trampoline(void *arg);

	hrt_call	_call{};
};

} // namespace px4
