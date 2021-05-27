/****************************************************************************
 *
 *   Copyright (c) 2012-2020 PX4 Development Team. All rights reserved.
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

#if defined(__PX4_NUTTX)
#include <malloc.h>
#endif
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform/cpuload.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_cpuload.h>
#include <uORB/topics/task_stack_info.h>

#if defined(__PX4_LINUX)
#include <sys/times.h>
#endif

namespace load_mon
{

class LoadMon : public ModuleBase<LoadMon>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	LoadMon();
	~LoadMon() override;

	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[])
	{
		return print_usage("unknown command");
	}

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	void start();

private:
	/** Do a compute and schedule the next cycle. */
	void Run() override;

	/** Do a calculation of the CPU load and publish it. */
	void cpuload();

	/* Stack check only available on Nuttx */
#if defined(__PX4_NUTTX)
	/* Calculate stack usage */
	void stack_usage();

	int _stack_task_index{0};

	uORB::Publication<task_stack_info_s> _task_stack_info_pub{ORB_ID(task_stack_info)};
#endif
	uORB::Publication<vehicle_cpuload_s> _cpuload_pub {ORB_ID(vehicle_cpuload)};

#if defined(__PX4_LINUX)
	FILE *_proc_fd = nullptr;
	/* calculate usage directly from clock ticks on Linux */
	clock_t _last_total_time_stamp{};
	clock_t _last_spent_time_stamp{};
#elif defined(__PX4_NUTTX)
	hrt_abstime _last_idle_time {0};
	hrt_abstime _last_idle_time_sample{0};
#endif

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::SYS_STCK_EN>) _param_sys_stck_en
	)
};

} // namespace load_mon
