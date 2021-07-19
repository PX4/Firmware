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

#include "LoadMon.hpp"

#if defined(__PX4_NUTTX)
// if free stack space falls below this, print a warning
#if defined(CONFIG_ARMV7M_STACKCHECK)
static constexpr unsigned STACK_LOW_WARNING_THRESHOLD = 100;
#else
static constexpr unsigned STACK_LOW_WARNING_THRESHOLD = 300;
#endif

static constexpr unsigned FDS_LOW_WARNING_THRESHOLD = 2; ///< if free file descriptors fall below this, print a warning
#endif

using namespace time_literals;

namespace load_mon
{

LoadMon::LoadMon() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

LoadMon::~LoadMon()
{
	ScheduleClear();
	perf_free(_cycle_perf);
}

int LoadMon::task_spawn(int argc, char *argv[])
{
	LoadMon *obj = new LoadMon();

	if (!obj) {
		PX4_ERR("alloc failed");
		return -1;
	}

	_object.store(obj);
	_task_id = task_id_is_work_queue;

	/* Schedule a cycle to start things. */
	obj->start();

	return 0;
}

void LoadMon::start()
{
	ScheduleOnInterval(500_ms); // 2 Hz
}

void LoadMon::Run()
{
#if defined (__PX4_LINUX)

	if (_proc_fd == nullptr) {	// init fd
		_proc_fd = fopen("/proc/meminfo", "r");

		if (_proc_fd == nullptr) {
			PX4_ERR("Failed to open /proc/meminfo");
		}
	}

#endif
	perf_begin(_cycle_perf);

	cpuload();

#if defined(__PX4_NUTTX)

	if (_param_sys_stck_en.get()) {
		stack_usage();
	}

#endif

	if (should_exit()) {
		ScheduleClear();
#if defined (__PX4_LINUX)
		fclose(_proc_fd);
#endif
		exit_and_cleanup();
	}

	perf_end(_cycle_perf);
}

void LoadMon::cpuload()
{
#if defined(__PX4_LINUX)
	tms spent_time_stamp_struct;
	clock_t total_time_stamp = times(&spent_time_stamp_struct);
	clock_t spent_time_stamp = spent_time_stamp_struct.tms_utime + spent_time_stamp_struct.tms_stime;

	if (_last_total_time_stamp == 0 || _last_spent_time_stamp == 0) {
		// Just get the time in the first iteration */
		_last_total_time_stamp = total_time_stamp;
		_last_spent_time_stamp = spent_time_stamp;
		return;
	}

	// compute system load
	const float interval = total_time_stamp - _last_total_time_stamp;
	const float interval_spent_time = spent_time_stamp - _last_spent_time_stamp;
#elif defined(__PX4_NUTTX)

	if (_last_idle_time == 0) {
		// Just get the time in the first iteration */
		_last_idle_time = system_load.tasks[0].total_runtime;
		_last_idle_time_sample = hrt_absolute_time();
		return;
	}

	irqstate_t irqstate = enter_critical_section();
	const hrt_abstime now = hrt_absolute_time();
	const hrt_abstime total_runtime = system_load.tasks[0].total_runtime;
	leave_critical_section(irqstate);

	// compute system load
	const float interval = now - _last_idle_time_sample;
	const float interval_idletime = total_runtime - _last_idle_time;
#endif

	cpuload_s cpuload{};
#if defined(__PX4_LINUX)
	/* following calculation is based on free(1)
	 * https://gitlab.com/procps-ng/procps/-/blob/master/proc/sysinfo.c */
	char line[256];
	int32_t kb_main_total = -1;
	int32_t kb_main_free = -1;
	int32_t kb_page_cache = -1;
	int32_t kb_slab_reclaimable = -1;
	int32_t kb_main_buffers = -1;
	int parsedCount = 0;

	if (_proc_fd != nullptr) {
		while (fgets(line, sizeof(line), _proc_fd)) {
			if (sscanf(line, "MemTotal: %d kB", &kb_main_total) == 1) {
				++parsedCount;
				continue;
			}

			if (sscanf(line, "MemFree: %d kB", &kb_main_free) == 1) {
				++parsedCount;
				continue;
			}

			if (sscanf(line, "Cached: %d kB", &kb_page_cache) == 1) {
				++parsedCount;
				continue;
			}

			if (sscanf(line, "SReclaimable: %d kB", &kb_slab_reclaimable) == 1) {
				++parsedCount;
				continue;
			}

			if (sscanf(line, "Buffers: %d kB", &kb_main_buffers) == 1) {
				++parsedCount;
				continue;
			}
		}

		fseek(_proc_fd, 0, SEEK_END);

		if (parsedCount == 5) {
			int32_t kb_main_cached = kb_page_cache + kb_slab_reclaimable;
			int32_t mem_used = kb_main_total - kb_main_free - kb_main_cached - kb_main_buffers;

			if (mem_used < 0) {
				mem_used = kb_main_total - kb_main_free;
			}

			cpuload.ram_usage = (float)mem_used / kb_main_total;

		} else {
			PX4_ERR("Could not parse /proc/meminfo");
			cpuload.ram_usage = -1;
		}

	} else {
		cpuload.ram_usage = -1;
	}

	cpuload.load = interval_spent_time / interval;
#elif defined(__PX4_NUTTX)
	// get ram usage
	struct mallinfo mem = mallinfo();
	cpuload.ram_usage = (float)mem.uordblks / mem.arena;
	cpuload.load = 1.f - interval_idletime / interval;
#endif
	cpuload.timestamp = hrt_absolute_time();

	_cpuload_pub.publish(cpuload);

	// store for next iteration
#if defined(__PX4_LINUX)
	_last_total_time_stamp = total_time_stamp;
	_last_spent_time_stamp = spent_time_stamp;
#elif defined(__PX4_NUTTX)
	_last_idle_time = total_runtime;
	_last_idle_time_sample = now;
#endif
}

#if defined(__PX4_NUTTX)
void LoadMon::stack_usage()
{
	unsigned stack_free = 0;

	bool checked_task = false;

	task_stack_info_s task_stack_info{};
	static_assert(sizeof(task_stack_info.task_name) == CONFIG_TASK_NAME_SIZE,
		      "task_stack_info.task_name must match NuttX CONFIG_TASK_NAME_SIZE");

	sched_lock();

	if (system_load.tasks[_stack_task_index].valid && (system_load.tasks[_stack_task_index].tcb->pid > 0)) {

		stack_free = up_check_tcbstack_remain(system_load.tasks[_stack_task_index].tcb);

		strncpy((char *)task_stack_info.task_name, system_load.tasks[_stack_task_index].tcb->name, CONFIG_TASK_NAME_SIZE - 1);
		task_stack_info.task_name[CONFIG_TASK_NAME_SIZE - 1] = '\0';

		checked_task = true;

#if CONFIG_NFILE_DESCRIPTORS_PER_BLOCK > 0
		unsigned int tcb_num_used_fds = 0; // number of used file descriptors
		struct filelist *filelist = &system_load.tasks[_stack_task_index].tcb->group->tg_filelist;

		for (int fdr = 0; fdr < filelist->fl_rows; fdr++) {
			for (int fdc = 0; fdc < CONFIG_NFILE_DESCRIPTORS_PER_BLOCK; fdc++) {
				if (filelist->fl_files[fdr][fdc].f_inode) {
					++tcb_num_used_fds;
				}
			}
		}

#endif // CONFIG_NFILE_DESCRIPTORS_PER_BLOCK
	}

	sched_unlock();

	if (checked_task) {
		task_stack_info.stack_free = stack_free;
		task_stack_info.timestamp = hrt_absolute_time();

		_task_stack_info_pub.publish(task_stack_info);

		// Found task low on stack, report and exit. Continue here in next cycle.
		if (stack_free < STACK_LOW_WARNING_THRESHOLD) {
			PX4_WARN("%s low on stack! (%i bytes left)", task_stack_info.task_name, stack_free);
		}
	}

	// Continue after last checked task next cycle
	_stack_task_index = (_stack_task_index + 1) % CONFIG_FS_PROCFS_MAX_TASKS;
}
#endif

int LoadMon::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Background process running periodically on the low priority work queue to calculate the CPU load and RAM
usage and publish the `cpuload` topic.

On NuttX it also checks the stack usage of each process and if it falls below 300 bytes, a warning is output,
which will also appear in the log file.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("load_mon", "system");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the background task");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int load_mon_main(int argc, char *argv[])
{
	return LoadMon::main(argc, argv);
}

} // namespace load_mon
