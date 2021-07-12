/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
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

#ifdef CONFIG_SCHED_INSTRUMENTATION

#include <sched.h>
#include <stdint.h>
#include <stdbool.h>

struct system_load_taskinfo_s {
	uint64_t total_runtime{0};		///< Runtime since start (start_time - total_runtime)/(start_time - current_time) = load
	uint64_t curr_start_time{0};		///< Start time of the current scheduling slot
	struct tcb_s *tcb {nullptr};
	bool valid{false};			///< Task is currently active / valid
};

struct system_load_s {
	uint64_t start_time{0};
	system_load_taskinfo_s tasks[CONFIG_FS_PROCFS_MAX_TASKS] {};
	int total_count{0};
	int running_count{0};
	bool initialized{false};
};

__BEGIN_DECLS

__EXPORT extern struct system_load_s system_load;

__EXPORT void cpuload_initialize_once(void);

__EXPORT void cpuload_monitor_start(void);
__EXPORT void cpuload_monitor_stop(void);

__END_DECLS

#endif
