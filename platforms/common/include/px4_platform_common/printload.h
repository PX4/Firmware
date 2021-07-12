/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file printload.h
 *
 * Print the current system load.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#pragma once

#include <px4_platform_common/px4_config.h>

#include <stdint.h>

#ifndef CONFIG_FS_PROCFS_MAX_TASKS
#define CONFIG_FS_PROCFS_MAX_TASKS 64
#endif

struct print_load_s {
	uint64_t total_user_time{0};

	int running_count{0};
	int blocked_count{0};

	uint64_t new_time{0};
	uint64_t interval_start_time{0};
	uint64_t last_times[CONFIG_FS_PROCFS_MAX_TASKS] {};
	float interval_time_us{0.f};
};

__BEGIN_DECLS

__EXPORT void init_print_load(struct print_load_s *s);

__EXPORT void print_load(int fd, struct print_load_s *print_state);


typedef void (*print_load_callback_f)(void *user);

/**
 * Print load to a buffer, and call cb after each written line (buffer will not include '\n')
 */
void print_load_buffer(char *buffer, int buffer_length, print_load_callback_f cb, void *user,
		       struct print_load_s *print_state);

__END_DECLS
