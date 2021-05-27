/****************************************************************************
 *
 *   Copyright (C) 2020 Technology Innovation Institute. All rights reserved.
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
 * @file kernel_modules.c
 *
 * Provide the kernel side module loading interface
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/sem.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/tasks.h>
#include "board_config.h"

#include <stdint.h>
#include <errno.h>

#include <nuttx/lib/builtin.h>

#include <NuttX/kernel_builtin/kernel_builtin_prototypes.h>

FAR const struct builtin_s g_kernel_builtins[] = {
#include <NuttX/kernel_builtin/kernel_builtins.h>
};

const int g_n_kernel_builtins = sizeof(g_kernel_builtins) / sizeof(struct builtin_s);


/* internal functions */

__EXPORT int launch_builtin_module(int argc, char *argv[])
{
	int i;
	FAR const struct builtin_s *builtin = NULL;

	for (i = 0; i < g_n_kernel_builtins; i++) {
		if (!strcmp(g_kernel_builtins[i].name, argv[0])) {
			builtin = &g_kernel_builtins[i];
			break;
		}
	}

	if (builtin) {
		/* This is running in the userspace thread, created by nsh, and
		   called via boardctl. Just call the main directly */
		builtin->main(argc, argv);
		return OK;
	}

	return ENOENT;
}
