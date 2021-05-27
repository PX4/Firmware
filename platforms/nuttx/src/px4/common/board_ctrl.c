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
 * @file board_ctrl.c
 *
 * Provide a kernel-userspace boardctl_ioctl interface
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/sem.h>
#include <px4_platform_common/defines.h>
#include <px4_platform/board_ctrl.h>
#include "board_config.h"

#include <stdint.h>
#include <errno.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_orb_dev.h>

struct {
	unsigned base;
	ioctl_ptr_t ioctl_func;
} ioctl_ptrs[] = {
	{0, NULL},
	{0, NULL},
	{0, NULL},
	{0, NULL}
};
#define MAX_IOCTL_PTRS (sizeof(ioctl_ptrs)/sizeof(ioctl_ptrs[0]))

int launch_builtin_module(int argc, char *argv[]);

/* internal functions */

int px4_register_boardct_ioctl(unsigned base, ioctl_ptr_t func)
{
	unsigned i;
	int ret = PX4_ERROR;

	for (i = 0; i < MAX_IOCTL_PTRS; i++) {
		if (ioctl_ptrs[i].base == 0) {
			ioctl_ptrs[i].base = base;
			ioctl_ptrs[i].ioctl_func = func;
			ret = PX4_OK;
			break;
		}
	}

	return ret;
}

/************************************************************************************
 * Name: board_ioctl
 *
 * Description:
 *   px4 platform layer kernel-userspace interfaces
 *
 ************************************************************************************/

int
board_ioctl(unsigned int cmd, uintptr_t arg)
{
	px4_boardctl_t *kcmd = (px4_boardctl_t *)arg;
	unsigned i;

	if (cmd == PX4_KERNEL_CMD) {
		/* Launch module on kernel side */
		kcmd->ret = launch_builtin_module(kcmd->argc, kcmd->argv);

	} else {
		/* Run some other registered ioctl */
		for (i = 0; i < MAX_IOCTL_PTRS; i++) {
			if ((cmd & 0xFF00) == ioctl_ptrs[i].base) {
				return ioctl_ptrs[i].ioctl_func(cmd, arg);
			}
		}

		return -EINVAL;
	}

	return OK;
}
