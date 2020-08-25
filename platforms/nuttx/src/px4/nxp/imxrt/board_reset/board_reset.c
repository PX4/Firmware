/****************************************************************************
 *
 *   Copyright (C) 2017 PX4 Development Team. All rights reserved.
 *   Author: @author David Sidrane <david_s5@nscdg.com>
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
 * @file board_reset.c
 * Implementation of IMXRT based Board RESET API
 */

#include <px4_platform_common/px4_config.h>
#include <errno.h>
#include <nuttx/board.h>
#include <up_arch.h>
#include <hardware/imxrt_snvs.h>

#define PX4_IMXRT_RTC_REBOOT_REG 3 // Must be common with bootloader and:

#if CONFIG_IMXRT_RTC_MAGIC_REG == PX4_IMXRT_RTC_REBOOT_REG
#  error CONFIG_IMXRT_RTC_MAGIC_REG can nt have the save value as PX4_IMXRT_RTC_REBOOT_REG
#endif

int board_reset(int status)
{
	up_systemreset();
	return 0;
}

int board_set_bootload_mode(board_reset_e mode)
{
	uint32_t regvalue = 0;

	switch (mode) {
	case board_reset_normal:
	case board_reset_extended:
		break;

	case board_reset_enter_bootloader:
		regvalue = 0xb007b007;
		break;

	default:
		return -EINVAL;
	}

	putreg32(regvalue, IMXRT_SNVS_LPGPR(PX4_IMXRT_RTC_REBOOT_REG));
	return OK;
}


void board_system_reset(int status)
{
#if defined(BOARD_HAS_ON_RESET)
	board_on_reset(status);
#endif

#ifdef CONFIG_BOARDCTL_RESET
	board_reset(status);
#endif

	while (1);
}
