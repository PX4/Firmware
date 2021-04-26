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
 * @file board_reset.cpp
 * Implementation of STM32 based Board RESET API
 */

#include <px4_platform_common/px4_config.h>
#include <systemlib/px4_macros.h>
#include <errno.h>
#include <stm32_pwr.h>
#include <stm32_rtc.h>
#include <nuttx/board.h>

#ifdef CONFIG_BOARDCTL_RESET

/****************************************************************************
 * Name: board_configure_reset
 *
 * Description:
 *   Configures the device that maintains the state shared by the
 *   application and boot loader. This is usually an RTC.
 *
 * Input Parameters:
 *   mode  - The type of reset. See reset_mode_e
 *
 * Returned Value:
 *   0 for Success
 *   1 if invalid argument
 *
 ****************************************************************************/

static const uint32_t modes[] = {
	/*                                      to  tb   */
	/* BOARD_RESET_MODE_CLEAR                5   y   */  0,
	/* BOARD_RESET_MODE_BOOT_TO_BL           0   n   */  0xb0070001,
	/* BOARD_RESET_MODE_BOOT_TO_VALID_APP    0   y   */  0xb0070002,
	/* BOARD_RESET_MODE_CAN_BL               10  n   */  0xb0080000,
	/* BOARD_RESET_MODE_RTC_BOOT_FWOK        0   n   */  0xb0093a26
};

int board_configure_reset(reset_mode_e mode, uint32_t arg)
{
	int rv = -1;

	if (mode < arraySize(modes)) {

		stm32_pwr_enablebkp(true);

		arg = mode == BOARD_RESET_MODE_CAN_BL ? arg & ~0xff : 0;

		// Check if we can to use the new register definition

#ifndef STM32_RTC_BK0R
		*(uint32_t *)STM32_BKP_BASE = modes[mode] | arg;
#else
		*(uint32_t *)STM32_RTC_BK0R = modes[mode] | arg;
#endif
		stm32_pwr_enablebkp(false);
		rv = OK;
	}

	return rv;
}

/****************************************************************************
 * Name: board_reset
 *
 * Description:
 *   Reset board.  Support for this function is required by board-level
 *   logic if CONFIG_BOARDCTL_RESET is selected.
 *
 * Input Parameters:
 *   status - Status information provided with the reset event.  This
 *            meaning of this status information is board-specific.  If not
 *            used by a board, the value zero may be provided in calls to
 *            board_reset().
 *
 * Returned Value:
 *   If this function returns, then it was not possible to power-off the
 *   board due to some constraints.  The return value int this case is a
 *   board-specific reason for the failure to shutdown.
 *
 ****************************************************************************/

int board_reset(int status)
{
	if (status == 1) {
		board_configure_reset(BOARD_RESET_MODE_BOOT_TO_BL, 0);
	}

#if defined(BOARD_HAS_ON_RESET)
	board_on_reset(status);
#endif

	up_systemreset();
	return 0;
}

#endif /* CONFIG_BOARDCTL_RESET */

#if defined(SUPPORT_ALT_CAN_BOOTLOADER)
/****************************************************************************
 * Name: board_booted_by_px4
 *
 * Description:
 *   Determines if the the boot loader was PX4
 *
 * Input Parameters:
 *   none
 *
 * Returned Value:
 *   true if booted byt a PX4 bootloader.
 *
 ****************************************************************************/
bool board_booted_by_px4(void)
{
	uint32_t *vectors = (uint32_t *) STM32_FLASH_BASE;

	/* Nuttx uses common vector */

	return (vectors[2] == vectors[3]) && (vectors[4] == vectors[5]);
}
#endif
