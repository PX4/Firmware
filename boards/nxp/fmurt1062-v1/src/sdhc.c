/****************************************************************************
 *
 *   Copyright (C) 2016-2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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
/* A micro Secure Digital (SD) card slot is available on the board connected to
 * the SD Host Controller (USDHC1) signals of the MCU. This slot will accept
 * micro format SD memory cards.
 *
 *   ------------ ------------- --------
 *    SD Card Slot Board Signal  IMXRT Pin
 *    ------------ ------------- --------
 *    DAT0         USDHC1_DATA0  GPIO_SD_B0_02
 *    DAT1         USDHC1_DATA1  GPIO_SD_B0_03
 *    DAT2         USDHC1_DATA2  GPIO_SD_B0_04
 *    CD/DAT3      USDHC1_DATA3  GPIO_SD_B0_05
 *    CMD          USDHC1_CMD    GPIO_SD_B0_00
 *    CLK          USDHC1_CLK    GPIO_SD_B0_01
 *    CD           USDHC1_CD     GPIO_B1_12
 *    ------------ ------------- --------
 *
 * There are no Write Protect available to the IMXRT.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <px4_log.h>

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>

#include "chip.h"
#include "imxrt_usdhc.h"

#include "board_config.h"

#ifdef CONFIG_IMXRT_USDHC
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/
/****************************************************************************
 * Private Data
 ****************************************************************************/
/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fmurt1062_usdhc_initialize
 *
 * Description:
 *   Inititialize the SDHC SD card slot
 *
 ****************************************************************************/

int fmurt1062_usdhc_initialize(void)
{
	int ret;

	/* Mount the SDHC-based MMC/SD block driver */
	/* First, get an instance of the SDHC interface */

	struct sdio_dev_s *sdhc = imxrt_usdhc_initialize(CONFIG_NSH_MMCSDSLOTNO);

	if (!sdhc) {
		PX4_ERR("ERROR: Failed to initialize SDHC slot %d\n", CONFIG_NSH_MMCSDSLOTNO);
		return -ENODEV;
	}

	/* Now bind the SDHC interface to the MMC/SD driver */

	ret = mmcsd_slotinitialize(CONFIG_NSH_MMCSDMINOR, sdhc);

	if (ret != OK) {
		PX4_ERR("ERROR: Failed to bind SDHC to the MMC/SD driver: %d\n", ret);
		return ret;
	}

	syslog(LOG_INFO, "Successfully bound SDHC to the MMC/SD driver\n");

	return OK;
}
#endif /* CONFIG_IMXRT_USDHC */
