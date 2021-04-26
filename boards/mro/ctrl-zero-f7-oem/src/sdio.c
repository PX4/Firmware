/****************************************************************************
 *
 *   Copyright (C) 2014, 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <board_config.h>

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>

#include "chip.h"
#include "board_config.h"
#include "stm32_gpio.h"
#include "stm32_sdmmc.h"

#ifdef CONFIG_MMCSD


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Card detections requires card support and a card detection GPIO */

#define HAVE_NCD   1
#if !defined(GPIO_SDMMC1_NCD)
#  undef HAVE_NCD
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct sdio_dev_s *sdio_dev;
#ifdef HAVE_NCD
static bool g_sd_inserted = 0xff; /* Impossible value */
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_ncd_interrupt
 *
 * Description:
 *   Card detect interrupt handler.
 *
 ****************************************************************************/

#ifdef HAVE_NCD
static int stm32_ncd_interrupt(int irq, FAR void *context)
{
	bool present = !stm32_gpioread(GPIO_SDMMC1_NCD);

	if (sdio_dev && present != g_sd_inserted) {
		sdio_mediachange(sdio_dev, present);
		g_sd_inserted = present;
	}

	return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_sdio_initialize
 *
 * Description:
 *   Initialize SDIO-based MMC/SD card support
 *
 ****************************************************************************/

int stm32_sdio_initialize(void)
{
	int ret;

#ifdef HAVE_NCD
	/* Card detect */
	bool cd_status;

	/* Configure the card detect GPIO */
	stm32_configgpio(GPIO_SDMMC1_NCD);

	/* Register an interrupt handler for the card detect pin */
	stm32_gpiosetevent(GPIO_SDMMC1_NCD, true, true, true, stm32_ncd_interrupt);
#endif

	/* Mount the SDIO-based MMC/SD block driver */
	/* First, get an instance of the SDIO interface */
	finfo("Initializing SDIO slot %d\n", SDIO_SLOTNO);

	sdio_dev = sdio_initialize(SDIO_SLOTNO);

	if (!sdio_dev) {
		syslog(LOG_ERR, "[boot] Failed to initialize SDIO slot %d\n", SDIO_SLOTNO);
		return -ENODEV;
	}

	/* Now bind the SDIO interface to the MMC/SD driver */

	finfo("Bind SDIO to the MMC/SD driver, minor=%d\n", SDIO_MINOR);

	ret = mmcsd_slotinitialize(SDIO_MINOR, sdio_dev);

	if (ret != OK) {
		syslog(LOG_ERR, "[boot] Failed to bind SDIO to the MMC/SD driver: %d\n", ret);
		return ret;
	}

	finfo("Successfully bound SDIO to the MMC/SD driver\n");

#ifdef HAVE_NCD
	/* Use SD card detect pin to check if a card is g_sd_inserted */

	cd_status = !stm32_gpioread(GPIO_SDMMC1_NCD);
	finfo("Card detect : %d\n", cd_status);

	sdio_mediachange(sdio_dev, cd_status);
#else
	/* Assume that the SD card is inserted.  What choice do we have? */

	sdio_mediachange(sdio_dev, true);
#endif

	return OK;
}

#endif /* CONFIG_MMCSD */
