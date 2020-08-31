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
 * the SD Host Controller (SDHC) signals of the MCU. This slot will accept micro
 * format SD memory cards. The SD card detect pin (PTE6) is an open switch that
 * shorts with VDD when card is inserted.
 *
 *   ------------ ------------- --------
 *    SD Card Slot Board Signal  K66 Pin
 *    ------------ ------------- --------
 *    DAT0         SDHC0_D0      PTE1
 *    DAT1         SDHC0_D1      PTE0
 *    DAT2         SDHC0_D2      PTE5
 *    CD/DAT3      SDHC0_D3      PTE4
 *    CMD          SDHC0_CMD     PTE3
 *    CLK          SDHC0_DCLK    PTE2
 *    SWITCH       D_CARD_DETECT PTE6
 *    ------------ ------------- --------
 *
 * There is no Write Protect pin available to the K66.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <px4_platform_common/px4_config.h>

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>

#include "kinetis.h"

#include "board_config.h"

#ifdef CONFIG_KINETIS_SDHC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This structure holds static information unique to one SDHC peripheral */

struct fmuk66_sdhc_state_s {
	struct sdio_dev_s *sdhc;    /* R/W device handle */
	bool inserted;              /* TRUE: card is inserted */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* HSCMI device state */

static struct fmuk66_sdhc_state_s g_sdhc;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(GPIO_SD_CARDDETECT)
/****************************************************************************
 * Name: fmuk66_mediachange
 ****************************************************************************/

static void fmuk66_mediachange(struct fmuk66_sdhc_state_s *sdhc)
{
	bool inserted;

	/* Get the current value of the card detect pin.  This pin is pulled up on
	 * board.  So low means that a card is present.
	 */

	inserted = !kinetis_gpioread(GPIO_SD_CARDDETECT);
	mcinfo("inserted: %s\n", inserted ? "Yes" : "No");

	/* Has the pin changed state? */

	if (inserted != sdhc->inserted) {
		mcinfo("Media change: %d->%d\n",  sdhc->inserted, inserted);

		/* Yes.. perform the appropriate action (this might need some debounce). */

		sdhc->inserted = inserted;
		sdhc_mediachange(sdhc->sdhc, inserted);

#ifdef CONFIG_FMUK66_SDHC_AUTOMOUNT
		/* Let the automounter know about the insertion event */

		fmuk66_automount_event(fmuk66_cardinserted());
#endif
	}
}

/****************************************************************************
 * Name: fmuk66_cdinterrupt
 ****************************************************************************/

static int fmuk66_cdinterrupt(int irq, FAR void *context, FAR void *args)
{
	/* All of the work is done by fmuk66_mediachange() */

	fmuk66_mediachange((struct fmuk66_sdhc_state_s *) args);
	return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fmuk66_sdhc_initialize
 *
 * Description:
 *   Inititialize the SDHC SD card slot
 *
 ****************************************************************************/

int fmuk66_sdhc_initialize(void)
{
	int ret;
	struct fmuk66_sdhc_state_s   *sdhc = &g_sdhc;
	/* Configure GPIO pins */

	VDD_3V3_SD_CARD_EN(true);

#if defined(GPIO_SD_CARDDETECT)
	kinetis_pinconfig(GPIO_SD_CARDDETECT);

	/* Attached the card detect interrupt (but don't enable it yet) */

	kinetis_pinirqattach(GPIO_SD_CARDDETECT, fmuk66_cdinterrupt, sdhc);
#endif
	/* Configure the write protect GPIO -- None */

	/* Mount the SDHC-based MMC/SD block driver */
	/* First, get an instance of the SDHC interface */

	mcinfo("Initializing SDHC slot %d\n", CONFIG_NSH_MMCSDSLOTNO);

	sdhc->sdhc = sdhc_initialize(CONFIG_NSH_MMCSDSLOTNO);

	if (!sdhc->sdhc) {
		mcerr("ERROR: Failed to initialize SDHC slot %d\n", CONFIG_NSH_MMCSDSLOTNO);
		return -ENODEV;
	}

//   Testing done on SanDISK HC all failed sd_bench with Drive/Slew other than default and _PIN_OUTPUT_FAST|_PIN_OUTPUT_HIGHDRIVE
//	_PIN_OUTPUT_FAST|_PIN_OUTPUT_HIGHDRIVE    Square noisy, pass SanDISK HC
//  _PIN_OUTPUT_FAST|_PIN_OUTPUT_LOWDRIVE     Square noisy, pass SanDISK HC
//  _PIN_OUTPUT_HIGHDRIVE|_PIN_OUTPUT_SLOW    sinusoidal fail SanDISK HC pass SanDISK HC1
//  _PIN_OUTPUT_LOWDRIVE|_PIN_OUTPUT_SLOW     sinusoidal fail SanDISK HC pass SanDISK HC1
//                       _PIN_OUTPUT_SLOW     sinusoidal fail SanDISK HC pass SanDISK HC1

	// This up dating of the driver setting is for EMI issue with GPS and FCC
	// With this setting the clock is sinusoidal N.B. sd_bench fails on SanDISK HC, but
	// Passes SanDISK **HC1** - use HC1 or Kingston cards!

	kinetis_pinconfig(PIN_SDHC0_DCLK | _PIN_OUTPUT_HIGHDRIVE | _PIN_OUTPUT_SLOW);

	/* Now bind the SDHC interface to the MMC/SD driver */

	mcinfo("Bind SDHC to the MMC/SD driver, minor=%d\n", CONFIG_NSH_MMCSDMINOR);

	ret = mmcsd_slotinitialize(CONFIG_NSH_MMCSDMINOR, sdhc->sdhc);

	if (ret != OK) {
		syslog(LOG_ERR, "ERROR: Failed to bind SDHC to the MMC/SD driver: %d\n", ret);
		return ret;
	}

	syslog(LOG_ERR, "Successfully bound SDHC to the MMC/SD driver\n");

#if defined(GPIO_SD_CARDDETECT)
	/* Handle the initial card state */

	fmuk66_mediachange(sdhc);

	/* Enable CD interrupts to handle subsequent media changes */

	kinetis_pinirqenable(GPIO_SD_CARDDETECT);
#else
	sdhc_mediachange(sdhc->sdhc, true);
#endif
	return OK;
}

/****************************************************************************
 * Name: fmuk66_cardinserted
 *
 * Description:
 *   Check if a card is inserted into the SDHC slot
 *
 ****************************************************************************/

#ifdef HAVE_AUTOMOUNTER
bool fmuk66_cardinserted(void)
{
	bool inserted;

	/* Get the current value of the card detect pin.  This pin is pulled up on
	 * board.  So low means that a card is present.
	 */

	inserted = !kinetis_gpioread(GPIO_SD_CARDDETECT);
	mcinfo("inserted: %s\n", inserted ? "Yes" : "No");
	return inserted;
}
#endif

/****************************************************************************
 * Name: fmuk66_writeprotected
 *
 * Description:
 *   Check if a card is inserted into the SDHC slot
 *
 ****************************************************************************/

#ifdef HAVE_AUTOMOUNTER
bool fmuk66_writeprotected(void)
{
	/* There are no write protect pins */

	return false;
}
#endif

#endif /* HAVE_MMCSD */
