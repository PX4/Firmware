/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *       Author: David Sidrane <david.sidrane@nscdg.com>
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

#include <nuttx/config.h>
#include <systemlib/px4_macros.h>

#include "boot_config.h"
#include "flash.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <limits.h>

#include "hardware/s32k1xx_ftfc.h"


#define S32K1XX_PROGMEM_BLOCK_SECTOR_SIZE 4096
#define S32K1XX_FLASH_BASE_ADDRESS        0
#define S32K1XX_PROGMEM_PAGE_SIZE         8

CCASSERT(S32K1XX_PROGMEM_PAGE_SIZE == LATER_FLAHSED_WORDS *sizeof(uint32_t));



typedef union fccob_flash_addr_t {
	uint32_t addr;
	struct {
		uint8_t fccob3;
		uint8_t fccob2;
		uint8_t fccob1;
		uint8_t pad;
	} fccobs;
} fccob_flash_addr_t;


static uint8_t zero_dirty = 0xff;

ssize_t up_progmem_getpage(size_t addr);

locate_code(".ramfunc")
static inline void wait_ftfc_ready(void)
{
	while ((getreg8(S32K1XX_FTFC_FSTAT) & FTTC_FSTAT_CCIF) == 0) {
		/* Busy */
	}
}

locate_code(".ramfunc")
static uint32_t execute_ftfc_command(void)
{
	uint8_t regval;
	uint32_t retval;

	/* Clear CCIF to launch command */

	regval = getreg8(S32K1XX_FTFC_FSTAT);
	regval |= FTTC_FSTAT_CCIF;

	irqstate_t flags;

	flags = enter_critical_section();

	putreg8(regval, S32K1XX_FTFC_FSTAT);

	wait_ftfc_ready();

	leave_critical_section(flags);

	retval = getreg8(S32K1XX_FTFC_FSTAT);

	if (retval & (FTTC_FSTAT_MGSTAT0 | FTTC_FSTAT_FPVIOL |
		      FTTC_FSTAT_ACCERR | FTTC_FSTAT_RDCOLERR)) {
		return retval; /* Error has occured */

	} else {
		return 0; /* success */
	}
}

locate_code(".ramfunc")
ssize_t up_progmem_getpage(size_t addr)
{
	return addr / S32K1XX_PROGMEM_BLOCK_SECTOR_SIZE;
}


locate_code(".ramfunc")
ssize_t up_progmem_eraseblock(size_t block)
{
	static bool once = false;

	if (!once) {
		once = true;
		zero_dirty = 0xff;
	}

	fccob_flash_addr_t dest;
	dest.addr = (block * S32K1XX_PROGMEM_BLOCK_SECTOR_SIZE) + S32K1XX_FLASH_BASE_ADDRESS;

	/* Clear FSTAT error bits */

	putreg8(FTTC_FSTAT_FPVIOL | FTTC_FSTAT_ACCERR | FTTC_FSTAT_RDCOLERR,
		S32K1XX_FTFC_FSTAT);

	/* Set FTFC command */

	putreg8(S32K1XX_FTFC_ERASE_SECTOR, S32K1XX_FTFC_FCCOB0);

	/* Destination address of sector to erase */

	putreg8(dest.fccobs.fccob1, S32K1XX_FTFC_FCCOB1);
	putreg8(dest.fccobs.fccob2, S32K1XX_FTFC_FCCOB2);
	putreg8(dest.fccobs.fccob3, S32K1XX_FTFC_FCCOB3);

	if (execute_ftfc_command() & (FTTC_FSTAT_MGSTAT0 | FTTC_FSTAT_FPVIOL |
				      FTTC_FSTAT_ACCERR | FTTC_FSTAT_RDCOLERR)) {
		return -EIO; /* Error has occured */
	}

	return (ssize_t)S32K1XX_PROGMEM_BLOCK_SECTOR_SIZE;
}


locate_code(".ramfunc")
ssize_t up_progmem_write(size_t addr, FAR const void *buf, size_t count)
{
	fccob_flash_addr_t dest;
	uint8_t *src = (uint8_t *)buf;
	ssize_t cashed = 0;


	if (addr == APPLICATION_LOAD_ADDRESS) {

		/* On the first pass we will not write the first 8 bytes, and leave them erased. */

		if (zero_dirty == 0xff) {
			cashed = S32K1XX_PROGMEM_PAGE_SIZE;
			zero_dirty    = 0;
			addr += S32K1XX_PROGMEM_PAGE_SIZE;
			src += S32K1XX_PROGMEM_PAGE_SIZE;
			count -= S32K1XX_PROGMEM_PAGE_SIZE;

		} else {

			/* On the second pass we will write the first 8 bytes. */

			cashed = count - S32K1XX_PROGMEM_PAGE_SIZE;
			count = S32K1XX_PROGMEM_PAGE_SIZE;
			zero_dirty = 0xff;
		}
	}

	if (count % S32K1XX_PROGMEM_PAGE_SIZE != 0) {
		return -EINVAL;
	}

	dest.addr = addr;

	for (size_t i = 0; i < count / S32K1XX_PROGMEM_PAGE_SIZE ; i++) {
		wait_ftfc_ready();

		/* Clear FSTAT error bits */

		putreg8(FTTC_FSTAT_FPVIOL | FTTC_FSTAT_ACCERR | FTTC_FSTAT_RDCOLERR,
			S32K1XX_FTFC_FSTAT);

		/* Set FTFC command */

		putreg8(S32K1XX_FTFC_PROGRAM_PHRASE, S32K1XX_FTFC_FCCOB0);

		/* Destination address */

		putreg8(dest.fccobs.fccob1, S32K1XX_FTFC_FCCOB1);
		putreg8(dest.fccobs.fccob2, S32K1XX_FTFC_FCCOB2);
		putreg8(dest.fccobs.fccob3, S32K1XX_FTFC_FCCOB3);

		/* Write data */

		for (int j = 0; j < S32K1XX_PROGMEM_PAGE_SIZE; j++) {
			putreg8(src[j], S32K1XX_FTFC_FCCOB7 + j);
		}

		if (execute_ftfc_command() & (FTTC_FSTAT_MGSTAT0 | FTTC_FSTAT_FPVIOL |
					      FTTC_FSTAT_ACCERR | FTTC_FSTAT_RDCOLERR)) {
			return -EIO; /* Error has occured */
		}

		dest.addr = dest.addr + S32K1XX_PROGMEM_PAGE_SIZE;
		src = src + S32K1XX_PROGMEM_PAGE_SIZE;
	}

	return count + cashed;
}
