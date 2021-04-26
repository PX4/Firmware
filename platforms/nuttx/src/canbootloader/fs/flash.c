/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *       Author: Ben Dyer <ben_dyer@mac.com>
 *               Pavel Kirienko <pavel.kirienko@zubax.com>
 *               David Sidrane <david_s5@nscdg.com>
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

#include "boot_config.h"

#include <stdint.h>
#include <stdlib.h>

/* Force the interface to be defined. Even if the NuttX config is not
 * providing the driver, but the the bootloader is.
 */

#if !defined(CONFIG_ARCH_HAVE_PROGMEM)
#  define CONFIG_ARCH_HAVE_PROGMEM
#endif

#include <nuttx/progmem.h>

#include "flash.h"
#include "blsched.h"

/****************************************************************************
 * Name: bl_flash_erase
 *
 * Description:
 *   This function erases the flash starting at address and ending at
 *   address + nbytes.
 *
 * Input Parameters:
 *   address - A word-aligned address within the first page of flash to erase
 *   nbytes - The number of bytes to erase, rounding up to the next page.
 *
 *
 *
 * Returned value:
 *   On success FLASH_OK On Error one of the flash_error_t
 *
 ****************************************************************************/

flash_error_t bl_flash_erase(size_t address, size_t nbytes)
{
	/*
	 * FIXME (?): this may take a long time, and while flash is being erased it
	 * might not be possible to execute interrupts, send NodeStatus messages etc.
	 * We can pass a per page callback or yeild */

	flash_error_t status = FLASH_ERROR_AFU;


	ssize_t bllastpage = up_progmem_getpage(address - 1);

	if (bllastpage < 0) {
		return FLASH_ERROR_AFU;
	}

	ssize_t appstartpage = up_progmem_getpage(address);
	ssize_t appendpage = up_progmem_getpage(address + nbytes - 4);

	if (appendpage < 0 || appstartpage < 0) {
		return FLASH_ERROR_AFU;
	}

	status = FLASH_ERROR_SUICIDE;

	if (bllastpage >= 0 && appstartpage > bllastpage) {

		/* Erase the whole application flash region */

		status = FLASH_OK;

		while (status == FLASH_OK && appstartpage <= appendpage) {
			bl_sched_yield();
			ssize_t ps = up_progmem_eraseblock(appstartpage++);

			if (ps <= 0) {
				status = FLASH_ERASE_ERROR;
			}
		}
	}

	return status;
}

/****************************************************************************
 * Name: bl_flash_write
 *
 * Description:
 *   This function writes the flash starting at the given address
 *
 * Input Parameters:
 *   flash_address - The address of the flash to write
 *                   must be word aligned
 *   data          - A pointer to a buffer count bytes to be written
 *                   to the flash.
 *   count         - Number of bytes to write
 *
 * Returned value:
 *   On success FLASH_OK On Error one of the flash_error_t
 *
 ****************************************************************************/

flash_error_t bl_flash_write(uint32_t flash_address, uint8_t *data, ssize_t count)
{

	flash_error_t status = FLASH_ERROR;

	if (flash_address >= APPLICATION_LOAD_ADDRESS &&
	    (flash_address + count) <= (uint32_t) APPLICATION_LAST_8BIT_ADDRRESS) {
		if (count  ==
		    up_progmem_write((size_t) flash_address, (void *)data, count)) {
			status = FLASH_OK;
		}
	}

	return status;
}
