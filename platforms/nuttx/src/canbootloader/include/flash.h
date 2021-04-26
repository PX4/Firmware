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

#pragma once

typedef enum {
	FLASH_OK = 0,
	FLASH_ERROR,
	FLASH_ERASE_ERROR,
	FLASH_ERASE_VERIFY_ERROR,
	FLASH_ERROR_SUICIDE,
	FLASH_ERROR_AFU,

} flash_error_t;

#if defined(OPT_LATER_FLAHSED_WORDS)
#  define LATER_FLAHSED_WORDS OPT_LATER_FLAHSED_WORDS /* The number of 32 bit words not written until
                                 * after CRC verification
                                */
#else
#  define LATER_FLAHSED_WORDS 1 /* The number of 32 bit words not written until
                                 * after CRC verification
                                */
#endif
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
flash_error_t bl_flash_erase(size_t address, size_t nbytes);

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
flash_error_t bl_flash_write(uint32_t flash_address, uint8_t *data, ssize_t count);
