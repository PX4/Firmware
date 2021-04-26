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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdlib.h>
#include "crc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: crc16_add
 *
 * Description:
 *   Use to calculates a CRC-16-CCITT using the polynomial of
 *   0x1021 by adding a value successive values.
 *
 * Input Parameters:
 *    crc   - The running total of the crc 16
 *    value - The value to add
 *
 * Returned Value:
 *   The current crc16 with the value processed.
 *
 ****************************************************************************/

uint16_t crc16_add(uint16_t crc, uint8_t value)
{
	uint32_t i;
	const uint16_t poly = 0x1021u;
	crc ^= (uint16_t)((uint16_t) value << 8u);

	for (i = 0; i < 8; i++) {
		if (crc & (1u << 15u)) {
			crc = (uint16_t)((crc << 1u) ^ poly);

		} else {
			crc = (uint16_t)(crc << 1u);
		}
	}

	return crc;
}

/****************************************************************************
 * Name: crc16_signature
 *
 * Description:
 *   Calculates a CRC-16-CCITT using the crc16_add
 *   function
 *
 * Input Parameters:
 *    initial - The Initial value to uses as the crc's starting point
 *    length  - The number of bytes to add to the crc
 *    bytes   - A pointer to any array of length bytes
 *
 * Returned Value:
 *   The crc16 of the array of bytes
 *
 ****************************************************************************/

uint16_t crc16_signature(uint16_t initial, size_t length, const uint8_t *bytes)
{
	size_t i;

	for (i = 0u; i < length; i++) {
		initial = crc16_add(initial, bytes[i]);
	}

	return initial ^ CRC16_OUTPUT_XOR;
}

/****************************************************************************
 * Name: crc32_signature
 *
 * Description:
 *   Calculates a CRC-32 function
 *
 * Input Parameters:
 *    acc     - The accumulator value to uses as the crc's starting point
 *    length  - The number of bytes to add to the crc
 *    bytes   - A pointer to any array of length bytes
 *
 * Returned Value:
 *   The crc32 of the array of bytes
 *
 ****************************************************************************/

uint32_t crc32_signature(uint32_t acc, size_t length, const uint8_t *bytes)
{
	size_t i;
	const uint32_t poly = 0xedb88320u;
	const uint8_t  bits = 8u;
	uint8_t        w = bits;

	for (i = 0u; i < length; i++) {
		acc ^= bytes[i];
		w = bits;

		while (w--) {
			const uint32_t xor = -(acc & 1);
			acc >>= 1;
			acc ^= (poly & xor);
		}
	}

	return acc;
}


/****************************************************************************
 * Name: crc64_add_word
 *
 * Description:
 *   Calculates a CRC-64-WE using the polynomial of 0x42F0E1EBA9EA3693
 *   See http://reveng.sourceforge.net/crc-catalogue/17plus.htm#crc.cat-bits.64
 *   Check: 0x62EC59E3F1A4F00A
 *
 * Input Parameters:
 *    crc   - The running total of the crc 64
 *    value - The value to add
 *
 * Returned Value:
 *   The current crc64 with the value processed.
 *
 ****************************************************************************/
__EXPORT
uint64_t crc64_add_word(uint64_t crc, uint32_t value)
{
	uint32_t i, j;
	uint8_t byte;
	const uint64_t poly = 0x42F0E1EBA9EA3693ull;

	for (j = 0; j < 4; j++) {
		byte = ((uint8_t *) &value)[j];
		crc ^= (uint64_t) byte << 56u;

		for (i = 0; i < 8; i++) {
			if (crc & (1ull << 63u)) {
				crc = (uint64_t)(crc << 1u) ^ poly;

			} else {
				crc = (uint64_t)(crc << 1u);
			}
		}
	}

	return crc;
}
