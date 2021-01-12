/****************************************************************************
 *
 *   Copyright (C) 2018 PX4 Development Team. All rights reserved.
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
 * @file board_identity.c
 * Implementation of imxrt based Board identity API
 */

#include <px4_platform_common/px4_config.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <arm_arch.h>
#include <hardware/imxrt_ocotp.h>

#define CPU_UUID_BYTE_FORMAT_ORDER          {3, 2, 1, 0, 7, 6, 5, 4}
#define SWAP_UINT32(x) (((x) >> 24) | (((x) & 0x00ff0000) >> 8) | (((x) & 0x0000ff00) << 8) | ((x) << 24))


static const uint16_t soc_arch_id = PX4_SOC_ARCH_ID;

/* A type suitable for holding the reordering array for the byte format of the UUID
 */

typedef const uint8_t uuid_uint8_reorder_t[PX4_CPU_UUID_BYTE_LENGTH];

void board_get_uuid(uuid_byte_t uuid_bytes)
{
	uuid_uint8_reorder_t reorder = CPU_UUID_BYTE_FORMAT_ORDER;

	union {
		uuid_byte_t b;
		uuid_uint32_t w;
	} id;

	/* Copy the serial from the OCOTP */

	board_get_uuid32(id.w);

	/* swap endianess */

	for (int i = 0; i < PX4_CPU_UUID_BYTE_LENGTH; i++) {
		uuid_bytes[i] = id.b[reorder[i]];
	}
}

void board_get_uuid32(uuid_uint32_t uuid_words)
{
	/* IMXRT_OCOTP_CFG1:0x420[10:0], IMXRT_OCOTP_CFG0:0x410[31:0] LOT_NO_ENC[42:0](SJC_CHALL/UNIQUE_ID[42:0])
	 *    43 bits  FSL-wide unique,encoded LOT ID STD II/SJC CHALLENGE/ Unique ID
	 * 0x420[15:11] WAFER_NO[4:0]( SJC_CHALL[47:43] /UNIQUE_ID[47:43])
	 *     5 bits The wafer number of the wafer on which the device was fabricated/SJC CHALLENGE/ Unique ID
	 * 0x420[23:16] DIE-YCORDINATE[7:0]( SJC_CHALL[55:48] /UNIQUE_ID[55:48])
	 *     8 bits The Y-coordinate of the die location on the wafer/SJC CHALLENGE/Unique ID
	 * 0x420[31:24] DIE-XCORDINATE[7:0]( SJC_CHALL[63:56] /UNIQUE_ID[63:56] )
	 *    8 bits The X-coordinate of the die location on the wafer/SJC CHALLENGE/Unique ID
	 *
	 *         word [0] word[1]
	 * SJC_CHALL[63:32] [31:00]
	 */

	uuid_words[0] = getreg32(IMXRT_OCOTP_CFG1);
	uuid_words[1] = getreg32(IMXRT_OCOTP_CFG0);
}

int board_get_uuid32_formated(char *format_buffer, int size,
			      const char *format,
			      const char *seperator)
{
	uuid_uint32_t uuid;
	board_get_uuid32(uuid);

	int offset = 0;
	int sep_size = seperator ? strlen(seperator) : 0;

	for (unsigned int i = 0; i < PX4_CPU_UUID_WORD32_LENGTH; i++) {
		offset += snprintf(&format_buffer[offset], size - ((i * 2 * sizeof(uint32_t)) + 1), format, uuid[i]);

		if (sep_size && i < PX4_CPU_UUID_WORD32_LENGTH - 1) {
			strcat(&format_buffer[offset], seperator);
			offset += sep_size;
		}
	}

	return 0;
}

int board_get_mfguid(mfguid_t mfgid)
{
	board_get_uuid(* (uuid_byte_t *) mfgid);
	return PX4_CPU_MFGUID_BYTE_LENGTH;
}

int board_get_mfguid_formated(char *format_buffer, int size)
{
	mfguid_t mfguid;

	board_get_mfguid(mfguid);
	int offset  = 0;

	for (unsigned int i = 0; i < PX4_CPU_MFGUID_BYTE_LENGTH; i++) {
		offset += snprintf(&format_buffer[offset], size - offset, "%02x", mfguid[i]);
	}

	return offset;
}

int board_get_px4_guid(px4_guid_t px4_guid)
{
	uint8_t  *pb = (uint8_t *) &px4_guid[0];
	*pb++ = (soc_arch_id >> 8) & 0xff;
	*pb++ = (soc_arch_id & 0xff);

	for (unsigned i = 0; i < PX4_GUID_BYTE_LENGTH - (sizeof(soc_arch_id) + PX4_CPU_UUID_BYTE_LENGTH); i++) {
		*pb++ = 0;
	}

	board_get_uuid(pb);
	return PX4_GUID_BYTE_LENGTH;
}

int board_get_px4_guid_formated(char *format_buffer, int size)
{
	px4_guid_t px4_guid;
	board_get_px4_guid(px4_guid);
	int offset  = 0;

	/* size should be 2 per byte + 1 for termination
	 * So it needs to be odd
	 */
	size = size & 1 ? size : size - 1;

	/* Discard from MSD */
	for (unsigned i = PX4_GUID_BYTE_LENGTH - size / 2; offset < size && i < PX4_GUID_BYTE_LENGTH; i++) {
		offset += snprintf(&format_buffer[offset], size - offset, "%02x", px4_guid[i]);
	}

	return offset;
}
