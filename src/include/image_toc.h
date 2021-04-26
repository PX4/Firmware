/****************************************************************************
 *
 *   Copyright (C) 2020 Technology Innovation Institute. All rights reserved.
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

#ifndef _IMAGE_TOC_H
#define _IMAGE_TOC_H

/* Table of contents entry flags, describing what to
   do with the image
*/
#define TOC_FLAG1_BOOT 0x1
#define TOC_FLAG1_VTORS 0x2
#define TOC_FLAG1_CHECK_SIGNATURE 0x4
#define TOC_FLAG1_DECRYPT 0x8

#define TOC_FLAG1_RDCT 0x80

#define TOC_START_MAGIC 0x00434f54 /* "TOC" */
#define TOC_END_MAGIC 0x00444e45 /* "END" */

/* TOC version, can be used to disable SW downgrade */
#ifndef BOARD_IMAGE_TOC_VERSION
#define TOC_VERSION 1
#else
#define TOC_VERSION BOARD_IMAGE_TOC_VERSION
#endif


/* Markers for TOC start and end in the image */

typedef const struct __attribute__((__packed__)) image_toc_start {
	const uint32_t magic;
	const uint32_t version;
} image_toc_start_t;

/* Table of contents structure. */

typedef struct __attribute__((__packed__)) image_toc_entry {
	unsigned char name[4];  /* Name of the section */
	const void *start;      /* Start address of the section in flash */
	const void *end;        /* End of the section */
	const void *target;     /* Copy target address of the section */
	uint8_t signature_idx;  /* Index to the signature in the TOC */
	uint8_t signature_key;  /* Key index for the signature */
	uint8_t encryption_key; /* Key index for encryption */
	uint8_t flags1;         /* Flags */
	uint32_t reserved;      /* e.g. for more flags */
} image_toc_entry_t;

#define IMAGE_MAIN_TOC(len)                              \
	const struct __attribute__((__packed__)) image_toc   \
	{                                                    \
		image_toc_start_t start;                         \
		image_toc_entry_t entry[len];                    \
		uint32_t end;                                    \
	} _main_toc __attribute__((section(".main_toc")))

/* An R&D certificate info structure, which can be used
   to e.g. allow unsigned boot on a single device.
   Certificate signature follows the data
*/

#define RDCT_CAPS0_ALLOW_UNSIGNED_BOOT 0x1

typedef struct __attribute__((__packed__))
{
	uint8_t device_uuid[16];
	uint32_t caps[4];
	uint8_t creator_info[16];
	uint8_t customer_info[16];
	uint64_t creation_date;
	uint64_t valid_until;
	uint8_t signature[];
} image_cert_t;

#endif
