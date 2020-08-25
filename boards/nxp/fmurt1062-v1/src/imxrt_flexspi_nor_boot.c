/****************************************************************************
 * config/imxrt1060-evk/src/imxrt_flexspi_nor_boot.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Ivan Ucherdzhiev <ivanucherdjiev@gmail.com>
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

#include "imxrt_flexspi_nor_boot.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

__attribute__((section(".boot_hdr.ivt")))
const struct ivt_s g_image_vector_table = {
	IVT_HEADER,                         /* IVT Header */
	IMAGE_ENTRY_ADDRESS,                /* Image  Entry Function */
	IVT_RSVD,                           /* Reserved = 0 */
	(uint32_t)DCD_ADDRESS,              /* Address where DCD information is stored */
	(uint32_t)BOOT_DATA_ADDRESS,        /* Address where BOOT Data Structure is stored */
	(uint32_t)IMAG_VECTOR_TABLE,        /* Pointer to IVT Self (absolute address */
	(uint32_t)CSF_ADDRESS,              /* Address where CSF file is stored */
	IVT_RSVD                            /* Reserved = 0 */
};

__attribute__((section(".boot_hdr.boot_data")))
const struct boot_data_s g_boot_data = {
	IMAGE_DEST,                         /* boot start location */
	(IMAGE_DEST_END - IMAGE_DEST),      /* size */
	PLUGIN_FLAG,                        /* Plugin flag */
	0xffffffff                          /* empty - extra data word */
};
