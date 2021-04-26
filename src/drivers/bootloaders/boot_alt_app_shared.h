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

#pragma once

#include <nuttx/compiler.h>
#if defined(SUPPORT_ALT_CAN_BOOTLOADER)
__BEGIN_DECLS

/****************************************************************************
 *
 * Bootloader and Application shared structure.
 *
 * The data in this structure is passed in SRAM or the the CAN filter
 * registers from bootloader to application and application to bootloader.
 *
 * Do not assume any mapping or location for the passing of this data
 * that is done in the read and write routines and is abstracted by design.
 *
 * For reference, the following is performed based on eRole in API calls
 * defined below:
 *
 *      The application must write BOOTLOADER_COMMON_APP_SIGNATURE to the
 *      signature field when passing data to the bootloader; when the
 *      bootloader passes data to the application, it must write
 *      BOOTLOADER_COMMON_BOOTLOADER_SIGNATURE to the signature field.
 *
 *      The CRC is calculated over the structure from signature to the
 *      last byte. The resulting values are then copied to the CAN filter
 *      registers by bootloader_app_shared_read and
 *      bootloader_app_shared_write.
 *
****************************************************************************/

#define BL_ALT_APP_SHARED_SIGNATURE 0xc544ad9a
typedef begin_packed_struct struct bootloader_alt_app_shared_t {
	uint32_t  signature;
	uint32_t  reserved[4];
	uint8_t   fw_server_node_id;
	uint8_t   node_id;
	uint8_t   path[201];
} end_packed_struct bootloader_alt_app_shared_t;
#pragma GCC diagnostic pop

extern bootloader_alt_app_shared_t _sapp_bl_shared;

/****************************************************************************
 * Name: bootloader_alt_app_shared_read
 *
 * Description:
 *   Based on the role requested, this function will conditionally populate
 *   a bootloader_alt_app_shared_t structure from the physical locations used
 *   to transfer the shared data to/from an application (internal data) .
 *
 *   The functions will only populate the structure and return a status
 *   indicating success, if the internal data has the correct signature as
 *   requested by the Role AND has a valid crc.
 *
 * Input Parameters:
 *   shared - A pointer to a bootloader_alt_app_shared_t return the data in if
 *   the internal data is valid for the requested Role
 *
 * Returned value:
 *   OK     - Indicates that the internal data has been copied to callers
 *            bootloader_alt_app_shared_t structure.
 *
 *  -EBADR  - The Role or crc of the internal data was not valid. The copy
 *            did not occur.
 *
 ****************************************************************************/

int bootloader_alt_app_shared_read(bootloader_alt_app_shared_t *alt_shared);

/****************************************************************************
 * Name: bootloader_alt_app_shared_write
 *
 * Description:
 *   Based on the role, this function will commit the data passed
 *   into the physical locations used to transfer the shared data to/from
 *   an application (internal data) .
 *
 *   The functions will populate the signature and crc the data
 *   based on the provided Role.
 *
 * Input Parameters:
 *   shared - A pointer to a bootloader_alt_app_shared_t data to commit to
 *   the internal data for passing to/from an application.
 *
 * Returned value:
 *   None.
 *
 ****************************************************************************/

void bootloader_alt_app_shared_write(bootloader_alt_app_shared_t *alt_shared);

/****************************************************************************
 * Name: bootloader_alt_app_shared_invalidate
 *
 * Description:
 *   Invalidates the data passed the physical locations used to transfer
 *   the shared data to/from an application (internal data) .
 *
 *   The functions will invalidate the signature and crc and should be used
 *   to prevent deja vu.
 *
 * Input Parameters:
 *   None.
 *
 * Returned value:
 *   None.
 *
 ****************************************************************************/

void bootloader_alt_app_shared_invalidate(void);

__END_DECLS
#endif
