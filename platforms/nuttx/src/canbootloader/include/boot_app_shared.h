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

#include <nuttx/compiler.h>

__BEGIN_DECLS

/* Define the signature for the Application descriptor as 'APDesc' and a
 * revision number of 00 used in app_descriptor_t
 */

#define APP_DESCRIPTOR_SIGNATURE_ID 'A','P','D','e','s','c'
#define APP_DESCRIPTOR_SIGNATURE_REV '0','0'
#define APP_DESCRIPTOR_SIGNATURE APP_DESCRIPTOR_SIGNATURE_ID, APP_DESCRIPTOR_SIGNATURE_REV

/* N.B. the .ld file must emit this sections */
# define boot_app_shared_section __attribute__((section(".app_descriptor")))

/* eRole defines the role of the bootloader_app_shared_t structure */

typedef enum eRole  {
	Invalid,
	App,
	BootLoader
} eRole_t;

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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wattributes"
typedef begin_packed_struct struct bootloader_app_shared_t {
	union {
		uint64_t ull;
		uint32_t ul[2];
		uint8_t  valid;
	} crc;
	uint32_t signature;
	uint32_t bus_speed;
	uint32_t node_id;
} end_packed_struct bootloader_app_shared_t;
#pragma GCC diagnostic pop

/****************************************************************************
 *
 * Application firmware descriptor.
 *
 * This structure located by the linker script somewhere after the vector table.
 * (within the first several kilobytes of the beginning address of the
 * application);
 *
 * This structure must be aligned on an 8-byte boundary.
 *
 * The bootloader will scan through the application FLASH image until it
 * finds the signature.
 *
 * The image_crc is calculated as follows:
 *      1) All fields of this structure must be initialized with the correct
 *         information about the firmware image bin file
 *         (Here after refereed to as image)
 *      2) image_crc set to 0;
 *      3) The CRC 64 is calculated over the image from offset 0 up to and including the
 *         last byte of the image file.
 *      4) The calculated CRC 64 is stored in image_crc
 *      5) The new image file is then written to a file a ".img" extension.
 *
****************************************************************************/

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wattributes"
#pragma GCC diagnostic ignored "-Wpacked"
typedef begin_packed_struct struct app_descriptor_t {
	uint8_t signature[sizeof(uint64_t)];
	uint64_t image_crc;
	uint32_t image_size;
	uint32_t vcs_commit;
	uint8_t major_version;
	uint8_t minor_version;
	uint8_t reserved[6];
} end_packed_struct app_descriptor_t;
#pragma GCC diagnostic pop

/****************************************************************************
 * Name: bootloader_app_shared_read
 *
 * Description:
 *   Based on the role requested, this function will conditionally populate
 *   a bootloader_app_shared_t structure from the physical locations used
 *   to transfer the shared data to/from an application (internal data) .
 *
 *   The functions will only populate the structure and return a status
 *   indicating success, if the internal data has the correct signature as
 *   requested by the Role AND has a valid crc.
 *
 * Input Parameters:
 *   shared - A pointer to a bootloader_app_shared_t return the data in if
 *   the internal data is valid for the requested Role
 *   role   - An eRole_t of App or BootLoader to validate the internal data
 *            against. For a Bootloader this would be the value of App to
 *            read the application passed data.
 *
 * Returned value:
 *   OK     - Indicates that the internal data has been copied to callers
 *            bootloader_app_shared_t structure.
 *
 *  -EBADR  - The Role or crc of the internal data was not valid. The copy
 *            did not occur.
 *
 ****************************************************************************/

int bootloader_app_shared_read(bootloader_app_shared_t *shared, eRole_t role);

/****************************************************************************
 * Name: bootloader_app_shared_write
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
 *   shared - A pointer to a bootloader_app_shared_t data to commit to
 *   the internal data for passing to/from an application.
 *   role   - An eRole_t of App or BootLoader to use in the internal data
 *            to be passed to/from an application. For a Bootloader this
 *            would be the value of Bootloader to write to the passed data.
 *            to the application via the internal data.
 *
 * Returned value:
 *   None.
 *
 ****************************************************************************/

void bootloader_app_shared_write(bootloader_app_shared_t *shared, eRole_t role);

/****************************************************************************
 * Name: bootloader_app_shared_invalidate
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

void bootloader_app_shared_invalidate(void);

__END_DECLS
