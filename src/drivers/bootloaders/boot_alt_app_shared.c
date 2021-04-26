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

#if defined(SUPPORT_ALT_CAN_BOOTLOADER)
#include <stdint.h>
#include <string.h>

#include "chip.h"
#include "stm32.h"

#include <errno.h>

#include "boot_alt_app_shared.h"

#include <lib/systemlib/crc.h>

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
 *   role   - An eRole_t of App or BootLoader to validate the internal data
 *            against. For a Bootloader this would be the value of App to
 *            read the application passed data.
 *
 * Returned value:
 *   OK     - Indicates that the internal data has been copied to callers
 *            bootloader_alt_app_shared_t structure.
 *
 *  -EBADR  - internal data was not valid. The copy did not occur.
 *
 ****************************************************************************/
__EXPORT int bootloader_alt_app_shared_read(bootloader_alt_app_shared_t *alt_shared)
{
	int rv = EBADR;
	bootloader_alt_app_shared_t *bootloader_alt_app_shared = (bootloader_alt_app_shared_t *)&_sapp_bl_shared;

	if (bootloader_alt_app_shared->signature == BL_ALT_APP_SHARED_SIGNATURE) {
		*alt_shared = *bootloader_alt_app_shared;
		rv = 0;
	}

	return rv;
}

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
 *   role   - An eRole_t of App or BootLoader to use in the internal data
 *            to be passed to/from an application. For a Bootloader this
 *            would be the value of Bootloader to write to the passed data.
 *            to the application via the internal data.
 *
 * Returned value:
 *   None.
 *
 ****************************************************************************/
__EXPORT void bootloader_alt_app_shared_write(bootloader_alt_app_shared_t *alt_shared)
{

	bootloader_alt_app_shared_t *bootloader_alt_app_shared = (bootloader_alt_app_shared_t *)&_sapp_bl_shared;
	*bootloader_alt_app_shared  = *alt_shared;
	bootloader_alt_app_shared->signature = BL_ALT_APP_SHARED_SIGNATURE;

}

/****************************************************************************
 * Name: bootloader_app_shared_invalidate
 *
 * Description:
 *   Invalidates the data passed the physical locations used to transfer
 *   the shared data to/from an application (internal data) .
 *
 *   The functions will invalidate the signature and crc and shoulf be used
 *   to prevent deja vu.
 *
 * Input Parameters:
 *   None.
 *
 * Returned value:
 *   None.
 *
 ****************************************************************************/
__EXPORT void bootloader_alt_app_shared_invalidate(void)
{
	bootloader_alt_app_shared_t *bootloader_alt_app_shared = (bootloader_alt_app_shared_t *)&_sapp_bl_shared;
	bootloader_alt_app_shared->signature = 0;

}
#endif
