/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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

/*
 * @file board.h
 *
 * bootloader board interface
 * This file contains the common interfaces that all boards
 * have to supply
 */
#pragma once

#include "uavcan.h"
#include <nuttx/compiler.h>
#include <stdint.h>

typedef enum {
	off,
	reset,
	autobaud_start,
	autobaud_end,
	allocation_start,
	allocation_end,
	fw_update_start,
	fw_update_erase_fail,
	fw_update_invalid_response,
	fw_update_timeout,
	fw_update_invalid_crc,
	jump_to_app,
} uiindication_t;

#ifndef __ASSEMBLY__

/************************************************************************************
 * Name: stm32_boarddeinitialize
 *
 * Description:
 *   This function is called by the bootloader code priore to booting
 *   the application. Is should place the HW into an benign initialized state.
 *
 ************************************************************************************/

void stm32_boarddeinitialize(void);

/****************************************************************************
 * Name: board_get_product_name
 *
 * Description:
 *   Called to retrive the product name. The retuned alue is a assumed
 *   to be written to a pascal style string that will be length prefixed
 *   and not null terminated
 *
 * Input Parameters:
 *    product_name - A pointer to a buffer to write the name.
 *    maxlen       - The imum number of chatater that can be written
 *
 * Returned Value:
 *   The length of charaacters written to the buffer.
 *
 ****************************************************************************/

uint8_t board_get_product_name(uint8_t *product_name, size_t maxlen);

/****************************************************************************
 * Name: board_get_hardware_version
 *
 * Description:
 *   Called to retrieve the hardware version information. The function
 *   will first initialize the the callers struct to all zeros.
 *
 * Input Parameters:
 *    hw_version - A pointer to a uavcan_hardwareversion_t.
 *
 * Returned Value:
 *   Length of the unique_id
 *
 ****************************************************************************/

size_t board_get_hardware_version(uavcan_HardwareVersion_t *hw_version);

/****************************************************************************
 * Name: board_indicate
 *
 * Description:
 *   Provides User feedback to indicate the state of the bootloader
 *   on board specific  hardware.
 *
 * Input Parameters:
 *    indication - A member of the uiindication_t
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void board_indicate(uiindication_t indication);


#endif /* __ASSEMBLY__ */
