/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file boot_config.h
 *
 * bootloader definitions that configures the behavior and options
 * of the Boot loader
 * This file is relies on the parent folder's boot_config.h file and defines
 * different usages of the hardware for bootloading
 */

#pragma once


/************************************************************************************
 * Included Files
 ************************************************************************************/

/* Bring in the board_config.h definitions
 * todo:make this be pulled in from a targed's build
 * files in nuttx*/

#include "board_config.h"
#include "uavcan.h"
#include <nuttx/compiler.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OPT_PREFERRED_NODE_ID ANY_NODE_ID

//todo:wrap OPT_x in in ifdefs for command line definitions
#define OPT_TBOOT_MS                    5000
#define OPT_NODE_STATUS_RATE_MS         800
#define OPT_NODE_INFO_RATE_MS           50
#define OPT_BL_NUMBER_TIMERS            7
#undef  S_RGB_LED_DMA                   // use the poled one
/*
 *  This Option set is set to 1 ensure a provider of firmware has an
 *  opportunity update the node's firmware.
 *  This Option is the default policy and can be overridden by
 *  a jumper
 *  When this Policy is set, the node will ignore tboot and
 *  wait indefinitely for a GetNodeInfo request before booting.
 *
 *  OPT_WAIT_FOR_GETNODEINFO_JUMPER_GPIO_INVERT is used to allow
 *  the polarity of the jumper to be True Active
 *
 *  wait  OPT_WAIT_FOR_GETNODEINFO  OPT_WAIT_FOR_GETNODEINFO_JUMPER_GPIO
 *                                                 Jumper
 *   yes           1                       0         x
 *   yes           1                       1       Active
 *   no            1                       1       Not Active
 *   no            0                       0         X
 *   yes           0                       1       Active
 *   no            0                       1       Not Active
 *
 */
#define OPT_WAIT_FOR_GETNODEINFO                    0
#define OPT_WAIT_FOR_GETNODEINFO_JUMPER_GPIO        1
#define OPT_WAIT_FOR_GETNODEINFO_JUMPER_GPIO_INVERT 0

#define OPT_ENABLE_WD           1

#define OPT_RESTART_TIMEOUT_MS          20000

/* Reserved for the Booloader */
#define OPT_BOOTLOADER_SIZE_IN_K        (24 * 1024)

/* Reserved for the application out of the total
 * system flash minus the BOOTLOADER_SIZE_IN_K
 */
#define OPT_APPLICATION_RESERVER_IN_K    (0)

#define OPT_APPLICATION_IMAGE_OFFSET    OPT_BOOTLOADER_SIZE_IN_K
#define OPT_APPLICATION_IMAGE_LENGTH    (FLASH_SIZE-(OPT_BOOTLOADER_SIZE_IN_K + OPT_APPLICATION_RESERVER_IN_K))


#define FLASH_BASE              0
#define FLASH_SIZE              (1024*1024)

#define APPLICATION_LOAD_ADDRESS (FLASH_BASE + OPT_APPLICATION_IMAGE_OFFSET)
#define APPLICATION_SIZE (FLASH_SIZE-OPT_APPLICATION_IMAGE_OFFSET)
#define APPLICATION_LAST_8BIT_ADDRRESS  ((uint8_t *)((APPLICATION_LOAD_ADDRESS+APPLICATION_SIZE)-sizeof(uint8_t)))
#define APPLICATION_LAST_32BIT_ADDRRESS ((uint32_t *)((APPLICATION_LOAD_ADDRESS+APPLICATION_SIZE)-sizeof(uint32_t)))
#define APPLICATION_LAST_64BIT_ADDRRESS ((uint64_t *)((APPLICATION_LOAD_ADDRESS+APPLICATION_SIZE)-sizeof(uint64_t)))

#define OPT_USE_YIELD
#define OPT_LATER_FLAHSED_WORDS 2

/* Bootloader Option*****************************************************************
 *
 */
#define GPIO_GETNODEINFO_JUMPER (PIN_PTC14)
