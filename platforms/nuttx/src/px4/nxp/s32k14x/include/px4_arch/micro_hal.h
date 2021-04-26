/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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


#include "../../../nxp_common/include/px4_arch/micro_hal.h"

__BEGIN_DECLS

#define PX4_SOC_ARCH_ID             PX4_SOC_ARCH_ID_NXPS32K146

// Fixme: using ??
#define PX4_BBSRAM_SIZE             2048
#define PX4_BBSRAM_GETDESC_IOCTL    0
#define PX4_NUMBER_I2C_BUSES        2

#define GPIO_OUTPUT_SET             GPIO_OUTPUT_ONE
#define GPIO_OUTPUT_CLEAR           GPIO_OUTPUT_ZERO

#include <chip.h>
#include <s32k1xx_pin.h>
#include <s32k1xx_lpspi.h>
#include <s32k1xx_lpi2c.h>
#include <s32k1xx_flexcan.h>
//#include <s32k1xx_uid.h>

/* s32k1xx defines the 128 bit UUID as
 *  init32_t[4] that can be read as words
 *  init32_t[0] PX4_CPU_UUID_ADDRESS[0] bits 127:96 (offset 0)
 *  init32_t[1] PX4_CPU_UUID_ADDRESS[1] bits 95:64  (offset 4)
 *  init32_t[2] PX4_CPU_UUID_ADDRESS[1] bits 63:32  (offset 8)
 *  init32_t[3] PX4_CPU_UUID_ADDRESS[3] bits 31:0   (offset C)
 *
 *  PX4 uses the words in bigendian order MSB to LSB
 *   word  [0]    [1]    [2]   [3]
 *   bits 127:96  95-64  63-32, 31-00,
 */
#define PX4_CPU_UUID_BYTE_LENGTH                16
#define PX4_CPU_UUID_WORD32_LENGTH              (PX4_CPU_UUID_BYTE_LENGTH/sizeof(uint32_t))

/* The mfguid will be an array of bytes with
 * MSD @ index 0 - LSD @ index PX4_CPU_MFGUID_BYTE_LENGTH-1
 *
 * It will be converted to a string with the MSD on left and LSD on the right most position.
 */
#define PX4_CPU_MFGUID_BYTE_LENGTH              PX4_CPU_UUID_BYTE_LENGTH

/* define common formating across all commands */

#define PX4_CPU_UUID_WORD32_FORMAT              "%08x"
#define PX4_CPU_UUID_WORD32_SEPARATOR           ":"

#define PX4_CPU_UUID_WORD32_UNIQUE_H            3 /* Least significant digits change the most */
#define PX4_CPU_UUID_WORD32_UNIQUE_M            2 /* Middle High significant digits */
#define PX4_CPU_UUID_WORD32_UNIQUE_L            1 /* Middle Low significant digits */
#define PX4_CPU_UUID_WORD32_UNIQUE_N            0 /* Most significant digits change the least */

/*                                               Separator    nnn:nnn:nnnn     2 char per byte           term */
#define PX4_CPU_UUID_WORD32_FORMAT_SIZE         (PX4_CPU_UUID_WORD32_LENGTH-1+(2*PX4_CPU_UUID_BYTE_LENGTH)+1)
#define PX4_CPU_MFGUID_FORMAT_SIZE              ((2*PX4_CPU_MFGUID_BYTE_LENGTH)+1)

#define s32k1xx_bbsram_savepanic(fileno, context, length) (0) // todo:Not implemented yet

#define px4_savepanic(fileno, context, length)   s32k1xx_bbsram_savepanic(fileno, context, length)

/* bus_num is zero based on s32k1xx and must be translated from the legacy one based */

#define PX4_BUS_OFFSET       1                  /* s32k1xx buses are 0 based and adjustment is needed */

#define px4_spibus_initialize(bus_num_1based)   s32k1xx_lpspibus_initialize(PX4_BUS_NUMBER_FROM_PX4(bus_num_1based))

#define px4_i2cbus_initialize(bus_num_1based)   s32k1xx_i2cbus_initialize(PX4_BUS_NUMBER_FROM_PX4(bus_num_1based))
#define px4_i2cbus_uninitialize(pdev)           s32k1xx_i2cbus_uninitialize(pdev)

#define px4_arch_configgpio(pinset)             s32k1xx_pinconfig(pinset)
#define px4_arch_unconfiggpio(pinset)
#define px4_arch_gpioread(pinset)               s32k1xx_gpioread(pinset)
#define px4_arch_gpiowrite(pinset, value)       s32k1xx_gpiowrite(pinset, value)

/* s32k1xx_gpiosetevent is not implemented and will need to be added */

#define px4_arch_gpiosetevent(pinset,r,f,e,fp,a)  s32k1xx_gpiosetevent(pinset,r,f,e,fp,a)

#define I2C_RESET(q)

/* CAN bootloader usage */

#define TIMER_HRT_CYCLES_PER_US (STM32_HCLK_FREQUENCY/1000000)
#define TIMER_HRT_CYCLES_PER_MS (STM32_HCLK_FREQUENCY/1000)

#define crc_HiLOC       S32K1XX_CAN0_RXIMR27
#define crc_LoLOC       S32K1XX_CAN0_RXIMR28
#define signature_LOC   S32K1XX_CAN0_RXIMR29
#define bus_speed_LOC   S32K1XX_CAN0_RXIMR30
#define node_id_LOC     S32K1XX_CAN0_RXIMR31

__END_DECLS
