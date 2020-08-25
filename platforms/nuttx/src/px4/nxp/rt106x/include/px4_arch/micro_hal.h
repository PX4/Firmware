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


#include <px4_platform/micro_hal.h>

__BEGIN_DECLS

#define PX4_SOC_ARCH_ID             PX4_SOC_ARCH_ID_NXPIMXRT1062

#// Fixme: using ??
#define PX4_BBSRAM_SIZE             2048
#define PX4_BBSRAM_GETDESC_IOCTL    0
#define PX4_NUMBER_I2C_BUSES        4

#define GPIO_OUTPUT_SET             GPIO_OUTPUT_ONE
#define GPIO_OUTPUT_CLEAR           GPIO_OUTPUT_ZERO

#include <chip.h>
#include <imxrt_lpspi.h>
#include <imxrt_lpi2c.h>
//#    include <imxrt_uid.h> todo:Upsteam UID access

/* imxrt defines the 64 bit UUID as
 *
 *  OCOTP 0x410 bits 31:0
 *  OCOTP 0x420 bits 63:32
 *
 *  PX4 uses the words in bigendian order MSB to LSB
 *   word  [0]    [1]
 *   bits 63-32, 31-00,
 */
#define PX4_CPU_UUID_BYTE_LENGTH                8
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

#define PX4_CPU_UUID_WORD32_UNIQUE_H            0 /* Least significant digits change the most (die wafer,X,Y */
#define PX4_CPU_UUID_WORD32_UNIQUE_M            1 /* Most significant digits change the least (lot#) */

/*                                                  Separator    nnn:nnn:nnnn     2 char per byte           term */
#define PX4_CPU_UUID_WORD32_FORMAT_SIZE         (PX4_CPU_UUID_WORD32_LENGTH-1+(2*PX4_CPU_UUID_BYTE_LENGTH)+1)
#define PX4_CPU_MFGUID_FORMAT_SIZE              ((2*PX4_CPU_MFGUID_BYTE_LENGTH)+1)

#define imxrt_bbsram_savepanic(fileno, context, length) (0) // todo:Not implemented yet

#define px4_savepanic(fileno, context, length)   imxrt_bbsram_savepanic(fileno, context, length)

/* bus_num is 1 based on imx and must be translated from the legacy one based */

#define PX4_BUS_OFFSET       0                  /* imxrt buses are 1 based no adjustment needed */

#define px4_spibus_initialize(bus_num_1based)   imxrt_lpspibus_initialize(PX4_BUS_NUMBER_FROM_PX4(bus_num_1based))

#define px4_i2cbus_initialize(bus_num_1based)   imxrt_i2cbus_initialize(PX4_BUS_NUMBER_FROM_PX4(bus_num_1based))
#define px4_i2cbus_uninitialize(pdev)           imxrt_i2cbus_uninitialize(pdev)

#define px4_arch_configgpio(pinset)             imxrt_config_gpio(pinset)
#define px4_arch_unconfiggpio(pinset)
#define px4_arch_gpioread(pinset)               imxrt_gpio_read(pinset)
#define px4_arch_gpiowrite(pinset, value)       imxrt_gpio_write(pinset, value)

/* imxrt_gpiosetevent is not implemented and will need to be added */

#define px4_arch_gpiosetevent(pinset,r,f,e,fp,a)  imxrt_gpiosetevent(pinset,r,f,e,fp,a)

#define PX4_MAKE_GPIO_INPUT(gpio) (((gpio) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT | IOMUX_SCHMITT_TRIGGER | IOMUX_PULL_UP_47K | IOMUX_DRIVE_HIZ))
#define PX4_MAKE_GPIO_OUTPUT(gpio) (((gpio) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | IOMUX_CMOS_OUTPUT | IOMUX_PULL_KEEP | IOMUX_DRIVE_33OHM  | IOMUX_SPEED_MEDIUM | IOMUX_SLEW_FAST))

__END_DECLS
