/****************************************************************************
 * scripts/memory.ld
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

/* The STM32F765IIT6 has 2048 KiB of main FLASH memory.  This FLASH memory
 * can be accessed from either the AXIM interface at address 0x0800:0000 or
 * from the ITCM interface at address 0x0020:0000.
 *
 * Additional information, including the option bytes, is available at at
 * FLASH at address 0x1ff0:0000 (AXIM) or 0x0010:0000 (ITCM).
 *
 * In the STM32F765IIT6, two different boot spaces can be selected through
 * the BOOT pin and the boot base address programmed in the BOOT_ADD0 and
 * BOOT_ADD1 option bytes:
 *
 *   1) BOOT=0: Boot address defined by user option byte BOOT_ADD0[15:0].
 *      ST programmed value: Flash on ITCM at 0x0020:0000
 *   2) BOOT=1: Boot address defined by user option byte BOOT_ADD1[15:0].
 *      ST programmed value: System bootloader at 0x0010:0000
 *
 * NuttX does not modify these option byes.  On the unmodified NUCLEO-144
 * board, the BOOT0 pin is at ground so by default, the STM32F765IIT6 will
 * boot from address 0x0020:0000 in ITCM FLASH.
 *
 * The STM32F765IIT6 also has 512 KiB of data SRAM (in addition to ITCM SRAM).
 * SRAM is split up into three blocks:
 *
 *   1) 128 KiB of DTCM SRM beginning at address 0x2000:0000
 *   2) 368 KiB of SRAM1 beginning at address 0x2002:0000
 *   3)  16 KiB of SRAM2 beginning at address 0x2007:c000
 *
 * When booting from FLASH, FLASH memory is aliased to address 0x0000:0000
 * where the code expects to begin execution by jumping to the entry point in
 * the 0x0800:0000 address range.
 *
 * Bootloader reserves the first 32K bank (2 Mbytes Flash memory single bank)
 * organization (256 bits read width)
 */
 
MEMORY
{
  /* ITCM boot address */

  itcm  (rwx) : ORIGIN = 0x00208000, LENGTH = 2048K-32K

  /* 2048KB FLASH, bootloader reserves the first 32kb */

  kflash (rx) : ORIGIN = 0x08008000, LENGTH = 1024K-32K
  uflash (rx) : ORIGIN = 0x08100000, LENGTH = 1024K

  /* ITCM RAM */

  itcm_ram  (rwx) : ORIGIN = 0x00000000, LENGTH = 16K

  /* DTCM SRAM */

  dtcm_ram  (rwx) : ORIGIN = 0x20000000, LENGTH = 128K

  /* 368KB of contiguous SRAM1 */

  ksram (rwx) : ORIGIN = 0x20020000, LENGTH = 128K
  usram (rwx) : ORIGIN = 0x20040000, LENGTH = 368K-128K

  /* 16KB of SRAM2 */

  sram2 (rwx) : ORIGIN = 0x2007c000, LENGTH = 16K
}
