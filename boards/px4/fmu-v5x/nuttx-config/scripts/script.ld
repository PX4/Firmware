/****************************************************************************
 * scripts/script.ld
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
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
    FLASH_ITCM (rx) : ORIGIN = 0x00208000, LENGTH = 2016K
    FLASH_AXIM (rx) : ORIGIN = 0x08008000, LENGTH = 2016K

    ITCM_RAM (rwx) : ORIGIN = 0x00000000, LENGTH = 16K
    DTCM_RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
    SRAM1    (rwx) : ORIGIN = 0x20020000, LENGTH = 368K
    SRAM2    (rwx) : ORIGIN = 0x2007c000, LENGTH = 16K
}

OUTPUT_ARCH(arm)
EXTERN(_vectors)
ENTRY(_stext)

/*
 * Ensure that abort() is present in the final object.  The exception handling
 * code pulled in by libgcc.a requires it (and that code cannot be easily avoided).
 */
EXTERN(abort)
EXTERN(_bootdelay_signature)
EXTERN(board_get_manifest)

SECTIONS
{
	.text : {
		_stext = ABSOLUTE(.);
		*(.vectors)
		. = ALIGN(32);
		/*
		This signature provides the bootloader with a way to delay booting
		*/
		_bootdelay_signature = ABSOLUTE(.);
		FILL(0xffecc2925d7d05c5)
		. += 8;
		*(.text .text.*)
		*(.fixup)
		*(.gnu.warning)
		*(.rodata .rodata.*)
		*(.gnu.linkonce.t.*)
		*(.glue_7)
		*(.glue_7t)
		*(.got)
		*(.gcc_except_table)
		*(.gnu.linkonce.r.*)
		_etext = ABSOLUTE(.);

	} > FLASH_AXIM

	/*
	 * Init functions (static constructors and the like)
	 */
	.init_section : {
		_sinit = ABSOLUTE(.);
		KEEP(*(.init_array .init_array.*))
		_einit = ABSOLUTE(.);
	} > FLASH_AXIM

	/*
	 * Construction data for parameters.
	 */
	__param ALIGN(4): {
		__param_start = ABSOLUTE(.);
		KEEP(*(__param*))
		__param_end = ABSOLUTE(.);
	} > FLASH_AXIM

	.ARM.extab : {
		*(.ARM.extab*)
	} > FLASH_AXIM

	__exidx_start = ABSOLUTE(.);
	.ARM.exidx : {
		*(.ARM.exidx*)
	} > FLASH_AXIM
	__exidx_end = ABSOLUTE(.);

	_eronly = ABSOLUTE(.);

	.data : {
		_sdata = ABSOLUTE(.);
		*(.data .data.*)
		*(.gnu.linkonce.d.*)
		CONSTRUCTORS
		_edata = ABSOLUTE(.);
	} > SRAM1 AT > FLASH_AXIM

	.bss : {
		_sbss = ABSOLUTE(.);
		*(.bss .bss.*)
		*(.gnu.linkonce.b.*)
		*(COMMON)
		. = ALIGN(4);
		_ebss = ABSOLUTE(.);
	} > SRAM1

	/* Stabs debugging sections. */
	.stab 0 : { *(.stab) }
	.stabstr 0 : { *(.stabstr) }
	.stab.excl 0 : { *(.stab.excl) }
	.stab.exclstr 0 : { *(.stab.exclstr) }
	.stab.index 0 : { *(.stab.index) }
	.stab.indexstr 0 : { *(.stab.indexstr) }
	.comment 0 : { *(.comment) }
	.debug_abbrev 0 : { *(.debug_abbrev) }
	.debug_info 0 : { *(.debug_info) }
	.debug_line 0 : { *(.debug_line) }
	.debug_pubnames 0 : { *(.debug_pubnames) }
	.debug_aranges 0 : { *(.debug_aranges) }

	.ramfunc : {
		_sramfuncs = .;
		*(.ramfunc  .ramfunc.*)
		. = ALIGN(4);
		_eramfuncs = .;
	} > ITCM_RAM AT > FLASH_AXIM

	_framfuncs = LOADADDR(.ramfunc);
}
