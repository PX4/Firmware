/****************************************************************************
 * configs/nxp_fmurt1062-v1/scripts/flash.ld
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
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

/* The FMURT1062 has 8MiB of QSPI FLASH beginning at address,
 * 0x0060:0000, Upto 512Kb of DTCM RAM beginning at 0x2000:0000, and 1MiB OCRAM
 * beginning at 0x2020:0000.  Neither DTCM or SDRAM are used in this
 * configuratin.
 */

MEMORY
{
	flash (rx)  : ORIGIN = 0x60000000, LENGTH = 8M
	sram  (rwx) : ORIGIN = 0x20200000, LENGTH = 768K
	itcm  (rwx) : ORIGIN = 0x00000000, LENGTH = 128K
	dtcm  (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

OUTPUT_ARCH(arm)
EXTERN(_vectors)
EXTERN(g_flash_config)
EXTERN(g_image_vector_table)
EXTERN(g_boot_data)

ENTRY(_stext)

SECTIONS
{
	/* Image Vector Table and Boot Data for booting from external flash */

	.boot_hdr : ALIGN(4)
	{
		FILL(0xff)
		__boot_hdr_start__ = ABSOLUTE(.) ;
		KEEP(*(.boot_hdr.conf))
		. = 0x1000 ;
		KEEP(*(.boot_hdr.ivt))
		. = 0x1020 ;
		KEEP(*(.boot_hdr.boot_data))
		. = 0x1030 ;
		KEEP(*(.boot_hdr.dcd_data))
		__boot_hdr_end__ = ABSOLUTE(.) ;
		. = 0x2000 ;
	} >flash

	.text :
	{
		_stext = ABSOLUTE(.);
		*(.vectors)
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

	} > flash

	/*
	 * Init functions (static constructors and the like)
	 */

	.init_section :
	{
		_sinit = ABSOLUTE(.);
         KEEP(*(.init_array .init_array.*))
		_einit = ABSOLUTE(.);
	} > flash

	.ARM.extab :
	{
		*(.ARM.extab*)
	} > flash

	__exidx_start = ABSOLUTE(.);

	.ARM.exidx :
	{
		*(.ARM.exidx*)
	} > flash
	__exidx_end = ABSOLUTE(.);

	_eronly = ABSOLUTE(.);

	.data :
	{
		_sdata = ABSOLUTE(.);
		*(.data .data.*)
		*(.gnu.linkonce.d.*)
		CONSTRUCTORS
		_edata = ABSOLUTE(.);
	} > sram AT > flash

	.ramfunc ALIGN(4):
	{
		_sramfuncs = ABSOLUTE(.);
		*(.ramfunc  .ramfunc.*)
		_eramfuncs = ABSOLUTE(.);
	} > sram AT > flash

	_framfuncs = LOADADDR(.ramfunc);

	.bss :
	{
		_sbss = ABSOLUTE(.);
		*(.bss .bss.*)
		*(.gnu.linkonce.b.*)
		*(COMMON)
		_ebss = ABSOLUTE(.);
	} > sram

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
}
