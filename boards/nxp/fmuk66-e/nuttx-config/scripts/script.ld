/****************************************************************************
 * configs/nxp_fmuk66-e/scripts/flash.ld
 *
 *   Copyright (C) 2016, 2017 Gregory Nutt. All rights reserved.
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

/* The MK66FN2M0VLQ18 has 2 MiB of FLASH beginning at address 0x0000:0000 and
 * 256 KiB of SRAM beginning at address 0x1fff:0000- (SRAM_L) (64Kib) and
 * 0x2000:0000 196 Kib  (SRAM_U).
 *
 * NOTE: that the first part of the K66 FLASH region is reserved for
 *       interrupt vectflash and, following that, is a region from 0x0000:0400
 *       to 0x0000:040f that is reserved for the FLASH control fields (FCF).
 *
 * NOTE: The on-chip RAM is split evenly among SRAM_L and SRAM_U. The RAM is
 *       also implemented such that the SRAM_L and SRAM_U ranges form a
 *       contiguous block in the memory map.
 */

MEMORY
{
	vectflash  (rx)  : ORIGIN = 0x00006000, LENGTH = 1K
	cfmprotect (rx)  : ORIGIN = 0x00006400, LENGTH = 16
	progflash  (rx)  : ORIGIN = 0x00006410, LENGTH = 2M - ((6*4K) + 1K + 16)
	datasram  (rwx)  : ORIGIN = 0x1fff0000, LENGTH = 256K
}

OUTPUT_ARCH(arm)
ENTRY(__start)
EXTERN(__flashconfigbytes)
SECTIONS
{
	.vectors : {
		_svectors = ABSOLUTE(.);
		 KEEP(*(.vectors))
		_evectors = ABSOLUTE(.);
	} > vectflash

	.cfmprotect : {
		KEEP(*(.cfmconfig))
	} > cfmprotect

	.text : {
		_stext = ABSOLUTE(.);
		*(.text .text.*)
		*(.fixup)
		*(.gnu.warning)
		*(.rodata .rodata.*)
		*(.gnu.linkonce.t.*)
		*(.got)
		*(.gcc_except_table)
		*(.gnu.linkonce.r.*)
		_etext = ABSOLUTE(.);

		 . = ALIGN(8);
		 FILL(0xff)
		 . += 8;
	} > progflash =0xff

	/*
	 * Init functions (static constructors and the like)
	 */

	.init_section : {
		_sinit = ABSOLUTE(.);
         KEEP(*(.init_array .init_array.*))
		_einit = ABSOLUTE(.);
		 . = ALIGN(8);
		 FILL(0xff)
		 . += 8;
	} > progflash

	.ARM.extab : {
		*(.ARM.extab*)
		 . = ALIGN(8);
		 FILL(0xff)
		 . += 8;
	} > progflash

	__exidx_start = ABSOLUTE(.);
	.ARM.exidx : {
		*(.ARM.exidx*)
	} > progflash
	__exidx_end = ABSOLUTE(.);

	_eronly = ABSOLUTE(.);

	.data : {
		_sdata = ABSOLUTE(.);
		*(.data .data.*)
		*(.gnu.linkonce.d.*)
		CONSTRUCTORS
		_edata = ABSOLUTE(.);
		 . = ALIGN(8);
		 FILL(0xff)
		 . += 8;
	} > datasram AT > progflash

	.ramfunc ALIGN(4): {
		_sramfuncs = ABSOLUTE(.);
		*(.ramfunc  .ramfunc.*)
		_eramfuncs = ABSOLUTE(.);
		 . = ALIGN(8);
		 FILL(0xff)
		 . += 8;
	} > datasram AT > progflash =0xff

	_framfuncs = LOADADDR(.ramfunc);

	.bss : {
		_sbss = ABSOLUTE(.);
		*(.bss .bss.*)
		*(.gnu.linkonce.b.*)
		*(COMMON)
		. = ALIGN(4);
		_ebss = ABSOLUTE(.);
	} > datasram

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
