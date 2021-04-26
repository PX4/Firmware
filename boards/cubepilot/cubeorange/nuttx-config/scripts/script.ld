/****************************************************************************
 * scripts/script.ld
 *
 *   Copyright (C) 2020 Gregory Nutt. All rights reserved.
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

/* The board uses an STM32H743XIH6 and has 2048Kb of main FLASH memory.
 * The flash memory is partitioned into a User Flash memory and a System
 * Flash memory. Each of these memories has two banks:
 *
 *   1) User Flash memory:
 *
 *      Bank 1: Start address 0x0800:0000 to 0x080F:FFFF with 8 sectors, 128Kb each
 *      Bank 2: Start address 0x0810:0000 to 0x081F:FFFF with 8 sectors, 128Kb each
 *
 *   2) System Flash memory:
 *
 *      Bank 1: Start address 0x1FF0:0000 to 0x1FF1:FFFF with 1 x 128Kb sector
 *      Bank 1: Start address 0x1FF4:0000 to 0x1FF5:FFFF with 1 x 128Kb sector
 *
 *   3) User option bytes for user configuration, only in Bank 1.
 *
 * In the STM32H743II, two different boot spaces can be selected through
 * the BOOT pin and the boot base address programmed in the BOOT_ADD0 and
 * BOOT_ADD1 option bytes:
 *
 *   1) BOOT=0: Boot address defined by user option byte BOOT_ADD0[15:0].
 *      ST programmed value: Flash memory at 0x0800:0000
 *   2) BOOT=1: Boot address defined by user option byte BOOT_ADD1[15:0].
 *      ST programmed value: System bootloader at 0x1FF0:0000
 *
 * There's a switch on board, the BOOT0 pin is at ground so by default,
 * the STM32 will boot to address 0x0800:0000 in FLASH unless the switch is
 * drepresed, then the boot will be from 0x1FF0:0000
 *
 * The STM32H743ZI also has 1024Kb of data SRAM.
 * SRAM is split up into several blocks and into three power domains:
 *
 *   1) TCM SRAMs are dedicated to the Cortex-M7 and are accessible with
 *      0 wait states by the Cortex-M7 and by MDMA through AHBS slave bus
 *
 *      1.1) 128Kb of DTCM-RAM beginning at address 0x2000:0000
 *
 *           The DTCM-RAM is organized as 2 x 64Kb DTCM-RAMs on 2 x 32 bit
 *           DTCM ports. The DTCM-RAM could be used for critical real-time
 *           data, such as interrupt service routines or stack / heap memory.
 *           Both DTCM-RAMs can be used in parallel (for load/store operations)
 *           thanks to the Cortex-M7 dual issue capability.
 *
 *      1.2)  64Kb of ITCM-RAM beginning at address 0x0000:0000
 *
 *           This RAM is connected to ITCM 64-bit interface designed for
 *           execution of critical real-times routines by the CPU.
 *
 *   2) AXI SRAM (D1 domain) accessible by all system masters except BDMA
 *      through D1 domain AXI bus matrix
 *
 *      2.1) 512Kb of SRAM beginning at address 0x2400:0000
 *
 *   3) AHB SRAM (D2 domain) accessible by all system masters except BDMA
 *      through D2 domain AHB bus matrix
 *
 *      3.1) 128Kb of SRAM1 beginning at address 0x3000:0000
 *      3.2) 128Kb of SRAM2 beginning at address 0x3002:0000
 *      3.3)  32Kb of SRAM3 beginning at address 0x3004:0000
 *
 *      SRAM1 - SRAM3 are one contiguous block: 288Kb at address 0x3000:0000
 *
 *   4) AHB SRAM (D3 domain) accessible by most of system masters
 *      through D3 domain AHB bus matrix
 *
 *      4.1)  64Kb of SRAM4 beginning at address 0x3800:0000
 *      4.1)   4Kb of backup RAM beginning at address 0x3880:0000
 *
 * When booting from FLASH, FLASH memory is aliased to address 0x0000:0000
 * where the code expects to begin execution by jumping to the entry point in
 * the 0x0800:0000 address range.
 */

MEMORY
{
	ITCM_RAM  (rwx) : ORIGIN = 0x00000000, LENGTH =   64K
	FLASH      (rx) : ORIGIN = 0x08020000, LENGTH = 1920K

	DTCM1_RAM (rwx) : ORIGIN = 0x20000000, LENGTH =   64K
	DTCM2_RAM (rwx) : ORIGIN = 0x20010000, LENGTH =   64K
	AXI_SRAM  (rwx) : ORIGIN = 0x24000000, LENGTH =  512K /* D1 domain AXI bus */
	SRAM1     (rwx) : ORIGIN = 0x30000000, LENGTH =  128K /* D2 domain AHB bus */
	SRAM2     (rwx) : ORIGIN = 0x30020000, LENGTH =  128K /* D2 domain AHB bus */
	SRAM3     (rwx) : ORIGIN = 0x30040000, LENGTH =   32K /* D2 domain AHB bus */
	SRAM4     (rwx) : ORIGIN = 0x38000000, LENGTH =   64K /* D3 domain */
	BKPRAM    (rwx) : ORIGIN = 0x38800000, LENGTH =    4K
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

	} > FLASH

	/*
	 * Init functions (static constructors and the like)
	 */
	.init_section : {
		_sinit = ABSOLUTE(.);
		KEEP(*(.init_array .init_array.*))
		_einit = ABSOLUTE(.);
	} > FLASH


	.ARM.extab : {
		*(.ARM.extab*)
	} > FLASH

	__exidx_start = ABSOLUTE(.);
	.ARM.exidx : {
		*(.ARM.exidx*)
	} > FLASH
	__exidx_end = ABSOLUTE(.);

	_eronly = ABSOLUTE(.);

	.data : {
		_sdata = ABSOLUTE(.);
		*(.data .data.*)
		*(.gnu.linkonce.d.*)
		CONSTRUCTORS
		_edata = ABSOLUTE(.);
	} > AXI_SRAM AT > FLASH

	.bss : {
		_sbss = ABSOLUTE(.);
		*(.bss .bss.*)
		*(.gnu.linkonce.b.*)
		*(COMMON)
		. = ALIGN(4);
		_ebss = ABSOLUTE(.);
	} > AXI_SRAM

	/* Emit the the D3 power domain section for locating BDMA data */

	.sram4_reserve (NOLOAD) :
	{
		*(.sram4)
		. = ALIGN(4);
		_sram4_heap_start = ABSOLUTE(.);
	} > SRAM4

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
