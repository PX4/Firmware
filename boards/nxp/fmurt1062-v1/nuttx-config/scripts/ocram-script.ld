/****************************************************************************
 * boards/arm/imxrt/imxrt1060-evk/scripts/flash-ocram.ld
 *
 *   Copyright (C) 2018, 2020 Gregory Nutt. All rights reserved.
 *   Authors: Ivan Ucherdzhiev <ivanucherdjiev@gmail.com>
 *            David Sidrane <david.sidrane@nscdg.com>
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
 * configuration.
 *
 * The default flexram setting on the iMXRT 1062 is
 * 256Kib to OCRRAM, 128Kib ITCM and 128Kib DTCM.
 * This can be changed by using a dcd by minipulating
 * IOMUX GPR16 and GPR17.
 * The configuartion we will use is 384Kib to OCRRAM, 0Kib ITCM and
 * 128Kib DTCM.
 *
 * This is the OCRAM inker script.
 * The NXP ROM bootloader will move the FLASH image to OCRAM.
 * We must reserve 32K for the bootloader' OCRAM usage from  the OCRAM Size
 * and an additinal 8K for the ivt_s which is IVT_SIZE(8K) This 40K can be
 * reused once the application is running.
 *
 *      0x2020:A000 to 0x202d:ffff - The application Image's vector table
 *      0x2020:8000 to 0x2020:A000 - IVT
 *      0x2020:0000 to 0x2020:7fff - NXP ROM bootloader.
 *
 * We artificially split the FLASH to allow locating sections that we do not
 * want loaded inoto OCRAM. This is to save on OCRAM where the speen of the
 * code does not matter.
 *
 */

MEMORY
{
  flash    (rx)  : ORIGIN = 0x60000000, LENGTH = 7M
  flashxip (rx)  : ORIGIN = 0x60700000, LENGTH = 1M
  /*                     Vectors @ boot+ivt   OCRAM2   Flex RAM     Boot  IVT */
  sram     (rwx) : ORIGIN = 0x2020A000, LENGTH = 512K + 256K + 128K - (32K + 8K)
  itcm     (rwx) : ORIGIN = 0x00000000, LENGTH = 0K
  dtcm     (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

OUTPUT_ARCH(arm)
EXTERN(_vectors)
EXTERN(g_flash_config)
EXTERN(g_image_vector_table)
EXTERN(g_boot_data)
EXTERN(g_dcd_data)

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
    } > flash

	/* Catch all the section we want not in OCRAM so that the *(.text .text.*) in flash does not */

    .flashxip : ALIGN(4)
	{
		FILL(0xff)
		/* Order matters */
		*(.text.__start)
		*(.text.imxrt_ocram_initialize)

		*(.slow_memory)
		*(.text.*ModuleBase*)
		*(.text.*print_message*)
		*(.text._ZN4Ekf2C2Eb)
		*(.text._ZN9CommanderC2Ev)
		*(.text.*PreFlightCheck*)
		*(.text.*calibrat*)
		*(.text.*initEv)
		*(.text.*probe*)
		*(.text.*thread_main*);
		*(.text.*listener*)
		*(.text.*BlockLocalPositionEstimator*)
		*(.text.nsh_*)
		*(.text.lib_vscanf)
		*(.text.lib_vsprintf)
		*(.text.*configure_streams_to_default*)
		*(.text.*_main)
		*(.text.*GPSDriverAshtech*)
		*(.rodata .rodata.*)
		*(.fixup)
		*(.gnu.warning)
		*(.gnu.linkonce.t.*)
		*(.glue_7)
		*(.glue_7t)
		*(.got)
		*(.gcc_except_table)
		*(.gnu.linkonce.r.*)

    } > flashxip

	/* Sections that will go to OCRAM */

    .text :
    {
        _stext = ABSOLUTE(.);
        *(.vectors)
        *(.text .text.*)
        _etext = ABSOLUTE(.);

    } > sram AT> flash

	/*
	 * Init functions (static constructors and the like)
	 */

    .init_section :
    {
        _sinit = ABSOLUTE(.);
         KEEP(*(.init_array .init_array.*))
        _einit = ABSOLUTE(.);
    }  > flash

    .ARM.extab :
    {
        *(.ARM.extab*)
    } > flash

	__exidx_start = ABSOLUTE(.);

    .ARM.exidx :
    {
        *(.ARM.exidx*)
        __exidx_end = ABSOLUTE(.);
    } > flash

    _eronly = ABSOLUTE(.);

    .data :
    {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
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
        . = ALIGN(4);
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
