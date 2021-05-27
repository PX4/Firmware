/****************************************************************************
 * kernel-space.ld
 *
 *   Copyright (C) 2020 Technology Innovation Institute. All rights reserved.
 *   Author: Jukka Laitinen <jukka.laitinen@ssrc.tii.ae>
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

/* NOTE:  This depends on the memory.ld script having been included prior to
 * this script.
 */

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
    } > kflash

    /*
     * Init functions (static constructors and the like)
     */
    .init_section : {
        _sinit = ABSOLUTE(.);
	KEEP(*(.init_array .init_array.*))
        _einit = ABSOLUTE(.);
    } > kflash

    /*
     * Construction data for parameters.
     */
     __param ALIGN(4): {
     __param_start = ABSOLUTE(.);
     KEEP(*(__param*))
     __param_end = ABSOLUTE(.);
     } > kflash

    .ARM.extab : {
        *(.ARM.extab*)
    } > kflash

    __exidx_start = ABSOLUTE(.);
    .ARM.exidx : {
        *(.ARM.exidx*)
    } > kflash

    __exidx_end = ABSOLUTE(.);

    _eronly = ABSOLUTE(.);

    .data : {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > ksram AT > kflash

    .bss : {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = ABSOLUTE(.);
    } > ksram

    /* Stabs debugging sections */

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
    } > ksram AT > kflash

    _framfuncs = LOADADDR(.ramfunc);
}
