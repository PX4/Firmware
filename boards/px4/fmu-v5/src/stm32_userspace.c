/****************************************************************************
 * boards/arm/stm32f7/stm32f746g-disco/kernel/stm32_userspace.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>

#include <nuttx/arch.h>
#include <nuttx/mm/mm.h>
#include <nuttx/wqueue.h>
#include <nuttx/userspace.h>

#if !defined(CONFIG_BUILD_FLAT) && !defined(__KERNEL__)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_NUTTX_USERSPACE
#  error "CONFIG_NUTTX_USERSPACE not defined"
#endif

#if CONFIG_NUTTX_USERSPACE != 0x08100000
#  error "CONFIG_NUTTX_USERSPACE must be 0x08100000 to match memory.ld"
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* These 'addresses' of these values are setup by the linker script. They are
 * not actual uint32_t storage locations! They are only used meaningfully in
 * the following way:
 *
 *  - The linker script defines, for example, the symbol_sdata.
 *  - The declaration extern uint32_t _sdata; makes C happy.  C will believe
 *    that the value _sdata is the address of a uint32_t variable _data (it
 *    is not!).
 *  - We can recover the linker value then by simply taking the address of
 *    of _data.  like:  uint32_t *pdata = &_sdata;
 */

extern uint32_t _stext;           /* Start of .text */
extern uint32_t _etext;           /* End_1 of .text + .rodata */
extern const uint32_t _eronly;    /* End+1 of read only section (.text + .rodata) */
extern uint32_t _sdata;           /* Start of .data */
extern uint32_t _edata;           /* End+1 of .data */
extern uint32_t _sbss;            /* Start of .bss */
extern uint32_t _ebss;            /* End+1 of .bss */

/* This is the user space entry point */

int CONFIG_USER_ENTRYPOINT(int argc, char *argv[]);

const struct userspace_s userspace __attribute__((section(".userspace"))) = {
	/* General memory map */

	.us_entrypoint    = (main_t)CONFIG_USER_ENTRYPOINT,
	.us_textstart     = (uintptr_t) &_stext,
	.us_textend       = (uintptr_t) &_etext,
	.us_datasource    = (uintptr_t) &_eronly,
	.us_datastart     = (uintptr_t) &_sdata,
	.us_dataend       = (uintptr_t) &_edata,
	.us_bssstart      = (uintptr_t) &_sbss,
	.us_bssend        = (uintptr_t) &_ebss,

	/* Memory manager heap structure */

	.us_heap          = &g_mmheap,

	/* Task/thread startup routines */

	.task_startup     = nxtask_startup,
#ifndef CONFIG_DISABLE_PTHREAD
	.pthread_startup  = pthread_startup,
#endif

	/* Signal handler trampoline */

	.signal_handler   = up_signal_handler,

	/* User-space work queue support (declared in include/nuttx/wqueue.h) */

#ifdef CONFIG_LIB_USRWORK
	.work_usrstart    = work_usrstart,
#endif
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* !CONFIG_BUILD_FLAT && !__KERNEL__ */
