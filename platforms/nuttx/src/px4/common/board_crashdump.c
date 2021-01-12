/****************************************************************************
 *
 *   Copyright (C) 2017-2019 PX4 Development Team. All rights reserved.
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

/**
 * @file board_crashdump.c
 *
 * Provides common board logic for crashdump callout
 * and hardfault log support
 */

#ifdef CONFIG_BOARD_CRASHDUMP

#include <board_config.h>

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <limits.h>
#include <errno.h>
#include <sys/ioctl.h>

#include <nuttx/board.h>

#include "arm_internal.h"
#include <systemlib/hardfault_log.h>
#include "nvic.h"

#if defined(CONFIG_STM32F7_BBSRAM) && defined(CONFIG_STM32F7_SAVE_CRASHDUMP)
#  define HAS_BBSRAM CONFIG_STM32F7_BBSRAM
#  define BBSRAM_FILE_COUNT CONFIG_STM32F7_BBSRAM_FILES
#  define SAVE_CRASHDUMP CONFIG_STM32F7_SAVE_CRASHDUMP
#elif defined(CONFIG_STM32H7_BBSRAM) && defined(CONFIG_STM32H7_SAVE_CRASHDUMP)
#  define HAS_BBSRAM CONFIG_STM32H7_BBSRAM
#  define BBSRAM_FILE_COUNT CONFIG_STM32H7_BBSRAM_FILES
#  define SAVE_CRASHDUMP CONFIG_STM32H7_SAVE_CRASHDUMP
#elif defined(CONFIG_STM32_BBSRAM) && defined(CONFIG_STM32_SAVE_CRASHDUMP)
#  define HAS_BBSRAM CONFIG_STM32_BBSRAM
#  define BBSRAM_FILE_COUNT CONFIG_STM32_BBSRAM_FILES
#  define SAVE_CRASHDUMP CONFIG_STM32_SAVE_CRASHDUMP
#endif

int board_hardfault_init(int display_to_console, bool allow_prompt)
{

	int hadCrash = -1;
#if defined(HAS_BBSRAM)

	/* NB. the use of the console requires the hrt running
	 * to poll the DMA
	 */

	/* Using Battery Backed Up SRAM */

	int filesizes[BBSRAM_FILE_COUNT + 1] = BSRAM_FILE_SIZES;

	stm32_bbsraminitialize(BBSRAM_PATH, filesizes);

#if defined(SAVE_CRASHDUMP)

	/* Panic Logging in Battery Backed Up Files */

	/*
	 * In an ideal world, if a fault happens in flight the
	 * system save it to BBSRAM will then reboot. Upon
	 * rebooting, the system will log the fault to disk, recover
	 * the flight state and continue to fly.  But if there is
	 * a fault on the bench or in the air that prohibit the recovery
	 * or committing the log to disk, the things are too broken to
	 * fly. So the question is:
	 *
	 * Did we have a hard fault and not make it far enough
	 * through the boot sequence to commit the fault data to
	 * the SD card?
	 */

	/* Do we have an uncommitted hard fault in BBSRAM?
	 *  - this will be reset after a successful commit to SD
	 */
	hadCrash = hardfault_check_status("boot");

	if (hadCrash == OK) {

		syslog(LOG_ERR, "[boot] There is a hard fault logged. Hold down the SPACE BAR," \
		       " while booting to review!\n");

		/* Yes. So add one to the boot count - this will be reset after a successful
		 * commit to SD
		 */

		int reboots = hardfault_increment_reboot("boot", false);

		if (reboots < 0) {
			return -EIO;
		}

		if (reboots >= 32000) {
			reboots = hardfault_increment_reboot("boot", true);
			return -ENOSPC;
		}

		/* Also end the misery for a user that holds for a key down on the console */

		int bytesWaiting = 0;
		ioctl(fileno(stdin), FIONREAD, (unsigned long)((uintptr_t) &bytesWaiting));

		if (reboots > display_to_console || bytesWaiting != 0) {

			/* Since we can not commit the fault dump to disk. Display it
			 * to the console.
			 */

			if (!allow_prompt) {
				bytesWaiting = 0;
			}

			if (display_to_console != INT_MAX) {
				hardfault_write("boot", fileno(stdout), HARDFAULT_DISPLAY_FORMAT, false);
			}

			syslog(LOG_ERR, "[boot] There were %d reboots with Hard fault that were not committed to disk%s\n",
			       reboots,
			       (bytesWaiting == 0 ? "" : " - Boot halted Due to Key Press\n"));


			/* For those of you with a debugger set a break point on up_assert and
			 * then set dbgContinue = 1 and go.
			 */

			/* Clear any key press that got us here */

			volatile bool dbgContinue = bytesWaiting == 0;
			int c = '>';

			while (!dbgContinue) {

				switch (c) {

				case EOF:
				case '\n':
				case '\r':
				case ' ':
					goto read;

				default:

					putchar(c);
					putchar('\n');

					switch (c) {

					case 'D':
					case 'd':
						hardfault_write("boot", fileno(stdout), HARDFAULT_DISPLAY_FORMAT, false);
						break;

					case 'C':
					case 'c':
						hardfault_rearm("boot");
						hardfault_increment_reboot("boot", true);
						break;

					case 'B':
					case 'b':
						dbgContinue = true;
						break;

					default:
						break;
					} // Inner Switch

					syslog(LOG_INFO, "\nEnter B - Continue booting\n" \
					       "Enter C - Clear the fault log\n" \
					       "Enter D - Dump fault log\n\n?>");
					fflush(stdout);

read:

					if (!dbgContinue) {
						c = getchar();
					}

					break;

				} // outer switch
			} // for

		} // inner if
	} // outer if

#endif // SAVE_CRASHDUMP
#endif // HAS_BBSRAM
	return hadCrash == OK ? 1 : 0;
}

static void copy_reverse(stack_word_t *dest, stack_word_t *src, int size)
{
	while (size--) {
		*dest++ = *src--;
	}
}

static uint32_t *__attribute__((noinline)) __sdata_addr(void)
{
	return &_sdata;
}


__EXPORT void board_crashdump(uintptr_t currentsp, FAR void *tcb, FAR const char *filename, int lineno)
{
#ifndef BOARD_CRASHDUMP_RESET_ONLY
	/* We need a chunk of ram to save the complete context in.
	 * Since we are going to reboot we will use &_sdata
	 * which is the lowest memory and the amount we will save
	 * _should be_ below any resources we need herein.
	 * Unfortunately this is hard to test. See dead below
	 */

	fullcontext_s *pdump = (fullcontext_s *)__sdata_addr();

	(void)enter_critical_section();

	struct tcb_s *rtcb = (struct tcb_s *)tcb;

	/* Zero out everything */

	memset(pdump, 0, sizeof(fullcontext_s));

	/* Save Info */

	pdump->info.lineno = lineno;

	if (filename) {

		int offset = 0;
		unsigned int len = strlen((char *)filename) + 1;

		if (len > sizeof(pdump->info.filename)) {
			offset = len - sizeof(pdump->info.filename) ;
		}

		strncpy(pdump->info.filename, (char *)&filename[offset], sizeof(pdump->info.filename));
	}

	/* Save the value of the pointer for current_regs as debugging info.
	 * It should be NULL in case of an ASSERT and will aid in cross
	 * checking the validity of system memory at the time of the
	 * fault.
	 */

	pdump->info.current_regs = (uintptr_t) CURRENT_REGS;

	/* Save Context */


#if CONFIG_TASK_NAME_SIZE > 0
	strncpy(pdump->info.name, rtcb->name, CONFIG_TASK_NAME_SIZE);
#endif

	pdump->info.pid = rtcb->pid;

	pdump->info.fault_regs.cfsr  = getreg32(NVIC_CFAULTS);
	pdump->info.fault_regs.hfsr  = getreg32(NVIC_HFAULTS);
	pdump->info.fault_regs.dfsr  = getreg32(NVIC_DFAULTS);
	pdump->info.fault_regs.mmfsr = getreg32(NVIC_MEMMANAGE_ADDR);
	pdump->info.fault_regs.bfsr  = getreg32(NVIC_BFAULT_ADDR);
	pdump->info.fault_regs.afsr  = getreg32(NVIC_AFAULTS);
#if defined(CONFIG_ARCH_CORTEXM7)
	pdump->info.fault_regs.abfsr = getreg32(NVIC_ABFSR);
#endif
	pdump->info.flags |= eFaultRegPresent;

	/* If  current_regs is not NULL then we are in an interrupt context
	 * and the user context is in current_regs else we are running in
	 * the users context
	 */

	if (CURRENT_REGS) {
		pdump->info.stacks.interrupt.sp = currentsp;

		pdump->info.flags |= (eRegsPresent | eUserStackPresent | eIntStackPresent);
		memcpy(pdump->info.regs, (void *)CURRENT_REGS, sizeof(pdump->info.regs));
		pdump->info.stacks.user.sp = pdump->info.regs[REG_R13];

	} else {

		/* users context */
		pdump->info.flags |= eUserStackPresent;

		pdump->info.stacks.user.sp = currentsp;
	}

	if (pdump->info.pid == 0) {

		pdump->info.stacks.user.top = g_idle_topstack - 4;
		pdump->info.stacks.user.size = CONFIG_IDLETHREAD_STACKSIZE;

	} else {
		pdump->info.stacks.user.top = (uint32_t) rtcb->adj_stack_ptr;
		pdump->info.stacks.user.size = (uint32_t) rtcb->adj_stack_size;
	}

#if CONFIG_ARCH_INTERRUPTSTACK > 3

	/* Get the limits on the interrupt stack memory */

	pdump->info.stacks.interrupt.top = (uint32_t)&g_intstackbase;
	pdump->info.stacks.interrupt.size  = (CONFIG_ARCH_INTERRUPTSTACK & ~3);

	/* If In interrupt Context save the interrupt stack data centered
	 * about the interrupt stack pointer
	 */

	if ((pdump->info.flags & eIntStackPresent) != 0) {
		stack_word_t *ps = (stack_word_t *) pdump->info.stacks.interrupt.sp;
		copy_reverse(pdump->istack, &ps[arraySize(pdump->istack) / 2], arraySize(pdump->istack));
	}

	/* Is it Invalid? */

	if (!(pdump->info.stacks.interrupt.sp <= pdump->info.stacks.interrupt.top &&
	      pdump->info.stacks.interrupt.sp > pdump->info.stacks.interrupt.top - pdump->info.stacks.interrupt.size)) {
		pdump->info.flags |= eInvalidIntStackPrt;
	}

#endif

	/* If In interrupt context or User save the user stack data centered
	 * about the user stack pointer
	 */
	if ((pdump->info.flags & eUserStackPresent) != 0) {
		stack_word_t *ps = (stack_word_t *) pdump->info.stacks.user.sp;
		copy_reverse(pdump->ustack, &ps[arraySize(pdump->ustack) / 2], arraySize(pdump->ustack));
	}

	/* Is it Invalid? */

	if (!(pdump->info.stacks.user.sp <= pdump->info.stacks.user.top &&
	      pdump->info.stacks.user.sp > pdump->info.stacks.user.top - pdump->info.stacks.user.size)) {
		pdump->info.flags |= eInvalidUserStackPtr;
	}

	int rv = px4_savepanic(HARDFAULT_FILENO, (uint8_t *)pdump, sizeof(fullcontext_s));

	/* Test if memory got wiped because of using _sdata */

	if (rv == -ENXIO) {
		char *dead = "Memory wiped - dump not saved!";

		while (*dead) {
			arm_lowputc(*dead++);
		}

	} else if (rv == -ENOSPC) {

		/* hard fault again */

		arm_lowputc('!');
	}

#endif /* BOARD_CRASHDUMP_RESET_ONLY */

	/* All boards need to do a reset here!
	 *
	 * Since we needed a chunk of ram to save the complete
	 * context in and have corrupted it.  We can not allow
	 * the OS to run again. We used &_sdata which is the lowest memory
	 * and it could be used by the OS.
	*/

	board_reset(CONFIG_BOARD_ASSERT_RESET_VALUE);
}

#endif /* CONFIG_BOARD_CRASHDUMP */
