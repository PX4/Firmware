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

#include <px4_platform_common/init.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_manifest.h>
#include <px4_platform_common/console_buffer.h>
#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>
#include <px4_platform/cpuload.h>
#include <uORB/uORB.h>

#include <fcntl.h>

int px4_platform_console_init(void)
{
#if !defined(CONFIG_DEV_CONSOLE) && defined(CONFIG_DEV_NULL)

	/* Support running nsh on a board with out a console
	 * Without this the assumption that the fd 0..2 are
	 * std{in..err} will be wrong. NSH will read/write to the
	 * fd it opens for the init script or nested scripts assigned
	 * to fd 0..2.
	 *
	 */

	int fd = open("/dev/null", O_RDWR);

	if (fd == 0) {
		/* Successfully opened /dev/null as stdin (fd == 0) */

		(void)fs_dupfd2(0, 1);
		(void)fs_dupfd2(0, 2);
		(void)fs_fdopen(0, O_RDONLY,         NULL, NULL);
		(void)fs_fdopen(1, O_WROK | O_CREAT, NULL, NULL);
		(void)fs_fdopen(2, O_WROK | O_CREAT, NULL, NULL);

	} else {
		/* We failed to open /dev/null OR for some reason, we opened
		 * it and got some file descriptor other than 0.
		 */

		if (fd > 0) {
			(void)close(fd);
		}

		return -ENFILE;

	}

#endif
	return OK;
}


#if !defined(CONFIG_BUILD_FLAT)
typedef CODE void (*initializer_t)(void);
extern initializer_t _sinit;
extern initializer_t _einit;
extern uintptr_t _stext;
extern uintptr_t _etext;

static void cxx_initialize(void)
{
	initializer_t *initp;

	/* Visit each entry in the initialization table */

	for (initp = &_sinit; initp != &_einit; initp++) {
		initializer_t initializer = *initp;

		/* Make sure that the address is non-NULL and lies in the text
		* region defined by the linker script.  Some toolchains may put
		* NULL values or counts in the initialization table.
		*/

		if ((FAR void *)initializer >= (FAR void *)&_stext &&
		    (FAR void *)initializer < (FAR void *)&_etext) {
			initializer();
		}
	}
}
#endif


int px4_platform_init(void)
{
	/* In protected/kernel build this is called from user space thread via
	 *  BOARDCTL.
	 *  In PX4 there are static C++ objects created also in kernel side;
	 *  initialize them here
	 */
#if !defined(CONFIG_BUILD_FLAT)
	cxx_initialize();
#endif
	int ret = px4_platform_console_init();

	if (ret < 0) {
		return ret;
	}

	ret = px4_console_buffer_init();

	if (ret < 0) {
		return ret;
	}

	// replace stdout with our buffered console
	int fd_buf = open(CONSOLE_BUFFER_DEVICE, O_WRONLY);

	if (fd_buf >= 0) {
		dup2(fd_buf, 1);
		// keep stderr(2) untouched: the buffered console will use it to output to the original console
		close(fd_buf);
	}

	hrt_init();

	param_init();

	/* configure CPU load estimation */
#ifdef CONFIG_SCHED_INSTRUMENTATION
	cpuload_initialize_once();
#endif

	px4::WorkQueueManagerStart();

	uorb_start();

	px4_log_initialize();

	return PX4_OK;
}

int px4_platform_configure(void)
{
	return px4_mft_configure(board_get_manifest());

}
