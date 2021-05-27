/****************************************************************************
*
* Copyright (c) 2020 Technology Innovation Institute. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in
* the documentation and/or other materials provided with the
* distribution.
* 3. Neither the name PX4 nor the names of its contributors may be
* used to endorse or promote products derived from this software
* without specific prior written permission.
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
* @file px4_insmod.c
*
* px4_insmod command. Used to launch kernel modules
*
* @author Jukka Laitinen <jukkax@ssrc.tii.ae>
*/

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform/board_ctrl.h>
#include <sys/boardctl.h>

/* This function is not intended to be called from
   command line, it is only used in builtin_list.h
   to implicitly launch a module in kenel side.
   See builtin_list.h for details.
*/
extern "C" int launch_kmod_main(int argc, char *argv[])
{
	px4_boardctl_t module_args = {argc, argv, PX4_ERROR};
	boardctl(PX4_KERNEL_CMD, (uintptr_t)&module_args);
	return module_args.ret;
}


/* This function is used to explicitly launch a kernel
   side module from nsh shell
*/
extern "C" int px4_insmod_main(int argc, char *argv[])
{
	/* defaults to an error */
	int ret = 1;

#if defined(__PX4_NUTTX) && !defined(CONFIG_BUILD_FLAT)

	if (argc > 1) {
		ret = launch_kmod_main(argc - 1, argv + 1);
	}

#endif

	return ret;
}
