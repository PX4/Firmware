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
* @file userspace_init.c
*
* userspace_init command. Used in startup scripts to initialize
* px4 user space services
*
* @author Jukka Laitinen <jukkax@ssrc.tii.ae>
*/

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <parameters/param.h>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>

extern "C" int userspace_init_main(int argc, char *argv[])
{
	/* defaults to an error */
	int ret = 1;

#if defined(__PX4_NUTTX) && !defined(CONFIG_BUILD_FLAT)
	PX4_INFO("Initializing PX4 userspace");

	hrt_init();

	param_init();

	px4::WorkQueueManagerStart();

#endif

	return ret;
}
