/****************************************************************************
 *
 *   Copyright (C) 2020 Technology Innovation Institute. All rights reserved.
 *                 Author: Jukka Laitinen <jukkax@ssrc.tii.ae>
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

#ifndef PX4_BOARD_CTRL_H_
#define PX4_BOARD_CTRL_H_

#define _PLATFORMIOCBASE		(0x4000)
#define _PLATFORMIOC(_n)		(_PX4_IOC(_PLATFORMIOCBASE, _n))
#define PX4_KERNEL_CMD			_PLATFORMIOC(1)

typedef struct px4_boardctl {
	int argc;
	char **argv;
	int ret;
} px4_boardctl_t;

typedef int (*ioctl_ptr_t)(unsigned int, unsigned long);

#ifdef __cplusplus
extern "C" {
#endif

/* Function to register a px4 boardctl handler */
int px4_register_boardct_ioctl(unsigned base, ioctl_ptr_t func);

#ifdef __cplusplus
}
#endif

#endif
