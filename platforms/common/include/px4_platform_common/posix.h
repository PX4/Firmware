/****************************************************************************
 *
 *   Copyright (c) 2015 Mark Charlebois. All rights reserved.
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
 * @file px4_posix.h
 *
 * Includes POSIX-like functions for virtual character devices
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/tasks.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>

#if defined(__PX4_QURT)
#include <dspal_types.h>
#else
#include <sys/types.h>
#endif

#include "sem.h"

#define  PX4_F_RDONLY 1
#define  PX4_F_WRONLY 2

#ifdef __PX4_NUTTX

#include <poll.h>

typedef struct pollfd px4_pollfd_struct_t;
typedef pollevent_t px4_pollevent_t;

#if defined(__cplusplus)
#define _GLOBAL ::
#else
#define _GLOBAL
#endif
#define px4_open 	_GLOBAL open
#define px4_close 	_GLOBAL close
#define px4_ioctl 	_GLOBAL ioctl
#define px4_write 	_GLOBAL write
#define px4_read 	_GLOBAL read
#define px4_poll 	_GLOBAL poll
#define px4_access 	_GLOBAL access
#define px4_getpid 	_GLOBAL getpid

#define  PX4_STACK_OVERHEAD	0

#elif defined(__PX4_POSIX)

#define	 PX4_STACK_OVERHEAD	(1024 * 24)

#define px4_cache_aligned_data()
#define px4_cache_aligned_alloc malloc

__BEGIN_DECLS

typedef short px4_pollevent_t;

typedef struct {
	/* This part of the struct is POSIX-like */
	int		fd;       /* The descriptor being polled */
	px4_pollevent_t 	events;   /* The input event flags */
	px4_pollevent_t 	revents;  /* The output event flags */

	/* Required for PX4 compatibility */
	px4_sem_t   *sem;  	/* Pointer to semaphore used to post output event */
	void   *priv;     	/* For use by drivers */
} px4_pollfd_struct_t;

#ifndef POLLIN
#define POLLIN       (0x01)
#endif

__EXPORT int 		px4_open(const char *path, int flags, ...);
__EXPORT int 		px4_close(int fd);
__EXPORT ssize_t	px4_read(int fd, void *buffer, size_t buflen);
__EXPORT ssize_t	px4_write(int fd, const void *buffer, size_t buflen);
__EXPORT int		px4_ioctl(int fd, int cmd, unsigned long arg);
__EXPORT int		px4_poll(px4_pollfd_struct_t *fds, unsigned int nfds, int timeout);
__EXPORT int		px4_access(const char *pathname, int mode);
__EXPORT px4_task_t	px4_getpid(void);

__END_DECLS
#else
#error "No TARGET OS Provided"
#endif


// The stack size is intended for 32-bit architectures; therefore
// we often run out of stack space when pointers are larger than 4 bytes.
// Double the stack size on posix when we're on a 64-bit architecture.
// Most full-scale OS use 1-4K of memory from the stack themselves
#define PX4_STACK_ADJUSTED(_s) (_s * (__SIZEOF_POINTER__ >> 2) + PX4_STACK_OVERHEAD)

__BEGIN_DECLS

__EXPORT void		px4_show_files(void);

__END_DECLS
