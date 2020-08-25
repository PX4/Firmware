/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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

#ifndef _DRV_UORB_H
#define _DRV_UORB_H

/**
 * @file drv_orb_dev.h
 *
 * uORB published object driver.
 */

#include <px4_platform_common/defines.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdint.h>

#define _ORBIOCBASE		(0x2600)
#define _ORBIOC(_n)		(_PX4_IOC(_ORBIOCBASE, _n))

/*
 * IOCTLs for individual topics.
 */

/** Check whether the topic has been updated since it was last read, sets *(bool *)arg */
#define ORBIOCUPDATED		_ORBIOC(11)

/** Set the minimum interval at which the topic can be seen to be updated for this subscription */
#define ORBIOCSETINTERVAL	_ORBIOC(12)

/** Get the global advertiser handle for the topic */
#define ORBIOCGADVERTISER	_ORBIOC(13)

/** Get the priority for the topic */
#define ORBIOCGPRIORITY		_ORBIOC(14)

/** Set the queue size of the topic */
#define ORBIOCSETQUEUESIZE	_ORBIOC(15)

/** Get the minimum interval at which the topic can be seen to be updated for this subscription */
#define ORBIOCGETINTERVAL	_ORBIOC(16)

/** Check whether the topic is advertised, sets *(unsigned long *)arg to 1 if advertised, 0 otherwise */
#define ORBIOCISADVERTISED	_ORBIOC(17)

#endif /* _DRV_UORB_H */
