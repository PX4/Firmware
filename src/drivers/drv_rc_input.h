/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file drv_rc_input.h
 *
 * R/C input interface.
 */

#ifndef _DRV_RC_INPUT_H
#define _DRV_RC_INPUT_H

#include <stdint.h>
#include <sys/ioctl.h>
#include <uORB/topics/input_rc.h>

#include "drv_orb_dev.h"

/**
 * Path for the default R/C input device.
 *
 * Note that on systems with more than one R/C input path (e.g.
 * PX4FMU with PX4IO connected) there may be other devices that
 * respond to this protocol.
 *
 * Input data may be obtained by subscribing to the input_rc
 * object, or by poll/reading from the device.
 */
#define RC_INPUT0_DEVICE_PATH	"/dev/input_rc0"

/**
 * Input signal type, value is a control position from zero to 100
 * percent.
 */
typedef uint16_t		rc_input_t;

#define _RC_INPUT_BASE		0x2b00

/** Enable RSSI input via ADC */
#define RC_INPUT_ENABLE_RSSI_ANALOG	_IOC(_RC_INPUT_BASE, 1)

/** Enable RSSI input via PWM signal */
#define RC_INPUT_ENABLE_RSSI_PWM	_IOC(_RC_INPUT_BASE, 2)

#endif /* _DRV_RC_INPUT_H */
