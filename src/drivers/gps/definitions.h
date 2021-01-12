/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file definitions.h
 * common platform-specific definitions & abstractions for gps
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <px4_platform_common/defines.h>
#include <uORB/topics/satellite_info.h>
#include <uORB/topics/sensor_gps.h>

#define GPS_INFO(...) PX4_INFO(__VA_ARGS__)
#define GPS_WARN(...) PX4_WARN(__VA_ARGS__)
#define GPS_ERR(...) PX4_ERR(__VA_ARGS__)

#define gps_usleep px4_usleep

/**
 * Get the current time in us. Function signature:
 * uint64_t hrt_absolute_time()
 */
#define gps_absolute_time hrt_absolute_time
typedef hrt_abstime gps_abstime;


// TODO: this functionality is not available on the Snapdragon yet
#ifdef __PX4_QURT
#define NO_MKTIME
#endif
