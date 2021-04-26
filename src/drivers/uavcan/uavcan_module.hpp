/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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

#pragma once

#include <px4_platform_common/px4_config.h>

#include <drivers/device/device.h>

/**
 * @file uavcan.hpp
 *
 * Public header for the UAVCAN module
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 * @author David Sidrane <david_s5@nscdg.com>
 * @author Holger Steinhaus <holger@steinhaus-home.de>
 */

// firmware paths
#define UAVCAN_MAX_PATH_LENGTH (128 + 40)
#define UAVCAN_SD_ROOT_PATH    "/fs/microsd/"
#define UAVCAN_FIRMWARE_PATH   UAVCAN_SD_ROOT_PATH"ufw"
#define UAVCAN_ROMFS_FW_PATH   "/etc/uavcan/fw"
#define UAVCAN_ROMFS_FW_PREFIX "_"

// logging
#define UAVCAN_NODE_DB_PATH UAVCAN_SD_ROOT_PATH"/uavcan.db"
#define UAVCAN_LOG_FILE     UAVCAN_NODE_DB_PATH"/trace.log"

// device files
// TODO: split IOCTL interface in ESC and node related functionality, then change UAVCAN_DEVICE_PATH to "/dev/uavcan/node"
#define UAVCAN_DEVICE_PATH     "/dev/uavcan/esc"
#define UAVCAN_ESC_DEVICE_PATH "/dev/uavcan/esc"

// ioctl interface
#define _UAVCAN_IOC(_n)               (_IOC(_UAVCAN_IOCBASE, _n))
#define _UAVCAN_IOCBASE               (0x4000)                        // IOCTL base for module UAVCAN
/*
 * Query if node identification is in progress. Returns:
 *      EINVAL - not applicable in the current operating mode
 *      ETIME  - network discovery complete
 *      OK (0) - network discovery in progress
 */
#define UAVCAN_IOCG_NODEID_INPROGRESS _UAVCAN_IOC(1)
/*
 * Set hardpoint command. Accepts a pointer to uavcan::equipment::hardpoint::Command; returns nothing.
 * The pointer may be invalidated once the call returns.
 */
#define UAVCAN_IOCS_HARDPOINT_SET       _UAVCAN_IOC(10)

// public prototypes
extern "C" __EXPORT int uavcan_main(int argc, char *argv[]);
