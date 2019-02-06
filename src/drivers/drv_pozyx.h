/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file drv_pozyx.h
 *
 * Pozyx device API
 *
 * @author Andreas Antener <andreas@uaventure.com>
 **/

#pragma once

#include <stdint.h>
#include <sys/ioctl.h>

#include "drv_sensor.h" // include sensor driver interfaces

#define POZYX_BASE_DEVICE_PATH	"/dev/pozyx"
#define POZYX0_DEVICE_PATH	"/dev/pozyx0"

#define POZYX_MSG_HEADER		0x01
#define POZYX_MSGID_BEACON_CONFIG	0x02
#define POZYX_MSGID_BEACON_DIST		0x03
#define POZYX_MSGID_POSITION 		0x04

#define POZYX_READ_LEN 			8
// buffer needs to be as large as the biggest package (18) plus POZYX_READ_LEN
#define POZYX_BUF_LEN 			26

/*
 * Messages used by the Arduino sketch to send data
 */
struct pozyx_beacon_config_s {
	uint8_t beacon_id;
	uint8_t beacon_count;
	int32_t x;
	int32_t y;
	int32_t z;
};

struct pozyx_distance_s {
	uint8_t beacon_id;
	uint32_t distance;
};

struct pozyx_position_s {
	int32_t x;
	int32_t y;
	int32_t z;
	int16_t position_error;
};
