/****************************************************************************
 *
 *   Copyright (c) 2017-2021 PX4 Development Team. All rights reserved.
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
 * @file tfmini_plus_parser.h - modified from tfmini_parser.h
 * @author Eren Ipek <eren.ipek@maxwell-innovations.com>
 *
 * Declarations of parser for the Benewake TFmini Plus laser rangefinder series
 */

#pragma once

#include <stdint.h>
#include <stdlib.h>

// Data Format for Benewake TFmini models
// ======================================
// 9 bytes total per message:
// 0) 0x59
// 1) 0x59
// 2) Dist_L (low 8bit)
// 3) Dist_H (high 8bit)
// 4) Strength_L (low 8bit)
// 5) Strength_H (high 8bit)
// 6) Temperature_H
// 7) Temperature_L
// 8) Checksum parity bit (low 8bit), Checksum = Byte1 + Byte2 +...+Byte8. This is only a low 8bit though

// Command -and answers- Frame definitions for TFmini-plus model
// =============================================================
// 0) 0x5A
// 1) Len: the total length of the frame（include Head and Checksum，unit: byte)
// 2) ID: identifier code of command
// 3;N-2) Data: data segment. Little endian format
// N-1) Checksum: sum of all bytes from Head to payload. Lower 8 bits

#define TFMINI_PLUS_DATA_HEADER 0x59

#define TFMINI_PLUS_CMD_HEADER1 0x5A

enum class TFMINI_PLUS_PARSE_STATE {
	STATE0_UNSYNC = 0,
	STATE1_SYNC_1,
	STATE1_SYNC_2,
	STATE2_GOT_DIST_L,
	STATE2_GOT_DIST_H,
	STATE3_GOT_STRENGTH_L,
	STATE3_GOT_STRENGTH_H,
	STATE4_TEMPL,
	STATE5_TEMPH,
	STATE6_GOT_CHECKSUM
};

int tfmini_parse(uint8_t c, TFMINI_PLUS_PARSE_STATE *state,
		 float *dist, uint16_t *strength, float *temperature);
