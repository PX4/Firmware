/****************************************************************************
 *
 *   Copyright (c) 2017-2020 PX4 Development Team. All rights reserved.
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
 * @file modified from sf0x_parser.cpp
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Chuong Nguyen <chnguye7@asu.edu>
 * @author Ayush Gaud <ayush.gaud@gmail.com>
 *
 * Declarations of parser for the Benewake TFmini laser rangefinder series
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
// 6) Reserved bytes
// 7) Original signal quality degree
// 8) Checksum parity bit (low 8bit), Checksum = Byte1 + Byte2 +...+Byte8. This is only a low 8bit though

// Command -and answers- Frame definitions for TFmini model
// =============================================================
// 8 bytes total per message:
// 0) 0x42
// 1) 0x57
// 2) 0x02 by default
// 3) 0x00 on sending - 0x01 or 0xFF or 0x0f on receiving
// 4;5) EE FF double byte parameter, EE is low 8 bit; FF is high 8 bits
// 6) GG single byte parameter
// 7) HH instruction code

// Command -and answers- Frame definitions for TFmini-plus model
// =============================================================
// 0) 0x5A
// 1) Len: the total length of the frame（include Head and Checksum，unit: byte)
// 2) ID: identifier code of command
// 3;N-2) Data: data segment. Little endian format
// N-1) Checksum: sum of all bytes from Head to payload. Lower 8 bits

#define TFMINI_DATA_HEADER 0x59
#define TFMINI_CMD_HEADER1 0x42
#define TFMINI_CMD_HEADER2 0x57
#define TFMINI_CMD_SIZE 8

#define TFMINIPLUS_CMD_HEADER1 0x5A
#define TFMINIPLUS_INVALID_MEASURE 0xFFFF

enum class TFMINI_MODEL {
	MODEL_UNKNOWN = 0,
	MODEL_TFMINI,
	MODEL_TFMINIPLUS
};

enum class TFMINI_PARSE_STATE {
	STATE0_UNSYNC = 0,
	STATE1_SYNC_1,
	STATE1_SYNC_2,
	STATE2_GOT_DIST_L,
	STATE2_GOT_DIST_H,
	STATE3_GOT_STRENGTH_L,
	STATE3_GOT_STRENGTH_H,
	STATE4_GOT_RESERVED,
	STATE5_GOT_QUALITY,
	STATE6_GOT_CHECKSUM,
	STATE7_GOT_COMMAND_RESPONSE,
	STATE8_GOT_RESPONSE_CHECKSUM
};

int tfmini_parse(uint8_t c, uint8_t *parserbuf, unsigned *parserbuf_index, TFMINI_PARSE_STATE *state,
		 TFMINI_MODEL hw_model, float *dist, uint16_t *strength, float *temperature, uint8_t *commandresponse,
		 uint8_t *commandresponse_size);
