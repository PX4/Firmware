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
 * @file tfmini_plus_parser.cpp - modified from tfmini_parser.cpp
 * @author Eren Ipek <eren.ipek@maxwell-innovations.com>
 *
 * Declarations of parser for the Benewake TFmini Plus laser rangefinder series
 */

#include "tfmini_plus_parser.h"
#include <string.h>

// #define TFMINI_DEBUG

#ifdef TFMINI_DEBUG
#include <stdio.h>

const char *parser_state[] = {
	"0_UNSYNC",
	"1_SYNC_1",
	"1_SYNC_2",
	"2_GOT_DIST_L",
	"2_GOT_DIST_H",
	"3_GOT_STRENGTH_L",
	"3_GOT_STRENGTH_H",
	"4_GOT_TEMPERATURE_L",
	"5_GOT_TEMPERATURE_h",
	"6_GOT_RESPONSE_CHECKSUM"
};
#endif

int tfmini_parse(uint8_t c, TFMINI_PLUS_PARSE_STATE *state,
		 float *dist, uint16_t *strength, float *temperature)
{
	int ret = -1;
	static uint8_t parserbuf[9];
	static unsigned parserbuf_index = 0;
	unsigned char cksm = 0;

	switch (*state) {
	case TFMINI_PLUS_PARSE_STATE::STATE6_GOT_CHECKSUM:
		if (c == TFMINI_PLUS_DATA_HEADER) {
			*state = TFMINI_PLUS_PARSE_STATE::STATE1_SYNC_1;
			parserbuf[parserbuf_index] = c;
			(parserbuf_index)++;

		} else {
			*state = TFMINI_PLUS_PARSE_STATE::STATE0_UNSYNC;
		}

		break;

	case TFMINI_PLUS_PARSE_STATE::STATE0_UNSYNC:
		if (c == TFMINI_PLUS_DATA_HEADER) {
			*state = TFMINI_PLUS_PARSE_STATE::STATE1_SYNC_1;
			parserbuf[parserbuf_index] = c;
			(parserbuf_index)++;

		} else {
			*state = TFMINI_PLUS_PARSE_STATE::STATE0_UNSYNC;
			parserbuf_index = 0;
		}

		break;

	case TFMINI_PLUS_PARSE_STATE::STATE1_SYNC_1:
		if (c == TFMINI_PLUS_DATA_HEADER) {
			*state = TFMINI_PLUS_PARSE_STATE::STATE1_SYNC_2;
			parserbuf[parserbuf_index] = c;
			(parserbuf_index)++;

		} else {
			*state = TFMINI_PLUS_PARSE_STATE::STATE0_UNSYNC;
			parserbuf_index = 0;
		}

		break;

	case TFMINI_PLUS_PARSE_STATE::STATE1_SYNC_2:
		*state = TFMINI_PLUS_PARSE_STATE::STATE2_GOT_DIST_L;
		parserbuf[parserbuf_index] = c;
		(parserbuf_index)++;

		break;

	case TFMINI_PLUS_PARSE_STATE::STATE2_GOT_DIST_L:
		*state = TFMINI_PLUS_PARSE_STATE::STATE2_GOT_DIST_H;
		parserbuf[parserbuf_index] = c;
		(parserbuf_index)++;

		break;

	case TFMINI_PLUS_PARSE_STATE::STATE2_GOT_DIST_H:
		*state = TFMINI_PLUS_PARSE_STATE::STATE3_GOT_STRENGTH_L;
		parserbuf[parserbuf_index] = c;
		(parserbuf_index)++;

		break;

	case TFMINI_PLUS_PARSE_STATE::STATE3_GOT_STRENGTH_L:
		*state = TFMINI_PLUS_PARSE_STATE::STATE3_GOT_STRENGTH_H;
		parserbuf[parserbuf_index] = c;
		(parserbuf_index)++;

		break;

	case TFMINI_PLUS_PARSE_STATE::STATE3_GOT_STRENGTH_H:
		*state = TFMINI_PLUS_PARSE_STATE::STATE4_TEMPL;
		parserbuf[parserbuf_index] = c;
		(parserbuf_index)++;

		break;

	case TFMINI_PLUS_PARSE_STATE::STATE4_TEMPL:
		*state = TFMINI_PLUS_PARSE_STATE::STATE5_TEMPH;
		parserbuf[parserbuf_index] = c;
		(parserbuf_index)++;

		break;

	case TFMINI_PLUS_PARSE_STATE::STATE5_TEMPH:
		// Find the checksum
		cksm = 0;

		for (int i = 0; i < 8; i++) {
			cksm += parserbuf[i];
		}

		if (c == cksm) {
			parserbuf[parserbuf_index] = '\0';
			*dist = static_cast<float>(parserbuf[2] + (parserbuf[3] << 8)) / 100.f;
			*strength = parserbuf[4] + (parserbuf[5] << 8);
			*temperature = static_cast<float>(parserbuf[6] + (parserbuf[7] << 8)) / 8.f - 256.f;
			*state = TFMINI_PLUS_PARSE_STATE::STATE6_GOT_CHECKSUM;
			parserbuf_index = 0;
			ret = 0;

		} else {
			*state = TFMINI_PLUS_PARSE_STATE::STATE0_UNSYNC;
			parserbuf_index = 0;
		}

		break;

	}

#ifdef TFMINI_DEBUG
	printf("state: TFMINI_PLUS_PARSE_STATE%s\n", parser_state[*state]);
#endif

	return ret;
}
