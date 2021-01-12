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

/**
 * @file parser.cpp
 * @author Lorenz Meier <lm@inf.ethz.ch>
 *
 * Driver for the Lightware laser rangefinder series
 */

#include "parser.h"
#include <string.h>
#include <stdlib.h>

//#define LW_DEBUG

#ifdef LW_DEBUG
#include <stdio.h>

const char *parser_state[] = {
	"0_UNSYNC",
	"1_SYNC",
	"2_GOT_DIGIT0",
	"3_GOT_DOT",
	"4_GOT_DIGIT1",
	"5_GOT_DIGIT2",
	"6_GOT_CARRIAGE_RETURN"
};
#endif

int lightware_parser(char c, char *parserbuf, unsigned *parserbuf_index, enum LW_PARSE_STATE *state, float *dist)
{
	int ret = -1;
	char *end;

	switch (*state) {
	case LW_PARSE_STATE0_UNSYNC:
		if (c == '\n') {
			*state = LW_PARSE_STATE1_SYNC;
			(*parserbuf_index) = 0;
		}

		break;

	case LW_PARSE_STATE1_SYNC:
		if (c >= '0' && c <= '9') {
			*state = LW_PARSE_STATE2_GOT_DIGIT0;
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;
		}

		break;

	case LW_PARSE_STATE2_GOT_DIGIT0:
		if (c >= '0' && c <= '9') {
			*state = LW_PARSE_STATE2_GOT_DIGIT0;
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;

		} else if (c == '.') {
			*state = LW_PARSE_STATE3_GOT_DOT;
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;

		} else {
			*state = LW_PARSE_STATE0_UNSYNC;
		}

		break;

	case LW_PARSE_STATE3_GOT_DOT:
		if (c >= '0' && c <= '9') {
			*state = LW_PARSE_STATE4_GOT_DIGIT1;
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;

		} else {
			*state = LW_PARSE_STATE0_UNSYNC;
		}

		break;

	case LW_PARSE_STATE4_GOT_DIGIT1:
		if (c >= '0' && c <= '9') {
			*state = LW_PARSE_STATE5_GOT_DIGIT2;
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;

		} else {
			*state = LW_PARSE_STATE0_UNSYNC;
		}

		break;

	case LW_PARSE_STATE5_GOT_DIGIT2:
		if (c == '\r') {
			*state = LW_PARSE_STATE6_GOT_CARRIAGE_RETURN;

		} else {
			*state = LW_PARSE_STATE0_UNSYNC;
		}

		break;

	case LW_PARSE_STATE6_GOT_CARRIAGE_RETURN:
		if (c == '\n') {
			parserbuf[*parserbuf_index] = '\0';
			*dist = strtod(parserbuf, &end);
			*state = LW_PARSE_STATE1_SYNC;
			*parserbuf_index = 0;
			ret = 0;

		} else {
			*state = LW_PARSE_STATE0_UNSYNC;
		}

		break;
	}

#ifdef LW_DEBUG
	printf("state: LW_PARSE_STATE%s\n", parser_state[*state]);
#endif

	return ret;
}
