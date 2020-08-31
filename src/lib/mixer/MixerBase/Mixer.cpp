/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file mixer.cpp
 *
 * Programmable multi-channel mixer library.
 */

#include "Mixer.hpp"

#include <math.h>
#include <cstring>
#include <ctype.h>

#define debug(fmt, args...)	do { } while(0)
//#define debug(fmt, args...)	do { printf("[mixer] " fmt "\n", ##args); } while(0)

float
Mixer::get_control(uint8_t group, uint8_t index)
{
	float value;

	_control_cb(_cb_handle, group, index, value);

	return value;
}

const char *
Mixer::findtag(const char *buf, unsigned &buflen, char tag)
{
	while (buflen >= 2) {
		if ((buf[0] == tag) && (buf[1] == ':')) {
			return buf;
		}

		buf++;
		buflen--;
	}

	return nullptr;
}

char
Mixer::findnexttag(const char *buf, unsigned buflen)
{
	while (buflen >= 2) {
		if (isupper(buf[0]) && buf[1] == ':') {
			return buf[0];
		}

		buf++;
		buflen--;
	}

	return 0;
}

const char *
Mixer::skipline(const char *buf, unsigned &buflen)
{
	const char *p;

	/* if we can find a CR or NL in the buffer, skip up to it */
	if ((p = (const char *)memchr(buf, '\r', buflen)) || (p = (const char *)memchr(buf, '\n', buflen))) {
		/* skip up to it AND one beyond - could be on the NUL symbol now */
		buflen -= (p - buf) + 1;
		return p + 1;
	}

	return nullptr;
}

bool
Mixer::string_well_formed(const char *buf, unsigned &buflen)
{
	/* enforce that the mixer ends with a new line */
	for (int i = buflen - 1; i >= 0; i--) {
		if (buf[i] == '\0') {
			continue;
		}

		/* require a space or newline at the end of the buffer, fail on printable chars */
		if (buf[i] == '\n' || buf[i] == '\r') {
			/* found a line ending, so no split symbols / numbers. good. */
			return true;
		}

	}

	debug("pre-parser rejected: No newline in buf");

	return false;
}
