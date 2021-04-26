/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file mixer_AllocatedActuatorMixer.cpp
 *
 * Mixer for allocated actuators.
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include "AllocatedActuatorMixer.hpp"

#include <mathlib/mathlib.h>
#include <cstdio>
#include <px4_platform_common/defines.h>

// #define debug(fmt, args...)	do { } while(0)
#define debug(fmt, args...)	do { printf("[mixer] " fmt "\n", ##args); } while(0)

AllocatedActuatorMixer::AllocatedActuatorMixer(ControlCallback control_cb,
		uintptr_t cb_handle,
		uint8_t index) :
	Mixer(control_cb, cb_handle)
{
	if (index < 8) {
		_control_group = 4;
		_control_index = index;

	} else if (index < 16) {
		_control_group = 5;
		_control_index = index - 8;

	} else {
		debug("'A:' invalid index");
	}
}

unsigned AllocatedActuatorMixer::set_trim(float trim)
{
	return 1;
}

unsigned AllocatedActuatorMixer::get_trim(float *trim)
{
	*trim = 0.0f;
	return 1;
}

int
AllocatedActuatorMixer::parse(const char *buf, unsigned &buflen, uint8_t &index)
{
	int ret;
	int i;

	// enforce that the mixer ends with a new line
	if (!string_well_formed(buf, buflen)) {
		return -1;
	}

	// parse line
	if ((ret = sscanf(buf, "A: %d", &i)) != 1) {
		debug("'A:' parser: failed on '%s'", buf);
		return -1;
	}

	buf = skipline(buf, buflen);

	if (buf == nullptr) {
		debug("'A:' parser: no line ending, line is incomplete");
		return -1;
	}

	// check parsed index
	if (i < 16) {
		index = i;

	} else {
		debug("'A:' parser: invalid index");
		return -1;
	}

	return 0;
}

AllocatedActuatorMixer *
AllocatedActuatorMixer::from_text(Mixer::ControlCallback control_cb, uintptr_t cb_handle, const char *buf,
				  unsigned &buflen)
{
	uint8_t index;

	if (parse(buf, buflen, index) == 0) {
		return new AllocatedActuatorMixer(control_cb, cb_handle, index);

	} else {
		return nullptr;
	}
}

unsigned
AllocatedActuatorMixer::mix(float *outputs, unsigned space)
{
	if (space < 1) {
		return 0;
	}

	_control_cb(_cb_handle,
		    _control_group,
		    _control_index,
		    *outputs);

	return 1;
}

void
AllocatedActuatorMixer::groups_required(uint32_t &groups)
{
	groups |= 1 << _control_group;
}
