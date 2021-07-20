/****************************************************************************
 *
 *   Copyright (C) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file mixer_group.cpp
 *
 * Mixer collection.
 */

#include "MixerGroup.hpp"

#include "AllocatedActuatorMixer/AllocatedActuatorMixer.hpp"
#include "HelicopterMixer/HelicopterMixer.hpp"
#include "MultirotorMixer/MultirotorMixer.hpp"
#include "NullMixer/NullMixer.hpp"
#include "SimpleMixer/SimpleMixer.hpp"
#include <px4_platform_common/log.h>

#ifndef MODULE_NAME
#define MODULE_NAME "mixer"
#endif

unsigned
MixerGroup::mix(float *outputs, unsigned space)
{
	unsigned index = 0;

	for (auto mixer : _mixers) {
		index += mixer->mix(outputs + index, space - index);

		if (index >= space) {
			break;
		}
	}

	return index;
}

/*
 * set_trims() has no effect except for the SimpleMixer implementation for which set_trim()
 * always returns the value one.
 * The only other existing implementation is MultirotorMixer, which ignores the trim value
 * and returns _rotor_count.
 */
unsigned
MixerGroup::set_trims(int16_t *values, unsigned n)
{
	unsigned index = 0;

	for (auto mixer : _mixers) {
		// convert from integer to float
		// to be safe, clamp offset to range of [-500, 500] usec
		float offset = math::constrain((float)values[index] / 10000, -1.0f, 1.0f);

		PX4_DEBUG("set trim: %d, offset: %5.3f", values[index], (double)offset);
		index += mixer->set_trim(offset);

		if (index >= n) {
			break;
		}
	}

	return index;
}

/*
 * get_trims() has no effect except for the SimpleMixer implementation for which get_trim()
 * always returns the value one and sets the trim value.
 * The only other existing implementation is MultirotorMixer, which ignores the trim value
 * and returns _rotor_count.
 */
unsigned
MixerGroup::get_trims(int16_t *values)
{
	unsigned index_mixer = 0;
	unsigned index = 0;

	for (auto mixer : _mixers) {
		float trim = 0;
		index_mixer += mixer->get_trim(&trim);

		// MultirotorMixer returns the number of motors so we
		// loop through index_mixer and set the same trim value for all motors
		while (index < index_mixer) {
			values[index] = trim * 10000;
			index++;
		}
	}

	return index;
}

void
MixerGroup::set_thrust_factor(float val)
{
	for (auto mixer : _mixers) {
		mixer->set_thrust_factor(val);
	}
}

void
MixerGroup::set_airmode(Mixer::Airmode airmode)
{
	for (auto mixer : _mixers) {
		mixer->set_airmode(airmode);
	}
}

unsigned
MixerGroup::get_multirotor_count()
{
	for (auto mixer : _mixers) {
		unsigned rotor_count = mixer->get_multirotor_count();

		if (rotor_count > 0) {
			return rotor_count;
		}
	}

	return 0;
}

uint16_t
MixerGroup::get_saturation_status()
{
	uint16_t sat = 0;

	for (auto mixer : _mixers) {
		sat |= mixer->get_saturation_status();
	}

	return sat;
}

void
MixerGroup::groups_required(uint32_t &groups)
{
	for (auto mixer : _mixers) {
		mixer->groups_required(groups);
	}
}

int
MixerGroup::load_from_buf(Mixer::ControlCallback control_cb, uintptr_t cb_handle, const char *buf, unsigned &buflen)
{
	int ret = -1;
	const char *end = buf + buflen;

	/*
	 * Loop until either we have emptied the buffer, or we have failed to
	 * allocate something when we expected to.
	 */
	while (buflen > 0) {
		Mixer *m = nullptr;
		const char *p = end - buflen;
		unsigned resid = buflen;

		/*
		 * Use the next character as a hint to decide which mixer class to construct.
		 */
		switch (*p) {
		case 'Z':
			m = NullMixer::from_text(p, resid);
			break;

		case 'A':
			m = AllocatedActuatorMixer::from_text(control_cb, cb_handle, p, resid);
			break;

		case 'M':
			m = SimpleMixer::from_text(control_cb, cb_handle, p, resid);
			break;

		case 'R':
			m = MultirotorMixer::from_text(control_cb, cb_handle, p, resid);
			break;

		case 'H':
			m = HelicopterMixer::from_text(control_cb, cb_handle, p, resid);
			break;

		default:
			/* it's probably junk or whitespace, skip a byte and retry */
			buflen--;
			continue;
		}

		/*
		 * If we constructed something, add it to the group.
		 */
		if (m != nullptr) {
			add_mixer(m);

			/* we constructed something */
			ret = 0;

			/* only adjust buflen if parsing was successful */
			buflen = resid;
			PX4_DEBUG("SUCCESS - buflen: %d", buflen);

		} else {

			/*
			 * There is data in the buffer that we expected to parse, but it didn't,
			 * so give up for now.
			 */
			break;
		}
	}

	/* nothing more in the buffer for us now */
	return ret;
}

void MixerGroup::set_max_delta_out_once(float delta_out_max)
{
	for (auto mixer : _mixers) {
		mixer->set_max_delta_out_once(delta_out_max);
	}
}

void
MixerGroup::set_dt_once(float dt)
{
	for (auto mixer : _mixers) {
		mixer->set_dt_once(dt);
	}
}
