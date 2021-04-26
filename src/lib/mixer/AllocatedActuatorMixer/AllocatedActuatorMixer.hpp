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
 * @file mixer_AllocatedActuatorMixer.hpp
 *
 * Mixer for allocated actuators.
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#pragma once

#include <lib/mixer/MixerBase/Mixer.hpp>

/**
 * Mixer for allocated actuators.
 *
 * Copies a single actuator to a single output.
 */
class AllocatedActuatorMixer : public Mixer
{
public:
	/**
	 * Constructor
	 *
	 * @param index	Actuator index (0..15)
	 */
	AllocatedActuatorMixer(ControlCallback control_cb,
			       uintptr_t cb_handle,
			       uint8_t index);
	virtual ~AllocatedActuatorMixer() = default;

	// no copy, assignment, move, move assignment
	AllocatedActuatorMixer(const AllocatedActuatorMixer &) = delete;
	AllocatedActuatorMixer &operator=(const AllocatedActuatorMixer &) = delete;
	AllocatedActuatorMixer(AllocatedActuatorMixer &&) = delete;
	AllocatedActuatorMixer &operator=(AllocatedActuatorMixer &&) = delete;

	/**
	 * Factory method with full external configuration.
	 *
	 * Given a pointer to a buffer containing a text description of the mixer,
	 * returns a pointer to a new instance of the mixer.
	 *
	 * @param control_cb		The callback to invoke when fetching a
	 *				control value.
	 * @param cb_handle		Handle passed to the control callback.
	 * @param buf			Buffer containing a text description of
	 *				the mixer.
	 * @param buflen		Length of the buffer in bytes, adjusted
	 *				to reflect the bytes consumed.
	 * @return			A new AllocatedActuatorMixer instance, or nullptr
	 *				if the text format is bad.
	 */
	static AllocatedActuatorMixer *from_text(Mixer::ControlCallback control_cb,
			uintptr_t cb_handle,
			const char *buf,
			unsigned &buflen);

	unsigned mix(float *outputs, unsigned space) override;
	void groups_required(uint32_t &groups) override;
	unsigned set_trim(float trim) override;
	unsigned get_trim(float *trim) override;

private:
	uint8_t	_control_group;	/**< group from which the input reads */
	uint8_t	_control_index;	/**< index within the control group */

	static int parse(const char *buf,
			 unsigned &buflen,
			 uint8_t &index);
};
