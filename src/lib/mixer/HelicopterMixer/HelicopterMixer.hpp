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

#pragma once

#include <mixer/MixerBase/Mixer.hpp>

/** helicopter swash servo mixer */
struct mixer_heli_servo_s {
	float angle;
	float arm_length;
	float scale;
	float offset;
	float min_output;
	float max_output;
};

#define HELI_CURVES_NR_POINTS 5

/** helicopter swash plate mixer */
struct mixer_heli_s {
	uint8_t				control_count;	/**< number of inputs */
	float				throttle_curve[HELI_CURVES_NR_POINTS];
	float				pitch_curve[HELI_CURVES_NR_POINTS];
	mixer_heli_servo_s		servos[4];	/**< up to four inputs */
};

/**
 * Generic helicopter mixer for helicopters with swash plate.
 *
 * Collects four inputs (roll, pitch, yaw, thrust) and mixes them to servo commands
 * for swash plate tilting and throttle- and pitch curves.
 */
class HelicopterMixer : public Mixer
{
public:
	/**
	 * Constructor.
	 *
	 * @param control_cb		Callback invoked to read inputs.
	 * @param cb_handle		Passed to control_cb.
	 * @param mixer_info		Pointer to heli mixer configuration
	 */
	HelicopterMixer(ControlCallback control_cb, uintptr_t cb_handle, mixer_heli_s mixer_info);
	virtual ~HelicopterMixer() = default;

	// no copy, assignment, move, move assignment
	HelicopterMixer(const HelicopterMixer &) = delete;
	HelicopterMixer &operator=(const HelicopterMixer &) = delete;
	HelicopterMixer(HelicopterMixer &&) = delete;
	HelicopterMixer &operator=(HelicopterMixer &&) = delete;

	/**
	 * Factory method.
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
	 * @return			A new HelicopterMixer instance, or nullptr
	 *				if the text format is bad.
	 */
	static HelicopterMixer		*from_text(Mixer::ControlCallback control_cb, uintptr_t cb_handle, const char *buf,
			unsigned &buflen);

	unsigned			mix(float *outputs, unsigned space) override;

	void				groups_required(uint32_t &groups) override { groups |= (1 << 0); }

	unsigned			set_trim(float trim) override { return 4; }
	unsigned			get_trim(float *trim) override { return 4; }

private:
	mixer_heli_s			_mixer_info;
};
