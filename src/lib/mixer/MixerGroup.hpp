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

#include "MixerBase/Mixer.hpp"

/**
 * Group of mixers, built up from single mixers and processed
 * in order when mixing.
 */
class MixerGroup
{
public:
	MixerGroup() = default;

	~MixerGroup()
	{
		reset();
	}

	// no copy, assignment, move, move assignment
	MixerGroup(const MixerGroup &) = delete;
	MixerGroup &operator=(const MixerGroup &) = delete;
	MixerGroup(MixerGroup &&) = delete;
	MixerGroup &operator=(MixerGroup &&) = delete;

	unsigned			mix(float *outputs, unsigned space);

	uint16_t			get_saturation_status();

	void				groups_required(uint32_t &groups);

	/**
	 * Add a mixer to the group.
	 *
	 * @param mixer			The mixer to be added.
	 */
	void				add_mixer(Mixer *mixer) { _mixers.add(mixer); }

	/**
	 * Remove all the mixers from the group.
	 */
	void				reset() { _mixers.clear(); }

	/**
	 * Count the mixers in the group.
	 */
	unsigned			count() const { return _mixers.size(); }

	/**
	 * Adds mixers to the group based on a text description in a buffer.
	 *
	 * Mixer definitions begin with a single capital letter and a colon.
	 * The actual format of the mixer definition varies with the individual
	 * mixers; they are summarised here, but see ROMFS/mixers/README for
	 * more details.
	 *
	 * Null Mixer
	 * ..........
	 *
	 * The null mixer definition has the form:
	 *
	 *   Z:
	 *
	 * Simple Mixer
	 * ............
	 *
	 * A simple mixer definition begins with:
	 *
	 *   M: <control count>
	 *   O: <-ve scale> <+ve scale> <offset> <lower limit> <upper limit>
	 *
	 * The second line O: can be omitted. In that case 'O: 10000 10000 0 -10000 10000' is used.
	 * The definition continues with <control count> entries describing the control
	 * inputs and their scaling, in the form:
	 *
	 *   S: <group> <index> <-ve scale> <+ve scale> <offset> <lower limit> <upper limit>
	 *
	 * Multirotor Mixer
	 * ................
	 *
	 * The multirotor mixer definition is a single line of the form:
	 *
	 * R: <geometry> <roll scale> <pitch scale> <yaw scale> <deadband>
	 *
	 * Helicopter Mixer
	 * ................
	 *
	 * The helicopter mixer includes throttle and pitch curves
	 *
	 * H: <swash plate servo count>
	 * T: <0> <2500> <5000> <7500> <10000>
	 * P: <-10000> <-5000> <0> <5000> <10000>
	 *
	 * The definition continues with <swash plate servo count> entries describing
	 * the position of the servo, in the following form:
	 *
	 *   S: <angle (deg)> <normalized arm length> <scale> <offset> <lower limit> <upper limit>
	 *
	 * @param buf			The mixer configuration buffer.
	 * @param buflen		The length of the buffer, updated to reflect
	 *				bytes as they are consumed.
	 * @return			Zero on successful load, nonzero otherwise.
	 */
	int				load_from_buf(Mixer::ControlCallback control_cb, uintptr_t cb_handle, const char *buf, unsigned &buflen);

	/**
	 * @brief      Update slew rate parameter. This tells instances of the class MultirotorMixer
	 *             the maximum allowed change of the output values per cycle.
	 *             The value is only valid for one cycle, in order to have continuous
	 *             slew rate limiting this function needs to be called before every call
	 *             to mix().
	 *
	 * @param[in]  delta_out_max  Maximum delta output.
	 *
	 */
	void 				set_max_delta_out_once(float delta_out_max);

	/*
	 * Invoke the set_offset method of each mixer in the group
	 * for each value in page r_page_servo_control_trim
	 */
	unsigned			set_trims(int16_t *v, unsigned n);
	unsigned			get_trims(int16_t *values);

	/**
	 * @brief      Sets the thrust factor used to calculate mapping from desired thrust to motor control signal output.
	 *
	 * @param[in]  val   The value
	 */
	void				set_thrust_factor(float val);

	void				set_airmode(Mixer::Airmode airmode);

	unsigned			get_multirotor_count();

	void 				set_dt_once(float dt);

private:
	List<Mixer *>			_mixers;	/**< linked list of mixers */
};
