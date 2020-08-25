/****************************************************************************
 *
 *   Copyright (c) 2015-2018 PX4 Development Team. All rights reserved.
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
 * @file integrator.h
 *
 * A resettable integrator
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Julian Oes <julian@oes.ch>
 */

#pragma once

#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

class Integrator
{
public:
	Integrator(uint32_t auto_reset_interval = 4000 /* 250 Hz */, bool coning_compensation = false);
	~Integrator() = default;

	/**
	 * Put an item into the integral.
	 *
	 * @param timestamp	Timestamp of the current value.
	 * @param val		Item to put.
	 * @param integral	Current integral in case the integrator did reset, else the value will not be modified
	 * @param integral_dt	Get the dt in us of the current integration (only if reset).
	 * @return		true if putting the item triggered an integral reset and the integral should be
	 *			published.
	 */
	bool put(const uint64_t &timestamp, const matrix::Vector3f &val, matrix::Vector3f &integral, uint32_t &integral_dt);

	/**
	 * Set auto reset interval during runtime. This won't reset the integrator.
	 *
	 * @param auto_reset_interval	    	New reset time interval for the integrator (+- 10%).
	 */
	void set_autoreset_interval(uint32_t auto_reset_interval) { _auto_reset_interval = 0.90f * auto_reset_interval; }

private:
	uint32_t _auto_reset_interval{0};			/**< the interval after which the content will be published
							     and the integrator reset, 0 if no auto-reset */

	uint64_t _last_integration_time{0};		/**< timestamp of the last integration step */
	uint64_t _last_reset_time{0};			/**< last auto-announcement of integral value */

	matrix::Vector3f _alpha{0.0f, 0.0f, 0.0f};			/**< integrated value before coning corrections are applied */
	matrix::Vector3f _last_alpha{0.0f, 0.0f, 0.0f};			/**< previous value of _alpha */
	matrix::Vector3f _beta{0.0f, 0.0f, 0.0f};				/**< accumulated coning corrections */
	matrix::Vector3f _last_val{0.0f, 0.0f, 0.0f};	/**< previous input */
	matrix::Vector3f _last_delta_alpha{0.0f, 0.0f, 0.0f};		/**< integral from previous previous sampling interval */

	bool _coning_comp_on{false};				/**< true to turn on coning corrections */

	/* Do a reset.
	 *
	 * @param integral_dt	Get the dt in us of the current integration.
	 */
	void _reset(uint32_t &integral_dt);
};
