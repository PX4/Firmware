/****************************************************************************
 *
 *   Copyright (c) 2015-2020 PX4 Development Team. All rights reserved.
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
 * @file Integrator.hpp
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
	Integrator(bool coning_compensation = false) : _coning_comp_on(coning_compensation) {}
	~Integrator() = default;

	/**
	 * Put an item into the integral.
	 *
	 * @param timestamp	Timestamp of the current value.
	 * @param val		Item to put.
	 * @return		true if data was accepted and integrated.
	 */
	bool put(const uint64_t &timestamp, const matrix::Vector3f &val);

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
	bool put(const uint64_t &timestamp, const matrix::Vector3f &val, matrix::Vector3f &integral, uint32_t &integral_dt)
	{
		return put(timestamp, val) && reset(integral, integral_dt);
	}

	/**
	 * Set reset interval during runtime. This won't reset the integrator.
	 *
	 * @param reset_interval	    	New reset time interval for the integrator.
	 */
	void set_reset_interval(uint32_t reset_interval) { _reset_interval_min = reset_interval; }

	/**
	 * Set required samples for reset. This won't reset the integrator.
	 *
	 * @param reset_samples	    	New reset time interval for the integrator.
	 */
	void set_reset_samples(uint8_t reset_samples) { _reset_samples_min = reset_samples; }
	uint8_t get_reset_samples() const { return _reset_samples_min; }

	/**
	 * Is the Integrator ready to reset?
	 *
	 * @return		true if integrator has sufficient data (minimum interval & samples satisfied) to reset.
	 */
	bool integral_ready() const { return (_integrated_samples >= _reset_samples_min) || (_last_integration_time >= (_last_reset_time + _reset_interval_min)); }

	/* Reset integrator and return current integral & integration time
	 *
	 * @param integral_dt	Get the dt in us of the current integration.
	 * @return		true if integral valid
	 */
	bool reset(matrix::Vector3f &integral, uint32_t &integral_dt);

private:
	uint64_t _last_integration_time{0}; /**< timestamp of the last integration step */
	uint64_t _last_reset_time{0};       /**< last auto-announcement of integral value */

	matrix::Vector3f _alpha{0.f, 0.f, 0.f};            /**< integrated value before coning corrections are applied */
	matrix::Vector3f _last_alpha{0.f, 0.f, 0.f};       /**< previous value of _alpha */
	matrix::Vector3f _beta{0.f, 0.f, 0.f};             /**< accumulated coning corrections */
	matrix::Vector3f _last_val{0.f, 0.f, 0.f};         /**< previous input */
	matrix::Vector3f _last_delta_alpha{0.f, 0.f, 0.f}; /**< integral from previous previous sampling interval */

	uint32_t _reset_interval_min{1}; /**< the interval after which the content will be published and the integrator reset */

	uint8_t _integrated_samples{0};
	uint8_t _reset_samples_min{1};

	const bool _coning_comp_on{false};                       /**< true to turn on coning corrections */
};
