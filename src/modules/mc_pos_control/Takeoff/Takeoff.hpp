/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file Takeoff.hpp
 *
 * A class handling all takeoff states and a smooth ramp up of the motors.
 */

#pragma once

#include <lib/hysteresis/hysteresis.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/takeoff_status.h>

using namespace time_literals;

enum class TakeoffState {
	disarmed = takeoff_status_s::TAKEOFF_STATE_DISARMED,
	spoolup = takeoff_status_s::TAKEOFF_STATE_SPOOLUP,
	ready_for_takeoff = takeoff_status_s::TAKEOFF_STATE_READY_FOR_TAKEOFF,
	rampup = takeoff_status_s::TAKEOFF_STATE_RAMPUP,
	flight = takeoff_status_s::TAKEOFF_STATE_FLIGHT
};

class Takeoff
{
public:
	Takeoff() = default;
	~Takeoff() = default;

	// initialize parameters
	void setSpoolupTime(const float seconds) { _spoolup_time_hysteresis.set_hysteresis_time_from(false, seconds * 1_s); }
	void setTakeoffRampTime(const float seconds) { _takeoff_ramp_time = seconds; }

	/**
	 * Calculate a vertical velocity to initialize the takeoff ramp
	 * that when passed to the velocity controller results in a zero throttle setpoint.
	 * @param hover_thrust normalized thrsut value with which the vehicle hovers
	 * @param velocity_p_gain proportional gain of the velocity controller to calculate the thrust
	 */
	void generateInitialRampValue(const float velocity_p_gain);

	/**
	 * Update the state for the takeoff.
	 * @param setpoint a vehicle_local_position_setpoint_s structure
	 * @return true if setpoint has updated correctly
	 */
	void updateTakeoffState(const bool armed, const bool landed, const bool want_takeoff,
				const float takeoff_desired_vz, const bool skip_takeoff, const hrt_abstime &now_us);

	/**
	 * Update and return the velocity constraint ramp value during takeoff.
	 * By ramping up _takeoff_ramp_vz during the takeoff and using it to constain the maximum climb rate a smooth takeoff behavior is achieved.
	 * Returns zero on the ground and takeoff_desired_vz in flight.
	 * @param dt time in seconds since the last call/loop iteration
	 * @param takeoff_desired_vz end value for the velocity ramp
	 * @return true if setpoint has updated correctly
	 */
	float updateRamp(const float dt, const float takeoff_desired_vz);

	TakeoffState getTakeoffState() { return _takeoff_state; }

private:
	TakeoffState _takeoff_state = TakeoffState::disarmed;

	systemlib::Hysteresis _spoolup_time_hysteresis{false}; ///< becomes true MPC_SPOOLUP_TIME seconds after the vehicle was armed

	float _takeoff_ramp_time{0.f};
	float _takeoff_ramp_vz_init{0.f}; ///< verticval velocity resulting in zero thrust
	float _takeoff_ramp_progress{0.f}; ///< asecnding from 0 to 1
};
