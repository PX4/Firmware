/****************************************************************************
 *
 *   Copyright (c) 2017-2020 PX4 Development Team. All rights reserved.
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

#include "TECS.hpp"

#include <lib/ecl/geo/geo.h>

#include <px4_platform_common/defines.h>

using math::constrain;
using math::max;
using math::min;

static constexpr float DT_MIN = 0.001f;	///< minimum allowed value of _dt (sec)
static constexpr float DT_MAX = 1.0f;	///< max value of _dt allowed before a filter state reset is performed (sec)

/**
 * @file TECS.cpp
 *
 * @author Paul Riseborough
 */

/*
 * This function implements a complementary filter to estimate the climb rate when
 * inertial nav data is not available. It also calculates a true airspeed derivative
 * which is used by the airspeed complimentary filter.
 */
void TECS::update_vehicle_state_estimates(float airspeed, const matrix::Dcmf &rotMat,
		const matrix::Vector3f &accel_body, bool altitude_lock, bool in_air,
		float altitude, float vz)
{
	// calculate the time lapsed since the last update
	uint64_t now = hrt_absolute_time();
	float dt = constrain((now - _state_update_timestamp) * 1.0e-6f, DT_MIN, DT_MAX);

	bool reset_altitude = false;

	if (_state_update_timestamp == 0 || dt > DT_MAX) {
		dt = DT_DEFAULT;
		reset_altitude = true;
	}

	if (!altitude_lock || !in_air) {
		reset_altitude = true;
	}

	if (reset_altitude) {
		_states_initialized = false;
	}

	_state_update_timestamp = now;
	_EAS = airspeed;

	_in_air = in_air;

	// Set the velocity and position state to the the INS data
	_vert_vel_state = -vz;
	_vert_pos_state = altitude;

	// Update and average speed rate of change if airspeed is being measured
	if (PX4_ISFINITE(airspeed) && airspeed_sensor_enabled()) {
		// Assuming the vehicle is flying X axis forward, use the X axis measured acceleration
		// compensated for gravity to estimate the rate of change of speed
		float speed_deriv_raw = rotMat(2, 0) * CONSTANTS_ONE_G + accel_body(0);

		// Apply some noise filtering
		_speed_derivative = 0.95f * _speed_derivative + 0.05f * speed_deriv_raw;

	} else {
		_speed_derivative = 0.0f;
	}

	if (!_in_air) {
		_states_initialized = false;
	}

}

void TECS::_update_speed_states(float airspeed_setpoint, float indicated_airspeed, float EAS2TAS)
{
	// Calculate the time in seconds since the last update and use the default time step value if out of bounds
	uint64_t now = hrt_absolute_time();
	const float dt = constrain((now - _speed_update_timestamp) * 1.0e-6f, DT_MIN, DT_MAX);

	// Convert equivalent airspeed quantities to true airspeed
	_EAS_setpoint = airspeed_setpoint;
	_TAS_setpoint  = _EAS_setpoint * EAS2TAS;
	_TAS_max   = _indicated_airspeed_max * EAS2TAS;
	_TAS_min   = _indicated_airspeed_min * EAS2TAS;

	// If airspeed measurements are not being used, fix the airspeed estimate to halfway between
	// min and max limits
	if (!PX4_ISFINITE(indicated_airspeed) || !airspeed_sensor_enabled()) {
		_EAS = 0.5f * (_indicated_airspeed_min + _indicated_airspeed_max);

	} else {
		_EAS = indicated_airspeed;
	}

	// If first time through or not flying, reset airspeed states
	if (_speed_update_timestamp == 0 || !_in_air) {
		_tas_rate_state = 0.0f;
		_tas_state = (_EAS * EAS2TAS);
	}

	// Obtain a smoothed airspeed estimate using a second order complementary filter

	// Update TAS rate state
	float tas_error = (_EAS * EAS2TAS) - _tas_state;
	float tas_rate_state_input = tas_error * _tas_estimate_freq * _tas_estimate_freq;

	// limit integrator input to prevent windup
	if (_tas_state < 3.1f) {
		tas_rate_state_input = max(tas_rate_state_input, 0.0f);

	}

	// Update TAS state
	_tas_rate_state = _tas_rate_state + tas_rate_state_input * dt;
	float tas_state_input = _tas_rate_state + _speed_derivative + tas_error * _tas_estimate_freq * 1.4142f;
	_tas_state = _tas_state + tas_state_input * dt;

	// Limit the airspeed state to a minimum of 3 m/s
	_tas_state = max(_tas_state, 3.0f);
	_speed_update_timestamp = now;

}

void TECS::_update_speed_setpoint()
{
	// Set the airspeed demand to the minimum value if an underspeed or
	// or a uncontrolled descent condition exists to maximise climb rate
	if ((_uncommanded_descent_recovery) || (_underspeed_detected)) {
		_TAS_setpoint = _TAS_min;
	}

	_TAS_setpoint = constrain(_TAS_setpoint, _TAS_min, _TAS_max);

	// Calculate limits for the demanded rate of change of speed based on physical performance limits
	// with a 50% margin to allow the total energy controller to correct for errors.
	float velRateMax = 0.5f * _STE_rate_max / _tas_state;
	float velRateMin = 0.5f * _STE_rate_min / _tas_state;

	_TAS_setpoint_adj = constrain(_TAS_setpoint, _TAS_min, _TAS_max);

	// calculate the demanded rate of change of speed proportional to speed error
	// and apply performance limits
	_TAS_rate_setpoint = constrain((_TAS_setpoint_adj - _tas_state) * _speed_error_gain, velRateMin, velRateMax);

}

void TECS::_update_height_setpoint(float desired, float state)
{
	// Detect first time through and initialize previous value to demand
	if (PX4_ISFINITE(desired) && fabsf(_hgt_setpoint_in_prev) < 0.1f) {
		_hgt_setpoint_in_prev = desired;
	}

	// Apply a 2 point moving average to demanded height to reduce
	// intersampling noise effects.
	if (PX4_ISFINITE(desired)) {
		_hgt_setpoint = 0.5f * (desired + _hgt_setpoint_in_prev);

	} else {
		_hgt_setpoint = _hgt_setpoint_in_prev;
	}

	_hgt_setpoint_in_prev = _hgt_setpoint;

	// Apply a rate limit to respect vehicle performance limitations
	if ((_hgt_setpoint - _hgt_setpoint_prev) > (_max_climb_rate * _dt)) {
		_hgt_setpoint = _hgt_setpoint_prev + _max_climb_rate * _dt;

	} else if ((_hgt_setpoint - _hgt_setpoint_prev) < (-_max_sink_rate * _dt)) {
		_hgt_setpoint = _hgt_setpoint_prev - _max_sink_rate * _dt;
	}

	_hgt_setpoint_prev = _hgt_setpoint;

	// Apply a first order noise filter
	_hgt_setpoint_adj = 0.1f * _hgt_setpoint + 0.9f * _hgt_setpoint_adj_prev;

	// Calculate the demanded climb rate proportional to height error plus a feedforward term to provide
	// tight tracking during steady climb and descent manoeuvres.
	_hgt_rate_setpoint = (_hgt_setpoint_adj - state) * _height_error_gain + _height_setpoint_gain_ff *
			     (_hgt_setpoint_adj - _hgt_setpoint_adj_prev) / _dt;
	_hgt_setpoint_adj_prev = _hgt_setpoint_adj;

	// Limit the rate of change of height demand to respect vehicle performance limits
	if (_hgt_rate_setpoint > _max_climb_rate) {
		_hgt_rate_setpoint = _max_climb_rate;

	} else if (_hgt_rate_setpoint < -_max_sink_rate) {
		_hgt_rate_setpoint = -_max_sink_rate;
	}
}

void TECS::_detect_underspeed()
{
	if (!_detect_underspeed_enabled) {
		_underspeed_detected = false;
		return;
	}

	if (((_tas_state < _TAS_min * 0.9f) && (_throttle_setpoint >= _throttle_setpoint_max * 0.95f))
	    || ((_vert_pos_state < _hgt_setpoint_adj) && _underspeed_detected)) {

		_underspeed_detected = true;

	} else {
		_underspeed_detected = false;
	}
}

void TECS::_update_energy_estimates()
{
	// Calculate specific energy demands in units of (m**2/sec**2)
	_SPE_setpoint = _hgt_setpoint_adj * CONSTANTS_ONE_G; // potential energy
	_SKE_setpoint = 0.5f * _TAS_setpoint_adj * _TAS_setpoint_adj; // kinetic energy

	// Calculate specific energy rate demands in units of (m**2/sec**3)
	_SPE_rate_setpoint = _hgt_rate_setpoint * CONSTANTS_ONE_G; // potential energy rate of change
	_SKE_rate_setpoint = _tas_state * _TAS_rate_setpoint; // kinetic energy rate of change

	// Calculate specific energies in units of (m**2/sec**2)
	_SPE_estimate = _vert_pos_state * CONSTANTS_ONE_G; // potential energy
	_SKE_estimate = 0.5f * _tas_state * _tas_state; // kinetic energy

	// Calculate specific energy rates in units of (m**2/sec**3)
	_SPE_rate = _vert_vel_state * CONSTANTS_ONE_G; // potential energy rate of change
	_SKE_rate = _tas_state * _speed_derivative;// kinetic energy rate of change
}

void TECS::_update_throttle_setpoint(const float throttle_cruise, const matrix::Dcmf &rotMat)
{
	// Calculate total energy error
	_STE_error = _SPE_setpoint - _SPE_estimate + _SKE_setpoint - _SKE_estimate;

	// Calculate demanded rate of change of total energy, respecting vehicle limits
	float STE_rate_setpoint = constrain((_SPE_rate_setpoint + _SKE_rate_setpoint), _STE_rate_min, _STE_rate_max);

	// Calculate the total energy rate error, applying a first order IIR filter
	// to reduce the effect of accelerometer noise
	_STE_rate_error = 0.2f * (STE_rate_setpoint - _SPE_rate - _SKE_rate) + 0.8f * _STE_rate_error;

	// Calculate the throttle demand
	if (_underspeed_detected) {
		// always use full throttle to recover from an underspeed condition
		_throttle_setpoint = 1.0f;

	} else {
		// Adjust the demanded total energy rate to compensate for induced drag rise in turns.
		// Assume induced drag scales linearly with normal load factor.
		// The additional normal load factor is given by (1/cos(bank angle) - 1)
		float cosPhi = sqrtf((rotMat(0, 1) * rotMat(0, 1)) + (rotMat(1, 1) * rotMat(1, 1)));
		STE_rate_setpoint = STE_rate_setpoint + _load_factor_correction * (1.0f / constrain(cosPhi, 0.1f, 1.0f) - 1.0f);

		// Calculate a predicted throttle from the demanded rate of change of energy, using the cruise throttle
		// as the starting point. Assume:
		// Specific total energy rate = _STE_rate_max is achieved when throttle is set to _throttle_setpoint_max
		// Specific total energy rate = 0 at cruise throttle
		// Specific total energy rate = _STE_rate_min is achieved when throttle is set to _throttle_setpoint_min
		float throttle_predicted = 0.0f;

		if (STE_rate_setpoint >= 0) {
			// throttle is between cruise and maximum
			throttle_predicted = throttle_cruise + STE_rate_setpoint / _STE_rate_max * (_throttle_setpoint_max - throttle_cruise);

		} else {
			// throttle is between cruise and minimum
			throttle_predicted = throttle_cruise + STE_rate_setpoint / _STE_rate_min * (_throttle_setpoint_min - throttle_cruise);

		}

		// Calculate gain scaler from specific energy error to throttle
		float STE_to_throttle = 1.0f / (_throttle_time_constant * (_STE_rate_max - _STE_rate_min));

		// Add proportional and derivative control feedback to the predicted throttle and constrain to throttle limits
		_throttle_setpoint = (_STE_error + _STE_rate_error * _throttle_damping_gain) * STE_to_throttle + throttle_predicted;
		_throttle_setpoint = constrain(_throttle_setpoint, _throttle_setpoint_min, _throttle_setpoint_max);

		// Rate limit the throttle demand
		if (fabsf(_throttle_slewrate) > 0.01f) {
			float throttle_increment_limit = _dt * (_throttle_setpoint_max - _throttle_setpoint_min) * _throttle_slewrate;
			_throttle_setpoint = constrain(_throttle_setpoint, _last_throttle_setpoint - throttle_increment_limit,
						       _last_throttle_setpoint + throttle_increment_limit);
		}

		_last_throttle_setpoint = _throttle_setpoint;

		if (_integrator_gain > 0.0f) {
			// Calculate throttle integrator state upper and lower limits with allowance for
			// 10% throttle saturation to accommodate noise on the demand.
			float integ_state_max = _throttle_setpoint_max - _throttle_setpoint + 0.1f;
			float integ_state_min = _throttle_setpoint_min - _throttle_setpoint - 0.1f;

			// Calculate a throttle demand from the integrated total energy error
			// This will be added to the total throttle demand to compensate for steady state errors
			_throttle_integ_state = _throttle_integ_state + (_STE_error * _integrator_gain) * _dt * STE_to_throttle;

			if (_climbout_mode_active) {
				// During climbout, set the integrator to maximum throttle to prevent transient throttle drop
				// at end of climbout when we transition to closed loop throttle control
				_throttle_integ_state = integ_state_max;

			} else {
				// Respect integrator limits during closed loop operation.
				_throttle_integ_state = constrain(_throttle_integ_state, integ_state_min, integ_state_max);
			}

		} else {
			_throttle_integ_state = 0.0f;
		}

		if (airspeed_sensor_enabled()) {
			// Add the integrator feedback during closed loop operation with an airspeed sensor
			_throttle_setpoint = _throttle_setpoint + _throttle_integ_state;

		} else {
			// when flying without an airspeed sensor, use the predicted throttle only
			_throttle_setpoint = throttle_predicted;

		}

		_throttle_setpoint = constrain(_throttle_setpoint, _throttle_setpoint_min, _throttle_setpoint_max);
	}
}

void TECS::_detect_uncommanded_descent()
{
	/*
	 * This function detects a condition that can occur when the demanded airspeed is greater than the
	 * aircraft can achieve in level flight. When this occurs, the vehicle will continue to reduce height
	 * while attempting to maintain speed.
	*/

	// Calculate rate of change of total specific energy
	float STE_rate = _SPE_rate + _SKE_rate;

	// If total energy is very low and reducing, throttle is high, and we are not in an underspeed condition, then enter uncommanded descent recovery mode
	bool enter_mode = !_uncommanded_descent_recovery && !_underspeed_detected && (_STE_error > 200.0f) && (STE_rate < 0.0f)
			  && (_throttle_setpoint >= _throttle_setpoint_max * 0.9f);

	// If we enter an underspeed condition or recover the required total energy, then exit uncommanded descent recovery mode
	bool exit_mode = _uncommanded_descent_recovery && (_underspeed_detected || (_STE_error < 0.0f));

	if (enter_mode) {
		_uncommanded_descent_recovery = true;

	} else if (exit_mode) {
		_uncommanded_descent_recovery = false;

	}
}

void TECS::_update_pitch_setpoint()
{
	/*
	 * The SKE_weighting variable controls how speed and height control are prioritised by the pitch demand calculation.
	 * A weighting of 1 givea equal speed and height priority
	 * A weighting of 0 gives 100% priority to height control and must be used when no airspeed measurement is available.
	 * A weighting of 2 provides 100% priority to speed control and is used when:
	 * a) an underspeed condition is detected.
	 * b) during climbout where a minimum pitch angle has been set to ensure height is gained. If the airspeed
	 * rises above the demanded value, the pitch angle demand is increased by the TECS controller to prevent the vehicle overspeeding.
	 * The weighting can be adjusted between 0 and 2 depending on speed and height accuracy requirements.
	*/

	// Calculate the weighting applied to control of specific kinetic energy error
	float SKE_weighting = constrain(_pitch_speed_weight, 0.0f, 2.0f);

	if ((_underspeed_detected || _climbout_mode_active) && airspeed_sensor_enabled()) {
		SKE_weighting = 2.0f;

	} else if (!airspeed_sensor_enabled()) {
		SKE_weighting = 0.0f;
	}

	// Calculate the weighting applied to control of specific potential energy error
	float SPE_weighting = 2.0f - SKE_weighting;

	// Calculate the specific energy balance demand which specifies how the available total
	// energy should be allocated to speed (kinetic energy) and height (potential energy)
	float SEB_setpoint = _SPE_setpoint * SPE_weighting - _SKE_setpoint * SKE_weighting;

	// Calculate the specific energy balance rate demand
	float SEB_rate_setpoint = _SPE_rate_setpoint * SPE_weighting - _SKE_rate_setpoint * SKE_weighting;

	// Calculate the specific energy balance and balance rate error
	_SEB_error = SEB_setpoint - (_SPE_estimate * SPE_weighting - _SKE_estimate * SKE_weighting);
	_SEB_rate_error = SEB_rate_setpoint - (_SPE_rate * SPE_weighting - _SKE_rate * SKE_weighting);

	// Calculate derivative from change in climb angle to rate of change of specific energy balance
	float climb_angle_to_SEB_rate = _tas_state * _pitch_time_constant * CONSTANTS_ONE_G;

	if (_integrator_gain > 0.0f) {
		// Calculate pitch integrator input term
		float pitch_integ_input = _SEB_error * _integrator_gain;

		// Prevent the integrator changing in a direction that will increase pitch demand saturation
		// Decay the integrator at the control loop time constant if the pitch demand from the previous time step is saturated
		if (_pitch_setpoint_unc > _pitch_setpoint_max) {
			pitch_integ_input = min(pitch_integ_input,
						min((_pitch_setpoint_max - _pitch_setpoint_unc) * climb_angle_to_SEB_rate / _pitch_time_constant, 0.0f));

		} else if (_pitch_setpoint_unc < _pitch_setpoint_min) {
			pitch_integ_input = max(pitch_integ_input,
						max((_pitch_setpoint_min - _pitch_setpoint_unc) * climb_angle_to_SEB_rate / _pitch_time_constant, 0.0f));
		}

		// Update the pitch integrator state.
		_pitch_integ_state = _pitch_integ_state + pitch_integ_input * _dt;

	} else {
		_pitch_integ_state = 0.0f;
	}

	// Calculate a specific energy correction that doesn't include the integrator contribution
	float SEB_correction = _SEB_error + _SEB_rate_error * _pitch_damping_gain + SEB_rate_setpoint * _pitch_time_constant;

	// During climbout, bias the demanded pitch angle so that a zero speed error produces a pitch angle
	// demand equal to the minimum pitch angle set by the mission plan. This prevents the integrator
	// having to catch up before the nose can be raised to reduce excess speed during climbout.
	if (_climbout_mode_active) {
		SEB_correction += _pitch_setpoint_min * climb_angle_to_SEB_rate;
	}

	// Sum the correction terms and convert to a pitch angle demand. This calculation assumes:
	// a) The climb angle follows pitch angle with a lag that is small enough not to destabilise the control loop.
	// b) The offset between climb angle and pitch angle (angle of attack) is constant, excluding the effect of
	// pitch transients due to control action or turbulence.
	_pitch_setpoint_unc = (SEB_correction + _pitch_integ_state) / climb_angle_to_SEB_rate;
	_pitch_setpoint = constrain(_pitch_setpoint_unc, _pitch_setpoint_min, _pitch_setpoint_max);

	// Comply with the specified vertical acceleration limit by applying a pitch rate limit
	float ptchRateIncr = _dt * _vert_accel_limit / _tas_state;

	if ((_pitch_setpoint - _last_pitch_setpoint) > ptchRateIncr) {
		_pitch_setpoint = _last_pitch_setpoint + ptchRateIncr;

	} else if ((_pitch_setpoint - _last_pitch_setpoint) < -ptchRateIncr) {
		_pitch_setpoint = _last_pitch_setpoint - ptchRateIncr;
	}

	_last_pitch_setpoint = _pitch_setpoint;
}

void TECS::_initialize_states(float pitch, float throttle_cruise, float baro_altitude, float pitch_min_climbout,
			      float EAS2TAS)
{
	if (_pitch_update_timestamp == 0 || _dt > DT_MAX || !_in_air || !_states_initialized) {
		// On first time through or when not using TECS of if there has been a large time slip,
		// states must be reset to allow filters to a clean start
		_vert_vel_state = 0.0f;
		_vert_pos_state = baro_altitude;
		_tas_rate_state = 0.0f;
		_tas_state = _EAS * EAS2TAS;
		_throttle_integ_state =  0.0f;
		_pitch_integ_state = 0.0f;
		_last_throttle_setpoint = (_in_air ? throttle_cruise : 0.0f);;
		_last_pitch_setpoint = constrain(pitch, _pitch_setpoint_min, _pitch_setpoint_max);
		_pitch_setpoint_unc = _last_pitch_setpoint;
		_hgt_setpoint_adj_prev = baro_altitude;
		_hgt_setpoint_adj = _hgt_setpoint_adj_prev;
		_hgt_setpoint_prev = _hgt_setpoint_adj_prev;
		_hgt_setpoint_in_prev = _hgt_setpoint_adj_prev;
		_TAS_setpoint_last = _EAS * EAS2TAS;
		_TAS_setpoint_adj = _TAS_setpoint_last;
		_underspeed_detected = false;
		_uncommanded_descent_recovery = false;
		_STE_rate_error = 0.0f;

		if (_dt > DT_MAX || _dt < DT_MIN) {
			_dt = DT_DEFAULT;
		}

	} else if (_climbout_mode_active) {
		// During climbout use the lower pitch angle limit specified by the
		// calling controller
		_pitch_setpoint_min	   = pitch_min_climbout;

		// throttle lower limit is set to a value that prevents throttle reduction
		_throttle_setpoint_min  = _throttle_setpoint_max - 0.01f;

		// height demand and associated states are set to track the measured height
		_hgt_setpoint_adj_prev  = baro_altitude;
		_hgt_setpoint_adj       = _hgt_setpoint_adj_prev;
		_hgt_setpoint_prev      = _hgt_setpoint_adj_prev;

		// airspeed demand states are set to track the measured airspeed
		_TAS_setpoint_last      = _EAS * EAS2TAS;
		_TAS_setpoint_adj       = _EAS * EAS2TAS;

		// disable speed and decent error condition checks
		_underspeed_detected = false;
		_uncommanded_descent_recovery = false;
	}

	_states_initialized = true;
}

void TECS::_update_STE_rate_lim()
{
	// Calculate the specific total energy upper rate limits from the max throttle climb rate
	_STE_rate_max = _max_climb_rate * CONSTANTS_ONE_G;

	// Calculate the specific total energy lower rate limits from the min throttle sink rate
	_STE_rate_min = - _min_sink_rate * CONSTANTS_ONE_G;
}

void TECS::update_pitch_throttle(const matrix::Dcmf &rotMat, float pitch, float baro_altitude, float hgt_setpoint,
				 float EAS_setpoint, float indicated_airspeed, float eas_to_tas, bool climb_out_setpoint, float pitch_min_climbout,
				 float throttle_min, float throttle_max, float throttle_cruise, float pitch_limit_min, float pitch_limit_max)
{
	// Calculate the time since last update (seconds)
	uint64_t now = hrt_absolute_time();
	_dt = constrain((now - _pitch_update_timestamp) * 1e-6f, DT_MIN, DT_MAX);

	// Set class variables from inputs
	_throttle_setpoint_max = throttle_max;
	_throttle_setpoint_min = throttle_min;
	_pitch_setpoint_max = pitch_limit_max;
	_pitch_setpoint_min = pitch_limit_min;
	_climbout_mode_active = climb_out_setpoint;

	// Initialize selected states and variables as required
	_initialize_states(pitch, throttle_cruise, baro_altitude, pitch_min_climbout, eas_to_tas);

	// Don't run TECS control algorithms when not in flight
	if (!_in_air) {
		return;
	}

	// Update the true airspeed state estimate
	_update_speed_states(EAS_setpoint, indicated_airspeed, eas_to_tas);

	// Calculate rate limits for specific total energy
	_update_STE_rate_lim();

	// Detect an underspeed condition
	_detect_underspeed();

	// Detect an uncommanded descent caused by an unachievable airspeed demand
	_detect_uncommanded_descent();

	// Calculate the demanded true airspeed
	_update_speed_setpoint();

	// Calculate the demanded height
	_update_height_setpoint(hgt_setpoint, baro_altitude);

	// Calculate the specific energy values required by the control loop
	_update_energy_estimates();

	// Calculate the throttle demand
	_update_throttle_setpoint(throttle_cruise, rotMat);

	// Calculate the pitch demand
	_update_pitch_setpoint();

	// Update time stamps
	_pitch_update_timestamp = now;

	// Set TECS mode for next frame
	if (_underspeed_detected) {
		_tecs_mode = ECL_TECS_MODE_UNDERSPEED;

	} else if (_uncommanded_descent_recovery) {
		_tecs_mode = ECL_TECS_MODE_BAD_DESCENT;

	} else if (_climbout_mode_active) {
		_tecs_mode = ECL_TECS_MODE_CLIMBOUT;

	} else {
		// This is the default operation mode
		_tecs_mode = ECL_TECS_MODE_NORMAL;
	}

}
