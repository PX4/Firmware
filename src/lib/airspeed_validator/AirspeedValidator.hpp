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
 * @file AirspeedValidator.hpp
 * Calculates airspeed from differential pressure and checks if this airspeed is valid.
 */

#pragma once

#include <airspeed/airspeed.h>
#include <ecl/airdata/WindEstimator.hpp>
#include <uORB/topics/airspeed_wind.h>


using matrix::Dcmf;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector3f;

using namespace time_literals;

struct airspeed_validator_update_data {
	uint64_t timestamp;
	float airspeed_indicated_raw;
	float airspeed_true_raw;
	uint64_t airspeed_timestamp;
	float lpos_vx;
	float lpos_vy;
	float lpos_vz;
	bool lpos_valid;
	float lpos_evh;
	float lpos_evv;
	float att_q[4];
	float air_pressure_pa;
	float air_temperature_celsius;
	float accel_z;
	float vel_test_ratio;
	float mag_test_ratio;
	bool in_fixed_wing_flight;
};

class AirspeedValidator
{
public:
	AirspeedValidator() = default;
	~AirspeedValidator() = default;

	void update_airspeed_validator(const airspeed_validator_update_data &input_data);

	void reset_airspeed_to_invalid(const uint64_t timestamp);

	float get_IAS() { return _IAS; }
	float get_CAS() { return _CAS; }
	float get_TAS() { return _TAS; }
	bool get_airspeed_valid() { return _airspeed_valid; }
	float get_CAS_scale() {return _CAS_scale;}

	airspeed_wind_s get_wind_estimator_states(uint64_t timestamp);

	// setters wind estimator parameters
	void set_wind_estimator_wind_p_noise(float wind_sigma) { _wind_estimator.set_wind_p_noise(wind_sigma); }
	void set_wind_estimator_tas_scale_p_noise(float tas_scale_sigma) { _wind_estimator.set_tas_scale_p_noise(tas_scale_sigma); }
	void set_wind_estimator_tas_noise(float tas_sigma) { _wind_estimator.set_tas_noise(tas_sigma); }
	void set_wind_estimator_beta_noise(float beta_var) { _wind_estimator.set_beta_noise(beta_var); }
	void set_wind_estimator_tas_gate(uint8_t gate_size)
	{
		_tas_gate = gate_size;
		_wind_estimator.set_tas_gate(gate_size);
	}

	void set_wind_estimator_beta_gate(uint8_t gate_size) { _wind_estimator.set_beta_gate(gate_size); }
	void set_wind_estimator_scale_estimation_on(bool scale_estimation_on) { _wind_estimator_scale_estimation_on = scale_estimation_on;}

	void set_airspeed_scale_manual(float airspeed_scale_manual);

	// setters for failure detection tuning parameters
	void set_tas_innov_threshold(float tas_innov_threshold) { _tas_innov_threshold = tas_innov_threshold; }
	void set_tas_innov_integ_threshold(float tas_innov_integ_threshold) { _tas_innov_integ_threshold = tas_innov_integ_threshold; }
	void set_checks_fail_delay(float checks_fail_delay) { _checks_fail_delay = checks_fail_delay; }
	void set_checks_clear_delay(float checks_clear_delay) { _checks_clear_delay = checks_clear_delay; }

	void set_airspeed_stall(float airspeed_stall) { _airspeed_stall = airspeed_stall; }

private:

	WindEstimator _wind_estimator{}; ///< wind estimator instance running in this particular airspeedValidator

	// wind estimator parameter
	bool _wind_estimator_scale_estimation_on{false};	///< online scale estimation (IAS-->CAS) is on
	float _airspeed_scale_manual{1.0f}; ///< manually entered airspeed scale

	// general states
	bool _in_fixed_wing_flight{false}; ///< variable to bypass innovation and load factor checks
	float _IAS{0.0f}; ///< indicated airsped in m/s
	float _CAS{0.0f}; ///< calibrated airspeed in m/s
	float _TAS{0.0f}; ///< true airspeed in m/s
	float _CAS_scale{1.0f}; ///< scale factor from IAS to CAS

	// states of innovation check
	float _tas_gate{1.0f}; ///< gate size of airspeed innovation (to calculate tas_test_ratio)
	bool _innovations_check_failed{false};  ///< true when airspeed innovations have failed consistency checks
	float _tas_innov_threshold{1.0}; ///< innovation error threshold for triggering innovation check failure
	float _tas_innov_integ_threshold{-1.0}; ///< integrator innovation error threshold for triggering innovation check failure
	uint64_t	_time_last_aspd_innov_check{0};	///< time airspeed innovation was last checked (uSec)
	uint64_t	_time_last_tas_pass{0};		///< last time innovation checks passed
	float		_apsd_innov_integ_state{0.0f};	///< integral of excess normalised airspeed innovation (sec)
	static constexpr uint64_t TAS_INNOV_FAIL_DELAY{1_s};	///< time required for innovation levels to pass or fail (usec)
	uint64_t	_time_wind_estimator_initialized{0};		///< time last time wind estimator was initialized (uSec)

	// states of load factor check
	bool _load_factor_check_failed{false}; ///< load_factor check has detected failure
	float _airspeed_stall{8.0f}; ///< stall speed of aircraft used for load factor check
	float	_load_factor_ratio{0.5f};	///< ratio of maximum load factor predicted by stall speed to measured load factor

	// states of airspeed valid declaration
	bool _airspeed_valid{true}; ///< airspeed valid (pitot or groundspeed-windspeed)
	int _checks_fail_delay{3}; ///< delay for airspeed invalid declaration after single check failure (Sec)
	int _checks_clear_delay{3}; ///< delay for airspeed valid declaration after all checks passed again (Sec)
	uint64_t	_time_checks_passed{0};	///< time the checks have last passed (uSec)
	uint64_t	_time_checks_failed{0};	///< time the checks have last not passed (uSec)

	void update_in_fixed_wing_flight(bool in_fixed_wing_flight) { _in_fixed_wing_flight = in_fixed_wing_flight; }

	void update_wind_estimator(const uint64_t timestamp, float airspeed_true_raw, bool lpos_valid, float lpos_vx,
				   float lpos_vy,
				   float lpos_vz,
				   float lpos_evh, float lpos_evv, const float att_q[4]);
	void update_CAS_scale();
	void update_CAS_TAS(float air_pressure_pa, float air_temperature_celsius);
	void check_airspeed_innovation(uint64_t timestamp, float estimator_status_vel_test_ratio,
				       float estimator_status_mag_test_ratio);
	void check_load_factor(float accel_z);
	void update_airspeed_valid_status(const uint64_t timestamp);
	void reset();

};
