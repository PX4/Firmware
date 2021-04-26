/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include <drivers/drv_hrt.h>
#include <ecl/airdata/WindEstimator.hpp>
#include <matrix/math.hpp>
#include <parameters/param.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/airspeed/airspeed.h>
#include <AirspeedValidator.hpp>
#include <systemlib/mavlink_log.h>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/estimator_selector_status.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/mavlink_log.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_setpoint.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vtol_vehicle_status.h>
#include <uORB/topics/airspeed_wind.h>

using namespace time_literals;

static constexpr uint32_t SCHEDULE_INTERVAL{100_ms};	/**< The schedule interval in usec (10 Hz) */

using matrix::Dcmf;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector3f;

class AirspeedModule : public ModuleBase<AirspeedModule>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:

	AirspeedModule();

	~AirspeedModule() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

private:

	void Run() override;

	static constexpr int MAX_NUM_AIRSPEED_SENSORS = 3; /**< Support max 3 airspeed sensors */
	enum airspeed_index {
		DISABLED_INDEX = -1,
		GROUND_MINUS_WIND_INDEX,
		FIRST_SENSOR_INDEX,
		SECOND_SENSOR_INDEX,
		THIRD_SENSOR_INDEX
	};

	uORB::Publication<airspeed_validated_s> _airspeed_validated_pub {ORB_ID(airspeed_validated)};			/**< airspeed validated topic*/
	uORB::PublicationMulti<airspeed_wind_s> _wind_est_pub[MAX_NUM_AIRSPEED_SENSORS + 1] {{ORB_ID(airspeed_wind)}, {ORB_ID(airspeed_wind)}, {ORB_ID(airspeed_wind)}, {ORB_ID(airspeed_wind)}}; /**< wind estimate topic (for each airspeed validator + purely sideslip fusion) */
	orb_advert_t 	_mavlink_log_pub {nullptr}; 						/**< mavlink log topic*/

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _estimator_selector_status_sub{ORB_ID(estimator_selector_status)};
	uORB::Subscription _estimator_status_sub{ORB_ID(estimator_status)};
	uORB::Subscription _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};
	uORB::Subscription _vehicle_air_data_sub{ORB_ID(vehicle_air_data)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vtol_vehicle_status_sub{ORB_ID(vtol_vehicle_status)};
	uORB::Subscription _position_setpoint_sub{ORB_ID(position_setpoint)};
	uORB::SubscriptionMultiArray<airspeed_s, MAX_NUM_AIRSPEED_SENSORS> _airspeed_subs{ORB_ID::airspeed};


	estimator_status_s _estimator_status {};
	vehicle_acceleration_s _accel {};
	vehicle_air_data_s _vehicle_air_data {};
	vehicle_attitude_s _vehicle_attitude {};
	vehicle_land_detected_s _vehicle_land_detected {};
	vehicle_local_position_s _vehicle_local_position {};
	vehicle_status_s _vehicle_status {};
	vtol_vehicle_status_s _vtol_vehicle_status {};
	position_setpoint_s _position_setpoint {};

	WindEstimator	_wind_estimator_sideslip; /**< wind estimator instance only fusing sideslip */
	airspeed_wind_s _wind_estimate_sideslip {}; /**< wind estimate message for wind estimator instance only fusing sideslip */

	int32_t _number_of_airspeed_sensors{0}; /**<  number of airspeed sensors in use (detected during initialization)*/
	int32_t _prev_number_of_airspeed_sensors{0}; /**<  number of airspeed sensors in previous loop (to detect a new added sensor)*/
	AirspeedValidator _airspeed_validator[MAX_NUM_AIRSPEED_SENSORS] {}; /**< airspeedValidator instances (one for each sensor) */

	hrt_abstime _time_now_usec{0};
	int _valid_airspeed_index{-2}; /**< index of currently chosen (valid) airspeed sensor */
	int _prev_airspeed_index{-2}; /**< previously chosen airspeed sensor index */
	bool _initialized{false}; /**< module initialized*/
	bool _vehicle_local_position_valid{false}; /**< local position (from GPS) valid */
	bool _in_takeoff_situation{true}; /**< in takeoff situation (defined as not yet stall speed reached) */
	float _ground_minus_wind_TAS{0.0f}; /**< true airspeed from groundspeed minus windspeed */
	float _ground_minus_wind_CAS{0.0f}; /**< calibrated airspeed from groundspeed minus windspeed */

	bool _scale_estimation_previously_on{false}; /**< scale_estimation was on in the last cycle */

	hrt_abstime _time_last_airspeed_update[MAX_NUM_AIRSPEED_SENSORS] {};

	perf_counter_t _perf_elapsed{};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::ASPD_W_P_NOISE>) _param_west_w_p_noise,
		(ParamFloat<px4::params::ASPD_SC_P_NOISE>) _param_west_sc_p_noise,
		(ParamFloat<px4::params::ASPD_TAS_NOISE>) _param_west_tas_noise,
		(ParamFloat<px4::params::ASPD_BETA_NOISE>) _param_west_beta_noise,
		(ParamInt<px4::params::ASPD_TAS_GATE>) _param_west_tas_gate,
		(ParamInt<px4::params::ASPD_BETA_GATE>) _param_west_beta_gate,
		(ParamInt<px4::params::ASPD_SCALE_EST>) _param_west_scale_estimation_on,
		(ParamFloat<px4::params::ASPD_SCALE>) _param_west_airspeed_scale,
		(ParamInt<px4::params::ASPD_PRIMARY>) _param_airspeed_primary_index,
		(ParamInt<px4::params::ASPD_DO_CHECKS>) _param_airspeed_checks_on,
		(ParamInt<px4::params::ASPD_FALLBACK_GW>) _param_airspeed_fallback_gw,

		(ParamFloat<px4::params::ASPD_FS_INNOV>) _tas_innov_threshold, /**< innovation check threshold */
		(ParamFloat<px4::params::ASPD_FS_INTEG>) _tas_innov_integ_threshold, /**< innovation check integrator threshold */
		(ParamInt<px4::params::ASPD_FS_T_STOP>) _checks_fail_delay, /**< delay to declare airspeed invalid */
		(ParamInt<px4::params::ASPD_FS_T_START>) _checks_clear_delay, /**<  delay to declare airspeed valid again */
		(ParamFloat<px4::params::ASPD_STALL>) _airspeed_stall /**<  stall speed*/
	)

	void 		init(); 	/**< initialization of the airspeed validator instances */
	void 		check_for_connected_airspeed_sensors(); /**< check for airspeed sensors (airspeed topics) and get _number_of_airspeed_sensors */
	void		update_params(); /**< update parameters */
	void 		poll_topics(); /**< poll all topics required beside airspeed (e.g. current temperature) */
	void 		update_wind_estimator_sideslip(); /**< update the wind estimator instance only fusing sideslip */
	void		update_ground_minus_wind_airspeed(); /**< update airspeed estimate based on groundspeed minus windspeed */
	void 		select_airspeed_and_publish(); /**< select airspeed sensor (or groundspeed-windspeed) */

};

AirspeedModule::AirspeedModule():
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	// initialise parameters
	update_params();

	_perf_elapsed = perf_alloc(PC_ELAPSED, MODULE_NAME": elapsed");
}

AirspeedModule::~AirspeedModule()
{
	ScheduleClear();

	perf_free(_perf_elapsed);
}

int
AirspeedModule::task_spawn(int argc, char *argv[])
{
	AirspeedModule *dev = new AirspeedModule();

	// check if the trampoline is called for the first time
	if (!dev) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(dev);

	dev->ScheduleOnInterval(SCHEDULE_INTERVAL, 10000);
	_task_id = task_id_is_work_queue;
	return PX4_OK;

}
void
AirspeedModule::init()
{
	check_for_connected_airspeed_sensors();

	// Set the default sensor
	if (_param_airspeed_primary_index.get() > _number_of_airspeed_sensors) {
		// constrain the index to the number of sensors connected
		_valid_airspeed_index = math::min(_param_airspeed_primary_index.get(), _number_of_airspeed_sensors);

		if (_number_of_airspeed_sensors == 0) {
			mavlink_log_info(&_mavlink_log_pub,
					 "No airspeed sensor detected. Switch to non-airspeed mode.");

		} else {
			mavlink_log_info(&_mavlink_log_pub,
					 "Primary airspeed index bigger than number connected sensors. Take last sensor.");
		}

	} else {
		// set index to the one provided in the parameter ASPD_PRIMARY
		_valid_airspeed_index =	_param_airspeed_primary_index.get();
	}

	_prev_airspeed_index = _valid_airspeed_index; // used to detect a sensor switching
}

void
AirspeedModule::check_for_connected_airspeed_sensors()
{
	// check for new connected airspeed sensor
	int detected_airspeed_sensors = 0;

	if (_param_airspeed_primary_index.get() > 0) {

		for (int i = 0; i < _airspeed_subs.size(); i++) {
			if (!_airspeed_subs[i].advertised()) {
				break;
			}

			detected_airspeed_sensors = i + 1;
		}

	} else {
		// user has selected groundspeed-windspeed as primary source, or disabled airspeed
		detected_airspeed_sensors = 0;
	}

	_number_of_airspeed_sensors = detected_airspeed_sensors;
}


void
AirspeedModule::Run()
{
	_time_now_usec = hrt_absolute_time(); // hrt time of the current cycle

	// do not run the airspeed selector until 2s after system boot,
	// as data from airspeed sensor	and estimator may not be valid yet
	if (_time_now_usec < 2_s) {
		return;
	}

	perf_begin(_perf_elapsed);

	if (!_initialized) {
		init(); // initialize airspeed validator instances
		_initialized = true;
	}

	parameter_update_s update;

	if (_parameter_update_sub.update(&update)) {
		update_params();
	}



	bool armed = (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

	// check for new connected airspeed sensors as long as we're disarmed
	if (!armed) {
		check_for_connected_airspeed_sensors();
	}

	poll_topics();
	update_wind_estimator_sideslip();
	update_ground_minus_wind_airspeed();

	if (_number_of_airspeed_sensors > 0) {

		// disable checks if not a fixed-wing or the vehicle is landing/landed, as then airspeed can fall below stall speed
		// and wind estimate isn't accurate anymore. Even better would be to have a reliable "ground_contact" detection
		// for fixed-wing landings.
		const bool in_air_fixed_wing = !_vehicle_land_detected.landed &&
					       _position_setpoint.type != position_setpoint_s::SETPOINT_TYPE_LAND &&
					       _vehicle_status.system_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING;

		// Prepare data for airspeed_validator
		struct airspeed_validator_update_data input_data = {};
		input_data.timestamp = _time_now_usec;
		input_data.lpos_vx = _vehicle_local_position.vx;
		input_data.lpos_vy = _vehicle_local_position.vy;
		input_data.lpos_vz = _vehicle_local_position.vz;
		input_data.lpos_valid = _vehicle_local_position_valid;
		input_data.lpos_evh = _vehicle_local_position.evh;
		input_data.lpos_evv = _vehicle_local_position.evv;
		input_data.att_q[0] = _vehicle_attitude.q[0];
		input_data.att_q[1] = _vehicle_attitude.q[1];
		input_data.att_q[2] = _vehicle_attitude.q[2];
		input_data.att_q[3] = _vehicle_attitude.q[3];
		input_data.air_pressure_pa = _vehicle_air_data.baro_pressure_pa;
		input_data.accel_z = _accel.xyz[2];
		input_data.vel_test_ratio = _estimator_status.vel_test_ratio;
		input_data.mag_test_ratio = _estimator_status.mag_test_ratio;

		// iterate through all airspeed sensors, poll new data from them and update their validators
		for (int i = 0; i < _number_of_airspeed_sensors; i++) {

			// poll raw airspeed topic of the i-th sensor
			airspeed_s airspeed_raw;

			if (_airspeed_subs[i].update(&airspeed_raw)) {

				input_data.airspeed_indicated_raw = airspeed_raw.indicated_airspeed_m_s;
				input_data.airspeed_true_raw = airspeed_raw.true_airspeed_m_s;
				input_data.airspeed_timestamp = airspeed_raw.timestamp;
				input_data.air_temperature_celsius = airspeed_raw.air_temperature_celsius;

				// takeoff situation is active from start till one of the sensors' IAS or groundspeed_CAS is above stall speed
				if (airspeed_raw.indicated_airspeed_m_s > _airspeed_stall.get() || _ground_minus_wind_CAS > _airspeed_stall.get()) {
					_in_takeoff_situation = false;
				}

				// reset takeoff_situation to true when not in air and not in fixed-wing mode
				if (!in_air_fixed_wing) {
					_in_takeoff_situation = true;
				}

				input_data.in_fixed_wing_flight = (armed && in_air_fixed_wing && !_in_takeoff_situation);

				// push input data into airspeed validator
				_airspeed_validator[i].update_airspeed_validator(input_data);

				_time_last_airspeed_update[i] = _time_now_usec;

			} else if (_time_now_usec - _time_last_airspeed_update[i] > 1_s) {
				// declare airspeed invalid if more then 1s since last raw airspeed update
				_airspeed_validator[i].reset_airspeed_to_invalid(_time_now_usec);

			}
		}
	}

	select_airspeed_and_publish();

	perf_end(_perf_elapsed);

	if (should_exit()) {
		exit_and_cleanup();
	}
}

void AirspeedModule::update_params()
{
	updateParams();

	_wind_estimator_sideslip.set_wind_p_noise(_param_west_w_p_noise.get());
	_wind_estimator_sideslip.set_tas_scale_p_noise(_param_west_sc_p_noise.get());
	_wind_estimator_sideslip.set_tas_noise(_param_west_tas_noise.get());
	_wind_estimator_sideslip.set_beta_noise(_param_west_beta_noise.get());
	_wind_estimator_sideslip.set_tas_gate(_param_west_tas_gate.get());
	_wind_estimator_sideslip.set_beta_gate(_param_west_beta_gate.get());

	for (int i = 0; i < _number_of_airspeed_sensors; i++) {
		_airspeed_validator[i].set_wind_estimator_wind_p_noise(_param_west_w_p_noise.get());
		_airspeed_validator[i].set_wind_estimator_tas_scale_p_noise(_param_west_sc_p_noise.get());
		_airspeed_validator[i].set_wind_estimator_tas_noise(_param_west_tas_noise.get());
		_airspeed_validator[i].set_wind_estimator_beta_noise(_param_west_beta_noise.get());
		_airspeed_validator[i].set_wind_estimator_tas_gate(_param_west_tas_gate.get());
		_airspeed_validator[i].set_wind_estimator_beta_gate(_param_west_beta_gate.get());
		_airspeed_validator[i].set_wind_estimator_scale_estimation_on(_param_west_scale_estimation_on.get());

		// only apply manual entered airspeed scale to first airspeed measurement
		// TODO: enable multiple airspeed sensors
		_airspeed_validator[0].set_airspeed_scale_manual(_param_west_airspeed_scale.get());

		_airspeed_validator[i].set_tas_innov_threshold(_tas_innov_threshold.get());
		_airspeed_validator[i].set_tas_innov_integ_threshold(_tas_innov_integ_threshold.get());
		_airspeed_validator[i].set_checks_fail_delay(_checks_fail_delay.get());
		_airspeed_validator[i].set_checks_clear_delay(_checks_clear_delay.get());
		_airspeed_validator[i].set_airspeed_stall(_airspeed_stall.get());
	}

	// if the airspeed scale estimation is enabled and the airspeed is valid,
	// then set the scale inside the wind estimator to -1 such that it starts to estimate it
	if (!_scale_estimation_previously_on && _param_west_scale_estimation_on.get()) {
		if (_valid_airspeed_index > 0) {
			// set it to a negative value to start estimation inside wind estimator
			_airspeed_validator[0].set_airspeed_scale_manual(-1.0f);

		} else {
			mavlink_log_info(&_mavlink_log_pub, "Airspeed: can't estimate scale as no valid sensor.");
			_param_west_scale_estimation_on.set(0); // reset this param to 0 as estimation was not turned on
			_param_west_scale_estimation_on.commit_no_notification();
		}

	} else if (_scale_estimation_previously_on && !_param_west_scale_estimation_on.get()) {
		if (_valid_airspeed_index > 0) {

			_param_west_airspeed_scale.set(_airspeed_validator[_valid_airspeed_index - 1].get_CAS_scale());
			_param_west_airspeed_scale.commit_no_notification();
			_airspeed_validator[_valid_airspeed_index - 1].set_airspeed_scale_manual(_param_west_airspeed_scale.get());

			mavlink_log_info(&_mavlink_log_pub, "Airspeed: estimated scale (ASPD_SCALE): %0.2f",
					 (double)_airspeed_validator[_valid_airspeed_index - 1].get_CAS_scale());

		} else {
			mavlink_log_info(&_mavlink_log_pub, "Airspeed: can't estimate scale as no valid sensor.");
		}
	}

	_scale_estimation_previously_on = _param_west_scale_estimation_on.get();

}

void AirspeedModule::poll_topics()
{
	// use primary estimator_status
	if (_estimator_selector_status_sub.updated()) {
		estimator_selector_status_s estimator_selector_status;

		if (_estimator_selector_status_sub.copy(&estimator_selector_status)) {
			if (estimator_selector_status.primary_instance != _estimator_status_sub.get_instance()) {
				_estimator_status_sub.ChangeInstance(estimator_selector_status.primary_instance);
			}
		}
	}

	_estimator_status_sub.update(&_estimator_status);
	_vehicle_acceleration_sub.update(&_accel);
	_vehicle_air_data_sub.update(&_vehicle_air_data);
	_vehicle_attitude_sub.update(&_vehicle_attitude);
	_vehicle_land_detected_sub.update(&_vehicle_land_detected);
	_vehicle_status_sub.update(&_vehicle_status);
	_vtol_vehicle_status_sub.update(&_vtol_vehicle_status);
	_vehicle_local_position_sub.update(&_vehicle_local_position);
	_position_setpoint_sub.update(&_position_setpoint);


	_vehicle_local_position_valid = (_time_now_usec - _vehicle_local_position.timestamp < 1_s)
					&& (_vehicle_local_position.timestamp > 0) && _vehicle_local_position.v_xy_valid;
}

void AirspeedModule::update_wind_estimator_sideslip()
{
	// update wind and airspeed estimator
	_wind_estimator_sideslip.update(_time_now_usec);

	if (_vehicle_local_position_valid && !_vtol_vehicle_status.vtol_in_rw_mode) {
		Vector3f vI(_vehicle_local_position.vx, _vehicle_local_position.vy, _vehicle_local_position.vz);
		Quatf q(_vehicle_attitude.q);

		_wind_estimator_sideslip.fuse_beta(_time_now_usec, vI, q);
	}

	_wind_estimate_sideslip.timestamp = _time_now_usec;
	float wind[2];
	_wind_estimator_sideslip.get_wind(wind);
	_wind_estimate_sideslip.windspeed_north = wind[0];
	_wind_estimate_sideslip.windspeed_east = wind[1];
	float wind_cov[2];
	_wind_estimator_sideslip.get_wind_var(wind_cov);
	_wind_estimate_sideslip.variance_north = wind_cov[0];
	_wind_estimate_sideslip.variance_east = wind_cov[1];
	_wind_estimate_sideslip.tas_innov = _wind_estimator_sideslip.get_tas_innov();
	_wind_estimate_sideslip.tas_innov_var = _wind_estimator_sideslip.get_tas_innov_var();
	_wind_estimate_sideslip.beta_innov = _wind_estimator_sideslip.get_beta_innov();
	_wind_estimate_sideslip.beta_innov_var = _wind_estimator_sideslip.get_beta_innov_var();
	_wind_estimate_sideslip.tas_scale = _wind_estimator_sideslip.get_tas_scale();
	_wind_estimate_sideslip.source = airspeed_wind_s::SOURCE_AS_BETA_ONLY;
}

void AirspeedModule::update_ground_minus_wind_airspeed()
{
	// calculate airspeed estimate based on groundspeed-windspeed to use as fallback
	const float TAS_north = _vehicle_local_position.vx - _wind_estimate_sideslip.windspeed_north;
	const float TAS_east = _vehicle_local_position.vy - _wind_estimate_sideslip.windspeed_east;
	const float TAS_down = _vehicle_local_position.vz; // no wind estimate in z
	_ground_minus_wind_TAS = sqrtf(TAS_north * TAS_north + TAS_east * TAS_east + TAS_down * TAS_down);
	_ground_minus_wind_CAS = calc_CAS_from_TAS(_ground_minus_wind_TAS, _vehicle_air_data.baro_pressure_pa,
				 _vehicle_air_data.baro_temp_celcius);
}


void AirspeedModule::select_airspeed_and_publish()
{
	const bool airspeed_sensor_switching_necessary = _prev_airspeed_index < airspeed_index::FIRST_SENSOR_INDEX ||
			!_airspeed_validator[_prev_airspeed_index - 1].get_airspeed_valid();
	const bool airspeed_sensor_switching_allowed = _number_of_airspeed_sensors > 0 &&
			_param_airspeed_primary_index.get() > airspeed_index::GROUND_MINUS_WIND_INDEX && _param_airspeed_checks_on.get();
	const bool airspeed_sensor_added = _prev_number_of_airspeed_sensors < _number_of_airspeed_sensors;

	if (airspeed_sensor_switching_necessary && (airspeed_sensor_switching_allowed || airspeed_sensor_added)) {

		_valid_airspeed_index = airspeed_index::DISABLED_INDEX;

		// loop through all sensors and take the first valid one
		for (int i = 0; i < _number_of_airspeed_sensors; i++) {
			if (_airspeed_validator[i].get_airspeed_valid()) {
				_valid_airspeed_index = i + 1;
				break;
			}
		}
	}

	// check if airspeed based on ground-wind speed is valid and can be published
	if (_param_airspeed_primary_index.get() > airspeed_index::DISABLED_INDEX &&
	    (_valid_airspeed_index < airspeed_index::FIRST_SENSOR_INDEX
	     || _param_airspeed_primary_index.get() == airspeed_index::GROUND_MINUS_WIND_INDEX)) {

		// _vehicle_local_position_valid determines if ground-wind estimate is valid
		if (_vehicle_local_position_valid &&
		    (_param_airspeed_fallback_gw.get() || _param_airspeed_primary_index.get() == airspeed_index::GROUND_MINUS_WIND_INDEX)) {
			_valid_airspeed_index = airspeed_index::GROUND_MINUS_WIND_INDEX;

		} else {
			_valid_airspeed_index = airspeed_index::DISABLED_INDEX;
		}
	}

	// print warning or info, depending of whether airspeed got declared invalid or healthy
	if (_valid_airspeed_index != _prev_airspeed_index &&
	    (_number_of_airspeed_sensors > 0 || !_vehicle_land_detected.landed) &&
	    _valid_airspeed_index != _prev_airspeed_index) {
		if (_prev_airspeed_index > 0) {
			mavlink_log_critical(&_mavlink_log_pub, "Airspeed sensor failure detected (%i, %i)", _prev_airspeed_index,
					     _valid_airspeed_index);

		} else if (_prev_airspeed_index == 0 && _valid_airspeed_index == -1) {
			mavlink_log_info(&_mavlink_log_pub, "Airspeed estimation invalid");

		} else if (_prev_airspeed_index == -1 && _valid_airspeed_index == 0) {
			mavlink_log_info(&_mavlink_log_pub, "Airspeed estimation valid");

		} else {
			mavlink_log_info(&_mavlink_log_pub, "Airspeed sensor healthy, start using again (%i, %i)", _prev_airspeed_index,
					 _valid_airspeed_index);
		}
	}

	_prev_airspeed_index = _valid_airspeed_index;
	_prev_number_of_airspeed_sensors = _number_of_airspeed_sensors;

	airspeed_validated_s airspeed_validated = {};
	airspeed_validated.timestamp = _time_now_usec;
	airspeed_validated.true_ground_minus_wind_m_s = NAN;
	airspeed_validated.calibrated_ground_minus_wind_m_s = NAN;
	airspeed_validated.indicated_airspeed_m_s = NAN;
	airspeed_validated.calibrated_airspeed_m_s = NAN;
	airspeed_validated.true_airspeed_m_s = NAN;
	airspeed_validated.airspeed_sensor_measurement_valid = false;
	airspeed_validated.selected_airspeed_index = _valid_airspeed_index;

	switch (_valid_airspeed_index) {
	case airspeed_index::DISABLED_INDEX:
		break;

	case airspeed_index::GROUND_MINUS_WIND_INDEX:
		airspeed_validated.indicated_airspeed_m_s = _ground_minus_wind_CAS;
		airspeed_validated.calibrated_airspeed_m_s = _ground_minus_wind_CAS;
		airspeed_validated.true_airspeed_m_s = _ground_minus_wind_TAS;
		airspeed_validated.calibrated_ground_minus_wind_m_s = _ground_minus_wind_CAS;
		airspeed_validated.true_ground_minus_wind_m_s = _ground_minus_wind_TAS;

		break;

	default:
		airspeed_validated.indicated_airspeed_m_s = _airspeed_validator[_valid_airspeed_index - 1].get_IAS();
		airspeed_validated.calibrated_airspeed_m_s = _airspeed_validator[_valid_airspeed_index - 1].get_CAS();
		airspeed_validated.true_airspeed_m_s = _airspeed_validator[_valid_airspeed_index - 1].get_TAS();
		airspeed_validated.calibrated_ground_minus_wind_m_s = _ground_minus_wind_CAS;
		airspeed_validated.true_ground_minus_wind_m_s = _ground_minus_wind_TAS;
		airspeed_validated.airspeed_sensor_measurement_valid = true;
		break;
	}

	_airspeed_validated_pub.publish(airspeed_validated);

	_wind_est_pub[0].publish(_wind_estimate_sideslip);

	// publish the wind estimator states from all airspeed validators
	for (int i = 0; i < _number_of_airspeed_sensors; i++) {
		airspeed_wind_s wind_est = _airspeed_validator[i].get_wind_estimator_states(_time_now_usec);

		if (i == 0) {
			wind_est.source = airspeed_wind_s::SOURCE_AS_SENSOR_1;

		} else if (i == 1) {
			wind_est.source = airspeed_wind_s::SOURCE_AS_SENSOR_2;

		} else {
			wind_est.source = airspeed_wind_s::SOURCE_AS_SENSOR_3;
		}

		_wind_est_pub[i + 1].publish(wind_est);
	}

}

int AirspeedModule::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		int ret = AirspeedModule::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	return print_usage("unknown command");
}

int AirspeedModule::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module provides a single airspeed_validated topic, containing indicated (IAS),
calibrated (CAS), true airspeed (TAS) and the information if the estimation currently
is invalid and if based sensor readings or on groundspeed minus windspeed.
Supporting the input of multiple "raw" airspeed inputs, this module automatically switches
to a valid sensor in case of failure detection. For failure detection as well as for
the estimation of a scale factor from IAS to CAS, it runs several wind estimators
and also publishes those.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("airspeed_estimator", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int airspeed_selector_main(int argc, char *argv[])
{
	return AirspeedModule::main(argc, argv);
}
