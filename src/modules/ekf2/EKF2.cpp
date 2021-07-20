/****************************************************************************
 *
 *   Copyright (c) 2015-2021 PX4 Development Team. All rights reserved.
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

#include "EKF2.hpp"

using namespace time_literals;
using math::constrain;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

pthread_mutex_t ekf2_module_mutex = PTHREAD_MUTEX_INITIALIZER;
static px4::atomic<EKF2 *> _objects[EKF2_MAX_INSTANCES] {};
#if !defined(CONSTRAINED_FLASH)
static px4::atomic<EKF2Selector *> _ekf2_selector {nullptr};
#endif // !CONSTRAINED_FLASH

EKF2::EKF2(bool multi_mode, const px4::wq_config_t &config, bool replay_mode):
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, config),
	_replay_mode(replay_mode && !multi_mode),
	_multi_mode(multi_mode),
	_instance(multi_mode ? -1 : 0),
	_attitude_pub(multi_mode ? ORB_ID(estimator_attitude) : ORB_ID(vehicle_attitude)),
	_local_position_pub(multi_mode ? ORB_ID(estimator_local_position) : ORB_ID(vehicle_local_position)),
	_global_position_pub(multi_mode ? ORB_ID(estimator_global_position) : ORB_ID(vehicle_global_position)),
	_odometry_pub(multi_mode ? ORB_ID(estimator_odometry) : ORB_ID(vehicle_odometry)),
	_wind_pub(multi_mode ? ORB_ID(estimator_wind) : ORB_ID(wind)),
	_params(_ekf.getParamHandle()),
	_param_ekf2_min_obs_dt(_params->sensor_interval_min_ms),
	_param_ekf2_mag_delay(_params->mag_delay_ms),
	_param_ekf2_baro_delay(_params->baro_delay_ms),
	_param_ekf2_gps_delay(_params->gps_delay_ms),
	_param_ekf2_of_delay(_params->flow_delay_ms),
	_param_ekf2_rng_delay(_params->range_delay_ms),
	_param_ekf2_asp_delay(_params->airspeed_delay_ms),
	_param_ekf2_ev_delay(_params->ev_delay_ms),
	_param_ekf2_avel_delay(_params->auxvel_delay_ms),
	_param_ekf2_gyr_noise(_params->gyro_noise),
	_param_ekf2_acc_noise(_params->accel_noise),
	_param_ekf2_gyr_b_noise(_params->gyro_bias_p_noise),
	_param_ekf2_acc_b_noise(_params->accel_bias_p_noise),
	_param_ekf2_mag_e_noise(_params->mage_p_noise),
	_param_ekf2_mag_b_noise(_params->magb_p_noise),
	_param_ekf2_wind_noise(_params->wind_vel_p_noise),
	_param_ekf2_terr_noise(_params->terrain_p_noise),
	_param_ekf2_terr_grad(_params->terrain_gradient),
	_param_ekf2_gps_v_noise(_params->gps_vel_noise),
	_param_ekf2_gps_p_noise(_params->gps_pos_noise),
	_param_ekf2_noaid_noise(_params->pos_noaid_noise),
	_param_ekf2_baro_noise(_params->baro_noise),
	_param_ekf2_baro_gate(_params->baro_innov_gate),
	_param_ekf2_gnd_eff_dz(_params->gnd_effect_deadzone),
	_param_ekf2_gnd_max_hgt(_params->gnd_effect_max_hgt),
	_param_ekf2_gps_p_gate(_params->gps_pos_innov_gate),
	_param_ekf2_gps_v_gate(_params->gps_vel_innov_gate),
	_param_ekf2_tas_gate(_params->tas_innov_gate),
	_param_ekf2_head_noise(_params->mag_heading_noise),
	_param_ekf2_mag_noise(_params->mag_noise),
	_param_ekf2_eas_noise(_params->eas_noise),
	_param_ekf2_beta_gate(_params->beta_innov_gate),
	_param_ekf2_beta_noise(_params->beta_noise),
	_param_ekf2_mag_decl(_params->mag_declination_deg),
	_param_ekf2_hdg_gate(_params->heading_innov_gate),
	_param_ekf2_mag_gate(_params->mag_innov_gate),
	_param_ekf2_decl_type(_params->mag_declination_source),
	_param_ekf2_mag_type(_params->mag_fusion_type),
	_param_ekf2_mag_acclim(_params->mag_acc_gate),
	_param_ekf2_mag_yawlim(_params->mag_yaw_rate_gate),
	_param_ekf2_gps_check(_params->gps_check_mask),
	_param_ekf2_req_eph(_params->req_hacc),
	_param_ekf2_req_epv(_params->req_vacc),
	_param_ekf2_req_sacc(_params->req_sacc),
	_param_ekf2_req_nsats(_params->req_nsats),
	_param_ekf2_req_pdop(_params->req_pdop),
	_param_ekf2_req_hdrift(_params->req_hdrift),
	_param_ekf2_req_vdrift(_params->req_vdrift),
	_param_ekf2_aid_mask(_params->fusion_mode),
	_param_ekf2_hgt_mode(_params->vdist_sensor_type),
	_param_ekf2_terr_mask(_params->terrain_fusion_mode),
	_param_ekf2_noaid_tout(_params->valid_timeout_max),
	_param_ekf2_rng_noise(_params->range_noise),
	_param_ekf2_rng_sfe(_params->range_noise_scaler),
	_param_ekf2_rng_gate(_params->range_innov_gate),
	_param_ekf2_min_rng(_params->rng_gnd_clearance),
	_param_ekf2_rng_pitch(_params->rng_sens_pitch),
	_param_ekf2_rng_aid(_params->range_aid),
	_param_ekf2_rng_a_vmax(_params->max_vel_for_range_aid),
	_param_ekf2_rng_a_hmax(_params->max_hagl_for_range_aid),
	_param_ekf2_rng_a_igate(_params->range_aid_innov_gate),
	_param_ekf2_rng_qlty_t(_params->range_valid_quality_s),
	_param_ekf2_evv_gate(_params->ev_vel_innov_gate),
	_param_ekf2_evp_gate(_params->ev_pos_innov_gate),
	_param_ekf2_of_n_min(_params->flow_noise),
	_param_ekf2_of_n_max(_params->flow_noise_qual_min),
	_param_ekf2_of_qmin(_params->flow_qual_min),
	_param_ekf2_of_gate(_params->flow_innov_gate),
	_param_ekf2_imu_pos_x(_params->imu_pos_body(0)),
	_param_ekf2_imu_pos_y(_params->imu_pos_body(1)),
	_param_ekf2_imu_pos_z(_params->imu_pos_body(2)),
	_param_ekf2_gps_pos_x(_params->gps_pos_body(0)),
	_param_ekf2_gps_pos_y(_params->gps_pos_body(1)),
	_param_ekf2_gps_pos_z(_params->gps_pos_body(2)),
	_param_ekf2_rng_pos_x(_params->rng_pos_body(0)),
	_param_ekf2_rng_pos_y(_params->rng_pos_body(1)),
	_param_ekf2_rng_pos_z(_params->rng_pos_body(2)),
	_param_ekf2_of_pos_x(_params->flow_pos_body(0)),
	_param_ekf2_of_pos_y(_params->flow_pos_body(1)),
	_param_ekf2_of_pos_z(_params->flow_pos_body(2)),
	_param_ekf2_ev_pos_x(_params->ev_pos_body(0)),
	_param_ekf2_ev_pos_y(_params->ev_pos_body(1)),
	_param_ekf2_ev_pos_z(_params->ev_pos_body(2)),
	_param_ekf2_arsp_thr(_params->arsp_thr),
	_param_ekf2_tau_vel(_params->vel_Tau),
	_param_ekf2_tau_pos(_params->pos_Tau),
	_param_ekf2_gbias_init(_params->switch_on_gyro_bias),
	_param_ekf2_abias_init(_params->switch_on_accel_bias),
	_param_ekf2_angerr_init(_params->initial_tilt_err),
	_param_ekf2_abl_lim(_params->acc_bias_lim),
	_param_ekf2_abl_acclim(_params->acc_bias_learn_acc_lim),
	_param_ekf2_abl_gyrlim(_params->acc_bias_learn_gyr_lim),
	_param_ekf2_abl_tau(_params->acc_bias_learn_tc),
	_param_ekf2_drag_noise(_params->drag_noise),
	_param_ekf2_bcoef_x(_params->bcoef_x),
	_param_ekf2_bcoef_y(_params->bcoef_y),
	_param_ekf2_mcoef(_params->mcoef),
	_param_ekf2_aspd_max(_params->max_correction_airspeed),
	_param_ekf2_pcoef_xp(_params->static_pressure_coef_xp),
	_param_ekf2_pcoef_xn(_params->static_pressure_coef_xn),
	_param_ekf2_pcoef_yp(_params->static_pressure_coef_yp),
	_param_ekf2_pcoef_yn(_params->static_pressure_coef_yn),
	_param_ekf2_pcoef_z(_params->static_pressure_coef_z),
	_param_ekf2_move_test(_params->is_moving_scaler),
	_param_ekf2_mag_check(_params->check_mag_strength),
	_param_ekf2_synthetic_mag_z(_params->synthesize_mag_z),
	_param_ekf2_gsf_tas_default(_params->EKFGSF_tas_default)
{
}

EKF2::~EKF2()
{
	perf_free(_ecl_ekf_update_perf);
	perf_free(_ecl_ekf_update_full_perf);
	perf_free(_msg_missed_imu_perf);
	perf_free(_msg_missed_air_data_perf);
	perf_free(_msg_missed_airspeed_perf);
	perf_free(_msg_missed_distance_sensor_perf);
	perf_free(_msg_missed_gps_perf);
	perf_free(_msg_missed_landing_target_pose_perf);
	perf_free(_msg_missed_magnetometer_perf);
	perf_free(_msg_missed_odometry_perf);
	perf_free(_msg_missed_optical_flow_perf);
}

bool EKF2::multi_init(int imu, int mag)
{
	// advertise immediately to ensure consistent uORB instance numbering
	_attitude_pub.advertise();
	_local_position_pub.advertise();
	_global_position_pub.advertise();
	_odometry_pub.advertise();
	_wind_pub.advertise();

	_ekf2_timestamps_pub.advertise();
	_ekf_gps_drift_pub.advertise();
	_estimator_innovation_test_ratios_pub.advertise();
	_estimator_innovation_variances_pub.advertise();
	_estimator_innovations_pub.advertise();
	_estimator_optical_flow_vel_pub.advertise();
	_estimator_sensor_bias_pub.advertise();
	_estimator_states_pub.advertise();
	_estimator_status_pub.advertise();
	_estimator_status_flags_pub.advertise();
	_estimator_visual_odometry_aligned_pub.advertised();
	_yaw_est_pub.advertise();
	_baro_bias_estimate_pub.advertise();

	bool changed_instance = _vehicle_imu_sub.ChangeInstance(imu) && _magnetometer_sub.ChangeInstance(mag);

	const int status_instance = _estimator_states_pub.get_instance();

	if ((status_instance >= 0) && changed_instance
	    && (_attitude_pub.get_instance() == status_instance)
	    && (_local_position_pub.get_instance() == status_instance)
	    && (_global_position_pub.get_instance() == status_instance)) {

		_instance = status_instance;

		ScheduleNow();
		return true;
	}

	PX4_ERR("publication instance problem: %d att: %d lpos: %d gpos: %d", status_instance,
		_attitude_pub.get_instance(), _local_position_pub.get_instance(), _global_position_pub.get_instance());

	return false;
}

int EKF2::print_status()
{
	PX4_INFO_RAW("ekf2:%d attitude: %d, local position: %d, global position: %d\n", _instance, _ekf.attitude_valid(),
		     _ekf.local_position_is_valid(), _ekf.global_position_is_valid());
	perf_print_counter(_ecl_ekf_update_perf);
	perf_print_counter(_ecl_ekf_update_full_perf);
	perf_print_counter(_msg_missed_imu_perf);
	perf_print_counter(_msg_missed_air_data_perf);
	perf_print_counter(_msg_missed_airspeed_perf);
	perf_print_counter(_msg_missed_distance_sensor_perf);
	perf_print_counter(_msg_missed_gps_perf);
	perf_print_counter(_msg_missed_landing_target_pose_perf);
	perf_print_counter(_msg_missed_magnetometer_perf);
	perf_print_counter(_msg_missed_odometry_perf);
	perf_print_counter(_msg_missed_optical_flow_perf);

	return 0;
}

void EKF2::Run()
{
	if (should_exit()) {
		_sensor_combined_sub.unregisterCallback();
		_vehicle_imu_sub.unregisterCallback();

		return;
	}

	// check for parameter updates
	if (_parameter_update_sub.updated() || !_callback_registered) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();

		_ekf.set_min_required_gps_health_time(_param_ekf2_req_gps_h.get() * 1_s);

		// The airspeed scale factor correcton is only available via parameter as used by the airspeed module
		param_t param_aspd_scale = param_find("ASPD_SCALE");

		if (param_aspd_scale != PARAM_INVALID) {
			param_get(param_aspd_scale, &_airspeed_scale_factor);
		}
	}

	if (!_callback_registered) {
		if (_multi_mode) {
			_callback_registered = _vehicle_imu_sub.registerCallback();

		} else {
			_callback_registered = _sensor_combined_sub.registerCallback();
		}

		if (!_callback_registered) {
			PX4_WARN("%d - failed to register callback, retrying", _instance);
			ScheduleDelayed(1_s);
			return;
		}
	}

	if (_vehicle_command_sub.updated()) {
		vehicle_command_s vehicle_command;

		if (_vehicle_command_sub.update(&vehicle_command)) {
			if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN) {
				if (!_ekf.control_status_flags().in_air) {

					uint64_t origin_time {};
					double latitude = vehicle_command.param5;
					double longitude = vehicle_command.param6;
					float altitude = vehicle_command.param7;

					_ekf.setEkfGlobalOrigin(latitude, longitude, altitude);

					// Validate the ekf origin status.
					_ekf.getEkfGlobalOrigin(origin_time, latitude, longitude, altitude);
					PX4_INFO("New NED origin (LLA): %3.10f, %3.10f, %4.3f\n", latitude, longitude, static_cast<double>(altitude));
				}
			}
		}
	}

	bool imu_updated = false;
	imuSample imu_sample_new {};

	hrt_abstime imu_dt = 0; // for tracking time slip later

	if (_multi_mode) {
		const unsigned last_generation = _vehicle_imu_sub.get_last_generation();
		vehicle_imu_s imu;
		imu_updated = _vehicle_imu_sub.update(&imu);

		if (imu_updated && (_vehicle_imu_sub.get_last_generation() != last_generation + 1)) {
			perf_count(_msg_missed_imu_perf);
		}

		imu_sample_new.time_us = imu.timestamp_sample;
		imu_sample_new.delta_ang_dt = imu.delta_angle_dt * 1.e-6f;
		imu_sample_new.delta_ang = Vector3f{imu.delta_angle};
		imu_sample_new.delta_vel_dt = imu.delta_velocity_dt * 1.e-6f;
		imu_sample_new.delta_vel = Vector3f{imu.delta_velocity};

		if (imu.delta_velocity_clipping > 0) {
			imu_sample_new.delta_vel_clipping[0] = imu.delta_velocity_clipping & vehicle_imu_s::CLIPPING_X;
			imu_sample_new.delta_vel_clipping[1] = imu.delta_velocity_clipping & vehicle_imu_s::CLIPPING_Y;
			imu_sample_new.delta_vel_clipping[2] = imu.delta_velocity_clipping & vehicle_imu_s::CLIPPING_Z;
		}

		imu_dt = imu.delta_angle_dt;

		if ((_device_id_accel == 0) || (_device_id_gyro == 0)) {
			_device_id_accel = imu.accel_device_id;
			_device_id_gyro = imu.gyro_device_id;
			_imu_calibration_count = imu.calibration_count;

		} else if ((imu.calibration_count > _imu_calibration_count)
			   || (imu.accel_device_id != _device_id_accel)
			   || (imu.gyro_device_id != _device_id_gyro)) {

			PX4_DEBUG("%d - resetting IMU bias", _instance);
			_device_id_accel = imu.accel_device_id;
			_device_id_gyro = imu.gyro_device_id;

			_ekf.resetImuBias();
			_imu_calibration_count = imu.calibration_count;
		}

	} else {
		const unsigned last_generation = _vehicle_imu_sub.get_last_generation();
		sensor_combined_s sensor_combined;
		imu_updated = _sensor_combined_sub.update(&sensor_combined);

		if (imu_updated && (_sensor_combined_sub.get_last_generation() != last_generation + 1)) {
			perf_count(_msg_missed_imu_perf);
		}

		imu_sample_new.time_us = sensor_combined.timestamp;
		imu_sample_new.delta_ang_dt = sensor_combined.gyro_integral_dt * 1.e-6f;
		imu_sample_new.delta_ang = Vector3f{sensor_combined.gyro_rad} * imu_sample_new.delta_ang_dt;
		imu_sample_new.delta_vel_dt = sensor_combined.accelerometer_integral_dt * 1.e-6f;
		imu_sample_new.delta_vel = Vector3f{sensor_combined.accelerometer_m_s2} * imu_sample_new.delta_vel_dt;

		if (sensor_combined.accelerometer_clipping > 0) {
			imu_sample_new.delta_vel_clipping[0] = sensor_combined.accelerometer_clipping & sensor_combined_s::CLIPPING_X;
			imu_sample_new.delta_vel_clipping[1] = sensor_combined.accelerometer_clipping & sensor_combined_s::CLIPPING_Y;
			imu_sample_new.delta_vel_clipping[2] = sensor_combined.accelerometer_clipping & sensor_combined_s::CLIPPING_Z;
		}

		imu_dt = sensor_combined.gyro_integral_dt;

		if (_sensor_selection_sub.updated() || (_device_id_accel == 0 || _device_id_gyro == 0)) {
			sensor_selection_s sensor_selection;

			if (_sensor_selection_sub.copy(&sensor_selection)) {
				if (_device_id_accel != sensor_selection.accel_device_id) {
					_ekf.resetAccelBias();
					_device_id_accel = sensor_selection.accel_device_id;
				}

				if (_device_id_gyro != sensor_selection.gyro_device_id) {
					_ekf.resetGyroBias();
					_device_id_gyro = sensor_selection.gyro_device_id;
				}
			}
		}
	}

	if (imu_updated) {
		const hrt_abstime now = imu_sample_new.time_us;

		// push imu data into estimator
		_ekf.setIMUData(imu_sample_new);
		PublishAttitude(now); // publish attitude immediately (uses quaternion from output predictor)

		// integrate time to monitor time slippage
		if (_start_time_us > 0) {
			_integrated_time_us += imu_dt;
			_last_time_slip_us = (imu_sample_new.time_us - _start_time_us) - _integrated_time_us;

		} else {
			_start_time_us = imu_sample_new.time_us;
			_last_time_slip_us = 0;
		}

		// update all other topics if they have new data
		if (_status_sub.updated()) {
			vehicle_status_s vehicle_status;

			if (_status_sub.copy(&vehicle_status)) {
				const bool is_fixed_wing = (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING);

				// only fuse synthetic sideslip measurements if conditions are met
				_ekf.set_fuse_beta_flag(is_fixed_wing && (_param_ekf2_fuse_beta.get() == 1));

				// let the EKF know if the vehicle motion is that of a fixed wing (forward flight only relative to wind)
				_ekf.set_is_fixed_wing(is_fixed_wing);

				_preflt_checker.setVehicleCanObserveHeadingInFlight(vehicle_status.vehicle_type !=
						vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);

				_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

				// update standby (arming state) flag
				const bool standby = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_STANDBY);

				if (_standby != standby) {
					_standby = standby;

					// reset preflight checks if transitioning in or out of standby arming state
					_preflt_checker.reset();
				}
			}
		}

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_ekf.set_in_air_status(!vehicle_land_detected.landed);

				if (_armed && (_param_ekf2_gnd_eff_dz.get() > 0.f)) {
					if (!_had_valid_terrain) {
						// update ground effect flag based on land detector state if we've never had valid terrain data
						_ekf.set_gnd_effect_flag(vehicle_land_detected.in_ground_effect);
					}

				} else {
					_ekf.set_gnd_effect_flag(false);
				}
			}
		}

		// ekf2_timestamps (using 0.1 ms relative timestamps)
		ekf2_timestamps_s ekf2_timestamps {
			.timestamp = now,
			.airspeed_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.distance_sensor_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.optical_flow_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.vehicle_air_data_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.vehicle_magnetometer_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.visual_odometry_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
		};

		UpdateAirspeedSample(ekf2_timestamps);
		UpdateAuxVelSample(ekf2_timestamps);
		UpdateBaroSample(ekf2_timestamps);
		UpdateGpsSample(ekf2_timestamps);
		UpdateMagSample(ekf2_timestamps);
		UpdateRangeSample(ekf2_timestamps);

		vehicle_odometry_s ev_odom;
		const bool new_ev_odom = UpdateExtVisionSample(ekf2_timestamps, ev_odom);

		optical_flow_s optical_flow;
		const bool new_optical_flow = UpdateFlowSample(ekf2_timestamps, optical_flow);


		// run the EKF update and output
		const hrt_abstime ekf_update_start = hrt_absolute_time();

		if (_ekf.update()) {
			perf_set_elapsed(_ecl_ekf_update_full_perf, hrt_elapsed_time(&ekf_update_start));

			PublishLocalPosition(now);
			PublishOdometry(now, imu_sample_new);
			PublishGlobalPosition(now);
			PublishSensorBias(now);
			PublishWindEstimate(now);

			// publish status/logging messages
			PublishEkfDriftMetrics(now);
			PublishEventFlags(now);
			PublishStates(now);
			PublishStatus(now);
			PublishStatusFlags(now);
			PublishInnovations(now, imu_sample_new);
			PublishInnovationTestRatios(now);
			PublishInnovationVariances(now);
			PublishYawEstimatorStatus(now);
			PublishBaroBiasEstimate(now);

			UpdateMagCalibration(now);

		} else {
			// ekf no update
			perf_set_elapsed(_ecl_ekf_update_perf, hrt_elapsed_time(&ekf_update_start));
		}

		// publish external visual odometry after fixed frame alignment if new odometry is received
		if (new_ev_odom) {
			PublishOdometryAligned(now, ev_odom);
		}

		if (new_optical_flow) {
			PublishOpticalFlowVel(now, optical_flow);
		}

		// publish ekf2_timestamps
		_ekf2_timestamps_pub.publish(ekf2_timestamps);
	}
}

void EKF2::PublishAttitude(const hrt_abstime &timestamp)
{
	if (_ekf.attitude_valid()) {
		// generate vehicle attitude quaternion data
		vehicle_attitude_s att;
		att.timestamp_sample = timestamp;
		const Quatf q{_ekf.calculate_quaternion()};
		q.copyTo(att.q);

		_ekf.get_quat_reset(&att.delta_q_reset[0], &att.quat_reset_counter);
		att.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
		_attitude_pub.publish(att);

	}  else if (_replay_mode) {
		// in replay mode we have to tell the replay module not to wait for an update
		// we do this by publishing an attitude with zero timestamp
		vehicle_attitude_s att{};
		_attitude_pub.publish(att);
	}
}

void EKF2::PublishBaroBiasEstimate(const hrt_abstime &timestamp)
{

	baro_bias_estimate_s bbe{};
	bbe.timestamp = timestamp;
	bbe.timestamp_sample = timestamp;
	bbe.baro_device_id = _device_id_baro;
	_ekf.getBaroBiasEstimatorStatus(bbe.bias,
					bbe.bias_var,
					bbe.innov,
					bbe.innov_var,
					bbe.innov_test_ratio);
	_baro_bias_estimate_pub.publish(bbe);
}

void EKF2::PublishEkfDriftMetrics(const hrt_abstime &timestamp)
{
	// publish GPS drift data only when updated to minimise overhead
	float gps_drift[3];
	bool blocked;

	if (_ekf.get_gps_drift_metrics(gps_drift, &blocked)) {
		ekf_gps_drift_s drift_data;
		drift_data.hpos_drift_rate = gps_drift[0];
		drift_data.vpos_drift_rate = gps_drift[1];
		drift_data.hspd = gps_drift[2];
		drift_data.blocked = blocked;
		drift_data.timestamp = _replay_mode ? timestamp : hrt_absolute_time();

		_ekf_gps_drift_pub.publish(drift_data);
	}
}

void EKF2::PublishEventFlags(const hrt_abstime &timestamp)
{
	// information events
	uint32_t information_events = _ekf.information_event_status().value;
	bool information_event_updated = false;

	if (information_events != 0) {
		information_event_updated = true;
		_filter_information_event_changes++;
	}

	// warning events
	uint32_t warning_events = _ekf.warning_event_status().value;
	bool warning_event_updated = false;

	if (warning_events != 0) {
		warning_event_updated = true;
		_filter_warning_event_changes++;
	}

	if (information_event_updated || warning_event_updated) {
		estimator_event_flags_s event_flags{};
		event_flags.timestamp_sample = timestamp;

		event_flags.information_event_changes           = _filter_information_event_changes;
		event_flags.gps_checks_passed                   = _ekf.information_event_flags().gps_checks_passed;
		event_flags.reset_vel_to_gps                    = _ekf.information_event_flags().reset_vel_to_gps;
		event_flags.reset_vel_to_flow                   = _ekf.information_event_flags().reset_vel_to_flow;
		event_flags.reset_vel_to_vision                 = _ekf.information_event_flags().reset_vel_to_vision;
		event_flags.reset_vel_to_zero                   = _ekf.information_event_flags().reset_vel_to_zero;
		event_flags.reset_pos_to_last_known             = _ekf.information_event_flags().reset_pos_to_last_known;
		event_flags.reset_pos_to_gps                    = _ekf.information_event_flags().reset_pos_to_gps;
		event_flags.reset_pos_to_vision                 = _ekf.information_event_flags().reset_pos_to_vision;
		event_flags.starting_gps_fusion                 = _ekf.information_event_flags().starting_gps_fusion;
		event_flags.starting_vision_pos_fusion          = _ekf.information_event_flags().starting_vision_pos_fusion;
		event_flags.starting_vision_vel_fusion          = _ekf.information_event_flags().starting_vision_vel_fusion;
		event_flags.starting_vision_yaw_fusion          = _ekf.information_event_flags().starting_vision_yaw_fusion;
		event_flags.yaw_aligned_to_imu_gps              = _ekf.information_event_flags().yaw_aligned_to_imu_gps;

		event_flags.warning_event_changes               = _filter_warning_event_changes;
		event_flags.gps_quality_poor                    = _ekf.warning_event_flags().gps_quality_poor;
		event_flags.gps_fusion_timout                   = _ekf.warning_event_flags().gps_fusion_timout;
		event_flags.gps_data_stopped                    = _ekf.warning_event_flags().gps_data_stopped;
		event_flags.gps_data_stopped_using_alternate    = _ekf.warning_event_flags().gps_data_stopped_using_alternate;
		event_flags.height_sensor_timeout               = _ekf.warning_event_flags().height_sensor_timeout;
		event_flags.stopping_navigation                 = _ekf.warning_event_flags().stopping_mag_use;
		event_flags.invalid_accel_bias_cov_reset        = _ekf.warning_event_flags().invalid_accel_bias_cov_reset;
		event_flags.bad_yaw_using_gps_course            = _ekf.warning_event_flags().bad_yaw_using_gps_course;
		event_flags.stopping_mag_use                    = _ekf.warning_event_flags().stopping_mag_use;
		event_flags.vision_data_stopped                 = _ekf.warning_event_flags().vision_data_stopped;
		event_flags.emergency_yaw_reset_mag_stopped     = _ekf.warning_event_flags().emergency_yaw_reset_mag_stopped;

		event_flags.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
		_estimator_event_flags_pub.publish(event_flags);
	}

	_ekf.clear_information_events();
	_ekf.clear_warning_events();
}

void EKF2::PublishGlobalPosition(const hrt_abstime &timestamp)
{
	if (_ekf.global_position_is_valid() && !_preflt_checker.hasFailed()) {
		const Vector3f position{_ekf.getPosition()};

		// generate and publish global position data
		vehicle_global_position_s global_pos;
		global_pos.timestamp_sample = timestamp;

		// Position of local NED origin in GPS / WGS84 frame
		map_projection_reproject(&_ekf.global_origin(), position(0), position(1), &global_pos.lat, &global_pos.lon);

		float delta_xy[2];
		_ekf.get_posNE_reset(delta_xy, &global_pos.lat_lon_reset_counter);

		global_pos.alt = -position(2) + _ekf.getEkfGlobalOriginAltitude(); // Altitude AMSL in meters
		global_pos.alt_ellipsoid = filter_altitude_ellipsoid(global_pos.alt);

		// global altitude has opposite sign of local down position
		float delta_z;
		uint8_t z_reset_counter;
		_ekf.get_posD_reset(&delta_z, &z_reset_counter);
		global_pos.delta_alt = -delta_z;

		_ekf.get_ekf_gpos_accuracy(&global_pos.eph, &global_pos.epv);

		if (_ekf.isTerrainEstimateValid()) {
			// Terrain altitude in m, WGS84
			global_pos.terrain_alt = _ekf.getEkfGlobalOriginAltitude() - _ekf.getTerrainVertPos();
			global_pos.terrain_alt_valid = true;

		} else {
			global_pos.terrain_alt = NAN;
			global_pos.terrain_alt_valid = false;
		}

		global_pos.dead_reckoning = _ekf.inertial_dead_reckoning(); // True if this position is estimated through dead-reckoning
		global_pos.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
		_global_position_pub.publish(global_pos);
	}
}

void EKF2::PublishInnovations(const hrt_abstime &timestamp, const imuSample &imu)
{
	// publish estimator innovation data
	estimator_innovations_s innovations{};
	innovations.timestamp_sample = timestamp;
	_ekf.getGpsVelPosInnov(innovations.gps_hvel, innovations.gps_vvel, innovations.gps_hpos, innovations.gps_vpos);
	_ekf.getEvVelPosInnov(innovations.ev_hvel, innovations.ev_vvel, innovations.ev_hpos, innovations.ev_vpos);
	_ekf.getBaroHgtInnov(innovations.baro_vpos);
	_ekf.getRngHgtInnov(innovations.rng_vpos);
	_ekf.getAuxVelInnov(innovations.aux_hvel);
	_ekf.getFlowInnov(innovations.flow);
	_ekf.getHeadingInnov(innovations.heading);
	_ekf.getMagInnov(innovations.mag_field);
	_ekf.getDragInnov(innovations.drag);
	_ekf.getAirspeedInnov(innovations.airspeed);
	_ekf.getBetaInnov(innovations.beta);
	_ekf.getHaglInnov(innovations.hagl);
	// Not yet supported
	innovations.aux_vvel = NAN;

	innovations.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_estimator_innovations_pub.publish(innovations);

	// calculate noise filtered velocity innovations which are used for pre-flight checking
	if (_standby) {
		// TODO: move to run before publications
		_preflt_checker.setUsingGpsAiding(_ekf.control_status_flags().gps);
		_preflt_checker.setUsingFlowAiding(_ekf.control_status_flags().opt_flow);
		_preflt_checker.setUsingEvPosAiding(_ekf.control_status_flags().ev_pos);
		_preflt_checker.setUsingEvVelAiding(_ekf.control_status_flags().ev_vel);

		_preflt_checker.update(imu.delta_ang_dt, innovations);
	}
}

void EKF2::PublishInnovationTestRatios(const hrt_abstime &timestamp)
{
	// publish estimator innovation test ratio data
	estimator_innovations_s test_ratios{};
	test_ratios.timestamp_sample = timestamp;
	_ekf.getGpsVelPosInnovRatio(test_ratios.gps_hvel[0], test_ratios.gps_vvel, test_ratios.gps_hpos[0],
				    test_ratios.gps_vpos);
	_ekf.getEvVelPosInnovRatio(test_ratios.ev_hvel[0], test_ratios.ev_vvel, test_ratios.ev_hpos[0], test_ratios.ev_vpos);
	_ekf.getBaroHgtInnovRatio(test_ratios.baro_vpos);
	_ekf.getRngHgtInnovRatio(test_ratios.rng_vpos);
	_ekf.getAuxVelInnovRatio(test_ratios.aux_hvel[0]);
	_ekf.getFlowInnovRatio(test_ratios.flow[0]);
	_ekf.getHeadingInnovRatio(test_ratios.heading);
	_ekf.getMagInnovRatio(test_ratios.mag_field[0]);
	_ekf.getDragInnovRatio(&test_ratios.drag[0]);
	_ekf.getAirspeedInnovRatio(test_ratios.airspeed);
	_ekf.getBetaInnovRatio(test_ratios.beta);
	_ekf.getHaglInnovRatio(test_ratios.hagl);
	// Not yet supported
	test_ratios.aux_vvel = NAN;

	test_ratios.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_estimator_innovation_test_ratios_pub.publish(test_ratios);
}

void EKF2::PublishInnovationVariances(const hrt_abstime &timestamp)
{
	// publish estimator innovation variance data
	estimator_innovations_s variances{};
	variances.timestamp_sample = timestamp;
	_ekf.getGpsVelPosInnovVar(variances.gps_hvel, variances.gps_vvel, variances.gps_hpos, variances.gps_vpos);
	_ekf.getEvVelPosInnovVar(variances.ev_hvel, variances.ev_vvel, variances.ev_hpos, variances.ev_vpos);
	_ekf.getBaroHgtInnovVar(variances.baro_vpos);
	_ekf.getRngHgtInnovVar(variances.rng_vpos);
	_ekf.getAuxVelInnovVar(variances.aux_hvel);
	_ekf.getFlowInnovVar(variances.flow);
	_ekf.getHeadingInnovVar(variances.heading);
	_ekf.getMagInnovVar(variances.mag_field);
	_ekf.getDragInnovVar(variances.drag);
	_ekf.getAirspeedInnovVar(variances.airspeed);
	_ekf.getBetaInnovVar(variances.beta);
	_ekf.getHaglInnovVar(variances.hagl);
	// Not yet supported
	variances.aux_vvel = NAN;

	variances.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_estimator_innovation_variances_pub.publish(variances);
}

void EKF2::PublishLocalPosition(const hrt_abstime &timestamp)
{
	vehicle_local_position_s lpos;
	// generate vehicle local position data
	lpos.timestamp_sample = timestamp;

	// Position of body origin in local NED frame
	const Vector3f position{_ekf.getPosition()};
	lpos.x = position(0);
	lpos.y = position(1);
	lpos.z = position(2);

	// Velocity of body origin in local NED frame (m/s)
	const Vector3f velocity{_ekf.getVelocity()};
	lpos.vx = velocity(0);
	lpos.vy = velocity(1);
	lpos.vz = velocity(2);

	// vertical position time derivative (m/s)
	lpos.z_deriv = _ekf.getVerticalPositionDerivative();

	// Acceleration of body origin in local frame
	const Vector3f vel_deriv{_ekf.getVelocityDerivative()};
	lpos.ax = vel_deriv(0);
	lpos.ay = vel_deriv(1);
	lpos.az = vel_deriv(2);

	// TODO: better status reporting
	lpos.xy_valid = _ekf.local_position_is_valid() && !_preflt_checker.hasHorizFailed();
	lpos.z_valid = !_preflt_checker.hasVertFailed();
	lpos.v_xy_valid = _ekf.local_position_is_valid() && !_preflt_checker.hasHorizFailed();
	lpos.v_z_valid = !_preflt_checker.hasVertFailed();

	// Position of local NED origin in GPS / WGS84 frame
	if (_ekf.global_origin_valid()) {
		lpos.ref_timestamp = _ekf.global_origin().timestamp;
		lpos.ref_lat = math::degrees(_ekf.global_origin().lat_rad); // Reference point latitude in degrees
		lpos.ref_lon = math::degrees(_ekf.global_origin().lon_rad); // Reference point longitude in degrees
		lpos.ref_alt = _ekf.getEkfGlobalOriginAltitude();           // Reference point in MSL altitude meters
		lpos.xy_global = true;
		lpos.z_global = true;

	} else {
		lpos.ref_timestamp = 0;
		lpos.ref_lat = static_cast<double>(NAN);
		lpos.ref_lon = static_cast<double>(NAN);
		lpos.ref_alt = NAN;
		lpos.xy_global = false;
		lpos.z_global = false;
	}

	Quatf delta_q_reset;
	_ekf.get_quat_reset(&delta_q_reset(0), &lpos.heading_reset_counter);

	lpos.heading = Eulerf(_ekf.getQuaternion()).psi();
	lpos.delta_heading = Eulerf(delta_q_reset).psi();

	// Distance to bottom surface (ground) in meters
	// constrain the distance to ground to _rng_gnd_clearance
	lpos.dist_bottom = math::max(_ekf.getTerrainVertPos() - lpos.z, _param_ekf2_min_rng.get());
	lpos.dist_bottom_valid = _ekf.isTerrainEstimateValid();
	lpos.dist_bottom_sensor_bitfield = _ekf.getTerrainEstimateSensorBitfield();

	if (!_had_valid_terrain) {
		_had_valid_terrain = lpos.dist_bottom_valid;
	}

	// only consider ground effect if compensation is configured and the vehicle is armed (props spinning)
	if ((_param_ekf2_gnd_eff_dz.get() > 0.0f) && _armed && lpos.dist_bottom_valid) {
		// set ground effect flag if vehicle is closer than a specified distance to the ground
		_ekf.set_gnd_effect_flag(lpos.dist_bottom < _param_ekf2_gnd_max_hgt.get());

		// if we have no valid terrain estimate and never had one then use ground effect flag from land detector
		// _had_valid_terrain is used to make sure that we don't fall back to using this option
		// if we temporarily lose terrain data due to the distance sensor getting out of range
	}

	_ekf.get_ekf_lpos_accuracy(&lpos.eph, &lpos.epv);
	_ekf.get_ekf_vel_accuracy(&lpos.evh, &lpos.evv);

	// get state reset information of position and velocity
	_ekf.get_posD_reset(&lpos.delta_z, &lpos.z_reset_counter);
	_ekf.get_velD_reset(&lpos.delta_vz, &lpos.vz_reset_counter);
	_ekf.get_posNE_reset(&lpos.delta_xy[0], &lpos.xy_reset_counter);
	_ekf.get_velNE_reset(&lpos.delta_vxy[0], &lpos.vxy_reset_counter);

	// get control limit information
	_ekf.get_ekf_ctrl_limits(&lpos.vxy_max, &lpos.vz_max, &lpos.hagl_min, &lpos.hagl_max);

	// convert NaN to INFINITY
	if (!PX4_ISFINITE(lpos.vxy_max)) {
		lpos.vxy_max = INFINITY;
	}

	if (!PX4_ISFINITE(lpos.vz_max)) {
		lpos.vz_max = INFINITY;
	}

	if (!PX4_ISFINITE(lpos.hagl_min)) {
		lpos.hagl_min = INFINITY;
	}

	if (!PX4_ISFINITE(lpos.hagl_max)) {
		lpos.hagl_max = INFINITY;
	}

	// publish vehicle local position data
	lpos.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_local_position_pub.publish(lpos);
}

void EKF2::PublishOdometry(const hrt_abstime &timestamp, const imuSample &imu)
{
	// generate vehicle odometry data
	vehicle_odometry_s odom;
	odom.timestamp_sample = imu.time_us;

	odom.local_frame = vehicle_odometry_s::LOCAL_FRAME_NED;

	// Vehicle odometry position
	const Vector3f position{_ekf.getPosition()};
	odom.x = position(0);
	odom.y = position(1);
	odom.z = position(2);

	// Vehicle odometry linear velocity
	odom.velocity_frame = vehicle_odometry_s::LOCAL_FRAME_FRD;
	const Vector3f velocity{_ekf.getVelocity()};
	odom.vx = velocity(0);
	odom.vy = velocity(1);
	odom.vz = velocity(2);

	// Vehicle odometry quaternion
	_ekf.getQuaternion().copyTo(odom.q);

	// Vehicle odometry angular rates
	const Vector3f gyro_bias{_ekf.getGyroBias()};
	const Vector3f rates{imu.delta_ang / imu.delta_ang_dt};
	odom.rollspeed = rates(0) - gyro_bias(0);
	odom.pitchspeed = rates(1) - gyro_bias(1);
	odom.yawspeed = rates(2) - gyro_bias(2);

	// get the covariance matrix size
	static constexpr size_t POS_URT_SIZE = sizeof(odom.pose_covariance) / sizeof(odom.pose_covariance[0]);
	static constexpr size_t VEL_URT_SIZE = sizeof(odom.velocity_covariance) / sizeof(odom.velocity_covariance[0]);

	// Get covariances to vehicle odometry
	float covariances[24];
	_ekf.covariances_diagonal().copyTo(covariances);

	// initially set pose covariances to 0
	for (size_t i = 0; i < POS_URT_SIZE; i++) {
		odom.pose_covariance[i] = 0.0;
	}

	// set the position variances
	odom.pose_covariance[odom.COVARIANCE_MATRIX_X_VARIANCE] = covariances[7];
	odom.pose_covariance[odom.COVARIANCE_MATRIX_Y_VARIANCE] = covariances[8];
	odom.pose_covariance[odom.COVARIANCE_MATRIX_Z_VARIANCE] = covariances[9];

	// TODO: implement propagation from quaternion covariance to Euler angle covariance
	// by employing the covariance law

	// initially set velocity covariances to 0
	for (size_t i = 0; i < VEL_URT_SIZE; i++) {
		odom.velocity_covariance[i] = 0.0;
	}

	// set the linear velocity variances
	odom.velocity_covariance[odom.COVARIANCE_MATRIX_VX_VARIANCE] = covariances[4];
	odom.velocity_covariance[odom.COVARIANCE_MATRIX_VY_VARIANCE] = covariances[5];
	odom.velocity_covariance[odom.COVARIANCE_MATRIX_VZ_VARIANCE] = covariances[6];

	// publish vehicle odometry data
	odom.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_odometry_pub.publish(odom);
}

void EKF2::PublishOdometryAligned(const hrt_abstime &timestamp, const vehicle_odometry_s &ev_odom)
{
	const Quatf quat_ev2ekf = _ekf.getVisionAlignmentQuaternion(); // rotates from EV to EKF navigation frame
	const Dcmf ev_rot_mat(quat_ev2ekf);

	vehicle_odometry_s aligned_ev_odom{ev_odom};

	// Rotate external position and velocity into EKF navigation frame
	const Vector3f aligned_pos = ev_rot_mat * Vector3f(ev_odom.x, ev_odom.y, ev_odom.z);
	aligned_ev_odom.x = aligned_pos(0);
	aligned_ev_odom.y = aligned_pos(1);
	aligned_ev_odom.z = aligned_pos(2);

	switch (ev_odom.velocity_frame) {
	case vehicle_odometry_s::BODY_FRAME_FRD: {
			const Vector3f aligned_vel = Dcmf(_ekf.getQuaternion()) * Vector3f(ev_odom.vx, ev_odom.vy, ev_odom.vz);
			aligned_ev_odom.vx = aligned_vel(0);
			aligned_ev_odom.vy = aligned_vel(1);
			aligned_ev_odom.vz = aligned_vel(2);
			break;
		}

	case vehicle_odometry_s::LOCAL_FRAME_FRD: {
			const Vector3f aligned_vel = ev_rot_mat * Vector3f(ev_odom.vx, ev_odom.vy, ev_odom.vz);
			aligned_ev_odom.vx = aligned_vel(0);
			aligned_ev_odom.vy = aligned_vel(1);
			aligned_ev_odom.vz = aligned_vel(2);
			break;
		}
	}

	aligned_ev_odom.velocity_frame = vehicle_odometry_s::LOCAL_FRAME_NED;

	// Compute orientation in EKF navigation frame
	Quatf ev_quat_aligned = quat_ev2ekf * Quatf(ev_odom.q) ;
	ev_quat_aligned.normalize();

	ev_quat_aligned.copyTo(aligned_ev_odom.q);
	quat_ev2ekf.copyTo(aligned_ev_odom.q_offset);

	_estimator_visual_odometry_aligned_pub.publish(aligned_ev_odom);
}

void EKF2::PublishSensorBias(const hrt_abstime &timestamp)
{
	// estimator_sensor_bias
	estimator_sensor_bias_s bias{};
	bias.timestamp_sample = timestamp;

	const Vector3f gyro_bias{_ekf.getGyroBias()};
	const Vector3f accel_bias{_ekf.getAccelBias()};
	const Vector3f mag_bias{_mag_cal_last_bias};

	// only publish on change
	if ((gyro_bias - _last_gyro_bias_published).longerThan(0.001f)
	    || (accel_bias - _last_accel_bias_published).longerThan(0.001f)
	    || (mag_bias - _last_mag_bias_published).longerThan(0.001f)) {

		// take device ids from sensor_selection_s if not using specific vehicle_imu_s
		if (_device_id_gyro != 0) {
			bias.gyro_device_id = _device_id_gyro;
			gyro_bias.copyTo(bias.gyro_bias);
			bias.gyro_bias_limit = math::radians(20.f); // 20 degrees/s see Ekf::constrainStates()
			_ekf.getGyroBiasVariance().copyTo(bias.gyro_bias_variance);
			bias.gyro_bias_valid = true;

			_last_gyro_bias_published = gyro_bias;
		}

		if ((_device_id_accel != 0) && !(_param_ekf2_aid_mask.get() & MASK_INHIBIT_ACC_BIAS)) {
			bias.accel_device_id = _device_id_accel;
			accel_bias.copyTo(bias.accel_bias);
			bias.accel_bias_limit = _params->acc_bias_lim;
			_ekf.getAccelBiasVariance().copyTo(bias.accel_bias_variance);
			bias.accel_bias_valid = !_ekf.fault_status_flags().bad_acc_bias;

			_last_accel_bias_published = accel_bias;
		}

		if (_device_id_mag != 0) {
			bias.mag_device_id = _device_id_mag;
			mag_bias.copyTo(bias.mag_bias);
			bias.mag_bias_limit = 0.5f; // 0.5 Gauss see Ekf::constrainStates()
			_mag_cal_last_bias_variance.copyTo(bias.mag_bias_variance);
			bias.mag_bias_valid = _mag_cal_available;

			_last_mag_bias_published = mag_bias;
		}

		bias.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
		_estimator_sensor_bias_pub.publish(bias);
	}
}

void EKF2::PublishStates(const hrt_abstime &timestamp)
{
	// publish estimator states
	estimator_states_s states;
	states.timestamp_sample = timestamp;
	states.n_states = Ekf::_k_num_states;
	_ekf.getStateAtFusionHorizonAsVector().copyTo(states.states);
	_ekf.covariances_diagonal().copyTo(states.covariances);
	states.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_estimator_states_pub.publish(states);
}

void EKF2::PublishStatus(const hrt_abstime &timestamp)
{
	estimator_status_s status{};
	status.timestamp_sample = timestamp;

	_ekf.getOutputTrackingError().copyTo(status.output_tracking_error);

	_ekf.get_gps_check_status(&status.gps_check_fail_flags);

	// only report enabled GPS check failures (the param indexes are shifted by 1 bit, because they don't include
	// the GPS Fix bit, which is always checked)
	status.gps_check_fail_flags &= ((uint16_t)_params->gps_check_mask << 1) | 1;

	status.control_mode_flags = _ekf.control_status().value;
	status.filter_fault_flags = _ekf.fault_status().value;

	uint16_t innov_check_flags_temp = 0;
	_ekf.get_innovation_test_status(innov_check_flags_temp, status.mag_test_ratio,
					status.vel_test_ratio, status.pos_test_ratio,
					status.hgt_test_ratio, status.tas_test_ratio,
					status.hagl_test_ratio, status.beta_test_ratio);

	// Bit mismatch between ecl and Firmware, combine the 2 first bits to preserve msg definition
	// TODO: legacy use only, those flags are also in estimator_status_flags
	status.innovation_check_flags = (innov_check_flags_temp >> 1) | (innov_check_flags_temp & 0x1);

	_ekf.get_ekf_lpos_accuracy(&status.pos_horiz_accuracy, &status.pos_vert_accuracy);
	_ekf.get_ekf_soln_status(&status.solution_status_flags);
	_ekf.getImuVibrationMetrics().copyTo(status.vibe);

	// reset counters
	status.reset_count_vel_ne = _ekf.state_reset_status().velNE_counter;
	status.reset_count_vel_d = _ekf.state_reset_status().velD_counter;
	status.reset_count_pos_ne = _ekf.state_reset_status().posNE_counter;
	status.reset_count_pod_d = _ekf.state_reset_status().posD_counter;
	status.reset_count_quat = _ekf.state_reset_status().quat_counter;

	status.time_slip = _last_time_slip_us * 1e-6f;

	status.pre_flt_fail_innov_heading = _preflt_checker.hasHeadingFailed();
	status.pre_flt_fail_innov_vel_horiz = _preflt_checker.hasHorizVelFailed();
	status.pre_flt_fail_innov_vel_vert = _preflt_checker.hasVertVelFailed();
	status.pre_flt_fail_innov_height = _preflt_checker.hasHeightFailed();
	status.pre_flt_fail_mag_field_disturbed = _ekf.control_status_flags().mag_field_disturbed;

	status.accel_device_id = _device_id_accel;
	status.baro_device_id = _device_id_baro;
	status.gyro_device_id = _device_id_gyro;
	status.mag_device_id = _device_id_mag;

	status.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_estimator_status_pub.publish(status);
}

void EKF2::PublishStatusFlags(const hrt_abstime &timestamp)
{
	// publish at ~ 1 Hz (or immediately if filter control status or fault status changes)
	bool update = (hrt_elapsed_time(&_last_status_flag_update) >= 1_s);

	// filter control status
	if (_ekf.control_status().value != _filter_control_status) {
		update = true;
		_filter_control_status = _ekf.control_status().value;
		_filter_control_status_changes++;
	}

	// filter fault status
	if (_ekf.fault_status().value != _filter_fault_status) {
		update = true;
		_filter_fault_status = _ekf.fault_status().value;
		_filter_fault_status_changes++;
	}

	// innovation check fail status
	if (_ekf.innov_check_fail_status().value != _innov_check_fail_status) {
		update = true;
		_innov_check_fail_status = _ekf.innov_check_fail_status().value;
		_innov_check_fail_status_changes++;
	}

	if (update) {
		estimator_status_flags_s status_flags{};
		status_flags.timestamp_sample = timestamp;

		status_flags.control_status_changes   = _filter_control_status_changes;
		status_flags.cs_tilt_align            = _ekf.control_status_flags().tilt_align;
		status_flags.cs_yaw_align             = _ekf.control_status_flags().yaw_align;
		status_flags.cs_gps                   = _ekf.control_status_flags().gps;
		status_flags.cs_opt_flow              = _ekf.control_status_flags().opt_flow;
		status_flags.cs_mag_hdg               = _ekf.control_status_flags().mag_hdg;
		status_flags.cs_mag_3d                = _ekf.control_status_flags().mag_3D;
		status_flags.cs_mag_dec               = _ekf.control_status_flags().mag_dec;
		status_flags.cs_in_air                = _ekf.control_status_flags().in_air;
		status_flags.cs_wind                  = _ekf.control_status_flags().wind;
		status_flags.cs_baro_hgt              = _ekf.control_status_flags().baro_hgt;
		status_flags.cs_rng_hgt               = _ekf.control_status_flags().rng_hgt;
		status_flags.cs_gps_hgt               = _ekf.control_status_flags().gps_hgt;
		status_flags.cs_ev_pos                = _ekf.control_status_flags().ev_pos;
		status_flags.cs_ev_yaw                = _ekf.control_status_flags().ev_yaw;
		status_flags.cs_ev_hgt                = _ekf.control_status_flags().ev_hgt;
		status_flags.cs_fuse_beta             = _ekf.control_status_flags().fuse_beta;
		status_flags.cs_mag_field_disturbed   = _ekf.control_status_flags().mag_field_disturbed;
		status_flags.cs_fixed_wing            = _ekf.control_status_flags().fixed_wing;
		status_flags.cs_mag_fault             = _ekf.control_status_flags().mag_fault;
		status_flags.cs_fuse_aspd             = _ekf.control_status_flags().fuse_aspd;
		status_flags.cs_gnd_effect            = _ekf.control_status_flags().gnd_effect;
		status_flags.cs_rng_stuck             = _ekf.control_status_flags().rng_stuck;
		status_flags.cs_gps_yaw               = _ekf.control_status_flags().gps_yaw;
		status_flags.cs_mag_aligned_in_flight = _ekf.control_status_flags().mag_aligned_in_flight;
		status_flags.cs_ev_vel                = _ekf.control_status_flags().ev_vel;
		status_flags.cs_synthetic_mag_z       = _ekf.control_status_flags().synthetic_mag_z;
		status_flags.cs_vehicle_at_rest       = _ekf.control_status_flags().vehicle_at_rest;

		status_flags.fault_status_changes     = _filter_fault_status_changes;
		status_flags.fs_bad_mag_x             = _ekf.fault_status_flags().bad_mag_x;
		status_flags.fs_bad_mag_y             = _ekf.fault_status_flags().bad_mag_y;
		status_flags.fs_bad_mag_z             = _ekf.fault_status_flags().bad_mag_z;
		status_flags.fs_bad_hdg               = _ekf.fault_status_flags().bad_hdg;
		status_flags.fs_bad_mag_decl          = _ekf.fault_status_flags().bad_mag_decl;
		status_flags.fs_bad_airspeed          = _ekf.fault_status_flags().bad_airspeed;
		status_flags.fs_bad_sideslip          = _ekf.fault_status_flags().bad_sideslip;
		status_flags.fs_bad_optflow_x         = _ekf.fault_status_flags().bad_optflow_X;
		status_flags.fs_bad_optflow_y         = _ekf.fault_status_flags().bad_optflow_Y;
		status_flags.fs_bad_vel_n             = _ekf.fault_status_flags().bad_vel_N;
		status_flags.fs_bad_vel_e             = _ekf.fault_status_flags().bad_vel_E;
		status_flags.fs_bad_vel_d             = _ekf.fault_status_flags().bad_vel_D;
		status_flags.fs_bad_pos_n             = _ekf.fault_status_flags().bad_pos_N;
		status_flags.fs_bad_pos_e             = _ekf.fault_status_flags().bad_pos_E;
		status_flags.fs_bad_pos_d             = _ekf.fault_status_flags().bad_pos_D;
		status_flags.fs_bad_acc_bias          = _ekf.fault_status_flags().bad_acc_bias;
		status_flags.fs_bad_acc_vertical      = _ekf.fault_status_flags().bad_acc_vertical;
		status_flags.fs_bad_acc_clipping      = _ekf.fault_status_flags().bad_acc_clipping;

		status_flags.innovation_fault_status_changes = _innov_check_fail_status_changes;
		status_flags.reject_hor_vel                  = _ekf.innov_check_fail_status_flags().reject_hor_vel;
		status_flags.reject_ver_vel                  = _ekf.innov_check_fail_status_flags().reject_ver_vel;
		status_flags.reject_hor_pos                  = _ekf.innov_check_fail_status_flags().reject_hor_pos;
		status_flags.reject_ver_pos                  = _ekf.innov_check_fail_status_flags().reject_ver_pos;
		status_flags.reject_mag_x                    = _ekf.innov_check_fail_status_flags().reject_mag_x;
		status_flags.reject_mag_y                    = _ekf.innov_check_fail_status_flags().reject_mag_y;
		status_flags.reject_mag_z                    = _ekf.innov_check_fail_status_flags().reject_mag_z;
		status_flags.reject_yaw                      = _ekf.innov_check_fail_status_flags().reject_yaw;
		status_flags.reject_airspeed                 = _ekf.innov_check_fail_status_flags().reject_airspeed;
		status_flags.reject_sideslip                 = _ekf.innov_check_fail_status_flags().reject_sideslip;
		status_flags.reject_hagl                     = _ekf.innov_check_fail_status_flags().reject_hagl;
		status_flags.reject_optflow_x                = _ekf.innov_check_fail_status_flags().reject_optflow_X;
		status_flags.reject_optflow_y                = _ekf.innov_check_fail_status_flags().reject_optflow_Y;

		status_flags.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
		_estimator_status_flags_pub.publish(status_flags);

		_last_status_flag_update = status_flags.timestamp;
	}
}

void EKF2::PublishYawEstimatorStatus(const hrt_abstime &timestamp)
{
	static_assert(sizeof(yaw_estimator_status_s::yaw) / sizeof(float) == N_MODELS_EKFGSF,
		      "yaw_estimator_status_s::yaw wrong size");

	yaw_estimator_status_s yaw_est_test_data;

	if (_ekf.getDataEKFGSF(&yaw_est_test_data.yaw_composite, &yaw_est_test_data.yaw_variance,
			       yaw_est_test_data.yaw,
			       yaw_est_test_data.innov_vn, yaw_est_test_data.innov_ve,
			       yaw_est_test_data.weight)) {

		yaw_est_test_data.timestamp_sample = timestamp;
		yaw_est_test_data.timestamp = _replay_mode ? timestamp : hrt_absolute_time();

		_yaw_est_pub.publish(yaw_est_test_data);
	}
}

void EKF2::PublishWindEstimate(const hrt_abstime &timestamp)
{
	if (_ekf.get_wind_status()) {
		// Publish wind estimate only if ekf declares them valid
		wind_s wind{};
		wind.timestamp_sample = timestamp;

		const Vector2f wind_vel = _ekf.getWindVelocity();
		const Vector2f wind_vel_var = _ekf.getWindVelocityVariance();
		_ekf.getAirspeedInnov(wind.tas_innov);
		_ekf.getAirspeedInnovVar(wind.tas_innov_var);
		_ekf.getBetaInnov(wind.beta_innov);
		_ekf.getBetaInnovVar(wind.beta_innov_var);

		wind.windspeed_north = wind_vel(0);
		wind.windspeed_east = wind_vel(1);
		wind.variance_north = wind_vel_var(0);
		wind.variance_east = wind_vel_var(1);
		wind.timestamp = _replay_mode ? timestamp : hrt_absolute_time();

		_wind_pub.publish(wind);
	}
}

void EKF2::PublishOpticalFlowVel(const hrt_abstime &timestamp, const optical_flow_s &flow_sample)
{
	estimator_optical_flow_vel_s flow_vel{};
	flow_vel.timestamp_sample = flow_sample.timestamp;

	_ekf.getFlowVelBody().copyTo(flow_vel.vel_body);
	_ekf.getFlowVelNE().copyTo(flow_vel.vel_ne);
	_ekf.getFlowUncompensated().copyTo(flow_vel.flow_uncompensated_integral);
	_ekf.getFlowCompensated().copyTo(flow_vel.flow_compensated_integral);
	_ekf.getFlowGyro().copyTo(flow_vel.gyro_rate_integral);
	flow_vel.timestamp = _replay_mode ? timestamp : hrt_absolute_time();

	_estimator_optical_flow_vel_pub.publish(flow_vel);
}

float EKF2::filter_altitude_ellipsoid(float amsl_hgt)
{
	float height_diff = static_cast<float>(_gps_alttitude_ellipsoid) * 1e-3f - amsl_hgt;

	if (_gps_alttitude_ellipsoid_previous_timestamp == 0) {

		_wgs84_hgt_offset = height_diff;
		_gps_alttitude_ellipsoid_previous_timestamp = _gps_time_usec;

	} else if (_gps_time_usec != _gps_alttitude_ellipsoid_previous_timestamp) {

		// apply a 10 second first order low pass filter to baro offset
		float dt = 1e-6f * (_gps_time_usec - _gps_alttitude_ellipsoid_previous_timestamp);
		_gps_alttitude_ellipsoid_previous_timestamp = _gps_time_usec;
		float offset_rate_correction = 0.1f * (height_diff - _wgs84_hgt_offset);
		_wgs84_hgt_offset += dt * constrain(offset_rate_correction, -0.1f, 0.1f);
	}

	return amsl_hgt + _wgs84_hgt_offset;
}

void EKF2::UpdateAirspeedSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF airspeed sample
	const unsigned last_generation = _airspeed_sub.get_last_generation();
	airspeed_s airspeed;

	if (_airspeed_sub.update(&airspeed)) {
		if (_msg_missed_airspeed_perf == nullptr) {
			_msg_missed_airspeed_perf = perf_alloc(PC_COUNT, MODULE_NAME": airspeed messages missed");

		} else if (_airspeed_sub.get_last_generation() != last_generation + 1) {
			perf_count(_msg_missed_airspeed_perf);
		}

		// The airspeed measurement received via the airspeed.msg topic has not been corrected
		// for scale favtor errors and requires the ASPD_SCALE correction to be applied.
		// This could be avoided if true_airspeed_m_s from the airspeed-validated.msg topic
		// was used instead, however this would introduce a potential circular dependency
		// via the wind estimator that uses EKF velocity estimates.
		const float true_airspeed_m_s = airspeed.true_airspeed_m_s * _airspeed_scale_factor;

		// only set airspeed data if condition for airspeed fusion are met
		if ((_param_ekf2_arsp_thr.get() > FLT_EPSILON) && (true_airspeed_m_s > _param_ekf2_arsp_thr.get())) {

			airspeedSample airspeed_sample {
				.time_us = airspeed.timestamp,
				.true_airspeed = true_airspeed_m_s,
				.eas2tas = airspeed.true_airspeed_m_s / airspeed.indicated_airspeed_m_s,
			};
			_ekf.setAirspeedData(airspeed_sample);
		}

		ekf2_timestamps.airspeed_timestamp_rel = (int16_t)((int64_t)airspeed.timestamp / 100 -
				(int64_t)ekf2_timestamps.timestamp / 100);
	}
}

void EKF2::UpdateAuxVelSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF auxillary velocity sample
	//  - use the landing target pose estimate as another source of velocity data
	const unsigned last_generation = _landing_target_pose_sub.get_last_generation();
	landing_target_pose_s landing_target_pose;

	if (_landing_target_pose_sub.update(&landing_target_pose)) {
		if (_msg_missed_landing_target_pose_perf == nullptr) {
			_msg_missed_landing_target_pose_perf = perf_alloc(PC_COUNT, MODULE_NAME": landing_target_pose messages missed");

		} else if (_landing_target_pose_sub.get_last_generation() != last_generation + 1) {
			perf_count(_msg_missed_landing_target_pose_perf);
		}

		// we can only use the landing target if it has a fixed position and  a valid velocity estimate
		if (landing_target_pose.is_static && landing_target_pose.rel_vel_valid) {
			// velocity of vehicle relative to target has opposite sign to target relative to vehicle
			auxVelSample auxvel_sample{
				.time_us = landing_target_pose.timestamp,
				.vel = Vector3f{-landing_target_pose.vx_rel, -landing_target_pose.vy_rel, 0.0f},
				.velVar = Vector3f{landing_target_pose.cov_vx_rel, landing_target_pose.cov_vy_rel, 0.0f},
			};
			_ekf.setAuxVelData(auxvel_sample);
		}
	}
}

void EKF2::UpdateBaroSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF baro sample
	const unsigned last_generation = _airdata_sub.get_last_generation();
	vehicle_air_data_s airdata;

	if (_airdata_sub.update(&airdata)) {
		if (_msg_missed_air_data_perf == nullptr) {
			_msg_missed_air_data_perf = perf_alloc(PC_COUNT, MODULE_NAME": vehicle_air_data messages missed");

		} else if (_airdata_sub.get_last_generation() != last_generation + 1) {
			perf_count(_msg_missed_air_data_perf);
		}

		_ekf.set_air_density(airdata.rho);

		_ekf.setBaroData(baroSample{airdata.timestamp_sample, airdata.baro_alt_meter});

		_device_id_baro = airdata.baro_device_id;

		ekf2_timestamps.vehicle_air_data_timestamp_rel = (int16_t)((int64_t)airdata.timestamp / 100 -
				(int64_t)ekf2_timestamps.timestamp / 100);
	}
}

bool EKF2::UpdateExtVisionSample(ekf2_timestamps_s &ekf2_timestamps, vehicle_odometry_s &ev_odom)
{
	// EKF external vision sample
	bool new_ev_odom = false;
	const unsigned last_generation = _ev_odom_sub.get_last_generation();

	if (_ev_odom_sub.update(&ev_odom)) {
		if (_msg_missed_odometry_perf == nullptr) {
			_msg_missed_odometry_perf = perf_alloc(PC_COUNT, MODULE_NAME": vehicle_visual_odometry messages missed");

		} else if (_ev_odom_sub.get_last_generation() != last_generation + 1) {
			perf_count(_msg_missed_odometry_perf);
		}

		extVisionSample ev_data{};

		// if error estimates are unavailable, use parameter defined defaults

		// check for valid velocity data
		if (PX4_ISFINITE(ev_odom.vx) && PX4_ISFINITE(ev_odom.vy) && PX4_ISFINITE(ev_odom.vz)) {
			ev_data.vel(0) = ev_odom.vx;
			ev_data.vel(1) = ev_odom.vy;
			ev_data.vel(2) = ev_odom.vz;

			if (ev_odom.velocity_frame == vehicle_odometry_s::BODY_FRAME_FRD) {
				ev_data.vel_frame = velocity_frame_t::BODY_FRAME_FRD;

			} else {
				ev_data.vel_frame = velocity_frame_t::LOCAL_FRAME_FRD;
			}

			// velocity measurement error from ev_data or parameters
			float param_evv_noise_var = sq(_param_ekf2_evv_noise.get());

			if (!_param_ekf2_ev_noise_md.get() && PX4_ISFINITE(ev_odom.velocity_covariance[ev_odom.COVARIANCE_MATRIX_VX_VARIANCE])
			    && PX4_ISFINITE(ev_odom.velocity_covariance[ev_odom.COVARIANCE_MATRIX_VY_VARIANCE])
			    && PX4_ISFINITE(ev_odom.velocity_covariance[ev_odom.COVARIANCE_MATRIX_VZ_VARIANCE])) {
				ev_data.velCov(0, 0) = ev_odom.velocity_covariance[ev_odom.COVARIANCE_MATRIX_VX_VARIANCE];
				ev_data.velCov(0, 1) = ev_data.velCov(1, 0) = ev_odom.velocity_covariance[1];
				ev_data.velCov(0, 2) = ev_data.velCov(2, 0) = ev_odom.velocity_covariance[2];
				ev_data.velCov(1, 1) = ev_odom.velocity_covariance[ev_odom.COVARIANCE_MATRIX_VY_VARIANCE];
				ev_data.velCov(1, 2) = ev_data.velCov(2, 1) = ev_odom.velocity_covariance[7];
				ev_data.velCov(2, 2) = ev_odom.velocity_covariance[ev_odom.COVARIANCE_MATRIX_VZ_VARIANCE];

			} else {
				ev_data.velCov = matrix::eye<float, 3>() * param_evv_noise_var;
			}
		}

		// check for valid position data
		if (PX4_ISFINITE(ev_odom.x) && PX4_ISFINITE(ev_odom.y) && PX4_ISFINITE(ev_odom.z)) {
			ev_data.pos(0) = ev_odom.x;
			ev_data.pos(1) = ev_odom.y;
			ev_data.pos(2) = ev_odom.z;

			float param_evp_noise_var = sq(_param_ekf2_evp_noise.get());

			// position measurement error from ev_data or parameters
			if (!_param_ekf2_ev_noise_md.get() && PX4_ISFINITE(ev_odom.pose_covariance[ev_odom.COVARIANCE_MATRIX_X_VARIANCE])
			    && PX4_ISFINITE(ev_odom.pose_covariance[ev_odom.COVARIANCE_MATRIX_Y_VARIANCE])
			    && PX4_ISFINITE(ev_odom.pose_covariance[ev_odom.COVARIANCE_MATRIX_Z_VARIANCE])) {
				ev_data.posVar(0) = fmaxf(param_evp_noise_var, ev_odom.pose_covariance[ev_odom.COVARIANCE_MATRIX_X_VARIANCE]);
				ev_data.posVar(1) = fmaxf(param_evp_noise_var, ev_odom.pose_covariance[ev_odom.COVARIANCE_MATRIX_Y_VARIANCE]);
				ev_data.posVar(2) = fmaxf(param_evp_noise_var, ev_odom.pose_covariance[ev_odom.COVARIANCE_MATRIX_Z_VARIANCE]);

			} else {
				ev_data.posVar.setAll(param_evp_noise_var);
			}
		}

		// check for valid orientation data
		if (PX4_ISFINITE(ev_odom.q[0])) {
			ev_data.quat = Quatf(ev_odom.q);

			// orientation measurement error from ev_data or parameters
			float param_eva_noise_var = sq(_param_ekf2_eva_noise.get());

			if (!_param_ekf2_ev_noise_md.get() && PX4_ISFINITE(ev_odom.pose_covariance[ev_odom.COVARIANCE_MATRIX_YAW_VARIANCE])) {
				ev_data.angVar = fmaxf(param_eva_noise_var, ev_odom.pose_covariance[ev_odom.COVARIANCE_MATRIX_YAW_VARIANCE]);

			} else {
				ev_data.angVar = param_eva_noise_var;
			}
		}

		// use timestamp from external computer, clocks are synchronized when using MAVROS
		ev_data.time_us = ev_odom.timestamp_sample;
		_ekf.setExtVisionData(ev_data);

		new_ev_odom = true;

		ekf2_timestamps.visual_odometry_timestamp_rel = (int16_t)((int64_t)ev_odom.timestamp / 100 -
				(int64_t)ekf2_timestamps.timestamp / 100);
	}

	return new_ev_odom;
}

bool EKF2::UpdateFlowSample(ekf2_timestamps_s &ekf2_timestamps, optical_flow_s &optical_flow)
{
	// EKF flow sample
	bool new_optical_flow = false;
	const unsigned last_generation = _optical_flow_sub.get_last_generation();

	if (_optical_flow_sub.update(&optical_flow)) {
		if (_msg_missed_optical_flow_perf == nullptr) {
			_msg_missed_optical_flow_perf = perf_alloc(PC_COUNT, MODULE_NAME": optical_flow messages missed");

		} else if (_optical_flow_sub.get_last_generation() != last_generation + 1) {
			perf_count(_msg_missed_optical_flow_perf);
		}

		flowSample flow {
			.time_us = optical_flow.timestamp,
			// NOTE: the EKF uses the reverse sign convention to the flow sensor. EKF assumes positive LOS rate
			// is produced by a RH rotation of the image about the sensor axis.
			.flow_xy_rad = Vector2f{-optical_flow.pixel_flow_x_integral, -optical_flow.pixel_flow_y_integral},
			.gyro_xyz = Vector3f{-optical_flow.gyro_x_rate_integral, -optical_flow.gyro_y_rate_integral, -optical_flow.gyro_z_rate_integral},
			.dt = 1e-6f * (float)optical_flow.integration_timespan,
			.quality = optical_flow.quality,
		};

		if (PX4_ISFINITE(optical_flow.pixel_flow_y_integral) &&
		    PX4_ISFINITE(optical_flow.pixel_flow_x_integral) &&
		    flow.dt < 1) {

			// Save sensor limits reported by the optical flow sensor
			_ekf.set_optical_flow_limits(optical_flow.max_flow_rate, optical_flow.min_ground_distance,
						     optical_flow.max_ground_distance);

			_ekf.setOpticalFlowData(flow);

			new_optical_flow = true;
		}

		ekf2_timestamps.optical_flow_timestamp_rel = (int16_t)((int64_t)optical_flow.timestamp / 100 -
				(int64_t)ekf2_timestamps.timestamp / 100);
	}

	return new_optical_flow;
}

void EKF2::UpdateGpsSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF GPS message
	const unsigned last_generation = _vehicle_gps_position_sub.get_last_generation();
	vehicle_gps_position_s vehicle_gps_position;

	if (_vehicle_gps_position_sub.update(&vehicle_gps_position)) {
		if (_msg_missed_gps_perf == nullptr) {
			_msg_missed_gps_perf = perf_alloc(PC_COUNT, MODULE_NAME": vehicle_gps_position messages missed");

		} else if (_vehicle_gps_position_sub.get_last_generation() != last_generation + 1) {
			perf_count(_msg_missed_gps_perf);
		}

		gps_message gps_msg{
			.time_usec = vehicle_gps_position.timestamp,
			.lat = vehicle_gps_position.lat,
			.lon = vehicle_gps_position.lon,
			.alt = vehicle_gps_position.alt,
			.yaw = vehicle_gps_position.heading,
			.yaw_offset = vehicle_gps_position.heading_offset,
			.fix_type = vehicle_gps_position.fix_type,
			.eph = vehicle_gps_position.eph,
			.epv = vehicle_gps_position.epv,
			.sacc = vehicle_gps_position.s_variance_m_s,
			.vel_m_s = vehicle_gps_position.vel_m_s,
			.vel_ned = Vector3f{
				vehicle_gps_position.vel_n_m_s,
				vehicle_gps_position.vel_e_m_s,
				vehicle_gps_position.vel_d_m_s
			},
			.vel_ned_valid = vehicle_gps_position.vel_ned_valid,
			.nsats = vehicle_gps_position.satellites_used,
			.pdop = sqrtf(vehicle_gps_position.hdop *vehicle_gps_position.hdop
				      + vehicle_gps_position.vdop * vehicle_gps_position.vdop),
		};
		_ekf.setGpsData(gps_msg);

		_gps_time_usec = gps_msg.time_usec;
		_gps_alttitude_ellipsoid = vehicle_gps_position.alt_ellipsoid;
	}
}

void EKF2::UpdateMagSample(ekf2_timestamps_s &ekf2_timestamps)
{
	const unsigned last_generation = _magnetometer_sub.get_last_generation();
	vehicle_magnetometer_s magnetometer;

	if (_magnetometer_sub.update(&magnetometer)) {
		if (_msg_missed_magnetometer_perf == nullptr) {
			_msg_missed_magnetometer_perf = perf_alloc(PC_COUNT, MODULE_NAME": vehicle_magnetometer messages missed");

		} else if (_magnetometer_sub.get_last_generation() != last_generation + 1) {
			perf_count(_msg_missed_magnetometer_perf);
		}

		bool reset = false;

		// check if magnetometer has changed
		if (magnetometer.device_id != _device_id_mag) {
			if (_device_id_mag != 0) {
				PX4_WARN("%d - mag sensor ID changed %" PRIu32 " -> %" PRIu32, _instance, _device_id_mag, magnetometer.device_id);
			}

			reset = true;

		} else if (magnetometer.calibration_count > _mag_calibration_count) {
			// existing calibration has changed, reset saved mag bias
			PX4_DEBUG("%d - mag %" PRIu32 " calibration updated, resetting bias", _instance, _device_id_mag);
			reset = true;
		}

		if (reset) {
			_ekf.resetMagBias();
			_device_id_mag = magnetometer.device_id;
			_mag_calibration_count = magnetometer.calibration_count;

			// reset magnetometer bias learning
			_mag_cal_total_time_us = 0;
			_mag_cal_last_us = 0;
			_mag_cal_available = false;
		}

		_ekf.setMagData(magSample{magnetometer.timestamp_sample, Vector3f{magnetometer.magnetometer_ga}});

		ekf2_timestamps.vehicle_magnetometer_timestamp_rel = (int16_t)((int64_t)magnetometer.timestamp / 100 -
				(int64_t)ekf2_timestamps.timestamp / 100);
	}
}

void EKF2::UpdateRangeSample(ekf2_timestamps_s &ekf2_timestamps)
{
	if (!_distance_sensor_selected) {
		// get subscription index of first downward-facing range sensor
		uORB::SubscriptionMultiArray<distance_sensor_s> distance_sensor_subs{ORB_ID::distance_sensor};

		if (distance_sensor_subs.advertised()) {
			for (unsigned i = 0; i < distance_sensor_subs.size(); i++) {
				distance_sensor_s distance_sensor;

				if (distance_sensor_subs[i].copy(&distance_sensor)) {
					// only use the first instace which has the correct orientation
					if ((hrt_elapsed_time(&distance_sensor.timestamp) < 100_ms)
					    && (distance_sensor.orientation == distance_sensor_s::ROTATION_DOWNWARD_FACING)) {

						if (_distance_sensor_sub.ChangeInstance(i)) {
							int ndist = orb_group_count(ORB_ID(distance_sensor));

							if (ndist > 1) {
								PX4_INFO("%d - selected distance_sensor:%d (%d advertised)", _instance, i, ndist);
							}

							_distance_sensor_selected = true;
							break;
						}
					}
				}
			}
		}
	}

	// EKF range sample
	const unsigned last_generation = _distance_sensor_sub.get_last_generation();
	distance_sensor_s distance_sensor;

	if (_distance_sensor_sub.update(&distance_sensor)) {
		if (_msg_missed_distance_sensor_perf == nullptr) {
			_msg_missed_distance_sensor_perf = perf_alloc(PC_COUNT, MODULE_NAME": distance_sensor messages missed");

		} else if (_distance_sensor_sub.get_last_generation() != last_generation + 1) {
			perf_count(_msg_missed_distance_sensor_perf);
		}

		if (distance_sensor.orientation == distance_sensor_s::ROTATION_DOWNWARD_FACING) {
			rangeSample range_sample {
				.time_us = distance_sensor.timestamp,
				.rng = distance_sensor.current_distance,
				.quality = distance_sensor.signal_quality,
			};
			_ekf.setRangeData(range_sample);

			// Save sensor limits reported by the rangefinder
			_ekf.set_rangefinder_limits(distance_sensor.min_distance, distance_sensor.max_distance);

			_last_range_sensor_update = distance_sensor.timestamp;
			return;
		}

		ekf2_timestamps.distance_sensor_timestamp_rel = (int16_t)((int64_t)distance_sensor.timestamp / 100 -
				(int64_t)ekf2_timestamps.timestamp / 100);
	}

	if (hrt_elapsed_time(&_last_range_sensor_update) > 1_s) {
		_distance_sensor_selected = false;
	}
}

void EKF2::UpdateMagCalibration(const hrt_abstime &timestamp)
{
	// Check if conditions are OK for learning of magnetometer bias values
	// the EKF is operating in the correct mode and there are no filter faults
	if (_ekf.control_status_flags().in_air && _ekf.control_status_flags().mag_3D && (_ekf.fault_status().value == 0)) {

		if (_mag_cal_last_us != 0) {
			_mag_cal_total_time_us += timestamp - _mag_cal_last_us;

			// Start checking mag bias estimates when we have accumulated sufficient calibration time
			if (_mag_cal_total_time_us > 30_s) {
				_mag_cal_last_bias = _ekf.getMagBias();
				_mag_cal_last_bias_variance = _ekf.getMagBiasVariance();
				_mag_cal_available = true;
			}
		}

		_mag_cal_last_us = timestamp;

	} else {
		// conditions are NOT OK for learning magnetometer bias, reset timestamp
		// but keep the accumulated calibration time
		_mag_cal_last_us = 0;

		if (_ekf.fault_status().value != 0) {
			// if a filter fault has occurred, assume previous learning was invalid and do not
			// count it towards total learning time.
			_mag_cal_total_time_us = 0;
		}
	}

	if (!_armed) {
		// update stored declination value
		if (!_mag_decl_saved) {
			float declination_deg;

			if (_ekf.get_mag_decl_deg(&declination_deg)) {
				_param_ekf2_mag_decl.set(declination_deg);
				_mag_decl_saved = true;

				if (!_multi_mode) {
					_param_ekf2_mag_decl.commit_no_notification();
				}
			}
		}
	}
}

int EKF2::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int EKF2::task_spawn(int argc, char *argv[])
{
	bool success = false;
	bool replay_mode = false;

	if (argc > 1 && !strcmp(argv[1], "-r")) {
		PX4_INFO("replay mode enabled");
		replay_mode = true;
	}

#if !defined(CONSTRAINED_FLASH)
	bool multi_mode = false;
	int32_t imu_instances = 0;
	int32_t mag_instances = 0;

	int32_t sens_imu_mode = 1;
	param_get(param_find("SENS_IMU_MODE"), &sens_imu_mode);

	if (sens_imu_mode == 0) {
		// ekf selector requires SENS_IMU_MODE = 0
		multi_mode = true;

		// IMUs (1 - 4 supported)
		param_get(param_find("EKF2_MULTI_IMU"), &imu_instances);

		if (imu_instances < 1 || imu_instances > 4) {
			const int32_t imu_instances_limited = math::constrain(imu_instances, static_cast<int32_t>(1), static_cast<int32_t>(4));
			PX4_WARN("EKF2_MULTI_IMU limited %" PRId32 " -> %" PRId32, imu_instances, imu_instances_limited);
			param_set_no_notification(param_find("EKF2_MULTI_IMU"), &imu_instances_limited);
			imu_instances = imu_instances_limited;
		}

		int32_t sens_mag_mode = 1;
		param_get(param_find("SENS_MAG_MODE"), &sens_mag_mode);

		if (sens_mag_mode == 0) {
			param_get(param_find("EKF2_MULTI_MAG"), &mag_instances);

			// Mags (1 - 4 supported)
			if (mag_instances < 1 || mag_instances > 4) {
				const int32_t mag_instances_limited = math::constrain(mag_instances, static_cast<int32_t>(1), static_cast<int32_t>(4));
				PX4_WARN("EKF2_MULTI_MAG limited %" PRId32 " -> %" PRId32, mag_instances, mag_instances_limited);
				param_set_no_notification(param_find("EKF2_MULTI_MAG"), &mag_instances_limited);
				mag_instances = mag_instances_limited;
			}

		} else {
			mag_instances = 1;
		}
	}

	if (multi_mode) {
		// Start EKF2Selector if it's not already running
		if (_ekf2_selector.load() == nullptr) {
			EKF2Selector *inst = new EKF2Selector();

			if (inst) {
				_ekf2_selector.store(inst);

			} else {
				PX4_ERR("Failed to create EKF2 selector");
				return PX4_ERROR;
			}
		}

		const hrt_abstime time_started = hrt_absolute_time();
		const int multi_instances = math::min(imu_instances * mag_instances, static_cast<int32_t>(EKF2_MAX_INSTANCES));
		int multi_instances_allocated = 0;

		// allocate EKF2 instances until all found or arming
		uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};

		bool ekf2_instance_created[4][4] {}; // IMUs * mags

		while ((multi_instances_allocated < multi_instances)
		       && (vehicle_status_sub.get().arming_state != vehicle_status_s::ARMING_STATE_ARMED)
		       && ((hrt_elapsed_time(&time_started) < 30_s)
			   || (vehicle_status_sub.get().hil_state == vehicle_status_s::HIL_STATE_ON))) {

			vehicle_status_sub.update();

			for (uint8_t mag = 0; mag < mag_instances; mag++) {
				uORB::SubscriptionData<vehicle_magnetometer_s> vehicle_mag_sub{ORB_ID(vehicle_magnetometer), mag};

				for (uint8_t imu = 0; imu < imu_instances; imu++) {

					uORB::SubscriptionData<vehicle_imu_s> vehicle_imu_sub{ORB_ID(vehicle_imu), imu};
					vehicle_mag_sub.update();

					// Mag & IMU data must be valid, first mag can be ignored initially
					if ((vehicle_mag_sub.get().device_id != 0 || mag == 0)
					    && (vehicle_imu_sub.get().accel_device_id != 0)
					    && (vehicle_imu_sub.get().gyro_device_id != 0)) {

						if (!ekf2_instance_created[imu][mag]) {
							EKF2 *ekf2_inst = new EKF2(true, px4::ins_instance_to_wq(imu), false);

							if (ekf2_inst && ekf2_inst->multi_init(imu, mag)) {
								int actual_instance = ekf2_inst->instance(); // match uORB instance numbering

								if ((actual_instance >= 0) && (_objects[actual_instance].load() == nullptr)) {
									_objects[actual_instance].store(ekf2_inst);
									success = true;
									multi_instances_allocated++;
									ekf2_instance_created[imu][mag] = true;

									if (actual_instance == 0) {
										// force selector to run immediately if first instance started
										_ekf2_selector.load()->ScheduleNow();
									}

									PX4_DEBUG("starting instance %d, IMU:%" PRIu8 " (%" PRIu32 "), MAG:%" PRIu8 " (%" PRIu32 ")", actual_instance,
										  imu, vehicle_imu_sub.get().accel_device_id,
										  mag, vehicle_mag_sub.get().device_id);

									// sleep briefly before starting more instances
									px4_usleep(10000);

								} else {
									PX4_ERR("instance numbering problem instance: %d", actual_instance);
									delete ekf2_inst;
									break;
								}

							} else {
								PX4_ERR("alloc and init failed imu: %" PRIu8 " mag:%" PRIu8, imu, mag);
								px4_usleep(1000000);
								break;
							}
						}

					} else {
						px4_usleep(50000); // give the sensors extra time to start
						continue;
					}
				}
			}

			if (multi_instances_allocated < multi_instances) {
				px4_usleep(100000);
			}
		}

	}

#endif // !CONSTRAINED_FLASH

	else {
		// otherwise launch regular
		EKF2 *ekf2_inst = new EKF2(false, px4::wq_configurations::INS0, replay_mode);

		if (ekf2_inst) {
			_objects[0].store(ekf2_inst);
			ekf2_inst->ScheduleNow();
			success = true;
		}
	}

	return success ? PX4_OK : PX4_ERROR;
}

int EKF2::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Attitude and position estimator using an Extended Kalman Filter. It is used for Multirotors and Fixed-Wing.

The documentation can be found on the [ECL/EKF Overview & Tuning](https://docs.px4.io/master/en/advanced_config/tuning_the_ecl_ekf.html) page.

ekf2 can be started in replay mode (`-r`): in this mode it does not access the system time, but only uses the
timestamps from the sensor topics.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ekf2", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('r', "Enable replay mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
#if !defined(CONSTRAINED_FLASH)
	PRINT_MODULE_USAGE_COMMAND_DESCR("select_instance", "Request switch to new estimator instance");
	PRINT_MODULE_USAGE_ARG("<instance>", "Specify desired estimator instance", false);
#endif // !CONSTRAINED_FLASH
	return 0;
}

extern "C" __EXPORT int ekf2_main(int argc, char *argv[])
{
	if (argc <= 1 || strcmp(argv[1], "-h") == 0) {
		return EKF2::print_usage();
	}

	if (strcmp(argv[1], "start") == 0) {
		int ret = 0;
		EKF2::lock_module();

		ret = EKF2::task_spawn(argc - 1, argv + 1);

		if (ret < 0) {
			PX4_ERR("start failed (%i)", ret);
		}

		EKF2::unlock_module();
		return ret;

#if !defined(CONSTRAINED_FLASH)
	} else if (strcmp(argv[1], "select_instance") == 0) {

		if (EKF2::trylock_module()) {
			if (_ekf2_selector.load()) {
				if (argc > 2) {
					int instance = atoi(argv[2]);
					_ekf2_selector.load()->RequestInstance(instance);
				} else {
					EKF2::unlock_module();
					return EKF2::print_usage("instance required");
				}

			} else {
				PX4_ERR("multi-EKF not active, unable to select instance");
			}

			EKF2::unlock_module();

		} else {
			PX4_WARN("module locked, try again later");
		}

		return 0;
#endif // !CONSTRAINED_FLASH
	} else if (strcmp(argv[1], "status") == 0) {
		if (EKF2::trylock_module()) {
#if !defined(CONSTRAINED_FLASH)
			if (_ekf2_selector.load()) {
				_ekf2_selector.load()->PrintStatus();
			}
#endif // !CONSTRAINED_FLASH

			for (int i = 0; i < EKF2_MAX_INSTANCES; i++) {
				if (_objects[i].load()) {
					PX4_INFO_RAW("\n");
					_objects[i].load()->print_status();
				}
			}

			EKF2::unlock_module();

		} else {
			PX4_WARN("module locked, try again later");
		}

		return 0;

	} else if (strcmp(argv[1], "stop") == 0) {
		EKF2::lock_module();

		if (argc > 2) {
			int instance = atoi(argv[2]);

			if (instance >= 0 && instance < EKF2_MAX_INSTANCES) {
				PX4_INFO("stopping instance %d", instance);
				EKF2 *inst = _objects[instance].load();

				if (inst) {
					inst->request_stop();
					px4_usleep(20000); // 20 ms
					delete inst;
					_objects[instance].store(nullptr);
				}
			} else {
				PX4_ERR("invalid instance %d", instance);
			}

		} else {
			// otherwise stop everything
			bool was_running = false;

#if !defined(CONSTRAINED_FLASH)
			if (_ekf2_selector.load()) {
				PX4_INFO("stopping ekf2 selector");
				_ekf2_selector.load()->Stop();
				delete _ekf2_selector.load();
				_ekf2_selector.store(nullptr);
				was_running = true;
			}
#endif // !CONSTRAINED_FLASH

			for (int i = 0; i < EKF2_MAX_INSTANCES; i++) {
				EKF2 *inst = _objects[i].load();

				if (inst) {
					PX4_INFO("stopping ekf2 instance %d", i);
					was_running = true;
					inst->request_stop();
					px4_usleep(20000); // 20 ms
					delete inst;
					_objects[i].store(nullptr);
				}
			}

			if (!was_running) {
				PX4_WARN("not running");
			}
		}

		EKF2::unlock_module();
		return PX4_OK;
	}

	EKF2::lock_module(); // Lock here, as the method could access _object.
	int ret = EKF2::custom_command(argc - 1, argv + 1);
	EKF2::unlock_module();

	return ret;
}
