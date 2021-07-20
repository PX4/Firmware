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

/**
 * @file EKF2.cpp
 * Implementation of the attitude and position estimator.
 *
 * @author Roman Bapst
 */

#ifndef EKF2_HPP
#define EKF2_HPP

#include "EKF2Selector.hpp"

#include <float.h>

#include <containers/LockGuard.hpp>
#include <drivers/drv_hrt.h>
#include "EKF/ekf.h"
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/time.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/baro_bias_estimate.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/ekf2_timestamps.h>
#include <uORB/topics/ekf_gps_drift.h>
#include <uORB/topics/estimator_event_flags.h>
#include <uORB/topics/estimator_innovations.h>
#include <uORB/topics/estimator_optical_flow_vel.h>
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/estimator_states.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/estimator_status_flags.h>
#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/wind.h>
#include <uORB/topics/yaw_estimator_status.h>

#include "Utility/PreFlightChecker.hpp"

extern pthread_mutex_t ekf2_module_mutex;

class EKF2 final : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	EKF2() = delete;
	EKF2(bool multi_mode, const px4::wq_config_t &config, bool replay_mode);
	~EKF2() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	int print_status();

	bool should_exit() const { return _task_should_exit.load(); }

	void request_stop() { _task_should_exit.store(true); }

	static void lock_module() { pthread_mutex_lock(&ekf2_module_mutex); }
	static bool trylock_module() { return (pthread_mutex_trylock(&ekf2_module_mutex) == 0); }
	static void unlock_module() { pthread_mutex_unlock(&ekf2_module_mutex); }

	bool multi_init(int imu, int mag);

	int instance() const { return _instance; }

private:
	void Run() override;

	void PublishAttitude(const hrt_abstime &timestamp);
	void PublishBaroBiasEstimate(const hrt_abstime &timestamp);
	void PublishEkfDriftMetrics(const hrt_abstime &timestamp);
	void PublishEventFlags(const hrt_abstime &timestamp);
	void PublishGlobalPosition(const hrt_abstime &timestamp);
	void PublishInnovations(const hrt_abstime &timestamp, const imuSample &imu);
	void PublishInnovationTestRatios(const hrt_abstime &timestamp);
	void PublishInnovationVariances(const hrt_abstime &timestamp);
	void PublishLocalPosition(const hrt_abstime &timestamp);
	void PublishOdometry(const hrt_abstime &timestamp, const imuSample &imu);
	void PublishOdometryAligned(const hrt_abstime &timestamp, const vehicle_odometry_s &ev_odom);
	void PublishOpticalFlowVel(const hrt_abstime &timestamp, const optical_flow_s &optical_flow);
	void PublishSensorBias(const hrt_abstime &timestamp);
	void PublishStates(const hrt_abstime &timestamp);
	void PublishStatus(const hrt_abstime &timestamp);
	void PublishStatusFlags(const hrt_abstime &timestamp);
	void PublishWindEstimate(const hrt_abstime &timestamp);
	void PublishYawEstimatorStatus(const hrt_abstime &timestamp);

	void UpdateAirspeedSample(ekf2_timestamps_s &ekf2_timestamps);
	void UpdateAuxVelSample(ekf2_timestamps_s &ekf2_timestamps);
	void UpdateBaroSample(ekf2_timestamps_s &ekf2_timestamps);
	bool UpdateExtVisionSample(ekf2_timestamps_s &ekf2_timestamps, vehicle_odometry_s &ev_odom);
	bool UpdateFlowSample(ekf2_timestamps_s &ekf2_timestamps, optical_flow_s &optical_flow);
	void UpdateGpsSample(ekf2_timestamps_s &ekf2_timestamps);
	void UpdateMagSample(ekf2_timestamps_s &ekf2_timestamps);
	void UpdateRangeSample(ekf2_timestamps_s &ekf2_timestamps);

	void UpdateMagCalibration(const hrt_abstime &timestamp);

	/*
	 * Calculate filtered WGS84 height from estimated AMSL height
	 */
	float filter_altitude_ellipsoid(float amsl_hgt);

	static constexpr float sq(float x) { return x * x; };

	const bool _replay_mode{false};			///< true when we use replay data from a log
	const bool _multi_mode;
	int _instance{0};

	px4::atomic_bool _task_should_exit{false};

	// time slip monitoring
	uint64_t _integrated_time_us = 0;	///< integral of gyro delta time from start (uSec)
	uint64_t _start_time_us = 0;		///< system time at EKF start (uSec)
	int64_t _last_time_slip_us = 0;		///< Last time slip (uSec)

	perf_counter_t _ecl_ekf_update_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": ECL update")};
	perf_counter_t _ecl_ekf_update_full_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": ECL full update")};
	perf_counter_t _msg_missed_imu_perf{perf_alloc(PC_COUNT, MODULE_NAME": IMU message missed")};
	perf_counter_t _msg_missed_air_data_perf{nullptr};
	perf_counter_t _msg_missed_airspeed_perf{nullptr};
	perf_counter_t _msg_missed_distance_sensor_perf{nullptr};
	perf_counter_t _msg_missed_gps_perf{nullptr};
	perf_counter_t _msg_missed_landing_target_pose_perf{nullptr};
	perf_counter_t _msg_missed_magnetometer_perf{nullptr};
	perf_counter_t _msg_missed_odometry_perf{nullptr};
	perf_counter_t _msg_missed_optical_flow_perf{nullptr};

	// Used to check, save and use learned magnetometer biases
	hrt_abstime _mag_cal_last_us{0};	///< last time the EKF was operating a mode that estimates magnetomer biases (uSec)
	hrt_abstime _mag_cal_total_time_us{0};	///< accumulated calibration time since the last save

	Vector3f _mag_cal_last_bias{};	///< last valid XYZ magnetometer bias estimates (Gauss)
	Vector3f _mag_cal_last_bias_variance{};	///< variances for the last valid magnetometer XYZ bias estimates (Gauss**2)
	bool _mag_cal_available{false};	///< true when an unsaved valid calibration for the XYZ magnetometer bias is available

	// Used to control saving of mag declination to be used on next startup
	bool _mag_decl_saved = false;	///< true when the magnetic declination has been saved

	bool _had_valid_terrain{false};			///< true if at any time there was a valid terrain estimate

	uint64_t _gps_time_usec{0};
	int32_t _gps_alttitude_ellipsoid{0};			///< altitude in 1E-3 meters (millimeters) above ellipsoid
	uint64_t _gps_alttitude_ellipsoid_previous_timestamp{0}; ///< storage for previous timestamp to compute dt
	float   _wgs84_hgt_offset = 0;  ///< height offset between AMSL and WGS84

	uint8_t _imu_calibration_count{0};
	uint8_t _mag_calibration_count{0};

	uint32_t _device_id_accel{0};
	uint32_t _device_id_baro{0};
	uint32_t _device_id_gyro{0};
	uint32_t _device_id_mag{0};

	Vector3f _last_accel_bias_published{};
	Vector3f _last_gyro_bias_published{};
	Vector3f _last_mag_bias_published{};

	float _airspeed_scale_factor{1.0f}; ///< scale factor correction applied to airspeed measurements

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _airdata_sub{ORB_ID(vehicle_air_data)};
	uORB::Subscription _airspeed_sub{ORB_ID(airspeed)};
	uORB::Subscription _distance_sensor_sub{ORB_ID(distance_sensor)};
	uORB::Subscription _ev_odom_sub{ORB_ID(vehicle_visual_odometry)};
	uORB::Subscription _landing_target_pose_sub{ORB_ID(landing_target_pose)};
	uORB::Subscription _magnetometer_sub{ORB_ID(vehicle_magnetometer)};
	uORB::Subscription _optical_flow_sub{ORB_ID(optical_flow)};
	uORB::Subscription _sensor_selection_sub{ORB_ID(sensor_selection)};
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription _vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};

	uORB::SubscriptionCallbackWorkItem _sensor_combined_sub{this, ORB_ID(sensor_combined)};
	uORB::SubscriptionCallbackWorkItem _vehicle_imu_sub{this, ORB_ID(vehicle_imu)};

	bool _callback_registered{false};

	bool _distance_sensor_selected{false}; // because we can have several distance sensor instances with different orientations
	bool _armed{false};
	bool _standby{false}; // standby arming state

	hrt_abstime _last_status_flag_update{0};
	hrt_abstime _last_range_sensor_update{0};

	uint32_t _filter_control_status{0};
	uint32_t _filter_fault_status{0};
	uint32_t _innov_check_fail_status{0};

	uint32_t _filter_control_status_changes{0};
	uint32_t _filter_fault_status_changes{0};
	uint32_t _innov_check_fail_status_changes{0};
	uint32_t _filter_warning_event_changes{0};
	uint32_t _filter_information_event_changes{0};

	uORB::PublicationMulti<baro_bias_estimate_s>         _baro_bias_estimate_pub{ORB_ID(baro_bias_estimate)};
	uORB::PublicationMulti<ekf2_timestamps_s>            _ekf2_timestamps_pub{ORB_ID(ekf2_timestamps)};
	uORB::PublicationMulti<ekf_gps_drift_s>              _ekf_gps_drift_pub{ORB_ID(ekf_gps_drift)};
	uORB::PublicationMulti<estimator_innovations_s>      _estimator_innovation_test_ratios_pub{ORB_ID(estimator_innovation_test_ratios)};
	uORB::PublicationMulti<estimator_innovations_s>      _estimator_innovation_variances_pub{ORB_ID(estimator_innovation_variances)};
	uORB::PublicationMulti<estimator_innovations_s>      _estimator_innovations_pub{ORB_ID(estimator_innovations)};
	uORB::PublicationMulti<estimator_optical_flow_vel_s> _estimator_optical_flow_vel_pub{ORB_ID(estimator_optical_flow_vel)};
	uORB::PublicationMulti<estimator_sensor_bias_s>      _estimator_sensor_bias_pub{ORB_ID(estimator_sensor_bias)};
	uORB::PublicationMulti<estimator_states_s>           _estimator_states_pub{ORB_ID(estimator_states)};
	uORB::PublicationMulti<estimator_status_s>           _estimator_status_pub{ORB_ID(estimator_status)};
	uORB::PublicationMulti<estimator_status_flags_s>     _estimator_status_flags_pub{ORB_ID(estimator_status_flags)};
	uORB::PublicationMulti<estimator_event_flags_s>      _estimator_event_flags_pub{ORB_ID(estimator_event_flags)};
	uORB::PublicationMulti<vehicle_odometry_s>           _estimator_visual_odometry_aligned_pub{ORB_ID(estimator_visual_odometry_aligned)};
	uORB::PublicationMulti<yaw_estimator_status_s>       _yaw_est_pub{ORB_ID(yaw_estimator_status)};

	// publications with topic dependent on multi-mode
	uORB::PublicationMulti<vehicle_attitude_s>           _attitude_pub;
	uORB::PublicationMulti<vehicle_local_position_s>     _local_position_pub;
	uORB::PublicationMulti<vehicle_global_position_s>    _global_position_pub;
	uORB::PublicationMulti<vehicle_odometry_s>           _odometry_pub;
	uORB::PublicationMulti<wind_s>              _wind_pub;


	PreFlightChecker _preflt_checker;

	Ekf _ekf;

	parameters *_params;	///< pointer to ekf parameter struct (located in _ekf class instance)

	DEFINE_PARAMETERS(
		(ParamExtInt<px4::params::EKF2_MIN_OBS_DT>)
		_param_ekf2_min_obs_dt,	///< Maximum time delay of any sensor used to increase buffer length to handle large timing jitter (mSec)
		(ParamExtFloat<px4::params::EKF2_MAG_DELAY>)
		_param_ekf2_mag_delay,	///< magnetometer measurement delay relative to the IMU (mSec)
		(ParamExtFloat<px4::params::EKF2_BARO_DELAY>)
		_param_ekf2_baro_delay,	///< barometer height measurement delay relative to the IMU (mSec)
		(ParamExtFloat<px4::params::EKF2_GPS_DELAY>)
		_param_ekf2_gps_delay,	///< GPS measurement delay relative to the IMU (mSec)
		(ParamExtFloat<px4::params::EKF2_OF_DELAY>)
		_param_ekf2_of_delay,	///< optical flow measurement delay relative to the IMU (mSec) - this is to the middle of the optical flow integration interval
		(ParamExtFloat<px4::params::EKF2_RNG_DELAY>)
		_param_ekf2_rng_delay,	///< range finder measurement delay relative to the IMU (mSec)
		(ParamExtFloat<px4::params::EKF2_ASP_DELAY>)
		_param_ekf2_asp_delay,	///< airspeed measurement delay relative to the IMU (mSec)
		(ParamExtFloat<px4::params::EKF2_EV_DELAY>)
		_param_ekf2_ev_delay,	///< off-board vision measurement delay relative to the IMU (mSec)
		(ParamExtFloat<px4::params::EKF2_AVEL_DELAY>)
		_param_ekf2_avel_delay,	///< auxillary velocity measurement delay relative to the IMU (mSec)

		(ParamExtFloat<px4::params::EKF2_GYR_NOISE>)
		_param_ekf2_gyr_noise,	///< IMU angular rate noise used for covariance prediction (rad/sec)
		(ParamExtFloat<px4::params::EKF2_ACC_NOISE>)
		_param_ekf2_acc_noise,	///< IMU acceleration noise use for covariance prediction (m/sec**2)

		// process noise
		(ParamExtFloat<px4::params::EKF2_GYR_B_NOISE>)
		_param_ekf2_gyr_b_noise,	///< process noise for IMU rate gyro bias prediction (rad/sec**2)
		(ParamExtFloat<px4::params::EKF2_ACC_B_NOISE>)
		_param_ekf2_acc_b_noise,///< process noise for IMU accelerometer bias prediction (m/sec**3)
		(ParamExtFloat<px4::params::EKF2_MAG_E_NOISE>)
		_param_ekf2_mag_e_noise,	///< process noise for earth magnetic field prediction (Gauss/sec)
		(ParamExtFloat<px4::params::EKF2_MAG_B_NOISE>)
		_param_ekf2_mag_b_noise,	///< process noise for body magnetic field prediction (Gauss/sec)
		(ParamExtFloat<px4::params::EKF2_WIND_NOISE>)
		_param_ekf2_wind_noise,	///< process noise for wind velocity prediction (m/sec**2)
		(ParamExtFloat<px4::params::EKF2_TERR_NOISE>) _param_ekf2_terr_noise,	///< process noise for terrain offset (m/sec)
		(ParamExtFloat<px4::params::EKF2_TERR_GRAD>)
		_param_ekf2_terr_grad,	///< gradient of terrain used to estimate process noise due to changing position (m/m)

		(ParamExtFloat<px4::params::EKF2_GPS_V_NOISE>)
		_param_ekf2_gps_v_noise,	///< minimum allowed observation noise for gps velocity fusion (m/sec)
		(ParamExtFloat<px4::params::EKF2_GPS_P_NOISE>)
		_param_ekf2_gps_p_noise,	///< minimum allowed observation noise for gps position fusion (m)
		(ParamExtFloat<px4::params::EKF2_NOAID_NOISE>)
		_param_ekf2_noaid_noise,	///< observation noise for non-aiding position fusion (m)
		(ParamExtFloat<px4::params::EKF2_BARO_NOISE>)
		_param_ekf2_baro_noise,	///< observation noise for barometric height fusion (m)
		(ParamExtFloat<px4::params::EKF2_BARO_GATE>)
		_param_ekf2_baro_gate,	///< barometric height innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_GND_EFF_DZ>)
		_param_ekf2_gnd_eff_dz,	///< barometric deadzone range for negative innovations (m)
		(ParamExtFloat<px4::params::EKF2_GND_MAX_HGT>)
		_param_ekf2_gnd_max_hgt,	///< maximum height above the ground level for expected negative baro innovations (m)
		(ParamExtFloat<px4::params::EKF2_GPS_P_GATE>)
		_param_ekf2_gps_p_gate,	///< GPS horizontal position innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_GPS_V_GATE>)
		_param_ekf2_gps_v_gate,	///< GPS velocity innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_TAS_GATE>)
		_param_ekf2_tas_gate,	///< True Airspeed innovation consistency gate size (STD)

		// control of magnetometer fusion
		(ParamExtFloat<px4::params::EKF2_HEAD_NOISE>)
		_param_ekf2_head_noise,	///< measurement noise used for simple heading fusion (rad)
		(ParamExtFloat<px4::params::EKF2_MAG_NOISE>)
		_param_ekf2_mag_noise,		///< measurement noise used for 3-axis magnetoemeter fusion (Gauss)
		(ParamExtFloat<px4::params::EKF2_EAS_NOISE>)
		_param_ekf2_eas_noise,		///< measurement noise used for airspeed fusion (m/sec)
		(ParamExtFloat<px4::params::EKF2_BETA_GATE>)
		_param_ekf2_beta_gate, ///< synthetic sideslip innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_BETA_NOISE>) _param_ekf2_beta_noise,	///< synthetic sideslip noise (rad)
		(ParamExtFloat<px4::params::EKF2_MAG_DECL>) _param_ekf2_mag_decl,///< magnetic declination (degrees)
		(ParamExtFloat<px4::params::EKF2_HDG_GATE>)
		_param_ekf2_hdg_gate,///< heading fusion innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_MAG_GATE>)
		_param_ekf2_mag_gate,	///< magnetometer fusion innovation consistency gate size (STD)
		(ParamExtInt<px4::params::EKF2_DECL_TYPE>)
		_param_ekf2_decl_type,	///< bitmask used to control the handling of declination data
		(ParamExtInt<px4::params::EKF2_MAG_TYPE>)
		_param_ekf2_mag_type,	///< integer used to specify the type of magnetometer fusion used
		(ParamExtFloat<px4::params::EKF2_MAG_ACCLIM>)
		_param_ekf2_mag_acclim,	///< integer used to specify the type of magnetometer fusion used
		(ParamExtFloat<px4::params::EKF2_MAG_YAWLIM>)
		_param_ekf2_mag_yawlim,	///< yaw rate threshold used by mode select logic (rad/sec)

		(ParamExtInt<px4::params::EKF2_GPS_CHECK>)
		_param_ekf2_gps_check,	///< bitmask used to control which GPS quality checks are used
		(ParamExtFloat<px4::params::EKF2_REQ_EPH>) _param_ekf2_req_eph,	///< maximum acceptable horiz position error (m)
		(ParamExtFloat<px4::params::EKF2_REQ_EPV>) _param_ekf2_req_epv,	///< maximum acceptable vert position error (m)
		(ParamExtFloat<px4::params::EKF2_REQ_SACC>) _param_ekf2_req_sacc,	///< maximum acceptable speed error (m/s)
		(ParamExtInt<px4::params::EKF2_REQ_NSATS>) _param_ekf2_req_nsats,	///< minimum acceptable satellite count
		(ParamExtFloat<px4::params::EKF2_REQ_PDOP>)
		_param_ekf2_req_pdop,	///< maximum acceptable position dilution of precision
		(ParamExtFloat<px4::params::EKF2_REQ_HDRIFT>)
		_param_ekf2_req_hdrift,	///< maximum acceptable horizontal drift speed (m/s)
		(ParamExtFloat<px4::params::EKF2_REQ_VDRIFT>) _param_ekf2_req_vdrift,	///< maximum acceptable vertical drift speed (m/s)

		// measurement source control
		(ParamExtInt<px4::params::EKF2_AID_MASK>)
		_param_ekf2_aid_mask,		///< bitmasked integer that selects which of the GPS and optical flow aiding sources will be used
		(ParamExtInt<px4::params::EKF2_HGT_MODE>) _param_ekf2_hgt_mode,	///< selects the primary source for height data
		(ParamExtInt<px4::params::EKF2_TERR_MASK>)
		_param_ekf2_terr_mask, ///< bitmasked integer that selects which of range finder and optical flow aiding sources will be used for terrain estimation
		(ParamExtInt<px4::params::EKF2_NOAID_TOUT>)
		_param_ekf2_noaid_tout,	///< maximum lapsed time from last fusion of measurements that constrain drift before the EKF will report the horizontal nav solution invalid (uSec)

		// range finder fusion
		(ParamExtFloat<px4::params::EKF2_RNG_NOISE>)
		_param_ekf2_rng_noise,	///< observation noise for range finder measurements (m)
		(ParamExtFloat<px4::params::EKF2_RNG_SFE>) _param_ekf2_rng_sfe, ///< scale factor from range to range noise (m/m)
		(ParamExtFloat<px4::params::EKF2_RNG_GATE>)
		_param_ekf2_rng_gate,	///< range finder fusion innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_MIN_RNG>) _param_ekf2_min_rng,	///< minimum valid value for range when on ground (m)
		(ParamExtFloat<px4::params::EKF2_RNG_PITCH>) _param_ekf2_rng_pitch,	///< range sensor pitch offset (rad)
		(ParamExtInt<px4::params::EKF2_RNG_AID>)
		_param_ekf2_rng_aid,		///< enables use of a range finder even if primary height source is not range finder
		(ParamExtFloat<px4::params::EKF2_RNG_A_VMAX>)
		_param_ekf2_rng_a_vmax,	///< maximum allowed horizontal velocity for range aid (m/s)
		(ParamExtFloat<px4::params::EKF2_RNG_A_HMAX>)
		_param_ekf2_rng_a_hmax,	///< maximum allowed absolute altitude (AGL) for range aid (m)
		(ParamExtFloat<px4::params::EKF2_RNG_A_IGATE>)
		_param_ekf2_rng_a_igate,	///< gate size used for innovation consistency checks for range aid fusion (STD)
		(ParamExtFloat<px4::params::EKF2_RNG_QLTY_T>)
		_param_ekf2_rng_qlty_t, ///< Minimum duration during which the reported range finder signal quality needs to be non-zero in order to be declared valid (s)

		// vision estimate fusion
		(ParamInt<px4::params::EKF2_EV_NOISE_MD>)
		_param_ekf2_ev_noise_md,	///< determine source of vision observation noise
		(ParamFloat<px4::params::EKF2_EVP_NOISE>)
		_param_ekf2_evp_noise,	///< default position observation noise for exernal vision measurements (m)
		(ParamFloat<px4::params::EKF2_EVV_NOISE>)
		_param_ekf2_evv_noise,	///< default velocity observation noise for exernal vision measurements (m/s)
		(ParamFloat<px4::params::EKF2_EVA_NOISE>)
		_param_ekf2_eva_noise,	///< default angular observation noise for exernal vision measurements (rad)
		(ParamExtFloat<px4::params::EKF2_EVV_GATE>)
		_param_ekf2_evv_gate,	///< external vision velocity innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_EVP_GATE>)
		_param_ekf2_evp_gate,	///< external vision position innovation consistency gate size (STD)

		// optical flow fusion
		(ParamExtFloat<px4::params::EKF2_OF_N_MIN>)
		_param_ekf2_of_n_min,	///< best quality observation noise for optical flow LOS rate measurements (rad/sec)
		(ParamExtFloat<px4::params::EKF2_OF_N_MAX>)
		_param_ekf2_of_n_max,	///< worst quality observation noise for optical flow LOS rate measurements (rad/sec)
		(ParamExtInt<px4::params::EKF2_OF_QMIN>)
		_param_ekf2_of_qmin,	///< minimum acceptable quality integer from  the flow sensor
		(ParamExtFloat<px4::params::EKF2_OF_GATE>)
		_param_ekf2_of_gate,	///< optical flow fusion innovation consistency gate size (STD)

		// sensor positions in body frame
		(ParamExtFloat<px4::params::EKF2_IMU_POS_X>) _param_ekf2_imu_pos_x,		///< X position of IMU in body frame (m)
		(ParamExtFloat<px4::params::EKF2_IMU_POS_Y>) _param_ekf2_imu_pos_y,		///< Y position of IMU in body frame (m)
		(ParamExtFloat<px4::params::EKF2_IMU_POS_Z>) _param_ekf2_imu_pos_z,		///< Z position of IMU in body frame (m)
		(ParamExtFloat<px4::params::EKF2_GPS_POS_X>) _param_ekf2_gps_pos_x,		///< X position of GPS antenna in body frame (m)
		(ParamExtFloat<px4::params::EKF2_GPS_POS_Y>) _param_ekf2_gps_pos_y,		///< Y position of GPS antenna in body frame (m)
		(ParamExtFloat<px4::params::EKF2_GPS_POS_Z>) _param_ekf2_gps_pos_z,		///< Z position of GPS antenna in body frame (m)
		(ParamExtFloat<px4::params::EKF2_RNG_POS_X>) _param_ekf2_rng_pos_x,		///< X position of range finder in body frame (m)
		(ParamExtFloat<px4::params::EKF2_RNG_POS_Y>) _param_ekf2_rng_pos_y,		///< Y position of range finder in body frame (m)
		(ParamExtFloat<px4::params::EKF2_RNG_POS_Z>) _param_ekf2_rng_pos_z,		///< Z position of range finder in body frame (m)
		(ParamExtFloat<px4::params::EKF2_OF_POS_X>)
		_param_ekf2_of_pos_x,	///< X position of optical flow sensor focal point in body frame (m)
		(ParamExtFloat<px4::params::EKF2_OF_POS_Y>)
		_param_ekf2_of_pos_y,	///< Y position of optical flow sensor focal point in body frame (m)
		(ParamExtFloat<px4::params::EKF2_OF_POS_Z>)
		_param_ekf2_of_pos_z,	///< Z position of optical flow sensor focal point in body frame (m)
		(ParamExtFloat<px4::params::EKF2_EV_POS_X>)
		_param_ekf2_ev_pos_x,		///< X position of VI sensor focal point in body frame (m)
		(ParamExtFloat<px4::params::EKF2_EV_POS_Y>)
		_param_ekf2_ev_pos_y,		///< Y position of VI sensor focal point in body frame (m)
		(ParamExtFloat<px4::params::EKF2_EV_POS_Z>)
		_param_ekf2_ev_pos_z,		///< Z position of VI sensor focal point in body frame (m)

		// control of airspeed and sideslip fusion
		(ParamExtFloat<px4::params::EKF2_ARSP_THR>)
		_param_ekf2_arsp_thr, 	///< A value of zero will disabled airspeed fusion. Any positive value sets the minimum airspeed which will be used (m/sec)
		(ParamInt<px4::params::EKF2_FUSE_BETA>)
		_param_ekf2_fuse_beta,		///< Controls synthetic sideslip fusion, 0 disables, 1 enables

		// output predictor filter time constants
		(ParamExtFloat<px4::params::EKF2_TAU_VEL>)
		_param_ekf2_tau_vel,		///< time constant used by the output velocity complementary filter (sec)
		(ParamExtFloat<px4::params::EKF2_TAU_POS>)
		_param_ekf2_tau_pos,		///< time constant used by the output position complementary filter (sec)

		// IMU switch on bias parameters
		(ParamExtFloat<px4::params::EKF2_GBIAS_INIT>)
		_param_ekf2_gbias_init,	///< 1-sigma gyro bias uncertainty at switch on (rad/sec)
		(ParamExtFloat<px4::params::EKF2_ABIAS_INIT>)
		_param_ekf2_abias_init,	///< 1-sigma accelerometer bias uncertainty at switch on (m/sec**2)
		(ParamExtFloat<px4::params::EKF2_ANGERR_INIT>)
		_param_ekf2_angerr_init,	///< 1-sigma tilt error after initial alignment using gravity vector (rad)

		// EKF accel bias learning control
		(ParamExtFloat<px4::params::EKF2_ABL_LIM>) _param_ekf2_abl_lim,	///< Accelerometer bias learning limit (m/s**2)
		(ParamExtFloat<px4::params::EKF2_ABL_ACCLIM>)
		_param_ekf2_abl_acclim,	///< Maximum IMU accel magnitude that allows IMU bias learning (m/s**2)
		(ParamExtFloat<px4::params::EKF2_ABL_GYRLIM>)
		_param_ekf2_abl_gyrlim,	///< Maximum IMU gyro angular rate magnitude that allows IMU bias learning (m/s**2)
		(ParamExtFloat<px4::params::EKF2_ABL_TAU>)
		_param_ekf2_abl_tau,	///< Time constant used to inhibit IMU delta velocity bias learning (sec)

		// Multi-rotor drag specific force fusion
		(ParamExtFloat<px4::params::EKF2_DRAG_NOISE>)
		_param_ekf2_drag_noise,	///< observation noise variance for drag specific force measurements (m/sec**2)**2
		(ParamExtFloat<px4::params::EKF2_BCOEF_X>) _param_ekf2_bcoef_x,		///< ballistic coefficient along the X-axis (kg/m**2)
		(ParamExtFloat<px4::params::EKF2_BCOEF_Y>) _param_ekf2_bcoef_y,		///< ballistic coefficient along the Y-axis (kg/m**2)
		(ParamExtFloat<px4::params::EKF2_MCOEF>) _param_ekf2_mcoef,		///< propeller momentum drag coefficient (1/s)

		// Corrections for static pressure position error where Ps_error = Ps_meas - Ps_truth
		// Coef = Ps_error / Pdynamic, where Pdynamic = 1/2 * density * TAS**2
		(ParamExtFloat<px4::params::EKF2_ASPD_MAX>)
		_param_ekf2_aspd_max,		///< upper limit on airspeed used for correction  (m/s**2)
		(ParamExtFloat<px4::params::EKF2_PCOEF_XP>)
		_param_ekf2_pcoef_xp,	///< static pressure position error coefficient along the positive X body axis
		(ParamExtFloat<px4::params::EKF2_PCOEF_XN>)
		_param_ekf2_pcoef_xn,	///< static pressure position error coefficient along the negative X body axis
		(ParamExtFloat<px4::params::EKF2_PCOEF_YP>)
		_param_ekf2_pcoef_yp,	///< static pressure position error coefficient along the positive Y body axis
		(ParamExtFloat<px4::params::EKF2_PCOEF_YN>)
		_param_ekf2_pcoef_yn,	///< static pressure position error coefficient along the negative Y body axis
		(ParamExtFloat<px4::params::EKF2_PCOEF_Z>)
		_param_ekf2_pcoef_z,	///< static pressure position error coefficient along the Z body axis

		// Test used to determine if the vehicle is static or moving
		(ParamExtFloat<px4::params::EKF2_MOVE_TEST>)
		_param_ekf2_move_test,	///< scaling applied to IMU data thresholds used to determine if the vehicle is static or moving.

		(ParamFloat<px4::params::EKF2_REQ_GPS_H>) _param_ekf2_req_gps_h, ///< Required GPS health time
		(ParamExtInt<px4::params::EKF2_MAG_CHECK>) _param_ekf2_mag_check, ///< Mag field strength check
		(ParamExtInt<px4::params::EKF2_SYNT_MAG_Z>)
		_param_ekf2_synthetic_mag_z, ///< Enables the use of a synthetic value for the Z axis of the magnetometer calculated from the 3D magnetic field vector at the location of the drone.

		// Used by EKF-GSF experimental yaw estimator
		(ParamExtFloat<px4::params::EKF2_GSF_TAS>)
		_param_ekf2_gsf_tas_default	///< default value of true airspeed assumed during fixed wing operation

	)
};
#endif // !EKF2_HPP
