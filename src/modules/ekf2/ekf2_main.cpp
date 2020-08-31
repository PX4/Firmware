/****************************************************************************
 *
 *   Copyright (c) 2015-2019 PX4 Development Team. All rights reserved.
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
 * @file ekf2_main.cpp
 * Implementation of the attitude and position estimator.
 *
 * @author Roman Bapst
 */

#include <float.h>

#include <drivers/drv_hrt.h>
#include <lib/ecl/EKF/ekf.h>
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
#include <uORB/topics/airspeed.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/ekf2_timestamps.h>
#include <uORB/topics/ekf_gps_drift.h>
#include <uORB/topics/ekf_gps_position.h>
#include <uORB/topics/estimator_innovations.h>
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/wind_estimate.h>
#include <uORB/topics/yaw_estimator_status.h>

#include "Utility/PreFlightChecker.hpp"

// defines used to specify the mask position for use of different accuracy metrics in the GPS blending algorithm
#define BLEND_MASK_USE_SPD_ACC      1
#define BLEND_MASK_USE_HPOS_ACC     2
#define BLEND_MASK_USE_VPOS_ACC     4

// define max number of GPS receivers supported and 0 base instance used to access virtual 'blended' GPS solution
#define GPS_MAX_RECEIVERS 2
#define GPS_BLENDED_INSTANCE 2

using math::constrain;
using namespace time_literals;

class Ekf2 final : public ModuleBase<Ekf2>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	explicit Ekf2(bool replay_mode = false);
	~Ekf2() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	int getRangeSubIndex(); ///< get subscription index of first downward-facing range sensor
	void fillGpsMsgWithVehicleGpsPosData(gps_message &msg, const vehicle_gps_position_s &data);

	PreFlightChecker _preflt_checker;
	void runPreFlightChecks(float dt, const filter_control_status_u &control_status,
				const vehicle_status_s &vehicle_status,
				const estimator_innovations_s &innov);
	void resetPreFlightChecks();

	template<typename Param>
	void update_mag_bias(Param &mag_bias_param, int axis_index);

	template<typename Param>
	bool update_mag_decl(Param &mag_decl_param);

	void publish_attitude(const hrt_abstime &timestamp);
	void publish_wind_estimate(const hrt_abstime &timestamp);
	void publish_yaw_estimator_status(const hrt_abstime &timestamp);

	/*
	 * Update the internal state estimate for a blended GPS solution that is a weighted average of the phsyical
	 * receiver solutions. This internal state cannot be used directly by estimators because if physical receivers
	 * have significant position differences, variation in receiver estimated accuracy will cause undesirable
	 * variation in the position solution.
	*/
	bool blend_gps_data();

	/*
	 * Calculate internal states used to blend GPS data from multiple receivers using weightings calculated
	 * by calc_blend_weights()
	 * States are written to _gps_state and _gps_blended_state class variables
	 */
	void update_gps_blend_states();

	/*
	 * The location in _gps_blended_state will move around as the relative accuracy changes.
	 * To mitigate this effect a low-pass filtered offset from each GPS location to the blended location is
	 * calculated.
	*/
	void update_gps_offsets();

	/*
	 * Apply the steady state physical receiver offsets calculated by update_gps_offsets().
	*/
	void apply_gps_offsets();

	/*
	 Calculate GPS output that is a blend of the offset corrected physical receiver data
	*/
	void calc_gps_blend_output();

	/*
	 * Calculate filtered WGS84 height from estimated AMSL height
	 */
	float filter_altitude_ellipsoid(float amsl_hgt);

	inline float sq(float x) { return x * x; };

	const bool 	_replay_mode;			///< true when we use replay data from a log

	// time slip monitoring
	uint64_t _integrated_time_us = 0;	///< integral of gyro delta time from start (uSec)
	uint64_t _start_time_us = 0;		///< system time at EKF start (uSec)
	int64_t _last_time_slip_us = 0;		///< Last time slip (uSec)

	perf_counter_t _ekf_update_perf;

	// Initialise time stamps used to send sensor data to the EKF and for logging
	uint8_t _invalid_mag_id_count = 0;	///< number of times an invalid magnetomer device ID has been detected

	// Used to check, save and use learned magnetometer biases
	hrt_abstime _last_magcal_us = 0;	///< last time the EKF was operating a mode that estimates magnetomer biases (uSec)
	hrt_abstime _total_cal_time_us = 0;	///< accumulated calibration time since the last save

	float _last_valid_mag_cal[3] = {};	///< last valid XYZ magnetometer bias estimates (mGauss)
	bool _valid_cal_available[3] = {};	///< true when an unsaved valid calibration for the XYZ magnetometer bias is available
	float _last_valid_variance[3] = {};	///< variances for the last valid magnetometer XYZ bias estimates (mGauss**2)

	// Used to control saving of mag declination to be used on next startup
	bool _mag_decl_saved = false;	///< true when the magnetic declination has been saved

	// set pose/velocity as invalid if standard deviation is bigger than max_std_dev
	// TODO: the user should be allowed to set these values by a parameter
	static constexpr float ep_max_std_dev = 100.0f;	///< Maximum permissible standard deviation for estimated position
	static constexpr float eo_max_std_dev = 100.0f;	///< Maximum permissible standard deviation for estimated orientation
	//static constexpr float ev_max_std_dev = 100.0f;	///< Maximum permissible standard deviation for estimated velocity

	// GPS blending and switching
	gps_message _gps_state[GPS_MAX_RECEIVERS] {}; ///< internal state data for the physical GPS
	gps_message _gps_blended_state{};		///< internal state data for the blended GPS
	gps_message _gps_output[GPS_MAX_RECEIVERS + 1] {}; ///< output state data for the physical and blended GPS
	Vector2f _NE_pos_offset_m[GPS_MAX_RECEIVERS] = {}; ///< Filtered North,East position offset from GPS instance to blended solution in _output_state.location (m)
	float _hgt_offset_mm[GPS_MAX_RECEIVERS] = {};	///< Filtered height offset from GPS instance relative to blended solution in _output_state.location (mm)
	Vector3f _blended_antenna_offset = {};		///< blended antenna offset
	float _blend_weights[GPS_MAX_RECEIVERS] = {};	///< blend weight for each GPS. The blend weights must sum to 1.0 across all instances.
	uint64_t _time_prev_us[GPS_MAX_RECEIVERS] = {};	///< the previous value of time_us for that GPS instance - used to detect new data.
	uint8_t _gps_best_index = 0;			///< index of the physical receiver with the lowest reported error
	uint8_t _gps_select_index = 0;			///< 0 = GPS1, 1 = GPS2, 2 = blended
	uint8_t _gps_time_ref_index =
		0;		///< index of the receiver that is used as the timing reference for the blending update
	uint8_t _gps_oldest_index = 0;			///< index of the physical receiver with the oldest data
	uint8_t _gps_newest_index = 0;			///< index of the physical receiver with the newest data
	uint8_t _gps_slowest_index = 0;			///< index of the physical receiver with the slowest update rate
	float _gps_dt[GPS_MAX_RECEIVERS] = {};		///< average time step in seconds.
	bool  _gps_new_output_data = false;		///< true if there is new output data for the EKF
	bool _had_valid_terrain = false;		///< true if at any time there was a valid terrain estimate

	int32_t _gps_alttitude_ellipsoid[GPS_MAX_RECEIVERS] {};	///< altitude in 1E-3 meters (millimeters) above ellipsoid
	uint64_t _gps_alttitude_ellipsoid_previous_timestamp[GPS_MAX_RECEIVERS] {}; ///< storage for previous timestamp to compute dt
	float   _wgs84_hgt_offset = 0;  ///< height offset between AMSL and WGS84

	bool _imu_bias_reset_request{false};

	// republished aligned external visual odometry
	bool new_ev_data_received = false;
	vehicle_odometry_s _ev_odom{};

	uORB::Subscription _airdata_sub{ORB_ID(vehicle_air_data)};
	uORB::Subscription _airspeed_sub{ORB_ID(airspeed)};
	uORB::Subscription _ev_odom_sub{ORB_ID(vehicle_visual_odometry)};
	uORB::Subscription _landing_target_pose_sub{ORB_ID(landing_target_pose)};
	uORB::Subscription _magnetometer_sub{ORB_ID(vehicle_magnetometer)};
	uORB::Subscription _optical_flow_sub{ORB_ID(optical_flow)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _sensor_selection_sub{ORB_ID(sensor_selection)};
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};

	uORB::SubscriptionCallbackWorkItem _sensor_combined_sub{this, ORB_ID(sensor_combined)};
	static constexpr int MAX_SENSOR_COUNT = 3;
	uORB::SubscriptionCallbackWorkItem _vehicle_imu_subs[MAX_SENSOR_COUNT] {
		{this, ORB_ID(vehicle_imu), 0},
		{this, ORB_ID(vehicle_imu), 1},
		{this, ORB_ID(vehicle_imu), 2}
	};
	int _imu_sub_index{-1};
	bool _callback_registered{false};
	int _lockstep_component{-1};

	// because we can have several distance sensor instances with different orientations
	static constexpr int MAX_RNG_SENSOR_COUNT = 4;
	uORB::Subscription _range_finder_subs[MAX_RNG_SENSOR_COUNT] {{ORB_ID(distance_sensor), 0}, {ORB_ID(distance_sensor), 1}, {ORB_ID(distance_sensor), 2}, {ORB_ID(distance_sensor), 3}};
	int _range_finder_sub_index = -1; // index for downward-facing range finder subscription

	// because we can have multiple GPS instances
	uORB::Subscription _gps_subs[GPS_MAX_RECEIVERS] {{ORB_ID(vehicle_gps_position), 0}, {ORB_ID(vehicle_gps_position), 1}};

	sensor_selection_s		_sensor_selection{};
	vehicle_land_detected_s		_vehicle_land_detected{};
	vehicle_status_s		_vehicle_status{};

	uORB::Publication<ekf2_timestamps_s>			_ekf2_timestamps_pub{ORB_ID(ekf2_timestamps)};
	uORB::Publication<ekf_gps_drift_s>			_ekf_gps_drift_pub{ORB_ID(ekf_gps_drift)};
	uORB::Publication<ekf_gps_position_s>			_blended_gps_pub{ORB_ID(ekf_gps_position)};
	uORB::Publication<estimator_innovations_s>		_estimator_innovation_test_ratios_pub{ORB_ID(estimator_innovation_test_ratios)};
	uORB::Publication<estimator_innovations_s>		_estimator_innovation_variances_pub{ORB_ID(estimator_innovation_variances)};
	uORB::Publication<estimator_innovations_s>		_estimator_innovations_pub{ORB_ID(estimator_innovations)};
	uORB::Publication<estimator_sensor_bias_s>		_estimator_sensor_bias_pub{ORB_ID(estimator_sensor_bias)};
	uORB::Publication<estimator_status_s>			_estimator_status_pub{ORB_ID(estimator_status)};
	uORB::Publication<vehicle_attitude_s>			_att_pub{ORB_ID(vehicle_attitude)};
	uORB::Publication<vehicle_odometry_s>			_vehicle_odometry_pub{ORB_ID(vehicle_odometry)};
	uORB::Publication<yaw_estimator_status_s>		_yaw_est_pub{ORB_ID(yaw_estimator_status)};
	uORB::PublicationData<vehicle_global_position_s>	_vehicle_global_position_pub{ORB_ID(vehicle_global_position)};
	uORB::PublicationData<vehicle_local_position_s>		_vehicle_local_position_pub{ORB_ID(vehicle_local_position)};
	uORB::PublicationData<vehicle_odometry_s>		_vehicle_visual_odometry_aligned_pub{ORB_ID(vehicle_visual_odometry_aligned)};
	uORB::PublicationMulti<wind_estimate_s>			_wind_pub{ORB_ID(wind_estimate)};

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

		(ParamInt<px4::params::EKF2_IMU_ID>) _param_ekf2_imu_id,

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
		(ParamFloat<px4::params::EKF2_ARSP_THR>)
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

		// EKF saved XYZ magnetometer bias values
		(ParamFloat<px4::params::EKF2_MAGBIAS_X>) _param_ekf2_magbias_x,		///< X magnetometer bias (mGauss)
		(ParamFloat<px4::params::EKF2_MAGBIAS_Y>) _param_ekf2_magbias_y,		///< Y magnetometer bias (mGauss)
		(ParamFloat<px4::params::EKF2_MAGBIAS_Z>) _param_ekf2_magbias_z,		///< Z magnetometer bias (mGauss)
		(ParamInt<px4::params::EKF2_MAGBIAS_ID>)
		_param_ekf2_magbias_id,		///< ID of the magnetometer sensor used to learn the bias values
		(ParamFloat<px4::params::EKF2_MAGB_VREF>)
		_param_ekf2_magb_vref, ///< Assumed error variance of previously saved magnetometer bias estimates (mGauss**2)
		(ParamFloat<px4::params::EKF2_MAGB_K>)
		_param_ekf2_magb_k,	///< maximum fraction of the learned magnetometer bias that is saved at each disarm

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

		// GPS blending
		(ParamInt<px4::params::EKF2_GPS_MASK>)
		_param_ekf2_gps_mask,	///< mask defining when GPS accuracy metrics are used to calculate the blend ratio
		(ParamFloat<px4::params::EKF2_GPS_TAU>)
		_param_ekf2_gps_tau,		///< time constant controlling how rapidly the offset used to bring GPS solutions together is allowed to change (sec)

		// Test used to determine if the vehicle is static or moving
		(ParamExtFloat<px4::params::EKF2_MOVE_TEST>)
		_param_ekf2_move_test,	///< scaling applied to IMU data thresholds used to determine if the vehicle is static or moving.

		(ParamFloat<px4::params::EKF2_REQ_GPS_H>) _param_ekf2_req_gps_h, ///< Required GPS health time
		(ParamExtInt<px4::params::EKF2_MAG_CHECK>) _param_ekf2_mag_check, ///< Mag field strength check

		// Used by EKF-GSF experimental yaw estimator
		(ParamExtFloat<px4::params::EKF2_GSF_TAS>)
		_param_ekf2_gsf_tas_default	///< default value of true airspeed assumed during fixed wing operation

	)

};

Ekf2::Ekf2(bool replay_mode):
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_replay_mode(replay_mode),
	_ekf_update_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": update")),
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
	_param_ekf2_aspd_max(_params->max_correction_airspeed),
	_param_ekf2_pcoef_xp(_params->static_pressure_coef_xp),
	_param_ekf2_pcoef_xn(_params->static_pressure_coef_xn),
	_param_ekf2_pcoef_yp(_params->static_pressure_coef_yp),
	_param_ekf2_pcoef_yn(_params->static_pressure_coef_yn),
	_param_ekf2_pcoef_z(_params->static_pressure_coef_z),
	_param_ekf2_move_test(_params->is_moving_scaler),
	_param_ekf2_mag_check(_params->check_mag_strength),
	_param_ekf2_gsf_tas_default(_params->EKFGSF_tas_default)
{
	// initialise parameter cache
	updateParams();

	_ekf.set_min_required_gps_health_time(_param_ekf2_req_gps_h.get() * 1_s);
}

Ekf2::~Ekf2()
{
	px4_lockstep_unregister_component(_lockstep_component);
	perf_free(_ekf_update_perf);
}

bool Ekf2::init()
{
	const uint32_t device_id = _param_ekf2_imu_id.get();

	// if EKF2_IMU_ID is non-zero we use the corresponding IMU, otherwise the voted primary (sensor_combined)
	if (device_id != 0) {
		for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
			vehicle_imu_s imu{};

			if (_vehicle_imu_subs[i].copy(&imu)) {
				if ((imu.accel_device_id > 0) && (imu.accel_device_id == device_id)) {
					if (_vehicle_imu_subs[i].registerCallback()) {
						PX4_INFO("subscribed to vehicle_imu:%d (%d)", i, device_id);
						_imu_sub_index = i;
						_callback_registered = true;
						return true;
					}
				}
			}
		}

	} else {
		_imu_sub_index = -1;

		if (_sensor_combined_sub.registerCallback()) {
			_callback_registered = true;
			return true;
		}
	}

	PX4_WARN("failed to register callback, retrying in 1 second");
	ScheduleDelayed(1_s); // retry in 1 second

	return true;
}

int Ekf2::print_status()
{
	PX4_INFO("local position: %s", (_ekf.local_position_is_valid()) ? "valid" : "invalid");
	PX4_INFO("global position: %s", (_ekf.global_position_is_valid()) ? "valid" : "invalid");

	PX4_INFO("time slip: %" PRId64 " us", _last_time_slip_us);

	perf_print_counter(_ekf_update_perf);

	return 0;
}

template<typename Param>
void Ekf2::update_mag_bias(Param &mag_bias_param, int axis_index)
{
	if (_valid_cal_available[axis_index]) {

		// calculate weighting using ratio of variances and update stored bias values
		const float weighting = constrain(_param_ekf2_magb_vref.get() / (_param_ekf2_magb_vref.get() +
						  _last_valid_variance[axis_index]), 0.0f, _param_ekf2_magb_k.get());
		const float mag_bias_saved = mag_bias_param.get();

		_last_valid_mag_cal[axis_index] = weighting * _last_valid_mag_cal[axis_index] + mag_bias_saved;

		mag_bias_param.set(_last_valid_mag_cal[axis_index]);
		mag_bias_param.commit_no_notification();

		_valid_cal_available[axis_index] = false;
	}
}

template<typename Param>
bool Ekf2::update_mag_decl(Param &mag_decl_param)
{
	// update stored declination value
	float declination_deg;

	if (_ekf.get_mag_decl_deg(&declination_deg)) {
		mag_decl_param.set(declination_deg);
		mag_decl_param.commit_no_notification();
		return true;
	}

	return false;
}

void Ekf2::Run()
{
	if (should_exit()) {
		_sensor_combined_sub.unregisterCallback();

		for (auto &i : _vehicle_imu_subs) {
			i.unregisterCallback();
		}

		exit_and_cleanup();
		return;
	}

	if (!_callback_registered) {
		init();
		return;
	}

	bool updated = false;
	imuSample imu_sample_new {};

	hrt_abstime imu_dt = 0; // for tracking time slip later
	estimator_sensor_bias_s bias{};

	if (_imu_sub_index >= 0) {
		vehicle_imu_s imu;
		updated = _vehicle_imu_subs[_imu_sub_index].update(&imu);

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

		bias.accel_device_id = imu.accel_device_id;
		bias.gyro_device_id = imu.gyro_device_id;

	} else {
		sensor_combined_s sensor_combined;
		updated = _sensor_combined_sub.update(&sensor_combined);

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
	}

	if (updated) {

		// check for parameter updates
		if (_parameter_update_sub.updated()) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			updateParams();
		}

		const hrt_abstime now = imu_sample_new.time_us;

		// ekf2_timestamps (using 0.1 ms relative timestamps)
		ekf2_timestamps_s ekf2_timestamps{};
		ekf2_timestamps.timestamp = now;

		ekf2_timestamps.airspeed_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
		ekf2_timestamps.distance_sensor_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
		ekf2_timestamps.optical_flow_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
		ekf2_timestamps.vehicle_air_data_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
		ekf2_timestamps.vehicle_magnetometer_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
		ekf2_timestamps.visual_odometry_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;

		// update all other topics if they have new data
		if (_status_sub.update(&_vehicle_status)) {

			const bool is_fixed_wing = (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING);

			// only fuse synthetic sideslip measurements if conditions are met
			_ekf.set_fuse_beta_flag(is_fixed_wing && (_param_ekf2_fuse_beta.get() == 1));

			// let the EKF know if the vehicle motion is that of a fixed wing (forward flight only relative to wind)
			_ekf.set_is_fixed_wing(is_fixed_wing);
		}

		// Always update sensor selction first time through if time stamp is non zero
		if (_sensor_selection_sub.updated() || (_sensor_selection.timestamp == 0)) {
			const sensor_selection_s sensor_selection_prev = _sensor_selection;

			if (_sensor_selection_sub.copy(&_sensor_selection)) {
				if ((sensor_selection_prev.timestamp > 0) && (_sensor_selection.timestamp > sensor_selection_prev.timestamp)) {

					if (_imu_sub_index < 0) {
						if (_sensor_selection.accel_device_id != sensor_selection_prev.accel_device_id) {
							_imu_bias_reset_request = true;
						}

						if (_sensor_selection.gyro_device_id != sensor_selection_prev.gyro_device_id) {
							_imu_bias_reset_request = true;
						}
					}
				}
			}
		}

		// attempt reset until successful
		if (_imu_bias_reset_request) {
			_imu_bias_reset_request = !_ekf.reset_imu_bias();
		}

		// push imu data into estimator
		_ekf.setIMUData(imu_sample_new);

		// publish attitude immediately (uses quaternion from output predictor)
		publish_attitude(now);

		// read mag data
		if (_magnetometer_sub.updated()) {
			vehicle_magnetometer_s magnetometer;

			if (_magnetometer_sub.copy(&magnetometer)) {
				// Reset learned bias parameters if there has been a persistant change in magnetometer ID
				// Do not reset parmameters when armed to prevent potential time slips casued by parameter set
				// and notification events
				// Check if there has been a persistant change in magnetometer ID
				if (magnetometer.device_id != 0
				    && (magnetometer.device_id != (uint32_t)_param_ekf2_magbias_id.get())) {

					if (_invalid_mag_id_count < 200) {
						_invalid_mag_id_count++;
					}

				} else {
					if (_invalid_mag_id_count > 0) {
						_invalid_mag_id_count--;
					}
				}

				if ((_vehicle_status.arming_state != vehicle_status_s::ARMING_STATE_ARMED) && (_invalid_mag_id_count > 100)) {
					// the sensor ID used for the last saved mag bias is not confirmed to be the same as the current sensor ID
					// this means we need to reset the learned bias values to zero
					_param_ekf2_magbias_x.set(0.f);
					_param_ekf2_magbias_x.commit_no_notification();
					_param_ekf2_magbias_y.set(0.f);
					_param_ekf2_magbias_y.commit_no_notification();
					_param_ekf2_magbias_z.set(0.f);
					_param_ekf2_magbias_z.commit_no_notification();
					_param_ekf2_magbias_id.set(magnetometer.device_id);
					_param_ekf2_magbias_id.commit();

					_invalid_mag_id_count = 0;

					PX4_INFO("Mag sensor ID changed to %i", _param_ekf2_magbias_id.get());
				}

				magSample mag_sample {};
				mag_sample.mag(0) = magnetometer.magnetometer_ga[0] - _param_ekf2_magbias_x.get();
				mag_sample.mag(1) = magnetometer.magnetometer_ga[1] - _param_ekf2_magbias_y.get();
				mag_sample.mag(2) = magnetometer.magnetometer_ga[2] - _param_ekf2_magbias_z.get();
				mag_sample.time_us = magnetometer.timestamp;

				_ekf.setMagData(mag_sample);
				ekf2_timestamps.vehicle_magnetometer_timestamp_rel = (int16_t)((int64_t)magnetometer.timestamp / 100 -
						(int64_t)ekf2_timestamps.timestamp / 100);
			}
		}

		// read baro data
		if (_airdata_sub.updated()) {
			vehicle_air_data_s airdata;

			if (_airdata_sub.copy(&airdata)) {
				_ekf.set_air_density(airdata.rho);
				const baroSample baro_sample {airdata.baro_alt_meter, airdata.timestamp_sample};
				_ekf.setBaroData(baro_sample);
				ekf2_timestamps.vehicle_air_data_timestamp_rel = (int16_t)((int64_t)airdata.timestamp / 100 -
						(int64_t)ekf2_timestamps.timestamp / 100);
			}
		}

		// read gps1 data if available
		bool gps1_updated = _gps_subs[0].updated();

		if (gps1_updated) {
			vehicle_gps_position_s gps;

			if (_gps_subs[0].copy(&gps)) {
				fillGpsMsgWithVehicleGpsPosData(_gps_state[0], gps);
				_gps_alttitude_ellipsoid[0] = gps.alt_ellipsoid;
			}
		}

		// check for second GPS receiver data
		bool gps2_updated = _gps_subs[1].updated();

		if (gps2_updated) {
			vehicle_gps_position_s gps;

			if (_gps_subs[1].copy(&gps)) {
				fillGpsMsgWithVehicleGpsPosData(_gps_state[1], gps);
				_gps_alttitude_ellipsoid[1] = gps.alt_ellipsoid;
			}
		}

		if ((_param_ekf2_gps_mask.get() == 0) && gps1_updated) {
			// When GPS blending is disabled we always use the first receiver instance
			_ekf.setGpsData(_gps_state[0]);

		} else if ((_param_ekf2_gps_mask.get() > 0) && (gps1_updated || gps2_updated)) {
			// blend dual receivers if available

			// calculate blending weights
			if (!blend_gps_data()) {
				// handle case where the blended states cannot be updated
				// Only use selected receiver data if it has been updated
				_gps_new_output_data = (gps1_updated && _gps_select_index == 0) ||
						       (gps2_updated && _gps_select_index == 1);

				// Reset relative position offsets to zero
				_NE_pos_offset_m[0].zero();
				_NE_pos_offset_m[1].zero();
				_hgt_offset_mm[0] = _hgt_offset_mm[1] = 0.0f;
			}

			if (_gps_new_output_data) {
				// correct the _gps_state data for steady state offsets and write to _gps_output
				apply_gps_offsets();

				// calculate a blended output from the offset corrected receiver data
				if (_gps_select_index == 2) {
					calc_gps_blend_output();
				}

				// write selected GPS to EKF
				_ekf.setGpsData(_gps_output[_gps_select_index]);

				// log blended solution as a third GPS instance
				ekf_gps_position_s gps;
				gps.timestamp = _gps_output[_gps_select_index].time_usec;
				gps.lat = _gps_output[_gps_select_index].lat;
				gps.lon = _gps_output[_gps_select_index].lon;
				gps.alt = _gps_output[_gps_select_index].alt;
				gps.fix_type = _gps_output[_gps_select_index].fix_type;
				gps.eph = _gps_output[_gps_select_index].eph;
				gps.epv = _gps_output[_gps_select_index].epv;
				gps.s_variance_m_s = _gps_output[_gps_select_index].sacc;
				gps.vel_m_s = _gps_output[_gps_select_index].vel_m_s;
				gps.vel_n_m_s = _gps_output[_gps_select_index].vel_ned(0);
				gps.vel_e_m_s = _gps_output[_gps_select_index].vel_ned(1);
				gps.vel_d_m_s = _gps_output[_gps_select_index].vel_ned(2);
				gps.vel_ned_valid = _gps_output[_gps_select_index].vel_ned_valid;
				gps.satellites_used = _gps_output[_gps_select_index].nsats;
				gps.heading = _gps_output[_gps_select_index].yaw;
				gps.heading_offset = _gps_output[_gps_select_index].yaw_offset;
				gps.selected = _gps_select_index;

				// Publish to the EKF blended GPS topic
				_blended_gps_pub.publish(gps);

				// clear flag to avoid re-use of the same data
				_gps_new_output_data = false;
			}
		}

		if (_airspeed_sub.updated()) {
			airspeed_s airspeed;

			if (_airspeed_sub.copy(&airspeed)) {
				// only set airspeed data if condition for airspeed fusion are met
				if ((_param_ekf2_arsp_thr.get() > FLT_EPSILON) && (airspeed.true_airspeed_m_s > _param_ekf2_arsp_thr.get())) {

					airspeedSample airspeed_sample {};
					airspeed_sample.time_us = airspeed.timestamp;
					airspeed_sample.eas2tas = airspeed.true_airspeed_m_s / airspeed.indicated_airspeed_m_s;
					airspeed_sample.true_airspeed = airspeed.true_airspeed_m_s;
					_ekf.setAirspeedData(airspeed_sample);
				}

				ekf2_timestamps.airspeed_timestamp_rel = (int16_t)((int64_t)airspeed.timestamp / 100 -
						(int64_t)ekf2_timestamps.timestamp / 100);
			}
		}

		if (_optical_flow_sub.updated()) {
			optical_flow_s optical_flow;

			if (_optical_flow_sub.copy(&optical_flow)) {
				flowSample flow {};
				// NOTE: the EKF uses the reverse sign convention to the flow sensor. EKF assumes positive LOS rate
				// is produced by a RH rotation of the image about the sensor axis.
				flow.flow_xy_rad(0) = -optical_flow.pixel_flow_x_integral;
				flow.flow_xy_rad(1) = -optical_flow.pixel_flow_y_integral;
				flow.gyro_xyz(0) = -optical_flow.gyro_x_rate_integral;
				flow.gyro_xyz(1) = -optical_flow.gyro_y_rate_integral;
				flow.gyro_xyz(2) = -optical_flow.gyro_z_rate_integral;
				flow.quality = optical_flow.quality;
				flow.dt = 1e-6f * (float)optical_flow.integration_timespan;
				flow.time_us = optical_flow.timestamp;

				if (PX4_ISFINITE(optical_flow.pixel_flow_y_integral) &&
				    PX4_ISFINITE(optical_flow.pixel_flow_x_integral) &&
				    flow.dt < 1) {

					_ekf.setOpticalFlowData(flow);
				}

				// Save sensor limits reported by the optical flow sensor
				_ekf.set_optical_flow_limits(optical_flow.max_flow_rate, optical_flow.min_ground_distance,
							     optical_flow.max_ground_distance);

				ekf2_timestamps.optical_flow_timestamp_rel = (int16_t)((int64_t)optical_flow.timestamp / 100 -
						(int64_t)ekf2_timestamps.timestamp / 100);
			}
		}

		if (_range_finder_sub_index >= 0) {

			if (_range_finder_subs[_range_finder_sub_index].updated()) {
				distance_sensor_s range_finder;

				if (_range_finder_subs[_range_finder_sub_index].copy(&range_finder)) {
					rangeSample range_sample {};
					range_sample.rng = range_finder.current_distance;
					range_sample.quality = range_finder.signal_quality;
					range_sample.time_us = range_finder.timestamp;
					_ekf.setRangeData(range_sample);

					// Save sensor limits reported by the rangefinder
					_ekf.set_rangefinder_limits(range_finder.min_distance, range_finder.max_distance);

					ekf2_timestamps.distance_sensor_timestamp_rel = (int16_t)((int64_t)range_finder.timestamp / 100 -
							(int64_t)ekf2_timestamps.timestamp / 100);
				}
			}

		} else {
			_range_finder_sub_index = getRangeSubIndex();
		}

		// get external vision data
		// if error estimates are unavailable, use parameter defined defaults
		new_ev_data_received = false;

		if (_ev_odom_sub.updated()) {
			new_ev_data_received = true;

			// copy both attitude & position, we need both to fill a single extVisionSample
			_ev_odom_sub.copy(&_ev_odom);

			extVisionSample ev_data {};

			// check for valid velocity data
			if (PX4_ISFINITE(_ev_odom.vx) && PX4_ISFINITE(_ev_odom.vy) && PX4_ISFINITE(_ev_odom.vz)) {
				ev_data.vel(0) = _ev_odom.vx;
				ev_data.vel(1) = _ev_odom.vy;
				ev_data.vel(2) = _ev_odom.vz;

				if (_ev_odom.velocity_frame == vehicle_odometry_s::BODY_FRAME_FRD) {
					ev_data.vel_frame = estimator::BODY_FRAME_FRD;

				} else {
					ev_data.vel_frame = estimator::LOCAL_FRAME_FRD;
				}

				// velocity measurement error from ev_data or parameters
				float param_evv_noise_var = sq(_param_ekf2_evv_noise.get());

				if (!_param_ekf2_ev_noise_md.get() && PX4_ISFINITE(_ev_odom.velocity_covariance[_ev_odom.COVARIANCE_MATRIX_VX_VARIANCE])
				    && PX4_ISFINITE(_ev_odom.velocity_covariance[_ev_odom.COVARIANCE_MATRIX_VY_VARIANCE])
				    && PX4_ISFINITE(_ev_odom.velocity_covariance[_ev_odom.COVARIANCE_MATRIX_VZ_VARIANCE])) {
					ev_data.velCov(0, 0) = _ev_odom.velocity_covariance[_ev_odom.COVARIANCE_MATRIX_VX_VARIANCE];
					ev_data.velCov(0, 1) = ev_data.velCov(1, 0) = _ev_odom.velocity_covariance[1];
					ev_data.velCov(0, 2) = ev_data.velCov(2, 0) = _ev_odom.velocity_covariance[2];
					ev_data.velCov(1, 1) = _ev_odom.velocity_covariance[_ev_odom.COVARIANCE_MATRIX_VY_VARIANCE];
					ev_data.velCov(1, 2) = ev_data.velCov(2, 1) = _ev_odom.velocity_covariance[7];
					ev_data.velCov(2, 2) = _ev_odom.velocity_covariance[_ev_odom.COVARIANCE_MATRIX_VZ_VARIANCE];

				} else {
					ev_data.velCov = matrix::eye<float, 3>() * param_evv_noise_var;
				}
			}

			// check for valid position data
			if (PX4_ISFINITE(_ev_odom.x) && PX4_ISFINITE(_ev_odom.y) && PX4_ISFINITE(_ev_odom.z)) {
				ev_data.pos(0) = _ev_odom.x;
				ev_data.pos(1) = _ev_odom.y;
				ev_data.pos(2) = _ev_odom.z;

				float param_evp_noise_var = sq(_param_ekf2_evp_noise.get());

				// position measurement error from ev_data or parameters
				if (!_param_ekf2_ev_noise_md.get() && PX4_ISFINITE(_ev_odom.pose_covariance[_ev_odom.COVARIANCE_MATRIX_X_VARIANCE])
				    && PX4_ISFINITE(_ev_odom.pose_covariance[_ev_odom.COVARIANCE_MATRIX_Y_VARIANCE])
				    && PX4_ISFINITE(_ev_odom.pose_covariance[_ev_odom.COVARIANCE_MATRIX_Z_VARIANCE])) {
					ev_data.posVar(0) = fmaxf(param_evp_noise_var, _ev_odom.pose_covariance[_ev_odom.COVARIANCE_MATRIX_X_VARIANCE]);
					ev_data.posVar(1) = fmaxf(param_evp_noise_var, _ev_odom.pose_covariance[_ev_odom.COVARIANCE_MATRIX_Y_VARIANCE]);
					ev_data.posVar(2) = fmaxf(param_evp_noise_var, _ev_odom.pose_covariance[_ev_odom.COVARIANCE_MATRIX_Z_VARIANCE]);

				} else {
					ev_data.posVar.setAll(param_evp_noise_var);
				}
			}

			// check for valid orientation data
			if (PX4_ISFINITE(_ev_odom.q[0])) {
				ev_data.quat = matrix::Quatf(_ev_odom.q);

				// orientation measurement error from ev_data or parameters
				float param_eva_noise_var = sq(_param_ekf2_eva_noise.get());

				if (!_param_ekf2_ev_noise_md.get() && PX4_ISFINITE(_ev_odom.pose_covariance[_ev_odom.COVARIANCE_MATRIX_YAW_VARIANCE])) {
					ev_data.angVar = fmaxf(param_eva_noise_var, _ev_odom.pose_covariance[_ev_odom.COVARIANCE_MATRIX_YAW_VARIANCE]);

				} else {
					ev_data.angVar = param_eva_noise_var;
				}
			}

			// use timestamp from external computer, clocks are synchronized when using MAVROS
			ev_data.time_us = _ev_odom.timestamp_sample;
			_ekf.setExtVisionData(ev_data);

			ekf2_timestamps.visual_odometry_timestamp_rel = (int16_t)((int64_t)_ev_odom.timestamp / 100 -
					(int64_t)ekf2_timestamps.timestamp / 100);
		}

		bool vehicle_land_detected_updated = _vehicle_land_detected_sub.updated();

		if (vehicle_land_detected_updated) {
			if (_vehicle_land_detected_sub.copy(&_vehicle_land_detected)) {
				_ekf.set_in_air_status(!_vehicle_land_detected.landed);
			}
		}

		// use the landing target pose estimate as another source of velocity data
		if (_landing_target_pose_sub.updated()) {
			landing_target_pose_s landing_target_pose;

			if (_landing_target_pose_sub.copy(&landing_target_pose)) {
				// we can only use the landing target if it has a fixed position and  a valid velocity estimate
				if (landing_target_pose.is_static && landing_target_pose.rel_vel_valid) {
					// velocity of vehicle relative to target has opposite sign to target relative to vehicle
					auxVelSample auxvel_sample {};
					auxvel_sample.vel = matrix::Vector3f{-landing_target_pose.vx_rel, -landing_target_pose.vy_rel, 0.0f};
					auxvel_sample.velVar = matrix::Vector3f{landing_target_pose.cov_vx_rel, landing_target_pose.cov_vy_rel, 0.0f};
					auxvel_sample.time_us = landing_target_pose.timestamp;
					_ekf.setAuxVelData(auxvel_sample);
				}
			}
		}

		// run the EKF update and output
		perf_begin(_ekf_update_perf);
		const bool ekf_updated = _ekf.update();
		perf_end(_ekf_update_perf);

		// integrate time to monitor time slippage
		if (_start_time_us == 0) {
			_start_time_us = now;
			_last_time_slip_us = 0;

		} else if (_start_time_us > 0) {
			_integrated_time_us += imu_dt;
			_last_time_slip_us = (now - _start_time_us) - _integrated_time_us;
		}

		if (ekf_updated) {

			filter_control_status_u control_status;
			_ekf.get_control_mode(&control_status.value);

			// only publish position after successful alignment
			if (control_status.flags.tilt_align) {
				// generate vehicle local position data
				vehicle_local_position_s &lpos = _vehicle_local_position_pub.get();

				// generate vehicle odometry data
				vehicle_odometry_s odom{};

				lpos.timestamp = now;

				odom.timestamp = hrt_absolute_time();
				odom.timestamp_sample = now;

				odom.local_frame = vehicle_odometry_s::LOCAL_FRAME_NED;

				// Position of body origin in local NED frame
				Vector3f position = _ekf.getPosition();
				const float lpos_x_prev = lpos.x;
				const float lpos_y_prev = lpos.y;
				lpos.x = (_ekf.local_position_is_valid()) ? position(0) : 0.0f;
				lpos.y = (_ekf.local_position_is_valid()) ? position(1) : 0.0f;
				lpos.z = position(2);

				// Vehicle odometry position
				odom.x = lpos.x;
				odom.y = lpos.y;
				odom.z = lpos.z;

				// Velocity of body origin in local NED frame (m/s)
				const Vector3f velocity = _ekf.getVelocity();
				lpos.vx = velocity(0);
				lpos.vy = velocity(1);
				lpos.vz = velocity(2);

				// Vehicle odometry linear velocity
				odom.velocity_frame = vehicle_odometry_s::LOCAL_FRAME_FRD;
				odom.vx = lpos.vx;
				odom.vy = lpos.vy;
				odom.vz = lpos.vz;

				// vertical position time derivative (m/s)
				lpos.z_deriv = _ekf.getVerticalPositionDerivative();

				// Acceleration of body origin in local frame
				Vector3f vel_deriv = _ekf.getVelocityDerivative();
				lpos.ax = vel_deriv(0);
				lpos.ay = vel_deriv(1);
				lpos.az = vel_deriv(2);

				// TODO: better status reporting
				lpos.xy_valid = _ekf.local_position_is_valid() && !_preflt_checker.hasHorizFailed();
				lpos.z_valid = !_preflt_checker.hasVertFailed();
				lpos.v_xy_valid = _ekf.local_position_is_valid() && !_preflt_checker.hasHorizFailed();
				lpos.v_z_valid = !_preflt_checker.hasVertFailed();

				// Position of local NED origin in GPS / WGS84 frame
				map_projection_reference_s ekf_origin;
				uint64_t origin_time;

				// true if position (x,y,z) has a valid WGS-84 global reference (ref_lat, ref_lon, alt)
				const bool ekf_origin_valid = _ekf.get_ekf_origin(&origin_time, &ekf_origin, &lpos.ref_alt);
				lpos.xy_global = ekf_origin_valid;
				lpos.z_global = ekf_origin_valid;

				if (ekf_origin_valid && (origin_time > lpos.ref_timestamp)) {
					lpos.ref_timestamp = origin_time;
					lpos.ref_lat = ekf_origin.lat_rad * 180.0 / M_PI; // Reference point latitude in degrees
					lpos.ref_lon = ekf_origin.lon_rad * 180.0 / M_PI; // Reference point longitude in degrees
				}

				// The rotation of the tangent plane vs. geographical north
				const matrix::Quatf q = _ekf.getQuaternion();

				matrix::Quatf delta_q_reset;
				_ekf.get_quat_reset(&delta_q_reset(0), &lpos.heading_reset_counter);

				lpos.heading = matrix::Eulerf(q).psi();
				lpos.delta_heading = matrix::Eulerf(delta_q_reset).psi();

				// Vehicle odometry quaternion
				q.copyTo(odom.q);

				// Vehicle odometry angular rates
				const Vector3f gyro_bias = _ekf.getGyroBias();
				const Vector3f rates(imu_sample_new.delta_ang * imu_sample_new.delta_ang_dt);
				odom.rollspeed = rates(0) - gyro_bias(0);
				odom.pitchspeed = rates(1) - gyro_bias(1);
				odom.yawspeed = rates(2) - gyro_bias(2);

				lpos.dist_bottom_valid = _ekf.get_terrain_valid();

				float terrain_vpos = _ekf.getTerrainVertPos();
				lpos.dist_bottom = terrain_vpos - lpos.z; // Distance to bottom surface (ground) in meters

				// constrain the distance to ground to _rng_gnd_clearance
				if (lpos.dist_bottom < _param_ekf2_min_rng.get()) {
					lpos.dist_bottom = _param_ekf2_min_rng.get();
				}

				if (!_had_valid_terrain) {
					_had_valid_terrain = lpos.dist_bottom_valid;
				}

				// only consider ground effect if compensation is configured and the vehicle is armed (props spinning)
				if (_param_ekf2_gnd_eff_dz.get() > 0.0f && (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED)) {
					// set ground effect flag if vehicle is closer than a specified distance to the ground
					if (lpos.dist_bottom_valid) {
						_ekf.set_gnd_effect_flag(lpos.dist_bottom < _param_ekf2_gnd_max_hgt.get());

						// if we have no valid terrain estimate and never had one then use ground effect flag from land detector
						// _had_valid_terrain is used to make sure that we don't fall back to using this option
						// if we temporarily lose terrain data due to the distance sensor getting out of range

					} else if (vehicle_land_detected_updated && !_had_valid_terrain) {
						// update ground effect flag based on land detector state
						_ekf.set_gnd_effect_flag(_vehicle_land_detected.in_ground_effect);
					}

				} else {
					_ekf.set_gnd_effect_flag(false);
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

				// Get covariances to vehicle odometry
				float covariances[24];
				_ekf.covariances_diagonal().copyTo(covariances);

				// get the covariance matrix size
				const size_t POS_URT_SIZE = sizeof(odom.pose_covariance) / sizeof(odom.pose_covariance[0]);
				const size_t VEL_URT_SIZE = sizeof(odom.velocity_covariance) / sizeof(odom.velocity_covariance[0]);

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

				// publish vehicle local position data
				_vehicle_local_position_pub.update();

				// publish vehicle odometry data
				_vehicle_odometry_pub.publish(odom);

				// publish external visual odometry after fixed frame alignment if new odometry is received
				if (new_ev_data_received) {
					const Quatf quat_ev2ekf = _ekf.getVisionAlignmentQuaternion(); // rotates from EV to EKF navigation frame
					const Dcmf ev_rot_mat(quat_ev2ekf);

					vehicle_odometry_s aligned_ev_odom = _ev_odom;

					// Rotate external position and velocity into EKF navigation frame
					const Vector3f aligned_pos = ev_rot_mat * Vector3f(_ev_odom.x, _ev_odom.y, _ev_odom.z);
					aligned_ev_odom.x = aligned_pos(0);
					aligned_ev_odom.y = aligned_pos(1);
					aligned_ev_odom.z = aligned_pos(2);

					switch (_ev_odom.velocity_frame) {
					case vehicle_odometry_s::BODY_FRAME_FRD: {
							const Vector3f aligned_vel = Dcmf(_ekf.getQuaternion()) *
										     Vector3f(_ev_odom.vx, _ev_odom.vy, _ev_odom.vz);
							aligned_ev_odom.vx = aligned_vel(0);
							aligned_ev_odom.vy = aligned_vel(1);
							aligned_ev_odom.vz = aligned_vel(2);
							break;
						}

					case vehicle_odometry_s::LOCAL_FRAME_FRD: {
							const Vector3f aligned_vel = ev_rot_mat *
										     Vector3f(_ev_odom.vx, _ev_odom.vy, _ev_odom.vz);
							aligned_ev_odom.vx = aligned_vel(0);
							aligned_ev_odom.vy = aligned_vel(1);
							aligned_ev_odom.vz = aligned_vel(2);
							break;
						}
					}

					aligned_ev_odom.velocity_frame = vehicle_odometry_s::LOCAL_FRAME_NED;

					// Compute orientation in EKF navigation frame
					Quatf ev_quat_aligned = quat_ev2ekf * matrix::Quatf(_ev_odom.q) ;
					ev_quat_aligned.normalize();

					ev_quat_aligned.copyTo(aligned_ev_odom.q);
					quat_ev2ekf.copyTo(aligned_ev_odom.q_offset);

					_vehicle_visual_odometry_aligned_pub.publish(aligned_ev_odom);
				}

				if (_ekf.global_position_is_valid() && !_preflt_checker.hasFailed()) {
					// generate and publish global position data
					vehicle_global_position_s &global_pos = _vehicle_global_position_pub.get();

					global_pos.timestamp = now;

					if (fabsf(lpos_x_prev - lpos.x) > FLT_EPSILON || fabsf(lpos_y_prev - lpos.y) > FLT_EPSILON) {
						map_projection_reproject(&ekf_origin, lpos.x, lpos.y, &global_pos.lat, &global_pos.lon);
					}

					global_pos.lat_lon_reset_counter = lpos.xy_reset_counter;

					global_pos.alt = -lpos.z + lpos.ref_alt; // Altitude AMSL in meters
					global_pos.alt_ellipsoid = filter_altitude_ellipsoid(global_pos.alt);

					// global altitude has opposite sign of local down position
					global_pos.delta_alt = -lpos.delta_z;

					_ekf.get_ekf_gpos_accuracy(&global_pos.eph, &global_pos.epv);

					global_pos.terrain_alt_valid = lpos.dist_bottom_valid;

					if (global_pos.terrain_alt_valid) {
						global_pos.terrain_alt = lpos.ref_alt - terrain_vpos; // Terrain altitude in m, WGS84

					} else {
						global_pos.terrain_alt = 0.0f; // Terrain altitude in m, WGS84
					}

					global_pos.dead_reckoning = _ekf.inertial_dead_reckoning(); // True if this position is estimated through dead-reckoning

					_vehicle_global_position_pub.update();
				}
			}

			{
				// publish all corrected sensor readings and bias estimates after mag calibration is updated above
				bias.timestamp = now;

				// take device ids from sensor_selection_s if not using specific vehicle_imu_s
				if (_imu_sub_index < 0) {
					bias.gyro_device_id = _sensor_selection.gyro_device_id;
					bias.accel_device_id = _sensor_selection.accel_device_id;
				}

				bias.mag_device_id = _param_ekf2_magbias_id.get();

				// In-run bias estimates
				_ekf.getGyroBias().copyTo(bias.gyro_bias);
				_ekf.getAccelBias().copyTo(bias.accel_bias);

				bias.mag_bias[0] = _last_valid_mag_cal[0];
				bias.mag_bias[1] = _last_valid_mag_cal[1];
				bias.mag_bias[2] = _last_valid_mag_cal[2];

				_estimator_sensor_bias_pub.publish(bias);
			}

			// publish estimator status
			estimator_status_s status;
			status.timestamp = now;
			_ekf.getStateAtFusionHorizonAsVector().copyTo(status.states);
			status.n_states = 24;
			_ekf.covariances_diagonal().copyTo(status.covariances);
			_ekf.getOutputTrackingError().copyTo(status.output_tracking_error);
			_ekf.get_gps_check_status(&status.gps_check_fail_flags);
			// only report enabled GPS check failures (the param indexes are shifted by 1 bit, because they don't include
			// the GPS Fix bit, which is always checked)
			status.gps_check_fail_flags &= ((uint16_t)_params->gps_check_mask << 1) | 1;
			status.control_mode_flags = control_status.value;
			_ekf.get_filter_fault_status(&status.filter_fault_flags);
			_ekf.get_innovation_test_status(status.innovation_check_flags, status.mag_test_ratio,
							status.vel_test_ratio, status.pos_test_ratio,
							status.hgt_test_ratio, status.tas_test_ratio,
							status.hagl_test_ratio, status.beta_test_ratio);

			status.pos_horiz_accuracy = _vehicle_local_position_pub.get().eph;
			status.pos_vert_accuracy = _vehicle_local_position_pub.get().epv;
			_ekf.get_ekf_soln_status(&status.solution_status_flags);
			_ekf.getImuVibrationMetrics().copyTo(status.vibe);
			status.time_slip = _last_time_slip_us * 1e-6f;
			status.health_flags = 0.0f; // unused
			status.timeout_flags = 0.0f; // unused
			status.pre_flt_fail_innov_heading = _preflt_checker.hasHeadingFailed();
			status.pre_flt_fail_innov_vel_horiz = _preflt_checker.hasHorizVelFailed();
			status.pre_flt_fail_innov_vel_vert = _preflt_checker.hasVertVelFailed();
			status.pre_flt_fail_innov_height = _preflt_checker.hasHeightFailed();
			status.pre_flt_fail_mag_field_disturbed = control_status.flags.mag_field_disturbed;

			_estimator_status_pub.publish(status);

			// publish GPS drift data only when updated to minimise overhead
			float gps_drift[3];
			bool blocked;

			if (_ekf.get_gps_drift_metrics(gps_drift, &blocked)) {
				ekf_gps_drift_s drift_data;
				drift_data.timestamp = now;
				drift_data.hpos_drift_rate = gps_drift[0];
				drift_data.vpos_drift_rate = gps_drift[1];
				drift_data.hspd = gps_drift[2];
				drift_data.blocked = blocked;

				_ekf_gps_drift_pub.publish(drift_data);
			}

			{
				/* Check and save learned magnetometer bias estimates */

				// Check if conditions are OK for learning of magnetometer bias values
				if (!_vehicle_land_detected.landed && // not on ground
				    (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) && // vehicle is armed
				    !status.filter_fault_flags && // there are no filter faults
				    control_status.flags.mag_3D) { // the EKF is operating in the correct mode

					if (_last_magcal_us == 0) {
						_last_magcal_us = now;

					} else {
						_total_cal_time_us += now - _last_magcal_us;
						_last_magcal_us = now;
					}

				} else if (status.filter_fault_flags != 0) {
					// if a filter fault has occurred, assume previous learning was invalid and do not
					// count it towards total learning time.
					_total_cal_time_us = 0;

					for (bool &cal_available : _valid_cal_available) {
						cal_available = false;
					}

				} else {
					// conditions are NOT OK for learning magnetometer bias, reset timestamp
					// but keep the accumulated calibration time
					_last_magcal_us = now;
				}

				// Start checking mag bias estimates when we have accumulated sufficient calibration time
				if (_total_cal_time_us > 120_s) {
					// we have sufficient accumulated valid flight time to form a reliable bias estimate
					// check that the state variance for each axis is within a range indicating filter convergence
					const float max_var_allowed = 100.0f * _param_ekf2_magb_vref.get();
					const float min_var_allowed = 0.01f * _param_ekf2_magb_vref.get();

					// Declare all bias estimates invalid if any variances are out of range
					bool all_estimates_invalid = false;

					for (uint8_t axis_index = 0; axis_index <= 2; axis_index++) {
						if (status.covariances[axis_index + 19] < min_var_allowed
						    || status.covariances[axis_index + 19] > max_var_allowed) {
							all_estimates_invalid = true;
						}
					}

					// Store valid estimates and their associated variances
					if (!all_estimates_invalid) {
						for (uint8_t axis_index = 0; axis_index <= 2; axis_index++) {
							_last_valid_mag_cal[axis_index] = status.states[axis_index + 19];
							_valid_cal_available[axis_index] = true;
							_last_valid_variance[axis_index] = status.covariances[axis_index + 19];
						}
					}
				}

				// Check and save the last valid calibration when we are disarmed
				if ((_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_STANDBY)
				    && (status.filter_fault_flags == 0)) {

					update_mag_bias(_param_ekf2_magbias_x, 0);
					update_mag_bias(_param_ekf2_magbias_y, 1);
					update_mag_bias(_param_ekf2_magbias_z, 2);

					// reset to prevent data being saved too frequently
					_total_cal_time_us = 0;
				}
			}

			publish_wind_estimate(now);

			publish_yaw_estimator_status(now);

			if (!_mag_decl_saved && (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_STANDBY)) {
				_mag_decl_saved = update_mag_decl(_param_ekf2_mag_decl);
			}

			{
				// publish estimator innovation data
				estimator_innovations_s innovations;
				innovations.timestamp = now;
				_ekf.getGpsVelPosInnov(&innovations.gps_hvel[0], innovations.gps_vvel, &innovations.gps_hpos[0],
						       innovations.gps_vpos);
				_ekf.getEvVelPosInnov(&innovations.ev_hvel[0], innovations.ev_vvel, &innovations.ev_hpos[0], innovations.ev_vpos);
				_ekf.getBaroHgtInnov(innovations.baro_vpos);
				_ekf.getRngHgtInnov(innovations.rng_vpos);
				_ekf.getAuxVelInnov(&innovations.aux_hvel[0]);
				_ekf.getFlowInnov(&innovations.flow[0]);
				_ekf.getHeadingInnov(innovations.heading);
				_ekf.getMagInnov(innovations.mag_field);
				_ekf.getDragInnov(&innovations.drag[0]);
				_ekf.getAirspeedInnov(innovations.airspeed);
				_ekf.getBetaInnov(innovations.beta);
				_ekf.getHaglInnov(innovations.hagl);
				// Not yet supported
				innovations.aux_vvel = NAN;
				innovations.fake_hpos[0] = innovations.fake_hpos[1] = innovations.fake_vpos = NAN;
				innovations.fake_hvel[0] = innovations.fake_hvel[1] = innovations.fake_vvel = NAN;

				// publish estimator innovation variance data
				estimator_innovations_s innovation_var;
				innovation_var.timestamp = now;
				_ekf.getGpsVelPosInnovVar(&innovation_var.gps_hvel[0], innovation_var.gps_vvel, &innovation_var.gps_hpos[0],
							  innovation_var.gps_vpos);
				_ekf.getEvVelPosInnovVar(&innovation_var.ev_hvel[0], innovation_var.ev_vvel, &innovation_var.ev_hpos[0],
							 innovation_var.ev_vpos);
				_ekf.getBaroHgtInnovVar(innovation_var.baro_vpos);
				_ekf.getRngHgtInnovVar(innovation_var.rng_vpos);
				_ekf.getAuxVelInnovVar(&innovation_var.aux_hvel[0]);
				_ekf.getFlowInnovVar(&innovation_var.flow[0]);
				_ekf.getHeadingInnovVar(innovation_var.heading);
				_ekf.getMagInnovVar(&innovation_var.mag_field[0]);
				_ekf.getDragInnovVar(&innovation_var.drag[0]);
				_ekf.getAirspeedInnovVar(innovation_var.airspeed);
				_ekf.getBetaInnovVar(innovation_var.beta);
				_ekf.getHaglInnovVar(innovation_var.hagl);
				// Not yet supported
				innovation_var.aux_vvel = NAN;
				innovation_var.fake_hpos[0] = innovation_var.fake_hpos[1] = innovation_var.fake_vpos = NAN;
				innovation_var.fake_hvel[0] = innovation_var.fake_hvel[1] = innovation_var.fake_vvel = NAN;


				// publish estimator innovation test ratio data
				estimator_innovations_s test_ratios;
				test_ratios.timestamp = now;
				_ekf.getGpsVelPosInnovRatio(test_ratios.gps_hvel[0], test_ratios.gps_vvel, test_ratios.gps_hpos[0],
							    test_ratios.gps_vpos);
				_ekf.getEvVelPosInnovRatio(test_ratios.ev_hvel[0], test_ratios.ev_vvel, test_ratios.ev_hpos[0],
							   test_ratios.ev_vpos);
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
				test_ratios.fake_hpos[0] = test_ratios.fake_hpos[1] = test_ratios.fake_vpos = NAN;
				test_ratios.fake_hvel[0] = test_ratios.fake_hvel[1] = test_ratios.fake_vvel = NAN;

				// calculate noise filtered velocity innovations which are used for pre-flight checking
				if (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_STANDBY) {
					float dt_seconds = imu_sample_new.delta_ang_dt;
					runPreFlightChecks(dt_seconds, control_status, _vehicle_status, innovations);

				} else {
					resetPreFlightChecks();
				}

				_estimator_innovations_pub.publish(innovations);
				_estimator_innovation_variances_pub.publish(innovation_var);
				_estimator_innovation_test_ratios_pub.publish(test_ratios);

			}
		}

		// publish ekf2_timestamps
		_ekf2_timestamps_pub.publish(ekf2_timestamps);

		if (_lockstep_component == -1) {
			_lockstep_component = px4_lockstep_register_component();
		}

		px4_lockstep_progress(_lockstep_component);
	}
}

void Ekf2::fillGpsMsgWithVehicleGpsPosData(gps_message &msg, const vehicle_gps_position_s &data)
{
	msg.time_usec = data.timestamp;
	msg.lat = data.lat;
	msg.lon = data.lon;
	msg.alt = data.alt;
	msg.yaw = data.heading;
	msg.yaw_offset = data.heading_offset;
	msg.fix_type = data.fix_type;
	msg.eph = data.eph;
	msg.epv = data.epv;
	msg.sacc = data.s_variance_m_s;
	msg.vel_m_s = data.vel_m_s;
	msg.vel_ned(0) = data.vel_n_m_s;
	msg.vel_ned(1) = data.vel_e_m_s;
	msg.vel_ned(2) = data.vel_d_m_s;
	msg.vel_ned_valid = data.vel_ned_valid;
	msg.nsats = data.satellites_used;
	msg.pdop = sqrtf(data.hdop * data.hdop + data.vdop * data.vdop);
}

void Ekf2::runPreFlightChecks(const float dt,
			      const filter_control_status_u &control_status,
			      const vehicle_status_s &vehicle_status,
			      const estimator_innovations_s &innov)
{
	const bool can_observe_heading_in_flight = (vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);

	_preflt_checker.setVehicleCanObserveHeadingInFlight(can_observe_heading_in_flight);
	_preflt_checker.setUsingGpsAiding(control_status.flags.gps);
	_preflt_checker.setUsingFlowAiding(control_status.flags.opt_flow);
	_preflt_checker.setUsingEvPosAiding(control_status.flags.ev_pos);
	_preflt_checker.setUsingEvVelAiding(control_status.flags.ev_vel);

	_preflt_checker.update(dt, innov);
}

void Ekf2::resetPreFlightChecks()
{
	_preflt_checker.reset();
}

int Ekf2::getRangeSubIndex()
{
	for (unsigned i = 0; i < MAX_RNG_SENSOR_COUNT; i++) {
		distance_sensor_s report{};

		if (_range_finder_subs[i].update(&report)) {
			// only use the first instace which has the correct orientation
			if (report.orientation == distance_sensor_s::ROTATION_DOWNWARD_FACING) {
				PX4_INFO("Found range finder with instance %d", i);
				return i;
			}
		}
	}

	return -1;
}

void Ekf2::publish_attitude(const hrt_abstime &timestamp)
{
	if (_ekf.attitude_valid()) {
		// generate vehicle attitude quaternion data
		vehicle_attitude_s att;
		att.timestamp = timestamp;

		const Quatf q{_ekf.calculate_quaternion()};
		q.copyTo(att.q);

		_ekf.get_quat_reset(&att.delta_q_reset[0], &att.quat_reset_counter);

		_att_pub.publish(att);

	}  else if (_replay_mode) {
		// in replay mode we have to tell the replay module not to wait for an update
		// we do this by publishing an attitude with zero timestamp
		vehicle_attitude_s att{};
		_att_pub.publish(att);
	}
}

void Ekf2::publish_yaw_estimator_status(const hrt_abstime &timestamp)
{
	yaw_estimator_status_s yaw_est_test_data{};

	static_assert(sizeof(yaw_estimator_status_s::yaw) / sizeof(float) == N_MODELS_EKFGSF,
		      "yaw_estimator_status_s::yaw wrong size");

	if (_ekf.getDataEKFGSF(&yaw_est_test_data.yaw_composite, &yaw_est_test_data.yaw_variance,
			       &yaw_est_test_data.yaw[0],
			       &yaw_est_test_data.innov_vn[0], &yaw_est_test_data.innov_ve[0],
			       &yaw_est_test_data.weight[0])) {

		yaw_est_test_data.timestamp = timestamp;

		_yaw_est_pub.publish(yaw_est_test_data);
	}
}

void Ekf2::publish_wind_estimate(const hrt_abstime &timestamp)
{
	if (_ekf.get_wind_status()) {
		// Publish wind estimate only if ekf declares them valid
		wind_estimate_s wind_estimate{};
		const Vector2f wind_vel = _ekf.getWindVelocity();
		const Vector2f wind_vel_var = _ekf.getWindVelocityVariance();
		_ekf.getAirspeedInnov(wind_estimate.tas_innov);
		_ekf.getAirspeedInnovVar(wind_estimate.tas_innov_var);
		_ekf.getBetaInnov(wind_estimate.beta_innov);
		_ekf.getBetaInnovVar(wind_estimate.beta_innov_var);
		wind_estimate.timestamp = timestamp;
		wind_estimate.windspeed_north = wind_vel(0);
		wind_estimate.windspeed_east = wind_vel(1);
		wind_estimate.variance_north = wind_vel_var(0);
		wind_estimate.variance_east = wind_vel_var(1);
		wind_estimate.tas_scale = 0.0f; //leave at 0 as scale is not estimated in ekf

		_wind_pub.publish(wind_estimate);
	}
}

bool Ekf2::blend_gps_data()
{
	// zero the blend weights
	memset(&_blend_weights, 0, sizeof(_blend_weights));

	/*
	 * If both receivers have the same update rate, use the oldest non-zero time.
	 * If two receivers with different update rates are used, use the slowest.
	 * If time difference is excessive, use newest to prevent a disconnected receiver
	 * from blocking updates.
	 */

	// Calculate the time step for each receiver with some filtering to reduce the effects of jitter
	// Find the largest and smallest time step.
	float dt_max = 0.0f;
	float dt_min = 0.3f;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		float raw_dt = 1e-6f * (float)(_gps_state[i].time_usec - _time_prev_us[i]);

		if (raw_dt > 0.0f && raw_dt < 0.3f) {
			_gps_dt[i] = 0.1f * raw_dt + 0.9f * _gps_dt[i];
		}

		if (_gps_dt[i] > dt_max) {
			dt_max = _gps_dt[i];
			_gps_slowest_index = i;
		}

		if (_gps_dt[i] < dt_min) {
			dt_min = _gps_dt[i];
		}
	}

	// Find the receiver that is last be updated
	uint64_t max_us = 0; // newest non-zero system time of arrival of a GPS message
	uint64_t min_us = -1; // oldest non-zero system time of arrival of a GPS message

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		// Find largest and smallest times
		if (_gps_state[i].time_usec > max_us) {
			max_us = _gps_state[i].time_usec;
			_gps_newest_index = i;
		}

		if ((_gps_state[i].time_usec < min_us) && (_gps_state[i].time_usec > 0)) {
			min_us = _gps_state[i].time_usec;
			_gps_oldest_index = i;
		}
	}

	if ((max_us - min_us) > 300000) {
		// A receiver has timed out so fall out of blending
		if (_gps_state[0].time_usec > _gps_state[1].time_usec) {
			_gps_select_index = 0;

		} else {
			_gps_select_index = 1;
		}

		return false;
	}

	// One receiver has lost 3D fix, fall out of blending
	if (_gps_state[0].fix_type > 2 && _gps_state[1].fix_type < 3) {
		_gps_select_index = 0;
		return false;

	} else if (_gps_state[1].fix_type > 2 && _gps_state[0].fix_type < 3) {
		_gps_select_index = 1;
		return false;
	}

	/*
	 * If the largest dt is less than 20% greater than the smallest, then we have  receivers
	 * running at the same rate then we wait until we have two messages with an arrival time
	 * difference that is less than 50% of the smallest time step and use the time stamp from
	 * the newest data.
	 * Else we have two receivers at different update rates and use the slowest receiver
	 * as the timing reference.
	 */

	if ((dt_max - dt_min) < 0.2f * dt_min) {
		// both receivers assumed to be running at the same rate
		if ((max_us - min_us) < (uint64_t)(5e5f * dt_min)) {
			// data arrival within a short time window enables the two measurements to be blended
			_gps_time_ref_index = _gps_newest_index;
			_gps_new_output_data = true;
		}

	} else {
		// both receivers running at different rates
		_gps_time_ref_index = _gps_slowest_index;

		if (_gps_state[_gps_time_ref_index].time_usec > _time_prev_us[_gps_time_ref_index]) {
			// blend data at the rate of the slower receiver
			_gps_new_output_data = true;
		}
	}

	if (_gps_new_output_data) {
		_gps_blended_state.time_usec = _gps_state[_gps_time_ref_index].time_usec;

		// calculate the sum squared speed accuracy across all GPS sensors
		float speed_accuracy_sum_sq = 0.0f;

		if (_param_ekf2_gps_mask.get() & BLEND_MASK_USE_SPD_ACC) {
			for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
				if (_gps_state[i].fix_type >= 3 && _gps_state[i].sacc > 0.0f) {
					speed_accuracy_sum_sq += _gps_state[i].sacc * _gps_state[i].sacc;

				} else {
					// not all receivers support this metric so set it to zero and don't use it
					speed_accuracy_sum_sq = 0.0f;
					break;
				}
			}
		}

		// calculate the sum squared horizontal position accuracy across all GPS sensors
		float horizontal_accuracy_sum_sq = 0.0f;

		if (_param_ekf2_gps_mask.get() & BLEND_MASK_USE_HPOS_ACC) {
			for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
				if (_gps_state[i].fix_type >= 2 && _gps_state[i].eph > 0.0f) {
					horizontal_accuracy_sum_sq += _gps_state[i].eph * _gps_state[i].eph;

				} else {
					// not all receivers support this metric so set it to zero and don't use it
					horizontal_accuracy_sum_sq = 0.0f;
					break;
				}
			}
		}

		// calculate the sum squared vertical position accuracy across all GPS sensors
		float vertical_accuracy_sum_sq = 0.0f;

		if (_param_ekf2_gps_mask.get() & BLEND_MASK_USE_VPOS_ACC) {
			for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
				if (_gps_state[i].fix_type >= 3 && _gps_state[i].epv > 0.0f) {
					vertical_accuracy_sum_sq += _gps_state[i].epv * _gps_state[i].epv;

				} else {
					// not all receivers support this metric so set it to zero and don't use it
					vertical_accuracy_sum_sq = 0.0f;
					break;
				}
			}
		}

		// Check if we can do blending using reported accuracy
		bool can_do_blending = (horizontal_accuracy_sum_sq > 0.0f || vertical_accuracy_sum_sq > 0.0f
					|| speed_accuracy_sum_sq > 0.0f);

		// if we can't do blending using reported accuracy, return false and hard switch logic will be used instead
		if (!can_do_blending) {
			return false;
		}

		float sum_of_all_weights = 0.0f;

		// calculate a weighting using the reported speed accuracy
		float spd_blend_weights[GPS_MAX_RECEIVERS] = {};

		if (speed_accuracy_sum_sq > 0.0f && (_param_ekf2_gps_mask.get() & BLEND_MASK_USE_SPD_ACC)) {
			// calculate the weights using the inverse of the variances
			float sum_of_spd_weights = 0.0f;

			for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
				if (_gps_state[i].fix_type >= 3 && _gps_state[i].sacc >= 0.001f) {
					spd_blend_weights[i] = 1.0f / (_gps_state[i].sacc * _gps_state[i].sacc);
					sum_of_spd_weights += spd_blend_weights[i];
				}
			}

			// normalise the weights
			if (sum_of_spd_weights > 0.0f) {
				for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
					spd_blend_weights[i] = spd_blend_weights[i] / sum_of_spd_weights;
				}

				sum_of_all_weights += 1.0f;
			}
		}

		// calculate a weighting using the reported horizontal position
		float hpos_blend_weights[GPS_MAX_RECEIVERS] = {};

		if (horizontal_accuracy_sum_sq > 0.0f && (_param_ekf2_gps_mask.get() & BLEND_MASK_USE_HPOS_ACC)) {
			// calculate the weights using the inverse of the variances
			float sum_of_hpos_weights = 0.0f;

			for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
				if (_gps_state[i].fix_type >= 2 && _gps_state[i].eph >= 0.001f) {
					hpos_blend_weights[i] = horizontal_accuracy_sum_sq / (_gps_state[i].eph * _gps_state[i].eph);
					sum_of_hpos_weights += hpos_blend_weights[i];
				}
			}

			// normalise the weights
			if (sum_of_hpos_weights > 0.0f) {
				for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
					hpos_blend_weights[i] = hpos_blend_weights[i] / sum_of_hpos_weights;
				}

				sum_of_all_weights += 1.0f;
			}
		}

		// calculate a weighting using the reported vertical position accuracy
		float vpos_blend_weights[GPS_MAX_RECEIVERS] = {};

		if (vertical_accuracy_sum_sq > 0.0f && (_param_ekf2_gps_mask.get() & BLEND_MASK_USE_VPOS_ACC)) {
			// calculate the weights using the inverse of the variances
			float sum_of_vpos_weights = 0.0f;

			for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
				if (_gps_state[i].fix_type >= 3 && _gps_state[i].epv >= 0.001f) {
					vpos_blend_weights[i] = vertical_accuracy_sum_sq / (_gps_state[i].epv * _gps_state[i].epv);
					sum_of_vpos_weights += vpos_blend_weights[i];
				}
			}

			// normalise the weights
			if (sum_of_vpos_weights > 0.0f) {
				for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
					vpos_blend_weights[i] = vpos_blend_weights[i] / sum_of_vpos_weights;
				}

				sum_of_all_weights += 1.0f;
			};
		}

		// calculate an overall weight
		for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
			_blend_weights[i] = (hpos_blend_weights[i] + vpos_blend_weights[i] + spd_blend_weights[i]) / sum_of_all_weights;
		}

		// With updated weights we can calculate a blended GPS solution and
		// offsets for each physical receiver
		update_gps_blend_states();
		update_gps_offsets();
		_gps_select_index = 2;

	}

	return true;
}

/*
 * Update the internal state estimate for a blended GPS solution that is a weighted average of the phsyical receiver solutions
 * with weights are calculated in calc_gps_blend_weights(). This internal state cannot be used directly by estimators
 * because if physical receivers have significant position differences,  variation in receiver estimated accuracy will
 * cause undesirable variation in the position solution.
*/
void Ekf2::update_gps_blend_states()
{
	// initialise the blended states so we can accumulate the results using the weightings for each GPS receiver.
	_gps_blended_state.time_usec = 0;
	_gps_blended_state.lat = 0;
	_gps_blended_state.lon = 0;
	_gps_blended_state.alt = 0;
	_gps_blended_state.fix_type = 0;
	_gps_blended_state.eph = FLT_MAX;
	_gps_blended_state.epv = FLT_MAX;
	_gps_blended_state.sacc = FLT_MAX;
	_gps_blended_state.vel_m_s = 0.0f;
	_gps_blended_state.vel_ned.setZero();
	_gps_blended_state.vel_ned_valid = true;
	_gps_blended_state.nsats = 0;
	_gps_blended_state.pdop = FLT_MAX;

	_blended_antenna_offset.zero();

	// combine the the GPS states into a blended solution using the weights calculated in calc_blend_weights()
	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		// blend the timing data
		_gps_blended_state.time_usec += (uint64_t)((double)_gps_state[i].time_usec * (double)_blend_weights[i]);

		// use the highest status
		if (_gps_state[i].fix_type > _gps_blended_state.fix_type) {
			_gps_blended_state.fix_type = _gps_state[i].fix_type;
		}

		// calculate a blended average speed and velocity vector
		_gps_blended_state.vel_m_s += _gps_state[i].vel_m_s * _blend_weights[i];
		_gps_blended_state.vel_ned += _gps_state[i].vel_ned * _blend_weights[i];

		// Assume blended error magnitude, DOP and sat count is equal to the best value from contributing receivers
		// If any receiver contributing has an invalid velocity, then report blended velocity as invalid
		if (_blend_weights[i] > 0.0f) {

			if (_gps_state[i].eph > 0.0f
			    && _gps_state[i].eph < _gps_blended_state.eph) {
				_gps_blended_state.eph = _gps_state[i].eph;
			}

			if (_gps_state[i].epv > 0.0f
			    && _gps_state[i].epv < _gps_blended_state.epv) {
				_gps_blended_state.epv = _gps_state[i].epv;
			}

			if (_gps_state[i].sacc > 0.0f
			    && _gps_state[i].sacc < _gps_blended_state.sacc) {
				_gps_blended_state.sacc = _gps_state[i].sacc;
			}

			if (_gps_state[i].pdop > 0
			    && _gps_state[i].pdop < _gps_blended_state.pdop) {
				_gps_blended_state.pdop = _gps_state[i].pdop;
			}

			if (_gps_state[i].nsats > 0
			    && _gps_state[i].nsats > _gps_blended_state.nsats) {
				_gps_blended_state.nsats = _gps_state[i].nsats;
			}

			if (!_gps_state[i].vel_ned_valid) {
				_gps_blended_state.vel_ned_valid = false;
			}

		}

		// TODO read parameters for individual GPS antenna positions and blend
		// Vector3f temp_antenna_offset = _antenna_offset[i];
		// temp_antenna_offset *= _blend_weights[i];
		// _blended_antenna_offset += temp_antenna_offset;

	}

	/*
	 * Calculate the instantaneous weighted average location using  available GPS instances and store in  _gps_state.
	 * This is statistically the most likely location, but may not be stable enough for direct use by the EKF.
	*/

	// Use the GPS with the highest weighting as the reference position
	float best_weight = 0.0f;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		if (_blend_weights[i] > best_weight) {
			best_weight = _blend_weights[i];
			_gps_best_index = i;
			_gps_blended_state.lat = _gps_state[i].lat;
			_gps_blended_state.lon = _gps_state[i].lon;
			_gps_blended_state.alt = _gps_state[i].alt;
		}
	}

	// Convert each GPS position to a local NEU offset relative to the reference position
	Vector2f blended_NE_offset_m;
	blended_NE_offset_m.zero();
	float blended_alt_offset_mm = 0.0f;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		if ((_blend_weights[i] > 0.0f) && (i != _gps_best_index)) {
			// calculate the horizontal offset
			Vector2f horiz_offset{};
			get_vector_to_next_waypoint((_gps_blended_state.lat / 1.0e7),
						    (_gps_blended_state.lon / 1.0e7), (_gps_state[i].lat / 1.0e7), (_gps_state[i].lon / 1.0e7),
						    &horiz_offset(0), &horiz_offset(1));

			// sum weighted offsets
			blended_NE_offset_m += horiz_offset * _blend_weights[i];

			// calculate vertical offset
			float vert_offset = (float)(_gps_state[i].alt - _gps_blended_state.alt);

			// sum weighted offsets
			blended_alt_offset_mm += vert_offset * _blend_weights[i];
		}
	}

	// Add the sum of weighted offsets to the reference position to obtain the blended position
	double lat_deg_now = (double)_gps_blended_state.lat * 1.0e-7;
	double lon_deg_now = (double)_gps_blended_state.lon * 1.0e-7;
	double lat_deg_res, lon_deg_res;
	add_vector_to_global_position(lat_deg_now, lon_deg_now, blended_NE_offset_m(0), blended_NE_offset_m(1), &lat_deg_res,
				      &lon_deg_res);
	_gps_blended_state.lat = (int32_t)(1.0E7 * lat_deg_res);
	_gps_blended_state.lon = (int32_t)(1.0E7 * lon_deg_res);
	_gps_blended_state.alt += (int32_t)blended_alt_offset_mm;

	// Take GPS heading from the highest weighted receiver that is publishing a valid .heading value
	uint8_t gps_best_yaw_index = 0;
	best_weight = 0.0f;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		if (PX4_ISFINITE(_gps_state[i].yaw) && (_blend_weights[i] > best_weight)) {
			best_weight = _blend_weights[i];
			gps_best_yaw_index = i;
		}
	}

	_gps_blended_state.yaw = _gps_state[gps_best_yaw_index].yaw;
	_gps_blended_state.yaw_offset = _gps_state[gps_best_yaw_index].yaw_offset;
}

/*
 * The location in _gps_blended_state will move around as the relative accuracy changes.
 * To mitigate this effect a low-pass filtered offset from each GPS location to the blended location is
 * calculated.
*/
void Ekf2::update_gps_offsets()
{

	// Calculate filter coefficients to be applied to the offsets for each GPS position and height offset
	// A weighting of 1 will make the offset adjust the slowest, a weighting of 0 will make it adjust with zero filtering
	float alpha[GPS_MAX_RECEIVERS] = {};
	float omega_lpf = 1.0f / fmaxf(_param_ekf2_gps_tau.get(), 1.0f);

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		if (_gps_state[i].time_usec - _time_prev_us[i] > 0) {
			// calculate the filter coefficient that achieves the time constant specified by the user adjustable parameter
			alpha[i] = constrain(omega_lpf * 1e-6f * (float)(_gps_state[i].time_usec - _time_prev_us[i]),
					     0.0f, 1.0f);

			_time_prev_us[i] = _gps_state[i].time_usec;
		}
	}

	// Calculate a filtered position delta for each GPS relative to the blended solution state
	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		Vector2f offset;
		get_vector_to_next_waypoint((_gps_state[i].lat / 1.0e7), (_gps_state[i].lon / 1.0e7),
					    (_gps_blended_state.lat / 1.0e7), (_gps_blended_state.lon / 1.0e7), &offset(0), &offset(1));
		_NE_pos_offset_m[i] = offset * alpha[i] + _NE_pos_offset_m[i] * (1.0f - alpha[i]);
		_hgt_offset_mm[i] = (float)(_gps_blended_state.alt - _gps_state[i].alt) *  alpha[i] +
				    _hgt_offset_mm[i] * (1.0f - alpha[i]);
	}

	// calculate offset limits from the largest difference between receivers
	Vector2f max_ne_offset{};
	float max_alt_offset = 0;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		for (uint8_t j = i; j < GPS_MAX_RECEIVERS; j++) {
			if (i != j) {
				Vector2f offset;
				get_vector_to_next_waypoint((_gps_state[i].lat / 1.0e7), (_gps_state[i].lon / 1.0e7),
							    (_gps_state[j].lat / 1.0e7), (_gps_state[j].lon / 1.0e7), &offset(0), &offset(1));
				max_ne_offset(0) = fmaxf(max_ne_offset(0), fabsf(offset(0)));
				max_ne_offset(1) = fmaxf(max_ne_offset(1), fabsf(offset(1)));
				max_alt_offset = fmaxf(max_alt_offset, fabsf((float)(_gps_state[i].alt - _gps_state[j].alt)));
			}
		}
	}

	// apply offset limits
	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		_NE_pos_offset_m[i](0) = constrain(_NE_pos_offset_m[i](0), -max_ne_offset(0), max_ne_offset(0));
		_NE_pos_offset_m[i](1) = constrain(_NE_pos_offset_m[i](1), -max_ne_offset(1), max_ne_offset(1));
		_hgt_offset_mm[i] = constrain(_hgt_offset_mm[i], -max_alt_offset, max_alt_offset);
	}

}


/*
 * Apply the steady state physical receiver offsets calculated by update_gps_offsets().
*/
void Ekf2::apply_gps_offsets()
{
	// calculate offset corrected output for each physical GPS.
	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		// Add the sum of weighted offsets to the reference position to obtain the blended position
		double lat_deg_now = (double)_gps_state[i].lat * 1.0e-7;
		double lon_deg_now = (double)_gps_state[i].lon * 1.0e-7;
		double lat_deg_res, lon_deg_res;
		add_vector_to_global_position(lat_deg_now, lon_deg_now, _NE_pos_offset_m[i](0), _NE_pos_offset_m[i](1), &lat_deg_res,
					      &lon_deg_res);
		_gps_output[i].lat = (int32_t)(1.0E7 * lat_deg_res);
		_gps_output[i].lon = (int32_t)(1.0E7 * lon_deg_res);
		_gps_output[i].alt = _gps_state[i].alt + (int32_t)_hgt_offset_mm[i];

		// other receiver data is used uncorrected
		_gps_output[i].time_usec	= _gps_state[i].time_usec;
		_gps_output[i].fix_type		= _gps_state[i].fix_type;
		_gps_output[i].vel_m_s		= _gps_state[i].vel_m_s;
		_gps_output[i].vel_ned		= _gps_state[i].vel_ned;
		_gps_output[i].eph		= _gps_state[i].eph;
		_gps_output[i].epv		= _gps_state[i].epv;
		_gps_output[i].sacc		= _gps_state[i].sacc;
		_gps_output[i].pdop		= _gps_state[i].pdop;
		_gps_output[i].nsats		= _gps_state[i].nsats;
		_gps_output[i].vel_ned_valid	= _gps_state[i].vel_ned_valid;
		_gps_output[i].yaw		= _gps_state[i].yaw;
		_gps_output[i].yaw_offset	= _gps_state[i].yaw_offset;

	}
}

/*
 Calculate GPS output that is a blend of the offset corrected physical receiver data
*/
void Ekf2::calc_gps_blend_output()
{
	// Convert each GPS position to a local NEU offset relative to the reference position
	// which is defined as the positon of the blended solution calculated from non offset corrected data
	Vector2f blended_NE_offset_m;
	blended_NE_offset_m.zero();
	float blended_alt_offset_mm = 0.0f;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		if (_blend_weights[i] > 0.0f) {
			// calculate the horizontal offset
			Vector2f horiz_offset{};
			get_vector_to_next_waypoint((_gps_blended_state.lat / 1.0e7),
						    (_gps_blended_state.lon / 1.0e7),
						    (_gps_output[i].lat / 1.0e7),
						    (_gps_output[i].lon / 1.0e7),
						    &horiz_offset(0),
						    &horiz_offset(1));

			// sum weighted offsets
			blended_NE_offset_m += horiz_offset * _blend_weights[i];

			// calculate vertical offset
			float vert_offset = (float)(_gps_output[i].alt - _gps_blended_state.alt);

			// sum weighted offsets
			blended_alt_offset_mm += vert_offset * _blend_weights[i];
		}
	}

	// Add the sum of weighted offsets to the reference position to obtain the blended position
	double lat_deg_now = (double)_gps_blended_state.lat * 1.0e-7;
	double lon_deg_now = (double)_gps_blended_state.lon * 1.0e-7;
	double lat_deg_res, lon_deg_res;
	add_vector_to_global_position(lat_deg_now, lon_deg_now, blended_NE_offset_m(0), blended_NE_offset_m(1), &lat_deg_res,
				      &lon_deg_res);
	_gps_output[GPS_BLENDED_INSTANCE].lat = (int32_t)(1.0E7 * lat_deg_res);
	_gps_output[GPS_BLENDED_INSTANCE].lon = (int32_t)(1.0E7 * lon_deg_res);
	_gps_output[GPS_BLENDED_INSTANCE].alt = _gps_blended_state.alt + (int32_t)blended_alt_offset_mm;

	// Copy remaining data from internal states to output
	_gps_output[GPS_BLENDED_INSTANCE].time_usec	= _gps_blended_state.time_usec;
	_gps_output[GPS_BLENDED_INSTANCE].fix_type	= _gps_blended_state.fix_type;
	_gps_output[GPS_BLENDED_INSTANCE].vel_m_s	= _gps_blended_state.vel_m_s;
	_gps_output[GPS_BLENDED_INSTANCE].vel_ned	= _gps_blended_state.vel_ned;
	_gps_output[GPS_BLENDED_INSTANCE].eph		= _gps_blended_state.eph;
	_gps_output[GPS_BLENDED_INSTANCE].epv		= _gps_blended_state.epv;
	_gps_output[GPS_BLENDED_INSTANCE].sacc		= _gps_blended_state.sacc;
	_gps_output[GPS_BLENDED_INSTANCE].pdop		= _gps_blended_state.pdop;
	_gps_output[GPS_BLENDED_INSTANCE].nsats		= _gps_blended_state.nsats;
	_gps_output[GPS_BLENDED_INSTANCE].vel_ned_valid	= _gps_blended_state.vel_ned_valid;
	_gps_output[GPS_BLENDED_INSTANCE].yaw		= _gps_blended_state.yaw;
	_gps_output[GPS_BLENDED_INSTANCE].yaw_offset	= _gps_blended_state.yaw_offset;

}

float Ekf2::filter_altitude_ellipsoid(float amsl_hgt)
{

	float height_diff = static_cast<float>(_gps_alttitude_ellipsoid[0]) * 1e-3f - amsl_hgt;

	if (_gps_alttitude_ellipsoid_previous_timestamp[0] == 0) {

		_wgs84_hgt_offset = height_diff;
		_gps_alttitude_ellipsoid_previous_timestamp[0] = _gps_state[0].time_usec;

	} else if (_gps_state[0].time_usec != _gps_alttitude_ellipsoid_previous_timestamp[0]) {

		// apply a 10 second first order low pass filter to baro offset
		float dt = 1e-6f * static_cast<float>(_gps_state[0].time_usec - _gps_alttitude_ellipsoid_previous_timestamp[0]);
		_gps_alttitude_ellipsoid_previous_timestamp[0] = _gps_state[0].time_usec;
		float offset_rate_correction = 0.1f * (height_diff - _wgs84_hgt_offset);
		_wgs84_hgt_offset += dt * math::constrain(offset_rate_correction, -0.1f, 0.1f);
	}

	return amsl_hgt + _wgs84_hgt_offset;
}

int Ekf2::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Ekf2::task_spawn(int argc, char *argv[])
{
	bool replay_mode = false;

	if (argc > 1 && !strcmp(argv[1], "-r")) {
		PX4_INFO("replay mode enabled");
		replay_mode = true;
	}

	Ekf2 *instance = new Ekf2(replay_mode);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int Ekf2::print_usage(const char *reason)
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

	return 0;
}

extern "C" __EXPORT int ekf2_main(int argc, char *argv[])
{
	return Ekf2::main(argc, argv);
}
