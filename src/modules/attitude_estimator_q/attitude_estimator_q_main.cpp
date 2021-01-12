/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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

/*
 * @file attitude_estimator_q_main.cpp
 *
 * Attitude estimator (quaternion based)
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <float.h>

#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <lib/ecl/geo_lookup/geo_mag_declination.h>
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <matrix/math.hpp>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <uORB/topics/vehicle_odometry.h>

using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;
using matrix::wrap_pi;

using namespace time_literals;

class AttitudeEstimatorQ : public ModuleBase<AttitudeEstimatorQ>, public ModuleParams, public px4::WorkItem
{
public:

	AttitudeEstimatorQ();
	~AttitudeEstimatorQ() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:

	void Run() override;

	void update_parameters(bool force = false);

	bool init_attq();

	bool update(float dt);

	// Update magnetic declination (in rads) immediately changing yaw rotation
	void update_mag_declination(float new_declination);


	const float _eo_max_std_dev = 100.0f;		/**< Maximum permissible standard deviation for estimated orientation */
	const float _dt_min = 0.00001f;
	const float _dt_max = 0.02f;

	uORB::SubscriptionCallbackWorkItem _sensors_sub{this, ORB_ID(sensor_combined)};

	uORB::SubscriptionInterval	_parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription		_gps_sub{ORB_ID(vehicle_gps_position)};
	uORB::Subscription		_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription		_vision_odom_sub{ORB_ID(vehicle_visual_odometry)};
	uORB::Subscription		_mocap_odom_sub{ORB_ID(vehicle_mocap_odometry)};
	uORB::Subscription		_magnetometer_sub{ORB_ID(vehicle_magnetometer)};

	uORB::Publication<vehicle_attitude_s>	_att_pub{ORB_ID(vehicle_attitude)};

	float		_mag_decl{0.0f};
	float		_bias_max{0.0f};

	Vector3f	_gyro;
	Vector3f	_accel;
	Vector3f	_mag;

	Vector3f	_vision_hdg;
	Vector3f	_mocap_hdg;

	Quatf		_q;
	Vector3f	_rates;
	Vector3f	_gyro_bias;

	Vector3f	_vel_prev;
	hrt_abstime	_vel_prev_t{0};

	Vector3f	_pos_acc;

	hrt_abstime	_last_time{0};

	bool		_inited{false};
	bool		_data_good{false};
	bool		_ext_hdg_good{false};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::ATT_W_ACC>) _param_att_w_acc,
		(ParamFloat<px4::params::ATT_W_MAG>) _param_att_w_mag,
		(ParamFloat<px4::params::ATT_W_EXT_HDG>) _param_att_w_ext_hdg,
		(ParamFloat<px4::params::ATT_W_GYRO_BIAS>) _param_att_w_gyro_bias,
		(ParamFloat<px4::params::ATT_MAG_DECL>) _param_att_mag_decl,
		(ParamInt<px4::params::ATT_MAG_DECL_A>) _param_att_mag_decl_a,
		(ParamInt<px4::params::ATT_EXT_HDG_M>) _param_att_ext_hdg_m,
		(ParamInt<px4::params::ATT_ACC_COMP>) _param_att_acc_comp,
		(ParamFloat<px4::params::ATT_BIAS_MAX>) _param_att_bias_mas,
		(ParamInt<px4::params::SYS_HAS_MAG>) _param_sys_has_mag
	)
};

AttitudeEstimatorQ::AttitudeEstimatorQ() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_vel_prev.zero();
	_pos_acc.zero();

	_gyro.zero();
	_accel.zero();
	_mag.zero();

	_vision_hdg.zero();
	_mocap_hdg.zero();

	_q.zero();
	_rates.zero();
	_gyro_bias.zero();

	update_parameters(true);
}

bool
AttitudeEstimatorQ::init()
{
	if (!_sensors_sub.registerCallback()) {
		PX4_ERR("sensor combined callback registration failed!");
		return false;
	}

	return true;
}

void
AttitudeEstimatorQ::Run()
{
	if (should_exit()) {
		_sensors_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	sensor_combined_s sensors;

	if (_sensors_sub.update(&sensors)) {

		update_parameters();

		// Feed validator with recent sensor data
		if (sensors.timestamp > 0) {
			_gyro(0) = sensors.gyro_rad[0];
			_gyro(1) = sensors.gyro_rad[1];
			_gyro(2) = sensors.gyro_rad[2];
		}

		if (sensors.accelerometer_timestamp_relative != sensor_combined_s::RELATIVE_TIMESTAMP_INVALID) {
			_accel(0) = sensors.accelerometer_m_s2[0];
			_accel(1) = sensors.accelerometer_m_s2[1];
			_accel(2) = sensors.accelerometer_m_s2[2];

			if (_accel.length() < 0.01f) {
				PX4_ERR("degenerate accel!");
				return;
			}
		}

		// Update magnetometer
		if (_magnetometer_sub.updated()) {
			vehicle_magnetometer_s magnetometer;

			if (_magnetometer_sub.copy(&magnetometer)) {
				_mag(0) = magnetometer.magnetometer_ga[0];
				_mag(1) = magnetometer.magnetometer_ga[1];
				_mag(2) = magnetometer.magnetometer_ga[2];

				if (_mag.length() < 0.01f) {
					PX4_ERR("degenerate mag!");
					return;
				}
			}

		}

		_data_good = true;

		// Update vision and motion capture heading
		_ext_hdg_good = false;

		if (_vision_odom_sub.updated()) {
			vehicle_odometry_s vision;

			if (_vision_odom_sub.copy(&vision)) {
				// validation check for vision attitude data
				bool vision_att_valid = PX4_ISFINITE(vision.q[0])
							&& (PX4_ISFINITE(vision.pose_covariance[vision.COVARIANCE_MATRIX_ROLL_VARIANCE]) ? sqrtf(fmaxf(
									vision.pose_covariance[vision.COVARIANCE_MATRIX_ROLL_VARIANCE],
									fmaxf(vision.pose_covariance[vision.COVARIANCE_MATRIX_PITCH_VARIANCE],
											vision.pose_covariance[vision.COVARIANCE_MATRIX_YAW_VARIANCE]))) <= _eo_max_std_dev : true);

				if (vision_att_valid) {
					Dcmf Rvis = Quatf(vision.q);
					Vector3f v(1.0f, 0.0f, 0.4f);

					// Rvis is Rwr (robot respect to world) while v is respect to world.
					// Hence Rvis must be transposed having (Rwr)' * Vw
					// Rrw * Vw = vn. This way we have consistency
					_vision_hdg = Rvis.transpose() * v;

					// vision external heading usage (ATT_EXT_HDG_M 1)
					if (_param_att_ext_hdg_m.get() == 1) {
						// Check for timeouts on data
						_ext_hdg_good = vision.timestamp_sample > 0 && (hrt_elapsed_time(&vision.timestamp_sample) < 500000);
					}
				}
			}
		}

		if (_mocap_odom_sub.updated()) {
			vehicle_odometry_s mocap;

			if (_mocap_odom_sub.copy(&mocap)) {
				// validation check for mocap attitude data
				bool mocap_att_valid = PX4_ISFINITE(mocap.q[0])
						       && (PX4_ISFINITE(mocap.pose_covariance[mocap.COVARIANCE_MATRIX_ROLL_VARIANCE]) ? sqrtf(fmaxf(
								       mocap.pose_covariance[mocap.COVARIANCE_MATRIX_ROLL_VARIANCE],
								       fmaxf(mocap.pose_covariance[mocap.COVARIANCE_MATRIX_PITCH_VARIANCE],
										       mocap.pose_covariance[mocap.COVARIANCE_MATRIX_YAW_VARIANCE]))) <= _eo_max_std_dev : true);

				if (mocap_att_valid) {
					Dcmf Rmoc = Quatf(mocap.q);
					Vector3f v(1.0f, 0.0f, 0.4f);

					// Rmoc is Rwr (robot respect to world) while v is respect to world.
					// Hence Rmoc must be transposed having (Rwr)' * Vw
					// Rrw * Vw = vn. This way we have consistency
					_mocap_hdg = Rmoc.transpose() * v;

					// Motion Capture external heading usage (ATT_EXT_HDG_M 2)
					if (_param_att_ext_hdg_m.get() == 2) {
						// Check for timeouts on data
						_ext_hdg_good = mocap.timestamp_sample > 0 && (hrt_elapsed_time(&mocap.timestamp_sample) < 500000);
					}
				}
			}
		}

		if (_gps_sub.updated()) {
			vehicle_gps_position_s gps;

			if (_gps_sub.copy(&gps)) {
				if (_param_att_mag_decl_a.get() && (gps.eph < 20.0f)) {
					/* set magnetic declination automatically */
					update_mag_declination(get_mag_declination_radians(gps.lat, gps.lon));
				}
			}
		}

		if (_local_position_sub.updated()) {
			vehicle_local_position_s lpos;

			if (_local_position_sub.copy(&lpos)) {

				if (_param_att_acc_comp.get() && (hrt_elapsed_time(&lpos.timestamp) < 20_ms)
				    && lpos.v_xy_valid && lpos.v_z_valid && (lpos.eph < 5.0f) && _inited) {

					/* position data is actual */
					const Vector3f vel(lpos.vx, lpos.vy, lpos.vz);

					/* velocity updated */
					if (_vel_prev_t != 0 && lpos.timestamp != _vel_prev_t) {
						float vel_dt = (lpos.timestamp - _vel_prev_t) / 1e6f;
						/* calculate acceleration in body frame */
						_pos_acc = _q.conjugate_inversed((vel - _vel_prev) / vel_dt);
					}

					_vel_prev_t = lpos.timestamp;
					_vel_prev = vel;

				} else {
					/* position data is outdated, reset acceleration */
					_pos_acc.zero();
					_vel_prev.zero();
					_vel_prev_t = 0;
				}
			}
		}

		/* time from previous iteration */
		hrt_abstime now = hrt_absolute_time();
		const float dt = math::constrain((now - _last_time) / 1e6f, _dt_min, _dt_max);
		_last_time = now;

		if (update(dt)) {
			vehicle_attitude_s att = {};
			att.timestamp_sample = sensors.timestamp;
			_q.copyTo(att.q);

			/* the instance count is not used here */
			att.timestamp = hrt_absolute_time();
			_att_pub.publish(att);

		}
	}
}

void
AttitudeEstimatorQ::update_parameters(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();

		// disable mag fusion if the system does not have a mag
		if (_param_sys_has_mag.get() == 0) {
			_param_att_w_mag.set(0.0f);
		}

		// if the weight is zero (=mag disabled), make sure the estimator initializes
		if (_param_att_w_mag.get() < FLT_EPSILON) {
			_mag(0) = 1.f;
			_mag(1) = 0.f;
			_mag(2) = 0.f;
		}

		update_mag_declination(math::radians(_param_att_mag_decl.get()));
	}
}

bool
AttitudeEstimatorQ::init_attq()
{
	// Rotation matrix can be easily constructed from acceleration and mag field vectors
	// 'k' is Earth Z axis (Down) unit vector in body frame
	Vector3f k = -_accel;
	k.normalize();

	// 'i' is Earth X axis (North) unit vector in body frame, orthogonal with 'k'
	Vector3f i = (_mag - k * (_mag * k));
	i.normalize();

	// 'j' is Earth Y axis (East) unit vector in body frame, orthogonal with 'k' and 'i'
	Vector3f j = k % i;

	// Fill rotation matrix
	Dcmf R;
	R.row(0) = i;
	R.row(1) = j;
	R.row(2) = k;

	// Convert to quaternion
	_q = R;

	// Compensate for magnetic declination
	Quatf decl_rotation = Eulerf(0.0f, 0.0f, _mag_decl);
	_q = _q * decl_rotation;

	_q.normalize();

	if (PX4_ISFINITE(_q(0)) && PX4_ISFINITE(_q(1)) &&
	    PX4_ISFINITE(_q(2)) && PX4_ISFINITE(_q(3)) &&
	    _q.length() > 0.95f && _q.length() < 1.05f) {
		_inited = true;

	} else {
		_inited = false;
	}

	return _inited;
}

bool
AttitudeEstimatorQ::update(float dt)
{
	if (!_inited) {

		if (!_data_good) {
			return false;
		}

		return init_attq();
	}

	Quatf q_last = _q;

	// Angular rate of correction
	Vector3f corr;
	float spinRate = _gyro.length();

	if (_param_att_ext_hdg_m.get() > 0 && _ext_hdg_good) {
		if (_param_att_ext_hdg_m.get() == 1) {
			// Vision heading correction
			// Project heading to global frame and extract XY component
			Vector3f vision_hdg_earth = _q.conjugate(_vision_hdg);
			float vision_hdg_err = wrap_pi(atan2f(vision_hdg_earth(1), vision_hdg_earth(0)));
			// Project correction to body frame
			corr += _q.conjugate_inversed(Vector3f(0.0f, 0.0f, -vision_hdg_err)) * _param_att_w_ext_hdg.get();
		}

		if (_param_att_ext_hdg_m.get() == 2) {
			// Mocap heading correction
			// Project heading to global frame and extract XY component
			Vector3f mocap_hdg_earth = _q.conjugate(_mocap_hdg);
			float mocap_hdg_err = wrap_pi(atan2f(mocap_hdg_earth(1), mocap_hdg_earth(0)));
			// Project correction to body frame
			corr += _q.conjugate_inversed(Vector3f(0.0f, 0.0f, -mocap_hdg_err)) * _param_att_w_ext_hdg.get();
		}
	}

	if (_param_att_ext_hdg_m.get() == 0 || !_ext_hdg_good) {
		// Magnetometer correction
		// Project mag field vector to global frame and extract XY component
		Vector3f mag_earth = _q.conjugate(_mag);
		float mag_err = wrap_pi(atan2f(mag_earth(1), mag_earth(0)) - _mag_decl);
		float gainMult = 1.0f;
		const float fifty_dps = 0.873f;

		if (spinRate > fifty_dps) {
			gainMult = math::min(spinRate / fifty_dps, 10.0f);
		}

		// Project magnetometer correction to body frame
		corr += _q.conjugate_inversed(Vector3f(0.0f, 0.0f, -mag_err)) * _param_att_w_mag.get() * gainMult;
	}

	_q.normalize();


	// Accelerometer correction
	// Project 'k' unit vector of earth frame to body frame
	// Vector3f k = _q.conjugate_inversed(Vector3f(0.0f, 0.0f, 1.0f));
	// Optimized version with dropped zeros
	Vector3f k(
		2.0f * (_q(1) * _q(3) - _q(0) * _q(2)),
		2.0f * (_q(2) * _q(3) + _q(0) * _q(1)),
		(_q(0) * _q(0) - _q(1) * _q(1) - _q(2) * _q(2) + _q(3) * _q(3))
	);

	// If we are not using acceleration compensation based on GPS velocity,
	// fuse accel data only if its norm is close to 1 g (reduces drift).
	const float accel_norm_sq = _accel.norm_squared();
	const float upper_accel_limit = CONSTANTS_ONE_G * 1.1f;
	const float lower_accel_limit = CONSTANTS_ONE_G * 0.9f;

	if (_param_att_acc_comp.get() || ((accel_norm_sq > lower_accel_limit * lower_accel_limit) &&
					  (accel_norm_sq < upper_accel_limit * upper_accel_limit))) {

		corr += (k % (_accel - _pos_acc).normalized()) * _param_att_w_acc.get();
	}

	// Gyro bias estimation
	if (spinRate < 0.175f) {
		_gyro_bias += corr * (_param_att_w_gyro_bias.get() * dt);

		for (int i = 0; i < 3; i++) {
			_gyro_bias(i) = math::constrain(_gyro_bias(i), -_bias_max, _bias_max);
		}

	}

	_rates = _gyro + _gyro_bias;

	// Feed forward gyro
	corr += _rates;

	// Apply correction to state
	_q += _q.derivative1(corr) * dt;

	// Normalize quaternion
	_q.normalize();

	if (!(PX4_ISFINITE(_q(0)) && PX4_ISFINITE(_q(1)) &&
	      PX4_ISFINITE(_q(2)) && PX4_ISFINITE(_q(3)))) {

		// Reset quaternion to last good state
		_q = q_last;
		_rates.zero();
		_gyro_bias.zero();
		return false;
	}

	return true;
}

void
AttitudeEstimatorQ::update_mag_declination(float new_declination)
{
	// Apply initial declination or trivial rotations without changing estimation
	if (!_inited || fabsf(new_declination - _mag_decl) < 0.0001f) {
		_mag_decl = new_declination;

	} else {
		// Immediately rotate current estimation to avoid gyro bias growth
		Quatf decl_rotation = Eulerf(0.0f, 0.0f, new_declination - _mag_decl);
		_q = _q * decl_rotation;
		_mag_decl = new_declination;
	}
}

int
AttitudeEstimatorQ::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int
AttitudeEstimatorQ::task_spawn(int argc, char *argv[])
{
	AttitudeEstimatorQ *instance = new AttitudeEstimatorQ();

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

int
AttitudeEstimatorQ::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Attitude estimator q.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("AttitudeEstimatorQ", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int attitude_estimator_q_main(int argc, char *argv[])
{
	return AttitudeEstimatorQ::main(argc, argv);
}
