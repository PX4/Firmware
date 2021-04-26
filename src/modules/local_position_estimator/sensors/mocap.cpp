#include "../BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>

extern orb_advert_t mavlink_log_pub;

// required number of samples for sensor
// to initialize
static const uint32_t		REQ_MOCAP_INIT_COUNT = 20;
static const uint32_t		MOCAP_TIMEOUT = 200000;	// 0.2 s

// set pose/velocity as invalid if standard deviation is bigger than EP_MAX_STD_DEV
// TODO: the user should be allowed to set these values by a parameter
static constexpr float 	EP_MAX_STD_DEV = 100.0f;

void BlockLocalPositionEstimator::mocapInit()
{
	// measure
	Vector<float, n_y_mocap> y;

	if (mocapMeasure(y) != OK) {
		_mocapStats.reset();
		return;
	}

	// if finished
	if (_mocapStats.getCount() > REQ_MOCAP_INIT_COUNT) {
		mavlink_log_info(&mavlink_log_pub, "[lpe] mocap position init: "
				 "%5.2f, %5.2f, %5.2f m std %5.2f, %5.2f, %5.2f m",
				 double(_mocapStats.getMean()(0)),
				 double(_mocapStats.getMean()(1)),
				 double(_mocapStats.getMean()(2)),
				 double(_mocapStats.getStdDev()(0)),
				 double(_mocapStats.getStdDev()(1)),
				 double(_mocapStats.getStdDev()(2)));
		_sensorTimeout &= ~SENSOR_MOCAP;
		_sensorFault &= ~SENSOR_MOCAP;

		// get reference for global position
		_ref_lat = math::degrees(_global_local_proj_ref.lat_rad);
		_ref_lon = math::degrees(_global_local_proj_ref.lon_rad);
		_ref_alt = _global_local_alt0;

		_is_global_cov_init = map_projection_initialized(&_global_local_proj_ref);

		if (!_map_ref.init_done && _is_global_cov_init && !_visionUpdated) {
			// initialize global origin using the mocap estimator reference (only if the vision estimation is not being fused as well)
			mavlink_log_info(&mavlink_log_pub, "[lpe] global origin init (mocap) : lat %6.2f lon %6.2f alt %5.1f m",
					 double(_ref_lat), double(_ref_lon), double(_ref_alt));
			map_projection_init(&_map_ref, _ref_lat, _ref_lon);
			// set timestamp when origin was set to current time
			_time_origin = _timeStamp;
		}

		if (!_altOriginInitialized) {
			_altOriginInitialized = true;
			_altOriginGlobal = true;
			_altOrigin = map_projection_initialized(&_global_local_proj_ref) ? _ref_alt : 0.0f;
		}
	}
}

int BlockLocalPositionEstimator::mocapMeasure(Vector<float, n_y_mocap> &y)
{
	uint8_t x_variance = _sub_mocap_odom.get().COVARIANCE_MATRIX_X_VARIANCE;
	uint8_t y_variance = _sub_mocap_odom.get().COVARIANCE_MATRIX_Y_VARIANCE;
	uint8_t z_variance = _sub_mocap_odom.get().COVARIANCE_MATRIX_Z_VARIANCE;

	if (PX4_ISFINITE(_sub_mocap_odom.get().pose_covariance[x_variance])) {
		// check if the mocap data is valid based on the covariances
		_mocap_eph = sqrtf(fmaxf(_sub_mocap_odom.get().pose_covariance[x_variance],
					 _sub_mocap_odom.get().pose_covariance[y_variance]));
		_mocap_epv = sqrtf(_sub_mocap_odom.get().pose_covariance[z_variance]);
		_mocap_xy_valid = _mocap_eph <= EP_MAX_STD_DEV;
		_mocap_z_valid = _mocap_epv <= EP_MAX_STD_DEV;

	} else {
		// if we don't have covariances, assume every reading
		_mocap_xy_valid = true;
		_mocap_z_valid = true;
	}

	if (!_mocap_xy_valid || !_mocap_z_valid) {
		_time_last_mocap = _sub_mocap_odom.get().timestamp_sample;
		return -1;

	} else {
		_time_last_mocap = _sub_mocap_odom.get().timestamp_sample;

		if (PX4_ISFINITE(_sub_mocap_odom.get().x)) {
			y.setZero();
			y(Y_mocap_x) = _sub_mocap_odom.get().x;
			y(Y_mocap_y) = _sub_mocap_odom.get().y;
			y(Y_mocap_z) = _sub_mocap_odom.get().z;
			_mocapStats.update(y);

			return OK;

		} else {
			return -1;
		}
	}
}

void BlockLocalPositionEstimator::mocapCorrect()
{
	// measure
	Vector<float, n_y_mocap> y;

	if (mocapMeasure(y) != OK) {
		mavlink_log_info(&mavlink_log_pub, "[lpe] mocap data invalid. eph: %f epv: %f", (double)_mocap_eph,
				 (double)_mocap_epv);
		return;
	}

	// mocap measurement matrix, measures position
	Matrix<float, n_y_mocap, n_x> C;
	C.setZero();
	C(Y_mocap_x, X_x) = 1;
	C(Y_mocap_y, X_y) = 1;
	C(Y_mocap_z, X_z) = 1;

	// noise matrix
	Matrix<float, n_y_mocap, n_y_mocap> R;
	R.setZero();

	// use std dev from mocap data if available
	if (_mocap_eph > _param_lpe_vic_p.get()) {
		R(Y_mocap_x, Y_mocap_x) = _mocap_eph * _mocap_eph;
		R(Y_mocap_y, Y_mocap_y) = _mocap_eph * _mocap_eph;

	} else {
		R(Y_mocap_x, Y_mocap_x) = _param_lpe_vic_p.get() * _param_lpe_vic_p.get();
		R(Y_mocap_y, Y_mocap_y) = _param_lpe_vic_p.get() * _param_lpe_vic_p.get();
	}

	if (_mocap_epv > _param_lpe_vic_p.get()) {
		R(Y_mocap_z, Y_mocap_z) = _mocap_epv * _mocap_epv;

	} else {
		R(Y_mocap_z, Y_mocap_z) = _param_lpe_vic_p.get() * _param_lpe_vic_p.get();
	}

	// residual
	Vector<float, n_y_mocap> r = y - C * _x;
	// residual covariance
	Matrix<float, n_y_mocap, n_y_mocap> S = C * m_P * C.transpose() + R;

	// publish innovations
	_pub_innov.get().ev_hpos[0] = r(0);
	_pub_innov.get().ev_hpos[1] = r(1);
	_pub_innov.get().ev_vpos    = r(2);
	_pub_innov.get().ev_hvel[0] = NAN;
	_pub_innov.get().ev_hvel[1] = NAN;
	_pub_innov.get().ev_vvel    = NAN;

	// publish innovation variances
	_pub_innov_var.get().ev_hpos[0] = S(0, 0);
	_pub_innov_var.get().ev_hpos[1] = S(1, 1);
	_pub_innov_var.get().ev_vpos    = S(2, 2);
	_pub_innov_var.get().ev_hvel[0] = NAN;
	_pub_innov_var.get().ev_hvel[1] = NAN;
	_pub_innov_var.get().ev_vvel    = NAN;

	// residual covariance, (inverse)
	Matrix<float, n_y_mocap, n_y_mocap> S_I = inv<float, n_y_mocap>(S);

	// fault detection
	float beta = (r.transpose() * (S_I * r))(0, 0);

	if (beta > BETA_TABLE[n_y_mocap]) {
		if (!(_sensorFault & SENSOR_MOCAP)) {
			//mavlink_log_info(&mavlink_log_pub, "[lpe] mocap fault, beta %5.2f", double(beta));
			_sensorFault |= SENSOR_MOCAP;
		}

	} else if (_sensorFault & SENSOR_MOCAP) {
		_sensorFault &= ~SENSOR_MOCAP;
		//mavlink_log_info(&mavlink_log_pub, "[lpe] mocap OK");
	}

	// kalman filter correction always
	Matrix<float, n_x, n_y_mocap> K = m_P * C.transpose() * S_I;
	Vector<float, n_x> dx = K * r;
	_x += dx;
	m_P -= K * C * m_P;
}

void BlockLocalPositionEstimator::mocapCheckTimeout()
{
	if (_timeStamp - _time_last_mocap > MOCAP_TIMEOUT) {
		if (!(_sensorTimeout & SENSOR_MOCAP)) {
			_sensorTimeout |= SENSOR_MOCAP;
			_mocapStats.reset();
			mavlink_log_info(&mavlink_log_pub, "[lpe] mocap timeout ");
		}
	}
}
