#include "../BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>

extern orb_advert_t mavlink_log_pub;

// required number of samples for sensor
// to initialize
//
static const uint32_t		REQ_LIDAR_INIT_COUNT = 10;
static const uint32_t		LIDAR_TIMEOUT = 1000000;	// 1.0 s

void BlockLocalPositionEstimator::lidarInit()
{
	// measure
	Vector<float, n_y_lidar> y;

	if (lidarMeasure(y) != OK) {
		_lidarStats.reset();
	}

	// if finished
	if (_lidarStats.getCount() > REQ_LIDAR_INIT_COUNT) {
		mavlink_log_info(&mavlink_log_pub, "[lpe] lidar init: "
				 "mean %d cm stddev %d cm",
				 int(100 * _lidarStats.getMean()(0)),
				 int(100 * _lidarStats.getStdDev()(0)));
		_sensorTimeout &= ~SENSOR_LIDAR;
		_sensorFault &= ~SENSOR_LIDAR;
	}
}

int BlockLocalPositionEstimator::lidarMeasure(Vector<float, n_y_lidar> &y)
{
	// measure
	float d = _sub_lidar->get().current_distance;
	float eps = 0.01f;	// 1 cm
	float min_dist = _sub_lidar->get().min_distance + eps;
	float max_dist = _sub_lidar->get().max_distance - eps;

	// prevent driver from setting min dist below eps
	if (min_dist < eps) {
		min_dist = eps;
	}

	// check for bad data
	if (d > max_dist || d < min_dist) {
		return -1;
	}

	// update stats
	_lidarStats.update(Scalarf(d));
	_time_last_lidar = _timeStamp;
	y.setZero();
	matrix::Eulerf euler(matrix::Quatf(_sub_att.get().q));
	y(0) = (d + _param_lpe_ldr_off_z.get()) *
	       cosf(euler.phi()) *
	       cosf(euler.theta());
	return OK;
}

void BlockLocalPositionEstimator::lidarCorrect()
{
	// measure lidar
	Vector<float, n_y_lidar> y;

	if (lidarMeasure(y) != OK) { return; }

	// measurement matrix
	Matrix<float, n_y_lidar, n_x> C;
	C.setZero();
	// y = -(z - tz)
	// TODO could add trig to make this an EKF correction
	C(Y_lidar_z, X_z) = -1;	// measured altitude, negative down dir.
	C(Y_lidar_z, X_tz) = 1;	// measured altitude, negative down dir.

	// use parameter covariance unless sensor provides reasonable value
	SquareMatrix<float, n_y_lidar> R;
	R.setZero();
	float cov = _sub_lidar->get().variance;

	if (cov < 1.0e-3f) {
		R(0, 0) = _param_lpe_ldr_z.get() * _param_lpe_ldr_z.get();

	} else {
		R(0, 0) = cov;
	}

	// residual
	Vector<float, n_y_lidar> r = y - C * _x;
	// residual covariance
	Matrix<float, n_y_lidar, n_y_lidar> S = C * m_P * C.transpose() + R;

	// publish innovations
	_pub_innov.get().hagl = r(0);
	_pub_innov_var.get().hagl = S(0, 0);

	// residual covariance, (inverse)
	Matrix<float, n_y_lidar, n_y_lidar> S_I = inv<float, n_y_lidar>(S);

	// fault detection
	float beta = (r.transpose() * (S_I * r))(0, 0);

	if (beta > BETA_TABLE[n_y_lidar]) {
		if (!(_sensorFault & SENSOR_LIDAR)) {
			mavlink_log_info(&mavlink_log_pub, "[lpe] lidar fault,  beta %5.2f", double(beta));
			_sensorFault |= SENSOR_LIDAR;
		}

		// abort correction
		return;

	} else if (_sensorFault & SENSOR_LIDAR) {
		_sensorFault &= ~SENSOR_LIDAR;
		mavlink_log_info(&mavlink_log_pub, "[lpe] lidar OK");
	}

	// kalman filter correction always
	Matrix<float, n_x, n_y_lidar> K = m_P * C.transpose() * S_I;
	Vector<float, n_x> dx = K * r;
	_x += dx;
	m_P -= K * C * m_P;
}

void BlockLocalPositionEstimator::lidarCheckTimeout()
{
	if (_timeStamp - _time_last_lidar > LIDAR_TIMEOUT) {
		if (!(_sensorTimeout & SENSOR_LIDAR)) {
			_sensorTimeout |= SENSOR_LIDAR;
			_lidarStats.reset();
			mavlink_log_info(&mavlink_log_pub, "[lpe] lidar timeout ");
		}
	}
}
