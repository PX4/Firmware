#include "../BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>

extern orb_advert_t mavlink_log_pub;

// required number of samples for sensor
// to initialize
static const int	REQ_SONAR_INIT_COUNT = 10;
static const uint32_t	SONAR_TIMEOUT = 5000000;	// 2.0 s
static const float	SONAR_MAX_INIT_STD = 0.3f;	// meters

void BlockLocalPositionEstimator::sonarInit()
{
	// measure
	Vector<float, n_y_sonar> y;

	if (_sonarStats.getCount() == 0) {
		_time_init_sonar = _timeStamp;
	}

	if (sonarMeasure(y) != OK) {
		return;
	}

	// if finished
	if (_sonarStats.getCount() > REQ_SONAR_INIT_COUNT) {
		if (_sonarStats.getStdDev()(0) > SONAR_MAX_INIT_STD) {
			mavlink_log_info(&mavlink_log_pub, "[lpe] sonar init std > min");
			_sonarStats.reset();

		} else if ((_timeStamp - _time_init_sonar) > SONAR_TIMEOUT) {
			mavlink_log_info(&mavlink_log_pub, "[lpe] sonar init timeout ");
			_sonarStats.reset();

		} else {
			PX4_INFO("[lpe] sonar init "
				 "mean %d cm std %d cm",
				 int(100 * _sonarStats.getMean()(0)),
				 int(100 * _sonarStats.getStdDev()(0)));
			_sensorTimeout &= ~SENSOR_SONAR;
			_sensorFault &= ~SENSOR_SONAR;
		}
	}
}

int BlockLocalPositionEstimator::sonarMeasure(Vector<float, n_y_sonar> &y)
{
	// measure
	float d = _sub_sonar->get().current_distance;
	float eps = 0.01f;	// 1 cm
	float min_dist = _sub_sonar->get().min_distance + eps;
	float max_dist = _sub_sonar->get().max_distance - eps;

	// prevent driver from setting min dist below eps
	if (min_dist < eps) {
		min_dist = eps;
	}

	// check for bad data
	if (d > max_dist || d < min_dist) {
		return -1;
	}

	// update stats
	_sonarStats.update(Scalarf(d));
	_time_last_sonar = _timeStamp;
	y.setZero();
	matrix::Eulerf euler(matrix::Quatf(_sub_att.get().q));
	y(0) = (d + _param_lpe_snr_off_z.get()) *
	       cosf(euler.phi()) *
	       cosf(euler.theta());
	return OK;
}

void BlockLocalPositionEstimator::sonarCorrect()
{
	// measure
	Vector<float, n_y_sonar> y;

	if (sonarMeasure(y) != OK) { return; }

	// do not use sonar if lidar is active and not faulty or timed out
	if (_lidarUpdated
	    && !(_sensorFault & SENSOR_LIDAR)
	    && !(_sensorTimeout & SENSOR_LIDAR)) { return; }

	// calculate covariance
	float cov = _sub_sonar->get().variance;

	if (cov < 1.0e-3f) {
		// use sensor value if reasoanble
		cov = _param_lpe_snr_z.get() * _param_lpe_snr_z.get();
	}

	// sonar measurement matrix and noise matrix
	Matrix<float, n_y_sonar, n_x> C;
	C.setZero();
	// y = -(z - tz)
	// TODO could add trig to make this an EKF correction
	C(Y_sonar_z, X_z) = -1;	// measured altitude, negative down dir.
	C(Y_sonar_z, X_tz) = 1;	// measured altitude, negative down dir.

	// covariance matrix
	SquareMatrix<float, n_y_sonar> R;
	R.setZero();
	R(0, 0) = cov;

	// residual
	Vector<float, n_y_sonar> r = y - C * _x;
	// residual covariance
	Matrix<float, n_y_sonar, n_y_sonar> S = C * m_P * C.transpose() + R;

	// publish innovations
	_pub_innov.get().hagl = r(0);
	_pub_innov_var.get().hagl = S(0, 0);

	// residual covariance, (inverse)
	Matrix<float, n_y_sonar, n_y_sonar> S_I = inv<float, n_y_sonar>(S);

	// fault detection
	float beta = (r.transpose()  * (S_I * r))(0, 0);

	if (beta > BETA_TABLE[n_y_sonar]) {
		if (!(_sensorFault & SENSOR_SONAR)) {
			_sensorFault |= SENSOR_SONAR;
			mavlink_log_info(&mavlink_log_pub, "[lpe] sonar fault,  beta %5.2f", double(beta));
		}

		// abort correction
		return;

	} else if (_sensorFault & SENSOR_SONAR) {
		_sensorFault &= ~SENSOR_SONAR;
		//mavlink_log_info(&mavlink_log_pub, "[lpe] sonar OK");
	}

	// kalman filter correction if no fault
	if (!(_sensorFault & SENSOR_SONAR)) {
		Matrix<float, n_x, n_y_sonar> K =
			m_P * C.transpose() * S_I;
		Vector<float, n_x> dx = K * r;
		_x += dx;
		m_P -= K * C * m_P;
	}
}

void BlockLocalPositionEstimator::sonarCheckTimeout()
{
	if (_timeStamp - _time_last_sonar > SONAR_TIMEOUT) {
		if (!(_sensorTimeout & SENSOR_SONAR)) {
			_sensorTimeout |= SENSOR_SONAR;
			_sonarStats.reset();
			mavlink_log_info(&mavlink_log_pub, "[lpe] sonar timeout ");
		}
	}
}
