#include "../BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>

extern orb_advert_t mavlink_log_pub;

// required number of samples for sensor
// to initialize
//
static const uint32_t		REQ_LAND_INIT_COUNT = 1;
static const uint32_t		LAND_TIMEOUT = 1000000;	// 1.0 s

void BlockLocalPositionEstimator::landInit()
{
	// measure
	Vector<float, n_y_land> y;

	if (landMeasure(y) != OK) {
		_landCount = 0;
	}

	// if finished
	if (_landCount > REQ_LAND_INIT_COUNT) {
		mavlink_log_info(&mavlink_log_pub, "[lpe] land init");
		_sensorTimeout &= ~SENSOR_LAND;
		_sensorFault &= ~SENSOR_LAND;
	}
}

int BlockLocalPositionEstimator::landMeasure(Vector<float, n_y_land> &y)
{
	_time_last_land = _timeStamp;
	y.setZero();
	_landCount += 1;
	return OK;
}

void BlockLocalPositionEstimator::landCorrect()
{
	// measure land
	Vector<float, n_y_land> y;

	if (landMeasure(y) != OK) { return; }

	// measurement matrix
	Matrix<float, n_y_land, n_x> C;
	C.setZero();
	// y = -(z - tz)
	C(Y_land_vx, X_vx) = 1;
	C(Y_land_vy, X_vy) = 1;
	C(Y_land_agl, X_z) = -1;// measured altitude, negative down dir.
	C(Y_land_agl, X_tz) = 1;// measured altitude, negative down dir.

	// use parameter covariance
	SquareMatrix<float, n_y_land> R;
	R.setZero();
	R(Y_land_vx, Y_land_vx) = _param_lpe_land_vxy.get() * _param_lpe_land_vxy.get();
	R(Y_land_vy, Y_land_vy) = _param_lpe_land_vxy.get() * _param_lpe_land_vxy.get();
	R(Y_land_agl, Y_land_agl) = _param_lpe_land_z.get() * _param_lpe_land_z.get();

	// residual
	Matrix<float, n_y_land, n_y_land> S_I = inv<float, n_y_land>((C * m_P * C.transpose()) + R);
	Vector<float, n_y_land> r = y - C * _x;
	_pub_innov.get().hagl = r(Y_land_agl);
	_pub_innov_var.get().hagl = R(Y_land_agl, Y_land_agl);

	// fault detection
	float beta = (r.transpose() * (S_I * r))(0, 0);

	// artifically increase beta threshhold to prevent fault during landing
	float beta_thresh = 1e2f;

	if (beta / BETA_TABLE[n_y_land] > beta_thresh) {
		if (!(_sensorFault & SENSOR_LAND)) {
			_sensorFault |= SENSOR_LAND;
			mavlink_log_info(&mavlink_log_pub, "[lpe] land fault,  beta %5.2f", double(beta));
		}

		// abort correction
		return;

	} else if (_sensorFault & SENSOR_LAND) {
		_sensorFault &= ~SENSOR_LAND;
		mavlink_log_info(&mavlink_log_pub, "[lpe] land OK");
	}

	// kalman filter correction always for land detector
	Matrix<float, n_x, n_y_land> K = m_P * C.transpose() * S_I;
	Vector<float, n_x> dx = K * r;
	_x += dx;
	m_P -= K * C * m_P;
}

void BlockLocalPositionEstimator::landCheckTimeout()
{
	if (_timeStamp - _time_last_land > LAND_TIMEOUT) {
		if (!(_sensorTimeout & SENSOR_LAND)) {
			_sensorTimeout |= SENSOR_LAND;
			_landCount = 0;
			mavlink_log_info(&mavlink_log_pub, "[lpe] land timeout ");
		}
	}
}
