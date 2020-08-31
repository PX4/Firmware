#include "../BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>

extern orb_advert_t mavlink_log_pub;

// required number of samples for sensor
// to initialize
static const uint32_t		REQ_BARO_INIT_COUNT = 100;
static const uint32_t		BARO_TIMEOUT = 100000;	// 0.1 s

void BlockLocalPositionEstimator::baroInit()
{
	// measure
	Vector<float, n_y_baro> y;

	if (baroMeasure(y) != OK) {
		_baroStats.reset();
		return;
	}

	// if finished
	if (_baroStats.getCount() > REQ_BARO_INIT_COUNT) {
		_baroAltOrigin = _baroStats.getMean()(0);
		mavlink_log_info(&mavlink_log_pub,
				 "[lpe] baro init %d m std %d cm",
				 (int)_baroStats.getMean()(0),
				 (int)(100 * _baroStats.getStdDev()(0)));
		_sensorTimeout &= ~SENSOR_BARO;
		_sensorFault &= ~SENSOR_BARO;

		if (!_altOriginInitialized) {
			_altOriginInitialized = true;
			_altOriginGlobal = false;
			_altOrigin = _baroAltOrigin;
		}
	}
}

int BlockLocalPositionEstimator::baroMeasure(Vector<float, n_y_baro> &y)
{
	//measure
	y.setZero();
	y(0) = _sub_airdata.get().baro_alt_meter;
	_baroStats.update(y);
	_time_last_baro = _sub_airdata.get().timestamp;
	return OK;
}

void BlockLocalPositionEstimator::baroCorrect()
{
	// measure
	Vector<float, n_y_baro> y;

	if (baroMeasure(y) != OK) { return; }

	// subtract baro origin alt
	y -= _baroAltOrigin;

	// baro measurement matrix
	Matrix<float, n_y_baro, n_x> C;
	C.setZero();
	C(Y_baro_z, X_z) = -1;	// measured altitude, negative down dir.

	Matrix<float, n_y_baro, n_y_baro> R;
	R.setZero();
	R(0, 0) = _param_lpe_bar_z.get() * _param_lpe_bar_z.get();

	// residual
	Matrix<float, n_y_baro, n_y_baro> S_I =
		inv<float, n_y_baro>((C * m_P * C.transpose()) + R);
	Vector<float, n_y_baro> r = y - (C * _x);

	// fault detection
	float beta = (r.transpose() * (S_I * r))(0, 0);

	if (beta > BETA_TABLE[n_y_baro]) {
		if (!(_sensorFault & SENSOR_BARO)) {
			mavlink_log_critical(&mavlink_log_pub, "[lpe] baro fault, r %5.2f m, beta %5.2f",
					     double(r(0)), double(beta));
			_sensorFault |= SENSOR_BARO;
		}

	} else if (_sensorFault & SENSOR_BARO) {
		_sensorFault &= ~SENSOR_BARO;
		mavlink_log_info(&mavlink_log_pub, "[lpe] baro OK");
	}

	// kalman filter correction always
	Matrix<float, n_x, n_y_baro> K = m_P * C.transpose() * S_I;
	Vector<float, n_x> dx = K * r;
	_x += dx;
	m_P -= K * C * m_P;
}

void BlockLocalPositionEstimator::baroCheckTimeout()
{
	if (_timeStamp - _time_last_baro > BARO_TIMEOUT) {
		if (!(_sensorTimeout & SENSOR_BARO)) {
			_sensorTimeout |= SENSOR_BARO;
			_baroStats.reset();
			mavlink_log_info(&mavlink_log_pub, "[lpe] baro timeout ");
		}
	}
}
