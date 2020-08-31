#include "../BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>

extern orb_advert_t mavlink_log_pub;

static const uint64_t 	TARGET_TIMEOUT =   2000000; // [us]

void BlockLocalPositionEstimator::landingTargetInit()
{
	if (_param_ltest_mode.get() == Target_Moving) {
		// target is in moving mode, do not initialize
		return;
	}

	Vector<float, n_y_target> y;

	if (landingTargetMeasure(y) == OK) {
		mavlink_log_info(&mavlink_log_pub, "Landing target init");
		_sensorTimeout &= ~SENSOR_LAND_TARGET;
		_sensorFault &= ~SENSOR_LAND_TARGET;
	}
}

int BlockLocalPositionEstimator::landingTargetMeasure(Vector<float, n_y_target> &y)
{
	if (_param_ltest_mode.get() == Target_Stationary) {
		if (_sub_landing_target_pose.get().rel_vel_valid) {
			y(0) = _sub_landing_target_pose.get().vx_rel;
			y(1) = _sub_landing_target_pose.get().vy_rel;
			_time_last_target = _timeStamp;

		} else {
			return -1;
		}

		return OK;

	}

	return -1;
}

void BlockLocalPositionEstimator::landingTargetCorrect()
{
	if (_param_ltest_mode.get() == Target_Moving) {
		// nothing to do in this mode
		return;
	}

	// measure
	Vector<float, n_y_target> y;

	if (landingTargetMeasure(y) != OK) { return; }

	// calculate covariance
	float cov_vx = _sub_landing_target_pose.get().cov_vx_rel;
	float cov_vy = _sub_landing_target_pose.get().cov_vy_rel;

	// use sensor value only if reasoanble
	if (cov_vx < _param_lpe_lt_cov.get() || cov_vy < _param_lpe_lt_cov.get()) {
		cov_vx = _param_lpe_lt_cov.get();
		cov_vy = _param_lpe_lt_cov.get();
	}

	// target measurement matrix and noise matrix
	Matrix<float, n_y_target, n_x> C;
	C.setZero();
	// residual = (y + vehicle velocity)
	// sign change because target velocitiy is -vehicle velocity
	C(Y_target_x, X_vx) = -1;
	C(Y_target_y, X_vy) = -1;

	// covariance matrix
	SquareMatrix<float, n_y_target> R;
	R.setZero();
	R(0, 0) = cov_vx;
	R(1, 1) = cov_vy;

	// residual
	Vector<float, n_y_target> r = y - C * _x;

	// residual covariance, (inverse)
	Matrix<float, n_y_target, n_y_target> S_I =
		inv<float, n_y_target>(C * m_P * C.transpose() + R);

	// fault detection
	float beta = (r.transpose()  * (S_I * r))(0, 0);

	if (beta > BETA_TABLE[n_y_target]) {
		if (!(_sensorFault & SENSOR_LAND_TARGET)) {
			mavlink_log_info(&mavlink_log_pub, "Landing target fault, beta %5.2f", double(beta));
			_sensorFault |= SENSOR_LAND_TARGET;
		}

		// abort correction
		return;

	} else if (_sensorFault & SENSOR_LAND_TARGET) {
		_sensorFault &= ~SENSOR_LAND_TARGET;
		mavlink_log_info(&mavlink_log_pub, "Landing target OK");
	}

	// kalman filter correction
	Matrix<float, n_x, n_y_target> K =
		m_P * C.transpose() * S_I;
	Vector<float, n_x> dx = K * r;
	_x += dx;
	m_P -= K * C * m_P;

}

void BlockLocalPositionEstimator::landingTargetCheckTimeout()
{
	if (_timeStamp - _time_last_target > TARGET_TIMEOUT) {
		if (!(_sensorTimeout & SENSOR_LAND_TARGET)) {
			_sensorTimeout |= SENSOR_LAND_TARGET;
			mavlink_log_info(&mavlink_log_pub, "Landing target timeout");
		}
	}
}
