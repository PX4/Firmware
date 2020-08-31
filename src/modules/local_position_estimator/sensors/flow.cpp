#include "../BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>

// mavlink pub
extern orb_advert_t mavlink_log_pub;

// required number of samples for sensor
// to initialize
static const uint32_t		REQ_FLOW_INIT_COUNT = 10;
static const uint32_t		FLOW_TIMEOUT = 1000000;	// 1 s

void BlockLocalPositionEstimator::flowInit()
{
	// measure
	Vector<float, n_y_flow> y;

	if (flowMeasure(y) != OK) {
		_flowQStats.reset();
		return;
	}

	// if finished
	if (_flowQStats.getCount() > REQ_FLOW_INIT_COUNT) {
		mavlink_log_info(&mavlink_log_pub, "[lpe] flow init: "
				 "quality %d std %d",
				 int(_flowQStats.getMean()(0)),
				 int(_flowQStats.getStdDev()(0)));
		_sensorTimeout &= ~SENSOR_FLOW;
		_sensorFault &= ~SENSOR_FLOW;
	}
}

int BlockLocalPositionEstimator::flowMeasure(Vector<float, n_y_flow> &y)
{
	matrix::Eulerf euler(matrix::Quatf(_sub_att.get().q));

	// check for sane pitch/roll
	if (euler.phi() > 0.5f || euler.theta() > 0.5f) {
		return -1;
	}

	// check for agl
	if (agl() < _sub_flow.get().min_ground_distance) {
		return -1;
	}

	// check quality
	float qual = _sub_flow.get().quality;

	if (qual < _param_lpe_flw_qmin.get()) {
		return -1;
	}

	// calculate range to center of image for flow
	if (!(_estimatorInitialized & EST_TZ)) {
		return -1;
	}

	float d = agl() * cosf(euler.phi()) * cosf(euler.theta());

	// optical flow in x, y axis
	// TODO consider making flow scale a states of the kalman filter
	float flow_x_rad = _sub_flow.get().pixel_flow_x_integral * _param_lpe_flw_scale.get();
	float flow_y_rad = _sub_flow.get().pixel_flow_y_integral * _param_lpe_flw_scale.get();
	float dt_flow = _sub_flow.get().integration_timespan / 1.0e6f;

	if (dt_flow > 0.5f || dt_flow < 1.0e-6f) {
		return -1;
	}

	// angular rotation in x, y axis
	float gyro_x_rad = 0;
	float gyro_y_rad = 0;

	if (_param_lpe_fusion.get() & FUSE_FLOW_GYRO_COMP) {
		gyro_x_rad = _flow_gyro_x_high_pass.update(
				     _sub_flow.get().gyro_x_rate_integral);
		gyro_y_rad = _flow_gyro_y_high_pass.update(
				     _sub_flow.get().gyro_y_rate_integral);
	}

	//warnx("flow x: %10.4f y: %10.4f gyro_x: %10.4f gyro_y: %10.4f d: %10.4f",
	//double(flow_x_rad), double(flow_y_rad), double(gyro_x_rad), double(gyro_y_rad), double(d));

	// compute velocities in body frame using ground distance
	// note that the integral rates in the optical_flow uORB topic are RH rotations about body axes
	Vector3f delta_b(
		+(flow_y_rad - gyro_y_rad) * d,
		-(flow_x_rad - gyro_x_rad) * d,
		0);

	// rotation of flow from body to nav frame
	Vector3f delta_n = _R_att * delta_b;

	// imporant to timestamp flow even if distance is bad
	_time_last_flow = _timeStamp;

	// measurement
	y(Y_flow_vx) = delta_n(0) / dt_flow;
	y(Y_flow_vy) = delta_n(1) / dt_flow;

	_flowQStats.update(Scalarf(_sub_flow.get().quality));

	return OK;
}

void BlockLocalPositionEstimator::flowCorrect()
{
	// measure flow
	Vector<float, n_y_flow> y;

	if (flowMeasure(y) != OK) { return; }

	// flow measurement matrix and noise matrix
	Matrix<float, n_y_flow, n_x> C;
	C.setZero();
	C(Y_flow_vx, X_vx) = 1;
	C(Y_flow_vy, X_vy) = 1;

	SquareMatrix<float, n_y_flow> R;
	R.setZero();

	// polynomial noise model, found using least squares fit
	// h, h**2, v, v*h, v*h**2
	const float p[5] = {0.04005232f, -0.00656446f, -0.26265873f,  0.13686658f, -0.00397357f};

	// prevent extrapolation past end of polynomial fit by bounding independent variables
	float h = agl();
	float v = y.norm();
	const float h_min = 2.0f;
	const float h_max = 8.0f;
	const float v_min = 0.5f;
	const float v_max = 1.0f;

	if (h > h_max) {
		h = h_max;
	}

	if (h < h_min) {
		h = h_min;
	}

	if (v > v_max) {
		v = v_max;
	}

	if (v < v_min) {
		v = v_min;
	}

	// compute polynomial value
	float flow_vxy_stddev = p[0] * h + p[1] * h * h + p[2] * v + p[3] * v * h + p[4] * v * h * h;

	const Vector3f rates{_sub_angular_velocity.get().xyz};
	float rotrate_sq = rates(0) * rates(0)
			   + rates(1) * rates(1)
			   + rates(2) * rates(2);

	matrix::Eulerf euler(matrix::Quatf(_sub_att.get().q));
	float rot_sq = euler.phi() * euler.phi() + euler.theta() * euler.theta();

	R(Y_flow_vx, Y_flow_vx) = flow_vxy_stddev * flow_vxy_stddev +
				  _param_lpe_flw_r.get() * _param_lpe_flw_r.get() * rot_sq +
				  _param_lpe_flw_rr.get() * _param_lpe_flw_rr.get() * rotrate_sq;
	R(Y_flow_vy, Y_flow_vy) = R(Y_flow_vx, Y_flow_vx);

	// residual
	Vector<float, 2> r = y - C * _x;

	// residual covariance
	Matrix<float, n_y_flow, n_y_flow> S = C * m_P * C.transpose() + R;

	// publish innovations
	_pub_innov.get().flow[0] = r(0);
	_pub_innov.get().flow[1] = r(1);
	_pub_innov_var.get().flow[0] = S(0, 0);
	_pub_innov_var.get().flow[1] = S(1, 1);

	// residual covariance, (inverse)
	Matrix<float, n_y_flow, n_y_flow> S_I = inv<float, n_y_flow>(S);

	// fault detection
	float beta = (r.transpose() * (S_I * r))(0, 0);

	if (beta > BETA_TABLE[n_y_flow]) {
		if (!(_sensorFault & SENSOR_FLOW)) {
			mavlink_log_info(&mavlink_log_pub, "[lpe] flow fault,  beta %5.2f", double(beta));
			_sensorFault |= SENSOR_FLOW;
		}

	} else if (_sensorFault & SENSOR_FLOW) {
		_sensorFault &= ~SENSOR_FLOW;
		mavlink_log_info(&mavlink_log_pub, "[lpe] flow OK");
	}

	if (!(_sensorFault & SENSOR_FLOW)) {
		Matrix<float, n_x, n_y_flow> K =
			m_P * C.transpose() * S_I;
		Vector<float, n_x> dx = K * r;
		_x += dx;
		m_P -= K * C * m_P;
	}
}

void BlockLocalPositionEstimator::flowCheckTimeout()
{
	if (_timeStamp - _time_last_flow > FLOW_TIMEOUT) {
		if (!(_sensorTimeout & SENSOR_FLOW)) {
			_sensorTimeout |= SENSOR_FLOW;
			_flowQStats.reset();
			mavlink_log_critical(&mavlink_log_pub, "[lpe] flow timeout ");
		}
	}
}
