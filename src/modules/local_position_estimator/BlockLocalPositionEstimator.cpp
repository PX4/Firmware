#include "BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <fcntl.h>
#include <systemlib/err.h>
#include <matrix/math.hpp>
#include <cstdlib>

orb_advert_t mavlink_log_pub = nullptr;

// required standard deviation of estimate for estimator to publish data
static const uint32_t		EST_STDDEV_XY_VALID = 2.0;	// 2.0 m
static const uint32_t		EST_STDDEV_Z_VALID = 2.0;	// 2.0 m
static const uint32_t		EST_STDDEV_TZ_VALID = 2.0;	// 2.0 m

static const float P_MAX = 1.0e6f;	// max allowed value in state covariance
static const float LAND_RATE = 10.0f;	// rate of land detector correction

static const char *msg_label = "[lpe] ";	// rate of land detector correction

BlockLocalPositionEstimator::BlockLocalPositionEstimator() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),

	// this block has no parent, and has name LPE
	SuperBlock(nullptr, "LPE"),

	// map projection
	_map_ref(),

	// flow gyro
	_flow_gyro_x_high_pass(this, "FGYRO_HP"),
	_flow_gyro_y_high_pass(this, "FGYRO_HP"),

	// stats
	_baroStats(this, ""),
	_sonarStats(this, ""),
	_lidarStats(this, ""),
	_flowQStats(this, ""),
	_visionStats(this, ""),
	_mocapStats(this, ""),
	_gpsStats(this, ""),

	// low pass
	_xLowPass(this, "X_LP"),

	// use same lp constant for agl
	_aglLowPass(this, "X_LP"),

	// delay
	_xDelay(this, ""),
	_tDelay(this, ""),

	// misc
	_timeStamp(hrt_absolute_time()),
	_time_origin(0),
	_timeStampLastBaro(hrt_absolute_time()),
	_time_last_hist(0),
	_time_last_flow(0),
	_time_last_baro(0),
	_time_last_gps(0),
	_time_last_lidar(0),
	_time_last_sonar(0),
	_time_init_sonar(0),
	_time_last_vision_p(0),
	_time_last_mocap(0),
	_time_last_land(0),
	_time_last_target(0),

	// reference altitudes
	_altOrigin(0),
	_altOriginInitialized(false),
	_altOriginGlobal(false),
	_baroAltOrigin(0),
	_gpsAltOrigin(0),

	// status
	_receivedGps(false),
	_lastArmedState(false),

	// masks
	_sensorTimeout(UINT16_MAX),
	_sensorFault(0),
	_estimatorInitialized(0),

	// sensor update flags
	_flowUpdated(false),
	_gpsUpdated(false),
	_visionUpdated(false),
	_mocapUpdated(false),
	_lidarUpdated(false),
	_sonarUpdated(false),
	_landUpdated(false),
	_baroUpdated(false),

	// sensor validation flags
	_vision_xy_valid(false),
	_vision_z_valid(false),
	_mocap_xy_valid(false),
	_mocap_z_valid(false),

	// sensor std deviations
	_vision_eph(0.0),
	_vision_epv(0.0),
	_mocap_eph(0.0),
	_mocap_epv(0.0),

	// local to global coversion related variables
	_is_global_cov_init(false),
	_ref_lat(0.0),
	_ref_lon(0.0),
	_ref_alt(0.0)
{
	_sensors_sub.set_interval_ms(10); // main prediction loop, 100 hz (lockstep requires to run at full rate)

	// assign distance subs to array
	_dist_subs[0] = &_sub_dist0;
	_dist_subs[1] = &_sub_dist1;
	_dist_subs[2] = &_sub_dist2;
	_dist_subs[3] = &_sub_dist3;

	// initialize A, B,  P, x, u
	_x.setZero();
	_u.setZero();
	initSS();

	// map
	_map_ref.init_done = false;

	// print fusion settings to console
	PX4_INFO("fuse gps: %d, flow: %d, vis_pos: %d, "
		 "landing_target: %d, land: %d, pub_agl_z: %d, flow_gyro: %d, "
		 "baro: %d\n",
		 (_param_lpe_fusion.get() & FUSE_GPS) != 0,
		 (_param_lpe_fusion.get() & FUSE_FLOW) != 0,
		 (_param_lpe_fusion.get() & FUSE_VIS_POS) != 0,
		 (_param_lpe_fusion.get() & FUSE_LAND_TARGET) != 0,
		 (_param_lpe_fusion.get() & FUSE_LAND) != 0,
		 (_param_lpe_fusion.get() & FUSE_PUB_AGL_Z) != 0,
		 (_param_lpe_fusion.get() & FUSE_FLOW_GYRO_COMP) != 0,
		 (_param_lpe_fusion.get() & FUSE_BARO) != 0);
}

bool
BlockLocalPositionEstimator::init()
{
	if (!_sensors_sub.registerCallback()) {
		PX4_ERR("sensor combined callback registration failed!");
		return false;
	}

	return true;
}

Vector<float, BlockLocalPositionEstimator::n_x> BlockLocalPositionEstimator::dynamics(
	float t,
	const Vector<float, BlockLocalPositionEstimator::n_x> &x,
	const Vector<float, BlockLocalPositionEstimator::n_u> &u)
{
	return m_A * x + m_B * u;
}

void BlockLocalPositionEstimator::Run()
{
	if (should_exit()) {
		_sensors_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	if (_vehicle_command_sub.updated()) {
		vehicle_command_s vehicle_command;

		if (_vehicle_command_sub.update(&vehicle_command)) {
			if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN) {
				const double latitude = vehicle_command.param5;
				const double longitude = vehicle_command.param6;
				const float altitude = vehicle_command.param7;

				map_projection_init_timestamped(&_global_local_proj_ref, latitude, longitude, vehicle_command.timestamp);
				_global_local_alt0 = altitude;

				PX4_INFO("New NED origin (LLA): %3.10f, %3.10f, %4.3f\n", latitude, longitude, static_cast<double>(altitude));
			}
		}
	}

	sensor_combined_s imu;

	if (!_sensors_sub.update(&imu)) {
		return;
	}

	uint64_t newTimeStamp = hrt_absolute_time();
	float dt = (newTimeStamp - _timeStamp) / 1.0e6f;
	_timeStamp = newTimeStamp;

	// set dt for all child blocks
	setDt(dt);

	// auto-detect connected rangefinders while not armed
	_sub_armed.update();
	bool armedState = _sub_armed.get().armed;

	if (!armedState && (_sub_lidar == nullptr || _sub_sonar == nullptr)) {
		// detect distance sensors
		for (size_t i = 0; i < N_DIST_SUBS; i++) {
			auto *s = _dist_subs[i];

			if (s == _sub_lidar || s == _sub_sonar) { continue; }

			if (s->update()) {

				if (s->get().timestamp == 0) { continue; }

				if (s->get().type == distance_sensor_s::MAV_DISTANCE_SENSOR_LASER &&
				    s->get().orientation == distance_sensor_s::ROTATION_DOWNWARD_FACING &&
				    _sub_lidar == nullptr) {
					_sub_lidar = s;
					mavlink_log_info(&mavlink_log_pub, "%sDownward-facing Lidar detected with ID %zu", msg_label, i);

				} else if (s->get().type == distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND &&
					   s->get().orientation == distance_sensor_s::ROTATION_DOWNWARD_FACING &&
					   _sub_sonar == nullptr) {
					_sub_sonar = s;
					mavlink_log_info(&mavlink_log_pub, "%sDownward-facing Sonar detected with ID %zu", msg_label, i);
				}
			}
		}
	}

	// reset pos, vel, and terrain on arming

	// XXX this will be re-enabled for indoor use cases using a
	// selection param, but is really not helping outdoors
	// right now.

	// if (!_lastArmedState && armedState) {

	//      // we just armed, we are at origin on the ground
	//      _x(X_x) = 0;
	//      _x(X_y) = 0;
	//      // reset Z or not? _x(X_z) = 0;

	//      // we aren't moving, all velocities are zero
	//      _x(X_vx) = 0;
	//      _x(X_vy) = 0;
	//      _x(X_vz) = 0;

	//      // assume we are on the ground, so terrain alt is local alt
	//      _x(X_tz) = _x(X_z);

	//      // reset lowpass filter as well
	//      _xLowPass.setState(_x);
	//      _aglLowPass.setState(0);
	// }

	_lastArmedState = armedState;

	// see which updates are available
	const bool paramsUpdated = _parameter_update_sub.updated();
	_baroUpdated = false;

	if ((_param_lpe_fusion.get() & FUSE_BARO) && _sub_airdata.update()) {
		if (_sub_airdata.get().timestamp != _timeStampLastBaro) {
			_baroUpdated = true;
			_timeStampLastBaro = _sub_airdata.get().timestamp;
		}
	}

	_flowUpdated = (_param_lpe_fusion.get() & FUSE_FLOW) && _sub_flow.update();
	_gpsUpdated = (_param_lpe_fusion.get() & FUSE_GPS) && _sub_gps.update();
	_visionUpdated = (_param_lpe_fusion.get() & FUSE_VIS_POS) && _sub_visual_odom.update();
	_mocapUpdated = _sub_mocap_odom.update();
	_lidarUpdated = (_sub_lidar != nullptr) && _sub_lidar->update();
	_sonarUpdated = (_sub_sonar != nullptr) && _sub_sonar->update();
	_landUpdated = landed() && ((_timeStamp - _time_last_land) > 1.0e6f / LAND_RATE);// throttle rate
	bool targetPositionUpdated = _sub_landing_target_pose.update();

	// get new data
	_sub_att.update();
	_sub_angular_velocity.update();

	// update parameters
	if (paramsUpdated) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		SuperBlock::updateParams();
		ModuleParams::updateParams();
		updateSSParams();
	}

	// is xy valid?
	bool vxy_stddev_ok = false;

	if (math::max(m_P(X_vx, X_vx), m_P(X_vy, X_vy)) < _param_lpe_vxy_pub.get() * _param_lpe_vxy_pub.get()) {
		vxy_stddev_ok = true;
	}

	if (_estimatorInitialized & EST_XY) {
		// if valid and gps has timed out, set to not valid
		if (!vxy_stddev_ok && (_sensorTimeout & SENSOR_GPS)) {
			_estimatorInitialized &= ~EST_XY;
		}

	} else {
		if (vxy_stddev_ok) {
			if (!(_sensorTimeout & SENSOR_GPS)
			    || !(_sensorTimeout & SENSOR_FLOW)
			    || !(_sensorTimeout & SENSOR_VISION)
			    || !(_sensorTimeout & SENSOR_MOCAP)
			    || !(_sensorTimeout & SENSOR_LAND)
			    || !(_sensorTimeout & SENSOR_LAND_TARGET)
			   ) {
				_estimatorInitialized |= EST_XY;
			}
		}
	}

	// is z valid?
	bool z_stddev_ok = sqrtf(m_P(X_z, X_z)) < _param_lpe_z_pub.get();

	if (_estimatorInitialized & EST_Z) {
		// if valid and baro has timed out, set to not valid
		if (!z_stddev_ok && (_sensorTimeout & SENSOR_BARO)) {
			_estimatorInitialized &= ~EST_Z;
		}

	} else {
		if (z_stddev_ok) {
			_estimatorInitialized |= EST_Z;
		}
	}

	// is terrain valid?
	bool tz_stddev_ok = sqrtf(m_P(X_tz, X_tz)) < _param_lpe_z_pub.get();

	if (_estimatorInitialized & EST_TZ) {
		if (!tz_stddev_ok) {
			_estimatorInitialized &= ~EST_TZ;
		}

	} else {
		if (tz_stddev_ok) {
			_estimatorInitialized |= EST_TZ;
		}
	}

	// check timeouts
	checkTimeouts();

	// if we have no lat, lon initialize projection to LPE_LAT, LPE_LON parameters
	if (!_map_ref.init_done && (_estimatorInitialized & EST_XY) && _param_lpe_fake_origin.get()) {
		map_projection_init(&_map_ref,
				    (double)_param_lpe_lat.get(),
				    (double)_param_lpe_lon.get());

		// set timestamp when origin was set to current time
		_time_origin = _timeStamp;

		mavlink_log_info(&mavlink_log_pub, "[lpe] global origin init (parameter) : lat %6.2f lon %6.2f alt %5.1f m",
				 double(_param_lpe_lat.get()), double(_param_lpe_lon.get()), double(_altOrigin));
	}

	// reinitialize x if necessary
	bool reinit_x = false;

	for (size_t i = 0; i < n_x; i++) {
		// should we do a reinit
		// of sensors here?
		// don't want it to take too long
		if (!PX4_ISFINITE(_x(i))) {
			reinit_x = true;
			mavlink_log_info(&mavlink_log_pub, "%sreinit x, x(%zu) not finite", msg_label, i);
			break;
		}
	}

	if (reinit_x) {
		for (size_t i = 0; i < n_x; i++) {
			_x(i) = 0;
		}

		mavlink_log_info(&mavlink_log_pub, "%sreinit x", msg_label);
	}

	// force P symmetry and reinitialize P if necessary
	bool reinit_P = false;

	for (size_t i = 0; i < n_x; i++) {
		for (size_t j = 0; j <= i; j++) {
			if (!PX4_ISFINITE(m_P(i, j))) {
				mavlink_log_info(&mavlink_log_pub,
						 "%sreinit P (%zu, %zu) not finite", msg_label, i, j);
				reinit_P = true;
			}

			if (i == j) {
				// make sure diagonal elements are positive
				if (m_P(i, i) <= 0) {
					mavlink_log_info(&mavlink_log_pub,
							 "%sreinit P (%zu, %zu) negative", msg_label, i, j);
					reinit_P = true;
				}

			} else {
				// copy elememnt from upper triangle to force
				// symmetry
				m_P(j, i) = m_P(i, j);
			}

			if (reinit_P) { break; }
		}

		if (reinit_P) { break; }
	}

	if (reinit_P) {
		initP();
	}

	// do prediction
	predict(imu);

	// sensor corrections/ initializations
	if (_gpsUpdated) {
		if (_sensorTimeout & SENSOR_GPS) {
			gpsInit();

		} else {
			gpsCorrect();
		}
	}

	if (_baroUpdated) {
		if (_sensorTimeout & SENSOR_BARO) {
			baroInit();

		} else {
			baroCorrect();
		}
	}

	if (_lidarUpdated) {
		if (_sensorTimeout & SENSOR_LIDAR) {
			lidarInit();

		} else {
			lidarCorrect();
		}
	}

	if (_sonarUpdated) {
		if (_sensorTimeout & SENSOR_SONAR) {
			sonarInit();

		} else {
			sonarCorrect();
		}
	}

	if (_flowUpdated) {
		if (_sensorTimeout & SENSOR_FLOW) {
			flowInit();

		} else {
			flowCorrect();
		}
	}

	if (_visionUpdated) {
		if (_sensorTimeout & SENSOR_VISION) {
			visionInit();

		} else {
			visionCorrect();
		}
	}

	if (_mocapUpdated) {
		if (_sensorTimeout & SENSOR_MOCAP) {
			mocapInit();

		} else {
			mocapCorrect();
		}
	}

	if (_landUpdated) {
		if (_sensorTimeout & SENSOR_LAND) {
			landInit();

		} else {
			landCorrect();
		}
	}

	if (targetPositionUpdated) {
		if (_sensorTimeout & SENSOR_LAND_TARGET) {
			landingTargetInit();

		} else {
			landingTargetCorrect();
		}
	}

	if (_altOriginInitialized) {
		// update all publications if possible
		publishLocalPos();
		publishOdom();
		publishEstimatorStatus();

		_pub_innov.get().timestamp_sample = _timeStamp;
		_pub_innov.get().timestamp = hrt_absolute_time();
		_pub_innov.update();

		_pub_innov_var.get().timestamp_sample = _timeStamp;
		_pub_innov_var.get().timestamp = hrt_absolute_time();
		_pub_innov_var.update();

		if ((_estimatorInitialized & EST_XY) && (_map_ref.init_done || _param_lpe_fake_origin.get())) {
			publishGlobalPos();
		}
	}

	// propagate delayed state, no matter what
	// if state is frozen, delayed state still
	// needs to be propagated with frozen state
	float dt_hist = 1.0e-6f * (_timeStamp - _time_last_hist);

	if (_time_last_hist == 0 ||
	    (dt_hist > HIST_STEP)) {
		_tDelay.update(Scalar<uint64_t>(_timeStamp));
		_xDelay.update(_x);
		_time_last_hist = _timeStamp;
	}
}

void BlockLocalPositionEstimator::checkTimeouts()
{
	baroCheckTimeout();
	gpsCheckTimeout();
	lidarCheckTimeout();
	flowCheckTimeout();
	sonarCheckTimeout();
	visionCheckTimeout();
	mocapCheckTimeout();
	landCheckTimeout();
	landingTargetCheckTimeout();
}

bool BlockLocalPositionEstimator::landed()
{
	if (!(_param_lpe_fusion.get() & FUSE_LAND)) {
		return false;
	}

	_sub_land.update();

	bool disarmed_not_falling = (!_sub_armed.get().armed) && (!_sub_land.get().freefall);

	return _sub_land.get().landed || disarmed_not_falling;
}

void BlockLocalPositionEstimator::publishLocalPos()
{
	const Vector<float, n_x> &xLP = _xLowPass.getState();

	// lie about eph/epv to allow visual odometry only navigation when velocity est. good
	float evh = sqrtf(m_P(X_vx, X_vx) + m_P(X_vy, X_vy));
	float evv = sqrtf(m_P(X_vz, X_vz));
	float eph = sqrtf(m_P(X_x, X_x) + m_P(X_y, X_y));
	float epv = sqrtf(m_P(X_z, X_z));

	float eph_thresh = 3.0f;
	float epv_thresh = 3.0f;

	if (evh < _param_lpe_vxy_pub.get()) {
		if (eph > eph_thresh) {
			eph = eph_thresh;
		}

		if (epv > epv_thresh) {
			epv = epv_thresh;
		}
	}

	// publish local position
	if (PX4_ISFINITE(_x(X_x)) && PX4_ISFINITE(_x(X_y)) && PX4_ISFINITE(_x(X_z)) &&
	    PX4_ISFINITE(_x(X_vx)) && PX4_ISFINITE(_x(X_vy))
	    && PX4_ISFINITE(_x(X_vz))) {
		_pub_lpos.get().timestamp_sample = _timeStamp;

		_pub_lpos.get().xy_valid = _estimatorInitialized & EST_XY;
		_pub_lpos.get().z_valid = _estimatorInitialized & EST_Z;
		_pub_lpos.get().v_xy_valid = _estimatorInitialized & EST_XY;
		_pub_lpos.get().v_z_valid = _estimatorInitialized & EST_Z;

		_pub_lpos.get().x = xLP(X_x);	// north
		_pub_lpos.get().y = xLP(X_y);	// east

		if (_param_lpe_fusion.get() & FUSE_PUB_AGL_Z) {
			_pub_lpos.get().z = -_aglLowPass.getState();	// agl

		} else {
			_pub_lpos.get().z = xLP(X_z);	// down
		}

		_pub_lpos.get().heading = matrix::Eulerf(matrix::Quatf(_sub_att.get().q)).psi();

		_pub_lpos.get().vx = xLP(X_vx);		// north
		_pub_lpos.get().vy = xLP(X_vy);		// east
		_pub_lpos.get().vz = xLP(X_vz);		// down

		// this estimator does not provide a separate vertical position time derivative estimate, so use the vertical velocity
		_pub_lpos.get().z_deriv = xLP(X_vz);

		_pub_lpos.get().ax = _u(U_ax);		// north
		_pub_lpos.get().ay = _u(U_ay);		// east
		_pub_lpos.get().az = _u(U_az);		// down

		_pub_lpos.get().xy_global = _estimatorInitialized & EST_XY;
		_pub_lpos.get().z_global = !(_sensorTimeout & SENSOR_BARO) && _altOriginGlobal;
		_pub_lpos.get().ref_timestamp = _time_origin;
		_pub_lpos.get().ref_lat = _map_ref.lat_rad * 180 / M_PI;
		_pub_lpos.get().ref_lon = _map_ref.lon_rad * 180 / M_PI;
		_pub_lpos.get().ref_alt = _altOrigin;
		_pub_lpos.get().dist_bottom = _aglLowPass.getState();
		// we estimate agl even when we don't have terrain info
		// if you are in terrain following mode this is important
		// so that if terrain estimation fails there isn't a
		// sudden altitude jump
		_pub_lpos.get().dist_bottom_valid = _estimatorInitialized & EST_Z;
		_pub_lpos.get().eph = eph;
		_pub_lpos.get().epv = epv;
		_pub_lpos.get().evh = evh;
		_pub_lpos.get().evv = evv;
		_pub_lpos.get().vxy_max = INFINITY;
		_pub_lpos.get().vz_max = INFINITY;
		_pub_lpos.get().hagl_min = INFINITY;
		_pub_lpos.get().hagl_max = INFINITY;
		_pub_lpos.get().timestamp = hrt_absolute_time();;
		_pub_lpos.update();
	}
}

void BlockLocalPositionEstimator::publishOdom()
{
	const Vector<float, n_x> &xLP = _xLowPass.getState();

	// publish vehicle odometry
	if (PX4_ISFINITE(_x(X_x)) && PX4_ISFINITE(_x(X_y)) && PX4_ISFINITE(_x(X_z)) &&
	    PX4_ISFINITE(_x(X_vx)) && PX4_ISFINITE(_x(X_vy))
	    && PX4_ISFINITE(_x(X_vz))) {

		_pub_odom.get().timestamp_sample = _timeStamp;
		_pub_odom.get().local_frame = vehicle_odometry_s::LOCAL_FRAME_NED;

		// position
		_pub_odom.get().x = xLP(X_x);	// north
		_pub_odom.get().y = xLP(X_y);	// east

		if (_param_lpe_fusion.get() & FUSE_PUB_AGL_Z) {
			_pub_odom.get().z = -_aglLowPass.getState();	// agl

		} else {
			_pub_odom.get().z = xLP(X_z);	// down
		}

		// orientation
		matrix::Quatf q = matrix::Quatf(_sub_att.get().q);
		q.copyTo(_pub_odom.get().q);

		// linear velocity
		_pub_odom.get().velocity_frame = vehicle_odometry_s::LOCAL_FRAME_FRD;
		_pub_odom.get().vx = xLP(X_vx);		// vel north
		_pub_odom.get().vy = xLP(X_vy);		// vel east
		_pub_odom.get().vz = xLP(X_vz);		// vel down

		// angular velocity
		_pub_odom.get().rollspeed = _sub_angular_velocity.get().xyz[0]; // roll rate
		_pub_odom.get().pitchspeed = _sub_angular_velocity.get().xyz[1]; // pitch rate
		_pub_odom.get().yawspeed = _sub_angular_velocity.get().xyz[2]; // yaw rate

		// get the covariance matrix size
		const size_t POS_URT_SIZE = sizeof(_pub_odom.get().pose_covariance) / sizeof(_pub_odom.get().pose_covariance[0]);
		const size_t VEL_URT_SIZE = sizeof(_pub_odom.get().velocity_covariance) / sizeof(
						    _pub_odom.get().velocity_covariance[0]);

		// initially set pose covariances to 0
		for (size_t i = 0; i < POS_URT_SIZE; i++) {
			_pub_odom.get().pose_covariance[i] = 0.0;
		}

		// set the position variances
		_pub_odom.get().pose_covariance[_pub_odom.get().COVARIANCE_MATRIX_X_VARIANCE] = m_P(X_vx, X_vx);
		_pub_odom.get().pose_covariance[_pub_odom.get().COVARIANCE_MATRIX_Y_VARIANCE] = m_P(X_vy, X_vy);
		_pub_odom.get().pose_covariance[_pub_odom.get().COVARIANCE_MATRIX_Z_VARIANCE] = m_P(X_vz, X_vz);

		// unknown orientation covariances
		// TODO: add orientation covariance to vehicle_attitude
		_pub_odom.get().pose_covariance[_pub_odom.get().COVARIANCE_MATRIX_ROLL_VARIANCE] = NAN;
		_pub_odom.get().pose_covariance[_pub_odom.get().COVARIANCE_MATRIX_PITCH_VARIANCE] = NAN;
		_pub_odom.get().pose_covariance[_pub_odom.get().COVARIANCE_MATRIX_YAW_VARIANCE] = NAN;

		// initially set velocity covariances to 0
		for (size_t i = 0; i < VEL_URT_SIZE; i++) {
			_pub_odom.get().velocity_covariance[i] = 0.0;
		}

		// set the linear velocity variances
		_pub_odom.get().velocity_covariance[_pub_odom.get().COVARIANCE_MATRIX_VX_VARIANCE] = m_P(X_vx, X_vx);
		_pub_odom.get().velocity_covariance[_pub_odom.get().COVARIANCE_MATRIX_VY_VARIANCE] = m_P(X_vy, X_vy);
		_pub_odom.get().velocity_covariance[_pub_odom.get().COVARIANCE_MATRIX_VZ_VARIANCE] = m_P(X_vz, X_vz);

		// unknown angular velocity covariances
		_pub_odom.get().velocity_covariance[_pub_odom.get().COVARIANCE_MATRIX_ROLLRATE_VARIANCE] = NAN;
		_pub_odom.get().velocity_covariance[_pub_odom.get().COVARIANCE_MATRIX_PITCHRATE_VARIANCE] = NAN;
		_pub_odom.get().velocity_covariance[_pub_odom.get().COVARIANCE_MATRIX_YAWRATE_VARIANCE] = NAN;

		_pub_odom.get().timestamp = hrt_absolute_time();
		_pub_odom.update();
	}
}

void BlockLocalPositionEstimator::publishEstimatorStatus()
{
	_pub_est_states.get().timestamp_sample = _timeStamp;

	for (size_t i = 0; i < n_x; i++) {
		_pub_est_states.get().states[i] = _x(i);
	}

	// matching EKF2 covariances indexing
	// quaternion - not determined, as it is a position estimator
	_pub_est_states.get().covariances[0] = NAN;
	_pub_est_states.get().covariances[1] = NAN;
	_pub_est_states.get().covariances[2] = NAN;
	_pub_est_states.get().covariances[3] = NAN;
	// linear velocity
	_pub_est_states.get().covariances[4] = m_P(X_vx, X_vx);
	_pub_est_states.get().covariances[5] = m_P(X_vy, X_vy);
	_pub_est_states.get().covariances[6] = m_P(X_vz, X_vz);
	// position
	_pub_est_states.get().covariances[7] = m_P(X_x, X_x);
	_pub_est_states.get().covariances[8] = m_P(X_y, X_y);
	_pub_est_states.get().covariances[9] = m_P(X_z, X_z);
	// gyro bias - not determined
	_pub_est_states.get().covariances[10] = NAN;
	_pub_est_states.get().covariances[11] = NAN;
	_pub_est_states.get().covariances[12] = NAN;
	// accel bias
	_pub_est_states.get().covariances[13] = m_P(X_bx, X_bx);
	_pub_est_states.get().covariances[14] = m_P(X_by, X_by);
	_pub_est_states.get().covariances[15] = m_P(X_bz, X_bz);

	// mag - not determined
	for (size_t i = 16; i <= 21; i++) {
		_pub_est_states.get().covariances[i] = NAN;
	}

	// replacing the hor wind cov with terrain altitude covariance
	_pub_est_states.get().covariances[22] = m_P(X_tz, X_tz);
	_pub_est_states.get().covariances[23] = NAN;

	_pub_est_states.get().n_states = n_x;
	_pub_est_states.get().timestamp = hrt_absolute_time();
	_pub_est_states.update();

	// estimator_status
	_pub_est_status.get().timestamp_sample = _timeStamp;
	_pub_est_status.get().health_flags = _sensorFault;
	_pub_est_status.get().timeout_flags = _sensorTimeout;
	_pub_est_status.get().pos_horiz_accuracy = _pub_gpos.get().eph;
	_pub_est_status.get().pos_vert_accuracy = _pub_gpos.get().epv;

	_pub_est_status.get().timestamp = hrt_absolute_time();
	_pub_est_status.update();
}

void BlockLocalPositionEstimator::publishGlobalPos()
{
	// publish global position
	double lat = 0;
	double lon = 0;
	const Vector<float, n_x> &xLP = _xLowPass.getState();
	map_projection_reproject(&_map_ref, xLP(X_x), xLP(X_y), &lat, &lon);
	float alt = -xLP(X_z) + _altOrigin;

	// lie about eph/epv to allow visual odometry only navigation when velocity est. good
	float evh = sqrtf(m_P(X_vx, X_vx) + m_P(X_vy, X_vy));
	float eph = sqrtf(m_P(X_x, X_x) + m_P(X_y, X_y));
	float epv = sqrtf(m_P(X_z, X_z));

	float eph_thresh = 3.0f;
	float epv_thresh = 3.0f;

	if (evh < _param_lpe_vxy_pub.get()) {
		if (eph > eph_thresh) {
			eph = eph_thresh;
		}

		if (epv > epv_thresh) {
			epv = epv_thresh;
		}
	}

	if (PX4_ISFINITE(lat) && PX4_ISFINITE(lon) && PX4_ISFINITE(alt) &&
	    PX4_ISFINITE(xLP(X_vx)) && PX4_ISFINITE(xLP(X_vy)) &&
	    PX4_ISFINITE(xLP(X_vz))) {
		_pub_gpos.get().timestamp_sample = _timeStamp;
		_pub_gpos.get().lat = lat;
		_pub_gpos.get().lon = lon;
		_pub_gpos.get().alt = alt;
		_pub_gpos.get().eph = eph;
		_pub_gpos.get().epv = epv;
		_pub_gpos.get().terrain_alt = _altOrigin - xLP(X_tz);
		_pub_gpos.get().terrain_alt_valid = _estimatorInitialized & EST_TZ;
		_pub_gpos.get().dead_reckoning = !(_estimatorInitialized & EST_XY);
		_pub_gpos.get().timestamp = hrt_absolute_time();
		_pub_gpos.update();
	}
}

void BlockLocalPositionEstimator::initP()
{
	m_P.setZero();
	// initialize to twice valid condition
	m_P(X_x, X_x) = 2 * EST_STDDEV_XY_VALID * EST_STDDEV_XY_VALID;
	m_P(X_y, X_y) = 2 * EST_STDDEV_XY_VALID * EST_STDDEV_XY_VALID;
	m_P(X_z, X_z) = 2 * EST_STDDEV_Z_VALID * EST_STDDEV_Z_VALID;
	m_P(X_vx, X_vx) = 2 * _param_lpe_vxy_pub.get() * _param_lpe_vxy_pub.get();
	m_P(X_vy, X_vy) = 2 * _param_lpe_vxy_pub.get() * _param_lpe_vxy_pub.get();
	// use vxy thresh for vz init as well
	m_P(X_vz, X_vz) = 2 * _param_lpe_vxy_pub.get() * _param_lpe_vxy_pub.get();
	// initialize bias uncertainty to small values to keep them stable
	m_P(X_bx, X_bx) = 1e-6;
	m_P(X_by, X_by) = 1e-6;
	m_P(X_bz, X_bz) = 1e-6;
	m_P(X_tz, X_tz) = 2 * EST_STDDEV_TZ_VALID * EST_STDDEV_TZ_VALID;
}

void BlockLocalPositionEstimator::initSS()
{
	initP();

	// dynamics matrix
	m_A.setZero();
	// derivative of position is velocity
	m_A(X_x, X_vx) = 1;
	m_A(X_y, X_vy) = 1;
	m_A(X_z, X_vz) = 1;

	// input matrix
	m_B.setZero();
	m_B(X_vx, U_ax) = 1;
	m_B(X_vy, U_ay) = 1;
	m_B(X_vz, U_az) = 1;

	// update components that depend on current state
	updateSSStates();
	updateSSParams();
}

void BlockLocalPositionEstimator::updateSSStates()
{
	// derivative of velocity is accelerometer acceleration
	// (in input matrix) - bias (in body frame)
	m_A(X_vx, X_bx) = -_R_att(0, 0);
	m_A(X_vx, X_by) = -_R_att(0, 1);
	m_A(X_vx, X_bz) = -_R_att(0, 2);

	m_A(X_vy, X_bx) = -_R_att(1, 0);
	m_A(X_vy, X_by) = -_R_att(1, 1);
	m_A(X_vy, X_bz) = -_R_att(1, 2);

	m_A(X_vz, X_bx) = -_R_att(2, 0);
	m_A(X_vz, X_by) = -_R_att(2, 1);
	m_A(X_vz, X_bz) = -_R_att(2, 2);
}

void BlockLocalPositionEstimator::updateSSParams()
{
	// input noise covariance matrix
	m_R.setZero();
	m_R(U_ax, U_ax) = _param_lpe_acc_xy.get() * _param_lpe_acc_xy.get();
	m_R(U_ay, U_ay) = _param_lpe_acc_xy.get() * _param_lpe_acc_xy.get();
	m_R(U_az, U_az) = _param_lpe_acc_z.get() * _param_lpe_acc_z.get();

	// process noise power matrix
	m_Q.setZero();
	float pn_p_sq = _param_lpe_pn_p.get() * _param_lpe_pn_p.get();
	float pn_v_sq = _param_lpe_pn_v.get() * _param_lpe_pn_v.get();
	m_Q(X_x, X_x) = pn_p_sq;
	m_Q(X_y, X_y) = pn_p_sq;
	m_Q(X_z, X_z) = pn_p_sq;
	m_Q(X_vx, X_vx) = pn_v_sq;
	m_Q(X_vy, X_vy) = pn_v_sq;
	m_Q(X_vz, X_vz) = pn_v_sq;

	// technically, the noise is in the body frame,
	// but the components are all the same, so
	// ignoring for now
	float pn_b_sq = _param_lpe_pn_b.get() * _param_lpe_pn_b.get();
	m_Q(X_bx, X_bx) = pn_b_sq;
	m_Q(X_by, X_by) = pn_b_sq;
	m_Q(X_bz, X_bz) = pn_b_sq;

	// terrain random walk noise ((m/s)/sqrt(hz)), scales with velocity
	float pn_t_noise_density =
		_param_lpe_pn_t.get() +
		(_param_lpe_t_max_grade.get() / 100.0f) * sqrtf(_x(X_vx) * _x(X_vx) + _x(X_vy) * _x(X_vy));
	m_Q(X_tz, X_tz) = pn_t_noise_density * pn_t_noise_density;
}

void BlockLocalPositionEstimator::predict(const sensor_combined_s &imu)
{
	// get acceleration
	_R_att = matrix::Dcm<float>(matrix::Quatf(_sub_att.get().q));
	Vector3f a(imu.accelerometer_m_s2);
	// note, bias is removed in dynamics function
	_u = _R_att * a;
	_u(U_az) += CONSTANTS_ONE_G;	// add g

	// update state space based on new states
	updateSSStates();

	// continuous time kalman filter prediction
	// integrate runge kutta 4th order
	// TODO move rk4 algorithm to matrixlib
	// https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
	float h = getDt();
	Vector<float, n_x> k1, k2, k3, k4;
	k1 = dynamics(0, _x, _u);
	k2 = dynamics(h / 2, _x + k1 * h / 2, _u);
	k3 = dynamics(h / 2, _x + k2 * h / 2, _u);
	k4 = dynamics(h, _x + k3 * h, _u);
	Vector<float, n_x> dx = (k1 + k2 * 2 + k3 * 2 + k4) * (h / 6);

	// don't integrate position if no valid xy data
	if (!(_estimatorInitialized & EST_XY))  {
		dx(X_x) = 0;
		dx(X_vx) = 0;
		dx(X_y) = 0;
		dx(X_vy) = 0;
	}

	// don't integrate z if no valid z data
	if (!(_estimatorInitialized & EST_Z))  {
		dx(X_z) = 0;
	}

	// don't integrate tz if no valid tz data
	if (!(_estimatorInitialized & EST_TZ))  {
		dx(X_tz) = 0;
	}

	// saturate bias
	float bx = dx(X_bx) + _x(X_bx);
	float by = dx(X_by) + _x(X_by);
	float bz = dx(X_bz) + _x(X_bz);

	if (std::abs(bx) > BIAS_MAX) {
		bx = BIAS_MAX * bx / std::abs(bx);
		dx(X_bx) = bx - _x(X_bx);
	}

	if (std::abs(by) > BIAS_MAX) {
		by = BIAS_MAX * by / std::abs(by);
		dx(X_by) = by - _x(X_by);
	}

	if (std::abs(bz) > BIAS_MAX) {
		bz = BIAS_MAX * bz / std::abs(bz);
		dx(X_bz) = bz - _x(X_bz);
	}

	// propagate
	_x += dx;
	Matrix<float, n_x, n_x> dP = (m_A * m_P + m_P * m_A.transpose() +
				      m_B * m_R * m_B.transpose() + m_Q) * getDt();

	// covariance propagation logic
	for (size_t i = 0; i < n_x; i++) {
		if (m_P(i, i) > P_MAX) {
			// if diagonal element greater than max, stop propagating
			dP(i, i) = 0;

			for (size_t j = 0; j < n_x; j++) {
				dP(i, j) = 0;
				dP(j, i) = 0;
			}
		}
	}

	m_P += dP;
	_xLowPass.update(_x);
	_aglLowPass.update(agl());
}

int BlockLocalPositionEstimator::getDelayPeriods(float delay, uint8_t *periods)
{
	float t_delay = 0;
	uint8_t i_hist = 0;

	for (i_hist = 1; i_hist < HIST_LEN; i_hist++) {
		t_delay = 1.0e-6f * (_timeStamp - _tDelay.get(i_hist)(0, 0));

		if (t_delay > delay) {
			break;
		}
	}

	*periods = i_hist;

	if (t_delay > DELAY_MAX) {
		mavlink_log_info(&mavlink_log_pub, "%sdelayed data old: %8.4f", msg_label, double(t_delay));
		return -1;
	}

	return OK;
}

int
BlockLocalPositionEstimator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int
BlockLocalPositionEstimator::task_spawn(int argc, char *argv[])
{
	BlockLocalPositionEstimator *instance = new BlockLocalPositionEstimator();

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
BlockLocalPositionEstimator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Attitude and position estimator using an Extended Kalman Filter.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("local_position_estimator", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int local_position_estimator_main(int argc, char *argv[])
{
	return BlockLocalPositionEstimator::main(argc, argv);
}
