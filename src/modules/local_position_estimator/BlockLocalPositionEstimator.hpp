#pragma once

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <lib/controllib/blocks.hpp>
#include <lib/ecl/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <matrix/Matrix.hpp>

// uORB Subscriptions
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_odometry.h>

// uORB Publications
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/estimator_states.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/estimator_innovations.h>

using namespace matrix;
using namespace control;
using namespace time_literals;

static const float DELAY_MAX = 0.5f;	// seconds
static const float HIST_STEP = 0.05f;	// 20 hz
static const float BIAS_MAX = 1e-1f;
static const size_t HIST_LEN = 10;	// DELAY_MAX / HIST_STEP;
static const size_t N_DIST_SUBS = 4;

// for fault detection
// chi squared distribution, false alarm probability 0.0001
// see fault_table.py
// note skip 0 index so we can use degree of freedom as index
static const float BETA_TABLE[7] = {0,
				    8.82050518214,
				    12.094592431,
				    13.9876612368,
				    16.0875642296,
				    17.8797700658,
				    19.6465647819,
				   };

class BlockLocalPositionEstimator : public ModuleBase<BlockLocalPositionEstimator>, public ModuleParams,
	public px4::WorkItem, public control::SuperBlock
{
// dynamics:
//
//	x(+) = A * x(-) + B * u(+)
//	y_i = C_i*x
//
// kalman filter
//
//	E[xx'] = P
//	E[uu'] = W
//	E[y_iy_i'] = R_i
//
//	prediction
//		x(+|-) = A*x(-|-) + B*u(+)
//		P(+|-) = A*P(-|-)*A' + B*W*B'
//
//	correction
//		x(+|+) =  x(+|-) + K_i * (y_i - H_i * x(+|-) )
//
//
// input:
//      ax, ay, az (acceleration NED)
//
// states:
//      px, py, pz , ( position NED, m)
//      vx, vy, vz ( vel NED, m/s),
//      bx, by, bz ( accel bias, m/s^2)
//      tz (terrain altitude, ASL, m)
//
// measurements:
//
//      sonar: pz (measured d*cos(phi)*cos(theta))
//
//      baro: pz
//
//      flow: vx, vy (flow is in body x, y frame)
//
//      gps: px, py, pz, vx, vy, vz (flow is in body x, y frame)
//
//      lidar: pz (actual measured d*cos(phi)*cos(theta))
//
//      vision: px, py, pz, vx, vy, vz
//
//      mocap: px, py, pz
//
//      land (detects when landed)): pz (always measures agl = 0)
//
public:

	BlockLocalPositionEstimator();
	~BlockLocalPositionEstimator() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:

	// constants
	enum {X_x = 0, X_y, X_z, X_vx, X_vy, X_vz, X_bx, X_by, X_bz, X_tz, n_x};
	enum {U_ax = 0, U_ay, U_az, n_u};
	enum {Y_baro_z = 0, n_y_baro};
	enum {Y_lidar_z = 0, n_y_lidar};
	enum {Y_flow_vx = 0, Y_flow_vy, n_y_flow};
	enum {Y_sonar_z = 0, n_y_sonar};
	enum {Y_gps_x = 0, Y_gps_y, Y_gps_z, Y_gps_vx, Y_gps_vy, Y_gps_vz, n_y_gps};
	enum {Y_vision_x = 0, Y_vision_y, Y_vision_z, n_y_vision};
	enum {Y_mocap_x = 0, Y_mocap_y, Y_mocap_z, n_y_mocap};
	enum {Y_land_vx = 0, Y_land_vy, Y_land_agl, n_y_land};
	enum {Y_target_x = 0, Y_target_y, n_y_target};
	enum {
		FUSE_GPS = 1 << 0,
		FUSE_FLOW = 1 << 1,
		FUSE_VIS_POS = 1 << 2,
		FUSE_LAND_TARGET = 1 << 3,
		FUSE_LAND = 1 << 4,
		FUSE_PUB_AGL_Z = 1 << 5,
		FUSE_FLOW_GYRO_COMP = 1 << 6,
		FUSE_BARO = 1 << 7
	};

	enum sensor_t {
		SENSOR_BARO = 1 << 0,
		SENSOR_GPS = 1 << 1,
		SENSOR_LIDAR = 1 << 2,
		SENSOR_FLOW = 1 << 3,
		SENSOR_SONAR = 1 << 4,
		SENSOR_VISION = 1 << 5,
		SENSOR_MOCAP = 1 << 6,
		SENSOR_LAND = 1 << 7,
		SENSOR_LAND_TARGET = 1 << 8,
	};

	enum estimate_t {
		EST_XY = 1 << 0,
		EST_Z = 1 << 1,
		EST_TZ = 1 << 2,
	};

	void Run() override;

	// methods
	// ----------------------------
	//
	Vector<float, n_x> dynamics(
		float t,
		const Vector<float, n_x> &x,
		const Vector<float, n_u> &u);
	void initP();
	void initSS();
	void updateSSStates();
	void updateSSParams();

	// predict the next state
	void predict(const sensor_combined_s &imu);

	// lidar
	int  lidarMeasure(Vector<float, n_y_lidar> &y);
	void lidarCorrect();
	void lidarInit();
	void lidarCheckTimeout();

	// sonar
	int  sonarMeasure(Vector<float, n_y_sonar> &y);
	void sonarCorrect();
	void sonarInit();
	void sonarCheckTimeout();

	// baro
	int  baroMeasure(Vector<float, n_y_baro> &y);
	void baroCorrect();
	void baroInit();
	void baroCheckTimeout();

	// gps
	int  gpsMeasure(Vector<double, n_y_gps> &y);
	void gpsCorrect();
	void gpsInit();
	void gpsCheckTimeout();

	// flow
	int  flowMeasure(Vector<float, n_y_flow> &y);
	void flowCorrect();
	void flowInit();
	void flowCheckTimeout();

	// vision
	int  visionMeasure(Vector<float, n_y_vision> &y);
	void visionCorrect();
	void visionInit();
	void visionCheckTimeout();

	// mocap
	int  mocapMeasure(Vector<float, n_y_mocap> &y);
	void mocapCorrect();
	void mocapInit();
	void mocapCheckTimeout();

	// land
	int  landMeasure(Vector<float, n_y_land> &y);
	void landCorrect();
	void landInit();
	void landCheckTimeout();

	// landing target
	int  landingTargetMeasure(Vector<float, n_y_target> &y);
	void landingTargetCorrect();
	void landingTargetInit();
	void landingTargetCheckTimeout();

	// timeouts
	void checkTimeouts();

	// misc
	inline float agl()
	{
		return _x(X_tz) - _x(X_z);
	}
	bool landed();
	int getDelayPeriods(float delay, uint8_t *periods);

	// publications
	void publishLocalPos();
	void publishGlobalPos();
	void publishOdom();
	void publishEstimatorStatus();
	void publishEk2fTimestamps();

	// attributes
	// ----------------------------

	// subscriptions
	uORB::SubscriptionCallbackWorkItem _sensors_sub{this, ORB_ID(sensor_combined)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::SubscriptionData<actuator_armed_s> _sub_armed{ORB_ID(actuator_armed)};
	uORB::SubscriptionData<vehicle_land_detected_s> _sub_land{ORB_ID(vehicle_land_detected)};
	uORB::SubscriptionData<vehicle_attitude_s> _sub_att{ORB_ID(vehicle_attitude)};
	uORB::SubscriptionData<vehicle_angular_velocity_s> _sub_angular_velocity{ORB_ID(vehicle_angular_velocity)};
	uORB::SubscriptionData<optical_flow_s> _sub_flow{ORB_ID(optical_flow)};
	uORB::SubscriptionData<vehicle_gps_position_s> _sub_gps{ORB_ID(vehicle_gps_position)};
	uORB::SubscriptionData<vehicle_odometry_s> _sub_visual_odom{ORB_ID(vehicle_visual_odometry)};
	uORB::SubscriptionData<vehicle_odometry_s> _sub_mocap_odom{ORB_ID(vehicle_mocap_odometry)};
	uORB::SubscriptionData<distance_sensor_s> _sub_dist0{ORB_ID(distance_sensor), 0};
	uORB::SubscriptionData<distance_sensor_s> _sub_dist1{ORB_ID(distance_sensor), 1};
	uORB::SubscriptionData<distance_sensor_s> _sub_dist2{ORB_ID(distance_sensor), 2};
	uORB::SubscriptionData<distance_sensor_s> _sub_dist3{ORB_ID(distance_sensor), 3};
	uORB::SubscriptionData<distance_sensor_s> *_dist_subs[N_DIST_SUBS] {};
	uORB::SubscriptionData<distance_sensor_s> *_sub_lidar{nullptr};
	uORB::SubscriptionData<distance_sensor_s> *_sub_sonar{nullptr};
	uORB::SubscriptionData<landing_target_pose_s> _sub_landing_target_pose{ORB_ID(landing_target_pose)};
	uORB::SubscriptionData<vehicle_air_data_s> _sub_airdata{ORB_ID(vehicle_air_data)};

	// publications
	uORB::PublicationData<vehicle_local_position_s> _pub_lpos{ORB_ID(vehicle_local_position)};
	uORB::PublicationData<vehicle_global_position_s> _pub_gpos{ORB_ID(vehicle_global_position)};
	uORB::PublicationData<vehicle_odometry_s> _pub_odom{ORB_ID(vehicle_odometry)};
	uORB::PublicationData<estimator_states_s> _pub_est_states{ORB_ID(estimator_states)};
	uORB::PublicationData<estimator_status_s> _pub_est_status{ORB_ID(estimator_status)};
	uORB::PublicationData<estimator_innovations_s> _pub_innov{ORB_ID(estimator_innovations)};
	uORB::PublicationData<estimator_innovations_s> _pub_innov_var{ORB_ID(estimator_innovation_variances)};

	// map projection
	struct map_projection_reference_s _map_ref;

	map_projection_reference_s _global_local_proj_ref{};
	float                      _global_local_alt0{NAN};

	// target mode paramters from landing_target_estimator module
	enum TargetMode {
		Target_Moving = 0,
		Target_Stationary = 1
	};

	// flow gyro filter
	BlockHighPass _flow_gyro_x_high_pass;
	BlockHighPass _flow_gyro_y_high_pass;

	// stats
	BlockStats<float, n_y_baro> _baroStats;
	BlockStats<float, n_y_sonar> _sonarStats;
	BlockStats<float, n_y_lidar> _lidarStats;
	BlockStats<float, 1> _flowQStats;
	BlockStats<float, n_y_vision> _visionStats;
	BlockStats<float, n_y_mocap> _mocapStats;
	BlockStats<double, n_y_gps> _gpsStats;
	uint16_t _landCount;

	// low pass
	BlockLowPassVector<float, n_x> _xLowPass;
	BlockLowPass _aglLowPass;

	// delay blocks
	BlockDelay<float, n_x, 1, HIST_LEN> _xDelay;
	BlockDelay<uint64_t, 1, 1, HIST_LEN> _tDelay;

	// misc
	uint64_t _timeStamp;
	uint64_t _time_origin;
	uint64_t _timeStampLastBaro;
	uint64_t _time_last_hist;
	uint64_t _time_last_flow;
	uint64_t _time_last_baro;
	uint64_t _time_last_gps;
	uint64_t _time_last_lidar;
	uint64_t _time_last_sonar;
	uint64_t _time_init_sonar;
	uint64_t _time_last_vision_p;
	uint64_t _time_last_mocap;
	uint64_t _time_last_land;
	uint64_t _time_last_target;

	// reference altitudes
	float _altOrigin;
	bool _altOriginInitialized;
	bool _altOriginGlobal; // true when the altitude of the origin is defined wrt a global reference frame
	float _baroAltOrigin;
	float _gpsAltOrigin;

	// status
	bool _receivedGps;
	bool _lastArmedState;

	// masks
	uint16_t _sensorTimeout;
	uint16_t _sensorFault;
	uint8_t _estimatorInitialized;

	// sensor update flags
	bool _flowUpdated;
	bool _gpsUpdated;
	bool _visionUpdated;
	bool _mocapUpdated;
	bool _lidarUpdated;
	bool _sonarUpdated;
	bool _landUpdated;
	bool _baroUpdated;

	// sensor validation flags
	bool _vision_xy_valid;
	bool _vision_z_valid;
	bool _mocap_xy_valid;
	bool _mocap_z_valid;

	// sensor std deviations
	float _vision_eph;
	float _vision_epv;
	float _mocap_eph;
	float _mocap_epv;

	// local to global coversion related variables
	bool _is_global_cov_init;
	double _ref_lat;
	double _ref_lon;
	float _ref_alt;

	// state space
	Vector<float, n_x>  _x;	// state vector
	Vector<float, n_u>  _u;	// input vector
	Matrix<float, n_x, n_x> m_P;	// state covariance matrix

	matrix::Dcm<float> _R_att;

	Matrix<float, n_x, n_x>  m_A;	// dynamics matrix
	Matrix<float, n_x, n_u>  m_B;	// input matrix
	Matrix<float, n_u, n_u>  m_R;	// input covariance
	Matrix<float, n_x, n_x>  m_Q;	// process noise covariance


	DEFINE_PARAMETERS(
		// general parameters
		(ParamInt<px4::params::LPE_FUSION>) _param_lpe_fusion,
		(ParamFloat<px4::params::LPE_VXY_PUB>) _param_lpe_vxy_pub,
		(ParamFloat<px4::params::LPE_Z_PUB>) _param_lpe_z_pub,

		// sonar parameters
		(ParamFloat<px4::params::LPE_SNR_Z>) _param_lpe_snr_z,
		(ParamFloat<px4::params::LPE_SNR_OFF_Z>) _param_lpe_snr_off_z,

		// lidar parameters
		(ParamFloat<px4::params::LPE_LDR_Z>) _param_lpe_ldr_z,
		(ParamFloat<px4::params::LPE_LDR_OFF_Z>) _param_lpe_ldr_off_z,

		// accel parameters
		(ParamFloat<px4::params::LPE_ACC_XY>) _param_lpe_acc_xy,
		(ParamFloat<px4::params::LPE_ACC_Z>) _param_lpe_acc_z,

		// baro parameters
		(ParamFloat<px4::params::LPE_BAR_Z>) _param_lpe_bar_z,

		// gps parameters
		(ParamFloat<px4::params::LPE_GPS_DELAY>) _param_lpe_gps_delay,
		(ParamFloat<px4::params::LPE_GPS_XY>) _param_lpe_gps_xy,
		(ParamFloat<px4::params::LPE_GPS_Z>) _param_lpe_gps_z,
		(ParamFloat<px4::params::LPE_GPS_VXY>) _param_lpe_gps_vxy,
		(ParamFloat<px4::params::LPE_GPS_VZ>) _param_lpe_gps_vz,
		(ParamFloat<px4::params::LPE_EPH_MAX>) _param_lpe_eph_max,
		(ParamFloat<px4::params::LPE_EPV_MAX>) _param_lpe_epv_max,

		// vision parameters
		(ParamFloat<px4::params::LPE_VIS_XY>) _param_lpe_vis_xy,
		(ParamFloat<px4::params::LPE_VIS_Z>) _param_lpe_vis_z,
		(ParamFloat<px4::params::LPE_VIS_DELAY>) _param_lpe_vis_delay,

		// mocap parameters
		(ParamFloat<px4::params::LPE_VIC_P>) _param_lpe_vic_p,

		// flow parameters
		(ParamFloat<px4::params::LPE_FLW_OFF_Z>) _param_lpe_flw_off_z,
		(ParamFloat<px4::params::LPE_FLW_SCALE>) _param_lpe_flw_scale,
		(ParamInt<px4::params::LPE_FLW_QMIN>) _param_lpe_flw_qmin,
		(ParamFloat<px4::params::LPE_FLW_R>) _param_lpe_flw_r,
		(ParamFloat<px4::params::LPE_FLW_RR>) _param_lpe_flw_rr,

		// land parameters
		(ParamFloat<px4::params::LPE_LAND_Z>) _param_lpe_land_z,
		(ParamFloat<px4::params::LPE_LAND_VXY>) _param_lpe_land_vxy,

		// process noise
		(ParamFloat<px4::params::LPE_PN_P>) _param_lpe_pn_p,
		(ParamFloat<px4::params::LPE_PN_V>) _param_lpe_pn_v,
		(ParamFloat<px4::params::LPE_PN_B>) _param_lpe_pn_b,
		(ParamFloat<px4::params::LPE_PN_T>) _param_lpe_pn_t,
		(ParamFloat<px4::params::LPE_T_MAX_GRADE>) _param_lpe_t_max_grade,

		(ParamFloat<px4::params::LPE_LT_COV>) _param_lpe_lt_cov,
		(ParamInt<px4::params::LTEST_MODE>) _param_ltest_mode,

		// init origin
		(ParamInt<px4::params::LPE_FAKE_ORIGIN>) _param_lpe_fake_origin,
		(ParamFloat<px4::params::LPE_LAT>) _param_lpe_lat,
		(ParamFloat<px4::params::LPE_LON>) _param_lpe_lon
	)

};
