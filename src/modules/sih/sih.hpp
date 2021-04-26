/****************************************************************************
*
*   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
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

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <matrix/matrix/math.hpp>   // matrix, vectors, dcm, quaterions
#include <conversion/rotation.h>    // math::radians,
#include <lib/ecl/geo/geo.h>        // to get the physical constants
#include <drivers/drv_hrt.h>        // to get the real time
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/barometer/PX4Barometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <lib/perf/perf_counter.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_angular_velocity.h>   // to publish groundtruth
#include <uORB/topics/vehicle_attitude.h>           // to publish groundtruth
#include <uORB/topics/vehicle_global_position.h>    // to publish groundtruth
#include <uORB/topics/distance_sensor.h>

using namespace time_literals;

class Sih : public ModuleBase<Sih>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	Sih();
	~Sih() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	static float generate_wgn();    // generate white Gaussian noise sample

	// generate white Gaussian noise sample as a 3D vector with specified std
	static matrix::Vector3f noiseGauss3f(float stdx, float stdy, float stdz);

	bool init();

private:
	void Run() override;

	void parameters_updated();

	// simulated sensor instances
	PX4Accelerometer _px4_accel{1310988}; // 1310988: DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
	PX4Gyroscope     _px4_gyro{1310988};  // 1310988: DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
	PX4Magnetometer  _px4_mag{197388};    //  197388: DRV_MAG_DEVTYPE_MAGSIM, BUS: 3, ADDR: 1, TYPE: SIMULATION
	PX4Barometer     _px4_baro{6620172};  // 6620172: DRV_BARO_DEVTYPE_BAROSIM, BUS: 1, ADDR: 4, TYPE: SIMULATION

	// to publish the gps position
	sensor_gps_s			_sensor_gps{};
	uORB::Publication<sensor_gps_s>	_sensor_gps_pub{ORB_ID(sensor_gps)};

	// to publish the distance sensor
	distance_sensor_s                    _distance_snsr{};
	uORB::Publication<distance_sensor_s> _distance_snsr_pub{ORB_ID(distance_sensor)};

	// angular velocity groundtruth
	vehicle_angular_velocity_s			_vehicle_angular_velocity_gt{};
	uORB::Publication<vehicle_angular_velocity_s>	_vehicle_angular_velocity_gt_pub{ORB_ID(vehicle_angular_velocity_groundtruth)};

	// attitude groundtruth
	vehicle_attitude_s				_att_gt{};
	uORB::Publication<vehicle_attitude_s>		_att_gt_pub{ORB_ID(vehicle_attitude_groundtruth)};

	// global position groundtruth
	vehicle_global_position_s			_gpos_gt{};
	uORB::Publication<vehicle_global_position_s>	_gpos_gt_pub{ORB_ID(vehicle_global_position_groundtruth)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _actuator_out_sub{ORB_ID(actuator_outputs)};

	// hard constants
	static constexpr uint16_t NB_MOTORS = 4;
	static constexpr float T1_C = 15.0f;                        // ground temperature in celcius
	static constexpr float T1_K = T1_C - CONSTANTS_ABSOLUTE_NULL_CELSIUS;   // ground temperature in Kelvin
	static constexpr float TEMP_GRADIENT  = -6.5f / 1000.0f;    // temperature gradient in degrees per metre

	void init_variables();
	void gps_fix();
	void gps_no_fix();
	void read_motors();
	void generate_force_and_torques();
	void equations_of_motion();
	void reconstruct_sensors_signals();
	void send_gps();
	void send_dist_snsr();
	void publish_sih();

	perf_counter_t  _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t  _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": cycle interval")};

	hrt_abstime _last_run{0};
	hrt_abstime _baro_time{0};
	hrt_abstime _gps_time{0};
	hrt_abstime _mag_time{0};
	hrt_abstime _gt_time{0};
	hrt_abstime _dist_snsr_time{0};
	hrt_abstime _now{0};
	float       _dt{0};         // sampling time [s]
	bool        _grounded{true};// whether the vehicle is on the ground

	matrix::Vector3f    _T_B;           // thrust force in body frame [N]
	matrix::Vector3f    _Fa_I;          // aerodynamic force in inertial frame [N]
	matrix::Vector3f    _Mt_B;          // thruster moments in the body frame [Nm]
	matrix::Vector3f    _Ma_B;          // aerodynamic moments in the body frame [Nm]
	matrix::Vector3f    _p_I;           // inertial position [m]
	matrix::Vector3f    _v_I;           // inertial velocity [m/s]
	matrix::Vector3f    _v_B;           // body frame velocity [m/s]
	matrix::Vector3f    _p_I_dot;       // inertial position differential
	matrix::Vector3f    _v_I_dot;       // inertial velocity differential
	matrix::Quatf       _q;             // quaternion attitude
	matrix::Dcmf        _C_IB;          // body to inertial transformation
	matrix::Vector3f    _w_B;           // body rates in body frame [rad/s]
	matrix::Quatf       _q_dot;         // quaternion differential
	matrix::Vector3f    _w_B_dot;       // body rates differential
	float       _u[NB_MOTORS];  // thruster signals


	// sensors reconstruction
	matrix::Vector3f    _acc;
	matrix::Vector3f    _mag;
	matrix::Vector3f    _gyro;
	matrix::Vector3f    _gps_vel;
	double      _gps_lat, _gps_lat_noiseless;
	double      _gps_lon, _gps_lon_noiseless;
	float       _gps_alt, _gps_alt_noiseless;
	float       _baro_p_mBar;   // reconstructed (simulated) pressure in mBar
	float       _baro_temp_c;   // reconstructed (simulated) barometer temperature in celcius

	// parameters
	float _MASS, _T_MAX, _Q_MAX, _L_ROLL, _L_PITCH, _KDV, _KDW, _H0, _T_TAU;
	double _LAT0, _LON0, _COS_LAT0;
	matrix::Vector3f _W_I;  // weight of the vehicle in inertial frame [N]
	matrix::Matrix3f _I;    // vehicle inertia matrix
	matrix::Matrix3f _Im1;  // inverse of the intertia matrix
	matrix::Vector3f _mu_I; // NED magnetic field in inertial frame [G]

	int _gps_used;
	float _baro_offset_m, _mag_offset_x, _mag_offset_y, _mag_offset_z;
	float _distance_snsr_min, _distance_snsr_max, _distance_snsr_override;

	// parameters defined in sih_params.c
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::IMU_GYRO_RATEMAX>) _imu_gyro_ratemax,

		(ParamFloat<px4::params::SIH_MASS>) _sih_mass,
		(ParamFloat<px4::params::SIH_IXX>) _sih_ixx,
		(ParamFloat<px4::params::SIH_IYY>) _sih_iyy,
		(ParamFloat<px4::params::SIH_IZZ>) _sih_izz,
		(ParamFloat<px4::params::SIH_IXY>) _sih_ixy,
		(ParamFloat<px4::params::SIH_IXZ>) _sih_ixz,
		(ParamFloat<px4::params::SIH_IYZ>) _sih_iyz,
		(ParamFloat<px4::params::SIH_T_MAX>) _sih_t_max,
		(ParamFloat<px4::params::SIH_Q_MAX>) _sih_q_max,
		(ParamFloat<px4::params::SIH_L_ROLL>) _sih_l_roll,
		(ParamFloat<px4::params::SIH_L_PITCH>) _sih_l_pitch,
		(ParamFloat<px4::params::SIH_KDV>) _sih_kdv,
		(ParamFloat<px4::params::SIH_KDW>) _sih_kdw,
		(ParamInt<px4::params::SIH_LOC_LAT0>) _sih_lat0,
		(ParamInt<px4::params::SIH_LOC_LON0>) _sih_lon0,
		(ParamFloat<px4::params::SIH_LOC_H0>) _sih_h0,
		(ParamFloat<px4::params::SIH_LOC_MU_X>) _sih_mu_x,
		(ParamFloat<px4::params::SIH_LOC_MU_Y>) _sih_mu_y,
		(ParamFloat<px4::params::SIH_LOC_MU_Z>) _sih_mu_z,
		(ParamInt<px4::params::SIH_GPS_USED>) _sih_gps_used,
		(ParamFloat<px4::params::SIH_BARO_OFFSET>) _sih_baro_offset,
		(ParamFloat<px4::params::SIH_MAG_OFFSET_X>) _sih_mag_offset_x,
		(ParamFloat<px4::params::SIH_MAG_OFFSET_Y>) _sih_mag_offset_y,
		(ParamFloat<px4::params::SIH_MAG_OFFSET_Z>) _sih_mag_offset_z,
		(ParamFloat<px4::params::SIH_DISTSNSR_MIN>) _sih_distance_snsr_min,
		(ParamFloat<px4::params::SIH_DISTSNSR_MAX>) _sih_distance_snsr_max,
		(ParamFloat<px4::params::SIH_DISTSNSR_OVR>) _sih_distance_snsr_override,
		(ParamFloat<px4::params::SIH_T_TAU>) _sih_thrust_tau
	)
};
