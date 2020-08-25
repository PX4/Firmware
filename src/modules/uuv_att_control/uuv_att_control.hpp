/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

/**
 *
 * This module is a modification of the rover attitide control module and is designed for the
 * TUHH hippocampus.
 *
 * All the acknowledgments and credits for the fw wing app are reported in those files.
 *
 * @author Daniel Duecker <daniel.duecker@tuhh.de>
 * @author Philipp Hastedt <philipp.hastedt@tuhh.de>
 * @author Tim Hansen <t.hansen@tuhh.de>
 */



#include <float.h>

#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <lib/pid/pid.h>
#include <matrix/math.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_controller_status.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/ekf2_timestamps.h>
#include <uORB/uORB.h>




using matrix::Eulerf;
using matrix::Quatf;
using matrix::Matrix3f;
using matrix::Vector3f;
using matrix::Dcmf;

using uORB::SubscriptionData;

class UUVAttitudeControl: public ModuleBase<UUVAttitudeControl>, public ModuleParams
{
public:
	UUVAttitudeControl();
	~UUVAttitudeControl();

	UUVAttitudeControl(const UUVAttitudeControl &) = delete;
	UUVAttitudeControl operator=(const UUVAttitudeControl &other) = delete;


	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static UUVAttitudeControl *instantiate(int argc, char *argv[]);

	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;


//	int start();
//	bool task_running() { return _task_running; }

private:
	uORB::Publication<position_controller_status_s>	_pos_ctrl_status_pub{ORB_ID(position_controller_status)};
	uORB::Publication<actuator_controls_s>		    _actuator_controls_pub{ORB_ID(actuator_controls_0)};

	uORB::Subscription	_parameter_update_sub{ORB_ID(parameter_update)};

	int	_vehicle_attitude_sp_sub{-1};	/**< vehicle attitude setpoint */
	int	_battery_status_sub{-1};	/**< battery status subscription */
	int	_vehicle_attitude_sub{-1};	/**< control state subscription */
	int	_angular_velocity_sub{-1};	/**< vehicle angular velocity subscription */
	int	_local_pos_sub{-1};		/**< local position subscription */
	int	_manual_control_sub{-1};	/**< notification of manual control updates */
	int	_vcontrol_mode_sub{-1};		/**< vehicle status subscription */
	int	_sensor_combined_sub{-1};	/**< sensor combined subscription */

	actuator_controls_s		_actuators {};		/**< actuator control inputs */
	manual_control_setpoint_s	_manual {};		/**< r/c channel data */
	vehicle_attitude_s		_vehicle_attitude {};	/**< control state */
	vehicle_angular_velocity_s	_angular_velocity{};	/**< angular velocity */
	vehicle_attitude_setpoint_s	_vehicle_attitude_sp {};/**< vehicle attitude setpoint */
	vehicle_control_mode_s		_vcontrol_mode {};	/**< vehicle control mode */
	sensor_combined_s		_sensor_combined{};
	vehicle_local_position_s	_local_pos{};		/**< vehicle local position */


	SubscriptionData<vehicle_acceleration_s>		_vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};
	hrt_abstime _control_position_last_called{0}; 	/**<last call of control_position  */
	perf_counter_t	_loop_perf;			/**< loop performance counter */


	// estimator reset counters
	uint8_t _pos_reset_counter{0};		// captures the number of times the estimator has reset the horizontal position


	bool _debug{false};	/**< if set to true, print debug output */
	int loop_counter = 0;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::UUV_ROLL_P>) _param_roll_p,
		(ParamFloat<px4::params::UUV_ROLL_D>) _param_roll_d,
		(ParamFloat<px4::params::UUV_PITCH_P>) _param_pitch_p,
		(ParamFloat<px4::params::UUV_PITCH_D>) _param_pitch_d,
		(ParamFloat<px4::params::UUV_YAW_P>) _param_yaw_p,
		(ParamFloat<px4::params::UUV_YAW_D>) _param_yaw_d,
		// control/input modes
		(ParamInt<px4::params::UUV_INPUT_MODE>) _param_input_mode,
		// direct access to inputs
		(ParamFloat<px4::params::UUV_DIRCT_ROLL>) _param_direct_roll,
		(ParamFloat<px4::params::UUV_DIRCT_PITCH>) _param_direct_pitch,
		(ParamFloat<px4::params::UUV_DIRCT_YAW>) _param_direct_yaw,
		(ParamFloat<px4::params::UUV_DIRCT_THRUST>) _param_direct_thrust
	)

	/**
	 * Update our local parameter cache.
	 */
	void	parameters_update(bool force = false);

	void	manual_control_setpoint_poll();
	void	position_setpoint_triplet_poll();
	void	vehicle_control_mode_poll();
	void 	vehicle_attitude_poll();
	void	vehicle_attitude_setpoint_poll();
	void	vehicle_local_position_poll();

	/**
	 * Control Attitude
	 */
	void control_attitude_geo(const vehicle_attitude_s &att, const vehicle_attitude_setpoint_s &att_sp);

	void control_attitude_pid(const vehicle_attitude_s &att, const vehicle_attitude_setpoint_s &att_sp, float deltaT);
	void constrain_actuator_commands(float roll_u, float pitch_u, float yaw_u, float thrust_u);


};
