/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * Implementation of a generic attitude controller based on classic orthogonal PIDs.
 *
 * @author Lorenz Meier 	<lm@inf.ethz.ch>
 * @author Thomas Gubler 	<thomasgubler@gmail.com>
 * @author Roman Bapst		<bapstr@ethz.ch>
 *
 */

#ifndef FIXEDWINGATTITUDECONTROL_HPP_
#define FIXEDWINGATTITUDECONTROL_HPP_

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include <drivers/drv_accel.h>
#include <drivers/drv_hrt.h>
#include <ecl/attitude_fw/ecl_pitch_controller.h>
#include <ecl/attitude_fw/ecl_roll_controller.h>
#include <ecl/attitude_fw/ecl_wheel_controller.h>
#include <ecl/attitude_fw/ecl_yaw_controller.h>
#include <mathlib/mathlib.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/fw_virtual_attitude_setpoint.h>
#include <uORB/topics/fw_virtual_rates_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/uORB.h>

class FixedwingAttitudeControl
{
public:
	/**
	 * Constructor
	 */
	FixedwingAttitudeControl();

	/**
	 * Destructor, also kills the main task.
	 */
	~FixedwingAttitudeControl();

	FixedwingAttitudeControl(FixedwingAttitudeControl *att_controller);
	FixedwingAttitudeControl(const FixedwingAttitudeControl &) = delete;
	FixedwingAttitudeControl &operator=(const FixedwingAttitudeControl &) = delete;

	/**
	 * Start the main task.
	 *
	 * @return	PX4_OK on success.
	 */
	int		start();

	/**
	 * Task status
	 *
	 * @return	true if the mainloop is running
	 */
	bool		task_running() { return _task_running; }

private:

	bool		_task_should_exit;		/**< if true, attitude control task should exit */
	bool		_task_running;			/**< if true, task is running in its mainloop */
	int		_control_task;			/**< task handle */

	int		_accel_sub;			/**< accelerometer subscription */
	int		_att_sp_sub;			/**< vehicle attitude setpoint */
	int		_battery_status_sub;		/**< battery status subscription */
	int		_ctrl_state_sub;		/**< control state subscription */
	int		_global_pos_sub;		/**< global position subscription */
	int		_manual_sub;			/**< notification of manual control updates */
	int		_params_sub;			/**< notification of parameter updates */
	int		_vcontrol_mode_sub;		/**< vehicle status subscription */
	int		_vehicle_land_detected_sub;	/**< vehicle land detected subscription */
	int		_vehicle_status_sub;		/**< vehicle status subscription */

	orb_advert_t	_rate_sp_pub;			/**< rate setpoint publication */
	orb_advert_t	_attitude_sp_pub;		/**< attitude setpoint point */
	orb_advert_t	_actuators_0_pub;		/**< actuator control group 0 setpoint */
	orb_advert_t	_actuators_2_pub;		/**< actuator control group 1 setpoint (Airframe) */

	orb_id_t _rates_sp_id;	// pointer to correct rates setpoint uORB metadata structure
	orb_id_t _actuators_id;	// pointer to correct actuator controls0 uORB metadata structure
	orb_id_t _attitude_setpoint_id;

	struct accel_report				_accel;			/**< body frame accelerations */
	struct actuator_controls_s			_actuators;		/**< actuator control inputs */
	struct actuator_controls_s			_actuators_airframe;	/**< actuator control inputs */
	struct battery_status_s				_battery_status;	/**< battery status */
	struct control_state_s				_ctrl_state;	/**< control state */
	struct manual_control_setpoint_s		_manual;		/**< r/c channel data */
	struct vehicle_attitude_setpoint_s		_att_sp;		/**< vehicle attitude setpoint */
	struct vehicle_control_mode_s			_vcontrol_mode;		/**< vehicle control mode */
	struct vehicle_global_position_s		_global_pos;		/**< global position */
	struct vehicle_land_detected_s			_vehicle_land_detected;	/**< vehicle land detected */
	struct vehicle_rates_setpoint_s			_rates_sp;	/* attitude rates setpoint */
	struct vehicle_status_s				_vehicle_status;	/**< vehicle status */

	perf_counter_t	_loop_perf;			/**< loop performance counter */
	perf_counter_t	_nonfinite_input_perf;		/**< performance counter for non finite input */
	perf_counter_t	_nonfinite_output_perf;		/**< performance counter for non finite output */

	bool		_debug;				/**< if set to true, print debug output */

	float _flaps_applied;
	float _flaperons_applied;

	struct {
		float p_tc;
		float p_p;
		float p_i;
		float p_ff;
		float p_rmax_pos;
		float p_rmax_neg;
		float p_integrator_max;

		float r_tc;
		float r_p;
		float r_i;
		float r_ff;
		float r_integrator_max;
		float r_rmax;

		float y_p;
		float y_i;
		float y_ff;
		float y_integrator_max;
		float y_coordinated_min_speed;
		int32_t y_coordinated_method;
		float y_rmax;

		float w_p;
		float w_i;
		float w_ff;
		float w_integrator_max;
		float w_rmax;

		float airspeed_min;
		float airspeed_trim;
		float airspeed_max;

		float trim_roll;
		float trim_pitch;
		float trim_yaw;

		float rollsp_offset_deg;		/**< Roll Setpoint Offset in deg */
		float pitchsp_offset_deg;		/**< Pitch Setpoint Offset in deg */
		float rollsp_offset_rad;		/**< Roll Setpoint Offset in rad */
		float pitchsp_offset_rad;		/**< Pitch Setpoint Offset in rad */

		float man_roll_max;				/**< Max Roll in rad */
		float man_pitch_max;			/**< Max Pitch in rad */
		float man_roll_scale;			/**< scale factor applied to roll actuator control in pure manual mode */
		float man_pitch_scale;			/**< scale factor applied to pitch actuator control in pure manual mode */
		float man_yaw_scale; 			/**< scale factor applied to yaw actuator control in pure manual mode */

		float flaps_scale;				/**< Scale factor for flaps */
		float flaperon_scale;			/**< Scale factor for flaperons */

		int vtol_type;					/**< VTOL type: 0 = tailsitter, 1 = tiltrotor */

		int bat_scale_en;			/**< Battery scaling enabled */

	} _parameters;			/**< local copies of interesting parameters */

	struct {
		param_t p_tc;
		param_t p_p;
		param_t p_i;
		param_t p_ff;
		param_t p_rmax_pos;
		param_t p_rmax_neg;
		param_t p_integrator_max;

		param_t r_tc;
		param_t r_p;
		param_t r_i;
		param_t r_ff;
		param_t r_integrator_max;
		param_t r_rmax;

		param_t y_p;
		param_t y_i;
		param_t y_ff;
		param_t y_integrator_max;
		param_t y_coordinated_min_speed;
		param_t y_coordinated_method;
		param_t y_rmax;

		param_t w_p;
		param_t w_i;
		param_t w_ff;
		param_t w_integrator_max;
		param_t w_rmax;

		param_t airspeed_min;
		param_t airspeed_trim;
		param_t airspeed_max;

		param_t trim_roll;
		param_t trim_pitch;
		param_t trim_yaw;

		param_t rollsp_offset_deg;
		param_t pitchsp_offset_deg;

		param_t man_roll_max;
		param_t man_pitch_max;
		param_t man_roll_scale;
		param_t man_pitch_scale;
		param_t man_yaw_scale;

		param_t flaps_scale;
		param_t flaperon_scale;

		param_t vtol_type;

		param_t bat_scale_en;

	} _parameter_handles;		/**< handles for interesting parameters */

	// Rotation matrix and euler angles to extract from control state
	math::Matrix<3, 3> _R;
	float _roll;
	float _pitch;
	float _yaw;

	ECL_RollController				_roll_ctrl;
	ECL_PitchController				_pitch_ctrl;
	ECL_YawController				_yaw_ctrl;
	ECL_WheelController			_wheel_ctrl;

	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Update control outputs
	 *
	 */
	void		control_update();

	/**
	 * Check for changes in vehicle control mode.
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for changes in manual inputs.
	 */
	void		vehicle_manual_poll();

	/**
	 * Check for accel updates.
	 */
	void		vehicle_accel_poll();

	/**
	 * Check for set triplet updates.
	 */
	void		vehicle_setpoint_poll();

	/**
	 * Check for global position updates.
	 */
	void		global_pos_poll();

	/**
	 * Check for vehicle status updates.
	 */
	void		vehicle_status_poll();

	/**
	 * Check for vehicle land detected updates.
	 */
	void		vehicle_land_detected_poll();

	/**
	 * Check for battery status updates.
	 */
	void		battery_status_poll();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude controller collection task.
	 */
	void		task_main();

};

namespace att_control
{
extern FixedwingAttitudeControl	*g_control;
}

#endif /* FIXEDWINGATTITUDECONTROL_HPP_ */
