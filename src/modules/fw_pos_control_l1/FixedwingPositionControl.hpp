/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file fw_pos_control_l1_main.hpp
 * Implementation of a generic position controller based on the L1 norm. Outputs a bank / roll
 * angle, equivalent to a lateral motion (for copters and rovers).
 *
 * The implementation for the controllers is in the ECL library. This class only
 * interfaces to the library.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 * @author Andreas Antener <andreas@uaventure.com>
 */

#ifndef FIXEDWINGPOSITIONCONTROL_HPP_
#define FIXEDWINGPOSITIONCONTROL_HPP_

#include "launchdetection/LaunchDetector.h"
#include "runway_takeoff/RunwayTakeoff.h"

#include <float.h>

#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <lib/l1/ECL_L1_Pos_Controller.hpp>
#include <lib/tecs/TECS.hpp>
#include <lib/landing_slope/Landingslope.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_controller_landing_status.h>
#include <uORB/topics/position_controller_status.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/tecs_status.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/uORB.h>
#include <vtol_att_control/vtol_type.h>

using math::constrain;
using math::max;
using math::min;
using math::radians;

using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector3f;
using matrix::wrap_pi;

using uORB::SubscriptionData;

using namespace launchdetection;
using namespace runwaytakeoff;
using namespace time_literals;

static constexpr float HDG_HOLD_DIST_NEXT =
	3000.0f; // initial distance of waypoint in front of plane in heading hold mode
static constexpr float HDG_HOLD_REACHED_DIST =
	1000.0f; // distance (plane to waypoint in front) at which waypoints are reset in heading hold mode
static constexpr float HDG_HOLD_SET_BACK_DIST = 100.0f; // distance by which previous waypoint is set behind the plane
static constexpr float HDG_HOLD_YAWRATE_THRESH = 0.15f;	// max yawrate at which plane locks yaw for heading hold mode
static constexpr float HDG_HOLD_MAN_INPUT_THRESH =
	0.01f; // max manual roll/yaw input from user which does not change the locked heading

static constexpr hrt_abstime T_ALT_TIMEOUT = 1_s; // time after which we abort landing if terrain estimate is not valid

static constexpr float THROTTLE_THRESH =
	0.05f;	///< max throttle from user which will not lead to motors spinning up in altitude controlled modes
static constexpr float MANUAL_THROTTLE_CLIMBOUT_THRESH =
	0.85f; ///< a throttle / pitch input above this value leads to the system switching to climbout mode
static constexpr float ALTHOLD_EPV_RESET_THRESH = 5.0f;

class FixedwingPositionControl final : public ModuleBase<FixedwingPositionControl>, public ModuleParams,
	public px4::WorkItem
{
public:
	FixedwingPositionControl(bool vtol = false);
	~FixedwingPositionControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	orb_advert_t	_mavlink_log_pub{nullptr};

	uORB::SubscriptionCallbackWorkItem _local_pos_sub{this, ORB_ID(vehicle_local_position)};

	uORB::Subscription _control_mode_sub{ORB_ID(vehicle_control_mode)};		///< control mode subscription
	uORB::Subscription _global_pos_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};	///< notification of manual control updates
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};		///< notification of parameter updates
	uORB::Subscription _pos_sp_triplet_sub{ORB_ID(position_setpoint_triplet)};
	uORB::Subscription _vehicle_air_data_sub{ORB_ID(vehicle_air_data)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};		///< vehicle attitude subscription
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};		///< vehicle command subscription
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};	///< vehicle land detected subscription
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};			///< vehicle status subscription
	uORB::SubscriptionData<vehicle_angular_velocity_s>	_vehicle_rates_sub{ORB_ID(vehicle_angular_velocity)};

	uORB::Publication<vehicle_attitude_setpoint_s>		_attitude_sp_pub;
	uORB::Publication<position_controller_status_s>		_pos_ctrl_status_pub{ORB_ID(position_controller_status)};			///< navigation capabilities publication
	uORB::Publication<position_controller_landing_status_s>	_pos_ctrl_landing_status_pub{ORB_ID(position_controller_landing_status)};	///< landing status publication
	uORB::Publication<tecs_status_s>			_tecs_status_pub{ORB_ID(tecs_status)};						///< TECS status publication

	manual_control_setpoint_s	_manual_control_setpoint {};			///< r/c channel data
	position_setpoint_triplet_s	_pos_sp_triplet {};		///< triplet of mission items
	vehicle_attitude_s		_att {};			///< vehicle attitude setpoint
	vehicle_attitude_setpoint_s	_att_sp {};			///< vehicle attitude setpoint
	vehicle_command_s		_vehicle_command {};		///< vehicle commands
	vehicle_control_mode_s		_control_mode {};		///< control mode
	vehicle_local_position_s	_local_pos {};			///< vehicle local position
	vehicle_land_detected_s		_vehicle_land_detected {};	///< vehicle land detected
	vehicle_status_s		_vehicle_status {};		///< vehicle status

	SubscriptionData<airspeed_validated_s>			_airspeed_validated_sub{ORB_ID(airspeed_validated)};
	SubscriptionData<vehicle_acceleration_s>	_vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};

	double _current_latitude{0};
	double _current_longitude{0};
	float _current_altitude{0.f};

	perf_counter_t	_loop_perf;				///< loop performance counter

	float	_hold_alt{0.0f};				///< hold altitude for altitude mode
	float	_takeoff_ground_alt{0.0f};			///< ground altitude at which plane was launched
	float	_hdg_hold_yaw{0.0f};				///< hold heading for velocity mode
	bool	_hdg_hold_enabled{false};			///< heading hold enabled
	bool	_yaw_lock_engaged{false};			///< yaw is locked for heading hold
	float	_althold_epv{0.0f};				///< the position estimate accuracy when engaging alt hold
	bool	_was_in_deadband{false};			///< wether the last stick input was in althold deadband

	position_setpoint_s _hdg_hold_prev_wp {};		///< position where heading hold started
	position_setpoint_s _hdg_hold_curr_wp {};		///< position to which heading hold flies

	hrt_abstime _control_position_last_called{0};		///< last call of control_position

	/* Landing */
	bool _land_noreturn_horizontal{false};
	bool _land_noreturn_vertical{false};
	bool _land_stayonground{false};
	bool _land_motor_lim{false};
	bool _land_onslope{false};
	bool _land_abort{false};

	Landingslope _landingslope;

	hrt_abstime _time_started_landing{0};			///< time at which landing started

	float _t_alt_prev_valid{0};				///< last terrain estimate which was valid
	hrt_abstime _time_last_t_alt{0};			///< time at which we had last valid terrain alt

	float _flare_height{0.0f};				///< estimated height to ground at which flare started
	float _flare_pitch_sp{0.0f};			///< Current forced (i.e. not determined using TECS) flare pitch setpoint
	float _flare_curve_alt_rel_last{0.0f};
	float _target_bearing{0.0f};				///< estimated height to ground at which flare started

	bool _was_in_air{false};				///< indicated wether the plane was in the air in the previous interation*/
	hrt_abstime _time_went_in_air{0};			///< time at which the plane went in the air

	/* Takeoff launch detection and runway */
	LaunchDetector _launchDetector;
	LaunchDetectionResult _launch_detection_state{LAUNCHDETECTION_RES_NONE};
	hrt_abstime _launch_detection_notify{0};

	RunwayTakeoff _runway_takeoff;

	bool _last_manual{false};				///< true if the last iteration was in manual mode (used to determine when a reset is needed)

	/* throttle and airspeed states */
	bool _airspeed_valid{false};				///< flag if a valid airspeed estimate exists
	hrt_abstime _airspeed_last_valid{0};			///< last time airspeed was received. Used to detect timeouts.
	float _airspeed{0.0f};
	float _eas2tas{1.0f};

	float _groundspeed_undershoot{0.0f};			///< ground speed error to min. speed in m/s

	Dcmf _R_nb;				///< current attitude
	float _roll{0.0f};
	float _pitch{0.0f};
	float _yaw{0.0f};

	bool _reinitialize_tecs{true};				///< indicates if the TECS states should be reinitialized (used for VTOL)
	bool _is_tecs_running{false};
	hrt_abstime _last_tecs_update{0};

	float _asp_after_transition{0.0f};
	bool _was_in_transition{false};

	bool _vtol_tailsitter{false};

	Vector2f _transition_waypoint{NAN, NAN};

	// estimator reset counters
	uint8_t _pos_reset_counter{0};				///< captures the number of times the estimator has reset the horizontal position
	uint8_t _alt_reset_counter{0};				///< captures the number of times the estimator has reset the altitude state

	ECL_L1_Pos_Controller	_l1_control;
	TECS			_tecs;

	enum FW_POSCTRL_MODE {
		FW_POSCTRL_MODE_AUTO,
		FW_POSCTRL_MODE_POSITION,
		FW_POSCTRL_MODE_ALTITUDE,
		FW_POSCTRL_MODE_OTHER
	} _control_mode_current{FW_POSCTRL_MODE_OTHER};		///< used to check the mode in the last control loop iteration. Use to check if the last iteration was in the same mode.

	param_t _param_handle_airspeed_trans{PARAM_INVALID};
	float _param_airspeed_trans{NAN};

	// Update our local parameter cache.
	int		parameters_update();

	// Update subscriptions
	void		airspeed_poll();
	void		control_update();
	void		vehicle_attitude_poll();
	void		vehicle_command_poll();
	void		vehicle_control_mode_poll();
	void		vehicle_status_poll();

	void		status_publish();
	void		landing_status_publish();
	void		tecs_status_publish();

	void		abort_landing(bool abort);

	/**
	 * Get a new waypoint based on heading and distance from current position
	 *
	 * @param heading the heading to fly to
	 * @param distance the distance of the generated waypoint
	 * @param waypoint_prev the waypoint at the current position
	 * @param waypoint_next the waypoint in the heading direction
	 */
	void		get_waypoint_heading_distance(float heading, position_setpoint_s &waypoint_prev,
			position_setpoint_s &waypoint_next, bool flag_init);

	/**
	 * Return the terrain estimate during takeoff or takeoff_alt if terrain estimate is not available
	 */
	float		get_terrain_altitude_takeoff(float takeoff_alt);

	/**
	 * Check if we are in a takeoff situation
	 */
	bool 		in_takeoff_situation();

	/**
	 * Do takeoff help when in altitude controlled modes
	 * @param hold_altitude altitude setpoint for controller
	 * @param pitch_limit_min minimum pitch allowed
	 */
	void 		do_takeoff_help(float *hold_altitude, float *pitch_limit_min);

	/**
	 * Update desired altitude base on user pitch stick input
	 *
	 * @param dt Time step
	 * @return true if climbout mode was requested by user (climb with max rate and min airspeed)
	 */
	bool		update_desired_altitude(float dt);

	bool		control_position(const hrt_abstime &now, const Vector2f &curr_pos, const Vector2f &ground_speed,
					 const position_setpoint_s &pos_sp_prev,
					 const position_setpoint_s &pos_sp_curr, const position_setpoint_s &pos_sp_next);
	void		control_takeoff(const hrt_abstime &now, const Vector2f &curr_pos, const Vector2f &ground_speed,
					const position_setpoint_s &pos_sp_prev,
					const position_setpoint_s &pos_sp_curr);
	void		control_landing(const hrt_abstime &now, const Vector2f &curr_pos, const Vector2f &ground_speed,
					const position_setpoint_s &pos_sp_prev,
					const position_setpoint_s &pos_sp_curr);

	float		get_tecs_pitch();
	float		get_tecs_thrust();

	float		get_demanded_airspeed();
	float		calculate_target_airspeed(float airspeed_demand, const Vector2f &ground_speed);

	/**
	 * Handle incoming vehicle commands
	 */
	void		handle_command();

	void		reset_takeoff_state(bool force = false);
	void		reset_landing_state();

	/*
	 * Call TECS : a wrapper function to call the TECS implementation
	 */
	void tecs_update_pitch_throttle(const hrt_abstime &now, float alt_sp, float airspeed_sp,
					float pitch_min_rad, float pitch_max_rad,
					float throttle_min, float throttle_max, float throttle_cruise,
					bool climbout_mode, float climbout_pitch_min_rad,
					uint8_t mode = tecs_status_s::TECS_MODE_NORMAL);

	DEFINE_PARAMETERS(

		(ParamFloat<px4::params::FW_AIRSPD_MAX>) _param_fw_airspd_max,
		(ParamFloat<px4::params::FW_AIRSPD_MIN>) _param_fw_airspd_min,
		(ParamFloat<px4::params::FW_AIRSPD_TRIM>) _param_fw_airspd_trim,

		(ParamFloat<px4::params::FW_CLMBOUT_DIFF>) _param_fw_clmbout_diff,

		(ParamFloat<px4::params::FW_GND_SPD_MIN>) _param_fw_gnd_spd_min,

		(ParamFloat<px4::params::FW_L1_DAMPING>) _param_fw_l1_damping,
		(ParamFloat<px4::params::FW_L1_PERIOD>) _param_fw_l1_period,
		(ParamFloat<px4::params::FW_L1_R_SLEW_MAX>) _param_fw_l1_r_slew_max,
		(ParamFloat<px4::params::FW_R_LIM>) _param_fw_r_lim,

		(ParamFloat<px4::params::FW_LND_AIRSPD_SC>) _param_fw_lnd_airspd_sc,
		(ParamFloat<px4::params::FW_LND_ANG>) _param_fw_lnd_ang,
		(ParamFloat<px4::params::FW_LND_FL_PMAX>) _param_fw_lnd_fl_pmax,
		(ParamFloat<px4::params::FW_LND_FL_PMIN>) _param_fw_lnd_fl_pmin,
		(ParamFloat<px4::params::FW_LND_FLALT>) _param_fw_lnd_flalt,
		(ParamFloat<px4::params::FW_LND_HHDIST>) _param_fw_lnd_hhdist,
		(ParamFloat<px4::params::FW_LND_HVIRT>) _param_fw_lnd_hvirt,
		(ParamFloat<px4::params::FW_LND_THRTC_SC>) _param_fw_thrtc_sc,
		(ParamFloat<px4::params::FW_LND_TLALT>) _param_fw_lnd_tlalt,
		(ParamBool<px4::params::FW_LND_EARLYCFG>) _param_fw_lnd_earlycfg,
		(ParamBool<px4::params::FW_LND_USETER>) _param_fw_lnd_useter,

		(ParamFloat<px4::params::FW_P_LIM_MAX>) _param_fw_p_lim_max,
		(ParamFloat<px4::params::FW_P_LIM_MIN>) _param_fw_p_lim_min,

		(ParamFloat<px4::params::FW_T_CLMB_MAX>) _param_fw_t_clmb_max,
		(ParamFloat<px4::params::FW_T_HRATE_FF>) _param_fw_t_hrate_ff,
		(ParamFloat<px4::params::FW_T_HRATE_P>) _param_fw_t_hrate_p,
		(ParamFloat<px4::params::FW_T_INTEG_GAIN>) _param_fw_t_integ_gain,
		(ParamFloat<px4::params::FW_T_PTCH_DAMP>) _param_fw_t_ptch_damp,
		(ParamFloat<px4::params::FW_T_RLL2THR>) _param_fw_t_rll2thr,
		(ParamFloat<px4::params::FW_T_SINK_MAX>) _param_fw_t_sink_max,
		(ParamFloat<px4::params::FW_T_SINK_MIN>) _param_fw_t_sink_min,
		(ParamFloat<px4::params::FW_T_SPD_OMEGA>) _param_fw_t_spd_omega,
		(ParamFloat<px4::params::FW_T_SPDWEIGHT>) _param_fw_t_spdweight,
		(ParamFloat<px4::params::FW_T_SRATE_P>) _param_fw_t_srate_p,
		(ParamFloat<px4::params::FW_T_THR_DAMP>) _param_fw_t_thr_damp,
		(ParamFloat<px4::params::FW_T_THRO_CONST>) _param_fw_t_thro_const,
		(ParamFloat<px4::params::FW_T_TIME_CONST>) _param_fw_t_time_const,
		(ParamFloat<px4::params::FW_T_VERT_ACC>) _param_fw_t_vert_acc,

		(ParamFloat<px4::params::FW_THR_ALT_SCL>) _param_fw_thr_alt_scl,
		(ParamFloat<px4::params::FW_THR_CRUISE>) _param_fw_thr_cruise,
		(ParamFloat<px4::params::FW_THR_IDLE>) _param_fw_thr_idle,
		(ParamFloat<px4::params::FW_THR_LND_MAX>) _param_fw_thr_lnd_max,
		(ParamFloat<px4::params::FW_THR_MAX>) _param_fw_thr_max,
		(ParamFloat<px4::params::FW_THR_MIN>) _param_fw_thr_min,
		(ParamFloat<px4::params::FW_THR_SLEW_MAX>) _param_fw_thr_slew_max,

		// external parameters
		(ParamInt<px4::params::FW_ARSP_MODE>) _param_fw_arsp_mode,

		(ParamFloat<px4::params::FW_PSP_OFF>) _param_fw_psp_off,
		(ParamFloat<px4::params::FW_RSP_OFF>) _param_fw_rsp_off,
		(ParamFloat<px4::params::FW_MAN_P_MAX>) _param_fw_man_p_max,
		(ParamFloat<px4::params::FW_MAN_R_MAX>) _param_fw_man_r_max,

		(ParamFloat<px4::params::NAV_LOITER_RAD>) _param_nav_loiter_rad

	)

};

#endif // FIXEDWINGPOSITIONCONTROL_HPP_
