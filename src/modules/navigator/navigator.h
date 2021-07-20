/***************************************************************************
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
 * @file navigator.h
 * Helper class to access missions
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 * @author Lorenz Meier <lorenz@px4.io>
 */

#pragma once

#include "enginefailure.h"
#include "follow_target.h"
#include "geofence.h"
#include "gpsfailure.h"
#include "land.h"
#include "precland.h"
#include "loiter.h"
#include "mission.h"
#include "navigator_mode.h"
#include "rtl.h"
#include "takeoff.h"

#include "navigation.h"

#include "GeofenceBreachAvoidance/geofence_breach_avoidance.h"

#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/geofence_result.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_controller_landing_status.h>
#include <uORB/topics/position_controller_status.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/transponder_report.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/uORB.h>

using namespace time_literals;

/**
 * Number of navigation modes that need on_active/on_inactive calls
 */
#define NAVIGATOR_MODE_ARRAY_SIZE 9

class Navigator : public ModuleBase<Navigator>, public ModuleParams
{
public:
	Navigator();
	~Navigator() override;

	Navigator(const Navigator &) = delete;
	Navigator operator=(const Navigator &) = delete;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static Navigator *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	/**
	 * Load fence from file
	 */
	void		load_fence_from_file(const char *filename);

	void		publish_vehicle_cmd(vehicle_command_s *vcmd);

	/**
	 * Generate an artificial traffic indication
	 *
	 * @param distance Horizontal distance to this vehicle
	 * @param direction Direction in earth frame from this vehicle in radians
	 * @param traffic_heading Travel direction of the traffic in earth frame in radians
	 * @param altitude_diff Altitude difference, positive is up
	 * @param hor_velocity Horizontal velocity of traffic, in m/s
	 * @param ver_velocity Vertical velocity of traffic, in m/s
	 * @param emitter_type, Type of vehicle, as a number
	 */
	void		fake_traffic(const char *callsign, float distance, float direction, float traffic_heading, float altitude_diff,
				     float hor_velocity, float ver_velocity, int emitter_type);

	/**
	 * Check nearby traffic for potential collisions
	 */
	void		check_traffic();

	/**
	 * Buffer for air traffic to control the amount of messages sent to a user
	 */
	bool		buffer_air_traffic(uint32_t icao_address);

	/**
	 * Setters
	 */
	void		set_can_loiter_at_sp(bool can_loiter) { _can_loiter_at_sp = can_loiter; }
	void		set_position_setpoint_triplet_updated() { _pos_sp_triplet_updated = true; }
	void		set_mission_result_updated() { _mission_result_updated = true; }

	/**
	 * Getters
	 */
	struct home_position_s *get_home_position() { return &_home_pos; }
	struct mission_result_s *get_mission_result() { return &_mission_result; }
	struct position_setpoint_triplet_s *get_position_setpoint_triplet() { return &_pos_sp_triplet; }
	struct position_setpoint_triplet_s *get_reposition_triplet() { return &_reposition_triplet; }
	struct position_setpoint_triplet_s *get_takeoff_triplet() { return &_takeoff_triplet; }
	struct vehicle_global_position_s *get_global_position() { return &_global_pos; }
	struct vehicle_land_detected_s *get_land_detected() { return &_land_detected; }
	struct vehicle_local_position_s *get_local_position() { return &_local_pos; }
	struct vehicle_status_s *get_vstatus() { return &_vstatus; }
	PrecLand *get_precland() { return &_precland; } /**< allow others, e.g. Mission, to use the precision land block */

	const vehicle_roi_s &get_vroi() { return _vroi; }
	void reset_vroi() { _vroi = {}; }

	bool home_alt_valid() { return (_home_pos.timestamp > 0 && _home_pos.valid_alt); }
	bool home_position_valid() { return (_home_pos.timestamp > 0 && _home_pos.valid_alt && _home_pos.valid_hpos && _home_pos.valid_lpos); }

	Geofence	&get_geofence() { return _geofence; }

	bool		get_can_loiter_at_sp() { return _can_loiter_at_sp; }
	float		get_loiter_radius() { return _param_nav_loiter_rad.get(); }

	/**
	 * Returns the default acceptance radius defined by the parameter
	 */
	float		get_default_acceptance_radius();

	/**
	 * Get the acceptance radius
	 *
	 * @return the distance at which the next waypoint should be used
	 */
	float		get_acceptance_radius();

	/**
	 * Get the default altitude acceptance radius (i.e. from parameters)
	 *
	 * @return the distance from the target altitude before considering the waypoint reached
	 */
	float		get_default_altitude_acceptance_radius();

	/**
	 * Get the altitude acceptance radius
	 *
	 * @return the distance from the target altitude before considering the waypoint reached
	 */
	float		get_altitude_acceptance_radius();

	/**
	 * Get the cruising speed
	 *
	 * @return the desired cruising speed for this mission
	 */
	float		get_cruising_speed();

	/**
	 * Set the cruising speed
	 *
	 * Passing a negative value or leaving the parameter away will reset the cruising speed
	 * to its default value.
	 *
	 * For VTOL: sets cruising speed for current mode only (multirotor or fixed-wing).
	 *
	 */
	void		set_cruising_speed(float speed = -1.0f);

	/**
	 * Reset cruising speed to default values
	 *
	 * For VTOL: resets both cruising speeds.
	 */
	void		reset_cruising_speed();

	/**
	 *  Set triplets to invalid
	 */
	void 		reset_triplets();

	/**
	 *  Set position setpoint to safe defaults
	 */
	void		reset_position_setpoint(position_setpoint_s &sp);

	/**
	 * Get the target throttle
	 *
	 * @return the desired throttle for this mission
	 */
	float		get_cruising_throttle();

	/**
	 * Set the target throttle
	 */
	void		set_cruising_throttle(float throttle = NAN) { _mission_throttle = throttle; }

	/**
	 * Get the yaw acceptance given the current mission item
	 *
	 * @param mission_item_yaw the yaw to use in case the controller-derived radius is finite
	 *
	 * @return the yaw at which the next waypoint should be used or NaN if the yaw at a waypoint
	 * should be ignored
	 */
	float 		get_yaw_acceptance(float mission_item_yaw);


	orb_advert_t	*get_mavlink_log_pub() { return &_mavlink_log_pub; }

	void		increment_mission_instance_count() { _mission_result.instance_count++; }
	int		mission_instance_count() const { return _mission_result.instance_count; }

	void 		set_mission_failure(const char *reason);

	void 		setMissionLandingInProgress(bool in_progress) { _mission_landing_in_progress = in_progress; }

	bool 		getMissionLandingInProgress() { return _mission_landing_in_progress; }

	bool		is_planned_mission() const { return _navigation_mode == &_mission; }
	bool		on_mission_landing() { return _mission.landing(); }
	bool		start_mission_landing() { return _mission.land_start(); }
	bool		get_mission_start_land_available() { return _mission.get_land_start_available(); }
	int 		get_mission_landing_index() { return _mission.get_land_start_index(); }
	double 	get_mission_landing_start_lat() { return _mission.get_landing_start_lat(); }
	double 	get_mission_landing_start_lon() { return _mission.get_landing_start_lon(); }
	float 	get_mission_landing_start_alt() { return _mission.get_landing_start_alt(); }

	double 	get_mission_landing_lat() { return _mission.get_landing_lat(); }
	double 	get_mission_landing_lon() { return _mission.get_landing_lon(); }
	float 	get_mission_landing_alt() { return _mission.get_landing_alt(); }

	// RTL
	bool		mission_landing_required() { return _rtl.rtl_type() == RTL::RTL_LAND; }
	int			rtl_type() { return _rtl.rtl_type(); }
	bool		in_rtl_state() const { return _vstatus.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL; }

	bool		abort_landing();

	void geofence_breach_check(bool &have_geofence_position_data);

	// Param access
	float		get_loiter_min_alt() const { return _param_mis_ltrmin_alt.get(); }
	float		get_takeoff_min_alt() const { return _param_mis_takeoff_alt.get(); }
	bool		get_takeoff_required() const { return _param_mis_takeoff_req.get(); }
	float		get_yaw_timeout() const { return _param_mis_yaw_tmt.get(); }
	float		get_yaw_threshold() const { return math::radians(_param_mis_yaw_err.get()); }
	int 		get_mnt_uav_yaw() const { return _param_mnt_uav_yaw.get(); }

	float		get_vtol_back_trans_deceleration() const { return _param_back_trans_dec_mss; }
	float		get_vtol_reverse_delay() const { return _param_reverse_delay; }

	bool		force_vtol();

	void		acquire_gimbal_control();
	void		release_gimbal_control();

private:
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::NAV_LOITER_RAD>) _param_nav_loiter_rad,	/**< loiter radius for fixedwing */
		(ParamFloat<px4::params::NAV_ACC_RAD>) _param_nav_acc_rad,	/**< acceptance for takeoff */
		(ParamFloat<px4::params::NAV_FW_ALT_RAD>)
		_param_nav_fw_alt_rad,	/**< acceptance radius for fixedwing altitude */
		(ParamFloat<px4::params::NAV_FW_ALTL_RAD>)
		_param_nav_fw_altl_rad,	/**< acceptance radius for fixedwing altitude before landing*/
		(ParamFloat<px4::params::NAV_MC_ALT_RAD>)
		_param_nav_mc_alt_rad,	/**< acceptance radius for multicopter altitude */
		(ParamInt<px4::params::NAV_FORCE_VT>) _param_nav_force_vt,	/**< acceptance radius for multicopter altitude */
		(ParamInt<px4::params::NAV_TRAFF_AVOID>) _param_nav_traff_avoid,	/**< avoiding other aircraft is enabled */
		(ParamFloat<px4::params::NAV_TRAFF_A_RADU>) _param_nav_traff_a_radu,	/**< avoidance Distance Unmanned*/
		(ParamFloat<px4::params::NAV_TRAFF_A_RADM>) _param_nav_traff_a_radm,	/**< avoidance Distance Manned*/

		// non-navigator parameters
		// Mission (MIS_*)
		(ParamFloat<px4::params::MIS_LTRMIN_ALT>) _param_mis_ltrmin_alt,
		(ParamFloat<px4::params::MIS_TAKEOFF_ALT>) _param_mis_takeoff_alt,
		(ParamBool<px4::params::MIS_TAKEOFF_REQ>) _param_mis_takeoff_req,
		(ParamFloat<px4::params::MIS_YAW_TMT>) _param_mis_yaw_tmt,
		(ParamFloat<px4::params::MIS_YAW_ERR>) _param_mis_yaw_err,
		(ParamInt<px4::params::MNT_UAV_YAW>) _param_mnt_uav_yaw
	)

	struct traffic_buffer_s {
		uint32_t 	icao_address;
		hrt_abstime timestamp;
	};

	int		_local_pos_sub{-1};
	int		_mission_sub{-1};
	int		_vehicle_status_sub{-1};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _global_pos_sub{ORB_ID(vehicle_global_position)};	/**< global position subscription */
	uORB::Subscription _gps_pos_sub{ORB_ID(vehicle_gps_position)};		/**< gps position subscription */
	uORB::Subscription _home_pos_sub{ORB_ID(home_position)};		/**< home position subscription */
	uORB::Subscription _land_detected_sub{ORB_ID(vehicle_land_detected)};	/**< vehicle land detected subscription */
	uORB::Subscription _pos_ctrl_landing_status_sub{ORB_ID(position_controller_landing_status)};	/**< position controller landing status subscription */
	uORB::Subscription _traffic_sub{ORB_ID(transponder_report)};		/**< traffic subscription */
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};	/**< vehicle commands (onboard and offboard) */

	uORB::SubscriptionData<position_controller_status_s>	_position_controller_status_sub{ORB_ID(position_controller_status)};

	uORB::Publication<geofence_result_s>		_geofence_result_pub{ORB_ID(geofence_result)};
	uORB::Publication<mission_result_s>		_mission_result_pub{ORB_ID(mission_result)};
	uORB::Publication<position_setpoint_triplet_s>	_pos_sp_triplet_pub{ORB_ID(position_setpoint_triplet)};
	uORB::Publication<vehicle_roi_s>		_vehicle_roi_pub{ORB_ID(vehicle_roi)};

	orb_advert_t	_mavlink_log_pub{nullptr};	/**< the uORB advert to send messages over mavlink */

	uORB::Publication<vehicle_command_ack_s>	_vehicle_cmd_ack_pub{ORB_ID(vehicle_command_ack)};
	uORB::Publication<vehicle_command_s>	_vehicle_cmd_pub{ORB_ID(vehicle_command)};

	// Subscriptions
	home_position_s					_home_pos{};		/**< home position for RTL */
	mission_result_s				_mission_result{};
	vehicle_global_position_s			_global_pos{};		/**< global vehicle position */
	vehicle_gps_position_s				_gps_pos{};		/**< gps position */
	vehicle_land_detected_s				_land_detected{};	/**< vehicle land_detected */
	vehicle_local_position_s			_local_pos{};		/**< local vehicle position */
	vehicle_status_s				_vstatus{};		/**< vehicle status */

	uint8_t						_previous_nav_state{}; /**< nav_state of the previous iteration*/

	// Publications
	geofence_result_s				_geofence_result{};
	position_setpoint_triplet_s			_pos_sp_triplet{};	/**< triplet of position setpoints */
	position_setpoint_triplet_s			_reposition_triplet{};	/**< triplet for non-mission direct position command */
	position_setpoint_triplet_s			_takeoff_triplet{};	/**< triplet for non-mission direct takeoff command */
	vehicle_roi_s					_vroi{};		/**< vehicle ROI */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	Geofence	_geofence;			/**< class that handles the geofence */
	bool		_geofence_violation_warning_sent{false}; /**< prevents spaming to mavlink */
	GeofenceBreachAvoidance _gf_breach_avoidance;
	hrt_abstime _last_geofence_check = 0;

	bool		_can_loiter_at_sp{false};			/**< flags if current position SP can be used to loiter */
	bool		_pos_sp_triplet_updated{false};		/**< flags if position SP triplet needs to be published */
	bool 		_pos_sp_triplet_published_invalid_once{false};	/**< flags if position SP triplet has been published once to UORB */
	bool		_mission_result_updated{false};		/**< flags if mission result has seen an update */

	NavigatorMode	*_navigation_mode{nullptr};		/**< abstract pointer to current navigation mode class */
	Mission		_mission;			/**< class that handles the missions */
	Loiter		_loiter;			/**< class that handles loiter */
	Takeoff		_takeoff;			/**< class for handling takeoff commands */
	Land		_land;			/**< class for handling land commands */
	PrecLand	_precland;			/**< class for handling precision land commands */
	RTL 		_rtl;				/**< class that handles RTL */
	EngineFailure	_engineFailure;			/**< class that handles the engine failure mode (FW only!) */
	GpsFailure	_gpsFailure;			/**< class that handles the OBC gpsfailure loss mode */
	FollowTarget	_follow_target;

	NavigatorMode *_navigation_mode_array[NAVIGATOR_MODE_ARRAY_SIZE];	/**< array of navigation modes */

	param_t _handle_back_trans_dec_mss{PARAM_INVALID};
	param_t _handle_reverse_delay{PARAM_INVALID};
	param_t _handle_mpc_jerk_auto{PARAM_INVALID};
	param_t _handle_mpc_acc_hor{PARAM_INVALID};

	float _param_back_trans_dec_mss{0.f};
	float _param_reverse_delay{0.f};
	float _param_mpc_jerk_auto{4.f}; /**< initialized with the default jerk auto value to prevent division by 0 if the parameter is accidentally set to 0 */
	float _param_mpc_acc_hor{3.f}; /**< initialized with the default horizontal acc value to prevent division by 0 if the parameter is accidentally set to 0 */

	float _mission_cruising_speed_mc{-1.0f};
	float _mission_cruising_speed_fw{-1.0f};
	float _mission_throttle{NAN};

	bool _mission_landing_in_progress{false};	// this flag gets set if the mission is currently executing on a landing pattern
	// if mission mode is inactive, this flag will be cleared after 2 seconds

	traffic_buffer_s _traffic_buffer{};

	// update subscriptions
	void		params_update();

	/**
	 * Publish a new position setpoint triplet for position controllers
	 */
	void		publish_position_setpoint_triplet();

	/**
	 * Publish the mission result so commander and mavlink know what is going on
	 */
	void		publish_mission_result();

	void		publish_vehicle_command_ack(const vehicle_command_s &cmd, uint8_t result);
};
