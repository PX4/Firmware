/****************************************************************************
 *
 *   Copyright (c) 2012-2020 PX4 Development Team. All rights reserved.
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
 * @file mavlink_receiver.h
 * MAVLink receiver thread
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 */

#pragma once

#include "mavlink_ftp.h"
#include "mavlink_log_handler.h"
#include "mavlink_mission.h"
#include "mavlink_parameters.h"
#include "mavlink_timesync.h"
#include "tune_publisher.h"

#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/barometer/PX4Barometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/cellular_status.h>
#include <uORB/topics/collision_report.h>
#include <uORB/topics/debug_array.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/debug_value.h>
#include <uORB/topics/debug_vect.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/follow_target.h>
#include <uORB/topics/gps_inject_data.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/irlock_report.h>
#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/log_message.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/obstacle_distance.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/onboard_computer_status.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/ping.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/radio_status.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/transponder_report.h>
#include <uORB/topics/tune_control.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_trajectory_bezier.h>
#include <uORB/topics/vehicle_trajectory_waypoint.h>

class Mavlink;

class MavlinkReceiver : public ModuleParams
{
public:
	MavlinkReceiver(Mavlink *parent);
	~MavlinkReceiver() override;

	/**
	 * Start the receiver thread
	 */
	static void receive_start(pthread_t *thread, Mavlink *parent);

	static void *start_helper(void *context);

private:

	void acknowledge(uint8_t sysid, uint8_t compid, uint16_t command, uint8_t result);

	/**
	 * Common method to handle both mavlink command types. T is one of mavlink_command_int_t or mavlink_command_long_t.
	 */
	template<class T>
	void handle_message_command_both(mavlink_message_t *msg, const T &cmd_mavlink,
					 const vehicle_command_s &vehicle_command);

	uint8_t handle_request_message_command(uint16_t message_id, float param2 = 0.0f, float param3 = 0.0f,
					       float param4 = 0.0f,
					       float param5 = 0.0f, float param6 = 0.0f, float param7 = 0.0f);

	void handle_message(mavlink_message_t *msg);

	void handle_message_adsb_vehicle(mavlink_message_t *msg);
	void handle_message_att_pos_mocap(mavlink_message_t *msg);
	void handle_message_battery_status(mavlink_message_t *msg);
	void handle_message_cellular_status(mavlink_message_t *msg);
	void handle_message_collision(mavlink_message_t *msg);
	void handle_message_command_ack(mavlink_message_t *msg);
	void handle_message_command_int(mavlink_message_t *msg);
	void handle_message_command_long(mavlink_message_t *msg);
	void handle_message_debug(mavlink_message_t *msg);
	void handle_message_debug_float_array(mavlink_message_t *msg);
	void handle_message_debug_vect(mavlink_message_t *msg);
	void handle_message_distance_sensor(mavlink_message_t *msg);
	void handle_message_follow_target(mavlink_message_t *msg);
	void handle_message_gps_global_origin(mavlink_message_t *msg);
	void handle_message_gps_rtcm_data(mavlink_message_t *msg);
	void handle_message_heartbeat(mavlink_message_t *msg);
	void handle_message_hil_gps(mavlink_message_t *msg);
	void handle_message_hil_optical_flow(mavlink_message_t *msg);
	void handle_message_hil_sensor(mavlink_message_t *msg);
	void handle_message_hil_state_quaternion(mavlink_message_t *msg);
	void handle_message_landing_target(mavlink_message_t *msg);
	void handle_message_logging_ack(mavlink_message_t *msg);
	void handle_message_manual_control(mavlink_message_t *msg);
	void handle_message_named_value_float(mavlink_message_t *msg);
	void handle_message_obstacle_distance(mavlink_message_t *msg);
	void handle_message_odometry(mavlink_message_t *msg);
	void handle_message_optical_flow_rad(mavlink_message_t *msg);
	void handle_message_ping(mavlink_message_t *msg);
	void handle_message_play_tune(mavlink_message_t *msg);
	void handle_message_play_tune_v2(mavlink_message_t *msg);
	void handle_message_radio_status(mavlink_message_t *msg);
	void handle_message_rc_channels_override(mavlink_message_t *msg);
	void handle_message_serial_control(mavlink_message_t *msg);
	void handle_message_set_actuator_control_target(mavlink_message_t *msg);
	void handle_message_set_attitude_target(mavlink_message_t *msg);
	void handle_message_set_mode(mavlink_message_t *msg);
	void handle_message_set_position_target_local_ned(mavlink_message_t *msg);
	void handle_message_set_position_target_global_int(mavlink_message_t *msg);
	void handle_message_statustext(mavlink_message_t *msg);
	void handle_message_trajectory_representation_bezier(mavlink_message_t *msg);
	void handle_message_trajectory_representation_waypoints(mavlink_message_t *msg);
	void handle_message_utm_global_position(mavlink_message_t *msg);
	void handle_message_vision_position_estimate(mavlink_message_t *msg);
	void handle_message_onboard_computer_status(mavlink_message_t *msg);


	void Run();

	/**
	 * Set the interval at which the given message stream is published.
	 * The rate is the number of messages per second.
	 *
	 * @param msgId The ID of the message interval to be set.
	 * @param interval The interval in usec to send the message.
	 * @param data_rate The total link data rate in bytes per second.
	 *
	 * @return PX4_OK on success, PX4_ERROR on fail.
	 */
	int set_message_interval(int msgId, float interval, int data_rate = -1);
	void get_message_interval(int msgId);

	/**
	 * Decode a switch position from a bitfield.
	 */
	switch_pos_t decode_switch_pos(uint16_t buttons, unsigned sw);

	/**
	 * Decode a switch position from a bitfield and state.
	 */
	int decode_switch_pos_n(uint16_t buttons, unsigned sw);

	bool evaluate_target_ok(int command, int target_system, int target_component);

	void fill_thrust(float *thrust_body_array, uint8_t vehicle_type, float thrust);

	void schedule_tune(const char *tune);

	/**
	 * @brief Updates the battery, optical flow, and flight ID subscribed parameters.
	 */
	void update_params();

	Mavlink				*_mavlink;

	MavlinkFTP			_mavlink_ftp;
	MavlinkLogHandler		_mavlink_log_handler;
	MavlinkMissionManager		_mission_manager;
	MavlinkParametersManager	_parameters_manager;
	MavlinkTimesync			_mavlink_timesync;

	mavlink_status_t		_status{}; ///< receiver status, used for mavlink_parse_char()

	// ORB publications
	uORB::Publication<actuator_controls_s>			_actuator_controls_pubs[4] {ORB_ID(actuator_controls_0), ORB_ID(actuator_controls_1), ORB_ID(actuator_controls_2), ORB_ID(actuator_controls_3)};
	uORB::Publication<airspeed_s>				_airspeed_pub{ORB_ID(airspeed)};
	uORB::Publication<battery_status_s>			_battery_pub{ORB_ID(battery_status)};
	uORB::Publication<cellular_status_s>			_cellular_status_pub{ORB_ID(cellular_status)};
	uORB::Publication<collision_report_s>			_collision_report_pub{ORB_ID(collision_report)};
	uORB::Publication<debug_array_s>			_debug_array_pub{ORB_ID(debug_array)};
	uORB::Publication<debug_key_value_s>			_debug_key_value_pub{ORB_ID(debug_key_value)};
	uORB::Publication<debug_value_s>			_debug_value_pub{ORB_ID(debug_value)};
	uORB::Publication<debug_vect_s>				_debug_vect_pub{ORB_ID(debug_vect)};
	uORB::Publication<differential_pressure_s>		_differential_pressure_pub{ORB_ID(differential_pressure)};
	uORB::Publication<follow_target_s>			_follow_target_pub{ORB_ID(follow_target)};
	uORB::Publication<irlock_report_s>			_irlock_report_pub{ORB_ID(irlock_report)};
	uORB::Publication<landing_target_pose_s>		_landing_target_pose_pub{ORB_ID(landing_target_pose)};
	uORB::Publication<log_message_s>			_log_message_pub{ORB_ID(log_message)};
	uORB::Publication<obstacle_distance_s>			_obstacle_distance_pub{ORB_ID(obstacle_distance)};
	uORB::Publication<offboard_control_mode_s>		_offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<onboard_computer_status_s>		_onboard_computer_status_pub{ORB_ID(onboard_computer_status)};
	uORB::Publication<optical_flow_s>			_flow_pub{ORB_ID(optical_flow)};
	uORB::Publication<position_setpoint_triplet_s>		_pos_sp_triplet_pub{ORB_ID(position_setpoint_triplet)};
	uORB::Publication<vehicle_attitude_s>			_attitude_pub{ORB_ID(vehicle_attitude)};
	uORB::Publication<vehicle_attitude_setpoint_s>		_att_sp_pub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Publication<vehicle_attitude_setpoint_s>		_mc_virtual_att_sp_pub{ORB_ID(mc_virtual_attitude_setpoint)};
	uORB::Publication<vehicle_attitude_setpoint_s>		_fw_virtual_att_sp_pub{ORB_ID(fw_virtual_attitude_setpoint)};
	uORB::Publication<vehicle_global_position_s>		_global_pos_pub{ORB_ID(vehicle_global_position)};
	uORB::Publication<vehicle_gps_position_s>		_gps_pub{ORB_ID(vehicle_gps_position)};
	uORB::Publication<vehicle_land_detected_s>		_land_detector_pub{ORB_ID(vehicle_land_detected)};
	uORB::Publication<vehicle_local_position_s>		_local_pos_pub{ORB_ID(vehicle_local_position)};
	uORB::Publication<vehicle_odometry_s>			_mocap_odometry_pub{ORB_ID(vehicle_mocap_odometry)};
	uORB::Publication<vehicle_odometry_s>			_visual_odometry_pub{ORB_ID(vehicle_visual_odometry)};
	uORB::Publication<vehicle_rates_setpoint_s>		_rates_sp_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Publication<vehicle_trajectory_bezier_s>	_trajectory_bezier_pub{ORB_ID(vehicle_trajectory_bezier)};
	uORB::Publication<vehicle_trajectory_waypoint_s>	_trajectory_waypoint_pub{ORB_ID(vehicle_trajectory_waypoint)};

	// ORB publications (multi)
	uORB::PublicationMulti<distance_sensor_s>		_distance_sensor_pub{ORB_ID(distance_sensor)};
	uORB::PublicationMulti<distance_sensor_s>		_flow_distance_sensor_pub{ORB_ID(distance_sensor)};
	uORB::PublicationMulti<input_rc_s>			_rc_pub{ORB_ID(input_rc)};
	uORB::PublicationMulti<manual_control_setpoint_s>	_manual_control_setpoint_pub{ORB_ID(manual_control_setpoint)};
	uORB::PublicationMulti<ping_s>				_ping_pub{ORB_ID(ping)};
	uORB::PublicationMulti<radio_status_s>			_radio_status_pub{ORB_ID(radio_status)};

	// ORB publications (queue length > 1)
	uORB::PublicationQueued<gps_inject_data_s>	_gps_inject_data_pub{ORB_ID(gps_inject_data)};
	uORB::PublicationQueued<transponder_report_s>	_transponder_report_pub{ORB_ID(transponder_report)};
	uORB::PublicationQueued<vehicle_command_ack_s>	_cmd_ack_pub{ORB_ID(vehicle_command_ack)};
	uORB::PublicationQueued<vehicle_command_s>	_cmd_pub{ORB_ID(vehicle_command)};

	// ORB subscriptions
	uORB::Subscription	_actuator_armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription	_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription	_parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription	_vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription	_vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription	_vehicle_status_sub{ORB_ID(vehicle_status)};

	// hil_sensor and hil_state_quaternion
	enum SensorSource {
		ACCEL		= 0b111,
		GYRO		= 0b111000,
		MAG		= 0b111000000,
		BARO		= 0b1101000000000,
		DIFF_PRESS	= 0b10000000000
	};
	PX4Accelerometer *_px4_accel{nullptr};
	PX4Barometer *_px4_baro{nullptr};
	PX4Gyroscope *_px4_gyro{nullptr};
	PX4Magnetometer *_px4_mag{nullptr};

	static constexpr unsigned int	MOM_SWITCH_COUNT{8};
	uint8_t				_mom_switch_pos[MOM_SWITCH_COUNT] {};
	uint16_t			_mom_switch_state{0};

	uint64_t			_global_ref_timestamp{0};

	map_projection_reference_s	_hil_local_proj_ref{};
	float				_hil_local_alt0{0.0f};
	bool				_hil_local_proj_inited{false};

	hrt_abstime			_last_utm_global_pos_com{0};

	// Allocated if needed.
	TunePublisher *_tune_publisher{nullptr};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::BAT_CRIT_THR>)     _param_bat_crit_thr,
		(ParamFloat<px4::params::BAT_EMERGEN_THR>)  _param_bat_emergen_thr,
		(ParamFloat<px4::params::BAT_LOW_THR>)      _param_bat_low_thr,
		(ParamInt<px4::params::COM_FLIGHT_UUID>)    _param_com_flight_uuid,
		(ParamFloat<px4::params::SENS_FLOW_MAXHGT>) _param_sens_flow_maxhgt,
		(ParamFloat<px4::params::SENS_FLOW_MAXR>)   _param_sens_flow_maxr,
		(ParamFloat<px4::params::SENS_FLOW_MINHGT>) _param_sens_flow_minhgt,
		(ParamInt<px4::params::SENS_FLOW_ROT>)      _param_sens_flow_rot
	);

	// Disallow copy construction and move assignment.
	MavlinkReceiver(const MavlinkReceiver &) = delete;
	MavlinkReceiver operator=(const MavlinkReceiver &) = delete;
};
