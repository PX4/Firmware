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
#include <lib/systemlib/mavlink_log.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/cellular_status.h>
#include <uORB/topics/collision_report.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/follow_target.h>
#include <uORB/topics/generator_status.h>
#include <uORB/topics/gimbal_manager_set_attitude.h>
#include <uORB/topics/gimbal_manager_set_manual_control.h>
#include <uORB/topics/gimbal_device_information.h>
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
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/transponder_report.h>
#include <uORB/topics/tune_control.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_trajectory_bezier.h>
#include <uORB/topics/vehicle_trajectory_waypoint.h>

#if !defined(CONSTRAINED_FLASH)
# include <uORB/topics/debug_array.h>
# include <uORB/topics/debug_key_value.h>
# include <uORB/topics/debug_value.h>
# include <uORB/topics/debug_vect.h>
#endif // !CONSTRAINED_FLASH

using namespace time_literals;

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

	/**
	 * Set the cruising speed in offboard control
	 *
	 * Passing a negative value or leaving the parameter away will reset the cruising speed
	 * to its default value.
	 *
	 * Sets cruising speed for current flight mode only (resets on mode changes).
	 *
	 */
	void set_offb_cruising_speed(float speed = -1.0f);

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
	void handle_message_distance_sensor(mavlink_message_t *msg);
	void handle_message_follow_target(mavlink_message_t *msg);
	void handle_message_generator_status(mavlink_message_t *msg);
	void handle_message_set_gps_global_origin(mavlink_message_t *msg);
	void handle_message_gps_rtcm_data(mavlink_message_t *msg);
	void handle_message_heartbeat(mavlink_message_t *msg);
	void handle_message_hil_gps(mavlink_message_t *msg);
	void handle_message_hil_optical_flow(mavlink_message_t *msg);
	void handle_message_hil_sensor(mavlink_message_t *msg);
	void handle_message_hil_state_quaternion(mavlink_message_t *msg);
	void handle_message_landing_target(mavlink_message_t *msg);
	void handle_message_logging_ack(mavlink_message_t *msg);
	void handle_message_manual_control(mavlink_message_t *msg);
	void handle_message_obstacle_distance(mavlink_message_t *msg);
	void handle_message_odometry(mavlink_message_t *msg);
	void handle_message_onboard_computer_status(mavlink_message_t *msg);
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
	void handle_message_set_position_target_global_int(mavlink_message_t *msg);
	void handle_message_set_position_target_local_ned(mavlink_message_t *msg);
	void handle_message_statustext(mavlink_message_t *msg);
	void handle_message_trajectory_representation_bezier(mavlink_message_t *msg);
	void handle_message_trajectory_representation_waypoints(mavlink_message_t *msg);
	void handle_message_utm_global_position(mavlink_message_t *msg);
	void handle_message_vision_position_estimate(mavlink_message_t *msg);
	void handle_message_gimbal_manager_set_attitude(mavlink_message_t *msg);
	void handle_message_gimbal_manager_set_manual_control(mavlink_message_t *msg);
	void handle_message_gimbal_device_information(mavlink_message_t *msg);

#if !defined(CONSTRAINED_FLASH)
	void handle_message_debug(mavlink_message_t *msg);
	void handle_message_debug_float_array(mavlink_message_t *msg);
	void handle_message_debug_vect(mavlink_message_t *msg);
	void handle_message_named_value_float(mavlink_message_t *msg);
#endif // !CONSTRAINED_FLASH

	void CheckHeartbeats(const hrt_abstime &t, bool force = false);

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

	orb_advert_t _mavlink_log_pub{nullptr};

	// subset of MAV_COMPONENTs we support
	enum SUPPORTED_COMPONENTS : uint8_t {
		COMP_ID_ALL,
		COMP_ID_AUTOPILOT1,

		COMP_ID_TELEMETRY_RADIO,

		COMP_ID_CAMERA,
		COMP_ID_CAMERA2,

		COMP_ID_GIMBAL,
		COMP_ID_LOG,
		COMP_ID_ADSB,
		COMP_ID_OSD,
		COMP_ID_PERIPHERAL,

		COMP_ID_FLARM,

		COMP_ID_GIMBAL2,

		COMP_ID_MISSIONPLANNER,
		COMP_ID_ONBOARD_COMPUTER,

		COMP_ID_PATHPLANNER,
		COMP_ID_OBSTACLE_AVOIDANCE,
		COMP_ID_VISUAL_INERTIAL_ODOMETRY,
		COMP_ID_PAIRING_MANAGER,

		COMP_ID_IMU,

		COMP_ID_GPS,
		COMP_ID_GPS2,

		COMP_ID_UDP_BRIDGE,
		COMP_ID_UART_BRIDGE,
		COMP_ID_TUNNEL_NODE,

		COMP_ID_MAX
	};

	// map of supported component IDs to MAV_COMP value
	static const uint8_t supported_component_map[COMP_ID_MAX];

	bool _reported_unsupported_comp_id{false};

	static constexpr int MAX_REMOTE_SYSTEM_IDS{8};
	uint8_t _system_id_map[MAX_REMOTE_SYSTEM_IDS] {};

	uint8_t  _last_index[MAX_REMOTE_SYSTEM_IDS][COMP_ID_MAX] {};    ///< Store the last received sequence ID for each system/componenet pair
	uint8_t  _sys_comp_present[MAX_REMOTE_SYSTEM_IDS][COMP_ID_MAX] {}; ///< First message flag
	uint64_t _total_received_counter{0};                            ///< The total number of successfully received messages
	uint64_t _total_received_supported_counter{0};                  ///< The total number of successfully received messages
	uint64_t _total_lost_counter{0};                                ///< Total messages lost during transmission.
	float    _running_loss_percent{0};                              ///< Loss rate

	uint8_t _mavlink_status_last_buffer_overrun{0};
	uint8_t _mavlink_status_last_parse_error{0};
	uint16_t _mavlink_status_last_packet_rx_drop_count{0};

	// ORB publications
	uORB::Publication<actuator_controls_s>			_actuator_controls_pubs[4] {ORB_ID(actuator_controls_0), ORB_ID(actuator_controls_1), ORB_ID(actuator_controls_2), ORB_ID(actuator_controls_3)};
	uORB::Publication<airspeed_s>				_airspeed_pub{ORB_ID(airspeed)};
	uORB::Publication<battery_status_s>			_battery_pub{ORB_ID(battery_status)};
	uORB::Publication<cellular_status_s>			_cellular_status_pub{ORB_ID(cellular_status)};
	uORB::Publication<collision_report_s>			_collision_report_pub{ORB_ID(collision_report)};
	uORB::Publication<differential_pressure_s>		_differential_pressure_pub{ORB_ID(differential_pressure)};
	uORB::Publication<follow_target_s>			_follow_target_pub{ORB_ID(follow_target)};
	uORB::Publication<gimbal_manager_set_attitude_s>	_gimbal_manager_set_attitude_pub{ORB_ID(gimbal_manager_set_attitude)};
	uORB::Publication<gimbal_manager_set_manual_control_s>	_gimbal_manager_set_manual_control_pub{ORB_ID(gimbal_manager_set_manual_control)};
	uORB::Publication<gimbal_device_information_s>		_gimbal_device_information_pub{ORB_ID(gimbal_device_information)};
	uORB::Publication<irlock_report_s>			_irlock_report_pub{ORB_ID(irlock_report)};
	uORB::Publication<landing_target_pose_s>		_landing_target_pose_pub{ORB_ID(landing_target_pose)};
	uORB::Publication<log_message_s>			_log_message_pub{ORB_ID(log_message)};
	uORB::Publication<obstacle_distance_s>			_obstacle_distance_pub{ORB_ID(obstacle_distance)};
	uORB::Publication<offboard_control_mode_s>		_offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<onboard_computer_status_s>		_onboard_computer_status_pub{ORB_ID(onboard_computer_status)};
	uORB::Publication<generator_status_s>			_generator_status_pub{ORB_ID(generator_status)};
	uORB::Publication<optical_flow_s>			_flow_pub{ORB_ID(optical_flow)};
	uORB::Publication<sensor_gps_s>				_sensor_gps_pub{ORB_ID(sensor_gps)};
	uORB::Publication<vehicle_attitude_s>			_attitude_pub{ORB_ID(vehicle_attitude)};
	uORB::Publication<vehicle_attitude_setpoint_s>		_att_sp_pub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Publication<vehicle_attitude_setpoint_s>		_mc_virtual_att_sp_pub{ORB_ID(mc_virtual_attitude_setpoint)};
	uORB::Publication<vehicle_attitude_setpoint_s>		_fw_virtual_att_sp_pub{ORB_ID(fw_virtual_attitude_setpoint)};
	uORB::Publication<vehicle_global_position_s>		_global_pos_pub{ORB_ID(vehicle_global_position)};
	uORB::Publication<vehicle_land_detected_s>		_land_detector_pub{ORB_ID(vehicle_land_detected)};
	uORB::Publication<vehicle_local_position_s>		_local_pos_pub{ORB_ID(vehicle_local_position)};
	uORB::Publication<vehicle_local_position_setpoint_s>	_trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<vehicle_odometry_s>			_mocap_odometry_pub{ORB_ID(vehicle_mocap_odometry)};
	uORB::Publication<vehicle_odometry_s>			_visual_odometry_pub{ORB_ID(vehicle_visual_odometry)};
	uORB::Publication<vehicle_rates_setpoint_s>		_rates_sp_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Publication<vehicle_trajectory_bezier_s>		_trajectory_bezier_pub{ORB_ID(vehicle_trajectory_bezier)};
	uORB::Publication<vehicle_trajectory_waypoint_s>	_trajectory_waypoint_pub{ORB_ID(vehicle_trajectory_waypoint)};

#if !defined(CONSTRAINED_FLASH)
	uORB::Publication<debug_array_s>			_debug_array_pub {ORB_ID(debug_array)};
	uORB::Publication<debug_key_value_s>			_debug_key_value_pub{ORB_ID(debug_key_value)};
	uORB::Publication<debug_value_s>			_debug_value_pub{ORB_ID(debug_value)};
	uORB::Publication<debug_vect_s>				_debug_vect_pub{ORB_ID(debug_vect)};
#endif // !CONSTRAINED_FLASH

	// ORB publications (multi)
	uORB::PublicationMulti<distance_sensor_s>		_distance_sensor_pub{ORB_ID(distance_sensor)};
	uORB::PublicationMulti<distance_sensor_s>		_flow_distance_sensor_pub{ORB_ID(distance_sensor)};
	uORB::PublicationMulti<input_rc_s>			_rc_pub{ORB_ID(input_rc)};
	uORB::PublicationMulti<manual_control_setpoint_s>	_manual_control_setpoint_pub{ORB_ID(manual_control_setpoint)};
	uORB::PublicationMulti<ping_s>				_ping_pub{ORB_ID(ping)};
	uORB::PublicationMulti<radio_status_s>			_radio_status_pub{ORB_ID(radio_status)};

	// ORB publications (queue length > 1)
	uORB::Publication<gps_inject_data_s>     _gps_inject_data_pub{ORB_ID(gps_inject_data)};
	uORB::Publication<transponder_report_s>  _transponder_report_pub{ORB_ID(transponder_report)};
	uORB::Publication<vehicle_command_s>     _cmd_pub{ORB_ID(vehicle_command)};
	uORB::Publication<vehicle_command_ack_s> _cmd_ack_pub{ORB_ID(vehicle_command_ack)};

	// ORB subscriptions
	uORB::Subscription	_actuator_armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription	_home_position_sub{ORB_ID(home_position)};
	uORB::Subscription	_vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription	_vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription	_vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription	_vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription 	_actuator_controls_3_sub{ORB_ID(actuator_controls_3)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

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

	hrt_abstime			_last_utm_global_pos_com{0};

	// Allocated if needed.
	TunePublisher *_tune_publisher{nullptr};

	hrt_abstime _last_heartbeat_check{0};

	hrt_abstime _heartbeat_type_antenna_tracker{0};
	hrt_abstime _heartbeat_type_gcs{0};
	hrt_abstime _heartbeat_type_onboard_controller{0};
	hrt_abstime _heartbeat_type_gimbal{0};
	hrt_abstime _heartbeat_type_adsb{0};
	hrt_abstime _heartbeat_type_camera{0};

	hrt_abstime _heartbeat_component_telemetry_radio{0};
	hrt_abstime _heartbeat_component_log{0};
	hrt_abstime _heartbeat_component_osd{0};
	hrt_abstime _heartbeat_component_obstacle_avoidance{0};
	hrt_abstime _heartbeat_component_visual_inertial_odometry{0};
	hrt_abstime _heartbeat_component_pairing_manager{0};
	hrt_abstime _heartbeat_component_udp_bridge{0};
	hrt_abstime _heartbeat_component_uart_bridge{0};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::BAT_CRIT_THR>)     _param_bat_crit_thr,
		(ParamFloat<px4::params::BAT_EMERGEN_THR>)  _param_bat_emergen_thr,
		(ParamFloat<px4::params::BAT_LOW_THR>)      _param_bat_low_thr,
		(ParamFloat<px4::params::SENS_FLOW_MAXHGT>) _param_sens_flow_maxhgt,
		(ParamFloat<px4::params::SENS_FLOW_MAXR>)   _param_sens_flow_maxr,
		(ParamFloat<px4::params::SENS_FLOW_MINHGT>) _param_sens_flow_minhgt,
		(ParamInt<px4::params::SENS_FLOW_ROT>)      _param_sens_flow_rot
	);

	// Disallow copy construction and move assignment.
	MavlinkReceiver(const MavlinkReceiver &) = delete;
	MavlinkReceiver operator=(const MavlinkReceiver &) = delete;
};
