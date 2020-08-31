/****************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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
 * @file mavlink_messages.cpp
 * MAVLink 2.0 message formatters implementation.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include "mavlink_main.h"
#include "mavlink_messages.h"
#include "mavlink_command_sender.h"
#include "mavlink_simple_analyzer.h"
#include "mavlink_high_latency2.h"

#include "streams/autopilot_version.h"
#include "streams/flight_information.h"
#include "streams/protocol_version.h"
#include "streams/storage_information.h"

#include <commander/px4_custom_mode.h>
#include <drivers/drv_pwm_output.h>
#include <lib/conversion/rotation.h>
#include <lib/ecl/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/time.h>
#include <systemlib/mavlink_log.h>
#include <math.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/camera_capture.h>
#include <uORB/topics/camera_trigger.h>
#include <uORB/topics/collision_report.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/debug_array.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/debug_value.h>
#include <uORB/topics/debug_vect.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/geofence_result.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/mavlink_log.h>
#include <uORB/topics/mount_orientation.h>
#include <uORB/topics/obstacle_distance.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/orbit_status.h>
#include <uORB/topics/position_controller_status.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/rpm.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/tecs_status.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/transponder_report.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/vehicle_imu_status.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_status_flags.h>
#include <uORB/topics/vehicle_trajectory_waypoint.h>
#include <uORB/topics/vtol_vehicle_status.h>
#include <uORB/topics/wind_estimate.h>

using matrix::Vector3f;
using matrix::wrap_2pi;

static uint16_t cm_uint16_from_m_float(float m)
{
	if (m < 0.0f) {
		return 0;

	} else if (m > 655.35f) {
		return 65535;
	}

	return (uint16_t)(m * 100.0f);
}

void get_mavlink_navigation_mode(const struct vehicle_status_s *const status, uint8_t *mavlink_base_mode,
				 union px4_custom_mode *custom_mode)
{
	custom_mode->data = 0;
	*mavlink_base_mode = 0;

	/* HIL */
	if (status->hil_state == vehicle_status_s::HIL_STATE_ON) {
		*mavlink_base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
	}

	/* arming state */
	if (status->arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		*mavlink_base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
	}

	/* main state */
	*mavlink_base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

	const uint8_t auto_mode_flags	= MAV_MODE_FLAG_AUTO_ENABLED
					  | MAV_MODE_FLAG_STABILIZE_ENABLED
					  | MAV_MODE_FLAG_GUIDED_ENABLED;

	switch (status->nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		*mavlink_base_mode	|= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
					   | (status->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING ? MAV_MODE_FLAG_STABILIZE_ENABLED : 0);
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_MANUAL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ACRO:
		*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_ACRO;
		break;

	case vehicle_status_s::NAVIGATION_STATE_RATTITUDE:
		*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_RATTITUDE;
		break;

	case vehicle_status_s::NAVIGATION_STATE_STAB:
		*mavlink_base_mode	|= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
					   | MAV_MODE_FLAG_STABILIZE_ENABLED;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_STABILIZED;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
		*mavlink_base_mode	|= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
					   | MAV_MODE_FLAG_STABILIZE_ENABLED;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_ALTCTL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_POSCTL:
		*mavlink_base_mode	|= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
					   | MAV_MODE_FLAG_STABILIZE_ENABLED
					   | MAV_MODE_FLAG_GUIDED_ENABLED; // TODO: is POSCTL GUIDED?
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_POSCTL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ORBIT:
		*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
				      | MAV_MODE_FLAG_STABILIZE_ENABLED
				      | MAV_MODE_FLAG_GUIDED_ENABLED;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_POSCTL;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_POSCTL_ORBIT;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_MISSION;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LOITER;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_RTL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDGPSFAIL:

	/* fallthrough */
	case vehicle_status_s::NAVIGATION_STATE_DESCEND:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LAND;
		break;

	case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
		*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_MANUAL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_OFFBOARD;
		break;

	case vehicle_status_s::NAVIGATION_STATE_MAX:
		/* this is an unused case, ignore */
		break;

	}
}

static void get_mavlink_mode_state(const struct vehicle_status_s *const status, uint8_t *mavlink_state,
				   uint8_t *mavlink_base_mode, uint32_t *mavlink_custom_mode)
{
	*mavlink_state = 0;
	*mavlink_base_mode = 0;
	*mavlink_custom_mode = 0;

	union px4_custom_mode custom_mode;
	get_mavlink_navigation_mode(status, mavlink_base_mode, &custom_mode);
	*mavlink_custom_mode = custom_mode.data;

	/* set system state */
	if (status->arming_state == vehicle_status_s::ARMING_STATE_INIT
	    || status->arming_state == vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE
	    || status->arming_state == vehicle_status_s::ARMING_STATE_STANDBY_ERROR) {	// TODO review
		*mavlink_state = MAV_STATE_UNINIT;

	} else if (status->arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		*mavlink_state = MAV_STATE_ACTIVE;

	} else if (status->arming_state == vehicle_status_s::ARMING_STATE_STANDBY) {
		*mavlink_state = MAV_STATE_STANDBY;

	} else if (status->arming_state == vehicle_status_s::ARMING_STATE_SHUTDOWN) {
		*mavlink_state = MAV_STATE_POWEROFF;

	} else {
		*mavlink_state = MAV_STATE_CRITICAL;
	}
}


class MavlinkStreamHeartbeat : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamHeartbeat::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "HEARTBEAT";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_HEARTBEAT;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamHeartbeat(mavlink);
	}

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_HEARTBEAT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

	bool const_rate() override
	{
		return true;
	}

private:
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};

	/* do not allow top copying this class */
	MavlinkStreamHeartbeat(MavlinkStreamHeartbeat &) = delete;
	MavlinkStreamHeartbeat &operator = (const MavlinkStreamHeartbeat &) = delete;

protected:
	explicit MavlinkStreamHeartbeat(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		// always send the heartbeat, independent of the update status of the topics
		vehicle_status_s status{};
		_status_sub.copy(&status);

		uint8_t base_mode = 0;
		uint32_t custom_mode = 0;
		uint8_t system_status = 0;
		get_mavlink_mode_state(&status, &system_status, &base_mode, &custom_mode);

		mavlink_msg_heartbeat_send(_mavlink->get_channel(), _mavlink->get_system_type(), MAV_AUTOPILOT_PX4,
					   base_mode, custom_mode, system_status);

		return true;
	}
};

class MavlinkStreamStatustext : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamStatustext::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "STATUSTEXT";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_STATUSTEXT;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamStatustext(mavlink);
	}

	unsigned get_size() override
	{
		return _mavlink->get_logbuffer()->empty() ? 0 : (MAVLINK_MSG_ID_STATUSTEXT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES);
	}

private:
	/* do not allow top copying this class */
	MavlinkStreamStatustext(MavlinkStreamStatustext &) = delete;
	MavlinkStreamStatustext &operator = (const MavlinkStreamStatustext &) = delete;

protected:
	int _id{0};

	explicit MavlinkStreamStatustext(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		if (!_mavlink->get_logbuffer()->empty() && _mavlink->is_connected()) {

			mavlink_log_s mavlink_log{};

			if (_mavlink->get_logbuffer()->get(&mavlink_log)) {

				mavlink_statustext_t msg{};
				const char *text = mavlink_log.text;
				constexpr unsigned max_chunk_size = sizeof(msg.text);
				msg.severity = mavlink_log.severity;
				msg.chunk_seq = 0;
				msg.id = _id++;
				unsigned text_size;

				while ((text_size = strlen(text)) > 0) {
					unsigned chunk_size = math::min(text_size, max_chunk_size);

					if (chunk_size < max_chunk_size) {
						memcpy(&msg.text[0], &text[0], chunk_size);
						// pad with zeros
						memset(&msg.text[0] + chunk_size, 0, max_chunk_size - chunk_size);

					} else {
						memcpy(&msg.text[0], &text[0], chunk_size);
					}

					mavlink_msg_statustext_send_struct(_mavlink->get_channel(), &msg);

					if (text_size <= max_chunk_size) {
						break;

					} else {
						text += max_chunk_size;
					}

					msg.chunk_seq += 1;
				}

				return true;
			}
		}

		return false;
	}
};

class MavlinkStreamCommandLong : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamCommandLong::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "COMMAND_LONG";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_COMMAND_LONG;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamCommandLong(mavlink);
	}

	unsigned get_size() override
	{
		return 0;	// commands stream is not regular and not predictable
	}

private:
	uORB::Subscription _cmd_sub{ORB_ID(vehicle_command)};

	/* do not allow top copying this class */
	MavlinkStreamCommandLong(MavlinkStreamCommandLong &) = delete;
	MavlinkStreamCommandLong &operator = (const MavlinkStreamCommandLong &) = delete;

protected:
	explicit MavlinkStreamCommandLong(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		struct vehicle_command_s cmd;
		bool sent = false;

		if (_cmd_sub.update(&cmd)) {

			if (!cmd.from_external) {
				PX4_DEBUG("sending command %d to %d/%d", cmd.command, cmd.target_system, cmd.target_component);

				MavlinkCommandSender::instance().handle_vehicle_command(cmd, _mavlink->get_channel());
				sent = true;

			} else {
				PX4_DEBUG("not forwarding command %d to %d/%d", cmd.command, cmd.target_system, cmd.target_component);
			}
		}

		MavlinkCommandSender::instance().check_timeout(_mavlink->get_channel());

		return sent;
	}
};

class MavlinkStreamSysStatus : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamSysStatus::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "SYS_STATUS";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SYS_STATUS;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamSysStatus(mavlink);
	}

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_SYS_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _cpuload_sub{ORB_ID(cpuload)};
	uORB::Subscription _battery_status_sub[ORB_MULTI_MAX_INSTANCES] {
		{ORB_ID(battery_status), 0}, {ORB_ID(battery_status), 1}, {ORB_ID(battery_status), 2}, {ORB_ID(battery_status), 3}
	};

	/* do not allow top copying this class */
	MavlinkStreamSysStatus(MavlinkStreamSysStatus &) = delete;
	MavlinkStreamSysStatus &operator = (const MavlinkStreamSysStatus &) = delete;

protected:
	explicit MavlinkStreamSysStatus(Mavlink *mavlink) : MavlinkStream(mavlink)
	{
	}

	bool send(const hrt_abstime t) override
	{
		bool updated_battery = false;

		for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
			if (_battery_status_sub[i].updated()) {
				updated_battery = true;
			}
		}

		if (_status_sub.updated() || _cpuload_sub.updated() || updated_battery) {
			vehicle_status_s status{};
			_status_sub.copy(&status);

			cpuload_s cpuload{};
			_cpuload_sub.copy(&cpuload);

			battery_status_s battery_status[ORB_MULTI_MAX_INSTANCES] {};

			for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
				_battery_status_sub[i].copy(&battery_status[i]);
			}

			int lowest_battery_index = 0;

			for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
				if (battery_status[i].connected && (battery_status[i].remaining < battery_status[lowest_battery_index].remaining)) {
					lowest_battery_index = i;
				}
			}

			mavlink_sys_status_t msg{};

			msg.onboard_control_sensors_present = status.onboard_control_sensors_present;
			msg.onboard_control_sensors_enabled = status.onboard_control_sensors_enabled;
			msg.onboard_control_sensors_health = status.onboard_control_sensors_health;

			msg.load = cpuload.load * 1000.0f;

			// TODO: Determine what data should be put here when there are multiple batteries.
			//  Right now, it uses the lowest battery. This is a safety decision, because if a client is only checking
			//  one battery using this message, it should be the lowest.
			//  In the future, this should somehow determine the "main" battery, or use the "type" field of BATTERY_STATUS
			//  to determine which battery is more important at a given time.
			const battery_status_s &lowest_battery = battery_status[lowest_battery_index];

			if (lowest_battery.connected) {
				msg.voltage_battery = lowest_battery.voltage_filtered_v * 1000.0f;
				msg.current_battery = lowest_battery.current_filtered_a * 100.0f;
				msg.battery_remaining = ceilf(lowest_battery.remaining * 100.0f);

			} else {
				msg.voltage_battery = UINT16_MAX;
				msg.current_battery = -1;
				msg.battery_remaining = -1;
			}

			mavlink_msg_sys_status_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamBatteryStatus : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamBatteryStatus::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "BATTERY_STATUS";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_BATTERY_STATUS;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamBatteryStatus(mavlink);
	}

	unsigned get_size() override
	{
		unsigned total_size = 0;

		for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
			if (_battery_status_sub[i].advertised()) {
				total_size += MAVLINK_MSG_ID_BATTERY_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
			}
		}

		return total_size;
	}

private:
	uORB::Subscription _battery_status_sub[ORB_MULTI_MAX_INSTANCES] {
		{ORB_ID(battery_status), 0}, {ORB_ID(battery_status), 1}, {ORB_ID(battery_status), 2}, {ORB_ID(battery_status), 3}
	};

	/* do not allow top copying this class */
	MavlinkStreamBatteryStatus(MavlinkStreamSysStatus &) = delete;
	MavlinkStreamBatteryStatus &operator = (const MavlinkStreamSysStatus &) = delete;

protected:
	explicit MavlinkStreamBatteryStatus(Mavlink *mavlink) : MavlinkStream(mavlink)
	{
	}

	bool send(const hrt_abstime t) override
	{
		bool updated = false;

		for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
			battery_status_s battery_status;

			if (_battery_status_sub[i].update(&battery_status)) {
				/* battery status message with higher resolution */
				mavlink_battery_status_t bat_msg{};
				// TODO: Determine how to better map between battery ID within the firmware and in MAVLink
				bat_msg.id = battery_status.id - 1;
				bat_msg.battery_function = MAV_BATTERY_FUNCTION_ALL;
				bat_msg.type = MAV_BATTERY_TYPE_LIPO;
				bat_msg.current_consumed = (battery_status.connected) ? battery_status.discharged_mah : -1;
				bat_msg.energy_consumed = -1;
				bat_msg.current_battery = (battery_status.connected) ? battery_status.current_filtered_a * 100 : -1;
				bat_msg.battery_remaining = (battery_status.connected) ? ceilf(battery_status.remaining * 100.0f) : -1;
				bat_msg.time_remaining = (battery_status.connected) ? battery_status.run_time_to_empty * 60 : 0;

				switch (battery_status.warning) {
				case (battery_status_s::BATTERY_WARNING_NONE):
					bat_msg.charge_state = MAV_BATTERY_CHARGE_STATE_OK;
					break;

				case (battery_status_s::BATTERY_WARNING_LOW):
					bat_msg.charge_state = MAV_BATTERY_CHARGE_STATE_LOW;
					break;

				case (battery_status_s::BATTERY_WARNING_CRITICAL):
					bat_msg.charge_state = MAV_BATTERY_CHARGE_STATE_CRITICAL;
					break;

				case (battery_status_s::BATTERY_WARNING_EMERGENCY):
					bat_msg.charge_state = MAV_BATTERY_CHARGE_STATE_EMERGENCY;
					break;

				case (battery_status_s::BATTERY_WARNING_FAILED):
					bat_msg.charge_state = MAV_BATTERY_CHARGE_STATE_FAILED;
					break;

				default:
					bat_msg.charge_state = MAV_BATTERY_CHARGE_STATE_UNDEFINED;
					break;
				}

				// check if temperature valid
				if (battery_status.connected && PX4_ISFINITE(battery_status.temperature)) {
					bat_msg.temperature = battery_status.temperature * 100.0f;

				} else {
					bat_msg.temperature = INT16_MAX;
				}

				static constexpr int mavlink_cells_max = (sizeof(bat_msg.voltages) / sizeof(bat_msg.voltages[0]));
				static constexpr int uorb_cells_max =
					(sizeof(battery_status.voltage_cell_v) / sizeof(battery_status.voltage_cell_v[0]));

				for (int cell = 0; cell < mavlink_cells_max; cell++) {
					if (battery_status.connected && (cell < battery_status.cell_count) && (cell < uorb_cells_max)) {
						bat_msg.voltages[cell] = battery_status.voltage_cell_v[cell] * 1000.0f;

					} else {
						bat_msg.voltages[cell] = UINT16_MAX;
					}
				}

				mavlink_msg_battery_status_send_struct(_mavlink->get_channel(), &bat_msg);

				updated = true;
			}

		}

		return updated;
	}
};


class MavlinkStreamSmartBatteryInfo : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamSmartBatteryInfo::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "SMART_BATTERY_INFO";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SMART_BATTERY_INFO;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamSmartBatteryInfo(mavlink);
	}

	unsigned get_size() override
	{
		unsigned total_size = 0;

		for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
			if (_battery_status_sub[i].advertised()) {
				total_size += MAVLINK_MSG_ID_BATTERY_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
			}
		}

		return total_size;
	}

private:
	uORB::Subscription _battery_status_sub[ORB_MULTI_MAX_INSTANCES] {
		{ORB_ID(battery_status), 0}, {ORB_ID(battery_status), 1}, {ORB_ID(battery_status), 2}, {ORB_ID(battery_status), 3}
	};

	/* do not allow top copying this class */
	MavlinkStreamSmartBatteryInfo(MavlinkStreamSysStatus &) = delete;
	MavlinkStreamSmartBatteryInfo &operator = (const MavlinkStreamSysStatus &) = delete;

protected:
	explicit MavlinkStreamSmartBatteryInfo(Mavlink *mavlink) : MavlinkStream(mavlink)
	{
	}

	bool send(const hrt_abstime t) override
	{
		bool updated = false;

		for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
			battery_status_s battery_status;

			if (_battery_status_sub[i].update(&battery_status)) {
				if (battery_status.serial_number == 0) {
					//This is not smart battery
					continue;
				}

				mavlink_smart_battery_info_t msg = {};

				msg.id = battery_status.id - 1;
				msg.capacity_full_specification = battery_status.capacity;
				msg.capacity_full = (int32_t)((float)(battery_status.state_of_health * battery_status.capacity) / 100.f);
				msg.cycle_count = battery_status.cycle_count;
				msg.serial_number = battery_status.serial_number + (battery_status.manufacture_date << 16);
				//msg.device_name = ??
				msg.weight = -1;
				msg.discharge_minimum_voltage = -1;
				msg.charging_minimum_voltage = -1;
				msg.resting_minimum_voltage = -1;


				mavlink_msg_smart_battery_info_send_struct(_mavlink->get_channel(), &msg);

				updated = true;
			}

		}

		return updated;
	}
};

class MavlinkStreamHighresIMU : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamHighresIMU::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "HIGHRES_IMU";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_HIGHRES_IMU;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamHighresIMU(mavlink);
	}

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_HIGHRES_IMU_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	uORB::Subscription _vehicle_imu_sub[ORB_MULTI_MAX_INSTANCES] {{ORB_ID(vehicle_imu), 0}, {ORB_ID(vehicle_imu), 1}, {ORB_ID(vehicle_imu), 2}, {ORB_ID(vehicle_imu), 3}};
	uORB::Subscription _sensor_selection_sub{ORB_ID(sensor_selection)};
	uORB::Subscription _bias_sub{ORB_ID(estimator_sensor_bias)};
	uORB::Subscription _differential_pressure_sub{ORB_ID(differential_pressure)};
	uORB::Subscription _magnetometer_sub{ORB_ID(vehicle_magnetometer)};
	uORB::Subscription _air_data_sub{ORB_ID(vehicle_air_data)};

	/* do not allow top copying this class */
	MavlinkStreamHighresIMU(MavlinkStreamHighresIMU &) = delete;
	MavlinkStreamHighresIMU &operator = (const MavlinkStreamHighresIMU &) = delete;

protected:
	explicit MavlinkStreamHighresIMU(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		bool updated = false;

		sensor_selection_s sensor_selection{};
		_sensor_selection_sub.copy(&sensor_selection);

		vehicle_imu_s imu;

		for (auto &imu_sub : _vehicle_imu_sub) {
			if (imu_sub.update(&imu)) {
				if (imu.accel_device_id == sensor_selection.accel_device_id) {
					updated = true;
					break;
				}
			}
		}

		if (updated) {
			uint16_t fields_updated = 0;

			fields_updated |= (1 << 0) | (1 << 1) | (1 << 2); // accel
			fields_updated |= (1 << 3) | (1 << 4) | (1 << 5); // gyro

			vehicle_magnetometer_s magnetometer{};

			if (_magnetometer_sub.update(&magnetometer)) {
				/* mark third group dimensions as changed */
				fields_updated |= (1 << 6) | (1 << 7) | (1 << 8);

			} else {
				_magnetometer_sub.copy(&magnetometer);
			}

			vehicle_air_data_s air_data{};

			if (_air_data_sub.update(&air_data)) {
				/* mark fourth group (baro fields) dimensions as changed */
				fields_updated |= (1 << 9) | (1 << 11) | (1 << 12);

			} else {
				_air_data_sub.copy(&air_data);
			}

			differential_pressure_s differential_pressure{};

			if (_differential_pressure_sub.update(&differential_pressure)) {
				/* mark fourth group (dpres field) dimensions as changed */
				fields_updated |= (1 << 10);

			} else {
				_differential_pressure_sub.copy(&differential_pressure);
			}

			estimator_sensor_bias_s bias{};
			_bias_sub.copy(&bias);

			const float accel_dt_inv = 1.e6f / (float)imu.delta_velocity_dt;
			Vector3f accel = (Vector3f{imu.delta_velocity} * accel_dt_inv) - Vector3f{bias.accel_bias};

			const float gyro_dt_inv = 1.e6f / (float)imu.delta_angle_dt;
			Vector3f gyro = (Vector3f{imu.delta_angle} * gyro_dt_inv) - Vector3f{bias.gyro_bias};

			mavlink_highres_imu_t msg{};

			msg.time_usec = imu.timestamp_sample;
			msg.xacc = accel(0);
			msg.yacc = accel(1);
			msg.zacc = accel(2);
			msg.xgyro = gyro(0);
			msg.ygyro = gyro(1);
			msg.zgyro = gyro(2);
			msg.xmag = magnetometer.magnetometer_ga[0] - bias.mag_bias[0];
			msg.ymag = magnetometer.magnetometer_ga[1] - bias.mag_bias[1];
			msg.zmag = magnetometer.magnetometer_ga[2] - bias.mag_bias[2];
			msg.abs_pressure = air_data.baro_pressure_pa;
			msg.diff_pressure = differential_pressure.differential_pressure_raw_pa;
			msg.pressure_alt = air_data.baro_alt_meter;
			msg.temperature = air_data.baro_temp_celcius;
			msg.fields_updated = fields_updated;

			mavlink_msg_highres_imu_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

template <int N, typename Derived>
class MavlinkStreamScaledPressureBase : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return Derived::get_name_static();
	}

	uint16_t get_id() override
	{
		return Derived::get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new Derived(mavlink);
	}

private:
	uORB::Subscription _differential_pressure_sub{ORB_ID(differential_pressure)};
	uORB::Subscription _sensor_baro_sub{ORB_ID(sensor_baro), N};

	/* do not allow top copying this class */
	MavlinkStreamScaledPressureBase(MavlinkStreamScaledPressureBase &) = delete;
	MavlinkStreamScaledPressureBase &operator = (const MavlinkStreamScaledPressureBase &) = delete;

protected:
	explicit MavlinkStreamScaledPressureBase(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		if (_sensor_baro_sub.updated() || _differential_pressure_sub.updated()) {
			sensor_baro_s sensor_baro{};
			differential_pressure_s differential_pressure{};
			_sensor_baro_sub.copy(&sensor_baro);
			_differential_pressure_sub.copy(&differential_pressure);

			typename Derived::mav_msg_type msg{};
			msg.time_boot_ms = sensor_baro.timestamp / 1000;
			msg.press_abs = sensor_baro.pressure;
			msg.press_diff = differential_pressure.differential_pressure_raw_pa;
			msg.temperature = sensor_baro.temperature;

			Derived::send(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

template <int N> class MavlinkStreamScaledPressure {};

template <>
class MavlinkStreamScaledPressure<0> : public MavlinkStreamScaledPressureBase<0, MavlinkStreamScaledPressure<0> >
{
public:
	typedef MavlinkStreamScaledPressureBase<0, MavlinkStreamScaledPressure<0> > Base;
	typedef mavlink_scaled_pressure_t mav_msg_type;

	explicit MavlinkStreamScaledPressure(Mavlink *mavlink) : Base(mavlink) {}

	static void send(mavlink_channel_t channel, const MavlinkStreamScaledPressure<0>::mav_msg_type *msg)
	{
		mavlink_msg_scaled_pressure_send_struct(channel, msg);
	}

	static const char *get_name_static()
	{
		return "SCALED_PRESSURE";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SCALED_PRESSURE;
	}

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_SCALED_PRESSURE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}
};

template <>
class MavlinkStreamScaledPressure<1> : public MavlinkStreamScaledPressureBase<1, MavlinkStreamScaledPressure<1> >
{
public:
	typedef MavlinkStreamScaledPressureBase<1, MavlinkStreamScaledPressure<1> > Base;
	typedef mavlink_scaled_pressure2_t mav_msg_type;

	explicit MavlinkStreamScaledPressure(Mavlink *mavlink) : Base(mavlink) {}

	static void send(mavlink_channel_t channel, const MavlinkStreamScaledPressure<1>::mav_msg_type *msg)
	{
		mavlink_msg_scaled_pressure2_send_struct(channel, msg);
	}

	static const char *get_name_static()
	{
		return "SCALED_PRESSURE2";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SCALED_PRESSURE2;
	}

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_SCALED_PRESSURE2_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}
};

template <>
class MavlinkStreamScaledPressure<2> : public MavlinkStreamScaledPressureBase<2, MavlinkStreamScaledPressure<2> >
{
public:
	typedef MavlinkStreamScaledPressureBase<2, MavlinkStreamScaledPressure<2> > Base;
	typedef mavlink_scaled_pressure3_t mav_msg_type;

	explicit MavlinkStreamScaledPressure(Mavlink *mavlink) : Base(mavlink) {}

	static void send(mavlink_channel_t channel, const MavlinkStreamScaledPressure<2>::mav_msg_type *msg)
	{
		mavlink_msg_scaled_pressure3_send_struct(channel, msg);
	}

	static const char *get_name_static()
	{
		return "SCALED_PRESSURE3";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SCALED_PRESSURE3;
	}

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_SCALED_PRESSURE3_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}
};

class MavlinkStreamScaledIMU : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamScaledIMU::get_name_static();
	}

	static const char *get_name_static()
	{
		return "SCALED_IMU";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SCALED_IMU;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamScaledIMU(mavlink);
	}

	unsigned get_size() override
	{
		return _raw_imu_sub.advertised() ? (MAVLINK_MSG_ID_SCALED_IMU_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	uORB::Subscription _raw_imu_sub{ORB_ID(vehicle_imu), 0};
	uORB::Subscription _raw_mag_sub{ORB_ID(sensor_mag), 0};

	// do not allow top copy this class
	MavlinkStreamScaledIMU(MavlinkStreamScaledIMU &) = delete;
	MavlinkStreamScaledIMU &operator = (const MavlinkStreamScaledIMU &) = delete;

protected:
	explicit MavlinkStreamScaledIMU(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		if (_raw_imu_sub.updated() || _raw_mag_sub.updated()) {

			vehicle_imu_s imu{};
			_raw_imu_sub.copy(&imu);

			sensor_mag_s sensor_mag{};
			_raw_mag_sub.copy(&sensor_mag);

			mavlink_scaled_imu_t msg{};

			msg.time_boot_ms = imu.timestamp / 1000;

			// Accelerometer in mG
			const float accel_dt_inv = 1.e6f / (float)imu.delta_velocity_dt;
			const Vector3f accel = Vector3f{imu.delta_velocity} * accel_dt_inv * 1000.0f / CONSTANTS_ONE_G;

			// Gyroscope in mrad/s
			const float gyro_dt_inv = 1.e6f / (float)imu.delta_angle_dt;
			const Vector3f gyro = Vector3f{imu.delta_angle} * gyro_dt_inv * 1000.0f;

			msg.xacc = (int16_t)accel(0);
			msg.yacc = (int16_t)accel(1);
			msg.zacc = (int16_t)accel(2);
			msg.xgyro = gyro(0);
			msg.ygyro = gyro(1);
			msg.zgyro = gyro(2);
			msg.xmag = sensor_mag.x * 1000.0f; // Gauss -> MilliGauss
			msg.ymag = sensor_mag.y * 1000.0f; // Gauss -> MilliGauss
			msg.zmag = sensor_mag.z * 1000.0f; // Gauss -> MilliGauss

			mavlink_msg_scaled_imu_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamScaledIMU2 : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamScaledIMU2::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "SCALED_IMU2";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SCALED_IMU2;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamScaledIMU2(mavlink);
	}

	unsigned get_size() override
	{
		return _raw_imu_sub.advertised() ? (MAVLINK_MSG_ID_SCALED_IMU2_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	uORB::Subscription _raw_imu_sub{ORB_ID(vehicle_imu), 1};
	uORB::Subscription _raw_mag_sub{ORB_ID(sensor_mag), 1};

	// do not allow top copy this class
	MavlinkStreamScaledIMU2(MavlinkStreamScaledIMU2 &) = delete;
	MavlinkStreamScaledIMU2 &operator = (const MavlinkStreamScaledIMU2 &) = delete;

protected:
	explicit MavlinkStreamScaledIMU2(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		if (_raw_imu_sub.updated() || _raw_mag_sub.updated()) {

			vehicle_imu_s imu{};
			_raw_imu_sub.copy(&imu);

			sensor_mag_s sensor_mag{};
			_raw_mag_sub.copy(&sensor_mag);

			mavlink_scaled_imu2_t msg{};

			msg.time_boot_ms = imu.timestamp / 1000;

			// Accelerometer in mG
			const float accel_dt_inv = 1.e6f / (float)imu.delta_velocity_dt;
			const Vector3f accel = Vector3f{imu.delta_velocity} * accel_dt_inv * 1000.0f / CONSTANTS_ONE_G;

			// Gyroscope in mrad/s
			const float gyro_dt_inv = 1.e6f / (float)imu.delta_angle_dt;
			const Vector3f gyro = Vector3f{imu.delta_angle} * gyro_dt_inv * 1000.0f;

			msg.xacc = (int16_t)accel(0);
			msg.yacc = (int16_t)accel(1);
			msg.zacc = (int16_t)accel(2);
			msg.xgyro = gyro(0);
			msg.ygyro = gyro(1);
			msg.zgyro = gyro(2);
			msg.xmag = sensor_mag.x * 1000.0f; // Gauss -> MilliGauss
			msg.ymag = sensor_mag.y * 1000.0f; // Gauss -> MilliGauss
			msg.zmag = sensor_mag.z * 1000.0f; // Gauss -> MilliGauss

			mavlink_msg_scaled_imu2_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamScaledIMU3 : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamScaledIMU3::get_name_static();
	}
	static constexpr const char *get_name_static()
	{
		return "SCALED_IMU3";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SCALED_IMU3;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamScaledIMU3(mavlink);
	}

	unsigned get_size() override
	{
		return _raw_imu_sub.advertised() ? (MAVLINK_MSG_ID_SCALED_IMU3_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	uORB::Subscription _raw_imu_sub{ORB_ID(vehicle_imu), 2};
	uORB::Subscription _raw_mag_sub{ORB_ID(sensor_mag), 2};

	// do not allow top copy this class
	MavlinkStreamScaledIMU3(MavlinkStreamScaledIMU3 &) = delete;
	MavlinkStreamScaledIMU3 &operator = (const MavlinkStreamScaledIMU3 &) = delete;

protected:
	explicit MavlinkStreamScaledIMU3(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		if (_raw_imu_sub.updated() || _raw_mag_sub.updated()) {

			vehicle_imu_s imu{};
			_raw_imu_sub.copy(&imu);

			sensor_mag_s sensor_mag{};
			_raw_mag_sub.copy(&sensor_mag);

			mavlink_scaled_imu3_t msg{};

			msg.time_boot_ms = imu.timestamp / 1000;

			// Accelerometer in mG
			const float accel_dt_inv = 1.e6f / (float)imu.delta_velocity_dt;
			const Vector3f accel = Vector3f{imu.delta_velocity} * accel_dt_inv * 1000.0f / CONSTANTS_ONE_G;

			// Gyroscope in mrad/s
			const float gyro_dt_inv = 1.e6f / (float)imu.delta_angle_dt;
			const Vector3f gyro = Vector3f{imu.delta_angle} * gyro_dt_inv * 1000.0f;

			msg.xacc = (int16_t)accel(0);
			msg.yacc = (int16_t)accel(1);
			msg.zacc = (int16_t)accel(2);
			msg.xgyro = gyro(0);
			msg.ygyro = gyro(1);
			msg.zgyro = gyro(2);
			msg.xmag = sensor_mag.x * 1000.0f; // Gauss -> MilliGauss
			msg.ymag = sensor_mag.y * 1000.0f; // Gauss -> MilliGauss
			msg.zmag = sensor_mag.z * 1000.0f; // Gauss -> MilliGauss

			mavlink_msg_scaled_imu3_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};


class MavlinkStreamAttitude : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamAttitude::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "ATTITUDE";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ATTITUDE;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamAttitude(mavlink);
	}

	unsigned get_size() override
	{
		return _att_sub.advertised() ? MAVLINK_MSG_ID_ATTITUDE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};

	/* do not allow top copying this class */
	MavlinkStreamAttitude(MavlinkStreamAttitude &) = delete;
	MavlinkStreamAttitude &operator = (const MavlinkStreamAttitude &) = delete;


protected:
	explicit MavlinkStreamAttitude(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		vehicle_attitude_s att;

		if (_att_sub.update(&att)) {
			vehicle_angular_velocity_s angular_velocity{};
			_angular_velocity_sub.copy(&angular_velocity);

			mavlink_attitude_t msg{};

			const matrix::Eulerf euler = matrix::Quatf(att.q);
			msg.time_boot_ms = att.timestamp / 1000;
			msg.roll = euler.phi();
			msg.pitch = euler.theta();
			msg.yaw = euler.psi();

			msg.rollspeed = angular_velocity.xyz[0];
			msg.pitchspeed = angular_velocity.xyz[1];
			msg.yawspeed = angular_velocity.xyz[2];

			mavlink_msg_attitude_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};


class MavlinkStreamAttitudeQuaternion : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamAttitudeQuaternion::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "ATTITUDE_QUATERNION";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ATTITUDE_QUATERNION;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamAttitudeQuaternion(mavlink);
	}

	unsigned get_size() override
	{
		return _att_sub.advertised() ? MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};

	/* do not allow top copying this class */
	MavlinkStreamAttitudeQuaternion(MavlinkStreamAttitudeQuaternion &) = delete;
	MavlinkStreamAttitudeQuaternion &operator = (const MavlinkStreamAttitudeQuaternion &) = delete;

protected:
	explicit MavlinkStreamAttitudeQuaternion(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		vehicle_attitude_s att;

		if (_att_sub.update(&att)) {
			vehicle_angular_velocity_s angular_velocity{};
			_angular_velocity_sub.copy(&angular_velocity);

			vehicle_status_s status{};
			_status_sub.copy(&status);

			mavlink_attitude_quaternion_t msg{};

			msg.time_boot_ms = att.timestamp / 1000;
			msg.q1 = att.q[0];
			msg.q2 = att.q[1];
			msg.q3 = att.q[2];
			msg.q4 = att.q[3];
			msg.rollspeed = angular_velocity.xyz[0];
			msg.pitchspeed = angular_velocity.xyz[1];
			msg.yawspeed = angular_velocity.xyz[2];

			if (status.is_vtol && status.is_vtol_tailsitter && (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING)) {
				// This is a tailsitter VTOL flying in fixed wing mode:
				// indicate that reported attitude should be rotated by
				// 90 degrees upward pitch for user display
				get_rot_quaternion(ROTATION_PITCH_90).copyTo(msg.repr_offset_q);

			} else {
				// Normal case
				// zero rotation should be [1 0 0 0]:
				// `get_rot_quaternion(ROTATION_NONE).copyTo(msg.repr_offset_q);`
				// but to save bandwidth, we instead send [0, 0, 0, 0].
				msg.repr_offset_q[0] = 0.0f;
				msg.repr_offset_q[1] = 0.0f;
				msg.repr_offset_q[2] = 0.0f;
				msg.repr_offset_q[3] = 0.0f;
			}

			mavlink_msg_attitude_quaternion_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamVFRHUD : public MavlinkStream
{
public:

	const char *get_name() const override
	{
		return MavlinkStreamVFRHUD::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "VFR_HUD";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_VFR_HUD;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamVFRHUD(mavlink);
	}

	unsigned get_size() override
	{
		if (_lpos_sub.advertised() || _airspeed_validated_sub.advertised()) {
			return MAVLINK_MSG_ID_VFR_HUD_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		}

		return 0;
	}

private:
	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _act0_sub{ORB_ID(actuator_controls_0)};
	uORB::Subscription _act1_sub{ORB_ID(actuator_controls_1)};
	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};
	uORB::Subscription _air_data_sub{ORB_ID(vehicle_air_data)};

	/* do not allow top copying this class */
	MavlinkStreamVFRHUD(MavlinkStreamVFRHUD &) = delete;
	MavlinkStreamVFRHUD &operator = (const MavlinkStreamVFRHUD &) = delete;

protected:
	explicit MavlinkStreamVFRHUD(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		if (_lpos_sub.updated() || _airspeed_validated_sub.updated()) {

			vehicle_local_position_s lpos{};
			_lpos_sub.copy(&lpos);

			actuator_armed_s armed{};
			_armed_sub.copy(&armed);

			airspeed_validated_s airspeed_validated{};
			_airspeed_validated_sub.copy(&airspeed_validated);

			mavlink_vfr_hud_t msg{};
			msg.airspeed = airspeed_validated.equivalent_airspeed_m_s;
			msg.groundspeed = sqrtf(lpos.vx * lpos.vx + lpos.vy * lpos.vy);
			msg.heading = math::degrees(wrap_2pi(lpos.heading));

			if (armed.armed) {
				actuator_controls_s act0{};
				actuator_controls_s act1{};
				_act0_sub.copy(&act0);
				_act1_sub.copy(&act1);

				// VFR_HUD throttle should only be used for operator feedback.
				// VTOLs switch between actuator_controls_0 and actuator_controls_1. During transition there isn't a
				// a single throttle value, but this should still be a useful heuristic for operator awareness.
				//
				// Use ACTUATOR_CONTROL_TARGET if accurate states are needed.
				msg.throttle = 100 * math::max(
						       act0.control[actuator_controls_s::INDEX_THROTTLE],
						       act1.control[actuator_controls_s::INDEX_THROTTLE]);

			} else {
				msg.throttle = 0.0f;
			}

			if (lpos.z_valid && lpos.z_global) {
				/* use local position estimate */
				msg.alt = -lpos.z + lpos.ref_alt;

			} else {
				vehicle_air_data_s air_data{};
				_air_data_sub.copy(&air_data);

				/* fall back to baro altitude */
				if (air_data.timestamp > 0) {
					msg.alt = air_data.baro_alt_meter;
				}
			}

			if (lpos.v_z_valid) {
				msg.climb = -lpos.vz;
			}

			mavlink_msg_vfr_hud_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};


class MavlinkStreamGPSRawInt : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamGPSRawInt::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "GPS_RAW_INT";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_GPS_RAW_INT;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamGPSRawInt(mavlink);
	}

	unsigned get_size() override
	{
		return _gps_sub.advertised() ? MAVLINK_MSG_ID_GPS_RAW_INT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _gps_sub{ORB_ID(vehicle_gps_position)};

	/* do not allow top copying this class */
	MavlinkStreamGPSRawInt(MavlinkStreamGPSRawInt &) = delete;
	MavlinkStreamGPSRawInt &operator = (const MavlinkStreamGPSRawInt &) = delete;

protected:
	explicit MavlinkStreamGPSRawInt(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		vehicle_gps_position_s gps;

		if (_gps_sub.update(&gps)) {
			mavlink_gps_raw_int_t msg{};

			msg.time_usec = gps.timestamp;
			msg.fix_type = gps.fix_type;
			msg.lat = gps.lat;
			msg.lon = gps.lon;
			msg.alt = gps.alt;
			msg.alt_ellipsoid = gps.alt_ellipsoid;
			msg.eph = gps.hdop * 100;
			msg.epv = gps.vdop * 100;
			msg.h_acc = gps.eph * 1e3f;
			msg.v_acc = gps.epv * 1e3f;
			msg.vel_acc = gps.s_variance_m_s * 1e3f;
			msg.hdg_acc = gps.c_variance_rad * 1e5f / M_DEG_TO_RAD_F;
			msg.vel = cm_uint16_from_m_float(gps.vel_m_s);
			msg.cog = math::degrees(wrap_2pi(gps.cog_rad)) * 1e2f;
			msg.satellites_visible = gps.satellites_used;

			mavlink_msg_gps_raw_int_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamGPS2Raw : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamGPS2Raw::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "GPS2_RAW";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_GPS2_RAW;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamGPS2Raw(mavlink);
	}

	unsigned get_size() override
	{
		return _gps2_sub.advertised() ? (MAVLINK_MSG_ID_GPS2_RAW_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	uORB::Subscription _gps2_sub{ORB_ID(vehicle_gps_position), 1};

	/* do not allow top copying this class */
	MavlinkStreamGPS2Raw(MavlinkStreamGPS2Raw &) = delete;
	MavlinkStreamGPS2Raw &operator = (const MavlinkStreamGPS2Raw &) = delete;

protected:
	explicit MavlinkStreamGPS2Raw(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		vehicle_gps_position_s gps;

		if (_gps2_sub.update(&gps)) {
			mavlink_gps2_raw_t msg = {};

			msg.time_usec = gps.timestamp;
			msg.fix_type = gps.fix_type;
			msg.lat = gps.lat;
			msg.lon = gps.lon;
			msg.alt = gps.alt;
			msg.eph = gps.eph * 1e3f;
			msg.epv = gps.epv * 1e3f;
			msg.vel = cm_uint16_from_m_float(gps.vel_m_s);
			msg.cog = math::degrees(wrap_2pi(gps.cog_rad)) * 1e2f;
			msg.satellites_visible = gps.satellites_used;
			//msg.dgps_numch = // Number of DGPS satellites
			//msg.dgps_age = // Age of DGPS info

			mavlink_msg_gps2_raw_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamSystemTime : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamSystemTime::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "SYSTEM_TIME";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SYSTEM_TIME;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamSystemTime(mavlink);
	}

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_SYSTEM_TIME_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	/* do not allow top copying this class */
	MavlinkStreamSystemTime(MavlinkStreamSystemTime &) = delete;
	MavlinkStreamSystemTime &operator = (const MavlinkStreamSystemTime &) = delete;

protected:
	explicit MavlinkStreamSystemTime(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		timespec tv;
		px4_clock_gettime(CLOCK_REALTIME, &tv);

		mavlink_system_time_t msg{};
		msg.time_boot_ms = hrt_absolute_time() / 1000;
		msg.time_unix_usec = (uint64_t)tv.tv_sec * 1000000 + tv.tv_nsec / 1000;

		// If the time is before 2001-01-01, it's probably the default 2000
		// and we don't need to bother sending it because it's definitely wrong.
		if (msg.time_unix_usec > 978307200000000) {
			mavlink_msg_system_time_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

class MavlinkStreamTimesync : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamTimesync::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "TIMESYNC";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_TIMESYNC;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamTimesync(mavlink);
	}

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_TIMESYNC_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	/* do not allow top copying this class */
	MavlinkStreamTimesync(MavlinkStreamTimesync &) = delete;
	MavlinkStreamTimesync &operator = (const MavlinkStreamTimesync &) = delete;

protected:
	explicit MavlinkStreamTimesync(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		mavlink_timesync_t msg{};

		msg.tc1 = 0;
		msg.ts1 = hrt_absolute_time() * 1000; // boot time in nanoseconds

		mavlink_msg_timesync_send_struct(_mavlink->get_channel(), &msg);

		return true;
	}
};

class MavlinkStreamADSBVehicle : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamADSBVehicle::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "ADSB_VEHICLE";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ADSB_VEHICLE;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamADSBVehicle(mavlink);
	}

	bool const_rate() override
	{
		return true;
	}

	unsigned get_size() override
	{
		return _pos_sub.advertised() ? MAVLINK_MSG_ID_ADSB_VEHICLE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _pos_sub{ORB_ID(transponder_report)};

	/* do not allow top copying this class */
	MavlinkStreamADSBVehicle(MavlinkStreamADSBVehicle &) = delete;
	MavlinkStreamADSBVehicle &operator = (const MavlinkStreamADSBVehicle &) = delete;

protected:
	explicit MavlinkStreamADSBVehicle(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		transponder_report_s pos;
		bool sent = false;

		while (_pos_sub.update(&pos)) {

			if (!(pos.flags & transponder_report_s::PX4_ADSB_FLAGS_RETRANSLATE)) {
				continue;
			}

			mavlink_adsb_vehicle_t msg{};
			msg.ICAO_address = pos.icao_address;
			msg.lat = pos.lat * 1e7;
			msg.lon = pos.lon * 1e7;
			msg.altitude_type = pos.altitude_type;
			msg.altitude = pos.altitude * 1e3f;
			msg.heading = (pos.heading + M_PI_F) / M_PI_F * 180.0f * 100.0f;
			msg.hor_velocity = pos.hor_velocity * 100.0f;
			msg.ver_velocity = pos.ver_velocity * 100.0f;
			memcpy(&msg.callsign[0], &pos.callsign[0], sizeof(msg.callsign));
			msg.emitter_type = pos.emitter_type;
			msg.tslc = pos.tslc;
			msg.squawk = pos.squawk;

			msg.flags = 0;

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS) { msg.flags |= ADSB_FLAGS_VALID_COORDS; }

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE) { msg.flags |= ADSB_FLAGS_VALID_ALTITUDE; }

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING) { msg.flags |= ADSB_FLAGS_VALID_HEADING; }

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY) { msg.flags |= ADSB_FLAGS_VALID_VELOCITY; }

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN) { msg.flags |= ADSB_FLAGS_VALID_CALLSIGN; }

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_SQUAWK) { msg.flags |= ADSB_FLAGS_VALID_SQUAWK; }

			mavlink_msg_adsb_vehicle_send_struct(_mavlink->get_channel(), &msg);
			sent = true;
		}

		return sent;
	}
};

class MavlinkStreamUTMGlobalPosition : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamUTMGlobalPosition::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "UTM_GLOBAL_POSITION";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_UTM_GLOBAL_POSITION;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamUTMGlobalPosition(mavlink);
	}

	bool const_rate() override
	{
		return true;
	}

	unsigned get_size() override
	{
		return _global_pos_sub.advertised() ? MAVLINK_MSG_ID_UTM_GLOBAL_POSITION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _global_pos_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _position_setpoint_triplet_sub{ORB_ID(position_setpoint_triplet)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _land_detected_sub{ORB_ID(vehicle_land_detected)};

	/* do not allow top copying this class */
	MavlinkStreamUTMGlobalPosition(MavlinkStreamUTMGlobalPosition &) = delete;
	MavlinkStreamUTMGlobalPosition &operator = (const MavlinkStreamUTMGlobalPosition &) = delete;

protected:
	explicit MavlinkStreamUTMGlobalPosition(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		vehicle_global_position_s global_pos;

		if (_global_pos_sub.update(&global_pos)) {
			mavlink_utm_global_position_t msg{};

			// Compute Unix epoch and set time field
			timespec tv;
			px4_clock_gettime(CLOCK_REALTIME, &tv);
			uint64_t unix_epoch = (uint64_t)tv.tv_sec * 1000000 + tv.tv_nsec / 1000;

			// If the time is before 2001-01-01, it's probably the default 2000
			if (unix_epoch > 978307200000000) {
				msg.time = unix_epoch;
				msg.flags |= UTM_DATA_AVAIL_FLAGS_TIME_VALID;
			}

#ifndef BOARD_HAS_NO_UUID
			px4_guid_t px4_guid;
			board_get_px4_guid(px4_guid);
			static_assert(sizeof(px4_guid_t) == sizeof(msg.uas_id), "GUID byte length mismatch");
			memcpy(&msg.uas_id, &px4_guid, sizeof(msg.uas_id));
			msg.flags |= UTM_DATA_AVAIL_FLAGS_UAS_ID_AVAILABLE;
#else
			// TODO Fill ID with something reasonable
			memset(&msg.uas_id[0], 0, sizeof(msg.uas_id));
#endif /* BOARD_HAS_NO_UUID */

			// Handle global position
			msg.lat = global_pos.lat * 1e7;
			msg.lon = global_pos.lon * 1e7;
			msg.alt = global_pos.alt_ellipsoid * 1000.0f;

			msg.h_acc = global_pos.eph * 1000.0f;
			msg.v_acc = global_pos.epv * 1000.0f;

			msg.flags |= UTM_DATA_AVAIL_FLAGS_POSITION_AVAILABLE;
			msg.flags |= UTM_DATA_AVAIL_FLAGS_ALTITUDE_AVAILABLE;

			// Handle local position
			vehicle_local_position_s local_pos;

			if (_local_pos_sub.copy(&local_pos)) {
				float evh = 0.0f;
				float evv = 0.0f;

				if (local_pos.v_xy_valid) {
					msg.vx = local_pos.vx * 100.0f;
					msg.vy = local_pos.vy * 100.0f;
					evh = local_pos.evh;
					msg.flags |= UTM_DATA_AVAIL_FLAGS_HORIZONTAL_VELO_AVAILABLE;
				}

				if (local_pos.v_z_valid) {
					msg.vz = local_pos.vz * 100.0f;
					evv = local_pos.evv;
					msg.flags |= UTM_DATA_AVAIL_FLAGS_VERTICAL_VELO_AVAILABLE;
				}

				msg.vel_acc = sqrtf(evh * evh + evv * evv) * 100.0f;

				if (local_pos.dist_bottom_valid) {
					msg.relative_alt = local_pos.dist_bottom * 1000.0f;
					msg.flags |= UTM_DATA_AVAIL_FLAGS_RELATIVE_ALTITUDE_AVAILABLE;
				}
			}

			vehicle_status_s vehicle_status{};
			_vehicle_status_sub.copy(&vehicle_status);

			bool vehicle_in_auto_mode = vehicle_status.timestamp > 0
						    && (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET
							|| vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND
							|| vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL
							|| vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND
							|| vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION
							|| vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER
							|| vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF
							|| vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL);

			// Handle next waypoint if it is valid
			position_setpoint_triplet_s position_setpoint_triplet;

			if (vehicle_in_auto_mode && _position_setpoint_triplet_sub.copy(&position_setpoint_triplet)) {
				if (position_setpoint_triplet.current.valid) {
					msg.next_lat = position_setpoint_triplet.current.lat * 1e7;
					msg.next_lon = position_setpoint_triplet.current.lon * 1e7;
					// HACK We assume that the offset between AMSL and WGS84 is constant between the current
					// vehicle position and the the target waypoint.
					msg.next_alt = (position_setpoint_triplet.current.alt + (global_pos.alt_ellipsoid - global_pos.alt)) * 1000.0f;
					msg.flags |= UTM_DATA_AVAIL_FLAGS_NEXT_WAYPOINT_AVAILABLE;
				}
			}

			// Handle flight state
			vehicle_land_detected_s land_detected{};
			_land_detected_sub.copy(&land_detected);

			if (vehicle_status.timestamp > 0 && land_detected.timestamp > 0
			    && vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
				if (land_detected.landed) {
					msg.flight_state |= UTM_FLIGHT_STATE_GROUND;

				} else {
					msg.flight_state |= UTM_FLIGHT_STATE_AIRBORNE;
				}

			} else {
				msg.flight_state |= UTM_FLIGHT_STATE_UNKNOWN;
			}

			msg.update_rate = 0; // Data driven mode

			mavlink_msg_utm_global_position_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamCollision : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamCollision::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "COLLISION";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_COLLISION;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamCollision(mavlink);
	}

	unsigned get_size() override
	{
		return _collision_sub.advertised() ? MAVLINK_MSG_ID_COLLISION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _collision_sub{ORB_ID(collision_report)};

	/* do not allow top copying this class */
	MavlinkStreamCollision(MavlinkStreamCollision &) = delete;
	MavlinkStreamCollision &operator = (const MavlinkStreamCollision &) = delete;

protected:
	explicit MavlinkStreamCollision(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		collision_report_s report;
		bool sent = false;

		while (_collision_sub.update(&report)) {
			mavlink_collision_t msg = {};

			msg.src = report.src;
			msg.id = report.id;
			msg.action = report.action;
			msg.threat_level = report.threat_level;
			msg.time_to_minimum_delta = report.time_to_minimum_delta;
			msg.altitude_minimum_delta = report.altitude_minimum_delta;
			msg.horizontal_minimum_delta = report.horizontal_minimum_delta;

			mavlink_msg_collision_send_struct(_mavlink->get_channel(), &msg);
			sent = true;
		}

		return sent;
	}
};

class MavlinkStreamCameraTrigger : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamCameraTrigger::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "CAMERA_TRIGGER";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_CAMERA_TRIGGER;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamCameraTrigger(mavlink);
	}

	bool const_rate() override
	{
		return true;
	}

	unsigned get_size() override
	{
		return _trigger_sub.advertised() ? MAVLINK_MSG_ID_CAMERA_TRIGGER_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _trigger_sub{ORB_ID(camera_trigger)};

	/* do not allow top copying this class */
	MavlinkStreamCameraTrigger(MavlinkStreamCameraTrigger &) = delete;
	MavlinkStreamCameraTrigger &operator = (const MavlinkStreamCameraTrigger &) = delete;

protected:
	explicit MavlinkStreamCameraTrigger(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		camera_trigger_s trigger;

		if (_trigger_sub.update(&trigger)) {
			mavlink_camera_trigger_t msg{};

			msg.time_usec = trigger.timestamp;
			msg.seq = trigger.seq;

			/* ensure that only active trigger events are sent */
			if (trigger.timestamp > 0) {

				mavlink_msg_camera_trigger_send_struct(_mavlink->get_channel(), &msg);

				vehicle_command_s vcmd{};
				vcmd.timestamp = hrt_absolute_time();
				vcmd.param1 = 0.0f; // all cameras
				vcmd.param2 = 0.0f; // duration 0 because only taking one picture
				vcmd.param3 = 1.0f; // only take one
				vcmd.param4 = NAN;
				vcmd.param5 = (double)NAN;
				vcmd.param6 = (double)NAN;
				vcmd.param7 = NAN;
				vcmd.command = MAV_CMD_IMAGE_START_CAPTURE;
				vcmd.target_system = mavlink_system.sysid;
				vcmd.target_component = MAV_COMP_ID_CAMERA;

				MavlinkCommandSender::instance().handle_vehicle_command(vcmd, _mavlink->get_channel());

				// TODO: move this camera_trigger and publish as a vehicle_command
				/* send MAV_CMD_DO_DIGICAM_CONTROL*/
				mavlink_command_long_t digicam_ctrl_cmd{};

				digicam_ctrl_cmd.target_system = 0; // 0 for broadcast
				digicam_ctrl_cmd.target_component = MAV_COMP_ID_CAMERA;
				digicam_ctrl_cmd.command = MAV_CMD_DO_DIGICAM_CONTROL;
				digicam_ctrl_cmd.confirmation = 0;
				digicam_ctrl_cmd.param1 = NAN;
				digicam_ctrl_cmd.param2 = NAN;
				digicam_ctrl_cmd.param3 = NAN;
				digicam_ctrl_cmd.param4 = NAN;
				digicam_ctrl_cmd.param5 = 1;   // take 1 picture
				digicam_ctrl_cmd.param6 = NAN;
				digicam_ctrl_cmd.param7 = NAN;

				mavlink_msg_command_long_send_struct(_mavlink->get_channel(), &digicam_ctrl_cmd);

				return true;
			}
		}

		return false;
	}
};

class MavlinkStreamCameraImageCaptured : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamCameraImageCaptured::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "CAMERA_IMAGE_CAPTURED";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	bool const_rate() override
	{
		return true;
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamCameraImageCaptured(mavlink);
	}

	unsigned get_size() override
	{
		return _capture_sub.advertised() ? MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _capture_sub{ORB_ID(camera_capture)};

	/* do not allow top copying this class */
	MavlinkStreamCameraImageCaptured(MavlinkStreamCameraImageCaptured &) = delete;
	MavlinkStreamCameraImageCaptured &operator = (const MavlinkStreamCameraImageCaptured &) = delete;

protected:
	explicit MavlinkStreamCameraImageCaptured(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		camera_capture_s capture;

		if (_capture_sub.update(&capture)) {

			mavlink_camera_image_captured_t msg{};

			msg.time_boot_ms = capture.timestamp / 1000;
			msg.time_utc = capture.timestamp_utc;
			msg.camera_id = 1;	// FIXME : get this from uORB
			msg.lat = capture.lat * 1e7;
			msg.lon = capture.lon * 1e7;
			msg.alt = capture.alt * 1e3f;
			msg.relative_alt = capture.ground_distance * 1e3f;
			msg.q[0] = capture.q[0];
			msg.q[1] = capture.q[1];
			msg.q[2] = capture.q[2];
			msg.q[3] = capture.q[3];
			msg.image_index = capture.seq;
			msg.capture_result = capture.result;
			msg.file_url[0] = '\0';

			mavlink_msg_camera_image_captured_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamGlobalPositionInt : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamGlobalPositionInt::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "GLOBAL_POSITION_INT";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_GLOBAL_POSITION_INT;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamGlobalPositionInt(mavlink);
	}

	unsigned get_size() override
	{
		return _gpos_sub.advertised() ? MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _gpos_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _home_sub{ORB_ID(home_position)};
	uORB::Subscription _air_data_sub{ORB_ID(vehicle_air_data)};

	/* do not allow top copying this class */
	MavlinkStreamGlobalPositionInt(MavlinkStreamGlobalPositionInt &) = delete;
	MavlinkStreamGlobalPositionInt &operator = (const MavlinkStreamGlobalPositionInt &) = delete;

protected:
	explicit MavlinkStreamGlobalPositionInt(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		vehicle_global_position_s gpos;
		vehicle_local_position_s lpos;

		if (_gpos_sub.update(&gpos) && _lpos_sub.update(&lpos)) {

			mavlink_global_position_int_t msg{};

			if (lpos.z_valid && lpos.z_global) {
				msg.alt = (-lpos.z + lpos.ref_alt) * 1000.0f;

			} else {
				// fall back to baro altitude
				vehicle_air_data_s air_data{};
				_air_data_sub.copy(&air_data);

				if (air_data.timestamp > 0) {
					msg.alt = air_data.baro_alt_meter * 1000.0f;
				}
			}

			home_position_s home{};
			_home_sub.copy(&home);

			if ((home.timestamp > 0) && home.valid_alt) {
				if (lpos.z_valid) {
					msg.relative_alt = -(lpos.z - home.z) * 1000.0f;

				} else {
					msg.relative_alt = msg.alt - (home.alt * 1000.0f);
				}

			} else {
				if (lpos.z_valid) {
					msg.relative_alt = -lpos.z * 1000.0f;
				}
			}

			msg.time_boot_ms = gpos.timestamp / 1000;
			msg.lat = gpos.lat * 1e7;
			msg.lon = gpos.lon * 1e7;

			msg.vx = lpos.vx * 100.0f;
			msg.vy = lpos.vy * 100.0f;
			msg.vz = lpos.vz * 100.0f;

			msg.hdg = math::degrees(wrap_2pi(lpos.heading)) * 100.0f;

			mavlink_msg_global_position_int_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamOdometry : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamOdometry::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "ODOMETRY";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ODOMETRY;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamOdometry(mavlink);
	}

	unsigned get_size() override
	{
		if (_mavlink->odometry_loopback_enabled()) {
			return _vodom_sub.advertised() ? MAVLINK_MSG_ID_ODOMETRY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;

		} else {
			return _odom_sub.advertised() ? MAVLINK_MSG_ID_ODOMETRY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
		}
	}

private:
	uORB::Subscription _odom_sub{ORB_ID(vehicle_odometry)};
	uORB::Subscription _vodom_sub{ORB_ID(vehicle_visual_odometry)};

	/* do not allow top copying this class */
	MavlinkStreamOdometry(MavlinkStreamOdometry &) = delete;
	MavlinkStreamOdometry &operator = (const MavlinkStreamOdometry &) = delete;

protected:
	explicit MavlinkStreamOdometry(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		vehicle_odometry_s odom;
		// check if it is to send visual odometry loopback or not
		bool odom_updated = false;

		mavlink_odometry_t msg{};

		if (_mavlink->odometry_loopback_enabled()) {
			odom_updated = _vodom_sub.update(&odom);

			// set the frame_id according to the local frame of the data
			if (odom.local_frame == vehicle_odometry_s::LOCAL_FRAME_NED) {
				msg.frame_id = MAV_FRAME_LOCAL_NED;

			} else {
				msg.frame_id = MAV_FRAME_LOCAL_FRD;
			}

			// source: external vision system
			msg.estimator_type = MAV_ESTIMATOR_TYPE_VISION;

		} else {
			odom_updated = _odom_sub.update(&odom);

			msg.frame_id = MAV_FRAME_LOCAL_NED;

			// source: PX4 estimator
			msg.estimator_type = MAV_ESTIMATOR_TYPE_AUTOPILOT;
		}

		if (odom_updated) {
			msg.time_usec = odom.timestamp_sample;
			msg.child_frame_id = MAV_FRAME_BODY_FRD;

			// Current position
			msg.x = odom.x;
			msg.y = odom.y;
			msg.z = odom.z;

			// Current orientation
			msg.q[0] = odom.q[0];
			msg.q[1] = odom.q[1];
			msg.q[2] = odom.q[2];
			msg.q[3] = odom.q[3];

			switch (odom.velocity_frame) {
			case vehicle_odometry_s::BODY_FRAME_FRD:
				msg.vx = odom.vx;
				msg.vy = odom.vy;
				msg.vz = odom.vz;
				break;

			case vehicle_odometry_s::LOCAL_FRAME_FRD:
			case vehicle_odometry_s::LOCAL_FRAME_NED:
				// Body frame to local frame
				const matrix::Dcmf R_body_to_local(matrix::Quatf(odom.q));

				// Rotate linear velocity from local to body frame
				const matrix::Vector3f linvel_body(R_body_to_local.transpose() *
								   matrix::Vector3f(odom.vx, odom.vy, odom.vz));

				msg.vx = linvel_body(0);
				msg.vy = linvel_body(1);
				msg.vz = linvel_body(2);
				break;
			}

			// Current body rates
			msg.rollspeed = odom.rollspeed;
			msg.pitchspeed = odom.pitchspeed;
			msg.yawspeed = odom.yawspeed;

			// get the covariance matrix size

			// pose_covariance
			static constexpr size_t POS_URT_SIZE = sizeof(odom.pose_covariance) / sizeof(odom.pose_covariance[0]);
			static_assert(POS_URT_SIZE == (sizeof(msg.pose_covariance) / sizeof(msg.pose_covariance[0])),
				      "Odometry Pose Covariance matrix URT array size mismatch");

			// velocity_covariance
			static constexpr size_t VEL_URT_SIZE = sizeof(odom.velocity_covariance) / sizeof(odom.velocity_covariance[0]);
			static_assert(VEL_URT_SIZE == (sizeof(msg.velocity_covariance) / sizeof(msg.velocity_covariance[0])),
				      "Odometry Velocity Covariance matrix URT array size mismatch");

			// copy pose covariances
			for (size_t i = 0; i < POS_URT_SIZE; i++) {
				msg.pose_covariance[i] = odom.pose_covariance[i];
			}

			// copy velocity covariances
			//TODO: Apply rotation matrix to transform from body-fixed NED to earth-fixed NED frame
			for (size_t i = 0; i < VEL_URT_SIZE; i++) {
				msg.velocity_covariance[i] = odom.velocity_covariance[i];
			}

			mavlink_msg_odometry_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;

	}
};

class MavlinkStreamLocalPositionNED : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamLocalPositionNED::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "LOCAL_POSITION_NED";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_LOCAL_POSITION_NED;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamLocalPositionNED(mavlink);
	}

	unsigned get_size() override
	{
		return _lpos_sub.advertised() ? MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position)};

	/* do not allow top copying this class */
	MavlinkStreamLocalPositionNED(MavlinkStreamLocalPositionNED &) = delete;
	MavlinkStreamLocalPositionNED &operator = (const MavlinkStreamLocalPositionNED &) = delete;

protected:
	explicit MavlinkStreamLocalPositionNED(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		vehicle_local_position_s lpos;

		if (_lpos_sub.update(&lpos)) {
			mavlink_local_position_ned_t msg{};

			msg.time_boot_ms = lpos.timestamp / 1000;
			msg.x = lpos.x;
			msg.y = lpos.y;
			msg.z = lpos.z;
			msg.vx = lpos.vx;
			msg.vy = lpos.vy;
			msg.vz = lpos.vz;

			mavlink_msg_local_position_ned_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamEstimatorStatus : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamEstimatorStatus::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "ESTIMATOR_STATUS";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ESTIMATOR_STATUS;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamEstimatorStatus(mavlink);
	}

	unsigned get_size() override
	{
		return _est_sub.advertised() ? MAVLINK_MSG_ID_ESTIMATOR_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _est_sub{ORB_ID(estimator_status)};

	/* do not allow top copying this class */
	MavlinkStreamEstimatorStatus(MavlinkStreamEstimatorStatus &) = delete;
	MavlinkStreamEstimatorStatus &operator = (const MavlinkStreamEstimatorStatus &) = delete;

protected:
	explicit MavlinkStreamEstimatorStatus(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		estimator_status_s est;

		if (_est_sub.update(&est)) {
			mavlink_estimator_status_t est_msg{};
			est_msg.time_usec = est.timestamp;
			est_msg.vel_ratio = est.vel_test_ratio;
			est_msg.pos_horiz_ratio = est.pos_test_ratio;
			est_msg.pos_vert_ratio = est.hgt_test_ratio;
			est_msg.mag_ratio = est.mag_test_ratio;
			est_msg.hagl_ratio = est.hagl_test_ratio;
			est_msg.tas_ratio = est.tas_test_ratio;
			est_msg.pos_horiz_accuracy = est.pos_horiz_accuracy;
			est_msg.pos_vert_accuracy = est.pos_vert_accuracy;
			est_msg.flags = est.solution_status_flags;
			mavlink_msg_estimator_status_send_struct(_mavlink->get_channel(), &est_msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamVibration : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamVibration::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "VIBRATION";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_VIBRATION;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamVibration(mavlink);
	}

	unsigned get_size() override
	{
		const unsigned size = MAVLINK_MSG_ID_VIBRATION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;

		if (_sensor_selection_sub.advertised()) {
			return size;
		}

		for (auto &x : _vehicle_imu_status_sub) {
			if (x.advertised()) {
				return size;
			}
		}

		return 0;
	}

private:
	uORB::Subscription _sensor_selection_sub{ORB_ID(sensor_selection)};

	uORB::Subscription _vehicle_imu_status_sub[ORB_MULTI_MAX_INSTANCES] {
		{ORB_ID(vehicle_imu_status), 0},
		{ORB_ID(vehicle_imu_status), 1},
		{ORB_ID(vehicle_imu_status), 2},
		{ORB_ID(vehicle_imu_status), 3},
	};

	/* do not allow top copying this class */
	MavlinkStreamVibration(MavlinkStreamVibration &) = delete;
	MavlinkStreamVibration &operator = (const MavlinkStreamVibration &) = delete;

protected:
	explicit MavlinkStreamVibration(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		bool updated = _sensor_selection_sub.updated();

		// check for vehicle_imu_status update
		if (!updated) {
			for (auto &sub : _vehicle_imu_status_sub) {
				if (sub.updated()) {
					updated = true;
					break;
				}
			}
		}

		if (updated) {

			mavlink_vibration_t msg{};
			msg.time_usec = hrt_absolute_time();

			// VIBRATION usage not to mavlink spec, this is our current usage.
			//  vibration_x : Primary gyro delta angle coning metric = filtered length of (delta_angle x prev_delta_angle)
			//  vibration_y : Primary gyro high frequency vibe = filtered length of (delta_angle - prev_delta_angle)
			//  vibration_z : Primary accel high frequency vibe = filtered length of (delta_velocity - prev_delta_velocity)

			sensor_selection_s sensor_selection{};
			_sensor_selection_sub.copy(&sensor_selection);

			// primary accel high frequency vibration metric
			if (sensor_selection.accel_device_id != 0) {
				for (auto &x : _vehicle_imu_status_sub) {
					vehicle_imu_status_s status;

					if (x.copy(&status)) {
						if (status.accel_device_id == sensor_selection.accel_device_id) {
							msg.vibration_x = status.gyro_coning_vibration;
							msg.vibration_y = status.gyro_vibration_metric;
							msg.vibration_z = status.accel_vibration_metric;
							break;
						}
					}
				}
			}

			// accel 0, 1, 2 cumulative clipping
			for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
				vehicle_imu_status_s status;

				if (_vehicle_imu_status_sub[i].copy(&status)) {

					const uint32_t clipping = status.accel_clipping[0] + status.accel_clipping[1] + status.accel_clipping[2];

					switch (i) {
					case 0:
						msg.clipping_0 = clipping;
						break;

					case 1:
						msg.clipping_1 = clipping;
						break;

					case 2:
						msg.clipping_2 = clipping;
						break;
					}
				}
			}

			mavlink_msg_vibration_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamAttPosMocap : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamAttPosMocap::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "ATT_POS_MOCAP";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ATT_POS_MOCAP;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamAttPosMocap(mavlink);
	}

	unsigned get_size() override
	{
		return _mocap_sub.advertised() ? MAVLINK_MSG_ID_ATT_POS_MOCAP_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _mocap_sub{ORB_ID(vehicle_mocap_odometry)};

	/* do not allow top copying this class */
	MavlinkStreamAttPosMocap(MavlinkStreamAttPosMocap &) = delete;
	MavlinkStreamAttPosMocap &operator = (const MavlinkStreamAttPosMocap &) = delete;

protected:
	explicit MavlinkStreamAttPosMocap(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		vehicle_odometry_s mocap;

		if (_mocap_sub.update(&mocap)) {
			mavlink_att_pos_mocap_t msg{};

			msg.time_usec = mocap.timestamp_sample;
			msg.q[0] = mocap.q[0];
			msg.q[1] = mocap.q[1];
			msg.q[2] = mocap.q[2];
			msg.q[3] = mocap.q[3];
			msg.x = mocap.x;
			msg.y = mocap.y;
			msg.z = mocap.z;

			mavlink_msg_att_pos_mocap_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};


class MavlinkStreamHomePosition : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamHomePosition::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "HOME_POSITION";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_HOME_POSITION;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamHomePosition(mavlink);
	}

	unsigned get_size() override
	{
		return _home_sub.advertised() ? (MAVLINK_MSG_ID_HOME_POSITION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	uORB::Subscription _home_sub{ORB_ID(home_position)};

	/* do not allow top copying this class */
	MavlinkStreamHomePosition(MavlinkStreamHomePosition &) = delete;
	MavlinkStreamHomePosition &operator = (const MavlinkStreamHomePosition &) = delete;

protected:
	explicit MavlinkStreamHomePosition(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		/* we're sending the GPS home periodically to ensure the
		 * the GCS does pick it up at one point */
		home_position_s home;

		if (_home_sub.advertised() && _home_sub.copy(&home)) {
			if (home.valid_hpos) {
				mavlink_home_position_t msg{};

				msg.latitude = home.lat * 1e7;
				msg.longitude = home.lon * 1e7;
				msg.altitude = home.alt * 1e3f;

				msg.x = home.x;
				msg.y = home.y;
				msg.z = home.z;

				matrix::Quatf q(matrix::Eulerf(0.0f, 0.0f, home.yaw));
				msg.q[0] = q(0);
				msg.q[1] = q(1);
				msg.q[2] = q(2);
				msg.q[3] = q(3);

				msg.approach_x = 0.0f;
				msg.approach_y = 0.0f;
				msg.approach_z = 0.0f;

				msg.time_usec = home.timestamp;

				mavlink_msg_home_position_send_struct(_mavlink->get_channel(), &msg);

				return true;
			}
		}

		return false;
	}
};


template <int N>
class MavlinkStreamServoOutputRaw : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamServoOutputRaw<N>::get_name_static();
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SERVO_OUTPUT_RAW;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static constexpr const char *get_name_static()
	{
		switch (N) {
		case 0:
			return "SERVO_OUTPUT_RAW_0";

		case 1:
			return "SERVO_OUTPUT_RAW_1";
		}
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamServoOutputRaw<N>(mavlink);
	}

	unsigned get_size() override
	{
		return _act_sub.advertised() ? MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _act_sub{ORB_ID(actuator_outputs), N};

	/* do not allow top copying this class */
	MavlinkStreamServoOutputRaw(MavlinkStreamServoOutputRaw &) = delete;
	MavlinkStreamServoOutputRaw &operator = (const MavlinkStreamServoOutputRaw &) = delete;

protected:
	explicit MavlinkStreamServoOutputRaw(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		actuator_outputs_s act;

		if (_act_sub.update(&act)) {
			mavlink_servo_output_raw_t msg{};

			static_assert(sizeof(act.output) / sizeof(act.output[0]) >= 16, "mavlink message requires at least 16 outputs");

			msg.time_usec = act.timestamp;
			msg.port = N;
			msg.servo1_raw = act.output[0];
			msg.servo2_raw = act.output[1];
			msg.servo3_raw = act.output[2];
			msg.servo4_raw = act.output[3];
			msg.servo5_raw = act.output[4];
			msg.servo6_raw = act.output[5];
			msg.servo7_raw = act.output[6];
			msg.servo8_raw = act.output[7];
			msg.servo9_raw = act.output[8];
			msg.servo10_raw = act.output[9];
			msg.servo11_raw = act.output[10];
			msg.servo12_raw = act.output[11];
			msg.servo13_raw = act.output[12];
			msg.servo14_raw = act.output[13];
			msg.servo15_raw = act.output[14];
			msg.servo16_raw = act.output[15];

			mavlink_msg_servo_output_raw_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

template <int N>
class MavlinkStreamActuatorControlTarget : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamActuatorControlTarget<N>::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		switch (N) {
		case 0:
			return "ACTUATOR_CONTROL_TARGET0";

		case 1:
			return "ACTUATOR_CONTROL_TARGET1";

		case 2:
			return "ACTUATOR_CONTROL_TARGET2";

		case 3:
			return "ACTUATOR_CONTROL_TARGET3";
		}
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamActuatorControlTarget<N>(mavlink);
	}

	unsigned get_size() override
	{
		return (_act_ctrl_sub
			&& _act_ctrl_sub->advertised()) ? (MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	uORB::Subscription *_act_ctrl_sub{nullptr};

	/* do not allow top copying this class */
	MavlinkStreamActuatorControlTarget(MavlinkStreamActuatorControlTarget &) = delete;
	MavlinkStreamActuatorControlTarget &operator = (const MavlinkStreamActuatorControlTarget &) = delete;

protected:
	explicit MavlinkStreamActuatorControlTarget(Mavlink *mavlink) : MavlinkStream(mavlink)
	{
		// XXX this can be removed once the multiplatform system remaps topics
		switch (N) {
		case 0:
			_act_ctrl_sub = new uORB::Subscription{ORB_ID(actuator_controls_0)};
			break;

		case 1:
			_act_ctrl_sub = new uORB::Subscription{ORB_ID(actuator_controls_1)};
			break;

		case 2:
			_act_ctrl_sub = new uORB::Subscription{ORB_ID(actuator_controls_2)};
			break;

		case 3:
			_act_ctrl_sub = new uORB::Subscription{ORB_ID(actuator_controls_3)};
			break;
		}
	}

	~MavlinkStreamActuatorControlTarget() override
	{
		delete _act_ctrl_sub;
	}

	bool send(const hrt_abstime t) override
	{
		actuator_controls_s act_ctrl;

		if (_act_ctrl_sub && _act_ctrl_sub->update(&act_ctrl)) {
			mavlink_actuator_control_target_t msg{};

			msg.time_usec = act_ctrl.timestamp;
			msg.group_mlx = N;

			for (unsigned i = 0; i < sizeof(msg.controls) / sizeof(msg.controls[0]); i++) {
				msg.controls[i] = act_ctrl.control[i];
			}

			mavlink_msg_actuator_control_target_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamHILActuatorControls : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamHILActuatorControls::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "HIL_ACTUATOR_CONTROLS";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamHILActuatorControls(mavlink);
	}

	unsigned get_size() override
	{
		return _act_sub.advertised() ? MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _act_sub{ORB_ID(actuator_outputs)};

	/* do not allow top copying this class */
	MavlinkStreamHILActuatorControls(MavlinkStreamHILActuatorControls &) = delete;
	MavlinkStreamHILActuatorControls &operator = (const MavlinkStreamHILActuatorControls &) = delete;

protected:
	explicit MavlinkStreamHILActuatorControls(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		actuator_outputs_s act;

		if (_act_sub.update(&act)) {
			vehicle_status_s status{};
			_status_sub.copy(&status);

			if ((status.timestamp > 0) && (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED)) {
				/* translate the current system state to mavlink state and mode */
				uint8_t mavlink_state;
				uint8_t mavlink_base_mode;
				uint32_t mavlink_custom_mode;
				mavlink_hil_actuator_controls_t msg = {};

				get_mavlink_mode_state(&status, &mavlink_state, &mavlink_base_mode, &mavlink_custom_mode);

				const float pwm_center = (PWM_DEFAULT_MAX + PWM_DEFAULT_MIN) / 2;

				unsigned system_type = _mavlink->get_system_type();

				/* scale outputs depending on system type */
				if (system_type == MAV_TYPE_QUADROTOR ||
				    system_type == MAV_TYPE_HEXAROTOR ||
				    system_type == MAV_TYPE_OCTOROTOR ||
				    system_type == MAV_TYPE_VTOL_DUOROTOR ||
				    system_type == MAV_TYPE_VTOL_QUADROTOR ||
				    system_type == MAV_TYPE_VTOL_RESERVED2) {

					/* multirotors: set number of rotor outputs depending on type */

					unsigned n;

					switch (system_type) {
					case MAV_TYPE_QUADROTOR:
						n = 4;
						break;

					case MAV_TYPE_HEXAROTOR:
						n = 6;
						break;

					case MAV_TYPE_VTOL_DUOROTOR:
						n = 2;
						break;

					case MAV_TYPE_VTOL_QUADROTOR:
						n = 4;
						break;

					case MAV_TYPE_VTOL_RESERVED2:
						n = 8;
						break;

					default:
						n = 8;
						break;
					}

					for (unsigned i = 0; i < 16; i++) {
						if (act.output[i] > PWM_DEFAULT_MIN / 2) {
							if (i < n) {
								/* scale PWM out 900..2100 us to 0..1 for rotors */
								msg.controls[i] = (act.output[i] - PWM_DEFAULT_MIN) / (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN);

							} else {
								/* scale PWM out 900..2100 us to -1..1 for other channels */
								msg.controls[i] = (act.output[i] - pwm_center) / ((PWM_DEFAULT_MAX - PWM_DEFAULT_MIN) / 2);
							}

						} else {
							/* send 0 when disarmed and for disabled channels */
							msg.controls[i] = 0.0f;
						}
					}

				} else {
					/* fixed wing: scale throttle to 0..1 and other channels to -1..1 */

					for (unsigned i = 0; i < 16; i++) {
						if (act.output[i] > PWM_DEFAULT_MIN / 2) {
							if (i != 3) {
								/* scale PWM out 900..2100 us to -1..1 for normal channels */
								msg.controls[i] = (act.output[i] - pwm_center) / ((PWM_DEFAULT_MAX - PWM_DEFAULT_MIN) / 2);

							} else {
								/* scale PWM out 900..2100 us to 0..1 for throttle */
								msg.controls[i] = (act.output[i] - PWM_DEFAULT_MIN) / (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN);
							}

						} else {
							/* set 0 for disabled channels */
							msg.controls[i] = 0.0f;
						}
					}
				}

				msg.time_usec = hrt_absolute_time();
				msg.mode = mavlink_base_mode;
				msg.flags = 0;

				mavlink_msg_hil_actuator_controls_send_struct(_mavlink->get_channel(), &msg);

				return true;
			}
		}

		return false;
	}
};

class MavlinkStreamPositionTargetGlobalInt : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamPositionTargetGlobalInt::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "POSITION_TARGET_GLOBAL_INT";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamPositionTargetGlobalInt(mavlink);
	}

	unsigned get_size() override
	{
		return _pos_sp_triplet_sub.advertised() ? MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT_LEN +
		       MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _lpos_sp_sub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Subscription _pos_sp_triplet_sub{ORB_ID(position_setpoint_triplet)};

	/* do not allow top copying this class */
	MavlinkStreamPositionTargetGlobalInt(MavlinkStreamPositionTargetGlobalInt &) = delete;
	MavlinkStreamPositionTargetGlobalInt &operator = (const MavlinkStreamPositionTargetGlobalInt &) = delete;

protected:
	explicit MavlinkStreamPositionTargetGlobalInt(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		vehicle_control_mode_s control_mode{};
		_control_mode_sub.copy(&control_mode);

		if (control_mode.flag_control_position_enabled) {

			position_setpoint_triplet_s pos_sp_triplet{};
			_pos_sp_triplet_sub.copy(&pos_sp_triplet);

			if (pos_sp_triplet.timestamp > 0 && pos_sp_triplet.current.valid && PX4_ISFINITE(pos_sp_triplet.current.lat)
			    && PX4_ISFINITE(pos_sp_triplet.current.lon)) {

				mavlink_position_target_global_int_t msg{};

				msg.time_boot_ms = hrt_absolute_time() / 1000;
				msg.coordinate_frame = MAV_FRAME_GLOBAL_INT;
				msg.lat_int = pos_sp_triplet.current.lat * 1e7;
				msg.lon_int = pos_sp_triplet.current.lon * 1e7;
				msg.alt = pos_sp_triplet.current.alt;

				vehicle_local_position_setpoint_s lpos_sp;

				if (_lpos_sp_sub.copy(&lpos_sp)) {
					// velocity
					msg.vx = lpos_sp.vx;
					msg.vy = lpos_sp.vy;
					msg.vz = lpos_sp.vz;

					// acceleration
					msg.afx = lpos_sp.acceleration[0];
					msg.afy = lpos_sp.acceleration[1];
					msg.afz = lpos_sp.acceleration[2];

					// yaw
					msg.yaw = lpos_sp.yaw;
					msg.yaw_rate = lpos_sp.yawspeed;
				}

				mavlink_msg_position_target_global_int_send_struct(_mavlink->get_channel(), &msg);

				return true;
			}
		}

		return false;
	}
};


class MavlinkStreamLocalPositionSetpoint : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamLocalPositionSetpoint::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "POSITION_TARGET_LOCAL_NED";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamLocalPositionSetpoint(mavlink);
	}

	unsigned get_size() override
	{
		return _pos_sp_sub.advertised() ? MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _pos_sp_sub{ORB_ID(vehicle_local_position_setpoint)};

	/* do not allow top copying this class */
	MavlinkStreamLocalPositionSetpoint(MavlinkStreamLocalPositionSetpoint &) = delete;
	MavlinkStreamLocalPositionSetpoint &operator = (const MavlinkStreamLocalPositionSetpoint &) = delete;

protected:
	explicit MavlinkStreamLocalPositionSetpoint(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		vehicle_local_position_setpoint_s pos_sp;

		if (_pos_sp_sub.update(&pos_sp)) {
			mavlink_position_target_local_ned_t msg{};

			msg.time_boot_ms = pos_sp.timestamp / 1000;
			msg.coordinate_frame = MAV_FRAME_LOCAL_NED;
			msg.x = pos_sp.x;
			msg.y = pos_sp.y;
			msg.z = pos_sp.z;
			msg.yaw = pos_sp.yaw;
			msg.yaw_rate = pos_sp.yawspeed;
			msg.vx = pos_sp.vx;
			msg.vy = pos_sp.vy;
			msg.vz = pos_sp.vz;
			msg.afx = pos_sp.acceleration[0];
			msg.afy = pos_sp.acceleration[1];
			msg.afz = pos_sp.acceleration[2];

			mavlink_msg_position_target_local_ned_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};


class MavlinkStreamAttitudeTarget : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamAttitudeTarget::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "ATTITUDE_TARGET";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ATTITUDE_TARGET;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamAttitudeTarget(mavlink);
	}

	unsigned get_size() override
	{
		return _att_sp_sub.advertised() ? MAVLINK_MSG_ID_ATTITUDE_TARGET_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _att_sp_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _att_rates_sp_sub{ORB_ID(vehicle_rates_setpoint)};

	/* do not allow top copying this class */
	MavlinkStreamAttitudeTarget(MavlinkStreamAttitudeTarget &) = delete;
	MavlinkStreamAttitudeTarget &operator = (const MavlinkStreamAttitudeTarget &) = delete;

protected:
	explicit MavlinkStreamAttitudeTarget(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		vehicle_attitude_setpoint_s att_sp;

		if (_att_sp_sub.update(&att_sp)) {

			mavlink_attitude_target_t msg{};

			msg.time_boot_ms = att_sp.timestamp / 1000;
			matrix::Quatf(att_sp.q_d).copyTo(msg.q);

			vehicle_rates_setpoint_s att_rates_sp{};
			_att_rates_sp_sub.copy(&att_rates_sp);

			msg.body_roll_rate = att_rates_sp.roll;
			msg.body_pitch_rate = att_rates_sp.pitch;
			msg.body_yaw_rate = att_rates_sp.yaw;

			msg.thrust = att_sp.thrust_body[0];

			mavlink_msg_attitude_target_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};


class MavlinkStreamRCChannels : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamRCChannels::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "RC_CHANNELS";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_RC_CHANNELS;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamRCChannels(mavlink);
	}

	unsigned get_size() override
	{
		return _rc_sub.advertised() ? (MAVLINK_MSG_ID_RC_CHANNELS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	uORB::Subscription _rc_sub{ORB_ID(input_rc)};

	/* do not allow top copying this class */
	MavlinkStreamRCChannels(MavlinkStreamRCChannels &) = delete;
	MavlinkStreamRCChannels &operator = (const MavlinkStreamRCChannels &) = delete;

protected:
	explicit MavlinkStreamRCChannels(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		input_rc_s rc;

		if (_rc_sub.update(&rc)) {

			/* send RC channel data and RSSI */
			mavlink_rc_channels_t msg{};

			msg.time_boot_ms = rc.timestamp / 1000;
			msg.chancount = rc.channel_count;
			msg.chan1_raw = (rc.channel_count > 0) ? rc.values[0] : UINT16_MAX;
			msg.chan2_raw = (rc.channel_count > 1) ? rc.values[1] : UINT16_MAX;
			msg.chan3_raw = (rc.channel_count > 2) ? rc.values[2] : UINT16_MAX;
			msg.chan4_raw = (rc.channel_count > 3) ? rc.values[3] : UINT16_MAX;
			msg.chan5_raw = (rc.channel_count > 4) ? rc.values[4] : UINT16_MAX;
			msg.chan6_raw = (rc.channel_count > 5) ? rc.values[5] : UINT16_MAX;
			msg.chan7_raw = (rc.channel_count > 6) ? rc.values[6] : UINT16_MAX;
			msg.chan8_raw = (rc.channel_count > 7) ? rc.values[7] : UINT16_MAX;
			msg.chan9_raw = (rc.channel_count > 8) ? rc.values[8] : UINT16_MAX;
			msg.chan10_raw = (rc.channel_count > 9) ? rc.values[9] : UINT16_MAX;
			msg.chan11_raw = (rc.channel_count > 10) ? rc.values[10] : UINT16_MAX;
			msg.chan12_raw = (rc.channel_count > 11) ? rc.values[11] : UINT16_MAX;
			msg.chan13_raw = (rc.channel_count > 12) ? rc.values[12] : UINT16_MAX;
			msg.chan14_raw = (rc.channel_count > 13) ? rc.values[13] : UINT16_MAX;
			msg.chan15_raw = (rc.channel_count > 14) ? rc.values[14] : UINT16_MAX;
			msg.chan16_raw = (rc.channel_count > 15) ? rc.values[15] : UINT16_MAX;
			msg.chan17_raw = (rc.channel_count > 16) ? rc.values[16] : UINT16_MAX;
			msg.chan18_raw = (rc.channel_count > 17) ? rc.values[17] : UINT16_MAX;

			msg.rssi = (rc.channel_count > 0) ? rc.rssi : 0;

			mavlink_msg_rc_channels_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};


class MavlinkStreamManualControl : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamManualControl::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "MANUAL_CONTROL";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_MANUAL_CONTROL;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamManualControl(mavlink);
	}

	unsigned get_size() override
	{
		return _manual_control_setpoint_sub.advertised() ?
		       (MAVLINK_MSG_ID_MANUAL_CONTROL_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};

	/* do not allow top copying this class */
	MavlinkStreamManualControl(MavlinkStreamManualControl &) = delete;
	MavlinkStreamManualControl &operator = (const MavlinkStreamManualControl &) = delete;

protected:
	explicit MavlinkStreamManualControl(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		manual_control_setpoint_s manual_control_setpoint;

		if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
			mavlink_manual_control_t msg{};

			msg.target = mavlink_system.sysid;
			msg.x = manual_control_setpoint.x * 1000;
			msg.y = manual_control_setpoint.y * 1000;
			msg.z = manual_control_setpoint.z * 1000;
			msg.r = manual_control_setpoint.r * 1000;
			unsigned shift = 2;
			msg.buttons = 0;
			msg.buttons |= (manual_control_setpoint.mode_switch << (shift * 0));
			msg.buttons |= (manual_control_setpoint.return_switch << (shift * 1));
			msg.buttons |= (manual_control_setpoint.posctl_switch << (shift * 2));
			msg.buttons |= (manual_control_setpoint.loiter_switch << (shift * 3));
			msg.buttons |= (manual_control_setpoint.acro_switch << (shift * 4));
			msg.buttons |= (manual_control_setpoint.offboard_switch << (shift * 5));

			mavlink_msg_manual_control_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamTrajectoryRepresentationWaypoints: public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamTrajectoryRepresentationWaypoints::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "TRAJECTORY_REPRESENTATION_WAYPOINTS";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamTrajectoryRepresentationWaypoints(mavlink);
	}

	unsigned get_size() override
	{
		if (_traj_wp_avoidance_sub.advertised()) {
			return MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		}

		return 0;
	}

private:
	uORB::Subscription _traj_wp_avoidance_sub{ORB_ID(vehicle_trajectory_waypoint_desired)};

	/* do not allow top copying this class */
	MavlinkStreamTrajectoryRepresentationWaypoints(MavlinkStreamTrajectoryRepresentationWaypoints &);
	MavlinkStreamTrajectoryRepresentationWaypoints &operator = (const MavlinkStreamTrajectoryRepresentationWaypoints &);

protected:
	explicit MavlinkStreamTrajectoryRepresentationWaypoints(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		vehicle_trajectory_waypoint_s traj_wp_avoidance_desired;

		if (_traj_wp_avoidance_sub.update(&traj_wp_avoidance_desired)) {
			mavlink_trajectory_representation_waypoints_t msg{};

			msg.time_usec = traj_wp_avoidance_desired.timestamp;
			int number_valid_points = 0;

			for (int i = 0; i < vehicle_trajectory_waypoint_s::NUMBER_POINTS; ++i) {
				msg.pos_x[i] = traj_wp_avoidance_desired.waypoints[i].position[0];
				msg.pos_y[i] = traj_wp_avoidance_desired.waypoints[i].position[1];
				msg.pos_z[i] = traj_wp_avoidance_desired.waypoints[i].position[2];

				msg.vel_x[i] = traj_wp_avoidance_desired.waypoints[i].velocity[0];
				msg.vel_y[i] = traj_wp_avoidance_desired.waypoints[i].velocity[1];
				msg.vel_z[i] = traj_wp_avoidance_desired.waypoints[i].velocity[2];

				msg.acc_x[i] = traj_wp_avoidance_desired.waypoints[i].acceleration[0];
				msg.acc_y[i] = traj_wp_avoidance_desired.waypoints[i].acceleration[1];
				msg.acc_z[i] = traj_wp_avoidance_desired.waypoints[i].acceleration[2];

				msg.pos_yaw[i] = traj_wp_avoidance_desired.waypoints[i].yaw;
				msg.vel_yaw[i] = traj_wp_avoidance_desired.waypoints[i].yaw_speed;

				switch (traj_wp_avoidance_desired.waypoints[i].type) {
				case position_setpoint_s::SETPOINT_TYPE_TAKEOFF:
					msg.command[i] = vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF;
					break;

				case position_setpoint_s::SETPOINT_TYPE_LOITER:
					msg.command[i] = vehicle_command_s::VEHICLE_CMD_NAV_LOITER_UNLIM;
					break;

				case position_setpoint_s::SETPOINT_TYPE_LAND:
					msg.command[i] = vehicle_command_s::VEHICLE_CMD_NAV_LAND;
					break;

				default:
					msg.command[i] = UINT16_MAX;
				}

				if (traj_wp_avoidance_desired.waypoints[i].point_valid) {
					number_valid_points++;
				}

			}

			msg.valid_points = number_valid_points;

			mavlink_msg_trajectory_representation_waypoints_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamOpticalFlowRad : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamOpticalFlowRad::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "OPTICAL_FLOW_RAD";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_OPTICAL_FLOW_RAD;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamOpticalFlowRad(mavlink);
	}

	unsigned get_size() override
	{
		return _flow_sub.advertised() ? (MAVLINK_MSG_ID_OPTICAL_FLOW_RAD_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	uORB::Subscription _flow_sub{ORB_ID(optical_flow)};

	/* do not allow top copying this class */
	MavlinkStreamOpticalFlowRad(MavlinkStreamOpticalFlowRad &) = delete;
	MavlinkStreamOpticalFlowRad &operator = (const MavlinkStreamOpticalFlowRad &) = delete;

protected:
	explicit MavlinkStreamOpticalFlowRad(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		optical_flow_s flow;

		if (_flow_sub.update(&flow)) {
			mavlink_optical_flow_rad_t msg{};

			msg.time_usec = flow.timestamp;
			msg.sensor_id = flow.sensor_id;
			msg.integrated_x = flow.pixel_flow_x_integral;
			msg.integrated_y = flow.pixel_flow_y_integral;
			msg.integrated_xgyro = flow.gyro_x_rate_integral;
			msg.integrated_ygyro = flow.gyro_y_rate_integral;
			msg.integrated_zgyro = flow.gyro_z_rate_integral;
			msg.distance = flow.ground_distance_m;
			msg.quality = flow.quality;
			msg.integration_time_us = flow.integration_timespan;
			msg.sensor_id = flow.sensor_id;
			msg.time_delta_distance_us = flow.time_since_last_sonar_update;
			msg.temperature = flow.gyro_temperature;

			mavlink_msg_optical_flow_rad_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamNamedValueFloat : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamNamedValueFloat::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "NAMED_VALUE_FLOAT";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_NAMED_VALUE_FLOAT;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamNamedValueFloat(mavlink);
	}

	unsigned get_size() override
	{
		return _debug_sub.advertised() ? MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _debug_sub{ORB_ID(debug_key_value)};

	/* do not allow top copying this class */
	MavlinkStreamNamedValueFloat(MavlinkStreamNamedValueFloat &) = delete;
	MavlinkStreamNamedValueFloat &operator = (const MavlinkStreamNamedValueFloat &) = delete;

protected:
	explicit MavlinkStreamNamedValueFloat(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		debug_key_value_s debug;

		if (_debug_sub.update(&debug)) {
			mavlink_named_value_float_t msg{};

			msg.time_boot_ms = debug.timestamp / 1000ULL;
			memcpy(msg.name, debug.key, sizeof(msg.name));
			/* enforce null termination */
			msg.name[sizeof(msg.name) - 1] = '\0';
			msg.value = debug.value;

			mavlink_msg_named_value_float_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamDebug : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamDebug::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "DEBUG";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_DEBUG;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamDebug(mavlink);
	}

	unsigned get_size() override
	{
		return _debug_sub.advertised() ? MAVLINK_MSG_ID_DEBUG_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _debug_sub{ORB_ID(debug_value)};

	/* do not allow top copying this class */
	MavlinkStreamDebug(MavlinkStreamDebug &) = delete;
	MavlinkStreamDebug &operator = (const MavlinkStreamDebug &) = delete;

protected:
	explicit MavlinkStreamDebug(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		debug_value_s debug;

		if (_debug_sub.update(&debug)) {
			mavlink_debug_t msg{};
			msg.time_boot_ms = debug.timestamp / 1000ULL;
			msg.ind = debug.ind;
			msg.value = debug.value;

			mavlink_msg_debug_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamDebugVect : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamDebugVect::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "DEBUG_VECT";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_DEBUG_VECT;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamDebugVect(mavlink);
	}

	unsigned get_size() override
	{
		return _debug_sub.advertised() ? MAVLINK_MSG_ID_DEBUG_VECT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _debug_sub{ORB_ID(debug_vect)};

	/* do not allow top copying this class */
	MavlinkStreamDebugVect(MavlinkStreamDebugVect &) = delete;
	MavlinkStreamDebugVect &operator = (const MavlinkStreamDebugVect &) = delete;

protected:
	explicit MavlinkStreamDebugVect(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		debug_vect_s debug;

		if (_debug_sub.update(&debug)) {
			mavlink_debug_vect_t msg{};

			msg.time_usec = debug.timestamp;
			memcpy(msg.name, debug.name, sizeof(msg.name));
			/* enforce null termination */
			msg.name[sizeof(msg.name) - 1] = '\0';
			msg.x = debug.x;
			msg.y = debug.y;
			msg.z = debug.z;

			mavlink_msg_debug_vect_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamDebugFloatArray : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamDebugFloatArray::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "DEBUG_FLOAT_ARRAY";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamDebugFloatArray(mavlink);
	}

	unsigned get_size() override
	{
		return _debug_array_sub.advertised() ? MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _debug_array_sub{ORB_ID(debug_array)};

	/* do not allow top copying this class */
	MavlinkStreamDebugFloatArray(MavlinkStreamDebugFloatArray &);
	MavlinkStreamDebugFloatArray &operator = (const MavlinkStreamDebugFloatArray &);

protected:
	explicit MavlinkStreamDebugFloatArray(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		debug_array_s debug;

		if (_debug_array_sub.update(&debug)) {
			mavlink_debug_float_array_t msg{};

			msg.time_usec = debug.timestamp;
			msg.array_id = debug.id;
			memcpy(msg.name, debug.name, sizeof(msg.name));
			/* enforce null termination */
			msg.name[sizeof(msg.name) - 1] = '\0';

			for (size_t i = 0; i < debug_array_s::ARRAY_SIZE; i++) {
				msg.data[i] = debug.data[i];
			}

			mavlink_msg_debug_float_array_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamNavControllerOutput : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamNavControllerOutput::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "NAV_CONTROLLER_OUTPUT";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamNavControllerOutput(mavlink);
	}

	unsigned get_size() override
	{
		return (_pos_ctrl_status_sub.advertised()) ?
		       MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _pos_ctrl_status_sub{ORB_ID(position_controller_status)};
	uORB::Subscription _tecs_status_sub{ORB_ID(tecs_status)};

	/* do not allow top copying this class */
	MavlinkStreamNavControllerOutput(MavlinkStreamNavControllerOutput &) = delete;
	MavlinkStreamNavControllerOutput &operator = (const MavlinkStreamNavControllerOutput &) = delete;

protected:
	explicit MavlinkStreamNavControllerOutput(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		if (_pos_ctrl_status_sub.updated()) {

			position_controller_status_s pos_ctrl_status{};
			_pos_ctrl_status_sub.copy(&pos_ctrl_status);

			tecs_status_s tecs_status{};
			_tecs_status_sub.copy(&tecs_status);

			mavlink_nav_controller_output_t msg{};

			msg.nav_roll = math::degrees(pos_ctrl_status.nav_roll);
			msg.nav_pitch = math::degrees(pos_ctrl_status.nav_pitch);
			msg.nav_bearing = (int16_t)math::degrees(pos_ctrl_status.nav_bearing);
			msg.target_bearing = (int16_t)math::degrees(pos_ctrl_status.target_bearing);
			msg.wp_dist = (uint16_t)pos_ctrl_status.wp_dist;
			msg.xtrack_error = pos_ctrl_status.xtrack_error;
			msg.alt_error = tecs_status.altitude_filtered - tecs_status.altitude_sp;
			msg.aspd_error = tecs_status.airspeed_filtered - tecs_status.airspeed_sp;

			mavlink_msg_nav_controller_output_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamCameraCapture : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamCameraCapture::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "CAMERA_CAPTURE";
	}

	static constexpr uint16_t get_id_static()
	{
		return 0;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamCameraCapture(mavlink);
	}

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_COMMAND_LONG_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};

	/* do not allow top copying this class */
	MavlinkStreamCameraCapture(MavlinkStreamCameraCapture &) = delete;
	MavlinkStreamCameraCapture &operator = (const MavlinkStreamCameraCapture &) = delete;

protected:
	explicit MavlinkStreamCameraCapture(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		vehicle_status_s status;

		if (_status_sub.update(&status)) {
			mavlink_command_long_t msg{};

			msg.target_system = 0;
			msg.target_component = MAV_COMP_ID_ALL;
			msg.command = MAV_CMD_DO_CONTROL_VIDEO;
			msg.confirmation = 0;
			msg.param1 = 0;
			msg.param2 = 0;
			msg.param3 = 0;
			/* set camera capture ON/OFF depending on arming state */
			msg.param4 = (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) ? 1 : 0;
			msg.param5 = 0;
			msg.param6 = 0;
			msg.param7 = 0;

			mavlink_msg_command_long_send_struct(_mavlink->get_channel(), &msg);
		}

		return true;
	}
};

class MavlinkStreamDistanceSensor : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamDistanceSensor::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "DISTANCE_SENSOR";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_DISTANCE_SENSOR;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamDistanceSensor(mavlink);
	}

	unsigned get_size() override
	{
		return _distance_sensor_sub.advertised() ? (MAVLINK_MSG_ID_DISTANCE_SENSOR_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	uORB::Subscription _distance_sensor_sub{ORB_ID(distance_sensor)};

	/* do not allow top copying this class */
	MavlinkStreamDistanceSensor(MavlinkStreamDistanceSensor &) = delete;
	MavlinkStreamDistanceSensor &operator = (const MavlinkStreamDistanceSensor &) = delete;

protected:
	explicit MavlinkStreamDistanceSensor(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		distance_sensor_s dist_sensor;

		if (_distance_sensor_sub.update(&dist_sensor)) {
			mavlink_distance_sensor_t msg{};

			msg.time_boot_ms = dist_sensor.timestamp / 1000; /* us to ms */

			switch (dist_sensor.type) {
			case MAV_DISTANCE_SENSOR_ULTRASOUND:
				msg.type = MAV_DISTANCE_SENSOR_ULTRASOUND;
				break;

			case MAV_DISTANCE_SENSOR_LASER:
				msg.type = MAV_DISTANCE_SENSOR_LASER;
				break;

			case MAV_DISTANCE_SENSOR_INFRARED:
				msg.type = MAV_DISTANCE_SENSOR_INFRARED;
				break;

			default:
				msg.type = MAV_DISTANCE_SENSOR_LASER;
				break;
			}

			msg.current_distance = dist_sensor.current_distance * 1e2f; // m to cm
			msg.id               = dist_sensor.id;
			msg.max_distance     = dist_sensor.max_distance * 1e2f;     // m to cm
			msg.min_distance     = dist_sensor.min_distance * 1e2f;     // m to cm
			msg.orientation      = dist_sensor.orientation;
			msg.covariance       = dist_sensor.variance * 1e4f;         // m^2 to cm^2

			mavlink_msg_distance_sensor_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamExtendedSysState : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamExtendedSysState::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "EXTENDED_SYS_STATE";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_EXTENDED_SYS_STATE;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamExtendedSysState(mavlink);
	}

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_EXTENDED_SYS_STATE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _landed_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _pos_sp_triplet_sub{ORB_ID(position_setpoint_triplet)};
	uORB::Subscription _control_mode_sub{ORB_ID(vehicle_control_mode)};
	mavlink_extended_sys_state_t _msg;

	/* do not allow top copying this class */
	MavlinkStreamExtendedSysState(MavlinkStreamExtendedSysState &) = delete;
	MavlinkStreamExtendedSysState &operator = (const MavlinkStreamExtendedSysState &) = delete;

protected:
	explicit MavlinkStreamExtendedSysState(Mavlink *mavlink) : MavlinkStream(mavlink),
		_msg()
	{
		_msg.vtol_state = MAV_VTOL_STATE_UNDEFINED;
		_msg.landed_state = MAV_LANDED_STATE_ON_GROUND;
	}

	bool send(const hrt_abstime t) override
	{
		bool updated = false;

		vehicle_status_s status;

		if (_status_sub.copy(&status)) {
			updated = true;

			if (status.is_vtol) {
				if (!status.in_transition_mode && status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
					_msg.vtol_state = MAV_VTOL_STATE_MC;

				} else if (!status.in_transition_mode) {
					_msg.vtol_state = MAV_VTOL_STATE_FW;

				} else if (status.in_transition_mode && status.in_transition_to_fw) {
					_msg.vtol_state = MAV_VTOL_STATE_TRANSITION_TO_FW;

				} else if (status.in_transition_mode) {
					_msg.vtol_state = MAV_VTOL_STATE_TRANSITION_TO_MC;
				}
			}
		}

		vehicle_land_detected_s land_detected;

		if (_landed_sub.copy(&land_detected)) {
			updated = true;

			if (land_detected.landed) {
				_msg.landed_state = MAV_LANDED_STATE_ON_GROUND;

			} else if (!land_detected.landed) {
				_msg.landed_state = MAV_LANDED_STATE_IN_AIR;

				vehicle_control_mode_s control_mode;
				position_setpoint_triplet_s pos_sp_triplet;

				if (_control_mode_sub.copy(&control_mode) && _pos_sp_triplet_sub.copy(&pos_sp_triplet)) {
					if (control_mode.flag_control_auto_enabled && pos_sp_triplet.current.valid) {
						if (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF) {
							_msg.landed_state = MAV_LANDED_STATE_TAKEOFF;

						} else if (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
							_msg.landed_state = MAV_LANDED_STATE_LANDING;
						}
					}
				}
			}
		}

		if (updated) {
			mavlink_msg_extended_sys_state_send_struct(_mavlink->get_channel(), &_msg);
		}

		return updated;
	}
};

class MavlinkStreamAltitude : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamAltitude::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "ALTITUDE";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ALTITUDE;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamAltitude(mavlink);
	}

	unsigned get_size() override
	{
		return _local_pos_sub.advertised() ? MAVLINK_MSG_ID_ALTITUDE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _home_sub{ORB_ID(home_position)};
	uORB::Subscription _air_data_sub{ORB_ID(vehicle_air_data)};

	/* do not allow top copying this class */
	MavlinkStreamAltitude(MavlinkStreamAltitude &) = delete;
	MavlinkStreamAltitude &operator = (const MavlinkStreamAltitude &) = delete;

protected:
	explicit MavlinkStreamAltitude(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		mavlink_altitude_t msg{};

		msg.altitude_monotonic = NAN;
		msg.altitude_amsl = NAN;
		msg.altitude_local = NAN;
		msg.altitude_relative = NAN;
		msg.altitude_terrain = NAN;
		msg.bottom_clearance = NAN;

		// always update monotonic altitude
		bool air_data_updated = false;
		vehicle_air_data_s air_data{};
		_air_data_sub.copy(&air_data);

		if (air_data.timestamp > 0) {
			msg.altitude_monotonic = air_data.baro_alt_meter;

			air_data_updated = true;
		}

		bool lpos_updated = false;

		vehicle_local_position_s local_pos;

		if (_local_pos_sub.copy(&local_pos)) {

			if (local_pos.z_valid) {
				if (local_pos.z_global) {
					msg.altitude_amsl = -local_pos.z + local_pos.ref_alt;

				} else {
					msg.altitude_amsl = msg.altitude_monotonic;
				}

				msg.altitude_local = -local_pos.z;

				home_position_s home{};
				_home_sub.copy(&home);

				if (home.valid_alt) {
					msg.altitude_relative = -(local_pos.z - home.z);

				} else {
					msg.altitude_relative = -local_pos.z;
				}

				if (local_pos.dist_bottom_valid) {
					msg.altitude_terrain = -local_pos.z - local_pos.dist_bottom;
					msg.bottom_clearance = local_pos.dist_bottom;
				}
			}

			lpos_updated = true;
		}

		// local position timeout after 10 ms
		// avoid publishing only baro altitude_monotonic if possible
		bool lpos_timeout = (hrt_elapsed_time(&local_pos.timestamp) > 10_ms);

		if (lpos_updated || (air_data_updated && lpos_timeout)) {
			msg.time_usec = hrt_absolute_time();
			mavlink_msg_altitude_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamWind : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamWind::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "WIND_COV";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_WIND_COV;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamWind(mavlink);
	}

	unsigned get_size() override
	{
		return _wind_estimate_sub.advertised() ? MAVLINK_MSG_ID_WIND_COV_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _wind_estimate_sub{ORB_ID(wind_estimate)};
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};

	/* do not allow top copying this class */
	MavlinkStreamWind(MavlinkStreamWind &) = delete;
	MavlinkStreamWind &operator = (const MavlinkStreamWind &) = delete;

protected:
	explicit MavlinkStreamWind(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		wind_estimate_s wind_estimate;

		if (_wind_estimate_sub.update(&wind_estimate)) {
			mavlink_wind_cov_t msg{};

			msg.time_usec = wind_estimate.timestamp;

			msg.wind_x = wind_estimate.windspeed_north;
			msg.wind_y = wind_estimate.windspeed_east;
			msg.wind_z = 0.0f;

			msg.var_horiz = wind_estimate.variance_north + wind_estimate.variance_east;
			msg.var_vert = 0.0f;

			vehicle_local_position_s lpos{};
			_local_pos_sub.copy(&lpos);
			msg.wind_alt = (lpos.z_valid && lpos.z_global) ? (-lpos.z + lpos.ref_alt) : NAN;

			msg.horiz_accuracy = 0.0f;
			msg.vert_accuracy = 0.0f;

			mavlink_msg_wind_cov_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamMountOrientation : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamMountOrientation::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "MOUNT_ORIENTATION";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_MOUNT_ORIENTATION;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamMountOrientation(mavlink);
	}

	unsigned get_size() override
	{
		return _mount_orientation_sub.advertised() ? MAVLINK_MSG_ID_MOUNT_ORIENTATION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _mount_orientation_sub{ORB_ID(mount_orientation)};
	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position)};

	/* do not allow top copying this class */
	MavlinkStreamMountOrientation(MavlinkStreamMountOrientation &) = delete;
	MavlinkStreamMountOrientation &operator = (const MavlinkStreamMountOrientation &) = delete;

protected:
	explicit MavlinkStreamMountOrientation(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		mount_orientation_s mount_orientation;

		if (_mount_orientation_sub.update(&mount_orientation)) {
			mavlink_mount_orientation_t msg{};

			msg.roll = math::degrees(mount_orientation.attitude_euler_angle[0]);
			msg.pitch = math::degrees(mount_orientation.attitude_euler_angle[1]);
			msg.yaw = math::degrees(mount_orientation.attitude_euler_angle[2]);

			vehicle_local_position_s lpos{};
			_lpos_sub.copy(&lpos);
			msg.yaw_absolute = math::degrees(matrix::wrap_2pi(lpos.heading + mount_orientation.attitude_euler_angle[2]));

			mavlink_msg_mount_orientation_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamGroundTruth : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamGroundTruth::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "GROUND_TRUTH";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_HIL_STATE_QUATERNION;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamGroundTruth(mavlink);
	}

	unsigned get_size() override
	{
		return (_att_sub.advertised()
			|| _gpos_sub.advertised()) ? MAVLINK_MSG_ID_HIL_STATE_QUATERNION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _angular_velocity_sub{ORB_ID(vehicle_angular_velocity_groundtruth)};
	uORB::Subscription _att_sub{ORB_ID(vehicle_attitude_groundtruth)};
	uORB::Subscription _gpos_sub{ORB_ID(vehicle_global_position_groundtruth)};
	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position_groundtruth)};

	/* do not allow top copying this class */
	MavlinkStreamGroundTruth(MavlinkStreamGroundTruth &) = delete;
	MavlinkStreamGroundTruth &operator = (const MavlinkStreamGroundTruth &) = delete;

protected:
	explicit MavlinkStreamGroundTruth(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		if (_angular_velocity_sub.updated() || _att_sub.updated() || _gpos_sub.updated() || _lpos_sub.updated()) {
			vehicle_attitude_s att{};
			_att_sub.copy(&att);

			vehicle_global_position_s gpos{};
			_gpos_sub.copy(&gpos);

			vehicle_local_position_s lpos{};
			_lpos_sub.copy(&lpos);

			vehicle_angular_velocity_s angular_velocity{};
			_angular_velocity_sub.copy(&angular_velocity);

			mavlink_hil_state_quaternion_t msg{};

			// vehicle_attitude -> hil_state_quaternion
			msg.attitude_quaternion[0] = att.q[0];
			msg.attitude_quaternion[1] = att.q[1];
			msg.attitude_quaternion[2] = att.q[2];
			msg.attitude_quaternion[3] = att.q[3];
			msg.rollspeed = angular_velocity.xyz[0];
			msg.pitchspeed = angular_velocity.xyz[1];
			msg.yawspeed = angular_velocity.xyz[2];

			// vehicle_global_position -> hil_state_quaternion
			// same units as defined in mavlink/common.xml
			msg.lat = gpos.lat * 1e7;
			msg.lon = gpos.lon * 1e7;
			msg.alt = gpos.alt * 1e3f;
			msg.vx = lpos.vx * 1e2f;
			msg.vy = lpos.vy * 1e2f;
			msg.vz = lpos.vz * 1e2f;
			msg.ind_airspeed = 0;
			msg.true_airspeed = 0;
			msg.xacc = lpos.ax;
			msg.yacc = lpos.ay;
			msg.zacc = lpos.az;

			mavlink_msg_hil_state_quaternion_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamPing : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamPing::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "PING";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_PING;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamPing(mavlink);
	}

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_PING_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

	bool const_rate() override
	{
		return true;
	}

private:
	uint32_t _sequence;

	/* do not allow top copying this class */
	MavlinkStreamPing(MavlinkStreamPing &) = delete;
	MavlinkStreamPing &operator = (const MavlinkStreamPing &) = delete;

protected:
	explicit MavlinkStreamPing(Mavlink *mavlink) : MavlinkStream(mavlink),
		_sequence(0)
	{}

	bool send(const hrt_abstime t) override
	{
		mavlink_ping_t msg = {};

		msg.time_usec = hrt_absolute_time();
		msg.seq = _sequence++;
		msg.target_system = 0; // All systems
		msg.target_component = 0; // All components

		mavlink_msg_ping_send_struct(_mavlink->get_channel(), &msg);

		return true;
	}
};

class MavlinkStreamOrbitStatus : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamOrbitStatus::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "ORBIT_EXECUTION_STATUS";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamOrbitStatus(mavlink);
	}

	unsigned get_size() override
	{
		return _orb_status_sub.advertised() ? MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _orb_status_sub{ORB_ID(orbit_status)};

	/* do not allow top copying this class */
	MavlinkStreamOrbitStatus(MavlinkStreamOrbitStatus &);
	MavlinkStreamOrbitStatus &operator = (const MavlinkStreamOrbitStatus &);

protected:
	explicit MavlinkStreamOrbitStatus(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		orbit_status_s _orbit_status;

		if (_orb_status_sub.update(&_orbit_status)) {
			mavlink_orbit_execution_status_t _msg_orbit_execution_status{};

			_msg_orbit_execution_status.time_usec = _orbit_status.timestamp;
			_msg_orbit_execution_status.radius = _orbit_status.radius;
			_msg_orbit_execution_status.frame = _orbit_status.frame;
			_msg_orbit_execution_status.x = _orbit_status.x * 1e7;
			_msg_orbit_execution_status.y = _orbit_status.y * 1e7;
			_msg_orbit_execution_status.z = _orbit_status.z;

			mavlink_msg_orbit_execution_status_send_struct(_mavlink->get_channel(), &_msg_orbit_execution_status);
		}

		return true;
	}
};

class MavlinkStreamObstacleDistance : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamObstacleDistance::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "OBSTACLE_DISTANCE";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_OBSTACLE_DISTANCE;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamObstacleDistance(mavlink);
	}

	unsigned get_size() override
	{
		return _obstacle_distance_fused_sub.advertised() ? (MAVLINK_MSG_ID_OBSTACLE_DISTANCE_LEN +
				MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	uORB::Subscription _obstacle_distance_fused_sub{ORB_ID(obstacle_distance_fused)};

	/* do not allow top copying this class */
	MavlinkStreamObstacleDistance(MavlinkStreamObstacleDistance &) = delete;
	MavlinkStreamObstacleDistance &operator = (const MavlinkStreamObstacleDistance &) = delete;

protected:
	explicit MavlinkStreamObstacleDistance(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		obstacle_distance_s obstacle_distance;

		if (_obstacle_distance_fused_sub.update(&obstacle_distance)) {
			mavlink_obstacle_distance_t msg{};

			msg.time_usec = obstacle_distance.timestamp;
			msg.sensor_type = obstacle_distance.sensor_type;
			memcpy(msg.distances, obstacle_distance.distances, sizeof(msg.distances));
			msg.increment = 0;
			msg.min_distance = obstacle_distance.min_distance;
			msg.max_distance = obstacle_distance.max_distance;
			msg.angle_offset = obstacle_distance.angle_offset;
			msg.increment_f = obstacle_distance.increment;
			msg.frame = obstacle_distance.frame;

			mavlink_msg_obstacle_distance_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};



class MavlinkStreamRawRpm : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamRawRpm::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "RAW_RPM";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_RAW_RPM;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamRawRpm(mavlink);
	}

	unsigned get_size() override
	{
		return _rpm_sub.advertised() ? MAVLINK_MSG_ID_RAW_RPM + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _rpm_sub{ORB_ID(rpm)};

	/* do not allow top copying this class */
	MavlinkStreamRawRpm(MavlinkStreamRawRpm &) = delete;
	MavlinkStreamRawRpm &operator = (const MavlinkStreamRawRpm &) = delete;

protected:
	explicit MavlinkStreamRawRpm(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		rpm_s rpm;

		if (_rpm_sub.update(&rpm)) {
			mavlink_raw_rpm_t msg{};

			msg.frequency = rpm.indicated_frequency_rpm;

			mavlink_msg_raw_rpm_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

static const StreamListItem streams_list[] = {
	create_stream_list_item<MavlinkStreamHeartbeat>(),
	create_stream_list_item<MavlinkStreamStatustext>(),
	create_stream_list_item<MavlinkStreamCommandLong>(),
	create_stream_list_item<MavlinkStreamSysStatus>(),
	create_stream_list_item<MavlinkStreamBatteryStatus>(),
	create_stream_list_item<MavlinkStreamSmartBatteryInfo>(),
	create_stream_list_item<MavlinkStreamHighresIMU>(),
	create_stream_list_item<MavlinkStreamScaledIMU>(),
	create_stream_list_item<MavlinkStreamScaledIMU2>(),
	create_stream_list_item<MavlinkStreamScaledIMU3>(),
	create_stream_list_item<MavlinkStreamScaledPressure<0> >(),
	// create_stream_list_item<MavlinkStreamScaledPressure<1> >(),
	// create_stream_list_item<MavlinkStreamScaledPressure<2> >(),
	create_stream_list_item<MavlinkStreamAttitude>(),
	create_stream_list_item<MavlinkStreamAttitudeQuaternion>(),
	create_stream_list_item<MavlinkStreamVFRHUD>(),
	create_stream_list_item<MavlinkStreamGPSRawInt>(),
	create_stream_list_item<MavlinkStreamGPS2Raw>(),
	create_stream_list_item<MavlinkStreamSystemTime>(),
	create_stream_list_item<MavlinkStreamTimesync>(),
	create_stream_list_item<MavlinkStreamGlobalPositionInt>(),
	create_stream_list_item<MavlinkStreamLocalPositionNED>(),
	create_stream_list_item<MavlinkStreamOdometry>(),
	create_stream_list_item<MavlinkStreamEstimatorStatus>(),
	create_stream_list_item<MavlinkStreamVibration>(),
	create_stream_list_item<MavlinkStreamAttPosMocap>(),
	create_stream_list_item<MavlinkStreamHomePosition>(),
	create_stream_list_item<MavlinkStreamServoOutputRaw<0> >(),
	create_stream_list_item<MavlinkStreamServoOutputRaw<1> >(),
	create_stream_list_item<MavlinkStreamHILActuatorControls>(),
	create_stream_list_item<MavlinkStreamPositionTargetGlobalInt>(),
	create_stream_list_item<MavlinkStreamLocalPositionSetpoint>(),
	create_stream_list_item<MavlinkStreamAttitudeTarget>(),
	create_stream_list_item<MavlinkStreamRCChannels>(),
	create_stream_list_item<MavlinkStreamManualControl>(),
	create_stream_list_item<MavlinkStreamTrajectoryRepresentationWaypoints>(),
	create_stream_list_item<MavlinkStreamOpticalFlowRad>(),
	create_stream_list_item<MavlinkStreamActuatorControlTarget<0> >(),
	create_stream_list_item<MavlinkStreamActuatorControlTarget<1> >(),
	create_stream_list_item<MavlinkStreamNamedValueFloat>(),
	create_stream_list_item<MavlinkStreamDebug>(),
	create_stream_list_item<MavlinkStreamDebugVect>(),
	create_stream_list_item<MavlinkStreamDebugFloatArray>(),
	create_stream_list_item<MavlinkStreamNavControllerOutput>(),
	create_stream_list_item<MavlinkStreamCameraCapture>(),
	create_stream_list_item<MavlinkStreamCameraTrigger>(),
	create_stream_list_item<MavlinkStreamCameraImageCaptured>(),
	create_stream_list_item<MavlinkStreamDistanceSensor>(),
	create_stream_list_item<MavlinkStreamExtendedSysState>(),
	create_stream_list_item<MavlinkStreamAltitude>(),
	create_stream_list_item<MavlinkStreamADSBVehicle>(),
	create_stream_list_item<MavlinkStreamUTMGlobalPosition>(),
	create_stream_list_item<MavlinkStreamCollision>(),
	create_stream_list_item<MavlinkStreamWind>(),
	create_stream_list_item<MavlinkStreamMountOrientation>(),
	create_stream_list_item<MavlinkStreamHighLatency2>(),
	create_stream_list_item<MavlinkStreamGroundTruth>(),
	create_stream_list_item<MavlinkStreamPing>(),
	create_stream_list_item<MavlinkStreamOrbitStatus>(),
	create_stream_list_item<MavlinkStreamObstacleDistance>(),
	create_stream_list_item<MavlinkStreamAutopilotVersion>(),
	create_stream_list_item<MavlinkStreamProtocolVersion>(),
	create_stream_list_item<MavlinkStreamFlightInformation>(),
	create_stream_list_item<MavlinkStreamStorageInformation>(),
	create_stream_list_item<MavlinkStreamRawRpm>()
};

const char *get_stream_name(const uint16_t msg_id)
{
	// search for stream with specified msg id in supported streams list
	for (const auto &stream : streams_list) {
		if (msg_id == stream.get_id()) {
			return stream.get_name();
		}
	}

	return nullptr;
}

MavlinkStream *create_mavlink_stream(const char *stream_name, Mavlink *mavlink)
{
	// search for stream with specified name in supported streams list
	if (stream_name != nullptr) {
		for (const auto &stream : streams_list) {
			if (strcmp(stream_name, stream.get_name()) == 0) {
				return stream.new_instance(mavlink);
			}
		}
	}

	return nullptr;
}

MavlinkStream *create_mavlink_stream(const uint16_t msg_id, Mavlink *mavlink)
{
	// search for stream with specified name in supported streams list
	for (const auto &stream : streams_list) {
		if (msg_id == stream.get_id()) {
			return stream.new_instance(mavlink);
		}
	}

	return nullptr;
}
