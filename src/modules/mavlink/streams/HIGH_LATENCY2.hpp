/****************************************************************************
 *
 *   Copyright (c) 2018-2020 PX4 Development Team. All rights reserved.
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

#ifndef HIGH_LATENCY2_HPP
#define HIGH_LATENCY2_HPP

#include <lib/ecl/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/estimator_selector_status.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/geofence_result.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/position_controller_status.h>
#include <uORB/topics/tecs_status.h>
#include <uORB/topics/wind.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_status_flags.h>

class MavlinkStreamHighLatency2 : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamHighLatency2(mavlink); }

	static constexpr const char *get_name_static() { return "HIGH_LATENCY2"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_HIGH_LATENCY2; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_HIGH_LATENCY2_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

	bool const_rate() override { return true; }

private:
	explicit MavlinkStreamHighLatency2(Mavlink *mavlink) :
		MavlinkStream(mavlink),
		_airspeed(SimpleAnalyzer::AVERAGE),
		_airspeed_sp(SimpleAnalyzer::AVERAGE),
		_climb_rate(SimpleAnalyzer::MAX),
		_eph(SimpleAnalyzer::MAX),
		_epv(SimpleAnalyzer::MAX),
		_groundspeed(SimpleAnalyzer::AVERAGE),
		_temperature(SimpleAnalyzer::AVERAGE),
		_throttle(SimpleAnalyzer::AVERAGE),
		_windspeed(SimpleAnalyzer::AVERAGE)
	{
		reset_last_sent();
	}

	struct PerBatteryData {
		PerBatteryData(uint8_t instance) : subscription(ORB_ID(battery_status), instance) {}
		uORB::Subscription subscription;
		SimpleAnalyzer analyzer{SimpleAnalyzer::AVERAGE};
		uint64_t timestamp{0};
		bool connected{false};
	};

	bool send() override
	{
		const hrt_abstime t = hrt_absolute_time();

		// only send the struct if transmitting is allowed
		// this assures that the stream timer is only reset when actually a message is transmitted
		if (_mavlink->should_transmit()) {
			mavlink_high_latency2_t msg{};
			set_default_values(msg);

			bool updated = _airspeed.valid();
			updated |= _airspeed_sp.valid();

			for (int i = 0; i < battery_status_s::MAX_INSTANCES; i++) {
				updated |= _batteries[i].analyzer.valid();
			}

			updated |= _climb_rate.valid();
			updated |= _eph.valid();
			updated |= _epv.valid();
			updated |= _groundspeed.valid();
			updated |= _temperature.valid();
			updated |= _throttle.valid();
			updated |= _windspeed.valid();
			updated |= write_airspeed(&msg);
			updated |= write_attitude_sp(&msg);
			updated |= write_battery_status(&msg);
			updated |= write_estimator_status(&msg);
			updated |= write_fw_ctrl_status(&msg);
			updated |= write_geofence_result(&msg);
			updated |= write_global_position(&msg);
			updated |= write_mission_result(&msg);
			updated |= write_tecs_status(&msg);
			updated |= write_vehicle_status(&msg);
			updated |= write_vehicle_status_flags(&msg);
			updated |= write_wind(&msg);

			if (updated) {
				msg.timestamp = t / 1000;

				msg.type = _mavlink->get_system_type();
				msg.autopilot = MAV_AUTOPILOT_PX4;

				if (_airspeed.valid()) {
					_airspeed.get_scaled(msg.airspeed, 5.0f);
				}

				if (_airspeed_sp.valid()) {
					_airspeed_sp.get_scaled(msg.airspeed_sp, 5.0f);
				}

				int lowest = 0;

				for (int i = 1; i < battery_status_s::MAX_INSTANCES; i++) {
					const bool battery_connected = _batteries[i].connected && _batteries[i].analyzer.valid();
					const bool battery_is_lowest = _batteries[i].analyzer.get_scaled(100.0f) <= _batteries[lowest].analyzer.get_scaled(
									       100.0f);

					if (battery_connected && battery_is_lowest) {
						lowest = i;
					}

				}

				if (_batteries[lowest].connected) {
					_batteries[lowest].analyzer.get_scaled(msg.battery, 100.0f);

				} else {
					msg.battery = -1;
				}


				if (_climb_rate.valid()) {
					_climb_rate.get_scaled(msg.climb_rate, 10.0f);
				}

				if (_eph.valid()) {
					_eph.get_scaled(msg.eph, 10.0f);
				}

				if (_epv.valid()) {
					_epv.get_scaled(msg.epv, 10.0f);
				}

				if (_groundspeed.valid()) {
					_groundspeed.get_scaled(msg.groundspeed, 5.0f);
				}

				if (_temperature.valid()) {
					_temperature.get_scaled(msg.temperature_air, 1.0f);
				}

				if (_throttle.valid()) {
					_throttle.get_scaled(msg.throttle, 100.0f);
				}

				if (_windspeed.valid()) {
					_windspeed.get_scaled(msg.windspeed, 5.0f);
				}

				reset_analysers(t);

				mavlink_msg_high_latency2_send_struct(_mavlink->get_channel(), &msg);
			}

			return updated;

		} else {
			// reset the analysers every 60 seconds if nothing should be transmitted
			if ((t - _last_reset_time) > 60_s) {
				reset_analysers(t);
				PX4_DEBUG("Resetting HIGH_LATENCY2 analysers");
			}

			return false;
		}
	}

	void reset_analysers(const hrt_abstime t)
	{
		// reset the analyzers
		_airspeed.reset();
		_airspeed_sp.reset();

		for (int i = 0; i < battery_status_s::MAX_INSTANCES; i++) {
			_batteries[i].analyzer.reset();
		}

		_climb_rate.reset();
		_eph.reset();
		_epv.reset();
		_groundspeed.reset();
		_temperature.reset();
		_throttle.reset();
		_windspeed.reset();

		_last_reset_time = t;
	}

	bool write_airspeed(mavlink_high_latency2_t *msg)
	{
		airspeed_s airspeed;

		if (_airspeed_sub.update(&airspeed)) {
			if (airspeed.confidence < 0.95f) { // the same threshold as for the commander
				msg->failure_flags |= HL_FAILURE_FLAG_DIFFERENTIAL_PRESSURE;
			}

			return true;
		}

		return false;
	}

	bool write_attitude_sp(mavlink_high_latency2_t *msg)
	{
		vehicle_attitude_setpoint_s attitude_sp;

		if (_attitude_sp_sub.update(&attitude_sp)) {
			msg->target_heading = static_cast<uint8_t>(math::degrees(matrix::wrap_2pi(attitude_sp.yaw_body)) * 0.5f);
			return true;
		}

		return false;
	}

	bool write_battery_status(mavlink_high_latency2_t *msg)
	{
		bool updated = false;

		for (int i = 0; i < battery_status_s::MAX_INSTANCES; i++) {
			battery_status_s battery;

			if (_batteries[i].subscription.update(&battery)) {
				updated = true;
				_batteries[i].connected = battery.connected;

				if (battery.warning > battery_status_s::BATTERY_WARNING_LOW) {
					msg->failure_flags |= HL_FAILURE_FLAG_BATTERY;
				}
			}
		}

		return updated;
	}

	bool write_estimator_status(mavlink_high_latency2_t *msg)
	{
		// use primary estimator_status
		if (_estimator_selector_status_sub.updated()) {
			estimator_selector_status_s estimator_selector_status;

			if (_estimator_selector_status_sub.copy(&estimator_selector_status)) {
				if (estimator_selector_status.primary_instance != _estimator_status_sub.get_instance()) {
					_estimator_status_sub.ChangeInstance(estimator_selector_status.primary_instance);
				}
			}
		}

		estimator_status_s estimator_status;

		if (_estimator_status_sub.update(&estimator_status)) {
			if (estimator_status.gps_check_fail_flags > 0 ||
			    estimator_status.filter_fault_flags > 0 ||
			    estimator_status.innovation_check_flags > 0) {

				msg->failure_flags |= HL_FAILURE_FLAG_ESTIMATOR;
			}

			return true;
		}

		return false;
	}

	bool write_fw_ctrl_status(mavlink_high_latency2_t *msg)
	{
		position_controller_status_s pos_ctrl_status;

		if (_pos_ctrl_status_sub.update(&pos_ctrl_status)) {
			uint16_t target_distance;
			convert_limit_safe(pos_ctrl_status.wp_dist * 0.1f, target_distance);
			msg->target_distance = target_distance;
			return true;
		}

		return false;
	}

	bool write_geofence_result(mavlink_high_latency2_t *msg)
	{
		geofence_result_s geofence;

		if (_geofence_sub.update(&geofence)) {
			if (geofence.geofence_violated) {
				msg->failure_flags |= HL_FAILURE_FLAG_GEOFENCE;
			}

			return true;
		}

		return false;
	}

	bool write_global_position(mavlink_high_latency2_t *msg)
	{
		vehicle_global_position_s global_pos;
		vehicle_local_position_s local_pos;

		if (_global_pos_sub.update(&global_pos) && _local_pos_sub.update(&local_pos)) {
			msg->latitude = global_pos.lat * 1e7;
			msg->longitude = global_pos.lon * 1e7;

			int16_t altitude = 0;

			if (global_pos.alt > 0) {
				convert_limit_safe(global_pos.alt + 0.5f, altitude);

			} else {
				convert_limit_safe(global_pos.alt - 0.5f, altitude);
			}

			msg->altitude = altitude;

			msg->heading = static_cast<uint8_t>(math::degrees(matrix::wrap_2pi(local_pos.heading)) * 0.5f);

			return true;
		}

		return false;
	}

	bool write_mission_result(mavlink_high_latency2_t *msg)
	{
		mission_result_s mission_result;

		if (_mission_result_sub.update(&mission_result)) {
			msg->wp_num = mission_result.seq_current;
			return true;
		}

		return false;
	}

	bool write_tecs_status(mavlink_high_latency2_t *msg)
	{
		tecs_status_s tecs_status;

		if (_tecs_status_sub.update(&tecs_status)) {
			int16_t target_altitude;
			convert_limit_safe(tecs_status.altitude_sp, target_altitude);
			msg->target_altitude = target_altitude;

			return true;
		}

		return false;
	}

	bool write_vehicle_status(mavlink_high_latency2_t *msg)
	{
		vehicle_status_s status;

		if (_status_sub.update(&status)) {
			if ((status.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE)
			    && !(status.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE)) {
				msg->failure_flags |= HL_FAILURE_FLAG_ABSOLUTE_PRESSURE;
			}

			if (((status.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_3D_ACCEL)
			     && !(status.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_3D_ACCEL)) ||
			    ((status.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_3D_ACCEL2) &&
			     !(status.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_3D_ACCEL2))) {
				msg->failure_flags |= HL_FAILURE_FLAG_3D_ACCEL;
			}

			if (((status.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_3D_GYRO)
			     && !(status.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_3D_GYRO)) ||
			    ((status.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_3D_GYRO2) &&
			     !(status.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_3D_GYRO2))) {
				msg->failure_flags |= HL_FAILURE_FLAG_3D_GYRO;
			}

			if (((status.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_3D_MAG)
			     && !(status.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_3D_MAG)) ||
			    ((status.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_3D_MAG2) &&
			     !(status.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_3D_MAG2))) {
				msg->failure_flags |= HL_FAILURE_FLAG_3D_MAG;
			}

			if ((status.onboard_control_sensors_enabled & MAV_SYS_STATUS_TERRAIN)
			    && !(status.onboard_control_sensors_health & MAV_SYS_STATUS_TERRAIN)) {
				msg->failure_flags |= HL_FAILURE_FLAG_TERRAIN;
			}

			if (status.rc_signal_lost) {
				msg->failure_flags |= HL_FAILURE_FLAG_RC_RECEIVER;
			}

			if (status.engine_failure) {
				msg->failure_flags |= HL_FAILURE_FLAG_ENGINE;
			}

			if (status.mission_failure) {
				msg->failure_flags |= HL_FAILURE_FLAG_MISSION;
			}

			// flight mode
			union px4_custom_mode custom_mode {get_px4_custom_mode(status.nav_state)};
			msg->custom_mode = custom_mode.custom_mode_hl;

			return true;
		}

		return false;
	}

	bool write_vehicle_status_flags(mavlink_high_latency2_t *msg)
	{
		vehicle_status_flags_s status_flags;

		if (_status_flags_sub.update(&status_flags)) {
			if (!status_flags.condition_global_position_valid) { //TODO check if there is a better way to get only GPS failure
				msg->failure_flags |= HL_FAILURE_FLAG_GPS;
			}

			if (status_flags.offboard_control_signal_lost && status_flags.offboard_control_signal_found_once) {
				msg->failure_flags |= HL_FAILURE_FLAG_OFFBOARD_LINK;
			}

			return true;
		}

		return false;
	}

	bool write_wind(mavlink_high_latency2_t *msg)
	{
		wind_s wind;

		if (_wind_sub.update(&wind)) {
			msg->wind_heading = static_cast<uint8_t>(math::degrees(matrix::wrap_2pi(atan2f(wind.windspeed_east,
					    wind.windspeed_north))) * 0.5f);
			return true;
		}

		return false;
	}

	void update_data() override
	{
		const hrt_abstime t = hrt_absolute_time();

		if (t > _last_update_time) {
			// first order low pass filter for the update rate
			_update_rate_filtered = 0.97f * _update_rate_filtered + 0.03f / ((t - _last_update_time) * 1e-6f);
			_last_update_time = t;
		}

		update_airspeed();
		update_tecs_status();
		update_battery_status();
		update_local_position();
		update_gps();
		update_vehicle_status();
		update_wind();
	}

	void update_airspeed()
	{
		airspeed_s airspeed;

		if (_airspeed_sub.update(&airspeed)) {
			_airspeed.add_value(airspeed.indicated_airspeed_m_s, _update_rate_filtered);
			_temperature.add_value(airspeed.air_temperature_celsius, _update_rate_filtered);
		}
	}


	void update_tecs_status()
	{
		tecs_status_s tecs_status;

		if (_tecs_status_sub.update(&tecs_status)) {
			_airspeed_sp.add_value(tecs_status.true_airspeed_sp, _update_rate_filtered);
		}
	}

	void update_battery_status()
	{
		battery_status_s battery;

		for (int i = 0; i < battery_status_s::MAX_INSTANCES; i++) {
			if (_batteries[i].subscription.update(&battery)) {
				_batteries[i].connected = battery.connected;
				_batteries[i].analyzer.add_value(battery.remaining, _update_rate_filtered);
			}
		}
	}

	void update_local_position()
	{
		vehicle_local_position_s local_pos;

		if (_local_pos_sub.update(&local_pos)) {
			_climb_rate.add_value(fabsf(local_pos.vz), _update_rate_filtered);
			_groundspeed.add_value(sqrtf(local_pos.vx * local_pos.vx + local_pos.vy * local_pos.vy), _update_rate_filtered);
		}
	}

	void update_gps()
	{
		vehicle_gps_position_s gps;

		if (_gps_sub.update(&gps)) {
			_eph.add_value(gps.eph, _update_rate_filtered);
			_epv.add_value(gps.epv, _update_rate_filtered);
		}
	}

	void update_vehicle_status()
	{
		vehicle_status_s status;

		if (_status_sub.update(&status)) {
			if (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
				actuator_controls_s actuator{};

				if (status.is_vtol && status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
					if (_actuator_1_sub.copy(&actuator)) {
						_throttle.add_value(actuator.control[actuator_controls_s::INDEX_THROTTLE], _update_rate_filtered);
					}

				} else {
					if (_actuator_0_sub.copy(&actuator)) {
						_throttle.add_value(actuator.control[actuator_controls_s::INDEX_THROTTLE], _update_rate_filtered);
					}
				}

			} else {
				_throttle.add_value(0.0f, _update_rate_filtered);
			}
		}
	}

	void update_wind()
	{
		wind_s wind;

		if (_wind_sub.update(&wind)) {
			_windspeed.add_value(sqrtf(wind.windspeed_north * wind.windspeed_north + wind.windspeed_east * wind.windspeed_east),
					     _update_rate_filtered);
		}
	}

	void set_default_values(mavlink_high_latency2_t &msg) const
	{
		msg.airspeed = 0;
		msg.airspeed_sp = 0;
		msg.altitude = 0;
		msg.autopilot = MAV_AUTOPILOT_ENUM_END;
		msg.battery = -1;
		msg.climb_rate = 0;
		msg.custom0 = INT8_MIN;
		msg.custom1 = INT8_MIN;
		msg.custom2 = INT8_MIN;
		msg.eph = UINT8_MAX;
		msg.epv = UINT8_MAX;
		msg.failure_flags = 0;
		msg.custom_mode = 0;
		msg.groundspeed = 0;
		msg.heading = 0;
		msg.latitude = 0;
		msg.longitude = 0;
		msg.target_altitude = 0;
		msg.target_distance = 0;
		msg.target_heading = 0;
		msg.temperature_air = INT8_MIN;
		msg.throttle = 0;
		msg.timestamp = 0;
		msg.type = MAV_TYPE_ENUM_END;
		msg.wind_heading = 0;
		msg.windspeed = 0;
		msg.wp_num = UINT16_MAX;
	}

	uORB::Subscription _actuator_0_sub{ORB_ID(actuator_controls_0)};
	uORB::Subscription _actuator_1_sub{ORB_ID(actuator_controls_1)};
	uORB::Subscription _airspeed_sub{ORB_ID(airspeed)};
	uORB::Subscription _attitude_sp_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _estimator_selector_status_sub{ORB_ID(estimator_selector_status)};
	uORB::Subscription _estimator_status_sub{ORB_ID(estimator_status)};
	uORB::Subscription _pos_ctrl_status_sub{ORB_ID(position_controller_status)};
	uORB::Subscription _geofence_sub{ORB_ID(geofence_result)};
	uORB::Subscription _global_pos_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _gps_sub{ORB_ID(vehicle_gps_position)};
	uORB::Subscription _mission_result_sub{ORB_ID(mission_result)};
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _status_flags_sub{ORB_ID(vehicle_status_flags)};
	uORB::Subscription _tecs_status_sub{ORB_ID(tecs_status)};
	uORB::Subscription _wind_sub{ORB_ID(wind)};

	SimpleAnalyzer _airspeed;
	SimpleAnalyzer _airspeed_sp;
	SimpleAnalyzer _climb_rate;
	SimpleAnalyzer _eph;
	SimpleAnalyzer _epv;
	SimpleAnalyzer _groundspeed;
	SimpleAnalyzer _temperature;
	SimpleAnalyzer _throttle;
	SimpleAnalyzer _windspeed;

	hrt_abstime _last_reset_time{0};
	hrt_abstime _last_update_time{0};
	float _update_rate_filtered{0};

	PerBatteryData _batteries[battery_status_s::MAX_INSTANCES] {0, 1, 2, 3};
};

#endif // HIGH_LATENCY2_HPP
