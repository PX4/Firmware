/****************************************************************************
 *
 *   Copyright (c) 2015 Mark Charlebois. All rights reserved.
 *   Copyright (c) 2016 Anton Matosov. All rights reserved.
 *   Copyright (c) 2017-2020 PX4 Development Team. All rights reserved.
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

#include "simulator.h"
#include <simulator_config.h>

#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/tasks.h>
#include <lib/ecl/geo/geo.h>
#include <drivers/drv_pwm_output.h>
#include <conversion/rotation.h>
#include <mathlib/mathlib.h>

#include <errno.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <pthread.h>
#include <sys/socket.h>
#include <termios.h>

#include <limits>

#ifdef ENABLE_UART_RC_INPUT
#ifndef B460800
#define B460800 460800
#endif

#ifndef B921600
#define B921600 921600
#endif

static int openUart(const char *uart_name, int baud);
#endif

static int _fd;
static unsigned char _buf[2048];
static sockaddr_in _srcaddr;
static unsigned _addrlen = sizeof(_srcaddr);

const unsigned mode_flag_armed = 128;
const unsigned mode_flag_custom = 1;

using namespace time_literals;

mavlink_hil_actuator_controls_t Simulator::actuator_controls_from_outputs()
{
	mavlink_hil_actuator_controls_t msg{};

	msg.time_usec = hrt_absolute_time() + hrt_absolute_time_offset();

	bool armed = (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

	const float pwm_center = (PWM_DEFAULT_MAX + PWM_DEFAULT_MIN) / 2;

	int _system_type = _param_mav_type.get();

	/* scale outputs depending on system type */
	if (_system_type == MAV_TYPE_QUADROTOR ||
	    _system_type == MAV_TYPE_HEXAROTOR ||
	    _system_type == MAV_TYPE_OCTOROTOR ||
	    _system_type == MAV_TYPE_VTOL_DUOROTOR ||
	    _system_type == MAV_TYPE_VTOL_QUADROTOR ||
	    _system_type == MAV_TYPE_VTOL_TILTROTOR ||
	    _system_type == MAV_TYPE_VTOL_RESERVED2) {

		/* multirotors: set number of rotor outputs depending on type */

		unsigned n;

		switch (_system_type) {
		case MAV_TYPE_VTOL_DUOROTOR:
			n = 2;
			break;

		case MAV_TYPE_QUADROTOR:
		case MAV_TYPE_VTOL_QUADROTOR:
		case MAV_TYPE_VTOL_TILTROTOR:
			n = 4;
			break;

		case MAV_TYPE_VTOL_RESERVED2:
			// this is the standard VTOL / quad plane with 5 propellers
			n = 5;
			break;

		case MAV_TYPE_HEXAROTOR:
			n = 6;
			break;

		default:
			n = 8;
			break;
		}

		for (unsigned i = 0; i < 16; i++) {
			if (armed) {
				if (i < n) {
					/* scale PWM out PWM_DEFAULT_MIN..PWM_DEFAULT_MAX us to 0..1 for rotors */
					msg.controls[i] = (_actuator_outputs.output[i] - PWM_DEFAULT_MIN) / (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN);

				} else {
					/* scale PWM out PWM_DEFAULT_MIN..PWM_DEFAULT_MAX us to -1..1 for other channels */
					msg.controls[i] = (_actuator_outputs.output[i] - pwm_center) / ((PWM_DEFAULT_MAX - PWM_DEFAULT_MIN) / 2);
				}

			} else {
				/* send 0 when disarmed and for disabled channels */
				msg.controls[i] = 0.0f;
			}
		}

	} else if (_system_type == MAV_TYPE_AIRSHIP) {
		/* airship: scale starboard and port throttle to 0..1 and other channels (tilt, tail thruster) to -1..1 */
		for (unsigned i = 0; i < 16; i++) {
			if (armed) {
				if (i < 2) {
					/* scale PWM out PWM_DEFAULT_MIN..PWM_DEFAULT_MAX us to 0..1 for rotors */
					msg.controls[i] = (_actuator_outputs.output[i] - PWM_DEFAULT_MIN) / (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN);

				} else {
					/* scale PWM out PWM_DEFAULT_MIN..PWM_DEFAULT_MAX us to -1..1 for other channels */
					msg.controls[i] = (_actuator_outputs.output[i] - pwm_center) / ((PWM_DEFAULT_MAX - PWM_DEFAULT_MIN) / 2);
				}

			} else {
				/* set 0 for disabled channels */
				msg.controls[i] = 0.0f;
			}
		}

	} else {
		/* fixed wing: scale throttle to 0..1 and other channels to -1..1 */

		for (unsigned i = 0; i < 16; i++) {
			if (armed) {
				if (i != 4) {
					/* scale PWM out PWM_DEFAULT_MIN..PWM_DEFAULT_MAX us to -1..1 for normal channels */
					msg.controls[i] = (_actuator_outputs.output[i] - pwm_center) / ((PWM_DEFAULT_MAX - PWM_DEFAULT_MIN) / 2);

				} else {
					/* scale PWM out PWM_DEFAULT_MIN..PWM_DEFAULT_MAX us to 0..1 for throttle */
					msg.controls[i] = (_actuator_outputs.output[i] - PWM_DEFAULT_MIN) / (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN);
				}

			} else {
				/* set 0 for disabled channels */
				msg.controls[i] = 0.0f;
			}
		}
	}

	msg.mode = mode_flag_custom;
	msg.mode |= (armed) ? mode_flag_armed : 0;
	msg.flags = 0;

#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	msg.flags |= 1;
#endif

	return msg;
}

void Simulator::send_controls()
{
	orb_copy(ORB_ID(actuator_outputs), _actuator_outputs_sub, &_actuator_outputs);

	if (_actuator_outputs.timestamp > 0) {
		mavlink_hil_actuator_controls_t hil_act_control = actuator_controls_from_outputs();

		mavlink_message_t message{};
		mavlink_msg_hil_actuator_controls_encode(_param_mav_sys_id.get(), _param_mav_comp_id.get(), &message, &hil_act_control);

		PX4_DEBUG("sending controls t=%ld (%ld)", _actuator_outputs.timestamp, hil_act_control.time_usec);

		send_mavlink_message(message);
	}
}

void Simulator::update_sensors(const hrt_abstime &time, const mavlink_hil_sensor_t &sensors)
{
	float temperature = NAN;

	// temperature only updated with baro
	if ((sensors.fields_updated & SensorSource::BARO) == SensorSource::BARO) {
		if (PX4_ISFINITE(sensors.temperature)) {
			temperature = sensors.temperature;

			_px4_accel.set_temperature(temperature);
			_px4_baro.set_temperature(temperature);
			_px4_gyro.set_temperature(temperature);
			_px4_mag.set_temperature(temperature);
		}
	}

	// accel
	if ((sensors.fields_updated & SensorSource::ACCEL) == SensorSource::ACCEL && !_param_sim_accel_block.get()) {
		_px4_accel.update(time, sensors.xacc, sensors.yacc, sensors.zacc);
	}

	// gyro
	if ((sensors.fields_updated & SensorSource::GYRO) == SensorSource::GYRO && !_param_sim_gyro_block.get()) {
		_px4_gyro.update(time, sensors.xgyro, sensors.ygyro, sensors.zgyro);
	}

	// magnetometer
	if ((sensors.fields_updated & SensorSource::MAG) == SensorSource::MAG && !_param_sim_mag_block.get()) {
		_px4_mag.update(time, sensors.xmag, sensors.ymag, sensors.zmag);
	}

	// baro
	if ((sensors.fields_updated & SensorSource::BARO) == SensorSource::BARO && !_param_sim_baro_block.get()) {
		_px4_baro.update(time, sensors.abs_pressure);
	}

	// differential pressure
	if ((sensors.fields_updated & SensorSource::DIFF_PRESS) == SensorSource::DIFF_PRESS && !_param_sim_dpres_block.get()) {
		differential_pressure_s report{};
		report.timestamp = time;
		report.temperature = temperature;
		report.differential_pressure_filtered_pa = sensors.diff_pressure * 100.0f; // convert from millibar to bar;
		report.differential_pressure_raw_pa = sensors.diff_pressure * 100.0f; // convert from millibar to bar;

		_differential_pressure_pub.publish(report);
	}
}

void Simulator::handle_message(const mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_HIL_SENSOR:
		handle_message_hil_sensor(msg);
		break;

	case MAVLINK_MSG_ID_HIL_OPTICAL_FLOW:
		handle_message_optical_flow(msg);
		break;

	case MAVLINK_MSG_ID_ODOMETRY:
		handle_message_odometry(msg);
		break;

	case MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE:
		handle_message_vision_position_estimate(msg);
		break;

	case MAVLINK_MSG_ID_DISTANCE_SENSOR:
		handle_message_distance_sensor(msg);
		break;

	case MAVLINK_MSG_ID_HIL_GPS:
		handle_message_hil_gps(msg);
		break;

	case MAVLINK_MSG_ID_RC_CHANNELS:
		handle_message_rc_channels(msg);
		break;

	case MAVLINK_MSG_ID_LANDING_TARGET:
		handle_message_landing_target(msg);
		break;

	case MAVLINK_MSG_ID_HIL_STATE_QUATERNION:
		handle_message_hil_state_quaternion(msg);
		break;
	}
}

void Simulator::handle_message_distance_sensor(const mavlink_message_t *msg)
{
	mavlink_distance_sensor_t dist;
	mavlink_msg_distance_sensor_decode(msg, &dist);
	publish_distance_topic(&dist);
}

void Simulator::handle_message_hil_gps(const mavlink_message_t *msg)
{
	mavlink_hil_gps_t hil_gps;
	mavlink_msg_hil_gps_decode(msg, &hil_gps);

	if (!_param_sim_gps_block.get()) {
		vehicle_gps_position_s gps{};

		gps.timestamp = hrt_absolute_time();
		gps.time_utc_usec = hil_gps.time_usec;
		gps.fix_type = hil_gps.fix_type;
		gps.lat = hil_gps.lat;
		gps.lon = hil_gps.lon;
		gps.alt = hil_gps.alt;
		gps.eph = (float)hil_gps.eph * 1e-2f; // cm -> m
		gps.epv = (float)hil_gps.epv * 1e-2f; // cm -> m
		gps.vel_m_s = (float)(hil_gps.vel) / 100.0f; // cm/s -> m/s
		gps.vel_n_m_s = (float)(hil_gps.vn) / 100.0f; // cm/s -> m/s
		gps.vel_e_m_s = (float)(hil_gps.ve) / 100.0f; // cm/s -> m/s
		gps.vel_d_m_s = (float)(hil_gps.vd) / 100.0f; // cm/s -> m/s
		gps.cog_rad = math::radians((float)(hil_gps.cog) / 100.0f); // cdeg -> rad
		gps.satellites_used = hil_gps.satellites_visible;
		gps.s_variance_m_s = 0.25f;

		// New publishers will be created based on the HIL_GPS ID's being different or not
		for (size_t i = 0; i < sizeof(_gps_ids) / sizeof(_gps_ids[0]); i++) {
			if (_vehicle_gps_position_pubs[i] && _gps_ids[i] == hil_gps.id) {
				_vehicle_gps_position_pubs[i]->publish(gps);
				break;

			}

			if (_vehicle_gps_position_pubs[i] == nullptr) {
				_vehicle_gps_position_pubs[i] = new uORB::PublicationMulti<vehicle_gps_position_s> {ORB_ID(vehicle_gps_position)};
				_gps_ids[i] = hil_gps.id;
				_vehicle_gps_position_pubs[i]->publish(gps);
				break;
			}
		}
	}
}

void Simulator::handle_message_hil_sensor(const mavlink_message_t *msg)
{
	mavlink_hil_sensor_t imu;
	mavlink_msg_hil_sensor_decode(msg, &imu);

	struct timespec ts;
	abstime_to_ts(&ts, imu.time_usec);
	px4_clock_settime(CLOCK_MONOTONIC, &ts);

	hrt_abstime now_us = hrt_absolute_time();

#if 0
	// This is just for to debug missing HIL_SENSOR messages.
	static hrt_abstime last_time = 0;
	hrt_abstime diff = now_us - last_time;
	float step = diff / 4000.0f;

	if (step > 1.1f || step < 0.9f) {
		PX4_INFO("HIL_SENSOR: imu time_usec: %lu, time_usec: %lu, diff: %lu, step: %.2f", imu.time_usec, now_us, diff, step);
	}

	last_time = now_us;
#endif

	update_sensors(now_us, imu);

#if defined(ENABLE_LOCKSTEP_SCHEDULER)

	if (!_has_initialized.load()) {
		_has_initialized.store(true);
	}

#endif
}

void Simulator::handle_message_hil_state_quaternion(const mavlink_message_t *msg)
{
	mavlink_hil_state_quaternion_t hil_state;
	mavlink_msg_hil_state_quaternion_decode(msg, &hil_state);

	uint64_t timestamp = hrt_absolute_time();

	/* angular velocity */
	vehicle_angular_velocity_s hil_angular_velocity{};
	{
		hil_angular_velocity.timestamp = timestamp;

		hil_angular_velocity.xyz[0] = hil_state.rollspeed;
		hil_angular_velocity.xyz[1] = hil_state.pitchspeed;
		hil_angular_velocity.xyz[2] = hil_state.yawspeed;

		// always publish ground truth attitude message
		_vehicle_angular_velocity_ground_truth_pub.publish(hil_angular_velocity);
	}

	/* attitude */
	vehicle_attitude_s hil_attitude{};
	{
		hil_attitude.timestamp = timestamp;

		matrix::Quatf q(hil_state.attitude_quaternion);
		q.copyTo(hil_attitude.q);

		// always publish ground truth attitude message
		_attitude_ground_truth_pub.publish(hil_attitude);
	}

	/* global position */
	vehicle_global_position_s hil_gpos{};
	{
		hil_gpos.timestamp = timestamp;

		hil_gpos.lat = hil_state.lat / 1E7;//1E7
		hil_gpos.lon = hil_state.lon / 1E7;//1E7
		hil_gpos.alt = hil_state.alt / 1E3;//1E3

		// always publish ground truth attitude message
		_gpos_ground_truth_pub.publish(hil_gpos);
	}

	/* local position */
	struct vehicle_local_position_s hil_lpos = {};
	{
		hil_lpos.timestamp = timestamp;

		double lat = hil_state.lat * 1e-7;
		double lon = hil_state.lon * 1e-7;

		if (!_hil_local_proj_inited) {
			_hil_local_proj_inited = true;
			map_projection_init(&_hil_local_proj_ref, lat, lon);
			_hil_ref_timestamp = timestamp;
			_hil_ref_lat = lat;
			_hil_ref_lon = lon;
			_hil_ref_alt = hil_state.alt / 1000.0f;
		}

		float x;
		float y;
		map_projection_project(&_hil_local_proj_ref, lat, lon, &x, &y);
		hil_lpos.timestamp = timestamp;
		hil_lpos.xy_valid = true;
		hil_lpos.z_valid = true;
		hil_lpos.v_xy_valid = true;
		hil_lpos.v_z_valid = true;
		hil_lpos.x = x;
		hil_lpos.y = y;
		hil_lpos.z = _hil_ref_alt - hil_state.alt / 1000.0f;
		hil_lpos.vx = hil_state.vx / 100.0f;
		hil_lpos.vy = hil_state.vy / 100.0f;
		hil_lpos.vz = hil_state.vz / 100.0f;
		matrix::Eulerf euler = matrix::Quatf(hil_attitude.q);
		hil_lpos.heading = euler.psi();
		hil_lpos.xy_global = true;
		hil_lpos.z_global = true;
		hil_lpos.ref_lat = _hil_ref_lat;
		hil_lpos.ref_lon = _hil_ref_lon;
		hil_lpos.ref_alt = _hil_ref_alt;
		hil_lpos.ref_timestamp = _hil_ref_timestamp;
		hil_lpos.vxy_max = std::numeric_limits<float>::infinity();
		hil_lpos.vz_max = std::numeric_limits<float>::infinity();
		hil_lpos.hagl_min = std::numeric_limits<float>::infinity();
		hil_lpos.hagl_max = std::numeric_limits<float>::infinity();

		// always publish ground truth attitude message
		_lpos_ground_truth_pub.publish(hil_lpos);
	}
}

void Simulator::handle_message_landing_target(const mavlink_message_t *msg)
{
	mavlink_landing_target_t landing_target_mavlink;
	mavlink_msg_landing_target_decode(msg, &landing_target_mavlink);

	irlock_report_s report{};
	report.timestamp = hrt_absolute_time();
	report.signature = landing_target_mavlink.target_num;
	report.pos_x = landing_target_mavlink.angle_x;
	report.pos_y = landing_target_mavlink.angle_y;
	report.size_x = landing_target_mavlink.size_x;
	report.size_y = landing_target_mavlink.size_y;

	_irlock_report_pub.publish(report);
}

void Simulator::handle_message_odometry(const mavlink_message_t *msg)
{
	publish_odometry_topic(msg);
}

void Simulator::handle_message_optical_flow(const mavlink_message_t *msg)
{
	mavlink_hil_optical_flow_t flow;
	mavlink_msg_hil_optical_flow_decode(msg, &flow);
	publish_flow_topic(&flow);
}

void Simulator::handle_message_rc_channels(const mavlink_message_t *msg)
{
	mavlink_rc_channels_t rc_channels;
	mavlink_msg_rc_channels_decode(msg, &rc_channels);

	input_rc_s rc_input{};
	rc_input.timestamp_last_signal = hrt_absolute_time();
	rc_input.channel_count = rc_channels.chancount;
	rc_input.rssi = rc_channels.rssi;
	rc_input.values[0] = rc_channels.chan1_raw;
	rc_input.values[1] = rc_channels.chan2_raw;
	rc_input.values[2] = rc_channels.chan3_raw;
	rc_input.values[3] = rc_channels.chan4_raw;
	rc_input.values[4] = rc_channels.chan5_raw;
	rc_input.values[5] = rc_channels.chan6_raw;
	rc_input.values[6] = rc_channels.chan7_raw;
	rc_input.values[7] = rc_channels.chan8_raw;
	rc_input.values[8] = rc_channels.chan9_raw;
	rc_input.values[9] = rc_channels.chan10_raw;
	rc_input.values[10] = rc_channels.chan11_raw;
	rc_input.values[11] = rc_channels.chan12_raw;
	rc_input.values[12] = rc_channels.chan13_raw;
	rc_input.values[13] = rc_channels.chan14_raw;
	rc_input.values[14] = rc_channels.chan15_raw;
	rc_input.values[15] = rc_channels.chan16_raw;
	rc_input.values[16] = rc_channels.chan17_raw;
	rc_input.values[17] = rc_channels.chan18_raw;

	rc_input.timestamp = hrt_absolute_time();

	// publish message
	_input_rc_pub.publish(rc_input);
}

void Simulator::handle_message_vision_position_estimate(const mavlink_message_t *msg)
{
	publish_odometry_topic(msg);
}

void Simulator::send_mavlink_message(const mavlink_message_t &aMsg)
{
	uint8_t  buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t bufLen = 0;

	bufLen = mavlink_msg_to_send_buffer(buf, &aMsg);

	ssize_t len;

	if (_ip == InternetProtocol::UDP) {
		len = ::sendto(_fd, buf, bufLen, 0, (struct sockaddr *)&_srcaddr, sizeof(_srcaddr));

	} else {
		len = ::send(_fd, buf, bufLen, 0);
	}

	if (len <= 0) {
		PX4_WARN("Failed sending mavlink message: %s", strerror(errno));
	}
}

void *Simulator::sending_trampoline(void * /*unused*/)
{
	_instance->send();
	return nullptr;
}

void Simulator::send()
{
#ifdef __PX4_DARWIN
	pthread_setname_np("sim_send");
#else
	pthread_setname_np(pthread_self(), "sim_send");
#endif

	// Before starting, we ought to send a heartbeat to initiate the SITL
	// simulator to start sending sensor data which will set the time and
	// get everything rolling.
	// Without this, we get stuck at px4_poll which waits for a time update.
	send_heartbeat();

	px4_pollfd_struct_t fds_actuator_outputs[1] = {};
	fds_actuator_outputs[0].fd = _actuator_outputs_sub;
	fds_actuator_outputs[0].events = POLLIN;

	while (true) {

		// Wait for up to 100ms for data.
		int pret = px4_poll(&fds_actuator_outputs[0], 1, 100);

		if (pret == 0) {
			// Timed out, try again.
			continue;
		}

		if (pret < 0) {
			PX4_ERR("poll error %s", strerror(errno));
			continue;
		}

		if (fds_actuator_outputs[0].revents & POLLIN) {
			// Got new data to read, update all topics.
			parameters_update(false);
			_vehicle_status_sub.update(&_vehicle_status);
			send_controls();
			// Wait for other modules, such as logger or ekf2
			px4_lockstep_wait_for_components();
		}
	}
}

void Simulator::request_hil_state_quaternion()
{
	mavlink_command_long_t cmd_long = {};
	mavlink_message_t message = {};
	cmd_long.command = MAV_CMD_SET_MESSAGE_INTERVAL;
	cmd_long.param1 = MAVLINK_MSG_ID_HIL_STATE_QUATERNION;
	cmd_long.param2 = 5e3;
	mavlink_msg_command_long_encode(_param_mav_sys_id.get(), _param_mav_comp_id.get(), &message, &cmd_long);
	send_mavlink_message(message);
}

void Simulator::send_heartbeat()
{
	mavlink_heartbeat_t hb = {};
	mavlink_message_t message = {};
	hb.autopilot = 12;
	hb.base_mode |= (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) ? 128 : 0;
	mavlink_msg_heartbeat_encode(_param_mav_sys_id.get(), _param_mav_comp_id.get(), &message, &hb);
	send_mavlink_message(message);
}

void Simulator::run()
{
#ifdef __PX4_DARWIN
	pthread_setname_np("sim_rcv");
#else
	pthread_setname_np(pthread_self(), "sim_rcv");
#endif

	struct sockaddr_in _myaddr {};
	_myaddr.sin_family = AF_INET;
	_myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	_myaddr.sin_port = htons(_port);

	if (_ip == InternetProtocol::UDP) {

		if ((_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
			PX4_ERR("Creating UDP socket failed: %s", strerror(errno));
			return;
		}

		if (bind(_fd, (struct sockaddr *)&_myaddr, sizeof(_myaddr)) < 0) {
			PX4_ERR("bind for UDP port %i failed (%i)", _port, errno);
			::close(_fd);
			return;
		}

		PX4_INFO("Waiting for simulator to connect on UDP port %u", _port);

		while (true) {
			// Once we receive something, we're most probably good and can carry on.
			int len = ::recvfrom(_fd, _buf, sizeof(_buf), 0,
					     (struct sockaddr *)&_srcaddr, (socklen_t *)&_addrlen);

			if (len > 0) {
				break;

			} else {
				system_usleep(100);
			}
		}

		PX4_INFO("Simulator connected on UDP port %u.", _port);

	} else {

		PX4_INFO("Waiting for simulator to accept connection on TCP port %u", _port);

		while (true) {
			if ((_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
				PX4_ERR("Creating TCP socket failed: %s", strerror(errno));
				return;
			}

			int yes = 1;
			int ret = setsockopt(_fd, IPPROTO_TCP, TCP_NODELAY, (char *) &yes, sizeof(int));

			if (ret != 0) {
				PX4_ERR("setsockopt failed: %s", strerror(errno));
			}

			socklen_t myaddr_len = sizeof(_myaddr);
			ret = connect(_fd, (struct sockaddr *)&_myaddr, myaddr_len);

			if (ret == 0) {
				break;

			} else {
				::close(_fd);
				system_usleep(100);
			}
		}

		PX4_INFO("Simulator connected on TCP port %u.", _port);

	}

	// Create a thread for sending data to the simulator.
	pthread_t sender_thread;

	pthread_attr_t sender_thread_attr;
	pthread_attr_init(&sender_thread_attr);
	pthread_attr_setstacksize(&sender_thread_attr, PX4_STACK_ADJUSTED(4000));

	struct sched_param param;
	(void)pthread_attr_getschedparam(&sender_thread_attr, &param);

	// sender thread should run immediately after new outputs are available
	//  to send the lockstep update to the simulation
	param.sched_priority = SCHED_PRIORITY_ACTUATOR_OUTPUTS + 1;
	(void)pthread_attr_setschedparam(&sender_thread_attr, &param);

	struct pollfd fds[2] = {};
	unsigned fd_count = 1;
	fds[0].fd = _fd;
	fds[0].events = POLLIN;

#ifdef ENABLE_UART_RC_INPUT
	// setup serial connection to autopilot (used to get manual controls)
	int serial_fd = openUart(PIXHAWK_DEVICE, PIXHAWK_DEVICE_BAUD);

	char serial_buf[1024];

	if (serial_fd >= 0) {
		fds[1].fd = serial_fd;
		fds[1].events = POLLIN;
		fd_count++;

		PX4_INFO("Start using %s for radio control input.", PIXHAWK_DEVICE);

	} else {
		PX4_INFO("Not using %s for radio control input. Assuming joystick input via MAVLink.", PIXHAWK_DEVICE);
	}

#endif

	// Subscribe to topics.
	// Only subscribe to the first actuator_outputs to fill a single HIL_ACTUATOR_CONTROLS.
	_actuator_outputs_sub = orb_subscribe_multi(ORB_ID(actuator_outputs), 0);

	// got data from simulator, now activate the sending thread
	pthread_create(&sender_thread, &sender_thread_attr, Simulator::sending_trampoline, nullptr);
	pthread_attr_destroy(&sender_thread_attr);

	mavlink_status_t mavlink_status = {};

	// Request HIL_STATE_QUATERNION for ground truth.
	request_hil_state_quaternion();

	while (true) {

		// wait for new mavlink messages to arrive
		int pret = ::poll(&fds[0], fd_count, 1000);

		if (pret == 0) {
			// Timed out.
			continue;
		}

		if (pret < 0) {
			PX4_ERR("poll error %d, %d", pret, errno);
			continue;
		}

		if (fds[0].revents & POLLIN) {

			int len = ::recvfrom(_fd, _buf, sizeof(_buf), 0, (struct sockaddr *)&_srcaddr, (socklen_t *)&_addrlen);

			if (len > 0) {
				mavlink_message_t msg;

				for (int i = 0; i < len; i++) {
					if (mavlink_parse_char(MAVLINK_COMM_0, _buf[i], &msg, &mavlink_status)) {
						handle_message(&msg);
					}
				}
			}
		}

#ifdef ENABLE_UART_RC_INPUT

		// got data from PIXHAWK
		if (fd_count > 1 && fds[1].revents & POLLIN) {
			int len = ::read(serial_fd, serial_buf, sizeof(serial_buf));

			if (len > 0) {
				mavlink_message_t msg;

				mavlink_status_t serial_status = {};

				for (int i = 0; i < len; ++i) {
					if (mavlink_parse_char(MAVLINK_COMM_1, serial_buf[i], &msg, &serial_status)) {
						handle_message(&msg);
					}
				}
			}
		}

#endif
	}

	orb_unsubscribe(_actuator_outputs_sub);
}

#ifdef ENABLE_UART_RC_INPUT
int openUart(const char *uart_name, int baud)
{
	/* process baud rate */
	int speed;

	switch (baud) {
	case 0:      speed = B0;      break;

	case 50:     speed = B50;     break;

	case 75:     speed = B75;     break;

	case 110:    speed = B110;    break;

	case 134:    speed = B134;    break;

	case 150:    speed = B150;    break;

	case 200:    speed = B200;    break;

	case 300:    speed = B300;    break;

	case 600:    speed = B600;    break;

	case 1200:   speed = B1200;   break;

	case 1800:   speed = B1800;   break;

	case 2400:   speed = B2400;   break;

	case 4800:   speed = B4800;   break;

	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

	case 230400: speed = B230400; break;

	case 460800: speed = B460800; break;

	case 921600: speed = B921600; break;

	default:
		PX4_ERR("Unsupported baudrate: %d", baud);
		return -EINVAL;
	}

	/* open uart */
	int uart_fd = ::open(uart_name, O_RDWR | O_NOCTTY);

	if (uart_fd < 0) {
		return uart_fd;
	}


	/* Try to set baud rate */
	struct termios uart_config = {};

	int termios_state;

	/* Back up the original uart configuration to restore it after exit */
	if ((termios_state = tcgetattr(uart_fd, &uart_config)) < 0) {
		PX4_ERR("tcgetattr failed for %s: %s\n", uart_name, strerror(errno));
		::close(uart_fd);
		return -1;
	}

	/* Set baud rate */
	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		PX4_ERR("cfsetispeed or cfsetospeed failed for %s: %s\n", uart_name, strerror(errno));
		::close(uart_fd);
		return -1;
	}

	// Make raw
	cfmakeraw(&uart_config);

	if ((termios_state = tcsetattr(uart_fd, TCSANOW, &uart_config)) < 0) {
		PX4_ERR("tcsetattr failed for %s: %s\n", uart_name, strerror(errno));
		::close(uart_fd);
		return -1;
	}

	return uart_fd;
}
#endif

int Simulator::publish_flow_topic(const mavlink_hil_optical_flow_t *flow_mavlink)
{
	optical_flow_s flow = {};
	flow.sensor_id = flow_mavlink->sensor_id;
	flow.timestamp = hrt_absolute_time();
	flow.time_since_last_sonar_update = 0;
	flow.frame_count_since_last_readout = 0; // ?
	flow.integration_timespan = flow_mavlink->integration_time_us;

	flow.ground_distance_m = flow_mavlink->distance;
	flow.gyro_temperature = flow_mavlink->temperature;
	flow.gyro_x_rate_integral = flow_mavlink->integrated_xgyro;
	flow.gyro_y_rate_integral = flow_mavlink->integrated_ygyro;
	flow.gyro_z_rate_integral = flow_mavlink->integrated_zgyro;
	flow.pixel_flow_x_integral = flow_mavlink->integrated_x;
	flow.pixel_flow_y_integral = flow_mavlink->integrated_y;
	flow.quality = flow_mavlink->quality;

	/* fill in sensor limits */
	float flow_rate_max;
	param_get(param_find("SENS_FLOW_MAXR"), &flow_rate_max);
	float flow_min_hgt;
	param_get(param_find("SENS_FLOW_MINHGT"), &flow_min_hgt);
	float flow_max_hgt;
	param_get(param_find("SENS_FLOW_MAXHGT"), &flow_max_hgt);

	flow.max_flow_rate = flow_rate_max;
	flow.min_ground_distance = flow_min_hgt;
	flow.max_ground_distance = flow_max_hgt;

	/* rotate measurements according to parameter */
	int32_t flow_rot_int;
	param_get(param_find("SENS_FLOW_ROT"), &flow_rot_int);
	const enum Rotation flow_rot = (Rotation)flow_rot_int;

	float zeroval = 0.0f;
	rotate_3f(flow_rot, flow.pixel_flow_x_integral, flow.pixel_flow_y_integral, zeroval);
	rotate_3f(flow_rot, flow.gyro_x_rate_integral, flow.gyro_y_rate_integral, flow.gyro_z_rate_integral);

	_flow_pub.publish(flow);

	return PX4_OK;
}

int Simulator::publish_odometry_topic(const mavlink_message_t *odom_mavlink)
{
	uint64_t timestamp = hrt_absolute_time();

	struct vehicle_odometry_s odom;

	odom.timestamp = timestamp;
	odom.timestamp_sample = timestamp;

	const size_t POS_URT_SIZE = sizeof(odom.pose_covariance) / sizeof(odom.pose_covariance[0]);

	if (odom_mavlink->msgid == MAVLINK_MSG_ID_ODOMETRY) {
		mavlink_odometry_t odom_msg;
		mavlink_msg_odometry_decode(odom_mavlink, &odom_msg);

		/* The position in the local NED frame */
		odom.x = odom_msg.x;
		odom.y = odom_msg.y;
		odom.z = odom_msg.z;

		/* The quaternion of the ODOMETRY msg represents a rotation from
		 * NED earth/local frame to XYZ body frame */
		matrix::Quatf q(odom_msg.q[0], odom_msg.q[1], odom_msg.q[2], odom_msg.q[3]);
		q.copyTo(odom.q);

		if (odom_msg.frame_id == MAV_FRAME_LOCAL_NED) {
			odom.local_frame = vehicle_odometry_s::LOCAL_FRAME_NED;

		} else {
			odom.local_frame = vehicle_odometry_s::LOCAL_FRAME_FRD;
		}

		static_assert(POS_URT_SIZE == (sizeof(odom_msg.pose_covariance) / sizeof(odom_msg.pose_covariance[0])),
			      "Odometry Pose Covariance matrix URT array size mismatch");

		/* The pose covariance URT */
		for (size_t i = 0; i < POS_URT_SIZE; i++) {
			odom.pose_covariance[i] = odom_msg.pose_covariance[i];
		}

		/* The velocity in the body-fixed frame */
		odom.velocity_frame = vehicle_odometry_s::BODY_FRAME_FRD;
		odom.vx = odom_msg.vx;
		odom.vy = odom_msg.vy;
		odom.vz = odom_msg.vz;

		/* The angular velocity in the body-fixed frame */
		odom.rollspeed = odom_msg.rollspeed;
		odom.pitchspeed = odom_msg.pitchspeed;
		odom.yawspeed = odom_msg.yawspeed;

		// velocity_covariance
		static constexpr size_t VEL_URT_SIZE = sizeof(odom.velocity_covariance) / sizeof(odom.velocity_covariance[0]);
		static_assert(VEL_URT_SIZE == (sizeof(odom_msg.velocity_covariance) / sizeof(odom_msg.velocity_covariance[0])),
			      "Odometry Velocity Covariance matrix URT array size mismatch");

		/* The velocity covariance URT */
		for (size_t i = 0; i < VEL_URT_SIZE; i++) {
			odom.velocity_covariance[i] = odom_msg.velocity_covariance[i];
		}

		/* Publish the odometry based on the source */
		if (odom_msg.estimator_type == MAV_ESTIMATOR_TYPE_VISION || odom_msg.estimator_type == MAV_ESTIMATOR_TYPE_VIO) {
			_visual_odometry_pub.publish(odom);

		} else if (odom_msg.estimator_type == MAV_ESTIMATOR_TYPE_MOCAP) {
			_mocap_odometry_pub.publish(odom);

		} else {
			PX4_ERR("Estimator source %u not supported. Unable to publish pose and velocity", odom_msg.estimator_type);
		}

	} else if (odom_mavlink->msgid == MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE) {
		mavlink_vision_position_estimate_t ev;
		mavlink_msg_vision_position_estimate_decode(odom_mavlink, &ev);
		/* The position in the local NED frame */
		odom.x = ev.x;
		odom.y = ev.y;
		odom.z = ev.z;
		/* The euler angles of the VISUAL_POSITION_ESTIMATE msg represent a
		 * rotation from NED earth/local frame to XYZ body frame */
		matrix::Quatf q(matrix::Eulerf(ev.roll, ev.pitch, ev.yaw));
		q.copyTo(odom.q);

		odom.local_frame = vehicle_odometry_s::LOCAL_FRAME_NED;

		static_assert(POS_URT_SIZE == (sizeof(ev.covariance) / sizeof(ev.covariance[0])),
			      "Vision Position Estimate Pose Covariance matrix URT array size mismatch");

		/* The pose covariance URT */
		for (size_t i = 0; i < POS_URT_SIZE; i++) {
			odom.pose_covariance[i] = ev.covariance[i];
		}

		/* The velocity in the local NED frame - unknown */
		odom.vx = NAN;
		odom.vy = NAN;
		odom.vz = NAN;
		/* The angular velocity in body-fixed frame - unknown */
		odom.rollspeed = NAN;
		odom.pitchspeed = NAN;
		odom.yawspeed = NAN;

		/* The velocity covariance URT - unknown */
		odom.velocity_covariance[0] = NAN;

		/* Publish the odometry */
		_visual_odometry_pub.publish(odom);
	}

	return PX4_OK;
}

int Simulator::publish_distance_topic(const mavlink_distance_sensor_t *dist_mavlink)
{
	distance_sensor_s dist{};
	dist.timestamp = hrt_absolute_time();
	dist.min_distance = dist_mavlink->min_distance / 100.0f;
	dist.max_distance = dist_mavlink->max_distance / 100.0f;
	dist.current_distance = dist_mavlink->current_distance / 100.0f;
	dist.type = dist_mavlink->type;
	dist.id = dist_mavlink->id;
	dist.variance = dist_mavlink->covariance * 1e-4f; // cm^2 to m^2

	// MAVLink DISTANCE_SENSOR signal_quality value of 0 means unset/unknown
	// quality value. Also it comes normalised between 1 and 100 while the uORB
	// signal quality is normalised between 0 and 100.
	dist.signal_quality = dist_mavlink->signal_quality == 0 ? -1 : 100 * (dist_mavlink->signal_quality - 1) / 99;

	switch (dist_mavlink->orientation) {
	case MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_PITCH_270:
		dist.orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
		break;

	case MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_PITCH_90:
		dist.orientation = distance_sensor_s::ROTATION_UPWARD_FACING;
		break;

	case MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_PITCH_180:
		dist.orientation = distance_sensor_s::ROTATION_BACKWARD_FACING;
		break;

	case MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_NONE:
		dist.orientation = distance_sensor_s::ROTATION_FORWARD_FACING;
		break;

	case MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_YAW_270:
		dist.orientation = distance_sensor_s::ROTATION_LEFT_FACING;
		break;

	case MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_YAW_90:
		dist.orientation = distance_sensor_s::ROTATION_RIGHT_FACING;
		break;

	default:
		dist.orientation = distance_sensor_s::ROTATION_CUSTOM;
	}

	dist.h_fov = dist_mavlink->horizontal_fov;
	dist.v_fov = dist_mavlink->vertical_fov;
	dist.q[0] = dist_mavlink->quaternion[0];
	dist.q[1] = dist_mavlink->quaternion[1];
	dist.q[2] = dist_mavlink->quaternion[2];
	dist.q[3] = dist_mavlink->quaternion[3];

	// New publishers will be created based on the sensor ID's being different or not
	for (size_t i = 0; i < sizeof(_dist_sensor_ids) / sizeof(_dist_sensor_ids[0]); i++) {
		if (_dist_pubs[i] && _dist_sensor_ids[i] == dist.id) {
			_dist_pubs[i]->publish(dist);
			break;

		}

		if (_dist_pubs[i] == nullptr) {
			_dist_pubs[i] = new uORB::PublicationMulti<distance_sensor_s> {ORB_ID(distance_sensor)};
			_dist_sensor_ids[i] = dist.id;
			_dist_pubs[i]->publish(dist);
			break;
		}
	}

	return PX4_OK;
}
