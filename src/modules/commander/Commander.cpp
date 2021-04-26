/****************************************************************************
 *
 *   Copyright (c) 2013-2021 PX4 Development Team. All rights reserved.
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
 * @file commander.cpp
 *
 * Main state machine / business logic
 *
 * @TODO This application is currently in a rewrite process. Main changes:
 *			- Calibration routines are moved into the event system
 *			- Commander is rewritten as class
 *			- State machines will be model driven
 */

#include "Commander.hpp"

/* commander module headers */
#include "Arming/PreFlightCheck/PreFlightCheck.hpp"
#include "Arming/ArmAuthorization/ArmAuthorization.h"
#include "Arming/HealthFlags/HealthFlags.h"
#include "commander_helper.h"
#include "esc_calibration.h"
#include "px4_custom_mode.h"
#include "state_machine_helper.h"

/* PX4 headers */
#include <dataman/dataman.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_tone_alarm.h>
#include <lib/ecl/geo/geo.h>
#include <mathlib/mathlib.h>
#include <navigator/navigation.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/external_reset_lockout.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/shutdown.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <circuit_breaker/circuit_breaker.h>
#include <systemlib/mavlink_log.h>

#include <math.h>
#include <float.h>
#include <cstring>

#include <uORB/topics/mavlink_log.h>

typedef enum VEHICLE_MODE_FLAG {
	VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED = 1, /* 0b00000001 Reserved for future use. | */
	VEHICLE_MODE_FLAG_TEST_ENABLED = 2, /* 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations. | */
	VEHICLE_MODE_FLAG_AUTO_ENABLED = 4, /* 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation. | */
	VEHICLE_MODE_FLAG_GUIDED_ENABLED = 8, /* 0b00001000 guided mode enabled, system flies MISSIONs / mission items. | */
	VEHICLE_MODE_FLAG_STABILIZE_ENABLED = 16, /* 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around. | */
	VEHICLE_MODE_FLAG_HIL_ENABLED = 32, /* 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational. | */
	VEHICLE_MODE_FLAG_MANUAL_INPUT_ENABLED = 64, /* 0b01000000 remote control input is enabled. | */
	VEHICLE_MODE_FLAG_SAFETY_ARMED = 128, /* 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. Additional note: this flag is to be ignore when sent in the command MAV_CMD_DO_SET_MODE and MAV_CMD_COMPONENT_ARM_DISARM shall be used instead. The flag can still be used to report the armed state. | */
	VEHICLE_MODE_FLAG_ENUM_END = 129, /*  | */
} VEHICLE_MODE_FLAG;

#if defined(BOARD_HAS_POWER_CONTROL)
static orb_advert_t power_button_state_pub = nullptr;
static int power_button_state_notification_cb(board_power_button_state_notification_e request)
{
	// Note: this can be called from IRQ handlers, so we publish a message that will be handled
	// on the main thread of commander.
	power_button_state_s button_state{};
	button_state.timestamp = hrt_absolute_time();
	const int ret = PWR_BUTTON_RESPONSE_SHUT_DOWN_PENDING;

	switch (request) {
	case PWR_BUTTON_IDEL:
		button_state.event = power_button_state_s::PWR_BUTTON_STATE_IDEL;
		break;

	case PWR_BUTTON_DOWN:
		button_state.event = power_button_state_s::PWR_BUTTON_STATE_DOWN;
		break;

	case PWR_BUTTON_UP:
		button_state.event = power_button_state_s::PWR_BUTTON_STATE_UP;
		break;

	case PWR_BUTTON_REQUEST_SHUT_DOWN:
		button_state.event = power_button_state_s::PWR_BUTTON_STATE_REQUEST_SHUTDOWN;
		break;

	default:
		PX4_ERR("unhandled power button state: %i", (int)request);
		return ret;
	}

	if (power_button_state_pub != nullptr) {
		orb_publish(ORB_ID(power_button_state), power_button_state_pub, &button_state);

	} else {
		PX4_ERR("power_button_state_pub not properly initialized");
	}

	return ret;
}
#endif // BOARD_HAS_POWER_CONTROL

#ifndef CONSTRAINED_FLASH
static bool send_vehicle_command(const uint32_t cmd, const float param1 = NAN, const float param2 = NAN,
				 const float param3 = NAN,  const float param4 = NAN, const double param5 = static_cast<double>(NAN),
				 const double param6 = static_cast<double>(NAN), const float param7 = NAN)
{
	vehicle_command_s vcmd{};
	vcmd.command = cmd;
	vcmd.param1 = param1;
	vcmd.param2 = param2;
	vcmd.param3 = param3;
	vcmd.param4 = param4;
	vcmd.param5 = param5;
	vcmd.param6 = param6;
	vcmd.param7 = param7;

	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	vcmd.source_system = vehicle_status_sub.get().system_id;
	vcmd.target_system = vehicle_status_sub.get().system_id;
	vcmd.source_component = vehicle_status_sub.get().component_id;
	vcmd.target_component = vehicle_status_sub.get().component_id;

	uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
	vcmd.timestamp = hrt_absolute_time();
	return vcmd_pub.publish(vcmd);
}

static bool broadcast_vehicle_command(const uint32_t cmd, const float param1 = NAN, const float param2 = NAN,
				      const float param3 = NAN,  const float param4 = NAN, const double param5 = static_cast<double>(NAN),
				      const double param6 = static_cast<double>(NAN), const float param7 = NAN)
{
	vehicle_command_s vcmd{};
	vcmd.command = cmd;
	vcmd.param1 = param1;
	vcmd.param2 = param2;
	vcmd.param3 = param3;
	vcmd.param4 = param4;
	vcmd.param5 = param5;
	vcmd.param6 = param6;
	vcmd.param7 = param7;

	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	vcmd.source_system = vehicle_status_sub.get().system_id;
	vcmd.target_system = 0;
	vcmd.source_component = vehicle_status_sub.get().component_id;
	vcmd.target_component = 0;

	uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
	vcmd.timestamp = hrt_absolute_time();
	return vcmd_pub.publish(vcmd);
}
#endif

int Commander::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

#ifndef CONSTRAINED_FLASH

	if (!strcmp(argv[0], "calibrate")) {
		if (argc > 1) {
			if (!strcmp(argv[1], "gyro")) {
				// gyro calibration: param1 = 1
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 1.f, 0.f, 0.f, 0.f, 0.0, 0.0, 0.f);

			} else if (!strcmp(argv[1], "mag")) {
				if (argc > 2 && (strcmp(argv[2], "quick") == 0)) {
					// magnetometer quick calibration: VEHICLE_CMD_FIXED_MAG_CAL_YAW
					send_vehicle_command(vehicle_command_s::VEHICLE_CMD_FIXED_MAG_CAL_YAW);

				} else {
					// magnetometer calibration: param2 = 1
					send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 0.f, 1.f, 0.f, 0.f, 0.0, 0.0, 0.f);
				}

			} else if (!strcmp(argv[1], "accel")) {
				if (argc > 2 && (strcmp(argv[2], "quick") == 0)) {
					// accelerometer quick calibration: param5 = 3
					send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 0.f, 0.f, 0.f, 0.f, 4.0, 0.0, 0.f);

				} else {
					// accelerometer calibration: param5 = 1
					send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 0.f, 0.f, 0.f, 0.f, 1.0, 0.0, 0.f);
				}

			} else if (!strcmp(argv[1], "level")) {
				// board level calibration: param5 = 2
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 0.f, 0.f, 0.f, 0.f, 2.0, 0.0, 0.f);

			} else if (!strcmp(argv[1], "airspeed")) {
				// airspeed calibration: param6 = 2
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 0.f, 0.f, 0.f, 0.f, 0.0, 2.0, 0.f);

			} else if (!strcmp(argv[1], "esc")) {
				// ESC calibration: param7 = 1
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 0.f, 0.f, 0.f, 0.f, 0.0, 0.0, 1.f);

			} else {
				PX4_ERR("argument %s unsupported.", argv[1]);
				return 1;
			}

			return 0;

		} else {
			PX4_ERR("missing argument");
		}
	}

	if (!strcmp(argv[0], "check")) {
		uORB::Subscription vehicle_status_sub{ORB_ID(vehicle_status)};
		vehicle_status_s vehicle_status{};
		vehicle_status_sub.copy(&vehicle_status);

		uORB::Subscription vehicle_status_flags_sub{ORB_ID(vehicle_status_flags)};
		vehicle_status_flags_s vehicle_status_flags{};
		vehicle_status_flags_sub.copy(&vehicle_status_flags);

		bool preflight_check_res = PreFlightCheck::preflightCheck(nullptr, vehicle_status, vehicle_status_flags, true, true,
					   30_s);
		PX4_INFO("Preflight check: %s", preflight_check_res ? "OK" : "FAILED");

		bool prearm_check_res = PreFlightCheck::preArmCheck(nullptr, vehicle_status_flags, safety_s{},
					PreFlightCheck::arm_requirements_t{}, vehicle_status);
		PX4_INFO("Prearm check: %s", prearm_check_res ? "OK" : "FAILED");

		print_health_flags(vehicle_status);

		return 0;
	}

	if (!strcmp(argv[0], "arm")) {
		float param2 = 0.f;

		// 21196: force arming/disarming (e.g. allow arming to override preflight checks and disarming in flight)
		if (argc > 1 && !strcmp(argv[1], "-f")) {
			param2 = 21196.f;
		}

		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.f, param2);

		return 0;
	}

	if (!strcmp(argv[0], "disarm")) {
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.f, 0.f);

		return 0;
	}

	if (!strcmp(argv[0], "takeoff")) {
		// switch to takeoff mode and arm
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF);
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.f, 0.f);

		return 0;
	}

	if (!strcmp(argv[0], "land")) {
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_LAND);

		return 0;
	}

	if (!strcmp(argv[0], "transition")) {
		uORB::Subscription vehicle_status_sub{ORB_ID(vehicle_status)};
		vehicle_status_s vehicle_status{};
		vehicle_status_sub.copy(&vehicle_status);
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION,
				     (float)(vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING ?
					     vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW :
					     vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC));

		return 0;
	}

	if (!strcmp(argv[0], "mode")) {
		if (argc > 1) {

			if (!strcmp(argv[1], "manual")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_MANUAL);

			} else if (!strcmp(argv[1], "altctl")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_ALTCTL);

			} else if (!strcmp(argv[1], "posctl")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_POSCTL);

			} else if (!strcmp(argv[1], "auto:mission")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
						     PX4_CUSTOM_SUB_MODE_AUTO_MISSION);

			} else if (!strcmp(argv[1], "auto:loiter")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
						     PX4_CUSTOM_SUB_MODE_AUTO_LOITER);

			} else if (!strcmp(argv[1], "auto:rtl")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
						     PX4_CUSTOM_SUB_MODE_AUTO_RTL);

			} else if (!strcmp(argv[1], "acro")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_ACRO);

			} else if (!strcmp(argv[1], "offboard")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_OFFBOARD);

			} else if (!strcmp(argv[1], "stabilized")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_STABILIZED);

			} else if (!strcmp(argv[1], "auto:takeoff")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
						     PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF);

			} else if (!strcmp(argv[1], "auto:land")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
						     PX4_CUSTOM_SUB_MODE_AUTO_LAND);

			} else if (!strcmp(argv[1], "auto:precland")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
						     PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND);

			} else {
				PX4_ERR("argument %s unsupported.", argv[1]);
			}

			return 0;

		} else {
			PX4_ERR("missing argument");
		}
	}

	if (!strcmp(argv[0], "lockdown")) {

		if (argc < 2) {
			Commander::print_usage("not enough arguments, missing [on, off]");
			return 1;
		}

		bool ret = send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_FLIGHTTERMINATION,
						strcmp(argv[1], "off") ? 2.0f : 0.0f /* lockdown */, 0.0f);

		return (ret ? 0 : 1);
	}

	if (!strcmp(argv[0], "pair")) {

		// GCS pairing request handled by a companion
		bool ret = broadcast_vehicle_command(vehicle_command_s::VEHICLE_CMD_START_RX_PAIR, 10.f);

		return (ret ? 0 : 1);
	}

	if (!strcmp(argv[0], "set_ekf_origin")) {
		if (argc > 3) {

			double latitude  = atof(argv[1]);
			double longitude = atof(argv[2]);
			float  altitude  = atof(argv[3]);

			// Set the ekf NED origin global coordinates.
			bool ret = send_vehicle_command(vehicle_command_s::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN,
							0.f, 0.f, 0.0, 0.0, latitude, longitude, altitude);
			return (ret ? 0 : 1);

		} else {
			PX4_ERR("missing argument");
			return 0;
		}
	}


#endif

	return print_usage("unknown command");
}

int Commander::print_status()
{
	PX4_INFO("arming: %s", arming_state_names[_status.arming_state]);
	PX4_INFO("navigation: %s", nav_state_names[_status.nav_state]);
	return 0;
}

extern "C" __EXPORT int commander_main(int argc, char *argv[])
{
	return Commander::main(argc, argv);
}

bool Commander::shutdown_if_allowed()
{
	return TRANSITION_DENIED != arming_state_transition(_status, _safety, vehicle_status_s::ARMING_STATE_SHUTDOWN,
			_armed, false /* fRunPreArmChecks */, &_mavlink_log_pub, _status_flags, _arm_requirements,
			hrt_elapsed_time(&_boot_timestamp), arm_disarm_reason_t::SHUTDOWN);
}

static constexpr const char *arm_disarm_reason_str(arm_disarm_reason_t calling_reason)
{
	switch (calling_reason) {
	case arm_disarm_reason_t::TRANSITION_TO_STANDBY: return "";

	case arm_disarm_reason_t::RC_STICK: return "RC";

	case arm_disarm_reason_t::RC_SWITCH: return "RC (switch)";

	case arm_disarm_reason_t::COMMAND_INTERNAL: return "internal command";

	case arm_disarm_reason_t::COMMAND_EXTERNAL: return "external command";

	case arm_disarm_reason_t::MISSION_START: return "mission start";

	case arm_disarm_reason_t::SAFETY_BUTTON: return "safety button";

	case arm_disarm_reason_t::AUTO_DISARM_LAND: return "landing";

	case arm_disarm_reason_t::AUTO_DISARM_PREFLIGHT: return "auto preflight disarming";

	case arm_disarm_reason_t::KILL_SWITCH: return "kill-switch";

	case arm_disarm_reason_t::LOCKDOWN: return "lockdown";

	case arm_disarm_reason_t::FAILURE_DETECTOR: return "failure detector";

	case arm_disarm_reason_t::SHUTDOWN: return "shutdown request";

	case arm_disarm_reason_t::UNIT_TEST: return "unit tests";
	}

	return "";
};

static constexpr const char *main_state_str(uint8_t main_state)
{
	switch (main_state) {
	case commander_state_s::MAIN_STATE_MANUAL: return "Manual";

	case commander_state_s::MAIN_STATE_ALTCTL: return "Altitude";

	case commander_state_s::MAIN_STATE_POSCTL: return "Position";

	case commander_state_s::MAIN_STATE_AUTO_MISSION: return "Mission";

	case commander_state_s::MAIN_STATE_AUTO_LOITER: return "Hold";

	case commander_state_s::MAIN_STATE_AUTO_RTL: return "RTL";

	case commander_state_s::MAIN_STATE_ACRO: return "Acro";

	case commander_state_s::MAIN_STATE_OFFBOARD: return "Offboard";

	case commander_state_s::MAIN_STATE_STAB: return "Stabilized";

	case commander_state_s::MAIN_STATE_AUTO_TAKEOFF: return "Takeoff";

	case commander_state_s::MAIN_STATE_AUTO_LAND: return "Land";

	case commander_state_s::MAIN_STATE_AUTO_FOLLOW_TARGET: return "Follow target";

	case commander_state_s::MAIN_STATE_AUTO_PRECLAND: return "Precision land";

	case commander_state_s::MAIN_STATE_ORBIT: return "Orbit";

	default: return "Unknown";
	}
}

transition_result_t Commander::arm(arm_disarm_reason_t calling_reason, bool run_preflight_checks)
{
	// allow a grace period for re-arming: preflight checks don't need to pass during that time, for example for accidential in-air disarming
	if (_param_com_rearm_grace.get() && (hrt_elapsed_time(&_last_disarmed_timestamp) < 5_s)) {
		run_preflight_checks = false;
	}

	if (run_preflight_checks) {
		if (_vehicle_control_mode.flag_control_manual_enabled) {
			if (_vehicle_control_mode.flag_control_climb_rate_enabled && _manual_control.isThrottleAboveCenter()) {
				mavlink_log_critical(&_mavlink_log_pub, "Arming denied: throttle above center");
				tune_negative(true);
				return TRANSITION_DENIED;

			}

			if (!_vehicle_control_mode.flag_control_climb_rate_enabled && !_manual_control.isThrottleLow()) {
				mavlink_log_critical(&_mavlink_log_pub, "Arming denied: high throttle");
				tune_negative(true);
				return TRANSITION_DENIED;
			}
		}

		if ((_param_geofence_action.get() == geofence_result_s::GF_ACTION_RTL)
		    && !_status_flags.condition_home_position_valid) {
			mavlink_log_critical(&_mavlink_log_pub, "Arming denied! Geofence RTL requires valid home");
			tune_negative(true);
			return TRANSITION_DENIED;
		}
	}

	transition_result_t arming_res = arming_state_transition(_status,
					 _safety,
					 vehicle_status_s::ARMING_STATE_ARMED,
					 _armed,
					 run_preflight_checks,
					 &_mavlink_log_pub,
					 _status_flags,
					 _arm_requirements,
					 hrt_elapsed_time(&_boot_timestamp), calling_reason);

	if (arming_res == TRANSITION_CHANGED) {
		mavlink_log_info(&_mavlink_log_pub, "Armed by %s", arm_disarm_reason_str(calling_reason));

		_status_changed = true;

	} else if (arming_res == TRANSITION_DENIED) {
		tune_negative(true);
	}

	return arming_res;
}

transition_result_t Commander::disarm(arm_disarm_reason_t calling_reason)
{
	transition_result_t arming_res = arming_state_transition(_status,
					 _safety,
					 vehicle_status_s::ARMING_STATE_STANDBY,
					 _armed,
					 false,
					 &_mavlink_log_pub,
					 _status_flags,
					 _arm_requirements,
					 hrt_elapsed_time(&_boot_timestamp), calling_reason);

	if (arming_res == TRANSITION_CHANGED) {
		mavlink_log_info(&_mavlink_log_pub, "Disarmed by %s", arm_disarm_reason_str(calling_reason));

		_status_changed = true;

	} else if (arming_res == TRANSITION_DENIED) {
		tune_negative(true);
	}

	return arming_res;
}

transition_result_t
Commander::try_mode_change(main_state_t desired_mode)
{
	transition_result_t res = main_state_transition(_status, desired_mode, _status_flags, _internal_state);

	if (res == TRANSITION_DENIED) {

		print_reject_mode(desired_mode);

		if (desired_mode == commander_state_s::MAIN_STATE_OFFBOARD) {
			/* offboard does not have a fallback */
			return res;
		}

		if (desired_mode == commander_state_s::MAIN_STATE_AUTO_MISSION) {
			desired_mode = commander_state_s::MAIN_STATE_AUTO_LOITER;
			res = main_state_transition(_status, desired_mode, _status_flags, _internal_state);
		}

		if (desired_mode == commander_state_s::MAIN_STATE_AUTO_RTL && (res == TRANSITION_DENIED)) {
			desired_mode = commander_state_s::MAIN_STATE_AUTO_LOITER;
			res = main_state_transition(_status, desired_mode, _status_flags, _internal_state);
		}

		if (desired_mode == commander_state_s::MAIN_STATE_AUTO_LAND && (res == TRANSITION_DENIED)) {
			desired_mode = commander_state_s::MAIN_STATE_AUTO_LOITER;
			res = main_state_transition(_status, desired_mode, _status_flags, _internal_state);
		}

		if (desired_mode == commander_state_s::MAIN_STATE_AUTO_TAKEOFF && (res == TRANSITION_DENIED)) {
			desired_mode = commander_state_s::MAIN_STATE_AUTO_LOITER;
			res = main_state_transition(_status, desired_mode, _status_flags, _internal_state);
		}

		if (desired_mode == commander_state_s::MAIN_STATE_AUTO_FOLLOW_TARGET && (res == TRANSITION_DENIED)) {
			desired_mode = commander_state_s::MAIN_STATE_AUTO_LOITER;
			res = main_state_transition(_status, desired_mode, _status_flags, _internal_state);
		}

		if (desired_mode == commander_state_s::MAIN_STATE_AUTO_LOITER && (res == TRANSITION_DENIED)) {
			/* fall back to position control */
			desired_mode = commander_state_s::MAIN_STATE_POSCTL;
			res = main_state_transition(_status, desired_mode, _status_flags, _internal_state);
		}

		if (desired_mode == commander_state_s::MAIN_STATE_POSCTL && (res == TRANSITION_DENIED)) {
			/* fall back to altitude control */
			desired_mode = commander_state_s::MAIN_STATE_ALTCTL;
			res = main_state_transition(_status, desired_mode, _status_flags, _internal_state);
		}

		if (desired_mode == commander_state_s::MAIN_STATE_ALTCTL && (res == TRANSITION_DENIED)) {
			/* fall back to stabilized */
			desired_mode = commander_state_s::MAIN_STATE_STAB;
			res = main_state_transition(_status, desired_mode, _status_flags, _internal_state);
		}

		if (desired_mode == commander_state_s::MAIN_STATE_STAB && (res == TRANSITION_DENIED)) {
			/* fall back to manual */
			desired_mode = commander_state_s::MAIN_STATE_MANUAL;
			res = main_state_transition(_status, desired_mode, _status_flags, _internal_state);
		}
	}

	return res;
}

Commander::Commander() :
	ModuleParams(nullptr),
	_failure_detector(this)
{
	_auto_disarm_landed.set_hysteresis_time_from(false, _param_com_disarm_preflight.get() * 1_s);

	_land_detector.landed = true;

	// XXX for now just set sensors as initialized
	_status_flags.condition_system_sensors_initialized = true;

	// We want to accept RC inputs as default
	_status.rc_input_mode = vehicle_status_s::RC_IN_MODE_DEFAULT;
	_status.nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;
	_status.nav_state_timestamp = hrt_absolute_time();
	_status.arming_state = vehicle_status_s::ARMING_STATE_INIT;

	/* mark all signals lost as long as they haven't been found */
	_status.rc_signal_lost = true;
	_status.data_link_lost = true;

	_status_flags.offboard_control_signal_lost = true;

	_status_flags.condition_power_input_valid = true;
	_status_flags.rc_calibration_valid = true;

	// default for vtol is rotary wing
	_vtol_status.vtol_in_rw_mode = true;

	/* init mission state, do it here to allow navigator to use stored mission even if mavlink failed to start */
	mission_init();
}

bool
Commander::handle_command(const vehicle_command_s &cmd)
{
	/* only handle commands that are meant to be handled by this system and component */
	if (cmd.target_system != _status.system_id || ((cmd.target_component != _status.component_id)
			&& (cmd.target_component != 0))) { // component_id 0: valid for all components
		return false;
	}

	/* result of the command */
	unsigned cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_UNSUPPORTED;

	/* request to set different system mode */
	switch (cmd.command) {
	case vehicle_command_s::VEHICLE_CMD_DO_REPOSITION: {

			// Just switch the flight mode here, the navigator takes care of
			// doing something sensible with the coordinates. Its designed
			// to not require navigator and command to receive / process
			// the data at the exact same time.

			// Check if a mode switch had been requested
			if ((((uint32_t)cmd.param2) & 1) > 0) {
				transition_result_t main_ret = main_state_transition(_status, commander_state_s::MAIN_STATE_AUTO_LOITER,
							       _status_flags, _internal_state);

				if ((main_ret != TRANSITION_DENIED)) {
					cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

				} else {
					cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
					mavlink_log_critical(&_mavlink_log_pub, "Reposition command rejected");
				}

			} else {
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_DO_SET_MODE: {
			uint8_t base_mode = (uint8_t)cmd.param1;
			uint8_t custom_main_mode = (uint8_t)cmd.param2;
			uint8_t custom_sub_mode = (uint8_t)cmd.param3;

			transition_result_t main_ret = TRANSITION_NOT_CHANGED;

			if (base_mode & VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED) {
				/* use autopilot-specific mode */
				if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_MANUAL) {
					/* MANUAL */
					main_ret = main_state_transition(_status, commander_state_s::MAIN_STATE_MANUAL, _status_flags, _internal_state);

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_ALTCTL) {
					/* ALTCTL */
					main_ret = main_state_transition(_status, commander_state_s::MAIN_STATE_ALTCTL, _status_flags, _internal_state);

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_POSCTL) {
					/* POSCTL */
					reset_posvel_validity();
					main_ret = main_state_transition(_status, commander_state_s::MAIN_STATE_POSCTL, _status_flags, _internal_state);

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_AUTO) {
					/* AUTO */
					if (custom_sub_mode > 0) {
						reset_posvel_validity();

						switch (custom_sub_mode) {
						case PX4_CUSTOM_SUB_MODE_AUTO_LOITER:
							main_ret = main_state_transition(_status, commander_state_s::MAIN_STATE_AUTO_LOITER, _status_flags, _internal_state);
							break;

						case PX4_CUSTOM_SUB_MODE_AUTO_MISSION:
							if (_status_flags.condition_auto_mission_available) {
								main_ret = main_state_transition(_status, commander_state_s::MAIN_STATE_AUTO_MISSION, _status_flags, _internal_state);

							} else {
								main_ret = TRANSITION_DENIED;
							}

							break;

						case PX4_CUSTOM_SUB_MODE_AUTO_RTL:
							main_ret = main_state_transition(_status, commander_state_s::MAIN_STATE_AUTO_RTL, _status_flags, _internal_state);
							break;

						case PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF:
							main_ret = main_state_transition(_status, commander_state_s::MAIN_STATE_AUTO_TAKEOFF, _status_flags, _internal_state);
							break;

						case PX4_CUSTOM_SUB_MODE_AUTO_LAND:
							main_ret = main_state_transition(_status, commander_state_s::MAIN_STATE_AUTO_LAND, _status_flags, _internal_state);
							break;

						case PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET:
							main_ret = main_state_transition(_status, commander_state_s::MAIN_STATE_AUTO_FOLLOW_TARGET, _status_flags,
											 _internal_state);
							break;

						case PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND:
							main_ret = main_state_transition(_status, commander_state_s::MAIN_STATE_AUTO_PRECLAND, _status_flags, _internal_state);
							break;

						default:
							main_ret = TRANSITION_DENIED;
							mavlink_log_critical(&_mavlink_log_pub, "Unsupported auto mode");
							break;
						}

					} else {
						main_ret = main_state_transition(_status, commander_state_s::MAIN_STATE_AUTO_MISSION, _status_flags, _internal_state);
					}

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_ACRO) {
					/* ACRO */
					main_ret = main_state_transition(_status, commander_state_s::MAIN_STATE_ACRO, _status_flags, _internal_state);

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_STABILIZED) {
					/* STABILIZED */
					main_ret = main_state_transition(_status, commander_state_s::MAIN_STATE_STAB, _status_flags, _internal_state);

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_OFFBOARD) {
					reset_posvel_validity();

					/* OFFBOARD */
					main_ret = main_state_transition(_status, commander_state_s::MAIN_STATE_OFFBOARD, _status_flags, _internal_state);
				}

			} else {
				/* use base mode */
				if (base_mode & VEHICLE_MODE_FLAG_AUTO_ENABLED) {
					/* AUTO */
					main_ret = main_state_transition(_status, commander_state_s::MAIN_STATE_AUTO_MISSION, _status_flags, _internal_state);

				} else if (base_mode & VEHICLE_MODE_FLAG_MANUAL_INPUT_ENABLED) {
					if (base_mode & VEHICLE_MODE_FLAG_GUIDED_ENABLED) {
						/* POSCTL */
						main_ret = main_state_transition(_status, commander_state_s::MAIN_STATE_POSCTL, _status_flags, _internal_state);

					} else if (base_mode & VEHICLE_MODE_FLAG_STABILIZE_ENABLED) {
						/* STABILIZED */
						main_ret = main_state_transition(_status, commander_state_s::MAIN_STATE_STAB, _status_flags, _internal_state);

					} else {
						/* MANUAL */
						main_ret = main_state_transition(_status, commander_state_s::MAIN_STATE_MANUAL, _status_flags, _internal_state);
					}
				}
			}

			if (main_ret != TRANSITION_DENIED) {
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM: {

			// Adhere to MAVLink specs, but base on knowledge that these fundamentally encode ints
			// for logic state parameters
			const int param1_arm = static_cast<int>(roundf(cmd.param1));

			if (param1_arm != 0 && param1_arm != 1) {
				mavlink_log_critical(&_mavlink_log_pub, "Unsupported ARM_DISARM param: %.3f", (double)cmd.param1);

			} else {
				const bool cmd_arms = (param1_arm == 1);

				// Arm is forced (checks skipped) when param2 is set to a magic number.
				const bool forced = (static_cast<int>(roundf(cmd.param2)) == 21196);

				if (!forced) {
					if (!(_land_detector.landed || _land_detector.maybe_landed) && !is_ground_rover(_status)) {
						if (cmd_arms) {
							if (_armed.armed) {
								mavlink_log_warning(&_mavlink_log_pub, "Arming denied! Already armed");

							} else {
								mavlink_log_critical(&_mavlink_log_pub, "Arming denied! Not landed");
							}

						} else {
							mavlink_log_critical(&_mavlink_log_pub, "Disarming denied! Not landed");
						}

						cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_DENIED;
						break;
					}

					const bool cmd_from_io = (static_cast<int>(roundf(cmd.param3)) == 1234);

					// Flick to in-air restore first if this comes from an onboard system and from IO
					if (cmd.source_system == _status.system_id && cmd.source_component == _status.component_id
					    && cmd_from_io && cmd_arms) {
						_status.arming_state = vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE;
					}
				}

				transition_result_t arming_res = TRANSITION_DENIED;

				if (cmd_arms) {
					if (cmd.from_external) {
						arming_res = arm(arm_disarm_reason_t::COMMAND_EXTERNAL);

					} else {
						arming_res = arm(arm_disarm_reason_t::COMMAND_INTERNAL, !forced);
					}

				} else {
					if (cmd.from_external) {
						arming_res = disarm(arm_disarm_reason_t::COMMAND_EXTERNAL);

					} else {
						arming_res = disarm(arm_disarm_reason_t::COMMAND_INTERNAL);
					}
				}

				if (arming_res == TRANSITION_DENIED) {
					cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;

				} else {
					cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

					/* update home position on arming if at least 500 ms from commander start spent to avoid setting home on in-air restart */
					if (cmd_arms && (arming_res == TRANSITION_CHANGED) &&
					    (hrt_absolute_time() > (_boot_timestamp + INAIR_RESTART_HOLDOFF_INTERVAL)) && !_home_pub.get().manual_home) {

						set_home_position();
					}
				}
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_DO_FLIGHTTERMINATION: {
			if (cmd.param1 > 1.5f) {
				if (!_lockdown_triggered) {
					_armed.lockdown = true;
					_lockdown_triggered = true;
					PX4_WARN("forcing lockdown (motors off)");
				}

			} else if (cmd.param1 > 0.5f) {
				//XXX update state machine?
				if (!_flight_termination_triggered) {
					_armed.force_failsafe = true;
					_flight_termination_triggered = true;
					PX4_WARN("forcing failsafe (termination)");
				}

				if ((int)cmd.param2 <= 0) {
					/* reset all commanded failure modes */
					PX4_WARN("reset all non-flighttermination failsafe commands");
				}

			} else {
				_armed.force_failsafe = false;
				_armed.lockdown = false;

				_lockdown_triggered = false;
				_flight_termination_triggered = false;

				PX4_WARN("disabling failsafe and lockdown");
			}

			cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_DO_SET_HOME: {
			bool use_current = cmd.param1 > 0.5f;

			if (use_current) {
				/* use current position */
				if (set_home_position()) {
					cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

				} else {
					cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
				}

			} else {
				const double lat = cmd.param5;
				const double lon = cmd.param6;
				const float alt = cmd.param7;

				if (PX4_ISFINITE(lat) && PX4_ISFINITE(lon) && PX4_ISFINITE(alt)) {
					const vehicle_local_position_s &local_pos = _local_position_sub.get();

					if (local_pos.xy_global && local_pos.z_global) {
						/* use specified position */
						home_position_s home{};
						home.timestamp = hrt_absolute_time();

						fillGlobalHomePos(home, lat, lon, alt);

						home.manual_home = true;

						// update local projection reference including altitude
						struct map_projection_reference_s ref_pos;
						map_projection_init(&ref_pos, local_pos.ref_lat, local_pos.ref_lon);
						float home_x;
						float home_y;
						map_projection_project(&ref_pos, lat, lon, &home_x, &home_y);
						const float home_z = -(alt - local_pos.ref_alt);
						fillLocalHomePos(home, home_x, home_y, home_z, 0.f);

						/* mark home position as set */
						_status_flags.condition_home_position_valid = _home_pub.update(home);

						cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

					} else {
						cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
					}

				} else {
					cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_DENIED;
				}
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH: {
			/* switch to RTL which ends the mission */
			if (TRANSITION_CHANGED == main_state_transition(_status, commander_state_s::MAIN_STATE_AUTO_RTL, _status_flags,
					_internal_state)) {
				mavlink_log_info(&_mavlink_log_pub, "Returning to launch");
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				mavlink_log_critical(&_mavlink_log_pub, "Return to launch denied");
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF: {
			/* ok, home set, use it to take off */
			if (TRANSITION_CHANGED == main_state_transition(_status, commander_state_s::MAIN_STATE_AUTO_TAKEOFF, _status_flags,
					_internal_state)) {
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else if (_internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_TAKEOFF) {
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				mavlink_log_critical(&_mavlink_log_pub, "Takeoff denied! Please disarm and retry");
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_NAV_LAND: {
			if (TRANSITION_DENIED != main_state_transition(_status, commander_state_s::MAIN_STATE_AUTO_LAND, _status_flags,
					_internal_state)) {
				mavlink_log_info(&_mavlink_log_pub, "Landing at current position");
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				mavlink_log_critical(&_mavlink_log_pub, "Landing denied! Please land manually");
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_NAV_PRECLAND: {
			if (TRANSITION_DENIED != main_state_transition(_status, commander_state_s::MAIN_STATE_AUTO_PRECLAND, _status_flags,
					_internal_state)) {
				mavlink_log_info(&_mavlink_log_pub, "Precision landing");
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				mavlink_log_critical(&_mavlink_log_pub, "Precision landing denied! Please land manually");
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_MISSION_START: {

			cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_DENIED;

			// check if current mission and first item are valid
			if (_status_flags.condition_auto_mission_available) {

				// requested first mission item valid
				if (PX4_ISFINITE(cmd.param1) && (cmd.param1 >= -1) && (cmd.param1 < _mission_result_sub.get().seq_total)) {

					// switch to AUTO_MISSION and ARM
					if ((TRANSITION_DENIED != main_state_transition(_status, commander_state_s::MAIN_STATE_AUTO_MISSION, _status_flags,
							_internal_state))
					    && (TRANSITION_DENIED != arm(arm_disarm_reason_t::MISSION_START))) {

						cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

					} else {
						cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
						mavlink_log_critical(&_mavlink_log_pub, "Mission start denied");
					}
				}

			} else {
				mavlink_log_critical(&_mavlink_log_pub, "Mission start denied! No valid mission");
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_CONTROL_HIGH_LATENCY: {
			// if no high latency telemetry exists send a failed acknowledge
			if (_high_latency_datalink_heartbeat > _boot_timestamp) {
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_FAILED;
				mavlink_log_critical(&_mavlink_log_pub, "Control high latency failed! Telemetry unavailable");
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_DO_ORBIT:

		// Switch to orbit state and let the orbit task handle the command further
		if (TRANSITION_DENIED != main_state_transition(_status, commander_state_s::MAIN_STATE_ORBIT, _status_flags,
				_internal_state)) {
			cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

		} else {
			cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
		}

		break;

	case vehicle_command_s::VEHICLE_CMD_DO_MOTOR_TEST:
		cmd_result = handle_command_motor_test(cmd);
		break;

	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN: {

			const int param1 = cmd.param1;

			if (param1 == 0) {
				// 0: Do nothing for autopilot
				answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);

#if defined(CONFIG_BOARDCTL_RESET)

			} else if ((param1 == 1) && shutdown_if_allowed() && (px4_reboot_request(false, 400_ms) == 0)) {
				// 1: Reboot autopilot
				answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);

				while (1) { px4_usleep(1); }

#endif // CONFIG_BOARDCTL_RESET

#if defined(CONFIG_BOARDCTL_POWEROFF)

			} else if ((param1 == 2) && shutdown_if_allowed() && (px4_shutdown_request(400_ms) == 0)) {
				// 2: Shutdown autopilot
				answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);

				while (1) { px4_usleep(1); }

#endif // CONFIG_BOARDCTL_POWEROFF

#if defined(CONFIG_BOARDCTL_RESET)

			} else if ((param1 == 3) && shutdown_if_allowed() && (px4_reboot_request(true, 400_ms) == 0)) {
				// 3: Reboot autopilot and keep it in the bootloader until upgraded.
				answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);

				while (1) { px4_usleep(1); }

#endif // CONFIG_BOARDCTL_RESET

			} else {
				answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_DENIED);
			}
		}

		break;

	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION: {

			if ((_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED)
			    || _status.arming_state == vehicle_status_s::ARMING_STATE_SHUTDOWN || _worker_thread.isBusy()) {

				// reject if armed or shutting down
				answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED);

			} else {

				/* try to go to INIT/PREFLIGHT arming state */
				if (TRANSITION_DENIED == arming_state_transition(_status, safety_s{}, vehicle_status_s::ARMING_STATE_INIT, _armed,
						false /* fRunPreArmChecks */, &_mavlink_log_pub, _status_flags,
						PreFlightCheck::arm_requirements_t{}, // arming requirements not relevant for switching to ARMING_STATE_INIT
						30_s, // time since boot not relevant for switching to ARMING_STATE_INIT
						(cmd.from_external ? arm_disarm_reason_t::COMMAND_EXTERNAL : arm_disarm_reason_t::COMMAND_INTERNAL))
				   ) {

					answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_DENIED);
					break;

				}

				if ((int)(cmd.param1) == 1) {
					/* gyro calibration */
					answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_status_flags.condition_calibration_enabled = true;
					_worker_thread.startTask(WorkerThread::Request::GyroCalibration);

				} else if ((int)(cmd.param1) == vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION ||
					   (int)(cmd.param5) == vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION ||
					   (int)(cmd.param7) == vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION) {
					/* temperature calibration: handled in events module */
					break;

				} else if ((int)(cmd.param2) == 1) {
					/* magnetometer calibration */
					answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_status_flags.condition_calibration_enabled = true;
					_worker_thread.startTask(WorkerThread::Request::MagCalibration);

				} else if ((int)(cmd.param3) == 1) {
					/* zero-altitude pressure calibration */
					answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_DENIED);

				} else if ((int)(cmd.param4) == 1) {
					/* RC calibration */
					answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);
					/* disable RC control input completely */
					_status_flags.rc_input_blocked = true;
					mavlink_log_info(&_mavlink_log_pub, "Calibration: Disabling RC input");

				} else if ((int)(cmd.param4) == 2) {
					/* RC trim calibration */
					answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_status_flags.condition_calibration_enabled = true;
					_worker_thread.startTask(WorkerThread::Request::RCTrimCalibration);

				} else if ((int)(cmd.param5) == 1) {
					/* accelerometer calibration */
					answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_status_flags.condition_calibration_enabled = true;
					_worker_thread.startTask(WorkerThread::Request::AccelCalibration);

				} else if ((int)(cmd.param5) == 2) {
					// board offset calibration
					answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_status_flags.condition_calibration_enabled = true;
					_worker_thread.startTask(WorkerThread::Request::LevelCalibration);

				} else if ((int)(cmd.param5) == 4) {
					// accelerometer quick calibration
					answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_status_flags.condition_calibration_enabled = true;
					_worker_thread.startTask(WorkerThread::Request::AccelCalibrationQuick);

				} else if ((int)(cmd.param6) == 1 || (int)(cmd.param6) == 2) {
					// TODO: param6 == 1 is deprecated, but we still accept it for a while (feb 2017)
					/* airspeed calibration */
					answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_status_flags.condition_calibration_enabled = true;
					_worker_thread.startTask(WorkerThread::Request::AirspeedCalibration);

				} else if ((int)(cmd.param7) == 1) {
					/* do esc calibration */
					if (check_battery_disconnected(&_mavlink_log_pub)) {
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);
						_status_flags.condition_calibration_enabled = true;
						_armed.in_esc_calibration_mode = true;
						_worker_thread.startTask(WorkerThread::Request::ESCCalibration);

					} else {
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_DENIED);
					}

				} else if ((int)(cmd.param4) == 0) {
					/* RC calibration ended - have we been in one worth confirming? */
					if (_status_flags.rc_input_blocked) {
						/* enable RC control input */
						_status_flags.rc_input_blocked = false;
						mavlink_log_info(&_mavlink_log_pub, "Calibration: Restoring RC input");
					}

					answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);

				} else {
					answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_UNSUPPORTED);
				}
			}

			break;
		}

	case vehicle_command_s::VEHICLE_CMD_FIXED_MAG_CAL_YAW: {
			// Magnetometer quick calibration using world magnetic model and known heading
			if ((_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED)
			    || (_status.arming_state == vehicle_status_s::ARMING_STATE_SHUTDOWN)
			    || _worker_thread.isBusy()) {

				// reject if armed or shutting down
				answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED);

			} else {
				answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);
				// parameter 1: Heading   (degrees)
				// parameter 3: Latitude  (degrees)
				// parameter 4: Longitude (degrees)

				// assume vehicle pointing north (0 degrees) if heading isn't specified
				const float heading_radians = PX4_ISFINITE(cmd.param1) ? math::radians(roundf(cmd.param1)) : 0.f;

				float latitude = NAN;
				float longitude = NAN;

				if (PX4_ISFINITE(cmd.param3) && PX4_ISFINITE(cmd.param4)) {
					// invalid if both lat & lon are 0 (current mavlink spec)
					if ((fabsf(cmd.param3) > 0) && (fabsf(cmd.param4) > 0)) {
						latitude = cmd.param3;
						longitude = cmd.param4;
					}
				}

				_status_flags.condition_calibration_enabled = true;
				_worker_thread.setMagQuickData(heading_radians, latitude, longitude);
				_worker_thread.startTask(WorkerThread::Request::MagCalibrationQuick);
			}

			break;
		}

	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_STORAGE: {

			if ((_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED)
			    || _status.arming_state == vehicle_status_s::ARMING_STATE_SHUTDOWN
			    || _worker_thread.isBusy()) {

				// reject if armed or shutting down
				answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED);

			} else {

				if (((int)(cmd.param1)) == 0) {
					answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_worker_thread.startTask(WorkerThread::Request::ParamLoadDefault);

				} else if (((int)(cmd.param1)) == 1) {
					answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_worker_thread.startTask(WorkerThread::Request::ParamSaveDefault);

				} else if (((int)(cmd.param1)) == 2) {
					answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);
					_worker_thread.startTask(WorkerThread::Request::ParamResetAll);
				}
			}

			break;
		}

	case vehicle_command_s::VEHICLE_CMD_START_RX_PAIR:
	case vehicle_command_s::VEHICLE_CMD_CUSTOM_0:
	case vehicle_command_s::VEHICLE_CMD_CUSTOM_1:
	case vehicle_command_s::VEHICLE_CMD_CUSTOM_2:
	case vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONTROL:
	case vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONFIGURE:
	case vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONTROL_QUAT:
	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_UAVCAN:
	case vehicle_command_s::VEHICLE_CMD_PAYLOAD_PREPARE_DEPLOY:
	case vehicle_command_s::VEHICLE_CMD_PAYLOAD_CONTROL_DEPLOY:
	case vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION:
	case vehicle_command_s::VEHICLE_CMD_DO_TRIGGER_CONTROL:
	case vehicle_command_s::VEHICLE_CMD_DO_DIGICAM_CONTROL:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_CAM_TRIGG_DIST:
	case vehicle_command_s::VEHICLE_CMD_OBLIQUE_SURVEY:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_CAM_TRIGG_INTERVAL:
	case vehicle_command_s::VEHICLE_CMD_SET_CAMERA_MODE:
	case vehicle_command_s::VEHICLE_CMD_SET_CAMERA_ZOOM:
	case vehicle_command_s::VEHICLE_CMD_SET_CAMERA_FOCUS:
	case vehicle_command_s::VEHICLE_CMD_DO_CHANGE_SPEED:
	case vehicle_command_s::VEHICLE_CMD_DO_LAND_START:
	case vehicle_command_s::VEHICLE_CMD_DO_GO_AROUND:
	case vehicle_command_s::VEHICLE_CMD_LOGGING_START:
	case vehicle_command_s::VEHICLE_CMD_LOGGING_STOP:
	case vehicle_command_s::VEHICLE_CMD_NAV_DELAY:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI:
	case vehicle_command_s::VEHICLE_CMD_NAV_ROI:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_LOCATION:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_WPNEXT_OFFSET:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_NONE:
	case vehicle_command_s::VEHICLE_CMD_INJECT_FAILURE:
	case vehicle_command_s::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN:
		/* ignore commands that are handled by other parts of the system */
		break;

	default:
		/* Warn about unsupported commands, this makes sense because only commands
		 * to this component ID (or all) are passed by mavlink. */
		answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_UNSUPPORTED);
		break;
	}

	if (cmd_result != vehicle_command_s::VEHICLE_CMD_RESULT_UNSUPPORTED) {
		/* already warned about unsupported commands in "default" case */
		answer_command(cmd, cmd_result);
	}

	return true;
}

unsigned
Commander::handle_command_motor_test(const vehicle_command_s &cmd)
{
	if (_armed.armed || (_safety.safety_switch_available && !_safety.safety_off)) {
		return vehicle_command_s::VEHICLE_CMD_RESULT_DENIED;
	}

	if (_param_com_mot_test_en.get() != 1) {
		return vehicle_command_s::VEHICLE_CMD_RESULT_DENIED;
	}

	test_motor_s test_motor{};
	test_motor.timestamp = hrt_absolute_time();
	test_motor.motor_number = (int)(cmd.param1 + 0.5f) - 1;

	int throttle_type = (int)(cmd.param2 + 0.5f);

	if (throttle_type != 0) { // 0: MOTOR_TEST_THROTTLE_PERCENT
		return vehicle_command_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
	}

	int motor_count = (int)(cmd.param5 + 0.5);

	if (motor_count > 1) {
		return vehicle_command_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
	}

	test_motor.action = test_motor_s::ACTION_RUN;
	test_motor.value = math::constrain(cmd.param3 / 100.f, 0.f, 1.f);

	if (test_motor.value < FLT_EPSILON) {
		// the message spec is not clear on whether 0 means stop, but it should be closer to what a user expects
		test_motor.value = -1.f;
	}

	test_motor.timeout_ms = (int)(cmd.param4 * 1000.f + 0.5f);

	// enforce a timeout and a maximum limit
	if (test_motor.timeout_ms == 0 || test_motor.timeout_ms > 3000) {
		test_motor.timeout_ms = 3000;
	}

	test_motor.driver_instance = 0; // the mavlink command does not allow to specify the instance, so set to 0 for now
	_test_motor_pub.publish(test_motor);

	return vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;
}

/**
* @brief This function initializes the home position an altitude of the vehicle. This happens first time we get a good GPS fix and each
*		 time the vehicle is armed with a good GPS fix.
**/
bool
Commander::set_home_position()
{
	// Need global and local position fix to be able to set home
	// but already set the home position in local coordinates if available
	// in case the global position is only valid after takeoff
	if (_status_flags.condition_local_position_valid) {

		// Set home position in local coordinates
		const vehicle_local_position_s &lpos = _local_position_sub.get();
		_heading_reset_counter = lpos.heading_reset_counter;

		home_position_s home{};
		home.timestamp = hrt_absolute_time();
		home.manual_home = false;
		fillLocalHomePos(home, lpos);

		if (_status_flags.condition_global_position_valid) {

			const vehicle_global_position_s &gpos = _global_position_sub.get();

			// Ensure that the GPS accuracy is good enough for intializing home
			if (isGPosGoodForInitializingHomePos(gpos)) {
				fillGlobalHomePos(home, gpos);
				setHomePosValid();
			}
		}

		_home_pub.update(home);
	}

	return _status_flags.condition_home_position_valid;
}

bool
Commander::set_in_air_home_position()
{
	if (_status_flags.condition_local_position_valid
	    && _status_flags.condition_global_position_valid) {

		const vehicle_global_position_s &gpos = _global_position_sub.get();
		home_position_s home{};

		// Ensure that the GPS accuracy is good enough for intializing home
		if (isGPosGoodForInitializingHomePos(gpos)) {
			home = _home_pub.get();
			home.timestamp = hrt_absolute_time();
			const vehicle_local_position_s &lpos = _local_position_sub.get();

			if (_home_pub.get().valid_lpos) {
				// Back-compute lon, lat and alt of home position given the home
				// and current positions in local frame
				map_projection_reference_s ref_pos;
				double home_lat;
				double home_lon;
				map_projection_init(&ref_pos, gpos.lat, gpos.lon);
				map_projection_reproject(&ref_pos, home.x - lpos.x, home.y - lpos.y, &home_lat, &home_lon);
				const float home_alt = gpos.alt + home.z;
				fillGlobalHomePos(home, home_lat, home_lon, home_alt);

			} else {
				// Home position in local frame is unknowm, set
				// home as current position
				fillLocalHomePos(home, lpos);
				fillGlobalHomePos(home, gpos);
			}

			setHomePosValid();
			_home_pub.update(home);
		}
	}

	return _status_flags.condition_home_position_valid;
}

bool
Commander::isGPosGoodForInitializingHomePos(const vehicle_global_position_s &gpos) const
{
	return (gpos.eph <= _param_com_home_h_t.get())
	       && (gpos.epv <= _param_com_home_v_t.get());
}

void
Commander::fillLocalHomePos(home_position_s &home, const vehicle_local_position_s &lpos) const
{
	fillLocalHomePos(home, lpos.x, lpos.y, lpos.z, lpos.heading);
}

void
Commander::fillLocalHomePos(home_position_s &home, float x, float y, float z, float heading) const
{
	home.x = x;
	home.y = y;
	home.z = z;
	home.valid_lpos = true;

	home.yaw = heading;
}

void Commander::fillGlobalHomePos(home_position_s &home, const vehicle_global_position_s &gpos) const
{
	fillGlobalHomePos(home, gpos.lat, gpos.lon, gpos.alt);
}

void Commander::fillGlobalHomePos(home_position_s &home, double lat, double lon, float alt) const
{
	home.lat = lat;
	home.lon = lon;
	home.valid_hpos = true;
	home.alt = alt;
	home.valid_alt = true;
}

void Commander::setHomePosValid()
{
	// play tune first time we initialize HOME
	if (!_status_flags.condition_home_position_valid) {
		tune_home_set(true);
	}

	// mark home position as set
	_status_flags.condition_home_position_valid = true;
}

bool
Commander::set_home_position_alt_only()
{
	const vehicle_local_position_s &lpos = _local_position_sub.get();

	if (!_home_pub.get().valid_alt && lpos.z_global) {
		// handle special case where we are setting only altitude using local position reference
		home_position_s home{};
		home.alt = lpos.ref_alt;
		home.valid_alt = true;

		home.timestamp = hrt_absolute_time();

		return _home_pub.update(home);
	}

	return false;
}

void
Commander::updateHomePositionYaw(float yaw)
{
	home_position_s home = _home_pub.get();

	home.yaw = yaw;
	home.timestamp = hrt_absolute_time();

	_home_pub.update(home);
}

void
Commander::run()
{
	bool sensor_fail_tune_played = false;

	const param_t param_airmode = param_find("MC_AIRMODE");
	const param_t param_rc_map_arm_switch = param_find("RC_MAP_ARM_SW");

	/* initialize */
	led_init();
	buzzer_init();

#if defined(BOARD_HAS_POWER_CONTROL)
	{
		// we need to do an initial publication to make sure uORB allocates the buffer, which cannot happen
		// in IRQ context.
		power_button_state_s button_state{};
		button_state.timestamp = hrt_absolute_time();
		button_state.event = 0xff;
		power_button_state_pub = orb_advertise(ORB_ID(power_button_state), &button_state);

		_power_button_state_sub.copy(&button_state);
	}

	if (board_register_power_state_notification_cb(power_button_state_notification_cb) != 0) {
		PX4_ERR("Failed to register power notification callback");
	}

#endif // BOARD_HAS_POWER_CONTROL

	get_circuit_breaker_params();

	bool param_init_forced = true;

	control_status_leds(true, _battery_warning);

	/* update vehicle status to find out vehicle type (required for preflight checks) */
	_status.system_type = _param_mav_type.get();

	if (is_rotary_wing(_status) || is_vtol(_status)) {
		_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	} else if (is_fixed_wing(_status)) {
		_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;

	} else if (is_ground_rover(_status)) {
		_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROVER;

	} else {
		_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_UNKNOWN;
	}

	_status.is_vtol = is_vtol(_status);
	_status.is_vtol_tailsitter = is_vtol_tailsitter(_status);

	_boot_timestamp = hrt_absolute_time();

	// initially set to failed
	_last_lpos_fail_time_us = _boot_timestamp;
	_last_gpos_fail_time_us = _boot_timestamp;
	_last_lvel_fail_time_us = _boot_timestamp;

	int32_t airmode = 0;
	int32_t rc_map_arm_switch = 0;

	_status.system_id = _param_mav_sys_id.get();
	arm_auth_init(&_mavlink_log_pub, &_status.system_id);

	// run preflight immediately to find all relevant parameters, but don't report
	PreFlightCheck::preflightCheck(&_mavlink_log_pub, _status, _status_flags, false,
				       true,
				       hrt_elapsed_time(&_boot_timestamp));

	while (!should_exit()) {

		/* update parameters */
		const bool params_updated = _parameter_update_sub.updated();

		if (params_updated || param_init_forced) {
			// clear update
			parameter_update_s update;
			_parameter_update_sub.copy(&update);

			// update parameters from storage
			updateParams();

			// NAV_DLL_ACT value 4 Data Link Auto Recovery (CASA Outback Challenge rules) deleted 2020-03-10
			if (_param_nav_dll_act.get() == 4) {
				mavlink_log_critical(&_mavlink_log_pub, "CASA Outback Challenge rules (NAV_DLL_ACT = 4) retired");
				_param_nav_dll_act.set(2); // value 2 Return mode
				_param_nav_dll_act.commit_no_notification();
			}

			// NAV_RCL_ACT value 4 RC Auto Recovery (CASA Outback Challenge rules) deleted 2020-03-10
			if (_param_nav_rcl_act.get() == 4) {
				mavlink_log_critical(&_mavlink_log_pub, "CASA Outback Challenge rules (NAV_RCL_ACT = 4) retired");
				_param_nav_rcl_act.set(2); // value 2 Return mode
				_param_nav_rcl_act.commit_no_notification();
			}

			/* update parameters */
			if (!_armed.armed) {
				_status.system_type = _param_mav_type.get();

				const bool is_rotary = is_rotary_wing(_status) || (is_vtol(_status) && _vtol_status.vtol_in_rw_mode);
				const bool is_fixed = is_fixed_wing(_status) || (is_vtol(_status) && !_vtol_status.vtol_in_rw_mode);
				const bool is_ground = is_ground_rover(_status);

				/* disable manual override for all systems that rely on electronic stabilization */
				if (is_rotary) {
					_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

				} else if (is_fixed) {
					_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;

				} else if (is_ground) {
					_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROVER;
				}

				/* set vehicle_status.is_vtol flag */
				_status.is_vtol = is_vtol(_status);
				_status.is_vtol_tailsitter = is_vtol_tailsitter(_status);

				/* check and update system / component ID */
				_status.system_id = _param_mav_sys_id.get();
				_status.component_id = _param_mav_comp_id.get();

				get_circuit_breaker_params();

				_status_changed = true;
			}

			_status_flags.avoidance_system_required = _param_com_obs_avoid.get();

			_status.rc_input_mode = _param_rc_in_off.get();

			_arm_requirements.arm_authorization = _param_arm_auth_required.get();
			_arm_requirements.esc_check = _param_escs_checks_required.get();
			_arm_requirements.global_position = !_param_arm_without_gps.get();
			_arm_requirements.mission = _param_arm_mission_required.get();

			/* flight mode slots */
			_flight_mode_slots[0] = _param_fltmode_1.get();
			_flight_mode_slots[1] = _param_fltmode_2.get();
			_flight_mode_slots[2] = _param_fltmode_3.get();
			_flight_mode_slots[3] = _param_fltmode_4.get();
			_flight_mode_slots[4] = _param_fltmode_5.get();
			_flight_mode_slots[5] = _param_fltmode_6.get();

			_auto_disarm_killed.set_hysteresis_time_from(false, _param_com_kill_disarm.get() * 1_s);

			/* check for unsafe Airmode settings: yaw airmode requires the use of an arming switch */
			if (param_airmode != PARAM_INVALID && param_rc_map_arm_switch != PARAM_INVALID) {
				param_get(param_airmode, &airmode);
				param_get(param_rc_map_arm_switch, &rc_map_arm_switch);

				if (airmode == 2 && rc_map_arm_switch == 0) {
					airmode = 1; // change to roll/pitch airmode
					param_set(param_airmode, &airmode);
					mavlink_log_critical(&_mavlink_log_pub, "Yaw Airmode requires the use of an Arm Switch")
				}
			}

			_offboard_available.set_hysteresis_time_from(true, _param_com_of_loss_t.get() * 1e6f);

			param_init_forced = false;
		}

		/* Update OA parameter */
		_status_flags.avoidance_system_required = _param_com_obs_avoid.get();

#if defined(BOARD_HAS_POWER_CONTROL)

		/* handle power button state */
		if (_power_button_state_sub.updated()) {
			power_button_state_s button_state;

			if (_power_button_state_sub.copy(&button_state)) {
				if (button_state.event == power_button_state_s::PWR_BUTTON_STATE_REQUEST_SHUTDOWN) {
#if defined(CONFIG_BOARDCTL_POWEROFF)

					if (shutdown_if_allowed() && (px4_shutdown_request() == 0)) {
						while (1) { px4_usleep(1); }
					}

#endif // CONFIG_BOARDCTL_POWEROFF
				}
			}
		}

#endif // BOARD_HAS_POWER_CONTROL

		offboard_control_update();

		if (_system_power_sub.updated()) {
			system_power_s system_power{};
			_system_power_sub.copy(&system_power);

			if (hrt_elapsed_time(&system_power.timestamp) < 1_s) {
				if (system_power.servo_valid &&
				    !system_power.brick_valid &&
				    !system_power.usb_connected) {
					/* flying only on servo rail, this is unsafe */
					_status_flags.condition_power_input_valid = false;

				} else {
					_status_flags.condition_power_input_valid = true;
				}

#if defined(CONFIG_BOARDCTL_RESET)

				if (!_status_flags.circuit_breaker_engaged_usb_check && _status_flags.usb_connected) {
					/* if the USB hardware connection went away, reboot */
					if (_system_power_usb_connected && !system_power.usb_connected) {
						/*
						 * Apparently the USB cable went away but we are still powered,
						 * so we bring the system back to a nominal state for flight.
						 * This is important to unload the USB stack of the OS which is
						 * a relatively complex piece of software that is non-essential
						 * for flight and continuing to run it would add a software risk
						 * without a need. The clean approach to unload it is to reboot.
						 */
						if (shutdown_if_allowed() && (px4_reboot_request(false, 400_ms) == 0)) {
							mavlink_log_critical(&_mavlink_log_pub, "USB disconnected, rebooting for flight safety");

							while (1) { px4_usleep(1); }
						}
					}
				}

#endif // CONFIG_BOARDCTL_RESET

				_system_power_usb_connected = system_power.usb_connected;
			}
		}

		/* Update land detector */
		if (_land_detector_sub.updated()) {
			const bool was_landed = _land_detector.landed;
			_land_detector_sub.copy(&_land_detector);

			// Only take actions if armed
			if (_armed.armed) {
				if (!was_landed && _land_detector.landed) {
					mavlink_log_info(&_mavlink_log_pub, "Landing detected");
					_status.takeoff_time = 0;

				} else if (was_landed && !_land_detector.landed) {
					mavlink_log_info(&_mavlink_log_pub, "Takeoff detected");
					_status.takeoff_time = hrt_absolute_time();
					_have_taken_off_since_arming = true;

					// Set all position and velocity test probation durations to takeoff value
					// This is a larger value to give the vehicle time to complete a failsafe landing
					// if faulty sensors cause loss of navigation shortly after takeoff.
					_gpos_probation_time_us = _param_com_pos_fs_prob.get() * 1_s;
					_lpos_probation_time_us = _param_com_pos_fs_prob.get() * 1_s;
					_lvel_probation_time_us = _param_com_pos_fs_prob.get() * 1_s;
				}

				// automatically set or update home position
				if (!_home_pub.get().manual_home) {
					// set the home position when taking off, but only if we were previously disarmed
					// and at least 500 ms from commander start spent to avoid setting home on in-air restart
					if (_should_set_home_on_takeoff && !_land_detector.landed &&
					    (hrt_elapsed_time(&_boot_timestamp) > INAIR_RESTART_HOLDOFF_INTERVAL)) {
						if (was_landed) {
							_should_set_home_on_takeoff = !set_home_position();

						} else if (_param_com_home_in_air.get()) {
							_should_set_home_on_takeoff = !set_in_air_home_position();
						}
					}
				}
			}
		}

		/* update safety topic */
		const bool safety_updated = _safety_sub.updated();

		if (safety_updated) {
			const bool previous_safety_off = _safety.safety_off;

			if (_safety_sub.copy(&_safety)) {
				set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_MOTORCONTROL, _safety.safety_switch_available, _safety.safety_off,
						 _safety.safety_switch_available, _status);

				// disarm if safety is now on and still armed
				if (_armed.armed && _safety.safety_switch_available && !_safety.safety_off) {

					bool safety_disarm_allowed = (_status.hil_state == vehicle_status_s::HIL_STATE_OFF);

					// prevent disarming via safety button if not landed
					if (hrt_elapsed_time(&_land_detector.timestamp) < 10_s) {
						if (!_land_detector.landed) {
							safety_disarm_allowed = false;
						}
					}

					if (safety_disarm_allowed) {
						disarm(arm_disarm_reason_t::SAFETY_BUTTON);
					}
				}

				// Notify the user if the status of the safety switch changes
				if (_safety.safety_switch_available && previous_safety_off != _safety.safety_off) {

					if (_safety.safety_off) {
						set_tune(tune_control_s::TUNE_ID_NOTIFY_POSITIVE);

					} else {
						tune_neutral(true);
					}

					_status_changed = true;
				}
			}
		}

		/* update vtol vehicle status*/
		if (_vtol_vehicle_status_sub.updated()) {
			/* vtol status changed */
			_vtol_vehicle_status_sub.copy(&_vtol_status);
			_status.vtol_fw_permanent_stab = _vtol_status.fw_permanent_stab;

			/* Make sure that this is only adjusted if vehicle really is of type vtol */
			if (is_vtol(_status)) {

				// Check if there has been any change while updating the flags
				const auto new_vehicle_type = _vtol_status.vtol_in_rw_mode ?
							      vehicle_status_s::VEHICLE_TYPE_ROTARY_WING :
							      vehicle_status_s::VEHICLE_TYPE_FIXED_WING;

				if (new_vehicle_type != _status.vehicle_type) {
					_status.vehicle_type = _vtol_status.vtol_in_rw_mode ?
							       vehicle_status_s::VEHICLE_TYPE_ROTARY_WING :
							       vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
					_status_changed = true;
				}

				if (_status.in_transition_mode != _vtol_status.vtol_in_trans_mode) {
					_status.in_transition_mode = _vtol_status.vtol_in_trans_mode;
					_status_changed = true;
				}

				if (_status.in_transition_to_fw != _vtol_status.in_transition_to_fw) {
					_status.in_transition_to_fw = _vtol_status.in_transition_to_fw;
					_status_changed = true;
				}

				if (_status_flags.vtol_transition_failure != _vtol_status.vtol_transition_failsafe) {
					_status_flags.vtol_transition_failure = _vtol_status.vtol_transition_failsafe;
					_status_changed = true;
				}

				const bool should_soft_stop = (_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);

				if (_armed.soft_stop != should_soft_stop) {
					_armed.soft_stop = should_soft_stop;
					_status_changed = true;
				}
			}
		}

		if (_esc_status_sub.updated()) {
			/* ESCs status changed */
			esc_status_check();

		} else if (_param_escs_checks_required.get() != 0) {

			if (!_status_flags.condition_escs_error) {

				if ((_last_esc_status_updated != 0) && (hrt_elapsed_time(&_last_esc_status_updated) > 700_ms)) {
					/* Detect timeout after first telemetry packet received
					 * Some DShot ESCs are unresponsive for ~550ms during their initialization, so we use a timeout higher than that
					 */

					mavlink_log_critical(&_mavlink_log_pub, "ESCs telemetry timeout");
					_status_flags.condition_escs_error = true;

				} else if (_last_esc_status_updated == 0 && hrt_elapsed_time(&_boot_timestamp) > 5000_ms) {
					/* Detect if esc telemetry is not connected after reboot */
					mavlink_log_critical(&_mavlink_log_pub, "ESCs telemetry not connected ");
					_status_flags.condition_escs_error = true;
				}
			}
		}

		estimator_check();

		// Auto disarm when landed or kill switch engaged
		if (_armed.armed) {

			// Check for auto-disarm on landing or pre-flight
			if (_param_com_disarm_land.get() > 0 || _param_com_disarm_preflight.get() > 0) {

				if (_param_com_disarm_land.get() > 0 && _have_taken_off_since_arming) {
					_auto_disarm_landed.set_hysteresis_time_from(false, _param_com_disarm_land.get() * 1_s);
					_auto_disarm_landed.set_state_and_update(_land_detector.landed, hrt_absolute_time());

				} else if (_param_com_disarm_preflight.get() > 0 && !_have_taken_off_since_arming) {
					_auto_disarm_landed.set_hysteresis_time_from(false, _param_com_disarm_preflight.get() * 1_s);
					_auto_disarm_landed.set_state_and_update(true, hrt_absolute_time());
				}

				if (_auto_disarm_landed.get_state()) {
					if (_have_taken_off_since_arming) {
						disarm(arm_disarm_reason_t::AUTO_DISARM_LAND);

					} else {
						disarm(arm_disarm_reason_t::AUTO_DISARM_PREFLIGHT);
					}
				}
			}

			// Auto disarm after 5 seconds if kill switch is engaged
			bool auto_disarm = _armed.manual_lockdown;

			// auto disarm if locked down to avoid user confusion
			//  skipped in HITL where lockdown is enabled for safety
			if (_status.hil_state != vehicle_status_s::HIL_STATE_ON) {
				auto_disarm |= _armed.lockdown;
			}

			_auto_disarm_killed.set_state_and_update(auto_disarm, hrt_absolute_time());

			if (_auto_disarm_killed.get_state()) {
				if (_armed.manual_lockdown) {
					disarm(arm_disarm_reason_t::KILL_SWITCH);

				} else {
					disarm(arm_disarm_reason_t::LOCKDOWN);
				}
			}

		} else {
			_auto_disarm_landed.set_state_and_update(false, hrt_absolute_time());
			_auto_disarm_killed.set_state_and_update(false, hrt_absolute_time());
		}

		if (_geofence_warning_action_on
		    && _internal_state.main_state != commander_state_s::MAIN_STATE_AUTO_RTL
		    && _internal_state.main_state != commander_state_s::MAIN_STATE_AUTO_LOITER
		    && _internal_state.main_state != commander_state_s::MAIN_STATE_AUTO_LAND) {

			// reset flag again when we switched out of it
			_geofence_warning_action_on = false;
		}

		_cpuload_sub.update(&_cpuload);

		battery_status_check();

		/* If in INIT state, try to proceed to STANDBY state */
		if (!_status_flags.condition_calibration_enabled && _status.arming_state == vehicle_status_s::ARMING_STATE_INIT) {

			arming_state_transition(_status, _safety, vehicle_status_s::ARMING_STATE_STANDBY, _armed,
						true /* fRunPreArmChecks */, &_mavlink_log_pub, _status_flags,
						_arm_requirements, hrt_elapsed_time(&_boot_timestamp),
						arm_disarm_reason_t::TRANSITION_TO_STANDBY);
		}

		/* start mission result check */
		if (_mission_result_sub.updated()) {
			const mission_result_s &mission_result = _mission_result_sub.get();

			const auto prev_mission_instance_count = mission_result.instance_count;
			_mission_result_sub.update();

			// if mission_result is valid for the current mission
			const bool mission_result_ok = (mission_result.timestamp > _boot_timestamp)
						       && (mission_result.instance_count > 0);

			_status_flags.condition_auto_mission_available = mission_result_ok && mission_result.valid;

			if (mission_result_ok) {
				if (_status.mission_failure != mission_result.failure) {
					_status.mission_failure = mission_result.failure;
					_status_changed = true;

					if (_status.mission_failure) {
						mavlink_log_critical(&_mavlink_log_pub, "Mission cannot be completed");
					}
				}

				/* Only evaluate mission state if home is set */
				if (_status_flags.condition_home_position_valid &&
				    (prev_mission_instance_count != mission_result.instance_count)) {

					if (!_status_flags.condition_auto_mission_available) {
						/* the mission is invalid */
						tune_mission_fail(true);

					} else if (mission_result.warning) {
						/* the mission has a warning */
						tune_mission_fail(true);

					} else {
						/* the mission is valid */
						tune_mission_ok(true);
					}
				}
			}

			// Transition main state to loiter or auto-mission after takeoff is completed.
			if (_armed.armed && !_land_detector.landed
			    && (_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF)
			    && (mission_result.timestamp >= _status.nav_state_timestamp)
			    && mission_result.finished) {

				if ((_param_takeoff_finished_action.get() == 1) && _status_flags.condition_auto_mission_available) {
					main_state_transition(_status, commander_state_s::MAIN_STATE_AUTO_MISSION, _status_flags, _internal_state);

				} else {
					main_state_transition(_status, commander_state_s::MAIN_STATE_AUTO_LOITER, _status_flags, _internal_state);
				}
			}
		}

		/* start geofence result check */
		_geofence_result_sub.update(&_geofence_result);

		const bool in_low_battery_failsafe = _battery_warning > battery_status_s::BATTERY_WARNING_LOW;

		// Geofence actions
		const bool geofence_action_enabled = _geofence_result.geofence_action != geofence_result_s::GF_ACTION_NONE;

		if (_armed.armed
		    && geofence_action_enabled
		    && !in_low_battery_failsafe) {

			// check for geofence violation transition
			if (_geofence_result.geofence_violated && !_geofence_violated_prev) {

				switch (_geofence_result.geofence_action) {
				case (geofence_result_s::GF_ACTION_NONE) : {
						// do nothing
						break;
					}

				case (geofence_result_s::GF_ACTION_WARN) : {
						// do nothing, mavlink critical messages are sent by navigator
						break;
					}

				case (geofence_result_s::GF_ACTION_LOITER) : {
						if (TRANSITION_CHANGED == main_state_transition(_status, commander_state_s::MAIN_STATE_AUTO_LOITER, _status_flags,
								_internal_state)) {
							_geofence_loiter_on = true;
						}

						break;
					}

				case (geofence_result_s::GF_ACTION_RTL) : {
						if (TRANSITION_CHANGED == main_state_transition(_status, commander_state_s::MAIN_STATE_AUTO_RTL, _status_flags,
								_internal_state)) {
							_geofence_rtl_on = true;
						}

						break;
					}

				case (geofence_result_s::GF_ACTION_LAND) : {
						if (TRANSITION_CHANGED == main_state_transition(_status, commander_state_s::MAIN_STATE_AUTO_LAND, _status_flags,
								_internal_state)) {
							_geofence_land_on = true;
						}

						break;
					}

				case (geofence_result_s::GF_ACTION_TERMINATE) : {
						PX4_WARN("Flight termination because of geofence");
						mavlink_log_critical(&_mavlink_log_pub, "Geofence violation! Flight terminated");
						_armed.force_failsafe = true;
						_status_changed = true;
						break;
					}
				}
			}

			_geofence_violated_prev = _geofence_result.geofence_violated;

			// reset if no longer in LOITER or if manually switched to LOITER
			const bool in_loiter_mode = _internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_LOITER;
			const bool manual_loiter_switch_on = _manual_control_switches.loiter_switch == manual_control_switches_s::SWITCH_POS_ON;

			if (!in_loiter_mode || manual_loiter_switch_on) {
				_geofence_loiter_on = false;
			}


			// reset if no longer in RTL or if manually switched to RTL
			const bool in_rtl_mode = _internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_RTL;
			const bool manual_return_switch_on = _manual_control_switches.return_switch == manual_control_switches_s::SWITCH_POS_ON;

			if (!in_rtl_mode || manual_return_switch_on) {
				_geofence_rtl_on = false;
			}

			// reset if no longer in LAND or if manually switched to LAND
			const bool in_land_mode = _internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_LAND;

			if (!in_land_mode) {
				_geofence_land_on = false;
			}

			_geofence_warning_action_on = _geofence_warning_action_on || (_geofence_loiter_on || _geofence_rtl_on
						      || _geofence_land_on);

		} else {
			// No geofence checks, reset flags
			_geofence_loiter_on = false;
			_geofence_rtl_on = false;
			_geofence_land_on = false;
			_geofence_warning_action_on = false;
			_geofence_violated_prev = false;
		}

		/* Check for mission flight termination */
		if (_armed.armed && _mission_result_sub.get().flight_termination &&
		    !_status_flags.circuit_breaker_flight_termination_disabled) {

			_armed.force_failsafe = true;
			_status_changed = true;

			if (!_flight_termination_printed) {
				mavlink_log_critical(&_mavlink_log_pub, "Geofence violation! Flight terminated");
				_flight_termination_printed = true;
			}

			if (_counter % (1000000 / COMMANDER_MONITORING_INTERVAL) == 0) {
				mavlink_log_critical(&_mavlink_log_pub, "Flight termination active");
			}
		}

		// Manual control input handling
		_manual_control.setRCAllowed(!_status_flags.rc_input_blocked);

		if (_manual_control.update()) {

			/* handle the case where RC signal was regained */
			if (!_status_flags.rc_signal_found_once) {
				_status_flags.rc_signal_found_once = true;
				set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_RCRECEIVER, true, true, _status_flags.rc_calibration_valid, _status);
				_status_changed = true;

			} else {
				if (_status.rc_signal_lost) {
					if (_rc_signal_lost_timestamp > 0) {
						mavlink_log_info(&_mavlink_log_pub, "Manual control regained after %.1fs",
								 hrt_elapsed_time(&_rc_signal_lost_timestamp) * 1e-6);
					}

					set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_RCRECEIVER, true, true, _status_flags.rc_calibration_valid, _status);
					_status_changed = true;
				}
			}

			_status.rc_signal_lost = false;

			const bool rc_arming_enabled = (_status.rc_input_mode != vehicle_status_s::RC_IN_MODE_OFF);

			if (rc_arming_enabled) {
				if (_manual_control.wantsDisarm(_vehicle_control_mode, _status, _manual_control_switches, _land_detector.landed)) {
					disarm(arm_disarm_reason_t::RC_STICK);
				}

				if (_manual_control.wantsArm(_vehicle_control_mode, _status, _manual_control_switches, _land_detector.landed)) {
					if (_vehicle_control_mode.flag_control_manual_enabled) {
						arm(arm_disarm_reason_t::RC_STICK);

					} else {
						mavlink_log_critical(&_mavlink_log_pub, "Not arming! Switch to a manual mode first");
						tune_negative(true);
					}
				}
			}

			// abort autonomous mode and switch to position mode if sticks are moved significantly
			if ((_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING)
			    && !in_low_battery_failsafe && !_geofence_warning_action_on
			    && _manual_control.wantsOverride(_vehicle_control_mode)) {
				if (main_state_transition(_status, commander_state_s::MAIN_STATE_POSCTL, _status_flags,
							  _internal_state) == TRANSITION_CHANGED) {
					tune_positive(true);
					mavlink_log_info(&_mavlink_log_pub, "Pilot took over control using sticks");
					_status_changed = true;
				}
			}

			if (!_armed.armed && _manual_control.isMavlink() && (_internal_state.main_state_changes == 0)) {
				// if there's never been a mode change force position control as initial state
				_internal_state.main_state = commander_state_s::MAIN_STATE_POSCTL;
			}

			if (_manual_control_switches_sub.update(&_manual_control_switches) || safety_updated) {

				// handle landing gear switch if configured and in a manual mode
				if ((_vehicle_control_mode.flag_control_manual_enabled) &&
				    (_manual_control_switches.gear_switch != manual_control_switches_s::SWITCH_POS_NONE) &&
				    (_last_manual_control_switches.gear_switch != manual_control_switches_s::SWITCH_POS_NONE) &&
				    (_manual_control_switches.gear_switch != _last_manual_control_switches.gear_switch)) {
					// Only switch the landing gear up if the user switched from gear down to gear up.
					int8_t gear = landing_gear_s::GEAR_KEEP;

					if (_manual_control_switches.gear_switch == manual_control_switches_s::SWITCH_POS_OFF) {
						gear = landing_gear_s::GEAR_DOWN;

					} else if (_manual_control_switches.gear_switch == manual_control_switches_s::SWITCH_POS_ON) {
						// gear up ignored unless flying
						if (!_land_detector.landed && !_land_detector.maybe_landed) {
							gear = landing_gear_s::GEAR_UP;

						} else {
							mavlink_log_critical(&_mavlink_log_pub, "Landed, unable to retract landing gear")
						}
					}

					if (gear != landing_gear_s::GEAR_KEEP) {
						landing_gear_s landing_gear{};
						landing_gear.landing_gear = gear;
						landing_gear.timestamp = hrt_absolute_time();
						_landing_gear_pub.publish(landing_gear);
					}
				}

				// evaluate the main state machine according to mode switches
				if (set_main_state(_status_changed) == TRANSITION_CHANGED) {
					// play tune on mode change only if armed, blink LED always
					tune_positive(_armed.armed);
					_status_changed = true;
				}
			}

			/* check throttle kill switch */
			if (_manual_control_switches.kill_switch == manual_control_switches_s::SWITCH_POS_ON) {
				/* set lockdown flag */
				if (!_armed.manual_lockdown) {
					const char kill_switch_string[] = "Kill-switch engaged";

					if (_land_detector.landed) {
						mavlink_log_info(&_mavlink_log_pub, kill_switch_string);

					} else {
						mavlink_log_critical(&_mavlink_log_pub, kill_switch_string);
					}

					_status_changed = true;
					_armed.manual_lockdown = true;
				}

			} else if (_manual_control_switches.kill_switch == manual_control_switches_s::SWITCH_POS_OFF) {
				if (_armed.manual_lockdown) {
					mavlink_log_info(&_mavlink_log_pub, "Kill-switch disengaged");
					_status_changed = true;
					_armed.manual_lockdown = false;
				}
			}

			/* no else case: do not change lockdown flag in unconfigured case */
		}

		if (!_manual_control.isRCAvailable()) {
			// set RC lost
			if (_status_flags.rc_signal_found_once && !_status.rc_signal_lost) {
				// ignore RC lost during calibration
				if (!_status_flags.condition_calibration_enabled && !_status_flags.rc_input_blocked) {
					mavlink_log_critical(&_mavlink_log_pub, "Manual control lost");
					_status.rc_signal_lost = true;
					_rc_signal_lost_timestamp = _manual_control.getLastRcTimestamp();
					set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_RCRECEIVER, true, true, false, _status);
					_status_changed = true;
				}
			}
		}

		// data link checks which update the status
		data_link_check();

		avoidance_check();

		// engine failure detection
		// TODO: move out of commander
		if (_actuator_controls_sub.updated()) {
			/* Check engine failure
			 * only for fixed wing for now
			 */
			if (!_status_flags.circuit_breaker_engaged_enginefailure_check &&
			    _status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING && !_status.is_vtol && _armed.armed) {

				actuator_controls_s actuator_controls{};
				_actuator_controls_sub.copy(&actuator_controls);

				const float throttle = actuator_controls.control[actuator_controls_s::INDEX_THROTTLE];
				const float current2throttle = _battery_current / throttle;

				if (((throttle > _param_ef_throttle_thres.get()) && (current2throttle < _param_ef_current2throttle_thres.get()))
				    || _status.engine_failure) {

					const float elapsed = hrt_elapsed_time(&_timestamp_engine_healthy) / 1e6f;

					/* potential failure, measure time */
					if ((_timestamp_engine_healthy > 0) && (elapsed > _param_ef_time_thres.get())
					    && !_status.engine_failure) {

						_status.engine_failure = true;
						_status_changed = true;

						PX4_ERR("Engine Failure");
						set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_MOTORCONTROL, true, true, false, _status);
					}
				}

			} else {
				/* no failure reset flag */
				_timestamp_engine_healthy = hrt_absolute_time();

				if (_status.engine_failure) {
					_status.engine_failure = false;
					_status_changed = true;
				}
			}
		}

		/* check if we are disarmed and there is a better mode to wait in */
		if (!_armed.armed) {
			/* if there is no radio control but GPS lock the user might want to fly using
			 * just a tablet. Since the RC will force its mode switch setting on connecting
			 * we can as well just wait in a hold mode which enables tablet control.
			 */
			if (_status.rc_signal_lost && (_internal_state.main_state == commander_state_s::MAIN_STATE_MANUAL)
			    && _status_flags.condition_global_position_valid) {

				main_state_transition(_status, commander_state_s::MAIN_STATE_AUTO_LOITER, _status_flags, _internal_state);
			}
		}

		/* handle commands last, as the system needs to be updated to handle them */
		while (_cmd_sub.updated()) {
			/* got command */
			const unsigned last_generation = _cmd_sub.get_last_generation();
			vehicle_command_s cmd;

			if (_cmd_sub.copy(&cmd)) {
				if (_cmd_sub.get_last_generation() != last_generation + 1) {
					PX4_ERR("vehicle_command lost, generation %d -> %d", last_generation, _cmd_sub.get_last_generation());
				}

				if (handle_command(cmd)) {
					_status_changed = true;
				}
			}
		}

		/* Check for failure detector status */
		if (_failure_detector.update(_status, _vehicle_control_mode)) {
			_status.failure_detector_status = _failure_detector.getStatus();
			_status_changed = true;

			if (_armed.armed) {
				if (_status.failure_detector_status & vehicle_status_s::FAILURE_ARM_ESC) {
					// 500ms is the PWM spoolup time. Within this timeframe controllers are not affecting actuator_outputs
					if (hrt_elapsed_time(&_status.armed_time) < 500_ms) {
						disarm(arm_disarm_reason_t::FAILURE_DETECTOR);
						mavlink_log_critical(&_mavlink_log_pub, "ESCs did not respond to arm request");
					}
				}

				if (_status.failure_detector_status & (vehicle_status_s::FAILURE_ROLL | vehicle_status_s::FAILURE_PITCH |
								       vehicle_status_s::FAILURE_ALT | vehicle_status_s::FAILURE_EXT)) {
					const bool is_second_after_takeoff = hrt_elapsed_time(&_status.takeoff_time) < (1_s * _param_com_lkdown_tko.get());

					if (is_second_after_takeoff && !_lockdown_triggered) {
						// This handles the case where something fails during the early takeoff phase
						_armed.lockdown = true;
						_lockdown_triggered = true;
						mavlink_log_emergency(&_mavlink_log_pub, "Critical failure detected: lockdown");

					} else if (!_status_flags.circuit_breaker_flight_termination_disabled &&
						   !_flight_termination_triggered && !_lockdown_triggered) {

						_armed.force_failsafe = true;
						_flight_termination_triggered = true;
						mavlink_log_emergency(&_mavlink_log_pub, "Critical failure detected: terminate flight");
						set_tune_override(tune_control_s::TUNE_ID_PARACHUTE_RELEASE);
					}
				}
			}
		}

		/* Get current timestamp */
		const hrt_abstime now = hrt_absolute_time();

		// automatically set or update home position
		if (!_home_pub.get().manual_home) {
			const vehicle_local_position_s &local_position = _local_position_sub.get();

			if (!_armed.armed) {
				if (_home_pub.get().valid_lpos) {
					if (_land_detector.landed && local_position.xy_valid && local_position.z_valid) {
						/* distance from home */
						float home_dist_xy = -1.0f;
						float home_dist_z = -1.0f;
						mavlink_wpm_distance_to_point_local(_home_pub.get().x, _home_pub.get().y, _home_pub.get().z,
										    local_position.x, local_position.y, local_position.z,
										    &home_dist_xy, &home_dist_z);

						if ((home_dist_xy > local_position.eph * 2.0f) || (home_dist_z > local_position.epv * 2.0f)) {

							/* update when disarmed, landed and moved away from current home position */
							set_home_position();
						}
					}

				} else {
					/* First time home position update - but only if disarmed */
					set_home_position();

					/* Set home position altitude to EKF origin height if home is not set and the EKF has a global origin.
					 * This allows home altitude to be used in the calculation of height above takeoff location when GPS
					 * use has commenced after takeoff. */
					if (!_status_flags.condition_home_position_valid) {
						set_home_position_alt_only();
					}
				}
			}
		}

		// check for arming state change
		if (_was_armed != _armed.armed) {
			_status_changed = true;

			if (_armed.armed) {
				if (!_land_detector.landed) { // check if takeoff already detected upon arming
					_have_taken_off_since_arming = true;
				}

			} else { // increase the flight uuid upon disarming
				const int32_t flight_uuid = _param_flight_uuid.get() + 1;
				_param_flight_uuid.set(flight_uuid);
				_param_flight_uuid.commit_no_notification();

				_last_disarmed_timestamp = hrt_absolute_time();

				_should_set_home_on_takeoff = true;
			}
		}

		if (!_armed.armed) {
			/* Reset the flag if disarmed. */
			_have_taken_off_since_arming = false;
		}

		/* now set navigation state according to failsafe and main state */
		bool nav_state_changed = set_nav_state(_status,
						       _armed,
						       _internal_state,
						       &_mavlink_log_pub,
						       (link_loss_actions_t)_param_nav_dll_act.get(),
						       _mission_result_sub.get().finished,
						       _mission_result_sub.get().stay_in_failsafe,
						       _status_flags,
						       _land_detector.landed,
						       (link_loss_actions_t)_param_nav_rcl_act.get(),
						       (offboard_loss_actions_t)_param_com_obl_act.get(),
						       (offboard_loss_rc_actions_t)_param_com_obl_rc_act.get(),
						       (position_nav_loss_actions_t)_param_com_posctl_navl.get(),
						       _param_com_rcl_act_t.get());

		if (nav_state_changed) {
			_status.nav_state_timestamp = hrt_absolute_time();
		}

		if (_status.failsafe != _failsafe_old) {
			_status_changed = true;

			if (_status.failsafe) {
				mavlink_log_info(&_mavlink_log_pub, "Failsafe mode activated");

			} else {
				mavlink_log_info(&_mavlink_log_pub, "Failsafe mode deactivated");
			}

			_failsafe_old = _status.failsafe;
		}

		/* publish states (armed, control_mode, vehicle_status, commander_state, vehicle_status_flags) at 2 Hz or immediately when changed */
		if (hrt_elapsed_time(&_status.timestamp) >= 500_ms || _status_changed || nav_state_changed) {

			update_control_mode();

			_status.timestamp = hrt_absolute_time();
			_status_pub.publish(_status);

			switch ((PrearmedMode)_param_com_prearm_mode.get()) {
			case PrearmedMode::DISABLED:
				/* skip prearmed state  */
				_armed.prearmed = false;
				break;

			case PrearmedMode::ALWAYS:
				/* safety is not present, go into prearmed
				* (all output drivers should be started / unlocked last in the boot process
				* when the rest of the system is fully initialized)
				*/
				_armed.prearmed = (hrt_elapsed_time(&_boot_timestamp) > 5_s);
				break;

			case PrearmedMode::SAFETY_BUTTON:
				if (_safety.safety_switch_available) {
					/* safety switch is present, go into prearmed if safety is off */
					_armed.prearmed = _safety.safety_off;

				} else {
					/* safety switch is not present, do not go into prearmed */
					_armed.prearmed = false;
				}

				break;

			default:
				_armed.prearmed = false;
				break;
			}

			_armed.timestamp = hrt_absolute_time();
			_armed_pub.publish(_armed);

			/* publish internal state for logging purposes */
			_internal_state.timestamp = hrt_absolute_time();
			_commander_state_pub.publish(_internal_state);

			// Evaluate current prearm status
			if (!_armed.armed && !_status_flags.condition_calibration_enabled) {
				bool preflight_check_res = PreFlightCheck::preflightCheck(nullptr, _status, _status_flags, false, true,
							   hrt_elapsed_time(&_boot_timestamp));

				// skip arm authorization check until actual arming attempt
				PreFlightCheck::arm_requirements_t arm_req = _arm_requirements;
				arm_req.arm_authorization = false;
				bool prearm_check_res = PreFlightCheck::preArmCheck(nullptr, _status_flags, _safety, arm_req, _status, false);

				set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_PREARM_CHECK, true, true, (preflight_check_res
						 && prearm_check_res), _status);
			}

			/* publish vehicle_status_flags */
			_status_flags.timestamp = hrt_absolute_time();
			_vehicle_status_flags_pub.publish(_status_flags);
		}

		/* play arming and battery warning tunes */
		if (!_arm_tune_played && _armed.armed &&
		    (_safety.safety_switch_available || (_safety.safety_switch_available && _safety.safety_off))) {

			/* play tune when armed */
			set_tune(tune_control_s::TUNE_ID_ARMING_WARNING);
			_arm_tune_played = true;

		} else if (!_status_flags.usb_connected &&
			   (_status.hil_state != vehicle_status_s::HIL_STATE_ON) &&
			   (_battery_warning == battery_status_s::BATTERY_WARNING_CRITICAL)) {
			/* play tune on battery critical */
			set_tune(tune_control_s::TUNE_ID_BATTERY_WARNING_FAST);

		} else if ((_status.hil_state != vehicle_status_s::HIL_STATE_ON) &&
			   (_battery_warning == battery_status_s::BATTERY_WARNING_LOW)) {
			/* play tune on battery warning */
			set_tune(tune_control_s::TUNE_ID_BATTERY_WARNING_SLOW);

		} else if (_status.failsafe) {
			tune_failsafe(true);

		} else {
			set_tune(tune_control_s::TUNE_ID_STOP);
		}

		/* reset arm_tune_played when disarmed */
		if (!_armed.armed || (_safety.safety_switch_available && !_safety.safety_off)) {

			// Notify the user that it is safe to approach the vehicle
			if (_arm_tune_played) {
				tune_neutral(true);
			}

			_arm_tune_played = false;
		}

		/* play sensor failure tunes if we already waited for hotplug sensors to come up and failed */
		_status_flags.condition_system_hotplug_timeout = (hrt_elapsed_time(&_boot_timestamp) > HOTPLUG_SENS_TIMEOUT);

		if (!sensor_fail_tune_played && (!_status_flags.condition_system_sensors_initialized
						 && _status_flags.condition_system_hotplug_timeout)) {

			set_tune_override(tune_control_s::TUNE_ID_GPS_WARNING);
			sensor_fail_tune_played = true;
			_status_changed = true;
		}

		_counter++;

		int blink_state = blink_msg_state();

		if (blink_state > 0) {
			/* blinking LED message, don't touch LEDs */
			if (blink_state == 2) {
				/* blinking LED message completed, restore normal state */
				control_status_leds(true, _battery_warning);
			}

		} else {
			/* normal state */
			control_status_leds(_status_changed, _battery_warning);
		}

		// check if the worker has finished
		if (_worker_thread.hasResult()) {
			int ret = _worker_thread.getResultAndReset();
			_armed.in_esc_calibration_mode = false;

			if (_status_flags.condition_calibration_enabled) { // did we do a calibration?
				_status_flags.condition_calibration_enabled = false;

				if (ret == 0) {
					tune_positive(true);

				} else {
					tune_negative(true);
				}
			}
		}

		_status_changed = false;

		/* store last position lock state */
		_last_condition_local_altitude_valid = _status_flags.condition_local_altitude_valid;
		_last_condition_local_position_valid = _status_flags.condition_local_position_valid;
		_last_condition_global_position_valid = _status_flags.condition_global_position_valid;

		_was_armed = _armed.armed;

		arm_auth_update(now, params_updated || param_init_forced);

		px4_indicate_external_reset_lockout(LockoutComponent::Commander, _armed.armed);

		px4_usleep(COMMANDER_MONITORING_INTERVAL);
	}

	rgbled_set_color_and_mode(led_control_s::COLOR_WHITE, led_control_s::MODE_OFF);

	/* close fds */
	led_deinit();
	buzzer_deinit();
}

void
Commander::get_circuit_breaker_params()
{
	_status_flags.circuit_breaker_engaged_power_check = circuit_breaker_enabled_by_val(_param_cbrk_supply_chk.get(),
			CBRK_SUPPLY_CHK_KEY);
	_status_flags.circuit_breaker_engaged_usb_check = circuit_breaker_enabled_by_val(_param_cbrk_usb_chk.get(),
			CBRK_USB_CHK_KEY);
	_status_flags.circuit_breaker_engaged_airspd_check = circuit_breaker_enabled_by_val(_param_cbrk_airspd_chk.get(),
			CBRK_AIRSPD_CHK_KEY);
	_status_flags.circuit_breaker_engaged_enginefailure_check = circuit_breaker_enabled_by_val(_param_cbrk_enginefail.get(),
			CBRK_ENGINEFAIL_KEY);
	_status_flags.circuit_breaker_flight_termination_disabled = circuit_breaker_enabled_by_val(_param_cbrk_flightterm.get(),
			CBRK_FLIGHTTERM_KEY);
	_status_flags.circuit_breaker_engaged_posfailure_check = circuit_breaker_enabled_by_val(_param_cbrk_velposerr.get(),
			CBRK_VELPOSERR_KEY);
	_status_flags.circuit_breaker_vtol_fw_arming_check = circuit_breaker_enabled_by_val(_param_cbrk_vtolarming.get(),
			CBRK_VTOLARMING_KEY);
}

void
Commander::control_status_leds(bool changed, const uint8_t battery_warning)
{
	bool overload = (_cpuload.load > 0.95f) || (_cpuload.ram_usage > 0.98f);

	if (_overload_start == 0 && overload) {
		_overload_start = hrt_absolute_time();

	} else if (!overload) {
		_overload_start = 0;
	}

	// driving the RGB led
	if (changed || _last_overload != overload) {
		uint8_t led_mode = led_control_s::MODE_OFF;
		uint8_t led_color = led_control_s::COLOR_WHITE;
		bool set_normal_color = false;

		uint64_t overload_warn_delay = (_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) ? 1_ms : 250_ms;

		/* set mode */
		if (overload && (hrt_elapsed_time(&_overload_start) > overload_warn_delay)) {
			led_mode = led_control_s::MODE_BLINK_FAST;
			led_color = led_control_s::COLOR_PURPLE;

		} else if (_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
			led_mode = led_control_s::MODE_ON;
			set_normal_color = true;

		} else if (!_status_flags.condition_system_sensors_initialized && _status_flags.condition_system_hotplug_timeout) {
			led_mode = led_control_s::MODE_BLINK_FAST;
			led_color = led_control_s::COLOR_RED;

		} else if (_status.arming_state == vehicle_status_s::ARMING_STATE_STANDBY) {
			led_mode = led_control_s::MODE_BREATHE;
			set_normal_color = true;

		} else if (!_status_flags.condition_system_sensors_initialized && !_status_flags.condition_system_hotplug_timeout) {
			led_mode = led_control_s::MODE_BREATHE;
			set_normal_color = true;

		} else if (_status.arming_state == vehicle_status_s::ARMING_STATE_INIT) {
			// if in init status it should not be in the error state
			led_mode = led_control_s::MODE_OFF;

		} else {	// STANDBY_ERROR and other states
			led_mode = led_control_s::MODE_BLINK_NORMAL;
			led_color = led_control_s::COLOR_RED;
		}

		if (set_normal_color) {
			/* set color */
			if (_status.failsafe) {
				led_color = led_control_s::COLOR_PURPLE;

			} else if (battery_warning == battery_status_s::BATTERY_WARNING_LOW) {
				led_color = led_control_s::COLOR_AMBER;

			} else if (battery_warning == battery_status_s::BATTERY_WARNING_CRITICAL) {
				led_color = led_control_s::COLOR_RED;

			} else {
				if (_status_flags.condition_home_position_valid && _status_flags.condition_global_position_valid) {
					led_color = led_control_s::COLOR_GREEN;

				} else {
					led_color = led_control_s::COLOR_BLUE;
				}
			}
		}

		if (led_mode != led_control_s::MODE_OFF) {
			rgbled_set_color_and_mode(led_color, led_mode);
		}
	}

	_last_overload = overload;

#if !defined(CONFIG_ARCH_LEDS) && defined(BOARD_HAS_CONTROL_STATUS_LEDS)

	/* this runs at around 20Hz, full cycle is 16 ticks = 10/16Hz */
	if (_armed.armed) {
		if (_status.failsafe) {
			BOARD_ARMED_LED_OFF();

			if (_leds_counter % 5 == 0) {
				BOARD_ARMED_STATE_LED_TOGGLE();
			}

		} else {
			BOARD_ARMED_STATE_LED_OFF();

			/* armed, solid */
			BOARD_ARMED_LED_ON();
		}

	} else if (_armed.ready_to_arm) {
		BOARD_ARMED_LED_OFF();

		/* ready to arm, blink at 1Hz */
		if (_leds_counter % 20 == 0) {
			BOARD_ARMED_STATE_LED_TOGGLE();
		}

	} else {
		BOARD_ARMED_LED_OFF();

		/* not ready to arm, blink at 10Hz */
		if (_leds_counter % 2 == 0) {
			BOARD_ARMED_STATE_LED_TOGGLE();
		}
	}

#endif

	/* give system warnings on error LED */
	if (overload) {
		if (_leds_counter % 2 == 0) {
			BOARD_OVERLOAD_LED_TOGGLE();
		}

	} else {
		BOARD_OVERLOAD_LED_OFF();
	}

	_leds_counter++;
}

transition_result_t
Commander::set_main_state(bool &changed)
{
	if (_safety.override_available && _safety.override_enabled) {
		return set_main_state_override_on(changed);

	} else {
		return set_main_state_rc();
	}
}

transition_result_t
Commander::set_main_state_override_on(bool &changed)
{
	const transition_result_t res = main_state_transition(_status, commander_state_s::MAIN_STATE_MANUAL, _status_flags,
					_internal_state);
	changed = (res == TRANSITION_CHANGED);

	return res;
}

transition_result_t
Commander::set_main_state_rc()
{
	if ((_manual_control_switches.timestamp == 0)
	    || (_manual_control_switches.timestamp == _last_manual_control_switches.timestamp)) {

		// no manual control or no update -> nothing changed
		return TRANSITION_NOT_CHANGED;
	}

	// Note: even if _status_flags.offboard_control_set_by_command is set
	// we want to allow rc mode change to take precedence.  This is a safety
	// feature, just in case offboard control goes crazy.

	// only switch mode based on RC switch if necessary to also allow mode switching via MAVLink
	bool should_evaluate_rc_mode_switch =
		(_last_manual_control_switches.offboard_switch != _manual_control_switches.offboard_switch)
		|| (_last_manual_control_switches.return_switch != _manual_control_switches.return_switch)
		|| (_last_manual_control_switches.mode_switch != _manual_control_switches.mode_switch)
		|| (_last_manual_control_switches.acro_switch != _manual_control_switches.acro_switch)
		|| (_last_manual_control_switches.posctl_switch != _manual_control_switches.posctl_switch)
		|| (_last_manual_control_switches.loiter_switch != _manual_control_switches.loiter_switch)
		|| (_last_manual_control_switches.mode_slot != _manual_control_switches.mode_slot)
		|| (_last_manual_control_switches.stab_switch != _manual_control_switches.stab_switch)
		|| (_last_manual_control_switches.man_switch != _manual_control_switches.man_switch);


	if (_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		// if already armed don't evaluate first time RC
		if (_last_manual_control_switches.timestamp == 0) {
			should_evaluate_rc_mode_switch = false;
			_last_manual_control_switches = _manual_control_switches;
		}

	} else {
		// not armed
		if (!should_evaluate_rc_mode_switch) {
			// to respect initial switch position (eg POSCTL) force RC switch re-evaluation if estimates become valid
			const bool altitude_got_valid = (!_last_condition_local_altitude_valid && _status_flags.condition_local_altitude_valid);
			const bool lpos_got_valid = (!_last_condition_local_position_valid && _status_flags.condition_local_position_valid);
			const bool gpos_got_valid = (!_last_condition_global_position_valid && _status_flags.condition_global_position_valid);

			if (altitude_got_valid || lpos_got_valid || gpos_got_valid) {
				should_evaluate_rc_mode_switch = true;
			}
		}
	}

	if (!should_evaluate_rc_mode_switch) {
		/* no timestamp change or no switch change -> nothing changed */
		return TRANSITION_NOT_CHANGED;
	}

	_last_manual_control_switches = _manual_control_switches;

	// reset the position and velocity validity calculation to give the best change of being able to select
	// the desired mode
	reset_posvel_validity();

	/* set main state according to RC switches */
	transition_result_t res = TRANSITION_NOT_CHANGED;

	/* offboard switch overrides main switch */
	if (_manual_control_switches.offboard_switch == manual_control_switches_s::SWITCH_POS_ON) {
		res = main_state_transition(_status, commander_state_s::MAIN_STATE_OFFBOARD, _status_flags, _internal_state);

		if (res == TRANSITION_DENIED) {
			print_reject_mode(commander_state_s::MAIN_STATE_OFFBOARD);
			/* mode rejected, continue to evaluate the main system mode */

		} else {
			/* changed successfully or already in this state */
			return res;
		}
	}

	/* RTL switch overrides main switch */
	if (_manual_control_switches.return_switch == manual_control_switches_s::SWITCH_POS_ON) {
		res = try_mode_change(commander_state_s::MAIN_STATE_AUTO_RTL);

		if (res != TRANSITION_DENIED) {
			/* changed successfully or already in this state */
			return res;
		}

		/* if we get here mode was rejected, continue to evaluate the main system mode */
	}

	/* Loiter switch overrides main switch */
	if (_manual_control_switches.loiter_switch == manual_control_switches_s::SWITCH_POS_ON) {
		res = main_state_transition(_status, commander_state_s::MAIN_STATE_AUTO_LOITER, _status_flags, _internal_state);

		if (res == TRANSITION_DENIED) {
			print_reject_mode(commander_state_s::MAIN_STATE_AUTO_LOITER);
			/* mode rejected, continue to evaluate the main system mode */

		} else {
			/* changed successfully or already in this state */
			return res;
		}
	}

	/* we know something has changed - check if we are in mode slot operation */
	if (_manual_control_switches.mode_slot != manual_control_switches_s::MODE_SLOT_NONE) {

		if (_manual_control_switches.mode_slot > manual_control_switches_s::MODE_SLOT_NUM) {
			PX4_WARN("m slot overflow");
			return TRANSITION_DENIED;
		}

		int new_mode = _flight_mode_slots[_manual_control_switches.mode_slot - 1];

		if (new_mode < 0) {
			/* slot is unused */
			res = TRANSITION_NOT_CHANGED;

		} else {
			res = try_mode_change(new_mode);
		}

		return res;

	} else if (_manual_control_switches.mode_switch != manual_control_switches_s::SWITCH_POS_NONE) {
		/* offboard and RTL switches off or denied, check main mode switch */
		switch (_manual_control_switches.mode_switch) {
		case manual_control_switches_s::SWITCH_POS_NONE:
			res = TRANSITION_NOT_CHANGED;
			break;

		case manual_control_switches_s::SWITCH_POS_OFF:		// MANUAL
			if (_manual_control_switches.stab_switch == manual_control_switches_s::SWITCH_POS_NONE &&
			    _manual_control_switches.man_switch == manual_control_switches_s::SWITCH_POS_NONE) {
				/*
				 * Legacy mode:
				 * Acro switch being used as stabilized switch in FW.
				 */
				if (_manual_control_switches.acro_switch == manual_control_switches_s::SWITCH_POS_ON) {
					/* manual mode is stabilized already for multirotors, so switch to acro
					 * for any non-manual mode
					 */
					if (_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && !_status.is_vtol) {
						res = main_state_transition(_status, commander_state_s::MAIN_STATE_ACRO, _status_flags, _internal_state);

					} else if (_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
						res = main_state_transition(_status, commander_state_s::MAIN_STATE_STAB, _status_flags, _internal_state);

					} else {
						res = main_state_transition(_status, commander_state_s::MAIN_STATE_MANUAL, _status_flags, _internal_state);
					}

				} else {
					res = main_state_transition(_status, commander_state_s::MAIN_STATE_MANUAL, _status_flags, _internal_state);
				}

			} else {
				/* New mode:
				 * - Acro is Acro
				 * - Manual is not default anymore when the manual switch is assigned
				 */
				if (_manual_control_switches.man_switch == manual_control_switches_s::SWITCH_POS_ON) {
					res = main_state_transition(_status, commander_state_s::MAIN_STATE_MANUAL, _status_flags, _internal_state);

				} else if (_manual_control_switches.acro_switch == manual_control_switches_s::SWITCH_POS_ON) {
					res = main_state_transition(_status, commander_state_s::MAIN_STATE_ACRO, _status_flags, _internal_state);

				} else if (_manual_control_switches.stab_switch == manual_control_switches_s::SWITCH_POS_ON) {
					res = main_state_transition(_status, commander_state_s::MAIN_STATE_STAB, _status_flags, _internal_state);

				} else if (_manual_control_switches.man_switch == manual_control_switches_s::SWITCH_POS_NONE) {
					// default to MANUAL when no manual switch is set
					res = main_state_transition(_status, commander_state_s::MAIN_STATE_MANUAL, _status_flags, _internal_state);

				} else {
					// default to STAB when the manual switch is assigned (but off)
					res = main_state_transition(_status, commander_state_s::MAIN_STATE_STAB, _status_flags, _internal_state);
				}
			}

			// TRANSITION_DENIED is not possible here
			break;

		case manual_control_switches_s::SWITCH_POS_MIDDLE:		// ASSIST
			if (_manual_control_switches.posctl_switch == manual_control_switches_s::SWITCH_POS_ON) {
				res = try_mode_change(commander_state_s::MAIN_STATE_POSCTL);

			} else {
				res = try_mode_change(commander_state_s::MAIN_STATE_ALTCTL);
			}

			break;

		case manual_control_switches_s::SWITCH_POS_ON:		// AUTO
			res = try_mode_change(commander_state_s::MAIN_STATE_AUTO_MISSION);
			break;

		default:
			break;
		}

	}

	return res;
}

void
Commander::reset_posvel_validity()
{
	// reset all the check probation times back to the minimum value
	_gpos_probation_time_us = POSVEL_PROBATION_MIN;
	_lpos_probation_time_us = POSVEL_PROBATION_MIN;
	_lvel_probation_time_us = POSVEL_PROBATION_MIN;

	// recheck validity
	UpdateEstimateValidity();
}

bool
Commander::check_posvel_validity(const bool data_valid, const float data_accuracy, const float required_accuracy,
				 const hrt_abstime &data_timestamp_us, hrt_abstime *last_fail_time_us, hrt_abstime *probation_time_us,
				 const bool was_valid)
{
	bool valid = was_valid;

	// constrain probation times
	if (_land_detector.landed) {
		*probation_time_us = POSVEL_PROBATION_MIN;
	}

	const bool data_stale = ((hrt_elapsed_time(&data_timestamp_us) > _param_com_pos_fs_delay.get() * 1_s)
				 || (data_timestamp_us == 0));
	const float req_accuracy = (was_valid ? required_accuracy * 2.5f : required_accuracy);

	const bool level_check_pass = data_valid && !data_stale && (data_accuracy < req_accuracy);

	// Check accuracy with hysteresis in both test level and time
	if (level_check_pass) {
		if (was_valid) {
			// still valid, continue to decrease probation time
			const int64_t probation_time_new = *probation_time_us - hrt_elapsed_time(last_fail_time_us);
			*probation_time_us = math::constrain(probation_time_new, POSVEL_PROBATION_MIN, POSVEL_PROBATION_MAX);

		} else {
			// check if probation period has elapsed
			if (hrt_elapsed_time(last_fail_time_us) > *probation_time_us) {
				valid = true;
			}
		}

	} else {
		// level check failed
		if (was_valid) {
			// FAILURE! no longer valid
			valid = false;

		} else {
			// failed again, increase probation time
			const int64_t probation_time_new = *probation_time_us + hrt_elapsed_time(last_fail_time_us) *
							   _param_com_pos_fs_gain.get();
			*probation_time_us = math::constrain(probation_time_new, POSVEL_PROBATION_MIN, POSVEL_PROBATION_MAX);
		}

		*last_fail_time_us = hrt_absolute_time();
	}

	if (was_valid != valid) {
		_status_changed = true;
	}

	return valid;
}

void
Commander::update_control_mode()
{
	_vehicle_control_mode = {};

	/* set vehicle_control_mode according to set_navigation_state */
	_vehicle_control_mode.flag_armed = _armed.armed;

	_vehicle_control_mode.flag_external_manual_override_ok =
		(_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING && !_status.is_vtol);

	switch (_status.nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		_vehicle_control_mode.flag_control_manual_enabled = true;
		_vehicle_control_mode.flag_control_rates_enabled = stabilization_required();
		_vehicle_control_mode.flag_control_attitude_enabled = stabilization_required();
		break;

	case vehicle_status_s::NAVIGATION_STATE_STAB:
		_vehicle_control_mode.flag_control_manual_enabled = true;
		_vehicle_control_mode.flag_control_rates_enabled = true;
		_vehicle_control_mode.flag_control_attitude_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
		_vehicle_control_mode.flag_control_manual_enabled = true;
		_vehicle_control_mode.flag_control_rates_enabled = true;
		_vehicle_control_mode.flag_control_attitude_enabled = true;
		_vehicle_control_mode.flag_control_altitude_enabled = true;
		_vehicle_control_mode.flag_control_climb_rate_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_POSCTL:
		_vehicle_control_mode.flag_control_manual_enabled = true;
		_vehicle_control_mode.flag_control_rates_enabled = true;
		_vehicle_control_mode.flag_control_attitude_enabled = true;
		_vehicle_control_mode.flag_control_altitude_enabled = true;
		_vehicle_control_mode.flag_control_climb_rate_enabled = true;
		_vehicle_control_mode.flag_control_position_enabled = !_status.in_transition_mode;
		_vehicle_control_mode.flag_control_velocity_enabled = !_status.in_transition_mode;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
		/* override is not ok for the RTL and recovery mode */
		_vehicle_control_mode.flag_external_manual_override_ok = false;

	/* fallthrough */
	case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
		_vehicle_control_mode.flag_control_auto_enabled = true;
		_vehicle_control_mode.flag_control_rates_enabled = true;
		_vehicle_control_mode.flag_control_attitude_enabled = true;
		_vehicle_control_mode.flag_control_altitude_enabled = true;
		_vehicle_control_mode.flag_control_climb_rate_enabled = true;
		_vehicle_control_mode.flag_control_position_enabled = !_status.in_transition_mode;
		_vehicle_control_mode.flag_control_velocity_enabled = !_status.in_transition_mode;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDGPSFAIL:
		_vehicle_control_mode.flag_control_rates_enabled = true;
		_vehicle_control_mode.flag_control_attitude_enabled = true;
		_vehicle_control_mode.flag_control_climb_rate_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ACRO:
		_vehicle_control_mode.flag_control_manual_enabled = true;
		_vehicle_control_mode.flag_control_rates_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_DESCEND:
		_vehicle_control_mode.flag_control_auto_enabled = false;
		_vehicle_control_mode.flag_control_rates_enabled = true;
		_vehicle_control_mode.flag_control_attitude_enabled = true;
		_vehicle_control_mode.flag_control_climb_rate_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
		/* disable all controllers on termination */
		_vehicle_control_mode.flag_control_termination_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
		_vehicle_control_mode.flag_control_offboard_enabled = true;

		if (_offboard_control_mode_sub.get().position) {
			_vehicle_control_mode.flag_control_position_enabled = true;
			_vehicle_control_mode.flag_control_velocity_enabled = true;
			_vehicle_control_mode.flag_control_altitude_enabled = true;
			_vehicle_control_mode.flag_control_climb_rate_enabled = true;
			_vehicle_control_mode.flag_control_acceleration_enabled = true;
			_vehicle_control_mode.flag_control_rates_enabled = true;
			_vehicle_control_mode.flag_control_attitude_enabled = true;

		} else if (_offboard_control_mode_sub.get().velocity) {
			_vehicle_control_mode.flag_control_velocity_enabled = true;
			_vehicle_control_mode.flag_control_altitude_enabled = true;
			_vehicle_control_mode.flag_control_climb_rate_enabled = true;
			_vehicle_control_mode.flag_control_acceleration_enabled = true;
			_vehicle_control_mode.flag_control_rates_enabled = true;
			_vehicle_control_mode.flag_control_attitude_enabled = true;

		} else if (_offboard_control_mode_sub.get().acceleration) {
			_vehicle_control_mode.flag_control_acceleration_enabled = true;
			_vehicle_control_mode.flag_control_rates_enabled = true;
			_vehicle_control_mode.flag_control_attitude_enabled = true;

		} else if (_offboard_control_mode_sub.get().attitude) {
			_vehicle_control_mode.flag_control_rates_enabled = true;
			_vehicle_control_mode.flag_control_attitude_enabled = true;

		} else if (_offboard_control_mode_sub.get().body_rate) {
			_vehicle_control_mode.flag_control_rates_enabled = true;
		}

		break;

	case vehicle_status_s::NAVIGATION_STATE_ORBIT:
		_vehicle_control_mode.flag_control_manual_enabled = false;
		_vehicle_control_mode.flag_control_auto_enabled = false;
		_vehicle_control_mode.flag_control_rates_enabled = true;
		_vehicle_control_mode.flag_control_attitude_enabled = true;
		_vehicle_control_mode.flag_control_altitude_enabled = true;
		_vehicle_control_mode.flag_control_climb_rate_enabled = true;
		_vehicle_control_mode.flag_control_position_enabled = !_status.in_transition_mode;
		_vehicle_control_mode.flag_control_velocity_enabled = !_status.in_transition_mode;
		break;

	default:
		break;
	}

	_vehicle_control_mode.timestamp = hrt_absolute_time();
	_control_mode_pub.publish(_vehicle_control_mode);
}

bool
Commander::stabilization_required()
{
	return (_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING ||		// is a rotary wing, or
		_status.vtol_fw_permanent_stab || 	// is a VTOL in fixed wing mode and stabilisation is on, or
		(_vtol_status.vtol_in_trans_mode && 	// is currently a VTOL transitioning AND
		 _status.vehicle_type ==
		 vehicle_status_s::VEHICLE_TYPE_FIXED_WING));	// is a fixed wing, ie: transitioning back to rotary wing mode
}

void
Commander::print_reject_mode(uint8_t main_state)
{
	if (hrt_elapsed_time(&_last_print_mode_reject_time) > 1_s) {

		mavlink_log_critical(&_mavlink_log_pub, "Switching to %s is currently not available.", main_state_str(main_state));

		/* only buzz if armed, because else we're driving people nuts indoors
		they really need to look at the leds as well. */
		tune_negative(_armed.armed);

		_last_print_mode_reject_time = hrt_absolute_time();
	}
}

void Commander::answer_command(const vehicle_command_s &cmd, uint8_t result)
{
	switch (result) {
	case vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED:
		break;

	case vehicle_command_s::VEHICLE_CMD_RESULT_DENIED:
		tune_negative(true);
		break;

	case vehicle_command_s::VEHICLE_CMD_RESULT_FAILED:
		tune_negative(true);
		break;

	case vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
		tune_negative(true);
		break;

	case vehicle_command_s::VEHICLE_CMD_RESULT_UNSUPPORTED:
		tune_negative(true);
		break;

	default:
		break;
	}

	/* publish ACK */
	vehicle_command_ack_s command_ack{};
	command_ack.command = cmd.command;
	command_ack.result = result;
	command_ack.target_system = cmd.source_system;
	command_ack.target_component = cmd.source_component;
	command_ack.timestamp = hrt_absolute_time();
	_command_ack_pub.publish(command_ack);
}

int Commander::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("commander",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT + 40,
				      3250,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	// wait until task is up & running
	if (wait_until_running() < 0) {
		_task_id = -1;
		return -1;
	}

	return 0;
}

Commander *Commander::instantiate(int argc, char *argv[])
{
	Commander *instance = new Commander();

	if (instance) {
		if (argc >= 2 && !strcmp(argv[1], "-h")) {
			instance->enable_hil();
		}
	}

	return instance;
}

void Commander::enable_hil()
{
	_status.hil_state = vehicle_status_s::HIL_STATE_ON;
}

void Commander::mission_init()
{
	/* init mission state, do it here to allow navigator to use stored mission even if mavlink failed to start */
	mission_s mission;

	if (dm_read(DM_KEY_MISSION_STATE, 0, &mission, sizeof(mission_s)) == sizeof(mission_s)) {
		if (mission.dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_0 || mission.dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_1) {
			if (mission.count > 0) {
				PX4_INFO("Mission #%d loaded, %u WPs, curr: %d", mission.dataman_id, mission.count, mission.current_seq);
			}

		} else {
			PX4_ERR("reading mission state failed");

			/* initialize mission state in dataman */
			mission.timestamp = hrt_absolute_time();
			mission.dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
			dm_write(DM_KEY_MISSION_STATE, 0, DM_PERSIST_POWER_ON_RESET, &mission, sizeof(mission_s));
		}

		_mission_pub.publish(mission);
	}
}

void Commander::data_link_check()
{
	for (auto &telemetry_status :  _telemetry_status_subs) {
		telemetry_status_s telemetry;

		if (telemetry_status.update(&telemetry)) {

			// handle different radio types
			switch (telemetry.type) {
			case telemetry_status_s::LINK_TYPE_USB:
				// set (but don't unset) telemetry via USB as active once a MAVLink connection is up
				_status_flags.usb_connected = true;
				break;

			case telemetry_status_s::LINK_TYPE_IRIDIUM: {
					iridiumsbd_status_s iridium_status;

					if (_iridiumsbd_status_sub.update(&iridium_status)) {
						_high_latency_datalink_heartbeat = iridium_status.last_heartbeat;

						if (_status.high_latency_data_link_lost) {
							if (hrt_elapsed_time(&_high_latency_datalink_lost) > (_param_com_hldl_reg_t.get() * 1_s)) {
								_status.high_latency_data_link_lost = false;
								_status_changed = true;
							}
						}

						const bool present = true;
						const bool enabled = true;
						const bool ok = (iridium_status.last_heartbeat > 0); // maybe at some point here an additional check should be made

						set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_SATCOM, present, enabled, ok, _status);
					}

					break;
				}
			}

			if (telemetry.heartbeat_type_gcs) {
				// Initial connection or recovery from data link lost
				if (_status.data_link_lost) {
					_status.data_link_lost = false;
					_status_changed = true;

					if (_datalink_last_heartbeat_gcs != 0) {
						mavlink_log_info(&_mavlink_log_pub, "Data link regained");
					}

					if (!_armed.armed && !_status_flags.condition_calibration_enabled) {
						// make sure to report preflight check failures to a connecting GCS
						PreFlightCheck::preflightCheck(&_mavlink_log_pub, _status, _status_flags, true, true,
									       hrt_elapsed_time(&_boot_timestamp));
					}
				}

				_datalink_last_heartbeat_gcs = telemetry.timestamp;
			}

			if (telemetry.heartbeat_type_onboard_controller) {
				if (_onboard_controller_lost) {
					_onboard_controller_lost = false;
					_status_changed = true;

					if (_datalink_last_heartbeat_onboard_controller != 0) {
						mavlink_log_info(&_mavlink_log_pub, "Onboard controller regained");
					}
				}

				_datalink_last_heartbeat_onboard_controller = telemetry.timestamp;
			}

			if (telemetry.heartbeat_component_obstacle_avoidance) {
				if (_avoidance_system_lost) {
					_avoidance_system_lost = false;
					_status_changed = true;
				}

				_datalink_last_heartbeat_avoidance_system = telemetry.timestamp;
				_status_flags.avoidance_system_valid = telemetry.avoidance_system_healthy;
			}
		}
	}


	// GCS data link loss failsafe
	if (!_status.data_link_lost) {
		if ((_datalink_last_heartbeat_gcs != 0)
		    && hrt_elapsed_time(&_datalink_last_heartbeat_gcs) > (_param_com_dl_loss_t.get() * 1_s)) {

			_status.data_link_lost = true;
			_status.data_link_lost_counter++;

			mavlink_log_info(&_mavlink_log_pub, "Connection to ground station lost");

			_status_changed = true;
		}
	}

	// ONBOARD CONTROLLER data link loss failsafe
	if ((_datalink_last_heartbeat_onboard_controller > 0)
	    && (hrt_elapsed_time(&_datalink_last_heartbeat_onboard_controller) > (_param_com_obc_loss_t.get() * 1_s))
	    && !_onboard_controller_lost) {

		mavlink_log_critical(&_mavlink_log_pub, "Connection to mission computer lost");
		_onboard_controller_lost = true;
		_status_changed = true;
	}

	// AVOIDANCE SYSTEM state check (only if it is enabled)
	if (_status_flags.avoidance_system_required && !_onboard_controller_lost) {
		// if heartbeats stop
		if (!_avoidance_system_lost && (_datalink_last_heartbeat_avoidance_system > 0)
		    && (hrt_elapsed_time(&_datalink_last_heartbeat_avoidance_system) > 5_s)) {

			_avoidance_system_lost = true;
			_status_flags.avoidance_system_valid = false;
		}
	}

	// high latency data link loss failsafe
	if (_high_latency_datalink_heartbeat > 0
	    && hrt_elapsed_time(&_high_latency_datalink_heartbeat) > (_param_com_hldl_loss_t.get() * 1_s)) {
		_high_latency_datalink_lost = hrt_absolute_time();

		if (!_status.high_latency_data_link_lost) {
			_status.high_latency_data_link_lost = true;
			mavlink_log_critical(&_mavlink_log_pub, "High latency data link lost");
			_status_changed = true;
		}
	}
}

void Commander::avoidance_check()
{
	for (auto &dist_sens_sub : _distance_sensor_subs) {
		distance_sensor_s distance_sensor;

		if (dist_sens_sub.update(&distance_sensor)) {
			if ((distance_sensor.orientation != distance_sensor_s::ROTATION_DOWNWARD_FACING) &&
			    (distance_sensor.orientation != distance_sensor_s::ROTATION_UPWARD_FACING)) {

				_valid_distance_sensor_time_us = distance_sensor.timestamp;
			}
		}
	}

	const bool cp_enabled =  _param_cp_dist.get() > 0.f;

	const bool distance_sensor_valid = hrt_elapsed_time(&_valid_distance_sensor_time_us) < 500_ms;
	const bool cp_healthy = _status_flags.avoidance_system_valid || distance_sensor_valid;

	const bool sensor_oa_present = cp_healthy || _status_flags.avoidance_system_required || cp_enabled;

	const bool auto_mode = _vehicle_control_mode.flag_control_auto_enabled;
	const bool pos_ctl_mode = (_vehicle_control_mode.flag_control_manual_enabled
				   && _vehicle_control_mode.flag_control_position_enabled);

	const bool sensor_oa_enabled = ((auto_mode && _status_flags.avoidance_system_required) || (pos_ctl_mode && cp_enabled));
	const bool sensor_oa_healthy = ((auto_mode && _status_flags.avoidance_system_valid) || (pos_ctl_mode && cp_healthy));

	set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_OBSTACLE_AVOIDANCE, sensor_oa_present, sensor_oa_enabled,
			 sensor_oa_healthy, _status);
}

void Commander::battery_status_check()
{
	// We need to update the status flag if ANY battery is updated, because the system source might have
	// changed, or might be nothing (if there is no battery connected)
	if (!_battery_status_subs.updated()) {
		// Nothing has changed since the last time this function was called, so nothing needs to be done now.
		return;
	}

	battery_status_s batteries[_battery_status_subs.size()];
	size_t num_connected_batteries = 0;

	for (auto &battery_sub : _battery_status_subs) {
		if (battery_sub.copy(&batteries[num_connected_batteries])) {
			if (batteries[num_connected_batteries].connected) {
				num_connected_batteries++;
			}
		}
	}

	// There are possibly multiple batteries, and we can't know which ones serve which purpose. So the safest
	// option is to check if ANY of them have a warning, and specifically find which one has the most
	// urgent warning.
	uint8_t worst_warning = battery_status_s::BATTERY_WARNING_NONE;
	// To make sure that all connected batteries are being regularly reported, we check which one has the
	// oldest timestamp.
	hrt_abstime oldest_update = hrt_absolute_time();

	_battery_current = 0.0f;
	float battery_level = 0.0f;


	// Only iterate over connected batteries. We don't care if a disconnected battery is not regularly publishing.
	for (size_t i = 0; i < num_connected_batteries; i++) {
		if (batteries[i].warning > worst_warning) {
			worst_warning = batteries[i].warning;
		}

		if (hrt_elapsed_time(&batteries[i].timestamp) > hrt_elapsed_time(&oldest_update)) {
			oldest_update = batteries[i].timestamp;
		}

		// Sum up current from all batteries.
		_battery_current += batteries[i].current_filtered_a;

		// average levels from all batteries
		battery_level += batteries[i].remaining;
	}

	battery_level /= num_connected_batteries;

	_rtl_flight_time_sub.update();
	float battery_usage_to_home = 0;

	if (hrt_absolute_time() - _rtl_flight_time_sub.get().timestamp < 2_s) {
		battery_usage_to_home = _rtl_flight_time_sub.get().rtl_limit_fraction;
	}

	uint8_t battery_range_warning = battery_status_s::BATTERY_WARNING_NONE;

	if (PX4_ISFINITE(battery_usage_to_home)) {
		float battery_at_home = battery_level - battery_usage_to_home;

		if (battery_at_home < _param_bat_crit_thr.get()) {
			battery_range_warning =  battery_status_s::BATTERY_WARNING_CRITICAL;

		} else if (battery_at_home < _param_bat_low_thr.get()) {
			battery_range_warning = battery_status_s::BATTERY_WARNING_LOW;
		}
	}

	if (battery_range_warning > worst_warning) {
		worst_warning = battery_range_warning;
	}

	bool battery_warning_level_increased_while_armed = false;
	bool update_internal_battery_state = false;

	if (_armed.armed) {
		if (worst_warning > _battery_warning) {
			battery_warning_level_increased_while_armed = true;
			update_internal_battery_state = true;
		}

	} else {
		if (_battery_warning != worst_warning) {
			update_internal_battery_state = true;
		}
	}

	if (update_internal_battery_state) {
		_battery_warning = worst_warning;
	}


	_status_flags.condition_battery_healthy =
		// All connected batteries are regularly being published
		(hrt_elapsed_time(&oldest_update) < 5_s)
		// There is at least one connected battery (in any slot)
		&& (num_connected_batteries > 0)
		// No currently-connected batteries have any warning
		&& (_battery_warning == battery_status_s::BATTERY_WARNING_NONE);

	// execute battery failsafe if the state has gotten worse while we are armed
	if (battery_warning_level_increased_while_armed) {
		battery_failsafe(&_mavlink_log_pub, _status, _status_flags, _internal_state, _battery_warning,
				 (low_battery_action_t)_param_com_low_bat_act.get());
	}

	// Handle shutdown request from emergency battery action
	if (update_internal_battery_state) {

		if (_battery_warning == battery_status_s::BATTERY_WARNING_EMERGENCY) {
#if defined(CONFIG_BOARDCTL_POWEROFF)

			if (shutdown_if_allowed() && (px4_shutdown_request(400_ms) == 0)) {
				mavlink_log_critical(&_mavlink_log_pub, "Dangerously low battery! Shutting system down");

				while (1) { px4_usleep(1); }

			} else {
				mavlink_log_critical(&_mavlink_log_pub, "System does not support shutdown");
			}

#endif // CONFIG_BOARDCTL_POWEROFF
		}
	}
}

void Commander::estimator_check()
{
	// Check if quality checking of position accuracy and consistency is to be performed
	const bool run_quality_checks = !_status_flags.circuit_breaker_engaged_posfailure_check;

	_local_position_sub.update();
	_global_position_sub.update();

	const vehicle_local_position_s &lpos = _local_position_sub.get();

	if (lpos.heading_reset_counter != _heading_reset_counter) {
		if (_status_flags.condition_home_position_valid) {
			updateHomePositionYaw(_home_pub.get().yaw + lpos.delta_heading);
		}

		_heading_reset_counter = lpos.heading_reset_counter;
	}

	const bool mag_fault_prev = (_estimator_status_sub.get().control_mode_flags & (1 << estimator_status_s::CS_MAG_FAULT));

	// use primary estimator_status
	if (_estimator_selector_status_sub.updated()) {
		estimator_selector_status_s estimator_selector_status;

		if (_estimator_selector_status_sub.copy(&estimator_selector_status)) {
			if (estimator_selector_status.primary_instance != _estimator_status_sub.get_instance()) {
				_estimator_status_sub.ChangeInstance(estimator_selector_status.primary_instance);
			}
		}
	}

	if (_estimator_status_sub.update()) {
		const estimator_status_s &estimator_status = _estimator_status_sub.get();

		// Check for a magnetomer fault and notify the user
		const bool mag_fault = (estimator_status.control_mode_flags & (1 << estimator_status_s::CS_MAG_FAULT));

		if (!mag_fault_prev && mag_fault) {
			mavlink_log_critical(&_mavlink_log_pub, "Stopping compass use! Check calibration on landing");
			set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_MAG, true, true, false, _status);
		}

		/* Check estimator status for signs of bad yaw induced post takeoff navigation failure
		 * for a short time interval after takeoff.
		 * Most of the time, the drone can recover from a bad initial yaw using GPS-inertial
		 * heading estimation (yaw emergency estimator) or GPS heading (fixed wings only), but
		 * if this does not fix the issue we need to stop using a position controlled
		 * mode to prevent flyaway crashes.
		 */

		if (run_quality_checks && _status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {

			if (_status.arming_state == vehicle_status_s::ARMING_STATE_STANDBY) {
				_nav_test_failed = false;
				_nav_test_passed = false;

			} else {
				if (!_nav_test_passed) {
					// Both test ratios need to pass/fail together to change the nav test status
					const bool innovation_pass = (estimator_status.vel_test_ratio < 1.0f) && (estimator_status.pos_test_ratio < 1.0f)
								     && (estimator_status.vel_test_ratio > FLT_EPSILON) && (estimator_status.pos_test_ratio > FLT_EPSILON);
					const bool innovation_fail = (estimator_status.vel_test_ratio >= 1.0f) && (estimator_status.pos_test_ratio >= 1.0f);

					if (innovation_pass) {
						_time_last_innov_pass = hrt_absolute_time();

						// if nav status is unconfirmed, confirm yaw angle as passed after 30 seconds or achieving 5 m/s of speed
						const bool sufficient_time = (_status.takeoff_time != 0) && (hrt_elapsed_time(&_status.takeoff_time) > 30_s);
						const bool sufficient_speed = matrix::Vector2f(lpos.vx, lpos.vy).longerThan(5.f);

						// Even if the test already failed, allow it to pass if it did not fail during the last 10 seconds
						if (hrt_elapsed_time(&_time_last_innov_fail) > 10_s
						    && (sufficient_time || sufficient_speed)) {
							_nav_test_passed = true;
							_nav_test_failed = false;
						}

					} else if (innovation_fail) {
						_time_last_innov_fail = hrt_absolute_time();

						if (!_nav_test_failed && hrt_elapsed_time(&_time_last_innov_pass) > 2_s) {
							// if the innovation test has failed continuously, declare the nav as failed
							_nav_test_failed = true;
							mavlink_log_emergency(&_mavlink_log_pub, "Navigation failure! Land and recalibrate sensors");
						}
					}
				}
			}
		}
	}

	// run position and velocity accuracy checks
	// Check if quality checking of position accuracy and consistency is to be performed
	if (run_quality_checks) {
		UpdateEstimateValidity();
	}

	_status_flags.condition_local_altitude_valid = lpos.z_valid
			&& (hrt_elapsed_time(&lpos.timestamp) < (_param_com_pos_fs_delay.get() * 1_s));


	// attitude
	vehicle_attitude_s attitude{};
	_vehicle_attitude_sub.copy(&attitude);
	const matrix::Quatf q{attitude.q};
	const matrix::Quatf q_norm{q.unit()};
	_status_flags.condition_attitude_valid = (hrt_elapsed_time(&attitude.timestamp) < 1_s)
			&& (matrix::Quatf(q - q_norm).length() < FLT_EPSILON);

	// angular velocity
	vehicle_angular_velocity_s angular_velocity{};
	_vehicle_angular_velocity_sub.copy(&angular_velocity);
	_status_flags.condition_angular_velocity_valid = (hrt_elapsed_time(&angular_velocity.timestamp) < 1_s)
			&& PX4_ISFINITE(angular_velocity.xyz[0]) && PX4_ISFINITE(angular_velocity.xyz[1])
			&& PX4_ISFINITE(angular_velocity.xyz[2]);
}

void Commander::UpdateEstimateValidity()
{
	const vehicle_local_position_s &lpos = _local_position_sub.get();
	const vehicle_global_position_s &gpos = _global_position_sub.get();
	const estimator_status_s &status = _estimator_status_sub.get();

	float lpos_eph_threshold_adj = _param_com_pos_fs_eph.get();

	// relax local position eph threshold in operator controlled position mode
	if (_internal_state.main_state == commander_state_s::MAIN_STATE_POSCTL &&
	    ((_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ALTCTL)
	     || (_status.nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL))) {

		// Set the allowable position uncertainty based on combination of flight and estimator state
		// When we are in a operator demanded position control mode and are solely reliant on optical flow, do not check position error because it will gradually increase throughout flight and the operator will compensate for the drift
		const bool reliant_on_opt_flow = ((status.control_mode_flags & (1 << estimator_status_s::CS_OPT_FLOW))
						  && !(status.control_mode_flags & (1 << estimator_status_s::CS_GPS))
						  && !(status.control_mode_flags & (1 << estimator_status_s::CS_EV_POS)));

		if (reliant_on_opt_flow) {
			lpos_eph_threshold_adj = INFINITY;
		}
	}

	_status_flags.condition_global_position_valid =
		check_posvel_validity(lpos.xy_valid && !_nav_test_failed, gpos.eph, _param_com_pos_fs_eph.get(), gpos.timestamp,
				      &_last_gpos_fail_time_us, &_gpos_probation_time_us, _status_flags.condition_global_position_valid);

	_status_flags.condition_local_position_valid =
		check_posvel_validity(lpos.xy_valid && !_nav_test_failed, lpos.eph, lpos_eph_threshold_adj, lpos.timestamp,
				      &_last_lpos_fail_time_us, &_lpos_probation_time_us, _status_flags.condition_local_position_valid);

	_status_flags.condition_local_velocity_valid =
		check_posvel_validity(lpos.v_xy_valid && !_nav_test_failed, lpos.evh, _param_com_vel_fs_evh.get(), lpos.timestamp,
				      &_last_lvel_fail_time_us, &_lvel_probation_time_us, _status_flags.condition_local_velocity_valid);
}

void
Commander::offboard_control_update()
{
	bool offboard_available = false;

	if (_offboard_control_mode_sub.updated()) {
		const offboard_control_mode_s old = _offboard_control_mode_sub.get();

		if (_offboard_control_mode_sub.update()) {
			const offboard_control_mode_s &ocm = _offboard_control_mode_sub.get();

			if (old.position != ocm.position ||
			    old.velocity != ocm.velocity ||
			    old.acceleration != ocm.acceleration ||
			    old.attitude != ocm.attitude ||
			    old.body_rate != ocm.body_rate) {

				_status_changed = true;
			}

			if (ocm.position || ocm.velocity || ocm.acceleration || ocm.attitude || ocm.body_rate) {
				offboard_available = true;
			}
		}
	}

	if (_offboard_control_mode_sub.get().position && !_status_flags.condition_local_position_valid) {
		offboard_available = false;

	} else if (_offboard_control_mode_sub.get().velocity && !_status_flags.condition_local_velocity_valid) {
		offboard_available = false;

	} else if (_offboard_control_mode_sub.get().acceleration && !_status_flags.condition_local_velocity_valid) {
		// OFFBOARD acceleration handled by position controller
		offboard_available = false;
	}

	_offboard_available.set_state_and_update(offboard_available, hrt_absolute_time());

	const bool offboard_lost = !_offboard_available.get_state();

	if (_status_flags.offboard_control_signal_lost != offboard_lost) {
		_status_flags.offboard_control_signal_lost = offboard_lost;
		_status_changed = true;
	}
}

void Commander::esc_status_check()
{
	esc_status_s esc_status{};

	_esc_status_sub.copy(&esc_status);

	if (esc_status.esc_count > 0) {

		char esc_fail_msg[50];
		esc_fail_msg[0] = '\0';

		int online_bitmask = (1 << esc_status.esc_count) - 1;

		// Check if ALL the ESCs are online
		if (online_bitmask == esc_status.esc_online_flags) {

			_status_flags.condition_escs_error = false;
			_last_esc_online_flags = esc_status.esc_online_flags;

		} else if (_last_esc_online_flags == esc_status.esc_online_flags)  {

			// Avoid checking the status if the flags are the same or if the mixer has not yet been loaded in the ESC driver

			_status_flags.condition_escs_error = true;

		} else if (esc_status.esc_online_flags < _last_esc_online_flags) {

			// Only warn the user when an ESC goes from ONLINE to OFFLINE. This is done to prevent showing Offline ESCs warnings at boot

			for (int index = 0; index < esc_status.esc_count; index++) {
				if ((esc_status.esc_online_flags & (1 << index)) == 0) {
					snprintf(esc_fail_msg + strlen(esc_fail_msg), sizeof(esc_fail_msg) - strlen(esc_fail_msg), "ESC%d ", index + 1);
					esc_fail_msg[sizeof(esc_fail_msg) - 1] = '\0';
				}
			}

			mavlink_log_critical(&_mavlink_log_pub, "%soffline", esc_fail_msg);

			_last_esc_online_flags = esc_status.esc_online_flags;
			_status_flags.condition_escs_error = true;
		}

		_status_flags.condition_escs_failure = false;

		for (int index = 0; index < esc_status.esc_count; index++) {

			if (esc_status.esc[index].failures > esc_report_s::FAILURE_NONE) {
				_status_flags.condition_escs_failure = true;

				if (esc_status.esc[index].failures != _last_esc_failure[index]) {

					if (esc_status.esc[index].failures & esc_report_s::FAILURE_OVER_CURRENT_MASK) {
						mavlink_log_critical(&_mavlink_log_pub, "ESC%d: over current", index + 1);
					}

					if (esc_status.esc[index].failures & esc_report_s::FAILURE_OVER_VOLTAGE_MASK) {
						mavlink_log_critical(&_mavlink_log_pub, "ESC%d: over voltage", index + 1);
					}

					if (esc_status.esc[index].failures & esc_report_s::FAILURE_OVER_TEMPERATURE_MASK) {
						mavlink_log_critical(&_mavlink_log_pub, "ESC%d: over temperature", index + 1);
					}

					if (esc_status.esc[index].failures & esc_report_s::FAILURE_OVER_RPM_MASK) {
						mavlink_log_critical(&_mavlink_log_pub, "ESC%d: over RPM", index + 1);
					}

					if (esc_status.esc[index].failures & esc_report_s::FAILURE_INCONSISTENT_CMD_MASK) {
						mavlink_log_critical(&_mavlink_log_pub, "ESC%d: command inconsistency", index + 1);
					}

					if (esc_status.esc[index].failures & esc_report_s::FAILURE_MOTOR_STUCK_MASK) {
						mavlink_log_critical(&_mavlink_log_pub, "ESC%d: motor stuck", index + 1);
					}

					if (esc_status.esc[index].failures & esc_report_s::FAILURE_GENERIC_MASK) {
						mavlink_log_critical(&_mavlink_log_pub, "ESC%d: generic failure - code %d", index + 1, esc_status.esc[index].esc_state);
					}

				}

				_last_esc_failure[index] = esc_status.esc[index].failures;
			}
		}

	}

	_last_esc_status_updated = esc_status.timestamp;
}

int Commander::print_usage(const char *reason)
{
	if (reason) {
		PX4_INFO("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
The commander module contains the state machine for mode switching and failsafe behavior.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("commander", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('h', "Enable HIL mode", true);
#ifndef CONSTRAINED_FLASH
	PRINT_MODULE_USAGE_COMMAND_DESCR("calibrate", "Run sensor calibration");
	PRINT_MODULE_USAGE_ARG("mag|accel|gyro|level|esc|airspeed", "Calibration type", false);
	PRINT_MODULE_USAGE_ARG("quick", "Quick calibration (accel only, not recommended)", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("check", "Run preflight checks");
	PRINT_MODULE_USAGE_COMMAND("arm");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Force arming (do not run preflight checks)", true);
	PRINT_MODULE_USAGE_COMMAND("disarm");
	PRINT_MODULE_USAGE_COMMAND("takeoff");
	PRINT_MODULE_USAGE_COMMAND("land");
	PRINT_MODULE_USAGE_COMMAND_DESCR("transition", "VTOL transition");
	PRINT_MODULE_USAGE_COMMAND_DESCR("mode", "Change flight mode");
	PRINT_MODULE_USAGE_ARG("manual|acro|offboard|stabilized|altctl|posctl|auto:mission|auto:loiter|auto:rtl|auto:takeoff|auto:land|auto:precland",
			"Flight mode", false);
	PRINT_MODULE_USAGE_COMMAND("pair");
	PRINT_MODULE_USAGE_COMMAND("lockdown");
	PRINT_MODULE_USAGE_ARG("off", "Turn lockdown off", true);
	PRINT_MODULE_USAGE_COMMAND("set_ekf_origin");
	PRINT_MODULE_USAGE_ARG("lat, lon, alt", "Origin Latitude, Longitude, Altitude", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("lat|lon|alt", "Origin latitude longitude altitude");
#endif
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 1;
}
