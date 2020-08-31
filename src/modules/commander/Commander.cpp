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
#include "accelerometer_calibration.h"
#include "airspeed_calibration.h"
#include "calibration_routines.h"
#include "commander_helper.h"
#include "esc_calibration.h"
#include "gyro_calibration.h"
#include "level_calibration.h"
#include "mag_calibration.h"
#include "px4_custom_mode.h"
#include "rc_calibration.h"
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
	VEHICLE_MODE_FLAG_SAFETY_ARMED = 128, /* 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. | */
	VEHICLE_MODE_FLAG_ENUM_END = 129, /*  | */
} VEHICLE_MODE_FLAG;

/* Mavlink log uORB handle */
static orb_advert_t mavlink_log_pub = nullptr;

/* flags */
static volatile bool thread_should_exit = false;	/**< daemon exit flag */
static volatile bool thread_running = false;		/**< daemon status flag */



static struct vehicle_status_s status = {};
static struct actuator_armed_s armed = {};

static struct vehicle_status_flags_s status_flags = {};

/**
 * Loop that runs at a lower rate and priority for calibration and parameter tasks.
 */
void *commander_low_prio_loop(void *arg);

static void answer_command(const vehicle_command_s &cmd, unsigned result,
			   uORB::PublicationQueued<vehicle_command_ack_s> &command_ack_pub);

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
static bool send_vehicle_command(uint16_t cmd, float param1 = NAN, float param2 = NAN, float param3 = NAN,
				 float param4 = NAN, float param5 = NAN, float param6 = NAN, float param7 = NAN)
{
	vehicle_command_s vcmd{};

	vcmd.param1 = param1;
	vcmd.param2 = param2;
	vcmd.param3 = param3;
	vcmd.param4 = param4;
	vcmd.param5 = (double)param5;
	vcmd.param6 = (double)param6;
	vcmd.param7 = param7;

	vcmd.command = cmd;

	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	vcmd.source_system = vehicle_status_sub.get().system_id;
	vcmd.target_system = vehicle_status_sub.get().system_id;
	vcmd.source_component = vehicle_status_sub.get().component_id;
	vcmd.target_component = vehicle_status_sub.get().component_id;

	vcmd.timestamp = hrt_absolute_time();

	uORB::PublicationQueued<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};

	return vcmd_pub.publish(vcmd);
}
#endif

extern "C" __EXPORT int commander_main(int argc, char *argv[])
{
	if (argc < 2) {
		Commander::print_usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			PX4_INFO("already running");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;

		Commander::main(argc, argv);

		unsigned constexpr max_wait_us = 1000000;
		unsigned constexpr max_wait_steps = 2000;

		unsigned i;

		for (i = 0; i < max_wait_steps; i++) {
			px4_usleep(max_wait_us / max_wait_steps);

			if (thread_running) {
				break;
			}
		}

		return !(i < max_wait_steps);
	}

	if (!strcmp(argv[1], "stop")) {

		if (!thread_running) {
			PX4_WARN("already stopped");
			return 0;
		}

		thread_should_exit = true;

		Commander::main(argc, argv);
		return 0;
	}

	/* commands needing the app to run below */
	if (!thread_running) {
		PX4_ERR("not started");
		return 1;
	}

	if (!strcmp(argv[1], "status")) {
		PX4_INFO("arming: %s", arming_state_names[status.arming_state]);
		return 0;
	}

#ifndef CONSTRAINED_FLASH

	if (!strcmp(argv[1], "calibrate")) {
		if (argc > 2) {
			if (!strcmp(argv[2], "gyro")) {
				// gyro calibration: param1 = 1
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f);

			} else if (!strcmp(argv[2], "mag")) {
				if (argc > 3 && (strcmp(argv[3], "quick") == 0)) {
					// magnetometer quick calibration: param2 = 2
					send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 0.f, 2.f, 0.f, 0.f, 0.f, 0.f, 0.f);

				} else {
					// magnetometer calibration: param2 = 1
					send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f);
				}

			} else if (!strcmp(argv[2], "accel")) {
				if (argc > 3 && (strcmp(argv[3], "quick") == 0)) {
					// accelerometer quick calibration: param5 = 3
					send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 0.f, 0.f, 0.f, 0.f, 4.f, 0.f, 0.f);

				} else {
					// accelerometer calibration: param5 = 1
					send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f);
				}


			} else if (!strcmp(argv[2], "level")) {
				// board level calibration: param5 = 2
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 0.f, 0.f, 0.f, 0.f, 2.f, 0.f, 0.f);

			} else if (!strcmp(argv[2], "airspeed")) {
				// airspeed calibration: param6 = 2
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 0.f, 0.f, 0.f, 0.f, 0.f, 2.f, 0.f);

			} else if (!strcmp(argv[2], "esc")) {
				// ESC calibration: param7 = 1
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f);

			} else {
				PX4_ERR("argument %s unsupported.", argv[2]);
				return 1;
			}

			return 0;

		} else {
			PX4_ERR("missing argument");
		}
	}

	if (!strcmp(argv[1], "check")) {
		bool preflight_check_res = PreFlightCheck::preflightCheck(nullptr, status, status_flags, true, true, true, 30_s);
		PX4_INFO("Preflight check: %s", preflight_check_res ? "OK" : "FAILED");

		bool prearm_check_res = PreFlightCheck::preArmCheck(nullptr, status_flags, safety_s{},
					PreFlightCheck::arm_requirements_t{}, status);
		PX4_INFO("Prearm check: %s", prearm_check_res ? "OK" : "FAILED");

		print_health_flags(status);

		return 0;
	}

	if (!strcmp(argv[1], "arm")) {
		float param2 = 0.f;

		// 21196: force arming/disarming (e.g. allow arming to override preflight checks and disarming in flight)
		if (argc > 2 && !strcmp(argv[2], "-f")) {
			param2 = 21196.f;
		}

		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.f, param2);

		return 0;
	}

	if (!strcmp(argv[1], "disarm")) {
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.f, 0.f);

		return 0;
	}

	if (!strcmp(argv[1], "takeoff")) {
		// switch to takeoff mode and arm
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF);
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.f, 0.f);

		return 0;
	}

	if (!strcmp(argv[1], "land")) {
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_LAND);

		return 0;
	}

	if (!strcmp(argv[1], "transition")) {
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION,
				     (float)(status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING ?
					     vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW :
					     vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC));

		return 0;
	}

	if (!strcmp(argv[1], "mode")) {
		if (argc > 2) {

			if (!strcmp(argv[2], "manual")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_MANUAL);

			} else if (!strcmp(argv[2], "altctl")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_ALTCTL);

			} else if (!strcmp(argv[2], "posctl")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_POSCTL);

			} else if (!strcmp(argv[2], "auto:mission")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
						     PX4_CUSTOM_SUB_MODE_AUTO_MISSION);

			} else if (!strcmp(argv[2], "auto:loiter")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
						     PX4_CUSTOM_SUB_MODE_AUTO_LOITER);

			} else if (!strcmp(argv[2], "auto:rtl")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
						     PX4_CUSTOM_SUB_MODE_AUTO_RTL);

			} else if (!strcmp(argv[2], "acro")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_ACRO);

			} else if (!strcmp(argv[2], "offboard")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_OFFBOARD);

			} else if (!strcmp(argv[2], "stabilized")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_STABILIZED);

			} else if (!strcmp(argv[2], "rattitude")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_RATTITUDE);

			} else if (!strcmp(argv[2], "auto:takeoff")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
						     PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF);

			} else if (!strcmp(argv[2], "auto:land")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
						     PX4_CUSTOM_SUB_MODE_AUTO_LAND);

			} else if (!strcmp(argv[2], "auto:precland")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
						     PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND);

			} else {
				PX4_ERR("argument %s unsupported.", argv[2]);
			}

			return 0;

		} else {
			PX4_ERR("missing argument");
		}
	}

	if (!strcmp(argv[1], "lockdown")) {

		if (argc < 3) {
			Commander::print_usage("not enough arguments, missing [on, off]");
			return 1;
		}

		bool ret = send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_FLIGHTTERMINATION,
						strcmp(argv[2], "off") ? 2.0f : 0.0f /* lockdown */, 0.0f);

		return (ret ? 0 : 1);
	}

#endif

	Commander::print_usage("unrecognized command");
	return 1;
}

bool Commander::shutdown_if_allowed()
{
	return TRANSITION_DENIED != arming_state_transition(&status, _safety, vehicle_status_s::ARMING_STATE_SHUTDOWN,
			&armed, false /* fRunPreArmChecks */, &mavlink_log_pub, &status_flags, _arm_requirements,
			hrt_elapsed_time(&_boot_timestamp), arm_disarm_reason_t::SHUTDOWN);
}

transition_result_t
Commander::arm_disarm(bool arm, bool run_preflight_checks, orb_advert_t *mavlink_log_pub_local,
		      arm_disarm_reason_t calling_reason)
{
	transition_result_t arming_res = TRANSITION_NOT_CHANGED;

	// Transition the armed state. By passing mavlink_log_pub to arming_state_transition it will
	// output appropriate error messages if the state cannot transition.
	arming_res = arming_state_transition(&status,
					     _safety,
					     arm ? vehicle_status_s::ARMING_STATE_ARMED : vehicle_status_s::ARMING_STATE_STANDBY,
					     &armed,
					     run_preflight_checks,
					     mavlink_log_pub_local,
					     &status_flags,
					     _arm_requirements,
					     hrt_elapsed_time(&_boot_timestamp), calling_reason);

	if (arming_res == TRANSITION_CHANGED) {
		const char *reason = "";

		switch (calling_reason) {
		case arm_disarm_reason_t::TRANSITION_TO_STANDBY: reason = ""; break;

		case arm_disarm_reason_t::RC_STICK: reason = "RC"; break;

		case arm_disarm_reason_t::RC_SWITCH: reason = "RC (switch)"; break;

		case arm_disarm_reason_t::COMMAND_INTERNAL: reason = "internal command"; break;

		case arm_disarm_reason_t::COMMAND_EXTERNAL: reason = "external command"; break;

		case arm_disarm_reason_t::MISSION_START: reason = "mission start"; break;

		case arm_disarm_reason_t::SAFETY_BUTTON: reason = "safety button"; break;

		case arm_disarm_reason_t::AUTO_DISARM_LAND: reason = "landing"; break;

		case arm_disarm_reason_t::AUTO_DISARM_PREFLIGHT: reason = "auto preflight disarming"; break;

		case arm_disarm_reason_t::KILL_SWITCH: reason = "kill-switch"; break;

		case arm_disarm_reason_t::LOCKDOWN: reason = "lockdown"; break;

		case arm_disarm_reason_t::FAILURE_DETECTOR: reason = "failure detector"; break;

		case arm_disarm_reason_t::SHUTDOWN: reason = "shutdown request"; break;

		case arm_disarm_reason_t::UNIT_TEST: reason = "unit tests"; break;
		}

		mavlink_log_info(mavlink_log_pub_local, "%s by %s", arm ? "Armed" : "Disarmed", reason);

	} else if (arming_res == TRANSITION_DENIED) {
		tune_negative(true);
	}

	return arming_res;
}

Commander::Commander() :
	ModuleParams(nullptr),
	_failure_detector(this)
{
	_auto_disarm_landed.set_hysteresis_time_from(false, _param_com_disarm_preflight.get() * 1_s);

	_land_detector.landed = true;

	// XXX for now just set sensors as initialized
	status_flags.condition_system_sensors_initialized = true;

	// We want to accept RC inputs as default
	status.rc_input_mode = vehicle_status_s::RC_IN_MODE_DEFAULT;
	status.nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;
	status.nav_state_timestamp = hrt_absolute_time();
	status.arming_state = vehicle_status_s::ARMING_STATE_INIT;

	/* mark all signals lost as long as they haven't been found */
	status.rc_signal_lost = true;
	status_flags.offboard_control_signal_lost = true;
	status.data_link_lost = true;

	status_flags.condition_power_input_valid = true;
	status_flags.rc_calibration_valid = true;

	// default for vtol is rotary wing
	_vtol_status.vtol_in_rw_mode = true;
}

bool
Commander::handle_command(vehicle_status_s *status_local, const vehicle_command_s &cmd, actuator_armed_s *armed_local,
			  uORB::PublicationQueued<vehicle_command_ack_s> &command_ack_pub)
{
	/* only handle commands that are meant to be handled by this system and component */
	if (cmd.target_system != status_local->system_id || ((cmd.target_component != status_local->component_id)
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
				transition_result_t main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_AUTO_LOITER,
							       status_flags, &_internal_state);

				if ((main_ret != TRANSITION_DENIED)) {
					cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

				} else {
					cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
					mavlink_log_critical(&mavlink_log_pub, "Reposition command rejected");
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

			transition_result_t arming_ret = TRANSITION_NOT_CHANGED;

			transition_result_t main_ret = TRANSITION_NOT_CHANGED;

			// We ignore base_mode & VEHICLE_MODE_FLAG_SAFETY_ARMED because
			// the command VEHICLE_CMD_COMPONENT_ARM_DISARM should be used
			// instead according to the latest mavlink spec.

			if (base_mode & VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED) {
				/* use autopilot-specific mode */
				if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_MANUAL) {
					/* MANUAL */
					main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_MANUAL, status_flags, &_internal_state);

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_ALTCTL) {
					/* ALTCTL */
					main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_ALTCTL, status_flags, &_internal_state);

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_POSCTL) {
					/* POSCTL */
					reset_posvel_validity(&_status_changed);
					main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_POSCTL, status_flags, &_internal_state);

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_AUTO) {
					/* AUTO */
					if (custom_sub_mode > 0) {
						reset_posvel_validity(&_status_changed);

						switch (custom_sub_mode) {
						case PX4_CUSTOM_SUB_MODE_AUTO_LOITER:
							main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_AUTO_LOITER, status_flags,
											 &_internal_state);
							break;

						case PX4_CUSTOM_SUB_MODE_AUTO_MISSION:
							if (status_flags.condition_auto_mission_available) {
								main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_AUTO_MISSION, status_flags,
												 &_internal_state);

							} else {
								main_ret = TRANSITION_DENIED;
							}

							break;

						case PX4_CUSTOM_SUB_MODE_AUTO_RTL:
							main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_AUTO_RTL, status_flags, &_internal_state);
							break;

						case PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF:
							main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_AUTO_TAKEOFF, status_flags,
											 &_internal_state);
							break;

						case PX4_CUSTOM_SUB_MODE_AUTO_LAND:
							main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_AUTO_LAND, status_flags,
											 &_internal_state);
							break;

						case PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET:
							main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_AUTO_FOLLOW_TARGET, status_flags,
											 &_internal_state);
							break;

						case PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND:
							main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_AUTO_PRECLAND, status_flags,
											 &_internal_state);
							break;

						default:
							main_ret = TRANSITION_DENIED;
							mavlink_log_critical(&mavlink_log_pub, "Unsupported auto mode");
							break;
						}

					} else {
						main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_AUTO_MISSION, status_flags,
										 &_internal_state);
					}

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_ACRO) {
					/* ACRO */
					main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_ACRO, status_flags, &_internal_state);

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_RATTITUDE) {
					/* RATTITUDE */
					main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_RATTITUDE, status_flags,
									 &_internal_state);

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_STABILIZED) {
					/* STABILIZED */
					main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_STAB, status_flags, &_internal_state);

				} else if (custom_main_mode == PX4_CUSTOM_MAIN_MODE_OFFBOARD) {
					reset_posvel_validity(&_status_changed);

					/* OFFBOARD */
					main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_OFFBOARD, status_flags, &_internal_state);
				}

			} else {
				/* use base mode */
				if (base_mode & VEHICLE_MODE_FLAG_AUTO_ENABLED) {
					/* AUTO */
					main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_AUTO_MISSION, status_flags,
									 &_internal_state);

				} else if (base_mode & VEHICLE_MODE_FLAG_MANUAL_INPUT_ENABLED) {
					if (base_mode & VEHICLE_MODE_FLAG_GUIDED_ENABLED) {
						/* POSCTL */
						main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_POSCTL, status_flags, &_internal_state);

					} else if (base_mode & VEHICLE_MODE_FLAG_STABILIZE_ENABLED) {
						/* STABILIZED */
						main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_STAB, status_flags, &_internal_state);

					} else {
						/* MANUAL */
						main_ret = main_state_transition(*status_local, commander_state_s::MAIN_STATE_MANUAL, status_flags, &_internal_state);
					}
				}
			}

			if ((arming_ret != TRANSITION_DENIED) && (main_ret != TRANSITION_DENIED)) {
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;

				if (arming_ret == TRANSITION_DENIED) {
					mavlink_log_critical(&mavlink_log_pub, "Arming command rejected");
				}
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM: {

			// Adhere to MAVLink specs, but base on knowledge that these fundamentally encode ints
			// for logic state parameters
			if (static_cast<int>(cmd.param1 + 0.5f) != 0 && static_cast<int>(cmd.param1 + 0.5f) != 1) {
				mavlink_log_critical(&mavlink_log_pub, "Unsupported ARM_DISARM param: %.3f", (double)cmd.param1);

			} else {

				bool cmd_arms = (static_cast<int>(cmd.param1 + 0.5f) == 1);

				// Arm/disarm is enforced when param2 is set to a magic number.
				const bool enforce = (static_cast<int>(roundf(cmd.param2)) == 21196);

				if (!enforce) {
					if (!(_land_detector.landed || _land_detector.maybe_landed) && !is_ground_rover(&status)) {
						if (cmd_arms) {
							if (armed_local->armed) {
								mavlink_log_warning(&mavlink_log_pub, "Arming denied! Already armed");

							} else {
								mavlink_log_critical(&mavlink_log_pub, "Arming denied! Not landed");
							}

						} else {
							mavlink_log_critical(&mavlink_log_pub, "Disarming denied! Not landed");
						}

						cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_DENIED;
						break;
					}

					const bool cmd_from_io = (static_cast<int>(roundf(cmd.param3)) == 1234);

					// Flick to inair restore first if this comes from an onboard system and from IO
					if (cmd.source_system == status_local->system_id && cmd.source_component == status_local->component_id
					    && cmd_from_io && cmd_arms) {
						status.arming_state = vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE;

					} else {
						// Refuse to arm if preflight checks have failed
						if (status_local->hil_state != vehicle_status_s::HIL_STATE_ON
						    && !status_flags.condition_system_sensors_initialized) {
							mavlink_log_critical(&mavlink_log_pub, "Arming denied! Preflight checks have failed");
							cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_DENIED;
							break;
						}

						const bool throttle_above_low = (_manual_control_setpoint.z > 0.1f);
						const bool throttle_above_center = (_manual_control_setpoint.z > 0.6f);

						if (cmd_arms && throttle_above_center &&
						    (status_local->nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL ||
						     status_local->nav_state == vehicle_status_s::NAVIGATION_STATE_ALTCTL)) {
							mavlink_log_critical(&mavlink_log_pub, "Arming denied! Throttle not centered");
							cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_DENIED;
							break;
						}

						if (cmd_arms && throttle_above_low &&
						    (status_local->nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL ||
						     status_local->nav_state == vehicle_status_s::NAVIGATION_STATE_ACRO ||
						     status_local->nav_state == vehicle_status_s::NAVIGATION_STATE_STAB ||
						     status_local->nav_state == vehicle_status_s::NAVIGATION_STATE_RATTITUDE)) {
							mavlink_log_critical(&mavlink_log_pub, "Arming denied! Throttle not zero");
							cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_DENIED;
							break;
						}
					}
				}

				transition_result_t arming_res = arm_disarm(cmd_arms, !enforce, &mavlink_log_pub,
								 (cmd.from_external ? arm_disarm_reason_t::COMMAND_EXTERNAL : arm_disarm_reason_t::COMMAND_INTERNAL));

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
					armed_local->lockdown = true;
					_lockdown_triggered = true;
					PX4_WARN("forcing lockdown (motors off)");
				}

			} else if (cmd.param1 > 0.5f) {
				//XXX update state machine?
				if (!_flight_termination_triggered) {
					armed_local->force_failsafe = true;
					_flight_termination_triggered = true;
					PX4_WARN("forcing failsafe (termination)");
				}

				if ((int)cmd.param2 <= 0) {
					/* reset all commanded failure modes */
					PX4_WARN("reset all non-flighttermination failsafe commands");
				}

			} else {
				armed_local->force_failsafe = false;
				armed_local->lockdown = false;

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

						home.lat = lat;
						home.lon = lon;
						home.alt = alt;

						home.manual_home = true;
						home.valid_alt = true;
						home.valid_hpos = true;

						// update local projection reference including altitude
						struct map_projection_reference_s ref_pos;
						map_projection_init(&ref_pos, local_pos.ref_lat, local_pos.ref_lon);
						map_projection_project(&ref_pos, lat, lon, &home.x, &home.y);
						home.z = -(alt - local_pos.ref_alt);

						/* mark home position as set */
						status_flags.condition_home_position_valid = _home_pub.update(home);

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

	case vehicle_command_s::VEHICLE_CMD_NAV_GUIDED_ENABLE: {
			transition_result_t res = TRANSITION_DENIED;

			if (_internal_state.main_state != commander_state_s::MAIN_STATE_OFFBOARD) {
				_main_state_pre_offboard = _internal_state.main_state;
			}

			if (cmd.param1 > 0.5f) {
				res = main_state_transition(*status_local, commander_state_s::MAIN_STATE_OFFBOARD, status_flags, &_internal_state);

				if (res == TRANSITION_DENIED) {
					print_reject_mode("OFFBOARD");
					status_flags.offboard_control_set_by_command = false;

				} else {
					/* Set flag that offboard was set via command, main state is not overridden by rc */
					status_flags.offboard_control_set_by_command = true;
				}

			} else {
				/* If the mavlink command is used to enable or disable offboard control:
				 * switch back to previous mode when disabling */
				res = main_state_transition(*status_local, _main_state_pre_offboard, status_flags, &_internal_state);
				status_flags.offboard_control_set_by_command = false;
			}

			if (res == TRANSITION_DENIED) {
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;

			} else {
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH: {
			/* switch to RTL which ends the mission */
			if (TRANSITION_CHANGED == main_state_transition(*status_local, commander_state_s::MAIN_STATE_AUTO_RTL, status_flags,
					&_internal_state)) {
				mavlink_log_info(&mavlink_log_pub, "Returning to launch");
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				mavlink_log_critical(&mavlink_log_pub, "Return to launch denied");
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF: {
			/* ok, home set, use it to take off */
			if (TRANSITION_CHANGED == main_state_transition(*status_local, commander_state_s::MAIN_STATE_AUTO_TAKEOFF, status_flags,
					&_internal_state)) {
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else if (_internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_TAKEOFF) {
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				mavlink_log_critical(&mavlink_log_pub, "Takeoff denied! Please disarm and retry");
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_NAV_LAND: {
			if (TRANSITION_CHANGED == main_state_transition(*status_local, commander_state_s::MAIN_STATE_AUTO_LAND, status_flags,
					&_internal_state)) {
				mavlink_log_info(&mavlink_log_pub, "Landing at current position");
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				mavlink_log_critical(&mavlink_log_pub, "Landing denied! Please land manually");
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_NAV_PRECLAND: {
			if (TRANSITION_CHANGED == main_state_transition(*status_local, commander_state_s::MAIN_STATE_AUTO_PRECLAND,
					status_flags, &_internal_state)) {
				mavlink_log_info(&mavlink_log_pub, "Precision landing");
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				mavlink_log_critical(&mavlink_log_pub, "Precision landing denied! Please land manually");
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_MISSION_START: {

			cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_DENIED;

			// check if current mission and first item are valid
			if (status_flags.condition_auto_mission_available) {

				// requested first mission item valid
				if (PX4_ISFINITE(cmd.param1) && (cmd.param1 >= -1) && (cmd.param1 < _mission_result_sub.get().seq_total)) {

					// switch to AUTO_MISSION and ARM
					if ((TRANSITION_DENIED != main_state_transition(*status_local, commander_state_s::MAIN_STATE_AUTO_MISSION, status_flags,
							&_internal_state))
					    && (TRANSITION_DENIED != arm_disarm(true, true, &mavlink_log_pub, arm_disarm_reason_t::MISSION_START))) {

						cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

					} else {
						cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
						mavlink_log_critical(&mavlink_log_pub, "Mission start denied");
					}
				}

			} else {
				mavlink_log_critical(&mavlink_log_pub, "Mission start denied! No valid mission");
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_CONTROL_HIGH_LATENCY: {
			// if no high latency telemetry exists send a failed acknowledge
			if (_high_latency_datalink_heartbeat > _boot_timestamp) {
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_FAILED;
				mavlink_log_critical(&mavlink_log_pub, "Control high latency failed! Telemetry unavailable");
			}
		}
		break;

	case vehicle_command_s::VEHICLE_CMD_DO_ORBIT:

		// Switch to orbit state and let the orbit task handle the command further
		if (TRANSITION_DENIED != main_state_transition(*status_local, commander_state_s::MAIN_STATE_ORBIT, status_flags,
				&_internal_state)) {
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
				answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);

#if defined(CONFIG_BOARDCTL_RESET)

			} else if ((param1 == 1) && shutdown_if_allowed() && (px4_reboot_request(false, 400_ms) == 0)) {
				// 1: Reboot autopilot
				answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);

				while (1) { px4_usleep(1); }

#endif // CONFIG_BOARDCTL_RESET

#if defined(CONFIG_BOARDCTL_POWEROFF)

			} else if ((param1 == 2) && shutdown_if_allowed() && (px4_shutdown_request(400_ms) == 0)) {
				// 2: Shutdown autopilot
				answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);

				while (1) { px4_usleep(1); }

#endif // CONFIG_BOARDCTL_POWEROFF

#if defined(CONFIG_BOARDCTL_RESET)

			} else if ((param1 == 3) && shutdown_if_allowed() && (px4_reboot_request(true, 400_ms) == 0)) {
				// 3: Reboot autopilot and keep it in the bootloader until upgraded.
				answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);

				while (1) { px4_usleep(1); }

#endif // CONFIG_BOARDCTL_RESET

			} else {
				answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_DENIED, command_ack_pub);
			}
		}

		break;

	case vehicle_command_s::VEHICLE_CMD_CUSTOM_0:
	case vehicle_command_s::VEHICLE_CMD_CUSTOM_1:
	case vehicle_command_s::VEHICLE_CMD_CUSTOM_2:
	case vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONTROL:
	case vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONFIGURE:
	case vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONTROL_QUAT:
	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION:
	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_STORAGE:
	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_UAVCAN:
	case vehicle_command_s::VEHICLE_CMD_PAYLOAD_PREPARE_DEPLOY:
	case vehicle_command_s::VEHICLE_CMD_PAYLOAD_CONTROL_DEPLOY:
	case vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION:
	case vehicle_command_s::VEHICLE_CMD_DO_TRIGGER_CONTROL:
	case vehicle_command_s::VEHICLE_CMD_DO_DIGICAM_CONTROL:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_CAM_TRIGG_DIST:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_CAM_TRIGG_INTERVAL:
	case vehicle_command_s::VEHICLE_CMD_SET_CAMERA_MODE:
	case vehicle_command_s::VEHICLE_CMD_SET_CAMERA_ZOOM:
	case vehicle_command_s::VEHICLE_CMD_DO_CHANGE_SPEED:
	case vehicle_command_s::VEHICLE_CMD_DO_LAND_START:
	case vehicle_command_s::VEHICLE_CMD_DO_GO_AROUND:
	case vehicle_command_s::VEHICLE_CMD_START_RX_PAIR:
	case vehicle_command_s::VEHICLE_CMD_LOGGING_START:
	case vehicle_command_s::VEHICLE_CMD_LOGGING_STOP:
	case vehicle_command_s::VEHICLE_CMD_NAV_DELAY:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI:
	case vehicle_command_s::VEHICLE_CMD_NAV_ROI:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_LOCATION:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_WPNEXT_OFFSET:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_NONE:
		/* ignore commands that are handled by other parts of the system */
		break;

	default:
		/* Warn about unsupported commands, this makes sense because only commands
		 * to this component ID (or all) are passed by mavlink. */
		answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_UNSUPPORTED, command_ack_pub);
		break;
	}

	if (cmd_result != vehicle_command_s::VEHICLE_CMD_RESULT_UNSUPPORTED) {
		/* already warned about unsupported commands in "default" case */
		answer_command(cmd, cmd_result, command_ack_pub);
	}

	return true;
}

unsigned
Commander::handle_command_motor_test(const vehicle_command_s &cmd)
{
	if (armed.armed || (_safety.safety_switch_available && !_safety.safety_off)) {
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
	if (status_flags.condition_global_position_valid && status_flags.condition_local_position_valid) {

		const vehicle_global_position_s &gpos = _global_position_sub.get();

		// Ensure that the GPS accuracy is good enough for intializing home
		if ((gpos.eph <= _param_com_home_h_t.get()) && (gpos.epv <= _param_com_home_v_t.get())) {

			const vehicle_local_position_s &lpos = _local_position_sub.get();

			// Set home position
			home_position_s home{};

			home.timestamp = hrt_absolute_time();

			home.lat = gpos.lat;
			home.lon = gpos.lon;
			home.valid_hpos = true;

			home.alt = gpos.alt;
			home.valid_alt = true;

			home.x = lpos.x;
			home.y = lpos.y;
			home.z = lpos.z;

			home.yaw = lpos.heading;
			_heading_reset_counter = lpos.heading_reset_counter;

			home.manual_home = false;

			// play tune first time we initialize HOME
			if (!status_flags.condition_home_position_valid) {
				tune_home_set(true);
			}

			// mark home position as set
			status_flags.condition_home_position_valid = _home_pub.update(home);

			return status_flags.condition_home_position_valid;
		}
	}

	return false;
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

	/* init mission state, do it here to allow navigator to use stored mission even if mavlink failed to start */
	mission_init();

	bool param_init_forced = true;

	control_status_leds(&status, &armed, true, _battery_warning);

	thread_running = true;

	/* update vehicle status to find out vehicle type (required for preflight checks) */
	status.system_type = _param_mav_type.get();

	if (is_rotary_wing(&status) || is_vtol(&status)) {
		status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	} else if (is_fixed_wing(&status)) {
		status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;

	} else if (is_ground_rover(&status)) {
		status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROVER;

	} else {
		status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_UNKNOWN;
	}

	status.is_vtol = is_vtol(&status);
	status.is_vtol_tailsitter = is_vtol_tailsitter(&status);

	_boot_timestamp = hrt_absolute_time();

	// initially set to failed
	_last_lpos_fail_time_us = _boot_timestamp;
	_last_gpos_fail_time_us = _boot_timestamp;
	_last_lvel_fail_time_us = _boot_timestamp;

	// user adjustable duration required to assert arm/disarm via throttle/rudder stick
	uint32_t rc_arm_hyst = _param_rc_arm_hyst.get() * COMMANDER_MONITORING_LOOPSPERMSEC;

	int32_t airmode = 0;
	int32_t rc_map_arm_switch = 0;

	/* initialize low priority thread */
	pthread_attr_t commander_low_prio_attr;
	pthread_attr_init(&commander_low_prio_attr);
	pthread_attr_setstacksize(&commander_low_prio_attr, PX4_STACK_ADJUSTED(3304));

#ifndef __PX4_QURT
	// This is not supported by QURT (yet).
	struct sched_param param;
	pthread_attr_getschedparam(&commander_low_prio_attr, &param);

	/* low priority */
	param.sched_priority = SCHED_PRIORITY_DEFAULT - 50;
	pthread_attr_setschedparam(&commander_low_prio_attr, &param);
#endif
	pthread_t commander_low_prio_thread;
	pthread_create(&commander_low_prio_thread, &commander_low_prio_attr, commander_low_prio_loop, nullptr);
	pthread_attr_destroy(&commander_low_prio_attr);


	status.system_id = _param_mav_sys_id.get();
	arm_auth_init(&mavlink_log_pub, &status.system_id);

	// run preflight immediately to find all relevant parameters, but don't report
	PreFlightCheck::preflightCheck(&mavlink_log_pub, status, status_flags, _arm_requirements.global_position, false, true,
				       hrt_elapsed_time(&_boot_timestamp));

	while (!should_exit()) {

		transition_result_t arming_ret = TRANSITION_NOT_CHANGED;

		/* update parameters */
		bool params_updated = _parameter_update_sub.updated();

		if (params_updated || param_init_forced) {
			// clear update
			parameter_update_s update;
			_parameter_update_sub.copy(&update);

			// update parameters from storage
			updateParams();

			// NAV_DLL_ACT value 4 Data Link Auto Recovery (CASA Outback Challenge rules) deleted 2020-03-10
			if (_param_nav_dll_act.get() == 4) {
				mavlink_log_critical(&mavlink_log_pub, "CASA Outback Challenge rules (NAV_DLL_ACT = 4) retired");
				_param_nav_dll_act.set(2); // value 2 Return mode
				_param_nav_dll_act.commit_no_notification();
			}

			// NAV_RCL_ACT value 4 RC Auto Recovery (CASA Outback Challenge rules) deleted 2020-03-10
			if (_param_nav_rcl_act.get() == 4) {
				mavlink_log_critical(&mavlink_log_pub, "CASA Outback Challenge rules (NAV_RCL_ACT = 4) retired");
				_param_nav_rcl_act.set(2); // value 2 Return mode
				_param_nav_rcl_act.commit_no_notification();
			}

			/* update parameters */
			if (!armed.armed) {
				status.system_type = _param_mav_type.get();

				const bool is_rotary = is_rotary_wing(&status) || (is_vtol(&status) && _vtol_status.vtol_in_rw_mode);
				const bool is_fixed = is_fixed_wing(&status) || (is_vtol(&status) && !_vtol_status.vtol_in_rw_mode);
				const bool is_ground = is_ground_rover(&status);

				/* disable manual override for all systems that rely on electronic stabilization */
				if (is_rotary) {
					status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

				} else if (is_fixed) {
					status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;

				} else if (is_ground) {
					status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROVER;
				}

				/* set vehicle_status.is_vtol flag */
				status.is_vtol = is_vtol(&status);
				status.is_vtol_tailsitter = is_vtol_tailsitter(&status);

				/* check and update system / component ID */
				status.system_id = _param_mav_sys_id.get();
				status.component_id = _param_mav_comp_id.get();

				get_circuit_breaker_params();

				_status_changed = true;
			}

			status_flags.avoidance_system_required = _param_com_obs_avoid.get();

			status.rc_input_mode = _param_rc_in_off.get();

			// percentage (* 0.01) needs to be doubled because RC total interval is 2, not 1
			_min_stick_change = _param_min_stick_change.get() * 0.02f;

			rc_arm_hyst = _param_rc_arm_hyst.get() * COMMANDER_MONITORING_LOOPSPERMSEC;

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
					mavlink_log_critical(&mavlink_log_pub, "Yaw Airmode requires the use of an Arm Switch")
				}
			}

			_offboard_available.set_hysteresis_time_from(true, _param_com_of_loss_t.get());

			param_init_forced = false;
		}

		/* Update OA parameter */
		status_flags.avoidance_system_required = _param_com_obs_avoid.get();

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
					status_flags.condition_power_input_valid = false;

				} else {
					status_flags.condition_power_input_valid = true;
				}

#if defined(CONFIG_BOARDCTL_RESET)

				if (!status_flags.circuit_breaker_engaged_usb_check && status_flags.usb_connected) {
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
							mavlink_log_critical(&mavlink_log_pub, "USB disconnected, rebooting for flight safety");

							while (1) { px4_usleep(1); }
						}
					}
				}

#endif // CONFIG_BOARDCTL_RESET

				_system_power_usb_connected = system_power.usb_connected;
			}
		}

		/* update safety topic */
		if (_safety_sub.updated()) {
			const bool previous_safety_off = _safety.safety_off;

			if (_safety_sub.copy(&_safety)) {
				// disarm if safety is now on and still armed
				if (armed.armed && _safety.safety_switch_available && !_safety.safety_off) {

					bool safety_disarm_allowed = (status.hil_state == vehicle_status_s::HIL_STATE_OFF);

					// if land detector is available then prevent disarming via safety button if not landed
					if (hrt_elapsed_time(&_land_detector.timestamp) < 1_s) {

						bool maybe_landing = (_land_detector.landed || _land_detector.maybe_landed);

						if (!maybe_landing) {
							safety_disarm_allowed = false;
						}
					}

					if (safety_disarm_allowed) {
						if (TRANSITION_CHANGED == arm_disarm(false, true, &mavlink_log_pub, arm_disarm_reason_t::SAFETY_BUTTON)) {
							_status_changed = true;
						}
					}
				}

				// Notify the user if the status of the safety switch changes
				if (_safety.safety_switch_available && previous_safety_off != _safety.safety_off) {

					if (_safety.safety_off) {
						set_tune(TONE_NOTIFY_POSITIVE_TUNE);

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
			status.vtol_fw_permanent_stab = _vtol_status.fw_permanent_stab;

			/* Make sure that this is only adjusted if vehicle really is of type vtol */
			if (is_vtol(&status)) {

				// Check if there has been any change while updating the flags
				const auto new_vehicle_type = _vtol_status.vtol_in_rw_mode ?
							      vehicle_status_s::VEHICLE_TYPE_ROTARY_WING :
							      vehicle_status_s::VEHICLE_TYPE_FIXED_WING;

				if (new_vehicle_type != status.vehicle_type) {
					status.vehicle_type = _vtol_status.vtol_in_rw_mode ?
							      vehicle_status_s::VEHICLE_TYPE_ROTARY_WING :
							      vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
					_status_changed = true;
				}

				if (status.in_transition_mode != _vtol_status.vtol_in_trans_mode) {
					status.in_transition_mode = _vtol_status.vtol_in_trans_mode;
					_status_changed = true;
				}

				if (status.in_transition_to_fw != _vtol_status.in_transition_to_fw) {
					status.in_transition_to_fw = _vtol_status.in_transition_to_fw;
					_status_changed = true;
				}

				if (status_flags.vtol_transition_failure != _vtol_status.vtol_transition_failsafe) {
					status_flags.vtol_transition_failure = _vtol_status.vtol_transition_failsafe;
					_status_changed = true;
				}

				const bool should_soft_stop = (status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);

				if (armed.soft_stop != should_soft_stop) {
					armed.soft_stop = should_soft_stop;
					_status_changed = true;
				}
			}
		}

		if (_esc_status_sub.updated()) {
			/* ESCs status changed */
			esc_status_s esc_status{};

			if (_esc_status_sub.copy(&esc_status)) {
				esc_status_check(esc_status);
			}
		}

		estimator_check(status_flags);

		/* Update land detector */
		if (_land_detector_sub.updated()) {
			_was_landed = _land_detector.landed;
			_land_detector_sub.copy(&_land_detector);

			// Only take actions if armed
			if (armed.armed) {
				if (_was_landed != _land_detector.landed) {
					if (_land_detector.landed) {
						mavlink_log_info(&mavlink_log_pub, "Landing detected");

					} else {
						mavlink_log_info(&mavlink_log_pub, "Takeoff detected");
						_have_taken_off_since_arming = true;

						// Set all position and velocity test probation durations to takeoff value
						// This is a larger value to give the vehicle time to complete a failsafe landing
						// if faulty sensors cause loss of navigation shortly after takeoff.
						_gpos_probation_time_us = _param_com_pos_fs_prob.get() * 1_s;
						_lpos_probation_time_us = _param_com_pos_fs_prob.get() * 1_s;
						_lvel_probation_time_us = _param_com_pos_fs_prob.get() * 1_s;
					}
				}
			}
		}


		// Auto disarm when landed or kill switch engaged
		if (armed.armed) {

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
					arm_disarm(false, true, &mavlink_log_pub,
						   (_have_taken_off_since_arming ? arm_disarm_reason_t::AUTO_DISARM_LAND : arm_disarm_reason_t::AUTO_DISARM_PREFLIGHT));
				}
			}

			// Auto disarm after 5 seconds if kill switch is engaged
			_auto_disarm_killed.set_state_and_update(armed.manual_lockdown || armed.lockdown, hrt_absolute_time());

			if (_auto_disarm_killed.get_state()) {
				if (armed.manual_lockdown) {
					arm_disarm(false, true, &mavlink_log_pub, arm_disarm_reason_t::KILL_SWITCH);

				} else {
					arm_disarm(false, true, &mavlink_log_pub, arm_disarm_reason_t::LOCKDOWN);
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

		/* update subsystem info which arrives from outside of commander*/
		subsystem_info_s info;

		while (_subsys_sub.update(&info))  {
			set_health_flags(info.subsystem_type, info.present, info.enabled, info.ok, status);
			_status_changed = true;
		}

		/* If in INIT state, try to proceed to STANDBY state */
		if (!status_flags.condition_calibration_enabled && status.arming_state == vehicle_status_s::ARMING_STATE_INIT) {

			arming_ret = arming_state_transition(&status, _safety, vehicle_status_s::ARMING_STATE_STANDBY, &armed,
							     true /* fRunPreArmChecks */, &mavlink_log_pub, &status_flags,
							     _arm_requirements, hrt_elapsed_time(&_boot_timestamp),
							     arm_disarm_reason_t::TRANSITION_TO_STANDBY);

			if (arming_ret == TRANSITION_DENIED) {
				/* do not complain if not allowed into standby */
				arming_ret = TRANSITION_NOT_CHANGED;
			}
		}

		/* start mission result check */
		const auto prev_mission_instance_count = _mission_result_sub.get().instance_count;

		if (_mission_result_sub.update()) {
			const mission_result_s &mission_result = _mission_result_sub.get();

			// if mission_result is valid for the current mission
			const bool mission_result_ok = (mission_result.timestamp > _boot_timestamp)
						       && (mission_result.instance_count > 0);

			status_flags.condition_auto_mission_available = mission_result_ok && mission_result.valid;

			if (mission_result_ok) {

				if (status.mission_failure != mission_result.failure) {
					status.mission_failure = mission_result.failure;
					_status_changed = true;

					if (status.mission_failure) {
						mavlink_log_critical(&mavlink_log_pub, "Mission cannot be completed");
					}
				}

				/* Only evaluate mission state if home is set */
				if (status_flags.condition_home_position_valid &&
				    (prev_mission_instance_count != mission_result.instance_count)) {

					if (!status_flags.condition_auto_mission_available) {
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
		}

		// update manual_control_setpoint before geofence (which might check sticks or switches)
		_manual_control_setpoint_sub.update(&_manual_control_setpoint);


		/* start geofence result check */
		_geofence_result_sub.update(&_geofence_result);

		const bool in_low_battery_failsafe = _battery_warning > battery_status_s::BATTERY_WARNING_LOW;

		// Geofence actions
		const bool geofence_action_enabled = _geofence_result.geofence_action != geofence_result_s::GF_ACTION_NONE;

		if (armed.armed
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
						if (TRANSITION_CHANGED == main_state_transition(status, commander_state_s::MAIN_STATE_AUTO_LOITER, status_flags,
								&_internal_state)) {
							_geofence_loiter_on = true;
						}

						break;
					}

				case (geofence_result_s::GF_ACTION_RTL) : {
						if (TRANSITION_CHANGED == main_state_transition(status, commander_state_s::MAIN_STATE_AUTO_RTL, status_flags,
								&_internal_state)) {
							_geofence_rtl_on = true;
						}

						break;
					}

				case (geofence_result_s::GF_ACTION_LAND) : {
						if (TRANSITION_CHANGED == main_state_transition(status, commander_state_s::MAIN_STATE_AUTO_LAND, status_flags,
								&_internal_state)) {
							_geofence_land_on = true;
						}

						break;
					}

				case (geofence_result_s::GF_ACTION_TERMINATE) : {
						PX4_WARN("Flight termination because of geofence");
						mavlink_log_critical(&mavlink_log_pub, "Geofence violation! Flight terminated");
						armed.force_failsafe = true;
						_status_changed = true;
						break;
					}
				}
			}

			_geofence_violated_prev = _geofence_result.geofence_violated;

			// reset if no longer in LOITER or if manually switched to LOITER
			const bool in_loiter_mode = _internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_LOITER;
			const bool manual_loiter_switch_on = _manual_control_setpoint.loiter_switch == manual_control_setpoint_s::SWITCH_POS_ON;

			if (!in_loiter_mode || manual_loiter_switch_on) {
				_geofence_loiter_on = false;
			}


			// reset if no longer in RTL or if manually switched to RTL
			const bool in_rtl_mode = _internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_RTL;
			const bool manual_return_switch_on = _manual_control_setpoint.return_switch == manual_control_setpoint_s::SWITCH_POS_ON;

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

		// abort auto mode or geofence reaction if sticks are moved significantly
		// but only if not in a low battery handling action
		const bool is_rotary_wing = status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

		const bool override_auto_mode =
			(_param_rc_override.get() & OVERRIDE_AUTO_MODE_BIT) &&
			(_internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_TAKEOFF ||
			 _internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_LAND    ||
			 _internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_RTL 	  ||
			 _internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_MISSION ||
			 _internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_LOITER ||
			 _internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_FOLLOW_TARGET ||
			 _internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_PRECLAND);

		const bool override_offboard_mode =
			(_param_rc_override.get() & OVERRIDE_OFFBOARD_MODE_BIT) &&
			_internal_state.main_state == commander_state_s::MAIN_STATE_OFFBOARD;

		if ((override_auto_mode || override_offboard_mode) && is_rotary_wing
		    && !in_low_battery_failsafe && !_geofence_warning_action_on) {
			// transition to previous state if sticks are touched
			if (hrt_elapsed_time(&_manual_control_setpoint.timestamp) < 1_s && // don't use uninitialized or old messages
			    ((fabsf(_manual_control_setpoint.x) > _min_stick_change) ||
			     (fabsf(_manual_control_setpoint.y) > _min_stick_change))) {
				// revert to position control in any case
				main_state_transition(status, commander_state_s::MAIN_STATE_POSCTL, status_flags, &_internal_state);
				mavlink_log_info(&mavlink_log_pub, "Pilot took over control using sticks");
			}
		}

		/* Check for mission flight termination */
		if (armed.armed && _mission_result_sub.get().flight_termination &&
		    !status_flags.circuit_breaker_flight_termination_disabled) {

			armed.force_failsafe = true;
			_status_changed = true;

			if (!_flight_termination_printed) {
				mavlink_log_critical(&mavlink_log_pub, "Geofence violation! Flight terminated");
				_flight_termination_printed = true;
			}

			if (_counter % (1000000 / COMMANDER_MONITORING_INTERVAL) == 0) {
				mavlink_log_critical(&mavlink_log_pub, "Flight termination active");
			}
		}

		/* RC input check */
		if (!status_flags.rc_input_blocked && _manual_control_setpoint.timestamp != 0 &&
		    (hrt_elapsed_time(&_manual_control_setpoint.timestamp) < (_param_com_rc_loss_t.get() * 1_s))) {

			/* handle the case where RC signal was regained */
			if (!status_flags.rc_signal_found_once) {
				status_flags.rc_signal_found_once = true;
				set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_RCRECEIVER, true, true, status_flags.rc_calibration_valid, status);
				_status_changed = true;

			} else {
				if (status.rc_signal_lost) {
					if (_rc_signal_lost_timestamp > 0) {
						mavlink_log_info(&mavlink_log_pub, "Manual control regained after %.1fs",
								 hrt_elapsed_time(&_rc_signal_lost_timestamp) * 1e-6);
					}

					set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_RCRECEIVER, true, true, status_flags.rc_calibration_valid, status);
					_status_changed = true;
				}
			}

			status.rc_signal_lost = false;

			const bool in_armed_state = (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
			const bool arm_switch_or_button_mapped =
				_manual_control_setpoint.arm_switch != manual_control_setpoint_s::SWITCH_POS_NONE;
			const bool arm_button_pressed = _param_arm_switch_is_button.get()
							&& (_manual_control_setpoint.arm_switch == manual_control_setpoint_s::SWITCH_POS_ON);

			/* DISARM
			 * check if left stick is in lower left position or arm button is pushed or arm switch has transition from arm to disarm
			 * and we are in MANUAL, Rattitude, or AUTO_READY mode or (ASSIST mode and landed)
			 * do it only for rotary wings in manual mode or fixed wing if landed.
			 * Disable stick-disarming if arming switch or button is mapped */
			const bool stick_in_lower_left = _manual_control_setpoint.r < -STICK_ON_OFF_LIMIT
							 && (_manual_control_setpoint.z < 0.1f)
							 && !arm_switch_or_button_mapped;
			const bool arm_switch_to_disarm_transition = !_param_arm_switch_is_button.get() &&
					(_last_manual_control_setpoint_arm_switch == manual_control_setpoint_s::SWITCH_POS_ON) &&
					(_manual_control_setpoint.arm_switch == manual_control_setpoint_s::SWITCH_POS_OFF);

			if (in_armed_state &&
			    (status.rc_input_mode != vehicle_status_s::RC_IN_MODE_OFF) &&
			    (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING || _land_detector.landed) &&
			    (stick_in_lower_left || arm_button_pressed || arm_switch_to_disarm_transition)) {

				const bool manual_thrust_mode = _internal_state.main_state == commander_state_s::MAIN_STATE_MANUAL
								|| _internal_state.main_state == commander_state_s::MAIN_STATE_ACRO
								|| _internal_state.main_state == commander_state_s::MAIN_STATE_STAB
								|| _internal_state.main_state == commander_state_s::MAIN_STATE_RATTITUDE;
				const bool rc_wants_disarm = (_stick_off_counter == rc_arm_hyst && _stick_on_counter < rc_arm_hyst)
							     || arm_switch_to_disarm_transition;

				if (rc_wants_disarm && (_land_detector.landed || manual_thrust_mode)) {
					arming_ret = arming_state_transition(&status, _safety, vehicle_status_s::ARMING_STATE_STANDBY, &armed,
									     true /* fRunPreArmChecks */,
									     &mavlink_log_pub, &status_flags, _arm_requirements, hrt_elapsed_time(&_boot_timestamp),
									     (arm_switch_to_disarm_transition ? arm_disarm_reason_t::RC_SWITCH : arm_disarm_reason_t::RC_STICK));
				}

				_stick_off_counter++;

			} else if (!(_param_arm_switch_is_button.get()
				     && _manual_control_setpoint.arm_switch == manual_control_setpoint_s::SWITCH_POS_ON)) {
				/* do not reset the counter when holding the arm button longer than needed */
				_stick_off_counter = 0;
			}

			/* ARM
			 * check if left stick is in lower right position or arm button is pushed or arm switch has transition from disarm to arm
			 * and we're in MANUAL mode.
			 * Disable stick-arming if arming switch or button is mapped */
			const bool stick_in_lower_right = _manual_control_setpoint.r > STICK_ON_OFF_LIMIT && _manual_control_setpoint.z < 0.1f
							  && !arm_switch_or_button_mapped;
			/* allow a grace period for re-arming: preflight checks don't need to pass during that time,
			 * for example for accidential in-air disarming */
			const bool in_arming_grace_period = (_last_disarmed_timestamp != 0)
							    && (hrt_elapsed_time(&_last_disarmed_timestamp) < 5_s);

			const bool arm_switch_to_arm_transition = !_param_arm_switch_is_button.get() &&
					(_last_manual_control_setpoint_arm_switch == manual_control_setpoint_s::SWITCH_POS_OFF) &&
					(_manual_control_setpoint.arm_switch == manual_control_setpoint_s::SWITCH_POS_ON) &&
					(_manual_control_setpoint.z < 0.1f || in_arming_grace_period);

			if (!in_armed_state &&
			    (status.rc_input_mode != vehicle_status_s::RC_IN_MODE_OFF) &&
			    (stick_in_lower_right || arm_button_pressed || arm_switch_to_arm_transition)) {

				if ((_stick_on_counter == rc_arm_hyst && _stick_off_counter < rc_arm_hyst) || arm_switch_to_arm_transition) {

					/* we check outside of the transition function here because the requirement
					 * for being in manual mode only applies to manual arming actions.
					 * the system can be armed in auto if armed via the GCS.
					 */
					if ((_internal_state.main_state != commander_state_s::MAIN_STATE_MANUAL)
					    && (_internal_state.main_state != commander_state_s::MAIN_STATE_ACRO)
					    && (_internal_state.main_state != commander_state_s::MAIN_STATE_STAB)
					    && (_internal_state.main_state != commander_state_s::MAIN_STATE_ALTCTL)
					    && (_internal_state.main_state != commander_state_s::MAIN_STATE_POSCTL)
					    && (_internal_state.main_state != commander_state_s::MAIN_STATE_RATTITUDE)
					   ) {
						print_reject_arm("Not arming: Switch to a manual mode first");

					} else if (!status_flags.condition_home_position_valid &&
						   (_param_geofence_action.get() == geofence_result_s::GF_ACTION_RTL)) {

						print_reject_arm("Not arming: Geofence RTL requires valid home");

					} else if (status.arming_state == vehicle_status_s::ARMING_STATE_STANDBY) {
						arming_ret = arming_state_transition(&status, _safety, vehicle_status_s::ARMING_STATE_ARMED, &armed,
										     !in_arming_grace_period /* fRunPreArmChecks */,
										     &mavlink_log_pub, &status_flags, _arm_requirements, hrt_elapsed_time(&_boot_timestamp),
										     (arm_switch_to_arm_transition ? arm_disarm_reason_t::RC_SWITCH : arm_disarm_reason_t::RC_STICK));

						if (arming_ret != TRANSITION_CHANGED) {
							px4_usleep(100000);
							print_reject_arm("Not arming: Preflight checks failed");
						}
					}
				}

				_stick_on_counter++;

			} else if (!(_param_arm_switch_is_button.get()
				     && _manual_control_setpoint.arm_switch == manual_control_setpoint_s::SWITCH_POS_ON)) {
				/* do not reset the counter when holding the arm button longer than needed */
				_stick_on_counter = 0;
			}

			_last_manual_control_setpoint_arm_switch = _manual_control_setpoint.arm_switch;

			if (arming_ret == TRANSITION_DENIED) {
				/*
				 * the arming transition can be denied to a number of reasons:
				 *  - pre-flight check failed (sensors not ok or not calibrated)
				 *  - safety not disabled
				 *  - system not in manual mode
				 */
				tune_negative(true);
			}

			/* evaluate the main state machine according to mode switches */
			bool first_rc_eval = (_last_manual_control_setpoint.timestamp == 0) && (_manual_control_setpoint.timestamp > 0);
			transition_result_t main_res = set_main_state(status, &_status_changed);

			/* store last position lock state */
			_last_condition_local_altitude_valid = status_flags.condition_local_altitude_valid;
			_last_condition_local_position_valid = status_flags.condition_local_position_valid;
			_last_condition_global_position_valid = status_flags.condition_global_position_valid;

			/* play tune on mode change only if armed, blink LED always */
			if (main_res == TRANSITION_CHANGED || first_rc_eval) {
				tune_positive(armed.armed);
				_status_changed = true;

			} else if (main_res == TRANSITION_DENIED) {
				/* DENIED here indicates bug in the commander */
				mavlink_log_critical(&mavlink_log_pub, "Switching to this mode is currently not possible");
			}

			/* check throttle kill switch */
			if (_manual_control_setpoint.kill_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
				/* set lockdown flag */
				if (!armed.manual_lockdown) {
					const char kill_switch_string[] = "Kill-switch engaged";

					if (_land_detector.landed) {
						mavlink_log_info(&mavlink_log_pub, kill_switch_string);

					} else {
						mavlink_log_critical(&mavlink_log_pub, kill_switch_string);
					}

					_status_changed = true;
					armed.manual_lockdown = true;
				}

			} else if (_manual_control_setpoint.kill_switch == manual_control_setpoint_s::SWITCH_POS_OFF) {
				if (armed.manual_lockdown) {
					mavlink_log_info(&mavlink_log_pub, "Kill-switch disengaged");
					_status_changed = true;
					armed.manual_lockdown = false;
				}
			}

			/* no else case: do not change lockdown flag in unconfigured case */

		} else {
			// set RC lost
			if (status_flags.rc_signal_found_once && !status.rc_signal_lost) {
				// ignore RC lost during calibration
				if (!status_flags.condition_calibration_enabled && !status_flags.rc_input_blocked) {
					mavlink_log_critical(&mavlink_log_pub, "Manual control lost");
					status.rc_signal_lost = true;
					_rc_signal_lost_timestamp = _manual_control_setpoint.timestamp;
					set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_RCRECEIVER, true, true, false, status);
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
			if (!status_flags.circuit_breaker_engaged_enginefailure_check &&
			    status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING && !status.is_vtol && armed.armed) {

				actuator_controls_s actuator_controls{};
				_actuator_controls_sub.copy(&actuator_controls);

				const float throttle = actuator_controls.control[actuator_controls_s::INDEX_THROTTLE];
				const float current2throttle = _battery_current / throttle;

				if (((throttle > _param_ef_throttle_thres.get()) && (current2throttle < _param_ef_current2throttle_thres.get()))
				    || status.engine_failure) {

					const float elapsed = hrt_elapsed_time(&_timestamp_engine_healthy) / 1e6f;

					/* potential failure, measure time */
					if ((_timestamp_engine_healthy > 0) && (elapsed > _param_ef_time_thres.get())
					    && !status.engine_failure) {

						status.engine_failure = true;
						_status_changed = true;

						PX4_ERR("Engine Failure");
						set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_MOTORCONTROL, true, true, false, status);
					}
				}

			} else {
				/* no failure reset flag */
				_timestamp_engine_healthy = hrt_absolute_time();

				if (status.engine_failure) {
					status.engine_failure = false;
					_status_changed = true;
				}
			}
		}

		/* Reset main state to loiter or auto-mission after takeoff is completed.
		 * Sometimes, the mission result topic is outdated and the mission is still signaled
		 * as finished even though we only just started with the takeoff. Therefore, we also
		 * check the timestamp of the mission_result topic. */
		if (_internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_TAKEOFF
		    && (_mission_result_sub.get().timestamp > _internal_state.timestamp)
		    && _mission_result_sub.get().finished) {

			const bool mission_available = (_mission_result_sub.get().timestamp > _boot_timestamp)
						       && (_mission_result_sub.get().instance_count > 0) && _mission_result_sub.get().valid;

			if ((_param_takeoff_finished_action.get() == 1) && mission_available) {
				main_state_transition(status, commander_state_s::MAIN_STATE_AUTO_MISSION, status_flags, &_internal_state);

			} else {
				main_state_transition(status, commander_state_s::MAIN_STATE_AUTO_LOITER, status_flags, &_internal_state);
			}
		}

		/* check if we are disarmed and there is a better mode to wait in */
		if (!armed.armed) {
			/* if there is no radio control but GPS lock the user might want to fly using
			 * just a tablet. Since the RC will force its mode switch setting on connecting
			 * we can as well just wait in a hold mode which enables tablet control.
			 */
			if (status.rc_signal_lost && (_internal_state.main_state == commander_state_s::MAIN_STATE_MANUAL)
			    && status_flags.condition_home_position_valid) {

				main_state_transition(status, commander_state_s::MAIN_STATE_AUTO_LOITER, status_flags, &_internal_state);
			}
		}

		/* handle commands last, as the system needs to be updated to handle them */
		if (_cmd_sub.updated()) {
			/* got command */
			const unsigned last_generation = _cmd_sub.get_last_generation();
			vehicle_command_s cmd;

			if (_cmd_sub.copy(&cmd)) {
				if (_cmd_sub.get_last_generation() != last_generation + 1) {
					PX4_ERR("vehicle_command lost, generation %d -> %d", last_generation, _cmd_sub.get_last_generation());
				}

				if (handle_command(&status, cmd, &armed, _command_ack_pub)) {
					_status_changed = true;
				}
			}
		}

		/* Check for failure detector status */
		const bool failure_detector_updated = _failure_detector.update(status);

		if (failure_detector_updated) {

			const uint8_t failure_status = _failure_detector.getStatus();

			if (failure_status != status.failure_detector_status) {
				status.failure_detector_status = failure_status;
				_status_changed = true;
			}
		}

		if (armed.armed &&
		    failure_detector_updated) {

			if (_failure_detector.isFailure()) {

				const hrt_abstime time_at_arm = armed.armed_time_ms * 1000;

				if (hrt_elapsed_time(&time_at_arm) < 500_ms) {
					// 500ms is the PWM spoolup time. Within this timeframe controllers are not affecting actuator_outputs

					if (status.failure_detector_status & vehicle_status_s::FAILURE_ARM_ESC) {
						arm_disarm(false, true, &mavlink_log_pub, arm_disarm_reason_t::FAILURE_DETECTOR);
						_status_changed = true;
						mavlink_log_critical(&mavlink_log_pub, "ESCs did not respond to arm request");
					}

				}

				if (hrt_elapsed_time(&_time_at_takeoff) < (1_s * _param_com_lkdown_tko.get())) {
					// This handles the case where something fails during the early takeoff phase
					if (!_lockdown_triggered) {

						armed.lockdown = true;
						_lockdown_triggered = true;
						_status_changed = true;

						mavlink_log_emergency(&mavlink_log_pub, "Critical failure detected: lockdown");
					}

				} else if (!status_flags.circuit_breaker_flight_termination_disabled &&
					   !_flight_termination_triggered && !_lockdown_triggered) {

					armed.force_failsafe = true;
					_flight_termination_triggered = true;
					_status_changed = true;

					mavlink_log_emergency(&mavlink_log_pub, "Critical failure detected: terminate flight");
					set_tune_override(TONE_PARACHUTE_RELEASE_TUNE);
				}
			}
		}

		/* Get current timestamp */
		const hrt_abstime now = hrt_absolute_time();

		// automatically set or update home position
		if (!_home_pub.get().manual_home) {
			const vehicle_local_position_s &local_position = _local_position_sub.get();

			// set the home position when taking off, but only if we were previously disarmed
			// and at least 500 ms from commander start spent to avoid setting home on in-air restart
			if (_should_set_home_on_takeoff && _was_landed && !_land_detector.landed &&
			    (hrt_elapsed_time(&_boot_timestamp) > INAIR_RESTART_HOLDOFF_INTERVAL)) {
				_should_set_home_on_takeoff = false;
				set_home_position();
			}

			if (!armed.armed) {
				if (status_flags.condition_home_position_valid) {
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
					if (!status_flags.condition_home_position_valid) {
						set_home_position_alt_only();
					}
				}
			}
		}

		// check for arming state change
		if (_was_armed != armed.armed) {
			_status_changed = true;

			if (armed.armed) {
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

		if (!armed.armed) {
			/* Reset the flag if disarmed. */
			_have_taken_off_since_arming = false;
		}

		_was_armed = armed.armed;

		/* now set navigation state according to failsafe and main state */
		bool nav_state_changed = set_nav_state(&status,
						       &armed,
						       &_internal_state,
						       &mavlink_log_pub,
						       (link_loss_actions_t)_param_nav_dll_act.get(),
						       _mission_result_sub.get().finished,
						       _mission_result_sub.get().stay_in_failsafe,
						       status_flags,
						       _land_detector.landed,
						       (link_loss_actions_t)_param_nav_rcl_act.get(),
						       (offboard_loss_actions_t)_param_com_obl_act.get(),
						       (offboard_loss_rc_actions_t)_param_com_obl_rc_act.get(),
						       (position_nav_loss_actions_t)_param_com_posctl_navl.get());

		if (nav_state_changed) {
			status.nav_state_timestamp = hrt_absolute_time();
		}

		if (status.failsafe != _failsafe_old) {
			_status_changed = true;

			if (status.failsafe) {
				mavlink_log_info(&mavlink_log_pub, "Failsafe mode activated");

			} else {
				mavlink_log_info(&mavlink_log_pub, "Failsafe mode deactivated");
			}

			_failsafe_old = status.failsafe;
		}

		/* publish states (armed, control_mode, vehicle_status, commander_state, vehicle_status_flags) at 2 Hz or immediately when changed */
		if (hrt_elapsed_time(&status.timestamp) >= 500_ms || _status_changed || nav_state_changed) {

			update_control_mode();

			status.timestamp = hrt_absolute_time();
			_status_pub.publish(status);

			switch ((PrearmedMode)_param_com_prearm_mode.get()) {
			case PrearmedMode::DISABLED:
				/* skip prearmed state  */
				armed.prearmed = false;
				break;

			case PrearmedMode::ALWAYS:
				/* safety is not present, go into prearmed
				* (all output drivers should be started / unlocked last in the boot process
				* when the rest of the system is fully initialized)
				*/
				armed.prearmed = (hrt_elapsed_time(&_boot_timestamp) > 5_s);
				break;

			case PrearmedMode::SAFETY_BUTTON:
				if (_safety.safety_switch_available) {
					/* safety switch is present, go into prearmed if safety is off */
					armed.prearmed = _safety.safety_off;

				} else {
					/* safety switch is not present, do not go into prearmed */
					armed.prearmed = false;
				}

				break;

			default:
				armed.prearmed = false;
				break;
			}

			armed.timestamp = hrt_absolute_time();
			_armed_pub.publish(armed);

			/* publish internal state for logging purposes */
			_internal_state.timestamp = hrt_absolute_time();
			_commander_state_pub.publish(_internal_state);

			/* publish vehicle_status_flags */
			status_flags.timestamp = hrt_absolute_time();

			// Evaluate current prearm status
			if (!armed.armed && !status_flags.condition_calibration_enabled) {
				bool preflight_check_res = PreFlightCheck::preflightCheck(nullptr, status, status_flags, true, false, true, 30_s);
				bool prearm_check_res = PreFlightCheck::preArmCheck(nullptr, status_flags, _safety, _arm_requirements, status, false);
				set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_PREARM_CHECK, true, true, (preflight_check_res
						 && prearm_check_res), status);
			}

			_vehicle_status_flags_pub.publish(status_flags);
		}

		/* play arming and battery warning tunes */
		if (!_arm_tune_played && armed.armed &&
		    (_safety.safety_switch_available || (_safety.safety_switch_available && _safety.safety_off))) {

			/* play tune when armed */
			set_tune(TONE_ARMING_WARNING_TUNE);
			_arm_tune_played = true;

		} else if (!status_flags.usb_connected &&
			   (status.hil_state != vehicle_status_s::HIL_STATE_ON) &&
			   (_battery_warning == battery_status_s::BATTERY_WARNING_CRITICAL)) {
			/* play tune on battery critical */
			set_tune(TONE_BATTERY_WARNING_FAST_TUNE);

		} else if ((status.hil_state != vehicle_status_s::HIL_STATE_ON) &&
			   (_battery_warning == battery_status_s::BATTERY_WARNING_LOW)) {
			/* play tune on battery warning */
			set_tune(TONE_BATTERY_WARNING_SLOW_TUNE);

		} else if (status.failsafe) {
			tune_failsafe(true);

		} else {
			set_tune(TONE_STOP_TUNE);
		}

		/* reset arm_tune_played when disarmed */
		if (!armed.armed || (_safety.safety_switch_available && !_safety.safety_off)) {

			//Notify the user that it is safe to approach the vehicle
			if (_arm_tune_played) {
				tune_neutral(true);
			}

			_arm_tune_played = false;
		}

		/* play sensor failure tunes if we already waited for hotplug sensors to come up and failed */
		status_flags.condition_system_hotplug_timeout = (hrt_elapsed_time(&_boot_timestamp) > HOTPLUG_SENS_TIMEOUT);

		if (!sensor_fail_tune_played && (!status_flags.condition_system_sensors_initialized
						 && status_flags.condition_system_hotplug_timeout)) {

			set_tune_override(TONE_GPS_WARNING_TUNE);
			sensor_fail_tune_played = true;
			_status_changed = true;
		}

		_counter++;

		int blink_state = blink_msg_state();

		if (blink_state > 0) {
			/* blinking LED message, don't touch LEDs */
			if (blink_state == 2) {
				/* blinking LED message completed, restore normal state */
				control_status_leds(&status, &armed, true, _battery_warning);
			}

		} else {
			/* normal state */
			control_status_leds(&status, &armed, _status_changed, _battery_warning);
		}

		_status_changed = false;

		arm_auth_update(now, params_updated || param_init_forced);

		px4_usleep(COMMANDER_MONITORING_INTERVAL);
	}

	thread_should_exit = true;

	/* wait for threads to complete */
	int ret = pthread_join(commander_low_prio_thread, nullptr);

	if (ret) {
		warn("join failed: %d", ret);
	}

	rgbled_set_color_and_mode(led_control_s::COLOR_WHITE, led_control_s::MODE_OFF);

	/* close fds */
	led_deinit();
	buzzer_deinit();

	thread_running = false;
}

void
Commander::get_circuit_breaker_params()
{
	status_flags.circuit_breaker_engaged_power_check = circuit_breaker_enabled_by_val(_param_cbrk_supply_chk.get(),
			CBRK_SUPPLY_CHK_KEY);
	status_flags.circuit_breaker_engaged_usb_check = circuit_breaker_enabled_by_val(_param_cbrk_usb_chk.get(),
			CBRK_USB_CHK_KEY);
	status_flags.circuit_breaker_engaged_airspd_check = circuit_breaker_enabled_by_val(_param_cbrk_airspd_chk.get(),
			CBRK_AIRSPD_CHK_KEY);
	status_flags.circuit_breaker_engaged_enginefailure_check = circuit_breaker_enabled_by_val(_param_cbrk_enginefail.get(),
			CBRK_ENGINEFAIL_KEY);
	status_flags.circuit_breaker_flight_termination_disabled = circuit_breaker_enabled_by_val(_param_cbrk_flightterm.get(),
			CBRK_FLIGHTTERM_KEY);
	status_flags.circuit_breaker_engaged_posfailure_check = circuit_breaker_enabled_by_val(_param_cbrk_velposerr.get(),
			CBRK_VELPOSERR_KEY);
	status_flags.circuit_breaker_vtol_fw_arming_check = circuit_breaker_enabled_by_val(_param_cbrk_vtolarming.get(),
			CBRK_VTOLARMING_KEY);
}

void
Commander::check_valid(const hrt_abstime &timestamp, const hrt_abstime &timeout, const bool valid_in, bool *valid_out,
		       bool *changed)
{
	hrt_abstime t = hrt_absolute_time();
	bool valid_new = (t < timestamp + timeout && t > timeout && valid_in);

	if (*valid_out != valid_new) {
		*valid_out = valid_new;
		*changed = true;
	}
}

void
Commander::control_status_leds(vehicle_status_s *status_local, const actuator_armed_s *actuator_armed, bool changed,
			       const uint8_t battery_warning)
{
	static hrt_abstime overload_start = 0;

	bool overload = (_cpuload.load > 0.95f) || (_cpuload.ram_usage > 0.98f);

	if (overload_start == 0 && overload) {
		overload_start = hrt_absolute_time();

	} else if (!overload) {
		overload_start = 0;
	}

	/* driving rgbled */
	if (changed || _last_overload != overload) {
		uint8_t led_mode = led_control_s::MODE_OFF;
		uint8_t led_color = led_control_s::COLOR_WHITE;
		bool set_normal_color = false;

		uint64_t overload_warn_delay = (status_local->arming_state == vehicle_status_s::ARMING_STATE_ARMED) ? 1_ms : 250_ms;

		/* set mode */
		if (overload && (hrt_elapsed_time(&overload_start) > overload_warn_delay)) {
			led_mode = led_control_s::MODE_BLINK_FAST;
			led_color = led_control_s::COLOR_PURPLE;

		} else if (status_local->arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
			led_mode = led_control_s::MODE_ON;
			set_normal_color = true;

		} else if (!status_flags.condition_system_sensors_initialized && status_flags.condition_system_hotplug_timeout) {
			led_mode = led_control_s::MODE_BLINK_FAST;
			led_color = led_control_s::COLOR_RED;

		} else if (status_local->arming_state == vehicle_status_s::ARMING_STATE_STANDBY) {
			led_mode = led_control_s::MODE_BREATHE;
			set_normal_color = true;

		} else if (!status_flags.condition_system_sensors_initialized && !status_flags.condition_system_hotplug_timeout) {
			led_mode = led_control_s::MODE_BREATHE;
			set_normal_color = true;

		} else if (status_local->arming_state == vehicle_status_s::ARMING_STATE_INIT) {
			// if in init status it should not be in the error state
			led_mode = led_control_s::MODE_OFF;

		} else {	// STANDBY_ERROR and other states
			led_mode = led_control_s::MODE_BLINK_NORMAL;
			led_color = led_control_s::COLOR_RED;
		}

		if (set_normal_color) {
			/* set color */
			if (status.failsafe) {
				led_color = led_control_s::COLOR_PURPLE;

			} else if (battery_warning == battery_status_s::BATTERY_WARNING_LOW) {
				led_color = led_control_s::COLOR_AMBER;

			} else if (battery_warning == battery_status_s::BATTERY_WARNING_CRITICAL) {
				led_color = led_control_s::COLOR_RED;

			} else {
				if (status_flags.condition_home_position_valid && status_flags.condition_global_position_valid) {
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

	/* board supports HW armed indicator */

	BOARD_INDICATE_ARMED_STATE(actuator_armed->armed);

#if !defined(CONFIG_ARCH_LEDS) && defined(BOARD_HAS_CONTROL_STATUS_LEDS)

	/* this runs at around 20Hz, full cycle is 16 ticks = 10/16Hz */
	if (actuator_armed->armed) {
		if (status.failsafe) {
			BOARD_ARMED_LED_OFF();

			if (_leds_counter % 5 == 0) {
				BOARD_ARMED_STATE_LED_TOGGLE();
			}

		} else {
			BOARD_ARMED_STATE_LED_OFF();

			/* armed, solid */
			BOARD_ARMED_LED_ON();
		}

	} else if (actuator_armed->ready_to_arm) {
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
Commander::set_main_state(const vehicle_status_s &status_local, bool *changed)
{
	if (_safety.override_available && _safety.override_enabled) {
		return set_main_state_override_on(status_local, changed);

	} else {
		return set_main_state_rc(status_local, changed);
	}
}

transition_result_t
Commander::set_main_state_override_on(const vehicle_status_s &status_local, bool *changed)
{
	const transition_result_t res = main_state_transition(status_local, commander_state_s::MAIN_STATE_MANUAL, status_flags,
					&_internal_state);
	*changed = (res == TRANSITION_CHANGED);

	return res;
}

transition_result_t
Commander::set_main_state_rc(const vehicle_status_s &status_local, bool *changed)
{
	/* set main state according to RC switches */
	transition_result_t res = TRANSITION_DENIED;

	// Note: even if status_flags.offboard_control_set_by_command is set
	// we want to allow rc mode change to take precidence.  This is a safety
	// feature, just in case offboard control goes crazy.

	const bool altitude_got_valid = (!_last_condition_local_altitude_valid && status_flags.condition_local_altitude_valid);
	const bool lpos_got_valid = (!_last_condition_local_position_valid && status_flags.condition_local_position_valid);
	const bool gpos_got_valid = (!_last_condition_global_position_valid && status_flags.condition_global_position_valid);
	const bool first_time_rc = (_last_manual_control_setpoint.timestamp == 0);
	const bool rc_values_updated = (_last_manual_control_setpoint.timestamp != _manual_control_setpoint.timestamp);
	const bool some_switch_changed =
		(_last_manual_control_setpoint.offboard_switch != _manual_control_setpoint.offboard_switch)
		|| (_last_manual_control_setpoint.return_switch != _manual_control_setpoint.return_switch)
		|| (_last_manual_control_setpoint.mode_switch != _manual_control_setpoint.mode_switch)
		|| (_last_manual_control_setpoint.acro_switch != _manual_control_setpoint.acro_switch)
		|| (_last_manual_control_setpoint.rattitude_switch != _manual_control_setpoint.rattitude_switch)
		|| (_last_manual_control_setpoint.posctl_switch != _manual_control_setpoint.posctl_switch)
		|| (_last_manual_control_setpoint.loiter_switch != _manual_control_setpoint.loiter_switch)
		|| (_last_manual_control_setpoint.mode_slot != _manual_control_setpoint.mode_slot)
		|| (_last_manual_control_setpoint.stab_switch != _manual_control_setpoint.stab_switch)
		|| (_last_manual_control_setpoint.man_switch != _manual_control_setpoint.man_switch);

	// only switch mode based on RC switch if necessary to also allow mode switching via MAVLink
	const bool should_evaluate_rc_mode_switch = first_time_rc
			|| altitude_got_valid
			|| lpos_got_valid
			|| gpos_got_valid
			|| (rc_values_updated && some_switch_changed);

	if (!should_evaluate_rc_mode_switch) {

		// store the last manual control setpoint set by the pilot in a manual state
		// if the system now later enters an autonomous state the pilot can move
		// the sticks to break out of the autonomous state

		if (!_geofence_warning_action_on
		    && (_internal_state.main_state == commander_state_s::MAIN_STATE_MANUAL ||
			_internal_state.main_state == commander_state_s::MAIN_STATE_ALTCTL ||
			_internal_state.main_state == commander_state_s::MAIN_STATE_POSCTL ||
			_internal_state.main_state == commander_state_s::MAIN_STATE_ACRO ||
			_internal_state.main_state == commander_state_s::MAIN_STATE_RATTITUDE ||
			_internal_state.main_state == commander_state_s::MAIN_STATE_STAB)) {

			_last_manual_control_setpoint.timestamp = _manual_control_setpoint.timestamp;
			_last_manual_control_setpoint.x = _manual_control_setpoint.x;
			_last_manual_control_setpoint.y = _manual_control_setpoint.y;
			_last_manual_control_setpoint.z = _manual_control_setpoint.z;
			_last_manual_control_setpoint.r = _manual_control_setpoint.r;
		}

		/* no timestamp change or no switch change -> nothing changed */
		return TRANSITION_NOT_CHANGED;
	}

	_last_manual_control_setpoint = _manual_control_setpoint;

	// reset the position and velocity validity calculation to give the best change of being able to select
	// the desired mode
	reset_posvel_validity(changed);

	/* offboard switch overrides main switch */
	if (_manual_control_setpoint.offboard_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_OFFBOARD, status_flags, &_internal_state);

		if (res == TRANSITION_DENIED) {
			print_reject_mode("OFFBOARD");
			/* mode rejected, continue to evaluate the main system mode */

		} else {
			/* changed successfully or already in this state */
			return res;
		}
	}

	/* RTL switch overrides main switch */
	if (_manual_control_setpoint.return_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_AUTO_RTL, status_flags, &_internal_state);

		if (res == TRANSITION_DENIED) {
			print_reject_mode("AUTO RTL");

			/* fallback to LOITER if home position not set */
			res = main_state_transition(status_local, commander_state_s::MAIN_STATE_AUTO_LOITER, status_flags, &_internal_state);
		}

		if (res != TRANSITION_DENIED) {
			/* changed successfully or already in this state */
			return res;
		}

		/* if we get here mode was rejected, continue to evaluate the main system mode */
	}

	/* Loiter switch overrides main switch */
	if (_manual_control_setpoint.loiter_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_AUTO_LOITER, status_flags, &_internal_state);

		if (res == TRANSITION_DENIED) {
			print_reject_mode("AUTO HOLD");

		} else {
			return res;
		}
	}

	/* we know something has changed - check if we are in mode slot operation */
	if (_manual_control_setpoint.mode_slot != manual_control_setpoint_s::MODE_SLOT_NONE) {

		if (_manual_control_setpoint.mode_slot > manual_control_setpoint_s::MODE_SLOT_NUM) {
			PX4_WARN("m slot overflow");
			return TRANSITION_DENIED;
		}

		int new_mode = _flight_mode_slots[_manual_control_setpoint.mode_slot - 1];

		if (new_mode < 0) {
			/* slot is unused */
			res = TRANSITION_NOT_CHANGED;

		} else {
			res = main_state_transition(status_local, new_mode, status_flags, &_internal_state);

			/* ensure that the mode selection does not get stuck here */
			int maxcount = 5;

			/* enable the use of break */
			/* fallback strategies, give the user the closest mode to what he wanted */
			while (res == TRANSITION_DENIED && maxcount > 0) {

				maxcount--;

				if (new_mode == commander_state_s::MAIN_STATE_AUTO_MISSION) {

					/* fall back to loiter */
					new_mode = commander_state_s::MAIN_STATE_AUTO_LOITER;
					print_reject_mode("AUTO MISSION");
					res = main_state_transition(status_local, new_mode, status_flags, &_internal_state);

					if (res != TRANSITION_DENIED) {
						break;
					}
				}

				if (new_mode == commander_state_s::MAIN_STATE_AUTO_RTL) {

					/* fall back to position control */
					new_mode = commander_state_s::MAIN_STATE_AUTO_LOITER;
					print_reject_mode("AUTO RTL");
					res = main_state_transition(status_local, new_mode, status_flags, &_internal_state);

					if (res != TRANSITION_DENIED) {
						break;
					}
				}

				if (new_mode == commander_state_s::MAIN_STATE_AUTO_LAND) {

					/* fall back to position control */
					new_mode = commander_state_s::MAIN_STATE_AUTO_LOITER;
					print_reject_mode("AUTO LAND");
					res = main_state_transition(status_local, new_mode, status_flags, &_internal_state);

					if (res != TRANSITION_DENIED) {
						break;
					}
				}

				if (new_mode == commander_state_s::MAIN_STATE_AUTO_TAKEOFF) {

					/* fall back to position control */
					new_mode = commander_state_s::MAIN_STATE_AUTO_LOITER;
					print_reject_mode("AUTO TAKEOFF");
					res = main_state_transition(status_local, new_mode, status_flags, &_internal_state);

					if (res != TRANSITION_DENIED) {
						break;
					}
				}

				if (new_mode == commander_state_s::MAIN_STATE_AUTO_FOLLOW_TARGET) {

					/* fall back to position control */
					new_mode = commander_state_s::MAIN_STATE_AUTO_LOITER;
					print_reject_mode("AUTO FOLLOW");
					res = main_state_transition(status_local, new_mode, status_flags, &_internal_state);

					if (res != TRANSITION_DENIED) {
						break;
					}
				}

				if (new_mode == commander_state_s::MAIN_STATE_AUTO_LOITER) {

					/* fall back to position control */
					new_mode = commander_state_s::MAIN_STATE_POSCTL;
					print_reject_mode("AUTO HOLD");
					res = main_state_transition(status_local, new_mode, status_flags, &_internal_state);

					if (res != TRANSITION_DENIED) {
						break;
					}
				}

				if (new_mode == commander_state_s::MAIN_STATE_POSCTL) {

					/* fall back to altitude control */
					new_mode = commander_state_s::MAIN_STATE_ALTCTL;
					print_reject_mode("POSITION CONTROL");
					res = main_state_transition(status_local, new_mode, status_flags, &_internal_state);

					if (res != TRANSITION_DENIED) {
						break;
					}
				}

				if (new_mode == commander_state_s::MAIN_STATE_ALTCTL) {

					/* fall back to stabilized */
					new_mode = commander_state_s::MAIN_STATE_STAB;
					print_reject_mode("ALTITUDE CONTROL");
					res = main_state_transition(status_local, new_mode, status_flags, &_internal_state);

					if (res != TRANSITION_DENIED) {
						break;
					}
				}

				if (new_mode == commander_state_s::MAIN_STATE_STAB) {

					/* fall back to manual */
					new_mode = commander_state_s::MAIN_STATE_MANUAL;
					print_reject_mode("STABILIZED");
					res = main_state_transition(status_local, new_mode, status_flags, &_internal_state);

					if (res != TRANSITION_DENIED) {
						break;
					}
				}
			}
		}

		return res;
	}

	/* offboard and RTL switches off or denied, check main mode switch */
	switch (_manual_control_setpoint.mode_switch) {
	case manual_control_setpoint_s::SWITCH_POS_NONE:
		res = TRANSITION_NOT_CHANGED;
		break;

	case manual_control_setpoint_s::SWITCH_POS_OFF:		// MANUAL
		if (_manual_control_setpoint.stab_switch == manual_control_setpoint_s::SWITCH_POS_NONE &&
		    _manual_control_setpoint.man_switch == manual_control_setpoint_s::SWITCH_POS_NONE) {
			/*
			 * Legacy mode:
			 * Acro switch being used as stabilized switch in FW.
			 */
			if (_manual_control_setpoint.acro_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
				/* manual mode is stabilized already for multirotors, so switch to acro
				 * for any non-manual mode
				 */
				if (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && !status.is_vtol) {
					res = main_state_transition(status_local, commander_state_s::MAIN_STATE_ACRO, status_flags, &_internal_state);

				} else if (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
					res = main_state_transition(status_local, commander_state_s::MAIN_STATE_STAB, status_flags, &_internal_state);

				} else {
					res = main_state_transition(status_local, commander_state_s::MAIN_STATE_MANUAL, status_flags, &_internal_state);
				}

			} else if (_manual_control_setpoint.rattitude_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
				/* Similar to acro transitions for multirotors.  FW aircraft don't need a
				 * rattitude mode.*/
				if (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
					res = main_state_transition(status_local, commander_state_s::MAIN_STATE_RATTITUDE, status_flags, &_internal_state);

				} else {
					res = main_state_transition(status_local, commander_state_s::MAIN_STATE_STAB, status_flags, &_internal_state);
				}

			} else {
				res = main_state_transition(status_local, commander_state_s::MAIN_STATE_MANUAL, status_flags, &_internal_state);
			}

		} else {
			/* New mode:
			 * - Acro is Acro
			 * - Manual is not default anymore when the manaul switch is assigned
			 */
			if (_manual_control_setpoint.man_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
				res = main_state_transition(status_local, commander_state_s::MAIN_STATE_MANUAL, status_flags, &_internal_state);

			} else if (_manual_control_setpoint.acro_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
				res = main_state_transition(status_local, commander_state_s::MAIN_STATE_ACRO, status_flags, &_internal_state);

			} else if (_manual_control_setpoint.rattitude_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
				res = main_state_transition(status_local, commander_state_s::MAIN_STATE_RATTITUDE, status_flags, &_internal_state);

			} else if (_manual_control_setpoint.stab_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
				res = main_state_transition(status_local, commander_state_s::MAIN_STATE_STAB, status_flags, &_internal_state);

			} else if (_manual_control_setpoint.man_switch == manual_control_setpoint_s::SWITCH_POS_NONE) {
				// default to MANUAL when no manual switch is set
				res = main_state_transition(status_local, commander_state_s::MAIN_STATE_MANUAL, status_flags, &_internal_state);

			} else {
				// default to STAB when the manual switch is assigned (but off)
				res = main_state_transition(status_local, commander_state_s::MAIN_STATE_STAB, status_flags, &_internal_state);
			}
		}

		// TRANSITION_DENIED is not possible here
		break;

	case manual_control_setpoint_s::SWITCH_POS_MIDDLE:		// ASSIST
		if (_manual_control_setpoint.posctl_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
			res = main_state_transition(status_local, commander_state_s::MAIN_STATE_POSCTL, status_flags, &_internal_state);

			if (res != TRANSITION_DENIED) {
				break;	// changed successfully or already in this state
			}

			print_reject_mode("POSITION CONTROL");
		}

		// fallback to ALTCTL
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_ALTCTL, status_flags, &_internal_state);

		if (res != TRANSITION_DENIED) {
			break;	// changed successfully or already in this mode
		}

		if (_manual_control_setpoint.posctl_switch != manual_control_setpoint_s::SWITCH_POS_ON) {
			print_reject_mode("ALTITUDE CONTROL");
		}

		// fallback to MANUAL
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_MANUAL, status_flags, &_internal_state);
		// TRANSITION_DENIED is not possible here
		break;

	case manual_control_setpoint_s::SWITCH_POS_ON:			// AUTO
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_AUTO_MISSION, status_flags, &_internal_state);

		if (res != TRANSITION_DENIED) {
			break;	// changed successfully or already in this state
		}

		print_reject_mode("AUTO MISSION");

		// fallback to LOITER if home position not set
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_AUTO_LOITER, status_flags, &_internal_state);

		if (res != TRANSITION_DENIED) {
			break;  // changed successfully or already in this state
		}

		// fallback to POSCTL
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_POSCTL, status_flags, &_internal_state);

		if (res != TRANSITION_DENIED) {
			break;  // changed successfully or already in this state
		}

		// fallback to ALTCTL
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_ALTCTL, status_flags, &_internal_state);

		if (res != TRANSITION_DENIED) {
			break;	// changed successfully or already in this state
		}

		// fallback to MANUAL
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_MANUAL, status_flags, &_internal_state);
		// TRANSITION_DENIED is not possible here
		break;

	default:
		break;
	}

	return res;
}

void
Commander::reset_posvel_validity(bool *changed)
{
	// reset all the check probation times back to the minimum value
	_gpos_probation_time_us = POSVEL_PROBATION_MIN;
	_lpos_probation_time_us = POSVEL_PROBATION_MIN;
	_lvel_probation_time_us = POSVEL_PROBATION_MIN;

	const vehicle_local_position_s &local_position = _local_position_sub.get();
	const vehicle_global_position_s &global_position = _global_position_sub.get();

	// recheck validity
	if (!_skip_pos_accuracy_check) {
		check_posvel_validity(true, global_position.eph, _eph_threshold_adj, global_position.timestamp,
				      &_last_gpos_fail_time_us, &_gpos_probation_time_us, &status_flags.condition_global_position_valid, changed);
	}

	check_posvel_validity(local_position.xy_valid, local_position.eph, _eph_threshold_adj, local_position.timestamp,
			      &_last_lpos_fail_time_us, &_lpos_probation_time_us, &status_flags.condition_local_position_valid, changed);
	check_posvel_validity(local_position.v_xy_valid, local_position.evh, _param_com_vel_fs_evh.get(),
			      local_position.timestamp,
			      &_last_lvel_fail_time_us, &_lvel_probation_time_us, &status_flags.condition_local_velocity_valid, changed);
}

bool
Commander::check_posvel_validity(const bool data_valid, const float data_accuracy, const float required_accuracy,
				 const hrt_abstime &data_timestamp_us, hrt_abstime *last_fail_time_us, hrt_abstime *probation_time_us, bool *valid_state,
				 bool *validity_changed)
{
	const bool was_valid = *valid_state;
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
		*validity_changed = true;
		*valid_state = valid;
	}

	return valid;
}

void
Commander::update_control_mode()
{
	vehicle_control_mode_s control_mode{};

	control_mode.timestamp = hrt_absolute_time();

	/* set vehicle_control_mode according to set_navigation_state */
	control_mode.flag_armed = armed.armed;
	control_mode.flag_external_manual_override_ok = (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING
			&& !status.is_vtol);

	switch (status.nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		control_mode.flag_control_manual_enabled = true;
		control_mode.flag_control_rates_enabled = stabilization_required();
		control_mode.flag_control_attitude_enabled = stabilization_required();
		break;

	case vehicle_status_s::NAVIGATION_STATE_STAB:
		control_mode.flag_control_manual_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_RATTITUDE:
		control_mode.flag_control_manual_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_rattitude_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
		control_mode.flag_control_manual_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_altitude_enabled = true;
		control_mode.flag_control_climb_rate_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_POSCTL:
		control_mode.flag_control_manual_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_altitude_enabled = true;
		control_mode.flag_control_climb_rate_enabled = true;
		control_mode.flag_control_position_enabled = !status.in_transition_mode;
		control_mode.flag_control_velocity_enabled = !status.in_transition_mode;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
		/* override is not ok for the RTL and recovery mode */
		control_mode.flag_external_manual_override_ok = false;

	/* fallthrough */
	case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
		control_mode.flag_control_auto_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_altitude_enabled = true;
		control_mode.flag_control_climb_rate_enabled = true;
		control_mode.flag_control_position_enabled = !status.in_transition_mode;
		control_mode.flag_control_velocity_enabled = !status.in_transition_mode;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDGPSFAIL:
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_climb_rate_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ACRO:
		control_mode.flag_control_manual_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_DESCEND:
		control_mode.flag_control_auto_enabled = false;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_climb_rate_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
		/* disable all controllers on termination */
		control_mode.flag_control_termination_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_OFFBOARD: {

			const offboard_control_mode_s &offboard_control_mode = _offboard_control_mode_sub.get();

			control_mode.flag_control_offboard_enabled = true;

			/*
			 * The control flags depend on what is ignored according to the offboard control mode topic
			 * Inner loop flags (e.g. attitude) also depend on outer loop ignore flags (e.g. position)
			 */
			control_mode.flag_control_rates_enabled =
				!offboard_control_mode.ignore_bodyrate_x ||
				!offboard_control_mode.ignore_bodyrate_y ||
				!offboard_control_mode.ignore_bodyrate_z ||
				!offboard_control_mode.ignore_attitude ||
				!offboard_control_mode.ignore_position ||
				!offboard_control_mode.ignore_velocity ||
				!offboard_control_mode.ignore_acceleration_force;

			control_mode.flag_control_attitude_enabled = !offboard_control_mode.ignore_attitude ||
					!offboard_control_mode.ignore_position ||
					!offboard_control_mode.ignore_velocity ||
					!offboard_control_mode.ignore_acceleration_force;

			// TO-DO: Add support for other modes than yawrate control
			control_mode.flag_control_yawrate_override_enabled =
				offboard_control_mode.ignore_bodyrate_x &&
				offboard_control_mode.ignore_bodyrate_y &&
				!offboard_control_mode.ignore_bodyrate_z &&
				!offboard_control_mode.ignore_attitude;

			control_mode.flag_control_rattitude_enabled = false;

			control_mode.flag_control_acceleration_enabled = !offboard_control_mode.ignore_acceleration_force &&
					!status.in_transition_mode;

			control_mode.flag_control_velocity_enabled = (!offboard_control_mode.ignore_velocity ||
					!offboard_control_mode.ignore_position) && !status.in_transition_mode &&
					!control_mode.flag_control_acceleration_enabled;

			control_mode.flag_control_climb_rate_enabled = (!offboard_control_mode.ignore_velocity ||
					!offboard_control_mode.ignore_position) && !control_mode.flag_control_acceleration_enabled;

			control_mode.flag_control_position_enabled = !offboard_control_mode.ignore_position && !status.in_transition_mode &&
					!control_mode.flag_control_acceleration_enabled;

			control_mode.flag_control_altitude_enabled = (!offboard_control_mode.ignore_velocity ||
					!offboard_control_mode.ignore_position) && !control_mode.flag_control_acceleration_enabled;

		}
		break;

	case vehicle_status_s::NAVIGATION_STATE_ORBIT:
		control_mode.flag_control_manual_enabled = false;
		control_mode.flag_control_auto_enabled = false;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_rattitude_enabled = false;
		control_mode.flag_control_altitude_enabled = true;
		control_mode.flag_control_climb_rate_enabled = true;
		control_mode.flag_control_position_enabled = !status.in_transition_mode;
		control_mode.flag_control_velocity_enabled = !status.in_transition_mode;
		control_mode.flag_control_acceleration_enabled = false;
		control_mode.flag_control_termination_enabled = false;
		break;

	default:
		break;
	}

	_control_mode_pub.publish(control_mode);
}

bool
Commander::stabilization_required()
{
	return (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING ||		// is a rotary wing, or
		status.vtol_fw_permanent_stab || 	// is a VTOL in fixed wing mode and stabilisation is on, or
		(_vtol_status.vtol_in_trans_mode && 	// is currently a VTOL transitioning AND
		 status.vehicle_type ==
		 vehicle_status_s::VEHICLE_TYPE_FIXED_WING));	// is a fixed wing, ie: transitioning back to rotary wing mode
}

void
Commander::print_reject_mode(const char *msg)
{
	const hrt_abstime t = hrt_absolute_time();

	if (t - _last_print_mode_reject_time > PRINT_MODE_REJECT_INTERVAL) {
		_last_print_mode_reject_time = t;
		mavlink_log_critical(&mavlink_log_pub, "REJECT %s", msg);

		/* only buzz if armed, because else we're driving people nuts indoors
		they really need to look at the leds as well. */
		tune_negative(armed.armed);
	}
}

void
Commander::print_reject_arm(const char *msg)
{
	const hrt_abstime t = hrt_absolute_time();

	if (t - _last_print_mode_reject_time > PRINT_MODE_REJECT_INTERVAL) {
		_last_print_mode_reject_time = t;
		mavlink_log_critical(&mavlink_log_pub, "%s", msg);
		tune_negative(true);
	}
}

void answer_command(const vehicle_command_s &cmd, unsigned result,
		    uORB::PublicationQueued<vehicle_command_ack_s> &command_ack_pub)
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

	command_ack.timestamp = hrt_absolute_time();
	command_ack.command = cmd.command;
	command_ack.result = (uint8_t)result;
	command_ack.target_system = cmd.source_system;
	command_ack.target_component = cmd.source_component;


	command_ack_pub.publish(command_ack);
}

void *commander_low_prio_loop(void *arg)
{
	/* Set thread name */
	px4_prctl(PR_SET_NAME, "commander_low_prio", px4_getpid());

	/* Subscribe to command topic */
	int cmd_sub = orb_subscribe(ORB_ID(vehicle_command));

	/* command ack */
	uORB::PublicationQueued<vehicle_command_ack_s> command_ack_pub{ORB_ID(vehicle_command_ack)};

	/* wakeup source(s) */
	px4_pollfd_struct_t fds[1];

	fds[0].fd = cmd_sub;
	fds[0].events = POLLIN;

	while (!thread_should_exit) {
		/* wait for up to 1000ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret < 0) {
			/* this is undesirable but not much we can do - might want to flag unhappy status */
			warn("commander: poll error %d, %d", pret, errno);
			continue;

		} else if (pret != 0) {
			struct vehicle_command_s cmd {};

			/* if we reach here, we have a valid command */
			orb_copy(ORB_ID(vehicle_command), cmd_sub, &cmd);

			/* only handle low-priority commands here */
			switch (cmd.command) {
			case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION: {

					if ((status.arming_state == vehicle_status_s::ARMING_STATE_ARMED)
					    || status.arming_state == vehicle_status_s::ARMING_STATE_SHUTDOWN) {

						// reject if armed or shutting down
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED, command_ack_pub);

					} else {

						int calib_ret = PX4_ERROR;

						/* try to go to INIT/PREFLIGHT arming state */
						if (TRANSITION_DENIED == arming_state_transition(&status, safety_s{}, vehicle_status_s::ARMING_STATE_INIT, &armed,
								false /* fRunPreArmChecks */, &mavlink_log_pub, &status_flags,
								PreFlightCheck::arm_requirements_t{}, // arming requirements not relevant for switching to ARMING_STATE_INIT
								30_s, // time since boot not relevant for switching to ARMING_STATE_INIT
								(cmd.from_external ? arm_disarm_reason_t::COMMAND_EXTERNAL : arm_disarm_reason_t::COMMAND_INTERNAL))
						   ) {

							answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_DENIED, command_ack_pub);
							break;

						} else {
							status_flags.condition_calibration_enabled = true;
						}

						if ((int)(cmd.param1) == 1) {
							/* gyro calibration */
							answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
							calib_ret = do_gyro_calibration(&mavlink_log_pub);

						} else if ((int)(cmd.param1) == vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION ||
							   (int)(cmd.param5) == vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION ||
							   (int)(cmd.param7) == vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION) {
							/* temperature calibration: handled in events module */
							break;

						} else if ((int)(cmd.param2) == 1) {
							/* magnetometer calibration */
							answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
							calib_ret = do_mag_calibration(&mavlink_log_pub);

						} else if ((int)(cmd.param2) == 2) {
							/* magnetometer calibration quick (requires GPS & attitude) */
							answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
							calib_ret = do_mag_calibration_quick(&mavlink_log_pub);

						} else if ((int)(cmd.param3) == 1) {
							/* zero-altitude pressure calibration */
							answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_DENIED, command_ack_pub);

						} else if ((int)(cmd.param4) == 1) {
							/* RC calibration */
							answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
							/* disable RC control input completely */
							status_flags.rc_input_blocked = true;
							calib_ret = OK;
							mavlink_log_info(&mavlink_log_pub, "Calibration: Disabling RC input");

						} else if ((int)(cmd.param4) == 2) {
							/* RC trim calibration */
							answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
							calib_ret = do_trim_calibration(&mavlink_log_pub);

						} else if ((int)(cmd.param5) == 1) {
							/* accelerometer calibration */
							answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
							calib_ret = do_accel_calibration(&mavlink_log_pub);

						} else if ((int)(cmd.param5) == 2) {
							// board offset calibration
							answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
							calib_ret = do_level_calibration(&mavlink_log_pub);

						} else if ((int)(cmd.param5) == 4) {
							// accelerometer quick calibration
							answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
							calib_ret = do_accel_calibration_quick(&mavlink_log_pub);

						} else if ((int)(cmd.param6) == 1 || (int)(cmd.param6) == 2) {
							// TODO: param6 == 1 is deprecated, but we still accept it for a while (feb 2017)
							/* airspeed calibration */
							answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
							calib_ret = do_airspeed_calibration(&mavlink_log_pub);

						} else if ((int)(cmd.param7) == 1) {
							/* do esc calibration */
							answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
							calib_ret = do_esc_calibration(&mavlink_log_pub, &armed);

						} else if ((int)(cmd.param4) == 0) {
							/* RC calibration ended - have we been in one worth confirming? */
							if (status_flags.rc_input_blocked) {
								/* enable RC control input */
								status_flags.rc_input_blocked = false;
								mavlink_log_info(&mavlink_log_pub, "Calibration: Restoring RC input");
							}

							answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
							/* this always succeeds */
							calib_ret = OK;

						} else {
							answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_UNSUPPORTED, command_ack_pub);
						}

						status_flags.condition_calibration_enabled = false;

						if (calib_ret == OK) {
							tune_positive(true);

							// time since boot not relevant here
							if (PreFlightCheck::preflightCheck(&mavlink_log_pub, status, status_flags, false, false, true, 30_s)) {

								status_flags.condition_system_sensors_initialized = true;
							}

							arming_state_transition(&status, safety_s{}, vehicle_status_s::ARMING_STATE_STANDBY, &armed,
										false /* fRunPreArmChecks */,
										&mavlink_log_pub, &status_flags,
										PreFlightCheck::arm_requirements_t{}, // arming requirements not relevant for switching to ARMING_STATE_STANDBY
										30_s, // time since boot not relevant for switching to ARMING_STATE_STANDBY
										(cmd.from_external ? arm_disarm_reason_t::COMMAND_EXTERNAL : arm_disarm_reason_t::COMMAND_INTERNAL));

						} else {
							tune_negative(true);
						}
					}

					break;
				}

			case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_STORAGE: {

					if ((status.arming_state == vehicle_status_s::ARMING_STATE_ARMED)
					    || status.arming_state == vehicle_status_s::ARMING_STATE_SHUTDOWN) {

						// reject if armed or shutting down
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED, command_ack_pub);

					} else {

						if (((int)(cmd.param1)) == 0) {
							int ret = param_load_default();

							if (ret == OK) {
								mavlink_log_info(&mavlink_log_pub, "Settings loaded");
								answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);

							} else {
								mavlink_log_critical(&mavlink_log_pub, "Error loading settings");

								/* convenience as many parts of NuttX use negative errno */
								if (ret < 0) {
									ret = -ret;
								}

								if (ret < 1000) {
									mavlink_log_critical(&mavlink_log_pub, "Error: %s", strerror(ret));
								}

								answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_FAILED, command_ack_pub);
							}

						} else if (((int)(cmd.param1)) == 1) {

							int ret = param_save_default();

							if (ret == OK) {
								/* do not spam MAVLink, but provide the answer / green led mechanism */
								answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);

							} else {
								mavlink_log_critical(&mavlink_log_pub, "Error saving settings");

								/* convenience as many parts of NuttX use negative errno */
								if (ret < 0) {
									ret = -ret;
								}

								if (ret < 1000) {
									mavlink_log_critical(&mavlink_log_pub, "Error: %s", strerror(ret));
								}

								answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_FAILED, command_ack_pub);
							}

						} else if (((int)(cmd.param1)) == 2) {

							/* reset parameters and save empty file */
							param_reset_all();

							/* do not spam MAVLink, but provide the answer / green led mechanism */
							mavlink_log_critical(&mavlink_log_pub, "Onboard parameters reset");
							answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
						}
					}

					break;
				}

			case vehicle_command_s::VEHICLE_CMD_START_RX_PAIR:
				/* just ack, implementation handled in the IO driver */
				answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
				break;

			default:
				/* don't answer on unsupported commands, it will be done in main loop */
				break;
			}
		}
	}

	px4_close(cmd_sub);

	return nullptr;
}

int Commander::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
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
	status.hil_state = vehicle_status_s::HIL_STATE_ON;
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
	for (auto &telemetry_status :  _telemetry_status_sub) {
		telemetry_status_s telemetry;

		if (telemetry_status.update(&telemetry)) {

			// handle different radio types
			switch (telemetry.type) {
			case telemetry_status_s::LINK_TYPE_USB:
				// set (but don't unset) telemetry via USB as active once a MAVLink connection is up
				status_flags.usb_connected = true;
				break;

			case telemetry_status_s::LINK_TYPE_IRIDIUM: {
					iridiumsbd_status_s iridium_status;

					if (_iridiumsbd_status_sub.update(&iridium_status)) {
						_high_latency_datalink_heartbeat = iridium_status.last_heartbeat;

						if (status.high_latency_data_link_lost) {
							if (hrt_elapsed_time(&_high_latency_datalink_lost) > (_param_com_hldl_reg_t.get() * 1_s)) {
								status.high_latency_data_link_lost = false;
								_status_changed = true;
							}
						}
					}

					break;
				}
			}

			for (const auto &hb : telemetry.heartbeats) {
				// handle different remote types
				switch (hb.type) {
				case telemetry_heartbeat_s::TYPE_GCS:

					// Initial connection or recovery from data link lost
					if (status.data_link_lost) {
						if (hb.timestamp > _datalink_last_heartbeat_gcs) {
							status.data_link_lost = false;
							_status_changed = true;

							if (!armed.armed && !status_flags.condition_calibration_enabled) {
								// make sure to report preflight check failures to a connecting GCS
								PreFlightCheck::preflightCheck(&mavlink_log_pub, status, status_flags,
											       _arm_requirements.global_position, true, true, hrt_elapsed_time(&_boot_timestamp));
							}

							if (_datalink_last_heartbeat_gcs != 0) {
								mavlink_log_info(&mavlink_log_pub, "Data link regained");
							}
						}
					}

					// Only keep the very last heartbeat timestamp, so we don't get confused
					// by multiple mavlink instances publishing different timestamps.
					if (hb.timestamp > _datalink_last_heartbeat_gcs) {
						_datalink_last_heartbeat_gcs = hb.timestamp;
					}

					break;

				case telemetry_heartbeat_s::TYPE_ONBOARD_CONTROLLER:
					if (_onboard_controller_lost) {
						if (hb.timestamp > _datalink_last_heartbeat_onboard_controller) {
							mavlink_log_info(&mavlink_log_pub, "Onboard controller regained");
							_onboard_controller_lost = false;
							_status_changed = true;
						}
					}

					_datalink_last_heartbeat_onboard_controller = hb.timestamp;

					if (hb.component_id == telemetry_heartbeat_s::COMP_ID_OBSTACLE_AVOIDANCE) {
						if (hb.timestamp > _datalink_last_heartbeat_avoidance_system) {
							_avoidance_system_status_change = (_datalink_last_status_avoidance_system != hb.state);
						}

						_datalink_last_heartbeat_avoidance_system = hb.timestamp;
						_datalink_last_status_avoidance_system = hb.state;

						if (_avoidance_system_lost) {
							_status_changed = true;
							_avoidance_system_lost = false;
							status_flags.avoidance_system_valid = true;
						}
					}

					break;
				}
			}
		}
	}


	// GCS data link loss failsafe
	if (!status.data_link_lost) {
		if ((_datalink_last_heartbeat_gcs != 0)
		    && hrt_elapsed_time(&_datalink_last_heartbeat_gcs) > (_param_com_dl_loss_t.get() * 1_s)) {

			status.data_link_lost = true;
			status.data_link_lost_counter++;

			mavlink_log_critical(&mavlink_log_pub, "Connection to ground station lost");

			_status_changed = true;
		}
	}

	// ONBOARD CONTROLLER data link loss failsafe (hard coded 5 seconds)
	if ((_datalink_last_heartbeat_onboard_controller > 0)
	    && (hrt_elapsed_time(&_datalink_last_heartbeat_onboard_controller) > 5_s)
	    && !_onboard_controller_lost) {

		mavlink_log_critical(&mavlink_log_pub, "Connection to mission computer lost");
		_onboard_controller_lost = true;
		_status_changed = true;
	}

	// AVOIDANCE SYSTEM state check (only if it is enabled)
	if (status_flags.avoidance_system_required && !_onboard_controller_lost) {

		//if heartbeats stop
		if (!_avoidance_system_lost && (_datalink_last_heartbeat_avoidance_system > 0)
		    && (hrt_elapsed_time(&_datalink_last_heartbeat_avoidance_system) > 5_s)) {

			_avoidance_system_lost = true;
			status_flags.avoidance_system_valid = false;
		}

		//if status changed
		if (_avoidance_system_status_change) {
			if (_datalink_last_status_avoidance_system == telemetry_heartbeat_s::STATE_BOOT) {
				status_flags.avoidance_system_valid = false;
			}

			if (_datalink_last_status_avoidance_system == telemetry_heartbeat_s::STATE_ACTIVE) {
				status_flags.avoidance_system_valid = true;
			}

			if (_datalink_last_status_avoidance_system == telemetry_heartbeat_s::STATE_CRITICAL) {
				status_flags.avoidance_system_valid = false;
				_status_changed = true;
			}

			if (_datalink_last_status_avoidance_system == telemetry_heartbeat_s::STATE_FLIGHT_TERMINATION) {
				status_flags.avoidance_system_valid = false;
				_status_changed = true;
			}

			_avoidance_system_status_change = false;
		}
	}


	// high latency data link loss failsafe
	if (_high_latency_datalink_heartbeat > 0
	    && hrt_elapsed_time(&_high_latency_datalink_heartbeat) > (_param_com_hldl_loss_t.get() * 1_s)) {
		_high_latency_datalink_lost = hrt_absolute_time();

		if (!status.high_latency_data_link_lost) {
			status.high_latency_data_link_lost = true;
			mavlink_log_critical(&mavlink_log_pub, "High latency data link lost");
			_status_changed = true;
		}
	}
}

void Commander::avoidance_check()
{

	for (unsigned i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
		if (_sub_distance_sensor[i].updated()) {
			distance_sensor_s distance_sensor {};
			_sub_distance_sensor[i].copy(&distance_sensor);

			if ((distance_sensor.orientation != distance_sensor_s::ROTATION_DOWNWARD_FACING) &&
			    (distance_sensor.orientation != distance_sensor_s::ROTATION_UPWARD_FACING)) {
				_valid_distance_sensor_time_us = distance_sensor.timestamp;
			}
		}
	}

	const bool cp_enabled =  _param_cp_dist.get() > 0.f;

	const bool distance_sensor_valid = hrt_elapsed_time(&_valid_distance_sensor_time_us) < 500_ms;
	const bool cp_healthy = status_flags.avoidance_system_valid || distance_sensor_valid;

	const bool sensor_oa_present = cp_healthy || status_flags.avoidance_system_required || cp_enabled;

	const bool auto_mode = _internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_MISSION
			       || _internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_LOITER
			       || _internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_RTL
			       || _internal_state.main_state == commander_state_s::MAIN_STATE_OFFBOARD
			       || _internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_TAKEOFF
			       || _internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_LAND;
	const bool pos_ctl_mode = _internal_state.main_state == commander_state_s::MAIN_STATE_POSCTL;

	const bool sensor_oa_enabled = ((auto_mode && status_flags.avoidance_system_required) || (pos_ctl_mode && cp_enabled));
	const bool sensor_oa_healthy = ((auto_mode && status_flags.avoidance_system_valid) || (pos_ctl_mode && cp_healthy));

	set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_OBSTACLE_AVOIDANCE, sensor_oa_present, sensor_oa_enabled,
			 sensor_oa_healthy, status);
}

void Commander::battery_status_check()
{
	bool battery_sub_updated = false;

	battery_status_s batteries[ORB_MULTI_MAX_INSTANCES];
	size_t num_connected_batteries = 0;

	for (size_t i = 0; i < sizeof(_battery_subs) / sizeof(_battery_subs[0]); i++) {
		// We need to update the status flag if ANY battery is updated, because the system source might have
		// changed, or might be nothing (if there is no battery connected)
		battery_sub_updated |= _battery_subs[i].updated();

		if (_battery_subs[i].copy(&batteries[num_connected_batteries])) {
			if (batteries[num_connected_batteries].connected) {
				num_connected_batteries++;
			}
		}
	}

	if (!battery_sub_updated) {
		// Nothing has changed since the last time this function was called, so nothing needs to be done now.
		return;
	}

	// There are possibly multiple batteries, and we can't know which ones serve which purpose. So the safest
	// option is to check if ANY of them have a warning, and specifically find which one has the most
	// urgent warning.
	uint8_t worst_warning = battery_status_s::BATTERY_WARNING_NONE;
	// To make sure that all connected batteries are being regularly reported, we check which one has the
	// oldest timestamp.
	hrt_abstime oldest_update = hrt_absolute_time();

	_battery_current = 0.0f;

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
	}

	bool battery_warning_level_increased_while_armed = false;
	bool update_internal_battery_state = false;

	if (armed.armed) {
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

	status_flags.condition_battery_healthy =
		// All connected batteries are regularly being published
		(hrt_elapsed_time(&oldest_update) < 5_s)
		// There is at least one connected battery (in any slot)
		&& (num_connected_batteries > 0)
		// No currently-connected batteries have any warning
		&& (_battery_warning == battery_status_s::BATTERY_WARNING_NONE);

	// execute battery failsafe if the state has gotten worse while we are armed
	if (battery_warning_level_increased_while_armed) {
		battery_failsafe(&mavlink_log_pub, status, status_flags, &_internal_state, _battery_warning,
				 (low_battery_action_t)_param_com_low_bat_act.get());
	}

	// Handle shutdown request from emergency battery action
	if (update_internal_battery_state) {

		if (_battery_warning == battery_status_s::BATTERY_WARNING_EMERGENCY) {
#if defined(CONFIG_BOARDCTL_POWEROFF)

			if (shutdown_if_allowed() && (px4_shutdown_request(400_ms) == 0)) {
				mavlink_log_critical(&mavlink_log_pub, "Dangerously low battery! Shutting system down");

				while (1) { px4_usleep(1); }

			} else {
				mavlink_log_critical(&mavlink_log_pub, "System does not support shutdown");
			}

#endif // CONFIG_BOARDCTL_POWEROFF
		}
	}
}

void Commander::estimator_check(const vehicle_status_flags_s &vstatus_flags)
{
	// Check if quality checking of position accuracy and consistency is to be performed
	const bool run_quality_checks = !status_flags.circuit_breaker_engaged_posfailure_check;

	_local_position_sub.update();
	_global_position_sub.update();

	const vehicle_local_position_s &lpos = _local_position_sub.get();
	const vehicle_global_position_s &gpos = _global_position_sub.get();

	if (lpos.heading_reset_counter != _heading_reset_counter) {
		if (vstatus_flags.condition_home_position_valid) {
			updateHomePositionYaw(_home_pub.get().yaw + lpos.delta_heading);
		}

		_heading_reset_counter = lpos.heading_reset_counter;
	}

	const bool mag_fault_prev = (_estimator_status_sub.get().control_mode_flags & (1 << estimator_status_s::CS_MAG_FAULT));

	if (_estimator_status_sub.update()) {
		const estimator_status_s &estimator_status = _estimator_status_sub.get();

		// Check for a magnetomer fault and notify the user
		const bool mag_fault = (estimator_status.control_mode_flags & (1 << estimator_status_s::CS_MAG_FAULT));

		if (!mag_fault_prev && mag_fault) {
			mavlink_log_critical(&mavlink_log_pub, "Stopping compass use! Check calibration on landing");
		}

		// Set the allowable position uncertainty based on combination of flight and estimator state
		// When we are in a operator demanded position control mode and are solely reliant on optical flow, do not check position error because it will gradually increase throughout flight and the operator will compensate for the drift
		const bool reliant_on_opt_flow = ((estimator_status.control_mode_flags & (1 << estimator_status_s::CS_OPT_FLOW))
						  && !(estimator_status.control_mode_flags & (1 << estimator_status_s::CS_GPS))
						  && !(estimator_status.control_mode_flags & (1 << estimator_status_s::CS_EV_POS)));

		const bool operator_controlled_position = (_internal_state.main_state == commander_state_s::MAIN_STATE_POSCTL);

		_skip_pos_accuracy_check = reliant_on_opt_flow && operator_controlled_position;

		if (_skip_pos_accuracy_check) {
			_eph_threshold_adj = INFINITY;

		} else {
			_eph_threshold_adj = _param_com_pos_fs_eph.get();
		}

		/* Check estimator status for signs of bad yaw induced post takeoff navigation failure
		 * for a short time interval after takeoff. Fixed wing vehicles can recover using GPS heading,
		 * but rotary wing vehicles cannot so the position and velocity validity needs to be latched
		 * to false after failure to prevent flyaway crashes */
		if (run_quality_checks && status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {

			if (status.arming_state == vehicle_status_s::ARMING_STATE_STANDBY) {
				// reset flags and timer
				_time_at_takeoff = hrt_absolute_time();
				_nav_test_failed = false;
				_nav_test_passed = false;

			} else if (_land_detector.landed) {
				// record time of takeoff
				_time_at_takeoff = hrt_absolute_time();

			} else {
				// if nav status is unconfirmed, confirm yaw angle as passed after 30 seconds or achieving 5 m/s of speed
				const bool sufficient_time = (hrt_elapsed_time(&_time_at_takeoff) > 30_s);
				const bool sufficient_speed = (lpos.vx * lpos.vx + lpos.vy * lpos.vy > 25.0f);

				bool innovation_pass = estimator_status.vel_test_ratio < 1.0f && estimator_status.pos_test_ratio < 1.0f;

				if (!_nav_test_failed) {
					if (!_nav_test_passed) {
						// pass if sufficient time or speed
						if (sufficient_time || sufficient_speed) {
							_nav_test_passed = true;
						}

						// record the last time the innovation check passed
						if (innovation_pass) {
							_time_last_innov_pass = hrt_absolute_time();
						}

						// if the innovation test has failed continuously, declare the nav as failed
						if (hrt_elapsed_time(&_time_last_innov_pass) > 1_s) {
							_nav_test_failed = true;
							mavlink_log_emergency(&mavlink_log_pub, "Critical navigation failure! Check sensor calibration");
						}
					}
				}
			}
		}
	}

	/* run global position accuracy checks */
	// Check if quality checking of position accuracy and consistency is to be performed
	if (run_quality_checks) {
		if (_nav_test_failed) {
			status_flags.condition_global_position_valid = false;
			status_flags.condition_local_position_valid = false;
			status_flags.condition_local_velocity_valid = false;

		} else {
			if (!_skip_pos_accuracy_check) {
				// use global position message to determine validity
				check_posvel_validity(true, gpos.eph, _eph_threshold_adj, gpos.timestamp, &_last_gpos_fail_time_us,
						      &_gpos_probation_time_us, &status_flags.condition_global_position_valid, &_status_changed);
			}

			// use local position message to determine validity
			check_posvel_validity(lpos.xy_valid, lpos.eph, _eph_threshold_adj, lpos.timestamp, &_last_lpos_fail_time_us,
					      &_lpos_probation_time_us, &status_flags.condition_local_position_valid, &_status_changed);

			check_posvel_validity(lpos.v_xy_valid, lpos.evh, _param_com_vel_fs_evh.get(), lpos.timestamp, &_last_lvel_fail_time_us,
					      &_lvel_probation_time_us, &status_flags.condition_local_velocity_valid, &_status_changed);
		}
	}

	check_valid(lpos.timestamp, _param_com_pos_fs_delay.get() * 1_s, lpos.z_valid,
		    &(status_flags.condition_local_altitude_valid), &_status_changed);
}

void
Commander::offboard_control_update()
{
	const offboard_control_mode_s &offboard_control_mode = _offboard_control_mode_sub.get();

	if (_offboard_control_mode_sub.updated()) {
		const offboard_control_mode_s old = offboard_control_mode;

		if (_offboard_control_mode_sub.update()) {
			if (old.ignore_thrust != offboard_control_mode.ignore_thrust ||
			    old.ignore_attitude != offboard_control_mode.ignore_attitude ||
			    old.ignore_bodyrate_x != offboard_control_mode.ignore_bodyrate_x ||
			    old.ignore_bodyrate_y != offboard_control_mode.ignore_bodyrate_y ||
			    old.ignore_bodyrate_z != offboard_control_mode.ignore_bodyrate_z ||
			    old.ignore_position != offboard_control_mode.ignore_position ||
			    old.ignore_velocity != offboard_control_mode.ignore_velocity ||
			    old.ignore_acceleration_force != offboard_control_mode.ignore_acceleration_force ||
			    old.ignore_alt_hold != offboard_control_mode.ignore_alt_hold) {

				_status_changed = true;
			}
		}
	}

	_offboard_available.set_state_and_update(
		hrt_elapsed_time(&offboard_control_mode.timestamp) < _param_com_of_loss_t.get() * 1e6f,
		hrt_absolute_time());

	const bool offboard_lost = !_offboard_available.get_state();

	if (status_flags.offboard_control_signal_lost != offboard_lost) {
		status_flags.offboard_control_signal_lost = offboard_lost;
		_status_changed = true;
	}
}

void Commander::esc_status_check(const esc_status_s &esc_status)
{
	char esc_fail_msg[50];
	esc_fail_msg[0] = '\0';

	int online_bitmask = (1 << esc_status.esc_count) - 1;

	// Check if ALL the ESCs are online
	if (online_bitmask == esc_status.esc_online_flags) {
		status_flags.condition_escs_error = false;
		_last_esc_online_flags = esc_status.esc_online_flags;

	} else if (_last_esc_online_flags == esc_status.esc_online_flags || esc_status.esc_count == 0)  {

		// Avoid checking the status if the flags are the same or if the mixer has not yet been loaded in the ESC driver

		status_flags.condition_escs_error = true;

	} else if (esc_status.esc_online_flags < _last_esc_online_flags) {

		// Only warn the user when an ESC goes from ONLINE to OFFLINE. This is done to prevent showing Offline ESCs warnings at boot

		for (int index = 0; index < esc_status.esc_count; index++) {
			if ((esc_status.esc_online_flags & (1 << index)) == 0) {
				snprintf(esc_fail_msg + strlen(esc_fail_msg), sizeof(esc_fail_msg) - strlen(esc_fail_msg), "ESC%d ", index + 1);
				esc_fail_msg[sizeof(esc_fail_msg) - 1] = '\0';
			}
		}

		mavlink_log_critical(&mavlink_log_pub, "%soffline", esc_fail_msg);

		_last_esc_online_flags = esc_status.esc_online_flags;
		status_flags.condition_escs_error = true;
	}
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
	PRINT_MODULE_USAGE_ARG("manual|acro|offboard|stabilized|rattitude|altctl|posctl|auto:mission|auto:loiter|auto:rtl|auto:takeoff|auto:land|auto:precland",
			"Flight mode", false);
	PRINT_MODULE_USAGE_COMMAND("lockdown");
	PRINT_MODULE_USAGE_ARG("off", "Turn lockdown off", true);
#endif
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 1;
}
