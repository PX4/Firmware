/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file state_machine_helper.cpp
 * State machine helper functions implementations
 *
 * @author Thomas Gubler	<thomas@px4.io>
 * @author Julian Oes		<julian@oes.ch>
 * @author Sander Smeets	<sander@droneslab.com>
 */
#include <px4_platform_common/px4_config.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_status.h>
#include <systemlib/mavlink_log.h>
#include <drivers/drv_hrt.h>
#include <float.h>

#include "state_machine_helper.h"
#include "commander_helper.h"

using namespace time_literals;

static constexpr const char reason_no_rc[] = "No manual control stick input";
static constexpr const char reason_no_offboard[] = "no offboard";
static constexpr const char reason_no_rc_and_no_offboard[] = "no RC and no offboard";
static constexpr const char reason_no_local_position[] = "no local position";
static constexpr const char reason_no_global_position[] = "no global position";
static constexpr const char reason_no_datalink[] = "no datalink";
static constexpr const char reason_no_rc_and_no_datalink[] = "no RC and no datalink";

// This array defines the arming state transitions. The rows are the new state, and the columns
// are the current state. Using new state and current state you can index into the array which
// will be true for a valid transition or false for a invalid transition. In some cases even
// though the transition is marked as true additional checks must be made. See arming_state_transition
// code for those checks.
static constexpr const bool arming_transitions[vehicle_status_s::ARMING_STATE_MAX][vehicle_status_s::ARMING_STATE_MAX]
= {
	//                                                    INIT,  STANDBY, ARMED, STANDBY_ERROR, SHUTDOWN, IN_AIR_RESTORE
	{ /* vehicle_status_s::ARMING_STATE_INIT */           true,  true,    false, true,          false,    false },
	{ /* vehicle_status_s::ARMING_STATE_STANDBY */        true,  true,    true,  false,         false,    false },
	{ /* vehicle_status_s::ARMING_STATE_ARMED */          false, true,    true,  false,         false,    true },
	{ /* vehicle_status_s::ARMING_STATE_STANDBY_ERROR */  true,  true,    true,  true,          false,    false },
	{ /* vehicle_status_s::ARMING_STATE_SHUTDOWN */       true,  true,    false, true,          true,     true },
	{ /* vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE */ false, false,   false, false,         false,    false }, // NYI
};

// You can index into the array with an arming_state_t in order to get its textual representation
const char *const arming_state_names[vehicle_status_s::ARMING_STATE_MAX] = {
	"INIT",
	"STANDBY",
	"ARMED",
	"STANDBY_ERROR",
	"SHUTDOWN",
	"IN_AIR_RESTORE",
};

// You can index into the array with an navigation_state_t in order to get its textual representation
const char *const nav_state_names[vehicle_status_s::NAVIGATION_STATE_MAX] = {
	"MANUAL",
	"ALTCTL",
	"POSCTL",
	"AUTO_MISSION",
	"AUTO_LOITER",
	"AUTO_RTL",
	"6: unallocated",
	"7: unallocated",
	"AUTO_LANDENGFAIL",
	"AUTO_LANDGPSFAIL",
	"ACRO",
	"11: UNUSED",
	"DESCEND",
	"TERMINATION",
	"OFFBOARD",
	"STAB",
	"16: UNUSED2",
	"AUTO_TAKEOFF",
	"AUTO_LAND",
	"AUTO_FOLLOW_TARGET",
	"AUTO_PRECLAND",
	"ORBIT"
};

static hrt_abstime last_preflight_check = 0;	///< initialize so it gets checked immediately

void set_link_loss_nav_state(vehicle_status_s &status, actuator_armed_s &armed,
			     const vehicle_status_flags_s &status_flags, commander_state_s &internal_state, link_loss_actions_t link_loss_act,
			     const float ll_delay);

void reset_link_loss_globals(actuator_armed_s &armed, const bool old_failsafe, const link_loss_actions_t link_loss_act);

void set_offboard_loss_nav_state(vehicle_status_s &status, actuator_armed_s &armed,
				 const vehicle_status_flags_s &status_flags,
				 const offboard_loss_actions_t offboard_loss_act);

void set_offboard_loss_rc_nav_state(vehicle_status_s &status, actuator_armed_s &armed,
				    const vehicle_status_flags_s &status_flags,
				    const offboard_loss_rc_actions_t offboard_loss_rc_act);

void reset_offboard_loss_globals(actuator_armed_s &armed, const bool old_failsafe,
				 const offboard_loss_actions_t offboard_loss_act,
				 const offboard_loss_rc_actions_t offboard_loss_rc_act);

transition_result_t arming_state_transition(vehicle_status_s &status, const safety_s &safety,
		const arming_state_t new_arming_state, actuator_armed_s &armed, const bool fRunPreArmChecks,
		orb_advert_t *mavlink_log_pub, vehicle_status_flags_s &status_flags,
		const PreFlightCheck::arm_requirements_t &arm_requirements,
		const hrt_abstime &time_since_boot, arm_disarm_reason_t calling_reason)
{
	// Double check that our static arrays are still valid
	static_assert(vehicle_status_s::ARMING_STATE_INIT == 0, "ARMING_STATE_INIT == 0");
	static_assert(vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE == vehicle_status_s::ARMING_STATE_MAX - 1,
		      "ARMING_STATE_IN_AIR_RESTORE == ARMING_STATE_MAX - 1");

	transition_result_t ret = TRANSITION_DENIED;
	arming_state_t current_arming_state = status.arming_state;
	bool feedback_provided = false;

	const bool hil_enabled = (status.hil_state == vehicle_status_s::HIL_STATE_ON);

	/* only check transition if the new state is actually different from the current one */
	if (new_arming_state == current_arming_state) {
		ret = TRANSITION_NOT_CHANGED;

	} else {

		/*
		 * Get sensing state if necessary
		 */
		bool preflight_check_ret = true;

		/* only perform the pre-arm check if we have to */
		if (fRunPreArmChecks && (new_arming_state == vehicle_status_s::ARMING_STATE_ARMED)
		    && !hil_enabled) {

			preflight_check_ret = PreFlightCheck::preflightCheck(mavlink_log_pub, status, status_flags, true, true,
					      time_since_boot);

			if (preflight_check_ret) {
				status_flags.condition_system_sensors_initialized = true;
			}

			feedback_provided = true;
		}

		/* re-run the pre-flight check as long as sensors are failing */
		if (!status_flags.condition_system_sensors_initialized
		    && fRunPreArmChecks
		    && ((new_arming_state == vehicle_status_s::ARMING_STATE_ARMED)
			|| (new_arming_state == vehicle_status_s::ARMING_STATE_STANDBY))
		    && !hil_enabled) {

			if ((last_preflight_check == 0) || (hrt_elapsed_time(&last_preflight_check) > 1000 * 1000)) {

				status_flags.condition_system_sensors_initialized = PreFlightCheck::preflightCheck(mavlink_log_pub, status,
						status_flags, false, status.arming_state != vehicle_status_s::ARMING_STATE_ARMED,
						time_since_boot);

				last_preflight_check = hrt_absolute_time();
			}
		}

		// Check that we have a valid state transition
		bool valid_transition = arming_transitions[new_arming_state][status.arming_state];

		if (valid_transition) {
			// We have a good transition. Now perform any secondary validation.
			if (new_arming_state == vehicle_status_s::ARMING_STATE_ARMED) {

				//      Do not perform pre-arm checks if coming from in air restore
				//      Allow if vehicle_status_s::HIL_STATE_ON
				if (status.arming_state != vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE) {

					bool prearm_check_ret = true;

					if (fRunPreArmChecks && preflight_check_ret) {
						// only bother running prearm if preflight was successful
						prearm_check_ret = PreFlightCheck::preArmCheck(mavlink_log_pub, status_flags, safety, arm_requirements, status);
					}

					if (!preflight_check_ret || !prearm_check_ret) {
						// the prearm and preflight checks already print the rejection reason
						feedback_provided = true;
						valid_transition = false;
					}
				}
			}
		}

		if (hil_enabled) {
			/* enforce lockdown in HIL */
			armed.lockdown = true;
			status_flags.condition_system_sensors_initialized = true;

			/* recover from a prearm fail */
			if (status.arming_state == vehicle_status_s::ARMING_STATE_STANDBY_ERROR) {
				status.arming_state = vehicle_status_s::ARMING_STATE_STANDBY;
			}

			// HIL can always go to standby
			if (new_arming_state == vehicle_status_s::ARMING_STATE_STANDBY) {
				valid_transition = true;
			}
		}

		if (!hil_enabled &&
		    (new_arming_state == vehicle_status_s::ARMING_STATE_STANDBY) &&
		    (status.arming_state != vehicle_status_s::ARMING_STATE_STANDBY_ERROR)) {

			// Sensors need to be initialized for STANDBY state, except for HIL
			if (!status_flags.condition_system_sensors_initialized) {
				feedback_provided = true;
				valid_transition = false;
			}
		}

		// Finish up the state transition
		if (valid_transition) {
			bool was_armed = armed.armed;
			armed.armed = (new_arming_state == vehicle_status_s::ARMING_STATE_ARMED);
			armed.ready_to_arm = (new_arming_state == vehicle_status_s::ARMING_STATE_ARMED)
					     || (new_arming_state == vehicle_status_s::ARMING_STATE_STANDBY);
			ret = TRANSITION_CHANGED;
			status.arming_state = new_arming_state;

			if (was_armed && !armed.armed) { // disarm transition
				status.latest_disarming_reason = (uint8_t)calling_reason;

			} else if (!was_armed && armed.armed) { // arm transition
				status.latest_arming_reason = (uint8_t)calling_reason;
			}

			if (new_arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
				status.armed_time = hrt_absolute_time();

			} else {
				status.armed_time = 0;
			}
		}
	}

	if (ret == TRANSITION_DENIED) {
		/* print to MAVLink and console if we didn't provide any feedback yet */
		if (!feedback_provided) {
			mavlink_log_critical(mavlink_log_pub, "Transition denied: %s to %s",
					     arming_state_names[status.arming_state], arming_state_names[new_arming_state]);
		}
	}

	return ret;
}

transition_result_t
main_state_transition(const vehicle_status_s &status, const main_state_t new_main_state,
		      const vehicle_status_flags_s &status_flags, commander_state_s &internal_state)
{
	// IMPORTANT: The assumption of callers of this function is that the execution of
	// this check is essentially "free". Therefore any runtime checking in here has to be
	// kept super lightweight. No complex logic or calls on external function should be
	// implemented here.

	transition_result_t ret = TRANSITION_DENIED;

	/* transition may be denied even if the same state is requested because conditions may have changed */
	switch (new_main_state) {
	case commander_state_s::MAIN_STATE_MANUAL:
	case commander_state_s::MAIN_STATE_STAB:
	case commander_state_s::MAIN_STATE_ACRO:
		ret = TRANSITION_CHANGED;
		break;

	case commander_state_s::MAIN_STATE_AIRSPEED:

		/* need at minimum airspeed estimate */
		if (status_flags.condition_airspeed_valid) {
			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_ALTCTL:

		/* need at minimum altitude estimate */
		if (status_flags.condition_local_altitude_valid ||
		    status_flags.condition_global_position_valid) {
			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_POSCTL:

		/* need at minimum local position estimate */
		if (status_flags.condition_local_position_valid ||
		    status_flags.condition_global_position_valid) {
			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_LOITER:

		/* need global position estimate */
		if (status_flags.condition_global_position_valid) {
			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_FOLLOW_TARGET:
	case commander_state_s::MAIN_STATE_ORBIT:

		/* Follow and orbit only implemented for multicopter */
		if (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_MISSION:

		/* need global position, home position, and a valid mission */
		if (status_flags.condition_global_position_valid &&
		    status_flags.condition_auto_mission_available) {

			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_RTL:

		/* need global position and home position */
		if (status_flags.condition_global_position_valid && status_flags.condition_home_position_valid) {
			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_TAKEOFF:
	case commander_state_s::MAIN_STATE_AUTO_LAND:

		/* need local position */
		if (status_flags.condition_local_position_valid) {
			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_PRECLAND:

		/* need local and global position, and precision land only implemented for multicopters */
		if (status_flags.condition_local_position_valid
		    && status_flags.condition_global_position_valid
		    && status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {

			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_OFFBOARD:

		/* need offboard signal */
		if (!status_flags.offboard_control_signal_lost) {

			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_MAX:
	default:
		break;
	}

	if (ret == TRANSITION_CHANGED) {
		if (internal_state.main_state != new_main_state) {
			internal_state.main_state = new_main_state;
			internal_state.main_state_changes++;
			internal_state.timestamp = hrt_absolute_time();

		} else {
			ret = TRANSITION_NOT_CHANGED;
		}
	}

	return ret;
}

/**
 * Enable failsafe and report to user
 */
void enable_failsafe(vehicle_status_s &status, bool old_failsafe, orb_advert_t *mavlink_log_pub, const char *reason)
{
	if (!old_failsafe && status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		// make sure intermittent failsafes don't lead to infinite delay by not constantly reseting the timestamp
		if (status.failsafe_timestamp == 0 ||
		    hrt_elapsed_time(&status.failsafe_timestamp) > 30_s) {
			status.failsafe_timestamp = hrt_absolute_time();
		}

		mavlink_log_critical(mavlink_log_pub, "Failsafe enabled: %s", reason);
	}

	status.failsafe = true;
}

/**
 * Check failsafe and main status and set navigation status for navigator accordingly
 */
bool set_nav_state(vehicle_status_s &status, actuator_armed_s &armed, commander_state_s &internal_state,
		   orb_advert_t *mavlink_log_pub, const link_loss_actions_t data_link_loss_act, const bool mission_finished,
		   const bool stay_in_failsafe, const vehicle_status_flags_s &status_flags, bool landed,
		   const link_loss_actions_t rc_loss_act, const offboard_loss_actions_t offb_loss_act,
		   const offboard_loss_rc_actions_t offb_loss_rc_act,
		   const position_nav_loss_actions_t posctl_nav_loss_act,
		   const float param_com_rcl_act_t, const int param_com_rcl_except)
{
	const navigation_state_t nav_state_old = status.nav_state;

	const bool is_armed = (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	const bool data_link_loss_act_configured = (data_link_loss_act > link_loss_actions_t::DISABLED);

	bool old_failsafe = status.failsafe;
	status.failsafe = false;

	// Safe to do reset flags here, as if loss state persists flags will be restored in the code below
	reset_link_loss_globals(armed, old_failsafe, rc_loss_act);
	reset_link_loss_globals(armed, old_failsafe, data_link_loss_act);
	reset_offboard_loss_globals(armed, old_failsafe, offb_loss_act, offb_loss_rc_act);

	// Failsafe decision logic for every normal non-failsafe mode
	switch (internal_state.main_state) {
	case commander_state_s::MAIN_STATE_ACRO:
	case commander_state_s::MAIN_STATE_MANUAL:
	case commander_state_s::MAIN_STATE_STAB:
	case commander_state_s::MAIN_STATE_AIRSPEED:
	case commander_state_s::MAIN_STATE_ALTCTL:

		// Require RC for all manual modes
		if (status.rc_signal_lost && is_armed) {
			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_rc);
			set_link_loss_nav_state(status, armed, status_flags, internal_state, rc_loss_act, param_com_rcl_act_t);

		} else {
			switch (internal_state.main_state) {
			case commander_state_s::MAIN_STATE_ACRO:
				status.nav_state = vehicle_status_s::NAVIGATION_STATE_ACRO;
				break;

			case commander_state_s::MAIN_STATE_MANUAL:
				status.nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;
				break;

			case commander_state_s::MAIN_STATE_STAB:
				status.nav_state = vehicle_status_s::NAVIGATION_STATE_STAB;
				break;

			case commander_state_s::MAIN_STATE_AIRSPEED:
				status.nav_state = vehicle_status_s::NAVIGATION_STATE_AIRSPEED;
				break;

			case commander_state_s::MAIN_STATE_ALTCTL:
				status.nav_state = vehicle_status_s::NAVIGATION_STATE_ALTCTL;
				break;

			default:
				status.nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;
				break;
			}
		}

		break;

	case commander_state_s::MAIN_STATE_POSCTL: {

			const bool rc_fallback_allowed = (posctl_nav_loss_act != position_nav_loss_actions_t::LAND_TERMINATE) || !is_armed;

			if (status.rc_signal_lost && is_armed) {
				enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_rc);
				set_link_loss_nav_state(status, armed, status_flags, internal_state, rc_loss_act, param_com_rcl_act_t);

				/* As long as there is RC, we can fallback to ALTCTL, or STAB. */
				/* A local position estimate is enough for POSCTL for multirotors,
				 * this enables POSCTL using e.g. flow.
				 * For fixedwing, a global position is needed. */

			} else if (check_invalid_pos_nav_state(status, old_failsafe, mavlink_log_pub, status_flags,
							       rc_fallback_allowed, status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING)) {
				// nothing to do - everything done in check_invalid_pos_nav_state

			} else {
				status.nav_state = vehicle_status_s::NAVIGATION_STATE_POSCTL;
			}
		}
		break;

	case commander_state_s::MAIN_STATE_AUTO_MISSION:

		/* go into failsafe
		 * - if we have an engine failure
		 * - if we have vtol transition failure
		 * - on data and RC link loss */

		if (is_armed && check_invalid_pos_nav_state(status, old_failsafe, mavlink_log_pub, status_flags, false, true)) {
			// nothing to do - everything done in check_invalid_pos_nav_state
		} else if (status.engine_failure) {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL;

		} else if (status_flags.vtol_transition_failure) {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;

		} else if (status.mission_failure) {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;

		} else if (status.data_link_lost && data_link_loss_act_configured
			   && is_armed && !landed) {
			// Data link lost, data link loss reaction configured -> do configured reaction
			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_datalink);
			set_link_loss_nav_state(status, armed, status_flags, internal_state, data_link_loss_act, 0);

		} else if (status.rc_signal_lost && !(param_com_rcl_except & RCLossExceptionBits::RCL_EXCEPT_MISSION)
			   && status_flags.rc_signal_found_once && is_armed && !landed) {
			// RC link lost, rc loss not disabled in mission, RC was used before -> RC loss reaction after delay
			// Safety pilot expects to be able to take over by RC in case anything unexpected happens
			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_rc);
			set_link_loss_nav_state(status, armed, status_flags, internal_state, rc_loss_act, param_com_rcl_act_t);

		} else if (status.rc_signal_lost && !(param_com_rcl_except & RCLossExceptionBits::RCL_EXCEPT_MISSION)
			   && status.data_link_lost && !data_link_loss_act_configured
			   && is_armed && !landed) {
			// All links lost, no data link loss reaction configured -> immediately do RC loss reaction
			// Lost all communication, by default it's considered unsafe to continue the mission
			// This is only reached when flying mission completely without RC (it was not present since boot)
			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_datalink);
			set_link_loss_nav_state(status, armed, status_flags, internal_state, rc_loss_act, 0);

		} else if (status.rc_signal_lost && (param_com_rcl_except & RCLossExceptionBits::RCL_EXCEPT_MISSION)
			   && status.data_link_lost && !data_link_loss_act_configured
			   && is_armed && !landed
			   && mission_finished) {
			// All links lost, all link loss reactions disabled -> return after mission finished
			// Pilot disabled all reactions, finish mission but then return to avoid lost vehicle
			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_rc_and_no_datalink);
			set_link_loss_nav_state(status, armed, status_flags, internal_state, link_loss_actions_t::AUTO_RTL, 0);

		} else if (!stay_in_failsafe) {
			// normal mission operation if there's no need to stay in failsafe
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_LOITER:

		/* go into failsafe on a engine failure */
		if (status.engine_failure) {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL;

		} else if (is_armed && check_invalid_pos_nav_state(status, old_failsafe, mavlink_log_pub, status_flags, false, true)) {
			// nothing to do - everything done in check_invalid_pos_nav_state
		} else if (status.data_link_lost && data_link_loss_act_configured && !landed && is_armed) {
			// Data link lost, data link loss reaction configured -> do configured reaction
			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_datalink);
			set_link_loss_nav_state(status, armed, status_flags, internal_state, data_link_loss_act, 0);

		} else if (status.rc_signal_lost && !(param_com_rcl_except & RCLossExceptionBits::RCL_EXCEPT_HOLD)
			   && status_flags.rc_signal_found_once && is_armed && !landed) {
			// RC link lost, rc loss not disabled in loiter, RC was used before -> RC loss reaction after delay
			// Safety pilot expects to be able to take over by RC in case anything unexpected happens
			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_rc);
			set_link_loss_nav_state(status, armed, status_flags, internal_state, rc_loss_act, param_com_rcl_act_t);

		} else if (status.rc_signal_lost && !(param_com_rcl_except & RCLossExceptionBits::RCL_EXCEPT_HOLD)
			   && status.data_link_lost && !data_link_loss_act_configured
			   && is_armed && !landed) {
			// All links lost, no data link loss reaction configured -> immediately do RC loss reaction
			// Lost all communication, by default it's considered unsafe to continue the mission
			// This is only reached when flying mission completely without RC (it was not present since boot)
			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_datalink);
			set_link_loss_nav_state(status, armed, status_flags, internal_state, rc_loss_act, 0);

		} else {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_RTL:

		/* require global position and home, also go into failsafe on an engine failure */

		if (status.engine_failure) {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL;

		} else if (is_armed && check_invalid_pos_nav_state(status, old_failsafe, mavlink_log_pub, status_flags, false, true)) {
			// nothing to do - everything done in check_invalid_pos_nav_state
		} else {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_FOLLOW_TARGET:

		/* require global position and home */

		if (status.engine_failure) {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL;

		} else if (is_armed && check_invalid_pos_nav_state(status, old_failsafe, mavlink_log_pub, status_flags, false, true)) {
			// nothing to do - everything done in check_invalid_pos_nav_state

		} else {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET;
		}

		break;

	case commander_state_s::MAIN_STATE_ORBIT:
		if (status.engine_failure) {
			// failsafe: on engine failure
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL;

			// Orbit can only be started via vehicle_command (mavlink). Recovery from failsafe into orbit
			// is not possible and therefore the internal_state needs to be adjusted.
			internal_state.main_state = commander_state_s::MAIN_STATE_POSCTL;

		} else if (is_armed && check_invalid_pos_nav_state(status, old_failsafe, mavlink_log_pub, status_flags, false, true)) {
			// failsafe: necessary position estimate lost; switching is done in check_invalid_pos_nav_state

			// Orbit can only be started via vehicle_command (mavlink). Consequently, recovery from failsafe into orbit
			// is not possible and therefore the internal_state needs to be adjusted.
			internal_state.main_state = commander_state_s::MAIN_STATE_POSCTL;

		} else if (status.data_link_lost && data_link_loss_act_configured && !landed && is_armed) {
			// failsafe: just datalink is lost and we're in air
			set_link_loss_nav_state(status, armed, status_flags, internal_state, data_link_loss_act, 0);

			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_datalink);

			// Orbit can only be started via vehicle_command (mavlink). Consequently, recovery from failsafe into orbit
			// is not possible and therefore the internal_state needs to be adjusted.
			internal_state.main_state = commander_state_s::MAIN_STATE_POSCTL;

		} else if (status.rc_signal_lost && status.data_link_lost && !data_link_loss_act_configured && is_armed) {
			// Orbit does not depend on RC but while armed & all links lost & when datalink loss is not set up, we failsafe
			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_rc);

			set_link_loss_nav_state(status, armed, status_flags, internal_state, rc_loss_act, 0);

			// Orbit can only be started via vehicle_command (mavlink). Consequently, recovery from failsafe into orbit
			// is not possible and therefore the internal_state needs to be adjusted.
			internal_state.main_state = commander_state_s::MAIN_STATE_POSCTL;

		} else {
			// no failsafe, RC is not mandatory for orbit
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_ORBIT;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_TAKEOFF:

		/* require local position */

		if (status.engine_failure) {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL;

		} else if (is_armed && check_invalid_pos_nav_state(status, old_failsafe, mavlink_log_pub, status_flags, false, false)) {
			// nothing to do - everything done in check_invalid_pos_nav_state
		} else if (status.data_link_lost && data_link_loss_act_configured && !landed && is_armed) {
			// Data link lost, data link loss reaction configured -> do configured reaction
			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_datalink);
			set_link_loss_nav_state(status, armed, status_flags, internal_state, data_link_loss_act, 0);

		} else if (status.rc_signal_lost && !(param_com_rcl_except & RCLossExceptionBits::RCL_EXCEPT_HOLD)
			   && status_flags.rc_signal_found_once && is_armed && !landed) {
			// RC link lost, rc loss not disabled in loiter, RC was used before -> RC loss reaction after delay
			// Safety pilot expects to be able to take over by RC in case anything unexpected happens
			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_rc);
			set_link_loss_nav_state(status, armed, status_flags, internal_state, rc_loss_act, param_com_rcl_act_t);

		} else if (status.rc_signal_lost && !(param_com_rcl_except & RCLossExceptionBits::RCL_EXCEPT_HOLD)
			   && status.data_link_lost && !data_link_loss_act_configured
			   && is_armed && !landed) {
			// All links lost, no data link loss reaction configured -> immediately do RC loss reaction
			// Lost all communication, by default it's considered unsafe to continue the mission
			// This is only reached when flying mission completely without RC (it was not present since boot)
			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_datalink);
			set_link_loss_nav_state(status, armed, status_flags, internal_state, rc_loss_act, 0);

		} else {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_LAND:

		/* require local position */

		if (status.engine_failure) {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL;

		} else if (is_armed && check_invalid_pos_nav_state(status, old_failsafe, mavlink_log_pub, status_flags, false, false)) {
			// nothing to do - everything done in check_invalid_pos_nav_state

		} else {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_PRECLAND:

		/* must be rotary wing plus same requirements as normal landing */

		if (status.engine_failure) {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL;

		} else if (is_armed && check_invalid_pos_nav_state(status, old_failsafe, mavlink_log_pub, status_flags, false, false)) {
			// nothing to do - everything done in check_invalid_pos_nav_state

		} else {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND;
		}

		break;

	case commander_state_s::MAIN_STATE_OFFBOARD:

		if (status_flags.offboard_control_signal_lost) {
			if (status.rc_signal_lost && !(param_com_rcl_except & RCLossExceptionBits::RCL_EXCEPT_OFFBOARD)) {
				// Offboard and RC are lost
				enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_rc_and_no_offboard);
				set_offboard_loss_nav_state(status, armed, status_flags, offb_loss_act);

			} else {
				// Offboard is lost, RC is ok
				enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_offboard);
				set_offboard_loss_rc_nav_state(status, armed, status_flags, offb_loss_rc_act);
			}

		} else {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
		}

	default:
		break;
	}

	return status.nav_state != nav_state_old;
}

bool check_invalid_pos_nav_state(vehicle_status_s &status, bool old_failsafe, orb_advert_t *mavlink_log_pub,
				 const vehicle_status_flags_s &status_flags, const bool use_rc, const bool using_global_pos)
{
	bool fallback_required = false;

	if (using_global_pos && !status_flags.condition_global_position_valid) {
		fallback_required = true;

	} else if (!using_global_pos
		   && (!status_flags.condition_local_position_valid || !status_flags.condition_local_velocity_valid)) {

		fallback_required = true;
	}

	if (fallback_required) {
		if (use_rc) {
			// fallback to a mode that gives the operator stick control
			if (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
			    && status_flags.condition_local_position_valid) {
				status.nav_state = vehicle_status_s::NAVIGATION_STATE_POSCTL;

			} else if (status_flags.condition_local_altitude_valid) {
				status.nav_state = vehicle_status_s::NAVIGATION_STATE_ALTCTL;

			} else {
				status.nav_state = vehicle_status_s::NAVIGATION_STATE_STAB;
			}

		} else {
			// go into a descent that does not require stick control
			if (status_flags.condition_local_position_valid) {
				status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;

			} else if (status_flags.condition_local_altitude_valid) {
				if (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
					status.nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;

				} else {
					// TODO: FW position controller doesn't run without condition_global_position_valid
					status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDGPSFAIL;
				}

			} else {
				status.nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
			}
		}

		if (using_global_pos) {
			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_global_position);

		} else {
			enable_failsafe(status, old_failsafe, mavlink_log_pub, reason_no_local_position);
		}

	}

	return fallback_required;

}

void set_link_loss_nav_state(vehicle_status_s &status, actuator_armed_s &armed,
			     const vehicle_status_flags_s &status_flags, commander_state_s &internal_state, link_loss_actions_t link_loss_act,
			     const float ll_delay)
{
	if (hrt_elapsed_time(&status.failsafe_timestamp) < (ll_delay * 1_s)
	    && link_loss_act != link_loss_actions_t::DISABLED) {
		// delay failsafe reaction by trying to hold position if configured
		link_loss_act = link_loss_actions_t::AUTO_LOITER;
	}

	// do the best you can according to the action set
	switch (link_loss_act) {
	case link_loss_actions_t::DISABLED:
		// If datalink loss failsafe is disabled then no action must be taken.
		break;

	case link_loss_actions_t::AUTO_RTL:
		if (status_flags.condition_global_position_valid && status_flags.condition_home_position_valid) {
			main_state_transition(status, commander_state_s::MAIN_STATE_AUTO_RTL, status_flags, internal_state);
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
			return;
		}

	// FALLTHROUGH
	case link_loss_actions_t::AUTO_LOITER:
		if (status_flags.condition_global_position_valid) {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
			return;
		}

	// FALLTHROUGH
	case link_loss_actions_t::AUTO_LAND:
		if (status_flags.condition_global_position_valid) {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;
			return;

		} else {
			if (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				if (status_flags.condition_local_position_valid) {
					status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;
					return;

				} else if (status_flags.condition_local_altitude_valid) {
					status.nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;
					return;
				}

			} else {
				// TODO: FW position controller doesn't run without condition_global_position_valid
				status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDGPSFAIL;
				return;
			}
		}

	// FALLTHROUGH
	case link_loss_actions_t::TERMINATE:
		status.nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
		armed.force_failsafe = true;
		break;

	case link_loss_actions_t::LOCKDOWN:
		armed.lockdown = true;
		break;
	}
}

void reset_link_loss_globals(actuator_armed_s &armed, const bool old_failsafe, const link_loss_actions_t link_loss_act)
{
	if (old_failsafe) {
		if (link_loss_act == link_loss_actions_t::TERMINATE) {
			armed.force_failsafe = false;

		} else if (link_loss_act == link_loss_actions_t::LOCKDOWN) {
			armed.lockdown = false;
		}
	}
}

void set_offboard_loss_nav_state(vehicle_status_s &status, actuator_armed_s &armed,
				 const vehicle_status_flags_s &status_flags,
				 const offboard_loss_actions_t offboard_loss_act)
{
	switch (offboard_loss_act) {
	case offboard_loss_actions_t::DISABLED:
		// If offboard loss failsafe is disabled then no action must be taken.
		return;

	case offboard_loss_actions_t::TERMINATE:
		status.nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
		armed.force_failsafe = true;
		return;

	case offboard_loss_actions_t::LOCKDOWN:
		armed.lockdown = true;
		return;

	case offboard_loss_actions_t::AUTO_RTL:
		if (status_flags.condition_global_position_valid && status_flags.condition_home_position_valid) {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
			return;
		}

	// FALLTHROUGH
	case offboard_loss_actions_t::AUTO_LOITER:
		if (status_flags.condition_global_position_valid) {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
			return;
		}

	// FALLTHROUGH
	case offboard_loss_actions_t::AUTO_LAND:
		if (status_flags.condition_global_position_valid) {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;
			return;
		}
	}

	// If none of the above worked, try to mitigate
	if (status_flags.condition_local_altitude_valid) {
		if (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;

		} else {
			// TODO: FW position controller doesn't run without condition_global_position_valid
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDGPSFAIL;
		}

	} else {
		status.nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
	}
}

void set_offboard_loss_rc_nav_state(vehicle_status_s &status, actuator_armed_s &armed,
				    const vehicle_status_flags_s &status_flags,
				    const offboard_loss_rc_actions_t offboard_loss_rc_act)
{
	switch (offboard_loss_rc_act) {
	case offboard_loss_rc_actions_t::DISABLED:
		// If offboard loss failsafe is disabled then no action must be taken.
		return;

	case offboard_loss_rc_actions_t::TERMINATE:
		status.nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
		armed.force_failsafe = true;
		return;

	case offboard_loss_rc_actions_t::LOCKDOWN:
		armed.lockdown = true;
		return;

	case offboard_loss_rc_actions_t::MANUAL_POSITION:
		if (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
		    && status_flags.condition_local_position_valid) {

			status.nav_state = vehicle_status_s::NAVIGATION_STATE_POSCTL;
			return;

		} else if (status_flags.condition_global_position_valid) {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_POSCTL;
			return;
		}

	// FALLTHROUGH
	case offboard_loss_rc_actions_t::MANUAL_ALTITUDE:
		if (status_flags.condition_local_altitude_valid) {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_ALTCTL;
			return;
		}

	// FALLTHROUGH
	case offboard_loss_rc_actions_t::MANUAL_ATTITUDE:
		status.nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;
		return;

	case offboard_loss_rc_actions_t::AUTO_RTL:
		if (status_flags.condition_global_position_valid && status_flags.condition_home_position_valid) {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
			return;
		}

	// FALLTHROUGH
	case offboard_loss_rc_actions_t::AUTO_LOITER:
		if (status_flags.condition_global_position_valid) {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
			return;
		}

	// FALLTHROUGH
	case offboard_loss_rc_actions_t::AUTO_LAND:
		if (status_flags.condition_global_position_valid) {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;
			return;
		}
	}

	// If none of the above worked, try to mitigate
	if (status_flags.condition_local_altitude_valid) {
		//TODO: Add case for rover
		if (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;

		} else {
			// TODO: FW position controller doesn't run without condition_global_position_valid
			status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDGPSFAIL;
		}

	} else {
		status.nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
	}
}

void reset_offboard_loss_globals(actuator_armed_s &armed, const bool old_failsafe,
				 const offboard_loss_actions_t offboard_loss_act,
				 const offboard_loss_rc_actions_t offboard_loss_rc_act)
{
	if (old_failsafe) {
		if (offboard_loss_act == offboard_loss_actions_t::TERMINATE
		    || offboard_loss_rc_act == offboard_loss_rc_actions_t::TERMINATE) {
			armed.force_failsafe = false;

		}

		if (offboard_loss_act == offboard_loss_actions_t::LOCKDOWN
		    || offboard_loss_rc_act == offboard_loss_rc_actions_t::LOCKDOWN) {
			armed.lockdown = false;
		}
	}
}

void battery_failsafe(orb_advert_t *mavlink_log_pub, const vehicle_status_s &status,
		      const vehicle_status_flags_s &status_flags, commander_state_s &internal_state, const uint8_t battery_warning,
		      const low_battery_action_t low_battery_action)
{
	switch (battery_warning) {
	case battery_status_s::BATTERY_WARNING_NONE:
		break;

	case battery_status_s::BATTERY_WARNING_LOW:
		mavlink_log_critical(mavlink_log_pub, "Low battery level! Return advised");
		break;

	case battery_status_s::BATTERY_WARNING_CRITICAL:

		static constexpr char battery_critical[] = "Critical battery level!";

		switch (low_battery_action) {
		case LOW_BAT_ACTION::WARNING:
			mavlink_log_critical(mavlink_log_pub, "%s, landing advised", battery_critical);
			break;

		case LOW_BAT_ACTION::RETURN:

		// FALLTHROUGH
		case LOW_BAT_ACTION::RETURN_OR_LAND:

			if (status_flags.condition_global_position_valid && status_flags.condition_home_position_valid) {
				if (!(internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_RTL ||
				      internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_LAND ||
				      internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_PRECLAND)) {

					internal_state.main_state = commander_state_s::MAIN_STATE_AUTO_RTL;
					internal_state.timestamp = hrt_absolute_time();
					mavlink_log_critical(mavlink_log_pub, "%s, executing RTL", battery_critical);
				}

			} else {
				if (!(internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_LAND ||
				      internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_PRECLAND)) {
					internal_state.main_state = commander_state_s::MAIN_STATE_AUTO_LAND;
					internal_state.timestamp = hrt_absolute_time();
					mavlink_log_emergency(mavlink_log_pub, "%s, can't execute RTL, landing instead", battery_critical);
				}
			}

			break;

		case LOW_BAT_ACTION::LAND:
			if (!(internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_LAND ||
			      internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_PRECLAND)) {
				internal_state.main_state = commander_state_s::MAIN_STATE_AUTO_LAND;
				internal_state.timestamp = hrt_absolute_time();
				mavlink_log_emergency(mavlink_log_pub, "%s, landing", battery_critical);
			}

			break;
		}

		break;

	case battery_status_s::BATTERY_WARNING_EMERGENCY:

		static constexpr char battery_dangerous[] = "Dangerous battery level!";

		switch (low_battery_action) {
		case LOW_BAT_ACTION::WARNING:
			mavlink_log_emergency(mavlink_log_pub, "%s, please land!", battery_dangerous);
			break;

		case LOW_BAT_ACTION::RETURN:
			if (status_flags.condition_global_position_valid && status_flags.condition_home_position_valid) {
				if (!(internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_RTL ||
				      internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_LAND ||
				      internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_PRECLAND)) {
					internal_state.main_state = commander_state_s::MAIN_STATE_AUTO_RTL;
					internal_state.timestamp = hrt_absolute_time();
					mavlink_log_critical(mavlink_log_pub, "%s, executing RTL", battery_dangerous);
				}

			} else {
				if (!(internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_LAND ||
				      internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_PRECLAND)) {
					internal_state.main_state = commander_state_s::MAIN_STATE_AUTO_LAND;
					internal_state.timestamp = hrt_absolute_time();
					mavlink_log_emergency(mavlink_log_pub, "%s, can't execute RTL, landing instead", battery_dangerous);
				}
			}

			break;

		case LOW_BAT_ACTION::RETURN_OR_LAND:

		// FALLTHROUGH
		case LOW_BAT_ACTION::LAND:
			if (!(internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_LAND ||
			      internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_PRECLAND)) {
				internal_state.main_state = commander_state_s::MAIN_STATE_AUTO_LAND;
				internal_state.timestamp = hrt_absolute_time();
				mavlink_log_emergency(mavlink_log_pub, "%s, landing", battery_dangerous);
			}

			break;
		}

		break;

	case battery_status_s::BATTERY_WARNING_FAILED:
		mavlink_log_emergency(mavlink_log_pub, "Battery failure detected");
		break;
	}
}
