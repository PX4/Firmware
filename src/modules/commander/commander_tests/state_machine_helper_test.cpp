/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Simon Wilks <sjwilks@gmail.com>
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
 * @file state_machine_helper_test.cpp
 * System state machine unit test.
 *
 */

#include "state_machine_helper_test.h"

#include "../state_machine_helper.h"
#include <unit_test.h>
#include "../Arming/PreFlightCheck/PreFlightCheck.hpp"

class StateMachineHelperTest : public UnitTest
{
public:
	StateMachineHelperTest() = default;
	~StateMachineHelperTest() override = default;

	bool run_tests() override;

private:
	bool armingStateTransitionTest();
	bool mainStateTransitionTest();
};

bool StateMachineHelperTest::armingStateTransitionTest()
{
	// These are the critical values from vehicle_status_s and actuator_armed_s which must be primed
	// to simulate machine state prior to testing an arming state transition. This structure is also
	// use to represent the expected machine state after the transition has been requested.
	typedef struct {
		arming_state_t  arming_state;   // vehicle_status_s.arming_state
		bool            armed;          // actuator_armed_s.armed
		bool            ready_to_arm;   // actuator_armed_s.ready_to_arm
	} ArmingTransitionVolatileState_t;

	// This structure represents a test case for arming_state_transition. It contains the machine
	// state prior to transition, the requested state to transition to and finally the expected
	// machine state after transition.
	typedef struct {
		const char                     *assertMsg;                              // Text to show when test case fails
		ArmingTransitionVolatileState_t current_state;                          // Machine state prior to transition
		hil_state_t                     hil_state;                              // Current vehicle_status_s.hil_state
		bool
		condition_system_sensors_initialized;   // Current vehicle_status_s.condition_system_sensors_initialized
		bool                            safety_switch_available;                // Current safety_s.safety_switch_available
		bool                            safety_off;                             // Current safety_s.safety_off
		arming_state_t                  requested_state;                        // Requested arming state to transition to
		ArmingTransitionVolatileState_t expected_state;                         // Expected machine state after transition
		transition_result_t             expected_transition_result;             // Expected result from arming_state_transition
	} ArmingTransitionTest_t;

	// We use these defines so that our test cases are more readable
#define ATT_ARMED true
#define ATT_DISARMED false
#define ATT_READY_TO_ARM true
#define ATT_NOT_READY_TO_ARM false
#define ATT_SENSORS_INITIALIZED true
#define ATT_SENSORS_NOT_INITIALIZED false
#define ATT_SAFETY_AVAILABLE true
#define ATT_SAFETY_NOT_AVAILABLE true
#define ATT_SAFETY_OFF true
#define ATT_SAFETY_ON false

	// These are test cases for arming_state_transition
	static const ArmingTransitionTest_t rgArmingTransitionTests[] = {
		// TRANSITION_NOT_CHANGED tests

		{
			"no transition: identical states",
			{ vehicle_status_s::ARMING_STATE_INIT, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_INIT,
			{ vehicle_status_s::ARMING_STATE_INIT, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, TRANSITION_NOT_CHANGED
		},

		// TRANSITION_CHANGED tests

		// Check all basic valid transitions, these don't require special state in vehicle_status_t or safety_s

		{
			"transition: init to standby",
			{ vehicle_status_s::ARMING_STATE_INIT, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_STANDBY,
			{ vehicle_status_s::ARMING_STATE_STANDBY, ATT_DISARMED, ATT_READY_TO_ARM }, TRANSITION_CHANGED
		},

		{
			"transition: init to standby error",
			{ vehicle_status_s::ARMING_STATE_INIT, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_STANDBY_ERROR,
			{ vehicle_status_s::ARMING_STATE_STANDBY_ERROR, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, TRANSITION_CHANGED
		},

		{
			"transition: init to reboot",
			{ vehicle_status_s::ARMING_STATE_INIT, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_SHUTDOWN,
			{ vehicle_status_s::ARMING_STATE_SHUTDOWN, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, TRANSITION_CHANGED
		},

		{
			"transition: standby to init",
			{ vehicle_status_s::ARMING_STATE_STANDBY, ATT_DISARMED, ATT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_INIT,
			{ vehicle_status_s::ARMING_STATE_INIT, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, TRANSITION_CHANGED
		},

		{
			"transition: standby to standby error",
			{ vehicle_status_s::ARMING_STATE_STANDBY, ATT_DISARMED, ATT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_STANDBY_ERROR,
			{ vehicle_status_s::ARMING_STATE_STANDBY_ERROR, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, TRANSITION_CHANGED
		},

		{
			"transition: standby to reboot",
			{ vehicle_status_s::ARMING_STATE_STANDBY, ATT_DISARMED, ATT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_SHUTDOWN,
			{ vehicle_status_s::ARMING_STATE_SHUTDOWN, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, TRANSITION_CHANGED
		},

		{
			"transition: armed to standby",
			{ vehicle_status_s::ARMING_STATE_ARMED, ATT_ARMED, ATT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_STANDBY,
			{ vehicle_status_s::ARMING_STATE_STANDBY, ATT_DISARMED, ATT_READY_TO_ARM }, TRANSITION_CHANGED
		},

		{
			"transition: standby error to reboot",
			{ vehicle_status_s::ARMING_STATE_STANDBY_ERROR, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_SHUTDOWN,
			{ vehicle_status_s::ARMING_STATE_SHUTDOWN, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, TRANSITION_CHANGED
		},

		{
			"transition: in air restore to armed",
			{ vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE, ATT_DISARMED, ATT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_ARMED,
			{ vehicle_status_s::ARMING_STATE_ARMED, ATT_ARMED, ATT_READY_TO_ARM }, TRANSITION_CHANGED
		},

		{
			"transition: in air restore to reboot",
			{ vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE, ATT_DISARMED, ATT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_SHUTDOWN,
			{ vehicle_status_s::ARMING_STATE_SHUTDOWN, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, TRANSITION_CHANGED
		},

		// hil on tests, standby error to standby not normally allowed

		{
			"transition: standby error to standby, hil on",
			{ vehicle_status_s::ARMING_STATE_STANDBY_ERROR, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_ON, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_STANDBY,
			{ vehicle_status_s::ARMING_STATE_STANDBY, ATT_DISARMED, ATT_READY_TO_ARM }, TRANSITION_CHANGED
		},

		// Safety switch arming tests

		{
			"transition: standby to armed, no safety switch",
			{ vehicle_status_s::ARMING_STATE_STANDBY, ATT_DISARMED, ATT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_ON, ATT_SENSORS_INITIALIZED, ATT_SAFETY_NOT_AVAILABLE, ATT_SAFETY_OFF,
			vehicle_status_s::ARMING_STATE_ARMED,
			{ vehicle_status_s::ARMING_STATE_ARMED, ATT_ARMED, ATT_READY_TO_ARM }, TRANSITION_CHANGED
		},

		{
			"transition: standby to armed, safety switch off",
			{ vehicle_status_s::ARMING_STATE_STANDBY, ATT_DISARMED, ATT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_ON, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_OFF,
			vehicle_status_s::ARMING_STATE_ARMED,
			{ vehicle_status_s::ARMING_STATE_ARMED, ATT_ARMED, ATT_READY_TO_ARM }, TRANSITION_CHANGED
		},

		// TRANSITION_DENIED tests

		// Check some important basic invalid transitions, these don't require special state in vehicle_status_t or safety_s

		{
			"no transition: init to armed",
			{ vehicle_status_s::ARMING_STATE_INIT, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_ARMED,
			{ vehicle_status_s::ARMING_STATE_INIT, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, TRANSITION_DENIED
		},

		{
			"no transition: armed to init",
			{ vehicle_status_s::ARMING_STATE_ARMED, ATT_ARMED, ATT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_INIT,
			{ vehicle_status_s::ARMING_STATE_ARMED, ATT_ARMED, ATT_READY_TO_ARM }, TRANSITION_DENIED
		},

		{
			"no transition: armed to reboot",
			{ vehicle_status_s::ARMING_STATE_ARMED, ATT_ARMED, ATT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_SHUTDOWN,
			{ vehicle_status_s::ARMING_STATE_ARMED, ATT_ARMED, ATT_READY_TO_ARM }, TRANSITION_DENIED
		},

		{
			"no transition: standby error to armed",
			{ vehicle_status_s::ARMING_STATE_STANDBY_ERROR, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_ARMED,
			{ vehicle_status_s::ARMING_STATE_STANDBY_ERROR, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, TRANSITION_DENIED
		},

		{
			"no transition: standby error to standby",
			{ vehicle_status_s::ARMING_STATE_STANDBY_ERROR, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_STANDBY,
			{ vehicle_status_s::ARMING_STATE_STANDBY_ERROR, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, TRANSITION_DENIED
		},

		{
			"no transition: reboot to armed",
			{ vehicle_status_s::ARMING_STATE_SHUTDOWN, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_ARMED,
			{ vehicle_status_s::ARMING_STATE_SHUTDOWN, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, TRANSITION_DENIED
		},

		{
			"no transition: in air restore to standby",
			{ vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_STANDBY,
			{ vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, TRANSITION_DENIED
		},

		// Sensor tests

		//{ "transition to standby error: init to standby - sensors not initialized",
		//    { vehicle_status_s::ARMING_STATE_INIT, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_NOT_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
		//    vehicle_status_s::ARMING_STATE_STANDBY,
		//    { vehicle_status_s::ARMING_STATE_STANDBY_ERROR, ATT_DISARMED, ATT_NOT_READY_TO_ARM }, TRANSITION_DENIED },

		// Safety switch arming tests

		{
			"no transition: init to armed, safety switch on",
			{ vehicle_status_s::ARMING_STATE_STANDBY, ATT_DISARMED, ATT_READY_TO_ARM }, vehicle_status_s::HIL_STATE_OFF, ATT_SENSORS_INITIALIZED, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_ARMED,
			{ vehicle_status_s::ARMING_STATE_STANDBY, ATT_DISARMED, ATT_READY_TO_ARM }, TRANSITION_DENIED
		},
	};

	struct vehicle_status_s status = {};
	struct vehicle_status_flags_s status_flags = {};
	struct safety_s         safety = {};
	struct actuator_armed_s armed = {};

	size_t cArmingTransitionTests = sizeof(rgArmingTransitionTests) / sizeof(rgArmingTransitionTests[0]);

	for (size_t i = 0; i < cArmingTransitionTests; i++) {
		const ArmingTransitionTest_t *test = &rgArmingTransitionTests[i];

		PreFlightCheck::arm_requirements_t arm_req{};

		// Setup initial machine state
		status.arming_state = test->current_state.arming_state;
		status_flags.condition_system_sensors_initialized = test->condition_system_sensors_initialized;
		status.hil_state = test->hil_state;
		// The power status of the test unit is not relevant for the unit test
		status_flags.circuit_breaker_engaged_power_check = true;
		safety.safety_switch_available = test->safety_switch_available;
		safety.safety_off = test->safety_off;
		armed.armed = test->current_state.armed;
		armed.ready_to_arm = test->current_state.ready_to_arm;

		// Attempt transition
		transition_result_t result = arming_state_transition(status, safety, test->requested_state, armed,
					     true /* enable pre-arm checks */,
					     nullptr /* no mavlink_log_pub */,
					     status_flags,
					     arm_req,
					     2e6, /* 2 seconds after boot, everything should be checked */
					     arm_disarm_reason_t::unit_test);

		// Validate result of transition
		ut_compare(test->assertMsg, test->expected_transition_result, result);
		ut_compare(test->assertMsg, status.arming_state, test->expected_state.arming_state);
		ut_compare(test->assertMsg, armed.armed, test->expected_state.armed);
		ut_compare(test->assertMsg, armed.ready_to_arm, test->expected_state.ready_to_arm);
	}

	return true;
}

bool StateMachineHelperTest::mainStateTransitionTest()
{
	// This structure represent a single test case for testing Main State transitions.
	typedef struct {
		const char *assertMsg;				// Text to show when test case fails
		uint8_t	 condition_bits;			// Bits for various condition_* values
		uint8_t	from_state;				// State prior to transition request
		main_state_t to_state;				// State to transition to
		transition_result_t	expected_transition_result;	// Expected result from main_state_transition call
	} MainTransitionTest_t;

	// Bits for condition_bits
#define MTT_ALL_NOT_VALID		0
#define MTT_ROTARY_WING			1 << 0
#define MTT_LOC_ALT_VALID		1 << 1
#define MTT_LOC_POS_VALID		1 << 2
#define MTT_HOME_POS_VALID		1 << 3
#define MTT_GLOBAL_POS_VALID		1 << 4

	static const MainTransitionTest_t rgMainTransitionTests[] = {

		// TRANSITION_NOT_CHANGED tests

		{
			"no transition: identical states",
			MTT_ALL_NOT_VALID,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_MANUAL, TRANSITION_NOT_CHANGED
		},

		// TRANSITION_CHANGED tests

		{
			"transition: MANUAL to ACRO - rotary",
			MTT_ROTARY_WING,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_ACRO, TRANSITION_CHANGED
		},

		{
			"transition: MANUAL to ACRO - not rotary",
			MTT_ALL_NOT_VALID,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_ACRO, TRANSITION_CHANGED
		},

		{
			"transition: ACRO to MANUAL",
			MTT_ALL_NOT_VALID,
			commander_state_s::MAIN_STATE_ACRO, commander_state_s::MAIN_STATE_MANUAL, TRANSITION_CHANGED
		},

		{
			"transition: MANUAL to AUTO_MISSION - global position valid, home position valid",
			MTT_GLOBAL_POS_VALID | MTT_HOME_POS_VALID,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_AUTO_MISSION, TRANSITION_CHANGED
		},

		{
			"transition: AUTO_MISSION to MANUAL - global position valid, home position valid",
			MTT_GLOBAL_POS_VALID | MTT_HOME_POS_VALID,
			commander_state_s::MAIN_STATE_AUTO_MISSION, commander_state_s::MAIN_STATE_MANUAL, TRANSITION_CHANGED
		},

		{
			"transition: MANUAL to AUTO_LOITER - global position valid",
			MTT_GLOBAL_POS_VALID,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_AUTO_LOITER, TRANSITION_CHANGED
		},

		{
			"transition: AUTO_LOITER to MANUAL - global position valid",
			MTT_GLOBAL_POS_VALID,
			commander_state_s::MAIN_STATE_AUTO_LOITER, commander_state_s::MAIN_STATE_MANUAL, TRANSITION_CHANGED
		},

		{
			"transition: MANUAL to AUTO_RTL - global position valid, home position valid",
			MTT_GLOBAL_POS_VALID | MTT_HOME_POS_VALID,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_AUTO_RTL, TRANSITION_CHANGED
		},

		{
			"transition: AUTO_RTL to MANUAL - global position valid, home position valid",
			MTT_GLOBAL_POS_VALID | MTT_HOME_POS_VALID,
			commander_state_s::MAIN_STATE_AUTO_RTL, commander_state_s::MAIN_STATE_MANUAL, TRANSITION_CHANGED
		},

		{
			"transition: MANUAL to ALTCTL - not rotary",
			MTT_LOC_ALT_VALID,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_ALTCTL, TRANSITION_CHANGED
		},

		{
			"transition: MANUAL to ALTCTL - rotary, global position not valid, local altitude valid",
			MTT_ROTARY_WING | MTT_LOC_ALT_VALID,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_ALTCTL, TRANSITION_CHANGED
		},

		{
			"transition: MANUAL to ALTCTL - rotary, global position valid, local altitude not valid",
			MTT_ROTARY_WING | MTT_GLOBAL_POS_VALID,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_ALTCTL, TRANSITION_CHANGED
		},

		{
			"transition: ALTCTL to MANUAL - local altitude valid",
			MTT_LOC_ALT_VALID,
			commander_state_s::MAIN_STATE_ALTCTL, commander_state_s::MAIN_STATE_MANUAL, TRANSITION_CHANGED
		},

		{
			"transition: MANUAL to POSCTL - local position not valid, global position valid",
			MTT_GLOBAL_POS_VALID,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_POSCTL, TRANSITION_CHANGED
		},

		{
			"transition: MANUAL to POSCTL - local position valid, global position not valid",
			MTT_LOC_POS_VALID,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_POSCTL, TRANSITION_CHANGED
		},

		{
			"transition: POSCTL to MANUAL - local position valid, global position valid",
			MTT_LOC_POS_VALID,
			commander_state_s::MAIN_STATE_POSCTL, commander_state_s::MAIN_STATE_MANUAL, TRANSITION_CHANGED
		},

		// TRANSITION_DENIED tests

		{
			"no transition: MANUAL to AUTO_MISSION - global position not valid",
			MTT_ALL_NOT_VALID,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_AUTO_MISSION, TRANSITION_DENIED
		},

		{
			"no transition: MANUAL to AUTO_LOITER - global position not valid",
			MTT_ALL_NOT_VALID,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_AUTO_LOITER, TRANSITION_DENIED
		},

		{
			"no transition: MANUAL to AUTO_RTL - global position not valid, home position not valid",
			MTT_ALL_NOT_VALID,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_AUTO_RTL, TRANSITION_DENIED
		},

		{
			"no transition: MANUAL to AUTO_RTL - global position not valid, home position valid",
			MTT_HOME_POS_VALID,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_AUTO_RTL, TRANSITION_DENIED
		},

		{
			"no transition: MANUAL to AUTO_RTL - global position valid, home position not valid",
			MTT_GLOBAL_POS_VALID,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_AUTO_RTL, TRANSITION_DENIED
		},

		{
			"transition: MANUAL to ALTCTL - not rotary",
			MTT_ALL_NOT_VALID,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_ALTCTL, TRANSITION_DENIED
		},

		{
			"no transition: MANUAL to ALTCTL - rotary, global position not valid, local altitude not valid",
			MTT_ROTARY_WING,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_ALTCTL, TRANSITION_DENIED
		},

		{
			"no transition: MANUAL to POSCTL - local position not valid, global position not valid",
			MTT_ALL_NOT_VALID,
			commander_state_s::MAIN_STATE_MANUAL, commander_state_s::MAIN_STATE_POSCTL, TRANSITION_DENIED
		},
	};

	size_t cMainTransitionTests = sizeof(rgMainTransitionTests) / sizeof(rgMainTransitionTests[0]);

	for (size_t i = 0; i < cMainTransitionTests; i++) {
		const MainTransitionTest_t *test = &rgMainTransitionTests[i];

		// Setup initial machine state
		struct vehicle_status_s current_vehicle_status = {};
		struct commander_state_s current_commander_state = {};
		struct vehicle_status_flags_s current_status_flags = {};

		current_commander_state.main_state = test->from_state;
		current_vehicle_status.vehicle_type = (test->condition_bits & MTT_ROTARY_WING) ?
						      vehicle_status_s::VEHICLE_TYPE_ROTARY_WING : vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
		current_status_flags.condition_local_altitude_valid = test->condition_bits & MTT_LOC_ALT_VALID;
		current_status_flags.condition_local_position_valid = test->condition_bits & MTT_LOC_POS_VALID;
		current_status_flags.condition_home_position_valid = test->condition_bits & MTT_HOME_POS_VALID;
		current_status_flags.condition_global_position_valid = test->condition_bits & MTT_GLOBAL_POS_VALID;
		current_status_flags.condition_auto_mission_available = true;

		// Attempt transition
		transition_result_t result = main_state_transition(current_vehicle_status, test->to_state, current_status_flags,
					     current_commander_state);

		// Validate result of transition
		ut_compare(test->assertMsg, test->expected_transition_result, result);

		if (test->expected_transition_result == result) {
			if (test->expected_transition_result == TRANSITION_CHANGED) {
				ut_compare(test->assertMsg, test->to_state, current_commander_state.main_state);

			} else {
				ut_compare(test->assertMsg, test->from_state, current_commander_state.main_state);
			}
		}
	}

	return true;
}

bool StateMachineHelperTest::run_tests()
{
	ut_run_test(armingStateTransitionTest);
	ut_run_test(mainStateTransitionTest);

	return (_tests_failed == 0);
}

ut_declare_test(stateMachineHelperTest, StateMachineHelperTest)
