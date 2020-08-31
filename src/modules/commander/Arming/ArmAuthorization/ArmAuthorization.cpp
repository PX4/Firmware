/****************************************************************************
 *
 *   Copyright (c) 2017  Intel Corporation. All rights reserved.
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
#include "ArmAuthorization.h"

#include <string.h>
#include <unistd.h>

#include <px4_platform_common/defines.h>
#include <px4_platform_common/px4_config.h>
#include <lib/parameters/param.h>
#include <systemlib/mavlink_log.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

static orb_advert_t *mavlink_log_pub;
static int command_ack_sub = -1;

static param_t param_arm_parameters;

static hrt_abstime auth_timeout;

static enum {
	ARM_AUTH_IDLE = 0,
	ARM_AUTH_WAITING_AUTH,
	ARM_AUTH_WAITING_AUTH_WITH_ACK,
	ARM_AUTH_MISSION_APPROVED
} state = ARM_AUTH_IDLE;

union {
	struct {
		uint8_t authorizer_system_id;
		union {
			uint16_t auth_method_arm_timeout_msec;
			uint16_t auth_method_two_arm_timeout_msec;
		} auth_method_param;
		uint8_t authentication_method;
	} __attribute__((packed)) struct_value;
	int32_t param_value;
} arm_parameters;

static uint8_t *system_id;

static uint8_t _auth_method_arm_req_check();
static uint8_t _auth_method_two_arm_check();

static uint8_t (*arm_check_method[ARM_AUTH_METHOD_LAST])() = {
	_auth_method_arm_req_check,
	_auth_method_two_arm_check,
};

static void arm_auth_request_msg_send()
{
	vehicle_command_s vcmd{};
	vcmd.timestamp = hrt_absolute_time();
	vcmd.command = vehicle_command_s::VEHICLE_CMD_ARM_AUTHORIZATION_REQUEST;
	vcmd.target_system = arm_parameters.struct_value.authorizer_system_id;

	uORB::PublicationQueued<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
	vcmd_pub.publish(vcmd);
}

static uint8_t _auth_method_arm_req_check()
{
	switch (state) {
	case ARM_AUTH_IDLE:
		/* no authentication in process? handle bellow */
		break;

	case ARM_AUTH_MISSION_APPROVED:
		return vehicle_command_ack_s::VEHICLE_RESULT_ACCEPTED;

	default:
		return vehicle_command_ack_s::VEHICLE_RESULT_DENIED;
	}

	/* handling ARM_AUTH_IDLE */
	arm_auth_request_msg_send();

	hrt_abstime now = hrt_absolute_time();
	auth_timeout = now + (arm_parameters.struct_value.auth_method_param.auth_method_arm_timeout_msec * 1000);
	state = ARM_AUTH_WAITING_AUTH;

	while (now < auth_timeout) {
		arm_auth_update(now);

		if (state != ARM_AUTH_WAITING_AUTH && state != ARM_AUTH_WAITING_AUTH_WITH_ACK) {
			break;
		}

		/* 0.5ms */
		px4_usleep(500);
		now = hrt_absolute_time();
	}

	switch (state) {
	case ARM_AUTH_WAITING_AUTH:
	case ARM_AUTH_WAITING_AUTH_WITH_ACK:
		state = ARM_AUTH_IDLE;
		mavlink_log_critical(mavlink_log_pub, "Arm auth: No response");
		break;

	default:
		break;
	}

	return state == ARM_AUTH_MISSION_APPROVED ?
	       vehicle_command_ack_s::VEHICLE_RESULT_ACCEPTED : vehicle_command_ack_s::VEHICLE_RESULT_DENIED;
}

static uint8_t _auth_method_two_arm_check()
{
	switch (state) {
	case ARM_AUTH_IDLE:
		/* no authentication in process? handle bellow */
		break;

	case ARM_AUTH_MISSION_APPROVED:
		return vehicle_command_ack_s::VEHICLE_RESULT_ACCEPTED;

	case ARM_AUTH_WAITING_AUTH:
	case ARM_AUTH_WAITING_AUTH_WITH_ACK:
		return vehicle_command_ack_s::VEHICLE_RESULT_TEMPORARILY_REJECTED;

	default:
		return vehicle_command_ack_s::VEHICLE_RESULT_DENIED;
	}

	/* handling ARM_AUTH_IDLE */
	arm_auth_request_msg_send();

	hrt_abstime now = hrt_absolute_time();
	auth_timeout = now + (arm_parameters.struct_value.auth_method_param.auth_method_arm_timeout_msec * 1000);
	state = ARM_AUTH_WAITING_AUTH;

	mavlink_log_critical(mavlink_log_pub, "Arm auth: Requesting authorization...");

	return vehicle_command_ack_s::VEHICLE_RESULT_TEMPORARILY_REJECTED;
}

uint8_t arm_auth_check()
{
	if (arm_parameters.struct_value.authentication_method < ARM_AUTH_METHOD_LAST) {
		return arm_check_method[arm_parameters.struct_value.authentication_method]();
	}

	return vehicle_command_ack_s::VEHICLE_RESULT_DENIED;
}

void arm_auth_update(hrt_abstime now, bool param_update)
{
	if (param_update) {
		param_get(param_arm_parameters, &arm_parameters.param_value);
	}

	switch (state) {
	case ARM_AUTH_WAITING_AUTH:
	case ARM_AUTH_WAITING_AUTH_WITH_ACK:
		/* handle bellow */
		break;

	case ARM_AUTH_MISSION_APPROVED:
		if (now > auth_timeout) {
			state = ARM_AUTH_IDLE;
		}

		return;

	case ARM_AUTH_IDLE:
	default:
		return;
	}

	/*
	 * handling ARM_AUTH_WAITING_AUTH, ARM_AUTH_WAITING_AUTH_WITH_ACK
	 */
	vehicle_command_ack_s command_ack;
	bool updated = false;

	orb_check(command_ack_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_command_ack), command_ack_sub, &command_ack);
	}

	if (updated
	    && command_ack.command == vehicle_command_s::VEHICLE_CMD_ARM_AUTHORIZATION_REQUEST
	    && command_ack.target_system == *system_id) {
		switch (command_ack.result) {
		case vehicle_command_ack_s::VEHICLE_RESULT_IN_PROGRESS:
			state = ARM_AUTH_WAITING_AUTH_WITH_ACK;
			break;

		case vehicle_command_ack_s::VEHICLE_RESULT_ACCEPTED:
			mavlink_log_critical(mavlink_log_pub,
					     "Arm auth: Authorized for the next %u seconds",
					     command_ack.result_param2);
			state = ARM_AUTH_MISSION_APPROVED;
			auth_timeout = now + (command_ack.result_param2 * 1000000);
			return;

		case vehicle_command_ack_s::VEHICLE_RESULT_TEMPORARILY_REJECTED:
			mavlink_log_critical(mavlink_log_pub, "Arm auth: Temporarily rejected");
			state = ARM_AUTH_IDLE;
			return;

		case vehicle_command_ack_s::VEHICLE_RESULT_DENIED:
		default:
			switch (command_ack.result_param1) {
			case vehicle_command_ack_s::ARM_AUTH_DENIED_REASON_NONE:
				/* Authorizer will send reason to ground station */
				break;

			case vehicle_command_ack_s::ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT:
				mavlink_log_critical(mavlink_log_pub, "Arm auth: Denied, waypoint %i have a invalid value", command_ack.result_param2);
				break;

			case vehicle_command_ack_s::ARM_AUTH_DENIED_REASON_TIMEOUT:
				mavlink_log_critical(mavlink_log_pub, "Arm auth: Denied by timeout in authorizer");
				break;

			case vehicle_command_ack_s::ARM_AUTH_DENIED_REASON_AIRSPACE_IN_USE:
				mavlink_log_critical(mavlink_log_pub, "Arm auth: Denied because airspace is in use");
				break;

			case vehicle_command_ack_s::ARM_AUTH_DENIED_REASON_BAD_WEATHER:
				mavlink_log_critical(mavlink_log_pub, "Arm auth: Denied because of bad weather");
				break;

			case vehicle_command_ack_s::ARM_AUTH_DENIED_REASON_GENERIC:
			default:
				mavlink_log_critical(mavlink_log_pub, "Arm auth: Denied");
			}

			state = ARM_AUTH_IDLE;
			return;
		}
	}

	if (now > auth_timeout) {
		mavlink_log_critical(mavlink_log_pub, "Arm auth: No response");
		state = ARM_AUTH_IDLE;
	}
}

void arm_auth_init(orb_advert_t *mav_log_pub, uint8_t *sys_id)
{
	system_id = sys_id;
	param_arm_parameters = param_find("COM_ARM_AUTH");
	command_ack_sub = orb_subscribe(ORB_ID(vehicle_command_ack));
	mavlink_log_pub = mav_log_pub;
}

enum arm_auth_methods arm_auth_method_get()
{
	return (enum arm_auth_methods) arm_parameters.struct_value.authentication_method;
}
