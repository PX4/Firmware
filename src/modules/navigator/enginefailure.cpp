/****************************************************************************
 *
 *   Copyright (c) 2013-2014 PX4 Development Team. All rights reserved.
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

/* @file enginefailure.cpp
* Helper class for a fixedwing engine failure mode
*
* @author Thomas Gubler <thomasgubler@gmail.com>
*/
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>

#include <systemlib/mavlink_log.h>
#include <systemlib/err.h>
#include <lib/ecl/geo/geo.h>
#include <navigator/navigation.h>

#include <uORB/uORB.h>
#include <uORB/topics/mission.h>

#include "navigator.h"
#include "enginefailure.h"

EngineFailure::EngineFailure(Navigator *navigator, NavigatorCore &navigator_core) :
	MissionBlock(navigator, navigator_core),
	_ef_state(EF_STATE_NONE)
{
}

void
EngineFailure::on_inactive()
{
	_ef_state = EF_STATE_NONE;
}

void
EngineFailure::on_activation()
{
	_ef_state = EF_STATE_NONE;
	advance_ef();
	set_ef_item();
}

void
EngineFailure::on_active()
{
	if (is_mission_item_reached()) {
		advance_ef();
		set_ef_item();
	}
}

void
EngineFailure::set_ef_item()
{
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	pos_sp_triplet->previous = pos_sp_triplet->current;
	_navigator->set_can_loiter_at_sp(false);

	switch (_ef_state) {
	case EF_STATE_LOITERDOWN: {
			//XXX create mission item at ground (below?) here

			_mission_item.lat = _navigator_core.getLatRad();
			_mission_item.lon = _navigator_core.getLonRad();
			_mission_item.altitude_is_relative = false;
			//XXX setting altitude to a very low value, evaluate other options
			_mission_item.altitude = _navigator_core.getHomeAltAMSLMeter() - 1000.0f;
			_mission_item.yaw = NAN;
			_mission_item.loiter_radius = _navigator_core.getLoiterRadiusMeter();
			_mission_item.nav_cmd = NAV_CMD_LOITER_UNLIMITED;
			_mission_item.acceptance_radius = _navigator_core.getAcceptanceRadiusMeter();
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;

			_navigator->set_can_loiter_at_sp(true);
			break;
		}

	default:
		break;
	}

	reset_mission_item_reached();

	/* convert mission item to current position setpoint and make it valid */
	mission_apply_limitation(_mission_item);
	mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->next.valid = false;

	_navigator->set_position_setpoint_triplet_updated();
}

void
EngineFailure::advance_ef()
{
	switch (_ef_state) {
	case EF_STATE_NONE:
		mavlink_log_emergency(_navigator->get_mavlink_log_pub(), "Engine failure. Loitering down");
		_ef_state = EF_STATE_LOITERDOWN;
		break;

	default:
		break;
	}
}
