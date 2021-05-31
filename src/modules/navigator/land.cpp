/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file land.cpp
 *
 * Helper class to land at the current position
 *
 * @author Andreas Antener <andreas@uaventure.com>
 */

#include "land.h"
#include "navigator.h"

Land::Land(Navigator *navigator, NavigatorCore &navigator_core) :
	MissionBlock(navigator, navigator_core)
{
}

void
Land::on_activation()
{
	/* set current mission item to Land */
	set_land_item(&_mission_item, true);
	_navigator->get_mission_result()->finished = false;
	_navigator->set_mission_result_updated();
	reset_mission_item_reached();

	/* convert mission item to current setpoint */
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	pos_sp_triplet->previous.valid = false;
	mission_apply_limitation(_mission_item);
	mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->next.valid = false;

	_navigator->set_can_loiter_at_sp(false);

	_navigator->set_position_setpoint_triplet_updated();
}

void
Land::on_active()
{
	/* for VTOL update landing location during back transition */
	if (_navigator_core.isVTOL() &&
	    _navigator_core.getIsInTransitionMode()) {
		struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
		pos_sp_triplet->current.lat = _navigator_core.getLatRad();
		pos_sp_triplet->current.lon = _navigator_core.getLonRad();
		_navigator->set_position_setpoint_triplet_updated();
	}


	if (_navigator_core.getLanded()) {
		_navigator->get_mission_result()->finished = true;
		_navigator->set_mission_result_updated();
		set_idle_item(&_mission_item);

		struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
		mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
		_navigator->set_position_setpoint_triplet_updated();
	}
}
