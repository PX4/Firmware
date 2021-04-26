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
/**
 * @file gpsfailure.cpp
 * Helper class for gpsfailure mode according to the OBC rules
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#include "gpsfailure.h"
#include "navigator.h"

#include <systemlib/mavlink_log.h>
#include <lib/ecl/geo/geo.h>
#include <navigator/navigation.h>
#include <uORB/uORB.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <mathlib/mathlib.h>

using matrix::Eulerf;
using matrix::Quatf;

GpsFailure::GpsFailure(Navigator *navigator) :
	MissionBlock(navigator),
	ModuleParams(navigator)
{
}

void
GpsFailure::on_inactive()
{
	/* reset GPSF state only if setpoint moved */
	if (!_navigator->get_can_loiter_at_sp()) {
		_gpsf_state = GPSF_STATE_NONE;
	}
}

void
GpsFailure::on_activation()
{
	_gpsf_state = GPSF_STATE_NONE;
	_timestamp_activation = hrt_absolute_time();
	advance_gpsf();
	set_gpsf_item();
}

void
GpsFailure::on_active()
{
	switch (_gpsf_state) {
	case GPSF_STATE_LOITER: {
			/* Position controller does not run in this mode:
			 * navigator has to publish an attitude setpoint */
			vehicle_attitude_setpoint_s att_sp = {};
			att_sp.timestamp = hrt_absolute_time();
			att_sp.roll_body = math::radians(_param_nav_gpsf_r.get());
			att_sp.pitch_body = math::radians(_param_nav_gpsf_p.get());
			att_sp.thrust_body[0] = _param_nav_gpsf_tr.get();

			Quatf q(Eulerf(att_sp.roll_body, att_sp.pitch_body, 0.0f));
			q.copyTo(att_sp.q_d);

			if (_navigator->get_vstatus()->is_vtol) {
				_fw_virtual_att_sp_pub.publish(att_sp);

			} else {
				_att_sp_pub.publish(att_sp);

			}

			/* Measure time */
			if ((_param_nav_gpsf_lt.get() > FLT_EPSILON) &&
			    (hrt_elapsed_time(&_timestamp_activation) > _param_nav_gpsf_lt.get() * 1e6f)) {
				/* no recovery, advance the state machine */
				PX4_WARN("GPS not recovered, switching to next failure state");
				advance_gpsf();
			}

			break;
		}

	case GPSF_STATE_TERMINATE:
		set_gpsf_item();
		advance_gpsf();
		break;

	default:
		break;
	}
}

void
GpsFailure::set_gpsf_item()
{
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	/* Set pos sp triplet to invalid to stop pos controller */
	pos_sp_triplet->previous.valid = false;
	pos_sp_triplet->current.valid = false;
	pos_sp_triplet->next.valid = false;

	switch (_gpsf_state) {
	case GPSF_STATE_TERMINATE: {
			/* Request flight termination from commander */
			_navigator->get_mission_result()->flight_termination = true;
			_navigator->set_mission_result_updated();
			PX4_WARN("GPS failure: request flight termination");
		}
		break;

	default:
		break;
	}

	_navigator->set_position_setpoint_triplet_updated();
}

void
GpsFailure::advance_gpsf()
{
	switch (_gpsf_state) {
	case GPSF_STATE_NONE:
		_gpsf_state = GPSF_STATE_LOITER;
		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Global position failure: loitering");
		break;

	case GPSF_STATE_LOITER:
		_gpsf_state = GPSF_STATE_TERMINATE;
		mavlink_log_emergency(_navigator->get_mavlink_log_pub(), "no GPS recovery, terminating flight");
		break;

	case GPSF_STATE_TERMINATE:
		PX4_WARN("terminate");
		_gpsf_state = GPSF_STATE_END;
		break;

	default:
		break;
	}
}
