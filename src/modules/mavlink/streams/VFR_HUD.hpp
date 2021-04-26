/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#ifndef VFR_HUD_HPP
#define VFR_HUD_HPP

#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_local_position.h>

class MavlinkStreamVFRHUD : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamVFRHUD(mavlink); }

	static constexpr const char *get_name_static() { return "VFR_HUD"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_VFR_HUD; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		if (_lpos_sub.advertised() || _airspeed_validated_sub.advertised()) {
			return MAVLINK_MSG_ID_VFR_HUD_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		}

		return 0;
	}

private:
	explicit MavlinkStreamVFRHUD(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _act0_sub{ORB_ID(actuator_controls_0)};
	uORB::Subscription _act1_sub{ORB_ID(actuator_controls_1)};
	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};
	uORB::Subscription _air_data_sub{ORB_ID(vehicle_air_data)};

	bool send() override
	{
		if (_lpos_sub.updated() || _airspeed_validated_sub.updated()) {

			vehicle_local_position_s lpos{};
			_lpos_sub.copy(&lpos);

			actuator_armed_s armed{};
			_armed_sub.copy(&armed);

			airspeed_validated_s airspeed_validated{};
			_airspeed_validated_sub.copy(&airspeed_validated);

			mavlink_vfr_hud_t msg{};
			msg.airspeed = airspeed_validated.calibrated_airspeed_m_s;
			msg.groundspeed = sqrtf(lpos.vx * lpos.vx + lpos.vy * lpos.vy);
			msg.heading = math::degrees(matrix::wrap_2pi(lpos.heading));

			if (armed.armed) {
				actuator_controls_s act0{};
				actuator_controls_s act1{};
				_act0_sub.copy(&act0);
				_act1_sub.copy(&act1);

				// VFR_HUD throttle should only be used for operator feedback.
				// VTOLs switch between actuator_controls_0 and actuator_controls_1. During transition there isn't a
				// a single throttle value, but this should still be a useful heuristic for operator awareness.
				//
				// Use ACTUATOR_CONTROL_TARGET if accurate states are needed.
				msg.throttle = 100 * math::max(
						       act0.control[actuator_controls_s::INDEX_THROTTLE],
						       act1.control[actuator_controls_s::INDEX_THROTTLE]);

			} else {
				msg.throttle = 0.0f;
			}

			if (lpos.z_valid && lpos.z_global) {
				/* use local position estimate */
				msg.alt = -lpos.z + lpos.ref_alt;

			} else {
				vehicle_air_data_s air_data{};
				_air_data_sub.copy(&air_data);

				/* fall back to baro altitude */
				if (air_data.timestamp > 0) {
					msg.alt = air_data.baro_alt_meter;
				}
			}

			if (lpos.v_z_valid) {
				msg.climb = -lpos.vz;
			}

			mavlink_msg_vfr_hud_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // VFR_HUD_HPP
