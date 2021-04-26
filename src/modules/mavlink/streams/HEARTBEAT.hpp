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

#ifndef HEARTBEAT_HPP
#define HEARTBEAT_HPP

#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_status_flags.h>

class MavlinkStreamHeartbeat : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamHeartbeat(mavlink); }

	static constexpr const char *get_name_static() { return "HEARTBEAT"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_HEARTBEAT; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	bool const_rate() override { return true; }

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_HEARTBEAT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	explicit MavlinkStreamHeartbeat(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_status_flags_sub{ORB_ID(vehicle_status_flags)};

	bool send() override
	{
		if (_mavlink->get_free_tx_buf() >= get_size()) {
			// always send the heartbeat, independent of the update status of the topics
			vehicle_status_s vehicle_status{};
			_vehicle_status_sub.copy(&vehicle_status);

			vehicle_status_flags_s vehicle_status_flags{};
			_vehicle_status_flags_sub.copy(&vehicle_status_flags);

			vehicle_control_mode_s vehicle_control_mode{};
			_vehicle_status_sub.copy(&vehicle_control_mode);


			// uint8_t base_mode (MAV_MODE_FLAG) - System mode bitmap.
			uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

			if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
				base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
			}

			if (vehicle_status.hil_state == vehicle_status_s::HIL_STATE_ON) {
				base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
			}

			if (vehicle_control_mode.flag_control_manual_enabled) {
				base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
			}

			if (vehicle_control_mode.flag_control_attitude_enabled) {
				base_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
			}

			if (vehicle_control_mode.flag_control_auto_enabled) {
				base_mode |= MAV_MODE_FLAG_AUTO_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED;
			}


			// uint32_t custom_mode - A bitfield for use for autopilot-specific flags
			union px4_custom_mode custom_mode {get_px4_custom_mode(vehicle_status.nav_state)};


			// uint8_t system_status (MAV_STATE) - System status flag.
			uint8_t system_status = MAV_STATE_UNINIT;

			switch (vehicle_status.arming_state) {
			case vehicle_status_s::ARMING_STATE_ARMED:
				system_status = vehicle_status.failsafe ? MAV_STATE_CRITICAL : MAV_STATE_ACTIVE;
				break;

			case vehicle_status_s::ARMING_STATE_STANDBY:
				system_status = MAV_STATE_STANDBY;
				break;

			case vehicle_status_s::ARMING_STATE_SHUTDOWN:
				system_status = MAV_STATE_POWEROFF;
				break;
			}

			// system_status overrides
			if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_TERMINATION) {
				system_status = MAV_STATE_FLIGHT_TERMINATION;

			} else if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL) {
				system_status = MAV_STATE_EMERGENCY;

			} else if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LANDGPSFAIL) {
				system_status = MAV_STATE_EMERGENCY;

			} else if (vehicle_status_flags.condition_calibration_enabled) {
				system_status = MAV_STATE_CALIBRATING;
			}


			mavlink_msg_heartbeat_send(_mavlink->get_channel(), _mavlink->get_system_type(), MAV_AUTOPILOT_PX4,
						   base_mode, custom_mode.data, system_status);

			return true;
		}

		return false;
	}
};

#endif // HEARTBEAT_HPP
