/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#ifndef HIL_ACTUATOR_CONTROLS_HPP
#define HIL_ACTUATOR_CONTROLS_HPP

#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>

class MavlinkStreamHILActuatorControls : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamHILActuatorControls(mavlink); }

	static constexpr const char *get_name_static() { return "HIL_ACTUATOR_CONTROLS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _act_sub.advertised() ? MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamHILActuatorControls(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _act_sub{ORB_ID(actuator_outputs)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};

	bool send() override
	{
		actuator_outputs_s act;

		if (_act_sub.update(&act)) {
			mavlink_hil_actuator_controls_t msg{};
			msg.time_usec = act.timestamp;

			static constexpr float pwm_center = (PWM_DEFAULT_MAX + PWM_DEFAULT_MIN) / 2;

			unsigned system_type = _mavlink->get_system_type();

			/* scale outputs depending on system type */
			if (system_type == MAV_TYPE_QUADROTOR ||
			    system_type == MAV_TYPE_HEXAROTOR ||
			    system_type == MAV_TYPE_OCTOROTOR ||
			    system_type == MAV_TYPE_VTOL_DUOROTOR ||
			    system_type == MAV_TYPE_VTOL_QUADROTOR ||
			    system_type == MAV_TYPE_VTOL_RESERVED2) {

				/* multirotors: set number of rotor outputs depending on type */

				unsigned n;

				switch (system_type) {
				case MAV_TYPE_QUADROTOR:
					n = 4;
					break;

				case MAV_TYPE_HEXAROTOR:
					n = 6;
					break;

				case MAV_TYPE_VTOL_DUOROTOR:
					n = 2;
					break;

				case MAV_TYPE_VTOL_QUADROTOR:
					n = 4;
					break;

				case MAV_TYPE_VTOL_RESERVED2:
					n = 8;
					break;

				default:
					n = 8;
					break;
				}

				for (unsigned i = 0; i < 16; i++) {
					if (act.output[i] > PWM_DEFAULT_MIN / 2) {
						if (i < n) {
							/* scale PWM out 900..2100 us to 0..1 for rotors */
							msg.controls[i] = (act.output[i] - PWM_DEFAULT_MIN) / (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN);

						} else {
							/* scale PWM out 900..2100 us to -1..1 for other channels */
							msg.controls[i] = (act.output[i] - pwm_center) / ((PWM_DEFAULT_MAX - PWM_DEFAULT_MIN) / 2);
						}

					} else {
						/* send 0 when disarmed and for disabled channels */
						msg.controls[i] = 0.0f;
					}
				}

			} else {
				/* fixed wing: scale throttle to 0..1 and other channels to -1..1 */
				for (unsigned i = 0; i < 16; i++) {
					if (act.output[i] > PWM_DEFAULT_MIN / 2) {
						if (i != 3) {
							/* scale PWM out 900..2100 us to -1..1 for normal channels */
							msg.controls[i] = (act.output[i] - pwm_center) / ((PWM_DEFAULT_MAX - PWM_DEFAULT_MIN) / 2);

						} else {
							/* scale PWM out 900..2100 us to 0..1 for throttle */
							msg.controls[i] = (act.output[i] - PWM_DEFAULT_MIN) / (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN);
						}

					} else {
						/* set 0 for disabled channels */
						msg.controls[i] = 0.0f;
					}
				}
			}

			// mode (MAV_MODE_FLAG)
			msg.mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

			vehicle_control_mode_s control_mode;

			if (_vehicle_control_mode_sub.copy(&control_mode)) {
				if (control_mode.flag_control_auto_enabled) {
					msg.mode |= MAV_MODE_FLAG_AUTO_ENABLED;
				}

				if (control_mode.flag_control_manual_enabled) {
					msg.mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
				}

				if (control_mode.flag_control_attitude_enabled) {
					msg.mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
				}
			}

			vehicle_status_s status;

			if (_vehicle_status_sub.copy(&status)) {
				if (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
					msg.mode |= MAV_MODE_FLAG_SAFETY_ARMED;
				}

				if (status.hil_state == vehicle_status_s::HIL_STATE_ON) {
					msg.mode |= MAV_MODE_FLAG_HIL_ENABLED;
				}

				if (status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) {
					msg.mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
				}
			}

			msg.flags = 0;

			mavlink_msg_hil_actuator_controls_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // HIL_ACTUATOR_CONTROLS_HPP
