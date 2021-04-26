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

#ifndef SCALED_PRESSURE3_HPP
#define SCALED_PRESSURE3_HPP

#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/sensor_baro.h>

class MavlinkStreamScaledPressure3 : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamScaledPressure3(mavlink); }

	static constexpr const char *get_name_static() { return "SCALED_PRESSURE3"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_SCALED_PRESSURE3; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		if (_sensor_baro_sub.advertised() || _differential_pressure_sub.advertised()) {
			return MAVLINK_MSG_ID_SCALED_PRESSURE3_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		}

		return 0;
	}

private:
	explicit MavlinkStreamScaledPressure3(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _differential_pressure_sub{ORB_ID(differential_pressure), 2};
	uORB::Subscription _sensor_baro_sub{ORB_ID(sensor_baro), 2};

	bool send() override
	{
		if (_sensor_baro_sub.updated() || _differential_pressure_sub.updated()) {
			mavlink_scaled_pressure3_t msg{};

			sensor_baro_s sensor_baro;

			if (_sensor_baro_sub.copy(&sensor_baro)) {
				msg.time_boot_ms = sensor_baro.timestamp / 1000;
				msg.press_abs = sensor_baro.pressure; // millibar to hPa
				msg.temperature = roundf(sensor_baro.temperature * 100.f); // centidegrees
			}

			differential_pressure_s differential_pressure;

			if (_differential_pressure_sub.copy(&differential_pressure)) {
				if (msg.time_boot_ms == 0) {
					msg.time_boot_ms = differential_pressure.timestamp / 1000;
				}

				msg.press_diff = differential_pressure.differential_pressure_raw_pa * 100.f; // Pa to hPa
				msg.temperature_press_diff = roundf(differential_pressure.temperature * 100.f); // centidegrees
			}

			mavlink_msg_scaled_pressure3_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // SCALED_PRESSURE3_HPP
