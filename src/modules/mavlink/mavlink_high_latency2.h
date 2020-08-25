/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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
 * @file mavlink_high_latency2.h
 *
 * @author Achermann Florian <acfloria@ethz.ch>
 */

#pragma once

#include "mavlink_main.h"
#include "mavlink_messages.h"
#include "mavlink_simple_analyzer.h"
#include "mavlink_stream.h"

#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/geofence_result.h>
#include <uORB/topics/position_controller_status.h>
#include <uORB/topics/tecs_status.h>
#include <uORB/topics/wind_estimate.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_status_flags.h>

class MavlinkStreamHighLatency2 : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamHighLatency2::get_name_static();
	}

	static const char *get_name_static()
	{
		return "HIGH_LATENCY2";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_HIGH_LATENCY2;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamHighLatency2(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_HIGH_LATENCY2_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

	bool const_rate()
	{
		return true;
	}

private:

	struct PerBatteryData {
		PerBatteryData(uint8_t instance) : subscription(ORB_ID(battery_status), instance) {}
		uORB::Subscription subscription;
		SimpleAnalyzer analyzer{SimpleAnalyzer::AVERAGE};
		uint64_t timestamp{0};
		bool connected{false};
	};

	uORB::Subscription _actuator_0_sub{ORB_ID(actuator_controls_0)};
	uORB::Subscription _actuator_1_sub{ORB_ID(actuator_controls_1)};
	uORB::Subscription _airspeed_sub{ORB_ID(airspeed)};
	uORB::Subscription _attitude_sp_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _estimator_status_sub{ORB_ID(estimator_status)};
	uORB::Subscription _pos_ctrl_status_sub{ORB_ID(position_controller_status)};
	uORB::Subscription _geofence_sub{ORB_ID(geofence_result)};
	uORB::Subscription _global_pos_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _gps_sub{ORB_ID(vehicle_gps_position)};
	uORB::Subscription _mission_result_sub{ORB_ID(mission_result)};
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _status_flags_sub{ORB_ID(vehicle_status_flags)};
	uORB::Subscription _tecs_status_sub{ORB_ID(tecs_status)};
	uORB::Subscription _wind_sub{ORB_ID(wind_estimate)};

	SimpleAnalyzer _airspeed;
	SimpleAnalyzer _airspeed_sp;
	SimpleAnalyzer _climb_rate;
	SimpleAnalyzer _eph;
	SimpleAnalyzer _epv;
	SimpleAnalyzer _groundspeed;
	SimpleAnalyzer _temperature;
	SimpleAnalyzer _throttle;
	SimpleAnalyzer _windspeed;

	hrt_abstime _last_reset_time = 0;
	hrt_abstime _last_update_time = 0;
	float _update_rate_filtered = 0.0f;

	PerBatteryData _batteries[ORB_MULTI_MAX_INSTANCES] {0, 1, 2, 3};

	/* do not allow top copying this class */
	MavlinkStreamHighLatency2(MavlinkStreamHighLatency2 &);
	MavlinkStreamHighLatency2 &operator = (const MavlinkStreamHighLatency2 &);

protected:
	explicit MavlinkStreamHighLatency2(Mavlink *mavlink);

	bool send(const hrt_abstime t);

	void reset_analysers(const hrt_abstime t);

	bool write_airspeed(mavlink_high_latency2_t *msg);

	bool write_attitude_sp(mavlink_high_latency2_t *msg);

	bool write_battery_status(mavlink_high_latency2_t *msg);

	bool write_estimator_status(mavlink_high_latency2_t *msg);

	bool write_fw_ctrl_status(mavlink_high_latency2_t *msg);

	bool write_geofence_result(mavlink_high_latency2_t *msg);

	bool write_global_position(mavlink_high_latency2_t *msg);

	bool write_mission_result(mavlink_high_latency2_t *msg);

	bool write_tecs_status(mavlink_high_latency2_t *msg);

	bool write_vehicle_status(mavlink_high_latency2_t *msg);

	bool write_vehicle_status_flags(mavlink_high_latency2_t *msg);

	bool write_wind_estimate(mavlink_high_latency2_t *msg);

	void update_data();

	void update_airspeed();

	void update_tecs_status();

	void update_battery_status();

	void update_local_position();

	void update_gps();

	void update_vehicle_status();

	void update_wind_estimate();

	void set_default_values(mavlink_high_latency2_t &msg) const;
};
