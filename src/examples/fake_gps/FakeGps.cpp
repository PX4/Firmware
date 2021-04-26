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

#include "FakeGps.hpp"

using namespace time_literals;

FakeGps::FakeGps(double latitude_deg, double longitude_deg, float altitude_m) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_latitude(latitude_deg * 10e6),
	_longitude(longitude_deg * 10e6),
	_altitude(altitude_m * 10e2f)
{
}

bool FakeGps::init()
{
	ScheduleOnInterval(SENSOR_INTERVAL_US);
	return true;
}

void FakeGps::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	sensor_gps_s sensor_gps{};
	sensor_gps.time_utc_usec = hrt_absolute_time() + 1613692609599954;
	sensor_gps.lat = _latitude;
	sensor_gps.lon = _longitude;
	sensor_gps.alt = _altitude;
	sensor_gps.alt_ellipsoid = _altitude;
	sensor_gps.s_variance_m_s = 0.3740f;
	sensor_gps.c_variance_rad = 0.6737f;
	sensor_gps.eph = 2.1060f;
	sensor_gps.epv = 3.8470f;
	sensor_gps.hdop = 0.8800f;
	sensor_gps.vdop = 1.3300f;
	sensor_gps.noise_per_ms = 101;
	sensor_gps.jamming_indicator = 35;
	sensor_gps.vel_m_s = 0.0420f;
	sensor_gps.vel_n_m_s = 0.0370f;
	sensor_gps.vel_e_m_s = 0.0200f;
	sensor_gps.vel_d_m_s = -0.0570f;
	sensor_gps.cog_rad = 0.3988f;
	sensor_gps.timestamp_time_relative = 0;
	sensor_gps.heading = NAN;
	sensor_gps.heading_offset = 0.0000;
	sensor_gps.fix_type = 4;
	sensor_gps.jamming_state = 0;
	sensor_gps.vel_ned_valid = true;
	sensor_gps.satellites_used = 14;
	sensor_gps.timestamp = hrt_absolute_time();
	_sensor_gps_pub.publish(sensor_gps);
}

int FakeGps::task_spawn(int argc, char *argv[])
{
	FakeGps *instance = new FakeGps();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int FakeGps::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FakeGps::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fake_gps", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int fake_gps_main(int argc, char *argv[])
{
	return FakeGps::main(argc, argv);
}
