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

#include "FakeMagnetometer.hpp"

#include <lib/ecl/geo_lookup/geo_mag_declination.h>

using namespace matrix;
using namespace time_literals;

FakeMagnetometer::FakeMagnetometer() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_px4_mag(0, ROTATION_NONE)
{
	_px4_mag.set_device_type(DRV_MAG_DEVTYPE_MAGSIM);
	_px4_mag.set_external(false);
}

bool FakeMagnetometer::init()
{
	ScheduleOnInterval(10_ms); // 100 Hz
	return true;
}

void FakeMagnetometer::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if (_vehicle_gps_position_sub.updated()) {
		vehicle_gps_position_s gps;

		if (_vehicle_gps_position_sub.copy(&gps)) {
			if (gps.eph < 1000) {

				const double lat = gps.lat / 1.e7;
				const double lon = gps.lon / 1.e7;

				// magnetic field data returned by the geo library using the current GPS position
				const float mag_declination_gps = get_mag_declination_radians(lat, lon);
				const float mag_inclination_gps = get_mag_inclination_radians(lat, lon);
				const float mag_strength_gps = get_mag_strength_gauss(lat, lon);

				_mag_earth_pred = Dcmf(Eulerf(0, -mag_inclination_gps, mag_declination_gps)) * Vector3f(mag_strength_gps, 0, 0);

				_mag_earth_available = true;
			}
		}
	}

	if (_mag_earth_available) {
		vehicle_attitude_s attitude;

		if (_vehicle_attitude_sub.update(&attitude)) {
			Vector3f expected_field = Dcmf{Quatf{attitude.q}} .transpose() * _mag_earth_pred;

			_px4_mag.update(hrt_absolute_time(), expected_field(0), expected_field(1), expected_field(2));
		}
	}
}

int FakeMagnetometer::task_spawn(int argc, char *argv[])
{
	FakeMagnetometer *instance = new FakeMagnetometer();

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

int FakeMagnetometer::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FakeMagnetometer::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Publish the earth magnetic field as a fake magnetometer (sensor_mag).
Requires vehicle_attitude and vehicle_gps_position.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fake_magnetometer", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int fake_magnetometer_main(int argc, char *argv[])
{
	return FakeMagnetometer::main(argc, argv);
}
