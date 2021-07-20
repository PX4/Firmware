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

/* Include Files */
#include "AFBRS50.hpp"

#include <lib/drivers/device/Device.hpp>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

#define AFBRS50_MEASURE_INTERVAL     (1000000 / 100) // 100Hz

/*! Define the SPI baud rate (to be used in the SPI module). */
#define SPI_BAUD_RATE 5000000

#include "s2pi.h"
#include "timer.h"
#include "argus_hal_test.h"

AFBRS50 *g_dev{nullptr};

AFBRS50::AFBRS50(uint8_t device_orientation):
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_px4_rangefinder(0, device_orientation)
{
	device::Device::DeviceId device_id{};
	device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SPI;
	device_id.devid_s.bus = BROADCOM_AFBR_S50_S2PI_SPI_BUS;
	device_id.devid_s.devtype = DRV_DIST_DEVTYPE_AFBRS50;

	_px4_rangefinder.set_device_id(device_id.devid);
}

AFBRS50::~AFBRS50()
{
	stop();

	perf_free(_sample_perf);
}

status_t AFBRS50::measurement_ready_callback(status_t status, void *data)
{
	if (!up_interrupt_context()) {
		if (status == STATUS_OK) {
			if (g_dev) {
				g_dev->ProcessMeasurement(data);
			}
		}
	}

	return status;
}

void AFBRS50::ProcessMeasurement(void *data)
{
	if (data != nullptr) {
		perf_count(_sample_perf);

		argus_results_t res{};
		status_t evaluate_status = Argus_EvaluateData(_hnd, &res, data);

		if ((evaluate_status == STATUS_OK) && (res.Status == 0)) {
			uint32_t result_mm = res.Bin.Range / (Q9_22_ONE / 1000);
			float result_m = static_cast<float>(result_mm) / 1000.f;
			int8_t quality = 100;

			// distance quality check
			if (result_m < _min_distance || result_m > _max_distance) {
				result_m = 0.0;
				quality = 0;
			}

			_px4_rangefinder.update(((res.TimeStamp.sec * 1000000ULL) + res.TimeStamp.usec), result_m, quality);
		}
	}
}

int AFBRS50::init()
{
	if (_hnd != nullptr) {
		// retry
		Argus_Deinit(_hnd);
		Argus_DestroyHandle(_hnd);
		_hnd = nullptr;
	}

	_hnd = Argus_CreateHandle();

	if (_hnd == nullptr) {
		PX4_ERR("Handle not initialized");
		return PX4_ERROR;
	}

	// Initialize the S2PI hardware required by the API.
	S2PI_Init(BROADCOM_AFBR_S50_S2PI_SPI_BUS, SPI_BAUD_RATE);

	status_t status = Argus_Init(_hnd, BROADCOM_AFBR_S50_S2PI_SPI_BUS);

	if (status == STATUS_OK) {
		uint32_t id = Argus_GetChipID(_hnd);
		uint32_t value = Argus_GetAPIVersion();
		uint8_t a = (value >> 24) & 0xFFU;
		uint8_t b = (value >> 16) & 0xFFU;
		uint8_t c = value & 0xFFFFU;
		PX4_INFO_RAW("AFBR-S50 Chip ID: %"  PRId32 ", API Version: %"  PRId32 " v%"  PRId8 ".%"  PRId8 ".%"  PRId8 "\n", id,
			     value, a, b, c);

		argus_module_version_t mv = Argus_GetModuleVersion(_hnd);

		switch (mv) {
		case AFBR_S50MV85G_V1:

		// FALLTHROUGH
		case AFBR_S50MV85G_V2:

		// FALLTHROUGH
		case AFBR_S50MV85G_V3:
			_min_distance = 0.08f;
			_max_distance = 10.f;
			_px4_rangefinder.set_min_distance(_min_distance);
			_px4_rangefinder.set_max_distance(_max_distance);
			_px4_rangefinder.set_fov(math::radians(6.f));
			PX4_INFO_RAW("AFBR-S50MV85G\n");
			break;

		case AFBR_S50LV85D_V1:
			_min_distance = 0.08f;
			_max_distance = 30.f;	// Short range mode
			_px4_rangefinder.set_min_distance(_min_distance);
			_px4_rangefinder.set_max_distance(_max_distance);
			_px4_rangefinder.set_fov(math::radians(6.f));
			PX4_INFO_RAW("AFBR-S50LV85D (v1)\n");
			break;

		case AFBR_S50MV68B_V1:
			_min_distance = 0.08f;
			_max_distance = 10.f;
			_px4_rangefinder.set_min_distance(_min_distance);
			_px4_rangefinder.set_max_distance(_max_distance);
			_px4_rangefinder.set_fov(math::radians(1.f));
			PX4_INFO_RAW("AFBR-S50MV68B (v1)\n");
			break;

		case AFBR_S50MV85I_V1:
			_min_distance = 0.08f;
			_max_distance = 5.f;
			_px4_rangefinder.set_min_distance(_min_distance);
			_px4_rangefinder.set_max_distance(_max_distance);
			_px4_rangefinder.set_fov(math::radians(6.f));
			PX4_INFO_RAW("AFBR-S50MV85I (v1)\n");
			break;

		case AFBR_S50SV85K_V1:
			_min_distance = 0.08f;
			_max_distance = 10.f;
			_px4_rangefinder.set_min_distance(_min_distance);
			_px4_rangefinder.set_max_distance(_max_distance);
			_px4_rangefinder.set_fov(math::radians(4.f));
			PX4_INFO_RAW("AFBR-S50SV85K (v1)\n");
			break;

		default:
			break;
		}

		_state = STATE::CONFIGURE;
		ScheduleDelayed(AFBRS50_MEASURE_INTERVAL);
		return PX4_OK;
	}

	return PX4_ERROR;
}

void AFBRS50::Run()
{
	// backup schedule
	ScheduleDelayed(100_ms);

	switch (_state) {
	case STATE::TEST: {
			Argus_VerifyHALImplementation(Argus_GetSPISlave(_hnd));

			_state = STATE::CONFIGURE;
			ScheduleDelayed(100_ms);
		}
		break;

	case STATE::CONFIGURE: {
			Argus_SetConfigurationFrameTime(_hnd, AFBRS50_MEASURE_INTERVAL);

			status_t status = Argus_StartMeasurementTimer(_hnd, measurement_ready_callback);

			if (status != STATUS_OK) {
				PX4_ERR("CONFIGURE status not okay: %"  PRIi32, status);
				_state = STATE::STOP;
				ScheduleNow();

			} else {
				_state = STATE::COLLECT;
				ScheduleDelayed(AFBRS50_MEASURE_INTERVAL);
			}
		}
		break;

	case STATE::COLLECT: {
			// currently handeled by measurement_ready_callback
		}
		break;

	case STATE::STOP: {
			Argus_StopMeasurementTimer(_hnd);
			Argus_Deinit(_hnd);
			Argus_DestroyHandle(_hnd);
		}
		break;

	default:
		break;
	}
}

void AFBRS50::stop()
{
	_state = STATE::STOP;
	ScheduleNow();
}

void AFBRS50::print_info()
{
	perf_print_counter(_sample_perf);
}

namespace afbrs50
{

static int start(const uint8_t rotation)
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_ERROR;
	}

	g_dev = new AFBRS50(rotation);

	if (g_dev == nullptr) {
		PX4_ERR("object instantiate failed");
		return PX4_ERROR;
	}

	// Initialize the sensor.
	if (g_dev->init() != PX4_OK) {
		PX4_ERR("driver start failed");
		delete g_dev;
		g_dev = nullptr;
		return PX4_ERROR;
	}

	return PX4_OK;
}

static int status()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	g_dev->print_info();

	return PX4_OK;
}

static int stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	}

	PX4_INFO("driver stopped");
	return PX4_OK;
}

static int usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Driver for the Broadcom AFBRS50.

### Examples

Attempt to start driver on a specified serial device.
$ afbrs50 start
Stop driver
$ afbrs50 stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("afbrs50", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", false);
	PRINT_MODULE_USAGE_PARAM_INT('r', 25, 0, 25, "Sensor rotation - downward facing by default", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop driver");
	return PX4_OK;
}

} // namespace

extern "C" __EXPORT int afbrs50_main(int argc, char *argv[])
{
	const char *myoptarg = nullptr;

	int ch = 0;
	int myoptind = 1;

	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;

	while ((ch = px4_getopt(argc, argv, "d:r", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'r':
			rotation = (uint8_t)atoi(myoptarg);
			break;

		default:
			PX4_WARN("Unknown option");
			return afbrs50::usage();
		}
	}

	if (myoptind >= argc) {
		return afbrs50::usage();
	}

	if (!strcmp(argv[myoptind], "start")) {
		return afbrs50::start(rotation);

	} else if (!strcmp(argv[myoptind], "status")) {
		return afbrs50::status();

	} else if (!strcmp(argv[myoptind], "stop")) {
		return afbrs50::stop();
	}

	return afbrs50::usage();
}
