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

#include "FakeGyro.hpp"

using namespace time_literals;

FakeGyro::FakeGyro() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_px4_gyro(1310988) // 1310988: DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
{
	_px4_gyro.set_scale(math::radians(2000.f) / static_cast<float>(INT16_MAX - 1)); // 2000 degrees/second max
}

bool FakeGyro::init()
{
	ScheduleOnInterval(SENSOR_RATE, SENSOR_RATE);
	return true;
}

void FakeGyro::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	sensor_gyro_fifo_s gyro{};
	gyro.timestamp_sample = hrt_absolute_time();
	gyro.samples = GYRO_RATE / (1e6f / SENSOR_RATE);
	gyro.dt = 1e6f / GYRO_RATE; // 8 kHz fake gyro;

	const float dt_s = gyro.dt * 1e-6f;
	const float x_freq = 15.f;  //  15,0 Hz X frequency
	const float y_freq = 63.5f; //  63.5 Hz Y frequency
	const float z_freq = 135.f; // 135.0 Hz Z frequency

	for (int n = 0; n < gyro.samples; n++) {
		_time += dt_s;
		const float k = 2.f * M_PI_F * _time;

		gyro.x[n] = (INT16_MAX - 1) * sinf(k * x_freq);
		gyro.y[n] = (INT16_MAX - 1) / 2 * sinf(k * y_freq);
		gyro.z[n] = (INT16_MAX - 1) * cosf(k * z_freq);
	}

	_px4_gyro.updateFIFO(gyro);
}

int FakeGyro::task_spawn(int argc, char *argv[])
{
	FakeGyro *instance = new FakeGyro();

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

int FakeGyro::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FakeGyro::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fake_gyro", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int fake_gyro_main(int argc, char *argv[])
{
	return FakeGyro::main(argc, argv);
}
