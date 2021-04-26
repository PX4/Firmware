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


/**
 * @file SRF05.c
 * @author David Sidrane <david.sidrane@nscdg.com>
 *
 * Interface for the HY-SRF05 / HC-SR05 and HC-SR04.
 * Precise Ultrasonic Range Sensor Module
 */

#include "SR05.hpp"

#if defined(HAVE_ULTRASOUND)

#include <px4_arch/micro_hal.h>

SRF05::SRF05(const uint8_t rotation) :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_px4_rangefinder(0 /* no device type for GPIO input */, rotation)
{
	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_SRF05);
	_px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND);
	_px4_rangefinder.set_min_distance(HXSRX0X_MIN_DISTANCE);
	_px4_rangefinder.set_max_distance(HXSRX0X_MAX_DISTANCE);
	_px4_rangefinder.set_fov(0.261799); // 15 degree FOV
}

SRF05::~SRF05()
{
	stop();
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_sensor_resets);
}

void SRF05::OnEdge(bool state)
{
	const hrt_abstime now = hrt_absolute_time();

	if (_state == STATE::WAIT_FOR_RISING || _state == STATE::WAIT_FOR_FALLING) {
		if (state) {
			px4_arch_gpiowrite(GPIO_ULTRASOUND_TRIGGER, 1);
			_rising_edge_time = now;
			_state = STATE::WAIT_FOR_FALLING;

		} else {
			_falling_edge_time = now;
			_state = STATE::SAMPLE;
			ScheduleNow();
		}
	}
}

int SRF05::EchoInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<SRF05 *>(arg)->OnEdge(px4_arch_gpioread(GPIO_ULTRASOUND_ECHO));
	return 0;
}

int
SRF05::init()
{
	px4_arch_configgpio(GPIO_ULTRASOUND_TRIGGER);
	px4_arch_configgpio(GPIO_ULTRASOUND_ECHO);
	px4_arch_gpiowrite(GPIO_ULTRASOUND_TRIGGER, 1);
	px4_arch_gpiosetevent(GPIO_ULTRASOUND_ECHO, true, true, false, &EchoInterruptCallback, this);
	_state = STATE::TRIGGER;
	ScheduleOnInterval(get_measure_interval());
	return PX4_OK;
}

void SRF05::stop()
{
	_state = STATE::EXIT;
	px4_arch_gpiosetevent(GPIO_ULTRASOUND_ECHO, false, false, false, nullptr, nullptr);
	px4_arch_gpiowrite(GPIO_ULTRASOUND_TRIGGER, 1);
	ScheduleClear();
}

void
SRF05::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	switch (_state) {

	case STATE::TRIGGER:
		_state = STATE::WAIT_FOR_RISING;
		px4_arch_gpiowrite(GPIO_ULTRASOUND_TRIGGER, 0); // ya ya I know they're wrong! It triggers on the falling edge.
		break;

	case STATE::WAIT_FOR_RISING:
	case STATE::WAIT_FOR_FALLING:
		_state = STATE::TRIGGER;
		perf_count(_sensor_resets);
		px4_arch_gpiowrite(GPIO_ULTRASOUND_TRIGGER, 1);
		break;

	case STATE::SAMPLE:
		_state = STATE::MEASURE;
		measure();
		_state = STATE::TRIGGER;
		break;

	case STATE::EXIT:
	default:
		break;
	}
}

int
SRF05::measure()
{
	perf_begin(_sample_perf);

	const hrt_abstime timestamp_sample = hrt_absolute_time();
	const hrt_abstime dt = _falling_edge_time - _rising_edge_time;

	const float current_distance = dt *  343.0f / 10e6f / 2.0f;

	if (dt > HXSRX0X_CONVERSION_TIMEOUT) {
		perf_count(_comms_errors);

	} else {
		_px4_rangefinder.update(timestamp_sample, current_distance);
	}

	perf_end(_sample_perf);
	return PX4_OK;
}

int SRF05::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


int SRF05::task_spawn(int argc, char *argv[])
{

	int ch = 0;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;

	while ((ch = px4_getopt(argc, argv, "R:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (uint8_t)atoi(myoptarg);
			PX4_INFO("Setting sr05 orientation to %d", (int)rotation);
			break;

		default:
			return print_usage();
		}
	}

	SRF05 *instance = new SRF05(rotation);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
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

int SRF05::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
  ### Description

  Driver for HY-SRF05 / HC-SR05 and HC-SR04 rangefinders.

  The sensor/driver must be enabled using the parameter SENS_EN_HXSRX0X.

  )DESCR_STR");

	PRINT_MODULE_USAGE_NAME("srf05", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
	PRINT_MODULE_USAGE_PARAM_INT('R', 25, 0, 25, "Sensor rotation - downward facing by default", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print driver status information");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop driver");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return PX4_OK;
}

int
SRF05::print_status()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_sensor_resets);
	printf("poll interval:  %u \n", get_measure_interval());
	return 0;
}

extern "C" __EXPORT int srf05_main(int argc, char *argv[])
{
	return SRF05::main(argc, argv);
}
#else
# error ("GPIO_ULTRASOUND_xxx not defined. Driver not supported.");
#endif
