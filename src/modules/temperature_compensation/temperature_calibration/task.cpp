/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file task.cpp
 *
 * Main task handling the temperature calibration process
 *
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#include <uORB/Publication.hpp>
#include <uORB/topics/sensor_gyro.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_led.h>

#include <unistd.h>

#include "common.h"
#include "temperature_calibration.h"
#include "accel.h"
#include "baro.h"
#include "gyro.h"

class TemperatureCalibration;

namespace temperature_calibration
{
px4::atomic<TemperatureCalibration *> instance{nullptr};
}

class TemperatureCalibration
{
public:

	TemperatureCalibration(bool accel, bool baro, bool gyro) : _accel(accel), _baro(baro), _gyro(gyro) {}
	~TemperatureCalibration() = default;

	/**
	 * Start task.
	 *
	 * @return		OK on success.
	 */
	int		start();

	static int do_temperature_calibration(int argc, char *argv[]);

	void		task_main();

	void exit_task() { _force_task_exit = true; }

private:
	void publish_led_control(led_control_s &led_control);

	uORB::Publication<led_control_s> _led_control_pub{ORB_ID(led_control)};

	bool	_force_task_exit = false;
	int	_control_task = -1;		// task handle for task

	const bool _accel; ///< enable accel calibration?
	const bool _baro; ///< enable baro calibration?
	const bool _gyro; ///< enable gyro calibration?
};

void TemperatureCalibration::task_main()
{
	// subscribe to all gyro instances
	int gyro_sub[SENSOR_COUNT_MAX] {-1, -1, -1};
	px4_pollfd_struct_t fds[SENSOR_COUNT_MAX] {};
	unsigned num_gyro = orb_group_count(ORB_ID(sensor_gyro));

	if (num_gyro > SENSOR_COUNT_MAX) {
		num_gyro = SENSOR_COUNT_MAX;
	}

	for (unsigned i = 0; i < num_gyro; i++) {
		gyro_sub[i] = orb_subscribe_multi(ORB_ID(sensor_gyro), i);
		fds[i].fd = gyro_sub[i];
		fds[i].events = POLLIN;
	}

	int32_t min_temp_rise = 24;
	param_get(param_find("SYS_CAL_TDEL"), &min_temp_rise);
	PX4_INFO("Waiting for %i degrees difference in sensor temperature", min_temp_rise);

	int32_t min_start_temp = 5;
	param_get(param_find("SYS_CAL_TMIN"), &min_start_temp);

	int32_t max_start_temp = 10;
	param_get(param_find("SYS_CAL_TMAX"), &max_start_temp);

	//init calibrators
	TemperatureCalibrationBase *calibrators[3] {};
	bool error_reported[3] {};
	int num_calibrators = 0;

	if (_accel) {
		calibrators[num_calibrators] = new TemperatureCalibrationAccel(min_temp_rise, min_start_temp, max_start_temp);

		if (calibrators[num_calibrators]) {
			++num_calibrators;

		} else {
			PX4_ERR("alloc failed");
		}
	}

	if (_baro) {
		calibrators[num_calibrators] = new TemperatureCalibrationBaro(min_temp_rise, min_start_temp, max_start_temp);

		if (calibrators[num_calibrators]) {
			++num_calibrators;

		} else {
			PX4_ERR("alloc failed");
		}
	}

	if (_gyro) {
		calibrators[num_calibrators] = new TemperatureCalibrationGyro(min_temp_rise, min_start_temp, max_start_temp, gyro_sub,
				num_gyro);

		if (calibrators[num_calibrators]) {
			++num_calibrators;

		} else {
			PX4_ERR("alloc failed");
		}
	}

	hrt_abstime next_progress_output = hrt_absolute_time() + 1e6;

	// control LED's: blink, then turn solid according to progress
	led_control_s led_control{};
	led_control.led_mask = 0xff;
	led_control.mode = led_control_s::MODE_BLINK_NORMAL;
	led_control.priority = led_control_s::MAX_PRIORITY;
	led_control.color = led_control_s::COLOR_YELLOW;
	led_control.num_blinks = 0;
	publish_led_control(led_control);
	int leds_completed = 0;

	bool abort_calibration = false;

	while (!_force_task_exit) {
		/* we poll on the gyro(s), since this is the sensor with the highest update rate.
		 * Each individual sensor will then check on its own if there's new data.
		 */
		int ret = px4_poll(fds, num_gyro, 1000);

		if (ret < 0) {
			// Poll error, sleep and try again
			px4_usleep(10000);
			continue;

		} else if (ret == 0) {
			// Poll timeout or no new data, do nothing
			continue;
		}

		//if gyro is not enabled: we must do an orb_copy here, so that poll() does not immediately return again
		if (!_gyro) {
			sensor_gyro_s gyro_data;

			for (unsigned i = 0; i < num_gyro; ++i) {
				orb_copy(ORB_ID(sensor_gyro), gyro_sub[i], &gyro_data);
			}
		}

		int min_progress = 110;

		for (int i = 0; i < num_calibrators; ++i) {
			ret = calibrators[i]->update();

			if (ret == -TC_ERROR_COMMUNICATION) {
				abort_calibration = true;
				PX4_ERR("Calibration won't start - sensor bad or communication error");
				_force_task_exit = true;
				break;

			} else if (ret == -TC_ERROR_INITIAL_TEMP_TOO_HIGH) {
				abort_calibration = true;
				PX4_ERR("Calibration won't start - sensor temperature too high");
				_force_task_exit = true;
				break;

			} else if (ret < 0 && !error_reported[i]) {
				// temperature has decreased so calibration is not being updated
				error_reported[i] = true;
				PX4_ERR("Calibration update step failed (%i)", ret);

			} else if (ret < min_progress) {
				// temperature is stable or increasing
				min_progress = ret;
			}
		}

		if (min_progress == 110 || abort_calibration) {
			break; // we are done
		}

		int led_progress = min_progress * BOARD_MAX_LEDS / 100;

		for (; leds_completed < led_progress; ++leds_completed) {
			led_control.led_mask = 1 << leds_completed;
			led_control.mode = led_control_s::MODE_ON;
			publish_led_control(led_control);
		}

		//print progress each second
		hrt_abstime now = hrt_absolute_time();

		if (now > next_progress_output) {
			PX4_INFO("Calibration progress: %i%%", min_progress);
			next_progress_output = now + 1e6;
		}
	}

	if (abort_calibration) {
		led_control.color = led_control_s::COLOR_RED;

	} else {
		PX4_INFO("Sensor Measurments completed");

		// save params immediately so that we can check the result and don't have to wait for param save timeout
		param_control_autosave(false);

		// do final calculations & parameter storage
		for (int i = 0; i < num_calibrators; ++i) {
			int ret = calibrators[i]->finish();

			if (ret < 0) {
				PX4_ERR("Failed to finish calibration process (%i)", ret);
			}
		}

		param_notify_changes();
		int ret = param_save_default();

		if (ret != 0) {
			PX4_ERR("Failed to save params (%i)", ret);
		}

		param_control_autosave(true);

		led_control.color = led_control_s::COLOR_GREEN;
	}

	// blink the LED's according to success/failure
	led_control.led_mask = 0xff;
	led_control.mode = led_control_s::MODE_BLINK_FAST;
	led_control.num_blinks = 0;
	publish_led_control(led_control);

	for (int i = 0; i < num_calibrators; ++i) {
		delete calibrators[i];
	}

	for (unsigned i = 0; i < num_gyro; i++) {
		orb_unsubscribe(gyro_sub[i]);
	}

	delete temperature_calibration::instance.load();
	temperature_calibration::instance.store(nullptr);
	PX4_INFO("Exiting temperature calibration task");
}

int TemperatureCalibration::do_temperature_calibration(int argc, char *argv[])
{
	temperature_calibration::instance.load()->task_main();
	return 0;
}

int TemperatureCalibration::start()
{
	_control_task = px4_task_spawn_cmd("temperature_calib",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   5800,
					   (px4_main_t)&TemperatureCalibration::do_temperature_calibration,
					   nullptr);

	if (_control_task < 0) {
		delete temperature_calibration::instance.load();
		temperature_calibration::instance.store(nullptr);
		PX4_ERR("start failed");
		return -errno;
	}

	return 0;
}

void TemperatureCalibration::publish_led_control(led_control_s &led_control)
{
	led_control.timestamp = hrt_absolute_time();
	_led_control_pub.publish(led_control);
}

int run_temperature_calibration(bool accel, bool baro, bool gyro)
{
	if (temperature_calibration::instance.load() == nullptr) {
		PX4_INFO("Starting temperature calibration task (accel=%i, baro=%i, gyro=%i)", (int)accel, (int)baro, (int)gyro);
		temperature_calibration::instance.store(new TemperatureCalibration(accel, baro, gyro));

		if (temperature_calibration::instance.load() == nullptr) {
			PX4_ERR("alloc failed");
			return 1;
		}

		return temperature_calibration::instance.load()->start();

	} else {
		PX4_WARN("temperature calibration task already running");
	}

	return PX4_ERROR;
}
