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

#include "worker_thread.hpp"
#include "accelerometer_calibration.h"
#include "airspeed_calibration.h"
#include "calibration_routines.h"
#include "esc_calibration.h"
#include "gyro_calibration.h"
#include "level_calibration.h"
#include "mag_calibration.h"
#include "rc_calibration.h"

#include <px4_platform_common/log.h>
#include <parameters/param.h>

WorkerThread::~WorkerThread()
{
	if (_state.load() == (int)State::Running) {
		/* wait for thread to complete */
		int ret = pthread_join(_thread_handle, nullptr);

		if (ret) {
			PX4_ERR("join failed: %d", ret);
		}
	}
}

void WorkerThread::startTask(Request request)
{
	if (isBusy()) {
		return;
	}

	_request = request;

	/* initialize low priority thread */
	pthread_attr_t low_prio_attr;
	pthread_attr_init(&low_prio_attr);
	pthread_attr_setstacksize(&low_prio_attr, PX4_STACK_ADJUSTED(3804));

#ifndef __PX4_QURT
	// This is not supported by QURT (yet).
	struct sched_param param;
	pthread_attr_getschedparam(&low_prio_attr, &param);

	/* low priority */
	param.sched_priority = SCHED_PRIORITY_DEFAULT - 50;
	pthread_attr_setschedparam(&low_prio_attr, &param);
#endif
	int ret = pthread_create(&_thread_handle, &low_prio_attr, &threadEntryTrampoline, this);
	pthread_attr_destroy(&low_prio_attr);

	if (ret == 0) {
		_state.store((int)State::Running);

	} else {
		PX4_ERR("Failed to start thread (%i)", ret);
		_state.store((int)State::Finished);
		_ret_value = ret;
	}
}

void *WorkerThread::threadEntryTrampoline(void *arg)
{
	WorkerThread *worker_thread = (WorkerThread *)arg;
	worker_thread->threadEntry();
	return nullptr;
}

void WorkerThread::threadEntry()
{
	px4_prctl(PR_SET_NAME, "commander_low_prio", px4_getpid());

	switch (_request) {
	case Request::GyroCalibration:
		_ret_value = do_gyro_calibration(&_mavlink_log_pub);
		break;

	case Request::MagCalibration:
		_ret_value = do_mag_calibration(&_mavlink_log_pub);
		break;

	case Request::RCTrimCalibration:
		_ret_value = do_trim_calibration(&_mavlink_log_pub);
		break;

	case Request::AccelCalibration:
		_ret_value = do_accel_calibration(&_mavlink_log_pub);
		break;

	case Request::LevelCalibration:
		_ret_value = do_level_calibration(&_mavlink_log_pub);
		break;

	case Request::AccelCalibrationQuick:
		_ret_value = do_accel_calibration_quick(&_mavlink_log_pub);
		break;

	case Request::AirspeedCalibration:
		_ret_value = do_airspeed_calibration(&_mavlink_log_pub);
		break;

	case Request::ESCCalibration:
		_ret_value = do_esc_calibration(&_mavlink_log_pub);
		break;

	case Request::MagCalibrationQuick:
		_ret_value = do_mag_calibration_quick(&_mavlink_log_pub, _heading_radians, _latitude, _longitude);
		break;

	case Request::ParamLoadDefault:
		_ret_value = param_load_default();

		if (_ret_value != 0) {
			mavlink_log_critical(&_mavlink_log_pub, "Error loading settings");
		}

		break;

	case Request::ParamSaveDefault:
		_ret_value = param_save_default();

		if (_ret_value != 0) {
			mavlink_log_critical(&_mavlink_log_pub, "Error saving settings");
		}

		break;

	case Request::ParamResetAll:
		param_reset_all();
		_ret_value = 0;
		break;
	}

	_state.store((int)State::Finished); // set this last to signal the main thread we're done
}

void WorkerThread::setMagQuickData(float heading_rad, float lat, float lon)
{
	_heading_radians = heading_rad;
	_latitude = lat;
	_longitude = lon;
}
