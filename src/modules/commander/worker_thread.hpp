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

#pragma once


#include <px4_platform_common/atomic.h>
#include <px4_platform_common/posix.h>
#include <systemlib/mavlink_log.h>
#include <uORB/uORB.h>

/**
 * @class WorkerThread
 * low priority background thread, started on demand, used for:
 * - calibration
 * - param saving
 */
class WorkerThread
{
public:
	enum class Request {
		GyroCalibration,
		MagCalibration,
		RCTrimCalibration,
		AccelCalibration,
		LevelCalibration,
		AccelCalibrationQuick,
		AirspeedCalibration,
		ESCCalibration,
		MagCalibrationQuick,

		ParamLoadDefault,
		ParamSaveDefault,
		ParamResetAll,
	};

	WorkerThread() = default;
	~WorkerThread();

	void setMagQuickData(float heading_rad, float lat, float lon);

	void startTask(Request request);

	bool isBusy() const { return _state.load() != (int)State::Idle; }
	bool hasResult() const { return _state.load() == (int)State::Finished; }
	int getResultAndReset() { _state.store((int)State::Idle); return _ret_value; }

private:
	enum class State {
		Idle,
		Running,
		Finished
	};

	static void *threadEntryTrampoline(void *arg);
	void threadEntry();

	px4::atomic_int _state{(int)State::Idle};
	pthread_t _thread_handle{};
	int _ret_value{};
	Request _request;
	orb_advert_t _mavlink_log_pub{nullptr};

	// extra arguments
	float _heading_radians;
	float _latitude;
	float _longitude;

};

