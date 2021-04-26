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

#include "GyroCalibration.hpp"

#include <lib/ecl/geo/geo.h>

using namespace time_literals;
using matrix::Vector3f;

GyroCalibration::GyroCalibration() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

GyroCalibration::~GyroCalibration()
{
	perf_free(_loop_interval_perf);
	perf_free(_calibration_updated_perf);
}

bool GyroCalibration::init()
{
	ScheduleOnInterval(INTERVAL_US);
	return true;
}

void GyroCalibration::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_count(_loop_interval_perf);

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {
			const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

			if (armed != _armed) {
				if (!_armed && armed) {
					// disarmed -> armed: run at minimal rate until disarmed
					ScheduleOnInterval(10_s);

				} else if (_armed && !armed) {
					// armed -> disarmed: start running again
					ScheduleOnInterval(INTERVAL_US);
				}

				_armed = armed;
				Reset();
			}
		}
	}

	if (_armed) {
		// do nothing if armed
		return;
	}

	if (_vehicle_status_flags_sub.updated()) {
		vehicle_status_flags_s vehicle_status_flags;

		if (_vehicle_status_flags_sub.copy(&vehicle_status_flags)) {
			if (_system_calibrating != vehicle_status_flags.condition_calibration_enabled) {
				Reset();
				_system_calibrating = vehicle_status_flags.condition_calibration_enabled;
			}
		}
	}

	if (_system_calibrating) {
		// do nothing if system is calibrating
		return;
	}


	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;

		if (_parameter_update_sub.copy(&param_update)) {
			// minimize updates immediately following parameter changes
			_last_calibration_update = param_update.timestamp;
		}

		for (auto &cal : _gyro_calibration) {
			cal.ParametersUpdate();
		}
	}


	// collect raw data from all available gyroscopes (sensor_gyro)
	for (int gyro = 0; gyro < _sensor_gyro_subs.size(); gyro++) {
		sensor_gyro_s sensor_gyro;

		while (_sensor_gyro_subs[gyro].update(&sensor_gyro)) {
			if (PX4_ISFINITE(sensor_gyro.temperature)) {
				if ((fabsf(_temperature[gyro] - sensor_gyro.temperature) > 1.f) || !PX4_ISFINITE(_temperature[gyro])) {
					PX4_DEBUG("gyro %d temperature change, resetting all %.6f -> %.6f", gyro, (double)_temperature[gyro],
						  (double)sensor_gyro.temperature);

					_temperature[gyro] = sensor_gyro.temperature;

					// reset all on any temperature change
					Reset();
				}

			} else {
				_temperature[gyro] = NAN;
			}

			if (_gyro_calibration[gyro].device_id() == sensor_gyro.device_id) {
				const Vector3f val{Vector3f{sensor_gyro.x, sensor_gyro.y, sensor_gyro.z} - _gyro_calibration[gyro].thermal_offset()};
				_gyro_mean[gyro].update(val);

			} else {
				// setting device id, reset all
				_gyro_calibration[gyro].set_device_id(sensor_gyro.device_id);
				Reset();
			}
		}
	}


	// check all accelerometers for possible movement
	for (int accel = 0; accel < _sensor_accel_subs.size(); accel++) {
		sensor_accel_s sensor_accel;

		if (_sensor_accel_subs[accel].update(&sensor_accel)) {
			const Vector3f acceleration{sensor_accel.x, sensor_accel.y, sensor_accel.z};

			if ((acceleration - _acceleration[accel]).longerThan(0.5f)) {
				// reset all on any change
				PX4_DEBUG("accel %d changed, resetting all %.5f", accel, (double)(acceleration - _acceleration[accel]).length());

				_acceleration[accel] = acceleration;
				Reset();
				return;

			} else if (acceleration.longerThan(CONSTANTS_ONE_G * 1.3f)) {
				Reset();
				return;
			}
		}
	}


	// check if sufficient data has been gathered to update calibration
	bool sufficient_samples = false;

	for (int gyro = 0; gyro < _sensor_gyro_subs.size(); gyro++) {
		if (_gyro_calibration[gyro].device_id() != 0) {
			// periodically check variance
			if ((_gyro_mean[gyro].count() % 100 == 0)) {
				PX4_DEBUG("gyro %d (%d) variance, [%.9f, %.9f, %.9f] %.9f", gyro, _gyro_calibration[gyro].device_id(),
					  (double)_gyro_mean[gyro].variance()(0), (double)_gyro_mean[gyro].variance()(1), (double)_gyro_mean[gyro].variance()(2),
					  (double)_gyro_mean[gyro].variance().length());

				if (_gyro_mean[gyro].variance().longerThan(0.001f)) {
					// reset all
					Reset();
					return;
				}
			}

			if (_gyro_mean[gyro].count() > 3000) {
				sufficient_samples = true;

			} else {
				sufficient_samples = false;
				return;
			}
		}
	}


	// update calibrations for all available gyros
	if (sufficient_samples && (hrt_elapsed_time(&_last_calibration_update) > 10_s)) {
		bool calibration_updated = false;

		for (int gyro = 0; gyro < _sensor_gyro_subs.size(); gyro++) {
			if (_gyro_calibration[gyro].device_id() != 0) {
				_gyro_calibration[gyro].set_calibration_index(gyro);

				const Vector3f old_offset{_gyro_calibration[gyro].offset()};

				if (_gyro_calibration[gyro].set_offset(_gyro_mean[gyro].mean())) {
					calibration_updated = true;

					PX4_INFO("gyro %d (%d) updating calibration, [%.4f, %.4f, %.4f] -> [%.4f, %.4f, %.4f] %.1f°C",
						 gyro, _gyro_calibration[gyro].device_id(),
						 (double)old_offset(0), (double)old_offset(1), (double)old_offset(2),
						 (double)_gyro_mean[gyro].mean()(0), (double)_gyro_mean[gyro].mean()(1), (double)_gyro_mean[gyro].mean()(2),
						 (double)_temperature[gyro]);

					perf_count(_calibration_updated_perf);
				}
			}
		}

		// save all calibrations
		if (calibration_updated) {
			bool param_save = false;

			for (int gyro = 0; gyro < _sensor_gyro_subs.size(); gyro++) {
				if (_gyro_calibration[gyro].device_id() != 0) {
					if (_gyro_calibration[gyro].ParametersSave()) {
						param_save = true;
					}
				}
			}

			if (param_save) {
				param_notify_changes();
				_last_calibration_update = hrt_absolute_time();
			}
		}

		Reset();
	}
}

int GyroCalibration::task_spawn(int argc, char *argv[])
{
	GyroCalibration *instance = new GyroCalibration();

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

int GyroCalibration::print_status()
{
	for (int gyro = 0; gyro < _sensor_gyro_subs.size(); gyro++) {
		if (_gyro_calibration[gyro].device_id() != 0) {
			PX4_INFO_RAW("gyro %d (%d), [%.5f, %.5f, %.5f] var: [%.9f, %.9f, %.9f] %.1f°C (count %d)\n",
				     gyro, _gyro_calibration[gyro].device_id(),
				     (double)_gyro_mean[gyro].mean()(0), (double)_gyro_mean[gyro].mean()(1), (double)_gyro_mean[gyro].mean()(2),
				     (double)_gyro_mean[gyro].variance()(0), (double)_gyro_mean[gyro].variance()(1), (double)_gyro_mean[gyro].variance()(2),
				     (double)_temperature[gyro], _gyro_mean[gyro].count());
		}
	}

	perf_print_counter(_loop_interval_perf);
	perf_print_counter(_calibration_updated_perf);
	return 0;
}

int GyroCalibration::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int GyroCalibration::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Simple online gyroscope calibration.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("gyro_calibration", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int gyro_calibration_main(int argc, char *argv[])
{
	return GyroCalibration::main(argc, argv);
}
