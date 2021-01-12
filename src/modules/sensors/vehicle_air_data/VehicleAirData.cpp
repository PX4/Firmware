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

#include "VehicleAirData.hpp"

#include <px4_platform_common/log.h>
#include <lib/ecl/geo/geo.h>

namespace sensors
{

using namespace matrix;

static constexpr uint32_t SENSOR_TIMEOUT{300_ms};

VehicleAirData::VehicleAirData() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_voter.set_timeout(SENSOR_TIMEOUT);
}

VehicleAirData::~VehicleAirData()
{
	Stop();
	perf_free(_cycle_perf);
}

bool VehicleAirData::Start()
{
	ScheduleNow();
	return true;
}

void VehicleAirData::Stop()
{
	Deinit();

	// clear all registered callbacks
	for (auto &sub : _sensor_sub) {
		sub.unregisterCallback();
	}
}

void VehicleAirData::SensorCorrectionsUpdate(bool force)
{
	if (_sensor_correction_sub.updated() || force) {
		sensor_correction_s corrections;

		if (_sensor_correction_sub.copy(&corrections)) {
			for (int baro_index = 0; baro_index < MAX_SENSOR_COUNT; baro_index++) {
				// find sensor (by device id) in sensor_correction
				const uint32_t device_id = _last_data[baro_index].device_id;

				if (device_id != 0) {
					for (int correction_index = 0; correction_index < MAX_SENSOR_COUNT; correction_index++) {
						if (corrections.baro_device_ids[correction_index] == device_id) {
							switch (correction_index) {
							case 0:
								_thermal_offset[baro_index] = corrections.baro_offset_0;
								break;

							case 1:
								_thermal_offset[baro_index] = corrections.baro_offset_1;
								break;

							case 2:
								_thermal_offset[baro_index] = corrections.baro_offset_2;
								break;

							case 3:
								_thermal_offset[baro_index] = corrections.baro_offset_3;
								break;
							}
						}
					}
				}
			}
		}
	}
}

void VehicleAirData::ParametersUpdate()
{
	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
	}
}

void VehicleAirData::Run()
{
	perf_begin(_cycle_perf);

	ParametersUpdate();

	SensorCorrectionsUpdate();

	bool updated[MAX_SENSOR_COUNT] {};

	for (int uorb_index = 0; uorb_index < MAX_SENSOR_COUNT; uorb_index++) {

		if (!_advertised[uorb_index]) {
			// use data's timestamp to throttle advertisement checks
			if ((_last_data[uorb_index].timestamp == 0) || (hrt_elapsed_time(&_last_data[uorb_index].timestamp) > 1_s)) {
				if (_sensor_sub[uorb_index].advertised()) {
					if (uorb_index > 0) {
						/* the first always exists, but for each further sensor, add a new validator */
						if (!_voter.add_new_validator()) {
							PX4_ERR("failed to add validator for %s %i", "BARO", uorb_index);
						}
					}

					_advertised[uorb_index] = true;

					// force temperature correction update
					SensorCorrectionsUpdate(true);

					if (_selected_sensor_sub_index < 0) {
						_sensor_sub[uorb_index].registerCallback();
					}

				} else {
					_last_data[uorb_index].timestamp = hrt_absolute_time();
				}
			}
		}

		if (_advertised[uorb_index]) {
			updated[uorb_index] = _sensor_sub[uorb_index].update(&_last_data[uorb_index]);

			if (updated[uorb_index]) {
				// millibar to Pa
				const float raw_pressure_pascals = _last_data[uorb_index].pressure * 100.f;

				// pressure corrected with offset (if available)
				const float pressure_corrected = (raw_pressure_pascals - _thermal_offset[uorb_index]);

				float vect[3] {pressure_corrected, _last_data[uorb_index].temperature, 0.f};
				_voter.put(uorb_index, _last_data[uorb_index].timestamp, vect, _last_data[uorb_index].error_count,
					   _priority[uorb_index]);
			}
		}
	}

	// check for the current best sensor
	int best_index = 0;
	_voter.get_best(hrt_absolute_time(), &best_index);

	if (best_index >= 0) {
		if (_selected_sensor_sub_index != best_index) {
			// clear all registered callbacks
			for (auto &sub : _sensor_sub) {
				sub.unregisterCallback();
			}

			if (_selected_sensor_sub_index >= 0) {
				PX4_INFO("%s switch from #%u -> #%d", "BARO", _selected_sensor_sub_index, best_index);
			}

			_selected_sensor_sub_index = best_index;
			_sensor_sub[_selected_sensor_sub_index].registerCallback();
		}
	}

	if ((_selected_sensor_sub_index >= 0)
	    && (_voter.get_sensor_state(_selected_sensor_sub_index) == DataValidator::ERROR_FLAG_NO_ERROR)
	    && updated[_selected_sensor_sub_index]) {

		const sensor_baro_s &baro = _last_data[_selected_sensor_sub_index];

		_baro_timestamp_sum += baro.timestamp_sample;
		_baro_sum += baro.pressure;
		_baro_sum_count++;

		if ((_param_sens_baro_rate.get() > 0)
		    && ((_last_publication_timestamp == 0)
			|| (hrt_elapsed_time(&_last_publication_timestamp) >= (1e6f / _param_sens_baro_rate.get())))) {

			const float pressure = _baro_sum / _baro_sum_count;
			const hrt_abstime timestamp_sample = _baro_timestamp_sum / _baro_sum_count;

			// reset
			_baro_timestamp_sum = 0;
			_baro_sum = 0.f;
			_baro_sum_count = 0;

			// populate vehicle_air_data with primary baro and publish
			vehicle_air_data_s out{};
			out.timestamp_sample = timestamp_sample;
			out.baro_device_id = baro.device_id;
			out.baro_temp_celcius = baro.temperature;

			// Convert from millibar to Pa and apply temperature compensation
			out.baro_pressure_pa = 100.0f * pressure - _thermal_offset[_selected_sensor_sub_index];

			// calculate altitude using the hypsometric equation
			static constexpr float T1 = 15.0f - CONSTANTS_ABSOLUTE_NULL_CELSIUS; // temperature at base height in Kelvin
			static constexpr float a = -6.5f / 1000.0f; // temperature gradient in degrees per metre

			// current pressure at MSL in kPa (QNH in hPa)
			const float p1 = _param_sens_baro_qnh.get() * 0.1f;

			// measured pressure in kPa
			const float p = out.baro_pressure_pa * 0.001f;

			/*
			 * Solve:
			 *
			 *     /        -(aR / g)     \
			 *    | (p / p1)          . T1 | - T1
			 *     \                      /
			 * h = -------------------------------  + h1
			 *                   a
			 */
			out.baro_alt_meter = (((powf((p / p1), (-(a * CONSTANTS_AIR_GAS_CONST) / CONSTANTS_ONE_G))) * T1) - T1) / a;

			// calculate air density
			// estimate air density assuming typical 20degC ambient temperature
			// TODO: use air temperature if available (differential pressure sensors)
			static constexpr float pressure_to_density = 1.0f / (CONSTANTS_AIR_GAS_CONST * (20.0f -
					CONSTANTS_ABSOLUTE_NULL_CELSIUS));

			out.rho = pressure_to_density * out.baro_pressure_pa;

			out.timestamp = hrt_absolute_time();
			_vehicle_air_data_pub.publish(out);

			_last_publication_timestamp = out.timestamp;
		}
	}

	// check failover and report
	if (_last_failover_count != _voter.failover_count()) {
		uint32_t flags = _voter.failover_state();
		int failover_index = _voter.failover_index();

		if (flags != DataValidator::ERROR_FLAG_NO_ERROR) {
			if (failover_index != -1) {
				const hrt_abstime now = hrt_absolute_time();

				if (now - _last_error_message > 3_s) {
					mavlink_log_emergency(&_mavlink_log_pub, "%s #%i failed: %s%s%s%s%s!",
							      "BARO",
							      failover_index,
							      ((flags & DataValidator::ERROR_FLAG_NO_DATA) ? " OFF" : ""),
							      ((flags & DataValidator::ERROR_FLAG_STALE_DATA) ? " STALE" : ""),
							      ((flags & DataValidator::ERROR_FLAG_TIMEOUT) ? " TIMEOUT" : ""),
							      ((flags & DataValidator::ERROR_FLAG_HIGH_ERRCOUNT) ? " ERR CNT" : ""),
							      ((flags & DataValidator::ERROR_FLAG_HIGH_ERRDENSITY) ? " ERR DNST" : ""));
					_last_error_message = now;
				}

				// reduce priority of failed sensor to the minimum
				_priority[failover_index] = 1;
			}
		}

		_last_failover_count = _voter.failover_count();
	}

	// reschedule timeout
	ScheduleDelayed(100_ms);

	perf_end(_cycle_perf);
}

void VehicleAirData::PrintStatus()
{
	if (_selected_sensor_sub_index >= 0) {
		PX4_INFO("selected barometer: %d (%d)", _last_data[_selected_sensor_sub_index].device_id, _selected_sensor_sub_index);

		if (fabsf(_thermal_offset[_selected_sensor_sub_index]) > 0.f) {
			PX4_INFO("%d temperature offset: %.4f", _last_data[_selected_sensor_sub_index].device_id,
				 (double)_thermal_offset[_selected_sensor_sub_index]);
		}
	}

	_voter.print();
}

}; // namespace sensors
