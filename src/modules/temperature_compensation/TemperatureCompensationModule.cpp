/****************************************************************************
 *
 *   Copyright (c) 2016-2020 PX4 Development Team. All rights reserved.
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

#include "TemperatureCompensationModule.h"

#include "temperature_calibration/temperature_calibration.h"

#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_command.h>

#include <systemlib/mavlink_log.h>

using namespace temperature_compensation;

TemperatureCompensationModule::TemperatureCompensationModule() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_loop_perf(perf_alloc(PC_ELAPSED, "temperature_compensation"))
{
}

TemperatureCompensationModule::~TemperatureCompensationModule()
{
	perf_free(_loop_perf);
}

void TemperatureCompensationModule::parameters_update()
{
	_temperature_compensation.parameters_update();

	// Gyro
	for (uint8_t uorb_index = 0; uorb_index < GYRO_COUNT_MAX; uorb_index++) {
		sensor_gyro_s report;

		if (_gyro_subs[uorb_index].copy(&report)) {
			int temp = _temperature_compensation.set_sensor_id_gyro(report.device_id, uorb_index);

			if (temp < 0) {
				PX4_INFO("No temperature calibration available for gyro %i (device id %u)", uorb_index, report.device_id);
				_corrections.gyro_device_ids[uorb_index] = 0;

			} else {
				_corrections.gyro_device_ids[uorb_index] = report.device_id;
			}
		}
	}

	// Accel
	for (uint8_t uorb_index = 0; uorb_index < ACCEL_COUNT_MAX; uorb_index++) {
		sensor_accel_s report;

		if (_accel_subs[uorb_index].copy(&report)) {
			int temp = _temperature_compensation.set_sensor_id_accel(report.device_id, uorb_index);

			if (temp < 0) {
				PX4_INFO("No temperature calibration available for accel %i (device id %u)", uorb_index, report.device_id);
				_corrections.accel_device_ids[uorb_index] = 0;

			} else {
				_corrections.accel_device_ids[uorb_index] = report.device_id;
			}
		}
	}

	// Baro
	for (uint8_t uorb_index = 0; uorb_index < BARO_COUNT_MAX; uorb_index++) {
		sensor_baro_s report;

		if (_baro_subs[uorb_index].copy(&report)) {
			int temp = _temperature_compensation.set_sensor_id_baro(report.device_id, uorb_index);

			if (temp < 0) {
				PX4_INFO("No temperature calibration available for baro %i (device id %u)", uorb_index, report.device_id);
				_corrections.baro_device_ids[uorb_index] = 0;

			} else {
				_corrections.baro_device_ids[uorb_index] = temp;
			}
		}
	}
}

void TemperatureCompensationModule::accelPoll()
{
	float *offsets[] = {_corrections.accel_offset_0, _corrections.accel_offset_1, _corrections.accel_offset_2, _corrections.accel_offset_3 };

	// For each accel instance
	for (uint8_t uorb_index = 0; uorb_index < ACCEL_COUNT_MAX; uorb_index++) {
		sensor_accel_s report;

		// Grab temperature from report
		if (_accel_subs[uorb_index].update(&report)) {
			if (PX4_ISFINITE(report.temperature)) {
				// Update the offsets and mark for publication if they've changed
				if (_temperature_compensation.update_offsets_accel(uorb_index, report.temperature, offsets[uorb_index]) == 2) {

					_corrections.accel_device_ids[uorb_index] = report.device_id;
					_corrections_changed = true;
				}
			}
		}
	}
}

void TemperatureCompensationModule::gyroPoll()
{
	float *offsets[] = {_corrections.gyro_offset_0, _corrections.gyro_offset_1, _corrections.gyro_offset_2, _corrections.gyro_offset_3 };

	// For each gyro instance
	for (uint8_t uorb_index = 0; uorb_index < GYRO_COUNT_MAX; uorb_index++) {
		sensor_gyro_s report;

		// Grab temperature from report
		if (_gyro_subs[uorb_index].update(&report)) {
			if (PX4_ISFINITE(report.temperature)) {
				// Update the offsets and mark for publication if they've changed
				if (_temperature_compensation.update_offsets_gyro(uorb_index, report.temperature, offsets[uorb_index]) == 2) {

					_corrections.gyro_device_ids[uorb_index] = report.device_id;
					_corrections_changed = true;
				}
			}
		}
	}
}

void TemperatureCompensationModule::baroPoll()
{
	float *offsets[] = {&_corrections.baro_offset_0, &_corrections.baro_offset_1, &_corrections.baro_offset_2, &_corrections.baro_offset_3 };

	// For each baro instance
	for (uint8_t uorb_index = 0; uorb_index < BARO_COUNT_MAX; uorb_index++) {
		sensor_baro_s report;

		// Grab temperature from report
		if (_baro_subs[uorb_index].update(&report)) {
			if (PX4_ISFINITE(report.temperature)) {
				// Update the offsets and mark for publication if they've changed
				if (_temperature_compensation.update_offsets_baro(uorb_index, report.temperature, offsets[uorb_index]) == 2) {

					_corrections.baro_device_ids[uorb_index] = report.device_id;
					_corrections_changed = true;
				}
			}
		}
	}
}

void TemperatureCompensationModule::Run()
{
	perf_begin(_loop_perf);

	// Check if user has requested to run the calibration routine
	while (_vehicle_command_sub.updated()) {
		vehicle_command_s cmd;

		if (_vehicle_command_sub.copy(&cmd)) {
			if (cmd.command == vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION) {
				bool got_temperature_calibration_command = false;
				bool accel = false;
				bool baro = false;
				bool gyro = false;

				if ((int)(cmd.param1) == vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION) {
					gyro = true;
					got_temperature_calibration_command = true;
				}

				if ((int)(cmd.param5) == vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION) {
					accel = true;
					got_temperature_calibration_command = true;
				}

				if ((int)(cmd.param7) == vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION) {
					baro = true;
					got_temperature_calibration_command = true;
				}

				if (got_temperature_calibration_command) {
					int ret = run_temperature_calibration(accel, baro, gyro);

					// publish ACK
					vehicle_command_ack_s command_ack{};

					if (ret == 0) {
						command_ack.result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

					} else {
						command_ack.result = vehicle_command_s::VEHICLE_CMD_RESULT_FAILED;
					}

					command_ack.timestamp = hrt_absolute_time();
					command_ack.command = cmd.command;
					command_ack.target_system = cmd.source_system;
					command_ack.target_component = cmd.source_component;

					uORB::Publication<vehicle_command_ack_s> command_ack_pub{ORB_ID(vehicle_command_ack)};
					command_ack_pub.publish(command_ack);
				}
			}
		}
	}

	// Check if any parameter has changed
	if (_parameter_update_sub.updated()) {
		// Read from param to clear updated flag
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		parameters_update();
	}

	accelPoll();
	gyroPoll();
	baroPoll();

	// publish sensor corrections if necessary
	if (_corrections_changed) {
		_corrections.timestamp = hrt_absolute_time();

		_sensor_correction_pub.publish(_corrections);

		_corrections_changed = false;
	}

	perf_end(_loop_perf);
}

int TemperatureCompensationModule::task_spawn(int argc, char *argv[])
{
	TemperatureCompensationModule *instance = new TemperatureCompensationModule();

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

bool TemperatureCompensationModule::init()
{
	ScheduleOnInterval(1_s);
	return true;
}

int TemperatureCompensationModule::custom_command(int argc, char *argv[])
{
	if (!strcmp(argv[0], "calibrate")) {

		bool accel_calib = false;
		bool baro_calib = false;
		bool gyro_calib = false;
		bool calib_all = true;
		int myoptind = 1;
		int ch;
		const char *myoptarg = nullptr;

		while ((ch = px4_getopt(argc, argv, "abg", &myoptind, &myoptarg)) != EOF) {
			switch (ch) {
			case 'a':
				accel_calib = true;
				calib_all = false;
				break;

			case 'b':
				baro_calib = true;
				calib_all = false;
				break;

			case 'g':
				gyro_calib = true;
				calib_all = false;
				break;

			default:
				print_usage("unrecognized flag");

				return PX4_ERROR;
			}
		}

		if (!is_running()) {
			PX4_WARN("background task not running");

			if (task_spawn(0, nullptr) != PX4_OK) {
				return PX4_ERROR;
			}
		}

		vehicle_command_s vcmd{};
		vcmd.timestamp = hrt_absolute_time();
		vcmd.param1 = (float)((gyro_calib
				       || calib_all) ? vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION : NAN);
		vcmd.param2 = NAN;
		vcmd.param3 = NAN;
		vcmd.param4 = NAN;
		vcmd.param5 = ((accel_calib
				|| calib_all) ? vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION : (double)NAN);
		vcmd.param6 = (double)NAN;
		vcmd.param7 = (float)((baro_calib
				       || calib_all) ? vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION : NAN);
		vcmd.command = vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION;

		uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
		vcmd_pub.publish(vcmd);

		return PX4_OK;

	} else {
		print_usage("unrecognized command");

		return PX4_ERROR;
	}
}

int TemperatureCompensationModule::print_status()
{
	_temperature_compensation.print_status();

	return PX4_OK;
}

int TemperatureCompensationModule::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
The temperature compensation module allows all of the gyro(s), accel(s), and baro(s) in the system to be temperature
compensated. The module monitors the data coming from the sensors and updates the associated sensor_correction topic
whenever a change in temperature is detected. The module can also be configured to perform the coeffecient calculation
routine at next boot, which allows the thermal calibration coeffecients to be calculated while the vehicle undergoes
a temperature cycle.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("temperature_compensation", "system");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the module, which monitors the sensors and updates the sensor_correction topic");
	PRINT_MODULE_USAGE_COMMAND_DESCR("calibrate", "Run temperature calibration process");
	PRINT_MODULE_USAGE_PARAM_FLAG('g', "calibrate the gyro", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('a', "calibrate the accel", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('b', "calibrate the baro (if none of these is given, all will be calibrated)", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int temperature_compensation_main(int argc, char *argv[])
{
	return TemperatureCompensationModule::main(argc, argv);
}
