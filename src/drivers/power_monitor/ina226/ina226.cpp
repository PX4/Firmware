/****************************************************************************
 *
 *   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
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
 * @file ina226.cpp
 * @author David Sidrane <david_s5@usa.net>
 *
 * Driver for the I2C attached INA226
 */

#include "ina226.h"


INA226::INA226(I2CSPIBusOption bus_option, const int bus, int bus_frequency, int address, int battery_index) :
	I2C(DRV_POWER_DEVTYPE_INA226, MODULE_NAME, bus, address, bus_frequency),
	ModuleParams(nullptr),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus, address),
	_sample_perf(perf_alloc(PC_ELAPSED, "ina226_read")),
	_comms_errors(perf_alloc(PC_COUNT, "ina226_com_err")),
	_collection_errors(perf_alloc(PC_COUNT, "ina226_collection_err")),
	_measure_errors(perf_alloc(PC_COUNT, "ina226_measurement_err")),
	_battery(battery_index, this, INA226_SAMPLE_INTERVAL_US)
{
	float fvalue = MAX_CURRENT;
	_max_current = fvalue;
	param_t ph = param_find("INA226_CURRENT");

	if (ph != PARAM_INVALID && param_get(ph, &fvalue) == PX4_OK) {
		_max_current = fvalue;
	}

	fvalue = INA226_SHUNT;
	_rshunt = fvalue;
	ph = param_find("INA226_SHUNT");

	if (ph != PARAM_INVALID && param_get(ph, &fvalue) == PX4_OK) {
		_rshunt = fvalue;
	}

	ph = param_find("INA226_CONFIG");
	int32_t value = INA226_CONFIG;
	_config = (uint16_t)value;

	if (ph != PARAM_INVALID && param_get(ph, &value) == PX4_OK) {
		_config = (uint16_t)value;
	}

	_mode_triggered = ((_config & INA226_MODE_MASK) >> INA226_MODE_SHIFTS) <=
			  ((INA226_MODE_SHUNT_BUS_TRIG & INA226_MODE_MASK) >>
			   INA226_MODE_SHIFTS);

	_current_lsb = _max_current / DN_MAX;
	_power_lsb = 25 * _current_lsb;

	// We need to publish immediately, to guarantee that the first instance of the driver publishes to uORB instance 0
	_battery.updateBatteryStatus(
		hrt_absolute_time(),
		0.0,
		0.0,
		false,
		battery_status_s::BATTERY_SOURCE_POWER_MODULE,
		0,
		0.0
	);
}

INA226::~INA226()
{
	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_collection_errors);
	perf_free(_measure_errors);
}

int INA226::read(uint8_t address, int16_t &data)
{
	// read desired little-endian value via I2C
	uint16_t received_bytes;
	const int ret = transfer(&address, 1, (uint8_t *)&received_bytes, sizeof(received_bytes));

	if (ret == PX4_OK) {
		data = swap16(received_bytes);

	} else {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret);
	}

	return ret;
}

int INA226::write(uint8_t address, uint16_t value)
{
	uint8_t data[3] = {address, ((uint8_t)((value & 0xff00) >> 8)), (uint8_t)(value & 0xff)};
	return transfer(data, sizeof(data), nullptr, 0);
}

int
INA226::init()
{
	int ret = PX4_ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != PX4_OK) {
		return ret;
	}

	write(INA226_REG_CONFIGURATION, INA226_RST);

	_cal = INA226_CONST / (_current_lsb * INA226_SHUNT);

	if (write(INA226_REG_CALIBRATION, _cal) < 0) {
		return -3;
	}

	// If we run in continuous mode then start it here

	if (!_mode_triggered) {
		ret = write(INA226_REG_CONFIGURATION, _config);

	} else {
		ret = PX4_OK;
	}

	start();
	_sensor_ok = true;

	_initialized = ret == PX4_OK;
	return ret;
}

int
INA226::force_init()
{
	int ret = init();

	start();

	return ret;
}

int
INA226::probe()
{
	int16_t value{0};

	if (read(INA226_MFG_ID, value) != PX4_OK || value != INA226_MFG_ID_TI) {
		PX4_DEBUG("probe mfgid %d", value);
		return -1;
	}

	if (read(INA226_MFG_DIEID, value) != PX4_OK || value != INA226_MFG_DIE) {
		PX4_DEBUG("probe die id %d", value);
		return -1;
	}

	return PX4_OK;
}

int
INA226::measure()
{
	int ret = PX4_OK;

	if (_mode_triggered) {
		ret = write(INA226_REG_CONFIGURATION, _config);

		if (ret < 0) {
			perf_count(_comms_errors);
			PX4_DEBUG("i2c::transfer returned %d", ret);
		}
	}

	return ret;
}

int
INA226::collect()
{
	perf_begin(_sample_perf);

	if (_parameter_update_sub.updated()) {
		// Read from topic to clear updated flag
		parameter_update_s parameter_update;
		_parameter_update_sub.copy(&parameter_update);

		updateParams();
	}

	// read from the sensor
	// Note: If the power module is connected backwards, then the values of _power, _current, and _shunt will be negative but otherwise valid.
	bool success{true};
	success = success && (read(INA226_REG_BUSVOLTAGE, _bus_voltage) == PX4_OK);
	// success = success && (read(INA226_REG_POWER, _power) == PX4_OK);
	success = success && (read(INA226_REG_CURRENT, _current) == PX4_OK);
	// success = success && (read(INA226_REG_SHUNTVOLTAGE, _shunt) == PX4_OK);

	if (!success) {
		PX4_DEBUG("error reading from sensor");
		_bus_voltage = _power = _current = _shunt = 0;
	}

	_actuators_sub.copy(&_actuator_controls);

	_battery.updateBatteryStatus(
		hrt_absolute_time(),
		(float) _bus_voltage * INA226_VSCALE,
		(float) _current * _current_lsb,
		success,
		battery_status_s::BATTERY_SOURCE_POWER_MODULE,
		0,
		_actuator_controls.control[actuator_controls_s::INDEX_THROTTLE]
	);

	perf_end(_sample_perf);

	if (success) {
		return PX4_OK;

	} else {
		return PX4_ERROR;
	}
}

void
INA226::start()
{
	ScheduleClear();

	/* reset the report ring and state machine */
	_collect_phase = false;

	_measure_interval = INA226_CONVERSION_INTERVAL;

	/* schedule a cycle to start things */
	ScheduleDelayed(5);
}

void
INA226::RunImpl()
{
	if (_initialized) {
		if (_collect_phase) {
			/* perform collection */
			if (collect() != PX4_OK) {
				perf_count(_collection_errors);
				/* if error restart the measurement state machine */
				start();
				return;
			}

			/* next phase is measurement */
			_collect_phase = !_mode_triggered;

			if (_measure_interval > INA226_CONVERSION_INTERVAL) {
				/* schedule a fresh cycle call when we are ready to measure again */
				ScheduleDelayed(_measure_interval - INA226_CONVERSION_INTERVAL);
				return;
			}
		}

		/* Measurement  phase */

		/* Perform measurement */
		if (measure() != PX4_OK) {
			perf_count(_measure_errors);
		}

		/* next phase is collection */
		_collect_phase = true;

		/* schedule a fresh cycle call when the measurement is done */
		ScheduleDelayed(INA226_CONVERSION_INTERVAL);

	} else {
		_battery.updateBatteryStatus(
			hrt_absolute_time(),
			0.0f,
			0.0f,
			false,
			battery_status_s::BATTERY_SOURCE_POWER_MODULE,
			0,
			0.0f
		);

		if (init() != PX4_OK) {
			ScheduleDelayed(INA226_INIT_RETRY_INTERVAL_US);
		}
	}
}

void
INA226::print_status()
{
	I2CSPIDriverBase::print_status();

	if (_initialized) {
		perf_print_counter(_sample_perf);
		perf_print_counter(_comms_errors);

		printf("poll interval:  %u \n", _measure_interval);

	} else {
		PX4_INFO("Device not initialized. Retrying every %d ms until battery is plugged in.",
			 INA226_INIT_RETRY_INTERVAL_US / 1000);
	}
}
