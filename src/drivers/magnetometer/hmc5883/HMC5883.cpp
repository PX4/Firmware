/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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

#include "HMC5883.hpp"

HMC5883::HMC5883(device::Device *interface, enum Rotation rotation, I2CSPIBusOption bus_option, int bus) :
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(interface->get_device_id()), bus_option, bus),
	_px4_mag(interface->get_device_id(), rotation),
	_interface(interface),
	_range_ga(1.9f),
	_collect_phase(false),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err")),
	_range_errors(perf_alloc(PC_COUNT, MODULE_NAME": rng_err")),
	_conf_errors(perf_alloc(PC_COUNT, MODULE_NAME": conf_err")),
	_range_bits(0),
	_conf_reg(0),
	_temperature_counter(0),
	_temperature_error_count(0)
{
	_px4_mag.set_external(_interface->external());
}

HMC5883::~HMC5883()
{
	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_range_errors);
	perf_free(_conf_errors);

	delete _interface;
}

int HMC5883::init()
{
	/* reset the device configuration */
	reset();

	_measure_interval = HMC5883_CONVERSION_INTERVAL;
	start();

	return PX4_OK;
}

int HMC5883::set_range(unsigned range)
{
	if (range < 0.88f) {
		_range_bits = 0x00;
		_px4_mag.set_scale(1.0f / 1370.0f);
		_range_ga = 0.88f;

	} else if (range <= 1.3f) {
		_range_bits = 0x01;
		_px4_mag.set_scale(1.0f / 1090.0f);
		_range_ga = 1.3f;

	} else if (range <= 2) {
		_range_bits = 0x02;
		_px4_mag.set_scale(1.0f / 820.0f);
		_range_ga = 1.9f;

	} else if (range <= 3) {
		_range_bits = 0x03;
		_px4_mag.set_scale(1.0f / 660.0f);
		_range_ga = 2.5f;

	} else if (range <= 4) {
		_range_bits = 0x04;
		_px4_mag.set_scale(1.0f / 440.0f);
		_range_ga = 4.0f;

	} else if (range <= 4.7f) {
		_range_bits = 0x05;
		_px4_mag.set_scale(1.0f / 390.0f);
		_range_ga = 4.7f;

	} else if (range <= 5.6f) {
		_range_bits = 0x06;
		_px4_mag.set_scale(1.0f / 330.0f);
		_range_ga = 5.6f;

	} else {
		_range_bits = 0x07;
		_px4_mag.set_scale(1.0f / 230.0f);
		_range_ga = 8.1f;
	}

	/*
	 * Send the command to set the range
	 */
	int ret = write_reg(ADDR_CONF_B, (_range_bits << 5));

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	uint8_t range_bits_in = 0;
	ret = read_reg(ADDR_CONF_B, range_bits_in);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	return !(range_bits_in == (_range_bits << 5));
}

/**
   check that the range register has the right value. This is done
   periodically to cope with I2C bus noise causing the range of the
   compass changing.
 */
void HMC5883::check_range()
{
	int ret;

	uint8_t range_bits_in = 0;
	ret = read_reg(ADDR_CONF_B, range_bits_in);

	if (OK != ret) {
		perf_count(_comms_errors);
		return;
	}

	if (range_bits_in != (_range_bits << 5)) {
		perf_count(_range_errors);
		ret = write_reg(ADDR_CONF_B, (_range_bits << 5));

		if (OK != ret) {
			perf_count(_comms_errors);
		}
	}
}

/**
   check that the configuration register has the right value. This is
   done periodically to cope with I2C bus noise causing the
   configuration of the compass to change.
 */
void HMC5883::check_conf()
{
	int ret;

	uint8_t conf_reg_in = 0;
	ret = read_reg(ADDR_CONF_A, conf_reg_in);

	if (OK != ret) {
		perf_count(_comms_errors);
		return;
	}

	if (conf_reg_in != _conf_reg) {
		perf_count(_conf_errors);
		ret = write_reg(ADDR_CONF_A, _conf_reg);

		if (OK != ret) {
			perf_count(_comms_errors);
		}
	}
}

void
HMC5883::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;

	/* schedule a cycle to start things */
	ScheduleNow();
}

int
HMC5883::reset()
{
	/* set range, ceil floating point number */
	return set_range(_range_ga + 0.5f);
}

void
HMC5883::RunImpl()
{
	if (_measure_interval == 0) {
		return;
	}

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		if (OK != collect()) {
			PX4_DEBUG("collection error");
			/* restart the measurement state machine */
			start();
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 */
		if (_measure_interval > HMC5883_CONVERSION_INTERVAL) {

			/* schedule a fresh cycle call when we are ready to measure again */
			ScheduleDelayed(_measure_interval - HMC5883_CONVERSION_INTERVAL);

			return;
		}
	}

	/* measurement phase */
	if (OK != measure()) {
		PX4_DEBUG("measure error");
	}

	/* next phase is collection */
	_collect_phase = true;

	if (_measure_interval > 0) {
		/* schedule a fresh cycle call when the measurement is done */
		ScheduleDelayed(HMC5883_CONVERSION_INTERVAL);
	}
}

int HMC5883::measure()
{
	/*
	 * Send the command to begin a measurement.
	 */
	int ret = write_reg(ADDR_MODE, MODE_REG_SINGLE_MODE);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	return ret;
}

int HMC5883::collect()
{
	struct { /* status register and data as read back from the device */
		uint8_t		x[2];
		uint8_t		z[2];
		uint8_t		y[2];
	} hmc_report{};

	struct {
		int16_t	x, y, z;
	} report{};

	uint8_t check_counter;

	float xraw_f;
	float yraw_f;
	float zraw_f;

	_px4_mag.set_error_count(perf_event_count(_comms_errors));

	perf_begin(_sample_perf);

	/*
	 * @note  We could read the status register here, which could tell us that
	 *        we were too early and that the output registers are still being
	 *        written.  In the common case that would just slow us down, and
	 *        we're better off just never being early.
	 */

	/* get measurements from the device */
	const hrt_abstime timestamp_sample = hrt_absolute_time();
	int ret = _interface->read(ADDR_DATA_OUT_X_MSB, (uint8_t *)&hmc_report, sizeof(hmc_report));

	if (ret != OK) {
		perf_count(_comms_errors);
		PX4_DEBUG("data/status read error");
		goto out;
	}

	/* swap the data we just received */
	report.x = (((int16_t)hmc_report.x[0]) << 8) + hmc_report.x[1];
	report.y = (((int16_t)hmc_report.y[0]) << 8) + hmc_report.y[1];
	report.z = (((int16_t)hmc_report.z[0]) << 8) + hmc_report.z[1];

	/*
	 * If any of the values are -4096, there was an internal math error in the sensor.
	 * Generalise this to a simple range check that will also catch some bit errors.
	 */
	if ((abs(report.x) > 2048) ||
	    (abs(report.y) > 2048) ||
	    (abs(report.z) > 2048)) {
		perf_count(_comms_errors);
		goto out;
	}

	if (_conf_reg & HMC5983_TEMP_SENSOR_ENABLE) {
		/*
		  if temperature compensation is enabled read the
		  temperature too.

		  We read the temperature every 10 samples to avoid
		  excessive I2C traffic
		 */
		if (_temperature_counter++ == 10) {
			uint8_t raw_temperature[2];

			_temperature_counter = 0;

			ret = _interface->read(ADDR_TEMP_OUT_MSB,
					       raw_temperature, sizeof(raw_temperature));

			if (ret == OK) {
				int16_t temp16 = (((int16_t)raw_temperature[0]) << 8) +
						 raw_temperature[1];
				float temperature = 25 + (temp16 / (16 * 8.0f));
				_px4_mag.set_temperature(temperature);
				_temperature_error_count = 0;

			} else {
				_temperature_error_count++;

				if (_temperature_error_count == 10) {
					/*
					  it probably really is an old HMC5883,
					  and can't do temperature. Disable it
					*/
					_temperature_error_count = 0;
					PX4_DEBUG("disabling temperature compensation");
					set_temperature_compensation(0);
				}
			}
		}
	}

	/*
	 * RAW outputs
	 *
	 * to align the sensor axes with the board, x and y need to be flipped
	 * and y needs to be negated
	 */
	if (!_px4_mag.external()) {
		// convert onboard so it matches offboard for the
		// scaling below
		report.y = -report.y;
		report.x = -report.x;
	}

	/* the standard external mag by 3DR has x pointing to the
	 * right, y pointing backwards, and z down, therefore switch x
	 * and y and invert y */
	xraw_f = -report.y;
	yraw_f = report.x;
	zraw_f = report.z;

	_px4_mag.update(timestamp_sample, xraw_f, yraw_f, zraw_f);

	/*
	  periodically check the range register and configuration
	  registers. With a bad I2C cable it is possible for the
	  registers to become corrupt, leading to bad readings. It
	  doesn't happen often, but given the poor cables some
	  vehicles have it is worth checking for.
	 */
	check_counter = perf_event_count(_sample_perf) % 256;

	if (check_counter == 0) {
		check_range();
	}

	if (check_counter == 128) {
		check_conf();
	}

	ret = OK;

out:
	perf_end(_sample_perf);
	return ret;
}

/*
  enable/disable temperature compensation on the HMC5983

  Unfortunately we don't yet know of a way to auto-detect the
  difference between the HMC5883 and HMC5983. Both of them do
  temperature sensing, but only the 5983 does temperature
  compensation. We have noy yet found a behaviour that can be reliably
  distinguished by reading registers to know which type a particular
  sensor is

  update: Current best guess is that many sensors marked HMC5883L on
  the package are actually 5983 but without temperature compensation
  tables. Reading the temperature works, but the mag field is not
  automatically adjusted for temperature. We suspect that there may be
  some early 5883L parts that don't have the temperature sensor at
  all, although we haven't found one yet. The code that reads the
  temperature looks for 10 failed transfers in a row and disables the
  temperature sensor if that happens. It is hoped that this copes with
  the genuine 5883L parts.
 */
int HMC5883::set_temperature_compensation(unsigned enable)
{
	int ret;
	/* get current config */
	ret = read_reg(ADDR_CONF_A, _conf_reg);

	if (OK != ret) {
		perf_count(_comms_errors);
		return -EIO;
	}

	if (enable != 0) {
		_conf_reg |= HMC5983_TEMP_SENSOR_ENABLE;

	} else {
		_conf_reg &= ~HMC5983_TEMP_SENSOR_ENABLE;
	}

	ret = write_reg(ADDR_CONF_A, _conf_reg);

	if (OK != ret) {
		perf_count(_comms_errors);
		return -EIO;
	}

	uint8_t conf_reg_ret = 0;

	if (read_reg(ADDR_CONF_A, conf_reg_ret) != OK) {
		perf_count(_comms_errors);
		return -EIO;
	}

	return conf_reg_ret == _conf_reg;
}

int HMC5883::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t buf = val;
	return _interface->write(reg, &buf, 1);
}

int HMC5883::read_reg(uint8_t reg, uint8_t &val)
{
	uint8_t buf = val;
	int ret = _interface->read(reg, &buf, 1);
	val = buf;
	return ret;
}

void HMC5883::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("interval:  %u us\n", _measure_interval);
}
