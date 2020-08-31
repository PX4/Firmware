/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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

#include "QMC5883.hpp"

QMC5883::QMC5883(device::Device *interface, enum Rotation rotation, I2CSPIBusOption bus_option, int bus,
		 int i2c_address) :
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(interface->get_device_id()), bus_option, bus, i2c_address),
	_px4_mag(interface->get_device_id(), rotation),
	_interface(interface),
	_collect_phase(false),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err")),
	_range_errors(perf_alloc(PC_COUNT, MODULE_NAME": rng_err")),
	_conf_errors(perf_alloc(PC_COUNT, MODULE_NAME": conf_err")),
	_conf_reg(0),
	_temperature_counter(0),
	_temperature_error_count(0)
{
	_px4_mag.set_external(_interface->external());
}

QMC5883::~QMC5883()
{
	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_range_errors);
	perf_free(_conf_errors);

	delete _interface;
}

int QMC5883::init()
{
	/* reset the device configuration */
	reset();

	_measure_interval = QMC5883_CONVERSION_INTERVAL;
	start();

	return PX4_OK;
}

/**
   check that the configuration register has the right value. This is
   done periodically to cope with I2C bus noise causing the
   configuration of the compass to change.
 */
void QMC5883::check_conf()
{
	uint8_t conf_reg_in = 0;
	int ret = read_reg(QMC5883_ADDR_CONTROL_1, conf_reg_in);

	if (OK != ret) {
		perf_count(_comms_errors);
		return;
	}

	if (conf_reg_in != _conf_reg) {
		perf_count(_conf_errors);
		ret = write_reg(QMC5883_ADDR_CONTROL_1, _conf_reg);

		if (OK != ret) {
			perf_count(_comms_errors);
		}
	}
}

void QMC5883::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;

	/* schedule a cycle to start things */
	ScheduleNow();
}

int QMC5883::reset()
{
	/* read 0x00 once */
	uint8_t data_bits_in = 0;
	read_reg(QMC5883_ADDR_DATA_OUT_X_LSB, data_bits_in);

	/* software reset */
	write_reg(QMC5883_ADDR_CONTROL_2, QMC5883_SOFT_RESET);

	/* set reset period to 0x01 */
	write_reg(QMC5883_ADDR_SET_RESET, QMC5883_SET_DEFAULT);

	// use fixed range of 2G
	_px4_mag.set_scale(1.f / 12000.f); // 12000 LSB/Gauss at +/- 2G range

	/* set control register */
	_conf_reg = QMC5883_MODE_REG_CONTINOUS_MODE |
		    QMC5883_OUTPUT_DATA_RATE_200 |
		    QMC5883_OVERSAMPLE_512 |
		    QMC5883_OUTPUT_RANGE_2G;
	write_reg(QMC5883_ADDR_CONTROL_1, _conf_reg);

	return OK;
}

void QMC5883::RunImpl()
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
		if (_measure_interval > QMC5883_CONVERSION_INTERVAL) {
			/* schedule a fresh cycle call when we are ready to measure again */
			ScheduleDelayed(_measure_interval - QMC5883_CONVERSION_INTERVAL);

			return;
		}
	}

	/* next phase is collection */
	_collect_phase = true;

	if (_measure_interval > 0) {
		/* schedule a fresh cycle call when the measurement is done */
		ScheduleDelayed(QMC5883_CONVERSION_INTERVAL);
	}
}

int QMC5883::collect()
{
	struct { /* status register and data as read back from the device */
		uint8_t		x[2];
		uint8_t		y[2];
		uint8_t		z[2];
	} qmc_report{};

	struct {
		int16_t		x, y, z;
	} report{};

	int	ret;
	uint8_t check_counter;

	perf_begin(_sample_perf);

	float xraw_f;
	float yraw_f;
	float zraw_f;

	_px4_mag.set_error_count(perf_event_count(_comms_errors));

	/*
	 * @note  We could read the status register here, which could tell us that
	 *        we were too early and that the output registers are still being
	 *        written.  In the common case that would just slow us down, and
	 *        we're better off just never being early.
	 */

	/* get measurements from the device */
	const hrt_abstime timestamp_sample = hrt_absolute_time();
	ret = _interface->read(QMC5883_ADDR_DATA_OUT_X_LSB, (uint8_t *)&qmc_report, sizeof(qmc_report));

	if (ret != OK) {
		perf_count(_comms_errors);
		PX4_DEBUG("data/status read error");
		goto out;
	}

	/* map data we just received LSB, MSB */
	report.x = (((int16_t)qmc_report.x[1]) << 8) + qmc_report.x[0];
	report.y = (((int16_t)qmc_report.y[1]) << 8) + qmc_report.y[0];
	report.z = (((int16_t)qmc_report.z[1]) << 8) + qmc_report.z[0];

	/*
	 * If any of the values are -4096, there was an internal math error in the sensor.
	 * Generalise this to a simple range check that will also catch some bit errors.
	 */
	if ((abs(report.x) > QMC5883_MAX_COUNT) ||
	    (abs(report.y) > QMC5883_MAX_COUNT) ||
	    (abs(report.z) > QMC5883_MAX_COUNT)) {
		perf_count(_comms_errors);
		goto out;
	}

	/* get temperature measurements from the device */
	if (_temperature_counter++ == 100) {
		uint8_t raw_temperature[2];

		_temperature_counter = 0;

		ret = _interface->read(QMC5883_ADDR_TEMP_OUT_LSB,
				       raw_temperature, sizeof(raw_temperature));

		if (ret == OK) {
			int16_t temp16 = (((int16_t)raw_temperature[1]) << 8) +
					 raw_temperature[0];
			float temperature = QMC5883_TEMP_OFFSET + temp16 * 1.0f / 100.0f;
			_px4_mag.set_temperature(temperature);
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
	//TODO: sort out axes mapping
	xraw_f = -report.y;
	yraw_f = report.x;
	zraw_f = report.z;

	_px4_mag.update(timestamp_sample, xraw_f, yraw_f, zraw_f);

	/*
	  periodically check the configuration
	  registers. With a bad I2C cable it is possible for the
	  registers to become corrupt, leading to bad readings. It
	  doesn't happen often, but given the poor cables some
	  vehicles have it is worth checking for.
	 */
	check_counter = perf_event_count(_sample_perf) % 256;

	if (check_counter == 0) {
		check_conf();
	}

	ret = OK;

out:
	perf_end(_sample_perf);
	return ret;
}

int QMC5883::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t buf = val;
	return _interface->write(reg, &buf, 1);
}

int QMC5883::read_reg(uint8_t reg, uint8_t &val)
{
	uint8_t buf = val;
	int ret = _interface->read(reg, &buf, 1);
	val = buf;
	return ret;
}

void QMC5883::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("interval:  %u us\n", _measure_interval);
}
