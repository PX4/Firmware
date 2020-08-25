/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file lps22hb.cpp
 *
 * Driver for the LPS22HB barometer connected via I2C or SPI.
 */

#include "LPS22HB.hpp"

/* Max measurement rate is 25Hz */
#define LPS22HB_CONVERSION_INTERVAL	(1000000 / 25)	/* microseconds */

LPS22HB::LPS22HB(I2CSPIBusOption bus_option, int bus, device::Device *interface) :
	CDev(nullptr),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(interface->get_device_id()), bus_option, bus),
	_interface(interface),
	_sample_perf(perf_alloc(PC_ELAPSED, "lps22hb_read")),
	_comms_errors(perf_alloc(PC_COUNT, "lps22hb_comms_errors"))
{
	// set the interface device type
	_interface->set_device_type(DRV_BARO_DEVTYPE_LPS22HB);
}

LPS22HB::~LPS22HB()
{
	if (_class_instance != -1) {
		unregister_class_devname(BARO_BASE_DEVICE_PATH, _class_instance);
	}

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);

	delete _interface;
}

int
LPS22HB::init()
{
	int ret = CDev::init();

	if (ret != OK) {
		PX4_DEBUG("CDev init failed");
		goto out;
	}

	if (reset() != OK) {
		goto out;
	}

	/* register alternate interfaces if we have to */
	_class_instance = register_class_devname(BARO_BASE_DEVICE_PATH);

	ret = OK;

	_measure_interval = LPS22HB_CONVERSION_INTERVAL;
	start();

out:
	return ret;
}

int
LPS22HB::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	unsigned dummy = arg;

	switch (cmd) {
	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default polling rate */
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_interval == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_interval = (LPS22HB_CONVERSION_INTERVAL);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						_measure_interval = (LPS22HB_CONVERSION_INTERVAL);
						start();
					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_interval == 0);

					/* convert hz to tick interval via microseconds */
					unsigned interval = (1000000 / arg);

					/* check against maximum rate */
					if (interval < (LPS22HB_CONVERSION_INTERVAL)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_interval = interval;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCRESET:
		return reset();

	case DEVIOCGDEVICEID:
		return _interface->ioctl(cmd, dummy);

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}

void
LPS22HB::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;

	/* schedule a cycle to start things */
	ScheduleNow();
}

int
LPS22HB::reset()
{
	int ret = PX4_ERROR;

	ret = write_reg(CTRL_REG2, BOOT | SWRESET);

	return ret;
}

void
LPS22HB::RunImpl()
{
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
		if (_measure_interval > LPS22HB_CONVERSION_INTERVAL) {

			/* schedule a fresh cycle call when we are ready to measure again */
			ScheduleDelayed(_measure_interval - LPS22HB_CONVERSION_INTERVAL);

			return;
		}
	}

	/* measurement phase */
	if (OK != measure()) {
		PX4_DEBUG("measure error");
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(LPS22HB_CONVERSION_INTERVAL);
}

int
LPS22HB::measure()
{
	// Send the command to begin a 16-bit measurement.
	int ret = write_reg(CTRL_REG2, IF_ADD_INC | ONE_SHOT);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	return ret;
}

int
LPS22HB::collect()
{
	perf_begin(_sample_perf);
	sensor_baro_s new_report;

	/* get measurements from the device : MSB enables register address auto-increment */
#pragma pack(push, 1)
	struct {
		uint8_t		STATUS;
		uint8_t		PRESS_OUT_XL;
		uint8_t		PRESS_OUT_L;
		uint8_t		PRESS_OUT_H;
		uint8_t		TEMP_OUT_L;
		uint8_t		TEMP_OUT_H;
	} report;
#pragma pack(pop)

	/* this should be fairly close to the end of the measurement, so the best approximation of the time */
	new_report.timestamp = hrt_absolute_time();
	int ret = _interface->read(STATUS, (uint8_t *)&report, sizeof(report));

	if (ret != OK) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	// To obtain the pressure in hPa, take the two’s complement of the complete word and then divide by 4096 LSB/hPa.
	uint32_t P = report.PRESS_OUT_XL + (report.PRESS_OUT_L << 8) + (report.PRESS_OUT_H << 16);

	uint32_t TEMP_OUT = report.TEMP_OUT_L + (report.TEMP_OUT_H << 8);

	/* Pressure and MSL in mBar */
	new_report.pressure = P / 4096.0f;
	new_report.temperature = 42.5f + (TEMP_OUT / 480.0f);

	/* get device ID */
	new_report.device_id = _interface->get_device_id();
	new_report.error_count = perf_event_count(_comms_errors);

	if (_baro_topic != nullptr) {
		/* publish it */
		orb_publish(ORB_ID(sensor_baro), _baro_topic, &new_report);

	} else {
		bool sensor_is_onboard = !_interface->external();
		_baro_topic = orb_advertise_multi(ORB_ID(sensor_baro), &new_report, &_orb_class_instance,
						  (sensor_is_onboard) ? ORB_PRIO_HIGH : ORB_PRIO_MAX);

		if (_baro_topic == nullptr) {
			PX4_ERR("advertise failed");
		}
	}

	_last_report = new_report;

	ret = OK;

	perf_end(_sample_perf);
	return ret;
}

int
LPS22HB::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t buf = val;
	return _interface->write(reg, &buf, 1);
}

int
LPS22HB::read_reg(uint8_t reg, uint8_t &val)
{
	uint8_t buf = val;
	int ret = _interface->read(reg, &buf, 1);
	val = buf;
	return ret;
}

void
LPS22HB::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);

	PX4_INFO("poll interval:  %u", _measure_interval);

	print_message(_last_report);
}
