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
 * @file rm3100.cpp
 *
 * Driver for the RM3100 magnetometer connected via I2C or SPI.
 *
 * Based on the lis3mdl driver.
 */

#include "rm3100.h"

RM3100::RM3100(device::Device *interface, const char *path, enum Rotation rotation) :
	CDev("RM3100", path),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(interface->get_device_id())),
	_interface(interface),
	_reports(nullptr),
	_scale{},
	_last_report{},
	_mag_topic(nullptr),
	_comms_errors(perf_alloc(PC_COUNT, "rm3100_comms_errors")),
	_conf_errors(perf_alloc(PC_COUNT, "rm3100_conf_errors")),
	_range_errors(perf_alloc(PC_COUNT, "rm3100_range_errors")),
	_sample_perf(perf_alloc(PC_ELAPSED, "rm3100_read")),
	_calibrated(false),
	_continuous_mode_set(false),
	_mode(SINGLE),
	_rotation(rotation),
	_measure_interval(0),
	_class_instance(-1),
	_orb_class_instance(-1),
	_range_scale(1.0f / (RM3100_SENSITIVITY * UTESLA_TO_GAUSS)),
	_check_state_cnt(0)
{
	// set the device type from the interface
	_device_id.devid_s.bus_type = _interface->get_device_bus_type();
	_device_id.devid_s.bus = _interface->get_device_bus();
	_device_id.devid_s.address = _interface->get_device_address();
	_device_id.devid_s.devtype = DRV_MAG_DEVTYPE_RM3100;

	// default scaling
	_scale.x_offset = 0;
	_scale.x_scale = 1.0f;
	_scale.y_offset = 0;
	_scale.y_scale = 1.0f;
	_scale.z_offset = 0;
	_scale.z_scale = 1.0f;
}

RM3100::~RM3100()
{
	/* make sure we are truly inactive */
	stop();

	if (_reports != nullptr) {
		delete _reports;
	}

	if (_class_instance != -1) {
		unregister_class_devname(MAG_BASE_DEVICE_PATH, _class_instance);
	}

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_range_errors);
	perf_free(_conf_errors);
}

int
RM3100::self_test()
{
	/* Stop current measurements */
	stop();

	/* Chances are that a poll event was triggered, so wait for conversion and read registers in order to clear DRDY bit */
	usleep(RM3100_CONVERSION_INTERVAL);
	collect();

	/* Fail if calibration is not good */
	int ret = 0;
	uint8_t cmd = 0;

	/* Configure mag into self test mode */
	cmd = BIST_SELFTEST;
	ret = _interface->write(ADDR_BIST, &cmd, 1);

	if (ret != PX4_OK) {
		return ret;
	}

	/* Now we need to write to POLL to launch self test */
	cmd = POLL_XYZ;
	ret = _interface->write(ADDR_POLL, &cmd, 1);

	if (ret != PX4_OK) {
		return ret;
	}

	/* Now wait for status register */
	usleep(RM3100_CONVERSION_INTERVAL);

	if (check_measurement() != PX4_OK) {
		return -1;;
	}

	/* Now check BIST register to see whether self test is ok or not*/
	ret = _interface->read(ADDR_BIST, &cmd, 1);

	if (ret != PX4_OK) {
		return ret;
	}

	ret = !((cmd & BIST_XYZ_OK) == BIST_XYZ_OK);

	/* Restart measurement state machine */
	start();

	return ret;
}

int
RM3100::check_measurement()
{
	uint8_t status = 0;
	int ret = -1;

	ret = _interface->read(ADDR_STATUS, &status, 1);

	if (ret != 0) {
		return ret;
	}

	return !((status & STATUS_DRDY) == STATUS_DRDY) ;
}

int
RM3100::collect()
{
	/* Check whether a measurement is available or not, otherwise return immediately */
	if (check_measurement() != 0) {
		DEVICE_DEBUG("No measurement available");
		return 0;
	}

#pragma pack(push, 1)
	struct {
		uint8_t x[3];
		uint8_t y[3];
		uint8_t z[3];
	}       rm_report;
#pragma pack(pop)

	int     ret = 0;

	int32_t xraw;
	int32_t yraw;
	int32_t zraw;

	float xraw_f;
	float yraw_f;
	float zraw_f;

	sensor_mag_s new_mag_report;
	bool sensor_is_onboard = false;

	perf_begin(_sample_perf);

	new_mag_report.timestamp = hrt_absolute_time();
	new_mag_report.error_count = perf_event_count(_comms_errors);
	new_mag_report.scaling = _range_scale;
	new_mag_report.device_id = _device_id.devid;

	ret = _interface->read(ADDR_MX, (uint8_t *)&rm_report, sizeof(rm_report));

	if (ret != OK) {
		perf_count(_comms_errors);
		PX4_WARN("Register read error.");
		return ret;
	}

	/* Rearrange mag data */
	xraw = ((rm_report.x[0] << 16) | (rm_report.x[1] << 8) | rm_report.x[2]);
	yraw = ((rm_report.y[0] << 16) | (rm_report.y[1] << 8) | rm_report.y[2]);
	zraw = ((rm_report.z[0] << 16) | (rm_report.z[1] << 8) | rm_report.z[2]);

	/* Convert 24 bit signed values to 32 bit signed values */
	convert_signed(&xraw);
	convert_signed(&yraw);
	convert_signed(&zraw);

	/* There is no temperature sensor */
	new_mag_report.temperature = 0.0f;

	// XXX revisit for SPI part, might require a bus type IOCTL
	unsigned dummy = 0;
	sensor_is_onboard = !_interface->ioctl(MAGIOCGEXTERNAL, dummy);
	new_mag_report.is_external = !sensor_is_onboard;

	/**
	 * RAW outputs
	 * As we only have 16 bits to store raw data, the following values are not correct
	 */
	new_mag_report.x_raw = (int16_t)(xraw >> 8);
	new_mag_report.y_raw = (int16_t)(yraw >> 8);
	new_mag_report.z_raw = (int16_t)(zraw >> 8);

	xraw_f = xraw;
	yraw_f = yraw;
	zraw_f = zraw;

	/* apply user specified rotation */
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	new_mag_report.x = ((xraw_f * _range_scale) - _scale.x_offset) * _scale.x_scale;
	new_mag_report.y = ((yraw_f * _range_scale) - _scale.y_offset) * _scale.y_scale;
	new_mag_report.z = ((zraw_f * _range_scale) - _scale.z_offset) * _scale.z_scale;

	if (!(_pub_blocked)) {

		if (_mag_topic != nullptr) {
			/* publish it */
			orb_publish(ORB_ID(sensor_mag), _mag_topic, &new_mag_report);

		} else {
			_mag_topic = orb_advertise_multi(ORB_ID(sensor_mag), &new_mag_report,
							 &_orb_class_instance, (sensor_is_onboard) ? ORB_PRIO_HIGH : ORB_PRIO_MAX);

			if (_mag_topic == nullptr) {
				DEVICE_DEBUG("ADVERT FAIL");
			}
		}
	}

	_last_report = new_mag_report;

	/* post a report to the ring */
	_reports->force(&new_mag_report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;

	perf_end(_sample_perf);
	return ret;
}

void
RM3100::convert_signed(int32_t *n)
{
	/* Sensor returns values as 24 bit signed values, so we need to manually convert to 32 bit signed values */
	if ((*n & (1 << 23)) == (1 << 23)) {
		*n |= 0xFF000000;
	}
}

void
RM3100::Run()
{
	/* _measure_interval == 0  is used as _task_should_exit */
	if (_measure_interval == 0) {
		return;
	}

	/* Collect last measurement at the start of every cycle */
	if (collect() != OK) {
		DEVICE_DEBUG("collection error");
		/* restart the measurement state machine */
		start();
		return;
	}


	if (measure() != OK) {
		DEVICE_DEBUG("measure error");
	}

	if (_measure_interval > 0) {
		/* schedule a fresh cycle call when the measurement is done */
		ScheduleDelayed(_measure_interval);
	}
}

int
RM3100::init()
{
	int ret = PX4_ERROR;

	ret = CDev::init();

	if (ret != OK) {
		DEVICE_DEBUG("CDev init failed");
		return ret;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(sensor_mag_s));

	if (_reports == nullptr) {
		return PX4_ERROR;
	}

	/* reset the device configuration */
	reset();

	_class_instance = register_class_devname(MAG_BASE_DEVICE_PATH);

	ret = self_test();

	if (ret != PX4_OK) {
		PX4_ERR("self test failed");
	}

	return ret;
}

int
RM3100::ioctl(struct file *file_pointer, int cmd, unsigned long arg)
{
	unsigned dummy = 0;

	switch (cmd) {
	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* zero would be bad */
			case 0:
				return -EINVAL;

			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool not_started = (_measure_interval == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_interval = (RM3100_CONVERSION_INTERVAL);

					/* if we need to start the poll state machine, do it */
					if (not_started) {
						start();
					}

					return PX4_OK;
				}

			/* Uses arg (hz) for a custom poll rate */
			default: {
					/* do we need to start internal polling? */
					bool not_started = (_measure_interval == 0);

					/* convert hz to tick interval via microseconds */
					unsigned interval = (1000000 / arg);

					/* check against maximum rate */
					if (interval < RM3100_CONVERSION_INTERVAL) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_interval = interval;

					/* if we need to start the poll state machine, do it */
					if (not_started) {
						start();
					}

					return PX4_OK;
				}
			}
		}

	case SENSORIOCRESET:
		return reset();

	case MAGIOCSRANGE:
		/* field measurement range cannot be configured for this sensor (8 Gauss) */
		return OK;

	case MAGIOCSSCALE:
		/* set new scale factors */
		memcpy(&_scale, (struct mag_calibration_s *)arg, sizeof(_scale));
		return 0;

	case MAGIOCGSCALE:
		/* copy out scale factors */
		memcpy((struct mag_calibration_s *)arg, &_scale, sizeof(_scale));
		return 0;


	case MAGIOCCALIBRATE:
		/* This is left for compatibility with the IOCTL call in mag calibration */
		return OK;

	case MAGIOCGEXTERNAL:
		DEVICE_DEBUG("MAGIOCGEXTERNAL in main driver");
		return _interface->ioctl(cmd, dummy);

	case DEVIOCGDEVICEID:
		return _interface->ioctl(cmd, dummy);

	default:
		/* give it to the superclass */
		return CDev::ioctl(file_pointer, cmd, arg);
	}
}

int
RM3100::measure()
{
	int ret = 0;
	uint8_t cmd = 0;

	/* Send the command to begin a measurement. */
	if ((_mode == CONTINUOUS) && !_continuous_mode_set) {
		cmd = (CMM_DEFAULT | CONTINUOUS_MODE);
		ret = _interface->write(ADDR_CMM, &cmd, 1);
		_continuous_mode_set = true;

	} else if (_mode == SINGLE) {
		if (_continuous_mode_set) {
			/* This is needed for polling mode */
			cmd = (CMM_DEFAULT | POLLING_MODE);
			ret = _interface->write(ADDR_CMM, &cmd, 1);

			if (ret != OK) {
				perf_count(_comms_errors);
				return ret;
			}

			_continuous_mode_set = false;
		}

		cmd = POLL_XYZ;
		ret = _interface->write(ADDR_POLL, &cmd, 1);
	}


	if (ret != OK) {
		perf_count(_comms_errors);
	}

	return ret;
}

void
RM3100::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	PX4_INFO("poll interval:  %u", _measure_interval);
	print_message(_last_report);
	_reports->print_info("report queue");
}

int
RM3100::reset()
{
	int ret = 0;

	ret = set_default_register_values();

	if (ret != OK) {
		return PX4_ERROR;
	}

	return PX4_OK;
}

int
RM3100::read(struct file *file_pointer, char *buffer, size_t buffer_len)
{
	unsigned count = buffer_len / sizeof(sensor_mag_s);
	sensor_mag_s *mag_buf = reinterpret_cast<sensor_mag_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_measure_interval > 0) {
		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(mag_buf)) {
				ret += sizeof(sensor_mag_s);
				mag_buf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	/* XXX really it'd be nice to lock against other readers here */
	do {
		_reports->flush();

		/* trigger a measurement */
		if (measure() != OK) {
			ret = -EIO;
			break;
		}

		/* wait for it to complete */
		usleep(RM3100_CONVERSION_INTERVAL);

		/* run the collection phase */
		if (collect() != OK) {
			ret = -EIO;
			break;
		}

		if (_reports->get(mag_buf)) {
			ret = sizeof(sensor_mag_s);
		}
	} while (0);

	return ret;
}

int
RM3100::set_default_register_values()
{
	uint8_t cmd[2] = {0, 0};

	cmd[0] = CCX_DEFAULT_MSB;
	cmd[1] = CCX_DEFAULT_LSB;
	_interface->write(ADDR_CCX, cmd, 2);

	cmd[0] = CCY_DEFAULT_MSB;
	cmd[1] = CCY_DEFAULT_LSB;
	_interface->write(ADDR_CCY, cmd, 2);

	cmd[0] = CCZ_DEFAULT_MSB;
	cmd[1] = CCZ_DEFAULT_LSB;
	_interface->write(ADDR_CCZ, cmd, 2);

	cmd[0] = CMM_DEFAULT;
	_interface->write(ADDR_CMM, cmd, 1);

	cmd[0] = TMRC_DEFAULT;
	_interface->write(ADDR_TMRC, cmd, 1);

	cmd[0] = BIST_DEFAULT;
	_interface->write(ADDR_BIST, cmd, 1);

	return PX4_OK;
}

void
RM3100::start()
{
	/* reset the report ring and state machine */
	_reports->flush();

	set_default_register_values();
	_measure_interval = (RM3100_CONVERSION_INTERVAL);

	/* schedule a cycle to start things */
	ScheduleNow();
}

void
RM3100::stop()
{
	if (_measure_interval > 0) {
		/* ensure no new items are queued while we cancel this one */
		_measure_interval = 0;
		ScheduleClear();
	}
}
