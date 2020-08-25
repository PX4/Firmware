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
	CDev("HMC5883", nullptr),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(interface->get_device_id()), bus_option, bus),
	_interface(interface),
	_reports(nullptr),
	_scale{},
	_range_scale(0), /* default range scale from counts to gauss */
	_range_ga(1.9f),
	_collect_phase(false),
	_class_instance(-1),
	_orb_class_instance(-1),
	_mag_topic(nullptr),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err")),
	_range_errors(perf_alloc(PC_COUNT, MODULE_NAME": rng_err")),
	_conf_errors(perf_alloc(PC_COUNT, MODULE_NAME": conf_err")),
	_sensor_ok(false),
	_rotation(rotation),
	_range_bits(0),
	_conf_reg(0),
	_temperature_counter(0),
	_temperature_error_count(0)
{
	// set the device type from the interface
	_device_id.devid_s.bus_type = _interface->get_device_bus_type();
	_device_id.devid_s.bus = _interface->get_device_bus();
	_device_id.devid_s.address = _interface->get_device_address();
	_device_id.devid_s.devtype = DRV_MAG_DEVTYPE_HMC5883;

	// default scaling
	_scale.x_offset = 0;
	_scale.x_scale = 1.0f;
	_scale.y_offset = 0;
	_scale.y_scale = 1.0f;
	_scale.z_offset = 0;
	_scale.z_scale = 1.0f;
}

HMC5883::~HMC5883()
{
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
HMC5883::init()
{
	int ret = PX4_ERROR;

	ret = CDev::init();

	if (ret != OK) {
		DEVICE_DEBUG("CDev init failed");
		goto out;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(sensor_mag_s));

	if (_reports == nullptr) {
		goto out;
	}

	/* reset the device configuration */
	reset();

	_class_instance = register_class_devname(MAG_BASE_DEVICE_PATH);

	ret = OK;
	/* sensor is ok, but not calibrated */
	_sensor_ok = true;

	_measure_interval = HMC5883_CONVERSION_INTERVAL;
	start();

out:
	return ret;
}

int HMC5883::set_range(unsigned range)
{
	if (range < 0.88f) {
		_range_bits = 0x00;
		_range_scale = 1.0f / 1370.0f;
		_range_ga = 0.88f;

	} else if (range <= 1.3f) {
		_range_bits = 0x01;
		_range_scale = 1.0f / 1090.0f;
		_range_ga = 1.3f;

	} else if (range <= 2) {
		_range_bits = 0x02;
		_range_scale = 1.0f / 820.0f;
		_range_ga = 1.9f;

	} else if (range <= 3) {
		_range_bits = 0x03;
		_range_scale = 1.0f / 660.0f;
		_range_ga = 2.5f;

	} else if (range <= 4) {
		_range_bits = 0x04;
		_range_scale = 1.0f / 440.0f;
		_range_ga = 4.0f;

	} else if (range <= 4.7f) {
		_range_bits = 0x05;
		_range_scale = 1.0f / 390.0f;
		_range_ga = 4.7f;

	} else if (range <= 5.6f) {
		_range_bits = 0x06;
		_range_scale = 1.0f / 330.0f;
		_range_ga = 5.6f;

	} else {
		_range_bits = 0x07;
		_range_scale = 1.0f / 230.0f;
		_range_ga = 8.1f;
	}

	int ret;

	/*
	 * Send the command to set the range
	 */
	ret = write_reg(ADDR_CONF_B, (_range_bits << 5));

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
void HMC5883::check_range(void)
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
void HMC5883::check_conf(void)
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

ssize_t
HMC5883::read(cdev::file_t *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(sensor_mag_s);
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
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		/* wait for it to complete */
		px4_usleep(HMC5883_CONVERSION_INTERVAL);

		/* run the collection phase */
		if (OK != collect()) {
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
HMC5883::ioctl(cdev::file_t *filp, int cmd, unsigned long arg)
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
					_measure_interval = HMC5883_CONVERSION_INTERVAL;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_interval == 0);

					/* convert hz to interval in microseconds */
					unsigned interval = (1000000 / arg);

					/* check against maximum rate */
					if (interval < HMC5883_CONVERSION_INTERVAL) {
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

	case MAGIOCSRANGE:
		return set_range(arg);

	case MAGIOCSSCALE:
		/* set new scale factors */
		memcpy(&_scale, (struct mag_calibration_s *)arg, sizeof(_scale));
		return 0;

	case MAGIOCGSCALE:
		/* copy out scale factors */
		memcpy((struct mag_calibration_s *)arg, &_scale, sizeof(_scale));
		return 0;

	case MAGIOCCALIBRATE:
		return calibrate(filp, arg);

	case MAGIOCEXSTRAP:
		return set_excitement(arg);

	case MAGIOCGEXTERNAL:
		DEVICE_DEBUG("MAGIOCGEXTERNAL in main driver");
		return _interface->ioctl(cmd, dummy);

	case MAGIOCSTEMPCOMP:
		return set_temperature_compensation(arg);

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}

void
HMC5883::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

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
			DEVICE_DEBUG("collection error");
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
		DEVICE_DEBUG("measure error");
	}

	/* next phase is collection */
	_collect_phase = true;

	if (_measure_interval > 0) {
		/* schedule a fresh cycle call when the measurement is done */
		ScheduleDelayed(HMC5883_CONVERSION_INTERVAL);
	}
}

int
HMC5883::measure()
{
	int ret;

	/*
	 * Send the command to begin a measurement.
	 */
	ret = write_reg(ADDR_MODE, MODE_REG_SINGLE_MODE);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	return ret;
}

int
HMC5883::collect()
{
#pragma pack(push, 1)
	struct { /* status register and data as read back from the device */
		uint8_t		x[2];
		uint8_t		z[2];
		uint8_t		y[2];
	}	hmc_report;
#pragma pack(pop)
	struct {
		int16_t		x, y, z;
	} report;

	int	ret;
	uint8_t check_counter;

	perf_begin(_sample_perf);
	sensor_mag_s new_report;
	bool sensor_is_onboard = false;

	float xraw_f;
	float yraw_f;
	float zraw_f;

	/* this should be fairly close to the end of the measurement, so the best approximation of the time */
	new_report.timestamp = hrt_absolute_time();
	new_report.error_count = perf_event_count(_comms_errors);
	new_report.scaling = _range_scale;
	new_report.device_id = _device_id.devid;

	/*
	 * @note  We could read the status register here, which could tell us that
	 *        we were too early and that the output registers are still being
	 *        written.  In the common case that would just slow us down, and
	 *        we're better off just never being early.
	 */

	/* get measurements from the device */
	ret = _interface->read(ADDR_DATA_OUT_X_MSB, (uint8_t *)&hmc_report, sizeof(hmc_report));

	if (ret != OK) {
		perf_count(_comms_errors);
		DEVICE_DEBUG("data/status read error");
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

	/* get measurements from the device */
	new_report.temperature = 0;

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
				new_report.temperature = 25 + (temp16 / (16 * 8.0f));
				_temperature_error_count = 0;

			} else {
				_temperature_error_count++;

				if (_temperature_error_count == 10) {
					/*
					  it probably really is an old HMC5883,
					  and can't do temperature. Disable it
					*/
					_temperature_error_count = 0;
					DEVICE_DEBUG("disabling temperature compensation");
					set_temperature_compensation(0);
				}
			}

		} else {
			new_report.temperature = _last_report.temperature;
		}
	}

	/*
	 * RAW outputs
	 *
	 * to align the sensor axes with the board, x and y need to be flipped
	 * and y needs to be negated
	 */
	new_report.x_raw = -report.y;
	new_report.y_raw = report.x;
	/* z remains z */
	new_report.z_raw = report.z;

	/* scale values for output */

	// XXX revisit for SPI part, might require a bus type IOCTL
	unsigned dummy;
	sensor_is_onboard = !_interface->ioctl(MAGIOCGEXTERNAL, dummy);
	new_report.is_external = !sensor_is_onboard;

	if (sensor_is_onboard) {
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

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	new_report.x = ((xraw_f * _range_scale) - _scale.x_offset) * _scale.x_scale;
	/* flip axes and negate value for y */
	new_report.y = ((yraw_f * _range_scale) - _scale.y_offset) * _scale.y_scale;
	/* z remains z */
	new_report.z = ((zraw_f * _range_scale) - _scale.z_offset) * _scale.z_scale;

	if (!(_pub_blocked)) {

		if (_mag_topic != nullptr) {
			/* publish it */
			orb_publish(ORB_ID(sensor_mag), _mag_topic, &new_report);

		} else {
			_mag_topic = orb_advertise_multi(ORB_ID(sensor_mag), &new_report,
							 &_orb_class_instance, (sensor_is_onboard) ? ORB_PRIO_HIGH : ORB_PRIO_MAX);

			if (_mag_topic == nullptr) {
				DEVICE_DEBUG("ADVERT FAIL");
			}
		}
	}

	_last_report = new_report;

	/* post a report to the ring */
	_reports->force(&new_report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

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

/**
 * Automatic scale calibration.
 *
 * Basic idea:
 *
 *   output = (ext field +- 1.1 Ga self-test) * scale factor
 *
 * and consequently:
 *
 *   1.1 Ga = (excited - normal) * scale factor
 *   scale factor = (excited - normal) / 1.1 Ga
 *
 *   sxy = (excited - normal) / 766	| for conf reg. B set to 0x60 / Gain = 3
 *   sz  = (excited - normal) / 713	| for conf reg. B set to 0x60 / Gain = 3
 *
 * By subtracting the non-excited measurement the pure 1.1 Ga reading
 * can be extracted and the sensitivity of all axes can be matched.
 *
 * SELF TEST OPERATION
 * To check the HMC5883L for proper operation, a self test feature in incorporated
 * in which the sensor offset straps are excited to create a nominal field strength
 * (bias field) to be measured. To implement self test, the least significant bits
 * (MS1 and MS0) of configuration register A are changed from 00 to 01 (positive bias)
 * or 10 (negetive bias), e.g. 0x11 or 0x12.
 * Then, by placing the mode register into single-measurement mode (0x01),
 * two data acquisition cycles will be made on each magnetic vector.
 * The first acquisition will be a set pulse followed shortly by measurement
 * data of the external field. The second acquisition will have the offset strap
 * excited (about 10 mA) in the positive bias mode for X, Y, and Z axes to create
 * about a ±1.1 gauss self test field plus the external field. The first acquisition
 * values will be subtracted from the second acquisition, and the net measurement
 * will be placed into the data output registers.
 * Since self test adds ~1.1 Gauss additional field to the existing field strength,
 * using a reduced gain setting prevents sensor from being saturated and data registers
 * overflowed. For example, if the configuration register B is set to 0x60 (Gain=3),
 * values around +766 LSB (1.16 Ga * 660 LSB/Ga) will be placed in the X and Y data
 * output registers and around +713 (1.08 Ga * 660 LSB/Ga) will be placed in Z data
 * output register. To leave the self test mode, change MS1 and MS0 bit of the
 * configuration register A back to 00 (Normal Measurement Mode), e.g. 0x10.
 * Using the self test method described above, the user can scale sensor
 */
int HMC5883::calibrate(cdev::file_t *filp, unsigned enable)
{
	sensor_mag_s report{};
	ssize_t sz;
	int ret = 1;
	uint8_t good_count = 0;

	// XXX do something smarter here
	int fd = (int)enable;

	struct mag_calibration_s mscale_previous;
	mscale_previous.x_offset = 0.0f;
	mscale_previous.x_scale = 1.0f;
	mscale_previous.y_offset = 0.0f;
	mscale_previous.y_scale = 1.0f;
	mscale_previous.z_offset = 0.0f;
	mscale_previous.z_scale = 1.0f;

	struct mag_calibration_s mscale_null;
	mscale_null.x_offset = 0.0f;
	mscale_null.x_scale = 1.0f;
	mscale_null.y_offset = 0.0f;
	mscale_null.y_scale = 1.0f;
	mscale_null.z_offset = 0.0f;
	mscale_null.z_scale = 1.0f;

	float sum_excited[3] = {0.0f, 0.0f, 0.0f};

	/* expected axis scaling. The datasheet says that 766 will
	 * be places in the X and Y axes and 713 in the Z
	 * axis. Experiments show that in fact 766 is placed in X,
	 * and 713 in Y and Z. This is relative to a base of 660
	 * LSM/Ga, giving 1.16 and 1.08 */
	float expected_cal[3] = { 1.16f, 1.08f, 1.08f };

	/* start the sensor polling at 50 Hz */
	if (OK != ioctl(filp, SENSORIOCSPOLLRATE, 50)) {
		PX4_ERR("FAILED: SENSORIOCSPOLLRATE 50Hz");
		ret = 1;
		goto out;
	}

	/* Set to 2.5 Gauss. We ask for 3 to get the right part of
	 * the chained if statement above. */
	if (OK != ioctl(filp, MAGIOCSRANGE, 3)) {
		PX4_ERR("FAILED: MAGIOCSRANGE 2.5 Ga");
		ret = 1;
		goto out;
	}

	if (OK != ioctl(filp, MAGIOCEXSTRAP, 1)) {
		PX4_ERR("FAILED: MAGIOCEXSTRAP 1");
		ret = 1;
		goto out;
	}

	if (OK != ioctl(filp, MAGIOCGSCALE, (long unsigned int)&mscale_previous)) {
		PX4_ERR("FAILED: MAGIOCGSCALE 1");
		ret = 1;
		goto out;
	}

	if (OK != ioctl(filp, MAGIOCSSCALE, (long unsigned int)&mscale_null)) {
		PX4_ERR("FAILED: MAGIOCSSCALE 1");
		ret = 1;
		goto out;
	}

	// discard 10 samples to let the sensor settle
	for (uint8_t i = 0; i < 10; i++) {
		px4_pollfd_struct_t fds{};

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = px4_poll(&fds, 1, 2000);

		if (ret != 1) {
			PX4_ERR("ERROR: TIMEOUT 1");
			goto out;
		}

		/* now go get it */
		sz = px4_read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			PX4_ERR("ERROR: READ 1");
			ret = -EIO;
			goto out;
		}
	}

	/* read the sensor up to 150x, stopping when we have 50 good values */
	for (uint8_t i = 0; i < 150 && good_count < 50; i++) {
		px4_pollfd_struct_t fds{};

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = px4_poll(&fds, 1, 2000);

		if (ret != 1) {
			PX4_ERR("ERROR: TIMEOUT 2");
			goto out;
		}

		/* now go get it */
		sz = px4_read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			PX4_ERR("ERROR: READ 2");
			ret = -EIO;
			goto out;
		}

		float cal[3] = {fabsf(expected_cal[0] / report.x),
				fabsf(expected_cal[1] / report.y),
				fabsf(expected_cal[2] / report.z)
			       };

		if (cal[0] > 0.3f && cal[0] < 1.7f &&
		    cal[1] > 0.3f && cal[1] < 1.7f &&
		    cal[2] > 0.3f && cal[2] < 1.7f) {
			good_count++;
			sum_excited[0] += cal[0];
			sum_excited[1] += cal[1];
			sum_excited[2] += cal[2];
		}
	}

	if (good_count < 5) {
		ret = -EIO;
		goto out;
	}

	float scaling[3];

	scaling[0] = sum_excited[0] / good_count;
	scaling[1] = sum_excited[1] / good_count;
	scaling[2] = sum_excited[2] / good_count;

	/* set scaling in device */
	mscale_previous.x_scale = 1.0f / scaling[0];
	mscale_previous.y_scale = 1.0f / scaling[1];
	mscale_previous.z_scale = 1.0f / scaling[2];

	ret = OK;

out:

	if (OK != ioctl(filp, MAGIOCSSCALE, (long unsigned int)&mscale_previous)) {
		PX4_ERR("FAILED: MAGIOCSSCALE 2");
	}

	/* set back to normal mode */
	/* Set to 1.9 Gauss */
	if (OK != px4_ioctl(fd, MAGIOCSRANGE, 2)) {
		PX4_ERR("FAILED: MAGIOCSRANGE 1.9 Ga");
	}

	if (OK != px4_ioctl(fd, MAGIOCEXSTRAP, 0)) {
		PX4_ERR("FAILED: MAGIOCEXSTRAP 0");
	}

	if (ret == OK) {
		if (check_scale()) {
			/* failed */
			PX4_ERR("FAILED: SCALE");
			ret = PX4_ERROR;
		}

	}

	return ret;
}

int HMC5883::check_scale()
{
	bool scale_valid;

	if ((-FLT_EPSILON + 1.0f < _scale.x_scale && _scale.x_scale < FLT_EPSILON + 1.0f) &&
	    (-FLT_EPSILON + 1.0f < _scale.y_scale && _scale.y_scale < FLT_EPSILON + 1.0f) &&
	    (-FLT_EPSILON + 1.0f < _scale.z_scale && _scale.z_scale < FLT_EPSILON + 1.0f)) {
		/* scale is one */
		scale_valid = false;

	} else {
		scale_valid = true;
	}

	/* return 0 if calibrated, 1 else */
	return !scale_valid;
}

int HMC5883::check_offset()
{
	bool offset_valid;

	if ((-2.0f * FLT_EPSILON < _scale.x_offset && _scale.x_offset < 2.0f * FLT_EPSILON) &&
	    (-2.0f * FLT_EPSILON < _scale.y_offset && _scale.y_offset < 2.0f * FLT_EPSILON) &&
	    (-2.0f * FLT_EPSILON < _scale.z_offset && _scale.z_offset < 2.0f * FLT_EPSILON)) {
		/* offset is zero */
		offset_valid = false;

	} else {
		offset_valid = true;
	}

	/* return 0 if calibrated, 1 else */
	return !offset_valid;
}

int HMC5883::set_excitement(unsigned enable)
{
	int ret;
	/* arm the excitement strap */
	ret = read_reg(ADDR_CONF_A, _conf_reg);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	_conf_reg &= ~0x03; // reset previous excitement mode

	if (((int)enable) < 0) {
		_conf_reg |= 0x01;

	} else if (enable > 0) {
		_conf_reg |= 0x02;

	}

	// ::printf("set_excitement enable=%d regA=0x%x\n", (int)enable, (unsigned)_conf_reg);

	ret = write_reg(ADDR_CONF_A, _conf_reg);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	uint8_t conf_reg_ret = 0;
	read_reg(ADDR_CONF_A, conf_reg_ret);

	//print_info();

	return !(_conf_reg == conf_reg_ret);
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

int
HMC5883::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t buf = val;
	return _interface->write(reg, &buf, 1);
}

int
HMC5883::read_reg(uint8_t reg, uint8_t &val)
{
	uint8_t buf = val;
	int ret = _interface->read(reg, &buf, 1);
	val = buf;
	return ret;
}

float
HMC5883::meas_to_float(uint8_t in[2])
{
	union {
		uint8_t	b[2];
		int16_t	w;
	} u;

	u.b[0] = in[1];
	u.b[1] = in[0];

	return (float) u.w;
}

void
HMC5883::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("interval:  %u us\n", _measure_interval);
	print_message(_last_report);
	_reports->print_info("report queue");
}
