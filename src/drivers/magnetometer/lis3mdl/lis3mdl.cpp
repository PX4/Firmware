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

/**
 * @file lis3mdl.cpp
 *
 * Driver for the LIS3MDL magnetometer connected via I2C or SPI.
 *
 * Based on the hmc5883 driver.
 */

#include <px4_platform_common/time.h>
#include "lis3mdl.h"

LIS3MDL::LIS3MDL(device::Device *interface, const char *path, enum Rotation rotation) :
	CDev("LIS3MDL", path),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(interface->get_device_id())),
	_interface(interface),
	_reports(nullptr),
	_scale{},
	_last_report{},
	_mag_topic(nullptr),
	_comms_errors(perf_alloc(PC_COUNT, "lis3mdl_comms_errors")),
	_conf_errors(perf_alloc(PC_COUNT, "lis3mdl_conf_errors")),
	_range_errors(perf_alloc(PC_COUNT, "lis3mdl_range_errors")),
	_sample_perf(perf_alloc(PC_ELAPSED, "lis3mdl_read")),
	_calibrated(false),
	_continuous_mode_set(false),
	_mode(CONTINUOUS),
	_rotation(rotation),
	_measure_interval(0),
	_class_instance(-1),
	_orb_class_instance(-1),
	_range_ga(4.0f),
	_range_scale(0),                // default range scale from counts to gauss */
	_check_state_cnt(0),
	_cntl_reg1(
		CNTL_REG1_DEFAULT),     // 1 11 111 0 0 | temp-en, ultra high performance (XY), fast_odr disabled, self test disabled
	_cntl_reg2(CNTL_REG2_DEFAULT),  // 4 gauss FS range, reboot settings default
	_cntl_reg3(CNTL_REG3_DEFAULT),  // operating mode CONTINUOUS!
	_cntl_reg4(CNTL_REG4_DEFAULT),  // Z-axis ultra high performance mode
	_cntl_reg5(CNTL_REG5_DEFAULT),  // fast read disabled, continious update disabled (block data update)
	_range_bits(0),
	_temperature_counter(0),
	_temperature_error_count(0)
{
	// set the device type from the interface
	_device_id.devid_s.bus_type = _interface->get_device_bus_type();
	_device_id.devid_s.bus = _interface->get_device_bus();
	_device_id.devid_s.address = _interface->get_device_address();
	_device_id.devid_s.devtype = DRV_MAG_DEVTYPE_LIS3MDL;

	// default scaling
	_scale.x_offset = 0;
	_scale.x_scale = 1.0f;
	_scale.y_offset = 0;
	_scale.y_scale = 1.0f;
	_scale.z_offset = 0;
	_scale.z_scale = 1.0f;
}

LIS3MDL::~LIS3MDL()
{
	/* make sure we are truly inactive */
	stop();

	if (_mag_topic != nullptr) {
		orb_unadvertise(_mag_topic);
	}

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
LIS3MDL::calibrate(struct file *file_pointer, unsigned enable)
{
	sensor_mag_s report;
	ssize_t sz;
	int ret = 1;
	uint8_t num_samples = 10;

	// XXX do something smarter here
	int fd = (int)enable;

	float sum_excited[3] = {0.0f, 0.0f, 0.0f};
	float sum_non_excited[3] = {0.0f, 0.0f, 0.0f};

	/* start the sensor polling at 50 Hz */
	if (ioctl(file_pointer, SENSORIOCSPOLLRATE, 50) != OK) {
		warn("FAILED: SENSORIOCSPOLLRATE 50Hz");
		ret = 1;
		goto out;
	}

	/* Set to 12 Gauss */
	if (ioctl(file_pointer, MAGIOCSRANGE, 12) != OK) {
		PX4_WARN("FAILED: MAGIOCSRANGE 12 Ga");
		ret = 1;
		goto out;
	}

	px4_usleep(20000);

	/* discard 10 samples to let the sensor settle */
	for (uint8_t i = 0; i < num_samples; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = ::poll(&fds, 1, 2000);

		if (ret != 1) {
			warn("ERROR: TIMEOUT 1");
			goto out;
		}

		/* now go get it */
		sz = ::read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			warn("ERROR: READ 1");
			ret = -EIO;
			goto out;
		}
	}

	/* read the sensor up to 10x */
	for (uint8_t i = 0; i < num_samples; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = ::poll(&fds, 1, 2000);

		if (ret != 1) {
			warn("ERROR: TIMEOUT 2");
			goto out;
		}

		/* now go get it */
		sz = ::read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			warn("ERROR: READ 2");
			ret = -EIO;
			goto out;
		}

		sum_non_excited[0] += report.x;
		sum_non_excited[1] += report.y;
		sum_non_excited[2] += report.z;
	}

	sum_non_excited[0] /= num_samples;
	sum_non_excited[1] /= num_samples;
	sum_non_excited[2] /= num_samples;

	/* excite strap and take measurements */
	if (ioctl(file_pointer, MAGIOCEXSTRAP, 1) != OK) {
		PX4_WARN("FAILED: MAGIOCEXSTRAP 1");
		ret = 1;
		goto out;
	}

	px4_usleep(60000);

	/* discard 10 samples to let the sensor settle */
	for (uint8_t i = 0; i < num_samples; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = ::poll(&fds, 1, 2000);

		if (ret != 1) {
			warn("ERROR: TIMEOUT 1");
			goto out;
		}

		/* now go get it */
		sz = ::read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			warn("ERROR: READ 1");
			ret = -EIO;
			goto out;
		}
	}

	/* read the sensor up to 10x */
	for (uint8_t i = 0; i < 10; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = ::poll(&fds, 1, 2000);

		if (ret != 1) {
			warn("ERROR: TIMEOUT 2");
			goto out;
		}

		/* now go get it */
		sz = ::read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			warn("ERROR: READ 2");
			ret = -EIO;
			goto out;
		}

		sum_excited[0] += report.x;
		sum_excited[1] += report.y;
		sum_excited[2] += report.z;
	}

	sum_excited[0] /= num_samples;
	sum_excited[1] /= num_samples;
	sum_excited[2] /= num_samples;

	if (1.0f < fabsf(sum_excited[0] - sum_non_excited[0]) && fabsf(sum_excited[0] - sum_non_excited[0]) < 3.0f &&
	    1.0f < fabsf(sum_excited[1] - sum_non_excited[1]) && fabsf(sum_excited[1] - sum_non_excited[1]) < 3.0f &&
	    0.1f < fabsf(sum_excited[2] - sum_non_excited[2]) && fabsf(sum_excited[2] - sum_non_excited[2]) < 1.0f) {
		ret = OK;

	} else {
		ret = -EIO;
		goto out;
	}

out:

	/* set back to normal mode */
	set_range(4);
	set_default_register_values();

	px4_usleep(20000);

	return ret;
}

int
LIS3MDL::check_offset()
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

int
LIS3MDL::check_scale()
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

int
LIS3MDL::collect()
{
#pragma pack(push, 1)
	struct {
		uint8_t x[2];
		uint8_t y[2];
		uint8_t z[2];
	}       lis_report;

	struct {
		int16_t x;
		int16_t y;
		int16_t z;
		int16_t t;
	} report;
#pragma pack(pop)

	int     ret = 0;
	uint8_t buf_rx[2] = {0};

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

	ret = _interface->read(ADDR_OUT_X_L, (uint8_t *)&lis_report, sizeof(lis_report));

	/**
	 * Silicon Bug: the X axis will be read instead of the temperature registers if you do a sequential read through XYZ.
	 * The temperature registers must be addressed directly.
	 */
	ret = _interface->read(ADDR_OUT_T_L, (uint8_t *)&buf_rx, sizeof(buf_rx));

	if (ret != OK) {
		perf_count(_comms_errors);
		PX4_WARN("Register read error.");
		return ret;
	}

	report.x = (int16_t)((lis_report.x[1] << 8) | lis_report.x[0]);
	report.y = (int16_t)((lis_report.y[1] << 8) | lis_report.y[0]);
	report.z = (int16_t)((lis_report.z[1] << 8) | lis_report.z[0]);

	report.t = (int16_t)((buf_rx[1] << 8) | buf_rx[0]);

	float temperature = report.t;
	new_mag_report.temperature = 25.0f + (temperature / 8.0f);

	// XXX revisit for SPI part, might require a bus type IOCTL

	unsigned dummy = 0;
	sensor_is_onboard = !_interface->ioctl(MAGIOCGEXTERNAL, dummy);
	new_mag_report.is_external = !sensor_is_onboard;

	/**
	 * RAW outputs
	 */
	new_mag_report.x_raw = report.x;
	new_mag_report.y_raw = report.y;
	new_mag_report.z_raw = report.z;

	xraw_f = report.x;
	yraw_f = report.y;
	zraw_f = report.z;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	new_mag_report.x = ((xraw_f * _range_scale) - _scale.x_offset) * _scale.x_scale;
	/* flip axes and negate value for y */
	new_mag_report.y = ((yraw_f * _range_scale) - _scale.y_offset) * _scale.y_scale;
	/* z remains z */
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
LIS3MDL::Run()
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
		ScheduleDelayed(LIS3MDL_CONVERSION_INTERVAL);
	}
}

int
LIS3MDL::init()
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

	return PX4_OK;
}

int
LIS3MDL::ioctl(struct file *file_pointer, int cmd, unsigned long arg)
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
					_measure_interval = (LIS3MDL_CONVERSION_INTERVAL);

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
		return calibrate(file_pointer, arg);

	case MAGIOCEXSTRAP:
		return set_excitement(arg);

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
LIS3MDL::measure()
{
	int ret = 0;

	/* Send the command to begin a measurement. */
	if ((_mode == CONTINUOUS) && !_continuous_mode_set) {
		ret = write_reg(ADDR_CTRL_REG3, MODE_REG_CONTINOUS_MODE);
		_continuous_mode_set = true;

	} else if (_mode == SINGLE) {
		ret = write_reg(ADDR_CTRL_REG3, MODE_REG_SINGLE_MODE);
		_continuous_mode_set = false;
	}


	if (ret != OK) {
		perf_count(_comms_errors);
	}

	return ret;
}

void
LIS3MDL::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	PX4_INFO("poll interval:  %u", _measure_interval);
	print_message(_last_report);
	_reports->print_info("report queue");
}

int
LIS3MDL::reset()
{
	int ret = 0;

	ret = set_default_register_values();

	if (ret != OK) {
		return PX4_ERROR;
	}

	ret = set_range(_range_ga);

	if (ret != OK) {
		return PX4_ERROR;
	}

	return PX4_OK;
}

int
LIS3MDL::read(struct file *file_pointer, char *buffer, size_t buffer_len)
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
		px4_usleep(LIS3MDL_CONVERSION_INTERVAL);

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
LIS3MDL::set_default_register_values()
{
	write_reg(ADDR_CTRL_REG1, CNTL_REG1_DEFAULT);
	write_reg(ADDR_CTRL_REG2, CNTL_REG2_DEFAULT);
	write_reg(ADDR_CTRL_REG3, CNTL_REG3_DEFAULT);
	write_reg(ADDR_CTRL_REG4, CNTL_REG4_DEFAULT);
	write_reg(ADDR_CTRL_REG5, CNTL_REG5_DEFAULT);

	return PX4_OK;
}

int
LIS3MDL::set_excitement(unsigned enable)
{
	int ret;
	/* arm the excitement strap */
	ret = read_reg(ADDR_CTRL_REG1, _cntl_reg1);

	if (ret != OK) {
		perf_count(_comms_errors);
	}

	_cntl_reg1 &= ~0x01; // reset previous excitement mode

	if (enable > 0) {
		_cntl_reg1 |= 0x01;
	}

	::printf("set_excitement enable=%d cntl1=0x%x\n", (int)enable, (unsigned)_cntl_reg1);

	ret = write_reg(ADDR_CTRL_REG1, _cntl_reg1);

	if (ret != OK) {
		perf_count(_comms_errors);
	}

	uint8_t conf_reg_ret = 0;
	read_reg(ADDR_CTRL_REG1, conf_reg_ret);

	//print_info();

	return !(_cntl_reg1 == conf_reg_ret);
}

int
LIS3MDL::set_range(unsigned range)
{
	if (range <= 4) {
		_range_bits = 0x00;
		_range_scale = 1.0f / 6842.0f;
		_range_ga = 4.0f;

	} else if (range <= 8) {
		_range_bits = 0x01;
		_range_scale = 1.0f / 3421.0f;
		_range_ga = 8.0f;

	} else if (range <= 12) {
		_range_bits = 0x02;
		_range_scale = 1.0f / 2281.0f;
		_range_ga = 12.0f;

	} else {
		_range_bits = 0x03;
		_range_scale = 1.0f / 1711.0f;
		_range_ga = 16.0f;
	}

	int ret = 0;

	/*
	 * Send the command to set the range
	 */
	ret = write_reg(ADDR_CTRL_REG2, (_range_bits << 5));

	if (ret != OK) {
		perf_count(_comms_errors);
	}

	uint8_t range_bits_in = 0;
	ret = read_reg(ADDR_CTRL_REG2, range_bits_in);

	if (ret != OK) {
		perf_count(_comms_errors);
	}

	if (range_bits_in == (_range_bits << 5)) {
		return PX4_OK;

	} else {
		return PX4_ERROR;
	}
}

void
LIS3MDL::start()
{
	/* reset the report ring and state machine */
	_reports->flush();

	set_default_register_values();

	/* schedule a cycle to start things */
	ScheduleNow();
}

void
LIS3MDL::stop()
{
	if (_measure_interval > 0) {
		/* ensure no new items are queued while we cancel this one */
		_measure_interval = 0;
		ScheduleClear();
	}
}

int
LIS3MDL::read_reg(uint8_t reg, uint8_t &val)
{
	uint8_t buf = val;
	int ret = _interface->read(reg, &buf, 1);
	val = buf;
	return ret;
}

int
LIS3MDL::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t buf = val;
	return _interface->write(reg, &buf, 1);
}
