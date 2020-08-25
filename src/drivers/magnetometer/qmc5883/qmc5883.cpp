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

/**
 * @file qmc5883.cpp
 *
 * Driver for the QMC5883 magnetometer connected via I2C or SPI.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <nuttx/clock.h>

#include <board_config.h>

#include <perf/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_mag.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_device.h>

#include <uORB/uORB.h>

#include <float.h>
#include <getopt.h>
#include <lib/conversion/rotation.h>

#include "qmc5883.h"

/*
 * QMC5883 internal constants and data structures.
 */

/* Max measurement rate is 200Hz */
#define QMC5883_CONVERSION_INTERVAL	(1000000 / 150)	/* microseconds */
#define QMC5883_MAX_COUNT 			32767

#define QMC5883_ADDR_DATA_OUT_X_LSB		0x00
#define QMC5883_ADDR_DATA_OUT_X_MSB		0x01
#define QMC5883_ADDR_DATA_OUT_Y_LSB		0x02
#define QMC5883_ADDR_DATA_OUT_Y_MSB		0x03
#define QMC5883_ADDR_DATA_OUT_Z_LSB		0x04
#define QMC5883_ADDR_DATA_OUT_Z_MSB		0x05
#define QMC5883_ADDR_STATUS			0x06
#define QMC5883_ADDR_TEMP_OUT_LSB 		0x07
#define QMC5883_ADDR_TEMP_OUT_MSB 		0x08
#define QMC5883_ADDR_CONTROL_1 			0x09
#define QMC5883_ADDR_CONTROL_2 			0x0A
#define QMC5883_ADDR_SET_RESET 			0x0B


#define QMC5883_STATUS_REG_DRDY 		(1 << 0)  /* Data Ready: "0": no new data, "1": new data is ready */
#define QMC5883_STATUS_REG_OVL 			(1 << 1)  /* Overflow Flag: "0": normal, "1": data overflow */
#define QMC5883_STATUS_REG_DOR			(1 << 2)  /* Data Skip: "0": normal, "1": data skipped for reading */

/* Control Register 1 */
#define QMC5883_MODE_REG_STANDBY 		(0 << 0)
#define QMC5883_MODE_REG_CONTINOUS_MODE         (1 << 0)
#define QMC5883_OUTPUT_DATA_RATE_10 		(0 << 2) /* Hz */
#define QMC5883_OUTPUT_DATA_RATE_50 		(1 << 2)
#define QMC5883_OUTPUT_DATA_RATE_100 		(2 << 2)
#define QMC5883_OUTPUT_DATA_RATE_200 		(3 << 2)
#define QMC5883_OUTPUT_RANGE_2G 		(0 << 4)  /* +/- 2 gauss */
#define QMC5883_OUTPUT_RANGE_8G 		(1 << 4)  /* +/- 8 gauss */
#define QMC5883_OVERSAMPLE_512 			(0 << 6)  /* controls digital filter bw - larger OSR -> smaller bw */
#define QMC5883_OVERSAMPLE_256 			(1 << 6)
#define QMC5883_OVERSAMPLE_128 			(2 << 6)
#define QMC5883_OVERSAMPLE_64 			(3 << 6)

/* Control Register 2 */
#define QMC5883_INT_ENB 			(1 << 0)
#define QMC5883_ROL_PNT 			(1 << 6)
#define QMC5883_SOFT_RESET 			(1 << 7)

/* Set Register */
#define QMC5883_SET_DEFAULT 			(1 << 0)

#define QMC5883_TEMP_OFFSET 50 /* deg celsius */

enum QMC5883_BUS {
	QMC5883_BUS_ALL = 0,
	QMC5883_BUS_I2C_INTERNAL,
	QMC5883_BUS_I2C_EXTERNAL,
	QMC5883_BUS_SPI
};

class QMC5883 : public device::CDev, public px4::ScheduledWorkItem
{
public:
	QMC5883(device::Device *interface, const char *path, enum Rotation rotation);
	virtual ~QMC5883();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Stop the automatic measurement state machine.
	 */
	void			stop();

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

protected:
	Device			*_interface;

private:
	unsigned		_measure_interval{0};

	ringbuffer::RingBuffer	*_reports;
	struct mag_calibration_s	_scale;
	float 			_range_scale;
	float 			_range_ga;
	bool			_collect_phase;
	int			_class_instance;
	int			_orb_class_instance;

	orb_advert_t		_mag_topic;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_range_errors;
	perf_counter_t		_conf_errors;

	/* status reporting */
	bool			_sensor_ok;		/**< sensor was found and reports ok */

	enum Rotation		_rotation;

	sensor_mag_s	_last_report {};         /**< used for info() */

	uint8_t			_range_bits;
	uint8_t			_conf_reg;
	uint8_t			_temperature_counter;
	uint8_t			_temperature_error_count;

	/**
	 * Initialise the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void			start();

	/**
	 * Reset the device
	 */
	int			reset();

	/**
	 * check the sensor configuration.
	 *
	 * checks that the config of the sensor is correctly set, to
	 * cope with communication errors causing the configuration to
	 * change
	 */
	void 			check_conf(void);

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 *
	 * This is the heart of the measurement state machine.  This function
	 * alternately starts a measurement, or collects the data from the
	 * previous measurement.
	 *
	 * When the interval between measurements is greater than the minimum
	 * measurement interval, a gap is inserted between collection
	 * and measurement to provide the most recent measurement possible
	 * at the next interval.
	 */
	void			Run() override;

	/**
	 * Write a register.
	 *
	 * @param reg		The register to write.
	 * @param val		The value to write.
	 * @return		OK on write success.
	 */
	int			write_reg(uint8_t reg, uint8_t val);

	/**
	 * Read a register.
	 *
	 * @param reg		The register to read.
	 * @param val		The value read.
	 * @return		OK on read success.
	 */
	int			read_reg(uint8_t reg, uint8_t &val);

	/**
	 * Collect the result of the most recent measurement.
	 */
	int			collect();

	/* this class has pointer data members, do not allow copying it */
	QMC5883(const QMC5883 &);
	QMC5883 operator=(const QMC5883 &);
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int qmc5883_main(int argc, char *argv[]);


QMC5883::QMC5883(device::Device *interface, const char *path, enum Rotation rotation) :
	CDev("QMC5883", path),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(interface->get_device_id())),
	_interface(interface),
	_reports(nullptr),
	_scale{},
	_range_scale(1.0f / 12000.0f),
	_range_ga(2.0f),
	_collect_phase(false),
	_class_instance(-1),
	_orb_class_instance(-1),
	_mag_topic(nullptr),
	_sample_perf(perf_alloc(PC_ELAPSED, "qmc5883_read")),
	_comms_errors(perf_alloc(PC_COUNT, "qmc5883_com_err")),
	_range_errors(perf_alloc(PC_COUNT, "qmc5883_rng_err")),
	_conf_errors(perf_alloc(PC_COUNT, "qmc5883_conf_err")),
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
	_device_id.devid_s.devtype = DRV_MAG_DEVTYPE_QMC5883;

	// default scaling
	_scale.x_offset = 0;
	_scale.x_scale = 1.0f;
	_scale.y_offset = 0;
	_scale.y_scale = 1.0f;
	_scale.z_offset = 0;
	_scale.z_scale = 1.0f;
}

QMC5883::~QMC5883()
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
QMC5883::init()
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
	/* sensor is ok */
	_sensor_ok = true;
out:
	return ret;
}

/**
   check that the configuration register has the right value. This is
   done periodically to cope with I2C bus noise causing the
   configuration of the compass to change.
 */
void QMC5883::check_conf(void)
{
	int ret;

	uint8_t conf_reg_in = 0;
	ret = read_reg(QMC5883_ADDR_CONTROL_1, conf_reg_in);

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

ssize_t
QMC5883::read(struct file *filp, char *buffer, size_t buflen)
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

		/* wait for it to complete */
		usleep(QMC5883_CONVERSION_INTERVAL);

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
QMC5883::ioctl(struct file *filp, int cmd, unsigned long arg)
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
					_measure_interval = QMC5883_CONVERSION_INTERVAL;

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

					/* convert hz to tick interval via microseconds */
					unsigned interval = (1000000 / arg);

					/* check against maximum rate */
					if (interval < QMC5883_CONVERSION_INTERVAL) {
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
		return OK;

	case MAGIOCSSCALE:
		/* set new scale factors */
		memcpy(&_scale, (struct mag_calibration_s *)arg, sizeof(_scale));
		return 0;

	case MAGIOCGSCALE:
		/* copy out scale factors */
		memcpy((struct mag_calibration_s *)arg, &_scale, sizeof(_scale));
		return 0;

	case MAGIOCGEXTERNAL:
		DEVICE_DEBUG("MAGIOCGEXTERNAL in main driver");
		return _interface->ioctl(cmd, dummy);

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}

void
QMC5883::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	/* schedule a cycle to start things */
	ScheduleNow();
}

void
QMC5883::stop()
{
	if (_measure_interval > 0) {
		/* ensure no new items are queued while we cancel this one */
		_measure_interval = 0;
		ScheduleClear();
	}
}

int
QMC5883::reset()
{
	/* read 0x00 once */
	uint8_t data_bits_in = 0;
	read_reg(QMC5883_ADDR_DATA_OUT_X_LSB, data_bits_in);

	/* software reset */
	write_reg(QMC5883_ADDR_CONTROL_2, QMC5883_SOFT_RESET);

	/* set reset period to 0x01 */
	write_reg(QMC5883_ADDR_SET_RESET, QMC5883_SET_DEFAULT);

	//use fixed range of 2G
	_range_scale = 1.0f / 12000.0f;   // 12000 LSB/Gauss at +/- 2G range
	_range_ga = 2.00f;
	_range_bits = 0x00;

	/* set control register */
	_conf_reg = QMC5883_MODE_REG_CONTINOUS_MODE |
		    QMC5883_OUTPUT_DATA_RATE_200 |
		    QMC5883_OVERSAMPLE_512 |
		    QMC5883_OUTPUT_RANGE_2G;
	write_reg(QMC5883_ADDR_CONTROL_1, _conf_reg);

	return OK;
}

void
QMC5883::Run()
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


int
QMC5883::collect()
{
#pragma pack(push, 1)
	struct { /* status register and data as read back from the device */
		uint8_t		x[2];
		uint8_t		y[2];
		uint8_t		z[2];
	}	qmc_report;
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
	ret = _interface->read(QMC5883_ADDR_DATA_OUT_X_LSB, (uint8_t *)&qmc_report, sizeof(qmc_report));

	if (ret != OK) {
		perf_count(_comms_errors);
		DEVICE_DEBUG("data/status read error");
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
	new_report.temperature = 0;

	if (_temperature_counter++ == 100) {
		uint8_t raw_temperature[2];

		_temperature_counter = 0;

		ret = _interface->read(QMC5883_ADDR_TEMP_OUT_LSB,
				       raw_temperature, sizeof(raw_temperature));

		if (ret == OK) {
			int16_t temp16 = (((int16_t)raw_temperature[1]) << 8) +
					 raw_temperature[0];
			new_report.temperature = QMC5883_TEMP_OFFSET + temp16 * 1.0f / 100.0f;
		}

	} else {
		new_report.temperature = _last_report.temperature;
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
	//TODO: sort out axes mapping
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

int
QMC5883::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t buf = val;
	return _interface->write(reg, &buf, 1);
}

int
QMC5883::read_reg(uint8_t reg, uint8_t &val)
{
	uint8_t buf = val;
	int ret = _interface->read(reg, &buf, 1);
	val = buf;
	return ret;
}

void
QMC5883::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u us\n", _measure_interval);
	print_message(_last_report);
	_reports->print_info("report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace qmc5883
{

/*
  list of supported bus configurations
 */
struct qmc5883_bus_option {
	enum QMC5883_BUS busid;
	const char *devpath;
	QMC5883_constructor interface_constructor;
	uint8_t busnum;
	QMC5883	*dev;
} bus_options[] = {
	{ QMC5883_BUS_I2C_EXTERNAL, "/dev/qmc5883_ext", &QMC5883_I2C_interface, PX4_I2C_BUS_EXPANSION, NULL },
#ifdef PX4_I2C_BUS_EXPANSION1
	{ QMC5883_BUS_I2C_EXTERNAL, "/dev/qmc5883_ext1", &QMC5883_I2C_interface, PX4_I2C_BUS_EXPANSION1, NULL },
#endif
#ifdef PX4_I2C_BUS_EXPANSION2
	{ QMC5883_BUS_I2C_EXTERNAL, "/dev/qmc5883_ext2", &QMC5883_I2C_interface, PX4_I2C_BUS_EXPANSION2, NULL },
#endif
#ifdef PX4_I2C_BUS_ONBOARD
	{ QMC5883_BUS_I2C_INTERNAL, "/dev/qmc5883_int", &QMC5883_I2C_interface, PX4_I2C_BUS_ONBOARD, NULL },
#endif
#ifdef PX4_SPIDEV_HMC
	{ QMC5883_BUS_SPI, "/dev/qmc5883_spi", &QMC5883_SPI_interface, PX4_SPI_BUS_SENSORS, NULL },
#endif
};
#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))

void	start(enum QMC5883_BUS busid, enum Rotation rotation);
int		stop();
bool	start_bus(struct qmc5883_bus_option &bus, enum Rotation rotation);
struct qmc5883_bus_option &find_bus(enum QMC5883_BUS busid);
void	test(enum QMC5883_BUS busid);
void	reset(enum QMC5883_BUS busid);
int	info(enum QMC5883_BUS busid);
int	temp_enable(QMC5883_BUS busid, bool enable);
void	usage();

/**
 * start driver for a specific bus option
 */
bool
start_bus(struct qmc5883_bus_option &bus, enum Rotation rotation)
{
	if (bus.dev != nullptr) {
		errx(1, "bus option already started");
	}

	device::Device *interface = bus.interface_constructor(bus.busnum);

	if (interface->init() != OK) {
		delete interface;
		warnx("no device on bus %u (type: %u)", (unsigned)bus.busnum, (unsigned)bus.busid);
		return false;
	}

	bus.dev = new QMC5883(interface, bus.devpath, rotation);

	if (bus.dev != nullptr && OK != bus.dev->init()) {
		delete bus.dev;
		bus.dev = NULL;
		return false;
	}

	int fd = open(bus.devpath, O_RDONLY);

	if (fd < 0) {
		return false;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		close(fd);
		errx(1, "Failed to setup poll rate");
	}

	close(fd);

	return true;
}

/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * is either successfully up and running or failed to start.
 */
void
start(enum QMC5883_BUS busid, enum Rotation rotation)
{
	bool started = false;

	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (busid == QMC5883_BUS_ALL && bus_options[i].dev != NULL) {
			// this device is already started
			continue;
		}

		if (busid != QMC5883_BUS_ALL && bus_options[i].busid != busid) {
			// not the one that is asked for
			continue;
		}

		started |= start_bus(bus_options[i], rotation);
	}

	if (!started) {
		exit(1);
	}
}

int
stop()
{
	bool stopped = false;

	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (bus_options[i].dev != nullptr) {
			bus_options[i].dev->stop();
			delete bus_options[i].dev;
			bus_options[i].dev = nullptr;
			stopped = true;
		}
	}

	return !stopped;
}

/**
 * find a bus structure for a busid
 */
struct qmc5883_bus_option &find_bus(enum QMC5883_BUS busid)
{
	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if ((busid == QMC5883_BUS_ALL ||
		     busid == bus_options[i].busid) && bus_options[i].dev != NULL) {
			return bus_options[i];
		}
	}

	errx(1, "bus %u not started", (unsigned)busid);
}


/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test(enum QMC5883_BUS busid)
{
	struct qmc5883_bus_option &bus = find_bus(busid);
	sensor_mag_s report;
	ssize_t sz;
	int ret;
	const char *path = bus.devpath;

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'qmc5883 start')", path);
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		err(1, "immediate read failed");
	}

	print_message(report);

	/* check if mag is onboard or external */
	if ((ret = ioctl(fd, MAGIOCGEXTERNAL, 0)) < 0) {
		errx(1, "failed to get if mag is onboard or external");
	}

	warnx("device active: %s", ret ? "external" : "onboard");

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			errx(1, "timed out waiting for sensor data");
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			err(1, "periodic read failed");
		}

		print_message(report);
	}

	PX4_INFO("PASS");
	exit(0);
}

/**
 * Reset the driver.
 */
void
reset(enum QMC5883_BUS busid)
{
	struct qmc5883_bus_option &bus = find_bus(busid);
	const char *path = bus.devpath;

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

	exit(0);
}


/**
 * Print a little info about the driver.
 */
int
info(enum QMC5883_BUS busid)
{
	struct qmc5883_bus_option &bus = find_bus(busid);

	warnx("running on bus: %u (%s)\n", (unsigned)bus.busid, bus.devpath);
	bus.dev->print_info();
	exit(0);
}

void
usage()
{
	warnx("missing command: try 'start', 'info', 'test', 'reset', 'info'");
	warnx("options:");
	warnx("    -R rotation");
	warnx("    -X only external bus");
#if (PX4_I2C_BUS_ONBOARD || PX4_SPIDEV_HMC)
	warnx("    -I only internal bus");
#endif
}

} // namespace

int
qmc5883_main(int argc, char *argv[])
{
	int ch;
	enum QMC5883_BUS busid = QMC5883_BUS_ALL;
	enum Rotation rotation = ROTATION_NONE;

	if (argc < 2) {
		qmc5883::usage();
		exit(0);
	}

	while ((ch = getopt(argc, argv, "XISR:CT")) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (enum Rotation)atoi(optarg);
			break;
#if (PX4_I2C_BUS_ONBOARD || PX4_SPIDEV_HMC)

		case 'I':
			busid = QMC5883_BUS_I2C_INTERNAL;
			break;
#endif

		case 'X':
			busid = QMC5883_BUS_I2C_EXTERNAL;
			break;

		case 'S':
			busid = QMC5883_BUS_SPI;
			break;

		default:
			qmc5883::usage();
			exit(0);
		}
	}

	const char *verb = argv[optind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		qmc5883::start(busid, rotation);

		exit(0);
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(verb, "stop")) {
		return qmc5883::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		qmc5883::test(busid);
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		qmc5883::reset(busid);
	}


	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info") || !strcmp(verb, "status")) {
		qmc5883::info(busid);
	}

	errx(1, "unrecognized command, try 'start', 'test', 'reset', or 'info'");
}
