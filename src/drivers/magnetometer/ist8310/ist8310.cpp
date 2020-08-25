/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file ist8310.cpp
 *
 * Driver for the IST8310 magnetometer connected via I2C.
 *
 * @author David Sidrane
 * @author Maelok Dong
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

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
#include <nuttx/clock.h>

#include <board_config.h>

#include <perf/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/device/i2c.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_device.h>

#include <uORB/uORB.h>

#include <float.h>
#include <lib/conversion/rotation.h>


/*
 * IST8310 internal constants and data structures.
 */

/* Max measurement rate is 160Hz, however with 160 it will be set to 166 Hz, therefore workaround using 150
 * The datasheet gives 200Hz maximum measurement rate, but it's not true according to tech support from iSentek*/
#define IST8310_CONVERSION_INTERVAL	(1000000 / 100) /* microseconds */

#define IST8310_BUS_I2C_ADDR		0xE
#define IST8310_DEFAULT_BUS_SPEED	400000

/*
 * FSR:
 *   x, y: +- 1600 µT
 *   z:    +- 2500 µT
 *
 * Resolution according to datasheet is 0.3µT/LSB
 */
#define IST8310_RESOLUTION	0.3

static const int16_t IST8310_MAX_VAL_XY	= (1600 / IST8310_RESOLUTION) + 1;
static const int16_t IST8310_MIN_VAL_XY	= -IST8310_MAX_VAL_XY;
static const int16_t IST8310_MAX_VAL_Z  = (2500 / IST8310_RESOLUTION) + 1;
static const int16_t IST8310_MIN_VAL_Z  = -IST8310_MAX_VAL_Z;

/* Hardware definitions */

#define ADDR_WAI                0		/* WAI means 'Who Am I'*/
# define WAI_EXPECTED_VALUE     0x10

#define ADDR_STAT1              0x02
# define STAT1_DRDY_SHFITS      0x0
# define STAT1_DRDY             (1 << STAT1_DRDY_SHFITS)
# define STAT1_DRO_SHFITS       0x1
# define STAT1_DRO              (1 << STAT1_DRO_SHFITS)

#define ADDR_DATA_OUT_X_LSB     0x03
#define ADDR_DATA_OUT_X_MSB     0x04
#define ADDR_DATA_OUT_Y_LSB     0x05
#define ADDR_DATA_OUT_Y_MSB     0x06
#define ADDR_DATA_OUT_Z_LSB     0x07
#define ADDR_DATA_OUT_Z_MSB     0x08

#define ADDR_STAT2              0x09
# define STAT2_INT_SHFITS       3
# define STAT2_INT              (1 << STAT2_INT_SHFITS)

#define ADDR_CTRL1              0x0a
# define CTRL1_MODE_SHFITS      0
# define CTRL1_MODE_STDBY       (0 << CTRL1_MODE_SHFITS)
# define CTRL1_MODE_SINGLE      (1 << CTRL1_MODE_SHFITS)

#define ADDR_CTRL2              0x0b
# define CTRL2_SRST_SHFITS      0   /* Begin POR (auto cleared) */
# define CTRL2_SRST             (1 << CTRL2_SRST_SHFITS)
# define CTRL2_DRP_SHIFTS       2
# define CTRL2_DRP              (1 << CTRL2_DRP_SHIFTS)
# define CTRL2_DREN_SHIFTS      3
# define CTRL2_DREN             (1 << CTRL2_DREN_SHIFTS)

#define ADDR_CTRL3				0x41
# define CTRL3_SAMPLEAVG_16		0x24	/* Sample Averaging 16 */
# define CTRL3_SAMPLEAVG_8		0x1b	/* Sample Averaging 8 */
# define CTRL3_SAMPLEAVG_4		0x12	/* Sample Averaging 4 */
# define CTRL3_SAMPLEAVG_2		0x09	/* Sample Averaging 2 */

#define ADDR_CTRL4				0x42
# define CTRL4_SRPD				0xC0	/* Set Reset Pulse Duration */

#define ADDR_STR                0x0c
# define STR_SELF_TEST_SHFITS   6
# define STR_SELF_TEST_ON       (1 << STR_SELF_TEST_SHFITS)
# define STR_SELF_TEST_OFF      (0 << STR_SELF_TEST_SHFITS)

#define ADDR_Y11_Low			0x9c
#define ADDR_Y11_High			0x9d
#define ADDR_Y12_Low			0x9e
#define ADDR_Y12_High			0x9f
#define ADDR_Y13_Low			0xa0
#define ADDR_Y13_High			0xa1
#define ADDR_Y21_Low			0xa2
#define ADDR_Y21_High			0xa3
#define ADDR_Y22_Low			0xa4
#define ADDR_Y22_High			0xa5
#define ADDR_Y23_Low			0xa6
#define ADDR_Y23_High			0xa7
#define ADDR_Y31_Low			0xa8
#define ADDR_Y31_High			0xa9
#define ADDR_Y32_Low			0xaa
#define ADDR_Y32_High			0xab
#define ADDR_Y33_Low			0xac
#define ADDR_Y33_High			0xad

#define ADDR_TEMPL              0x1c
#define ADDR_TEMPH              0x1d

class IST8310 : public device::I2C, public I2CSPIDriver<IST8310>
{
public:
	IST8310(I2CSPIBusOption bus_option, int bus_number, int address, enum Rotation rotation, int bus_frequency);
	virtual ~IST8310();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	virtual int     init();

	virtual ssize_t     read(struct file *filp, char *buffer, size_t buflen);
	virtual int         ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Initialise the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void		start();

	/**
	 * Reset the device
	 */
	int         reset();

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
	void            RunImpl();

protected:
	int probe() override;

	void            print_status() override;

private:

	unsigned        _measure_interval{IST8310_CONVERSION_INTERVAL};

	ringbuffer::RingBuffer  *_reports{nullptr};

	struct mag_calibration_s	_scale {};
	float				_range_scale{0.003f}; /* default range scale from counts to gauss */

	bool        _collect_phase{false};
	int         _class_instance{-1};
	int         _orb_class_instance{-1};

	orb_advert_t        _mag_topic{nullptr};

	perf_counter_t      _sample_perf;
	perf_counter_t      _comms_errors;
	perf_counter_t      _range_errors;
	perf_counter_t      _conf_errors;

	enum Rotation       _rotation;

	sensor_mag_s   _last_report{};           /**< used for info() */

	uint8_t 		_ctl3_reg{0};
	uint8_t			_ctl4_reg{0};

	/**
	 * check the sensor configuration.
	 *
	 * checks that the config of the sensor is correctly set, to
	 * cope with communication errors causing the configuration to
	 * change
	 */
	void            check_conf(void);

	/**
	 * Write a register.
	 *
	 * @param reg       The register to write.
	 * @param val       The value to write.
	 * @return      OK on write success.
	 */
	int         write_reg(uint8_t reg, uint8_t val);

	/**
	 * Write to a register block.
	 *
	 * @param address   The register address to write to.
	 * @param data      The buffer to write from.
	 * @param count     The number of bytes to write.
	 * @return      OK on write success.
	 */
	int     write(unsigned address, void *data, unsigned count);

	/**
	 * Read a register.
	 *
	 * @param reg       The register to read.
	 * @param val       The value read.
	 * @return      OK on read success.
	 */
	int         read_reg(uint8_t reg, uint8_t &val);

	/**
	 * read register block.
	 *
	 * @param address   The register address to read from.
	 * @param data      The buffer to read into.
	 * @param count     The number of bytes to read.
	 * @return      OK on write success.
	 */
	int read(unsigned address, void *data, unsigned count);

	/**
	 * Issue a measurement command.
	 *
	 * @return      OK if the measurement command was successful.
	 */
	int         measure();

	/**
	 * Collect the result of the most recent measurement.
	 */
	int         collect();

	/**
	 * Convert a big-endian signed 16-bit value to a float.
	 *
	 * @param in        A signed 16-bit big-endian value.
	 * @return      The floating-point representation of the value.
	 */
	float       meas_to_float(uint8_t in[2]);

	/**
	* Place the device in self test mode
	*
	* @return 0 if mode is set, 1 else
	*/
	int         set_selftest(unsigned enable);

	/* this class has pointer data members, do not allow copying it */
	IST8310(const IST8310 &);
	IST8310 operator=(const IST8310 &);
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int ist8310_main(int argc, char *argv[]);


IST8310::IST8310(I2CSPIBusOption bus_option, int bus_number, int address, enum Rotation rotation, int bus_frequency) :
	I2C("IST8310", nullptr, bus_number, address, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus_number),
	_sample_perf(perf_alloc(PC_ELAPSED, "ist8310_read")),
	_comms_errors(perf_alloc(PC_COUNT, "ist8310_com_err")),
	_range_errors(perf_alloc(PC_COUNT, "ist8310_rng_err")),
	_conf_errors(perf_alloc(PC_COUNT, "ist8310_conf_err")),
	_rotation(rotation)
{
	_device_id.devid_s.devtype = DRV_MAG_DEVTYPE_IST8310;

	// default scaling
	_scale.x_offset = 0;
	_scale.x_scale = 1.0f;
	_scale.y_offset = 0;
	_scale.y_scale = 1.0f;
	_scale.z_offset = 0;
	_scale.z_scale = 1.0f;
}

IST8310::~IST8310()
{
	delete _reports;

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
IST8310::init()
{
	int ret = PX4_ERROR;

	ret = I2C::init();

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
out:
	return ret;
}

int
IST8310::write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address;
	memcpy(&buf[1], data, count);

	return transfer(&buf[0], count + 1, nullptr, 0);
}

int
IST8310::read(unsigned address, void *data, unsigned count)
{
	uint8_t cmd = address;
	return transfer(&cmd, 1, (uint8_t *)data, count);
}


/**
   check that the configuration register has the right value. This is
   done periodically to cope with I2C bus noise causing the
   configuration of the compass to change.
 */
void IST8310::check_conf(void)
{
	int ret;

	uint8_t ctrl_reg_in = 0;
	ret = read_reg(ADDR_CTRL3, ctrl_reg_in);

	if (OK != ret) {
		perf_count(_comms_errors);
		return;
	}

	if (ctrl_reg_in != _ctl3_reg) {
		perf_count(_conf_errors);
		ret = write_reg(ADDR_CTRL3, _ctl3_reg);

		if (OK != ret) {
			perf_count(_comms_errors);
		}
	}

	ret = read_reg(ADDR_CTRL4, ctrl_reg_in);

	if (OK != ret) {
		perf_count(_comms_errors);
		return;
	}

	if (ctrl_reg_in != _ctl4_reg) {
		perf_count(_conf_errors);
		ret = write_reg(ADDR_CTRL4, _ctl4_reg);

		if (OK != ret) {
			perf_count(_comms_errors);
		}
	}
}

ssize_t
IST8310::read(struct file *filp, char *buffer, size_t buflen)
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
		usleep(IST8310_CONVERSION_INTERVAL);

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
IST8310::ioctl(struct file *filp, int cmd, unsigned long arg)
{
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
					_measure_interval = IST8310_CONVERSION_INTERVAL;

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
					if (interval < IST8310_CONVERSION_INTERVAL) {
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

	case MAGIOCEXSTRAP:
		return set_selftest(arg);

	case MAGIOCSSCALE:
		/* set new scale factors */
		memcpy(&_scale, (struct mag_calibration_s *)arg, sizeof(_scale));
		return 0;

	case MAGIOCGSCALE:
		/* copy out scale factors */
		memcpy((struct mag_calibration_s *)arg, &_scale, sizeof(_scale));
		return 0;

	case MAGIOCGEXTERNAL:
		return external();

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}

void
IST8310::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	/* schedule a cycle to start things */
	ScheduleNow();
}

int
IST8310::reset()
{
	/* software reset */
	write_reg(ADDR_CTRL2, CTRL2_SRST);

	/* configure control register 3 */
	_ctl3_reg = CTRL3_SAMPLEAVG_16;
	write_reg(ADDR_CTRL3, _ctl3_reg);

	/* configure control register 4 */
	_ctl4_reg = CTRL4_SRPD;
	write_reg(ADDR_CTRL4, _ctl4_reg);

	return OK;
}

int
IST8310::probe()
{
	uint8_t data[1] = {0};

	_retries = 10;

	if (read(ADDR_WAI, &data[0], 1)) {
		DEVICE_DEBUG("read_reg fail");
		return -EIO;
	}

	_retries = 2;

	if ((data[0] != WAI_EXPECTED_VALUE)) {
		DEVICE_DEBUG("ID byte mismatch (%02x) expected %02x", data[0], WAI_EXPECTED_VALUE);
		return -EIO;
	}

	return OK;
}

void
IST8310::RunImpl()
{
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
		if (_measure_interval > IST8310_CONVERSION_INTERVAL) {

			/* schedule a fresh cycle call when we are ready to measure again */
			ScheduleDelayed(_measure_interval - IST8310_CONVERSION_INTERVAL);

			return;
		}
	}

	/* measurement phase */
	if (OK != measure()) {
		DEVICE_DEBUG("measure error");
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(IST8310_CONVERSION_INTERVAL);
}

int
IST8310::measure()
{
	/*
	 * Send the command to begin a measurement.
	 */
	int ret = write_reg(ADDR_CTRL1, CTRL1_MODE_SINGLE);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	return ret;
}

int
IST8310::collect()
{
#pragma pack(push, 1)
	struct { /* status register and data as read back from the device */
		uint8_t     x[2];
		uint8_t     y[2];
		uint8_t     z[2];
	} report_buffer;
#pragma pack(pop)
	struct {
		int16_t     x, y, z;
	} report;

	int ret;
	uint8_t check_counter;

	perf_begin(_sample_perf);
	sensor_mag_s new_report;
	const bool sensor_is_external = external();

	float xraw_f;
	float yraw_f;
	float zraw_f;

	/* this should be fairly close to the end of the measurement, so the best approximation of the time */
	new_report.timestamp = hrt_absolute_time();
	new_report.is_external = sensor_is_external;
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
	ret = read(ADDR_DATA_OUT_X_LSB, (uint8_t *)&report_buffer, sizeof(report_buffer));

	if (ret != OK) {
		perf_count(_comms_errors);
		DEVICE_DEBUG("I2C read error");
		goto out;
	}

	/* swap the data we just received */
	report.x = (((int16_t)report_buffer.x[1]) << 8) | (int16_t)report_buffer.x[0];
	report.y = (((int16_t)report_buffer.y[1]) << 8) | (int16_t)report_buffer.y[0];
	report.z = (((int16_t)report_buffer.z[1]) << 8) | (int16_t)report_buffer.z[0];


	/*
	 * Check if value makes sense according to the FSR and Resolution of
	 * this sensor, discarding outliers
	 */
	if (report.x > IST8310_MAX_VAL_XY || report.x < IST8310_MIN_VAL_XY ||
	    report.y > IST8310_MAX_VAL_XY || report.y < IST8310_MIN_VAL_XY ||
	    report.z > IST8310_MAX_VAL_Z  || report.z < IST8310_MIN_VAL_Z) {
		perf_count(_range_errors);
		DEVICE_DEBUG("data/status read error");
		goto out;
	}

	/* temperature measurement is not available on IST8310 */
	new_report.temperature = 0;

	/*
	 * raw outputs
	 *
	 * Sensor doesn't follow right hand rule, swap x and y to make it obey
	 * it.
	 */
	new_report.x_raw = report.y;
	new_report.y_raw = report.x;
	new_report.z_raw = report.z;

	/* scale values for output */
	xraw_f = report.y;
	yraw_f = report.x;
	zraw_f = report.z;

	/* apply user specified rotation */
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);
	new_report.x = ((xraw_f * _range_scale) - _scale.x_offset) * _scale.x_scale;
	new_report.y = ((yraw_f * _range_scale) - _scale.y_offset) * _scale.y_scale;
	new_report.z = ((zraw_f * _range_scale) - _scale.z_offset) * _scale.z_scale;

	if (!(_pub_blocked)) {

		if (_mag_topic != nullptr) {
			/* publish it */
			orb_publish(ORB_ID(sensor_mag), _mag_topic, &new_report);

		} else {
			_mag_topic = orb_advertise_multi(ORB_ID(sensor_mag), &new_report,
							 &_orb_class_instance, sensor_is_external ? ORB_PRIO_MAX : ORB_PRIO_HIGH);

			if (_mag_topic == nullptr) {
				DEVICE_DEBUG("ADVERT FAIL");
			}
		}
	}

	_last_report = new_report;

	/* post a report to the ring */
	_reports->force(&new_report);

	/*
	  periodically check the range register and configuration
	  registers. With a bad I2C cable it is possible for the
	  registers to become corrupt, leading to bad readings. It
	  doesn't happen often, but given the poor cables some
	  vehicles have it is worth checking for.
	 */
	check_counter = perf_event_count(_sample_perf) % 256;

	if (check_counter == 128) {
		check_conf();
	}

	ret = OK;

out:
	perf_end(_sample_perf);
	return ret;
}

int
IST8310::set_selftest(unsigned enable)
{
	int ret;
	uint8_t str;
	/* arm the excitement strap */
	ret = read_reg(ADDR_STR, str);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	str &= ~STR_SELF_TEST_ON; // reset previous test

	if (enable > 0) {
		str |= STR_SELF_TEST_ON;

	}

	ret = write_reg(ADDR_STR, str);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	uint8_t str_reg_ret = 0;
	read_reg(ADDR_STR, str_reg_ret);

	return !(str == str_reg_ret);
}

int
IST8310::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t buf = val;
	return write(reg, &buf, 1);
}

int
IST8310::read_reg(uint8_t reg, uint8_t &val)
{
	uint8_t buf = val;
	int ret = read(reg, &buf, 1);
	val = buf;
	return ret;
}

float
IST8310::meas_to_float(uint8_t in[2])
{
	union {
		uint8_t b[2];
		int16_t w;
	} u;

	u.b[0] = in[1];
	u.b[1] = in[0];

	return (float) u.w;
}

void
IST8310::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u interval\n", _measure_interval);
	print_message(_last_report);
	_reports->print_info("report queue");
}

I2CSPIDriverBase *
IST8310::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator, int runtime_instance)
{
	IST8310 *interface = new IST8310(iterator.configuredBusOption(), iterator.bus(), cli.i2c_address, cli.rotation,
						 cli.bus_frequency);

	if (interface == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (interface->init() != OK) {
		delete interface;
		PX4_DEBUG("no device on bus %i (devid 0x%x)", iterator.bus(), iterator.devid());
		return nullptr;
	}

	interface->start();

	return interface;
}

void
IST8310::print_usage()
{
	PRINT_MODULE_USAGE_NAME("ist8310", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("magnetometer");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0xE);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, 35, "Rotation", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

int
ist8310_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = IST8310;
	BusCLIArguments cli{true, false};
	cli.i2c_address = IST8310_BUS_I2C_ADDR;
	cli.default_i2c_frequency = IST8310_DEFAULT_BUS_SPEED;

	while ((ch = cli.getopt(argc, argv, "R:")) != EOF) {
		switch (ch) {
		case 'R':
			cli.rotation = (enum Rotation)atoi(cli.optarg());
			break;
		}
	}

	const char *verb = cli.optarg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_MAG_DEVTYPE_IST8310);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return 1;
}
