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
 * @file LSM303agr.cpp
 * Driver for the ST LSM303AGR MEMS accelerometer / magnetometer connected via SPI.
 * NOTE: currently only the mag is implemented
 */

#include "LSM303AGR.hpp"

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <ecl/geo/geo.h>

/* SPI protocol address bits */
#define DIR_READ				(1<<7)
#define DIR_WRITE				(0<<7)
#define ADDR_INCREMENT			(1<<6)

/* Max measurement rate is 100Hz */
#define CONVERSION_INTERVAL	(1000000 / 100)	/* microseconds */

static constexpr uint8_t LSM303AGR_WHO_AM_I_M = 0x40;

/*
  we set the timer interrupt to run a bit faster than the desired
  sample rate and then throw away duplicates using the data ready bit.
  This time reduction is enough to cope with worst case timing jitter
  due to other timers
 */
#define LSM303AGR_TIMER_REDUCTION				200

LSM303AGR::LSM303AGR(int bus, const char *path, uint32_t device, enum Rotation rotation) :
	SPI("LSM303AGR", path, bus, device, SPIDEV_MODE3, 8 * 1000 * 1000),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(get_device_id())),
	_mag_sample_perf(perf_alloc(PC_ELAPSED, "LSM303AGR_mag_read")),
	_bad_registers(perf_alloc(PC_COUNT, "LSM303AGR_bad_reg")),
	_bad_values(perf_alloc(PC_COUNT, "LSM303AGR_bad_val")),
	_rotation(rotation)
{
	/* Prime _mag with parents devid. */
	_device_id.devid = _device_id.devid;
	_device_id.devid_s.devtype = DRV_MAG_DEVTYPE_LSM303AGR;

	_mag_scale.x_offset = 0.0f;
	_mag_scale.x_scale = 1.0f;
	_mag_scale.y_offset = 0.0f;
	_mag_scale.y_scale = 1.0f;
	_mag_scale.z_offset = 0.0f;
	_mag_scale.z_scale = 1.0f;
}

LSM303AGR::~LSM303AGR()
{
	/* make sure we are truly inactive */
	stop();

	if (_class_instance != -1) {
		unregister_class_devname(MAG_BASE_DEVICE_PATH, _class_instance);
	}

	/* delete the perf counter */
	perf_free(_mag_sample_perf);
	perf_free(_bad_registers);
	perf_free(_bad_values);
}

int
LSM303AGR::init()
{
	int ret = PX4_OK;

	/* do SPI init (and probe) first */
	if (SPI::init() != OK) {
		PX4_ERR("SPI init failed");
		return PX4_ERROR;
	}

	_class_instance = register_class_devname(MAG_BASE_DEVICE_PATH);

	reset();

	self_test();

	reset();

	/* fill report structures */
	measure();

	return ret;
}

int
LSM303AGR::reset()
{
	// Single mode
	// Output data rate configuration: 100Hz
	write_reg(CFG_REG_A_M, CFG_REG_A_M_MD0 | CFG_REG_A_M_ODR1 | CFG_REG_A_M_ODR0);

	// Enable low pass filter
	write_reg(CFG_REG_B_M, CFG_REG_B_M_OFF_CANC | CFG_REG_B_M_OFF_LPF);

	// Disable I2C
	write_reg(CFG_REG_C_M, CFG_REG_C_M_I2C_DIS);

	return PX4_OK;
}

bool
LSM303AGR::self_test()
{
	// Magnetometer self-test procedure (LSM303AGR DocID027765 Rev 5 page 25/68)

	uint8_t status_m = 0;

	write_reg(CFG_REG_A_M, 0x0C);
	write_reg(CFG_REG_B_M, 0x02);
	write_reg(CFG_REG_C_M, 0x10);

	// sleep 20ms
	usleep(20000);

	uint16_t OUTX_NOST = 0;
	uint16_t OUTY_NOST = 0;
	uint16_t OUTZ_NOST = 0;

	// Check Zyxda 50 times and discard
	// average x, y, z
	for (int i = 0; i < 50; i++) {

		status_m = read_reg(STATUS_REG_M);

		OUTX_NOST += read_reg(OUTX_L_REG_M) + (read_reg(OUTX_H_REG_M) << 8);
		OUTY_NOST += read_reg(OUTY_L_REG_M) + (read_reg(OUTY_H_REG_M) << 8);
		OUTZ_NOST += read_reg(OUTZ_L_REG_M) + (read_reg(OUTZ_H_REG_M) << 8);
	}

	// enable self-test
	write_reg(CFG_REG_C_M, 0x12);

	// wait for 60ms
	usleep(60000);

	// Check Zyxda
	status_m = read_reg(STATUS_REG_M);

	// Read mag x, y, z to clear Zyxda bit
	read_reg(OUTX_L_REG_M);
	read_reg(OUTX_H_REG_M);
	read_reg(OUTY_L_REG_M);
	read_reg(OUTY_H_REG_M);
	read_reg(OUTZ_L_REG_M);
	read_reg(OUTZ_H_REG_M);

	uint16_t OUTX_ST = 0;
	uint16_t OUTY_ST = 0;
	uint16_t OUTZ_ST = 0;

	// Read the output registers after checking the Zyxda bit 50 times
	// average x, y, z
	for (int i = 0; i < 50; i++) {

		status_m = read_reg(STATUS_REG_M);

		OUTX_NOST += read_reg(OUTX_L_REG_M) + (read_reg(OUTX_H_REG_M) << 8);
		OUTY_NOST += read_reg(OUTY_L_REG_M) + (read_reg(OUTY_H_REG_M) << 8);
		OUTZ_NOST += read_reg(OUTZ_L_REG_M) + (read_reg(OUTZ_H_REG_M) << 8);
	}

	const uint16_t abs_x = abs(OUTX_ST - OUTX_NOST);
	const uint16_t abs_y = abs(OUTY_ST - OUTY_NOST);
	const uint16_t abs_z = abs(OUTZ_ST - OUTZ_NOST);

	// TODO: proper ranges?
	const bool x_valid = (abs_x > 0 && abs_x < UINT16_MAX);
	const bool y_valid = (abs_y > 0 && abs_y < UINT16_MAX);
	const bool z_valid = (abs_z > 0 && abs_z < UINT16_MAX);

	if (!x_valid || !y_valid || !z_valid) {

		PX4_ERR("self-test failed");

		PX4_INFO("STATUS_M: %X", status_m);
		PX4_INFO("ABS(OUTX_ST - OUTX_NOST) = %d", abs_x);
		PX4_INFO("ABS(OUTY_ST - OUTY_NOST) = %d", abs_y);
		PX4_INFO("ABS(OUTZ_ST - OUTZ_NOST) = %d", abs_z);
	}

	// disable self test
	write_reg(CFG_REG_C_M, 0x10);

	// Idle mode
	write_reg(CFG_REG_A_M, 0x03);

	return true;
}

int
LSM303AGR::probe()
{
	/* verify that the device is attached and functioning */
	bool success = (read_reg(WHO_AM_I_M) == LSM303AGR_WHO_AM_I_M);

	if (success) {
		return OK;
	}

	return -EIO;
}

int
LSM303AGR::ioctl(struct file *filp, int cmd, unsigned long arg)
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
					_measure_interval = (CONVERSION_INTERVAL);

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
					if (interval < CONVERSION_INTERVAL) {
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

	case MAGIOCSSCALE:
		/* set new scale factors */
		memcpy(&_mag_scale, (struct mag_calibration_s *)arg, sizeof(_mag_scale));

		return 0;

	case MAGIOCGSCALE:
		/* copy out scale factors */
		memcpy((struct mag_calibration_s *)arg, &_mag_scale, sizeof(_mag_scale));
		return 0;

	case MAGIOCGEXTERNAL:
		return external();

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}

uint8_t
LSM303AGR::read_reg(unsigned reg)
{
	uint8_t cmd[2];

	cmd[0] = reg | DIR_READ;
	cmd[1] = 0;

	transfer(cmd, cmd, sizeof(cmd));

	return cmd[1];
}

void
LSM303AGR::write_reg(unsigned reg, uint8_t value)
{
	uint8_t	cmd[2];

	cmd[0] = reg | DIR_WRITE;
	cmd[1] = value;

	transfer(cmd, nullptr, sizeof(cmd));
}

void
LSM303AGR::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;

	/* schedule a cycle to start things */
	ScheduleNow();
}

void
LSM303AGR::stop()
{
	if (_measure_interval > 0) {
		/* ensure no new items are queued while we cancel this one */
		_measure_interval = 0;
		ScheduleClear();
	}
}

void
LSM303AGR::Run()
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
		if (_measure_interval > CONVERSION_INTERVAL) {

			/* schedule a fresh cycle call when we are ready to measure again */
			ScheduleDelayed(_measure_interval - CONVERSION_INTERVAL);

			return;
		}
	}

	/* measurement phase */
	measure();

	/* next phase is collection */
	_collect_phase = true;

	if (_measure_interval > 0) {
		/* schedule a fresh cycle call when the measurement is done */
		ScheduleDelayed(CONVERSION_INTERVAL);
	}
}

void
LSM303AGR::measure()
{
	/*
	 * Send the command to begin a measurement.
	 */
	write_reg(CFG_REG_A_M, CFG_REG_A_M_MD0 | CFG_REG_A_M_ODR1 | CFG_REG_A_M_ODR0);
}

int
LSM303AGR::collect()
{
	const uint8_t status = read_reg(STATUS_REG_M);

	// only publish new data
	if (status & STATUS_REG_M_Zyxda) {
		/* start the performance counter */
		perf_begin(_mag_sample_perf);

		sensor_mag_s mag_report = {};
		mag_report.timestamp = hrt_absolute_time();

		// switch to right hand coordinate system in place
		mag_report.x_raw = read_reg(OUTX_L_REG_M) + (read_reg(OUTX_H_REG_M) << 8);
		mag_report.y_raw = read_reg(OUTY_L_REG_M) + (read_reg(OUTY_H_REG_M) << 8);
		mag_report.z_raw = -(read_reg(OUTZ_L_REG_M) + (read_reg(OUTZ_H_REG_M) << 8));

		float xraw_f = mag_report.x_raw;
		float yraw_f = mag_report.y_raw;
		float zraw_f = mag_report.z_raw;

		/* apply user specified rotation */
		rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

		mag_report.x = ((xraw_f * _mag_range_scale) - _mag_scale.x_offset) * _mag_scale.x_scale;
		mag_report.y = ((yraw_f * _mag_range_scale) - _mag_scale.y_offset) * _mag_scale.y_scale;
		mag_report.z = ((zraw_f * _mag_range_scale) - _mag_scale.z_offset) * _mag_scale.z_scale;
		mag_report.scaling = _mag_range_scale;
		mag_report.error_count = perf_event_count(_bad_registers) + perf_event_count(_bad_values);

		/* remember the temperature. The datasheet isn't clear, but it
		 * seems to be a signed offset from 25 degrees C in units of 0.125C
		 */
		//mag_report.temperature = 25 + (raw_mag_report.temperature * 0.125f);;
		mag_report.device_id = _device_id.devid;
		mag_report.is_external = external();

		if (!(_pub_blocked)) {

			if (_mag_topic != nullptr) {
				/* publish it */
				orb_publish(ORB_ID(sensor_mag), _mag_topic, &mag_report);

			} else {
				_mag_topic = orb_advertise_multi(ORB_ID(sensor_mag), &mag_report, &_mag_orb_class_instance,
								 (mag_report.is_external) ? ORB_PRIO_HIGH : ORB_PRIO_MAX);

				if (_mag_topic == nullptr) {
					DEVICE_DEBUG("ADVERT FAIL");
				}
			}
		}


		/* stop the perf counter */
		perf_end(_mag_sample_perf);
	}

	return OK;
}

void
LSM303AGR::print_info()
{
	perf_print_counter(_mag_sample_perf);
	perf_print_counter(_bad_registers);
	perf_print_counter(_bad_values);
}
