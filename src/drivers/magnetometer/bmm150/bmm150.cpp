/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file bmm150.cpp
 * Driver for the Bosch BMM 150 MEMS magnetometer connected via I2C.
 */

#include "bmm150.hpp"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

extern "C" __EXPORT int bmm150_main(int argc, char *argv[]);

BMM150::BMM150(I2CSPIBusOption bus_option, const int bus, int bus_frequency, enum Rotation rotation) :
	I2C("BMM150", nullptr, bus, BMM150_SLAVE_ADDRESS, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_call_interval(0),
	_reports(nullptr),
	_collect_phase(false),
	_scale{},
	_range_scale(0.01), /* default range scale from from uT to gauss */
	_topic(nullptr),
	_orb_class_instance(-1),
	_class_instance(-1),
	_power(BMM150_DEFAULT_POWER_MODE),
	_output_data_rate(BMM150_DATA_RATE_30HZ),
	_calibrated(false),
	dig_x1(0),
	dig_y1(0),
	dig_x2(0),
	dig_y2(0),
	dig_z1(0),
	dig_z2(0),
	dig_z3(0),
	dig_z4(0),
	dig_xy1(0),
	dig_xy2(0),
	dig_xyz1(0),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_bad_transfers(perf_alloc(PC_COUNT, MODULE_NAME": bad transfers")),
	_good_transfers(perf_alloc(PC_COUNT, MODULE_NAME": good transfers")),
	_measure_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": measure")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms errors")),
	_duplicates(perf_alloc(PC_COUNT, MODULE_NAME": duplicates")),
	_rotation(rotation),
	_got_duplicate(false)
{
	_device_id.devid_s.devtype = DRV_MAG_DEVTYPE_BMM150;

	// default scaling
	_scale.x_offset = 0;
	_scale.x_scale = 1.0f;
	_scale.y_offset = 0;
	_scale.y_scale = 1.0f;
	_scale.z_offset = 0;
	_scale.z_scale = 1.0f;
}

BMM150::~BMM150()
{
	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}


	if (_class_instance != -1) {
		unregister_class_devname(MAG_BASE_DEVICE_PATH, _class_instance);
	}

	if (_topic != nullptr) {
		orb_unadvertise(_topic);
	}


	/* delete the perf counter */
	perf_free(_sample_perf);
	perf_free(_bad_transfers);
	perf_free(_good_transfers);
	perf_free(_measure_perf);
	perf_free(_comms_errors);
	perf_free(_duplicates);
}

int BMM150::init()
{
	int ret = OK;

	/* do I2C init (and probe) first */
	ret = I2C::init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("I2C setup failed");
		return ret;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(sensor_mag_s));

	if (_reports == nullptr) {
		goto out;
	}

	/* Bring the device to sleep mode */
	modify_reg(BMM150_POWER_CTRL_REG, 1, 1);
	up_udelay(10000);


	/* check id*/
	if (read_reg(BMM150_CHIP_ID_REG) != BMM150_CHIP_ID) {
		DEVICE_DEBUG("id of magnetometer is not: 0x%02x", BMM150_CHIP_ID);
		return -EIO;
	}

	if (reset() != OK) {
		goto out;
	}

	init_trim_registers();

	_class_instance = register_class_devname(MAG_BASE_DEVICE_PATH);

	if (measure()) {
		return -EIO;
	}

	up_udelay(10000);

	if (collect()) {
		return -EIO;
	}

	/* advertise sensor topic, measure manually to initialize valid report */
	sensor_mag_s mrb;
	_reports->get(&mrb);

	/* measurement will have generated a report, publish */
	_topic = orb_advertise_multi(ORB_ID(sensor_mag), &mrb,
				     &_orb_class_instance, (external()) ? ORB_PRIO_MAX : ORB_PRIO_HIGH);

	if (_topic == nullptr) {
		PX4_ERR("ADVERT FAIL");
		return -ENOMEM;
	}

	_call_interval = 1000000 / BMM150_MAX_DATA_RATE;
	start();

out:
	return ret;
}

int
BMM150::probe()
{
	/* During I2C Initialization, sensor is in suspend mode
	 * and chip Id will return 0x00, hence returning OK. After
	 * I2C initialization, sensor is brought to sleep mode
	 * and Chip Id is verified. In sleep mode, we can read
	 * chip id. */

	/* @Note: Please read BMM150 Datasheet for more details */
	return OK;
}


void
BMM150::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	/* schedule a cycle to start things */
	ScheduleNow();
}

ssize_t
BMM150::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(sensor_mag_s);
	sensor_mag_s *mag_buf = reinterpret_cast<sensor_mag_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_call_interval > 0) {
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
		usleep(BMM150_CONVERSION_INTERVAL);

		/* run the collection phase */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}


		if (_reports->get(mag_buf)) {
			ret = sizeof(sensor_mag_s);
		}
	} while (0);

	/* return the number of bytes transferred */
	return ret;

}

void
BMM150::RunImpl()
{
	if (_collect_phase) {
		collect();
		unsigned wait_gap = _call_interval - BMM150_CONVERSION_INTERVAL;

		if (wait_gap != 0) {
			// need to wait some time before new measurement
			ScheduleDelayed(wait_gap);

			return;
		}
	}

	measure();

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(BMM150_CONVERSION_INTERVAL);
}

int
BMM150::measure()
{
	_collect_phase = true;

	perf_begin(_measure_perf);

	/* start measure */
	int ret = set_power_mode(BMM150_FORCED_MODE);

	if (ret != OK) {
		perf_count(_comms_errors);
		perf_cancel(_measure_perf);
		return -EIO;
	}

	perf_end(_measure_perf);

	return OK;
}



int
BMM150::collect()
{
	_collect_phase = false;

	bool mag_notify = true;
	uint8_t mag_data[8], status;
	uint16_t resistance, lsb, msb, msblsb;
	sensor_mag_s mrb{};


	/* start collecting data */
	perf_begin(_sample_perf);

	status = read_reg(BMM150_R_LSB);

	/* Read Magnetometer data*/
	if (OK != get_data(BMM150_DATA_X_LSB_REG, mag_data, sizeof(mag_data))) {
		return -EIO;
	}

	/* Array holding the mag XYZ and R data
	mag_data[0] - X LSB
	mag_data[1] - X MSB
	mag_data[2] - Y LSB
	mag_data[3] - Y MSB
	mag_data[4] - Z LSB
	mag_data[5] - Z MSB
	mag_data[6] - R LSB
	mag_data[7] - R MSB
	*/

	/* Extract X axis data */
	lsb = ((mag_data[0] & 0xF8) >> 3);
	msb = (((int8_t)mag_data[1]) << 5);
	msblsb = (msb | lsb);
	mrb.x_raw = (int16_t)msblsb;


	/* Extract Y axis data */
	lsb = ((mag_data[2] & 0xF8) >> 3);
	msb = (((int8_t)mag_data[3]) << 5);
	msblsb = (msb | lsb);
	mrb.y_raw = (int16_t)msblsb;

	/* Extract Z axis data */
	lsb = ((mag_data[4] & 0xFE) >> 1);
	msb = (((int8_t)mag_data[5]) << 7);
	msblsb = (msb | lsb);
	mrb.z_raw = (int16_t)msblsb;

	/* Extract Resistance data */
	lsb = ((mag_data[6] & 0xFC) >> 2);
	msb = (mag_data[7] << 6);
	msblsb = (msb | lsb);
	resistance = (uint16_t)msblsb;

	/* Check whether data is new or old */
	if (!(status & 0x01)) {
		perf_end(_sample_perf);
		perf_count(_duplicates);
		_got_duplicate = true;
		return -EIO;
	}

	_got_duplicate = false;

	if (mrb.x_raw == 0 &&
	    mrb.y_raw == 0 &&
	    mrb.z_raw == 0 &&
	    resistance == 0) {
		// all zero data - probably a I2C bus error
		perf_count(_comms_errors);
		perf_count(_bad_transfers);
		perf_end(_sample_perf);
		return -EIO;
	}

	perf_count(_good_transfers);

	/* Compensation for X axis */
	if (mrb.x_raw != BMM150_FLIP_OVERFLOW_ADCVAL) {
		/* no overflow */
		if ((resistance != 0) && (dig_xyz1 != 0)) {
			mrb.x = ((dig_xyz1 * 16384.0 / resistance) - 16384.0);
			mrb.x = (((mrb.x_raw * ((((dig_xy2 * ((float)(mrb.x * mrb.x / (float)268435456.0)) + mrb.x * dig_xy1 /
						   (float)16384.0)) + (float)256.0) * (dig_x2 + (float)160.0))) / (float)8192.0) + (dig_x1 * (float)8.0)) / (float)16.0;

		} else {
			mrb.x = BMM150_OVERFLOW_OUTPUT_FLOAT;
		}

	} else {
		mrb.x = BMM150_OVERFLOW_OUTPUT_FLOAT;
	}

	/* Compensation for Y axis */
	if (mrb.y_raw != BMM150_FLIP_OVERFLOW_ADCVAL) {
		/* no overflow */
		if ((resistance != 0) && (dig_xyz1 != 0)) {

			mrb.y = ((((float)dig_xyz1) * (float)16384.0 / resistance) - (float)16384.0);
			mrb.y = (((mrb.y_raw * ((((dig_xy2 * (mrb.y * mrb.y / (float)268435456.0) + mrb.y * dig_xy1 / (float)16384.0)) +
						 (float)256.0) * (dig_y2 + (float)160.0))) / (float)8192.0) + (dig_y1 * (float)8.0)) / (float)16.0;


		} else {
			mrb.y = BMM150_OVERFLOW_OUTPUT_FLOAT;
		}

	} else {
		/* overflow, set output to 0.0f */
		mrb.y = BMM150_OVERFLOW_OUTPUT_FLOAT;
	}


	/* Compensation for Z axis */
	if (mrb.z_raw != BMM150_HALL_OVERFLOW_ADCVAL) {
		/* no overflow */
		if ((dig_z2 != 0) && (dig_z1 != 0) && (dig_xyz1 != 0) && (resistance != 0)) {
			mrb.z = ((((mrb.z_raw - dig_z4) * (float)131072.0) - (dig_z3 * (resistance - dig_xyz1))) / ((
						dig_z2 + dig_z1 * resistance / (float)32768.0) * (float)4.0)) / (float)16.0;
		}

	} else {
		/* overflow, set output to 0.0f */
		mrb.z = BMM150_OVERFLOW_OUTPUT_FLOAT;
	}


	mrb.timestamp = hrt_absolute_time();
	mrb.is_external = external();

	// report the error count as the number of bad transfers.
	// This allows the higher level code to decide if it
	// should use this sensor based on whether it has had failures
	mrb.error_count = perf_event_count(_bad_transfers);

	// apply user specified rotation
	rotate_3f(_rotation, mrb.x, mrb.y, mrb.z);


	/* Scaling the data */
	mrb.x = ((mrb.x * _range_scale) - _scale.x_offset) * _scale.x_scale;
	mrb.y = ((mrb.y * _range_scale) - _scale.y_offset) * _scale.y_scale;
	mrb.z = ((mrb.z * _range_scale) - _scale.z_offset) * _scale.z_scale;

	mrb.scaling = _range_scale;
	mrb.device_id = _device_id.devid;

	_last_report.x = mrb.x;
	_last_report.y = mrb.y;
	_last_report.z = mrb.z;

	_reports->force(&mrb);

	/* notify anyone waiting for data */
	if (mag_notify) {
		poll_notify(POLLIN);
	}

	if (mag_notify && !(_pub_blocked)) {
		/* publish it */
		orb_publish(ORB_ID(sensor_mag), _topic, &mrb);
	}

	perf_end(_sample_perf);
	return OK;

}

int
BMM150::ioctl(struct file *filp, int cmd, unsigned long arg)
{

	switch (cmd) {

	case MAGIOCSSCALE:
		return OK;

	case MAGIOCGSCALE:
		return OK;

	case MAGIOCCALIBRATE:
		return OK;

	case MAGIOCEXSTRAP:
		return OK;

	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}


int BMM150::reset()
{
	int ret = OK;

	/* Soft-reset */
	modify_reg(BMM150_POWER_CTRL_REG, BMM150_SOFT_RESET_MASK, BMM150_SOFT_RESET_VALUE);
	up_udelay(5000);

	/* Enable Magnetometer in normal mode */
	ret += set_power_mode(BMM150_DEFAULT_POWER_MODE);
	up_udelay(1000);

	/* Set the data rate to default */
	ret += set_data_rate(BMM150_DEFAULT_ODR);

	/* set the preset mode as regular*/
	ret += set_presetmode(BMM150_PRESETMODE_REGULAR);

	return OK;
}


int
BMM150::self_test()
{
	if (perf_event_count(_sample_perf) == 0) {
		collect();
	}

	/* return 0 on success, 1 else */
	return (perf_event_count(_sample_perf) > 0) ? 0 : 1;
}

uint8_t
BMM150::read_reg(uint8_t reg)
{
	const uint8_t cmd = reg;
	uint8_t ret;

	transfer(&cmd, 1, &ret, 1);

	return ret;
}

int
BMM150::write_reg(uint8_t reg, uint8_t value)
{
	const uint8_t cmd[2] = { reg, value};

	return transfer(cmd, 2, nullptr, 0);
}

int
BMM150::get_data(uint8_t reg, uint8_t *data, unsigned len)
{
	const uint8_t cmd = reg;

	return transfer(&cmd, 1, data, len);

}

void
BMM150::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t val;

	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_reg(reg, val);
}



int BMM150::set_power_mode(uint8_t power_mode)
{
	uint8_t setbits = 0;
	uint8_t clearbits = BMM150_POWER_MASK;

	switch (power_mode) {
	case BMM150_NORMAL_MODE:
		_power = 0;
		break;

	case BMM150_FORCED_MODE:
		_power = 1;
		break;

	case BMM150_SLEEP_MODE:
		_power = 3;
		break;

	default:
		return -EINVAL;
	}

	setbits |= power_mode;
	modify_reg(BMM150_CTRL_REG, clearbits, setbits);

	return OK;

}

int BMM150::set_data_rate(uint8_t data_rate)
{
	uint8_t setbits = 0;
	uint8_t clearbits = BMM150_OUTPUT_DATA_RATE_MASK;

	switch (data_rate) {
	case BMM150_DATA_RATE_10HZ:
		_output_data_rate = 10;
		break;

	case BMM150_DATA_RATE_02HZ:
		_output_data_rate = 2;
		break;

	case BMM150_DATA_RATE_06HZ:
		_output_data_rate = 6;
		break;

	case BMM150_DATA_RATE_08HZ:
		_output_data_rate = 8;
		break;

	case BMM150_DATA_RATE_15HZ:
		_output_data_rate = 15;
		break;

	case BMM150_DATA_RATE_20HZ:
		_output_data_rate = 20;
		break;

	case BMM150_DATA_RATE_25HZ:
		_output_data_rate = 25;
		break;

	case BMM150_DATA_RATE_30HZ:
		_output_data_rate = 30;
		break;

	default:
		return -EINVAL;
	}

	setbits |= data_rate;
	modify_reg(BMM150_CTRL_REG, clearbits, setbits);

	return OK;

}


int BMM150::init_trim_registers(void)
{
	int ret = OK;
	uint8_t data[2] = {0};
	uint16_t msb, lsb, msblsb;

	dig_x1 = read_reg(BMM150_DIG_X1);
	dig_y1 = read_reg(BMM150_DIG_Y1);
	dig_x2 = read_reg(BMM150_DIG_X2);
	dig_y2 = read_reg(BMM150_DIG_Y2);
	dig_xy1 = read_reg(BMM150_DIG_XY1);
	dig_xy2 = read_reg(BMM150_DIG_XY2);

	ret += get_data(BMM150_DIG_Z1_LSB, data, 2);
	lsb = data[0];
	msb = (data[1] << 8);
	msblsb = (msb | lsb);
	dig_z1 = (uint16_t)msblsb;

	ret += get_data(BMM150_DIG_Z2_LSB, data, 2);
	lsb = data[0];
	msb = ((int8_t)data[1] << 8);
	msblsb = (msb | lsb);
	dig_z2 = (int16_t)msblsb;

	ret += get_data(BMM150_DIG_Z3_LSB, data, 2);
	lsb = data[0];
	msb = ((int8_t)data[1] << 8);
	msblsb = (msb | lsb);
	dig_z3 = (int16_t)msblsb;

	ret += get_data(BMM150_DIG_Z4_LSB, data, 2);
	lsb = data[0];
	msb = ((int8_t)data[1] << 8);
	msblsb = (msb | lsb);
	dig_z4 = (int16_t)msblsb;

	ret += get_data(BMM150_DIG_XYZ1_LSB, data, 2);
	lsb = data[0];
	msb = ((data[1] & 0x7F) << 8);
	msblsb = (msb | lsb);
	dig_xyz1 = (uint16_t)msblsb;

	return ret;

}

int BMM150::set_rep_xy(uint8_t rep_xy)
{
	uint8_t cmd[2] = {BMM150_XY_REP_CTRL_REG, rep_xy};

	return transfer((const uint8_t *)cmd, sizeof(cmd), nullptr, 0);

}

int BMM150::set_rep_z(uint8_t rep_z)
{
	uint8_t cmd[2] = {BMM150_Z_REP_CTRL_REG, rep_z};

	return transfer((const uint8_t *)cmd, sizeof(cmd), nullptr, 0);

}


int BMM150::set_presetmode(uint8_t presetmode)
{
	int ret = OK;
	uint8_t data_rate, rep_xy, rep_z;

	if (presetmode == 1) {
		data_rate = BMM150_LOWPOWER_DR;
		rep_xy = BMM150_LOWPOWER_REPXY;
		rep_z = BMM150_LOWPOWER_REPZ;

	} else if (presetmode == 2) {
		data_rate = BMM150_REGULAR_DR;
		rep_xy = BMM150_REGULAR_REPXY;
		rep_z = BMM150_REGULAR_REPZ;

	} else if (presetmode == 3) {
		data_rate = BMM150_HIGHACCURACY_DR;
		rep_xy = BMM150_HIGHACCURACY_REPXY;
		rep_z = BMM150_HIGHACCURACY_REPZ;

	} else if (presetmode == 4) {
		data_rate = BMM150_ENHANCED_DR;
		rep_xy = BMM150_ENHANCED_REPXY;
		rep_z = BMM150_ENHANCED_REPZ;

	} else {
		return -EINVAL;
	}

	ret =  set_data_rate(data_rate);

	ret += set_rep_xy(rep_xy);
	ret += set_rep_z(rep_z);

	return ret;

}

void
BMM150::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_bad_transfers);
	perf_print_counter(_good_transfers);
	_reports->print_info("mag queue");

	printf("output  (%.2f %.2f %.2f)\n", (double)_last_report.x, (double)_last_report.y, (double)_last_report.z);
	printf("offsets (%.2f %.2f %.2f)\n", (double)_scale.x_offset, (double)_scale.y_offset, (double)_scale.z_offset);
	printf("scaling (%.2f %.2f %.2f) 1/range_scale %.2f ",
	       (double)_scale.x_scale, (double)_scale.y_scale, (double)_scale.z_scale,
	       (double)(1.0f / _range_scale));
	printf("\n");

}

void
BMM150::print_registers()
{
	printf("BMM150 registers\n");

	uint8_t reg = BMM150_CHIP_ID_REG;
	uint8_t v = read_reg(reg);
	printf("Chip Id: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = BMM150_INT_SETT_CTRL_REG;
	v = read_reg(reg);
	printf("Int sett Ctrl reg: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = BMM150_AXES_EN_CTRL_REG;
	v = read_reg(reg);
	printf("Axes En Ctrl reg: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

}

void
BMM150::print_usage()
{
	PRINT_MODULE_USAGE_NAME("bmm150", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("magnetometer");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, 35, "Rotation", true);
	PRINT_MODULE_USAGE_COMMAND("reset");
	PRINT_MODULE_USAGE_COMMAND("regdump");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

void
BMM150::custom_method(const BusCLIArguments &cli)
{
	switch (cli.custom1) {
	case 0: reset();
		break;

	case 1: print_registers();
		break;
	}
}

I2CSPIDriverBase *BMM150::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
				      int runtime_instance)
{
	BMM150 *interface = new BMM150(iterator.configuredBusOption(), iterator.bus(), cli.bus_frequency, cli.rotation);

	if (interface == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (interface->init() != OK) {
		delete interface;
		PX4_DEBUG("no device on bus %i (devid 0x%x)", iterator.bus(), iterator.devid());
		return nullptr;
	}

	return interface;
}

int
bmm150_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = BMM150;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = BMM150_BUS_SPEED;

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

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_MAG_DEVTYPE_BMM150);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	if (!strcmp(verb, "reset")) {
		cli.custom1 = 0;
		return ThisDriver::module_custom_method(cli, iterator);
	}

	if (!strcmp(verb, "regdump")) {
		cli.custom1 = 1;
		return ThisDriver::module_custom_method(cli, iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
