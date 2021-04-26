/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file bmp388.cpp
 *
 * Driver for the BMP388 barometric pressure sensor connected via SPI or I2C
 *
 * Refer to: https://github.com/BoschSensortec/BMP3-Sensor-API
 */

#include "bmp388.h"

BMP388::BMP388(I2CSPIBusOption bus_option, int bus, IBMP388 *interface) :
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(interface->get_device_id()), bus_option, bus,
		     interface->get_device_address()),
	_px4_baro(interface->get_device_id()),
	_interface(interface),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_measure_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": measure")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms errors"))
{
}

BMP388::~BMP388()
{
	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_measure_perf);
	perf_free(_comms_errors);

	delete _interface;
}

int
BMP388::init()
{
	if (!soft_reset()) {
		PX4_DEBUG("failed to reset baro during init");
		return -EIO;
	}

	if (_interface->get_reg(BMP3_CHIP_ID_ADDR) != BMP3_CHIP_ID) {
		PX4_WARN("id of your baro is not: 0x%02x", BMP3_CHIP_ID);
		return -EIO;
	}

	_cal = _interface->get_calibration(BMP3_CALIB_DATA_ADDR);

	if (!_cal) {
		PX4_WARN("failed to get baro cal init");
		return -EIO;
	}

	if (!validate_trimming_param()) {
		PX4_WARN("failed to validate trim param");
		return -EIO;
	}

	if (!set_sensor_settings()) {
		PX4_WARN("failed to set sensor settings");
		return -EIO;
	}

	start();

	return OK;
}

void
BMP388::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_measure_perf);
	perf_print_counter(_comms_errors);
	printf("measurement interval:  %u us \n", _measure_interval);
}

void
BMP388::start()
{
	_collect_phase = false;

	// wait a bit longer for the first measurement, as otherwise the first readout might fail
	ScheduleOnInterval(_measure_interval, _measure_interval * 3);
}

void
BMP388::RunImpl()
{
	if (_collect_phase) {
		collect();
	}

	measure();
}

int
BMP388::measure()
{
	_collect_phase = true;

	perf_begin(_measure_perf);

	/* start measurement */
	if (!set_op_mode(BMP3_FORCED_MODE)) {
		PX4_DEBUG("failed to set operating mode");
		perf_count(_comms_errors);
		perf_cancel(_measure_perf);
		return -EIO;
	}

	perf_end(_measure_perf);

	return OK;
}

int
BMP388::collect()
{
	_collect_phase = false;

	/* enable pressure and temperature */
	uint8_t sensor_comp = BMP3_PRESS | BMP3_TEMP;
	bmp3_data data{};

	perf_begin(_sample_perf);

	/* this should be fairly close to the end of the conversion, so the best approximation of the time */
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	if (!get_sensor_data(sensor_comp, &data)) {
		perf_count(_comms_errors);
		perf_cancel(_sample_perf);
		return -EIO;
	}

	_px4_baro.set_error_count(perf_event_count(_comms_errors));

	float temperature = (float)(data.temperature / 100.0f);
	float pressure = (float)(data.pressure / 100.0f); // to Pascal
	pressure = pressure / 100.0f; // to mbar

	_px4_baro.set_temperature(temperature);
	_px4_baro.update(timestamp_sample, pressure);

	perf_end(_sample_perf);

	return OK;
}

/*!
 * @brief This API performs the soft reset of the sensor.
 *
 * Refer: https://github.com/BoschSensortec/BMP3-Sensor-API/blob/master/bmp3.c
 */
bool
BMP388::soft_reset()
{
	bool    result = false;
	uint8_t status;
	int     ret;

	status = _interface->get_reg(BMP3_SENS_STATUS_REG_ADDR);

	if (status & BMP3_CMD_RDY) {
		ret = _interface->set_reg(BPM3_CMD_SOFT_RESET, BMP3_CMD_ADDR);

		if (ret == OK) {
			usleep(BMP3_POST_RESET_WAIT_TIME);
			status = _interface->get_reg(BMP3_ERR_REG_ADDR);

			if ((status & BMP3_CMD_ERR) == 0) {
				result = true;
			}
		}
	}

	return result;
}

/*
 * @brief function to calculate CRC for the trimming parameters
 *
 * Refer: https://github.com/BoschSensortec/BMP3-Sensor-API/blob/master/self-test/bmp3_selftest.c
 * */
static int8_t cal_crc(uint8_t seed, uint8_t data)
{
	int8_t poly = 0x1D;
	int8_t var2;
	uint8_t i;

	for (i = 0; i < 8; i++) {
		if ((seed & 0x80) ^ (data & 0x80)) {
			var2 = 1;

		} else {
			var2 = 0;
		}

		seed = (seed & 0x7F) << 1;
		data = (data & 0x7F) << 1;
		seed = seed ^ (uint8_t)(poly * var2);
	}

	return (int8_t)seed;
}

/*
 * @brief Function to verify the trimming parameters
 *
 * Refer: https://github.com/BoschSensortec/BMP3-Sensor-API/blob/master/self-test/bmp3_selftest.c
 * */
bool
BMP388::validate_trimming_param()
{
	uint8_t crc = 0xFF;
	uint8_t stored_crc;
	uint8_t *trim_param = (uint8_t *)_cal;

	static_assert(BMP3_CALIB_DATA_LEN <= sizeof(*_cal), "unexpected struct size");

	for (int i = 0; i < BMP3_CALIB_DATA_LEN; i++) {
		crc = (uint8_t)cal_crc(crc, trim_param[i]);
	}

	crc = (crc ^ 0xFF);

	stored_crc = _interface->get_reg(BMP3_TRIM_CRC_DATA_ADDR);

	return stored_crc == crc;
}

uint32_t
BMP388::get_measurement_time()
{
	/*
	  From BST-BMP388-DS001.pdf, page 25, table 21

	  Pressure      Temperature   Measurement     Max Time
	  Oversample    Oversample    Time (Forced)
	  x1            1x            4.9 ms          5.7 ms
	  x2            1x            6.9 ms          8.7 ms
	  x4            1x            10.9 ms         13.3 ms
	  x8            1x            18.9 ms         22.5 ms
	  x16           2x            36.9 ms         43.3 ms
	  x32           2x            68.9 ms         (not documented)
	*/

	uint32_t meas_time_us = 0; // unsupported value by default

	if (osr_t == BMP3_NO_OVERSAMPLING) {
		switch (osr_p) {
		case BMP3_NO_OVERSAMPLING:
			meas_time_us = 5700;
			break;

		case BMP3_OVERSAMPLING_2X:
			meas_time_us = 8700;
			break;

		case BMP3_OVERSAMPLING_4X:
			meas_time_us = 13300;
			break;

		case BMP3_OVERSAMPLING_8X:
			meas_time_us = 22500;
			break;
		}

	} else if (osr_t == BMP3_OVERSAMPLING_2X) {
		switch (osr_p) {
		case BMP3_OVERSAMPLING_16X:
			meas_time_us = 43300;
			break;

		case BMP3_OVERSAMPLING_32X:
			meas_time_us = 68900;
			break;
		}
	}

	return meas_time_us;
}

/*!
 * @brief This API sets the power control(pressure enable and
 * temperature enable), over sampling, odr and filter
 * settings in the sensor.
 *
 * Refer: https://github.com/BoschSensortec/BMP3-Sensor-API/blob/master/bmp3.c
 */
bool
BMP388::set_sensor_settings()
{
	_measure_interval = get_measurement_time();

	if (_measure_interval == 0) {
		PX4_WARN("unsupported oversampling selected");
		return false;
	}

	/* Select the pressure and temperature sensor to be enabled */
	uint8_t pwc_ctl_reg = 0;
	pwc_ctl_reg = BMP3_SET_BITS_POS_0(pwc_ctl_reg, BMP3_PRESS_EN, BMP3_ENABLE);
	pwc_ctl_reg = BMP3_SET_BITS(pwc_ctl_reg, BMP3_TEMP_EN, BMP3_ENABLE);

	int ret = _interface->set_reg(pwc_ctl_reg, BMP3_PWR_CTRL_ADDR);

	if (ret != OK) {
		PX4_WARN("failed to set settings BMP3_PWR_CTRL_ADDR");
		return false;
	}

	/* Select the over sampling settings for pressure and temperature */
	uint8_t osr_ctl_reg = 0;
	osr_ctl_reg = BMP3_SET_BITS_POS_0(osr_ctl_reg, BMP3_PRESS_OS, osr_p);
	osr_ctl_reg = BMP3_SET_BITS(osr_ctl_reg, BMP3_TEMP_OS, osr_t);

	ret = _interface->set_reg(osr_ctl_reg, BMP3_OSR_ADDR);

	if (ret != OK) {
		PX4_WARN("failed to set settings BMP3_OSR_ADDR");
		return false;
	}

	/* Using 'forced mode' so this is not required but here for future use possibly */
	uint8_t odr_ctl_reg = 0;
	odr_ctl_reg = BMP3_SET_BITS_POS_0(odr_ctl_reg, BMP3_ODR, odr);

	ret = _interface->set_reg(odr_ctl_reg, BMP3_ODR_ADDR);

	if (ret != OK) {
		PX4_WARN("failed to set output data rate register");
		return false;
	}

	uint8_t iir_ctl_reg = 0;
	iir_ctl_reg = BMP3_SET_BITS(iir_ctl_reg, BMP3_IIR_FILTER, iir_coef);
	ret = _interface->set_reg(iir_ctl_reg, BMP3_IIR_ADDR);

	if (ret != OK) {
		PX4_WARN("failed to set IIR settings");
		return false;
	}

	return true;
}


/*!
 * @brief This API sets the power mode of the sensor.
 *
 * Refer: https://github.com/BoschSensortec/BMP3-Sensor-API/blob/master/bmp3.c
 */
bool
BMP388::set_op_mode(uint8_t op_mode)
{
	bool    result = false;
	uint8_t last_set_mode;
	uint8_t op_mode_reg_val;
	int     ret = OK;

	op_mode_reg_val = _interface->get_reg(BMP3_PWR_CTRL_ADDR);
	last_set_mode = BMP3_GET_BITS(op_mode_reg_val, BMP3_OP_MODE);

	/* Device needs to be put in sleep mode to transition */
	if (last_set_mode != BMP3_SLEEP_MODE) {
		op_mode_reg_val = op_mode_reg_val & (~(BMP3_OP_MODE_MSK));
		ret = _interface->set_reg(op_mode_reg_val, BMP3_PWR_CTRL_ADDR);

		if (ret != OK) {
			return false;
		}

		px4_usleep(BMP3_POST_SLEEP_WAIT_TIME);
	}

	if (ret == OK) {
		op_mode_reg_val = _interface->get_reg(BMP3_PWR_CTRL_ADDR);
		op_mode_reg_val = BMP3_SET_BITS(op_mode_reg_val, BMP3_OP_MODE, op_mode);
		ret = _interface->set_reg(op_mode_reg_val, BMP3_PWR_CTRL_ADDR);

		if (ret != OK) {
			return false;
		}

		result = true;
	}

	return result;
}

/*!
 *  @brief This internal API is used to parse the pressure or temperature or
 *  both the data and store it in the bmp3_uncomp_data structure instance.
 *
 * Refer: https://github.com/BoschSensortec/BMP3-Sensor-API/blob/master/bmp3.c
 */
static void parse_sensor_data(const uint8_t *reg_data, struct bmp3_uncomp_data *uncomp_data)
{
	uint32_t data_xlsb;
	uint32_t data_lsb;
	uint32_t data_msb;

	data_xlsb = (uint32_t)reg_data[0];
	data_lsb = (uint32_t)reg_data[1] << 8;
	data_msb = (uint32_t)reg_data[2] << 16;
	uncomp_data->pressure = data_msb | data_lsb | data_xlsb;

	data_xlsb = (uint32_t)reg_data[3];
	data_lsb = (uint32_t)reg_data[4] << 8;
	data_msb = (uint32_t)reg_data[5] << 16;
	uncomp_data->temperature = data_msb | data_lsb | data_xlsb;
}


/*!
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in integer data type.
 * For eg if returned temperature is 2426 then it is 2426/100 = 24.26 deg Celsius
 *
 * Refer: https://github.com/BoschSensortec/BMP3-Sensor-API/blob/master/bmp3.c
 */
static int64_t compensate_temperature(const struct bmp3_uncomp_data *uncomp_data, struct bmp3_calib_data *calib_data)
{
	int64_t partial_data1;
	int64_t partial_data2;
	int64_t partial_data3;
	int64_t partial_data4;
	int64_t partial_data5;
	int64_t partial_data6;
	int64_t comp_temp;

	partial_data1 = ((int64_t)uncomp_data->temperature - (256 * calib_data->reg_calib_data.par_t1));
	partial_data2 = calib_data->reg_calib_data.par_t2 * partial_data1;
	partial_data3 = (partial_data1 * partial_data1);
	partial_data4 = (int64_t)partial_data3 * calib_data->reg_calib_data.par_t3;
	partial_data5 = ((int64_t)(partial_data2 * 262144) + partial_data4);
	partial_data6 = partial_data5 / 4294967296;

	/* Store t_lin in dev. structure for pressure calculation */
	calib_data->reg_calib_data.t_lin = partial_data6;
	comp_temp = (int64_t)((partial_data6 * 25) / 16384);

	return comp_temp;
}

/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in integer data type.
 * for eg return if pressure is 9528709 which is 9528709/100 = 95287.09 Pascal or 952.8709 hecto Pascal
 *
 * Refer: https://github.com/BoschSensortec/BMP3-Sensor-API/blob/master/bmp3.c
 */
static uint64_t compensate_pressure(const struct bmp3_uncomp_data *uncomp_data,
				    const struct bmp3_calib_data *calib_data)
{
	const struct bmp3_reg_calib_data *reg_calib_data = &calib_data->reg_calib_data;
	int64_t partial_data1;
	int64_t partial_data2;
	int64_t partial_data3;
	int64_t partial_data4;
	int64_t partial_data5;
	int64_t partial_data6;
	int64_t offset;
	int64_t sensitivity;
	uint64_t comp_press;

	partial_data1 = reg_calib_data->t_lin * reg_calib_data->t_lin;
	partial_data2 = partial_data1 / 64;
	partial_data3 = (partial_data2 * reg_calib_data->t_lin) / 256;
	partial_data4 = (reg_calib_data->par_p8 * partial_data3) / 32;
	partial_data5 = (reg_calib_data->par_p7 * partial_data1) * 16;
	partial_data6 = (reg_calib_data->par_p6 * reg_calib_data->t_lin) * 4194304;
	offset = (reg_calib_data->par_p5 * 140737488355328) + partial_data4 + partial_data5 + partial_data6;
	partial_data2 = (reg_calib_data->par_p4 * partial_data3) / 32;
	partial_data4 = (reg_calib_data->par_p3 * partial_data1) * 4;
	partial_data5 = (reg_calib_data->par_p2 - 16384) * reg_calib_data->t_lin * 2097152;
	sensitivity = ((reg_calib_data->par_p1 - 16384) * 70368744177664) + partial_data2 + partial_data4 + partial_data5;
	partial_data1 = (sensitivity / 16777216) * uncomp_data->pressure;
	partial_data2 = reg_calib_data->par_p10 * reg_calib_data->t_lin;
	partial_data3 = partial_data2 + (65536 * reg_calib_data->par_p9);
	partial_data4 = (partial_data3 * uncomp_data->pressure) / 8192;
	/*dividing by 10 followed by multiplying by 10 to avoid overflow caused by (uncomp_data->pressure * partial_data4) */
	partial_data5 = (uncomp_data->pressure * (partial_data4 / 10)) / 512;
	partial_data5 = partial_data5 * 10;
	partial_data6 = (int64_t)((uint64_t)uncomp_data->pressure * (uint64_t)uncomp_data->pressure);
	partial_data2 = (reg_calib_data->par_p11 * partial_data6) / 65536;
	partial_data3 = (partial_data2 * uncomp_data->pressure) / 128;
	partial_data4 = (offset / 4) + partial_data1 + partial_data5 + partial_data3;
	comp_press = (((uint64_t)partial_data4 * 25) / (uint64_t)1099511627776);

	return comp_press;
}

/*!
 * @brief This internal API is used to compensate the pressure or temperature
 * or both the data according to the component selected by the user.
 *
 * Refer: https://github.com/BoschSensortec/BMP3-Sensor-API/blob/master/bmp3.c
 */
bool
BMP388::compensate_data(uint8_t sensor_comp,
			const struct bmp3_uncomp_data *uncomp_data,
			struct bmp3_data *comp_data)
{
	int8_t rslt = OK;
	struct bmp3_calib_data calib_data = {0};
	struct bmp3_reg_calib_data *reg_calib_data = &calib_data.reg_calib_data;
	memcpy(reg_calib_data, _cal, 21);

	if ((uncomp_data != NULL) && (comp_data != NULL)) {
		if (sensor_comp & (BMP3_PRESS | BMP3_TEMP)) {
			comp_data->temperature = compensate_temperature(uncomp_data, &calib_data);
		}

		if (sensor_comp & BMP3_PRESS) {
			comp_data->pressure = compensate_pressure(uncomp_data, &calib_data);
		}

	} else {
		rslt = -1;
	}

	return (rslt == 0);
}

/*!
 * @brief This API reads the pressure, temperature or both data from the
 * sensor, compensates the data and store it in the bmp3_data structure
 * instance passed by the user.
 */
bool
BMP388::get_sensor_data(uint8_t sensor_comp, struct bmp3_data *comp_data)
{
	bool result = false;
	int8_t rslt;

	uint8_t reg_data[BMP3_P_T_DATA_LEN];
	struct bmp3_uncomp_data uncomp_data;

	rslt = _interface->get_reg_buf(BMP3_SENS_STATUS_REG_ADDR, reg_data, BMP3_P_T_DATA_LEN);

	if (rslt == OK) {
		uint8_t status = reg_data[0];

		// check if data ready (both temp and pressure)
		if ((status & (3 << 5)) != (3 << 5)) {
			return false;
		}

		parse_sensor_data(reg_data + 1, &uncomp_data);
		result = compensate_data(sensor_comp, &uncomp_data, comp_data);
	}

	return result;
}
