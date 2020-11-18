/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "bno055.hpp"

using namespace std::placeholders;

BNO055::BNO055(I2CSPIBusOption bus_option, int bus, int bus_frequency) :
	I2C(DRV_IMU_DEVTYPE_BNO055, MODULE_NAME, bus, BNO055_I2C_ADDR1, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_px4_accel(get_device_id()),
	_px4_gyro(get_device_id()),
	_px4_mag(get_device_id())
{
}

BNO055::~BNO055()
{
}

int BNO055::init()
{
	// do I2C init
	int ret = I2C::init();

	if (ret != OK) {
		PX4_ERR("I2C setup failed");
		return ret;
	}

	// set up bno lib
	// the bind() is used since the (C) lib wants pointer-to-function, but our (C++) wrapper has pointer-to-member
	bno055_struct.bus_write = &BNO055::write_reg_trampoline;
	bno055_struct.bus_read = &BNO055::read_reg_trampoline;
	bno055_struct.delay_msec = &BNO055::bno_msleep;
	bno055_struct.dev_addr = BNO055_I2C_ADDR1; // TODO
	bno055_struct.i2c_calling_obj = this;

	// init the bosch lib
	ret = bno055_init(&bno055_struct);
	if (ret != OK) {
		PX4_ERR("Bosch lib init failed");
		return ret;
	}

	// check if the sensor has the right chip id (read in bno055_init)
	if (bno055_struct.chip_id != 0xA0) {
		PX4_ERR("Chip ID is wrong, need A0, got %x", bno055_struct.chip_id);
		return -1;
	}

	// reset sensor first
	// TODO doesn't work yet
	ret = reset();
	if (ret != PX4_OK) {
		PX4_ERR("Can't reset sensor, is it connected?");
		return ret;
	}
	// wait some time for the sensor to reset
	bno_msleep(700);

	// first go into config mode
	ret += bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
	ret += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
	uint8_t operation_mode = 255;
	ret += bno055_get_operation_mode(&operation_mode);
	if (ret != OK || operation_mode != BNO055_OPERATION_MODE_CONFIG) {
		PX4_ERR("Can't enter Config Mode, mode is %x, ret is %x", operation_mode, ret);
		return -1;
	}

	// TODO check (and update) those values, set scaling
	ret += bno055_set_accel_bw(ACCEL_BW_REGVAL);
	ret += bno055_set_gyro_bw(GYRO_BW_REGVAL);
	ret += bno055_set_mag_data_output_rate(MAG_RATE_REGVAL);
	if (ret != OK) {
		PX4_ERR("Can't set parameters");
		return ret;
	}

	// switch to AMG mode: all sensors, no onboard fusion
	ret += bno055_set_operation_mode(BNO055_OPERATION_MODE_AMG);
	operation_mode = 255;
	ret += bno055_get_operation_mode(&operation_mode);
	if (ret != OK || operation_mode != BNO055_OPERATION_MODE_AMG) {
		PX4_ERR("Can't switch to running mode, current mode is %x", operation_mode);
		return ret;
	}

	// schedule sensor polling
	// TODO make a separate step? Or just integrate it here?
	start();

	return ret;
}

int BNO055::reset()
{
	return bno055_set_sys_rst(1);
}

int8_t BNO055::read_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
	return transfer(&reg_addr, 1, reg_data, cnt);
}

int8_t BNO055::write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
	uint8_t buffer[cnt + 1];
	buffer[0] = reg_addr;
	memcpy(buffer + 1, reg_data, cnt);
	return transfer(buffer, cnt+1, nullptr, 0);
}

void BNO055::start()
{
	PX4_INFO("Polling BNO055 every %u us", POLLING_INTERVAL_US);
	ScheduleOnInterval(POLLING_INTERVAL_US);
}

void BNO055::RunImpl()
{
	hrt_abstime current_time = hrt_absolute_time();

	if (bno055_convert_double_accel_xyz_msq(&accel_xyz) != OK) {
		PX4_DEBUG("Reading accel failed, skipping");
		_px4_accel.increase_error_count();

	} else {
		_px4_accel.update(current_time, accel_xyz.x, accel_xyz.y, accel_xyz.z);
	}

	if (bno055_convert_double_gyro_xyz_rps(&gyro_xyz) != OK) {
		_px4_gyro.increase_error_count();

	} else {
		_px4_gyro.update(current_time, gyro_xyz.x, gyro_xyz.y, gyro_xyz.z);
	}

	// only read the magnetometer data when new data is available
	if ((current_time - mag_last_read) > MAG_INTERVAL_US) {

		if (bno055_convert_double_mag_xyz_uT(&mag_xyz) != OK) {
			PX4_DEBUG("Reading mag failed, skipping");
			_px4_mag.increase_error_count();

		} else {
			_px4_mag.update(current_time, mag_xyz.x/100, mag_xyz.y/100, mag_xyz.z/100);
		}

		mag_last_read = current_time;
	}

}
