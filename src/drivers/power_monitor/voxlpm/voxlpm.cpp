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
 * @file voxlpm.cpp
 * Driver for the VOXL Power Management unit
 */

#include "voxlpm.hpp"

/*
 * The VOXLPM has two LTC2946 ICs on it.
 * Address 0x6A - measures battery voltage and current with a 0.0005 ohm sense resistor
 * Address 0x6B - measures 5VDC ouptut voltage and current
 */
VOXLPM::VOXLPM(I2CSPIBusOption bus_option, const int bus, int bus_frequency, VOXLPM_CH_TYPE ch_type) :
	I2C("voxlpm", nullptr, bus, (ch_type == VOXLPM_CH_TYPE_VBATT) ? VOXLPM_LTC2946_ADDR_VBATT : VOXLPM_LTC2946_ADDR_P5VD,
	    bus_frequency),
	ModuleParams(nullptr),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": sample")),
	_ch_type(ch_type),
	_battery(1, this)
{
	if (_ch_type == VOXLPM_CH_TYPE_VBATT) {
		_rsense = VOXLPM_RSENSE_VBATT;

	} else {
		_rsense = VOXLPM_RSENSE_5VOUT;
	}
}

VOXLPM::~VOXLPM()
{
	perf_free(_sample_perf);
}

int
VOXLPM::init()
{
	int ret = PX4_ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK) {
		return ret;
	}

	write_reg(DEFAULT_CTRLA_REG_VAL, VOXLPM_LTC2946_CTRLA_REG);
	write_reg(DEFAULT_CTRLB_REG_VAL, VOXLPM_LTC2946_CTRLB_REG);

	_battery.reset();

	start();

	return PX4_OK;
}

void
VOXLPM::print_status()
{
	perf_print_counter(_sample_perf);

	if (_ch_type == VOXLPM_CH_TYPE_VBATT) {
		printf("- type: BATT\n");

	} else {
		printf("- type: P5VDC\n");
	}

	printf("  - voltage: %9.2f VDC \n", (double)_voltage);
	printf("  - current: %9.2f ADC \n", (double)_amperage);
	printf("  - rsense: %9.6f ohm \n", (double)_rsense);
	printf("  - meas interval:  %u us \n", _meas_interval);
}

void
VOXLPM::start()
{
	ScheduleOnInterval(_meas_interval, 1000);
}

void
VOXLPM::RunImpl()
{
	measure();
}

int
VOXLPM::measure()
{
	parameter_update_s update;

	if (_parameter_sub.update(&update)) {
		updateParams();
	}


	_voltage = 0.0f;
	_amperage = 0.0f;

	uint8_t vraw[2];
	uint8_t iraw[2];

	perf_begin(_sample_perf);

	hrt_abstime tnow = hrt_absolute_time();

	int curr_read_ret = read_reg_buf(VOXLPM_LTC2946_DELTA_SENSE_MSB_REG, iraw, sizeof(iraw)); // 0x14
	int volt_read_ret = read_reg_buf(VOXLPM_LTC2946_VIN_MSB_REG, vraw, sizeof(vraw));         // 0x1E

	if ((volt_read_ret == 0) && (curr_read_ret == 0)) {
		uint16_t volt16 = (((uint16_t)vraw[0]) << 8) | vraw[1];   // MSB first
		volt16        >>= 4;                                      // data is 12 bit and left-aligned
		_voltage        = (volt16 / VOXLPM_LTC2946_RESOLUTION) * VOXLPM_LTC2946_VFS_SENSE;

		uint16_t curr16 = (((uint16_t)iraw[0]) << 8) | iraw[1];   // MSB first
		curr16        >>= 4;                                      // data is 12 bit and left-aligned
		_amperage       = curr16 / VOXLPM_LTC2946_RESOLUTION * VOXLPM_LTC2946_VFS_DELTA_SENSE / _rsense;

		switch (_ch_type) {
		case VOXLPM_CH_TYPE_VBATT: {
				_battery.updateBatteryStatus(tnow, _voltage, _amperage, true, true, 0, 0, true);
			}

		// fallthrough

		case VOXLPM_CH_TYPE_P5VDC: {
				memset(&_pm_status, 0x00, sizeof(_pm_status));
				_pm_status.timestamp = tnow;
				_pm_status.voltage_v = (float) _voltage;
				_pm_status.current_a = (float) _amperage;

				//_pm_pub_topic.power_w   = (float) _power * _power_lsb;
				_pm_pub_topic.publish(_pm_status);
			}
			break;

		}

	} else {
		switch (_ch_type) {
		case VOXLPM_CH_TYPE_VBATT: {
				_battery.updateBatteryStatus(tnow, 0.0, 0.0, true, true, 0, 0, true);
			}
			break;

		default:
			break;
		}
	}

	perf_end(_sample_perf);

	return OK;
}

uint8_t
VOXLPM::read_reg(uint8_t addr)
{
	uint8_t cmd[2] = { (uint8_t)(addr), 0};
	transfer(&cmd[0], 1, &cmd[1], 1);

	return cmd[1];
}

int
VOXLPM::read_reg_buf(uint8_t addr, uint8_t *buf, uint8_t len)
{
	const uint8_t cmd = (uint8_t)(addr);
	return transfer(&cmd, sizeof(cmd), buf, len);
}

int
VOXLPM::write_reg(uint8_t value, uint8_t addr)
{
	uint8_t cmd[2] = { (uint8_t)(addr), value};
	return transfer(cmd, sizeof(cmd), nullptr, 0);
}
