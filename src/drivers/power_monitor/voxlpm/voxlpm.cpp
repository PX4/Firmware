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
 * Driver for the VOXL Power Module unit
 */

#include "voxlpm.hpp"

/*
 * The VOXLPM v2 has two LTC2946 ICs on it.
 * Address 0x6A - measures battery voltage and current with a 0.0005 ohm sense resistor
 * Address 0x6B - measures 5VDC ouptut voltage and current with a 0.005 ohm sense resistor
 *
 * The VOXLPM v3 has two INA231 ICs on it.
 * Address 0x44 - measures battery voltage and current with a 0.0005 ohm sense resistor
 * Address 0x45 - measures 5VDC/12VDC ouptut voltage and current with a 0.005 ohm sense resistor
 */
VOXLPM::VOXLPM(I2CSPIBusOption bus_option, const int bus, int bus_frequency, VOXLPM_CH_TYPE ch_type) :
	I2C(DRV_POWER_DEVTYPE_VOXLPM, MODULE_NAME, bus, VOXLPM_INA231_ADDR_VBATT, bus_frequency),
	ModuleParams(nullptr),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": sample")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms_errors")),
	_ch_type(ch_type),
	_battery(1, this, _meas_interval_us)
{
}

VOXLPM::~VOXLPM()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
VOXLPM::init()
{
	_initialized = false;
	int ret = PX4_ERROR;

	if (_ch_type == VOXLPM_CH_TYPE_VBATT) {
		_battery.updateBatteryStatus(
			hrt_absolute_time(),
			0.0,
			0.0,
			false,
			battery_status_s::BATTERY_SOURCE_POWER_MODULE,
			0,
			0.0
		);
	}

	/* do I2C init, it will probe the bus for two possible configurations, LTC2946 or INA231 */
	if (I2C::init() != OK) {
		return ret;
	}

	/* If we've probed and succeeded we'll have an accurate address here for the VBat addr */
	uint8_t addr = get_device_address();

	if (addr == VOXLPM_LTC2946_ADDR_VBATT || addr == VOXLPM_LTC2946_ADDR_P5VD) {
		_pm_type = VOXLPM_TYPE_V2_LTC;
		load_params(_pm_type, _ch_type);
		ret = init_ltc2946();

	} else if (addr == VOXLPM_INA231_ADDR_VBATT || addr == VOXLPM_INA231_ADDR_P5_12VDC) {
		_pm_type = VOXLPM_TYPE_V3_INA;
		load_params(_pm_type, _ch_type);
		ret = init_ina231();

	} else {
		PX4_ERR("Unkown device address");
		ret = PX4_ERROR;
	}

	if (ret == PX4_OK) {
		_initialized = true;
		start();
	}

	return ret;
}

int
VOXLPM::force_init()
{
	int ret = init();
	start();
	return ret;
}

int
VOXLPM::probe()
{
	int ret = PX4_ERROR;
	uint8_t data[2];

	uint8_t addr;

	/* Try LTC2946 first */
	if (_ch_type == VOXLPM_CH_TYPE_VBATT) {
		addr = VOXLPM_LTC2946_ADDR_VBATT;

	} else {
		addr = VOXLPM_LTC2946_ADDR_P5VD;
	}

	set_device_address(addr);

	/* Check status register */
	ret = read_reg_buf(VOXLPM_LTC2946_STATUS_REG, data, sizeof(data));

	if (ret) {
		/* Try INA231 next */
		if (_ch_type == VOXLPM_CH_TYPE_VBATT) {
			addr = VOXLPM_INA231_ADDR_VBATT;

		} else {
			addr = VOXLPM_INA231_ADDR_P5_12VDC;
		}

		set_device_address(addr);

		/* Check config register */
		ret = read_reg_buf(INA231_REG_CONFIG, data, sizeof(data));
	}

	return ret;
}

int
VOXLPM::load_params(VOXLPM_TYPE pm_type, VOXLPM_CH_TYPE ch_type)
{
	if (pm_type == VOXLPM_TYPE_V2_LTC) {
		/* No configuration needed */
		_rshunt = (ch_type == VOXLPM_CH_TYPE_VBATT) ? VOXLPM_LTC2946_VBAT_SHUNT : VOXLPM_LTC2946_VREG_SHUNT;

	} else if (pm_type == VOXLPM_TYPE_V3_INA) {

		_rshunt = -1.0f;
		float fvalue = -1.0f;
		param_t ph;

		/* Allow for configuration */
		if (_ch_type == VOXLPM_CH_TYPE_VBATT) {
			ph = param_find("VOXLPM_SHUNT_BAT");

			if (ph != PARAM_INVALID && param_get(ph, &fvalue) == PX4_OK) {
				_rshunt = fvalue;
			}

		} else {
			ph = param_find("VOXLPM_SHUNT_REG");

			if (ph != PARAM_INVALID && param_get(ph, &fvalue) == PX4_OK) {
				_rshunt = fvalue;
			}
		}

		if (_rshunt < 0) {
			_rshunt = (_ch_type == VOXLPM_CH_TYPE_VBATT) ? VOXLPM_INA231_VBAT_SHUNT : VOXLPM_INA231_VREG_SHUNT;
		}

	}

	return PX4_OK;
}

int
VOXLPM::init_ltc2946()
{
	write_reg(VOXLPM_LTC2946_CTRLA_REG, DEFAULT_LTC2946_CTRLA_REG_VAL);
	write_reg(VOXLPM_LTC2946_CTRLB_REG, DEFAULT_LTC2946_CTRLB_REG_VAL);
	return PX4_OK;
}

int
VOXLPM::init_ina231()
{
	int ret = PX4_OK;
	uint16_t cmd;

	/* Reset */
	cmd = INA231_RST_BIT;
	ret = write_word_swapped(INA231_REG_CONFIG, cmd);

	if (ret) {
		PX4_ERR("Failed to reset INA231");
		return ret;
	}

	/* Config */
	cmd = INA231_CONFIG;
	ret = write_word_swapped(INA231_REG_CONFIG, cmd);

	if (ret) {
		PX4_ERR("Failed to config INA231");
		return ret;
	}

	if (_ch_type == VOXLPM_CH_TYPE_VBATT) {
		_cal = (INA231_CONST / (VOXLPM_INA231_VBAT_I_LSB * _rshunt));

	} else {
		_cal = (INA231_CONST / (VOXLPM_INA231_VREG_I_LSB * _rshunt));
	}

	/* Set calibration */
	ret = write_word_swapped(INA231_REG_CALIBRATION, _cal);

	if (ret) {
		PX4_ERR("Failed to calibrate INA231");
		return ret;
	}

	return PX4_OK;
}

void
VOXLPM::print_status()
{
	perf_print_counter(_sample_perf);

	switch (_pm_type) {
	case VOXLPM_TYPE_V2_LTC:
		printf("- V2 (LTC2964)\n");
		break;

	case VOXLPM_TYPE_V3_INA:
		printf("- V3 (INA231)\n");
		break;

	default:
		break;
	}

	switch (_ch_type) {
	case VOXLPM_CH_TYPE_VBATT:
		printf("- type: BATT\n");
		break;

	case VOXLPM_CH_TYPE_P5VDC:
		printf("- type: P5VDC\n");
		break;

	case VOXLPM_CH_TYPE_P12VDC:
		printf("- type: P12VDC\n");
		break;

	default:
		printf("- type: UNKOWN\n");
		break;
	}

	printf("  - voltage: %9.4f VDC \n", (double)_voltage);
	printf("  - current: %9.4f ADC \n", (double)_amperage);
	printf("  - shunt: %9.4f mV, %9.4f mA\n", (double)_vshunt * 1000, (double)_vshuntamps * 1000);
	printf("  - rsense: %9.6f ohm, cal: %i\n", (double)_rshunt, _cal);
	printf("  - meas interval:  %u us \n", _meas_interval_us);
}

void
VOXLPM::start()
{
	ScheduleOnInterval(_meas_interval_us, 1000);
}

void
VOXLPM::RunImpl()
{
	measure();
}

int
VOXLPM::measure()
{
	int ret = PX4_ERROR;

	if (!_initialized) {
		ret = init();

		if (ret) {
			return ret;
		}
	}

	if (_parameter_update_sub.updated()) {
		// Read from topic to clear updated flag
		parameter_update_s parameter_update;
		_parameter_update_sub.copy(&parameter_update);

		updateParams();
	}

	perf_begin(_sample_perf);

	_voltage = 0.0f;
	_amperage = 0.0f;
	hrt_abstime tnow = hrt_absolute_time();

	switch (_pm_type) {
	case VOXLPM_TYPE_V2_LTC:
		ret = measure_ltc2946();
		break;

	case VOXLPM_TYPE_V3_INA:
		ret = measure_ina231();
		break;

	default:
		break;
	}

	if (ret == PX4_OK) {
		switch (_ch_type) {
		case VOXLPM_CH_TYPE_VBATT: {
				_actuators_sub.copy(&_actuator_controls);

				_battery.updateBatteryStatus(tnow,
							     _voltage,
							     _amperage,
							     true,
							     battery_status_s::BATTERY_SOURCE_POWER_MODULE,
							     0,
							     _actuator_controls.control[actuator_controls_s::INDEX_THROTTLE]);
			}

		// fallthrough
		case VOXLPM_CH_TYPE_P5VDC:
		case VOXLPM_CH_TYPE_P12VDC: {
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
		perf_count(_comms_errors);

		switch (_ch_type) {
		case VOXLPM_CH_TYPE_VBATT: {
				_battery.updateBatteryStatus(tnow,
							     0.0,
							     0.0,
							     true,
							     battery_status_s::BATTERY_SOURCE_POWER_MODULE,
							     0,
							     0.0);
			}
			break;

		default:
			break;
		}
	}

	perf_end(_sample_perf);

	return ret;
}

int
VOXLPM::measure_ltc2946()
{
	int ret = PX4_ERROR;
	uint8_t vraw[2];
	uint8_t iraw[2];

	int amp_ret = read_reg_buf(VOXLPM_LTC2946_DELTA_SENSE_MSB_REG, iraw, sizeof(iraw));  // 0x14
	int volt_ret = read_reg_buf(VOXLPM_LTC2946_VIN_MSB_REG, vraw, sizeof(vraw));         // 0x1E

	if ((amp_ret == 0) && (volt_ret == 0)) {
		uint16_t volt16 = (((uint16_t)vraw[0]) << 8) | vraw[1];   // MSB first
		volt16        >>= 4;                                      // data is 12 bit and left-aligned
		_voltage        = (volt16 / VOXLPM_LTC2946_RESOLUTION) * VOXLPM_LTC2946_VFS_SENSE;

		uint16_t curr16 = (((uint16_t)iraw[0]) << 8) | iraw[1];   // MSB first
		curr16        >>= 4;                                      // data is 12 bit and left-aligned
		_amperage       = curr16 / VOXLPM_LTC2946_RESOLUTION * VOXLPM_LTC2946_VFS_DELTA_SENSE / _rshunt;
		ret = PX4_OK;
	}

	return ret;
}

int
VOXLPM::measure_ina231()
{
	int ret = PX4_ERROR;

	uint8_t raw_vshunt[2];
	uint8_t raw_vbus[2];
	uint8_t raw_amps[2];

	int16_t vshunt = -1;
	uint16_t vbus = -1;
	uint16_t amps = 0;

	int vshunt_ret = read_reg_buf(INA231_REG_SHUNTVOLTAGE, raw_vshunt, sizeof(raw_vshunt));
	int vbus_ret = read_reg_buf(INA231_REG_BUSVOLTAGE, raw_vbus, sizeof(raw_vbus));
	int amp_ret = read_reg_buf(INA231_REG_CURRENT, raw_amps, sizeof(raw_amps));

	if ((vshunt_ret == 0) && (vbus_ret == 0) && (amp_ret == 0)) {
		vshunt = (((int16_t)raw_vshunt[0]) << 8) | raw_vshunt[1];  // MSB first
		vbus   = (((uint16_t)raw_vbus[0]) << 8) | raw_vbus[1];  // MSB first
		amps   = (((uint16_t)raw_amps[0]) << 8) | raw_amps[1];  // MSB first

		_voltage = (float) vbus * INA231_VBUSSCALE;
		_vshunt = (float) vshunt * INA231_VSHUNTSCALE;

		if (_ch_type == VOXLPM_CH_TYPE_VBATT) {
			/* vshunt is in microvolts, convert to AMPs */
			_vshuntamps = ((float) _vshunt / VOXLPM_INA231_VBAT_SHUNT);
			_amperage = (float) amps * VOXLPM_INA231_VBAT_I_LSB;

		} else {
			/* vshunt is in microvolts, convert to AMPs */
			_vshuntamps = ((float) _vshunt / VOXLPM_INA231_VREG_SHUNT);
			_amperage = (float) amps * VOXLPM_INA231_VREG_I_LSB;
		}

		ret = PX4_OK;
	}

	return ret;
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
VOXLPM::write_reg(uint8_t addr, uint8_t value)
{
	uint8_t cmd[2] = { (uint8_t)(addr), value};
	return transfer(cmd, sizeof(cmd), nullptr, 0);
}

int
VOXLPM::write_word_swapped(uint8_t addr, uint16_t value)
{
	uint8_t data[3] = {};
	data[0] = addr;
	data[1] = (value & 0xFF00) >> 8;
	data[2] = (value & 0x00FF);
	return transfer(data, sizeof(data), nullptr, 0);
}
