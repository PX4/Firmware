/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

#include "MS5525.hpp"

int
MS5525::measure()
{
	int ret = PX4_ERROR;

	if (_inited) {
		// send the command to begin a conversion.
		uint8_t cmd = _current_cmd;
		ret = transfer(&cmd, 1, nullptr, 0);

		if (ret != PX4_OK) {
			perf_count(_comms_errors);
		}

	} else {
		_inited = init_ms5525();

		if (_inited) {
			ret = PX4_OK;
		}
	}

	return ret;
}

bool
MS5525::init_ms5525()
{
	// Step 1 - reset
	uint8_t cmd = CMD_RESET;
	int ret = transfer(&cmd, 1, nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return false;
	}

	px4_usleep(3000);

	// Step 2 - read calibration coefficients from prom

	// prom layout
	// 0 factory data and the setup
	// 1-6 calibration coefficients
	// 7 serial code and CRC
	uint16_t prom[8];

	for (uint8_t i = 0; i < 8; i++) {
		cmd = CMD_PROM_START + i * 2;

		// request PROM value
		ret = transfer(&cmd, 1, nullptr, 0);

		if (ret != PX4_OK) {
			perf_count(_comms_errors);
			return false;
		}

		// read 2 byte value
		uint8_t val[2];
		ret = transfer(nullptr, 0, &val[0], 2);

		if (ret == PX4_OK) {
			prom[i] = (val[0] << 8) | val[1];

		} else {
			perf_count(_comms_errors);
			return false;
		}
	}

	// Step 3 - check CRC
	const uint8_t crc = prom_crc4(prom);
	const uint8_t onboard_crc = prom[7] & 0xF;

	if (crc == onboard_crc) {
		// store valid calibration coefficients
		C1 = prom[1];
		C2 = prom[2];
		C3 = prom[3];
		C4 = prom[4];
		C5 = prom[5];
		C6 = prom[6];

		Tref = int64_t(C5) * (1UL << Q5);
		_device_id.devid_s.devtype = DRV_DIFF_PRESS_DEVTYPE_MS5525;

		return true;

	} else {
		PX4_ERR("CRC mismatch");
		return false;
	}
}

uint8_t
MS5525::prom_crc4(uint16_t n_prom[]) const
{
	// see Measurement Specialties AN520

	// crc remainder
	unsigned int n_rem = 0x00;

	// original value of the crc
	unsigned int crc_read = n_prom[7]; // save read CRC
	n_prom[7] = (0xFF00 & (n_prom[7])); // CRC byte is replaced by 0

	// operation is performed on bytes
	for (int cnt = 0; cnt < 16; cnt++) {
		// choose LSB or MSB
		if (cnt % 2 == 1) {
			n_rem ^= (unsigned short)((n_prom[cnt >> 1]) & 0x00FF);

		} else {
			n_rem ^= (unsigned short)(n_prom[cnt >> 1] >> 8);
		}

		for (uint8_t n_bit = 8; n_bit > 0; n_bit--) {
			if (n_rem & (0x8000)) {
				n_rem = (n_rem << 1) ^ 0x3000;

			} else {
				n_rem = (n_rem << 1);
			}
		}
	}

	n_rem = (0x000F & (n_rem >> 12)); // final 4-bit reminder is CRC code
	n_prom[7] = crc_read; // restore the crc_read to its original place

	return (n_rem ^ 0x00);
}

int
MS5525::collect()
{
	perf_begin(_sample_perf);

	// read ADC
	uint8_t cmd = CMD_ADC_READ;
	int ret = transfer(&cmd, 1, nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	// read 24 bits from the sensor
	uint8_t val[3];
	ret = transfer(nullptr, 0, &val[0], 3);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	uint32_t adc = (val[0] << 16) | (val[1] << 8) | val[2];

	// If the conversion is not executed before the ADC read command, or the ADC read command is repeated, it will give 0 as the output
	// result. If the ADC read command is sent during conversion the result will be 0, the conversion will not stop and
	// the final result will be wrong. Conversion sequence sent during the already started conversion process will yield
	// incorrect result as well.
	if (adc == 0) {
		perf_count(_comms_errors);
		return EAGAIN;
	}

	if (_current_cmd == CMD_CONVERT_PRES) {
		D1 = adc;
		_pressure_count++;

		if (_pressure_count > 9) {
			_pressure_count = 0;
			_current_cmd = CMD_CONVERT_TEMP;
		}

	} else if (_current_cmd == CMD_CONVERT_TEMP) {
		D2 = adc;
		_current_cmd = CMD_CONVERT_PRES;

		// only calculate and publish after new pressure readings
		return PX4_OK;
	}

	// not ready yet
	if (D1 == 0 || D2 == 0) {
		return EAGAIN;
	}

	// Difference between actual and reference temperature
	//  dT = D2 - Tref
	const int64_t dT = D2 - Tref;

	// Measured temperature
	//  TEMP = 20°C + dT * TEMPSENS
	const int64_t TEMP = 2000 + (dT * int64_t(C6)) / (1UL << Q6);

	// Offset at actual temperature
	//  OFF = OFF_T1 + TCO * dT
	const int64_t OFF = int64_t(C2) * (1UL << Q2) + (int64_t(C4) * dT) / (1UL << Q4);

	// Sensitivity at actual temperature
	//  SENS = SENS_T1 + TCS * dT
	const int64_t SENS = int64_t(C1) * (1UL << Q1) + (int64_t(C3) * dT) / (1UL << Q3);

	// Temperature Compensated Pressure (example 24996 = 2.4996 psi)
	//  P = D1 * SENS - OFF
	const int64_t P = (D1 * SENS / (1UL << 21) - OFF) / (1UL << 15);

	const float diff_press_PSI = P * 0.0001f;

	// 1 PSI = 6894.76 Pascals
	static constexpr float PSI_to_Pa = 6894.757f;
	const float diff_press_pa_raw = diff_press_PSI * PSI_to_Pa;

	const float temperature_c = TEMP * 0.01f;

	differential_pressure_s diff_pressure = {
		.timestamp = hrt_absolute_time(),
		.error_count = perf_event_count(_comms_errors),
		.differential_pressure_raw_pa = diff_press_pa_raw - _diff_pres_offset,
		.differential_pressure_filtered_pa =  _filter.apply(diff_press_pa_raw) - _diff_pres_offset,
		.temperature = temperature_c,
		.device_id = _device_id.devid
	};

	_airspeed_pub.publish(diff_pressure);

	ret = OK;

	perf_end(_sample_perf);

	return ret;
}

void
MS5525::RunImpl()
{
	int ret = PX4_ERROR;

	// collection phase
	if (_collect_phase) {
		// perform collection
		ret = collect();

		if (OK != ret) {
			/* restart the measurement state machine */
			_collect_phase = false;
			_sensor_ok = false;
			ScheduleNow();
			return;
		}

		// next phase is measurement
		_collect_phase = false;

		// is there a collect->measure gap?
		if (_measure_interval > CONVERSION_INTERVAL) {

			// schedule a fresh cycle call when we are ready to measure again
			ScheduleDelayed(_measure_interval - CONVERSION_INTERVAL);

			return;
		}
	}

	/* measurement phase */
	ret = measure();

	if (OK != ret) {
		DEVICE_DEBUG("measure error");
	}

	_sensor_ok = (ret == OK);

	// next phase is collection
	_collect_phase = true;

	// schedule a fresh cycle call when the measurement is done
	ScheduleDelayed(CONVERSION_INTERVAL);
}
