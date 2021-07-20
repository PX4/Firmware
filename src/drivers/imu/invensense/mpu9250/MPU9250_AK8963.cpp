/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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

#include "MPU9250_AK8963.hpp"

#include "MPU9250.hpp"

using namespace time_literals;

namespace AKM_AK8963
{

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

MPU9250_AK8963::MPU9250_AK8963(MPU9250 &mpu9250, enum Rotation rotation) :
	ScheduledWorkItem("mpu9250_ak8963", px4::device_bus_to_wq(mpu9250.get_device_id())),
	_mpu9250(mpu9250),
	_px4_mag(mpu9250.get_device_id(), rotation)
{
	_px4_mag.set_device_type(DRV_MAG_DEVTYPE_AK8963);
	_px4_mag.set_external(mpu9250.external());

	// in 16-bit sampling mode the mag resolution is 1.5 milli Gauss per bit */
	_px4_mag.set_scale(1.5e-3f);
}

MPU9250_AK8963::~MPU9250_AK8963()
{
	perf_free(_bad_transfer_perf);
	perf_free(_magnetic_sensor_overflow_perf);
}

bool MPU9250_AK8963::Reset()
{
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleNow();
	return true;
}

void MPU9250_AK8963::PrintInfo()
{
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_magnetic_sensor_overflow_perf);
}

void MPU9250_AK8963::Run()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		// CNTL2 SRST: Soft reset
		_mpu9250.I2CSlaveRegisterWrite(I2C_ADDRESS_DEFAULT, (uint8_t)Register::CNTL2, CNTL2_BIT::SRST);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::READ_WHO_AM_I;
		ScheduleDelayed(100_ms);
		break;

	case STATE::READ_WHO_AM_I:
		_mpu9250.I2CSlaveExternalSensorDataEnable(I2C_ADDRESS_DEFAULT, (uint8_t)Register::WIA, 1);
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(100_ms);
		break;

	case STATE::WAIT_FOR_RESET: {

			uint8_t WIA = 0;
			_mpu9250.I2CSlaveExternalSensorDataRead(&WIA, 1);

			if (WIA == Device_ID) {
				// if reset succeeded then configure
				if (!_sensitivity_adjustments_loaded) {
					// Set Fuse ROM Access mode before reading Fuse ROM data.
					_mpu9250.I2CSlaveRegisterWrite(I2C_ADDRESS_DEFAULT, (uint8_t)Register::CNTL1,
								       CNTL1_BIT::BIT_16 | CNTL1_BIT::FUSE_ROM_ACCESS_MODE);

					// Read ASAX, ASAY, ASAZ
					_mpu9250.I2CSlaveExternalSensorDataEnable(I2C_ADDRESS_DEFAULT, (uint8_t)Register::ASAX, 3);
					_state = STATE::READ_SENSITIVITY_ADJUSTMENTS;
					ScheduleDelayed(100_ms);

				} else {
					// set continuous mode 2 (100 Hz)
					_mpu9250.I2CSlaveRegisterWrite(I2C_ADDRESS_DEFAULT, (uint8_t)Register::CNTL1,
								       CNTL1_BIT::BIT_16 | CNTL1_BIT::CONTINUOUS_MODE_2);

					_state = STATE::READ;
					ScheduleDelayed(100_ms);
				}

			} else {
				// RESET not complete
				if ((now - _reset_timestamp) > 1000_ms) {
					PX4_DEBUG("AK8963 reset failed, retrying");
					_state = STATE::RESET;
					ScheduleDelayed(1000_ms);

				} else {
					PX4_DEBUG("AK8963 reset not complete, check again in 100 ms");
					ScheduleDelayed(100_ms);
				}
			}
		}

		break;

	case STATE::READ_SENSITIVITY_ADJUSTMENTS: {
			// read FUSE ROM (to get ASA corrections)
			uint8_t response[3] {};
			_mpu9250.I2CSlaveExternalSensorDataRead((uint8_t *)&response, sizeof(response));

			bool valid = true;

			for (int i = 0; i < 3; i++) {
				if (response[i] != 0 && response[i] != 0xFF) {
					_sensitivity[i] = ((float)(response[i] - 128) / 256.f) + 1.f;

				} else {
					valid = false;
				}
			}

			_sensitivity_adjustments_loaded = valid;

			// After reading fuse ROM data, set power-down mode (MODE[3:0]=“0000”) before the transition to another mode.
			_mpu9250.I2CSlaveRegisterWrite(I2C_ADDRESS_DEFAULT, (uint8_t)Register::CNTL1, 0);
			_state = STATE::RESET;
			ScheduleDelayed(100_ms);
		}
		break;

	case STATE::READ: {
			TransferBuffer buffer{};
			const hrt_abstime timestamp_sample = hrt_absolute_time();
			bool ret = _mpu9250.I2CSlaveExternalSensorDataRead((uint8_t *)&buffer, sizeof(TransferBuffer));

			bool success = false;

			if (ret) {
				if (buffer.ST2 & ST2_BIT::HOFL) {
					perf_count(_magnetic_sensor_overflow_perf);

				} else if (buffer.ST2 & ST2_BIT::BITM) {
					const int16_t x = combine(buffer.HXH, buffer.HXL);
					const int16_t y = combine(buffer.HYH, buffer.HYL);
					const int16_t z = combine(buffer.HZH, buffer.HZL);

					// sensor's frame is +Y forward (X), -X right (Y), +Z down (Z)
					// adjust with sensitivity scale factors
					float x_f = y * _sensitivity[0];   // X := +Y
					float y_f = -x * _sensitivity[1];  // Y := -X
					float z_f = z * _sensitivity[2];   // Z := +Z

					_px4_mag.update(timestamp_sample, x_f, y_f, z_f);

					success = true;

					if (_failure_count > 0) {
						_failure_count--;
					}

					ScheduleDelayed(20_ms); // ~50 Hz
					return;
				}
			}

			if (!success) {
				perf_count(_bad_transfer_perf);
				_failure_count++;

				if (_failure_count > 10) {
					Reset();
					return;
				}
			}

			// ensure mpu9250 slave sensor reading is configured
			_mpu9250.I2CSlaveExternalSensorDataEnable(I2C_ADDRESS_DEFAULT, (uint8_t)Register::ST1, sizeof(TransferBuffer));
			ScheduleDelayed(100_ms);
		}

		break;
	}
}

} // namespace AKM_AK8963
