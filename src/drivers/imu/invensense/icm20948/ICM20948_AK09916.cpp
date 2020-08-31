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

#include "ICM20948_AK09916.hpp"

#include "ICM20948.hpp"

using namespace time_literals;

namespace AKM_AK09916
{

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

ICM20948_AK09916::ICM20948_AK09916(ICM20948 &icm20948, enum Rotation rotation) :
	ScheduledWorkItem("icm20948_ak09916", px4::device_bus_to_wq(icm20948.get_device_id())),
	_icm20948(icm20948),
	_px4_mag(icm20948.get_device_id(), rotation)
{
	_px4_mag.set_device_type(DRV_MAG_DEVTYPE_AK09916);
	_px4_mag.set_external(icm20948.external());

	// mag resolution is 1.5 milli Gauss per bit (0.15 μT/LSB)
	_px4_mag.set_scale(1.5e-3f);
}

ICM20948_AK09916::~ICM20948_AK09916()
{
	perf_free(_bad_transfer_perf);
	perf_free(_magnetic_sensor_overflow_perf);
}

bool ICM20948_AK09916::Reset()
{
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleNow();
	return true;
}

void ICM20948_AK09916::PrintInfo()
{
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_magnetic_sensor_overflow_perf);
}

void ICM20948_AK09916::Run()
{
	switch (_state) {
	case STATE::RESET:
		// CNTL3 SRST: Soft reset
		_icm20948.I2CSlaveRegisterWrite(I2C_ADDRESS_DEFAULT, (uint8_t)Register::CNTL3, CNTL3_BIT::SRST);
		_reset_timestamp = hrt_absolute_time();
		_failure_count = 0;
		_state = STATE::READ_WHO_AM_I;
		ScheduleDelayed(100_ms);
		break;

	case STATE::READ_WHO_AM_I:
		_icm20948.I2CSlaveExternalSensorDataEnable(I2C_ADDRESS_DEFAULT, (uint8_t)Register::WIA1, 1);
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(100_ms);
		break;

	case STATE::WAIT_FOR_RESET: {

			uint8_t WIA1 = 0;
			_icm20948.I2CSlaveExternalSensorDataRead(&WIA1, 1);

			if (WIA1 == Company_ID) {
				// set continuous mode 3 (50 Hz)
				_icm20948.I2CSlaveRegisterWrite(I2C_ADDRESS_DEFAULT, (uint8_t)Register::CNTL2, CNTL2_BIT::MODE3);

				_state = STATE::READ;
				ScheduleOnInterval(20_ms, 100_ms); // 50 Hz

			} else {
				// RESET not complete
				if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
					PX4_DEBUG("AK09916 reset failed, retrying");
					_state = STATE::RESET;
					ScheduleDelayed(1000_ms);

				} else {
					PX4_DEBUG("AK09916 reset not complete, check again in 100 ms");
					ScheduleDelayed(100_ms);
				}
			}
		}

		break;

	case STATE::READ: {
			TransferBuffer buffer{};
			const hrt_abstime timestamp_sample = hrt_absolute_time();
			bool ret = _icm20948.I2CSlaveExternalSensorDataRead((uint8_t *)&buffer, sizeof(TransferBuffer));

			bool success = false;

			if (ret) {
				if (buffer.ST2 & ST2_BIT::HOFL) {
					perf_count(_magnetic_sensor_overflow_perf);

				} else if (buffer.ST1 & ST1_BIT::DRDY) {
					const int16_t x = combine(buffer.HXH, buffer.HXL);
					const int16_t y = combine(buffer.HYH, buffer.HYL);
					const int16_t z = combine(buffer.HZH, buffer.HZL);

					// sensor's frame is +X forward (X), +Y right (Y), +Z down (Z)
					_px4_mag.update(timestamp_sample, x, y, z);

					success = true;

					if (_failure_count > 0) {
						_failure_count--;
					}
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

			// ensure icm20948 slave sensor reading is configured
			_icm20948.I2CSlaveExternalSensorDataEnable(I2C_ADDRESS_DEFAULT, (uint8_t)Register::ST1, sizeof(TransferBuffer));
		}

		break;
	}
}

} // namespace AKM_AK09916
