/****************************************************************************
 *
 *   Copyright (c) 2014-2019, 2021 PX4 Development Team. All rights reserved.
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
 * @file LidarLiteI2C.cpp
 * @author Allyson Kreft
 *
 * Driver for the PulsedLight Lidar-Lite range finders connected via I2C.
 */

#include "LidarLiteI2C.h"

LidarLiteI2C::LidarLiteI2C(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_px4_rangefinder(get_device_id(), config.rotation)
{
	_px4_rangefinder.set_min_distance(LL40LS_MIN_DISTANCE);
	_px4_rangefinder.set_max_distance(LL40LS_MAX_DISTANCE);
	_px4_rangefinder.set_fov(0.008); // Divergence 8 mRadian

	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_LL40LS); /// TODO
}

LidarLiteI2C::~LidarLiteI2C()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_sensor_resets);
	perf_free(_sensor_zero_resets);
}

int
LidarLiteI2C::init()
{
	// Perform I2C init (and probe) first.
	if (I2C::init() != PX4_OK) {
		return PX4_ERROR;
	}

	start();
	return PX4_OK;
}

void
LidarLiteI2C::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_sensor_resets);
	perf_print_counter(_sensor_zero_resets);
	printf("poll interval:  %" PRIu32 "\n", get_measure_interval());
}

int
LidarLiteI2C::read_reg(const uint8_t reg, uint8_t &val)
{
	return lidar_transfer(&reg, 1, &val, 1);
}

int
LidarLiteI2C::write_reg(const uint8_t reg, const uint8_t &val)
{
	const uint8_t cmd[2] = { reg, val };
	return transfer(&cmd[0], 2, nullptr, 0);
}

int
LidarLiteI2C::lidar_transfer(const uint8_t *send, const unsigned send_len, uint8_t *recv, const unsigned recv_len)
{
	if (send != nullptr && send_len > 0) {
		int ret = transfer(send, send_len, nullptr, 0);

		if (ret != PX4_OK) {
			return ret;
		}
	}

	if (recv != nullptr && recv_len > 0) {
		return transfer(nullptr, 0, recv, recv_len);
	}

	return PX4_ERROR;
}

int
LidarLiteI2C::probe()
{
	// cope with both old and new I2C bus address
	const uint8_t addresses[2] = { LL40LS_BASEADDR, LL40LS_BASEADDR_OLD };

	uint8_t id_high = 0;
	uint8_t id_low = 0;

	for (uint8_t i = 0; i < sizeof(addresses); i++) {

		set_device_address(addresses[i]);

		/**
		 * The probing is divided into different cases. One to detect v2, one for v3 and v1 and one for v3HP.
		 * The reason for this is that registers are not consistent between different versions.
		 * The v3HP doesn't have the HW VERSION (or at least no version is specified),
		 * v1 and v3 don't have the unit id register while v2 has both.
		 * It would be better if we had a proper WHOAMI register.
		 */
		if ((read_reg(LL40LS_HW_VERSION, _hw_version) == OK) &&
		    (read_reg(LL40LS_SW_VERSION, _sw_version) == OK)) {

			if (read_reg(LL40LS_UNIT_ID_HIGH, id_high) == OK &&
			    read_reg(LL40LS_UNIT_ID_LOW, id_low) == OK) {
				_unit_id = (uint16_t)((id_high << 8) | id_low) & 0xFFFF;
			}

			if (_hw_version > 0) {

				if (_unit_id > 0) {
					// v2
					PX4_INFO("probe success - hw: %" PRIu8 ", sw:%" PRIu8 ", id: %" PRIu16, _hw_version, _sw_version, _unit_id);
					_px4_rangefinder.set_max_distance(LL40LS_MAX_DISTANCE_V2);

				} else {
					// v1 and v3
					PX4_INFO("probe success - hw: %" PRIu8 ", sw:%" PRIu8, _hw_version, _sw_version);
				}

			} else {

				if (_unit_id > 0) {
					// v3hp
					_is_v3hp = true;
					PX4_INFO("probe success - id: %" PRIu16, _unit_id);
				}
			}

			_retries = 1;
			return OK;
		}

		PX4_DEBUG("probe failed unit_id=0x%02" PRIx16 " hw_version=0x%02" PRIu8 " sw_version=0x%02" PRIu8,
			  _unit_id,  _hw_version, _sw_version);

	}

	// not found on any address
	return -EIO;
}

int
LidarLiteI2C::measure()
{
	if (_pause_measurements) {
		// we are in print_registers() and need to avoid
		// acquisition to keep the I2C peripheral on the
		// sensor active
		return OK;
	}

	// Send the command to begin a measurement.
	int ret = write_reg(LL40LS_MEASURE_REG, LL40LS_MSRREG_ACQUIRE);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret);

		// if we are getting lots of I2C transfer errors try
		// resetting the sensor
		if (perf_event_count(_comms_errors) % 10 == 0) {
			perf_count(_sensor_resets);
			reset_sensor();
		}

		return ret;
	}

	// remember when we sent the acquire so we can know when the
	// acquisition has timed out
	_acquire_time_usec = hrt_absolute_time();

	return OK;
}

int
LidarLiteI2C::reset_sensor()
{
	px4_usleep(15_ms);

	int ret = write_reg(LL40LS_SIG_COUNT_VAL_REG, LL40LS_SIG_COUNT_VAL_MAX);

	if (ret != PX4_OK) {
		return ret;
	}

	px4_usleep(15_ms);
	ret = write_reg(LL40LS_MEASURE_REG, LL40LS_MSRREG_RESET);


	if (ret != PX4_OK) {
		uint8_t sig_cnt;

		px4_usleep(15_ms);
		ret = read_reg(LL40LS_SIG_COUNT_VAL_REG, sig_cnt);

		if ((ret != PX4_OK) || (sig_cnt != LL40LS_SIG_COUNT_VAL_DEFAULT)) {
			PX4_INFO("Error: ll40ls reset failure. Exiting!\n");
			return ret;

		}
	}

	// wait for sensor reset to complete
	px4_usleep(50_ms);
	ret = write_reg(LL40LS_SIG_COUNT_VAL_REG, LL40LS_SIG_COUNT_VAL_MAX);

	if (ret != PX4_OK) {
		return ret;
	}

	// wait for register write to complete
	px4_usleep(1_ms);

	return OK;
}

void
LidarLiteI2C::print_registers()
{
	_pause_measurements = true;
	PX4_INFO("registers");
	// wait for a while to ensure the lidar is in a ready state
	px4_usleep(50_ms);

	for (uint8_t reg = 0; reg <= 0x67; reg++) {
		uint8_t val = 0;
		int ret = lidar_transfer(&reg, 1, &val, 1);

		if (ret != OK) {
			printf("%02" PRIx8 ":XX ", reg);

		} else {
			printf("%02" PRIx8 ":%02" PRIu8, reg, val);
		}

		if (reg % 16 == 15) {
			printf("\n");
		}
	}

	printf("\n");
	_pause_measurements = false;
}

int
LidarLiteI2C::collect()
{
	// read from the sensor
	uint8_t val[2] {};

	perf_begin(_sample_perf);

	// read the high and low byte distance registers
	uint8_t distance_reg = LL40LS_DISTHIGH_REG | LL40LS_AUTO_INCREMENT;
	int ret = lidar_transfer(&distance_reg, 1, &val[0], sizeof(val));

	// if the transfer failed or if the high bit of distance is
	// set then the distance is invalid
	if (ret < 0 || (val[0] & 0x80)) {
		if (hrt_absolute_time() - _acquire_time_usec > LL40LS_CONVERSION_TIMEOUT) {
			/*
			  NACKs from the sensor are expected when we
			  read before it is ready, so only consider it
			  an error if more than 100ms has elapsed.
			 */
			PX4_DEBUG("error reading from sensor: %d", ret);
			perf_count(_comms_errors);

			if (perf_event_count(_comms_errors) % 10 == 0) {
				perf_count(_sensor_resets);
				reset_sensor();
			}
		}

		perf_end(_sample_perf);
		// if we are getting lots of I2C transfer errors try
		// resetting the sensor
		return ret;
	}

	uint16_t distance_cm = (val[0] << 8) | val[1];
	const float distance_m = float(distance_cm) * 1e-2f;

	if (distance_cm == 0) {
		_zero_counter++;

		if (_zero_counter == 20) {
			/* we have had 20 zeros in a row - reset the
			   sensor. This is a known bad state of the
			   sensor where it returns 16 bits of zero for
			   the distance with a trailing NACK, and
			   keeps doing that even when the target comes
			   into a valid range.
			*/
			_zero_counter = 0;
			perf_end(_sample_perf);
			perf_count(_sensor_zero_resets);
			return reset_sensor();
		}

	} else {
		_zero_counter = 0;
	}

	// this should be fairly close to the end of the measurement, so the best approximation of the time
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	// Relative signal strength measurement, i.e. the strength of
	// the main signal peak compared to the general noise level.
	uint8_t signal_strength_reg = LL40LS_SIGNAL_STRENGTH_REG;
	ret = lidar_transfer(&signal_strength_reg, 1, &val[0], 1);

	// check if the transfer failed
	if (ret < 0) {
		if (hrt_elapsed_time(&_acquire_time_usec) > LL40LS_CONVERSION_TIMEOUT) {
			/*
			  NACKs from the sensor are expected when we
			  read before it is ready, so only consider it
			  an error if more than 100ms has elapsed.
			 */
			PX4_INFO("signal strength read failed");

			DEVICE_DEBUG("error reading signal strength from sensor: %d", ret);
			perf_count(_comms_errors);

			if (perf_event_count(_comms_errors) % 10 == 0) {
				perf_count(_sensor_resets);
				reset_sensor();
			}
		}

		perf_end(_sample_perf);
		// if we are getting lots of I2C transfer errors try
		// resetting the sensor
		return ret;
	}

	uint8_t ll40ls_signal_strength = val[0];
	uint8_t signal_quality;

	// We detect if V3HP is being used
	if (_is_v3hp) {
		//Normalize signal strength to 0...100 percent using the absolute signal strength.
		signal_quality = 100 * math::max(ll40ls_signal_strength - LL40LS_SIGNAL_STRENGTH_MIN_V3HP, 0) /
				 (LL40LS_SIGNAL_STRENGTH_MAX_V3HP - LL40LS_SIGNAL_STRENGTH_MIN_V3HP);

	} else {
		// Absolute peak strength measurement, i.e. absolute strength of main signal peak.
		uint8_t peak_strength_reg = LL40LS_PEAK_STRENGTH_REG;
		ret = lidar_transfer(&peak_strength_reg, 1, &val[0], 1);

		// check if the transfer failed
		if (ret < 0) {
			if (hrt_elapsed_time(&_acquire_time_usec) > LL40LS_CONVERSION_TIMEOUT) {
				/*
				NACKs from the sensor are expected when we
				read before it is ready, so only consider it
				an error if more than 100ms has elapsed.
				*/
				PX4_INFO("peak strength read failed");

				DEVICE_DEBUG("error reading peak strength from sensor: %d", ret);
				perf_count(_comms_errors);

				if (perf_event_count(_comms_errors) % 10 == 0) {
					perf_count(_sensor_resets);
					reset_sensor();
				}
			}

			perf_end(_sample_perf);
			// if we are getting lots of I2C transfer errors try
			// resetting the sensor
			return ret;
		}

		uint8_t ll40ls_peak_strength = val[0];

		// For v2 and v3 use ll40ls_signal_strength (a relative measure, i.e. peak strength to noise!) to reject potentially ambiguous measurements
		if (ll40ls_signal_strength <= LL40LS_SIGNAL_STRENGTH_LOW || distance_m < LL40LS_MIN_DISTANCE) {
			signal_quality = 0;

		} else {
			//Normalize signal strength to 0...100 percent using the absolute signal peak strength.
			signal_quality = 100 * math::max(ll40ls_peak_strength - LL40LS_PEAK_STRENGTH_LOW, 0) /
					 (LL40LS_PEAK_STRENGTH_HIGH - LL40LS_PEAK_STRENGTH_LOW);

		}
	}

	_px4_rangefinder.update(timestamp_sample, distance_m, signal_quality);

	perf_end(_sample_perf);
	return OK;
}

void LidarLiteI2C::start()
{
	// reset the report ring and state machine
	_collect_phase = false;

	// schedule a cycle to start things
	ScheduleNow();
}

void LidarLiteI2C::RunImpl()
{
	/* collection phase? */
	if (_collect_phase) {

		/* try a collection */
		if (OK != collect()) {
			PX4_DEBUG("collection error");

			/* if we've been waiting more than 200ms then
			   send a new acquire */
			if (hrt_elapsed_time(&_acquire_time_usec) > (LL40LS_CONVERSION_TIMEOUT * 2)) {
				_collect_phase = false;
			}

		} else {
			/* next phase is measurement */
			_collect_phase = false;

			/*
			 * Is there a collect->measure gap?
			 */
			if (get_measure_interval() > LL40LS_CONVERSION_INTERVAL) {

				/* schedule a fresh cycle call when we are ready to measure again */
				ScheduleDelayed(get_measure_interval() - LL40LS_CONVERSION_INTERVAL);

				return;
			}
		}
	}

	if (_collect_phase == false) {
		/* measurement phase */
		if (OK != measure()) {
			PX4_DEBUG("measure error");

		} else {
			/* next phase is collection. Don't switch to
			   collection phase until we have a successful
			   acquire request I2C transfer */
			_collect_phase = true;
		}
	}

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(LL40LS_CONVERSION_INTERVAL);
}
