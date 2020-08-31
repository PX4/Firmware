/****************************************************************************
 *
 *   Copyright (c) 2017-2019 PX4 Development Team. All rights reserved.
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

#include "MPL3115A2.hpp"

#define MPL3115A2_ADDRESS        0x60

#define MPL3115A2_REG_WHO_AM_I   0x0c
#define MPL3115A2_WHO_AM_I       0xC4

#define OUT_P_MSB                0x01

#define MPL3115A2_CTRL_REG1      0x26
#  define CTRL_REG1_ALT          (1 << 7)
#  define CTRL_REG1_RAW          (1 << 6)
#  define CTRL_REG1_OS_SHIFTS    (3)
#  define CTRL_REG1_OS_MASK      (0x7 << CTRL_REG1_OS_SHIFTS)
#  define CTRL_REG1_OS(n)        (((n)& 0x7) << CTRL_REG1_OS_SHIFTS)
#  define CTRL_REG1_RST          (1 << 2)
#  define CTRL_REG1_OST          (1 << 1)
#  define CTRL_REG1_SBYB         (1 << 0)

#define MPL3115A2_CONVERSION_INTERVAL	10000	/* microseconds */
#define MPL3115A2_OSR                   2       /* Over Sample rate of 4 18MS Minimum time between data samples */
#define MPL3115A2_CTRL_TRIGGER          (CTRL_REG1_OST | CTRL_REG1_OS(MPL3115A2_OSR))

MPL3115A2::MPL3115A2(I2CSPIBusOption bus_option, const int bus, int bus_frequency) :
	I2C(DRV_BARO_DEVTYPE_MPL3115A2, MODULE_NAME, bus, MPL3115A2_ADDRESS, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_px4_barometer(get_device_id()),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_measure_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": measure")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err"))
{
}

MPL3115A2::~MPL3115A2()
{
	perf_free(_sample_perf);
	perf_free(_measure_perf);
	perf_free(_comms_errors);
}

int MPL3115A2::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		PX4_ERR("init failed");
		return PX4_ERROR;
	}

	start();

	return PX4_OK;
}

int MPL3115A2::probe()
{
	_retries = 10;
	uint8_t whoami = 0;

	if ((RegisterRead(MPL3115A2_REG_WHO_AM_I, &whoami) > 0) && (whoami == MPL3115A2_WHO_AM_I)) {
		/*
		 * Disable retries; we may enable them selectively in some cases,
		 * but the device gets confused if we retry some of the commands.
		 */
		_retries = 0;
		return PX4_OK;
	}

	return -EIO;
}

int MPL3115A2::RegisterRead(uint8_t reg, void *data, unsigned count)
{
	uint8_t cmd = reg;
	int ret = transfer(&cmd, 1, (uint8_t *)data, count);
	return ret == PX4_OK ? count : ret;
}

int MPL3115A2::RegisterWrite(uint8_t reg, uint8_t data)
{
	uint8_t buf[2] = { reg, data};
	int ret = transfer(buf, sizeof(buf), NULL, 0);
	return ret == PX4_OK ? 2 : ret;
}

void MPL3115A2::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;

	/* schedule a cycle to start things */
	ScheduleNow();
}

int MPL3115A2::reset()
{
	int max = 10;
	RegisterWrite(MPL3115A2_CTRL_REG1, CTRL_REG1_RST);
	int rv = CTRL_REG1_RST;
	int ret = 1;

	while (ret == 1 && (rv & CTRL_REG1_RST) && max--) {
		usleep(400);
		ret = RegisterRead(MPL3115A2_CTRL_REG1, &rv);
	}

	return ret == 1 ? PX4_OK : ret;
}

void MPL3115A2::RunImpl()
{
	int ret = PX4_ERROR;

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		ret = collect();

		if (ret == -EIO) {
			/* issue a reset command to the sensor */
			reset();

			/* reset the collection state machine and try again - we need
			 * to wait 2.8 ms after issuing the sensor reset command
			 * according to the MPL3115A2 datasheet
			 */
			_collect_phase = false;
			ScheduleDelayed(2800);
			return;
		}

		if (ret == -EAGAIN) {
			/* Ready read it on next cycle */
			ScheduleDelayed(MPL3115A2_CONVERSION_INTERVAL);

			return;
		}

		/* next phase is measurement */
		_collect_phase = false;
	}

	/* Look for a ready condition */
	ret = measure();

	if (ret == -EIO) {
		/* issue a reset command to the sensor */
		reset();

		/* reset the collection state machine and try again */
		start();
		return;
	}

	/* next phase is measurement */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(MPL3115A2_CONVERSION_INTERVAL);
}

int MPL3115A2::measure()
{
	perf_begin(_measure_perf);

	// Send the command to read the ADC for P and T.
	unsigned addr = (MPL3115A2_CTRL_REG1 << 8) | MPL3115A2_CTRL_TRIGGER;

	/*
	 * Disable retries on this command; we can't know whether failure
	 * means the device did or did not see the command.
	 */
	_retries = 0;
	int ret = RegisterWrite((addr >> 8) & 0xff, addr & 0xff);

	if (ret == -EIO) {
		perf_count(_comms_errors);
	}

	perf_end(_measure_perf);

	return PX4_OK;
}

int MPL3115A2::collect()
{
	perf_begin(_sample_perf);

	uint8_t ctrl{};
	int ret = RegisterRead(MPL3115A2_CTRL_REG1, (void *)&ctrl, 1);

	if (ret == -EIO) {
		perf_end(_sample_perf);
		return ret;
	}

	if (ctrl & CTRL_REG1_OST) {
		perf_end(_sample_perf);
		return -EAGAIN;
	}


	/* read the most recent measurement
	 * 3 Pressure and 2 temprtture
	 */
	uint8_t	b[3 + 2] {};
	uint8_t reg = OUT_P_MSB;
	const hrt_abstime timestamp_sample = hrt_absolute_time();
	ret = transfer(&reg, 1, &b[0], sizeof(b));

	if (ret == -EIO) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

#pragma pack(push, 1)
	struct MPL3115A2_data_t {
		union {
			uint32_t q;
			uint16_t w[sizeof(q) / sizeof(uint16_t)];
			uint8_t  b[sizeof(q) / sizeof(uint8_t)];
		} pressure;

		union {
			uint16_t w;
			uint8_t  b[sizeof(w)];
		} temperature;
	} reading;
#pragma pack(pop)

	reading.pressure.q = ((uint32_t)b[0]) << 18 | ((uint32_t) b[1]) << 10 | (((uint32_t)b[2]) & 0xc0) << 2 | ((
				     b[2] & 0x30) >> 4);
	reading.temperature.w = ((uint16_t) b[3]) << 8 | (b[4] >> 4);

	float T = (float) reading.temperature.b[1] + ((float)(reading.temperature.b[0]) / 16.0f);
	float P = (float)(reading.pressure.q >> 8) + ((float)(reading.pressure.b[0]) / 4.0f);

	_px4_barometer.set_error_count(perf_event_count(_comms_errors));
	_px4_barometer.set_temperature(T);
	_px4_barometer.update(timestamp_sample, P / 100.0f);

	perf_end(_sample_perf);

	return PX4_OK;
}

void MPL3115A2::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
