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

#include "FXAS21002C.hpp"

/* SPI protocol address bits */
#define DIR_READ(a)                     ((a) | (1 << 7))
#define DIR_WRITE(a)                    ((a) & 0x7f)
#define swap16(w)                       __builtin_bswap16((w))

#define FXAS21002C_STATUS                0x00
#define FXAS21002C_OUT_X_MSB             0x01
#define FXAS21002C_OUT_X_LSB             0x02
#define FXAS21002C_OUT_Y_MSB             0x03
#define FXAS21002C_OUT_Y_LSB             0x04
#define FXAS21002C_OUT_Z_MSB             0x05
#define FXAS21002C_OUT_Z_LSB             0x06

#define FXAS21002C_DR_STATUS             0x07
#  define DR_STATUS_ZYXOW                (1 << 7)
#  define DR_STATUS_ZOW                  (1 << 6)
#  define DR_STATUS_YOW                  (1 << 5)
#  define DR_STATUS_XOW                  (1 << 4)
#  define DR_STATUS_ZYXDR                (1 << 3)
#  define DR_STATUS_ZDR                  (1 << 2)
#  define DR_STATUS_YDR                  (1 << 1)
#  define DR_STATUS_XDR                  (1 << 0)

#define FXAS21002C_F_STATUS              0x08
#  define F_STATUS_F_OVF                 (1 << 7)
#  define F_STATUS_F_WMKF                (1 << 6)
#  define F_STATUS_F_CNT_SHIFTS          0
#  define F_STATUS_F_CNT_MASK            (0x3f << F_STATUS_F_CNT_SHIFTS)

#define FXAS21002C_F_SETUP               0x09
#  define F_SETUP_F_MODE_SHIFTS          6
#  define F_SETUP_F_MODE_MASK            (0x3 << F_SETUP_F_MODE_SHIFTS)
#  define F_SETUP_F_WMRK_SHIFTS          0
#  define F_SETUP_F_WMRK_MASK            (0x3f << F_SETUP_F_WMRK_SHIFTS)

#define FXAS21002C_F_EVENT               0x0a
#  define F_EVENT_F_EVENT                (1 << 5)
#  define F_EVENT_FE_TIME_SHIFTS         0
#  define F_EVENT_FE_TIME_MASK           (0x1f << F_EVENT_FE_TIME_SHIFTS)

#define FXAS21002C_INT_SRC_FLAG          0x0b
#  define INT_SRC_FLAG_BOOTEND           (1 << 3)
#  define INT_SRC_FLAG_SRC_FIFO          (1 << 2)
#  define INT_SRC_FLAG_SRC_RT            (1 << 1)
#  define INT_SRC_FLAG_SRC_DRDY          (1 << 0)

#define FXAS21002C_WHO_AM_I              0x0c
#define   WHO_AM_I                       0xd7

#define FXAS21002C_CTRL_REG0             0x0d
#  define CTRL_REG0_BW_SHIFTS            6
#  define CTRL_REG0_BW_MASK              (0x3 << CTRL_REG0_BW_SHIFTS)
#  define CTRL_REG0_BW(n)                (((n) & 0x3) << CTRL_REG0_BW_SHIFTS)
#    define CTRL_REG0_BW_HIGH             CTRL_REG0_BW(0)
#    define CTRL_REG0_BW_MED              CTRL_REG0_BW(1)
#    define CTRL_REG0_BW_LOW              CTRL_REG0_BW(2)
#  define CTRL_REG0_SPIW                 (1 << 6)
#  define CTRL_REG0_SEL_SHIFTS           3
#  define CTRL_REG0_SEL_MASK             (0x2 << CTRL_REG0_SEL_SHIFTS)
#  define CTRL_REG0_HPF_EN               (1 << 2)
#  define CTRL_REG0_FS_SHIFTS            0
#  define CTRL_REG0_FS_MASK              (0x3 << CTRL_REG0_FS_SHIFTS)
#  define CTRL_REG0_FS_2000_DPS          (0 << CTRL_REG0_FS_SHIFTS)
#  define CTRL_REG0_FS_1000_DPS          (1 << CTRL_REG0_FS_SHIFTS)
#  define CTRL_REG0_FS_500_DPS           (2 << CTRL_REG0_FS_SHIFTS)
#  define CTRL_REG0_FS_250_DPS           (3 << CTRL_REG0_FS_SHIFTS)

#define FXAS21002C_RT_CFG                0x0e
#  define RT_CFG_ELE                     (1 << 3)
#  define RT_CFG_ZTEFE                   (1 << 2)
#  define RT_CFG_YTEFE                   (1 << 1)
#  define RT_CFG_XTEFE                   (1 << 0)

#define FXAS21002C_RT_SRC                0x0f
#  define RT_SRC_EA                      (1 << 6)
#  define RT_SRC_ZRT                     (1 << 5)
#  define RT_SRC_Z_RT_POL                (1 << 4)
#  define RT_SRC_YRT                     (1 << 3)
#  define RT_SRC_Y_RT_POL                (1 << 2)
#  define RT_SRC_XRT                     (1 << 1)
#  define RT_SRC_X_RT_POL                (1 << 0)

#define FXAS21002C_RT_THS                0x10
#  define RT_THS_DBCNTM                  (1 << 7)
#  define RT_THS_THS_SHIFTS              0
#  define RT_THS_THS_MASK                (0x7f << RT_THS_THS_SHIFTS)

#define FXAS21002C_RT_COUNT              0x11
#define FXAS21002C_TEMP                  0x12

#define FXAS21002C_CTRL_REG1             0x13
#  define CTRL_REG1_RST                  (1 << 6)
#  define CTRL_REG1_ST                   (1 << 5)
#  define CTRL_REG1_DR_SHIFTS             2
#  define CTRL_REG1_DR_MASK               (0x07 << CTRL_REG1_DR_SHIFTS)
#  define CTRL_REG1_DR_12_5               (7 << CTRL_REG1_DR_SHIFTS)
#  define CTRL_REG1_DR_12_5_1             (6 << CTRL_REG1_DR_SHIFTS)
#  define CTRL_REG1_DR_25HZ               (5 << CTRL_REG1_DR_SHIFTS)
#  define CTRL_REG1_DR_50HZ               (4 << CTRL_REG1_DR_SHIFTS)
#  define CTRL_REG1_DR_100HZ              (3 << CTRL_REG1_DR_SHIFTS)
#  define CTRL_REG1_DR_200HZ              (2 << CTRL_REG1_DR_SHIFTS)
#  define CTRL_REG1_DR_400HZ              (1 << CTRL_REG1_DR_SHIFTS)
#  define CTRL_REG1_DR_800HZ              (0 << CTRL_REG1_DR_SHIFTS)
#  define CTRL_REG1_ACTIVE               (1 << 1)
#  define CTRL_REG1_READY                (1 << 0)

#define FXAS21002C_CTRL_REG2             0x14
#  define CTRL_REG2_INT_CFG_FIFO         (1 << 7)
#  define CTRL_REG2_INT_EN_FIFO          (1 << 6)
#  define CTRL_REG2_INT_CFG_RT           (1 << 5)
#  define CTRL_REG2_INT_EN_RT            (1 << 4)
#  define CTRL_REG2_INT_CFG_DRDY         (1 << 3)
#  define CTRL_REG2_INT_EN_DRDY          (1 << 2)
#  define CTRL_REG2_IPOL                 (1 << 1)
#  define CTRL_REG2_PP_OD                (1 << 0)

#define FXAS21002C_CTRL_REG3             0x15
#  define CTRL_REG3_WRAPTOONE            (1 << 3)
#  define CTRL_REG3_EXTCTRLEN            (1 << 2)
#  define CTRL_REG3_FS_DOUBLE            (1 << 0)

#define DEF_REG(r)   {r, #r}

/* default values for this device */
#define FXAS21002C_MAX_RATE              800
#define FXAS21002C_DEFAULT_RATE          FXAS21002C_MAX_RATE
#define FXAS21002C_DEFAULT_RANGE_DPS     2000
#define FXAS21002C_DEFAULT_ONCHIP_FILTER_FREQ 	64 // ODR dependant

/*
  we set the timer interrupt to run a bit faster than the desired
  sample rate and then throw away duplicates using the data ready bit.
  This time reduction is enough to cope with worst case timing jitter
  due to other timers
  Typical reductions for the MPU6000 is 20% so 20% of 1/800 is 250 us
 */
#define FXAS21002C_TIMER_REDUCTION				250

/*
  list of registers that will be checked in check_registers(). Note
  that ADDR_WHO_AM_I must be first in the list.
 */
static constexpr uint8_t _checked_registers[] {
	FXAS21002C_WHO_AM_I,
	FXAS21002C_F_SETUP,
	FXAS21002C_CTRL_REG0,
	FXAS21002C_CTRL_REG1,
	FXAS21002C_CTRL_REG2,
	FXAS21002C_CTRL_REG3,
};

using namespace time_literals;

FXAS21002C::FXAS21002C(int bus, uint32_t device, enum Rotation rotation) :
	SPI("FXAS21002C", nullptr, bus, device, SPIDEV_MODE0, 2 * 1000 * 1000),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(this->get_device_id())),
	_px4_gyro(get_device_id(), (external() ? ORB_PRIO_VERY_HIGH : ORB_PRIO_DEFAULT), rotation),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_errors(perf_alloc(PC_COUNT, MODULE_NAME": err")),
	_bad_registers(perf_alloc(PC_COUNT, MODULE_NAME": bad register")),
	_duplicates(perf_alloc(PC_COUNT, MODULE_NAME": duplicate reading"))
{
	_px4_gyro.set_device_type(DRV_GYR_DEVTYPE_FXAS2100C);
}

FXAS21002C::~FXAS21002C()
{
	/* make sure we are truly inactive */
	stop();

	/* delete the perf counter */
	perf_free(_sample_perf);
	perf_free(_errors);
	perf_free(_bad_registers);
	perf_free(_duplicates);
}

int
FXAS21002C::init()
{
	/* do SPI init (and probe) first */
	if (SPI::init() != OK) {
		PX4_ERR("SPI init failed");
		return PX4_ERROR;
	}

	reset();

	start();

	return PX4_OK;
}

void
FXAS21002C::reset()
{
	/* write 0 0 0 000 00 = 0x00 to CTRL_REG1 to place FXOS21002 in Standby
	 * [6]: RST=0
	 * [5]: ST=0 self test disabled
	 * [4-2]: DR[2-0]=000 for 200Hz ODR
	 * [1-0]: Active=0, Ready=0 for Standby mode
	 */
	write_reg(FXAS21002C_CTRL_REG1, 0);

	/* write 0000 0000 = 0x00 to CTRL_REG0 to configure range and filters
	 * [7-6]: BW[1-0]=00, LPF 64 @ 800Hz ODR
	 *  [5]: SPIW=0 4 wire SPI
	 * [4-3]: SEL[1-0]=00 for 10Hz HPF at 200Hz ODR
	 *  [2]: HPF_EN=0 disable HPF
	 * [1-0]: FS[1-0]=00 for 1600dps (TBD CHANGE TO 2000dps when final trimmed parts available)
	 */
	write_checked_reg(FXAS21002C_CTRL_REG0, CTRL_REG0_BW_LOW | CTRL_REG0_FS_2000_DPS);

	/* write CTRL_REG1 to configure 800Hz ODR and enter Active mode */
	write_checked_reg(FXAS21002C_CTRL_REG1, CTRL_REG1_DR_800HZ | CTRL_REG1_ACTIVE);

	/* Set the default */
	set_samplerate(FXAS21002C_DEFAULT_RATE);
	set_range(FXAS21002C_DEFAULT_RANGE_DPS);
	set_onchip_lowpass_filter(FXAS21002C_DEFAULT_ONCHIP_FILTER_FREQ);

	_read = 0;
}

int
FXAS21002C::probe()
{
	/* verify that the device is attached and functioning */
	bool success = (read_reg(FXAS21002C_WHO_AM_I) == WHO_AM_I);

	if (success) {
		_checked_values[0] = WHO_AM_I;
		return OK;
	}

	return -EIO;
}

uint8_t
FXAS21002C::read_reg(unsigned reg)
{
	uint8_t cmd[2];

	cmd[0] = DIR_READ(reg);
	cmd[1] = 0;

	transfer(cmd, cmd, sizeof(cmd));

	return cmd[1];
}

void
FXAS21002C::write_reg(unsigned reg, uint8_t value)
{
	uint8_t cmd[2];

	cmd[0] = DIR_WRITE(reg);
	cmd[1] = value;

	transfer(cmd, nullptr, sizeof(cmd));
}

void
FXAS21002C::write_checked_reg(unsigned reg, uint8_t value)
{
	write_reg(reg, value);

	for (uint8_t i = 0; i < FXAS21002C_NUM_CHECKED_REGISTERS; i++) {
		if (reg == _checked_registers[i]) {
			_checked_values[i] = value;
		}
	}
}

void
FXAS21002C::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_checked_reg(reg, val);
}

int
FXAS21002C::set_range(unsigned max_dps)
{
	uint8_t bits = CTRL_REG0_FS_250_DPS;
	float new_range_scale_dps_digit;

	if (max_dps == 0) {
		max_dps = 2000;
	}

	if (max_dps <= 250) {
		//new_range = 250;
		new_range_scale_dps_digit = 7.8125e-3f;
		bits = CTRL_REG0_FS_250_DPS;

	} else if (max_dps <= 500) {
		//new_range = 500;
		new_range_scale_dps_digit = 15.625e-3f;
		bits = CTRL_REG0_FS_500_DPS;

	} else if (max_dps <= 1000) {
		//new_range = 1000;
		new_range_scale_dps_digit = 31.25e-3f;
		bits = CTRL_REG0_FS_1000_DPS;

	} else if (max_dps <= 2000) {
		//new_range = 2000;
		new_range_scale_dps_digit = 62.5e-3f;
		bits = CTRL_REG0_FS_2000_DPS;

	} else {
		return -EINVAL;
	}

	set_standby(_current_rate, true);

	_px4_gyro.set_scale(new_range_scale_dps_digit / 180.0f * M_PI_F);

	modify_reg(FXAS21002C_CTRL_REG0, CTRL_REG0_FS_MASK, bits);
	set_standby(_current_rate, false);

	return OK;
}

void
FXAS21002C::set_standby(int rate, bool standby_true)
{
	uint8_t c = 0;
	uint8_t s = 0;

	if (standby_true) {
		c = CTRL_REG1_ACTIVE | CTRL_REG1_READY;

	} else {
		s = CTRL_REG1_ACTIVE | CTRL_REG1_READY;
	}

	modify_reg(FXAS21002C_CTRL_REG1, c, s);

	// From the data sheet

	int wait_ms = (1000 / rate) + 60 + 1;

	usleep(wait_ms * 1000);
}

int
FXAS21002C::set_samplerate(unsigned frequency)
{
	uint8_t bits = 0;

	unsigned last_rate = _current_rate;

	if (frequency == 0) {
		frequency = FXAS21002C_DEFAULT_RATE;
	}

	if (frequency <= 13) {
		_current_rate = 13;
		bits = CTRL_REG1_DR_12_5;

	} else if (frequency <= 25) {
		_current_rate = 25;
		bits = CTRL_REG1_DR_25HZ;

	} else if (frequency <= 50) {
		_current_rate = 50;
		bits = CTRL_REG1_DR_50HZ;

	} else if (frequency <= 100) {
		_current_rate = 100;
		bits = CTRL_REG1_DR_100HZ;

	} else if (frequency <= 200) {
		_current_rate = 200;
		bits = CTRL_REG1_DR_200HZ;

	} else if (frequency <= 400) {
		_current_rate = 400;
		bits = CTRL_REG1_DR_400HZ;

	} else if (frequency <= 800) {
		_current_rate = 800;
		bits = CTRL_REG1_DR_800HZ;

	} else {
		return -EINVAL;
	}

	set_standby(last_rate, true);
	modify_reg(FXAS21002C_CTRL_REG1, CTRL_REG1_DR_MASK, bits);
	set_standby(_current_rate, false);

	return OK;
}

void
FXAS21002C::set_onchip_lowpass_filter(int frequency_hz)
{
	int high = 256 / (800 / _current_rate);
	int med = high / 2 ;
	int low = med / 2;

	if (_current_rate <= 25) {
		low = -1;
	}

	if (_current_rate == 13) {
		med = -1;
		low = -1;
	}

	uint8_t filter = CTRL_REG0_BW_HIGH;

	if (frequency_hz == 0) {
		filter = CTRL_REG0_BW_HIGH;

	} else if (frequency_hz <= low) {
		filter = CTRL_REG0_BW_LOW;

	} else if (frequency_hz <= med) {
		filter = CTRL_REG0_BW_MED;

	} else if (frequency_hz <= high) {
		filter = CTRL_REG0_BW_HIGH;
	}

	set_standby(_current_rate, true);
	modify_reg(FXAS21002C_CTRL_REG1, CTRL_REG0_BW_MASK, filter);
	set_standby(_current_rate, false);
}

void
FXAS21002C::start()
{
	/* make sure we are stopped first */
	stop();

	/* start polling at the specified rate */
	ScheduleOnInterval((1_s / FXAS21002C_DEFAULT_RATE) - FXAS21002C_TIMER_REDUCTION, 10000);
}

void
FXAS21002C::stop()
{
	ScheduleClear();
}

void
FXAS21002C::Run()
{
	/* make another measurement */
	measure();
}

void
FXAS21002C::check_registers(void)
{
	uint8_t v;

	if ((v = read_reg(_checked_registers[_checked_next])) != _checked_values[_checked_next]) {
		/*
		  if we get the wrong value then we know the SPI bus
		  or sensor is very sick. We set _register_wait to 20
		  and wait until we have seen 20 good values in a row
		  before we consider the sensor to be OK again.
		 */
		perf_count(_bad_registers);

		/*
		  try to fix the bad register value. We only try to
		  fix one per loop to prevent a bad sensor hogging the
		  bus. We skip zero as that is the WHO_AM_I, which
		  is not writeable
		 */
		if (_checked_next != 0) {
			write_reg(_checked_registers[_checked_next], _checked_values[_checked_next]);
		}

		_register_wait = 20;
	}

	_checked_next = (_checked_next + 1) % FXAS21002C_NUM_CHECKED_REGISTERS;
}

void
FXAS21002C::measure()
{
	// start the performance counter
	perf_begin(_sample_perf);

	/* status register and data as read back from the device */
#pragma pack(push, 1)
	struct {
		uint8_t		cmd;
		uint8_t		status;
		int16_t		x;
		int16_t		y;
		int16_t		z;
	} raw_gyro_report{};
#pragma pack(pop)

	check_registers();

	if (_register_wait != 0) {
		// we are waiting for some good transfers before using
		// the sensor again.
		_register_wait--;
		perf_end(_sample_perf);
		return;
	}

	/* fetch data from the sensor */
	raw_gyro_report.cmd = DIR_READ(FXAS21002C_STATUS);
	const hrt_abstime timestamp_sample = hrt_absolute_time();
	transfer((uint8_t *)&raw_gyro_report, (uint8_t *)&raw_gyro_report, sizeof(raw_gyro_report));

	if (!(raw_gyro_report.status & DR_STATUS_ZYXDR)) {
		perf_end(_sample_perf);
		perf_count(_duplicates);
		return;
	}

	/*
	 * The TEMP register contains an 8-bit 2's complement temperature value with a range
	 * of –128 °C to +127 °C and a scaling of 1 °C/LSB. The temperature data is only
	 * compensated (factory trim values applied) when the device is operating in the Active
	 * mode and actively measuring the angular rate.
	 */
	if ((_read % _current_rate) == 0) {
		const float temperature = read_reg(FXAS21002C_TEMP) * 1.0f;
		_px4_gyro.set_temperature(temperature);
	}

	// report the error count as the number of bad
	// register reads. This allows the higher level
	// code to decide if it should use this sensor based on
	// whether it has had failures
	_px4_gyro.set_error_count(perf_event_count(_bad_registers));

	int16_t x_raw = swap16(raw_gyro_report.x);
	int16_t y_raw = swap16(raw_gyro_report.y);
	int16_t z_raw = swap16(raw_gyro_report.z);

	_px4_gyro.update(timestamp_sample, x_raw, y_raw, z_raw);

	_read++;

	/* stop the perf counter */
	perf_end(_sample_perf);
}

void
FXAS21002C::print_info()
{
	printf("gyro reads:          %u\n", _read);
	perf_print_counter(_sample_perf);
	perf_print_counter(_errors);
	perf_print_counter(_bad_registers);
	perf_print_counter(_duplicates);
	::printf("checked_next: %u\n", _checked_next);

	for (uint8_t i = 0; i < FXAS21002C_NUM_CHECKED_REGISTERS; i++) {
		uint8_t v = read_reg(_checked_registers[i]);

		if (v != _checked_values[i]) {
			::printf("reg %02x:%02x should be %02x\n",
				 (unsigned)_checked_registers[i],
				 (unsigned)v,
				 (unsigned)_checked_values[i]);
		}
	}

	_px4_gyro.print_status();
}

void
FXAS21002C::print_registers()
{
	const struct {
		uint8_t reg;
		const char *name;
	} regmap[] = {
		DEF_REG(FXAS21002C_STATUS),
		DEF_REG(FXAS21002C_OUT_X_MSB),
		DEF_REG(FXAS21002C_OUT_X_LSB),
		DEF_REG(FXAS21002C_OUT_Y_MSB),
		DEF_REG(FXAS21002C_OUT_Y_LSB),
		DEF_REG(FXAS21002C_OUT_Z_MSB),
		DEF_REG(FXAS21002C_OUT_Z_LSB),
		DEF_REG(FXAS21002C_DR_STATUS),
		DEF_REG(FXAS21002C_F_STATUS),
		DEF_REG(FXAS21002C_F_SETUP),
		DEF_REG(FXAS21002C_F_EVENT),
		DEF_REG(FXAS21002C_INT_SRC_FLAG),
		DEF_REG(FXAS21002C_WHO_AM_I),
		DEF_REG(FXAS21002C_CTRL_REG0),
		DEF_REG(FXAS21002C_RT_CFG),
		DEF_REG(FXAS21002C_RT_SRC),
		DEF_REG(FXAS21002C_RT_THS),
		DEF_REG(FXAS21002C_RT_COUNT),
		DEF_REG(FXAS21002C_TEMP),
		DEF_REG(FXAS21002C_CTRL_REG1),
		DEF_REG(FXAS21002C_CTRL_REG2),
		DEF_REG(FXAS21002C_CTRL_REG3),
	};

	for (uint8_t i = 0; i < sizeof(regmap) / sizeof(regmap[0]); i++) {
		printf("0x%02x %d:%s\n", read_reg(regmap[i].reg), regmap[i].reg, regmap[i].name);
	}
}

void
FXAS21002C::test_error()
{
	// trigger an error
	write_reg(FXAS21002C_CTRL_REG1, 0);
}
