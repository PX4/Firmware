/****************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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
 * @file pca9685.cpp
 *
 * Driver for the PCA9685 I2C PWM module
 * The chip is used on the Adafruit I2C/PWM converter https://www.adafruit.com/product/815
 *
 * Parts of the code are adapted from the arduino library for the board
 * https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
 * for the license of these parts see the
 * arduino_Adafruit_PWM_Servo_Driver_Library_license.txt file
 * see https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library for contributors
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <ctype.h>
#include <math.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <nuttx/clock.h>

#include <perf/perf_counter.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>

#include <board_config.h>

#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4

#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE

#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9

#define ALLLED_ON_L 0xFA
#define ALLLED_ON_H 0xFB
#define ALLLED_OFF_L 0xFC
#define ALLLED_OF

#define ADDR 0x40	// I2C adress

#define PCA9685_PWMFREQ 60.0f
#define PCA9685_NCHANS 16 // total amount of pwm outputs

#define PCA9685_PWMMIN 150 // this is the 'minimum' pulse length count (out of 4096)
#define PCA9685_PWMMAX 600 // this is the 'maximum' pulse length count (out of 4096)_PWMFREQ 60.0f

#define PCA9685_PWMCENTER ((PCA9685_PWMMAX + PCA9685_PWMMIN)/2)
#define PCA9685_MAXSERVODEG 90.0f /* maximal servo deflection in degrees
				     PCA9685_PWMMIN <--> -PCA9685_MAXSERVODEG
				     PCA9685_PWMMAX <--> PCA9685_MAXSERVODEG
				     */
#define PCA9685_SCALE ((PCA9685_PWMMAX - PCA9685_PWMCENTER)/(M_DEG_TO_RAD_F * PCA9685_MAXSERVODEG)) // scales from rad to PWM

enum IOX_MODE {
	IOX_MODE_ON,
	IOX_MODE_TEST_OUT
};

using namespace time_literals;

class PCA9685 : public device::I2C, public I2CSPIDriver<PCA9685>
{
public:
	PCA9685(I2CSPIBusOption bus_option, int bus, int bus_frequency);
	~PCA9685() override = default;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	void print_status();

	int		init() override;
	int		reset();

	void RunImpl();

protected:
	void custom_method(const BusCLIArguments &cli) override;
private:

	enum IOX_MODE		_mode;
	uint64_t			_i2cpwm_interval;
	perf_counter_t		_comms_errors;

	uint8_t			_msg[6];

	int			_actuator_controls_sub;
	struct actuator_controls_s  _actuator_controls;
	uint16_t	    	_current_values[actuator_controls_s::NUM_ACTUATOR_CONTROLS]; /**< stores the current pwm output
										  values as sent to the setPin() */

	bool _mode_on_initialized;  /** Set to true after the first call of i2cpwm in mode IOX_MODE_ON */


	/**
	 * Helper function to set the pwm frequency
	 */
	int setPWMFreq(float freq);

	/**
	 * Helper function to set the demanded pwm value
	 * @param num pwm output number
	 */
	int setPWM(uint8_t num, uint16_t on, uint16_t off);

	/**
	 * Sets pin without having to deal with on/off tick placement and properly handles
	 * a zero value as completely off.  Optional invert parameter supports inverting
	 * the pulse for sinking to ground.
	 * @param num pwm output number
	 * @param val should be a value from 0 to 4095 inclusive.
	 */
	int setPin(uint8_t num, uint16_t val, bool invert = false);


	/* Wrapper to read a byte from addr */
	int read8(uint8_t addr, uint8_t &value);

	/* Wrapper to wite a byte to addr */
	int write8(uint8_t addr, uint8_t value);

};

PCA9685::PCA9685(I2CSPIBusOption bus_option, int bus, int bus_frequency) :
	I2C(DRV_PWM_DEVTYPE_PCA9685, MODULE_NAME, bus, ADDR, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_mode(IOX_MODE_ON),
	_i2cpwm_interval(1_s / 60.0f),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err")),
	_actuator_controls_sub(-1),
	_actuator_controls(),
	_mode_on_initialized(false)
{
	memset(_msg, 0, sizeof(_msg));
	memset(_current_values, 0, sizeof(_current_values));
}

int
PCA9685::init()
{
	int ret;
	ret = I2C::init();

	if (ret != OK) {
		return ret;
	}

	ret = reset();

	if (ret != OK) {
		return ret;
	}

	ret = setPWMFreq(PCA9685_PWMFREQ);

	if (ret == 0) {
		ScheduleNow();
	}

	return ret;
}

void
PCA9685::print_status()
{
	I2CSPIDriverBase::print_status();
	PX4_INFO("Mode: %u", _mode);
}

void
PCA9685::RunImpl()
{
	if (_mode == IOX_MODE_TEST_OUT) {
		setPin(0, PCA9685_PWMCENTER);

	} else {
		if (!_mode_on_initialized) {
			/* Subscribe to actuator control 2 (payload group for gimbal) */
			_actuator_controls_sub = orb_subscribe(ORB_ID(actuator_controls_2));
			/* set the uorb update interval lower than the driver pwm interval */
			orb_set_interval(_actuator_controls_sub, 1000.0f / PCA9685_PWMFREQ - 5);

			_mode_on_initialized = true;
		}

		/* Read the servo setpoints from the actuator control topics (gimbal) */
		bool updated;
		orb_check(_actuator_controls_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(actuator_controls_2), _actuator_controls_sub, &_actuator_controls);

			for (int i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROLS; i++) {
				/* Scale the controls to PWM, first multiply by pi to get rad,
				 * the control[i] values are on the range -1 ... 1 */
				uint16_t new_value = PCA9685_PWMCENTER +
						     (_actuator_controls.control[i] * M_PI_F * PCA9685_SCALE);
				DEVICE_DEBUG("%d: current: %u, new %u, control %.2f", i, _current_values[i], new_value,
					     (double)_actuator_controls.control[i]);

				if (new_value != _current_values[i] &&
				    new_value >= PCA9685_PWMMIN &&
				    new_value <= PCA9685_PWMMAX) {
					/* This value was updated, send the command to adjust the PWM value */
					setPin(i, new_value);
					_current_values[i] = new_value;
				}
			}
		}
	}

	ScheduleDelayed(_i2cpwm_interval);
}

int
PCA9685::setPWM(uint8_t num, uint16_t on, uint16_t off)
{
	int ret;
	/* convert to correct message */
	_msg[0] = LED0_ON_L + 4 * num;
	_msg[1] = on;
	_msg[2] = on >> 8;
	_msg[3] = off;
	_msg[4] = off >> 8;

	/* try i2c transfer */
	ret = transfer(_msg, 5, nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
		DEVICE_LOG("i2c::transfer returned %d", ret);
	}

	return ret;
}

int
PCA9685::setPin(uint8_t num, uint16_t val, bool invert)
{
	// Clamp value between 0 and 4095 inclusive.
	if (val > 4095) {
		val = 4095;
	}

	if (invert) {
		if (val == 0) {
			// Special value for signal fully on.
			return setPWM(num, 4096, 0);

		} else if (val == 4095) {
			// Special value for signal fully off.
			return setPWM(num, 0, 4096);

		} else {
			return setPWM(num, 0, 4095 - val);
		}

	} else {
		if (val == 4095) {
			// Special value for signal fully on.
			return setPWM(num, 4096, 0);

		} else if (val == 0) {
			// Special value for signal fully off.
			return setPWM(num, 0, 4096);

		} else {
			return setPWM(num, 0, val);
		}
	}

	return PX4_ERROR;
}

int
PCA9685::setPWMFreq(float freq)
{
	int ret  = OK;
	freq *= 0.9f;  /* Correct for overshoot in the frequency setting (see issue
		https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/issues/11). */
	float prescaleval = 25000000;
	prescaleval /= 4096;
	prescaleval /= freq;
	prescaleval -= 1;
	uint8_t prescale = uint8_t(prescaleval + 0.5f); //implicit floor()
	uint8_t oldmode;
	ret = read8(PCA9685_MODE1, oldmode);

	if (ret != OK) {
		return ret;
	}

	uint8_t newmode = (oldmode & 0x7F) | 0x10; // sleep

	ret = write8(PCA9685_MODE1, newmode); // go to sleep

	if (ret != OK) {
		return ret;
	}

	ret = write8(PCA9685_PRESCALE, prescale); // set the prescaler

	if (ret != OK) {
		return ret;
	}

	ret = write8(PCA9685_MODE1, oldmode);

	if (ret != OK) {
		return ret;
	}

	usleep(5000); //5ms delay (from arduino driver)

	ret = write8(PCA9685_MODE1, oldmode | 0xa1);  //  This sets the MODE1 register to turn on auto increment.

	if (ret != OK) {
		return ret;
	}

	return ret;
}

/* Wrapper to read a byte from addr */
int
PCA9685::read8(uint8_t addr, uint8_t &value)
{
	int ret = OK;

	/* send addr */
	ret = transfer(&addr, sizeof(addr), nullptr, 0);

	if (ret != OK) {
		goto fail_read;
	}

	/* get value */
	ret = transfer(nullptr, 0, &value, 1);

	if (ret != OK) {
		goto fail_read;
	}

	return ret;

fail_read:
	perf_count(_comms_errors);
	DEVICE_LOG("i2c::transfer returned %d", ret);

	return ret;
}

int PCA9685::reset(void)
{
	warnx("resetting");
	return write8(PCA9685_MODE1, 0x0);
}

/* Wrapper to wite a byte to addr */
int
PCA9685::write8(uint8_t addr, uint8_t value)
{
	int ret = OK;
	_msg[0] = addr;
	_msg[1] = value;
	/* send addr and value */
	ret = transfer(_msg, 2, nullptr, 0);

	if (ret != OK) {
		perf_count(_comms_errors);
		DEVICE_LOG("i2c::transfer returned %d", ret);
	}

	return ret;
}

void
PCA9685::print_usage()
{
	PRINT_MODULE_USAGE_NAME("pca9685", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_COMMAND("reset");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "enter test mode");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

I2CSPIDriverBase *PCA9685::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
				       int runtime_instance)
{
	PCA9685 *instance = new PCA9685(iterator.configuredBusOption(), iterator.bus(), cli.bus_frequency);

	if (!instance) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (OK != instance->init()) {
		delete instance;
		return nullptr;
	}

	return instance;
}

void PCA9685::custom_method(const BusCLIArguments &cli)
{
	switch (cli.custom1) {
	case 0: reset(); break;

	case 1: _mode = IOX_MODE_TEST_OUT; break;
	}
}

extern "C" int pca9685_main(int argc, char *argv[])
{
	using ThisDriver = PCA9685;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 100000;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_PWM_DEVTYPE_PCA9685);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	if (!strcmp(verb, "reset")) {
		cli.custom1 = 0;
		return ThisDriver::module_custom_method(cli, iterator);
	}

	if (!strcmp(verb, "test")) {
		cli.custom1 = 1;
		return ThisDriver::module_custom_method(cli, iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
