/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file mb12xx.cpp
 * @author Greg Hulands
 * @author Jon Verbeke <jon.verbeke@kuleuven.be>
 *
 * Driver for the Maxbotix sonar range finders connected via I2C.
 */

#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <semaphore.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <board_config.h>
#include <containers/Array.hpp>
#include <drivers/device/device.h>
#include <drivers/device/i2c.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>

using namespace time_literals;

/* Configuration Constants */
#define MB12XX_BASE_ADDR                        0x70   // 7-bit address is 0x70 = 112. 8-bit address is 0xE0 = 224.
#define MB12XX_MIN_ADDR                         0x5A   // 7-bit address is 0x5A = 90.  8-bit address is 0xB4 = 180.
#define MB12XX_BUS_SPEED                        100000 // 100kHz bus speed.

/* MB12xx Registers addresses */
#define MB12XX_TAKE_RANGE_REG                   0x51 // Measure range Register.
#define MB12XX_SET_ADDRESS_1                    0xAA // Change address 1 Register.
#define MB12XX_SET_ADDRESS_2                    0xA5 // Change address 2 Register.

/* Device limits */
#define MB12XX_MIN_DISTANCE                     (0.20f)
#define MB12XX_MAX_DISTANCE                     (7.65f)

#define MB12XX_MEASURE_INTERVAL                 100_ms // 60ms minimum for one sonar.
#define MB12XX_INTERVAL_BETWEEN_SUCCESIVE_FIRES 100_ms // 30ms minimum between each sonar measurement (watch out for interference!).

class MB12XX : public device::I2C, public ModuleParams, public I2CSPIDriver<MB12XX>
{
public:
	MB12XX(I2CSPIBusOption bus_option, const int bus, int bus_frequency, int address);
	virtual ~MB12XX();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	virtual int init() override;

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_status() override;

	/**
	 * Sets a new device address.
	 * @param address The new sensor address to be set: 200-224 even addresses only.
	 * @return Returns PX4_OK iff successful, PX4_ERROR otherwise.
	 */
	int set_address(const uint8_t address = MB12XX_BASE_ADDR);

	/**
	 * Initialise the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void start();

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 */
	void RunImpl();

protected:
	void custom_method(const BusCLIArguments &cli) override;

private:

	/**
	 * Collects the most recent sensor measurement data from the i2c bus.
	 */
	int collect();

	/**
	 * Gets the current sensor rotation value.
	 */
	int get_sensor_rotation(const size_t index);

	/**
	 * Sends an i2c measure command to start the next sonar ping.
	 */
	int measure();

	px4::Array<uint8_t, RANGE_FINDER_MAX_SENSORS> _sensor_addresses {};
	px4::Array<uint8_t, RANGE_FINDER_MAX_SENSORS> _sensor_rotations {};

	int _measure_interval{MB12XX_MEASURE_INTERVAL};	// Initialize the measure interval for a single sensor.

	int _sensor_index{0};	// Initialize counter for cycling i2c adresses to zero.

	int _sensor_count{0};

	orb_advert_t _distance_sensor_topic{nullptr};

	perf_counter_t _comms_error{perf_alloc(PC_ELAPSED, "mb12xx_comms_error")};
	perf_counter_t _sample_perf{perf_alloc(PC_COUNT, "mb12xx_sample_perf")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SENS_EN_MB12XX>)   _p_sensor_enabled,
		(ParamInt<px4::params::SENS_MB12_0_ROT>)  _p_sensor0_rot,
		(ParamInt<px4::params::SENS_MB12_1_ROT>)  _p_sensor1_rot,
		(ParamInt<px4::params::SENS_MB12_2_ROT>)  _p_sensor2_rot,
		(ParamInt<px4::params::SENS_MB12_3_ROT>)  _p_sensor3_rot,
		(ParamInt<px4::params::SENS_MB12_4_ROT>)  _p_sensor4_rot,
		(ParamInt<px4::params::SENS_MB12_5_ROT>)  _p_sensor5_rot,
		(ParamInt<px4::params::SENS_MB12_6_ROT>)  _p_sensor6_rot,
		(ParamInt<px4::params::SENS_MB12_7_ROT>)  _p_sensor7_rot,
		(ParamInt<px4::params::SENS_MB12_8_ROT>)  _p_sensor8_rot,
		(ParamInt<px4::params::SENS_MB12_9_ROT>)  _p_sensor9_rot,
		(ParamInt<px4::params::SENS_MB12_10_ROT>) _p_sensor10_rot,
		(ParamInt<px4::params::SENS_MB12_11_ROT>) _p_sensor11_rot
	);
};

MB12XX::MB12XX(I2CSPIBusOption bus_option, const int bus, int bus_frequency, int address) :
	I2C(DRV_DIST_DEVTYPE_MB12XX, MODULE_NAME, bus, address, bus_frequency),
	ModuleParams(nullptr),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus, address)
{
}

MB12XX::~MB12XX()
{
	// Unadvertise the distance sensor topic.
	if (_distance_sensor_topic != nullptr) {
		orb_unadvertise(_distance_sensor_topic);
	}

	// Free perf counters.
	perf_free(_comms_error);
	perf_free(_sample_perf);
}

int
MB12XX::collect()
{
	perf_begin(_sample_perf);
	uint8_t val[2] = {};

	// Increment i2c adress to next sensor.
	_sensor_index++;
	_sensor_index %= _sensor_count;

	// Set the sensor i2c adress for the active cycle.
	set_device_address(_sensor_addresses[_sensor_index]);

	// Transfer data from the bus.
	int ret_val = transfer(nullptr, 0, &val[0], 2);

	if (ret_val < 0) {
		PX4_ERR("sensor %i read failed, address: 0x%02X", _sensor_index, get_device_address());
		perf_count(_comms_error);
		perf_end(_sample_perf);
		return ret_val;
	}

	uint16_t distance_cm = val[0] << 8 | val[1];
	float distance_m = static_cast<float>(distance_cm) * 1e-2f;

	distance_sensor_s report;
	report.current_distance = distance_m;
	report.id               = _sensor_addresses[_sensor_index];
	report.max_distance     = MB12XX_MAX_DISTANCE;
	report.min_distance     = MB12XX_MIN_DISTANCE;
	report.orientation      = _sensor_rotations[_sensor_index];
	report.signal_quality   = -1;
	report.timestamp        = hrt_absolute_time();
	report.type             = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;
	report.variance         = 0.0f;

	int instance_id;
	orb_publish_auto(ORB_ID(distance_sensor), &_distance_sensor_topic, &report, &instance_id);

	// Begin the next measurement.
	if (measure() != PX4_OK) {
		PX4_INFO("sensor %i measurement error, address 0x%02X", _sensor_index, get_device_address());
		perf_count(_comms_error);
		perf_end(_sample_perf);
		return ret_val;
	}

	perf_end(_sample_perf);
	return PX4_OK;
}

int
MB12XX::get_sensor_rotation(const size_t index)
{
	switch (index) {
	case 0: return _p_sensor0_rot.get();

	case 1: return _p_sensor1_rot.get();

	case 2: return _p_sensor2_rot.get();

	case 3: return _p_sensor3_rot.get();

	case 4: return _p_sensor4_rot.get();

	case 5: return _p_sensor5_rot.get();

	case 6: return _p_sensor6_rot.get();

	case 7: return _p_sensor7_rot.get();

	case 8: return _p_sensor8_rot.get();

	case 9: return _p_sensor9_rot.get();

	case 10: return _p_sensor10_rot.get();

	case 11: return _p_sensor11_rot.get();

	default: return PX4_ERROR;
	}
}

int
MB12XX::init()
{
	if (_p_sensor_enabled.get() == 0) {
		PX4_WARN("disabled");
		return PX4_ERROR;
	}

	// Initialize the I2C device
	if (I2C::init() != OK) {
		return PX4_ERROR;
	}

	// Check for connected rangefinders on each i2c port by decrementing from the base address,
	// (MB12XX_BASE_ADDR = 112, MB12XX_MIN_ADDR = 90).
	for (uint8_t address = MB12XX_BASE_ADDR; address > MB12XX_MIN_ADDR ; address--) {
		set_device_address(address);

		if (measure() == PX4_OK) {
			// Store I2C address
			_sensor_addresses[_sensor_count] = address;
			_sensor_rotations[_sensor_count] = get_sensor_rotation(_sensor_count);
			_sensor_count++;

			PX4_INFO("sensor %i at address 0x%02X added", _sensor_count, get_device_address());

			if (_sensor_count >= RANGE_FINDER_MAX_SENSORS) {
				break;
			}

			px4_usleep(_measure_interval);
		}
	}

	// Return an error if no sensors were detected.
	if (_sensor_count == 0) {
		PX4_ERR("no sensors discovered");
		return PX4_ERROR;
	}

	// If more than one sonar is detected, adjust the meaure interval to avoid sensor interference.
	if (_sensor_count > 1) {
		_measure_interval = MB12XX_INTERVAL_BETWEEN_SUCCESIVE_FIRES;
	}

	PX4_INFO("Total sensors connected: %i", _sensor_count);
	return PX4_OK;
}

int
MB12XX::measure()
{
	// Send the command to take a measurement.
	uint8_t cmd = MB12XX_TAKE_RANGE_REG;
	int ret_val = transfer(&cmd, 1, nullptr, 0);

	return ret_val;
}

void
MB12XX::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_error);
	PX4_INFO("poll interval:  %ums", _measure_interval / 1000);

	for (int i = 0; i < _sensor_count; i++) {
		PX4_INFO("sensor: %i, address %u", i, _sensor_addresses[i]);
	}
}

void
MB12XX::RunImpl()
{
	// Collect the sensor data.
	if (collect() != PX4_OK) {
		PX4_INFO("collection error");
	}
}

int
MB12XX::set_address(const uint8_t address)
{
	if (_sensor_count > 1) {
		PX4_INFO("multiple sensors are connected");
		return PX4_ERROR;
	}

	if (address < 2    ||
	    address == 80  ||
	    address == 164 ||
	    address == 170 ||
	    address > 224) {
		PX4_ERR("incompatible address requested");
		return PX4_ERROR;
	}

	PX4_INFO("requested address: %u", address);

	uint8_t shifted_address = (address << 1);
	uint8_t cmd[3] = {MB12XX_SET_ADDRESS_1, MB12XX_SET_ADDRESS_2, shifted_address};

	if (transfer(cmd, sizeof(cmd), nullptr, 0) != PX4_OK) {
		PX4_INFO("could not set the address");
	}

	set_device_address(address);
	PX4_INFO("device address: %u", get_device_address());
	return PX4_OK;
}

void
MB12XX::start()
{
	// Fetch parameter values.
	ModuleParams::updateParams();

	// Schedule the driver cycle at regular intervals.
	ScheduleOnInterval(_measure_interval);
}

void
MB12XX::custom_method(const BusCLIArguments &cli)
{
	set_address(cli.i2c_address);
}

I2CSPIDriverBase *MB12XX::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
				      int runtime_instance)
{
	MB12XX *instance = new MB12XX(iterator.configuredBusOption(), iterator.bus(), cli.bus_frequency, cli.i2c_address);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (instance->init() != PX4_OK) {
		delete instance;
		return nullptr;
	}

	instance->start();
	return instance;
}


void
MB12XX::print_usage()
{
	PRINT_MODULE_USAGE_NAME("mb12xx", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x70);
	PRINT_MODULE_USAGE_COMMAND("set_address");
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x70);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int mb12xx_main(int argc, char *argv[])
{
	using ThisDriver = MB12XX;
	BusCLIArguments cli{true, false};
	cli.i2c_address = MB12XX_BASE_ADDR;
	cli.default_i2c_frequency = MB12XX_BUS_SPEED;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_DIST_DEVTYPE_MB12XX);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	if (!strcmp(verb, "set_address")) {
		return ThisDriver::module_custom_method(cli, iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
