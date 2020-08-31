/****************************************************************************
 *
 *   Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
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
 * @file MappyDot.cpp
 * @author Mohammed Kabir (mhkabir@mit.edu)
 * @author Mark Sauder (mcsauder@gmail.com)
 *
 * Driver for Mappydot infrared rangefinders connected via I2C.
 */

#include <string.h>

#include <containers/Array.hpp>
#include <drivers/device/i2c.h>
#include <drivers/drv_range_finder.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>
#include <uORB/topics/distance_sensor.h>

/* MappyDot Registers */
/* Basics */
#define MAPPYDOT_MEASUREMENT_BUDGET                         0x42
#define MAPPYDOT_READ_ERROR_CODE                            0x45
#define MAPPYDOT_CHECK_INTERRUPT                            0x49
#define MAPPYDOT_READ_ACCURACY                              0x52
#define MAPPYDOT_PERFORM_SINGLE_RANGE                       0x53
#define MAPPYDOT_SET_CONTINUOUS_RANGING_MODE                0x63
#define MAPPYDOT_RANGING_MEASUREMENT_MODE                   0x6D
#define MAPPYDOT_READ_DISTANCE                              0x72
#define MAPPYDOT_SET_SINGLE_RANGING_MODE                    0x73

/* Configuration */
#define MAPPYDOT_FILTERING_ENABLE                           0x46
#define MAPPYDOT_SIGNAL_LIMIT_CHECK_VALUE                   0x47
#define MAPPYDOT_ENABLE_CROSSTALK_COMPENSATION              0x4B
#define MAPPYDOT_SIGMA_LIMIT_CHECK_VALUE                    0x4C
#define MAPPYDOT_INTERSENSOR_CROSSTALK_MEASUREMENT_DELAY    0x51
#define MAPPYDOT_INTERSENSOR_CROSSTALK_REDUCTION_ENABLE     0x54
#define MAPPYDOT_AVERAGING_ENABLE                           0x56
#define MAPPYDOT_INTERSENSOR_SYNC_ENABLE                    0x59
#define MAPPYDOT_CALIBRATE_DISTANCE_OFFSET                  0x61
#define MAPPYDOT_SET_LED_THRESHOLD_DISTANCE_IN_MM           0x65
#define MAPPYDOT_FILTERING_DISABLE                          0x66
#define MAPPYDOT_SET_GPIO_MODE                              0x67
#define MAPPYDOT_AVERAGING_SAMPLES                          0x69
#define MAPPYDOT_DISABLE_CROSSTALK_COMPENSATION             0x6B
#define MAPPYDOT_SET_LED_MODE                               0x6C
#define MAPPYDOT_SET_GPIO_THRESHOLD_DISTANCE_IN_MM          0x6F
#define MAPPYDOT_REGION_OF_INTEREST                         0x70
#define MAPPYDOT_INTERSENSOR_CROSSTALK_TIMEOUT              0x71
#define MAPPYDOT_INTERSENSOR_CROSSTALK_REDUCTION_DISABLE    0x74
#define MAPPYDOT_CALIBRATE_SPAD                             0x75
#define MAPPYDOT_AVERAGING_DISABLE                          0x76
#define MAPPYDOT_CALIBRATE_CROSSTALK                        0x78
#define MAPPYDOT_INTERSENSOR_SYNC_DISABLE                   0x79

/* Settings */
#define MAPPYDOT_FIRMWARE_VERSION                           0x4E
#define MAPPYDOT_READ_CURRENT_SETTINGS                      0x62
#define MAPPYDOT_DEVICE_NAME                                0x64
#define MAPPYDOT_NAME_DEVICE                                0x6E
#define MAPPYDOT_WRITE_CURRENT_SETTINGS_AS_START_UP_DEFAULT 0x77
#define MAPPYDOT_RESTORE_FACTORY_DEFAULTS                   0x7A

/* Advanced */
#define MAPPYDOT_AMBIENT_RATE_RETURN                        0x41
#define MAPPYDOT_VL53L1X_NOT_SHUTDOWN                       0x48
#define MAPPYDOT_SIGNAL_RATE_RETURN                         0x4A
#define MAPPYDOT_RESET_VL53L1X_RANGING                      0x58
#define MAPPYDOT_VL53L1X_SHUTDOWN                           0x68
#define MAPPYDOT_READ_NONFILTERED_VALUE                     0x6A

/* Super Advanced */
#define MAPPYDOT_ENTER_FACTORY_MODE                         0x23 //"#"//"!#!#!#"
#define MAPPYDOT_WIPE_ALL_SETTINGS                          0x3C //"<"//"><><><" (Must be in factory mode)

/* Ranging Modes */
#define MAPPYDOT_LONG_RANGE                                 0x6C
#define MAPPYDOT_MED_RANGE                                  0x6D
#define MAPPYDOT_SHORT_RANGE                                0x73

/* LED Modes */
#define MAPPYDOT_LED_OFF                                    0x66
#define MAPPYDOT_LED_MEASUREMENT_OUTPUT                     0x6D
#define MAPPYDOT_LED_ON                                     0x6F
#define MAPPYDOT_LED_PWM_ENABLED                            0x70
#define MAPPYDOT_LED_THRESHOLD_ENABLED                      0x74

/* GPIO Modes */
#define MAPPYDOT_GPIO_LOW                                   0x66
#define MAPPYDOT_GPIO_MEASUREMENT_INTERRUPT                 0x6D
#define MAPPYDOT_GPIO_HIGH                                  0x6F
#define MAPPYDOT_GPIO_PWM_ENABLED                           0x70
#define MAPPYDOT_GPIO_THRESHOLD_ENABLED                     0x74

/* I2C Bootloader */
#define MAPPYDOT_REBOOT_TO_BOOTLOADER                       0x01

/* Device limits */
#define MAPPYDOT_MIN_DISTANCE                               (0.2f) // meters
#define MAPPYDOT_MAX_DISTANCE                               (4.f) // meters

#define MAPPYDOT_BUS_CLOCK                                  400000 // 400kHz bus speed

/* Configuration Constants */
#define MAPPYDOT_BASE_ADDR                                  0x08
#define MAPPYDOT_MEASUREMENT_INTERVAL_USEC                  50000  // 50ms measurement interval, 20Hz.

using namespace time_literals;

class MappyDot : public device::I2C, public ModuleParams, public I2CSPIDriver<MappyDot>
{
public:
	MappyDot(I2CSPIBusOption bus_option, const int bus, int bus_frequency);
	virtual ~MappyDot();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	/**
	 * Initializes the sensors, advertises uORB topic,
	 * sets device addresses
	 */
	virtual int init() override;

	/**
	 * Prints basic diagnostic information about the driver.
	 */
	void print_status() override;

	/**
	 * Initializes the automatic measurement state machine and starts the driver.
	 */
	void start();

	/**
	* Performs a poll cycle; collect from the previous measurement and start a new one.
	*/
	void RunImpl();

protected:

private:

	/**
	 * Sends an i2c measure command to check for presence of a sensor.
	 */
	int probe() override;

	/**
	 * Collects the most recent sensor measurement data from the i2c bus.
	 */
	int collect();

	/**
	 * Gets the current sensor rotation value.
	 */
	int get_sensor_rotation(const size_t index);

	px4::Array<uint8_t, RANGE_FINDER_MAX_SENSORS> _sensor_addresses {};
	px4::Array<uint8_t, RANGE_FINDER_MAX_SENSORS> _sensor_rotations {};

	int _sensor_count{0};

	orb_advert_t _distance_sensor_topic{nullptr};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, "mappydot_comms_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, "mappydot_sample_perf")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SENS_EN_MPDT>)    _p_sensor_enabled,
		(ParamInt<px4::params::SENS_MPDT0_ROT>)  _p_sensor0_rot,
		(ParamInt<px4::params::SENS_MPDT1_ROT>)  _p_sensor1_rot,
		(ParamInt<px4::params::SENS_MPDT2_ROT>)  _p_sensor2_rot,
		(ParamInt<px4::params::SENS_MPDT3_ROT>)  _p_sensor3_rot,
		(ParamInt<px4::params::SENS_MPDT4_ROT>)  _p_sensor4_rot,
		(ParamInt<px4::params::SENS_MPDT5_ROT>)  _p_sensor5_rot,
		(ParamInt<px4::params::SENS_MPDT6_ROT>)  _p_sensor6_rot,
		(ParamInt<px4::params::SENS_MPDT7_ROT>)  _p_sensor7_rot,
		(ParamInt<px4::params::SENS_MPDT8_ROT>)  _p_sensor8_rot,
		(ParamInt<px4::params::SENS_MPDT9_ROT>)  _p_sensor9_rot,
		(ParamInt<px4::params::SENS_MPDT10_ROT>) _p_sensor10_rot,
		(ParamInt<px4::params::SENS_MPDT11_ROT>) _p_sensor11_rot
	);
};


MappyDot::MappyDot(I2CSPIBusOption bus_option, const int bus, int bus_frequency) :
	I2C(DRV_DIST_DEVTYPE_MAPPYDOT, MODULE_NAME, bus, MAPPYDOT_BASE_ADDR, bus_frequency),
	ModuleParams(nullptr),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus)
{}

MappyDot::~MappyDot()
{
	// Unadvertise the distance sensor topic.
	if (_distance_sensor_topic != nullptr) {
		orb_unadvertise(_distance_sensor_topic);
	}

	// Free perf counters.
	perf_free(_comms_errors);
	perf_free(_sample_perf);
}

int
MappyDot::collect()
{
	uint8_t val[2] = {};
	perf_begin(_sample_perf);

	// Increment the sensor index, (limited to the number of sensors connected).
	for (int index = 0; index < _sensor_count; index++) {

		// Set address of the current sensor to collect data from.
		set_device_address(_sensor_addresses[index]);

		// Transfer data from the bus.
		int ret_val = transfer(nullptr, 0, &val[0], 2);

		if (ret_val < 0) {
			PX4_ERR("sensor %i read failed, address: 0x%02X", index, _sensor_addresses[index]);
			perf_count(_comms_errors);
			perf_end(_sample_perf);
			return ret_val;
		}

		uint16_t distance_mm = uint16_t(val[0]) << 8 | val[1];
		float distance_m = static_cast<float>(distance_mm) / 1000.f;

		distance_sensor_s report {};
		report.current_distance = distance_m;
		report.id               = _sensor_addresses[index];
		report.max_distance     = MAPPYDOT_MAX_DISTANCE;
		report.min_distance     = MAPPYDOT_MIN_DISTANCE;
		report.orientation      = _sensor_rotations[index];
		report.signal_quality   = -1;
		report.timestamp        = hrt_absolute_time();
		report.type             = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
		report.variance         = 0;

		int instance_id;
		orb_publish_auto(ORB_ID(distance_sensor), &_distance_sensor_topic, &report, &instance_id);
	}

	perf_end(_sample_perf);
	return PX4_OK;
}

int
MappyDot::get_sensor_rotation(const size_t index)
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
MappyDot::init()
{
	if (_p_sensor_enabled.get() == 0) {
		PX4_WARN("disabled");
		return PX4_ERROR;
	}

	if (I2C::init() != PX4_OK) {
		return PX4_ERROR;
	}

	// Allow for sensor auto-addressing time
	px4_usleep(1_s);

	// Check for connected rangefinders on each i2c port,
	// starting from the base address 0x08 and incrementing
	for (int i = 0; i <= RANGE_FINDER_MAX_SENSORS; i++) {
		set_device_address(MAPPYDOT_BASE_ADDR + i);

		// Check if a sensor is present.
		if (probe() != PX4_OK) {
			break;
		}

		// Store I2C address
		_sensor_addresses[i] = MAPPYDOT_BASE_ADDR + i;
		_sensor_rotations[i] = get_sensor_rotation(i);
		_sensor_count++;

		// Configure the sensor
		// Set measurement budget
		uint16_t budget_ms = MAPPYDOT_MEASUREMENT_INTERVAL_USEC / 1000;
		uint8_t budget_cmd[3] = {MAPPYDOT_MEASUREMENT_BUDGET,
					 uint8_t(budget_ms >> 8 & 0xFF),
					 uint8_t(budget_ms & 0xFF)
					};
		transfer(&budget_cmd[0], 3, nullptr, 0);
		px4_usleep(10_ms);

		// Configure long range mode
		uint8_t range_cmd[2] = {MAPPYDOT_RANGING_MEASUREMENT_MODE,
					MAPPYDOT_LONG_RANGE
				       };
		transfer(&range_cmd[0], 2, nullptr, 0);
		px4_usleep(10_ms);

		// Configure LED threshold
		uint16_t threshold_mm = 1000; // 1m
		uint8_t threshold_cmd[3] = {MAPPYDOT_SET_LED_THRESHOLD_DISTANCE_IN_MM,
					    uint8_t(threshold_mm >> 8 & 0xFF),
					    uint8_t(threshold_mm & 0xFF)
					   };
		transfer(&threshold_cmd[0], 3, nullptr, 0);
		px4_usleep(10_ms);

		PX4_INFO("sensor %i at address 0x%02X added", i, get_device_address());
	}

	if (_sensor_count == 0) {
		return PX4_ERROR;
	}

	PX4_INFO("%i sensors connected", _sensor_count);

	return PX4_OK;
}

int
MappyDot::probe()
{
	uint8_t cmd = MAPPYDOT_PERFORM_SINGLE_RANGE;
	int ret_val = transfer(&cmd, 1, nullptr, 0);

	return ret_val;
}

void
MappyDot::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_comms_errors);
	perf_print_counter(_sample_perf);
}

void
MappyDot::RunImpl()
{
	// Collect the sensor data.
	if (collect() != PX4_OK) {
		PX4_INFO("collection error");
		// If an error occurred, restart the measurement state machine.
		start();
		return;
	}
}

void
MappyDot::start()
{
	// Fetch parameter values.
	ModuleParams::updateParams();

	// Schedule the driver to run on a set interval
	ScheduleOnInterval(MAPPYDOT_MEASUREMENT_INTERVAL_USEC, 10000);
}

void
MappyDot::print_usage()
{
	PRINT_MODULE_USAGE_NAME("mappydot", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

I2CSPIDriverBase *MappyDot::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					int runtime_instance)
{
	MappyDot *instance = new MappyDot(iterator.configuredBusOption(), iterator.bus(), cli.bus_frequency);

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

extern "C" __EXPORT int mappydot_main(int argc, char *argv[])
{
	using ThisDriver = MappyDot;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = MAPPYDOT_BUS_CLOCK;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_DIST_DEVTYPE_MAPPYDOT);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
