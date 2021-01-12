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

#pragma once

#include "i2c.h"
#include "spi.h"

#include <stdint.h>

#include <containers/List.hpp>
#include <lib/conversion/rotation.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/sem.h>
#include <board_config.h>
#include <drivers/device/spi.h>

enum class I2CSPIBusOption : uint8_t {
	All = 0, ///< select all runnning instances
	I2CInternal,
	I2CExternal,
	SPIInternal,
	SPIExternal,
};

/**
 * @class I2CSPIInstance
 * I2C/SPI driver instance used by BusInstanceIterator to find running instances.
 */
class I2CSPIInstance : public ListNode<I2CSPIInstance *>
{
public:
	virtual ~I2CSPIInstance() = default;

private:
	I2CSPIInstance(const char *module_name, I2CSPIBusOption bus_option, int bus, uint8_t i2c_address, uint16_t type)
		: _module_name(module_name), _bus_option(bus_option), _bus(bus), _type(type), _i2c_address(i2c_address) {}

	friend class BusInstanceIterator;
	friend class I2CSPIDriverBase;

	const char *_module_name;
	const I2CSPIBusOption _bus_option;
	const int _bus;
	const int16_t _type; ///< device type (driver-specific)
	const int8_t _i2c_address; ///< I2C address (optional)
};

class BusCLIArguments
{
public:
	BusCLIArguments(bool i2c_support, bool spi_support)
		: _i2c_support(i2c_support), _spi_support(spi_support) {}

	/**
	 * Parse CLI arguments (for drivers that don't need any custom arguments, otherwise getopt() should be used)
	 * @return command (e.g. "start") or nullptr on error or unknown argument
	 */
	const char *parseDefaultArguments(int argc, char *argv[]);

	/**
	 * Like px4_getopt(), but adds and handles i2c/spi driver-specific arguments
	 */
	int getopt(int argc, char *argv[], const char *options);

	/**
	 * returns the current optional argument (for options like 'T:'), or the command (e.g. "start")
	 * @return nullptr or argument/command
	 */
	const char *optarg() const { return _optarg; }


	I2CSPIBusOption bus_option{I2CSPIBusOption::All};
	uint16_t type{0}; ///< device type (driver-specific)
	int requested_bus{-1};
	int chipselect_index{1};
	Rotation rotation{ROTATION_NONE};
	int bus_frequency{0};
	spi_mode_e spi_mode{SPIDEV_MODE3};
	uint8_t i2c_address{0}; ///< optional I2C address: a driver can set this to allow configuring the I2C address
	bool quiet_start{false}; ///< do not print a message when startup fails
	bool keep_running{false}; ///< keep driver running even if no device is detected on startup

	uint8_t orientation{0}; ///< distance_sensor_s::ROTATION_*

	int custom1{0}; ///< driver-specific custom argument
	int custom2{0}; ///< driver-specific custom argument
	void *custom_data{nullptr}; ///< driver-specific custom argument

	// driver defaults, if not specified via CLI
	int default_spi_frequency{-1}; ///< default spi bus frequency (driver needs to set this) [Hz]
	int default_i2c_frequency{-1}; ///< default i2c bus frequency (driver needs to set this) [Hz]

	bool support_keep_running{false}; ///< true if keep_running (see above) is supported

private:
	bool validateConfiguration();

	char _options[32] {};
	int _optind{1};
	const char *_optarg{nullptr};
	const bool _i2c_support;
	const bool _spi_support;
};

/**
 * @class BusInstanceIterator
 * Iterate over running instances and/or configured I2C/SPI buses with given filter options.
 */
class BusInstanceIterator
{
public:
	BusInstanceIterator(const char *module_name, const BusCLIArguments &cli_arguments, uint16_t devid_driver_index);
	~BusInstanceIterator();

	I2CSPIBusOption configuredBusOption() const { return _bus_option; }

	int runningInstancesCount() const;

	bool next();

	I2CSPIInstance *instance() const;
	void removeInstance();
	board_bus_types busType() const;
	int bus() const;
	uint32_t devid() const;
	spi_drdy_gpio_t DRDYGPIO() const;
	bool external() const;
	int externalBusIndex() const;

	void addInstance(I2CSPIInstance *instance);

	static I2CBusIterator::FilterType i2cFilter(I2CSPIBusOption bus_option);
	static SPIBusIterator::FilterType spiFilter(I2CSPIBusOption bus_option);
private:
	const char *_module_name;
	const I2CSPIBusOption _bus_option;
	const uint16_t _type;
	const uint8_t _i2c_address;
	SPIBusIterator _spi_bus_iterator;
	I2CBusIterator _i2c_bus_iterator;
	List<I2CSPIInstance *>::Iterator _current_instance;
};

/**
 * @class I2CSPIDriverBase
 * Base class for I2C/SPI driver modules (non-templated, used by I2CSPIDriver)
 */
class I2CSPIDriverBase : public px4::ScheduledWorkItem, public I2CSPIInstance
{
public:
	I2CSPIDriverBase(const char *module_name, const px4::wq_config_t &config, I2CSPIBusOption bus_option, int bus,
			 uint8_t i2c_address, uint16_t type)
		: ScheduledWorkItem(module_name, config),
		  I2CSPIInstance(module_name, bus_option, bus, i2c_address, type) {}

	static int module_stop(BusInstanceIterator &iterator);
	static int module_status(BusInstanceIterator &iterator);
	static int module_custom_method(const BusCLIArguments &cli, BusInstanceIterator &iterator,
					bool run_on_work_queue = true);

	using instantiate_method = I2CSPIDriverBase * (*)(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
				   int runtime_instance);
protected:
	virtual ~I2CSPIDriverBase() = default;

	virtual void print_status();

	virtual void custom_method(const BusCLIArguments &cli) {}

	/**
	 * Exiting the module. A driver can override this, for example to unregister interrupt callbacks.
	 * This will be called from the work queue.
	 * A module overriding this, needs to call I2CSPIDriverBase::exit_and_cleanup() as the very last statement.
	 */
	virtual void exit_and_cleanup() { ScheduleClear(); _task_exited.store(true); }

	bool should_exit() const { return _task_should_exit.load(); }

	static int module_start(const BusCLIArguments &cli, BusInstanceIterator &iterator, void(*print_usage)(),
				instantiate_method instantiate);

private:
	static void custom_method_trampoline(void *argument);

	void request_stop_and_wait();

	px4::atomic_bool _task_should_exit{false};
	px4::atomic_bool _task_exited{false};
};

/**
 * @class I2CSPIDriver
 * Base class for I2C/SPI driver modules
 */
template<class T>
class I2CSPIDriver : public I2CSPIDriverBase
{
public:
	static int module_start(const BusCLIArguments &cli, BusInstanceIterator &iterator)
	{
		return I2CSPIDriverBase::module_start(cli, iterator, &T::print_usage, &T::instantiate);
	}

protected:
	I2CSPIDriver(const char *module_name, const px4::wq_config_t &config, I2CSPIBusOption bus_option, int bus,
		     uint8_t i2c_address = 0, uint16_t type = 0)
		: I2CSPIDriverBase(module_name, config, bus_option, bus, i2c_address, type) {}

	virtual ~I2CSPIDriver() = default;

	// *INDENT-OFF* remove once there's astyle >3.1 in CI
	void Run() final
	{
		static_cast<T *>(this)->RunImpl();

		if (should_exit()) {
			exit_and_cleanup();
		}
	}
	// *INDENT-ON*
private:
};
