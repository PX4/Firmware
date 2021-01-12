/****************************************************************************
 *
 * Copyright (C) 2020 PX4 Development Team. All rights reserved.
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

#include <board_config.h>
#ifndef BOARD_DISABLE_I2C_SPI

#ifndef MODULE_NAME
#define MODULE_NAME "SPI_I2C"
#endif

#include <lib/drivers/device/Device.hpp>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/px4_work_queue/WorkItemSingleShot.hpp>
#include <px4_platform_common/log.h>
#include <px4_platform_common/getopt.h>

#include <pthread.h>

static List<I2CSPIInstance *> i2c_spi_module_instances; ///< list of currently running instances
static pthread_mutex_t i2c_spi_module_instances_mutex = PTHREAD_MUTEX_INITIALIZER;

const char *BusCLIArguments::parseDefaultArguments(int argc, char *argv[])
{
	if (getopt(argc, argv, "") == EOF) {
		return optarg();
	}

	// unexpected arguments
	return nullptr;
}

int BusCLIArguments::getopt(int argc, char *argv[], const char *options)
{
	if (_options[0] == 0) { // need to initialize
		if (!validateConfiguration()) {
			return EOF;
		}

		char *p = (char *)&_options;

		if (_i2c_support) {
			*(p++) = 'X'; // external
			*(p++) = 'I'; // internal

			if (i2c_address != 0) {
				*(p++) = 'a'; *(p++) = ':'; // I2C address
			}
		}

		if (_spi_support) {
			*(p++) = 'S'; // external
			*(p++) = 's'; // internal
			*(p++) = 'c'; *(p++) = ':'; // chip-select
			*(p++) = 'm'; *(p++) = ':'; // spi mode
		}

		if (support_keep_running) {
			*(p++) = 'k';
		}

		*(p++) = 'b'; *(p++) = ':'; // bus
		*(p++) = 'f'; *(p++) = ':'; // frequency
		*(p++) = 'q'; // quiet flag

		// copy all options
		const char *option = options;

		while (p != _options + sizeof(_options) && *option) {
			if (*option != ':') {
				// check for duplicates
				for (const char *c = _options; c != p; ++c) {
					if (*c == *option) {
						PX4_ERR("conflicting option: %c", *c);
						_options[0] = 0;
						return EOF;
					}
				}
			}

			*(p++) = *(option++);
		}

		if (p == _options + sizeof(_options)) {
			PX4_ERR("too many options");
			_options[0] = 0;
			return EOF;
		}

		*p = '\0';
	}

	int ch;

	while ((ch = px4_getopt(argc, argv, _options, &_optind, &_optarg)) != EOF) {
		switch (ch) {
		case 'X':
			bus_option = I2CSPIBusOption::I2CExternal;
			break;

		case 'I':
			bus_option = I2CSPIBusOption::I2CInternal;
			break;

		case 'a':
			if (i2c_address == 0) {
				return ch;
			}

			i2c_address = (int)strtol(_optarg, nullptr, 0);
			break;

		case 'S':
			bus_option = I2CSPIBusOption::SPIExternal;
			break;

		case 's':
			bus_option = I2CSPIBusOption::SPIInternal;
			break;

		case 'c':
			chipselect_index = atoi(_optarg);
			break;

		case 'b':
			requested_bus = atoi(_optarg);
			break;

		case 'f':
			bus_frequency = 1000 * atoi(_optarg);
			break;

		case 'm':
			spi_mode = (spi_mode_e)atoi(_optarg);
			break;

		case 'q':
			quiet_start = true;
			break;

		case 'k':
			if (!support_keep_running) {
				return ch;
			}

			keep_running = true;
			break;

		default:
			if (ch == '?') {
				// abort further parsing on unknown arguments
				_optarg = nullptr;
				return EOF;
			}

			return ch;
		}
	}

	if (ch == EOF) {
		_optarg = argv[_optind];

		// apply defaults if not provided
		if (bus_frequency == 0) {
			if (bus_option == I2CSPIBusOption::I2CExternal || bus_option == I2CSPIBusOption::I2CInternal) {
				bus_frequency = default_i2c_frequency;

			} else if (bus_option == I2CSPIBusOption::SPIExternal || bus_option == I2CSPIBusOption::SPIInternal) {
				bus_frequency = default_spi_frequency;
			}
		}
	}

	return ch;
}

bool BusCLIArguments::validateConfiguration()
{
	bool success = true;

	if (_i2c_support && default_i2c_frequency == -1) {
		PX4_ERR("Bug: driver %s does not set default_i2c_frequency", px4_get_taskname());
		success = false;
	}

	if (_spi_support && default_spi_frequency == -1) {
		PX4_ERR("Bug: driver %s does not set default_spi_frequency", px4_get_taskname());
		success = false;
	}

	return success;
}


BusInstanceIterator::BusInstanceIterator(const char *module_name,
		const BusCLIArguments &cli_arguments, uint16_t devid_driver_index)
	: _module_name(module_name), _bus_option(cli_arguments.bus_option), _type(cli_arguments.type),
	  _i2c_address(cli_arguments.i2c_address),
	  _spi_bus_iterator(spiFilter(cli_arguments.bus_option),
			    cli_arguments.bus_option == I2CSPIBusOption::SPIExternal ? cli_arguments.chipselect_index : devid_driver_index,
			    cli_arguments.requested_bus),
	  _i2c_bus_iterator(i2cFilter(cli_arguments.bus_option), cli_arguments.requested_bus),
	  _current_instance(i2c_spi_module_instances.end())
{
	// We lock the module instance list as long as this object is alive, since we iterate over the list.
	// Locking could be a bit more fine-grained, but the iterator is mostly only used sequentially, so not an issue.
	pthread_mutex_lock(&i2c_spi_module_instances_mutex);
	_current_instance = i2c_spi_module_instances.end();
}

BusInstanceIterator::~BusInstanceIterator()
{
	pthread_mutex_unlock(&i2c_spi_module_instances_mutex);
}

bool BusInstanceIterator::next()
{
	int bus = -1;

	if (busType() == BOARD_INVALID_BUS) {
		if (_current_instance == i2c_spi_module_instances.end()) { // either not initialized, or the first instance was removed
			_current_instance = i2c_spi_module_instances.begin();

		} else {
			++_current_instance;
		}

		while (_current_instance != i2c_spi_module_instances.end()) {
			if (strcmp((*_current_instance)->_module_name, _module_name) == 0 &&
			    _type == (*_current_instance)->_type) {
				return true;
			}

			++_current_instance;
		}

		return false;

	} else if (busType() == BOARD_SPI_BUS) {
		if (_spi_bus_iterator.next()) {
			bus = _spi_bus_iterator.bus().bus;
		}

	} else {
		if (_i2c_bus_iterator.next()) {
			bus = _i2c_bus_iterator.bus().bus;
		}
	}

	if (bus != -1) {
		// find matching runtime instance
		bool is_i2c = busType() == BOARD_I2C_BUS;

		for (_current_instance = i2c_spi_module_instances.begin(); _current_instance != i2c_spi_module_instances.end();
		     ++_current_instance) {
			if (strcmp((*_current_instance)->_module_name, _module_name) != 0) {
				continue;
			}

			if (_bus_option == (*_current_instance)->_bus_option && bus == (*_current_instance)->_bus &&
			    _type == (*_current_instance)->_type &&
			    (!is_i2c || _i2c_address == (*_current_instance)->_i2c_address)) {
				break;
			}
		}

		return true;
	}

	return false;
}

int BusInstanceIterator::runningInstancesCount() const
{
	int num_instances = 0;

	for (const auto &modules : i2c_spi_module_instances) {
		if (strcmp(modules->_module_name, _module_name) == 0) {
			++num_instances;
		}
	}

	return num_instances;
}

I2CSPIInstance *BusInstanceIterator::instance() const
{
	if (_current_instance == i2c_spi_module_instances.end()) {
		return nullptr;
	}

	return *_current_instance;
}

void BusInstanceIterator::removeInstance()
{
	// find previous node
	List<I2CSPIInstance *>::Iterator previous = i2c_spi_module_instances.begin();

	while (previous != i2c_spi_module_instances.end() && (*previous)->getSibling() != *_current_instance) {
		++previous;
	}

	i2c_spi_module_instances.remove(*_current_instance);
	_current_instance = previous; // previous can be i2c_spi_module_instances.end(), which means we removed the first item
}

void BusInstanceIterator::addInstance(I2CSPIInstance *instance)
{
	i2c_spi_module_instances.add(instance);
}

board_bus_types BusInstanceIterator::busType() const
{
	switch (_bus_option) {
	case I2CSPIBusOption::All:
		return BOARD_INVALID_BUS;

	case I2CSPIBusOption::I2CInternal:
	case I2CSPIBusOption::I2CExternal:
		return BOARD_I2C_BUS;

	case I2CSPIBusOption::SPIInternal:
	case I2CSPIBusOption::SPIExternal:
		return BOARD_SPI_BUS;
	}

	return BOARD_INVALID_BUS;
}

int BusInstanceIterator::bus() const
{
	if (busType() == BOARD_INVALID_BUS) {
		return -1;

	} else if (busType() == BOARD_SPI_BUS) {
		return _spi_bus_iterator.bus().bus;

	} else {
		return _i2c_bus_iterator.bus().bus;
	}
}

uint32_t BusInstanceIterator::devid() const
{
	if (busType() == BOARD_INVALID_BUS) {
		return 0;

	} else if (busType() == BOARD_SPI_BUS) {
		return _spi_bus_iterator.devid();

	} else {
		return 0;
	}
}

spi_drdy_gpio_t BusInstanceIterator::DRDYGPIO() const
{
	if (busType() == BOARD_INVALID_BUS) {
		return 0;

	} else if (busType() == BOARD_SPI_BUS) {
		return _spi_bus_iterator.DRDYGPIO();

	} else {
		return 0;
	}
}

bool BusInstanceIterator::external() const
{
	if (busType() == BOARD_INVALID_BUS) {
		return false;

	} else if (busType() == BOARD_SPI_BUS) {
		return _spi_bus_iterator.external();

	} else {
		return _i2c_bus_iterator.external();
	}
}

int BusInstanceIterator::externalBusIndex() const
{
	if (busType() == BOARD_INVALID_BUS) {
		return 0;

	} else if (busType() == BOARD_SPI_BUS) {
		return _spi_bus_iterator.externalBusIndex();

	} else {
		return _i2c_bus_iterator.externalBusIndex();
	}
}

I2CBusIterator::FilterType BusInstanceIterator::i2cFilter(I2CSPIBusOption bus_option)
{
	switch (bus_option) {
	case I2CSPIBusOption::All: return I2CBusIterator::FilterType::All;

	case I2CSPIBusOption::I2CExternal: return I2CBusIterator::FilterType::ExternalBus;

	case I2CSPIBusOption::I2CInternal: return I2CBusIterator::FilterType::InternalBus;

	default: break;
	}

	return I2CBusIterator::FilterType::All;
}

SPIBusIterator::FilterType BusInstanceIterator::spiFilter(I2CSPIBusOption bus_option)
{
	switch (bus_option) {
	case I2CSPIBusOption::SPIExternal: return SPIBusIterator::FilterType::ExternalBus;

	case I2CSPIBusOption::SPIInternal: return SPIBusIterator::FilterType::InternalBus;

	default: break;
	}

	return SPIBusIterator::FilterType::InternalBus;
}

struct I2CSPIDriverInitializing {
	const BusCLIArguments &cli;
	const BusInstanceIterator &iterator;
	I2CSPIDriverBase::instantiate_method instantiate;
	int runtime_instance;
	I2CSPIDriverBase *instance{nullptr};
};

static void initializer_trampoline(void *argument)
{
	I2CSPIDriverInitializing *data = (I2CSPIDriverInitializing *)argument;
	data->instance = data->instantiate(data->cli, data->iterator, data->runtime_instance);
}

int I2CSPIDriverBase::module_start(const BusCLIArguments &cli, BusInstanceIterator &iterator,
				   void(*print_usage)(), instantiate_method instantiate)
{
	if (iterator.configuredBusOption() == I2CSPIBusOption::All) {
		PX4_ERR("need to specify a bus type");
		print_usage();
		return -1;
	}

	bool started = false;

	while (iterator.next()) {
		if (iterator.instance()) {
			PX4_WARN("Already running on bus %i", iterator.bus());
			continue;
		}


		device::Device::DeviceId device_id{};
		device_id.devid_s.bus = iterator.bus();

		switch (iterator.busType()) {
		case BOARD_I2C_BUS: device_id.devid_s.bus_type = device::Device::DeviceBusType_I2C; break;

		case BOARD_SPI_BUS: device_id.devid_s.bus_type = device::Device::DeviceBusType_SPI; break;

		case BOARD_INVALID_BUS: device_id.devid_s.bus_type = device::Device::DeviceBusType_UNKNOWN; break;
		}

		const int runtime_instance = iterator.runningInstancesCount();
		I2CSPIDriverInitializing initializer_data{cli, iterator, instantiate, runtime_instance};
		// initialize the object and bus on the work queue thread - this will also probe for the device
		px4::WorkItemSingleShot initializer(px4::device_bus_to_wq(device_id.devid), initializer_trampoline, &initializer_data);
		initializer.ScheduleNow();
		initializer.wait();
		I2CSPIDriverBase *instance = initializer_data.instance;

		if (!instance) {
			PX4_DEBUG("instantiate failed (no device on bus %i (devid 0x%x)?)", iterator.bus(), iterator.devid());
			continue;
		}

		if (cli.i2c_address != 0 && instance->_i2c_address == 0) {
			PX4_ERR("Bug: driver %s does not pass the I2C address to I2CSPIDriverBase", instance->ItemName());
		}

		iterator.addInstance(instance);
		started = true;

		// print some info that we are running
		switch (iterator.busType()) {
		case BOARD_I2C_BUS:
			PX4_INFO_RAW("%s #%i on I2C bus %d%s\n", instance->ItemName(), runtime_instance, iterator.bus(),
				     iterator.external() ? " (external)" : "");

			break;

		case BOARD_SPI_BUS:
			PX4_INFO_RAW("%s #%i on SPI bus %d (devid=0x%x)",
				     instance->ItemName(), runtime_instance, iterator.bus(), PX4_SPI_DEV_ID(iterator.devid()));

			if (iterator.external()) {
				PX4_INFO_RAW(" (external, equal to '-b %i')\n", iterator.externalBusIndex());

			} else {
				PX4_INFO_RAW("\n");
			}

			break;

		case BOARD_INVALID_BUS:
			break;
		}
	}

	if (!started && !cli.quiet_start) {
		PX4_WARN("%s: no instance started (no device on bus?)", px4_get_taskname());
	}

	return started ? 0 : -1;
}


int I2CSPIDriverBase::module_stop(BusInstanceIterator &iterator)
{
	bool is_running = false;

	while (iterator.next()) {
		if (iterator.instance()) {
			I2CSPIDriverBase *instance = (I2CSPIDriverBase *)iterator.instance();
			instance->request_stop_and_wait();
			delete iterator.instance();
			iterator.removeInstance();
			is_running = true;
		}
	}

	if (!is_running) {
		PX4_ERR("Not running");
		return -1;
	}

	return 0;
}

int I2CSPIDriverBase::module_status(BusInstanceIterator &iterator)
{
	bool is_running = false;

	while (iterator.next()) {
		if (iterator.instance()) {
			I2CSPIDriverBase *instance = (I2CSPIDriverBase *)iterator.instance();
			instance->print_status();
			is_running = true;
		}
	}

	if (!is_running) {
		PX4_INFO("Not running");
		return -1;
	}

	return 0;
}

struct custom_method_data_t {
	I2CSPIDriverBase *instance;
	const BusCLIArguments &cli;
};

void I2CSPIDriverBase::custom_method_trampoline(void *argument)
{
	custom_method_data_t *data = (custom_method_data_t *)argument;
	data->instance->custom_method(data->cli);
}

int I2CSPIDriverBase::module_custom_method(const BusCLIArguments &cli, BusInstanceIterator &iterator,
		bool run_on_work_queue)
{
	while (iterator.next()) {
		if (iterator.instance()) {
			I2CSPIDriverBase *instance = (I2CSPIDriverBase *)iterator.instance();

			if (run_on_work_queue) {
				custom_method_data_t data{instance, cli};
				px4::WorkItemSingleShot runner(*instance, custom_method_trampoline, &data);
				runner.ScheduleNow();
				runner.wait();

			} else {
				instance->custom_method(cli);
			}
		}
	}

	return 0;
}

void I2CSPIDriverBase::print_status()
{
	bool is_i2c_bus = _bus_option == I2CSPIBusOption::I2CExternal || _bus_option == I2CSPIBusOption::I2CInternal;
	PX4_INFO("Running on %s Bus %i", is_i2c_bus ? "I2C" : "SPI", _bus);
}

void I2CSPIDriverBase::request_stop_and_wait()
{
	_task_should_exit.store(true);
	ScheduleNow(); // wake up the task (in case it is not scheduled anymore or just to be faster)
	unsigned int i = 0;

	do {
		px4_usleep(20000); // 20 ms
		// wait at most 2 sec
	} while (++i < 100 && !_task_exited.load());

	if (i >= 100) {
		PX4_ERR("Module did not respond to stop request");
	}
}

#endif /* BOARD_DISABLE_I2C_SPI */
