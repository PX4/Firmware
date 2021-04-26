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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>

#include "tcbp001ta.hpp"

enum class TCBP001TA_BUS {

	SPI_INTERNAL = 0
};

namespace tcbp001ta
{

// list of supported bus configurations
struct tcbp001ta_bus_option {
	TCBP001TA_BUS busid;
	TCBP001TA_constructor interface_constructor;
	uint8_t busnum;
	uint32_t address;
	TCBP001TA	*dev;
} bus_options[] = {

	{ TCBP001TA_BUS::SPI_INTERNAL, &tcbp001ta_spi_interface, PX4_SPI_BUS_BARO, PX4_SPIDEV_BARO, nullptr },
};

// find a bus structure for a busid
static struct tcbp001ta_bus_option *find_bus(TCBP001TA_BUS busid)
{
	for (tcbp001ta_bus_option &bus_option : bus_options) {
		if ((
			    busid == bus_option.busid) && bus_option.dev != nullptr) {

			return &bus_option;
		}
	}

	return nullptr;
}

static bool start_bus(tcbp001ta_bus_option &bus)
{
	tcbp001ta::ITCBP001TA *interface = bus.interface_constructor(bus.busnum, bus.address);

	if ((interface == nullptr) || (interface->init() != PX4_OK)) {
		PX4_WARN("no device on bus %u", (unsigned)bus.busid);
		delete interface;
		return false;
	}

	TCBP001TA *dev = new TCBP001TA(interface);

	if (dev == nullptr) {
		PX4_ERR("driver allocate failed");
		delete interface;
		return false;
	}

	if (dev->init() != PX4_OK) {
		PX4_ERR("driver start failed");
		delete dev;	// TCBP001TA deletes the interface
		return false;
	}

	bus.dev = dev;

	return true;
}

static int start(TCBP001TA_BUS busid)
{
	for (tcbp001ta_bus_option &bus_option : bus_options) {
		if (bus_option.dev != nullptr) {
			// this device is already started
			PX4_WARN("already started");
			continue;
		}

		if (bus_option.busid != busid) {
			// not the one that is asked for
			continue;
		}

		if (start_bus(bus_option)) {
			return PX4_OK;
		}
	}

	return PX4_ERROR;
}

static int stop(TCBP001TA_BUS busid)
{
	tcbp001ta_bus_option *bus = find_bus(busid);

	if (bus != nullptr && bus->dev != nullptr) {
		delete bus->dev;
		bus->dev = nullptr;

	} else {
		PX4_WARN("driver not running");
		return PX4_ERROR;
	}

	return PX4_OK;
}

static int status(TCBP001TA_BUS busid)
{
	tcbp001ta_bus_option *bus = find_bus(busid);

	if (bus != nullptr && bus->dev != nullptr) {
		bus->dev->print_info();
		return PX4_OK;
	}

	PX4_WARN("driver not running");
	return PX4_ERROR;
}

static int usage()
{
	PX4_INFO("missing command: try 'start', 'stop', 'status'");
	PX4_INFO("options:");
	PX4_INFO("    -X    (i2c external bus)");
	PX4_INFO("    -I    (i2c internal bus)");
	PX4_INFO("    -s    (spi internal bus)");
	PX4_INFO("    -S    (spi external bus)");

	return 0;
}

} // namespace

extern "C" int tcbp001ta_main(int argc, char *argv[])
{
	int myoptind = 1;
	// int ch;
	// const char *myoptarg = nullptr;

	TCBP001TA_BUS busid;
	busid = TCBP001TA_BUS::SPI_INTERNAL;



	// if (myoptind >= argc) {
	// 	return tcbp001ta::usage();
	// }

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		PX4_INFO("start");
		return tcbp001ta::start(busid);

	} else if (!strcmp(verb, "stop")) {
		return tcbp001ta::stop(busid);

	} else if (!strcmp(verb, "status")) {
		return tcbp001ta::status(busid);
	}

	return tcbp001ta::usage();
}
