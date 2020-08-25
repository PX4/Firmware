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

#include "IST8308.hpp"

#include <px4_platform_common/getopt.h>

namespace ist8308
{

enum class BUS {
	ALL = 0,
	I2C_EXTERNAL  = 1,
	I2C_EXTERNAL1 = 2,
	I2C_EXTERNAL2 = 3,
	I2C_EXTERNAL3 = 4,
	I2C_INTERNAL  = 5,
};

struct bus_option {
	BUS busid;
	uint8_t busnum;
	IST8308 *dev;
} bus_options[] = {
#if defined(PX4_I2C_BUS_ONBOARD)
	{ BUS::I2C_INTERNAL, PX4_I2C_BUS_ONBOARD, nullptr },
#endif
#if defined(PX4_I2C_BUS_EXPANSION)
	{ BUS::I2C_EXTERNAL, PX4_I2C_BUS_EXPANSION, nullptr },
#endif
#if defined(PX4_I2C_BUS_EXPANSION1)
	{ BUS::I2C_EXTERNAL1, PX4_I2C_BUS_EXPANSION1, nullptr },
#endif
#if defined(PX4_I2C_BUS_EXPANSION2)
	{ BUS::I2C_EXTERNAL2, PX4_I2C_BUS_EXPANSION2, nullptr },
#endif
#if defined(PX4_I2C_BUS_EXPANSION3)
	{ BUS::I2C_EXTERNAL3, PX4_I2C_BUS_EXPANSION3, nullptr },
#endif
};

// find a bus structure for a busid
static bus_option *find_bus(BUS busid)
{
	for (bus_option &bus_option : bus_options) {
		if ((busid == BUS::ALL || busid == bus_option.busid)
		    && bus_option.dev != nullptr) {

			return &bus_option;
		}
	}

	return nullptr;
}

static bool start_bus(bus_option &bus, enum Rotation rotation)
{
	IST8308 *dev = new IST8308(bus.busnum, I2C_ADDRESS_DEFAULT, rotation);

	if (dev == nullptr) {
		PX4_ERR("alloc failed");
		return false;
	}

	if (!dev->Init()) {
		PX4_ERR("driver start failed");
		delete dev;
		return false;
	}

	bus.dev = dev;

	return true;
}

static int start(BUS busid, enum Rotation rotation)
{
	for (bus_option &bus_option : bus_options) {
		if (bus_option.dev != nullptr) {
			// this device is already started
			PX4_WARN("already started");
			continue;
		}

		if (busid != BUS::ALL && bus_option.busid != busid) {
			// not the one that is asked for
			continue;
		}

		if (start_bus(bus_option, rotation)) {
			return 0;
		}
	}

	return -1;
}

static int stop(BUS busid)
{
	bus_option *bus = find_bus(busid);

	if (bus != nullptr && bus->dev != nullptr) {
		delete bus->dev;
		bus->dev = nullptr;

	} else {
		PX4_WARN("driver not running");
		return -1;
	}

	return 0;
}

static int status(BUS busid)
{
	bus_option *bus = find_bus(busid);

	if (bus != nullptr && bus->dev != nullptr) {
		bus->dev->PrintInfo();
		return 0;
	}

	PX4_WARN("driver not running");
	return -1;
}

static int usage()
{
	PX4_INFO("missing command: try 'start', 'stop', 'status'");
	PX4_INFO("options:");
	PX4_INFO("    -R rotation");

	return 0;
}

} // namespace ist8308

extern "C" int ist8308_main(int argc, char *argv[])
{
	enum Rotation rotation = ROTATION_NONE;
	int myoptind = 1;
	int ch = 0;
	const char *myoptarg = nullptr;
	ist8308::BUS busid = ist8308::BUS::ALL;

	// start options
	while ((ch = px4_getopt(argc, argv, "R:b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;

		case 'b':
			busid = (ist8308::BUS)atoi(myoptarg);
			break;

		default:
			return ist8308::usage();
		}
	}

	if (myoptind >= argc) {
		return ist8308::usage();
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		return ist8308::start(busid, rotation);

	} else if (!strcmp(verb, "stop")) {
		return ist8308::stop(busid);

	} else if (!strcmp(verb, "status")) {
		return ist8308::status(busid);

	}

	return ist8308::usage();
}
