/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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

#include "mpu9250.h"

enum class MPU9250_BUS {
	ALL = 0,
	I2C_INTERNAL,
	I2C_EXTERNAL,
	SPI_INTERNAL,
	SPI_INTERNAL2,
	SPI_EXTERNAL
};

namespace mpu9250
{

// list of supported bus configurations
struct mpu9250_bus_option {
	MPU9250_BUS busid;
	MPU9250_constructor interface_constructor;
	bool magpassthrough;
	uint8_t busnum;
	uint32_t address;
	MPU9250	*dev;
} bus_options[] = {
#if defined(USE_I2C)
#  if defined(PX4_I2C_BUS_ONBOARD) && defined(PX4_I2C_OBDEV_MPU9250)
	{ MPU9250_BUS::I2C_INTERNAL, &MPU9250_I2C_interface, false, PX4_I2C_BUS_ONBOARD, PX4_I2C_OBDEV_MPU9250, nullptr },
#  endif
#  if defined(PX4_I2C_BUS_EXPANSION) && defined(PX4_I2C_OBDEV_MPU9250)
	{ MPU9250_BUS::I2C_EXTERNAL, &MPU9250_I2C_interface, false, PX4_I2C_BUS_EXPANSION, PX4_I2C_OBDEV_MPU9250, nullptr },
#  endif
#  if defined(PX4_I2C_BUS_EXPANSION1) && defined(PX4_I2C_OBDEV_MPU9250)
	{ MPU9250_BUS::I2C_EXTERNAL, &MPU9250_I2C_interface, false, PX4_I2C_BUS_EXPANSION1, PX4_I2C_OBDEV_MPU9250, nullptr },
#  endif
#  if defined(PX4_I2C_BUS_EXPANSION2) && defined(PX4_I2C_OBDEV_MPU9250)
	{ MPU9250_BUS::I2C_EXTERNAL, &MPU9250_I2C_interface, false, PX4_I2C_BUS_EXPANSION2, PX4_I2C_OBDEV_MPU9250, nullptr },
#  endif
#endif
#if defined(PX4_SPI_BUS_SENSORS) && defined(PX4_SPIDEV_MPU)
	{ MPU9250_BUS::SPI_INTERNAL, &MPU9250_SPI_interface, true, PX4_SPI_BUS_SENSORS, PX4_SPIDEV_MPU, nullptr },
#endif
#if defined(PX4_SPI_BUS_SENSORS) && defined(PX4_SPIDEV_MPU2)
	{ MPU9250_BUS::SPI_INTERNAL2, &MPU9250_SPI_interface, true, PX4_SPI_BUS_SENSORS, PX4_SPIDEV_MPU2, nullptr },
#endif
#if defined(PX4_SPI_BUS_EXT) && defined(PX4_SPIDEV_EXT_MPU)
	{ MPU9250_BUS::SPI_EXTERNAL, &MPU9250_SPI_interface, true, PX4_SPI_BUS_EXT, PX4_SPIDEV_EXT_MPU, nullptr },
#endif
};

// find a bus structure for a busid
static mpu9250_bus_option *find_bus(MPU9250_BUS busid)
{
	for (mpu9250_bus_option &bus_option : bus_options) {
		if ((busid == MPU9250_BUS::ALL ||
		     busid == bus_option.busid) && bus_option.dev != nullptr) {

			return &bus_option;
		}
	}

	return nullptr;
}

static bool start_bus(mpu9250_bus_option &bus, enum Rotation rotation)
{
	device::Device *interface = bus.interface_constructor(bus.busnum, bus.address);

	if ((interface == nullptr) || (interface->init() != PX4_OK)) {
		PX4_WARN("no device on bus %u", (unsigned)bus.busid);
		delete interface;
		return false;
	}

	device::Device *mag_interface = nullptr;

#ifdef USE_I2C

	// For i2c interfaces, connect to the magnetomer directly
	if ((bus.busid == MPU9250_BUS::I2C_INTERNAL) || (bus.busid == MPU9250_BUS::I2C_EXTERNAL)) {
		mag_interface = AK8963_I2C_interface(bus.busnum);
	}

#endif // USE_I2C

	MPU9250 *dev = new MPU9250(interface, mag_interface, rotation);

	if (dev == nullptr) {
		PX4_ERR("alloc failed");
		delete mag_interface;
		return false;
	}

	if (dev->init() != PX4_OK) {
		PX4_ERR("driver start failed");
		delete dev;
		delete mag_interface;
		return false;
	}

	bus.dev = dev;

	return true;
}

static int start(MPU9250_BUS busid, enum Rotation rotation)
{
	for (mpu9250_bus_option &bus_option : bus_options) {
		if (bus_option.dev != nullptr) {
			// this device is already started
			PX4_WARN("already started");
			continue;
		}

		if (busid != MPU9250_BUS::ALL && bus_option.busid != busid) {
			// not the one that is asked for
			continue;
		}

		if (start_bus(bus_option, rotation)) {
			return 0;
		}
	}

	return -1;
}

static int stop(MPU9250_BUS busid)
{
	mpu9250_bus_option *bus = find_bus(busid);

	if (bus != nullptr && bus->dev != nullptr) {
		delete bus->dev;
		bus->dev = nullptr;

	} else {
		PX4_WARN("driver not running");
		return -1;
	}

	return 0;
}

static int status(MPU9250_BUS busid)
{
	mpu9250_bus_option *bus = find_bus(busid);

	if (bus != nullptr && bus->dev != nullptr) {
		bus->dev->print_info();
		return 0;
	}

	PX4_WARN("driver not running");
	return -1;
}

static int usage()
{
	PX4_INFO("missing command: try 'start', 'stop', 'status'");
	PX4_INFO("options:");
	PX4_INFO("    -X    (i2c external bus)");
	PX4_INFO("    -I    (i2c internal bus)");
	PX4_INFO("    -s    (spi internal bus)");
	PX4_INFO("    -S    (spi external bus)");
	PX4_INFO("    -t    (spi internal bus, 2nd instance)");
	PX4_INFO("    -R rotation");

	return 0;
}

} // namespace

extern "C" int mpu9250_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	MPU9250_BUS busid = MPU9250_BUS::ALL;
	enum Rotation rotation = ROTATION_NONE;

	while ((ch = px4_getopt(argc, argv, "XISstR:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			busid = MPU9250_BUS::I2C_EXTERNAL;
			break;

		case 'I':
			busid = MPU9250_BUS::I2C_INTERNAL;
			break;

		case 'S':
			busid = MPU9250_BUS::SPI_EXTERNAL;
			break;

		case 's':
			busid = MPU9250_BUS::SPI_INTERNAL;
			break;

		case 't':
			busid = MPU9250_BUS::SPI_INTERNAL2;
			break;

		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;

		default:
			return mpu9250::usage();
		}
	}

	if (myoptind >= argc) {
		return mpu9250::usage();
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		return mpu9250::start(busid, rotation);

	} else if (!strcmp(verb, "stop")) {
		return mpu9250::stop(busid);

	} else if (!strcmp(verb, "status")) {
		return mpu9250::status(busid);
	}

	return mpu9250::usage();
}
