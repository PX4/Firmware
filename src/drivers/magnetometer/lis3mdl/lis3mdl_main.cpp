/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file lis3mdl_main.cpp
 *
 * Driver for the LIS3MDL magnetometer connected via I2C or SPI.
 */

#include "lis3mdl_main.h"
#include <px4_platform_common/getopt.h>

/**
 * Driver 'main' command.
 */
extern "C" __EXPORT int lis3mdl_main(int argc, char *argv[]);

int
lis3mdl::calibrate(struct lis3mdl_bus_option &bus)
{
	int ret;
	const char *path = bus.devpath;

	PX4_INFO("running on bus: %u (%s)", (unsigned)bus.bus_id, bus.devpath);

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		PX4_WARN("%s open failed (try 'lis3mdl start' if the driver is not running", path);
		return PX4_ERROR;
	}

	if ((ret = ioctl(fd, MAGIOCCALIBRATE, fd)) != OK) {
		PX4_WARN("failed to enable sensor calibration mode");
	}

	close(fd);

	return ret;
}

int
lis3mdl::info(struct lis3mdl_bus_option &bus)
{
	PX4_INFO("running on bus: %u (%s)", (unsigned)bus.bus_id, bus.devpath);
	bus.dev->print_info();
	return PX4_OK;
}

int
lis3mdl::init(struct lis3mdl_bus_option &bus)
{
	const char *path = bus.devpath;

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		return PX4_ERROR;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		close(fd);
		errx(1, "Failed to setup poll rate");
		return PX4_ERROR;

	} else {
		PX4_INFO("Poll rate set to max (80hz)");
	}

	close(fd);

	return PX4_OK;
}

int
lis3mdl::start_bus(struct lis3mdl_bus_option &bus, Rotation rotation)
{
	if (bus.dev != nullptr) {
		errx(1, "bus option already started");
		return PX4_ERROR;
	}

	device::Device *interface = bus.interface_constructor(bus.busnum);

	if (interface->init() != OK) {
		delete interface;
		warnx("no device on bus %u", (unsigned)bus.bus_id);
		return PX4_ERROR;
	}

	bus.dev = new LIS3MDL(interface, bus.devpath, rotation);

	if (bus.dev != nullptr &&
	    bus.dev->init() != OK) {
		delete bus.dev;
		bus.dev = NULL;
		return PX4_ERROR;
	}

	return PX4_OK;
}

int
lis3mdl::start(struct lis3mdl_bus_option &bus, Rotation rotation)
{
	if (bus.dev == NULL) {
		return start_bus(bus, rotation);

	} else {
		// this device is already started
		return PX4_ERROR;
	}
}

int
lis3mdl::stop(struct lis3mdl_bus_option &bus)
{
	if (bus.dev != NULL) {
		bus.dev->stop();
		delete bus.dev;
		bus.dev = nullptr;
		return PX4_OK;

	} else {
		// this device is already stopped
		return PX4_ERROR;
	}
}

int
lis3mdl::test(struct lis3mdl_bus_option &bus)
{
	sensor_mag_s report;
	ssize_t sz;
	int ret;
	const char *path = bus.devpath;

	PX4_INFO("running on bus: %u (%s)", (unsigned)bus.bus_id, bus.devpath);

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		PX4_WARN("%s open failed (try 'lis3mdl start')", path);
		return PX4_ERROR;
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		PX4_WARN("immediate read failed");
		return PX4_ERROR;
	}

	print_message(report);

	/* check if mag is onboard or external */
	if (ioctl(fd, MAGIOCGEXTERNAL, 0) < 0) {
		PX4_WARN("failed to get if mag is onboard or external");
		return PX4_ERROR;
	}

	/* start the sensor polling at 2Hz */
	if (ioctl(fd, SENSORIOCSPOLLRATE, 2) != OK) {
		PX4_WARN("failed to set 2Hz poll rate");
		return PX4_ERROR;
	}

	struct pollfd fds;

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			PX4_WARN("timed out waiting for sensor data");
			return PX4_ERROR;
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			PX4_WARN("periodic read failed");
			return PX4_ERROR;
		}

		print_message(report);
	}

	PX4_INFO("PASS");
	return PX4_OK;
}

int
lis3mdl::reset(struct lis3mdl_bus_option &bus)
{
	const char *path = bus.devpath;

	PX4_INFO("running on bus: %u (%s)", (unsigned)bus.bus_id, bus.devpath);

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		PX4_WARN("open failed ");
		return PX4_ERROR;
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		PX4_WARN("driver reset failed");
		return PX4_ERROR;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_WARN("driver poll restart failed");
		return PX4_ERROR;
	}

	return PX4_OK;
}

void
lis3mdl::usage()
{
	PX4_WARN("missing command: try 'start', 'info', 'test', 'reset', 'info', 'calibrate'");
	PX4_WARN("options:");
	PX4_WARN("    -R rotation");
	PX4_WARN("    -C calibrate on start");
	PX4_WARN("    -X only external bus");
	PX4_WARN("    -S only spi bus");
#if (PX4_I2C_BUS_ONBOARD || PX4_SPIDEV_LIS)
	PX4_WARN("    -I only internal bus");
#endif
}

int
lis3mdl_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	bool calibrate = false;
	enum LIS3MDL_BUS bus_id = LIS3MDL_BUS_ALL;
	enum Rotation rotation = ROTATION_NONE;

	while ((ch = px4_getopt(argc, argv, "XISR:CT", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;
#if (PX4_I2C_BUS_ONBOARD || PX4_SPIDEV_LIS)

		case 'I':
			bus_id = LIS3MDL_BUS_I2C_INTERNAL;
			break;
#endif

		case 'X':
			bus_id = LIS3MDL_BUS_I2C_EXTERNAL;
			break;

		case 'S':
			bus_id = LIS3MDL_BUS_SPI;
			break;

		case 'C':
			calibrate = true;
			break;

		default:
			lis3mdl::usage();
			return PX4_ERROR;
		}
	}

	if (myoptind >= argc) {
		lis3mdl::usage();
		return PX4_ERROR;
	}

	const char *verb = argv[myoptind];
	int ret;
	bool dev_found = false;
	bool cmd_found = false;

	if (!strcmp(verb, "start")) {
		// Start/load the driver

		cmd_found = true;
		ret = 1; // default: failed, will be set to success if one start succeeds

		for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
			if (bus_id != LIS3MDL_BUS_ALL && bus_id != lis3mdl::bus_options[i].bus_id) {
				// not the one that is asked for
				continue;
			}

			dev_found = true;

			// Start/load the driver
			if (lis3mdl::start(lis3mdl::bus_options[i], rotation) == OK) {
				if (calibrate) {
					if (lis3mdl::calibrate(lis3mdl::bus_options[i]) != OK) {
						PX4_WARN("calibration failed");
						lis3mdl::stop(lis3mdl::bus_options[i]); //Stop, failed

					} else {
						PX4_INFO("calibration successful");
						lis3mdl::init(lis3mdl::bus_options[i]);
						ret = 0; // one succeed
					}

				} else {
					lis3mdl::init(lis3mdl::bus_options[i]);
					ret = 0; // one succeed
				}
			}
		}

	} else {
		// Other commands

		ret = 0; // default: success, will be set to failed if one action fails

		for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
			if (bus_id != LIS3MDL_BUS_ALL && bus_id != lis3mdl::bus_options[i].bus_id) {
				// not the one that is asked for
				continue;
			}

			if (lis3mdl::bus_options[i].dev == NULL) {
				if (bus_id != LIS3MDL_BUS_ALL) {
					PX4_ERR("bus %u not started", (unsigned)bus_id);
					return PX4_ERROR;

				} else {
					continue;
				}
			}

			dev_found = true;

			// Stop the driver
			if (!strcmp(verb, "stop")) {
				cmd_found = true;
				ret |= lis3mdl::stop(lis3mdl::bus_options[i]);
			}

			// Test the driver/device
			if (!strcmp(verb, "test")) {
				cmd_found = true;
				ret |= lis3mdl::test(lis3mdl::bus_options[i]);
			}

			// Reset the driver
			if (!strcmp(verb, "reset")) {
				cmd_found = true;
				ret |= lis3mdl::reset(lis3mdl::bus_options[i]);
			}

			// Print driver information
			if (!strcmp(verb, "info") ||
			    !strcmp(verb, "status")) {
				cmd_found = true;
				ret |= lis3mdl::info(lis3mdl::bus_options[i]);
			}

			// Autocalibrate the scaling
			if (!strcmp(verb, "calibrate")) {
				cmd_found = true;

				if (lis3mdl::calibrate(lis3mdl::bus_options[i]) == OK) {
					PX4_INFO("calibration successful");

				} else {
					PX4_WARN("calibration failed");
					ret = 1;
				}
			}
		}
	}

	if (!dev_found) {
		PX4_WARN("no device found, please start driver first");
		return PX4_ERROR;

	} else if (!cmd_found) {
		PX4_WARN("unrecognized command, try 'start', 'test', 'reset', 'calibrate' 'or 'info'");
		return PX4_ERROR;

	} else {
		return ret;
	}
}
