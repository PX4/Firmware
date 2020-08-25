/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file i2cdetect.cpp
 *
 * Simple tool to scan an I2C bus.
 *
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>

namespace i2cdetect
{

int detect(int bus)
{
	printf("Scanning I2C bus: %d\n", bus);

	int ret = PX4_ERROR;

	// attach to the i2c bus
	struct i2c_master_s *i2c_dev = px4_i2cbus_initialize(bus);

	if (i2c_dev == nullptr) {
		PX4_ERR("invalid bus %d", bus);
		return PX4_ERROR;
	}

	printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");

	for (int i = 0; i < 128; i += 16) {
		printf("%02x: ", i);

		for (int j = 0; j < 16; j++) {

			fflush(stdout);

			uint8_t addr = i + j;

			unsigned retry_count = 0;
			const unsigned retries = 5;

			bool found = false;

			do {
				uint8_t send_data = 0;
				uint8_t recv_data = 0;
				i2c_msg_s msgv[2] {};

				// send
				msgv[0].frequency = 100000;
				msgv[0].addr = addr;
				msgv[0].flags = 0;
				msgv[0].buffer = &send_data;
				msgv[0].length = sizeof(send_data);

				// recv
				msgv[1].frequency = 100000;
				msgv[1].addr = addr;
				msgv[1].flags = I2C_M_READ;
				msgv[1].buffer = &recv_data;;
				msgv[1].length = sizeof(recv_data);

				ret = I2C_TRANSFER(i2c_dev, &msgv[0], 2);

				// success
				if (ret == PX4_OK) {
					found = true;
					break;
				}

				// if we have already retried once, or we are going to give up, then reset the bus
				if ((retry_count >= 1) || (retry_count >= retries)) {
					I2C_RESET(i2c_dev);
				}

			} while (retry_count++ < retries);

			if (found) {
				printf("%02x ", addr);

			} else {
				printf("-- ");
			}
		}

		printf("\n");
	}

	px4_i2cbus_uninitialize(i2c_dev);

	return ret;
}

int usage(const char *reason = nullptr)
{
	if (reason) {
		PX4_ERR("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION("Utility to scan for I2C devices on a particular bus.");

	PRINT_MODULE_USAGE_NAME_SIMPLE("i2cdetect", "command");
	PRINT_MODULE_USAGE_PARAM_INT('b', 1, 4, 1, "I2C bus", true);

	return PX4_OK;
}

} // namespace i2cdetect

extern "C" {
	__EXPORT int i2cdetect_main(int argc, char *argv[]);
}

int i2cdetect_main(int argc, char *argv[])
{
	int i2c_bus = 1;

	int myoptind = 1;
	int ch = 0;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			// set i2c bus
			i2c_bus = strtol(myoptarg, nullptr, 0);
			break;

		default:
			i2cdetect::usage();
			return -1;
			break;
		}
	}

	return i2cdetect::detect(i2c_bus);
}
