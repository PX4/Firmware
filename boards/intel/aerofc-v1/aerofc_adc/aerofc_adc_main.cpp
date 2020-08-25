/****************************************************************************
 *
 *   Copyright (C) 2016 Intel Corporation. All rights reserved.
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

#include "AEROFC_ADC.hpp"

enum AEROFC_ADC_BUS {
	AEROFC_ADC_BUS_ALL = 0,
	AEROFC_ADC_BUS_I2C_INTERNAL,
	AEROFC_ADC_BUS_I2C_EXTERNAL
};

static constexpr struct aerofc_adc_bus_option {
	enum AEROFC_ADC_BUS busid;
	uint8_t busnum;
} bus_options[] = {
#ifdef PX4_I2C_BUS_EXPANSION
	{ AEROFC_ADC_BUS_I2C_EXTERNAL, PX4_I2C_BUS_EXPANSION },
#endif
#ifdef PX4_I2C_BUS_EXPANSION1
	{ AEROFC_ADC_BUS_I2C_EXTERNAL, PX4_I2C_BUS_EXPANSION1 },
#endif
#ifdef PX4_I2C_BUS_ONBOARD
	{ AEROFC_ADC_BUS_I2C_INTERNAL, PX4_I2C_BUS_ONBOARD },
#endif
};

extern "C" { __EXPORT int aerofc_adc_main(int argc, char *argv[]); }

static AEROFC_ADC *instance = nullptr;

static int test()
{
	int fd = px4_open(ADC0_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("can't open ADC device");
		return PX4_ERROR;
	}

	px4_adc_msg_t data[MAX_CHANNEL];
	ssize_t count = px4_read(fd, data, sizeof(data));

	if (count < 0) {
		PX4_ERR("read error");
		px4_close(fd);
		return PX4_ERROR;
	}

	unsigned channels = count / sizeof(data[0]);

	for (unsigned j = 0; j < channels; j++) {
		printf("%d: %u  ", data[j].am_channel, data[j].am_data);
	}

	printf("\n");
	px4_close(fd);

	return 0;
}

static void help()
{
	printf("missing command: try 'start' or 'test'\n");
	printf("options:\n");
	printf("    -I only internal I2C bus\n");
	printf("    -X only external I2C bus\n");
}

int aerofc_adc_main(int argc, char *argv[])
{
	int ch;
	enum AEROFC_ADC_BUS busid = AEROFC_ADC_BUS_ALL;

	while ((ch = getopt(argc, argv, "XI")) != EOF) {
		switch (ch) {
		case 'X':
			busid = AEROFC_ADC_BUS_I2C_EXTERNAL;
			break;

		case 'I':
			busid = AEROFC_ADC_BUS_I2C_INTERNAL;
			break;

		default:
			help();
			return -1;
		}
	}

	if (optind >= argc) {
		help();
		return PX4_ERROR;
	}

	const char *verb = argv[optind];

	if (!strcmp(verb, "start")) {
		if (instance) {
			PX4_WARN("AEROFC_ADC was already started");
			return PX4_OK;
		}

		for (uint8_t i = 0; i < (sizeof(bus_options) / sizeof(bus_options[0])); i++) {
			if (busid != AEROFC_ADC_BUS_ALL && busid != bus_options[i].busid) {
				continue;
			}

			instance = new AEROFC_ADC(bus_options[i].busnum);

			if (!instance) {
				PX4_WARN("No memory to instance AEROFC_ADC");
				return PX4_ERROR;
			}

			if (instance->init() == PX4_OK) {
				break;
			}

			PX4_WARN("AEROFC_ADC not found on busnum=%u", bus_options[i].busnum);
			delete instance;
			instance = nullptr;
		}

		if (!instance) {
			PX4_WARN("AEROFC_ADC not found");
			return PX4_ERROR;
		}

	} else if (!strcmp(verb, "test")) {
		return test();

	} else {
		PX4_WARN("Action not supported");
		help();
		return PX4_ERROR;
	}

	return PX4_OK;
}
