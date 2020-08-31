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
 * @file irlock.cpp
 * @author Michael Landes
 *
 * Driver for an IR-Lock and Pixy vision sensor connected via I2C.
 *
 * Created on: Nov 12, 2014
 **/

#include <string.h>

#include <drivers/device/i2c.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/irlock_report.h>

/** Configuration Constants **/
#define IRLOCK_I2C_ADDRESS		0x54 /** 7-bit address (non shifted) **/
#define IRLOCK_CONVERSION_INTERVAL_US	20000U /** us = 20ms = 50Hz **/

#define IRLOCK_SYNC			0xAA55
#define IRLOCK_RESYNC		0x5500
#define IRLOCK_ADJUST		0xAA

#define IRLOCK_RES_X 320
#define IRLOCK_RES_Y 200

#define IRLOCK_CENTER_X				(IRLOCK_RES_X/2)			// the x-axis center pixel position
#define IRLOCK_CENTER_Y				(IRLOCK_RES_Y/2)			// the y-axis center pixel position

#define IRLOCK_FOV_X (60.0f*M_PI_F/180.0f)
#define IRLOCK_FOV_Y (35.0f*M_PI_F/180.0f)

#define IRLOCK_TAN_HALF_FOV_X 0.57735026919f // tan(0.5 * 60 * pi/180)
#define IRLOCK_TAN_HALF_FOV_Y 0.31529878887f // tan(0.5 * 35 * pi/180)

#define IRLOCK_TAN_ANG_PER_PIXEL_X	(2*IRLOCK_TAN_HALF_FOV_X/IRLOCK_RES_X)
#define IRLOCK_TAN_ANG_PER_PIXEL_Y	(2*IRLOCK_TAN_HALF_FOV_Y/IRLOCK_RES_Y)

#define IRLOCK_OBJECTS_MAX	5	/** up to 5 objects can be detected/reported **/

struct irlock_target_s {
	uint16_t signature;	/** target signature **/
	float pos_x;	/** x-axis distance from center of image to center of target in units of tan(theta) **/
	float pos_y;	/** y-axis distance from center of image to center of target in units of tan(theta) **/
	float size_x;	/** size of target along x-axis in units of tan(theta) **/
	float size_y;	/** size of target along y-axis in units of tan(theta) **/
};

/** irlock_s structure returned from read calls **/
struct irlock_s {
	hrt_abstime timestamp; /** microseconds since system start **/
	uint8_t num_targets;
	irlock_target_s targets[IRLOCK_OBJECTS_MAX];
};

class IRLOCK : public device::I2C, public I2CSPIDriver<IRLOCK>
{
public:
	IRLOCK(I2CSPIBusOption bus_option, const int bus, int bus_frequency, const int address);
	~IRLOCK() override = default;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	int init() override;
	int probe() override;
	void print_status() override;

	/** read from device and schedule next read **/
	void		RunImpl();
private:

	/** low level communication with sensor **/
	int 		read_device();
	bool 		sync_device();
	int 		read_device_word(uint16_t *word);
	int 		read_device_block(struct irlock_target_s *block);

	/** internal variables **/
	uint32_t _read_failures{0};

	uORB::Publication<irlock_report_s> _irlock_report_topic{ORB_ID(irlock_report)};
};

IRLOCK::IRLOCK(I2CSPIBusOption bus_option, const int bus, int bus_frequency, const int address) :
	I2C(DRV_SENS_DEVTYPE_IRLOCK, MODULE_NAME, bus, address, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus, address)
{
}

int IRLOCK::init()
{
	/** initialise I2C bus **/
	int ret = I2C::init();

	if (ret != OK) {
		return ret;
	}

	ScheduleNow();
	return OK;
}

int IRLOCK::probe()
{
	/*
	 * IRLock defaults to sending 0x00 when there is no block
	 * data to return, so really all we can do is check to make
	 * sure a transfer completes successfully.
	 **/
	uint8_t byte;

	if (transfer(nullptr, 0, &byte, 1) != OK) {
		return -EIO;
	}

	return OK;
}

void IRLOCK::print_status()
{
	PX4_INFO("read errors: %lu", (unsigned long)_read_failures);
}

void IRLOCK::RunImpl()
{
	/** ignoring failure, if we do, we will be back again right away... **/
	read_device();

	ScheduleDelayed(IRLOCK_CONVERSION_INTERVAL_US);
}

bool IRLOCK::sync_device()
{
	uint8_t sync_byte;
	uint16_t sync_word;

	if (read_device_word(&sync_word) != OK) {
		return false;
	}

	if (sync_word == IRLOCK_RESYNC) {
		transfer(nullptr, 0, &sync_byte, 1);

		if (sync_byte == IRLOCK_ADJUST) {
			return true;
		}

	} else if (sync_word == IRLOCK_SYNC) {
		return true;
	}

	return false;
}

int IRLOCK::read_device()
{
	/** if we sync, then we are starting a new frame, else fail **/
	if (!sync_device()) {
		return -ENOTTY;
	}

	irlock_s report{};
	report.timestamp = hrt_absolute_time();
	report.num_targets = 0;

	while (report.num_targets < IRLOCK_OBJECTS_MAX) {
		if (!sync_device() || read_device_block(&report.targets[report.num_targets]) != OK) {
			break;
		}

		report.num_targets++;
	}

	// publish over uORB
	if (report.num_targets > 0) {
		irlock_report_s orb_report{};
		orb_report.timestamp = report.timestamp;
		orb_report.signature = report.targets[0].signature;
		orb_report.pos_x     = report.targets[0].pos_x;
		orb_report.pos_y     = report.targets[0].pos_y;
		orb_report.size_x    = report.targets[0].size_x;
		orb_report.size_y    = report.targets[0].size_y;

		_irlock_report_topic.publish(orb_report);
	}

	return OK;
}

int IRLOCK::read_device_word(uint16_t *word)
{
	uint8_t bytes[2] {};

	int status = transfer(nullptr, 0, &bytes[0], 2);
	*word = bytes[1] << 8 | bytes[0];

	return status;
}

int IRLOCK::read_device_block(irlock_target_s *block)
{
	uint8_t bytes[12];
	memset(bytes, 0, sizeof bytes);

	int status = transfer(nullptr, 0, &bytes[0], 12);
	uint16_t checksum = bytes[1] << 8 | bytes[0];
	uint16_t signature = bytes[3] << 8 | bytes[2];
	uint16_t pixel_x = bytes[5] << 8 | bytes[4];
	uint16_t pixel_y = bytes[7] << 8 | bytes[6];
	uint16_t pixel_size_x = bytes[9] << 8 | bytes[8];
	uint16_t pixel_size_y = bytes[11] << 8 | bytes[10];

	/** crc check **/
	if (signature + pixel_x + pixel_y + pixel_size_x + pixel_size_y != checksum) {
		_read_failures++;
		return -EIO;
	}

	/** convert to angles **/
	block->signature = signature;
	block->pos_x = (pixel_x - IRLOCK_CENTER_X) * IRLOCK_TAN_ANG_PER_PIXEL_X;
	block->pos_y = (pixel_y - IRLOCK_CENTER_Y) * IRLOCK_TAN_ANG_PER_PIXEL_Y;
	block->size_x = pixel_size_x * IRLOCK_TAN_ANG_PER_PIXEL_X;
	block->size_y = pixel_size_y * IRLOCK_TAN_ANG_PER_PIXEL_Y;
	return status;
}

void IRLOCK::print_usage()
{
	PRINT_MODULE_USAGE_NAME("irlock", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x54);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

I2CSPIDriverBase *IRLOCK::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
				      int runtime_instance)
{
	IRLOCK *instance = new IRLOCK(iterator.configuredBusOption(), iterator.bus(), cli.bus_frequency, cli.i2c_address);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (instance->init() != PX4_OK) {
		delete instance;
		return nullptr;
	}

	return instance;
}

extern "C" __EXPORT int irlock_main(int argc, char *argv[])
{
	using ThisDriver = IRLOCK;
	BusCLIArguments cli{true, false};
	cli.i2c_address = IRLOCK_I2C_ADDRESS;
	cli.default_i2c_frequency = 400000;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_SENS_DEVTYPE_IRLOCK);

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
