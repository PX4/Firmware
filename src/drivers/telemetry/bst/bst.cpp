/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file bst.cpp
 *
 * Black Sheep Telemetry driver
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <drivers/device/i2c.h>
#include <systemlib/err.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <matrix/math.hpp>

using namespace matrix;
using namespace time_literals;

#define BST_ADDR		0x76

namespace px4
{
namespace bst
{

#pragma pack(push, 1)

template <typename T>
struct BSTPacket {
	uint8_t length;
	uint8_t type;
	T payload;
	uint8_t crc;
};

struct BSTDeviceInfoRequest {
	uint8_t cmd;
};

struct BSTDeviceInfoReply {
	uint32_t hw_id;
	uint16_t fw_id;
	uint8_t dev_name_len;
	char dev_name[32];
};

struct BSTGPSPosition {
	int32_t lat;
	int32_t lon;
	uint16_t gs;
	uint16_t heading;
	uint16_t alt;
	uint8_t sats;
};

struct BSTAttitude {
	int16_t pitch;
	int16_t roll;
	int16_t yaw;
};

struct BSTBattery {
	uint16_t voltage;
	uint16_t current;
	uint8_t capacity[3];
};

#pragma pack(pop)

class BST : public device::I2C, public I2CSPIDriver<BST>
{
public:
	BST(const I2CSPIDriverConfig &config);
	~BST() override = default;

	static void print_usage();

	int		init() override;

	int		probe() override;

	void			RunImpl();
private:

	static constexpr unsigned		_interval{100_ms};

	uORB::Subscription	_gps_sub{ORB_ID(vehicle_gps_position)};
	uORB::Subscription	_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription	_battery_sub{ORB_ID(battery_status)};


	template <typename T>
	void			send_packet(BSTPacket<T> &packet)
	{
		packet.length = sizeof(packet) - 1;	// Length
		packet.crc = crc8(reinterpret_cast<uint8_t *>(&packet.type), sizeof(packet) - 2);

		transfer(reinterpret_cast<uint8_t *>(&packet), sizeof(packet), nullptr, 0);
	}

	template <typename T_SEND, typename T_RECV>
	void				send_packet(BSTPacket<T_SEND> &packet_send, BSTPacket<T_RECV> &packet_recv)
	{
		packet_send.length = sizeof(packet_send) - 1;	// Length
		packet_send.crc = crc8(reinterpret_cast<uint8_t *>(&packet_send.type), sizeof(packet_send) - 2);
		transfer(reinterpret_cast<uint8_t *>(&packet_send), sizeof(packet_send), reinterpret_cast<uint8_t *>(&packet_recv),
			 sizeof(packet_recv));
	}

	static uint8_t	crc8(uint8_t *data, size_t len);

	//! Byte swap unsigned short
	uint16_t swap_uint16(uint16_t val)
	{
		return (val << 8) | (val >> 8);
	}

	//! Byte swap short
	int16_t swap_int16(int16_t val)
	{
		return (val << 8) | ((val >> 8) & 0xFF);
	}

	//! Byte swap unsigned int
	uint32_t swap_uint32(uint32_t val)
	{
		val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF);
		return (val << 16) | (val >> 16);
	}

	//! Byte swap int
	int32_t swap_int32(int32_t val)
	{
		val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF);
		return (val << 16) | ((val >> 16) & 0xFFFF);
	}
};

BST::BST(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config)
{
}

int BST::probe()
{
	BSTPacket<BSTDeviceInfoRequest> dev_info_req = {};
	dev_info_req.type = 0x0A;
	dev_info_req.payload.cmd = 0x04;
	BSTPacket<BSTDeviceInfoReply> dev_info_reply = {};
	send_packet(dev_info_req, dev_info_reply);

	if (dev_info_reply.type != 0x05) {
		PX4_ERR("no devices found");
		return -EIO;
	}

	uint8_t *reply_raw = reinterpret_cast<uint8_t *>(&dev_info_reply);
	uint8_t crc_calc = crc8(reinterpret_cast<uint8_t *>(&dev_info_reply.type), dev_info_reply.length - 1);
	uint8_t crc_recv = reply_raw[dev_info_reply.length];

	if (crc_recv != crc_calc) {
		PX4_ERR("CRC error: got %02x, should be %02x", (int)crc_recv, (int)crc_calc);
		return -EIO;
	}

	dev_info_reply.payload.dev_name[dev_info_reply.payload.dev_name_len] = '\0';
	PX4_DEBUG("device info: hardware ID: 0x%08X, firmware ID: 0x%04X, device name: %s",
		  (int)swap_uint32(dev_info_reply.payload.hw_id), (int)swap_uint16(dev_info_reply.payload.fw_id),
		  dev_info_reply.payload.dev_name);

	_retries = 1;

	return OK;
}

int BST::init()
{
	int ret = I2C::init();

	if (ret != OK) {
		return ret;
	}

	ScheduleNow();

	return OK;
}

void BST::RunImpl()
{
	if (_attitude_sub.updated()) {
		vehicle_attitude_s att;
		_attitude_sub.copy(&att);
		Quatf q(att.q);
		Eulerf euler(q);

		BSTPacket<BSTAttitude> bst_att = {};
		bst_att.type = 0x1E;
		bst_att.payload.roll = swap_int32(euler.phi() * 10000);
		bst_att.payload.pitch = swap_int32(euler.theta() * 10000);
		bst_att.payload.yaw = swap_int32(euler.psi() * 10000);

		send_packet(bst_att);
	}

	if (_battery_sub.updated()) {
		battery_status_s batt;
		_battery_sub.copy(&batt);

		BSTPacket<BSTBattery> bst_batt = {};
		bst_batt.type = 0x08;
		bst_batt.payload.voltage = swap_uint16(batt.voltage_v * 10.0f);
		bst_batt.payload.current = swap_uint16(batt.current_a * 10.0f);
		uint32_t discharged = batt.discharged_mah;
		bst_batt.payload.capacity[0] = static_cast<uint8_t>(discharged >> 16);
		bst_batt.payload.capacity[1] = static_cast<uint8_t>(discharged >> 8);
		bst_batt.payload.capacity[2] = static_cast<uint8_t>(discharged);

		send_packet(bst_batt);
	}

	if (_gps_sub.updated()) {
		vehicle_gps_position_s gps;
		_gps_sub.copy(&gps);

		if (gps.fix_type >= 3 && gps.eph < 50.0f) {
			BSTPacket<BSTGPSPosition> bst_gps = {};
			bst_gps.type = 0x02;
			bst_gps.payload.lat = swap_int32(gps.lat);
			bst_gps.payload.lon = swap_int32(gps.lon);
			bst_gps.payload.alt = swap_int16(gps.alt / 1000 + 1000);
			bst_gps.payload.gs = swap_int16(gps.vel_m_s * 360.0f);
			bst_gps.payload.heading = swap_int16(gps.cog_rad * 18000.0f / M_PI_F);
			bst_gps.payload.sats = gps.satellites_used;

			send_packet(bst_gps);
		}
	}

	ScheduleDelayed(_interval);
}

uint8_t BST::crc8(uint8_t *data, size_t len)
{
	uint8_t crc = 0x00;

	while (len--) {
		crc ^= *data++;

		for (int i = 0; i < 8; i++) {
			crc = crc & 0x80 ? (crc << 1) ^ 0xD5 : crc << 1;
		}
	}

	return crc;
}

}
}

using namespace px4::bst;

void
BST::print_usage()
{
	PRINT_MODULE_USAGE_NAME("bst", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x76);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int bst_main(int argc, char *argv[])
{
	using ThisDriver = BST;
	BusCLIArguments cli{true, false};
	cli.i2c_address = BST_ADDR;
	cli.default_i2c_frequency = 100000;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_TEL_DEVTYPE_BST);

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
