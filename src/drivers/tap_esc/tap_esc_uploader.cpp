/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file tap_esc_uploader.cpp
 * firmware uploader for TAP ESC
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>

#include <lib/parameters/param.h>
#include <lib/systemlib/px4_macros.h>
#include <lib/systemlib/mavlink_log.h>

#include <sys/types.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <sys/stat.h>
#include <nuttx/arch.h>
#include <crc32.h>
#include <drivers/drv_hrt.h>
#include <sys/ioctl.h>

#include "drv_tap_esc.h"
#include "tap_esc_uploader.h"
#include "tap_esc_common.h"

// define for comms logging
//#define UDEBUG

const uint8_t TAP_ESC_UPLOADER::_device_mux_map[TAP_ESC_MAX_MOTOR_NUM] = ESC_POS;

TAP_ESC_UPLOADER::TAP_ESC_UPLOADER(const char *device, uint8_t esc_counter) :
	_esc_fd(-1),
	_fw_fd(-1),
	_device(device),
	_esc_counter(esc_counter),
	_uploader_packet{},
	_mavlink_log_pub(nullptr)
{

}

TAP_ESC_UPLOADER::~TAP_ESC_UPLOADER()
{
	tap_esc_common::deinitialise_uart(_esc_fd);
}

int32_t
TAP_ESC_UPLOADER::initialise_firmware_file(const char *filenames[], int &fw_fd)
{
	const char *filename = NULL;

	/* allow an early abort and look for file first */
	for (unsigned i = 0; filenames[i] != nullptr; i++) {
		fw_fd = open(filenames[i], O_RDONLY);

		if (fw_fd < 0) {
			PX4_DEBUG("failed to open %s", filenames[i]);
			continue;
		}

		PX4_DEBUG("using firmware from %s", filenames[i]);
		filename = filenames[i];
		break;
	}

	if (filename == NULL) {
		PX4_DEBUG("no firmware found");
		return -ENOENT;
	}

	struct stat st;

	if (stat(filename, &st) != 0) {
		PX4_DEBUG("Failed to stat %s - %d\n", filename, (int)errno);
		return -errno;
	}

	if (fw_fd == -1) {
		return -ENOENT;
	}

	return st.st_size;
}

int
TAP_ESC_UPLOADER::upload_id(uint8_t esc_id, int32_t fw_size)
{
	int	ret = -1;

	/******************************************
	 * first:send sync
	 ******************************************/
	PX4_DEBUG("uploader esc_id %d...", esc_id);

	/* look for the bootloader, blocking 320 ms */
	for (int i = 0; i < SYNC_RETRY_TIMES; i++) {
		ret = sync(esc_id);

		if (ret == OK) {
			break;

		} else {
			/* send sync interval time: 50ms */
			usleep(50000);
		}
	}

	if (ret != OK) {
		/* this is immediately fatal */
		PX4_DEBUG("esc_id %d bootloader not responding", esc_id);
		return -EIO;
	}

	/* do the usual program thing - allow for failure */
	for (unsigned retries = 0; retries < UPLOADER_RETRY_TIMES; retries++) {
		if (retries > 0) {
			PX4_DEBUG("esc_id %d retrying update...", esc_id);
			ret = sync(esc_id);

			if (ret != OK) {
				/* this is immediately fatal */
				PX4_DEBUG("esc_id %d bootloader not responding", esc_id);
				return -EIO;
			}
		}

		/******************************************
		* second: get device bootloader revision
		 ******************************************/
		uint32_t bl_rev;
		ret = get_device_info(esc_id, PROTO_GET_DEVICE, PROTO_DEVICE_BL_REV, bl_rev);

		if (ret == OK) {
			if (bl_rev <= PROTO_SUPPORT_BL_REV * 100) {
				PX4_DEBUG("esc_id %d found bootloader revision: %4.4f", esc_id, (double)bl_rev / 100);

			} else {
				PX4_DEBUG("esc_id %d found unsupported bootloader revision %4.4f, exiting", esc_id, (double)bl_rev / 100);
				return EPERM;
			}

		} else {
			PX4_DEBUG("esc_id %d found bootloader revision failed", esc_id);
		}

		/******************************************
		* third: erase program
		 ******************************************/
		ret = erase(esc_id);

		if (ret != OK) {
			PX4_DEBUG("esc_id %d %d erase failed", esc_id, ret);
			continue;
		}

		/******************************************
		* fourth: program
		 ******************************************/
		ret = program(esc_id, fw_size);

		if (ret != OK) {
			PX4_DEBUG("esc_id %d program failed", esc_id);
			continue;
		}

		/******************************************
		* fifth: verify flash crc
		 *****************************************/
		ret = verify_crc(esc_id, fw_size);

		if (ret != OK) {
			PX4_DEBUG("verify failed");
			continue;
		}

		/******************************************
		* sixth: reboot tap esc
		 *****************************************/
		ret = reboot(esc_id);

		if (ret != OK) {
			PX4_DEBUG("reboot failed,reboot again");
			ret = reboot(esc_id);
			usleep(2000);

			if (ret != OK) {
				PX4_DEBUG("reboot failed");
				return ret;
			}
		}

		PX4_DEBUG("esc_id %d uploader complete", esc_id);

		ret = OK;
		break;
	}

	return OK;
}

int
TAP_ESC_UPLOADER::upload(const char *filenames[])
{
	int	ret = -1;
	int32_t fw_size;
	uint16_t esc_fail_mask = 0;

	fw_size = initialise_firmware_file(filenames, _fw_fd);

	if (fw_size < 0) {
		PX4_DEBUG("initialise firmware file failed");
		return fw_size;
	}

	ret = tap_esc_common::initialise_uart(_device, _esc_fd);

	if (ret < 0) {
		PX4_DEBUG("initialise uart failed %s");
		return ret;
	}

	/* uploader esc_id(0,1,2,3,4,5), uploader begin esc id0 */
	for (unsigned esc_id = 0; esc_id < _esc_counter; esc_id++) {

		ret = upload_id(esc_id, fw_size);

		if (ret != OK) {
			esc_fail_mask = 1 << esc_id;
		}
	}

	// check all esc upload success after all esc uploading end,it will upload esc again which esc upload failed
	for (unsigned esc_id = 0; esc_id < _esc_counter; esc_id++) {

		if (esc_fail_mask & (1 << esc_id)) {
			ret = upload_id(esc_id, fw_size);
		}
	}

	esc_fail_mask = 0;

	// sleep for enough time for the TAP ESC chip to boot. This makes
	// update more reliably startup TAP ESC again after upload
	// set wait time for tap esc form reboot jump app(0.1269s measure by Saleae logic Analyzer)
	usleep(130 * 1000);

	return ret;
}

int
TAP_ESC_UPLOADER::log_versions()
{
	param_t param_esc_fw_ver = param_find("ESC_FW_VER");
	param_t param_esc_hw_ver = param_find("ESC_HW_VER");
	param_t param_esc_bl_ver = param_find("ESC_BL_VER");

	if (param_esc_fw_ver == PARAM_INVALID ||
	    param_esc_hw_ver == PARAM_INVALID ||
	    param_esc_bl_ver == PARAM_INVALID) {
		PX4_WARN("Cannot find one or more parameters, unable to log ESC versions.");
		return -1;
	}

	int ret = tap_esc_common::initialise_uart(_device, _esc_fd);

	if (ret < 0) {
		PX4_DEBUG("initialise uart failed %s");
		return ret;
	}

	// get information from first ESC
	uint32_t fw_ver, hw_ver, bl_ver;
	ret =  get_esc_versions(0, fw_ver, hw_ver, bl_ver);

	if (ret != OK) {
		PX4_DEBUG("Failed to get ESC 0 device info");
		return ret;
	}

	/* Get firmware versions of the remainig ESCs and compare with the first one
	   Since the parameters can only store one version (not six), we need to make
	   sure that all ESCs have matchin version numbers. If not, all version
	   parameters will be set to zero. */
	bool esc_versions_matching = true;

	for (unsigned esc_id = 1; esc_id < _esc_counter; esc_id++) {
		uint32_t temp_fw_ver, temp_hw_ver, temp_bl_ver;
		ret =  get_esc_versions(esc_id, temp_fw_ver, temp_hw_ver, temp_bl_ver);

		if (ret != OK) {
			PX4_DEBUG("Failed to get ESC %u device info", esc_id);
			return ret;
		}

		if (fw_ver != temp_fw_ver || hw_ver != temp_hw_ver ||  bl_ver != temp_bl_ver) {
			esc_versions_matching = false;
			break;
		}
	}

	if (!esc_versions_matching) {
		// One or more ESCs have mismatching versions
		fw_ver = 0; // version 0 means unknown
		hw_ver = 0; // version 0 means unknown
		bl_ver = 0; // version 0 means unknown
	}

	param_set(param_esc_fw_ver, &fw_ver);
	param_set(param_esc_hw_ver, &hw_ver);
	param_set(param_esc_bl_ver, &bl_ver);

	return ret;
}

int
TAP_ESC_UPLOADER::checkcrc(const char *filenames[])
{
	/*
	  check tap_esc flash CRC against CRC of a file
	 */
	int32_t fw_size;
	int ret = -1;
	uint16_t esc_fail_mask = 0;

	fw_size = initialise_firmware_file(filenames, _fw_fd);

	if (fw_size < 0) {
		PX4_ERR("initialise firmware file failed");
		return fw_size;
	}

	ret = tap_esc_common::initialise_uart(_device, _esc_fd);

	if (ret < 0) {
		PX4_DEBUG("initialise uart failed %s");
		return ret;
	}

	/* checkcrc esc_id(0,1,2,3,4,5), checkcrc begin esc id0 */
	for (unsigned esc_id = 0; esc_id < _esc_counter; esc_id++) {
		/* look for the bootloader, blocking 320 ms,uploader begin esc id0*/
		for (int i = 0; i < SYNC_RETRY_TIMES; i++) {
			ret = sync(esc_id);

			if (ret == OK) {
				break;

			} else {
				/* send sync interval time: 50ms */
				usleep(50000);
			}
		}

		if (ret != OK) {
			/* this is immediately fatal */
			PX4_DEBUG("esc_id %d bootloader not responding", esc_id);
			return -EIO;
		}

		uint32_t temp_revision;

		/* get device bootloader revision */
		ret = get_device_info(esc_id, PROTO_GET_DEVICE, PROTO_DEVICE_BL_REV, temp_revision);

		double test_esc_id;
		test_esc_id = esc_id;

		if (ret == OK) {
			mavlink_log_info(&_mavlink_log_pub, "esc_id %1.f found bootloader revision: %4.4f", test_esc_id,
					 (double)temp_revision / 100);

		} else {
			mavlink_log_info(&_mavlink_log_pub, "esc_id %1.f found bootloader revision failed", test_esc_id);
		}

		/* get device firmware revision */
		ret = get_device_info(esc_id, PROTO_GET_DEVICE, PROTO_DEVICE_FW_REV, temp_revision);

		if (ret == OK) {
			mavlink_log_info(&_mavlink_log_pub, "esc_id %1.f found firmware revision: %4.4f", test_esc_id,
					 (double)temp_revision / 100);

		} else {
			mavlink_log_info(&_mavlink_log_pub, "esc_id %1.f found firmware revision failed", test_esc_id);
		}

		/* get device hardware revision */
		ret = get_device_info(esc_id, PROTO_GET_DEVICE, PROTO_DEVICE_BOARD_REV, temp_revision);

		if (ret == OK) {
			mavlink_log_info(&_mavlink_log_pub, "esc_id %1.f found board revision: %02x", test_esc_id, temp_revision);

		}  else {
			mavlink_log_info(&_mavlink_log_pub, "esc_id %1.f found board revision failed", test_esc_id);
		}

		/* compare esc flash crc with .bin file crc */
		ret = verify_crc(esc_id, fw_size);

		if (ret == -EINVAL) {
			mavlink_log_info(&_mavlink_log_pub, "esc_id %1.f check CRC is different,will upload tap esc firmware ",
					 test_esc_id);
			ret = upload_id(esc_id, fw_size);

			if (ret != OK) {
				esc_fail_mask = 1 << esc_id;
			}

		} else {
			/* reboot tap esc_id */
			ret = reboot(esc_id);

			if (ret != OK) {
				PX4_DEBUG("reboot failed,reboot again");
				ret = reboot(esc_id);
				usleep(2000);

				if (ret != OK) {
					PX4_DEBUG("reboot failed");
					return ret;
				}
			}
		}
	}

	// check all esc upload success after all esc uploading end,it will upload esc again which esc upload failed
	for (unsigned esc_id = 0; esc_id < _esc_counter; esc_id++) {

		if (esc_fail_mask & (1 << esc_id)) {
			ret = upload_id(esc_id, fw_size);
		}
	}

	esc_fail_mask = 0;

	// sleep for enough time for the TAP ESC chip to boot. This makes
	// update more reliably startup TAP ESC again after upload
	// set wait time for tap esc form rebbot jump app(0.1269s measure by Saleae logic Analyzer)
	usleep(130 * 1000);

	return ret;
}

int
TAP_ESC_UPLOADER::recv_byte_with_timeout(uint8_t *c, unsigned timeout)
{
	struct pollfd fds[1];

	fds[0].fd = _esc_fd;
	fds[0].events = POLLIN;

	/* wait <timout> ms for a character */
	int ret = ::poll(&fds[0], 1, timeout);

	if (ret < 1) {
#ifdef UDEBUG
		PX4_DEBUG("poll timeout %d", ret);
#endif
		return -ETIMEDOUT;
	}

	read(_esc_fd, c, 1);
#ifdef UDEBUG
	PX4_DEBUG("recv_bytes 0x%02x", c);
#endif
	return OK;
}

int
TAP_ESC_UPLOADER::read_and_parse_data(unsigned timeout)
{
	uint8_t c = 0;
	int ret = -1;

	/* set a timeout Ms for a first byte */
	ret = recv_byte_with_timeout(&c, timeout);

	if (ret == OK) {
		do {
			/* try to parse the message from esc */
			ret = parse_tap_esc_feedback(c, &_uploader_packet);

			/* parse success all data */
			if (ret == OK) {
				return OK;
			}

			/* read a byte and buffer it with poll and a timeout 2Ms */
			ret = recv_byte_with_timeout(&c, 2);

			/* timeout in 1 is exceeded */
			if (ret == -ETIMEDOUT) {
				PX4_DEBUG("parse timeout %d data %d", ret, c);
				return ret;
			}

		} while (true);

	}

	return ret;
}

int
TAP_ESC_UPLOADER::parse_tap_esc_feedback(uint8_t decode_data, EscUploaderMessage *packetdata)
{
	static PARSR_ESC_STATE state = HEAD;
	static uint8_t data_index = 0;
	static uint8_t crc_data_cal;

#ifdef UDEBUG
	PX4_DEBUG("decode data 0x%02x", decode_data);
#endif

	switch (state) {
	case HEAD:
		if (decode_data == PACKET_HEAD) {
			packetdata->head = PACKET_HEAD; //just_keep the format
			state = LEN;

		}

		break;

	case LEN:
		if (decode_data < sizeof(packetdata->d)) {
			packetdata->len = decode_data;
			state = ID;

		} else {
			state = HEAD;
		}

		break;

	case ID:
		if (decode_data < PROTO_MSG_ID_MAX_NUM) {
			packetdata->msg_id = decode_data;
			data_index = 0;
			state = DATA;

		} else {
			state = HEAD;
		}

		break;

	case DATA:
		packetdata->d.data[data_index++] = decode_data;

		if (data_index >= packetdata->len) {
			crc_data_cal = crc8_esc((uint8_t *)(&packetdata->len), packetdata->len + 2);
			state = CRC;

		}

		break;

	case CRC:
		if (crc_data_cal == decode_data) {
			packetdata->crc_data = decode_data;
			state = HEAD;
			return OK;

		}

		state = HEAD;
		break;

	default:
		state = HEAD;
		break;

	}

	return EPROTO;
}
uint8_t
TAP_ESC_UPLOADER::crc8_esc(uint8_t *p, uint8_t len)
{
	uint8_t crc = 0;

	for (uint8_t i = 0; i < len; i++) {
		crc = tap_esc_common::crc_table[crc^*p++];
	}

	return crc;
}

uint8_t
TAP_ESC_UPLOADER::crc_packet(EscUploaderMessage &p)
{
	/* Calculate the crc include Len,ID,data */
	p.d.data[p.len] = crc8_esc(&p.len, p.len + 2);
	return p.len + offsetof(EscUploaderMessage, d) + 1;
}

int
TAP_ESC_UPLOADER::send_packet(EscUploaderMessage &packet, int responder)
{
	if (responder >= 0) {

		if (responder > _esc_counter) {
			return -EINVAL;
		}

		/* For messages with id ESCBUS_MSG_ID_REQUEST_INFO we need to send
		   the channel ID, not the ESC hardware ID */
		int esc_select_id = ((packet.msg_id == ESCBUS_MSG_ID_REQUEST_INFO) ? responder : _device_mux_map[responder]);
		tap_esc_common::select_responder(esc_select_id);
	}

	int packet_len = crc_packet(packet);
	int ret = ::write(_esc_fd, (uint8_t *)&packet.head, packet_len);

#ifdef UDEBUG
	uint8_t *buf = (uint8_t *)&packet;

	for (int i = 0; i < packet_len; i++) {
		PX4_DEBUG("buf[%d] 0x%02x %d", i, buf[i], ret);
	}

#endif

	if (ret != packet_len) {
#ifdef UDEBUG
		PX4_DEBUG("TX ERROR: ret: %d, errno: %d", ret, errno);
#endif
		return errno;
	}

	return OK;
}

int
TAP_ESC_UPLOADER::sync(uint8_t esc_id)
{
	int ret;

	/* send sync packet */
	EscUploaderMessage sync_packet = {PACKET_HEAD, sizeof(EscbusBootSyncPacket), PROTO_GET_SYNC};
	sync_packet.d.sync_packet.myID = esc_id;
	send_packet(sync_packet, esc_id);

	/* read and parse sync feedback packet, blocking 320ms between esc app jump boot,but if esc run boot,it waits time less than 2ms */
	/* set wait time for tap esc form app jump reboot(0.311748s measure by Saleae logic Analyzer) */
	ret = read_and_parse_data(320);

	if (ret != OK) {
		return ret;
	}

	/* check sync feedback is ok or fail */
	if (_uploader_packet.msg_id == PROTO_OK) {
		if (_uploader_packet.d.feedback_packet.myID != esc_id) {
			PX4_DEBUG("sync don't match myID: 0x%02x, esc_id: 0x%02x", _uploader_packet.d.feedback_packet.myID, esc_id);
			return -EIO;
		}

		if (_uploader_packet.d.feedback_packet.command != PROTO_GET_SYNC) {
			PX4_DEBUG("bad sync myID: 0x%02x, command: 0x%02x", _uploader_packet.d.feedback_packet.myID,
				  _uploader_packet.d.feedback_packet.command);
			return -EIO;
		}

	} else if (_uploader_packet.msg_id == PROTO_INVALID) {
		PX4_DEBUG("sync invalid: don't receive sync invalid: myID: 0x%02x, esc_id: 0x%02x",
			  _uploader_packet.d.feedback_packet.myID,
			  esc_id);
		return -EIO;

	} else if (_uploader_packet.msg_id == PROTO_FAILED) {
		PX4_DEBUG("sync failed: don't receive sync failed: myID: 0x%02x, esc_id: 0x%02x",
			  _uploader_packet.d.feedback_packet.myID,
			  esc_id);
		return -EIO;

	}

	return OK;
}

// TODO: It is weird, that we have a separate function to request the ESC
// versions when there is also the get_device_info() function. Problem is, that
// the versions request will return three values instead of only one, hence
// the need for this new function here. Ideally we would utilize
// get_device_info() for all requests...
int
TAP_ESC_UPLOADER::get_esc_versions(uint8_t esc_id, uint32_t &fw_ver, uint32_t &hw_ver, uint32_t &bl_ver)
{
	int ret;

	/*prepare information request packet */
	EscUploaderMessage device_info_packet = {PACKET_HEAD, sizeof(EscbusGetDevicePacket), ESCBUS_MSG_ID_REQUEST_INFO};
	device_info_packet.d.device_info_packet.myID = esc_id;
	device_info_packet.d.device_info_packet.deviceInfo = REQUEST_INFO_DEVICE;
	send_packet(device_info_packet, esc_id);

	/* read and parse device information feedback packet, blocking 50ms */
	ret = read_and_parse_data();

	if (ret != OK) {
		return ret;
	}

	/* check device information feedback is ok or fail */
	if (_uploader_packet.msg_id == ESCBUS_MSG_ID_DEVICE_INFO) {
		if (_uploader_packet.d.esc_version_packet.myID != esc_id) {
			PX4_DEBUG("get device firmware revision: ID mismatch, received: 0x%02x, asked for: 0x%02x",
				  _uploader_packet.d.esc_version_packet.myID, esc_id);
			return -EIO;
		}

		fw_ver = _uploader_packet.d.esc_version_packet.FwRev;
		hw_ver = _uploader_packet.d.esc_version_packet.HwRev;
		bl_ver = _uploader_packet.d.esc_version_packet.blRev;

	} else if (_uploader_packet.msg_id == PROTO_FAILED) {
		PX4_DEBUG("Failed to get ESC versions, ESC ID: 0x%02x, we sent to ID: 0x%02x, FwRev: 0x%02x",
			  _uploader_packet.d.esc_version_packet.myID, esc_id,
			  _uploader_packet.d.esc_version_packet.FwRev);
		return -EIO;

	} else if (_uploader_packet.msg_id == PROTO_INVALID) {
		PX4_DEBUG("Failed to get ESC versions, ESC reports invalid protocl, ESC ID: 0x%02x, we sent to ID: 0x%02x, FwRev: 0x%02x",
			  _uploader_packet.d.esc_version_packet.myID, esc_id,
			  _uploader_packet.d.esc_version_packet.FwRev);
		return -EIO;

	}

	return OK;
}

int
TAP_ESC_UPLOADER::get_device_info(uint8_t esc_id, uint8_t msg_id, uint8_t msg_arg, uint32_t &val)
{
	int ret;

	/*prepare information request packet */
	EscUploaderMessage device_info_packet = {PACKET_HEAD, sizeof(EscbusGetDevicePacket), msg_id};
	device_info_packet.d.device_info_packet.myID = esc_id;
	device_info_packet.d.device_info_packet.deviceInfo = msg_arg;
	send_packet(device_info_packet, esc_id);

	/* read and parse device information feedback packet, blocking 50ms */
	ret = read_and_parse_data();

	if (ret != OK) {
		return ret;
	}

	/* check device information feedback is ok or fail */
	switch (msg_arg) {

	case PROTO_DEVICE_BL_REV:
		if (_uploader_packet.msg_id == PROTO_OK) {
			if (_uploader_packet.d.bootloader_revis_packet.myID != esc_id) {
				PX4_DEBUG("get device bootloader revision id don't match, myID: 0x%02x, esc_id: 0x%02x",
					  _uploader_packet.d.bootloader_revis_packet.myID, esc_id);
				return -EIO;
			}

			val = _uploader_packet.d.bootloader_revis_packet.version;

		} else if (_uploader_packet.msg_id == PROTO_FAILED) {
			PX4_DEBUG("get device bootloader revision failed, myID: 0x%02x, esc_id: 0x%02x, ver: 0x%02x",
				  _uploader_packet.d.bootloader_revis_packet.myID, esc_id,
				  _uploader_packet.d.bootloader_revis_packet.version);
			return -EIO;

		} else if (_uploader_packet.msg_id == PROTO_INVALID) {
			PX4_DEBUG("get device bootloader revision invalid, myID: 0x%02x, esc_id: 0x%02x, ver: 0x%02x",
				  _uploader_packet.d.bootloader_revis_packet.myID, esc_id,
				  _uploader_packet.d.bootloader_revis_packet.version);
			return -EIO;

		}

		break;

	case PROTO_DEVICE_BOARD_ID:
		if (_uploader_packet.msg_id == PROTO_OK) {
			if (_uploader_packet.d.hardware_id_packet.myID != esc_id) {
				PX4_DEBUG("get device hardware_id id don't match, myID: 0x%02x, esc_id: 0x%02x",
					  _uploader_packet.d.hardware_id_packet.myID,
					  esc_id);
				return -EIO;
			}

			val = _uploader_packet.d.hardware_id_packet.targetSystemId;

		} else if (_uploader_packet.msg_id == PROTO_FAILED) {
			PX4_DEBUG("get device hardware id failed, myID: 0x%02x, esc_id: 0x%02x, tar: 0x%02x, tar: 0x%02x",
				  _uploader_packet.d.hardware_id_packet.myID, esc_id,
				  _uploader_packet.d.hardware_id_packet.targetSystemId);
			return -EIO;

		} else if (_uploader_packet.msg_id == PROTO_INVALID) {
			PX4_DEBUG("get device hardware id invalid, myID: 0x%02x, esc_id: 0x%02x, tar: 0x%02x, tar: 0x%02x",
				  _uploader_packet.d.hardware_id_packet.myID, esc_id,
				  _uploader_packet.d.hardware_id_packet.targetSystemId);
			return -EIO;

		}

		break;

	case PROTO_DEVICE_BOARD_REV:
		if (_uploader_packet.msg_id == PROTO_OK) {
			if (_uploader_packet.d.hardware_revis_packet.myID != esc_id) {
				PX4_DEBUG("get device hardware revision id don't match, myID: 0x%02x, esc_id: 0x%02x",
					  _uploader_packet.d.hardware_revis_packet.myID, esc_id);
				return -EIO;
			}

			val = _uploader_packet.d.hardware_revis_packet.boardRev;

		} else if (_uploader_packet.msg_id == PROTO_FAILED) {
			PX4_DEBUG("get device hardware revision failed, myID: 0x%02x, esc_id: 0x%02x, boardRev: 0x%02x",
				  _uploader_packet.d.hardware_revis_packet.myID, esc_id,
				  _uploader_packet.d.hardware_revis_packet.boardRev);
			return -EIO;

		} else if (_uploader_packet.msg_id == PROTO_INVALID) {
			PX4_DEBUG("get device hardware revision invalid, myID: 0x%02x, esc_id: 0x%02x, boardRev: 0x%02x",
				  _uploader_packet.d.hardware_revis_packet.myID, esc_id,
				  _uploader_packet.d.hardware_revis_packet.boardRev);
			return -EIO;

		}

		break;

	case PROTO_DEVICE_FW_SIZE:
		if (_uploader_packet.msg_id == PROTO_OK) {
			if (_uploader_packet.d.firmware_size_packet.myID != esc_id) {
				PX4_DEBUG("get device firmware size id don't match, myID: 0x%02x, esc_id: 0x%02x",
					  _uploader_packet.d.firmware_size_packet.myID, esc_id);
				return -EIO;
			}

			val = _uploader_packet.d.firmware_size_packet.FwSize;

		} else if (_uploader_packet.msg_id == PROTO_FAILED) {
			PX4_DEBUG("get device firmware size failed, myID: 0x%02x, esc_id: 0x%02x, FwSize:  0x%02x",
				  _uploader_packet.d.firmware_size_packet.myID, esc_id,
				  _uploader_packet.d.firmware_size_packet.FwSize);
			return -EIO;

		} else if (_uploader_packet.msg_id == PROTO_INVALID) {
			PX4_DEBUG("get device firmware size invalid, myID: 0x%02x, esc_id: 0x%02x, FwSize:  0x%02x",
				  _uploader_packet.d.firmware_size_packet.myID, esc_id,
				  _uploader_packet.d.firmware_size_packet.FwSize);
			return -EIO;

		}

		break;

	case REQUEST_INFO_DEVICE:
		if (_uploader_packet.msg_id == ESCBUS_MSG_ID_DEVICE_INFO) {
			if (_uploader_packet.d.firmware_revis_packet.myID != esc_id) {
				PX4_DEBUG("get device firmware revision id don't match, myID: 0x%02x, esc_id: 0x%02x",
					  _uploader_packet.d.firmware_revis_packet.myID, esc_id);
				return -EIO;
			}

			val = _uploader_packet.d.firmware_revis_packet.FwRev;

		}

		break;

	case PROTO_DEVICE_FW_REV:
		if (_uploader_packet.msg_id == PROTO_OK) {
			if (_uploader_packet.d.firmware_revis_packet.myID != esc_id) {
				PX4_DEBUG("get device firmware revision id don't match, myID: 0x%02x, esc_id: 0x%02x",
					  _uploader_packet.d.firmware_revis_packet.myID, esc_id);
				return -EIO;
			}

			val = _uploader_packet.d.firmware_revis_packet.FwRev;

		} else if (_uploader_packet.msg_id == PROTO_FAILED) {
			PX4_DEBUG("get device firmware revision failed, myID: 0x%02x,esc_id: 0x%02x, FwRev: 0x%02x",
				  _uploader_packet.d.firmware_revis_packet.myID, esc_id,
				  _uploader_packet.d.firmware_revis_packet.FwRev);
			return -EIO;

		} else if (_uploader_packet.msg_id == PROTO_INVALID) {
			PX4_DEBUG("get device firmware revision invalid, myID: 0x%02x,esc_id: 0x%02x, FwRev: 0x%02x",
				  _uploader_packet.d.firmware_revis_packet.myID, esc_id,
				  _uploader_packet.d.firmware_revis_packet.FwRev);
			return -EIO;

		}

		break;

	default:
		break;
	}

	return OK;
}

int
TAP_ESC_UPLOADER::erase(uint8_t esc_id)
{
	int ret;
	PX4_DEBUG("erase...");

	/* send erase packet */
	EscUploaderMessage erase_packet = {PACKET_HEAD, sizeof(EscbusBootErasePacket), PROTO_CHIP_ERASE};
	erase_packet.d.erase_packet.myID = esc_id;
	send_packet(erase_packet, esc_id);

	/* read and parse erase feedback packet, blocking 500ms */
	ret = read_and_parse_data(500);

	if (ret != OK) {
		return ret;
	}

	/* check erase feedback is ok or fail */
	if (_uploader_packet.msg_id == PROTO_OK) {
		if (_uploader_packet.d.feedback_packet.myID != esc_id) {
			PX4_DEBUG("erase id don't match myID: 0x%02x,esc_id: 0x%02x", _uploader_packet.d.feedback_packet.myID, esc_id);
			return -EIO;
		}

		if (_uploader_packet.d.feedback_packet.command != PROTO_CHIP_ERASE) {
			PX4_DEBUG("erase bad command, myID: 0x%02x,command: 0x%02x", _uploader_packet.d.feedback_packet.myID,
				  _uploader_packet.d.feedback_packet.command);
			return -EIO;
		}

	} else if (_uploader_packet.msg_id == PROTO_FAILED) {
		PX4_DEBUG("erase failed, myID: 0x%02x,esc_id: 0x%02x command: 0x%02x", _uploader_packet.d.feedback_packet.myID, esc_id,
			  _uploader_packet.d.feedback_packet.command);
		return -EIO;

	} else if (_uploader_packet.msg_id == PROTO_INVALID) {
		PX4_DEBUG("erase invalid, myID: 0x%02x, esc_id: 0x%02x command: 0x%02x", _uploader_packet.d.feedback_packet.myID,
			  esc_id,
			  _uploader_packet.d.feedback_packet.command);
		return -EIO;

	}

	return OK;
}


static int read_with_retry(int fd, void *buf, size_t n)
{
	int ret;
	uint8_t retries = 0;

	do {
		ret = read(fd, buf, n);

	} while (ret == -1 && retries++ < 100);

	if (retries != 0) {
		PX4_WARN("read of %u bytes needed %u retries", (unsigned)n,
			 (unsigned)retries);
	}

	return ret;
}

int
TAP_ESC_UPLOADER::program(uint8_t esc_id, size_t fw_size)
{
	uint8_t	*file_buf;
	ssize_t count;
	int ret;
	size_t sent = 0;

	file_buf = new uint8_t[PROG_MULTI_MAX];

	if (!file_buf) {
		PX4_DEBUG("Can't allocate program buffer");
		return -ENOMEM;
	}

	//ASSERT((fw_size & 3) == 0);
	//ASSERT((PROG_MULTI_MAX & 3) == 0);

	PX4_DEBUG("programming %u bytes...", (unsigned)fw_size);

	ret = lseek(_fw_fd, 0, SEEK_SET);

	while (sent < fw_size) {
		/* get more bytes to program */
		size_t n = fw_size - sent;

		if (n > PROG_MULTI_MAX) {
			n = PROG_MULTI_MAX;
		}

		count = read_with_retry(_fw_fd, file_buf, n);

		/* fill the rest with 0xff */
		if (n < PROG_MULTI_MAX) {
			/* if the file can read data */
			if (count > 0) {
				for (uint8_t i = count; i < PROG_MULTI_MAX; i++) {
					file_buf[i] = 0xff;
				}

				count = PROG_MULTI_MAX;
				n 	  = PROG_MULTI_MAX;
			}
		}

		if (count != (ssize_t)n) {
			PX4_DEBUG("firmware read of %u bytes at %u failed -> %d errno %d",
				  (unsigned)n,
				  (unsigned)sent,
				  (int)count,
				  (int)errno);
			ret = count;
			break;
		}

		sent += count;

		/* send program packet */
		EscUploaderMessage program_packet = {PACKET_HEAD, sizeof(EscbusBootProgPacket), PROTO_PROG_MULTI};
		program_packet.d.program_packet.myID = esc_id;

		for (uint8_t i = 0; i < PROG_MULTI_MAX; i++) {
			program_packet.d.program_packet.data[i] = file_buf[i];
		}

		send_packet(program_packet, esc_id);

		/* read and parse program feedback packet, blocking 100ms */
		ret = read_and_parse_data(100);

		if (ret != OK) {
			break;
		}

		/* check program is ok or fail */
		if (_uploader_packet.msg_id == PROTO_OK) {
			if (_uploader_packet.d.feedback_packet.myID != esc_id) {
				PX4_DEBUG("program id don't match myID: 0x%02x,esc_id: 0x%02x", _uploader_packet.d.feedback_packet.myID, esc_id);
				return -EIO;
			}

			if (_uploader_packet.d.feedback_packet.command != PROTO_PROG_MULTI) {
				PX4_DEBUG("program bad command, myID: 0x%02x,command: 0x%02x", _uploader_packet.d.feedback_packet.myID,
					  _uploader_packet.d.feedback_packet.command);
				return -EIO;
			}

		} else if (_uploader_packet.msg_id == PROTO_FAILED) {
			PX4_DEBUG("program failed: myID: 0x%02x, esc_id: 0x%02x command: 0x%02x", _uploader_packet.d.feedback_packet.myID,
				  esc_id,
				  _uploader_packet.d.feedback_packet.command);
			return -EIO;

		} else if (_uploader_packet.msg_id == PROTO_INVALID) {
			PX4_DEBUG("program invalid: myID: 0x%02x, esc_id: 0x%02x command: 0x%02x", _uploader_packet.d.feedback_packet.myID,
				  esc_id,
				  _uploader_packet.d.feedback_packet.command);
			return -EIO;

		}

	}

	delete [] file_buf;
	return ret;
}

int
TAP_ESC_UPLOADER::verify_crc(uint8_t esc_id, size_t fw_size_local)
{
	int ret;
	uint8_t	*file_buf;
	ssize_t count;
	uint32_t sum = 0;
	uint32_t bytes_read = 0;
	uint32_t crc = 0;
	uint32_t fw_size_remote;
	uint8_t fill_blank = 0xff;

	file_buf = new uint8_t[PROG_MULTI_MAX];

	PX4_DEBUG("verify...");
	lseek(_fw_fd, 0, SEEK_SET);

	ret = get_device_info(esc_id, PROTO_GET_DEVICE, PROTO_DEVICE_FW_SIZE, fw_size_remote);

	if (ret != OK) {
		PX4_DEBUG("could not read firmware size");
		return ret;
	}

	/* read through the firmware file again and calculate the checksum*/
	while (bytes_read < fw_size_local) {
		size_t n = fw_size_local - bytes_read;

		if (n > PROG_MULTI_MAX) {
			n = PROG_MULTI_MAX;
		}

		count = read_with_retry(_fw_fd, file_buf, n);

		/* fill the rest with 0xff */
		if (n < PROG_MULTI_MAX) {
			/* if the file can read data */
			if (count > 0) {
				for (uint8_t i = count; i < PROG_MULTI_MAX; i++) {
					file_buf[i] = fill_blank;
				}

				count = PROG_MULTI_MAX;
				n 	  = PROG_MULTI_MAX;
			}
		}

		if (count != (ssize_t)n) {
			PX4_DEBUG("firmware read of %u bytes at %u failed -> %d errno %d",
				  (unsigned)n,
				  (unsigned)bytes_read,
				  (int)count,
				  (int)errno);
			ret = count;
		}

		/* set the rest to 0xff */
		if (count == 0) {
			break;
		}

		/* stop if the file cannot be read */
		if (count < 0) {
			return count;
		}

		/* calculate crc32 sum */
		sum = crc32part(file_buf, PROG_MULTI_MAX, sum);

		bytes_read += count;
	}

	/* fill the rest with 0xff */
	while (bytes_read < fw_size_remote) {
		sum = crc32part(&fill_blank, sizeof(fill_blank), sum);
		bytes_read += sizeof(fill_blank);
	}

	/* send flash crc packet */
	EscUploaderMessage flash_crc_packet = {PACKET_HEAD, sizeof(EscbusFlashCRCPacket), PROTO_GET_CRC};
	flash_crc_packet.d.flash_crc_packet.myID = esc_id;
	send_packet(flash_crc_packet, esc_id);

	/* read and parse feedback CRC from tap esc,blocking 50ms */
	ret = read_and_parse_data();

	if (ret != OK) {
		return ret;
	}

	/* check flash crc feedback is ok or fail */
	if (_uploader_packet.msg_id == PROTO_OK) {
		if (_uploader_packet.d.feedback_crc_packet.myID != esc_id) {
			PX4_DEBUG("flash crc check id don't match myID: 0x%02x,esc_id: 0x%02x", _uploader_packet.d.feedback_packet.myID,
				  esc_id);
			return -EIO;
		}

		if (_uploader_packet.d.feedback_crc_packet.command != PROTO_GET_CRC) {
			PX4_DEBUG("flash crc check bad command, myID: 0x%02x command: 0x%02x", _uploader_packet.d.feedback_packet.myID,
				  _uploader_packet.d.feedback_packet.command);
			return -EIO;
		}

		crc = _uploader_packet.d.feedback_crc_packet.crc32;

	} else if (_uploader_packet.msg_id == PROTO_FAILED) {
		PX4_DEBUG("flash crc check failed: myID: 0x%02x, esc_id: 0x%02x, command: 0x%02x",
			  _uploader_packet.d.feedback_packet.myID,
			  esc_id, _uploader_packet.d.feedback_packet.command);
		return -EIO;

	} else if (_uploader_packet.msg_id == PROTO_INVALID) {
		PX4_DEBUG("flash crc check invalid: myID: 0x%02x, esc_id: 0x%02x, command: 0x%02x",
			  _uploader_packet.d.feedback_packet.myID,
			  esc_id, _uploader_packet.d.feedback_packet.command);
		return -EIO;
	}

	/* compare the CRC sum from the IO with the one calculated */
	if (sum != crc) {
		PX4_DEBUG("CRC mismatch: received: %u, expected: %u", static_cast<unsigned int>(crc), static_cast<unsigned int>(sum));
		return -EINVAL;
	}

	delete [] file_buf;
	return OK;
}

int
TAP_ESC_UPLOADER::reboot(uint8_t esc_id)
{
	int ret;

	/* send reboot packet */
	EscUploaderMessage reboot_packet = {PACKET_HEAD, sizeof(EscbusRebootPacket), PROTO_REBOOT};
	reboot_packet.d.reboot_packet.myID = esc_id;
	send_packet(reboot_packet, esc_id);

	/* read and parse reboot feedback packet, blocking 100ms */
	ret = read_and_parse_data();

	if (ret != OK) {
		return ret;
	}

	/* check reboot feedback is ok or fail */
	if (_uploader_packet.msg_id == PROTO_OK) {
		if (_uploader_packet.d.feedback_packet.myID != esc_id) {
			PX4_DEBUG("reboot id don't match myID: 0x%02x,esc_id: 0x%02x", _uploader_packet.d.feedback_packet.myID, esc_id);
			return -EIO;
		}

		if (_uploader_packet.d.feedback_packet.command != PROTO_REBOOT) {
			PX4_DEBUG("reboot receive bad command, myID: 0x%02x,command: 0x%02x", _uploader_packet.d.feedback_packet.myID,
				  _uploader_packet.d.feedback_packet.command);
			return -EIO;
		}

	} else if (_uploader_packet.msg_id == PROTO_FAILED) {
		PX4_DEBUG("reboot fail, myID: 0x%02x, esc_id: 0x%02x, command: 0x%02x", _uploader_packet.d.feedback_packet.myID, esc_id,
			  _uploader_packet.d.feedback_packet.command);
		return -EIO;

	} else if (_uploader_packet.msg_id == PROTO_INVALID) {
		PX4_DEBUG("reboot invalid,myID: 0x%02x, esc_id: 0x%02x, command: 0x%02x", _uploader_packet.d.feedback_packet.myID,
			  esc_id,
			  _uploader_packet.d.feedback_packet.command);
		return -EIO;

	}

	return OK;
}

int TAP_ESC_UPLOADER::read_esc_version_from_bin(int fw_fd, uint32_t &ver)
{
	int ret = -1;
	uint8_t version_buf[4];

	/* Jump to first byte of encoded firmware version */
	ret = lseek(fw_fd, ESC_VERSION_OFFSET_ADDR, SEEK_SET);

	if (ret < 0) {
		return ret;
	}

	ret = read(fw_fd, version_buf, ESC_FW_VER_BYTES);

	if (ret < 0) {
		return ret;
	}

	/* Decode ESC firmware version (little-Endian)*/
	ver = (version_buf[3] << 24) + (version_buf[2] << 16) + (version_buf[1] << 8) + version_buf[0];

	/* check the decoded value is invalid */
	if (!PX4_ISFINITE((double)ver) || ver == 0) {
		return -EBADRQC;
	}

	return OK;
}

int TAP_ESC_UPLOADER::update_fw(const char *filenames[])
{
	int ret = -1;

	ret = initialise_firmware_file(filenames, _fw_fd);

	if (ret < 0) {
		PX4_DEBUG("initialise firmware file failed");
		return ret;
	}

	uint32_t fw_binary_version = 0;
	ret = read_esc_version_from_bin(_fw_fd, fw_binary_version);

	/* Catch errors if firmware version could not be read from binay */
	if (ret == -EBADRQC) {
		PX4_WARN("Firmware version from binary is invalid - not updating!");
		return ret;

	} else if (ret != OK) {
		PX4_WARN("Could not read firmware version from binary - not updating!");
		return ret;
	}

	ret = tap_esc_common::initialise_uart(_device, _esc_fd);

	if (ret < 0) {
		PX4_DEBUG("Failed to initialize UART device %s (error code %i)", _device, ret);
		return ret;
	}

	/* check firmware version of each ESC */
	for (uint8_t esc_id = 0; esc_id < _esc_counter; esc_id++) {

		uint32_t fw_esc_version = 0;

		/* get device firmware revision */
		ret = get_device_info(esc_id, ESCBUS_MSG_ID_REQUEST_INFO, REQUEST_INFO_DEVICE, fw_esc_version);

		/* check if ESC firmware is too old to understand command, in which case the
		   response runs into a timeout */
		if (ret == -ETIMEDOUT) {
			PX4_INFO("ESC did not respond to version request command. Probably out of date.");
			/* Check CRC of ESC firmware and compare to CRC of binary file. On
			   mismatch the firmware is re-upload to the ESC. This will also
				 reboot the ESCs after it is done */
			return checkcrc(filenames);
		}

		/* check if firmware version of binary is newer than firmware currently on ESCs */
		else if (ret == OK && (fw_binary_version > fw_esc_version)) {
			PX4_INFO("esc_id %d has fw version mismatch (%4.4f instead of %4.4f)", esc_id, (double)fw_esc_version * 0.01,
				 (double)fw_binary_version * 0.01);
			PX4_INFO("At least one ESC has a version mismatch. Running checkcrc for all ESCs!");
			/* Check CRC of ESC firmware and compare to CRC of binary file. On
			   mismatch the firmware is re-upload to the ESC. This will also
				 reboot the ESCs after it is done */
			return checkcrc(filenames);
		}
	}

	/* ESCs did not require update */
	return CODE_ESCS_ALREADY_UPTODATE;
}
