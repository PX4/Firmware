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
 * @file tap_esc_uploader.h
 * Firmware uploader definitions for TAP ESC.
 */

#ifndef _TAP_ESC_UPLOADER_H
#define _TAP_ESC_UPLOADER_H

#include <drivers/tap_esc/drv_tap_esc.h>
#include <stdint.h>
#include <stdbool.h>
#include <systemlib/mavlink_log.h>

#define TAP_ESC_FW_SEARCH_PATHS {"/etc/extras/tap_esc.bin", "/fs/microsd/tap_esc.bin", nullptr }
#define PROTO_SUPPORT_BL_REV 5	/**< supported bootloader protocol revision */
#define SYNC_RETRY_TIMES     5	/**< (uint8) esc sync failed allow retry times*/
#define UPLOADER_RETRY_TIMES 2  /**< esc uploader failed allow retry times*/
#define ESCBUS_DATA_CRC_LEN 248 /**< length of data field is 255 and plus one byte for CRC*/
#define ESC_WAIT_BEFORE_READ 1	/**< ms, wait before reading to save read() calls*/
#define ESC_VERSION_OFFSET_ADDR 0x200 /**< ESCs firmware version offset address in tap_esc.bin file */
#define ESC_FW_VER_BYTES 4		/**< Number of bytes used to encode ESC firmware version */
#define CODE_ESCS_ALREADY_UPTODATE 1

class TAP_ESC_UPLOADER
{
public:
	TAP_ESC_UPLOADER(const char *device, uint8_t esc_counter);
	virtual ~TAP_ESC_UPLOADER();

	/*
	 * Upload ESC firmware from a binary.
	 * @param filenames
	 * @return OK on success, -errno otherwise
	 */
	int upload(const char *filenames[]);

	/*
	 * Compare ESC firmware CRC with that of a binary file.
	 * In case of mismatch, the binary file is uploaded to the ESCs.
	 * @param filenames
	 * @return OK on success, -errno otherwise
	 */
	int checkcrc(const char *filenames[]);

	/*
	 * Query ESCs for version information (firmware, hardware, bootloader)
	 * and store the response in the corresponding parameters.
	 */
	int log_versions();

	/*
	 * read version of ESC firmware from tap_esc.bin file
	 * @param filenames
	 * @param ver: get the ESC firmware version in format xx.yy * 100
	 * @return OK on success, or -errno otherwise.
	 */
	static int read_esc_version_from_bin(int fw_fd, uint32_t &ver);

	/*
	 * Update ESC firmware with newer version from binary, if necessary.
	 * @param filenames
	 * @return 'CODE_ESCS_ALREADY_UPTODATE' if no update is required, 'PX4_OK' if updated, or '-errno' otherwise.
	 */
	int update_fw(const char *filenames[]);

	/*
	 * Open firmware binary file from a prioritized list of filenames
	 * @param filenames array of paths to try and open, must be NULL-terminated
	 * @param fw_fd file descriptor for firmware binary
	 * @return file size on success or -errno on failure
	 */
	static int32_t 	initialise_firmware_file(const char *filenames[], int &fw_fd);

private:

#pragma pack(push,1)

	typedef enum {

		PROTO_NOP			= 0x00,

		/**
		 * receive tap esc feedback information
		 */
		PROTO_OK			= 0x10,		/**< INSYNC/OK - 'ok' response */
		PROTO_FAILED			= 0x11,		/**< INSYNC/FAILED  - 'fail' response */
		PROTO_INVALID			= 0x13,		/**< INSYNC/INVALID - 'invalid' response for bad commands */
		PROTO_BAD_SILICON_REV  		= 0x14,		/**< On the F4 series there is an issue with < Rev 3 silicon */

		/**
		 * send to tap esc command information
		 */
		PROTO_GET_SYNC			= 0x21,		/**< NOP for re-establishing sync */
		PROTO_GET_DEVICE_INFO		= 0x22,		/**< get device ID bytes */
		PROTO_CHIP_ERASE		= 0x23,		/**< erase program area and reset program address */
		PROTO_PROG_MULTI		= 0x27,		/**< write bytes at program address and increment */
		PROTO_GET_CRC			= 0x29,		/**< compute & return a CRC */
		PROTO_GET_OTP			= 0x2a,		/**< read a byte from OTP at the given address */
		PROTO_GET_SN			= 0x2b,		/**< read a word from UDID area ( Serial)  at the given address */
		PROTO_GET_CHIP			= 0x2c,		/**< read chip version (MCU IDCODE) */
		PROTO_SET_DELAY			= 0x2d,		/**< set minimum boot delay */
		PROTO_GET_CHIP_DES		= 0x2e,		/**< read chip version In ASCII */
		PROTO_REBOOT			= 0x30,		/**< boot the application */
		PROTO_MSG_ID_MAX_NUM    	= 0x40,		/**< support protocol maximum message ID command */

		/* argument values for PROTO_GET_DEVICE */
		PROTO_DEVICE_BL_REV	= 1,		/**< bootloader revision */
		PROTO_DEVICE_BOARD_ID	= 2,		/**< board ID Use for distinguishing ESC chip, each chip corresponds to a fixed value */
		PROTO_DEVICE_BOARD_REV	= 3,		/**< board revision */
		PROTO_DEVICE_FW_SIZE	= 4,		/**< size of flashable area */
		PROTO_DEVICE_VEC_AREA	= 5,		/**< contents of reserved vectors 7-10 */
		PROTO_DEVICE_FW_REV	= 6,		/**< firmware revision */

		PROG_MULTI_MAX		= 128,		/**< protocol max is 255, must be multiple of 4 */

	} ESCBUS_UPLOADER_MESSAGE_ID;

	/**< tap esc feedback packet,The command is the same as the command received */
	typedef struct {
		uint8_t myID;
		ESCBUS_UPLOADER_MESSAGE_ID command;
	} EscbusBootFeedbackPacket;

	/**< the real packet definition for PROTO_GET_SYNC */
	typedef struct {
		uint8_t myID;
	} EscbusBootSyncPacket;

	/**< the real packet definition for PROTO_CHIP_ERASE */
	typedef struct {
		uint8_t myID;
	} EscbusBootErasePacket;

	/**< the real packet definition for PROTO_PROG_MULTI */
	typedef struct {
		uint8_t myID;
		uint8_t data[PROG_MULTI_MAX];
	} EscbusBootProgPacket;

	/**< the real packet definition for PROTO_GET_CRC */
	typedef struct {
		uint8_t myID;
	} EscbusFlashCRCPacket;

	/**< the real packet feedback for CRC sum */
	typedef struct {
		uint8_t myID;
		uint32_t crc32;		/**< The offset address of verification, the length is fixed 128 bytes */
		ESCBUS_UPLOADER_MESSAGE_ID command;/**< The command is the same as the command received */
	} EscbusBootFeedbackCRCPacket;

	/**< the real packet definition for PROTO_REBOOT */
	typedef struct {
		uint8_t myID;
	} EscbusRebootPacket;

	/**< the real packet definition for PROTO_DEVICE */
	typedef struct {
		uint8_t myID;
		uint8_t deviceInfo;
	} EscbusGetDevicePacket;

	/**< the real packet definition for deviceInfo is PROTO_DEVICE_BL_REV */
	typedef struct {
		uint8_t myID;
		uint32_t version;
	} EscbusBootloaderRevisionPacket;

	/**< the real packet definition for deviceInfo is PROTO_DEVICE_BOARD_ID	*/
	typedef struct {
		uint8_t myID;
		uint32_t targetSystemId;/**< Use for distinguishing ESC chip, each chip corresponds to a fixed value(e.g. H520 targetSystemId is 0x02) */
	} EscbusHardwareIDPacket;

	/**< the real packet definition for deviceInfo is PROTO_DEVICE_BOARD_REV */
	typedef struct {
		uint8_t myID;
		uint32_t boardRev;
	} EscbusHardwareRevisionPacket;

	/**< the real packet definition for deviceInfo is PROTO_DEVICE_VERSION */
	typedef struct {
		uint8_t  myID;
		uint32_t FwRev;
		uint32_t HwRev;
		uint32_t blRev;
	} EscbusDeviceInfoPacket;

	/**< the real packet definition for deviceInfo is PROTO_DEVICE_FW_SIZE */
	typedef struct {
		uint8_t myID;
		uint32_t FwSize;
	} EscbusFirmwareSizePacket;

	/**< the real packet definition for deviceInfo is PROTO_DEVICE_FW_REV */
	typedef struct {
		uint8_t myID;
		uint32_t FwRev;
	} EscbusFirmwareRevisionPacket;

	/**< the real packet definition for PROTO_INVALID,When the package to ESC is parsed fail, ESC sends the command */
	typedef struct {
		uint8_t myID;
	} EscbusProtocolInvalidPacket;

	/*
	 * ESCBUS bootloader message type
	 */
	typedef struct {
		uint8_t head;		/**< start sign for a message package */
		uint8_t len;		/**< length of data field */
		uint8_t msg_id;		/**< ID for this message */
		union {
			EscbusBootFeedbackPacket	feedback_packet;
			EscbusBootSyncPacket 		sync_packet;
			EscbusBootErasePacket		erase_packet;
			EscbusBootProgPacket		program_packet;
			EscbusFlashCRCPacket 		flash_crc_packet;
			EscbusBootFeedbackCRCPacket	feedback_crc_packet;
			EscbusRebootPacket		reboot_packet;
			EscbusGetDevicePacket		device_info_packet;
			EscbusBootloaderRevisionPacket	bootloader_revis_packet;
			EscbusHardwareIDPacket		hardware_id_packet;
			EscbusHardwareRevisionPacket	hardware_revis_packet;
			EscbusFirmwareSizePacket	firmware_size_packet;
			EscbusFirmwareRevisionPacket	firmware_revis_packet;
			EscbusProtocolInvalidPacket	protocal_invalid_packet;
			EscbusDeviceInfoPacket          esc_version_packet;
			uint8_t data[ESCBUS_DATA_CRC_LEN];	/**< length of data field is 255 and plus one byte for CRC */
		} d;
		uint8_t crc_data;

	} EscUploaderMessage;

#pragma pack(pop)

	int			_esc_fd;
	int			_fw_fd;
	const char *_device;
	uint8_t 	_esc_counter;

	/* _device_mux_map[sel]:Asign the id's to the ESC to match the mux */
	static const uint8_t 	_device_mux_map[TAP_ESC_MAX_MOTOR_NUM];
	EscUploaderMessage  	_uploader_packet;
	orb_advert_t    	_mavlink_log_pub;
	int			upload_id(uint8_t esc_id, int32_t fw_size);
	int 			recv_byte_with_timeout(uint8_t *c, unsigned timeout);
	int 			read_and_parse_data(unsigned timeout = 50);
	int 			parse_tap_esc_feedback(uint8_t decode_data, EscUploaderMessage *packetdata);
	uint8_t 		crc8_esc(uint8_t *p, uint8_t len);
	uint8_t 		crc_packet(EscUploaderMessage &p);
	int 			send_packet(EscUploaderMessage &packet, int responder);
	int			sync(uint8_t esc_id);
	int			get_device_info(uint8_t esc_id, uint8_t msg_id, uint8_t msg_arg, uint32_t &val);
	int			get_esc_versions(uint8_t esc_id, uint32_t &fw_ver, uint32_t &hw_ver, uint32_t &bl_ver);
	int			erase(uint8_t esc_id);
	int			program(uint8_t esc_id, size_t fw_size);
	int			verify_crc(uint8_t esc_id, size_t fw_size_local);
	int			reboot(uint8_t esc_id);
};
#endif
