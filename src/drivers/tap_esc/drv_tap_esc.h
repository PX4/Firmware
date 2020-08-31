/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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

#pragma once

#include <board_config.h>

#include <stdint.h>

/* At the moment the only known use is with a current sensor */
#define ESC_HAVE_CURRENT_SENSOR

#define TAP_ESC_MAX_PACKET_LEN 20
#define TAP_ESC_MAX_MOTOR_NUM 8

#define PACKET_HEAD 0xfe

/* ESC_POS maps the values stored in the channelMapTable to reorder the ESC's
 * id so that that match the mux setting, so that the ressonder's data
 * will be read.
 * The index on channelMapTable[p] p is the physical ESC
 * The value it is set to is the logical value from ESC_POS[p]
 *  Phy Log
 *  0   0
 *  1   1
 *  2   4
 *  3   3
 *  4   2
 *  5   5
 *   ....
 *
 */

// Circular from back right in CCW direction
#define ESC_POS {0, 1, 4, 3, 2, 5, 7, 8}
// 0 is CW, 1 is CCW
#define ESC_DIR {0, 1, 0, 1, 0, 1, 0, 1}

#define RPMMAX 1900
#define RPMMIN 1200
#define RPMSTOPPED (RPMMIN - 10)


#define MAX_BOOT_TIME_MS		 (550) // Minimum time to wait after Power on before sending commands

#pragma pack(push,1)

/****** Run ***********/

#define RUN_CHANNEL_VALUE_MASK   (uint16_t)0x07ff
#define RUN_RED_LED_ON_MASK      (uint16_t)0x0800
#define RUN_GREEN_LED_ON_MASK    (uint16_t)0x1000
#define RUN_BLUE_LED_ON_MASK     (uint16_t)0x2000
#define RUN_LED_ON_MASK          (uint16_t)0x3800
#define RUN_FEEDBACK_ENABLE_MASK (uint16_t)0x4000
#define RUN_REVERSE_MASK         (uint16_t)0x8000

typedef struct {

	uint16_t rpm_flags[TAP_ESC_MAX_MOTOR_NUM];
} RunReq;

typedef struct {
	uint8_t channelID;
	uint8_t ESCStatus;
	int16_t speed; // -32767 - 32768
#if defined(ESC_HAVE_VOLTAGE_SENSOR)
	uint16_t voltage; // 0.00 - 100.00 V
#endif
#if defined(ESC_HAVE_CURRENT_SENSOR)
	uint16_t current; // 0.0 - 200.0 A
#endif
#if defined(ESC_HAVE_TEMPERATURE_SENSOR)
	uint8_t temperature; // 0 - 256 degree celsius
#endif
} RunInfoRepsonse;
/****** Run ***********/

/****** ConFigInfoBasic ***********/
typedef  struct {
	uint8_t  maxChannelInUse;
	uint8_t  channelMapTable[TAP_ESC_MAX_MOTOR_NUM];
	uint8_t  monitorMsgType;
	uint8_t  controlMode;
	uint16_t minChannelValue;
	uint16_t maxChannelValue;
} ConfigInfoBasicRequest;

typedef  struct {
	uint8_t  channelID;
	ConfigInfoBasicRequest resp;
} ConfigInfoBasicResponse;

#define ESC_MASK_MAP_CHANNEL           0x0f
#define ESC_MASK_MAP_RUNNING_DIRECTION 0xf0
/****** ConFigInfoBasicResponse ***********/

/****** InfoRequest  ***********/
typedef enum {
	REQUEST_INFO_BASIC = 0,
	REQUEST_INFO_FUll,
	REQUEST_INFO_RUN,
	REQUEST_INFO_STUDY,
	REQUEST_INFO_COMM,
	REQUEST_INFO_DEVICE,
} InfoTypes;

typedef  struct {
	uint8_t  channelID;
	uint8_t  requestInfoType;
} InfoRequest;

/****** InfoRequest ***********/

typedef  struct {
	uint8_t head;
	uint8_t len;
	uint8_t msg_id;
	union {
		InfoRequest 		reqInfo;
		ConfigInfoBasicRequest 	reqConfigInfoBasic;
		RunReq			reqRun;
		ConfigInfoBasicResponse rspConfigInfoBasic;
		RunInfoRepsonse		rspRunInfo;
		uint8_t bytes[100];
	} d;
	uint8_t crc_data;

} EscPacket;

#define UART_BUFFER_SIZE 128
typedef  struct {
	uint8_t head;
	uint8_t tail;
	uint8_t dat_cnt;
	uint8_t esc_feedback_buf[UART_BUFFER_SIZE];
} ESC_UART_BUF;

#pragma pack(pop)
/******************************************************************************************
 * ESCBUS_MSG_ID_RUN_INFO packet
 *
 * Monitor message of ESCs while motor is running
 *
 * channelID: assigned channel number
 *
 * ESCStatus: status of ESC
 * 	Num		Health status
 * 	0		HEALTHY
 * 	1		WARNING_LOW_VOLTAGE
 * 	2		WARNING_OVER_CURRENT
 * 	3		WARNING_OVER_HEAT
 *	4		ERROR_MOTOR_LOW_SPEED_LOSE_STEP
 *  5		ERROR_MOTOR_STALL
 *  6		ERROR_HARDWARE
 *  7		ERROR_LOSE_PROPELLER
 *  8		ERROR_OVER_CURRENT
 *
 * speed: -32767 - 32767 rpm
 *
 * temperature: 0 - 256 celsius degree (if available)
 * voltage: 0.00 - 100.00 V (if available)
 * current: 0.0 - 200.0 A (if available)
 */

typedef enum {
	ESC_STATUS_HEALTHY,
	ESC_STATUS_WARNING_LOW_VOLTAGE,
	ESC_STATUS_WARNING_OVER_HEAT,
	ESC_STATUS_ERROR_MOTOR_LOW_SPEED_LOSE_STEP,
	ESC_STATUS_ERROR_MOTOR_STALL,
	ESC_STATUS_ERROR_HARDWARE,
	ESC_STATUS_ERROR_LOSE_PROPELLER,
	ESC_STATUS_ERROR_OVER_CURRENT,
	ESC_STATUS_ERROR_MOTOR_HIGH_SPEED_LOSE_STEP,
	ESC_STATUS_ERROR_LOSE_CMD,
} ESCBUS_ENUM_ESC_STATUS;


typedef enum {
// messages or command to ESC
	ESCBUS_MSG_ID_CONFIG_BASIC = 0,
	ESCBUS_MSG_ID_CONFIG_FULL,
	ESCBUS_MSG_ID_RUN,
	ESCBUS_MSG_ID_TUNE,
	ESCBUS_MSG_ID_DO_CMD,
// messages from ESC
	ESCBUS_MSG_ID_REQUEST_INFO,
	ESCBUS_MSG_ID_CONFIG_INFO_BASIC,	// simple configuration info for request from flight controller
	ESCBUS_MSG_ID_CONFIG_INFO_FULL,// full configuration info for request from host such as computer
	ESCBUS_MSG_ID_RUN_INFO,// feedback message in RUN mode
	ESCBUS_MSG_ID_STUDY_INFO,	// studied parameters in STUDY mode
	ESCBUS_MSG_ID_COMM_INFO,	// communication method info
	ESCBUS_MSG_ID_DEVICE_INFO,// ESC device info
	ESCBUS_MSG_ID_ASSIGNED_ID,	// never touch ESCBUS_MSG_ID_MAX_NUM
	//boot loader used
	PROTO_OK = 0x10, // INSYNC/OK - 'ok' response
	PROTO_FAILED = 0x11, // INSYNC/FAILED - 'fail' response

	ESCBUS_MSG_ID_BOOT_SYNC = 0x21, // boot loader used
	PROTO_GET_DEVICE = 0x22, // get device ID bytes
	PROTO_CHIP_ERASE = 0x23, // erase program area and reset program address
	PROTO_PROG_MULTI = 0x27, // write bytes at program address and increment
	PROTO_GET_CRC = 0x29, // compute & return a CRC
	PROTO_BOOT = 0x30, // boot the application
	PROTO_GET_SOFTWARE_VERSION = 0x40,
	ESCBUS_MSG_ID_MAX_NUM,
}
ESCBUS_ENUM_MESSAGE_ID;

typedef enum {
	HEAD,
	LEN,
	ID,
	DATA,
	CRC,

} PARSR_ESC_STATE;
