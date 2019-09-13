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

#ifndef PX4_UWB_H
#define PX4_UWB_H

#include <termios.h>
#include <poll.h>
#include <sys/select.h>
#include <sys/time.h>
#include <px4_module.h>
#include <perf/perf_counter.h>

const uint8_t GRID_UUID[16] = {0x68, 0x91, 0xb6, 0x1c, 0x43, 0xd5, 0xb8, 0x33, 0xb4, 0xec, 0x46, 0x80, 0x7a, 0x31, 0x69, 0xe3};
const uint8_t BLANK_UUID[16] = {};
const uint8_t CMD_START_RANGING[4] = {0x8e, 0x00, 0x11, 0x01};
const uint8_t CMD_STOP_RANGING[4] = {0x8e, 0x00, 0x11, 0x00};
const uint8_t CMD_PURE_RANGING[4] = {0x8e, 0x00, 0x11, 0x02};

typedef struct {
	uint8_t cmd;      	// Should be 0x8E for position result message
	uint8_t sub_cmd;  	// Should be 0x01 for position result message
	uint8_t data_len; 	// Should be 0x30 for position result message
	uint8_t status;   	// 0x00 is no error
	float pos_x;	  	// X location relative to landing point
	float pos_y;		// Y location relative to landing point
	float pos_z;		// Z location relative to landing point
	float yaw_offset; 	// Yaw offset in degrees
	uint16_t counter;
	uint8_t time_offset;
	uint8_t grid_uuid[16];
	float landing_point_lat;
	float landing_point_lon;
	float landing_point_alt;
} __attribute__((packed)) position_msg_t;

class UWB : public ModuleBase<UWB>
{
public:
	UWB(const char *device_name, int baudrate);

	~UWB();

	/**
	 * @see ModuleBase::custom_command
	 */
	static int custom_command(int argc, char *argv[]);

	/**
	 * @see ModuleBase::print_usage
	 */
	static int print_usage(const char *reason = nullptr);

	/**
	 * @see ModuleBase::task_spawn
	 */
	static int task_spawn(int argc, char *argv[]);

	static UWB *instantiate(int argc, char *argv[]);

	void run() override;

private:

	bool _error = false;

	int _uart;
	fd_set _uart_set;
	struct timeval _uart_timeout {
		.tv_sec = 1,
		.tv_usec = 0
	};

	perf_counter_t _time_perf;
	perf_counter_t _read_count_perf;
	perf_counter_t _read_err_perf;
};


#endif //PX4_UWB_H
