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

#ifndef PX4_RDDRONE_H
#define PX4_RDDRONE_H

#include <termios.h>
#include <poll.h>
#include <sys/select.h>
#include <sys/time.h>
#include <px4_platform_common/module.h>
#include <perf/perf_counter.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/uwb_report.h>
#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/uwb_grid.h>
#include <uORB/topics/uwb_distance.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <matrix/math.hpp>
#include <matrix/Matrix.hpp>


#define MAX_ANCHORS 9
// These commands all require a 16-byte UUID. However, with the "pure ranging" and "stop ranging" commands, this UUID
// is unused. In the following constants, the UUID is automatically initialized to all 0s.
const uint8_t CMD_STOP_RANGING[20] = {0x8e, 0x00, 0x11, 0x00};
const uint8_t CMD_PURE_RANGING[20] = {0x8e, 0x00, 0x11, 0x02};

const uint8_t CMD_DISTANCE_RESULT[20] = {0x8e, 0x0A, 0x11, 0x02};
const uint8_t CMD_GRID_SURVEY[4] = {0x8e, 0x01, 0x01, 0x00};

// Currently, the "start ranging" command is unused. If in the future it is used, there will need to be a mechanism
// for populating the UUID field.
// TODO: Determine how to fill the UUID field in this command.
// Suggestion from Gerald: Make a file on the SD card with the grid UUIDs.
// Would probably make use of PX4_STORAGEDIR "/rddrone_config.txt"
// const uint8_t CMD_START_RANGING[20] = {0x8e, 0x00, 0x11, 0x01};


typedef struct {
	float lat;
	float lon;
	float alt;
	float yaw;
}gps_pos_t;

typedef struct {
	int32_t x;
	int32_t y;
	int32_t z;
}position_t;

typedef union {
	uint8_t all_flags; /**/
	struct {
		uint8_t spare7 :1, /* Unused */
		grid_moving :1, /* Explanation of bar */
		yaw_data :1, /* yaw y/n */
		gps_data :1, /* gps data y/n*/
		spare3 :1, /* Grid ok */
		spare2 :1, /* Grid info send*/
		grid_found :1, /* Grid found*/
		uwb_flag_status :1; /* Correctly send Grid info */
	};
} uwb_grid_flags;

// This is the message sent back from the UWB module, as defined in the documentation.
typedef struct {
	uint8_t cmd;      	// Should be 0x8E for position result message
	uint8_t sub_cmd;  	// Should be 0x0A for position result message
	uint8_t data_len; 	// Should be 0x30 for position result message
	uint8_t status;   	// 0x00 is no error
	uint16_t counter;	// Number of Ranges since last Start of Ranging
	uint8_t time_offset;	// time measured between ranging
	float yaw_offset; 	// Yaw offset in degrees
	uint16_t anchor_distance[MAX_ANCHORS]; //Raw anchor_distance distances in CM
	uint8_t stop; 		// Should be 0x1B
} __attribute__((packed)) position_msg_t;

typedef struct {
	uint8_t cmd;      	// Should be 0x8E for grid result message
	uint8_t sub_cmd;  	// Should be 0x0A for grid result message
	uint8_t data_len; 	// Should be 0x157 for grid result message
	uint8_t status;   	// 0x00 is no error
	uint32_t initator_time;  	//timestamp of init
	uwb_grid_flags	flag; 	//grid info flags ()
	uint8_t	grid_uuid[16];// Same UUID as for anchor 0
	uint8_t	anchor_nr;
	uint8_t preamble_id; 	//TODO Do this
	uint8_t	channel_id; 	//TODO Do this
	gps_pos_t gps;  	// GPS Position of grid
	position_t target_pos; //target
	position_t anchor_pos[MAX_ANCHORS];
	uint8_t stop; 		// Should be 27

} __attribute__((packed)) grid_msg_t;


typedef struct {
	uint8_t cmd;      	// Should be 0x8E for distance result message
	uint8_t sub_cmd;  	// Should be 0x0A for distance result message
	uint8_t data_len; 	// Should be 0x30 for distance result message
	uint8_t status;   	// 0x00 is no error
	uint16_t counter;	// Number of Ranges since last Start of Ranging
	uint8_t time_offset;	// time measured between ranging
	float yaw_offset; 	// Yaw offset in degrees
	uint16_t anchor_distance[MAX_ANCHORS]; //Raw anchor_distance distances in CM 2*9
	uint8_t stop; 		// Should be 0x1B
} __attribute__((packed)) distance_msg_t;



class RDDrone : public ModuleBase<RDDrone>
{
public:
	RDDrone(const char *device_name, speed_t baudrate);

	~RDDrone();

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

	static RDDrone *instantiate(int argc, char *argv[]);

	void run() override;

private:

	int _uart;
	fd_set _uart_set;
	struct timeval _uart_timeout {};

	perf_counter_t _read_count_perf;
	perf_counter_t _read_err_perf;

	uORB::Publication<uwb_report_s> _uwb_pub{ORB_ID(uwb_report)};
	uwb_report_s _uwb_report{};

	uORB::Publication<uwb_grid_s> _uwb_grid_pub{ORB_ID(uwb_grid)};
	uwb_grid_s _uwb_grid{};

	uORB::Publication<uwb_distance_s> _uwb_distance_pub{ORB_ID(uwb_distance)};
	uwb_distance_s _uwb_distance{};

	uORB::Publication<landing_target_pose_s> _landing_target_pub{ORB_ID(landing_target_pose)};
	landing_target_pose_s _landing_target{};

	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	vehicle_attitude_s _vehicle_attitude{};

	grid_msg_t _grid_survey_msg{};
	position_msg_t _distance_result_msg{};

	matrix::Dcmf _rddrone_to_nwu;
	matrix::Dcmf _nwu_to_ned{matrix::Eulerf(M_PI_F, 0.0f, 0.0f)};
	matrix::Vector3f _current_position_rddrone;
	matrix::Vector3f _current_position_ned;
};


#endif //PX4_RDDRONE_H
