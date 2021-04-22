/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/module_params.h>
#include <uORB/topics/parameter_update.h>
#include <px4_platform_common/module.h>
#include <perf/perf_counter.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/uwb_grid.h>
#include <uORB/topics/uwb_distance.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <matrix/math.hpp>
#include <matrix/Matrix.hpp>

#define MAX_ANCHORS 9
#define GRID_UUID 16
#define OFFSET 0 //to support multiple grids
// These commands all require a 16-byte UUID. However, with the "pure ranging" and "stop ranging" commands, this UUID
// is unused. In the following constants, the UUID is automatically initialized to all 0s.

#define UWB_CMD  0x8e
#define UWB_CMD_GRID  0x01
#define UWB_CMD_DISTANCE 0x0A
#define UWB_CMD_START  0x01
#define UWB_CMD_STOP  0x00
#define UWB_CMD_DEBUG  0x02
#define STOP_B 0x1b

#define UWB_CMD_LEN  0x05
#define UWB_CMD_DISTANCE_LEN 0x21
const uint8_t CMD_STOP_RANGING[UWB_CMD_LEN ] = {UWB_CMD, UWB_CMD_DISTANCE, 0x01, UWB_CMD_STOP,STOP_B};
const uint8_t CMD_START_RANGING[UWB_CMD_LEN ] = {UWB_CMD, UWB_CMD_DISTANCE, 0x01, UWB_CMD_START,STOP_B};
const uint8_t CMD_DISTANCE_RESULT[UWB_CMD_DISTANCE_LEN] = {UWB_CMD, UWB_CMD_DISTANCE, 0x11, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, STOP_B}; //example UUID
const uint8_t CMD_GRID_SURVEY[UWB_CMD_LEN] = {UWB_CMD, UWB_CMD_GRID, 0x01, 0x00, STOP_B};

// Currently, the "start ranging" command is unused. If in the future it is used, there will need to be a mechanism
// for populating the UUID field.
// TODO: Determine how to fill the UUID field in this command.
// Suggestion from Gerald: Make a file on the SD card with the grid UUIDs.
// Would probably make use of PX4_STORAGEDIR "/uwb_r4_config.txt"
// const uint8_t CMD_START_RANGING[20] = {0x8e, 0x00, 0x11, 0x01};
#define CONF_FILE "/fs/microsd/etc/uwb_r4_config.txt"

typedef struct {  //needs higher accuracy?
	float lat, lon, alt,yaw;//offset to true North
} gps_pos_t;

typedef struct {
	int16_t x, y, z; //axis in cm
} position_t; // Position of a device or target in 3D space

typedef union {
	uint8_t all_flags; /**/
	struct {
		uint8_t check_uuid :1, /* weather devices check UUID of UWB request */
        	send_payload :1, /* Send Payload with Timestamps */
        	groupdelay :1, /* do groupdelay on startup and send  it with payload */
        	grid_moving :1, /* grid is moving object */
        	/* payload flags */
        	gps_calc :1, /* calculate GPS ; if set to False Responder will send static GPS */
        	gps_data :1, /* Send GPS */
        	gps_vel :1, /* Send GPS_Velocity*/
        	spare7 :1; /* Correctly send Grid info */
	};
} uwb_grid_flags;

enum UWB_POS_ERROR_CODES{
	UWB_OK,
	UWB_ANC_BELOW_THREE,
	UWB_LIN_DEP_FOR_THREE,
	UWB_ANC_ON_ONE_LEVEL,
	UWB_LIN_DEP_FOR_FOUR,
	UWB_RANK_ZERO
};

typedef struct {
	uint8_t cmd;      	// Should be 0x8E for grid result message
	uint8_t sub_cmd;  	// Should be 0x0A for grid result message
	uint8_t data_len; 	// Should be 0x157 for grid result message
	uint8_t status;   	// 0x00 is no error
	uint32_t initator_time;  	//timestamp of init
	uwb_grid_flags	flag; 	//grid info flags ()
	uint8_t	grid_uuid[16];// Same UUID as for anchor 0
	uint8_t	num_anchors;	//number of anchors
	uint8_t preamble_id; 	//UWB Preamble
	uint8_t	channel_id; 	//UWB Channel
	gps_pos_t target_gps; //GPS position of Landing Point
	position_t target_pos; //target position
	position_t anchor_pos[MAX_ANCHORS]; // Position of each anchor
	uint8_t stop; 		// Should be 27

} __attribute__((packed)) grid_msg_t;


typedef struct {
	uint8_t cmd;      	// Should be 0x8E for distance result message
	uint8_t sub_cmd;  	// Should be 0x0A for distance result message
	uint8_t data_len; 	// Should be 0x30 for distance result message
	uint8_t status;   	// 0x00 is no error
	uint32_t counter;	// Number of Ranges since last Start of Ranging
	uint16_t flags;
	uint16_t anchor_distance[MAX_ANCHORS]; //Raw anchor_distance distances in CM 2*9
	uint8_t time_offset;	// time measured between ranging
	gps_pos_t gps_data;  	// GPS Position of grid
	uint8_t stop; 		// Should be 0x1B
} __attribute__((packed)) distance_msg_t;



class UWB_R4 : public ModuleBase<UWB_R4>, public ModuleParams
{
public:
	UWB_R4(const char *device_name, speed_t baudrate, bool uwb_pos_debug);

	~UWB_R4();

	/**
	 * @see ModuleBase::custom_command
	 */
	static int custom_command(int argc, char *argv[]);

	/**
	 * @see ModuleBase::print_usage
	 */
	static int print_usage(const char *reason = nullptr);

	/**
	 * @see ModuleBase::Multilateration
	 */
	UWB_POS_ERROR_CODES localization();

	/**
	 * @see ModuleBase::Grid Survey Result
	 */
	int grid_survey();

	/**
	 * @see ModuleBase::Distance Result
	 */
	int distance();


	/**
	 * @see ModuleBase::task_spawn
	 */
	static int task_spawn(int argc, char *argv[]);

	static UWB_R4 *instantiate(int argc, char *argv[]);

	void run() override;

protected:
	/*
	 * Update Params
	 * */

	void _update_params();

private:

	void parameters_update(int parameter_update_sub, bool force = false);

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::UWB_UUID_ON_SD>) _param_uwb_uuid_on_sd,	/**< UUID on SD card  */
		(ParamInt<px4::params::UWB_POS_DEBUG>) _param_uwb_pos_debug	/**< Debug POS */
	)



	int _uart;
	fd_set _uart_set;
	struct timeval _uart_timeout {};

	uORB::Publication<uwb_grid_s> _uwb_grid_pub{ORB_ID(uwb_grid)};
	uwb_grid_s _uwb_grid{};

	uORB::Publication<uwb_distance_s> _uwb_distance_pub{ORB_ID(uwb_distance)};
	uwb_distance_s _uwb_distance{};

	uORB::Publication<landing_target_pose_s> _landing_target_pub{ORB_ID(landing_target_pose)};
	landing_target_pose_s _landing_target{};

	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	vehicle_attitude_s _vehicle_attitude{};

	grid_msg_t _grid_survey_msg{};
	distance_msg_t _distance_result_msg{};
	float x_rel_position; //float for the Position result
	float y_rel_position;
	float z_rel_position;

	matrix::Dcmf _uwb_r4_to_nwu;
	matrix::Dcmf _nwu_to_ned{matrix::Eulerf(M_PI_F, 0.0f, 0.0f)};
	matrix::Vector3f _current_position_uwb_r4;
	matrix::Vector3f _current_position_ned;


	perf_counter_t _read_count_perf;
	perf_counter_t _read_err_perf;
};


#endif //PX4_RDDRONE_H
