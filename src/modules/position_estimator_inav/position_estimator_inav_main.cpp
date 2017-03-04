/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file position_estimator_inav_main.c
 * Model-identification based position estimator for multirotors
 * 基于多旋翼位置估计的模型识别
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Nuno Marques <n.marques21@hotmail.com>
 * @author Christoph Tobler <toblech@student.ethz.ch>
 */
#include <px4_posix.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <fcntl.h>
#include <string.h>
#include <px4_config.h>
#include <math.h>
#include <float.h>
#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vision_position_estimate.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/distance_sensor.h>
#include <poll.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>
#include <geo/geo.h>
#include <systemlib/systemlib.h>
#include <drivers/drv_hrt.h>
#include <platforms/px4_defines.h>

#include <terrain_estimation/terrain_estimator.h>
#include "position_estimator_inav_params.h"
#include "inertial_filter.h"

#define MIN_VALID_W 0.00001f
#define PUB_INTERVAL 10000	// limit publish rate to 100 Hz  0.01s
#define EST_BUF_SIZE 250000 / PUB_INTERVAL		// buffer size is 0.5s
#define MAX_WAIT_FOR_BARO_SAMPLE 3000000 // wait 3 secs for the baro to respond

static bool thread_should_exit = false; /**< Deamon exit flag */
static bool thread_running = false; /**< Deamon status flag */
static int position_estimator_inav_task; /**< Handle of deamon task / thread */
static bool inav_verbose_mode = false; // 惯性导航冗余模式

static const hrt_abstime vision_topic_timeout = 500000;	// Vision topic timeout = 0.5s
static const hrt_abstime mocap_topic_timeout = 500000;		// Mocap topic timeout = 0.5s
static const hrt_abstime gps_topic_timeout = 500000;		// GPS topic timeout = 0.5s
static const hrt_abstime flow_topic_timeout = 1000000;	// optical flow topic timeout = 1s
static const hrt_abstime lidar_timeout = 150000;	// lidar timeout = 150ms
static const hrt_abstime lidar_valid_timeout = 1000000;	// estimate lidar distance during this time after lidar loss
static const unsigned updates_counter_len = 1000000;    // 时间长度1s
static const float max_flow = 1.0f;	// max flow value that can be used, rad/s

extern "C" __EXPORT int position_estimator_inav_main(int argc, char *argv[]);

int position_estimator_inav_thread_main(int argc, char *argv[]);

static void usage(const char *reason);

static inline int min(int val1, int val2)
{
	return (val1 < val2) ? val1 : val2;
}

static inline int max(int val1, int val2)
{
	return (val1 > val2) ? val1 : val2;
}

/**
 * Print the correct usage.
 */
static void usage(const char *reason)
{
	if (reason && *reason) {
		PX4_INFO("%s", reason);
	}

	PX4_INFO("usage: position_estimator_inav {start|stop|status} [-v]\n");
	return;
}
 
/**
 * The position_estimator_inav_thread only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int position_estimator_inav_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
	}

	if (!strcmp(argv[1], "start")) {
		if (thread_running) {
			warnx("already running");
			/* this is not an error */
			return 0;
		}

		inav_verbose_mode = false;

		if ((argc > 2) && (!strcmp(argv[2], "-v"))) {
			inav_verbose_mode = true;
		}

		thread_should_exit = false;
		// 创建任务
		position_estimator_inav_task = px4_task_spawn_cmd("position_estimator_inav",
					       SCHED_DEFAULT, SCHED_PRIORITY_MAX - 5, 4600,
					       position_estimator_inav_thread_main,
					       (argv && argc > 2) ? (char *const *) &argv[2] : (char *const *) NULL);
		//这会启动一个拥有4600字节的栈，并将非deamon指令行 position_estimator_inav_thread_main传给递后台主函数
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (thread_running) {
			warnx("stop");
			thread_should_exit = true;

		} else {
			warnx("not started");
		}

		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("is running");

		} else {
			warnx("not started");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

#ifdef INAV_DEBUG
static void write_debug_log(const char *msg, float dt, float x_est[2], float y_est[2], float z_est[2],
			    float x_est_prev[2], float y_est_prev[2], float z_est_prev[2],
			    float acc[3], float corr_gps[3][2], float w_xy_gps_p, float w_xy_gps_v, float corr_mocap[3][1], float w_mocap_p,
			    float corr_vision[3][2], float w_xy_vision_p, float w_z_vision_p, float w_xy_vision_v)
{
	FILE *f = fopen(PX4_ROOTFSDIR"/fs/microsd/inav.log", "a");

	if (f) {
		char *s = malloc(256);
		unsigned n = snprintf(s, 256,
				      "%llu %s\n\tdt=%.5f x_est=[%.5f %.5f] y_est=[%.5f %.5f] z_est=[%.5f %.5f] x_est_prev=[%.5f %.5f] y_est_prev=[%.5f %.5f] z_est_prev=[%.5f %.5f]\n",
				      (unsigned long long)hrt_absolute_time(), msg, (double)dt,
				      (double)x_est[0], (double)x_est[1], (double)y_est[0], (double)y_est[1], (double)z_est[0], (double)z_est[1],
				      (double)x_est_prev[0], (double)x_est_prev[1], (double)y_est_prev[0], (double)y_est_prev[1], (double)z_est_prev[0],
				      (double)z_est_prev[1]);
		fwrite(s, 1, n, f);
		n = snprintf(s, 256,
			     "\tacc=[%.5f %.5f %.5f] gps_pos_corr=[%.5f %.5f %.5f] gps_vel_corr=[%.5f %.5f %.5f] w_xy_gps_p=%.5f w_xy_gps_v=%.5f mocap_pos_corr=[%.5f %.5f %.5f] w_mocap_p=%.5f\n",
			     (double)acc[0], (double)acc[1], (double)acc[2],
			     (double)corr_gps[0][0], (double)corr_gps[1][0], (double)corr_gps[2][0], (double)corr_gps[0][1], (double)corr_gps[1][1],
			     (double)corr_gps[2][1],
			     (double)w_xy_gps_p, (double)w_xy_gps_v, (double)corr_mocap[0][0], (double)corr_mocap[1][0], (double)corr_mocap[2][0],
			     (double)w_mocap_p);
		fwrite(s, 1, n, f);
		n = snprintf(s, 256,
			     "\tvision_pos_corr=[%.5f %.5f %.5f] vision_vel_corr=[%.5f %.5f %.5f] w_xy_vision_p=%.5f w_z_vision_p=%.5f w_xy_vision_v=%.5f\n",
			     (double)corr_vision[0][0], (double)corr_vision[1][0], (double)corr_vision[2][0], (double)corr_vision[0][1],
			     (double)corr_vision[1][1], (double)corr_vision[2][1],
			     (double)w_xy_vision_p, (double)w_z_vision_p, (double)w_xy_vision_v);
		fwrite(s, 1, n, f);
		free(s);
	}

	fsync(fileno(f));
	fclose(f);
}
#else
#define write_debug_log(...)
#endif
#include <uORB/topics/att_pos_mocap.h>

/****************************************************************************
 * main
 * 所需的高度信息是地理坐标系下的相对高度，
 * 整个算法的核心思想是由地理坐标系下的加速度通过积分，
 * 来获得速度、位置信息，而这个数据的精确程度是由机体测量的加速度通过减去偏差，
 * 再转换到地理坐标系求得的。
 * 这里气压计的作用就是计算一个校正系数来对加速度偏移量进行校正。
 ****************************************************************************/
int position_estimator_inav_thread_main(int argc, char *argv[])
{
	orb_advert_t mavlink_log_pub = nullptr;

	float x_est[2] = { 0.0f, 0.0f };	// pos, vel
	float y_est[2] = { 0.0f, 0.0f };	// pos, vel
	float z_est[2] = { 0.0f, 0.0f };	// pos, vel

	float est_buf[EST_BUF_SIZE][3][2];	// 缓存估计得到的位置
	float R_buf[EST_BUF_SIZE][3][3];	// 缓存估计得到的旋转矩阵
	float R_gps[3][3];					// 用于GPS moment校正的旋转矩阵
	memset(est_buf, 0, sizeof(est_buf));
	memset(R_buf, 0, sizeof(R_buf));
	memset(R_gps, 0, sizeof(R_gps));
	int buf_ptr = 0;

	static const float min_eph_epv = 2.0f;	// min EPH/EPV, used for weight calculation
	static const float max_eph_epv = 20.0f;	// max EPH/EPV acceptable for estimation

	float eph = max_eph_epv;
	float epv = 1.0f;

	float eph_flow = 1.0f;

	float eph_vision = 0.2f;
	float epv_vision = 0.2f;

	float eph_mocap = 0.05f;
	float epv_mocap = 0.05f;

	float x_est_prev[2], y_est_prev[2], z_est_prev[2];
	memset(x_est_prev, 0, sizeof(x_est_prev));
	memset(y_est_prev, 0, sizeof(y_est_prev));
	memset(z_est_prev, 0, sizeof(z_est_prev));

	int baro_init_cnt = 0;
	int baro_init_num = 200;
	float baro_offset = 0.0f;		// baro offset for reference altitude, initialized on start, then adjusted

	hrt_abstime accel_timestamp = 0;
	hrt_abstime baro_timestamp = 0;

	bool ref_inited = false;
	hrt_abstime ref_init_start = 0;
	const hrt_abstime ref_init_delay = 1000000;	// wait for 1s after 3D fix
	struct map_projection_reference_s ref;
	memset(&ref, 0, sizeof(ref));

	uint16_t accel_updates = 0;
	uint16_t baro_updates = 0;
	uint16_t gps_updates = 0;
	uint16_t attitude_updates = 0;
	uint16_t flow_updates = 0;
	uint16_t vision_updates = 0;
	uint16_t mocap_updates = 0;

	hrt_abstime updates_counter_start = hrt_absolute_time();
	hrt_abstime pub_last = hrt_absolute_time();

	hrt_abstime t_prev = 0;

	/* store error when sensor updates, but correct on each time step to avoid jumps in estimated value */
       // 当传感器更新时储存错误，在每一时刻进行校正以避免在位置估计中发生跳变
	float acc[] = { 0.0f, 0.0f, 0.0f };	// N E D
	float acc_bias[] = { 0.0f, 0.0f, 0.0f };	// body frame
	float corr_baro = 0.0f;		// D
	float corr_gps[3][2] = {
		{ 0.0f, 0.0f },		// N (pos, vel)
		{ 0.0f, 0.0f },		// E (pos, vel)
		{ 0.0f, 0.0f },		// D (pos, vel)
	};
	float w_gps_xy = 1.0f;
	float w_gps_z = 1.0f;

	float corr_vision[3][2] = {
		{ 0.0f, 0.0f },		// N (pos, vel)
		{ 0.0f, 0.0f },		// E (pos, vel)
		{ 0.0f, 0.0f },		// D (pos, vel)
	};

	float corr_mocap[3][1] = {
		{ 0.0f },		// N (pos)
		{ 0.0f },		// E (pos)
		{ 0.0f },		// D (pos)
	};
	const int mocap_heading = 2;

	float dist_ground = 0.0f;		//variables for lidar altitude estimation
	float corr_lidar = 0.0f;
	float lidar_offset = 0.0f;
	int lidar_offset_count = 0;
	bool lidar_first = true;
	bool use_lidar = false;
	bool use_lidar_prev = false;

	float corr_flow[] = { 0.0f, 0.0f };	// N E
	float w_flow = 0.0f;

	hrt_abstime lidar_time = 0;			// time of last lidar measurement (not filtered)
	                                                                //上一次lidar测量的时间(未滤波)
	hrt_abstime lidar_valid_time = 0;	// time of last lidar measurement used for correction (filtered)
	                                                                //用于校正的上一次lidar测量的时间(已滤波)

	int n_flow = 0;
	float gyro_offset_filtered[] = { 0.0f, 0.0f, 0.0f };
	float flow_gyrospeed[] = { 0.0f, 0.0f, 0.0f };
	float flow_gyrospeed_filtered[] = { 0.0f, 0.0f, 0.0f };
	float att_gyrospeed_filtered[] = { 0.0f, 0.0f, 0.0f };
	float yaw_comp[] = { 0.0f, 0.0f };
	hrt_abstime flow_time = 0;
	float flow_min_dist = 0.2f;

	bool gps_valid = false;			// GPS is valid
	bool lidar_valid = false;		// lidar is valid
	bool flow_valid = false;		// flow is valid
	bool flow_accurate = false;		// flow should be accurate (this flag not updated if flow_valid == false)
	bool vision_valid = false;		// vision is valid
	bool mocap_valid = false;		// mocap is valid

	/* declare and safely initialize all structs */
	struct actuator_controls_s actuator;
	memset(&actuator, 0, sizeof(actuator));
	struct actuator_armed_s armed;
	memset(&armed, 0, sizeof(armed));
	struct sensor_combined_s sensor;
	memset(&sensor, 0, sizeof(sensor));
	struct vehicle_gps_position_s gps;
	memset(&gps, 0, sizeof(gps));
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct vehicle_local_position_s local_pos;
	memset(&local_pos, 0, sizeof(local_pos));
	struct optical_flow_s flow;
	memset(&flow, 0, sizeof(flow));
	struct vision_position_estimate_s vision;
	memset(&vision, 0, sizeof(vision));
	struct att_pos_mocap_s mocap;
	memset(&mocap, 0, sizeof(mocap));
	struct vehicle_global_position_s global_pos;
	memset(&global_pos, 0, sizeof(global_pos));
	struct distance_sensor_s lidar;
	memset(&lidar, 0, sizeof(lidar));
	struct vehicle_rates_setpoint_s rates_setpoint;
	memset(&rates_setpoint, 0, sizeof(rates_setpoint));

	/* subscribe */
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	int actuator_sub = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);
	int armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int optical_flow_sub = orb_subscribe(ORB_ID(optical_flow));
	int vehicle_gps_position_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	int vision_position_estimate_sub = orb_subscribe(ORB_ID(vision_position_estimate));
	int att_pos_mocap_sub = orb_subscribe(ORB_ID(att_pos_mocap));
	int distance_sensor_sub = orb_subscribe(ORB_ID(distance_sensor));
	int vehicle_rate_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));

	/* advertise */
	orb_advert_t vehicle_local_position_pub = orb_advertise(ORB_ID(vehicle_local_position), &local_pos);
	orb_advert_t vehicle_global_position_pub = NULL;

	struct position_estimator_inav_params params;
	memset(&params, 0, sizeof(params));
	struct position_estimator_inav_param_handles pos_inav_param_handles;
	/* initialize parameter handles */
	// 初始化参数句柄
	inav_parameters_init(&pos_inav_param_handles);

	/* first parameters read at start up */
	// 开始时第一次读取数据
	struct parameter_update_s param_update;
	orb_copy(ORB_ID(parameter_update), parameter_update_sub,
		 &param_update); /* read from param topic to clear updated flag */
	/* first parameters update */
	// 第一次参数更新
	inav_parameters_update(&pos_inav_param_handles, &params); // (参数句柄，参数)

	px4_pollfd_struct_t fds_init[1] = {};
	fds_init[0].fd = sensor_combined_sub;
	fds_init[0].events = POLLIN;

	/* wait for initial baro value */
	// 主循环开始之前需要对气压计进行初始化
	bool wait_baro = true;
	TerrainEstimator terrain_estimator;

	thread_running = true;
	hrt_abstime baro_wait_for_sample_time = hrt_absolute_time();

	while (wait_baro && !thread_should_exit) {
		int ret = px4_poll(&fds_init[0], 1, 1000);

		if (ret < 0) {
			/* poll error */
			mavlink_log_info(&mavlink_log_pub, "[inav] poll error on init");
		} else if (hrt_absolute_time() - baro_wait_for_sample_time > MAX_WAIT_FOR_BARO_SAMPLE) {
			wait_baro = false;
			mavlink_log_info(&mavlink_log_pub, "[inav] timed out waiting for a baro sample");
		}
		else if (ret > 0) {
			if (fds_init[0].revents & POLLIN) {
				orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor);

				if (wait_baro && sensor.timestamp + sensor.baro_timestamp_relative != baro_timestamp) {
					baro_timestamp = sensor.timestamp + sensor.baro_timestamp_relative;
					baro_wait_for_sample_time = hrt_absolute_time();

					/* mean calculation over several measurements */
					// 多次测量取平均值
					if (baro_init_cnt < baro_init_num) {
						if (PX4_ISFINITE(sensor.baro_alt_meter)) {
							baro_offset += sensor.baro_alt_meter;
							baro_init_cnt++;
						}

					} else {
						wait_baro = false;
						baro_offset /= (float) baro_init_cnt; //测量200次，取气压计偏移的平均值
						local_pos.z_valid = true;
						local_pos.v_z_valid = true;
					}
				}
			}

		} else {
			PX4_WARN("INAV poll timeout");
		}
	}

	/* main loop */
	// 主循环开始
	px4_pollfd_struct_t fds[1];
	fds[0].fd = vehicle_attitude_sub; // 主题
	fds[0].events = POLLIN;

	while (!thread_should_exit) {
		int ret = px4_poll(fds, 1, 20); // wait maximal 20 ms = 50 Hz minimum rate 更新频率
		hrt_abstime t = hrt_absolute_time();

		if (ret < 0) {
			/* poll error */
			mavlink_log_info(&mavlink_log_pub, "[inav] poll error on init");
			continue;

		} else if (ret > 0) {
			/* act on attitude updates */
			// poll返回值大于0，进行姿态更新

			/* vehicle attitude */
			// 飞行器的姿态
			orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);
			attitude_updates++; // 将更新的数据放入vehicle_attitude_sub结构体中然后更新次数加一

			bool updated;

			/* parameter update */
			// 参数更新    
			orb_check(parameter_update_sub, &updated);  // 先check  再copy

			if (updated) {
				struct parameter_update_s update;
				orb_copy(ORB_ID(parameter_update), parameter_update_sub, &update);
				inav_parameters_update(&pos_inav_param_handles, &params);
			}

			/* actuator */
			// 执行器更新
			orb_check(actuator_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_sub, &actuator);
			}

			/* armed */
			orb_check(armed_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(actuator_armed), armed_sub, &armed);
			}

			/* sensor combined */
			orb_check(sensor_combined_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor);

				/*
				 * 接着在加速度时间帧更新的情况下，
				 * 如果满足姿态旋转矩阵有效，即R_valid为true，
				 * 那么为机体坐标系下加速度值accelerometer_m_s2修正偏移，即减去acc_bias。
				 * 接着转换到地理坐标系下，并给z轴加上重力加速度G
				 * 旋转矩阵无效的情况则重新为acc向量分配内存		
				 */
				if (sensor.timestamp + sensor.accelerometer_timestamp_relative != accel_timestamp) {
					if (att.R_valid) {
						/* correct accel bias */
						// 校正加速度偏差       
						sensor.accelerometer_m_s2[0] -= acc_bias[0];   //全部都是为了计算acc用于最后的位置速度估计校正，先计算acc_bias[]
						sensor.accelerometer_m_s2[1] -= acc_bias[1];   //选用各种外置传感器，分别计算其对加速度计偏差的矫正
						sensor.accelerometer_m_s2[2] -= acc_bias[2];   // 最后对传感器获得的加速度进行校正

						/* transform acceleration vector from body frame to NED frame */
						// 将加速度向量从机体坐标系转换到地球坐标系
						for (int i = 0; i < 3; i++) {
							acc[i] = 0.0f;

							for (int j = 0; j < 3; j++) {
								acc[i] += PX4_R(att.R, i, j) * sensor.accelerometer_m_s2[j];
							}
						}

						acc[2] += CONSTANTS_ONE_G;

					} else {
						memset(acc, 0, sizeof(acc));  // 加速度值赋0
					}
					// 加速度时间戳不断增大，不停更新，代表一直在取得新的加速度值
					accel_timestamp = sensor.timestamp + sensor.accelerometer_timestamp_relative;
					accel_updates++;
				}

				if (sensor.timestamp + sensor.baro_timestamp_relative != baro_timestamp) {
					corr_baro = baro_offset - sensor.baro_alt_meter - z_est[0]; // 首次获取气压计的矫正系数
					baro_timestamp = sensor.timestamp + sensor.baro_timestamp_relative; //气压计的时间戳更新
					baro_updates++;
				}
			}


			/* lidar alt estimation */
			// 激光雷达高度估计更新
			orb_check(distance_sensor_sub, &updated);

			/* update lidar separately, needed by terrain estimator */
			// 单独更新激光雷达，估计地势环境所需
			if (updated) {
				orb_copy(ORB_ID(distance_sensor), distance_sensor_sub, &lidar);
				lidar.current_distance += params.lidar_calibration_offset;
			}

			if (updated) { //check if altitude estimation for lidar is enabled and new sensor data
			                     // 检查lidar的高度估计是否使能以及新的传感器数据

				if (params.enable_lidar_alt_est && lidar.current_distance > lidar.min_distance && lidar.current_distance < lidar.max_distance
			    		&& (PX4_R(att.R, 2, 2) > 0.7f)) {//检查enable_lidar_alt_est参数是否置1，以使能lidar估计
 
					if (!use_lidar_prev && use_lidar) {
						lidar_first = true;
					}

					use_lidar_prev = use_lidar;

					lidar_time = t;
					dist_ground = lidar.current_distance * PX4_R(att.R, 2, 2); //vertical distance 垂直距离
																			   // 当机体x0y平面倾斜时，d为超声波测距lidar.current_distance
																			   // cos(theta)为两坐标系z轴的夹角PX4_R(att.R, 2, 2)
																			   // 与地面垂直距离为 d * cos(theta)

					if (lidar_first) {
						lidar_first = false; //初次检验的值仅适用于首次
						lidar_offset = dist_ground + z_est[0]; // 更新偏移
						mavlink_log_info(&mavlink_log_pub, "[inav] LIDAR: new ground offset");
						warnx("[inav] LIDAR: new ground offset");
					}

					corr_lidar = lidar_offset - dist_ground - z_est[0];  //更新校正值

					if (fabsf(corr_lidar) > params.lidar_err) { //check for skiped
						corr_lidar = 0;
						lidar_valid = false; //要使用光流首要要求lidar_valid为真
						lidar_offset_count++;

						if (lidar_offset_count > 3) { //if consecutive bigger/smaller measurements -> new ground offset -> reinit
											   //如果连续几次测量值都过大或者过小 -> 获得新的对地补偿量 -> 重新初始化
							lidar_first = true;
							lidar_offset_count = 0;
						}

					} else {
					// 校正值小于误差，校正值有效，再次更新，并使用激光雷达，更新有效使用时间
						corr_lidar = lidar_offset - dist_ground - z_est[0];
						lidar_valid = true;
						lidar_offset_count = 0;
						lidar_valid_time = t; //t = hrt_absolute_time() 更新雷达有效时间
					}
				} else { // enable_lidar_alt_est=0
					lidar_valid = false;
				}
			}

			/* optical flow */
			// 光流更新
			orb_check(optical_flow_sub, &updated);
			
			// 想用光流 第一个条件就是lidar是否有效，在位置估计里雷达就是距离传感器，
			// 对应的实际就是超声波传感器，（实测lidar的数据都是超声波提供的） 
			if (updated && lidar_valid) {
				orb_copy(ORB_ID(optical_flow), optical_flow_sub, &flow);

				flow_time = t;
				float flow_q = flow.quality / 255.0f;
				float dist_bottom = lidar.current_distance;

				if (dist_bottom > flow_min_dist && flow_q > params.flow_q_min && PX4_R(att.R, 2, 2) > 0.7f) {
					/* distance to surface 距离地面的高度 */
					//float flow_dist = dist_bottom / PX4_R(att.R, 2, 2); //use this if using sonar
					float flow_dist = dist_bottom; //use this if using lidar 如果使用了lidar则将离地距离赋值给flow_dist

					/* check if flow if too large for accurate measurements */
					/* calculate estimated velocity in body frame */
					// 计算机体坐标系中x、y轴向的估计速度
					float body_v_est[2] = { 0.0f, 0.0f }; 

					for (int i = 0; i < 2; i++) { //将估计的机体速度转换到地球坐标系x,y轴上
						body_v_est[i] = PX4_R(att.R, 0, i) * x_est[1] + PX4_R(att.R, 1, i) * y_est[1] + PX4_R(att.R, 2, i) * z_est[1];
					}

					/* set this flag if flow should be accurate according to current velocity and attitude rate estimate */
					//根据当前的速度和角速度估计判断光流是否精确，并将此标志位置位
					flow_accurate = fabsf(body_v_est[1] / flow_dist - att.rollspeed) < max_flow &&
							fabsf(body_v_est[0] / flow_dist + att.pitchspeed) < max_flow;//机体x对应地理-Y，机体y对应地理X. 速度/距离?

					/*calculate offset of flow-gyro using already calibrated gyro from autopilot*/
					// 使用已经校准的自驾仪上的陀螺仪计算光流模块上的陀螺仪的偏移
					flow_gyrospeed[0] = flow.gyro_x_rate_integral / (float)flow.integration_timespan * 1000000.0f;
					flow_gyrospeed[1] = flow.gyro_y_rate_integral / (float)flow.integration_timespan * 1000000.0f;
					flow_gyrospeed[2] = flow.gyro_z_rate_integral / (float)flow.integration_timespan * 1000000.0f;

					//moving average
					//每百次获取一次偏差，求均值
					if (n_flow >= 100) { //开始n_flow=0，进入else()
						gyro_offset_filtered[0] = flow_gyrospeed_filtered[0] - att_gyrospeed_filtered[0];
						gyro_offset_filtered[1] = flow_gyrospeed_filtered[1] - att_gyrospeed_filtered[1];
						gyro_offset_filtered[2] = flow_gyrospeed_filtered[2] - att_gyrospeed_filtered[2];
						n_flow = 0;  // 清零
						flow_gyrospeed_filtered[0] = 0.0f;
						flow_gyrospeed_filtered[1] = 0.0f;
						flow_gyrospeed_filtered[2] = 0.0f;
						att_gyrospeed_filtered[0] = 0.0f;
						att_gyrospeed_filtered[1] = 0.0f;
						att_gyrospeed_filtered[2] = 0.0f;

					} else { //迭代更新
						flow_gyrospeed_filtered[0] = (flow_gyrospeed[0] + n_flow * flow_gyrospeed_filtered[0]) / (n_flow + 1);
						flow_gyrospeed_filtered[1] = (flow_gyrospeed[1] + n_flow * flow_gyrospeed_filtered[1]) / (n_flow + 1);
						flow_gyrospeed_filtered[2] = (flow_gyrospeed[2] + n_flow * flow_gyrospeed_filtered[2]) / (n_flow + 1);
						att_gyrospeed_filtered[0] = (att.pitchspeed + n_flow * att_gyrospeed_filtered[0]) / (n_flow + 1);
						att_gyrospeed_filtered[1] = (att.rollspeed + n_flow * att_gyrospeed_filtered[1]) / (n_flow + 1);
						att_gyrospeed_filtered[2] = (att.yawspeed + n_flow * att_gyrospeed_filtered[2]) / (n_flow + 1);
						n_flow++;
					}


					/*yaw compensation (flow sensor is not in center of rotation) -> params in QGC*/
					// 偏航补偿(由于光流传感器不一定在机体的中心) -> 使用QGC中的参数也可以进行更改
					// 实际光流的安装 光流模块的y指向飞控的正前方，即X方向；光流模块的x指向左侧,即飞控的-Y
					// 这里皆取反表示补偿？
					yaw_comp[0] = - params.flow_module_offset_y * (flow_gyrospeed[2] - gyro_offset_filtered[2]);
					yaw_comp[1] = params.flow_module_offset_x * (flow_gyrospeed[2] - gyro_offset_filtered[2]);
					/*
					 * 光流如果安装在旋转中心，仅有自转时，光流x、y轴的输出应该为0。
					 * 但是当偏心安装中，仅有自转时，光流x、y轴会有一个额外输出，这个公式就是补偿这个额外的输出
					 */

					/* convert raw flow to angular flow (rad/s) */
					/* 
					 * 对于光流获得的弧度值我们需要通过相应的公式转换为角度值 ,
					 * 根据地理坐标系下机体角速度参考点的大小与角速度阈值关系来决定光流角度
					 *（XY平面，参考点数据通过uORB获取）
					 *
					 */
					float flow_ang[2];

					/* check for vehicle rates setpoint - it below threshold -> dont subtract -> better hover */
					// 检查飞行器的目标角度是否更新 -> 如果目標角度小于阈值 -> 不用减 -> 可以实现更好的悬停效果
					orb_check(vehicle_rate_sp_sub, &updated);
					if (updated)
						orb_copy(ORB_ID(vehicle_rates_setpoint), vehicle_rate_sp_sub, &rates_setpoint);

					float rate_threshold = 0.15f;

					if (fabsf(rates_setpoint.pitch) < rate_threshold) {
						//warnx("[inav] test ohne comp");
						flow_ang[0] = (flow.pixel_flow_x_integral / (float)flow.integration_timespan * 1000000.0f) * params.flow_k;//for now the flow has to be scaled (to small)
					}
					else {
						//warnx("[inav] test mit comp");
						//calculate flow [rad/s] and compensate for rotations (and offset of flow-gyro)
						// 高于阈值时，将光流测得的角速度和光流模块上的陀螺仪偏移作为旋转的补偿
						flow_ang[0] = ((flow.pixel_flow_x_integral - flow.gyro_x_rate_integral) / (float)flow.integration_timespan * 1000000.0f
							       + gyro_offset_filtered[0]) * params.flow_k;//for now the flow has to be scaled (to small)
					}

					if (fabsf(rates_setpoint.roll) < rate_threshold) {
						flow_ang[1] = (flow.pixel_flow_y_integral / (float)flow.integration_timespan * 1000000.0f) * params.flow_k;//for now the flow has to be scaled (to small)
																										     // flow_k光流比例因子
					}
					else {
						//calculate flow [rad/s] and compensate for rotations (and offset of flow-gyro)
						flow_ang[1] = ((flow.pixel_flow_y_integral - flow.gyro_y_rate_integral) / (float)flow.integration_timespan * 1000000.0f
							       + gyro_offset_filtered[1]) * params.flow_k;//for now the flow has to be scaled (to small)
					}

					/* flow measurements vector */
					// 光流的测量值--距离
					float flow_m[3];
					if (fabsf(rates_setpoint.yaw) < rate_threshold) {
						flow_m[0] = -flow_ang[0] * flow_dist;
						flow_m[1] = -flow_ang[1] * flow_dist;
					} else {
					/* 偏航设定值超过阈值时，用之前的补偿数据来补偿XY平面 */
						flow_m[0] = -flow_ang[0] * flow_dist - yaw_comp[0] * params.flow_k;
						flow_m[1] = -flow_ang[1] * flow_dist - yaw_comp[1] * params.flow_k;
					}
					flow_m[2] = z_est[1];

					/* velocity in NED */
					// 地理坐标系中的速度
					float flow_v[2] = { 0.0f, 0.0f };

					/* project measurements vector to NED basis, skip Z component */
					/* 投射到地理坐标系，不使用z轴所以不用计算 */
					for (int i = 0; i < 2; i++) {
						for (int j = 0; j < 3; j++) {
							flow_v[i] += PX4_R(att.R, i, j) * flow_m[j];
						}
					}

					/* velocity correction */
					// 速度校正
					corr_flow[0] = flow_v[0] - x_est[1];
					corr_flow[1] = flow_v[1] - y_est[1];
					/* adjust correction weight */
					// 调节校正权重 
					float flow_q_weight = (flow_q - params.flow_q_min) / (1.0f - params.flow_q_min);//光流比例因子的权重

					/*
					 * 根据品质因子，高度以及旋转矩阵的最后一个数据PX4_R(att.R, 2, 2)对光流的权重进行调整
					 * （这里用到了这个数据，之前判断需要大于0.7的，cos(theta)>0.7,表示两Z轴之间的夹角较小）
					 */
					w_flow = PX4_R(att.R, 2, 2) * flow_q_weight / fmaxf(1.0f, flow_dist);


					/* if flow is not accurate, reduce weight for it */
					// 如果光流不准确，则减少光流的权重，光流测得的角速度(rad/s)<1时，accurate
					// TODO make this more fuzzy
					if (!flow_accurate) {  //光流的准确度根据估计的速度与角速度判断
						w_flow *= 0.05f;
					}

					/* under ideal conditions, on 1m distance assume EPH = 10cm */
					// 在理想情况下，距离为1米时假定光流的水平位置精度为0.1m
					eph_flow = 0.1f / w_flow;

					flow_valid = true;

				} else { //光流距离地面距离过高，或者光流质量太差 
					w_flow = 0.0f;
					flow_valid = false;
				}

				flow_updates++;
			}

			/* check no vision circuit breaker is set */
			/*
			 * 
			 * 视觉断路器判断：
			 * 这是判断是否使用了视觉传感器，根据参数相关代码中的视觉参数来判断
			 *（无视觉工具时该参数为328754），没有视觉工具直接跳过该部分。
			 * 有视觉工具但未获得uORB数据更新也跳过该步骤。
			 *
			 */
			if (params.no_vision != CBRK_NO_VISION_KEY) {
				/* vehicle vision position */
				orb_check(vision_position_estimate_sub, &updated);

				if (updated) {
					orb_copy(ORB_ID(vision_position_estimate), vision_position_estimate_sub, &vision);

					static float last_vision_x = 0.0f;
					static float last_vision_y = 0.0f;
					static float last_vision_z = 0.0f;

					/* reset position estimate on first vision update */
					if (!vision_valid) {
						x_est[0] = vision.x;
						x_est[1] = vision.vx;
						y_est[0] = vision.y;
						y_est[1] = vision.vy;

						/* only reset the z estimate if the z weight parameter is not zero */
						if (params.w_z_vision_p > MIN_VALID_W) {
							z_est[0] = vision.z;
							z_est[1] = vision.vz;
						}

						vision_valid = true;

						last_vision_x = vision.x;
						last_vision_y = vision.y;
						last_vision_z = vision.z;

						warnx("VISION estimate valid");
						mavlink_log_info(&mavlink_log_pub, "[inav] VISION estimate valid");
					}

					/* calculate correction for position */
					corr_vision[0][0] = vision.x - x_est[0];
					corr_vision[1][0] = vision.y - y_est[0];
					corr_vision[2][0] = vision.z - z_est[0];

					static hrt_abstime last_vision_time = 0;

					float vision_dt = (vision.timestamp - last_vision_time) / 1e6f;
					last_vision_time = vision.timestamp;

					if (vision_dt > 0.000001f && vision_dt < 0.2f) {
						vision.vx = (vision.x - last_vision_x) / vision_dt;
						vision.vy = (vision.y - last_vision_y) / vision_dt;
						vision.vz = (vision.z - last_vision_z) / vision_dt;

						last_vision_x = vision.x;
						last_vision_y = vision.y;
						last_vision_z = vision.z;

						/* calculate correction for velocity */
						corr_vision[0][1] = vision.vx - x_est[1];
						corr_vision[1][1] = vision.vy - y_est[1];
						corr_vision[2][1] = vision.vz - z_est[1];

					} else {
						/* assume zero motion */
						corr_vision[0][1] = 0.0f - x_est[1];
						corr_vision[1][1] = 0.0f - y_est[1];
						corr_vision[2][1] = 0.0f - z_est[1];
					}

					vision_updates++;
				}
			}

			/* vehicle mocap position */
			// 动作捕捉更新
			orb_check(att_pos_mocap_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(att_pos_mocap), att_pos_mocap_sub, &mocap);

				if (!params.disable_mocap) {
					/* reset position estimate on first mocap update */
					if (!mocap_valid) {
						x_est[0] = mocap.x;
						y_est[0] = mocap.y;
						z_est[0] = mocap.z;

						mocap_valid = true;

						warnx("MOCAP data valid");
						mavlink_log_info(&mavlink_log_pub, "[inav] MOCAP data valid");
					}

					/* calculate correction for position */
					corr_mocap[0][0] = mocap.x - x_est[0];
					corr_mocap[1][0] = mocap.y - y_est[0];
					corr_mocap[2][0] = mocap.z - z_est[0];

					mocap_updates++;
				}
			}

			/* vehicle GPS position */
			// 飞行器的GPS位置
			orb_check(vehicle_gps_position_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_position_sub, &gps);

				bool reset_est = false;

				/* hysteresis for GPS quality */
				// GPS质量的滞后
				if (gps_valid) {
					if (gps.eph > max_eph_epv || gps.epv > max_eph_epv || gps.fix_type < 3) {
						gps_valid = false;
						mavlink_log_info(&mavlink_log_pub, "[inav] GPS signal lost");
						warnx("[inav] GPS signal lost");
					}

				} else {
					if (gps.eph < max_eph_epv * 0.7f && gps.epv < max_eph_epv * 0.7f && gps.fix_type >= 3) {
						gps_valid = true;
						reset_est = true;
						mavlink_log_info(&mavlink_log_pub, "[inav] GPS signal found");
						warnx("[inav] GPS signal found");
					}
				}

				if (gps_valid) { //GPS有效，获取经纬度与高度信息
					double lat = gps.lat * 1e-7;
					double lon = gps.lon * 1e-7;
					float alt = gps.alt * 1e-3;

					/* initialize reference position if needed */
					// 初始化参考位置
					if (!ref_inited) {
						if (ref_init_start == 0) {
							ref_init_start = t; //未初始化参考值，将当前时间赋给初始化开始的时间，即现在开始初始化

						} else if (t > ref_init_start + ref_init_delay) {
							ref_inited = true;

							/* set position estimate to (0, 0, 0), use GPS velocity for XY */
							// 设置初始位置估计点为(0,0,0)，使用GPS速度用于XY轴的测量
							x_est[0] = 0.0f; //x轴位置
							x_est[1] = gps.vel_n_m_s; // x轴速度
							y_est[0] = 0.0f;
							y_est[1] = gps.vel_e_m_s;

							local_pos.ref_lat = lat; //将GPS获取的经纬度赋给相对位置的经纬度
							local_pos.ref_lon = lon;
							local_pos.ref_alt = alt + z_est[0];
							local_pos.ref_timestamp = t;

							/* initialize projection */
							// 初始化    
							map_projection_init(&ref, lat, lon); //将度数转换为弧度，弧度才能进行三角运算 sin  cos
							// XXX replace this print
							warnx("init ref: lat=%.7f, lon=%.7f, alt=%8.4f", (double)lat, (double)lon, (double)alt);
							mavlink_log_info(&mavlink_log_pub, "[inav] init ref: %.7f, %.7f, %8.4f", (double)lat, (double)lon, (double)alt);
						}
					}

					if (ref_inited) {
						/* project GPS lat lon to plane */
						// 将GPS的经纬度投射到地面上
						float gps_proj[2]; //gps_proj变量来保存数据
						map_projection_project(&ref, lat, lon, &(gps_proj[0]), &(gps_proj[1]));// 将当前经纬度信息通过一系列计算后保存到proj中
																							   // 计算相对起点的绝对位置，该函数中使用到了地球半径

						/* reset position estimate when GPS becomes good */
						// GPS恢复时重置位置估计 使用gps_proj为位置，之前的GPS速度量为速度。
						if (reset_est) {
							x_est[0] = gps_proj[0];
							x_est[1] = gps.vel_n_m_s;
							y_est[0] = gps_proj[1];
							y_est[1] = gps.vel_e_m_s;
						}

						/* calculate index of estimated values in buffer */
						//计算估计值在缓冲区的位置（0-25，<0则加25）
						int est_i = buf_ptr - 1 - min(EST_BUF_SIZE - 1, max(0, (int)(params.delay_gps * 1000000.0f / PUB_INTERVAL)));
						//delay_gps=INAV_DELAY_GPS=0.2  对GPS延迟的补偿

						if (est_i < 0) {
							est_i += EST_BUF_SIZE;
						}

						/* calculate correction for position */
						// 计算位置校正
						corr_gps[0][0] = gps_proj[0] - est_buf[est_i][0][0];
						corr_gps[1][0] = gps_proj[1] - est_buf[est_i][1][0];
						corr_gps[2][0] = local_pos.ref_alt - alt - est_buf[est_i][2][0];

						/* calculate correction for velocity */
						// 计算速度校正
						if (gps.vel_ned_valid) {
							corr_gps[0][1] = gps.vel_n_m_s - est_buf[est_i][0][1];
							corr_gps[1][1] = gps.vel_e_m_s - est_buf[est_i][1][1];
							corr_gps[2][1] = gps.vel_d_m_s - est_buf[est_i][2][1];

						} else {//GPS的ned速度无效，则将速度校准归0
							corr_gps[0][1] = 0.0f;
							corr_gps[1][1] = 0.0f;
							corr_gps[2][1] = 0.0f;
						}

						/* save rotation matrix at this moment */
						// 保存此刻的旋转矩阵
						//将缓冲中的旋转矩阵R_buf赋值给用于GPS校正的旋转矩阵R_gps
						memcpy(R_gps, R_buf[est_i], sizeof(R_gps));

						w_gps_xy = min_eph_epv / fmaxf(min_eph_epv, gps.eph);
						w_gps_z = min_eph_epv / fmaxf(min_eph_epv, gps.epv);
					}

				} else {
					/* no GPS lock */
					// GPS无效
					memset(corr_gps, 0, sizeof(corr_gps)); 
					ref_init_start = 0;
				}

				gps_updates++;
			}
		}

		/* check for timeout on FLOW topic */
		// 检查光流话题是否超时
		if ((flow_valid || lidar_valid) && t > (flow_time + flow_topic_timeout)) {
		// 循环开始时间大于光流时间加上话题超时时间的和
			flow_valid = false; //禁用光流
			warnx("FLOW timeout");
			mavlink_log_info(&mavlink_log_pub, "[inav] FLOW timeout");
		}

		/* check for timeout on GPS topic */
		if (gps_valid && (t > (gps.timestamp + gps_topic_timeout))) {
			gps_valid = false;
			warnx("GPS timeout");
			mavlink_log_info(&mavlink_log_pub, "[inav] GPS timeout");
		}

		/* check for timeout on vision topic */
		if (vision_valid && (t > (vision.timestamp + vision_topic_timeout))) {
			vision_valid = false;
			warnx("VISION timeout");
			mavlink_log_info(&mavlink_log_pub, "[inav] VISION timeout");
		}

		/* check for timeout on mocap topic */
		if (mocap_valid && (t > (mocap.timestamp + mocap_topic_timeout))) {
			mocap_valid = false;
			warnx("MOCAP timeout");
			mavlink_log_info(&mavlink_log_pub, "[inav] MOCAP timeout");
		}

		/* check for lidar measurement timeout */
		if (lidar_valid && (t > (lidar_time + lidar_timeout))) {
			lidar_valid = false;
			warnx("LIDAR timeout");
			mavlink_log_info(&mavlink_log_pub, "[inav] LIDAR timeout");
		}

		float dt = t_prev > 0 ? (t - t_prev) / 1000000.0f : 0.0f;
		dt = fmaxf(fminf(0.02, dt), 0.0002);		// constrain dt from 0.2ms to 20 ms 限制dt在0.2s-0.02s之间 频率50-5000Hz
		t_prev = t;

		/* increase EPH/EPV on each step */
		// 在每一步增加EPH/EPV  因为对于GPS，EPH/EPV过大的话，变invalid
		if (eph < 0.000001f) { //get case where eph is 0 -> would stay 0
			eph = 0.001;
		}

		if (eph < max_eph_epv) {
			eph *= 1.0f + dt;
		}

		if (epv < 0.000001f) { //get case where epv is 0 -> would stay 0
			epv = 0.001;
		}

		if (epv < max_eph_epv) {
			epv += 0.005f * dt;	// add 1m to EPV each 200s (baro drift) 每200秒EPV加1米(由于气压计漂移)
		}

		/* use GPS if it's valid and reference position initialized */
		bool use_gps_xy = ref_inited && gps_valid && params.w_xy_gps_p > MIN_VALID_W;
		bool use_gps_z = ref_inited && gps_valid && params.w_z_gps_p > MIN_VALID_W;
		/* use VISION if it's valid and has a valid weight parameter */
		bool use_vision_xy = vision_valid && params.w_xy_vision_p > MIN_VALID_W;
		bool use_vision_z = vision_valid && params.w_z_vision_p > MIN_VALID_W;
		/* use MOCAP if it's valid and has a valid weight parameter */
		bool use_mocap = mocap_valid && params.w_mocap_p > MIN_VALID_W && params.att_ext_hdg_m == mocap_heading; //check if external heading is mocap

		if (params.disable_mocap) { //disable mocap if fake gps is used 如果使用了假GPS定位则禁用视觉捕捉装置
			use_mocap = false;
		}

		/* use flow if it's valid and (accurate or no GPS available) */
		//  如果光流装置有效则使用光流(光流精确或者没有可用的GPS)
		bool use_flow = flow_valid && (flow_accurate || !use_gps_xy);

		/* use LIDAR if it's valid and lidar altitude estimation is enabled */
		// 如果LIDAR装置有效同时lidar高度估计使能则使用LIDAR
		use_lidar = lidar_valid && params.enable_lidar_alt_est;

		bool can_estimate_xy = (eph < max_eph_epv) || use_gps_xy || use_flow || use_vision_xy || use_mocap;

		bool dist_bottom_valid = (t < lidar_valid_time + lidar_valid_timeout);

		float w_xy_gps_p = params.w_xy_gps_p * w_gps_xy;
		float w_xy_gps_v = params.w_xy_gps_v * w_gps_xy;
		float w_z_gps_p = params.w_z_gps_p * w_gps_z;
		float w_z_gps_v = params.w_z_gps_v * w_gps_z;

		float w_xy_vision_p = params.w_xy_vision_p;
		float w_xy_vision_v = params.w_xy_vision_v;
		float w_z_vision_p = params.w_z_vision_p;

		float w_mocap_p = params.w_mocap_p;

		/* reduce GPS weight if optical flow is good */
		// 如果光流状态良好则降低GPS权重
		if (use_flow && flow_accurate) {
			w_xy_gps_p *= params.w_gps_flow;
			w_xy_gps_v *= params.w_gps_flow;
		}

	//根据使用工具的情况，更新加速度计和气压计的偏移补偿
		/* baro offset correction */
		// 气压计偏移校正
		if (use_gps_z) { //如果使用了GPS用于Z轴的估计，则使用GPS再次校正气压计的偏移与矫正系数
			float offs_corr = corr_gps[2][0] * w_z_gps_p * dt;
			baro_offset += offs_corr;
			corr_baro += offs_corr;
		}

		/* accelerometer bias correction for GPS (use buffered rotation matrix) */
		// 使用缓冲区的旋转矩阵来校正加速度计偏差用于GPS
		float accel_bias_corr[3] = { 0.0f, 0.0f, 0.0f };
		// 分别计算若使用了相应的外置传感器，Flow，Vision，Mocap
		//对加速度计偏差的校正量accel_bias_corr[]，然后用于acc_bias[]的计算

		if (use_gps_xy) {
			accel_bias_corr[0] -= corr_gps[0][0] * w_xy_gps_p * w_xy_gps_p;
			accel_bias_corr[0] -= corr_gps[0][1] * w_xy_gps_v;
			accel_bias_corr[1] -= corr_gps[1][0] * w_xy_gps_p * w_xy_gps_p;
			accel_bias_corr[1] -= corr_gps[1][1] * w_xy_gps_v;
		}

		if (use_gps_z) {
			accel_bias_corr[2] -= corr_gps[2][0] * w_z_gps_p * w_z_gps_p;
			accel_bias_corr[2] -= corr_gps[2][1] * w_z_gps_v;
		}

		/* transform error vector from NED frame to body frame */
		// 将误差向量从NED坐标系(地球坐标系)转换到机体坐标系
		/*
		 * 由于该补偿处于地理坐标系，需要旋转矩阵转入机体坐标系用于飞行器，
		 * 分别对每个轴的加速度偏移值进行补偿
		 */
		for (int i = 0; i < 3; i++) {
			float c = 0.0f;

			for (int j = 0; j < 3; j++) {
				c += R_gps[j][i] * accel_bias_corr[j];
			}

			if (PX4_ISFINITE(c)) {
				acc_bias[i] += c * params.w_acc_bias * dt;  //得到acc_bias[] 用于主循环中加速度计的测量数据校正
			}
		}

		/* accelerometer bias correction for VISION (use buffered rotation matrix) */
		//使用缓冲区的旋转矩阵来校正加速度计偏差用于VISION
		accel_bias_corr[0] = 0.0f;  //重新初始化，计算其他传感器的accel_bias_corr[ ]
		accel_bias_corr[1] = 0.0f;
		accel_bias_corr[2] = 0.0f;

		if (use_vision_xy) {
			accel_bias_corr[0] -= corr_vision[0][0] * w_xy_vision_p * w_xy_vision_p;
			accel_bias_corr[0] -= corr_vision[0][1] * w_xy_vision_v;
			accel_bias_corr[1] -= corr_vision[1][0] * w_xy_vision_p * w_xy_vision_p;
			accel_bias_corr[1] -= corr_vision[1][1] * w_xy_vision_v;
		}

		if (use_vision_z) {
			accel_bias_corr[2] -= corr_vision[2][0] * w_z_vision_p * w_z_vision_p;
		}

		/* accelerometer bias correction for MOCAP (use buffered rotation matrix) */
		//使用缓冲区的旋转矩阵来校正加速度计偏差用于MOCAP
		accel_bias_corr[0] = 0.0f;
		accel_bias_corr[1] = 0.0f;
		accel_bias_corr[2] = 0.0f;

		if (use_mocap) {
			accel_bias_corr[0] -= corr_mocap[0][0] * w_mocap_p * w_mocap_p;
			accel_bias_corr[1] -= corr_mocap[1][0] * w_mocap_p * w_mocap_p;
			accel_bias_corr[2] -= corr_mocap[2][0] * w_mocap_p * w_mocap_p;
		}

		/* transform error vector from NED frame to body frame */
		// 将误差向量从地球坐标系转移到机体坐标系
		for (int i = 0; i < 3; i++) {
			float c = 0.0f;

			for (int j = 0; j < 3; j++) {
				c += PX4_R(att.R, j, i) * accel_bias_corr[j];
			}

			if (PX4_ISFINITE(c)) {
				acc_bias[i] += c * params.w_acc_bias * dt;
			}
		}

		/* accelerometer bias correction for flow and baro (assume that there is no delay) */
		// 假定没有延迟，校正加速度偏差用于光流和气压计
		accel_bias_corr[0] = 0.0f;
		accel_bias_corr[1] = 0.0f;
		accel_bias_corr[2] = 0.0f;

		if (use_flow) { //用光流速度校正系数来计算xy方向上的accel_bias_corr[ ]
			accel_bias_corr[0] -= corr_flow[0] * params.w_xy_flow;
			accel_bias_corr[1] -= corr_flow[1] * params.w_xy_flow;
		}

		if (use_lidar) { ////用雷达校正系数来计算z方向上的accel_bias_corr[ ]
			accel_bias_corr[2] -= corr_lidar * params.w_z_lidar * params.w_z_lidar;
		} else { //气压计的作用就是计算气压计校正系数来对加速度偏移量进行校正
			accel_bias_corr[2] -= corr_baro * params.w_z_baro * params.w_z_baro; 
		}

		/* transform error vector from NED frame to body frame */
		// 将误差向量从地球坐标系转移到机体坐标系
		for (int i = 0; i < 3; i++) {
			float c = 0.0f;

			for (int j = 0; j < 3; j++) {
				c += PX4_R(att.R, j, i) * accel_bias_corr[j];
			}

			if (PX4_ISFINITE(c)) {
				acc_bias[i] += c * params.w_acc_bias * dt;
			}
		}

		/*
		 * 至此已经得到了校正的加速度值，将其用于速度与位置的估计
		 * 依旧是分别计算各个外置传感器对xyz三个方向上的估计与校正
		 */
		/* inertial filter prediction for altitude */
		// 高度的惯性滤波器预测
		inertial_filter_predict(dt, z_est, acc[2]);//用加速度的积分预测速度和位置 

		if (!(PX4_ISFINITE(z_est[0]) && PX4_ISFINITE(z_est[1]))) {
			write_debug_log("BAD ESTIMATE AFTER Z PREDICTION", dt, x_est, y_est, z_est, x_est_prev, y_est_prev, z_est_prev,
					acc, corr_gps, w_xy_gps_p, w_xy_gps_v, corr_mocap, w_mocap_p,
					corr_vision, w_xy_vision_p, w_z_vision_p, w_xy_vision_v);
			memcpy(z_est, z_est_prev, sizeof(z_est));
		}

		/* inertial filter correction for altitude */
		// 高度的惯性滤波器校正
		if (use_lidar) {
			inertial_filter_correct(corr_lidar, dt, z_est, 0, params.w_z_lidar); 

		} else {
			inertial_filter_correct(corr_baro, dt, z_est, 0, params.w_z_baro);
		}

		if (use_gps_z) {
			epv = fminf(epv, gps.epv);

			inertial_filter_correct(corr_gps[2][0], dt, z_est, 0, w_z_gps_p);
			inertial_filter_correct(corr_gps[2][1], dt, z_est, 1, w_z_gps_v);
		}

		if (use_vision_z) {
			epv = fminf(epv, epv_vision);
			inertial_filter_correct(corr_vision[2][0], dt, z_est, 0, w_z_vision_p);
		}

		if (use_mocap) {
			epv = fminf(epv, epv_mocap);
			inertial_filter_correct(corr_mocap[2][0], dt, z_est, 0, w_mocap_p);
		}

		if (!(PX4_ISFINITE(z_est[0]) && PX4_ISFINITE(z_est[1]))) {
			write_debug_log("BAD ESTIMATE AFTER Z CORRECTION", dt, x_est, y_est, z_est, x_est_prev, y_est_prev, z_est_prev,
					acc, corr_gps, w_xy_gps_p, w_xy_gps_v, corr_mocap, w_mocap_p,
					corr_vision, w_xy_vision_p, w_z_vision_p, w_xy_vision_v);
			memcpy(z_est, z_est_prev, sizeof(z_est));  // 恢复上一刻的估计值
			memset(corr_gps, 0, sizeof(corr_gps)); //各校正系数复位
			memset(corr_vision, 0, sizeof(corr_vision));
			memset(corr_mocap, 0, sizeof(corr_mocap));
			corr_baro = 0;

		} else {
			memcpy(z_est_prev, z_est, sizeof(z_est));
		}

		if (can_estimate_xy) {
			/* inertial filter prediction for position */
			// 位置预测的惯性滤波器
			inertial_filter_predict(dt, x_est, acc[0]);
			inertial_filter_predict(dt, y_est, acc[1]);

			if (!(PX4_ISFINITE(x_est[0]) && PX4_ISFINITE(x_est[1]) && PX4_ISFINITE(y_est[0]) && PX4_ISFINITE(y_est[1]))) {
				write_debug_log("BAD ESTIMATE AFTER PREDICTION", dt, x_est, y_est, z_est, x_est_prev, y_est_prev, z_est_prev,
						acc, corr_gps, w_xy_gps_p, w_xy_gps_v, corr_mocap, w_mocap_p,
						corr_vision, w_xy_vision_p, w_z_vision_p, w_xy_vision_v);
				memcpy(x_est, x_est_prev, sizeof(x_est));
				memcpy(y_est, y_est_prev, sizeof(y_est));
			}

			/* inertial filter correction for position */
			// 位置校正的惯性滤波器
			if (use_flow) {
				eph = fminf(eph, eph_flow);

				inertial_filter_correct(corr_flow[0], dt, x_est, 1, params.w_xy_flow * w_flow);
				inertial_filter_correct(corr_flow[1], dt, y_est, 1, params.w_xy_flow * w_flow);
			}

			if (use_gps_xy) {
				eph = fminf(eph, gps.eph);

				inertial_filter_correct(corr_gps[0][0], dt, x_est, 0, w_xy_gps_p);
				inertial_filter_correct(corr_gps[1][0], dt, y_est, 0, w_xy_gps_p);

				if (gps.vel_ned_valid && t < gps.timestamp + gps_topic_timeout) {
					inertial_filter_correct(corr_gps[0][1], dt, x_est, 1, w_xy_gps_v);
					inertial_filter_correct(corr_gps[1][1], dt, y_est, 1, w_xy_gps_v);
				}
			}

			if (use_vision_xy) {
				eph = fminf(eph, eph_vision);

				inertial_filter_correct(corr_vision[0][0], dt, x_est, 0, w_xy_vision_p);
				inertial_filter_correct(corr_vision[1][0], dt, y_est, 0, w_xy_vision_p);

				if (w_xy_vision_v > MIN_VALID_W) {
					inertial_filter_correct(corr_vision[0][1], dt, x_est, 1, w_xy_vision_v);
					inertial_filter_correct(corr_vision[1][1], dt, y_est, 1, w_xy_vision_v);
				}
			}

			if (use_mocap) {
				eph = fminf(eph, eph_mocap);

				inertial_filter_correct(corr_mocap[0][0], dt, x_est, 0, w_mocap_p);
				inertial_filter_correct(corr_mocap[1][0], dt, y_est, 0, w_mocap_p);
			}

			if (!(PX4_ISFINITE(x_est[0]) && PX4_ISFINITE(x_est[1]) && PX4_ISFINITE(y_est[0]) && PX4_ISFINITE(y_est[1]))) {
				//xy方向上的速度位置估计超过限定值，报错并恢复
				write_debug_log("BAD ESTIMATE AFTER CORRECTION", dt, x_est, y_est, z_est, x_est_prev, y_est_prev, z_est_prev,
						acc, corr_gps, w_xy_gps_p, w_xy_gps_v, corr_mocap, w_mocap_p,
						corr_vision, w_xy_vision_p, w_z_vision_p, w_xy_vision_v);
				memcpy(x_est, x_est_prev, sizeof(x_est));
				memcpy(y_est, y_est_prev, sizeof(y_est));
				memset(corr_gps, 0, sizeof(corr_gps));
				memset(corr_vision, 0, sizeof(corr_vision));
				memset(corr_mocap, 0, sizeof(corr_mocap));
				memset(corr_flow, 0, sizeof(corr_flow));

			} else {//未超限，将x,y当前的估计值作为下一刻的参数
				memcpy(x_est_prev, x_est, sizeof(x_est));
				memcpy(y_est_prev, y_est, sizeof(y_est));
			}

		} else {
			/* gradually reset xy velocity estimates */
			// 未使能can_estimate_xy
			// 将平面内XY轴向的速度量清零，逐次减去自身单位时间量
			inertial_filter_correct(-x_est[1], dt, x_est, 1, params.w_xy_res_v);
			inertial_filter_correct(-y_est[1], dt, y_est, 1, params.w_xy_res_v);
		}

		/* run terrain estimator */
		// 进行地势估计
		terrain_estimator.predict(dt, &att, &sensor, &lidar);
		terrain_estimator.measurement_update(hrt_absolute_time(), &gps, &lidar, &att);

		if (inav_verbose_mode) {
			/* print updates rate */
			// 满足更新时间条件就输出更新频率
			//加速度计，气压计，GPS，高度，光流，视觉和动作捕捉装置
			if (t > updates_counter_start + updates_counter_len) {  //更新计数开始1s后，打印更新频率
				float updates_dt = (t - updates_counter_start) * 0.000001f;  //时间转换成秒s，当前时间与更新开始的时间差
				warnx(
					"updates rate: accelerometer = %.1f/s, baro = %.1f/s, gps = %.1f/s, attitude = %.1f/s, flow = %.1f/s, vision = %.1f/s, mocap = %.1f/s",
					(double)(accel_updates / updates_dt),
					(double)(baro_updates / updates_dt),
					(double)(gps_updates / updates_dt),
					(double)(attitude_updates / updates_dt),
					(double)(flow_updates / updates_dt),
					(double)(vision_updates / updates_dt),
					(double)(mocap_updates / updates_dt)); //更新频率=更新次数/时间差
				updates_counter_start = t; //将当前时间作为下一循环的开始时间
				accel_updates = 0; // 更新次数全置0，重新开始循环
				baro_updates = 0;
				gps_updates = 0;
				attitude_updates = 0;
				flow_updates = 0;
				vision_updates = 0;
				mocap_updates = 0;
			}
		}

		//接下来以100Hz频率更新数据（保存位置状态数据和旋转矩阵数据到缓冲区）
		if (t > pub_last + PUB_INTERVAL) {
			pub_last = t;

			/* push current estimate to buffer */
			// 将当前XYZ的位置速度估计推送到缓冲区
			est_buf[buf_ptr][0][0] = x_est[0]; // xyz的速度位置估计保存到缓冲区
			est_buf[buf_ptr][0][1] = x_est[1]; // 用于计算GPS校正系数corr_gps[][]
			est_buf[buf_ptr][1][0] = y_est[0]; // GPS校正系数进而用于气压计校正系数
			est_buf[buf_ptr][1][1] = y_est[1]; // corr_baro，accel_bias_corr,xyz的速度位置预测校正
			est_buf[buf_ptr][2][0] = z_est[0];
			est_buf[buf_ptr][2][1] = z_est[1];

			/* push current rotation matrix to buffer */
			//  将当前旋转矩阵推送到缓冲区
			memcpy(R_buf[buf_ptr], att.R, sizeof(att.R));

			buf_ptr++;

			if (buf_ptr >= EST_BUF_SIZE) {
				buf_ptr = 0;
			}

			//更新并发布位置数据
			//（两个主题vehicle_local_position和vehicle_global_position）
			/* publish local position */
			local_pos.xy_valid = can_estimate_xy;
			local_pos.v_xy_valid = can_estimate_xy;
			local_pos.xy_global = local_pos.xy_valid && use_gps_xy;
			local_pos.z_global = local_pos.z_valid && use_gps_z;
			local_pos.x = x_est[0];
			local_pos.vx = x_est[1];
			local_pos.y = y_est[0];
			local_pos.vy = y_est[1];
			local_pos.z = z_est[0];
			local_pos.vz = z_est[1];
			local_pos.yaw = att.yaw;
			local_pos.dist_bottom_valid = dist_bottom_valid;
			local_pos.eph = eph;
			local_pos.epv = epv;

			if (local_pos.dist_bottom_valid) {
				local_pos.dist_bottom = dist_ground;
				local_pos.dist_bottom_rate = - z_est[1];
			}

			local_pos.timestamp = t;

			orb_publish(ORB_ID(vehicle_local_position), vehicle_local_position_pub, &local_pos);

			if (local_pos.xy_global && local_pos.z_global) {
				/* publish global position */
				global_pos.timestamp = t;
				global_pos.time_utc_usec = gps.time_utc_usec;

				double est_lat, est_lon;
				map_projection_reproject(&ref, local_pos.x, local_pos.y, &est_lat, &est_lon); //反映射，返回角度

				global_pos.lat = est_lat;
				global_pos.lon = est_lon;
				global_pos.alt = local_pos.ref_alt - local_pos.z;

				global_pos.vel_n = local_pos.vx;
				global_pos.vel_e = local_pos.vy;
				global_pos.vel_d = local_pos.vz;

				global_pos.yaw = local_pos.yaw;

				global_pos.eph = eph;
				global_pos.epv = epv;

				if (terrain_estimator.is_valid()) {
					global_pos.terrain_alt = global_pos.alt - terrain_estimator.get_distance_to_ground();
					global_pos.terrain_alt_valid = true;

				} else {
					global_pos.terrain_alt_valid = false;
				}

				global_pos.pressure_alt = sensor.baro_alt_meter;

				if (vehicle_global_position_pub == NULL) {
					vehicle_global_position_pub = orb_advertise(ORB_ID(vehicle_global_position), &global_pos);

				} else {
					orb_publish(ORB_ID(vehicle_global_position), vehicle_global_position_pub, &global_pos);
				}
			}
		}
	}

	warnx("stopped");
	mavlink_log_info(&mavlink_log_pub, "[inav] stopped");
	thread_running = false;
	return 0;
}
