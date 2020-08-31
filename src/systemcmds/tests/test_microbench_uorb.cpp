/****************************************************************************
 *
 *  Copyright (C) 2018-2019 PX4 Development Team. All rights reserved.
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
 * @file test_microbench_uorb.cpp
 * Tests for microbench uORB functionality.
 */

#include <unit_test.h>

#include <time.h>
#include <stdlib.h>
#include <unistd.h>

#include <drivers/drv_hrt.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/micro_hal.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_gyro_fifo.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

namespace MicroBenchORB
{

#ifdef __PX4_NUTTX
#include <nuttx/irq.h>
static irqstate_t flags;
#endif

void lock()
{
#ifdef __PX4_NUTTX
	flags = px4_enter_critical_section();
#endif
}

void unlock()
{
#ifdef __PX4_NUTTX
	px4_leave_critical_section(flags);
#endif
}

#define PERF(name, op, count) do { \
		px4_usleep(1000); \
		reset(); \
		perf_counter_t p = perf_alloc(PC_ELAPSED, name); \
		for (int i = 0; i < count; i++) { \
			px4_usleep(1); \
			lock(); \
			perf_begin(p); \
			op; \
			perf_end(p); \
			unlock(); \
			reset(); \
		} \
		perf_print_counter(p); \
		perf_free(p); \
	} while (0)

class MicroBenchORB : public UnitTest
{
public:
	virtual bool run_tests();

private:

	bool time_px4_uorb();
	bool time_px4_uorb_direct();

	void reset();

	vehicle_status_s status;
	vehicle_local_position_s lpos;
	sensor_gyro_s gyro;
	sensor_gyro_fifo_s gyro_fifo;
};

bool MicroBenchORB::run_tests()
{
	ut_run_test(time_px4_uorb);
	ut_run_test(time_px4_uorb_direct);

	return (_tests_failed == 0);
}

template<typename T>
T random(T min, T max)
{
	const T scale = rand() / (T) RAND_MAX; /* [0, 1.0] */
	return min + scale * (max - min);      /* [min, max] */
}

void MicroBenchORB::reset()
{
	srand(time(nullptr));

	// initialize with random data
	status.timestamp = rand();
	status.mission_failure = rand();

	lpos.timestamp = rand();
	lpos.dist_bottom_valid = rand();

	gyro.timestamp = rand();

	gyro_fifo.timestamp = rand();
}

ut_declare_test_c(test_microbench_uorb, MicroBenchORB)

bool MicroBenchORB::time_px4_uorb()
{
	int fd_status = orb_subscribe(ORB_ID(vehicle_status));
	int fd_lpos = orb_subscribe(ORB_ID(vehicle_local_position));
	int fd_gyro = orb_subscribe(ORB_ID(sensor_gyro));
	int fd_gyro_fifo = orb_subscribe(ORB_ID(sensor_gyro_fifo));

	int ret = 0;
	bool updated = false;
	uint64_t time = 0;

	PERF("orb_check vehicle_status", ret = orb_check(fd_status, &updated), 100);
	PERF("orb_copy vehicle_status", ret = orb_copy(ORB_ID(vehicle_status), fd_status, &status), 100);

	printf("\n");

	PERF("orb_check vehicle_local_position", ret = orb_check(fd_lpos, &updated), 100);
	PERF("orb_copy vehicle_local_position", ret = orb_copy(ORB_ID(vehicle_local_position), fd_lpos, &lpos), 100);

	printf("\n");

	PERF("orb_check sensor_gyro", ret = orb_check(fd_gyro, &updated), 100);
	PERF("orb_copy sensor_gyro", ret = orb_copy(ORB_ID(sensor_gyro), fd_gyro, &gyro), 100);

	printf("\n");

	PERF("orb_check sensor_gyro_fifo", ret = orb_check(fd_gyro_fifo, &updated), 100);
	PERF("orb_copy sensor_gyro_fifo", ret = orb_copy(ORB_ID(sensor_gyro_fifo), fd_gyro_fifo, &gyro_fifo), 100);

	printf("\n");

	PERF("orb_exists sensor_accel 0", ret = orb_exists(ORB_ID(sensor_accel), 0), 100);
	PERF("orb_exists sensor_accel 1", ret = orb_exists(ORB_ID(sensor_accel), 1), 100);
	PERF("orb_exists sensor_accel 2", ret = orb_exists(ORB_ID(sensor_accel), 2), 100);
	PERF("orb_exists sensor_accel 3", ret = orb_exists(ORB_ID(sensor_accel), 3), 100);
	PERF("orb_exists sensor_accel 4", ret = orb_exists(ORB_ID(sensor_accel), 4), 100);
	PERF("orb_exists sensor_accel 5", ret = orb_exists(ORB_ID(sensor_accel), 5), 100);
	PERF("orb_exists sensor_accel 6", ret = orb_exists(ORB_ID(sensor_accel), 6), 100);
	PERF("orb_exists sensor_accel 7", ret = orb_exists(ORB_ID(sensor_accel), 7), 100);
	PERF("orb_exists sensor_accel 8", ret = orb_exists(ORB_ID(sensor_accel), 8), 100);
	PERF("orb_exists sensor_accel 9", ret = orb_exists(ORB_ID(sensor_accel), 9), 100);
	PERF("orb_exists sensor_accel 10", ret = orb_exists(ORB_ID(sensor_accel), 10), 100);

	orb_unsubscribe(fd_status);
	orb_unsubscribe(fd_lpos);
	orb_unsubscribe(fd_gyro);
	orb_unsubscribe(fd_gyro_fifo);

	return true;
}

bool MicroBenchORB::time_px4_uorb_direct()
{
	bool ret = false;
	bool updated = false;
	uint64_t time = 0;

	uORB::Subscription vstatus{ORB_ID(vehicle_status)};
	PERF("uORB::Subscription orb_check vehicle_status", ret = vstatus.updated(), 100);
	PERF("uORB::Subscription orb_copy vehicle_status", ret = vstatus.copy(&status), 100);

	printf("\n");

	uORB::Subscription local_pos{ORB_ID(vehicle_local_position)};
	PERF("uORB::Subscription orb_check vehicle_local_position", ret = local_pos.updated(), 100);
	PERF("uORB::Subscription orb_copy vehicle_local_position", ret = local_pos.copy(&lpos), 100);

	{
		printf("\n");
		uORB::Subscription sens_gyro0{ORB_ID(sensor_gyro), 0};
		PERF("uORB::Subscription orb_check sensor_gyro:0", ret = sens_gyro0.updated(), 100);
		PERF("uORB::Subscription orb_copy sensor_gyro:0", ret = sens_gyro0.copy(&gyro), 100);
	}

	{
		printf("\n");
		uORB::Subscription sens_gyro1{ORB_ID(sensor_gyro), 1};
		PERF("uORB::Subscription orb_check sensor_gyro:1", ret = sens_gyro1.updated(), 100);
		PERF("uORB::Subscription orb_copy sensor_gyro:1", ret = sens_gyro1.copy(&gyro), 100);
	}

	{
		printf("\n");
		uORB::Subscription sens_gyro2{ORB_ID(sensor_gyro), 2};
		PERF("uORB::Subscription orb_check sensor_gyro:2", ret = sens_gyro2.updated(), 100);
		PERF("uORB::Subscription orb_copy sensor_gyro:2", ret = sens_gyro2.copy(&gyro), 100);
	}

	{
		printf("\n");
		uORB::Subscription sens_gyro3{ORB_ID(sensor_gyro), 3};
		PERF("uORB::Subscription orb_check sensor_gyro:3", ret = sens_gyro3.updated(), 100);
		PERF("uORB::Subscription orb_copy sensor_gyro:3", ret = sens_gyro3.copy(&gyro), 100);
	}

	{
		printf("\n");
		uORB::Subscription sens_gyro_fifo0{ORB_ID(sensor_gyro_fifo), 0};
		PERF("uORB::Subscription orb_check sensor_gyro_fifo:0", ret = sens_gyro_fifo0.updated(), 100);
		PERF("uORB::Subscription orb_copy sensor_gyro_fifo:0", ret = sens_gyro_fifo0.copy(&gyro_fifo), 100);
	}

	return true;
}

} // namespace MicroBenchORB
