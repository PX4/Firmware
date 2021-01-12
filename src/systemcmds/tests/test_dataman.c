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
 * @file test_dataman.c
 * Tests for the data manager.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/posix.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include <drivers/drv_board_led.h>
#include <drivers/drv_hrt.h>
#include <semaphore.h>

#include "tests_main.h"

#include "dataman/dataman.h"

static px4_sem_t *sems;
static bool *task_returned_error;
int test_dataman(int argc, char *argv[]);

#define NUM_MISSIONS_TEST 50

#define DM_MAX_DATA_SIZE sizeof(struct mission_s)

static int
task_main(int argc, char *argv[])
{
	char buffer[DM_MAX_DATA_SIZE];

	PX4_INFO("Starting dataman test task %s", argv[2]);
	/* try to read an invalid item */
	int my_id = atoi(argv[2]);

	/* try to read an invalid item */
	if (dm_read(DM_KEY_NUM_KEYS, 0, buffer, sizeof(buffer)) >= 0) {
		PX4_ERR("%d read an invalid item failed", my_id);
		goto fail;
	}

	/* try to read an invalid index */
	if (dm_read(DM_KEY_SAFE_POINTS, DM_KEY_SAFE_POINTS_MAX, buffer, sizeof(buffer)) >= 0) {
		PX4_ERR("%d read an invalid index failed", my_id);
		goto fail;
	}

	srand(hrt_absolute_time() ^ my_id);
	unsigned hit = 0, miss = 0;
	hrt_abstime wstart = hrt_absolute_time();

	for (unsigned i = 0; i < NUM_MISSIONS_TEST; i++) {
		memset(buffer, my_id, sizeof(buffer));
		buffer[1] = i;
		unsigned hash = i ^ my_id;
		unsigned len = (hash % (DM_MAX_DATA_SIZE / 2)) + 2;

		int ret = dm_write(DM_KEY_WAYPOINTS_OFFBOARD_1, hash, DM_PERSIST_IN_FLIGHT_RESET, buffer, len);
		//PX4_INFO("ret: %d", ret);

		if (ret != len) {
			PX4_WARN("task %d: write failed, index %d, length %d", my_id, hash, len);
			goto fail;
		}

		if (i % (NUM_MISSIONS_TEST / 10) == 0) {
			PX4_INFO("task %d: %.0f%%", my_id, (double)i * 100.0f / NUM_MISSIONS_TEST);
		}

		px4_usleep(rand() & ((64 * 1024) - 1));
	}

	hrt_abstime rstart = hrt_absolute_time();
	hrt_abstime wend = rstart;

	for (unsigned i = 0; i < NUM_MISSIONS_TEST; i++) {
		unsigned hash = i ^ my_id;
		unsigned len2;
		unsigned len = (hash % (DM_MAX_DATA_SIZE / 2)) + 2;

		if ((len2 = dm_read(DM_KEY_WAYPOINTS_OFFBOARD_1, hash, buffer, sizeof(buffer))) < 2) {
			PX4_WARN("task %d: read failed length test, index %d", my_id, hash);
			goto fail;
		}

		if (buffer[0] == my_id) {
			hit++;

			if (len2 != len) {
				PX4_WARN("task %d: read failed length test, index %d, wanted %d, got %d", my_id, hash, len, len2);
				goto fail;
			}

			if (buffer[1] != i) {
				PX4_WARN("task %d: data verification failed, index %d, wanted %d, got %d", my_id, hash, my_id, buffer[1]);
				goto fail;
			}

		} else {
			miss++;
		}
	}

	hrt_abstime rend = hrt_absolute_time();
	PX4_INFO("task %d pass, hit %d, miss %d, io time read %" PRIu64 "ms. write %" PRIu64 "ms.",
		 my_id, hit, miss, (rend - rstart) / NUM_MISSIONS_TEST / 1000, (wend - wstart) / NUM_MISSIONS_TEST / 1000);
	px4_sem_post(sems + my_id);
	return 0;

fail:
	PX4_ERR("test_dataman FAILED: task %d, buffer %02x %02x %02x %02x %02x %02x",
		my_id, buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5]);
	px4_sem_post(sems + my_id);
	task_returned_error[my_id] = true;
	return -1;
}

int test_dataman(int argc, char *argv[])
{
	int i = 0;
	unsigned num_tasks = 4;
	char buffer[DM_MAX_DATA_SIZE];

	if (argc > 1) {
		num_tasks = atoi(argv[1]);
	}

	sems = (px4_sem_t *)malloc(num_tasks * sizeof(px4_sem_t));
	task_returned_error = (bool *)calloc(num_tasks, sizeof(bool));
	PX4_INFO("Running %d tasks", num_tasks);

	for (i = 0; i < num_tasks; i++) {
		int task;

		char a[16];
		snprintf(a, 16, "%d", i);

		char *av[] = {"tests_dataman", a, NULL};

		px4_sem_init(sems + i, 1, 0);
		/* sems use case is a signal */
		px4_sem_setprotocol(sems, SEM_PRIO_NONE);

		/* start the task */
		if ((task = px4_task_spawn_cmd("dataman", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT, 2048, task_main, av)) <= 0) {
			PX4_ERR("task start failed");
		}
	}

	for (i = 0; i < num_tasks; i++) {
		px4_sem_wait(sems + i);
		px4_sem_destroy(sems + i);
	}

	free(sems);

	bool got_error = false;

	for (i = 0; i < num_tasks; i++) {
		if (task_returned_error[i]) {
			got_error = true;
			break;
		}
	}

	free(task_returned_error);

	if (got_error) {
		return -1;
	}

	dm_restart(DM_INIT_REASON_IN_FLIGHT);

	for (i = 0; i < NUM_MISSIONS_TEST; i++) {
		if (dm_read(DM_KEY_WAYPOINTS_OFFBOARD_1, i, buffer, sizeof(buffer)) != 0) {
			break;
		}
	}

	if (i >= NUM_MISSIONS_TEST) {
		PX4_ERR("Restart in-flight failed");
		return -1;

	}

	dm_restart(DM_INIT_REASON_POWER_ON);

	for (i = 0; i < NUM_MISSIONS_TEST; i++) {
		if (dm_read(DM_KEY_WAYPOINTS_OFFBOARD_1, i, buffer, sizeof(buffer)) != 0) {
			PX4_ERR("Restart power-on failed");
			return -1;
		}
	}

	return 0;
}
