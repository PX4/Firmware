/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file dataman.cpp
 * DATAMANAGER driver.
 *
 * @author Jean Cyr
 * @author Lorenz Meier
 * @author Julian Oes
 * @author Thomas Gubler
 * @author David Sidrane
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/getopt.h>
#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>

#include "dataman.h"

__BEGIN_DECLS
__EXPORT int dataman_main(int argc, char *argv[]);
__END_DECLS

static constexpr int TASK_STACK_SIZE = 1220;

/* Private File based Operations */
static ssize_t _file_write(dm_item_t item, unsigned index, dm_persitence_t persistence, const void *buf,
			   size_t count);
static ssize_t _file_read(dm_item_t item, unsigned index, void *buf, size_t count);
static int  _file_clear(dm_item_t item);
static int  _file_restart(dm_reset_reason reason);
static int _file_initialize(unsigned max_offset);
static void _file_shutdown();

/* Private Ram based Operations */
static ssize_t _ram_write(dm_item_t item, unsigned index, dm_persitence_t persistence, const void *buf,
			  size_t count);
static ssize_t _ram_read(dm_item_t item, unsigned index, void *buf, size_t count);
static int  _ram_clear(dm_item_t item);
static int  _ram_restart(dm_reset_reason reason);
static int _ram_initialize(unsigned max_offset);
static void _ram_shutdown();

typedef struct dm_operations_t {
	ssize_t (*write)(dm_item_t item, unsigned index, dm_persitence_t persistence, const void *buf, size_t count);
	ssize_t (*read)(dm_item_t item, unsigned index, void *buf, size_t count);
	int (*clear)(dm_item_t item);
	int (*restart)(dm_reset_reason reason);
	int (*initialize)(unsigned max_offset);
	void (*shutdown)();
	int (*wait)(px4_sem_t *sem);
} dm_operations_t;

static constexpr dm_operations_t dm_file_operations = {
	.write   = _file_write,
	.read    = _file_read,
	.clear   = _file_clear,
	.restart = _file_restart,
	.initialize = _file_initialize,
	.shutdown = _file_shutdown,
	.wait = px4_sem_wait,
};

static constexpr dm_operations_t dm_ram_operations = {
	.write   = _ram_write,
	.read    = _ram_read,
	.clear   = _ram_clear,
	.restart = _ram_restart,
	.initialize = _ram_initialize,
	.shutdown = _ram_shutdown,
	.wait = px4_sem_wait,
};

static const dm_operations_t *g_dm_ops;

static struct {
	union {
		struct {
			int fd;
		} file;
		struct {
			uint8_t *data;
			uint8_t *data_end;
		} ram;
	};
	bool running;
} dm_operations_data;

/** Types of function calls supported by the worker task */
typedef enum {
	dm_write_func = 0,
	dm_read_func,
	dm_clear_func,
	dm_restart_func,
	dm_number_of_funcs
} dm_function_t;

/** Work task work item */
typedef struct {
	sq_entry_t link;	/**< list linkage */
	px4_sem_t wait_sem;
	unsigned char first;
	unsigned char func;
	ssize_t result;
	union {
		struct {
			dm_item_t item;
			unsigned index;
			dm_persitence_t persistence;
			const void *buf;
			size_t count;
		} write_params;
		struct {
			dm_item_t item;
			unsigned index;
			void *buf;
			size_t count;
		} read_params;
		struct {
			dm_item_t item;
		} clear_params;
		struct {
			dm_reset_reason reason;
		} restart_params;
	};
} work_q_item_t;

const size_t k_work_item_allocation_chunk_size = 8;

/* Usage statistics */
static unsigned g_func_counts[dm_number_of_funcs];

/* table of maximum number of instances for each item type */
static const unsigned g_per_item_max_index[DM_KEY_NUM_KEYS] = {
	DM_KEY_SAFE_POINTS_MAX,
	DM_KEY_FENCE_POINTS_MAX,
	DM_KEY_WAYPOINTS_OFFBOARD_0_MAX,
	DM_KEY_WAYPOINTS_OFFBOARD_1_MAX,
	DM_KEY_WAYPOINTS_ONBOARD_MAX,
	DM_KEY_MISSION_STATE_MAX,
	DM_KEY_COMPAT_MAX
};

#define DM_SECTOR_HDR_SIZE 4	/* data manager per item header overhead */

/* Table of the len of each item type */
static constexpr size_t g_per_item_size[DM_KEY_NUM_KEYS] = {
	sizeof(struct mission_safe_point_s) + DM_SECTOR_HDR_SIZE,
	sizeof(struct mission_fence_point_s) + DM_SECTOR_HDR_SIZE,
	sizeof(struct mission_item_s) + DM_SECTOR_HDR_SIZE,
	sizeof(struct mission_item_s) + DM_SECTOR_HDR_SIZE,
	sizeof(struct mission_item_s) + DM_SECTOR_HDR_SIZE,
	sizeof(struct mission_s) + DM_SECTOR_HDR_SIZE,
	sizeof(struct dataman_compat_s) + DM_SECTOR_HDR_SIZE
};

/* Table of offset for index 0 of each item type */
static unsigned int g_key_offsets[DM_KEY_NUM_KEYS];

/* Item type lock mutexes */
static px4_sem_t *g_item_locks[DM_KEY_NUM_KEYS];
static px4_sem_t g_sys_state_mutex_mission;
static px4_sem_t g_sys_state_mutex_fence;

static perf_counter_t _dm_read_perf{nullptr};
static perf_counter_t _dm_write_perf{nullptr};

/* The data manager store file handle and file name */
static const char *default_device_path = PX4_STORAGEDIR "/dataman";
static char *k_data_manager_device_path = nullptr;

static enum {
	BACKEND_NONE = 0,
	BACKEND_FILE,
	BACKEND_RAM,
	BACKEND_LAST
} backend = BACKEND_NONE;

/* The data manager work queues */

typedef struct {
	sq_queue_t q;		/* Nuttx queue */
	px4_sem_t mutex;		/* Mutual exclusion on work queue adds and deletes */
	unsigned size;		/* Current size of queue */
	unsigned max_size;	/* Maximum queue size reached */
} work_q_t;

static work_q_t g_free_q;	/* queue of free work items. So that we don't always need to call malloc and free*/
static work_q_t g_work_q;	/* pending work items. To be consumed by worker thread */

static px4_sem_t g_work_queued_sema;	/* To notify worker thread a work item has been queued */
static px4_sem_t g_init_sema;

static bool g_task_should_exit;	/**< if true, dataman task should exit */

static void init_q(work_q_t *q)
{
	sq_init(&(q->q));		/* Initialize the NuttX queue structure */
	px4_sem_init(&(q->mutex), 1, 1);	/* Queue is initially unlocked */
	q->size = q->max_size = 0;	/* Queue is initially empty */
}

static inline void
destroy_q(work_q_t *q)
{
	px4_sem_destroy(&(q->mutex));	/* Destroy the queue lock */
}

static inline void
lock_queue(work_q_t *q)
{
	px4_sem_wait(&(q->mutex));	/* Acquire the queue lock */
}

static inline void
unlock_queue(work_q_t *q)
{
	px4_sem_post(&(q->mutex));	/* Release the queue lock */
}

static work_q_item_t *
create_work_item()
{
	work_q_item_t *item;

	/* Try to reuse item from free item queue */
	lock_queue(&g_free_q);

	if ((item = (work_q_item_t *)sq_remfirst(&(g_free_q.q)))) {
		g_free_q.size--;
	}

	unlock_queue(&g_free_q);

	/* If we there weren't any free items then obtain memory for a new ones */
	if (item == nullptr) {
		item = (work_q_item_t *)malloc(k_work_item_allocation_chunk_size * sizeof(work_q_item_t));

		if (item) {
			item->first = 1;
			lock_queue(&g_free_q);

			for (size_t i = 1; i < k_work_item_allocation_chunk_size; i++) {
				(item + i)->first = 0;
				sq_addfirst(&(item + i)->link, &(g_free_q.q));
			}

			/* Update the queue size and potentially the maximum queue size */
			g_free_q.size += k_work_item_allocation_chunk_size - 1;

			if (g_free_q.size > g_free_q.max_size) {
				g_free_q.max_size = g_free_q.size;
			}

			unlock_queue(&g_free_q);
		}
	}

	/* If we got one then lock the item*/
	if (item) {
		px4_sem_init(&item->wait_sem, 1, 0);        /* Caller will wait on this... initially locked */

		/* item->wait_sem use case is a signal */

		px4_sem_setprotocol(&item->wait_sem, SEM_PRIO_NONE);
	}

	/* return the item pointer, or nullptr if all failed */
	return item;
}

/* Work queue management functions */

static inline void
destroy_work_item(work_q_item_t *item)
{
	px4_sem_destroy(&item->wait_sem); /* Destroy the item lock */
	/* Return the item to the free item queue for later reuse */
	lock_queue(&g_free_q);
	sq_addfirst(&item->link, &(g_free_q.q));

	/* Update the queue size and potentially the maximum queue size */
	if (++g_free_q.size > g_free_q.max_size) {
		g_free_q.max_size = g_free_q.size;
	}

	unlock_queue(&g_free_q);
}

static inline work_q_item_t *
dequeue_work_item()
{
	work_q_item_t *work;

	/* retrieve the 1st item on the work queue */
	lock_queue(&g_work_q);

	if ((work = (work_q_item_t *)sq_remfirst(&g_work_q.q))) {
		g_work_q.size--;
	}

	unlock_queue(&g_work_q);
	return work;
}

static int
enqueue_work_item_and_wait_for_result(work_q_item_t *item)
{
	/* put the work item at the end of the work queue */
	lock_queue(&g_work_q);
	sq_addlast(&item->link, &(g_work_q.q));

	/* Adjust the queue size and potentially the maximum queue size */
	if (++g_work_q.size > g_work_q.max_size) {
		g_work_q.max_size = g_work_q.size;
	}

	unlock_queue(&g_work_q);

	/* tell the work thread that work is available */
	px4_sem_post(&g_work_queued_sema);

	/* wait for the result */
	px4_sem_wait(&item->wait_sem);

	int result = item->result;

	destroy_work_item(item);

	return result;
}

static bool is_running()
{
	return dm_operations_data.running;
}

/* Calculate the offset in file of specific item */
static int
calculate_offset(dm_item_t item, unsigned index)
{

	/* Make sure the item type is valid */
	if (item >= DM_KEY_NUM_KEYS) {
		return -1;
	}

	/* Make sure the index for this item type is valid */
	if (index >= g_per_item_max_index[item]) {
		return -1;
	}

	/* Calculate and return the item index based on type and index */
	return g_key_offsets[item] + (index * g_per_item_size[item]);
}

/* Each data item is stored as follows
 *
 * byte 0: Length of user data item
 * byte 1: Persistence of this data item
 * byte 2: Unused (for future use)
 * byte 3: Unused (for future use)
 * byte DM_SECTOR_HDR_SIZE... : data item value
 *
 * The total size must not exceed g_per_item_max_index[item]
 */

/* write to the data manager RAM buffer  */
static ssize_t _ram_write(dm_item_t item, unsigned index, dm_persitence_t persistence, const void *buf,
			  size_t count)
{

	/* Get the offset for this item */
	int offset = calculate_offset(item, index);

	/* If item type or index out of range, return error */
	if (offset < 0) {
		return -1;
	}

	/* Make sure caller has not given us more data than we can handle */
	if (count > (g_per_item_size[item] - DM_SECTOR_HDR_SIZE)) {
		return -E2BIG;
	}

	uint8_t *buffer = &dm_operations_data.ram.data[offset];

	if (buffer > dm_operations_data.ram.data_end) {
		return -1;
	}

	/* Write out the data, prefixed with length and persistence level */
	buffer[0] = count;
	buffer[1] = persistence;
	buffer[2] = 0;
	buffer[3] = 0;

	if (count > 0) {
		memcpy(buffer + DM_SECTOR_HDR_SIZE, buf, count);
	}

	/* All is well... return the number of user data written */
	return count;
}

/* write to the data manager file */
static ssize_t
_file_write(dm_item_t item, unsigned index, dm_persitence_t persistence, const void *buf, size_t count)
{
	unsigned char buffer[g_per_item_size[item]];

	/* Get the offset for this item */
	const int offset = calculate_offset(item, index);

	/* If item type or index out of range, return error */
	if (offset < 0) {
		return -1;
	}

	/* Make sure caller has not given us more data than we can handle */
	if (count > (g_per_item_size[item] - DM_SECTOR_HDR_SIZE)) {
		return -E2BIG;
	}

	/* Write out the data, prefixed with length and persistence level */
	buffer[0] = count;
	buffer[1] = persistence;
	buffer[2] = 0;
	buffer[3] = 0;

	if (count > 0) {
		memcpy(buffer + DM_SECTOR_HDR_SIZE, buf, count);
	}

	count += DM_SECTOR_HDR_SIZE;

	if (lseek(dm_operations_data.file.fd, offset, SEEK_SET) != offset) {
		return -1;
	}

	if ((write(dm_operations_data.file.fd, buffer, count)) != (ssize_t)count) {
		return -1;
	}

	/* Make sure data is written to physical media */
	fsync(dm_operations_data.file.fd);

	/* All is well... return the number of user data written */
	return count - DM_SECTOR_HDR_SIZE;
}

/* Retrieve from the data manager RAM buffer*/
static ssize_t _ram_read(dm_item_t item, unsigned index, void *buf, size_t count)
{
	/* Get the offset for this item */
	int offset = calculate_offset(item, index);

	/* If item type or index out of range, return error */
	if (offset < 0) {
		return -1;
	}

	/* Make sure the caller hasn't asked for more data than we can handle */
	if (count > (g_per_item_size[item] - DM_SECTOR_HDR_SIZE)) {
		return -E2BIG;
	}

	/* Read the prefix and data */

	uint8_t *buffer = &dm_operations_data.ram.data[offset];

	if (buffer > dm_operations_data.ram.data_end) {
		return -1;
	}

	/* See if we got data */
	if (buffer[0] > 0) {
		/* We got more than requested!!! */
		if (buffer[0] > count) {
			return -1;
		}

		/* Looks good, copy it to the caller's buffer */
		memcpy(buf, buffer + DM_SECTOR_HDR_SIZE, buffer[0]);
	}

	/* Return the number of bytes of caller data read */
	return buffer[0];
}

/* Retrieve from the data manager file */
static ssize_t
_file_read(dm_item_t item, unsigned index, void *buf, size_t count)
{
	if (item >= DM_KEY_NUM_KEYS) {
		return -1;
	}

	unsigned char buffer[g_per_item_size[item]];

	/* Get the offset for this item */
	int offset = calculate_offset(item, index);

	/* If item type or index out of range, return error */
	if (offset < 0) {
		return -1;
	}

	/* Make sure the caller hasn't asked for more data than we can handle */
	if (count > (g_per_item_size[item] - DM_SECTOR_HDR_SIZE)) {
		return -E2BIG;
	}

	/* Read the prefix and data */
	int len = -1;

	if (lseek(dm_operations_data.file.fd, offset, SEEK_SET) == offset) {
		len = read(dm_operations_data.file.fd, buffer, count + DM_SECTOR_HDR_SIZE);
	}

	/* Check for read error */
	if (len < 0) {
		return -errno;
	}

	/* A zero length entry is a empty entry */
	if (len == 0) {
		buffer[0] = 0;
	}

	/* See if we got data */
	if (buffer[0] > 0) {
		/* We got more than requested!!! */
		if (buffer[0] > count) {
			return -1;
		}

		/* Looks good, copy it to the caller's buffer */
		memcpy(buf, buffer + DM_SECTOR_HDR_SIZE, buffer[0]);
	}

	/* Return the number of bytes of caller data read */
	return buffer[0];
}

static int  _ram_clear(dm_item_t item)
{
	int i;
	int result = 0;

	/* Get the offset of 1st item of this type */
	int offset = calculate_offset(item, 0);

	/* Check for item type out of range */
	if (offset < 0) {
		return -1;
	}

	/* Clear all items of this type */
	for (i = 0; (unsigned)i < g_per_item_max_index[item]; i++) {
		uint8_t *buf = &dm_operations_data.ram.data[offset];

		if (buf > dm_operations_data.ram.data_end) {
			result = -1;
			break;
		}

		buf[0] = 0;
		offset += g_per_item_size[item];
	}

	return result;
}

static int
_file_clear(dm_item_t item)
{
	int i, result = 0;

	/* Get the offset of 1st item of this type */
	int offset = calculate_offset(item, 0);

	/* Check for item type out of range */
	if (offset < 0) {
		return -1;
	}

	/* Clear all items of this type */
	for (i = 0; (unsigned)i < g_per_item_max_index[item]; i++) {
		char buf[1];

		if (lseek(dm_operations_data.file.fd, offset, SEEK_SET) != offset) {
			result = -1;
			break;
		}

		/* Avoid SD flash wear by only doing writes where necessary */
		if (read(dm_operations_data.file.fd, buf, 1) < 1) {
			break;
		}

		/* If item has length greater than 0 it needs to be overwritten */
		if (buf[0]) {
			if (lseek(dm_operations_data.file.fd, offset, SEEK_SET) != offset) {
				result = -1;
				break;
			}

			buf[0] = 0;

			if (write(dm_operations_data.file.fd, buf, 1) != 1) {
				result = -1;
				break;
			}
		}

		offset += g_per_item_size[item];
	}

	/* Make sure data is actually written to physical media */
	fsync(dm_operations_data.file.fd);
	return result;
}

/* Tell the data manager about the type of the last reset */
static int  _ram_restart(dm_reset_reason reason)
{
	uint8_t *buffer = dm_operations_data.ram.data;

	/* We need to scan the entire file and invalidate and data that should not persist after the last reset */

	/* Loop through all of the data segments and delete those that are not persistent */

	for (int item = (int)DM_KEY_SAFE_POINTS; item < (int)DM_KEY_NUM_KEYS; item++) {
		for (unsigned i = 0; i < g_per_item_max_index[item]; i++) {
			/* check if segment contains data */
			if (buffer[0]) {
				bool clear_entry = false;

				/* Whether data gets deleted depends on reset type and data segment's persistence setting */
				if (reason == DM_INIT_REASON_POWER_ON) {
					if (buffer[1] > DM_PERSIST_POWER_ON_RESET) {
						clear_entry = true;
					}

				} else {
					if (buffer[1] > DM_PERSIST_IN_FLIGHT_RESET) {
						clear_entry = true;
					}
				}

				/* Set segment to unused if data does not persist */
				if (clear_entry) {
					buffer[0] = 0;
				}
			}

			buffer += g_per_item_size[item];
		}
	}

	return 0;
}

static int
_file_restart(dm_reset_reason reason)
{
	int offset = 0;
	int result = 0;
	/* We need to scan the entire file and invalidate and data that should not persist after the last reset */

	/* Loop through all of the data segments and delete those that are not persistent */
	for (int item = (int)DM_KEY_SAFE_POINTS; item < (int)DM_KEY_NUM_KEYS; item++) {
		for (unsigned i = 0; i < g_per_item_max_index[item]; i++) {
			/* Get data segment at current offset */
			if (lseek(dm_operations_data.file.fd, offset, SEEK_SET) != offset) {
				result = -1;
				item = DM_KEY_NUM_KEYS;
				break;
			}

			uint8_t buffer[2];
			ssize_t len = read(dm_operations_data.file.fd, buffer, sizeof(buffer));

			if (len != sizeof(buffer)) {
				result = -1;
				item = DM_KEY_NUM_KEYS;
				break;
			}

			/* check if segment contains data */
			if (buffer[0]) {
				bool clear_entry = false;

				/* Whether data gets deleted depends on reset type and data segment's persistence setting */
				if (reason == DM_INIT_REASON_POWER_ON) {
					if (buffer[1] > DM_PERSIST_POWER_ON_RESET) {
						clear_entry = true;
					}

				} else {
					if (buffer[1] > DM_PERSIST_IN_FLIGHT_RESET) {
						clear_entry = true;
					}
				}

				/* Set segment to unused if data does not persist */
				if (clear_entry) {
					if (lseek(dm_operations_data.file.fd, offset, SEEK_SET) != offset) {
						result = -1;
						item = DM_KEY_NUM_KEYS;
						break;
					}

					buffer[0] = 0;

					len = write(dm_operations_data.file.fd, buffer, 1);

					if (len != 1) {
						result = -1;
						item = DM_KEY_NUM_KEYS;
						break;
					}
				}
			}

			offset += g_per_item_size[item];
		}
	}

	fsync(dm_operations_data.file.fd);

	/* tell the caller how it went */
	return result;
}

static int
_file_initialize(unsigned max_offset)
{
	/* See if the data manage file exists and is a multiple of the sector size */
	dm_operations_data.file.fd = open(k_data_manager_device_path, O_RDONLY | O_BINARY);

	if (dm_operations_data.file.fd >= 0) {
		// Read the mission state and check the hash
		struct dataman_compat_s compat_state;
		int ret = g_dm_ops->read(DM_KEY_COMPAT, 0, &compat_state, sizeof(compat_state));

		bool incompat = true;

		if (ret == sizeof(compat_state)) {
			if (compat_state.key == DM_COMPAT_KEY) {
				incompat = false;
			}
		}

		close(dm_operations_data.file.fd);

		if (incompat) {
			unlink(k_data_manager_device_path);
		}
	}

	/* Open or create the data manager file */
	dm_operations_data.file.fd = open(k_data_manager_device_path, O_RDWR | O_CREAT | O_BINARY, PX4_O_MODE_666);

	if (dm_operations_data.file.fd < 0) {
		PX4_WARN("Could not open data manager file %s", k_data_manager_device_path);
		px4_sem_post(&g_init_sema); /* Don't want to hang startup */
		return -1;
	}

	if ((unsigned)lseek(dm_operations_data.file.fd, max_offset, SEEK_SET) != max_offset) {
		close(dm_operations_data.file.fd);
		PX4_WARN("Could not seek data manager file %s", k_data_manager_device_path);
		px4_sem_post(&g_init_sema); /* Don't want to hang startup */
		return -1;
	}

	/* Write current compat info */
	struct dataman_compat_s compat_state;
	compat_state.key = DM_COMPAT_KEY;
	int ret = g_dm_ops->write(DM_KEY_COMPAT, 0, DM_PERSIST_POWER_ON_RESET, &compat_state, sizeof(compat_state));

	if (ret != sizeof(compat_state)) {
		PX4_ERR("Failed writing compat: %d", ret);
	}

	fsync(dm_operations_data.file.fd);
	dm_operations_data.running = true;

	return 0;
}

static int
_ram_initialize(unsigned max_offset)
{
	/* In memory */
	dm_operations_data.ram.data = (uint8_t *)malloc(max_offset);

	if (dm_operations_data.ram.data == nullptr) {
		PX4_WARN("Could not allocate %d bytes of memory", max_offset);
		px4_sem_post(&g_init_sema); /* Don't want to hang startup */
		return -1;
	}

	memset(dm_operations_data.ram.data, 0, max_offset);
	dm_operations_data.ram.data_end = &dm_operations_data.ram.data[max_offset - 1];
	dm_operations_data.running = true;

	return 0;
}

static void
_file_shutdown()
{
	close(dm_operations_data.file.fd);
	dm_operations_data.running = false;
}

static void
_ram_shutdown()
{
	free(dm_operations_data.ram.data);
	dm_operations_data.running = false;
}

/** Write to the data manager file */
__EXPORT ssize_t
dm_write(dm_item_t item, unsigned index, dm_persitence_t persistence, const void *buf, size_t count)
{
	work_q_item_t *work;

	/* Make sure data manager has been started and is not shutting down */
	if (!is_running() || g_task_should_exit) {
		return -1;
	}

	perf_begin(_dm_write_perf);

	/* get a work item and queue up a write request */
	if ((work = create_work_item()) == nullptr) {
		perf_end(_dm_write_perf);
		return -1;
	}

	work->func = dm_write_func;
	work->write_params.item = item;
	work->write_params.index = index;
	work->write_params.persistence = persistence;
	work->write_params.buf = buf;
	work->write_params.count = count;

	/* Enqueue the item on the work queue and wait for the worker thread to complete processing it */
	ssize_t ret = (ssize_t)enqueue_work_item_and_wait_for_result(work);
	perf_end(_dm_write_perf);
	return ret;
}

/** Retrieve from the data manager file */
__EXPORT ssize_t
dm_read(dm_item_t item, unsigned index, void *buf, size_t count)
{
	work_q_item_t *work;

	/* Make sure data manager has been started and is not shutting down */
	if (!is_running() || g_task_should_exit) {
		return -1;
	}

	perf_begin(_dm_read_perf);

	/* get a work item and queue up a read request */
	if ((work = create_work_item()) == nullptr) {
		perf_end(_dm_read_perf);
		return -1;
	}

	work->func = dm_read_func;
	work->read_params.item = item;
	work->read_params.index = index;
	work->read_params.buf = buf;
	work->read_params.count = count;

	/* Enqueue the item on the work queue and wait for the worker thread to complete processing it */
	ssize_t ret = (ssize_t)enqueue_work_item_and_wait_for_result(work);
	perf_end(_dm_read_perf);
	return ret;
}

/** Clear a data Item */
__EXPORT int
dm_clear(dm_item_t item)
{
	work_q_item_t *work;

	/* Make sure data manager has been started and is not shutting down */
	if (!is_running() || g_task_should_exit) {
		return -1;
	}

	/* get a work item and queue up a clear request */
	if ((work = create_work_item()) == nullptr) {
		return -1;
	}

	work->func = dm_clear_func;
	work->clear_params.item = item;

	/* Enqueue the item on the work queue and wait for the worker thread to complete processing it */
	return enqueue_work_item_and_wait_for_result(work);
}

__EXPORT int
dm_lock(dm_item_t item)
{
	/* Make sure data manager has been started and is not shutting down */
	if (!is_running() || g_task_should_exit) {
		errno = EINVAL;
		return -1;
	}

	if (item >= DM_KEY_NUM_KEYS) {
		errno = EINVAL;
		return -1;
	}

	if (g_item_locks[item]) {
		return px4_sem_wait(g_item_locks[item]);
	}

	errno = EINVAL;
	return -1;
}

__EXPORT int
dm_trylock(dm_item_t item)
{
	/* Make sure data manager has been started and is not shutting down */
	if (!is_running() || g_task_should_exit) {
		errno = EINVAL;
		return -1;
	}

	if (item >= DM_KEY_NUM_KEYS) {
		errno = EINVAL;
		return -1;
	}

	if (g_item_locks[item]) {
		return px4_sem_trywait(g_item_locks[item]);
	}

	errno = EINVAL;
	return -1;
}

/** Unlock a data Item */
__EXPORT void
dm_unlock(dm_item_t item)
{
	/* Make sure data manager has been started and is not shutting down */
	if (!is_running() || g_task_should_exit) {
		return;
	}

	if (item >= DM_KEY_NUM_KEYS) {
		return;
	}

	if (g_item_locks[item]) {
		px4_sem_post(g_item_locks[item]);
	}
}

/** Tell the data manager about the type of the last reset */
__EXPORT int
dm_restart(dm_reset_reason reason)
{
	work_q_item_t *work;

	/* Make sure data manager has been started and is not shutting down */
	if (!is_running() || g_task_should_exit) {
		return -1;
	}

	/* get a work item and queue up a restart request */
	if ((work = create_work_item()) == nullptr) {
		return -1;
	}

	work->func = dm_restart_func;
	work->restart_params.reason = reason;

	/* Enqueue the item on the work queue and wait for the worker thread to complete processing it */
	return enqueue_work_item_and_wait_for_result(work);
}

static int
task_main(int argc, char *argv[])
{
	/* Dataman can use disk or RAM */
	switch (backend) {
	case BACKEND_FILE:
		g_dm_ops = &dm_file_operations;
		break;

	case BACKEND_RAM:
		g_dm_ops = &dm_ram_operations;
		break;

	default:
		PX4_WARN("No valid backend set.");
		return -1;
	}

	work_q_item_t *work;

	/* Initialize global variables */
	g_key_offsets[0] = 0;

	for (int i = 0; i < ((int)DM_KEY_NUM_KEYS - 1); i++) {
		g_key_offsets[i + 1] = g_key_offsets[i] + (g_per_item_max_index[i] * g_per_item_size[i]);
	}

	unsigned max_offset = g_key_offsets[DM_KEY_NUM_KEYS - 1] + (g_per_item_max_index[DM_KEY_NUM_KEYS - 1] *
			      g_per_item_size[DM_KEY_NUM_KEYS - 1]);

	for (unsigned i = 0; i < dm_number_of_funcs; i++) {
		g_func_counts[i] = 0;
	}

	/* Initialize the item type locks, for now only DM_KEY_MISSION_STATE & DM_KEY_FENCE_POINTS supports locking */
	px4_sem_init(&g_sys_state_mutex_mission, 1, 1); /* Initially unlocked */
	px4_sem_init(&g_sys_state_mutex_fence, 1, 1); /* Initially unlocked */

	for (unsigned i = 0; i < DM_KEY_NUM_KEYS; i++) {
		g_item_locks[i] = nullptr;
	}

	g_item_locks[DM_KEY_MISSION_STATE] = &g_sys_state_mutex_mission;
	g_item_locks[DM_KEY_FENCE_POINTS] = &g_sys_state_mutex_fence;

	g_task_should_exit = false;

	init_q(&g_work_q);
	init_q(&g_free_q);

	px4_sem_init(&g_work_queued_sema, 1, 0);

	/* g_work_queued_sema use case is a signal */

	px4_sem_setprotocol(&g_work_queued_sema, SEM_PRIO_NONE);

	_dm_read_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": read");
	_dm_write_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": write");

	/* see if we need to erase any items based on restart type */
	int sys_restart_val;

	const char *restart_type_str = "Unknown restart";

	int ret = g_dm_ops->initialize(max_offset);

	if (ret) {
		g_task_should_exit = true;
		goto end;
	}

	if (param_get(param_find("SYS_RESTART_TYPE"), &sys_restart_val) == OK) {
		if (sys_restart_val == DM_INIT_REASON_POWER_ON) {
			restart_type_str = "Power on restart";
			g_dm_ops->restart(DM_INIT_REASON_POWER_ON);

		} else if (sys_restart_val == DM_INIT_REASON_IN_FLIGHT) {
			restart_type_str = "In flight restart";
			g_dm_ops->restart(DM_INIT_REASON_IN_FLIGHT);
		}
	}

	switch (backend) {
	case BACKEND_FILE:
		if (sys_restart_val != DM_INIT_REASON_POWER_ON) {
			PX4_INFO("%s, data manager file '%s' size is %d bytes",
				 restart_type_str, k_data_manager_device_path, max_offset);
		}

		break;

	case BACKEND_RAM:
		PX4_INFO("%s, data manager RAM size is %d bytes",
			 restart_type_str, max_offset);
		break;

	default:
		break;
	}

	/* Tell startup that the worker thread has completed its initialization */
	px4_sem_post(&g_init_sema);

	/* Start the endless loop, waiting for then processing work requests */
	while (true) {

		/* do we need to exit ??? */
		if (!g_task_should_exit) {
			/* wait for work */
			g_dm_ops->wait(&g_work_queued_sema);
		}

		/* Empty the work queue */
		while ((work = dequeue_work_item())) {

			/* handle each work item with the appropriate handler */
			switch (work->func) {
			case dm_write_func:
				g_func_counts[dm_write_func]++;
				work->result =
					g_dm_ops->write(work->write_params.item, work->write_params.index, work->write_params.persistence,
							work->write_params.buf,
							work->write_params.count);
				break;

			case dm_read_func:
				g_func_counts[dm_read_func]++;
				work->result =
					g_dm_ops->read(work->read_params.item, work->read_params.index, work->read_params.buf, work->read_params.count);
				break;

			case dm_clear_func:
				g_func_counts[dm_clear_func]++;
				work->result = g_dm_ops->clear(work->clear_params.item);
				break;

			case dm_restart_func:
				g_func_counts[dm_restart_func]++;
				work->result = g_dm_ops->restart(work->restart_params.reason);
				break;

			default: /* should never happen */
				work->result = -1;
				break;
			}

			/* Inform the caller that work is done */
			px4_sem_post(&work->wait_sem);
		}

		/* time to go???? */
		if (g_task_should_exit) {
			break;
		}
	}

	g_dm_ops->shutdown();

	/* The work queue is now empty, empty the free queue */
	for (;;) {
		if ((work = (work_q_item_t *)sq_remfirst(&(g_free_q.q))) == nullptr) {
			break;
		}

		if (work->first) {
			free(work);
		}
	}

end:
	backend = BACKEND_NONE;
	destroy_q(&g_work_q);
	destroy_q(&g_free_q);
	px4_sem_destroy(&g_work_queued_sema);
	px4_sem_destroy(&g_sys_state_mutex_mission);
	px4_sem_destroy(&g_sys_state_mutex_fence);

	perf_free(_dm_read_perf);
	_dm_read_perf = nullptr;

	perf_free(_dm_write_perf);
	_dm_write_perf = nullptr;

	return 0;
}

static int
start()
{
	int task;

	px4_sem_init(&g_init_sema, 1, 0);

	/* g_init_sema use case is a signal */

	px4_sem_setprotocol(&g_init_sema, SEM_PRIO_NONE);

	/* start the worker thread with low priority for disk IO */
	if ((task = px4_task_spawn_cmd("dataman", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT - 10, TASK_STACK_SIZE, task_main,
				       nullptr)) < 0) {
		px4_sem_destroy(&g_init_sema);
		PX4_ERR("task start failed");
		return -1;
	}

	/* wait for the thread to actually initialize */
	px4_sem_wait(&g_init_sema);
	px4_sem_destroy(&g_init_sema);

	return 0;
}

static void
status()
{
	/* display usage statistics */
	PX4_INFO("Writes   %d", g_func_counts[dm_write_func]);
	PX4_INFO("Reads    %d", g_func_counts[dm_read_func]);
	PX4_INFO("Clears   %d", g_func_counts[dm_clear_func]);
	PX4_INFO("Restarts %d", g_func_counts[dm_restart_func]);
	PX4_INFO("Max Q lengths work %d, free %d", g_work_q.max_size, g_free_q.max_size);
	perf_print_counter(_dm_read_perf);
	perf_print_counter(_dm_write_perf);
}

static void
stop()
{
	/* Tell the worker task to shut down */
	g_task_should_exit = true;
	px4_sem_post(&g_work_queued_sema);
}

static void
usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Module to provide persistent storage for the rest of the system in form of a simple database through a C API.
Multiple backends are supported:
- a file (eg. on the SD card)
- RAM (this is obviously not persistent)

It is used to store structured data of different types: mission waypoints, mission state and geofence polygons.
Each type has a specific type and a fixed maximum amount of storage items, so that fast random access is possible.

### Implementation
Reading and writing a single item is always atomic. If multiple items need to be read/modified atomically, there is
an additional lock per item type via `dm_lock`.

**DM_KEY_FENCE_POINTS** and **DM_KEY_SAFE_POINTS** items: the first data element is a `mission_stats_entry_s` struct,
which stores the number of items for these types. These items are always updated atomically in one transaction (from
the mavlink mission manager). During that time, navigator will try to acquire the geofence item lock, fail, and will not
check for geofence violations.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("dataman", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('f', nullptr, "<file>", "Storage file", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('r', "Use RAM backend (NOT persistent)", true);
	PRINT_MODULE_USAGE_PARAM_COMMENT("The options -f and -r are mutually exclusive. If nothing is specified, a file 'dataman' is used");

	PRINT_MODULE_USAGE_COMMAND_DESCR("poweronrestart", "Restart dataman (on power on)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("inflightrestart", "Restart dataman (in flight)");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

static int backend_check()
{
	if (backend != BACKEND_NONE) {
		PX4_WARN("-f and -r are mutually exclusive");
		usage();
		return -1;
	}

	return 0;
}

int
dataman_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
		return -1;
	}

	if (!strcmp(argv[1], "start")) {

		if (is_running()) {
			PX4_WARN("dataman already running");
			return -1;
		}

		int ch;
		int dmoptind = 1;
		const char *dmoptarg = nullptr;

		/* jump over start and look at options first */

		while ((ch = px4_getopt(argc, argv, "f:r", &dmoptind, &dmoptarg)) != EOF) {
			switch (ch) {
			case 'f':
				if (backend_check()) {
					return -1;
				}

				backend = BACKEND_FILE;
				k_data_manager_device_path = strdup(dmoptarg);
				PX4_INFO("dataman file set to: %s", k_data_manager_device_path);
				break;

			case 'r':
				if (backend_check()) {
					return -1;
				}

				backend = BACKEND_RAM;
				break;

			//no break
			default:
				usage();
				return -1;
			}
		}

		if (backend == BACKEND_NONE) {
			backend = BACKEND_FILE;
			k_data_manager_device_path = strdup(default_device_path);
		}

		start();

		if (!is_running()) {
			PX4_ERR("dataman start failed");
			free(k_data_manager_device_path);
			k_data_manager_device_path = nullptr;
			return -1;
		}

		return 0;
	}

	/* Worker thread should be running for all other commands */
	if (!is_running()) {
		PX4_WARN("dataman worker thread not running");
		usage();
		return -1;
	}

	if (!strcmp(argv[1], "stop")) {
		stop();
		free(k_data_manager_device_path);
		k_data_manager_device_path = nullptr;

	} else if (!strcmp(argv[1], "status")) {
		status();

	} else if (!strcmp(argv[1], "poweronrestart")) {
		dm_restart(DM_INIT_REASON_POWER_ON);

	} else if (!strcmp(argv[1], "inflightrestart")) {
		dm_restart(DM_INIT_REASON_IN_FLIGHT);

	} else {
		usage();
		return -1;
	}

	return 0;
}
