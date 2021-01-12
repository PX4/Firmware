/****************************************************************************
 *
 *   Copyright (c) 2013-2014 PX4 Development Team. All rights reserved.
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
 * @file mtd.c
 *
 * mtd service and utility app.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/spi.h>
#include <px4_platform_common/px4_mtd.h>
#include <px4_platform_common/getopt.h>

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
#include <sys/mount.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/nxffs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/drivers/drivers.h>

#include <arch/board/board.h>

#include "systemlib/px4_macros.h"
#include <parameters/param.h>

#include <board_config.h>

extern "C" __EXPORT int mtd_main(int argc, char *argv[]);

static int mtd_status(void)
{
	int ret = 0;
	bool running = false;
	unsigned int num_instances;

	const mtd_instance_s *instances = px4_mtd_get_instances(&num_instances);

	if (instances) {
		for (unsigned int i = 0; i < num_instances; ++i) {
			if (instances[i].mtd_dev) {

				unsigned long blocksize;
				unsigned long erasesize;
				unsigned long neraseblocks;
				unsigned int  blkpererase;
				unsigned int  nblocks;
				unsigned int  partsize;

				ret = px4_mtd_get_geometry(&instances[i], &blocksize, &erasesize, &neraseblocks, &blkpererase, &nblocks, &partsize);

				if (ret == 0) {

					PX4_INFO("Flash Geometry of instance %i:", i);

					printf("  blocksize:      %lu\n", blocksize);
					printf("  erasesize:      %lu\n", erasesize);
					printf("  neraseblocks:   %lu\n", neraseblocks);
					printf("  No. partitions: %u\n", instances[i].n_partitions_current);


					unsigned int  totalnblocks = 0;
					unsigned int  totalpartsize = 0;

					for (unsigned int p = 0; p < instances[i].n_partitions_current; p++) {
						FAR struct mtd_geometry_s geo;
						ret = instances[i].part_dev[p]->ioctl(instances[i].part_dev[p], MTDIOC_GEOMETRY, (unsigned long)((uintptr_t)&geo));
						printf("    partition: %u:\n", p);
						printf("     name:   %s\n", instances[i].partition_names[p]);
						printf("     blocks: %u (%u bytes)\n", geo.neraseblocks, erasesize * geo.neraseblocks);
						totalnblocks += geo.neraseblocks;
						totalpartsize += erasesize * geo.neraseblocks;
					}

					printf("  Device size: %u Blocks (%u bytes)\n", totalnblocks, totalpartsize);
					printf("  TOTAL SIZE: %u KiB\n", totalpartsize  / 1024);
				}

				running |= true;
			}
		}
	}

	if (!running) {
		PX4_INFO("MTD driver not started");
		return 1;
	}

	return ret;
}

static void	print_usage(void)
{
#if !defined(CONSTRAINED_FLASH)

	PRINT_MODULE_DESCRIPTION("Utility to mount and test partitions (based on FRAM/EEPROM storage as defined by the board)");

	PRINT_MODULE_USAGE_NAME("mtd", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print status information");
	PRINT_MODULE_USAGE_COMMAND_DESCR("readtest", "Perform read test");
	PRINT_MODULE_USAGE_COMMAND_DESCR("rwtest", "Perform read-write test");

	PRINT_MODULE_USAGE_COMMAND_DESCR("erase", "Erase partition(s)");
	PRINT_MODULE_USAGE_PARAM_COMMENT("The commands 'readtest' and 'rwtest' have an optional instance index:");
	PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, 1, "storage index (if the board has multiple storages)", true);

	PRINT_MODULE_USAGE_PARAM_COMMENT("The commands 'readtest', 'rwtest' and 'erase' have an optional parameter:");
	PRINT_MODULE_USAGE_ARG("<partition_name1> [<partition_name2> ...]",
			       "Partition names (eg. /fs/mtd_params), use system default if not provided", true);
#endif
}

int mtd_erase(mtd_instance_s &instance)
{
	uint8_t v[64];
	memset(v, 0xFF, sizeof(v));

	for (uint8_t i = 0; i < instance.n_partitions_current; i++) {

		uint32_t count = 0;
		printf("Erasing %s\n", instance.partition_names[i]);
		int fd = open(instance.partition_names[i], O_WRONLY);

		if (fd == -1) {
			PX4_ERR("Failed to open partition");
			return 1;
		}

		while (write(fd, v, sizeof(v)) == sizeof(v)) {
			count += sizeof(v);
		}

		printf("Erased %lu bytes\n", (unsigned long)count);
		close(fd);
	}

	return 0;
}

#if !defined(CONSTRAINED_FLASH)

/*
  readtest is useful during startup to validate the device is
  responding on the bus. It relies on the driver returning an error on
  bad reads (the ramtron driver does return an error)
 */
int mtd_readtest(const mtd_instance_s &instance)
{
	uint8_t v[128];

	for (uint8_t i = 0; i < instance.n_partitions_current; i++) {
		ssize_t count = 0;

		ssize_t expected_size = px4_mtd_get_partition_size(&instance, instance.partition_names[i]);

		if (expected_size == 0) {
			PX4_ERR("Failed partition size is 0");
			return 1;
		}

		printf("reading %s expecting %u bytes\n", instance.partition_names[i], expected_size);
		int fd = open(instance.partition_names[i], O_RDONLY);

		if (fd == -1) {
			PX4_ERR("Failed to open partition");
			return 1;
		}

		while (read(fd, v, sizeof(v)) == sizeof(v)) {
			count += sizeof(v);
		}

		if (count != expected_size) {
			PX4_ERR("Failed to read partition - got %u/%u bytes", count, expected_size);
			return 1;
		}

		close(fd);
	}

	printf("readtest OK\n");
	return 0;
}

/*
  rwtest is useful during startup to validate the device is
  responding on the bus for both reads and writes. It reads data in
  blocks and writes the data back, then reads it again, failing if the
  data isn't the same
 */
int mtd_rwtest(const mtd_instance_s &instance)
{
	uint8_t v[128], v2[128];

	for (uint8_t i = 0; i < instance.n_partitions_current; i++) {
		ssize_t count = 0;
		off_t offset = 0;

		ssize_t expected_size = px4_mtd_get_partition_size(&instance, instance.partition_names[i]);

		if (expected_size == 0) {
			PX4_ERR("Failed partition size is 0");
			return 1;
		}

		printf("rwtest %s testing %u bytes\n", instance.partition_names[i], expected_size);
		int fd = open(instance.partition_names[i], O_RDWR);

		if (fd == -1) {
			PX4_ERR("Failed to open partition");
			return 1;
		}

		while (read(fd, v, sizeof(v)) == sizeof(v)) {
			count += sizeof(v);

			if (lseek(fd, offset, SEEK_SET) != offset) {
				PX4_ERR("seek failed");
				return 1;
			}

			if (write(fd, v, sizeof(v)) != sizeof(v)) {
				PX4_ERR("write failed");
				return 1;
			}

			if (lseek(fd, offset, SEEK_SET) != offset) {
				PX4_ERR("seek failed");
				return 1;
			}

			if (read(fd, v2, sizeof(v2)) != sizeof(v2)) {
				PX4_ERR("read failed");
				return 1;
			}

			if (memcmp(v, v2, sizeof(v2)) != 0) {
				PX4_ERR("memcmp failed");
				return 1;
			}

			offset += sizeof(v);
		}

		if (count != expected_size) {
			PX4_ERR("Failed to read partition - got %u/%u bytes", count, expected_size);
			return 1;
		}

		close(fd);
	}

	printf("rwtest OK\n");
	return 0;
}
#endif

int mtd_main(int argc, char *argv[])
{
	int myoptind = 1;
	const char *myoptarg = NULL;
	int ch;
	int instance = 0;

	while ((ch = px4_getopt(argc, argv, "i:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'i':
			instance = atoi(myoptarg);
			break;

		default:
			print_usage();
			return -1;
			break;
		}
	}

	if (myoptind < argc) {
		unsigned int num_instances;
		mtd_instance_s *instances = px4_mtd_get_instances(&num_instances);

		if (instances == nullptr) {
			PX4_ERR("Driver not running");
			return -1;
		}

		if (instance < 0 || (unsigned) instance >= num_instances) {
			PX4_ERR("invalid instance");
			return -1;
		}

#if !defined(CONSTRAINED_FLASH)

		if (!strcmp(argv[myoptind], "readtest")) {
			return mtd_readtest(instances[instance]);
		}

		if (!strcmp(argv[myoptind], "rwtest")) {
			return mtd_rwtest(instances[instance]);
		}

#endif

		if (!strcmp(argv[myoptind], "status")) {
			return mtd_status();
		}

		if (!strcmp(argv[myoptind],  "erase")) {
			return mtd_erase(instances[instance]);
		}
	}

	print_usage();
	return 1;
}
