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
#pragma once
#include <stdint.h>

__BEGIN_DECLS

// The data needed to interface with mtd device's

typedef struct {
	struct mtd_dev_s *mtd_dev;
	int              *partition_block_counts;
	int              *partition_types;
	const char       **partition_names;
	struct mtd_dev_s **part_dev;
	uint32_t         devid;
	unsigned         n_partitions_current;
} mtd_instance_s;

/*
  mtd operations
 */

/*
 * Get device an pinter to the array of mtd_instance_s of the system
 *  count - receives the number of instances pointed to by the pointer
 *  retunred.
 *
 *  returns: - A pointer to the mtd_instance_s of the system
 *            This can be  Null if there are no mtd instances.
 *
 */
__EXPORT mtd_instance_s *px4_mtd_get_instances(unsigned int *count);

/*
  Get device complete geometry or a device
 */


__EXPORT int  px4_mtd_get_geometry(const mtd_instance_s *instance, unsigned long *blocksize, unsigned long *erasesize,
				   unsigned long *neraseblocks, unsigned *blkpererase, unsigned *nblocks,
				   unsigned *partsize);
/*
  Get size of a parttion on an instance.
 */
__EXPORT ssize_t px4_mtd_get_partition_size(const mtd_instance_s *instance, const char *partname);

FAR struct mtd_dev_s *px4_at24c_initialize(FAR struct i2c_master_s *dev,
		uint8_t address);

__END_DECLS
