/****************************************************************************
 *
 *   Copyright (C) 2015 PX4 Development Team. All rights reserved.
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
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <systemlib/err.h>
#include <uavcan/uavcan.hpp>
#include <uavcan/helpers/heap_based_pool_allocator.hpp>

// TODO: Entire UAVCAN application should be moved into a namespace later; this is the first step.
namespace uavcan_node
{

struct AllocatorSynchronizer {
	const ::irqstate_t state = ::enter_critical_section();
	~AllocatorSynchronizer() { ::leave_critical_section(state); }
};

struct Allocator : public uavcan::HeapBasedPoolAllocator<uavcan::MemPoolBlockSize, AllocatorSynchronizer> {
	static constexpr unsigned CapacitySoftLimit = 250;
	static constexpr unsigned CapacityHardLimit = 500;

	Allocator() :
		uavcan::HeapBasedPoolAllocator<uavcan::MemPoolBlockSize, AllocatorSynchronizer>(CapacitySoftLimit, CapacityHardLimit)
	{ }

	~Allocator()
	{
		if (getNumAllocatedBlocks() > 0) {
			PX4_ERR("UAVCAN LEAKS MEMORY: %u BLOCKS (%u BYTES) LOST",
				getNumAllocatedBlocks(), getNumAllocatedBlocks() * uavcan::MemPoolBlockSize);
		}
	}
};

}
