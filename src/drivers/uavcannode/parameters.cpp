/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file parameters.cpp
 *
 * Implements UAVCAN parameter system functions.
 *
 * @author Kenneth Thompson <ken@flyvoly.com>
 */

#include <string.h>

#include "uavcan_parameters.h"
#include "parameters.hpp"

static const uavcan_param_info_s *uavcan_param_info_base = (const uavcan_param_info_s *) &uavcan_parameters;
static const unsigned uavcan_param_info_count = uavcan_parameters.param_count;
static bool uavcan_param_initialized = false;

void uavcan_param_init()
{
	if (!uavcan_param_initialized) {
		for (unsigned i = 0; i < uavcan_parameters.param_count; ++i) {
			param_t handle = uavcan_param_get_handle_from_index(i);
			uavcan_param_map[i] = handle;
		}
		uavcan_param_initialized = true;
	}
}

const char *uavcan_param_name(unsigned index)
{
	if (index >= uavcan_param_info_count) {
		return nullptr;
	}

	return uavcan_param_info_base[index].name;
}

int uavcan_param_get_index(const char *name)
{
	int lhs = 0;
	int rhs = uavcan_param_info_count;
	int mid;

	// Find parameter using binary search
	while (lhs <= rhs) {
		mid = (rhs + lhs) / 2;
		const char *mid_name = uavcan_param_info_base[mid].name;
		int res = strcmp(name, mid_name);

		if (res == 0) {
			return mid;

		} else if (lhs == mid) {
			return -1;

		} else if (res < 0) {
			rhs = mid;

		} else {
			lhs = mid;
		}
	}

	return -1;
}

param_t uavcan_param_get_handle_from_index(unsigned index)
{
	if (index >= uavcan_param_info_count) {
		return PARAM_INVALID;
	}

	return param_find(uavcan_param_info_base[index].px4_param_name);
}

unsigned uavcan_param_count()
{
	return uavcan_param_info_count;
}

void uavcan_param_erase_all()
{
	for (unsigned int i = 0; i < uavcan_param_info_count; ++i) {
		param_reset(uavcan_param_get_handle_from_index(i));
	}
}
