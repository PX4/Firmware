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

#include "UavcanNodeParamManager.hpp"

#include <px4_platform_common/defines.h>
#include <lib/parameters/param.h>

namespace uavcannode
{

void UavcanNodeParamManager::getParamNameByIndex(Index index, Name &out_name) const
{
	if (index < param_count_used()) {
		out_name = param_name(param_for_used_index(index));
	}
}

void UavcanNodeParamManager::assignParamValue(const Name &name, const Value &value)
{
	param_t param_handle = param_find(name.c_str());

	if (param_handle == PARAM_INVALID) {
		// Invalid parameter name
		return;
	}

	// Assign input value to parameter if types match
	param_type_t value_type = param_type(param_handle);

	if (value.is(uavcan::protocol::param::Value::Tag::integer_value) && (value_type == PARAM_TYPE_INT32)) {
		auto val = *value.as<uavcan::protocol::param::Value::Tag::integer_value>();

		if ((val <= INT32_MAX) || (val >= INT32_MIN)) {
			int32_t in_param = val;
			param_set(param_handle, &in_param);
		}

	} else if (value.is(uavcan::protocol::param::Value::Tag::real_value) && (value_type == PARAM_TYPE_FLOAT)) {
		// TODO: min/max value checking for float
		float in_param = *value.as<uavcan::protocol::param::Value::Tag::real_value>();
		param_set(param_handle, &in_param);
	}
}

void UavcanNodeParamManager::readParamValue(const Name &name, Value &out_value) const
{
	param_t param_handle = param_find(name.c_str());

	if (param_handle == PARAM_INVALID) {
		// Invalid parameter name
		return;
	}

	// Copy current parameter value to out_value
	param_type_t value_type = param_type(param_handle);

	if (value_type == PARAM_TYPE_INT32) {
		int32_t current_value;
		param_get(param_handle, &current_value);
		out_value.to<uavcan::protocol::param::Value::Tag::integer_value>() = current_value;

	} else if (value_type == PARAM_TYPE_FLOAT) {
		float current_value;
		param_get(param_handle, &current_value);
		out_value.to<uavcan::protocol::param::Value::Tag::real_value>() = current_value;
	}
}

void UavcanNodeParamManager::readParamDefaultMaxMin(const Name &name, Value &out_default,
		NumericValue &out_max, NumericValue &out_min) const
{
	// TODO: get actual default value (will require a new function in param.h)
	param_t param_handle = param_find(name.c_str());

	if (param_handle == PARAM_INVALID) {
		// Invalid parameter name
		return;
	}

	param_type_t value_type = param_type(param_handle);

	if (value_type == PARAM_TYPE_INT32) {
		int32_t default_value = 0;
		param_get_default_value(param_handle, &default_value);
		out_default.to<uavcan::protocol::param::Value::Tag::integer_value>() = default_value;

		out_min.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = INT32_MIN;
		out_max.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = INT32_MAX;

	} else if (value_type == PARAM_TYPE_FLOAT) {
		float default_value = 0;
		param_get_default_value(param_handle, &default_value);
		out_default.to<uavcan::protocol::param::Value::Tag::real_value>() = default_value;

		out_min.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = -FLT_MAX;
		out_max.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = FLT_MAX;
	}
}

int UavcanNodeParamManager::saveAllParams()
{
	// Nothing to do here assuming autosave is turned on
	return 0;
}

int UavcanNodeParamManager::eraseAllParams()
{
	param_reset_all();
	return 0;
}

} // namespace uavcannode
