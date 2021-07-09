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
 * @file ParamManager.hpp
 *
 * Defines basic functionality of UAVCAN parameter management class
 *
 * @author Jacob Crabill <jacob@flyvoly.com>
 */

#pragma once

#include <lib/parameters/param.h>
#include <px4_platform_common/defines.h>

#include <uavcan/_register/Name_1_0.h>
#include <uavcan/_register/Value_1_0.h>

static constexpr uint16_t CANARD_PORT_ID_UNSET = 65535U;
static constexpr uint16_t CANARD_PORT_ID_MAX   = 32767U;

static bool px4_param_to_uavcan_port_id(param_t &in, uavcan_register_Value_1_0 &out)
{
	if (param_type(in) == PARAM_TYPE_INT32) {
		int32_t out_val {};
		const int res = param_get(in, &out_val);

		if (res != PX4_OK) {
			// Parameter not found / internal error
			return false;
		}

		if (out_val >= 0 && out_val <= CANARD_PORT_ID_MAX) {
			out.natural16.value.elements[0] = (uint16_t)out_val;

		} else {
			// "Invalid" value -- set to "UNSET"
			out.natural16.value.elements[0] = CANARD_PORT_ID_UNSET;
		}

		out.natural16.value.count = 1;
		uavcan_register_Value_1_0_select_natural16_(&out);
		return true;
	}

	return false;
};

static bool uavcan_port_id_to_px4_param(const uavcan_register_Value_1_0 &in, param_t &out)
{
	if (uavcan_register_Value_1_0_is_natural16_(&in) && in.natural16.value.count == 1) {
		if (param_type(out) == PARAM_TYPE_INT32) {
			int32_t in_val = in.natural16.value.elements[0];
			param_set(out, &in_val);
			return true;
		}
	}

	return false;
};

using param_2_reg_t = bool(*)(param_t &in, uavcan_register_Value_1_0 &out);
using reg_2_param_t = bool(*)(const uavcan_register_Value_1_0 &in, param_t &out);


typedef struct {
	const char *uavcan_name;
	const char *px4_name;
	param_2_reg_t px4_param_to_register_value;
	reg_2_param_t register_value_to_px4_param;
	bool is_mutable {true};
	bool is_persistent {true};
} UavcanParamBinder;


class UavcanParamManager
{
public:

	bool GetParamByName(const char *param_name, uavcan_register_Value_1_0 &value);
	bool GetParamByName(const uavcan_register_Name_1_0 &name, uavcan_register_Value_1_0 &value);
	bool SetParamByName(const uavcan_register_Name_1_0 &name, const uavcan_register_Value_1_0 &value);

private:


	const UavcanParamBinder _uavcan_params[12] {
		{"uavcan.pub.esc.0.id",                "UCAN1_ESC_PUB",		    px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.pub.servo.0.id",              "UCAN1_SERVO_PUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.pub.gps.0.id",                "UCAN1_GPS_PUB",		    px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.pub.actuator_outputs.0.id",   "UCAN1_ACTR_PUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.esc.0.id",                "UCAN1_ESC0_SUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.gps.0.id",                "UCAN1_GPS0_SUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.gps.1.id",                "UCAN1_GPS1_SUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.energy_source.0.id",      "UCAN1_BMS_ES_SUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.battery_status.0.id",     "UCAN1_BMS_BS_SUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.battery_parameters.0.id", "UCAN1_BMS_BP_SUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.legacy_bms.0.id",         "UCAN1_LG_BMS_SUB",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		{"uavcan.sub.uorb.sensor_gps.0.id",    "UCAN1_UORB_GPS",		px4_param_to_uavcan_port_id, uavcan_port_id_to_px4_param},
		//{"uavcan.sub.bms.0.id",   "UCAN1_BMS0_SUB"}, //FIXME instancing
		//{"uavcan.sub.bms.1.id",   "UCAN1_BMS1_SUB"},
	};
};
