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

#include <uavcan/_register/Name_1_0.h>
#include <uavcan/_register/Value_1_0.h>

typedef struct {
	const char *uavcan_name;
	const char *px4_name;
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

	const UavcanParamBinder _uavcan_params[10] {
		{"uavcan.pub.esc.0.id",              "UCAN1_ESC_PUB"},
		{"uavcan.pub.servo.0.id",            "UCAN1_SERVO_PUB"},
		{"uavcan.pub.gps.0.id",              "UCAN1_GPS_PUB"},
		{"uavcan.sub.esc.0.id",              "UCAN1_ESC0_PID"},
		{"uavcan.sub.gps.0.id",              "UCAN1_GPS0_PID"},
		{"uavcan.sub.gps.1.id",              "UCAN1_GPS1_PID"},
		{"uavcan.sub.energy_source.0.id",      "UCAN1_BMS_ES_PID"},
		{"uavcan.sub.battery_status.0.id",     "UCAN1_BMS_BS_PID"},
		{"uavcan.sub.battery_parameters.0.id", "UCAN1_BMS_BP_PID"},
		{"uavcan.sub.legacy_bms.0.id",         "UCAN1_LG_BMS_PID"},
		//{"uavcan.sub.bms.0.id",   "UCAN1_BMS0_PID"}, //FIXME instancing
		//{"uavcan.sub.bms.1.id",   "UCAN1_BMS1_PID"},
	};
};
