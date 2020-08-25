/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @author Beat Kueng <beat-kueng@gmx.net>
 */

#include "parameters.h"

namespace sensors
{

void initialize_parameter_handles(ParameterHandles &parameter_handles)
{
	/* Differential pressure offset */
	parameter_handles.diff_pres_offset_pa = param_find("SENS_DPRES_OFF");
#ifdef ADC_AIRSPEED_VOLTAGE_CHANNEL
	parameter_handles.diff_pres_analog_scale = param_find("SENS_DPRES_ANSC");
#endif /* ADC_AIRSPEED_VOLTAGE_CHANNEL */

	/* rotations */
	parameter_handles.board_rotation = param_find("SENS_BOARD_ROT");

	/* rotation offsets */
	parameter_handles.board_offset[0] = param_find("SENS_BOARD_X_OFF");
	parameter_handles.board_offset[1] = param_find("SENS_BOARD_Y_OFF");
	parameter_handles.board_offset[2] = param_find("SENS_BOARD_Z_OFF");

	parameter_handles.air_cmodel = param_find("CAL_AIR_CMODEL");
	parameter_handles.air_tube_length = param_find("CAL_AIR_TUBELEN");
	parameter_handles.air_tube_diameter_mm = param_find("CAL_AIR_TUBED_MM");

	(void)param_find("BAT_V_DIV");
	(void)param_find("BAT_A_PER_V");

	(void)param_find("CAL_ACC0_ID");
	(void)param_find("CAL_GYRO0_ID");

	(void)param_find("CAL_MAG0_ID");
	(void)param_find("CAL_MAG1_ID");
	(void)param_find("CAL_MAG2_ID");
	(void)param_find("CAL_MAG3_ID");
	(void)param_find("CAL_MAG0_ROT");
	(void)param_find("CAL_MAG1_ROT");
	(void)param_find("CAL_MAG2_ROT");
	(void)param_find("CAL_MAG3_ROT");
	(void)param_find("CAL_MAG_SIDES");

	(void)param_find("SYS_PARAM_VER");
	(void)param_find("SYS_AUTOSTART");
	(void)param_find("SYS_AUTOCONFIG");
	(void)param_find("TRIG_MODE");
	(void)param_find("UAVCAN_ENABLE");

	// Parameters controlling the on-board sensor thermal calibrator
	(void)param_find("SYS_CAL_TDEL");
	(void)param_find("SYS_CAL_TMAX");
	(void)param_find("SYS_CAL_TMIN");
}

void update_parameters(const ParameterHandles &parameter_handles, Parameters &parameters)
{
	/* Airspeed offset */
	param_get(parameter_handles.diff_pres_offset_pa, &(parameters.diff_pres_offset_pa));
#ifdef ADC_AIRSPEED_VOLTAGE_CHANNEL
	param_get(parameter_handles.diff_pres_analog_scale, &(parameters.diff_pres_analog_scale));
#endif /* ADC_AIRSPEED_VOLTAGE_CHANNEL */

	param_get(parameter_handles.board_rotation, &(parameters.board_rotation));

	param_get(parameter_handles.board_offset[0], &(parameters.board_offset[0]));
	param_get(parameter_handles.board_offset[1], &(parameters.board_offset[1]));
	param_get(parameter_handles.board_offset[2], &(parameters.board_offset[2]));

	param_get(parameter_handles.air_cmodel, &parameters.air_cmodel);
	param_get(parameter_handles.air_tube_length, &parameters.air_tube_length);
	param_get(parameter_handles.air_tube_diameter_mm, &parameters.air_tube_diameter_mm);
}

} /* namespace sensors */
