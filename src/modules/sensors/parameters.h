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

#pragma once

/**
 * @file parameters.h
 *
 * defines the list of parameters that are used within the sensors module
 *
 * @author Beat Kueng <beat-kueng@gmx.net>
 */

#include <lib/parameters/param.h>

namespace sensors
{

struct Parameters {
	float diff_pres_offset_pa;
#ifdef ADC_AIRSPEED_VOLTAGE_CHANNEL
	float diff_pres_analog_scale;
#endif /* ADC_AIRSPEED_VOLTAGE_CHANNEL */

	int32_t board_rotation;

	float board_offset[3];

	int32_t air_cmodel;
	float air_tube_length;
	float air_tube_diameter_mm;
};

struct ParameterHandles {
	param_t diff_pres_offset_pa;
#ifdef ADC_AIRSPEED_VOLTAGE_CHANNEL
	param_t diff_pres_analog_scale;
#endif /* ADC_AIRSPEED_VOLTAGE_CHANNEL */

	param_t board_rotation;

	param_t board_offset[3];

	param_t air_cmodel;
	param_t air_tube_length;
	param_t air_tube_diameter_mm;

};

/**
 * initialize ParameterHandles struct
 */
void initialize_parameter_handles(ParameterHandles &parameter_handles);


/**
 * Read out the parameters using the handles into the parameters struct.
 * @return 0 on success, <0 on error
 */
void update_parameters(const ParameterHandles &parameter_handles, Parameters &parameters);

} /* namespace sensors */
