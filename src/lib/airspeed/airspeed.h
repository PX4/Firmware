/****************************************************************************
 *
 *   Copyright (c) 2012-2013, 2017 PX4 Development Team. All rights reserved.
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
 * @file airspeed.h
 * Airspeed estimation declarations
 *
 * @author Lorenz Meier <lorenz@px4.io>
 *
 */

#ifndef AIRSPEED_H_
#define AIRSPEED_H_

#include "math.h"

__BEGIN_DECLS

enum AIRSPEED_SENSOR_MODEL {
	AIRSPEED_SENSOR_MODEL_MEMBRANE = 0,
	AIRSPEED_SENSOR_MODEL_SDP3X,
};

enum AIRSPEED_COMPENSATION_MODEL {
	AIRSPEED_COMPENSATION_MODEL_PITOT = 0,
	AIRSPEED_COMPENSATION_MODEL_NO_PITOT = 1,
	AIRSPEED_COMPENSATION_TUBE_PRESSURE_LOSS = 2
};

/**
 * Calculate indicated airspeed (IAS) and correct for friction inside pitot and tube.
 *
 * Note that the indicated airspeed is not the true airspeed because it
 * lacks the air density compensation. Use the calc_true_airspeed functions to get
 * the true airspeed.
 *
 * @param total_pressure pressure inside the pitot/prandtl tube
 * @param static_pressure pressure at the side of the tube/airplane
 * @return indicated airspeed in m/s
 */
__EXPORT float calc_IAS_corrected(enum AIRSPEED_COMPENSATION_MODEL pmodel,
				  enum AIRSPEED_SENSOR_MODEL smodel,
				  float tube_len, float tube_dia_mm, float differential_pressure, float pressure_ambient, float temperature_celsius);

/**
 * Calculate indicated airspeed (IAS).
 *
 * Note that the indicated airspeed is not the true airspeed because it
 * lacks the air density compensation. Use the calc_true_airspeed functions to get
 * the true airspeed.
 *
 * @param total_pressure pressure inside the pitot/prandtl tube
 * @param static_pressure pressure at the side of the tube/airplane
 * @return indicated airspeed in m/s
 */
__EXPORT float calc_IAS(float differential_pressure);

/**
 * Calculate true airspeed (TAS) from calibrated airspeed (CAS).
 *
 * Note that the true airspeed is NOT the groundspeed, because of the effects of wind.
 *
 * @param speed_equivalent current calibrated airspeed
 * @param pressure_ambient pressure at the side of the tube/airplane
 * @param temperature_celsius air temperature in degrees celcius
 * @return TAS in m/s
 */
__EXPORT float calc_TAS_from_CAS(float speed_indicated, float pressure_ambient,
				 float temperature_celsius);

/**
 * Calculate calibrated airspeed (CAS) from indicated airspeed (IAS).
 *
 * @param speed_indicated current indicated airspeed
 * @param scale scale from IAS to CAS (accounting for instrument and pitot position erros)
 * @return CAS in m/s
 */
__EXPORT float calc_CAS_from_IAS(float speed_indicated, float scale);


/**
 * Directly calculate true airspeed (TAS).
 *
 * Here we assume to have no instrument or pitot position error (IAS = CAS).
 * Note that the true airspeed is NOT the groundspeed, because of the effects of wind.
 *
 * @param total_pressure pressure inside the pitot/prandtl tube
 * @param static_pressure pressure at the side of the tube/airplane
 * @param temperature_celsius air temperature in degrees celcius
 * @return true airspeed in m/s
 */
__EXPORT float calc_TAS(float total_pressure, float static_pressure, float temperature_celsius);

/**
* Calculates air density.
*
* @param static_pressure ambient pressure in millibar
* @param temperature_celcius air / ambient temperature in celcius
*/
__EXPORT float get_air_density(float static_pressure, float temperature_celsius);

/**
 * Calculate calibrated airspeed (CAS) from true airspeed (TAS).
 * It is the inverse function to calc_TAS_from_CAS()
 *
 * @param speed_true current true airspeed
 * @param pressure_ambient pressure at the side of the tube/airplane
 * @param temperature_celsius air temperature in degrees celcius
 * @return CAS in m/s
 */
__EXPORT float calc_CAS_from_TAS(float speed_true, float pressure_ambient,
				 float temperature_celsius);

__END_DECLS

#endif
