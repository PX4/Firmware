/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 * @file simulator_params.c
 *
 * Parameters of software in the loop
 *
 * @author Mohamed Abdelkader <mohamedashraf123@gmail.com>
 */

/**
 * Simulator Battery drain interval
 *
 * @min 1
 * @max 86400
 * @increment 1
 * @unit s
 *
 * @group SITL
 */
PARAM_DEFINE_FLOAT(SIM_BAT_DRAIN, 60);

/**
 * Simulator Battery minimal percentage. Can be used to alter
 * the battery level during SITL- or HITL-simulation on the fly.
 * Particularly useful for testing different low-battery behaviour.
 *
 * @min 0
 * @max 100
 * @increment 0.1
 * @unit %
 *
 * @group SITL
 */
PARAM_DEFINE_FLOAT(SIM_BAT_MIN_PCT, 50.0f);

/**
 * Simulator GPS noise multiplier.
 *
 * @min 0
 * @max 10
 * @increment 0.1
 * @unit %
 *
 * @group SITL
 */
PARAM_DEFINE_FLOAT(SIM_GPS_NOISE_X, 0.0f);

/**
 * Simulator block GPS data.
 *
 * Enable to block the publication of any incoming simulation GPS data.
 *
 * @boolean
 * @group SITL
 */
PARAM_DEFINE_INT32(SIM_GPS_BLOCK, 0);

/**
 * Simulator block accelerometer data.
 *
 * Enable to block the publication of any incoming simulation accelerometer data.
 *
 * @boolean
 * @group SITL
 */
PARAM_DEFINE_INT32(SIM_ACCEL_BLOCK, 0);

/**
 * Simulator block gyroscope data.
 *
 * Enable to block the publication of any incoming simulation gyroscope data.
 *
 * @boolean
 * @group SITL
 */
PARAM_DEFINE_INT32(SIM_GYRO_BLOCK, 0);

/**
 * Simulator block magnetometer data.
 *
 * Enable to block the publication of any incoming simulation magnetometer data.
 *
 * @boolean
 * @group SITL
 */
PARAM_DEFINE_INT32(SIM_MAG_BLOCK, 0);

/**
 * Simulator block barometer data.
 *
 * Enable to block the publication of any incoming simulation barometer data.
 *
 * @boolean
 * @group SITL
 */
PARAM_DEFINE_INT32(SIM_BARO_BLOCK, 0);

/**
 * Simulator block differential pressure data.
 *
 * Enable to block the publication of any incoming simulation differential pressure data.
 *
 * @boolean
 * @group SITL
 */
PARAM_DEFINE_INT32(SIM_DPRES_BLOCK, 0);
