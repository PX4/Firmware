/****************************************************************************
 *
 *   Copyright (c) 2018-19 PX4 Development Team. All rights reserved.
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
 * @file heater_params.c
 * Heater parameters.
 *
 * @author Mark Sauder <mcsauder@gmail.com>
 * @author Alex Klimaj <alexklimaj@gmail.com>
 * @author Jake Dahl <dahl.jakejacob@gmail.com>
 */

/**
 * Target IMU device ID to regulate temperature.
 *
 * @category system
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_TEMP_ID, 0);

/**
 * Target IMU temperature.
 *
 * @category system
 * @group Sensors
 * @unit celcius
 * @min 0
 * @max 85.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(SENS_IMU_TEMP, 55.0f);

/**
 * IMU heater controller feedforward value.
 *
 * @category system
 * @group Sensors
 * @unit %
 * @min 0
 * @max 1.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(SENS_IMU_TEMP_FF, 0.05f);

/**
 * IMU heater controller integrator gain value.
 *
 * @category system
 * @group Sensors
 * @unit us/C
 * @min 0
 * @max 1.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(SENS_IMU_TEMP_I, 0.025f);

/**
 * IMU heater controller proportional gain value.
 *
 * @category system
 * @group Sensors
 * @unit us/C
 * @min 0
 * @max 2.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(SENS_IMU_TEMP_P, 1.0f);
