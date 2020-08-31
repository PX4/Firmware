/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file weathervane_params.c
 *
 * Parameters defined by the weathervane lib.
 *
 * @author Roman Bapst <roman@auterion.com>
 */

/**
 * Enable weathervane.
 *
 * @boolean
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(WV_EN, 0);

/**
 * Weather-vane roll angle to yawrate.
 *
 * The desired gain to convert roll sp into yaw rate sp.
 *
 * @min 0.0
 * @max 3.0
 * @unit Hz
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(WV_GAIN, 1.0f);

/**
 * Minimum roll angle setpoint for weathervane controller to demand a yaw-rate.
 *
 * @min 0
 * @max 5
 * @unit deg
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(WV_ROLL_MIN, 1.0f);

/**
 * Maximum yawrate the weathervane controller is allowed to demand.
 *
 * @min 0
 * @max 120
 * @unit deg/s
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(WV_YRATE_MAX, 90.0f);