/****************************************************************************
 *
 *   Copyright (c) 2012-2020 PX4 Development Team. All rights reserved.
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
 * Bitfield selecting mag sides for calibration
 *
 * If set to two side calibration, only the offsets are estimated, the scale
 * calibration is left unchanged. Thus an initial six side calibration is
 * recommended.
 *
 * Bits:
 * ORIENTATION_TAIL_DOWN = 1
 * ORIENTATION_NOSE_DOWN = 2
 * ORIENTATION_LEFT = 4
 * ORIENTATION_RIGHT = 8
 * ORIENTATION_UPSIDE_DOWN = 16
 * ORIENTATION_RIGHTSIDE_UP = 32
 *
 * @min 34
 * @max 63
 * @value 34 Two side calibration
 * @value 38 Three side calibration
 * @value 63 Six side calibration
 * @group Sensors
 */
PARAM_DEFINE_INT32(CAL_MAG_SIDES, 63);

/**
 * Type of magnetometer compensation
 *
 * @value 0 Disabled
 * @value 1 Throttle-based compensation
 * @value 2 Current-based compensation (battery_status instance 0)
 * @value 3 Current-based compensation (battery_status instance 1)
 *
 * @category system
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_MAG_COMP_TYP, 0);

/**
 * Automatically set external rotations.
 *
 * During calibration attempt to automatically determine the rotation of external magnetometers.
 *
 * @boolean
 * @group Sensors
 */
PARAM_DEFINE_INT32(CAL_MAG_ROT_AUTO, 1);

/**
 * Magnetometer max rate.
 *
 * Magnetometer data maximum publication rate. This is an upper bound,
 * actual magnetometer data rate is still dependant on the sensor.
 *
 * @min 1
 * @max 200
 * @group Sensors
 * @unit Hz
 *
 * @reboot_required true
 *
 */
PARAM_DEFINE_FLOAT(SENS_MAG_RATE, 50.0f);

/**
 * Sensors hub mag mode
 *
 * @value 0 Publish all magnetometers
 * @value 1 Publish primary magnetometer
 *
 * @category system
 * @reboot_required true
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_MAG_MODE, 1);
