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
 * ID of the Accelerometer that the calibration is for.
 *
 * @category system
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_ACC1_ID, 0);

/**
 * Accelerometer 1 priority.
 *
 * @value -1  Uninitialized
 * @value 0   Disabled
 * @value 1   Min
 * @value 25  Low
 * @value 50  Medium (Default)
 * @value 75  High
 * @value 100 Max
 *
 * @category system
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_ACC1_PRIO, -1);

/**
 * Rotation of accelerometer 1 relative to airframe.
 *
 * An internal sensor will force a value of -1, so a GCS
 * should only attempt to configure the rotation if the value is
 * greater than or equal to zero.
 *
 * @value -1 Internal
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 * @value 8 Roll 180°
 * @value 9 Roll 180°, Yaw 45°
 * @value 10 Roll 180°, Yaw 90°
 * @value 11 Roll 180°, Yaw 135°
 * @value 12 Pitch 180°
 * @value 13 Roll 180°, Yaw 225°
 * @value 14 Roll 180°, Yaw 270°
 * @value 15 Roll 180°, Yaw 315°
 * @value 16 Roll 90°
 * @value 17 Roll 90°, Yaw 45°
 * @value 18 Roll 90°, Yaw 90°
 * @value 19 Roll 90°, Yaw 135°
 * @value 20 Roll 270°
 * @value 21 Roll 270°, Yaw 45°
 * @value 22 Roll 270°, Yaw 90°
 * @value 23 Roll 270°, Yaw 135°
 * @value 24 Pitch 90°
 * @value 25 Pitch 270°
 * @value 26 Pitch 180°, Yaw 90°
 * @value 27 Pitch 180°, Yaw 270°
 * @value 28 Roll 90°, Pitch 90°
 * @value 29 Roll 180°, Pitch 90°
 * @value 30 Roll 270°, Pitch 90°
 * @value 31 Roll 90°, Pitch 180°
 * @value 32 Roll 270°, Pitch 180°
 * @value 33 Roll 90°, Pitch 270°
 * @value 34 Roll 180°, Pitch 270°
 * @value 35 Roll 270°, Pitch 270°
 * @value 36 Roll 90°, Pitch 180°, Yaw 90°
 * @value 37 Roll 90°, Yaw 270°
 * @value 38 Roll 90°, Pitch 68°, Yaw 293°
 * @value 39 Pitch 315°
 * @value 40 Roll 90°, Pitch 315°
 *
 * @min -1
 * @max 40
 * @reboot_required true
 * @category system
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_ACC1_ROT, -1);

/**
 * Accelerometer X-axis offset
 *
 * @category system
 * @group Sensor Calibration
 * @volatile
 */
PARAM_DEFINE_FLOAT(CAL_ACC1_XOFF, 0.0f);

/**
 * Accelerometer Y-axis offset
 *
 * @category system
 * @group Sensor Calibration
 * @volatile
 */
PARAM_DEFINE_FLOAT(CAL_ACC1_YOFF, 0.0f);

/**
 * Accelerometer Z-axis offset
 *
 * @category system
 * @group Sensor Calibration
 * @volatile
 */
PARAM_DEFINE_FLOAT(CAL_ACC1_ZOFF, 0.0f);

/**
 * Accelerometer X-axis scaling factor
 *
 * @category system
 * @group Sensor Calibration
 * @volatile
 */
PARAM_DEFINE_FLOAT(CAL_ACC1_XSCALE, 1.0f);

/**
 * Accelerometer Y-axis scaling factor
 *
 * @category system
 * @group Sensor Calibration
 * @volatile
 */
PARAM_DEFINE_FLOAT(CAL_ACC1_YSCALE, 1.0f);

/**
 * Accelerometer Z-axis scaling factor
 *
 * @category system
 * @group Sensor Calibration
 * @volatile
 */
PARAM_DEFINE_FLOAT(CAL_ACC1_ZSCALE, 1.0f);
