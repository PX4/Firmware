/****************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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
 * PX4Flow board rotation
 *
 * This parameter defines the yaw rotation of the PX4FLOW board relative to the vehicle body frame.
 * Zero rotation is defined as X on flow board pointing towards front of vehicle.
 * The recommneded installation default for the PX4FLOW board is with the Y axis forward (270 deg yaw).
 *
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 *
 * @reboot_required true
 *
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_FLOW_ROT, 6);

/**
 * Minimum height above ground when reliant on optical flow.
 *
 * This parameter defines the minimum distance from ground at which the optical flow sensor operates reliably.
 * The sensor may be usable below this height, but accuracy will progressively reduce to loss of focus.
 *
 * @unit m
 * @min 0.0
 * @max 1.0
 * @increment 0.1
 * @decimal 1
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_FLOW_MINHGT, 0.7f);

/**
 * Maximum height above ground when reliant on optical flow.
 *
 * This parameter defines the maximum distance from ground at which the optical flow sensor operates reliably.
 * The height setpoint will be limited to be no greater than this value when the navigation system
 * is completely reliant on optical flow data and the height above ground estimate is valid.
 * The sensor may be usable above this height, but accuracy will progressively degrade.
 *
 * @unit m
 * @min 1.0
 * @max 25.0
 * @increment 0.1
 * @decimal 1
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_FLOW_MAXHGT, 3.0f);

/**
 * Magnitude of maximum angular flow rate reliably measurable by the optical flow sensor.
 *
 * Optical flow data will not fused by the estimators if the magnitude of the flow rate exceeds this value and
 * control loops will be instructed to limit ground speed such that the flow rate produced by movement over ground
 * is less than 50% of this value.
 *
 * @unit rad/s
 * @min 1.0
 * @decimal 2
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_FLOW_MAXR, 2.5f);
