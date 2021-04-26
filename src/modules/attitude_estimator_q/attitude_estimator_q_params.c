/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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

/*
 * @file attitude_estimator_q_params.c
 *
 * Parameters for attitude estimator (quaternion based)
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

/**
 * Complimentary filter accelerometer weight
 *
 * @group Attitude Q estimator
 * @min 0
 * @max 1
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(ATT_W_ACC, 0.2f);

/**
 * Complimentary filter magnetometer weight
 *
 * Set to 0 to avoid using the magnetometer.
 *
 * @group Attitude Q estimator
 * @min 0
 * @max 1
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(ATT_W_MAG, 0.1f);

/**
 * Complimentary filter external heading weight
 *
 * @group Attitude Q estimator
 * @min 0
 * @max 1
 */
PARAM_DEFINE_FLOAT(ATT_W_EXT_HDG, 0.1f);

/**
 * Complimentary filter gyroscope bias weight
 *
 * @group Attitude Q estimator
 * @min 0
 * @max 1
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(ATT_W_GYRO_BIAS, 0.1f);

/**
 * Magnetic declination, in degrees
 *
 * This parameter is not used in normal operation,
 * as the declination is looked up based on the
 * GPS coordinates of the vehicle.
 *
 * @group Attitude Q estimator
 * @unit deg
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(ATT_MAG_DECL, 0.0f);

/**
 * Automatic GPS based declination compensation
 *
 * @group Attitude Q estimator
 * @boolean
 */
PARAM_DEFINE_INT32(ATT_MAG_DECL_A, 1);

/**
 * External heading usage mode (from Motion capture/Vision)
 *
 * Set to 1 to use heading estimate from vision.
 * Set to 2 to use heading from motion capture.
 *
 * @group Attitude Q estimator
 * @value 0 None
 * @value 1 Vision
 * @value 2 Motion Capture
 * @min 0
 * @max 2
 */
PARAM_DEFINE_INT32(ATT_EXT_HDG_M, 0);

/**
 * Acceleration compensation based on GPS velocity.
 *
 * @group Attitude Q estimator
 * @boolean
 */
PARAM_DEFINE_INT32(ATT_ACC_COMP, 1);

/**
 * Gyro bias limit
 *
 * @group Attitude Q estimator
 * @unit rad/s
 * @min 0
 * @max 2
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(ATT_BIAS_MAX, 0.05f);
