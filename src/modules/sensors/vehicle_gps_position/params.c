/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * Multi GPS Blending Control Mask.
 *
 * Set bits in the following positions to set which GPS accuracy metrics will be used to calculate the blending weight. Set to zero to disable and always used first GPS instance.
 * 0 : Set to true to use speed accuracy
 * 1 : Set to true to use horizontal position accuracy
 * 2 : Set to true to use vertical position accuracy
 *
 * @group Sensors
 * @min 0
 * @max 7
 * @bit 0 use speed accuracy
 * @bit 1 use hpos accuracy
 * @bit 2 use vpos accuracy
 */
PARAM_DEFINE_INT32(SENS_GPS_MASK, 0);

/**
 * Multi GPS Blending Time Constant
 *
 * Sets the longest time constant that will be applied to the calculation of GPS position and height offsets used to correct data from multiple GPS data for steady state position differences.
 *
 *
 * @group Sensors
 * @min 1.0
 * @max 100.0
 * @unit s
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(SENS_GPS_TAU, 10.0f);

/**
 * Multi GPS primary instance
 *
 * When no blending is active, this defines the preferred GPS receiver instance.
 * The GPS selection logic waits until the primary receiver is available to
 * send data to the EKF even if a secondary instance is already available.
 * The secondary instance is then only used if the primary one times out.
 *
 * To have an equal priority of all the instances, set this parameter to -1 and
 * the best receiver will be used.
 *
 * This parameter has no effect if blending is active.
 *
 * @group Sensors
 * @min -1
 * @max 1
 */
PARAM_DEFINE_INT32(SENS_GPS_PRIME, 0);
