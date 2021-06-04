/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file gpsfailure_params.c
 *
 * Parameters for GPSF navigation mode
 *
 */

/**
 * Loiter time
 *
 * The time in seconds the system should do open loop loiter and wait for GPS recovery
 * before it goes into flight termination. Set to 0 to disable.
 *
 * @unit s
 * @min 0.0
 * @max 3600.0
 * @decimal 0
 * @increment 1
 * @group GPS Failure Navigation
 */
PARAM_DEFINE_FLOAT(NAV_GPSF_LT, 0.0f);

/**
 * GPS failure fixed roll angle
 *
 * Roll in degrees during the loiter after the vehicle has lost GPS in an auto mode (e.g. mission or loiter)
 *
 * @unit deg
 * @min 0.0
 * @max 30.0
 * @decimal 1
 * @increment 0.5
 * @group GPS Failure Navigation
 */
PARAM_DEFINE_FLOAT(NAV_GPSF_R, 15.0f);

/**
 * Fixed pitch angle
 *
 * Pitch in degrees during the open loop loiter
 *
 * @unit deg
 * @min -30.0
 * @max 30.0
 * @decimal 1
 * @increment 0.5
 * @group GPS Failure Navigation
 */
PARAM_DEFINE_FLOAT(NAV_GPSF_P, 0.0f);

/**
 * Thrust
 *
 * Thrust value which is set during the open loop loiter
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group GPS Failure Navigation
 */
PARAM_DEFINE_FLOAT(NAV_GPSF_TR, 0.0f);
