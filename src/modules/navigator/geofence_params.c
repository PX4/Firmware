/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file geofence_params.c
 *
 * Parameters for geofence
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

/*
 * Geofence parameters, accessible via MAVLink
 */

/**
 * Geofence violation action.
 *
 * Note: Setting this value to 4 enables flight termination,
 * which will kill the vehicle on violation of the fence.
 * Due to the inherent danger of this, this function is
 * disabled using a software circuit breaker, which needs
 * to be reset to 0 to really shut down the system.
 *
 * @min 0
 * @max 5
 * @value 0 None
 * @value 1 Warning
 * @value 2 Hold mode
 * @value 3 Return mode
 * @value 4 Terminate
 * @value 5 Land mode
 * @group Geofence
 */
PARAM_DEFINE_INT32(GF_ACTION, 2);

/**
 * Geofence altitude mode
 *
 * Select which altitude (AMSL) source should be used for geofence calculations.
 *
 * @min 0
 * @max 1
 * @value 0 Autopilot estimator global position altitude (GPS)
 * @value 1 Raw barometer altitude (assuming standard atmospheric pressure)
 * @group Geofence
 */
PARAM_DEFINE_INT32(GF_ALTMODE, 0);

/**
 * Geofence source
 *
 * Select which position source should be used. Selecting GPS instead of global position makes sure that there is
 * no dependence on the position estimator
 * 0 = global position, 1 = GPS
 *
 * @min 0
 * @max 1
 * @value 0 GPOS
 * @value 1 GPS
 * @group Geofence
 */
PARAM_DEFINE_INT32(GF_SOURCE, 0);

/**
 * Geofence counter limit
 *
 * Set how many subsequent position measurements outside of the fence are needed before geofence violation is triggered
 *
 * @min -1
 * @max 10
 * @increment 1
 * @group Geofence
 */
PARAM_DEFINE_INT32(GF_COUNT, -1);

/**
 * Max horizontal distance in meters.
 *
 * Maximum horizontal distance in meters the vehicle can be from home before triggering a geofence action. Disabled if 0.
 *
 * @unit m
 * @min 0
 * @max 10000
 * @increment 1
 * @group Geofence
 */
PARAM_DEFINE_FLOAT(GF_MAX_HOR_DIST, 0);

/**
 * Max vertical distance in meters.
 *
 * Maximum vertical distance in meters the vehicle can be from home before triggering a geofence action. Disabled if 0.
 *
 * @unit m
 * @min 0
 * @max 10000
 * @increment 1
 * @group Geofence
 */
PARAM_DEFINE_FLOAT(GF_MAX_VER_DIST, 0);

/**
 * Use Pre-emptive geofence triggering
 *
 * Predict the motion of the vehicle and trigger the breach if it is determined that the current trajectory
 * would result in a breach happening before the vehicle can make evasive maneuvers.
 * The vehicle is then re-routed to a safe hold position (stop for multirotor, loiter for fixed wing).
 *
 * @boolean
 * @group Geofence
 */
PARAM_DEFINE_INT32(GF_PREDICT, 1);
