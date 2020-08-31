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
 * @file collisionprevention_params.c
 *
 * Parameters defined by the collisionprevention lib.
 *
 * @author Tanja Baumann <tanja@auterion.com>
 */

/**
 * Minimum distance the vehicle should keep to all obstacles
 *
 * Only used in Position mode. Collision avoidance is disabled by setting this parameter to a negative value
 *
 * @min -1
 * @max 15
 * @unit m
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(CP_DIST, -1.0f);

/**
 * Average delay of the range sensor message plus the tracking delay of the position controller in seconds
 *
 * Only used in Position mode.
 *
 * @min 0
 * @max 1
 * @unit s
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(CP_DELAY, 0.4f);

/**
 * Angle left/right from the commanded setpoint by which the collision prevention algorithm can choose to change the setpoint direction
 *
 * Only used in Position mode.
 *
 * @min 0
 * @max 90
 * @unit deg
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(CP_GUIDE_ANG, 30.f);

/**
 * Boolean to allow moving into directions where there is no sensor data (outside FOV)
 *
 * Only used in Position mode.
 *
 * @boolean
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(CP_GO_NO_DATA, 0);
