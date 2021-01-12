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
 * @file motor_params.c
 *
 * Parameters for motors.
 *
 */


/**
 * Minimum motor rise time (slew rate limit).
 *
 * Minimum time allowed for the motor input signal to pass through
 * a range of 1000 PWM units. A value x means that the motor signal
 * can only go from 1000 to 2000 PWM in maximum x seconds.
 *
 * Zero means that slew rate limiting is disabled.
 *
 * @min 0.0
 * @unit s/(1000*PWM)
 * @group PWM Outputs
 */
PARAM_DEFINE_FLOAT(MOT_SLEW_MAX, 0.0f);

/**
 * Thrust to motor control signal model parameter
 *
 * Parameter used to model the nonlinear relationship between
 * motor control signal (e.g. PWM) and static thrust.
 *
 * The model is: rel_thrust = factor * rel_signal^2 + (1-factor) * rel_signal,
 * where rel_thrust is the normalized thrust between 0 and 1, and
 * rel_signal is the relative motor control signal between 0 and 1.
 *
 * @min 0.0
 * @max 1.0
 * @group PWM Outputs
 */
PARAM_DEFINE_FLOAT(THR_MDL_FAC, 0.0f);
