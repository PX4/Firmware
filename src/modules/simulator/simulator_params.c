/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file simulator_params.c
 *
 * Parameters of software in the loop
 *
 * @author Mohamed Abdelkader <mohamedashraf123@gmail.com>
 */

/**
 * Servo Bandwidth
 *
 * Bandwidth of the servo actutor model in Hz. Used when the SITL flight dynamics model does not model model servo dynamics. If the SITL flight dynamics model does model servo dynamics, set to 0 to disable.
 *
 * @min 0.0
 * @max 15
 * @decimal 1
 * @increment 0.5
 * @group Simulator
 */
PARAM_DEFINE_FLOAT(SERVO_BW, 8.0f);

/**
 * Servo Slew Time
 *
 * Time required for the servo to slew between movement limits.
 *
 * @min 0.02
 * @max 0.2
 * @decimal 3
 * @increment 0.01
 * @group Simulator
 */
PARAM_DEFINE_FLOAT(SERVO_SLEW_TIME, 0.2f);
