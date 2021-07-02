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
 * @file vtol_att_control_params.c
 * Parameters for vtol attitude controller.
 *
 * @author Roman Bapst <roman@px4.io>
 * @author Sander Smeets <sander@droneslab.com>
 */

/**
 * Minimum PWM of VTOL motors when in fixed-wing mode
 *
 * @unit us
 * @min 900
 * @max 2000
 * @increment 1
 * @decimal 0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_IDLE_PWM_FW, 1000);

/**
 * Permanent stabilization in fw mode
 *
 * If set to one this parameter will cause permanent attitude stabilization in fw mode.
 * This parameter has been introduced for pure convenience sake.
 *
 * @boolean
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_FW_PERM_STAB, 0);

/**
 * VTOL Type (Tailsitter=0, Tiltrotor=1, Standard=2)
 *
 * @value 0 Tailsitter
 * @value 1 Tiltrotor
 * @value 2 Standard
 * @min 0
 * @max 2
 * @decimal 0
 * @reboot_required true
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_TYPE, 0);

/**
 * Lock elevons in multicopter mode
 *
 * If set to 1 the elevons are locked in multicopter mode
 *
 * @boolean
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_ELEV_MC_LOCK, 1);

/**
 * Duration of a front transition
 *
 * Time in seconds used for a transition
 *
 * @unit s
 * @min 0.00
 * @max 20.00
 * @increment 1
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_F_TRANS_DUR, 5.0f);

/**
 * Duration of a back transition
 *
 * Time in seconds used for a back transition
 *
 * @unit s
 * @min 0.00
 * @max 20.00
 * @increment 1
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_B_TRANS_DUR, 4.0f);

/**
 * Target throttle value for the transition to fixed wing flight.
 *
 * standard vtol: pusher
 *
 * tailsitter, tiltrotor: main throttle
 *
 * @min 0.0
 * @max 1.0
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_F_TRANS_THR, 1.0f);

/**
 * Target throttle value for the transition to hover flight.
 *
 * standard vtol: pusher
 *
 * tailsitter, tiltrotor: main throttle
 *
 * Note for standard vtol:
 * For ESCs and mixers that support reverse thrust on low PWM values set this to a negative value to apply active breaking
 * For ESCs that support thrust reversal with a control channel please set VT_B_REV_OUT and set this to a positive value to apply active breaking
 *
 * @min -1
 * @max 1
 * @increment 0.01
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_B_TRANS_THR, 0.0f);

/**
 * Approximate deceleration during back transition
 *
 * The approximate deceleration during a back transition in m/s/s
 * Used to calculate back transition distance in mission mode. A lower value will make the VTOL transition further from the destination waypoint.
 * For standard vtol and tiltrotors a controller is used to track this value during the transition.
 *
 * @unit m/s^2
 * @min 0.5
 * @max 10
 * @increment 0.1
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_B_DEC_MSS, 2.0f);

/**
 * Transition blending airspeed
 *
 * Airspeed at which we can start blending both fw and mc controls. Set to 0 to disable.
 *
 * @unit m/s
 * @min 0.00
 * @max 30.00
 * @increment 1
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_ARSP_BLEND, 8.0f);

/**
 * Transition airspeed
 *
 * Airspeed at which we can switch to fw mode
 *
 * @unit m/s
 * @min 0.00
 * @max 30.00
 * @increment 1
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_ARSP_TRANS, 10.0f);

/**
 * Front transition timeout
 *
 * Time in seconds after which transition will be cancelled. Disabled if set to 0.
 *
 * @unit s
 * @min 0.00
 * @max 30.00
 * @increment 1
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TRANS_TIMEOUT, 15.0f);

/**
 * Front transition minimum time
 *
 * Minimum time in seconds for front transition.
 *
 * @unit s
 * @min 0.0
 * @max 20.0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TRANS_MIN_TM, 2.0f);

/**
 * QuadChute Altitude
 *
 * Minimum altitude for fixed wing flight, when in fixed wing the altitude drops below this altitude
 * the vehicle will transition back to MC mode and enter failsafe RTL
 * @min 0.0
 * @max 200.0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_FW_MIN_ALT, 0.0f);

/**
 * Adaptive QuadChute
 *
 * Maximum negative altitude error for fixed wing flight. If the altitude drops below this value below the altitude setpoint
 * the vehicle will transition back to MC mode and enter failsafe RTL.
 * @min 0.0
 * @max 200.0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_FW_ALT_ERR, 0.0f);

/**
 * QuadChute Max Pitch
 *
 * Maximum pitch angle before QuadChute engages
 * Above this the vehicle will transition back to MC mode and enter failsafe RTL
 * @min 0
 * @max 180
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_FW_QC_P, 0);

/**
 * QuadChute Max Roll
 *
 * Maximum roll angle before QuadChute engages
 * Above this the vehicle will transition back to MC mode and enter failsafe RTL
 * @min 0
 * @max 180
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_FW_QC_R, 0);

/**
 * Airspeed less front transition time (open loop)
 *
 * The duration of the front transition when there is no airspeed feedback available.
 *
 * @unit s
 * @min 1.0
 * @max 30.0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_F_TR_OL_TM, 6.0f);

/**
 * The channel number of motors that must be turned off in fixed wing mode.
 *
 * @min 0
 * @max 12345678
 * @increment 1
 * @decimal 0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_FW_MOT_OFFID, 0);

/**
 * The channel number of motors which provide lift during hover.
 *
 * @min 0
 * @max 12345678
 * @increment 1
 * @decimal 0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_MOT_ID, 0);

/**
 * Differential thrust in forwards flight.
 *
 * Set to 1 to enable differential thrust in fixed-wing flight.
 *
 * @min 0
 * @max 1
 * @decimal 0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_FW_DIFTHR_EN, 0);

/**
 * Differential thrust scaling factor
 *
 * This factor specifies how the yaw input gets mapped to differential thrust in forwards flight.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_FW_DIFTHR_SC, 0.1f);

/**
 * Backtransition deceleration setpoint to pitch feedforward gain.
 *
 *
 * @unit rad s^2/m
 * @min 0
 * @max 0.2
 * @decimal 1
 * @increment 0.05
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_B_DEC_FF, 0.12f);

/**
 * Backtransition deceleration setpoint to pitch I gain.
 *
 *
 * @unit rad s/m
 * @min 0
 * @max 0.3
 * @decimal 1
 * @increment 0.05
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_B_DEC_I, 0.1f);

/**
 * Enable the usage of AUX outputs for hover motors.
 *
 * Set this parameter to true if the vehicle's hover motors are connected to the FMU (AUX) port.
 * Not required for boards that only have a FMU, and no IO.
 * Only applies for standard VTOL and tiltrotor.
 *
 * @boolean
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_MC_ON_FMU, 0);
