/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file control_allocator_params.c
 *
 * Parameters for the control allocator.
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */


/**
 * Airframe ID
 *
 * This is used to retrieve pre-computed control effectiveness matrix
 *
 * @min 0
 * @max 2
 * @value 0 Multirotor
 * @value 1 Standard VTOL (WIP)
 * @value 2 Tiltrotor VTOL (WIP)
 * @group Control Allocation
 */
PARAM_DEFINE_INT32(CA_AIRFRAME, 0);

/**
 * Control allocation method
 *
 * @value 0 Pseudo-inverse with output clipping (default)
 * @value 1 Pseudo-inverse with sequential desaturation technique
 * @min 0
 * @max 1
 * @group Control Allocation
 */
PARAM_DEFINE_INT32(CA_METHOD, 0);

/**
 * Battery power level scaler
 *
 * This compensates for voltage drop of the battery over time by attempting to
 * normalize performance across the operating range of the battery. The copter
 * should constantly behave as if it was fully charged with reduced max acceleration
 * at lower battery percentages. i.e. if hover is at 0.5 throttle at 100% battery,
 * it will still be 0.5 at 60% battery.
 *
 * @boolean
 * @group Control Allocation
 */
PARAM_DEFINE_INT32(CA_BAT_SCALE_EN, 0);

/**
 * Airspeed scaler
 *
 * This compensates for the variation of flap effectiveness with airspeed.
 *
 * @boolean
 * @group Control Allocation
 */
PARAM_DEFINE_INT32(CA_AIR_SCALE_EN, 0);

/**
 * Minimum value for actuator 0
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ACT0_MIN, 0.0);

/**
 * Minimum value for actuator 1
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ACT1_MIN, 0.0);

/**
 * Minimum value for actuator 2
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ACT2_MIN, 0.0);

/**
 * Minimum value for actuator 3
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ACT3_MIN, 0.0);

/**
 * Minimum value for actuator 4
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ACT4_MIN, 0.0);

/**
 * Minimum value for actuator 5
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ACT5_MIN, 0.0);

/**
 * Minimum value for actuator 6
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ACT6_MIN, 0.0);

/**
 * Minimum value for actuator 7
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ACT7_MIN, 0.0);

/**
 * Minimum value for actuator 8
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ACT8_MIN, 0.0);

/**
 * Minimum value for actuator 9
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ACT9_MIN, 0.0);

/**
 * Minimum value for actuator 10
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ACT10_MIN, 0.0);

/**
 * Minimum value for actuator 11
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ACT11_MIN, 0.0);

/**
 * Minimum value for actuator 12
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ACT12_MIN, 0.0);

/**
 * Minimum value for actuator 13
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ACT13_MIN, 0.0);

/**
 * Minimum value for actuator 14
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ACT14_MIN, 0.0);

/**
 * Minimum value for actuator 15
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ACT15_MIN, 0.0);

/**
 * Maximum value for actuator 0
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ACT0_MAX, 0.0);

/**
 * Maximum value for actuator 1
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ACT1_MAX, 0.0);

/**
 * Maximum value for actuator 2
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ACT2_MAX, 0.0);

/**
 * Maximum value for actuator 3
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ACT3_MAX, 0.0);

/**
 * Maximum value for actuator 4
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ACT4_MAX, 0.0);

/**
 * Maximum value for actuator 5
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ACT5_MAX, 0.0);

/**
 * Maximum value for actuator 6
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ACT6_MAX, 0.0);

/**
 * Maximum value for actuator 7
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ACT7_MAX, 0.0);

/**
 * Maximum value for actuator 8
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ACT8_MAX, 0.0);

/**
 * Maximum value for actuator 9
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ACT9_MAX, 0.0);

/**
 * Maximum value for actuator 10
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ACT10_MAX, 0.0);

/**
 * Maximum value for actuator 11
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ACT11_MAX, 0.0);

/**
 * Maximum value for actuator 12
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ACT12_MAX, 0.0);

/**
 * Maximum value for actuator 13
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ACT13_MAX, 0.0);

/**
 * Maximum value for actuator 14
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ACT14_MAX, 0.0);

/**
 * Maximum value for actuator 15
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ACT15_MAX, 0.0);
