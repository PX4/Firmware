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
 * @file ActuatorEffectivenessMultirotorParams.c
 *
 * Parameters for the actuator effectiveness of multirotors.
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

/**
 * Position of rotor 0 along X body axis
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R0_PX, 0.0);

/**
 * Position of rotor 0 along Y body axis
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R0_PY, 0.0);

/**
 * Position of rotor 0 along Z body axis
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R0_PZ, 0.0);

/**
 * Axis of rotor 0 thrust vector, X body axis component
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R0_AX, 0.0);

/**
 * Axis of rotor 0 thrust vector, Y body axis component
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R0_AY, 0.0);

/**
 * Axis of rotor 0 thrust vector, Z body axis component
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R0_AZ, -1.0);

/**
 * Thrust coefficient of rotor 0
 *
 * The thrust coefficient if defined as Thrust = CT * u^2,
 * where u (with value between CA_ACT0_MIN and CA_ACT0_MAX)
 * is the output signal sent to the motor controller.
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R0_CT, 0.0);

/**
 * Moment coefficient of rotor 0
 *
 * The moment coefficient if defined as Torque = KM * Thrust
 *
 * Use a positive value for a rotor with CCW rotation.
 * Use a negative value for a rotor with CW rotation.
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R0_KM, 0.05);

/**
 * Position of rotor 1 along X body axis
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R1_PX, 0.0);

/**
 * Position of rotor 1 along Y body axis
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R1_PY, 0.0);

/**
 * Position of rotor 1 along Z body axis
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R1_PZ, 0.0);

/**
 * Axis of rotor 1 thrust vector, X body axis component
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R1_AX, 0.0);

/**
 * Axis of rotor 1 thrust vector, Y body axis component
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R1_AY, 0.0);

/**
 * Axis of rotor 1 thrust vector, Z body axis component
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R1_AZ, -1.0);

/**
 * Thrust coefficient of rotor 1
 *
 * The thrust coefficient if defined as Thrust = CT * u^2,
 * where u (with value between CA_ACT1_MIN and CA_ACT1_MAX)
 * is the output signal sent to the motor controller.
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R1_CT, 0.0);

/**
 * Moment coefficient of rotor 1
 *
 * The moment coefficient if defined as Torque = KM * Thrust,
 *
 * Use a positive value for a rotor with CCW rotation.
 * Use a negative value for a rotor with CW rotation.
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R1_KM, 0.05);

/**
 * Position of rotor 2 along X body axis
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R2_PX, 0.0);

/**
 * Position of rotor 2 along Y body axis
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R2_PY, 0.0);

/**
 * Position of rotor 2 along Z body axis
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R2_PZ, 0.0);

/**
 * Axis of rotor 2 thrust vector, X body axis component
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R2_AX, 0.0);

/**
 * Axis of rotor 2 thrust vector, Y body axis component
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R2_AY, 0.0);

/**
 * Axis of rotor 2 thrust vector, Z body axis component
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R2_AZ, -1.0);

/**
 * Thrust coefficient of rotor 2
 *
 * The thrust coefficient if defined as Thrust = CT * u^2,
 * where u (with value between CA_ACT2_MIN and CA_ACT2_MAX)
 * is the output signal sent to the motor controller.
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R2_CT, 0.0);

/**
 * Moment coefficient of rotor 2
 *
 * The moment coefficient if defined as Torque = KM * Thrust
 *
 * Use a positive value for a rotor with CCW rotation.
 * Use a negative value for a rotor with CW rotation.
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R2_KM, 0.05);

/**
 * Position of rotor 3 along X body axis
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R3_PX, 0.0);

/**
 * Position of rotor 3 along Y body axis
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R3_PY, 0.0);

/**
 * Position of rotor 3 along Z body axis
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R3_PZ, 0.0);

/**
 * Axis of rotor 3 thrust vector, X body axis component
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R3_AX, 0.0);

/**
 * Axis of rotor 3 thrust vector, Y body axis component
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R3_AY, 0.0);

/**
 * Axis of rotor 3 thrust vector, Z body axis component
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R3_AZ, -1.0);

/**
 * Thrust coefficient of rotor 3
 *
 * The thrust coefficient if defined as Thrust = CT * u^2,
 * where u (with value between CA_ACT3_MIN and CA_ACT3_MAX)
 * is the output signal sent to the motor controller.
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R3_CT, 0.0);

/**
 * Moment coefficient of rotor 3
 *
 * The moment coefficient if defined as Torque = KM * Thrust
 *
 * Use a positive value for a rotor with CCW rotation.
 * Use a negative value for a rotor with CW rotation.
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R3_KM, 0.05);

/**
 * Position of rotor 4 along X body axis
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R4_PX, 0.0);

/**
 * Position of rotor 4 along Y body axis
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R4_PY, 0.0);

/**
 * Position of rotor 4 along Z body axis
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R4_PZ, 0.0);

/**
 * Axis of rotor 4 thrust vector, X body axis component
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R4_AX, 0.0);

/**
 * Axis of rotor 4 thrust vector, Y body axis component
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R4_AY, 0.0);

/**
 * Axis of rotor 4 thrust vector, Z body axis component
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R4_AZ, -1.0);

/**
 * Thrust coefficient of rotor 4
 *
 * The thrust coefficient if defined as Thrust = CT * u^2,
 * where u (with value between CA_ACT4_MIN and CA_ACT4_MAX)
 * is the output signal sent to the motor controller.
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R4_CT, 0.0);

/**
 * Moment coefficient of rotor 4
 *
 * The moment coefficient if defined as Torque = KM * Thrust
 *
 * Use a positive value for a rotor with CCW rotation.
 * Use a negative value for a rotor with CW rotation.
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R4_KM, 0.05);

/**
 * Position of rotor 5 along X body axis
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R5_PX, 0.0);

/**
 * Position of rotor 5 along Y body axis
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R5_PY, 0.0);

/**
 * Position of rotor 5 along Z body axis
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R5_PZ, 0.0);

/**
 * Axis of rotor 5 thrust vector, X body axis component
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R5_AX, 0.0);

/**
 * Axis of rotor 5 thrust vector, Y body axis component
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R5_AY, 0.0);

/**
 * Axis of rotor 5 thrust vector, Z body axis component
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R5_AZ, -1.0);

/**
 * Thrust coefficient of rotor 5
 *
 * The thrust coefficient if defined as Thrust = CT * u^2,
 * where u (with value between CA_ACT5_MIN and CA_ACT5_MAX)
 * is the output signal sent to the motor controller.
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R5_CT, 0.0);

/**
 * Moment coefficient of rotor 5
 *
 * The moment coefficient if defined as Torque = KM * Thrust
 *
 * Use a positive value for a rotor with CCW rotation.
 * Use a negative value for a rotor with CW rotation.
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R5_KM, 0.05);

/**
 * Position of rotor 6 along X body axis
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R6_PX, 0.0);

/**
 * Position of rotor 6 along Y body axis
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R6_PY, 0.0);

/**
 * Position of rotor 6 along Z body axis
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R6_PZ, 0.0);

/**
 * Axis of rotor 6 thrust vector, X body axis component
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R6_AX, 0.0);

/**
 * Axis of rotor 6 thrust vector, Y body axis component
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R6_AY, 0.0);

/**
 * Axis of rotor 6 thrust vector, Z body axis component
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R6_AZ, -1.0);

/**
 * Thrust coefficient of rotor 6
 *
 * The thrust coefficient if defined as Thrust = CT * u^2,
 * where u (with value between CA_ACT6_MIN and CA_ACT6_MAX)
 * is the output signal sent to the motor controller.
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R6_CT, 0.0);

/**
 * Moment coefficient of rotor 6
 *
 * The moment coefficient if defined as Torque = KM * Thrust
 *
 * Use a positive value for a rotor with CCW rotation.
 * Use a negative value for a rotor with CW rotation.
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R6_KM, 0.05);

/**
 * Position of rotor 7 along X body axis
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R7_PX, 0.0);

/**
 * Position of rotor 7 along Y body axis
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R7_PY, 0.0);

/**
 * Position of rotor 7 along Z body axis
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R7_PZ, 0.0);

/**
 * Axis of rotor 7 thrust vector, X body axis component
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R7_AX, 0.0);

/**
 * Axis of rotor 7 thrust vector, Y body axis component
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R7_AY, 0.0);

/**
 * Axis of rotor 7 thrust vector, Z body axis component
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R7_AZ, -1.0);

/**
 * Thrust coefficient of rotor 7
 *
 * The thrust coefficient if defined as Thrust = CT * u^2,
 * where u (with value between CA_ACT7_MIN and CA_ACT7_MAX)
 * is the output signal sent to the motor controller.
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R7_CT, 0.0);

/**
 * Moment coefficient of rotor 7
 *
 * The moment coefficient if defined as Torque = KM * Thrust
 *
 * Use a positive value for a rotor with CCW rotation.
 * Use a negative value for a rotor with CW rotation.
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MC_R7_KM, 0.05);
