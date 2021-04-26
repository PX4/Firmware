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
 * @file angular_velocity_controller_params.c
 * Parameters for angular velocity controller.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

/**
 * Body X axis angular velocity P gain
 *
 * Body X axis angular velocity proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @unit 1/s
 * @min 0.0
 * @max 20.0
 * @decimal 3
 * @increment 0.01
 * @group Angular Velocity Control
 */
PARAM_DEFINE_FLOAT(AVC_X_P, 18.f);

/**
 * Body X axis angular velocity I gain
 *
 * Body X axis angular velocity integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @unit Nm/rad
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group Angular Velocity Control
 */
PARAM_DEFINE_FLOAT(AVC_X_I, 0.2f);

/**
 * Body X axis angular velocity integrator limit
 *
 * Body X axis angular velocity integrator limit. Can be set to increase the amount of integrator available to counteract disturbances or reduced to improve settling time after large roll moment trim changes.
 *
 * @unit Nm
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Angular Velocity Control
 */
PARAM_DEFINE_FLOAT(AVC_X_I_LIM, 0.3f);

/**
 * Body X axis angular velocity D gain
 *
 * Body X axis angular velocity differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 4
 * @increment 0.01
 * @group Angular Velocity Control
 */
PARAM_DEFINE_FLOAT(AVC_X_D, 0.36f);

/**
 * Body X axis angular velocity feedforward gain
 *
 * Improves tracking performance.
 *
 * @unit Nm/(rad/s)
 * @min 0.0
 * @decimal 4
 * @group Angular Velocity Control
 */
PARAM_DEFINE_FLOAT(AVC_X_FF, 0.0f);

/**
 * Body X axis angular velocity controller gain
 *
 * Global gain of the controller.
 *
 * This gain scales the P, I and D terms of the controller:
 * output = AVC_X_K * (AVC_X_P * error
 * 			+ AVC_X_I * error_integral
 * 			+ AVC_X_D * error_derivative)
 * Set AVC_X_P=1 to implement a PID in the ideal form.
 * Set AVC_X_K=1 to implement a PID in the parallel form.
 *
 * @min 0.0
 * @max 5.0
 * @decimal 4
 * @increment 0.0005
 * @group Angular Velocity Control
 */
PARAM_DEFINE_FLOAT(AVC_X_K, 1.0f);

/**
 * Body Y axis angular velocity P gain
 *
 * Body Y axis angular velocity proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @unit 1/s
 * @min 0.0
 * @max 20.0
 * @decimal 3
 * @increment 0.01
 * @group Angular Velocity Control
 */
PARAM_DEFINE_FLOAT(AVC_Y_P, 18.f);

/**
 * Body Y axis angular velocity I gain
 *
 * Body Y axis angular velocity integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @unit Nm/rad
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group Angular Velocity Control
 */
PARAM_DEFINE_FLOAT(AVC_Y_I, 0.2f);

/**
 * Body Y axis angular velocity integrator limit
 *
 * Body Y axis angular velocity integrator limit. Can be set to increase the amount of integrator available to counteract disturbances or reduced to improve settling time after large pitch moment trim changes.
 *
 * @unit Nm
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Angular Velocity Control
 */
PARAM_DEFINE_FLOAT(AVC_Y_I_LIM, 0.3f);

/**
 * Body Y axis angular velocity D gain
 *
 * Body Y axis angular velocity differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 4
 * @increment 0.01
 * @group Angular Velocity Control
 */
PARAM_DEFINE_FLOAT(AVC_Y_D, 0.36f);

/**
 * Body Y axis angular velocity feedforward
 *
 * Improves tracking performance.
 *
 * @unit Nm/(rad/s)
 * @min 0.0
 * @decimal 4
 * @group Angular Velocity Control
 */
PARAM_DEFINE_FLOAT(AVC_Y_FF, 0.0f);

/**
 * Body Y axis angular velocity controller gain
 *
 * Global gain of the controller.
 *
 * This gain scales the P, I and D terms of the controller:
 * output = AVC_Y_K * (AVC_Y_P * error
 * 			     + AVC_Y_I * error_integral
 * 			     + AVC_Y_D * error_derivative)
 * Set AVC_Y_P=1 to implement a PID in the ideal form.
 * Set AVC_Y_K=1 to implement a PID in the parallel form.
 *
 * @min 0.0
 * @max 20.0
 * @decimal 4
 * @increment 0.0005
 * @group Angular Velocity Control
 */
PARAM_DEFINE_FLOAT(AVC_Y_K, 1.0f);

/**
 * Body Z axis angular velocity P gain
 *
 * Body Z axis angular velocity proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @unit 1/s
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @increment 0.01
 * @group Angular Velocity Control
 */
PARAM_DEFINE_FLOAT(AVC_Z_P, 7.f);

/**
 * Body Z axis angular velocity I gain
 *
 * Body Z axis angular velocity integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @unit Nm/rad
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Angular Velocity Control
 */
PARAM_DEFINE_FLOAT(AVC_Z_I, 0.1f);

/**
 * Body Z axis angular velocity integrator limit
 *
 * Body Z axis angular velocity integrator limit. Can be set to increase the amount of integrator available to counteract disturbances or reduced to improve settling time after large yaw moment trim changes.
 *
 * @unit Nm
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Angular Velocity Control
 */
PARAM_DEFINE_FLOAT(AVC_Z_I_LIM, 0.30f);

/**
 * Body Z axis angular velocity D gain
 *
 * Body Z axis angular velocity differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.01
 * @group Angular Velocity Control
 */
PARAM_DEFINE_FLOAT(AVC_Z_D, 0.0f);

/**
 * Body Z axis angular velocity feedforward
 *
 * Improves tracking performance.
 *
 * @unit Nm/(rad/s)
 * @min 0.0
 * @decimal 4
 * @increment 0.01
 * @group Angular Velocity Control
 */
PARAM_DEFINE_FLOAT(AVC_Z_FF, 0.0f);

/**
 * Body Z axis angular velocity controller gain
 *
 * Global gain of the controller.
 *
 * This gain scales the P, I and D terms of the controller:
 * output = AVC_Z_K * (AVC_Z_P * error
 * 			     + AVC_Z_I * error_integral
 * 			     + AVC_Z_D * error_derivative)
 * Set AVC_Z_P=1 to implement a PID in the ideal form.
 * Set AVC_Z_K=1 to implement a PID in the parallel form.
 *
 * @min 0.0
 * @max 5.0
 * @decimal 4
 * @increment 0.0005
 * @group Angular Velocity Control
 */
PARAM_DEFINE_FLOAT(AVC_Z_K, 1.0f);
