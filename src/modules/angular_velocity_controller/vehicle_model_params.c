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
 * @file vehicle_model_params.c
 * Parameters for vehicle model.
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

/**
 * Mass
 *
 * @unit kg
 * @decimal 5
 * @increment 0.00001
 * @group Vehicle Model
 */
PARAM_DEFINE_FLOAT(VM_MASS, 1.f);

/**
 * Inertia matrix, XX component
 *
 * @unit kg m^2
 * @decimal 5
 * @increment 0.00001
 * @group Vehicle Model
 */
PARAM_DEFINE_FLOAT(VM_INERTIA_XX, 0.01f);

/**
 * Inertia matrix, YY component
 *
 * @unit kg m^2
 * @decimal 5
 * @increment 0.00001
 * @group Vehicle Model
 */
PARAM_DEFINE_FLOAT(VM_INERTIA_YY, 0.01f);

/**
 * Inertia matrix, ZZ component
 *
 * @unit kg m^2
 * @decimal 5
 * @increment 0.00001
 * @group Vehicle Model
 */
PARAM_DEFINE_FLOAT(VM_INERTIA_ZZ, 0.01f);

/**
 * Inertia matrix, XY component
 *
 * @unit kg m^2
 * @decimal 5
 * @increment 0.00001
 * @group Vehicle Model
 */
PARAM_DEFINE_FLOAT(VM_INERTIA_XY, 0.f);

/**
 * Inertia matrix, XZ component
 *
 * @unit kg m^2
 * @decimal 5
 * @increment 0.00001
 * @group Vehicle Model
 */
PARAM_DEFINE_FLOAT(VM_INERTIA_XZ, 0.f);

/**
 * Inertia matrix, YZ component
 *
 * @unit kg m^2
 * @decimal 5
 * @increment 0.00001
 * @group Vehicle Model
 */
PARAM_DEFINE_FLOAT(VM_INERTIA_YZ, 0.f);
