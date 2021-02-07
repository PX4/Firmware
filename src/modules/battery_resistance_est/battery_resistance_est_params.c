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
 * Battery internal resistance online estimator
 *
 * Update period of BAT1_R_INTERNAL.
 *
 * @group Battery Calibration
 * @reboot_required true
 * @min 0
 * @unit s
 */
PARAM_DEFINE_FLOAT(BAT_RIN_UPDATE, 5.f);
/**
 * Battery internal resistance online estimator
 *
 * Initial per cell estimate of voltage open circuit in battery equivalent circuit model.
 *
 * @group Battery Calibration
 * @reboot_required true
 * @min 0
 * @unit V
 */
PARAM_DEFINE_FLOAT(BAT_VOC_INIT, 4.2f);
/**
 * Battery internal resistance online estimator
 *
 * Initial estimate of series ohmic resistor in battery (all cells) equivalent circuit model.
 *
 * @group Battery Calibration
 * @reboot_required true
 * @min 0
 * @unit Ohm
 */
PARAM_DEFINE_FLOAT(BAT_R_S_INIT, 0.1f);
/**
 * Battery internal resistance online estimator
 *
 * Initial estimate of ohmic resistor in RC network of battery (all cells) equivalent circuit model.
 *
 * @group Battery Calibration
 * @reboot_required true
 * @min 0
 * @unit Ohm
 */
PARAM_DEFINE_FLOAT(BAT_R_T_INIT, 0.05f);
/**
 * Battery internal resistance online estimator
 *
 * Initial estimate of capacitance (in farad, F) in RC network of battery (all cells) equivalent circuit model.
 *
 * @group Battery Calibration
 * @reboot_required true
 * @min 0
 */
PARAM_DEFINE_FLOAT(BAT_C_T_INIT, 500.0f);

/**
 * Battery internal resistance online estimator
 *
 * Max of per cell estimation clamp
 *
 * @group Battery Calibration
 * @reboot_required true
 * @unit Ohm
 */
PARAM_DEFINE_FLOAT(BAT_RIN_EST_MAX, 1.0f);

/**
 * Battery internal resistance online estimator
 *
 * Min of per cell estimation clamp
 *
 * @group Battery Calibration
 * @reboot_required true
 * @unit Ohm
 */
PARAM_DEFINE_FLOAT(BAT_RIN_EST_MIN, 0.001f);
/**
 * Battery internal resistance online estimator
 *
 * Initial per cell estimate of battery internal resistance.
 *
 * @group Battery Calibration
 * @reboot_required true
 * @unit Ohm
 */
PARAM_DEFINE_FLOAT(BAT_RIN_EST_INIT, 0.01f);
/**
 * Battery internal resistance online estimator
 *
 * Adaptation gain to update parameter vector of equivalent circuit model
 *
 * @group Battery Calibration
 * @reboot_required true
 * @unit Ohm
 */
PARAM_DEFINE_FLOAT(BAT_PARAM_GAIN, 0.001f);
/**
 * Battery internal resistance online estimator
 *
 * Initial per cell estimate of voltage load in battery equivalent circuit model.
 *
 * @group Battery Calibration
 * @reboot_required true
 * @min 0
 * @unit V
 */
PARAM_DEFINE_FLOAT(BAT_V_EST_INIT, 4.05f);
