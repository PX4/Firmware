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
 * @file battery_params.c
 *
 * Parameters defined by the battery lib, shared between all batteries.
 *
 * @author Julian Oes <julian@oes.ch>
 */

/**
 * Low threshold
 *
 * Sets the threshold when the battery will be reported as low.
 * This has to be higher than the critical threshold.
 *
 * @group Battery Calibration
 * @unit norm
 * @min 0.12
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 * @reboot_required true
 */
PARAM_DEFINE_FLOAT(BAT_LOW_THR, 0.15f);

/**
 * Critical threshold
 *
 * Sets the threshold when the battery will be reported as critically low.
 * This has to be lower than the low threshold. This threshold commonly
 * will trigger RTL.
 *
 * @group Battery Calibration
 * @unit norm
 * @min 0.05
 * @max 0.25
 * @decimal 2
 * @increment 0.01
 * @reboot_required true
 */
PARAM_DEFINE_FLOAT(BAT_CRIT_THR, 0.07f);

/**
 * Emergency threshold
 *
 * Sets the threshold when the battery will be reported as dangerously low.
 * This has to be lower than the critical threshold. This threshold commonly
 * will trigger landing.
 *
 * @group Battery Calibration
 * @unit norm
 * @min 0.03
 * @max 0.1
 * @decimal 2
 * @increment 0.01
 * @reboot_required true
 */
PARAM_DEFINE_FLOAT(BAT_EMERGEN_THR, 0.05f);
