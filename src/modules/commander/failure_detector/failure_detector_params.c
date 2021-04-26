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
 * @file failure_detector_params.c
 *
 * Parameters used by the Failure Detector.
 *
 * @author Mathieu Bresciani <brescianimathieu@gmail.com>
 */

#include <px4_platform_common/px4_config.h>
#include <parameters/param.h>

/**
 * FailureDetector Max Roll
 *
 * Maximum roll angle before FailureDetector triggers the attitude_failure flag.
 * The flag triggers flight termination (if @CBRK_FLIGHTTERM = 0),
 * which sets outputs to their failsafe values.
 * On takeoff the flag triggers lockdown (irrespective of @CBRK_FLIGHTTERM),
 * which disarms motors but does not set outputs to failsafe values.
 *
 * Setting this parameter to 0 disables the check
 *
 * @min 60
 * @max 180
 * @unit deg
 * @group Failure Detector
 */
PARAM_DEFINE_INT32(FD_FAIL_R, 60);

/**
 * FailureDetector Max Pitch
 *
 * Maximum pitch angle before FailureDetector triggers the attitude_failure flag.
 * The flag triggers flight termination (if @CBRK_FLIGHTTERM = 0),
 * which sets outputs to their failsafe values.
 * On takeoff the flag triggers lockdown (irrespective of @CBRK_FLIGHTTERM),
 * which disarms motors but does not set outputs to failsafe values.
 *
 * Setting this parameter to 0 disables the check
 *
 * @min 60
 * @max 180
 * @unit deg
 * @group Failure Detector
 */
PARAM_DEFINE_INT32(FD_FAIL_P, 60);

/**
 * Roll failure trigger time
 *
 * Seconds (decimal) that roll has to exceed FD_FAIL_R before being considered as a failure.
 *
 * @unit s
 * @min 0.02
 * @max 5
 * @decimal 2
 *
 * @group Failure Detector
 */
PARAM_DEFINE_FLOAT(FD_FAIL_R_TTRI, 0.3);

/**
 * Pitch failure trigger time
 *
 * Seconds (decimal) that pitch has to exceed FD_FAIL_P before being considered as a failure.
 *
 * @unit s
 * @min 0.02
 * @max 5
 * @decimal 2
 *
 * @group Failure Detector
 */
PARAM_DEFINE_FLOAT(FD_FAIL_P_TTRI, 0.3);

/**
 * Enable PWM input on for engaging failsafe from an external automatic trigger system (ATS).
 *
 * Enabled on either AUX5 or MAIN5 depending on board.
 * External ATS is required by ASTM F3322-18.
 *
 * @boolean
 * @reboot_required true
 * @group Failure Detector
 */
PARAM_DEFINE_INT32(FD_EXT_ATS_EN, 0);

/**
 * The PWM threshold from external automatic trigger system for engaging failsafe.
 *
 * External ATS is required by ASTM F3322-18.
 *
 * @unit us
 * @decimal 2
 *
 * @group Failure Detector
 */
PARAM_DEFINE_INT32(FD_EXT_ATS_TRIG, 1900);

/**
 * Enable checks on ESCs that report their arming state.
 *
 * If enabled, failure detector will verify that all the ESCs have successfully armed when the vehicle has transitioned to the armed state.
 * Timeout for receiving an acknowledgement from the ESCs is 0.3s, if no feedback is received the failure detector will auto disarm the vehicle.
 *
 * @boolean
 * @reboot_required true
 *
 * @group Failure Detector
 */
PARAM_DEFINE_INT32(FD_ESCS_EN, 1);
