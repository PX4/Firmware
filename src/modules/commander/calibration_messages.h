/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file calibration_messages.h
 *
 * Common calibration messages.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#ifndef CALIBRATION_MESSAGES_H_
#define CALIBRATION_MESSAGES_H_

// The calibration message defines which begin with CAL_QGC_ are used by QGroundControl to run a state
// machine to provide visual feedback for calibration. As such, the text for them or semantics of when
// they are displayed cannot be modified without causing QGC to break. If modifications are made, make
// sure to bump the calibration version number which will cause QGC to perform log based calibration
// instead of visual calibration until such a time as QGC is update to the new version.

// The number in the cal started message is used to indicate the version stamp for the current calibration code.
#define CAL_QGC_STARTED_MSG			"[cal] calibration started: 2 %s"
#define CAL_QGC_DONE_MSG			"[cal] calibration done: %s"
#define CAL_QGC_FAILED_MSG			"[cal] calibration failed: %s"
#define CAL_QGC_CANCELLED_MSG			"[cal] calibration cancelled"
#define CAL_QGC_PROGRESS_MSG			"[cal] progress <%u>"
#define CAL_QGC_ORIENTATION_DETECTED_MSG	"[cal] %s orientation detected"
#define CAL_QGC_SIDE_DONE_MSG			"[cal] %s side done, rotate to a different side"

#define CAL_ERROR_SENSOR_MSG		"[cal] calibration failed: reading sensor"
#define CAL_ERROR_RESET_CAL_MSG		"[cal] calibration failed: to reset, sensor %u"
#define CAL_ERROR_READ_CAL_MSG		"[cal] calibration failed: to read calibration"
#define CAL_ERROR_APPLY_CAL_MSG		"[cal] calibration failed: to apply calibration"
#define CAL_ERROR_SET_PARAMS_MSG	"[cal] calibration failed: to set parameters"

#endif /* CALIBRATION_MESSAGES_H_ */
