/****************************************************************************
 *
 *   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
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
 * @file PreFlightCheck.cpp
 */

#include "PreFlightCheck.hpp"

#include <drivers/drv_hrt.h>
#include <HealthFlags.h>
#include <lib/parameters/param.h>
#include <systemlib/mavlink_log.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/subsystem_info.h>

using namespace time_literals;

static constexpr unsigned max_mandatory_gyro_count = 1;
static constexpr unsigned max_optional_gyro_count = 3;
static constexpr unsigned max_mandatory_accel_count = 1;
static constexpr unsigned max_optional_accel_count = 3;
static constexpr unsigned max_mandatory_mag_count = 1;
static constexpr unsigned max_optional_mag_count = 4;
static constexpr unsigned max_mandatory_baro_count = 1;
static constexpr unsigned max_optional_baro_count = 1;

bool PreFlightCheck::preflightCheck(orb_advert_t *mavlink_log_pub, vehicle_status_s &status,
				    vehicle_status_flags_s &status_flags, const bool checkGNSS, bool reportFailures, const bool prearm,
				    const hrt_abstime &time_since_boot)
{
	const bool hil_enabled = (status.hil_state == vehicle_status_s::HIL_STATE_ON);

	reportFailures = (reportFailures && status_flags.condition_system_hotplug_timeout
			  && !status_flags.condition_calibration_enabled);

	const bool checkSensors = !hil_enabled;
	const bool checkDynamic = !hil_enabled;

	bool checkAirspeed = false;

	/* Perform airspeed check only if circuit breaker is not
	 * engaged and it's not a rotary wing */
	if (!status_flags.circuit_breaker_engaged_airspd_check &&
	    (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING || status.is_vtol)) {
		checkAirspeed = true;
	}

	bool failed = false;

	failed = failed || !airframeCheck(mavlink_log_pub, status);

	/* ---- MAG ---- */
	if (checkSensors) {
		int32_t sys_has_mag = 1;
		param_get(param_find("SYS_HAS_MAG"), &sys_has_mag);

		if (sys_has_mag == 1) {

			/* check all sensors individually, but fail only for mandatory ones */
			for (unsigned i = 0; i < max_optional_mag_count; i++) {
				const bool required = (i < max_mandatory_mag_count) && (sys_has_mag == 1);
				const bool report_fail = (reportFailures);

				int32_t device_id = -1;

				if (!magnetometerCheck(mavlink_log_pub, status, i, !required, device_id, report_fail)) {
					if (required) {
						failed = true;
					}
				}
			}

			// TODO: highest priority mag

			/* mag consistency checks (need to be performed after the individual checks) */
			if (!magConsistencyCheck(mavlink_log_pub, status, (reportFailures))) {
				failed = true;
			}
		}
	}

	/* ---- ACCEL ---- */
	if (checkSensors) {
		/* check all sensors individually, but fail only for mandatory ones */
		for (unsigned i = 0; i < max_optional_accel_count; i++) {
			const bool required = (i < max_mandatory_accel_count);
			const bool report_fail = (reportFailures);

			int32_t device_id = -1;

			if (!accelerometerCheck(mavlink_log_pub, status, i, !required, checkDynamic, device_id, report_fail)) {
				if (required) {
					failed = true;
				}
			}
		}

		// TODO: highest priority (from params)
	}

	/* ---- GYRO ---- */
	if (checkSensors) {
		/* check all sensors individually, but fail only for mandatory ones */
		for (unsigned i = 0; i < max_optional_gyro_count; i++) {
			const bool required = (i < max_mandatory_gyro_count);
			int32_t device_id = -1;

			if (!gyroCheck(mavlink_log_pub, status, i, !required, device_id, reportFailures)) {
				if (required) {
					failed = true;
				}
			}
		}

		// TODO: highest priority (from params)
	}

	/* ---- BARO ---- */
	if (checkSensors) {
		int32_t sys_has_baro = 1;
		param_get(param_find("SYS_HAS_BARO"), &sys_has_baro);

		bool baro_fail_reported = false;

		/* check all sensors, but fail only for mandatory ones */
		for (unsigned i = 0; i < max_optional_baro_count; i++) {
			const bool required = (i < max_mandatory_baro_count) && (sys_has_baro == 1);
			const bool report_fail = (reportFailures && !baro_fail_reported);

			int32_t device_id = -1;

			if (!baroCheck(mavlink_log_pub, status, i, !required, device_id, report_fail)) {
				if (required) {
					baro_fail_reported = true;
				}
			}
		}
	}

	/* ---- IMU CONSISTENCY ---- */
	// To be performed after the individual sensor checks have completed
	if (checkSensors) {
		if (!imuConsistencyCheck(mavlink_log_pub, status, reportFailures)) {
			failed = true;
		}
	}

	/* ---- AIRSPEED ---- */
	if (checkAirspeed) {
		int32_t optional = 0;
		param_get(param_find("FW_ARSP_MODE"), &optional);

		if (!airspeedCheck(mavlink_log_pub, status, (bool)optional, reportFailures, prearm) && !(bool)optional) {
			failed = true;
		}
	}

	/* ---- RC CALIBRATION ---- */
	if (status.rc_input_mode == vehicle_status_s::RC_IN_MODE_DEFAULT) {
		if (rcCalibrationCheck(mavlink_log_pub, reportFailures, status.is_vtol) != OK) {
			if (reportFailures) {
				mavlink_log_critical(mavlink_log_pub, "RC calibration check failed");
			}

			failed = true;

			set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_RCRECEIVER, status_flags.rc_signal_found_once, true, false, status);
			status_flags.rc_calibration_valid = false;

		} else {
			// The calibration is fine, but only set the overall health state to true if the signal is not currently lost
			status_flags.rc_calibration_valid = true;
			set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_RCRECEIVER, status_flags.rc_signal_found_once, true,
					 !status.rc_signal_lost, status);
		}
	}

	/* ---- SYSTEM POWER ---- */
	if (status_flags.condition_power_input_valid && !status_flags.circuit_breaker_engaged_power_check) {
		if (!powerCheck(mavlink_log_pub, status, reportFailures, prearm)) {
			failed = true;
		}
	}

	/* ---- Navigation EKF ---- */
	// only check EKF2 data if EKF2 is selected as the estimator and GNSS checking is enabled
	int32_t estimator_type = -1;

	if (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && !status.is_vtol) {
		param_get(param_find("SYS_MC_EST_GROUP"), &estimator_type);

	} else {
		// EKF2 is currently the only supported option for FW & VTOL
		estimator_type = 2;
	}

	if (estimator_type == 2) {
		// don't report ekf failures for the first 10 seconds to allow time for the filter to start
		bool report_ekf_fail = (time_since_boot > 10_s);

		if (!ekf2Check(mavlink_log_pub, status, false, reportFailures && report_ekf_fail, checkGNSS)) {
			failed = true;
		}
	}

	/* ---- Failure Detector ---- */
	if (!failureDetectorCheck(mavlink_log_pub, status, reportFailures, prearm)) {
		failed = true;
	}

	failed = failed || !manualControlCheck(mavlink_log_pub, reportFailures);
	failed = failed || !cpuResourceCheck(mavlink_log_pub, reportFailures);

	/* Report status */
	return !failed;
}
