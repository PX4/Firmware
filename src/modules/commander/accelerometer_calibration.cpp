/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @file accelerometer_calibration.cpp
 *
 * Implementation of accelerometer calibration.
 *
 * Transform acceleration vector to true orientation, scale and offset
 *
 * ===== Model =====
 * accel_corr = accel_T * (accel_raw - accel_offs)
 *
 * accel_corr[3] - fully corrected acceleration vector in body frame
 * accel_T[3][3] - accelerometers transform matrix, rotation and scaling transform
 * accel_raw[3]  - raw acceleration vector
 * accel_offs[3] - acceleration offset vector
 *
 * ===== Calibration =====
 *
 * Reference vectors
 * accel_corr_ref[6][3] = [  g  0  0 ]     // nose up
 *                        | -g  0  0 |     // nose down
 *                        |  0  g  0 |     // left side down
 *                        |  0 -g  0 |     // right side down
 *                        |  0  0  g |     // on back
 *                        [  0  0 -g ]     // level
 * accel_raw_ref[6][3]
 *
 * accel_corr_ref[i] = accel_T * (accel_raw_ref[i] - accel_offs), i = 0...5
 *
 * 6 reference vectors * 3 axes = 18 equations
 * 9 (accel_T) + 3 (accel_offs) = 12 unknown constants
 *
 * Find accel_offs
 *
 * accel_offs[i] = (accel_raw_ref[i*2][i] + accel_raw_ref[i*2+1][i]) / 2
 *
 * Find accel_T
 *
 * 9 unknown constants
 * need 9 equations -> use 3 of 6 measurements -> 3 * 3 = 9 equations
 *
 * accel_corr_ref[i*2] = accel_T * (accel_raw_ref[i*2] - accel_offs), i = 0...2
 *
 * Solve separate system for each row of accel_T:
 *
 * accel_corr_ref[j*2][i] = accel_T[i] * (accel_raw_ref[j*2] - accel_offs), j = 0...2
 *
 * A * x = b
 *
 * x = [ accel_T[0][i] ]
 *     | accel_T[1][i] |
 *     [ accel_T[2][i] ]
 *
 * b = [ accel_corr_ref[0][i] ]	// One measurement per side is enough
 *     | accel_corr_ref[2][i] |
 *     [ accel_corr_ref[4][i] ]
 *
 * a[i][j] = accel_raw_ref[i][j] - accel_offs[j], i = 0;2;4, j = 0...2
 *
 * Matrix A is common for all three systems:
 * A = [ a[0][0]  a[0][1]  a[0][2] ]
 *     | a[2][0]  a[2][1]  a[2][2] |
 *     [ a[4][0]  a[4][1]  a[4][2] ]
 *
 * x = A^-1 * b
 *
 * accel_T = A^-1 * g
 * g = 9.80665
 *
 * ===== Rotation =====
 *
 * Calibrating using model:
 * accel_corr = accel_T_r * (rot * accel_raw - accel_offs_r)
 *
 * Actual correction:
 * accel_corr = rot * accel_T * (accel_raw - accel_offs)
 *
 * Known: accel_T_r, accel_offs_r, rot
 * Unknown: accel_T, accel_offs
 *
 * Solution:
 * accel_T_r * (rot * accel_raw - accel_offs_r) = rot * accel_T * (accel_raw - accel_offs)
 * rot^-1 * accel_T_r * (rot * accel_raw - accel_offs_r) = accel_T * (accel_raw - accel_offs)
 * rot^-1 * accel_T_r * rot * accel_raw - rot^-1 * accel_T_r * accel_offs_r = accel_T * accel_raw - accel_T * accel_offs)
 * => accel_T = rot^-1 * accel_T_r * rot
 * => accel_offs = rot^-1 * accel_offs_r
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

// FIXME: Can some of these headers move out with detect_ move?

#include "accelerometer_calibration.h"
#include "calibration_messages.h"
#include "calibration_routines.h"
#include "commander_helper.h"

#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/time.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <math.h>
#include <float.h>
#include <mathlib/mathlib.h>
#include <string.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <lib/ecl/geo/geo.h>
#include <matrix/math.hpp>
#include <conversion/rotation.h>
#include <parameters/param.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_correction.h>
#include <uORB/Subscription.hpp>

using namespace time_literals;
using namespace matrix;

static const char *sensor_name = "accel";

static int32_t device_id[max_accel_sens];
static int device_prio_max = 0;
static int32_t device_id_primary = 0;

calibrate_return do_accel_calibration_measurements(orb_advert_t *mavlink_log_pub,
		float (&accel_offs)[max_accel_sens][3], float (&accel_T)[max_accel_sens][3][3], unsigned *active_sensors);
calibrate_return read_accelerometer_avg(int sensor_correction_sub, int (&subs)[max_accel_sens],
					float (&accel_avg)[max_accel_sens][detect_orientation_side_count][3], unsigned orient, unsigned samples_num);
int mat_invert3(float src[3][3], float dst[3][3]);
calibrate_return calculate_calibration_values(unsigned sensor,
		float (&accel_ref)[max_accel_sens][detect_orientation_side_count][3], float (&accel_T)[max_accel_sens][3][3],
		float (&accel_offs)[max_accel_sens][3], float g);

/// Data passed to calibration worker routine
typedef struct  {
	orb_advert_t	*mavlink_log_pub;
	unsigned	done_count;
	int		subs[max_accel_sens];
	float		accel_ref[max_accel_sens][detect_orientation_side_count][3];
	int		sensor_correction_sub;
} accel_worker_data_t;

int do_accel_calibration(orb_advert_t *mavlink_log_pub)
{
#if 1 // TODO: replace all IOCTL usage
	int fd;
#endif

	calibration_log_info(mavlink_log_pub, CAL_QGC_STARTED_MSG, sensor_name);

	struct accel_calibration_s accel_scale;
	accel_scale.x_offset = 0.0f;
	accel_scale.x_scale = 1.0f;
	accel_scale.y_offset = 0.0f;
	accel_scale.y_scale = 1.0f;
	accel_scale.z_offset = 0.0f;
	accel_scale.z_scale = 1.0f;

	int res = PX4_OK;

	char str[30];

	/* reset all sensors */
	for (unsigned s = 0; s < max_accel_sens; s++) {
#if 1 // TODO: replace all IOCTL usage
		sprintf(str, "%s%u", ACCEL_BASE_DEVICE_PATH, s);
		/* reset all offsets to zero and all scales to one */
		fd = px4_open(str, 0);

		if (fd < 0) {
			continue;
		}

		device_id[s] = px4_ioctl(fd, DEVIOCGDEVICEID, 0);

		res = px4_ioctl(fd, ACCELIOCSSCALE, (long unsigned int)&accel_scale);
		px4_close(fd);

		if (res != PX4_OK) {
			calibration_log_critical(mavlink_log_pub, CAL_ERROR_RESET_CAL_MSG, s);
		}

#else
		(void)sprintf(str, "CAL_ACC%u_XOFF", s);
		res = param_set_no_notification(param_find(str), &accel_scale.x_offset);

		if (res != PX4_OK) {
			PX4_ERR("unable to reset %s", str);
		}

		(void)sprintf(str, "CAL_ACC%u_YOFF", s);
		res = param_set_no_notification(param_find(str), &accel_scale.y_offset);

		if (res != PX4_OK) {
			PX4_ERR("unable to reset %s", str);
		}

		(void)sprintf(str, "CAL_ACC%u_ZOFF", s);
		res = param_set_no_notification(param_find(str), &accel_scale.z_offset);

		if (res != PX4_OK) {
			PX4_ERR("unable to reset %s", str);
		}

		(void)sprintf(str, "CAL_ACC%u_XSCALE", s);
		res = param_set_no_notification(param_find(str), &accel_scale.x_scale);

		if (res != PX4_OK) {
			PX4_ERR("unable to reset %s", str);
		}

		(void)sprintf(str, "CAL_ACC%u_YSCALE", s);
		res = param_set_no_notification(param_find(str), &accel_scale.y_scale);

		if (res != PX4_OK) {
			PX4_ERR("unable to reset %s", str);
		}

		(void)sprintf(str, "CAL_ACC%u_ZSCALE", s);
		res = param_set_no_notification(param_find(str), &accel_scale.z_scale);

		if (res != PX4_OK) {
			PX4_ERR("unable to reset %s", str);
		}

		param_notify_changes();
#endif
	}

	float accel_offs[max_accel_sens][3];
	float accel_T[max_accel_sens][3][3];
	unsigned active_sensors = 0;

	/* measure and calculate offsets & scales */
	if (res == PX4_OK) {
		calibrate_return cal_return = do_accel_calibration_measurements(mavlink_log_pub, accel_offs, accel_T, &active_sensors);

		if (cal_return == calibrate_return_cancelled) {
			// Cancel message already displayed, nothing left to do
			return PX4_ERROR;

		} else if (cal_return == calibrate_return_ok) {
			res = PX4_OK;

		} else {
			res = PX4_ERROR;
		}
	}

	if (res != PX4_OK) {
		if (active_sensors == 0) {
			calibration_log_critical(mavlink_log_pub, CAL_ERROR_SENSOR_MSG);
		}

		return PX4_ERROR;
	}

	/* measurements completed successfully, rotate calibration values */
	param_t board_rotation_h = param_find("SENS_BOARD_ROT");
	int32_t board_rotation_int;
	param_get(board_rotation_h, &(board_rotation_int));
	enum Rotation board_rotation_id = (enum Rotation)board_rotation_int;
	Dcmf board_rotation = get_rot_matrix(board_rotation_id);

	Dcmf board_rotation_t = board_rotation.transpose();

	bool tc_locked[3] = {false}; // true when the thermal parameter instance has already been adjusted by the calibrator

	for (unsigned uorb_index = 0; uorb_index < active_sensors; uorb_index++) {

		/* handle individual sensors, one by one */
		Vector3f accel_offs_vec(accel_offs[uorb_index]);
		Vector3f accel_offs_rotated = board_rotation_t *accel_offs_vec;
		Matrix3f accel_T_mat(accel_T[uorb_index]);
		Matrix3f accel_T_rotated = board_rotation_t *accel_T_mat * board_rotation;

		accel_scale.x_offset = accel_offs_rotated(0);
		accel_scale.x_scale = accel_T_rotated(0, 0);
		accel_scale.y_offset = accel_offs_rotated(1);
		accel_scale.y_scale = accel_T_rotated(1, 1);
		accel_scale.z_offset = accel_offs_rotated(2);
		accel_scale.z_scale = accel_T_rotated(2, 2);

		bool failed = false;

		failed = failed || (PX4_OK != param_set_no_notification(param_find("CAL_ACC_PRIME"), &(device_id_primary)));


		PX4_INFO("found offset %d: x: %.6f, y: %.6f, z: %.6f", uorb_index,
			 (double)accel_scale.x_offset,
			 (double)accel_scale.y_offset,
			 (double)accel_scale.z_offset);
		PX4_INFO("found scale %d: x: %.6f, y: %.6f, z: %.6f", uorb_index,
			 (double)accel_scale.x_scale,
			 (double)accel_scale.y_scale,
			 (double)accel_scale.z_scale);

		/* check if thermal compensation is enabled */
		int32_t tc_enabled_int;
		param_get(param_find("TC_A_ENABLE"), &(tc_enabled_int));

		if (tc_enabled_int == 1) {
			/* Get struct containing sensor thermal compensation data */
			sensor_correction_s sensor_correction{}; /**< sensor thermal corrections */
			uORB::Subscription sensor_correction_sub{ORB_ID(sensor_correction)};
			sensor_correction_sub.copy(&sensor_correction);

			/* don't allow a parameter instance to be calibrated more than once by another uORB instance */
			if (!tc_locked[sensor_correction.accel_mapping[uorb_index]]) {
				tc_locked[sensor_correction.accel_mapping[uorb_index]] = true;

				/* update the _X0_ terms to include the additional offset */
				int32_t handle;
				float val;

				for (unsigned axis_index = 0; axis_index < 3; axis_index++) {
					val = 0.0f;
					(void)sprintf(str, "TC_A%u_X0_%u", sensor_correction.accel_mapping[uorb_index], axis_index);
					handle = param_find(str);
					param_get(handle, &val);

					if (axis_index == 0) {
						val += accel_scale.x_offset;

					} else if (axis_index == 1) {
						val += accel_scale.y_offset;

					} else if (axis_index == 2) {
						val += accel_scale.z_offset;
					}

					failed |= (PX4_OK != param_set_no_notification(handle, &val));
				}

				/* update the _SCL_ terms to include the scale factor */
				for (unsigned axis_index = 0; axis_index < 3; axis_index++) {
					val = 1.0f;
					(void)sprintf(str, "TC_A%u_SCL_%u", sensor_correction.accel_mapping[uorb_index], axis_index);
					handle = param_find(str);

					if (axis_index == 0) {
						val = accel_scale.x_scale;

					} else if (axis_index == 1) {
						val = accel_scale.y_scale;

					} else if (axis_index == 2) {
						val = accel_scale.z_scale;
					}

					failed |= (PX4_OK != param_set_no_notification(handle, &val));
				}

				param_notify_changes();
			}

			// Ensure the calibration values used by the driver are at default settings when we are using thermal calibration data
			accel_scale.x_offset = 0.f;
			accel_scale.y_offset = 0.f;
			accel_scale.z_offset = 0.f;
			accel_scale.x_scale = 1.f;
			accel_scale.y_scale = 1.f;
			accel_scale.z_scale = 1.f;
		}

		// save the driver level calibration data
		(void)sprintf(str, "CAL_ACC%u_XOFF", uorb_index);
		failed |= (PX4_OK != param_set_no_notification(param_find(str), &(accel_scale.x_offset)));
		(void)sprintf(str, "CAL_ACC%u_YOFF", uorb_index);
		failed |= (PX4_OK != param_set_no_notification(param_find(str), &(accel_scale.y_offset)));
		(void)sprintf(str, "CAL_ACC%u_ZOFF", uorb_index);
		failed |= (PX4_OK != param_set_no_notification(param_find(str), &(accel_scale.z_offset)));
		(void)sprintf(str, "CAL_ACC%u_XSCALE", uorb_index);
		failed |= (PX4_OK != param_set_no_notification(param_find(str), &(accel_scale.x_scale)));
		(void)sprintf(str, "CAL_ACC%u_YSCALE", uorb_index);
		failed |= (PX4_OK != param_set_no_notification(param_find(str), &(accel_scale.y_scale)));
		(void)sprintf(str, "CAL_ACC%u_ZSCALE", uorb_index);
		failed |= (PX4_OK != param_set_no_notification(param_find(str), &(accel_scale.z_scale)));
		(void)sprintf(str, "CAL_ACC%u_ID", uorb_index);
		failed |= (PX4_OK != param_set_no_notification(param_find(str), &(device_id[uorb_index])));

		if (failed) {
			calibration_log_critical(mavlink_log_pub, CAL_ERROR_SET_PARAMS_MSG);
			return PX4_ERROR;
		}

#if 1 // TODO: replace all IOCTL usage
		sprintf(str, "%s%u", ACCEL_BASE_DEVICE_PATH, uorb_index);
		fd = px4_open(str, 0);

		if (fd < 0) {
			calibration_log_critical(mavlink_log_pub, CAL_QGC_FAILED_MSG, "sensor does not exist");
			res = PX4_ERROR;

		} else {
			res = px4_ioctl(fd, ACCELIOCSSCALE, (long unsigned int)&accel_scale);
			px4_close(fd);
		}

		if (res != PX4_OK) {
			calibration_log_critical(mavlink_log_pub, CAL_ERROR_APPLY_CAL_MSG);
		}

#endif
	}

	if (res == PX4_OK) {
		/* if there is a any preflight-check system response, let the barrage of messages through */
		px4_usleep(200000);

		calibration_log_info(mavlink_log_pub, CAL_QGC_DONE_MSG, sensor_name);

	} else {
		calibration_log_critical(mavlink_log_pub, CAL_QGC_FAILED_MSG, sensor_name);
	}

	/* give this message enough time to propagate */
	px4_usleep(600000);

	return res;
}

static calibrate_return accel_calibration_worker(detect_orientation_return orientation, int cancel_sub, void *data)
{
	const unsigned samples_num = 750;
	accel_worker_data_t *worker_data = (accel_worker_data_t *)(data);

	calibration_log_info(worker_data->mavlink_log_pub, "[cal] Hold still, measuring %s side",
			     detect_orientation_str(orientation));

	read_accelerometer_avg(worker_data->sensor_correction_sub, worker_data->subs, worker_data->accel_ref, orientation,
			       samples_num);

	calibration_log_info(worker_data->mavlink_log_pub, "[cal] %s side result: [%8.4f %8.4f %8.4f]",
			     detect_orientation_str(orientation),
			     (double)worker_data->accel_ref[0][orientation][0],
			     (double)worker_data->accel_ref[0][orientation][1],
			     (double)worker_data->accel_ref[0][orientation][2]);

	worker_data->done_count++;
	calibration_log_info(worker_data->mavlink_log_pub, CAL_QGC_PROGRESS_MSG, 17 * worker_data->done_count);

	return calibrate_return_ok;
}

calibrate_return do_accel_calibration_measurements(orb_advert_t *mavlink_log_pub,
		float (&accel_offs)[max_accel_sens][3], float (&accel_T)[max_accel_sens][3][3], unsigned *active_sensors)
{
	calibrate_return result = calibrate_return_ok;

	*active_sensors = 0;

	accel_worker_data_t worker_data;

	worker_data.mavlink_log_pub = mavlink_log_pub;
	worker_data.done_count = 0;

	bool data_collected[detect_orientation_side_count] = { false, false, false, false, false, false };

	// Initialise sub to sensor thermal compensation data
	worker_data.sensor_correction_sub = orb_subscribe(ORB_ID(sensor_correction));

	// Initialize subs to error condition so we know which ones are open and which are not
	for (size_t i = 0; i < max_accel_sens; i++) {
		worker_data.subs[i] = -1;
	}

	uint64_t timestamps[max_accel_sens] = {};

	// We should not try to subscribe if the topic doesn't actually exist and can be counted.
	const unsigned orb_accel_count = orb_group_count(ORB_ID(sensor_accel));

	// Warn that we will not calibrate more than max_accels accelerometers
	if (orb_accel_count > max_accel_sens) {
		calibration_log_critical(mavlink_log_pub, "Detected %u accels, but will calibrate only %u", orb_accel_count,
					 max_accel_sens);
	}

	for (unsigned cur_accel = 0; cur_accel < orb_accel_count && cur_accel < max_accel_sens; cur_accel++) {

		// Lock in to correct ORB instance
		bool found_cur_accel = false;

		for (unsigned i = 0; i < orb_accel_count && !found_cur_accel; i++) {
			worker_data.subs[cur_accel] = orb_subscribe_multi(ORB_ID(sensor_accel), i);

			sensor_accel_s report = {};
			orb_copy(ORB_ID(sensor_accel), worker_data.subs[cur_accel], &report);

#if 1 // TODO: replace all IOCTL usage

			// For NuttX, we get the UNIQUE device ID from the sensor driver via an IOCTL
			// and match it up with the one from the uORB subscription, because the
			// instance ordering of uORB and the order of the FDs may not be the same.

			if (report.device_id == (uint32_t)device_id[cur_accel]) {
				// Device IDs match, correct ORB instance for this accel
				found_cur_accel = true;
				// store initial timestamp - used to infer which sensors are active
				timestamps[cur_accel] = report.timestamp;

			} else {
				orb_unsubscribe(worker_data.subs[cur_accel]);
			}

#else

			// For the DriverFramework drivers, we fill device ID (this is the first time) by copying one report.
			device_id[cur_accel] = report.device_id;
			found_cur_accel = true;

#endif
		}

		if (!found_cur_accel) {
			calibration_log_critical(mavlink_log_pub, "Accel #%u (ID %u) no matching uORB devid", cur_accel, device_id[cur_accel]);
			result = calibrate_return_error;
			break;
		}

		if (device_id[cur_accel] != 0) {
			// Get priority
			int32_t prio;
			orb_priority(worker_data.subs[cur_accel], &prio);

			if (prio > device_prio_max) {
				device_prio_max = prio;
				device_id_primary = device_id[cur_accel];
			}

		} else {
			calibration_log_critical(mavlink_log_pub, "Accel #%u no device id, abort", cur_accel);
			result = calibrate_return_error;
			break;
		}
	}

	if (result == calibrate_return_ok) {
		int cancel_sub = calibrate_cancel_subscribe();
		result = calibrate_from_orientation(mavlink_log_pub, cancel_sub, data_collected, accel_calibration_worker, &worker_data,
						    false /* normal still */);
		calibrate_cancel_unsubscribe(cancel_sub);
	}

	/* close all subscriptions */
	for (unsigned i = 0; i < max_accel_sens; i++) {
		if (worker_data.subs[i] >= 0) {
			/* figure out which sensors were active */
			sensor_accel_s arp = {};
			(void)orb_copy(ORB_ID(sensor_accel), worker_data.subs[i], &arp);

			if (arp.timestamp != 0 && timestamps[i] != arp.timestamp) {
				(*active_sensors)++;
			}

			px4_close(worker_data.subs[i]);
		}
	}

	orb_unsubscribe(worker_data.sensor_correction_sub);

	if (result == calibrate_return_ok) {
		/* calculate offsets and transform matrix */
		for (unsigned i = 0; i < (*active_sensors); i++) {
			result = calculate_calibration_values(i, worker_data.accel_ref, accel_T, accel_offs, CONSTANTS_ONE_G);

			if (result != calibrate_return_ok) {
				calibration_log_critical(mavlink_log_pub, "ERROR: calibration calculation error");
				break;
			}
		}
	}

	return result;
}

/*
 * Read specified number of accelerometer samples, calculate average and dispersion.
 */
calibrate_return read_accelerometer_avg(int sensor_correction_sub, int (&subs)[max_accel_sens],
					float (&accel_avg)[max_accel_sens][detect_orientation_side_count][3], unsigned orient, unsigned samples_num)
{
	/* get total sensor board rotation matrix */
	param_t board_rotation_h = param_find("SENS_BOARD_ROT");
	param_t board_offset_x = param_find("SENS_BOARD_X_OFF");
	param_t board_offset_y = param_find("SENS_BOARD_Y_OFF");
	param_t board_offset_z = param_find("SENS_BOARD_Z_OFF");

	float board_offset[3];
	param_get(board_offset_x, &board_offset[0]);
	param_get(board_offset_y, &board_offset[1]);
	param_get(board_offset_z, &board_offset[2]);

	Dcmf board_rotation_offset = Eulerf(
					     M_DEG_TO_RAD_F * board_offset[0],
					     M_DEG_TO_RAD_F * board_offset[1],
					     M_DEG_TO_RAD_F * board_offset[2]);

	int32_t board_rotation_int;
	param_get(board_rotation_h, &(board_rotation_int));

	Dcmf board_rotation = board_rotation_offset * get_rot_matrix((enum Rotation)board_rotation_int);

	px4_pollfd_struct_t fds[max_accel_sens];

	for (unsigned i = 0; i < max_accel_sens; i++) {
		fds[i].fd = subs[i];
		fds[i].events = POLLIN;
	}

	unsigned counts[max_accel_sens] = { 0 };
	float accel_sum[max_accel_sens][3] {};

	unsigned errcount = 0;
	struct sensor_correction_s sensor_correction; /**< sensor thermal corrections */

	/* try to get latest thermal corrections */
	if (orb_copy(ORB_ID(sensor_correction), sensor_correction_sub, &sensor_correction) != 0) {
		/* use default values */
		memset(&sensor_correction, 0, sizeof(sensor_correction));

		for (unsigned i = 0; i < 3; i++) {
			sensor_correction.accel_scale_0[i] = 1.0f;
			sensor_correction.accel_scale_1[i] = 1.0f;
			sensor_correction.accel_scale_2[i] = 1.0f;
		}
	}

	/* use the first sensor to pace the readout, but do per-sensor counts */
	while (counts[0] < samples_num) {
		int poll_ret = px4_poll(&fds[0], max_accel_sens, 1000);

		if (poll_ret > 0) {

			for (unsigned s = 0; s < max_accel_sens; s++) {
				bool changed;
				orb_check(subs[s], &changed);

				if (changed) {

					sensor_accel_s arp;
					orb_copy(ORB_ID(sensor_accel), subs[s], &arp);

					// Apply thermal offset corrections in sensor/board frame
					if (s == 0) {
						accel_sum[s][0] += (arp.x - sensor_correction.accel_offset_0[0]);
						accel_sum[s][1] += (arp.y - sensor_correction.accel_offset_0[1]);
						accel_sum[s][2] += (arp.z - sensor_correction.accel_offset_0[2]);

					} else if (s == 1) {
						accel_sum[s][0] += (arp.x - sensor_correction.accel_offset_1[0]);
						accel_sum[s][1] += (arp.y - sensor_correction.accel_offset_1[1]);
						accel_sum[s][2] += (arp.z - sensor_correction.accel_offset_1[2]);

					} else if (s == 2) {
						accel_sum[s][0] += (arp.x - sensor_correction.accel_offset_2[0]);
						accel_sum[s][1] += (arp.y - sensor_correction.accel_offset_2[1]);
						accel_sum[s][2] += (arp.z - sensor_correction.accel_offset_2[2]);

					} else {
						accel_sum[s][0] += arp.x;
						accel_sum[s][1] += arp.y;
						accel_sum[s][2] += arp.z;
					}

					counts[s]++;
				}
			}

		} else {
			errcount++;
			continue;
		}

		if (errcount > samples_num / 10) {
			return calibrate_return_error;
		}
	}

	// rotate sensor measurements from sensor to body frame using board rotation matrix
	for (unsigned i = 0; i < max_accel_sens; i++) {
		Vector3f accel_sum_vec(&accel_sum[i][0]);
		accel_sum_vec = board_rotation * accel_sum_vec;

		for (size_t j = 0; j < 3; j++) {
			accel_sum[i][j] = accel_sum_vec(j);
		}
	}

	for (unsigned s = 0; s < max_accel_sens; s++) {
		for (unsigned i = 0; i < 3; i++) {
			accel_avg[s][orient][i] = accel_sum[s][i] / counts[s];
		}
	}

	return calibrate_return_ok;
}

int mat_invert3(float src[3][3], float dst[3][3])
{
	float det = src[0][0] * (src[1][1] * src[2][2] - src[1][2] * src[2][1]) -
		    src[0][1] * (src[1][0] * src[2][2] - src[1][2] * src[2][0]) +
		    src[0][2] * (src[1][0] * src[2][1] - src[1][1] * src[2][0]);

	if (fabsf(det) < FLT_EPSILON) {
		return PX4_ERROR;        // Singular matrix
	}

	dst[0][0] = (src[1][1] * src[2][2] - src[1][2] * src[2][1]) / det;
	dst[1][0] = (src[1][2] * src[2][0] - src[1][0] * src[2][2]) / det;
	dst[2][0] = (src[1][0] * src[2][1] - src[1][1] * src[2][0]) / det;
	dst[0][1] = (src[0][2] * src[2][1] - src[0][1] * src[2][2]) / det;
	dst[1][1] = (src[0][0] * src[2][2] - src[0][2] * src[2][0]) / det;
	dst[2][1] = (src[0][1] * src[2][0] - src[0][0] * src[2][1]) / det;
	dst[0][2] = (src[0][1] * src[1][2] - src[0][2] * src[1][1]) / det;
	dst[1][2] = (src[0][2] * src[1][0] - src[0][0] * src[1][2]) / det;
	dst[2][2] = (src[0][0] * src[1][1] - src[0][1] * src[1][0]) / det;

	return PX4_OK;
}

calibrate_return calculate_calibration_values(unsigned sensor,
		float (&accel_ref)[max_accel_sens][detect_orientation_side_count][3], float (&accel_T)[max_accel_sens][3][3],
		float (&accel_offs)[max_accel_sens][3], float g)
{
	/* calculate offsets */
	for (unsigned i = 0; i < 3; i++) {
		accel_offs[sensor][i] = (accel_ref[sensor][i * 2][i] + accel_ref[sensor][i * 2 + 1][i]) / 2;
	}

	/* fill matrix A for linear equations system*/
	float mat_A[3][3];
	memset(mat_A, 0, sizeof(mat_A));

	for (unsigned i = 0; i < 3; i++) {
		for (unsigned j = 0; j < 3; j++) {
			float a = accel_ref[sensor][i * 2][j] - accel_offs[sensor][j];
			mat_A[i][j] = a;
		}
	}

	/* calculate inverse matrix for A */
	float mat_A_inv[3][3];

	if (mat_invert3(mat_A, mat_A_inv) != PX4_OK) {
		return calibrate_return_error;
	}

	/* copy results to accel_T */
	for (unsigned i = 0; i < 3; i++) {
		for (unsigned j = 0; j < 3; j++) {
			/* simplify matrices mult because b has only one non-zero element == g at index i */
			accel_T[sensor][j][i] = mat_A_inv[j][i] * g;
		}
	}

	return calibrate_return_ok;
}

int do_level_calibration(orb_advert_t *mavlink_log_pub)
{
	bool success = false;
	int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	vehicle_attitude_s att{};

	calibration_log_info(mavlink_log_pub, CAL_QGC_STARTED_MSG, "level");

	param_t roll_offset_handle = param_find("SENS_BOARD_X_OFF");
	param_t pitch_offset_handle = param_find("SENS_BOARD_Y_OFF");
	param_t board_rot_handle = param_find("SENS_BOARD_ROT");

	// get old values
	float roll_offset_current;
	float pitch_offset_current;
	int32_t board_rot_current = 0;
	param_get(roll_offset_handle, &roll_offset_current);
	param_get(pitch_offset_handle, &pitch_offset_current);
	param_get(board_rot_handle, &board_rot_current);

	Dcmf board_rotation_offset = Eulerf(
					     math::radians(roll_offset_current),
					     math::radians(pitch_offset_current),
					     0.f);

	px4_pollfd_struct_t fds[1];
	fds[0].fd = att_sub;
	fds[0].events = POLLIN;

	float roll_mean = 0.0f;
	float pitch_mean = 0.0f;
	unsigned counter = 0;
	bool had_motion = true;
	int num_retries = 0;

	while (had_motion && num_retries++ < 50) {
		Vector2f min_angles{100.f, 100.f};
		Vector2f max_angles{-100.f, -100.f};
		roll_mean = 0.0f;
		pitch_mean = 0.0f;
		counter = 0;
		int last_progress_report = -100;
		const hrt_abstime calibration_duration = 500_ms;
		const hrt_abstime start = hrt_absolute_time();

		while (hrt_elapsed_time(&start) < calibration_duration) {
			int pollret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

			if (pollret <= 0) {
				// attitude estimator is not running
				calibration_log_critical(mavlink_log_pub, "attitude estimator not running - check system boot");
				calibration_log_critical(mavlink_log_pub, CAL_QGC_FAILED_MSG, "level");
				goto out;
			}

			int progress = 100 * hrt_elapsed_time(&start) / calibration_duration;

			if (progress >= last_progress_report + 20) {
				calibration_log_info(mavlink_log_pub, CAL_QGC_PROGRESS_MSG, progress);
				last_progress_report = progress;
			}

			orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);
			Eulerf att_euler = Quatf(att.q);

			// keep min + max angles
			for (int i = 0; i < 2; ++i) {
				if (att_euler(i) < min_angles(i)) { min_angles(i) = att_euler(i); }

				if (att_euler(i) > max_angles(i)) { max_angles(i) = att_euler(i); }
			}

			att_euler(2) = 0.f; // ignore yaw
			att_euler = Eulerf(board_rotation_offset * Dcmf(att_euler)); // subtract existing board rotation
			roll_mean += att_euler.phi();
			pitch_mean += att_euler.theta();
			++counter;
		}

		// motion detection: check that (max-min angle) is within a threshold.
		// The difference is typically <0.1 deg while at rest
		if (max_angles(0) - min_angles(0) < math::radians(0.5f) &&
		    max_angles(1) - min_angles(1) < math::radians(0.5f)) {
			had_motion = false;
		}
	}

	calibration_log_info(mavlink_log_pub, CAL_QGC_PROGRESS_MSG, 100);

	roll_mean /= counter;
	pitch_mean /= counter;

	if (had_motion) {
		calibration_log_critical(mavlink_log_pub, "motion during calibration");

	} else if (fabsf(roll_mean) > 0.8f) {
		calibration_log_critical(mavlink_log_pub, "excess roll angle");

	} else if (fabsf(pitch_mean) > 0.8f) {
		calibration_log_critical(mavlink_log_pub, "excess pitch angle");

	} else {
		roll_mean *= (float)M_RAD_TO_DEG;
		pitch_mean *= (float)M_RAD_TO_DEG;
		param_set_no_notification(roll_offset_handle, &roll_mean);
		param_set_no_notification(pitch_offset_handle, &pitch_mean);
		param_notify_changes();
		success = true;
	}

out:

	orb_unsubscribe(att_sub);

	if (success) {
		calibration_log_info(mavlink_log_pub, CAL_QGC_DONE_MSG, "level");
		return 0;

	} else {
		calibration_log_critical(mavlink_log_pub, CAL_QGC_FAILED_MSG, "level");
		return 1;
	}
}
