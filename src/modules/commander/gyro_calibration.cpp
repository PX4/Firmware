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
 * @file gyro_calibration.cpp
 *
 * Gyroscope calibration routine
 */

#include <px4_platform_common/px4_config.h>
#include "gyro_calibration.h"
#include "calibration_messages.h"
#include "calibration_routines.h"
#include "commander_helper.h"

#include <px4_platform_common/posix.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/time.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <mathlib/mathlib.h>
#include <string.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_correction.h>
#include <drivers/drv_gyro.h>
#include <systemlib/mavlink_log.h>
#include <parameters/param.h>
#include <systemlib/err.h>

static const char *sensor_name = "gyro";

static constexpr unsigned max_gyros = 3;

/// Data passed to calibration worker routine
typedef struct  {
	orb_advert_t		*mavlink_log_pub;
	int32_t			device_id[max_gyros];
	int			gyro_sensor_sub[max_gyros];
	int			sensor_correction_sub;
	struct gyro_calibration_s	gyro_scale[max_gyros];
	float last_sample_0[3];
} gyro_worker_data_t;

static calibrate_return gyro_calibration_worker(int cancel_sub, void *data)
{
	gyro_worker_data_t	*worker_data = (gyro_worker_data_t *)(data);
	unsigned		calibration_counter[max_gyros] = { 0 }, slow_count = 0;
	const unsigned		calibration_count = 250;
	sensor_gyro_s	gyro_report;
	unsigned		poll_errcount = 0;

	struct sensor_correction_s sensor_correction {}; /**< sensor thermal corrections */

	if (orb_copy(ORB_ID(sensor_correction), worker_data->sensor_correction_sub, &sensor_correction) != 0) {
		for (unsigned i = 0; i < 3; i++) {
			sensor_correction.gyro_scale_0[i] = 1.0f;
			sensor_correction.gyro_scale_1[i] = 1.0f;
			sensor_correction.gyro_scale_2[i] = 1.0f;
		}
	}

	px4_pollfd_struct_t fds[max_gyros];

	for (unsigned s = 0; s < max_gyros; s++) {
		fds[s].fd = worker_data->gyro_sensor_sub[s];
		fds[s].events = POLLIN;
	}

	memset(&worker_data->last_sample_0, 0, sizeof(worker_data->last_sample_0));

	/* use slowest gyro to pace, but count correctly per-gyro for statistics */
	while (slow_count < calibration_count) {
		if (calibrate_cancel_check(worker_data->mavlink_log_pub, cancel_sub)) {
			return calibrate_return_cancelled;
		}

		/* check if there are new thermal corrections */
		bool updated;
		orb_check(worker_data->sensor_correction_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(sensor_correction), worker_data->sensor_correction_sub, &sensor_correction);
		}

		int poll_ret = px4_poll(&fds[0], max_gyros, 1000);

		if (poll_ret > 0) {
			unsigned update_count = calibration_count;

			for (unsigned s = 0; s < max_gyros; s++) {
				if (calibration_counter[s] >= calibration_count) {
					// Skip if instance has enough samples
					continue;
				}

				bool changed;
				orb_check(worker_data->gyro_sensor_sub[s], &changed);

				if (changed) {
					orb_copy(ORB_ID(sensor_gyro), worker_data->gyro_sensor_sub[s], &gyro_report);
					float sample[3];

					if (s == 0) {
						// take a working copy
						sample[0] = (gyro_report.x - sensor_correction.gyro_offset_0[0]) * sensor_correction.gyro_scale_0[0];
						sample[1] = (gyro_report.y - sensor_correction.gyro_offset_0[1]) * sensor_correction.gyro_scale_0[1];
						sample[2] = (gyro_report.z - sensor_correction.gyro_offset_0[2]) * sensor_correction.gyro_scale_0[2];

						for (int i = 0; i < 3; ++i) {
							worker_data->last_sample_0[i] = sample[i];
						}

					} else if (s == 1) {
						sample[0] = (gyro_report.x - sensor_correction.gyro_offset_1[0]) * sensor_correction.gyro_scale_1[0];
						sample[1] = (gyro_report.y - sensor_correction.gyro_offset_1[1]) * sensor_correction.gyro_scale_1[1];
						sample[2] = (gyro_report.z - sensor_correction.gyro_offset_1[2]) * sensor_correction.gyro_scale_1[2];

					} else if (s == 2) {
						sample[0] = (gyro_report.x - sensor_correction.gyro_offset_2[0]) * sensor_correction.gyro_scale_2[0];
						sample[1] = (gyro_report.y - sensor_correction.gyro_offset_2[1]) * sensor_correction.gyro_scale_2[1];
						sample[2] = (gyro_report.z - sensor_correction.gyro_offset_2[2]) * sensor_correction.gyro_scale_2[2];

					} else {
						sample[0] = gyro_report.x;
						sample[1] = gyro_report.y;
						sample[2] = gyro_report.z;

					}

					worker_data->gyro_scale[s].x_offset += sample[0];
					worker_data->gyro_scale[s].y_offset += sample[1];
					worker_data->gyro_scale[s].z_offset += sample[2];
					calibration_counter[s]++;

				}

				// Maintain the sample count of the slowest sensor
				if (calibration_counter[s] && calibration_counter[s] < update_count) {
					update_count = calibration_counter[s];
				}

			}

			if (update_count % (calibration_count / 20) == 0) {
				calibration_log_info(worker_data->mavlink_log_pub, CAL_QGC_PROGRESS_MSG, (update_count * 100) / calibration_count);
			}

			// Propagate out the slowest sensor's count
			if (slow_count < update_count) {
				slow_count = update_count;
			}

		} else {
			poll_errcount++;
		}

		if (poll_errcount > 1000) {
			calibration_log_critical(worker_data->mavlink_log_pub, CAL_ERROR_SENSOR_MSG);
			return calibrate_return_error;
		}
	}

	for (unsigned s = 0; s < max_gyros; s++) {
		if (worker_data->device_id[s] != 0 && calibration_counter[s] < calibration_count / 2) {
			calibration_log_critical(worker_data->mavlink_log_pub, "ERROR: missing data, sensor %d", s)
			return calibrate_return_error;
		}

		worker_data->gyro_scale[s].x_offset /= calibration_counter[s];
		worker_data->gyro_scale[s].y_offset /= calibration_counter[s];
		worker_data->gyro_scale[s].z_offset /= calibration_counter[s];
	}

	return calibrate_return_ok;
}

int do_gyro_calibration(orb_advert_t *mavlink_log_pub)
{
	int			res = PX4_OK;
	gyro_worker_data_t	worker_data = {};

	calibration_log_info(mavlink_log_pub, CAL_QGC_STARTED_MSG, sensor_name);

	worker_data.mavlink_log_pub = mavlink_log_pub;

	gyro_calibration_s gyro_scale_zero{};
	int device_prio_max = 0;
	int32_t device_id_primary = 0;

	worker_data.sensor_correction_sub = orb_subscribe(ORB_ID(sensor_correction));

	for (unsigned s = 0; s < max_gyros; s++) {
		char str[30];

		// Reset gyro ids to unavailable.
		worker_data.device_id[s] = 0;
		// And set default subscriber values.
		worker_data.gyro_sensor_sub[s] = -1;
		(void)sprintf(str, "CAL_GYRO%u_ID", s);
		res = param_set_no_notification(param_find(str), &(worker_data.device_id[s]));

		if (res != PX4_OK) {
			calibration_log_critical(mavlink_log_pub, "Unable to reset CAL_GYRO%u_ID", s);
			return PX4_ERROR;
		}

		// Reset all offsets to 0
		(void)memcpy(&worker_data.gyro_scale[s], &gyro_scale_zero, sizeof(gyro_scale_zero));
#if 1 // TODO: replace all IOCTL usage
		sprintf(str, "%s%u", GYRO_BASE_DEVICE_PATH, s);
		int fd = px4_open(str, 0);

		if (fd >= 0) {
			worker_data.device_id[s] = px4_ioctl(fd, DEVIOCGDEVICEID, 0);
			res = px4_ioctl(fd, GYROIOCSSCALE, (long unsigned int)&gyro_scale_zero);
			px4_close(fd);

			if (res != PX4_OK) {
				calibration_log_critical(mavlink_log_pub, CAL_ERROR_RESET_CAL_MSG, s);
				return PX4_ERROR;
			}
		}

#else
		(void)sprintf(str, "CAL_GYRO%u_XOFF", s);
		res = param_set_no_notification(param_find(str), &gyro_scale_zero.x_offset);

		if (res != PX4_OK) {
			PX4_ERR("unable to reset %s", str);
		}

		(void)sprintf(str, "CAL_GYRO%u_YOFF", s);
		res = param_set_no_notification(param_find(str), &gyro_scale_zero.y_offset);

		if (res != PX4_OK) {
			PX4_ERR("unable to reset %s", str);
		}

		(void)sprintf(str, "CAL_GYRO%u_ZOFF", s);
		res = param_set_no_notification(param_find(str), &gyro_scale_zero.z_offset);

		if (res != PX4_OK) {
			PX4_ERR("unable to reset %s", str);
		}

		param_notify_changes();
#endif

	}

	// We should not try to subscribe if the topic doesn't actually exist and can be counted.
	const unsigned orb_gyro_count = orb_group_count(ORB_ID(sensor_gyro));

	// Warn that we will not calibrate more than max_gyros gyroscopes
	if (orb_gyro_count > max_gyros) {
		calibration_log_critical(mavlink_log_pub, "Detected %u gyros, but will calibrate only %u", orb_gyro_count, max_gyros);
	}

	for (unsigned cur_gyro = 0; cur_gyro < orb_gyro_count && cur_gyro < max_gyros; cur_gyro++) {

		// Lock in to correct ORB instance
		bool found_cur_gyro = false;

		for (unsigned i = 0; i < orb_gyro_count && !found_cur_gyro; i++) {
			worker_data.gyro_sensor_sub[cur_gyro] = orb_subscribe_multi(ORB_ID(sensor_gyro), i);

			sensor_gyro_s report{};
			orb_copy(ORB_ID(sensor_gyro), worker_data.gyro_sensor_sub[cur_gyro], &report);

#if 1 // TODO: replace all IOCTL usage

			// For NuttX, we get the UNIQUE device ID from the sensor driver via an IOCTL
			// and match it up with the one from the uORB subscription, because the
			// instance ordering of uORB and the order of the FDs may not be the same.

			if (report.device_id == (uint32_t)worker_data.device_id[cur_gyro]) {
				// Device IDs match, correct ORB instance for this gyro
				found_cur_gyro = true;

			} else {
				orb_unsubscribe(worker_data.gyro_sensor_sub[cur_gyro]);
			}

#else

			// For the DriverFramework drivers, we fill device ID (this is the first time) by copying one report.
			worker_data.device_id[cur_gyro] = report.device_id;
			found_cur_gyro = true;

#endif
		}

		if (!found_cur_gyro) {
			calibration_log_critical(mavlink_log_pub, "Gyro #%u (ID %u) no matching uORB devid", cur_gyro,
						 worker_data.device_id[cur_gyro]);
			res = calibrate_return_error;
			break;
		}

		if (worker_data.device_id[cur_gyro] != 0) {
			// Get priority
			int32_t prio;
			orb_priority(worker_data.gyro_sensor_sub[cur_gyro], &prio);

			if (prio > device_prio_max) {
				device_prio_max = prio;
				device_id_primary = worker_data.device_id[cur_gyro];
			}

		} else {
			calibration_log_critical(mavlink_log_pub, "Gyro #%u no device id, abort", cur_gyro);
		}
	}

	int cancel_sub  = calibrate_cancel_subscribe();

	unsigned try_count = 0;
	unsigned max_tries = 20;
	res = PX4_ERROR;

	do {
		// Calibrate gyro and ensure user didn't move
		calibrate_return cal_return = gyro_calibration_worker(cancel_sub, &worker_data);

		if (cal_return == calibrate_return_cancelled) {
			// Cancel message already sent, we are done here
			res = PX4_ERROR;
			break;

		} else if (cal_return == calibrate_return_error) {
			res = PX4_ERROR;

		} else {
			/* check offsets */
			float xdiff = worker_data.last_sample_0[0] - worker_data.gyro_scale[0].x_offset;
			float ydiff = worker_data.last_sample_0[1] - worker_data.gyro_scale[0].y_offset;
			float zdiff = worker_data.last_sample_0[2] - worker_data.gyro_scale[0].z_offset;

			/* maximum allowable calibration error */
			const float maxoff = math::radians(0.4f);

			if (!PX4_ISFINITE(worker_data.gyro_scale[0].x_offset) ||
			    !PX4_ISFINITE(worker_data.gyro_scale[0].y_offset) ||
			    !PX4_ISFINITE(worker_data.gyro_scale[0].z_offset) ||
			    fabsf(xdiff) > maxoff ||
			    fabsf(ydiff) > maxoff ||
			    fabsf(zdiff) > maxoff) {

				calibration_log_critical(mavlink_log_pub, "motion, retrying..");
				res = PX4_ERROR;

			} else {
				res = PX4_OK;
			}
		}

		try_count++;

	} while (res == PX4_ERROR && try_count <= max_tries);

	if (try_count >= max_tries) {
		calibration_log_critical(mavlink_log_pub, "ERROR: Motion during calibration");
		res = PX4_ERROR;
	}

	calibrate_cancel_unsubscribe(cancel_sub);

	for (unsigned s = 0; s < max_gyros; s++) {
		px4_close(worker_data.gyro_sensor_sub[s]);
	}

	if (res == PX4_OK) {

		/* set offset parameters to new values */
		bool failed = false;

		failed = failed || (PX4_OK != param_set_no_notification(param_find("CAL_GYRO_PRIME"), &(device_id_primary)));

		bool tc_locked[3] = {false}; // true when the thermal parameter instance has already been adjusted by the calibrator

		for (unsigned uorb_index = 0; uorb_index < max_gyros; uorb_index++) {
			if (worker_data.device_id[uorb_index] != 0) {
				char str[30];

				/* check if thermal compensation is enabled */
				int32_t tc_enabled_int;
				param_get(param_find("TC_G_ENABLE"), &(tc_enabled_int));

				if (tc_enabled_int == 1) {
					/* Get struct containing sensor thermal compensation data */
					struct sensor_correction_s sensor_correction; /**< sensor thermal corrections */
					memset(&sensor_correction, 0, sizeof(sensor_correction));
					orb_copy(ORB_ID(sensor_correction), worker_data.sensor_correction_sub, &sensor_correction);

					/* don't allow a parameter instance to be calibrated again by another uORB instance */
					if (!tc_locked[sensor_correction.gyro_mapping[uorb_index]]) {
						tc_locked[sensor_correction.gyro_mapping[uorb_index]] = true;

						/* update the _X0_ terms to include the additional offset */
						int32_t handle;
						float val;

						for (unsigned axis_index = 0; axis_index < 3; axis_index++) {
							val = 0.0f;
							(void)sprintf(str, "TC_G%u_X0_%u", sensor_correction.gyro_mapping[uorb_index], axis_index);
							handle = param_find(str);
							param_get(handle, &val);

							if (axis_index == 0) {
								val += worker_data.gyro_scale[uorb_index].x_offset;

							} else if (axis_index == 1) {
								val += worker_data.gyro_scale[uorb_index].y_offset;

							} else if (axis_index == 2) {
								val += worker_data.gyro_scale[uorb_index].z_offset;

							}

							failed |= (PX4_OK != param_set_no_notification(handle, &val));
						}

						param_notify_changes();
					}

					// Ensure the calibration values used the driver are at default settings
					worker_data.gyro_scale[uorb_index].x_offset = 0.f;
					worker_data.gyro_scale[uorb_index].y_offset = 0.f;
					worker_data.gyro_scale[uorb_index].z_offset = 0.f;
				}

				(void)sprintf(str, "CAL_GYRO%u_XOFF", uorb_index);
				failed |= (PX4_OK != param_set_no_notification(param_find(str), &(worker_data.gyro_scale[uorb_index].x_offset)));
				(void)sprintf(str, "CAL_GYRO%u_YOFF", uorb_index);
				failed |= (PX4_OK != param_set_no_notification(param_find(str), &(worker_data.gyro_scale[uorb_index].y_offset)));
				(void)sprintf(str, "CAL_GYRO%u_ZOFF", uorb_index);
				failed |= (PX4_OK != param_set_no_notification(param_find(str), &(worker_data.gyro_scale[uorb_index].z_offset)));

				(void)sprintf(str, "CAL_GYRO%u_ID", uorb_index);
				failed |= (PX4_OK != param_set_no_notification(param_find(str), &(worker_data.device_id[uorb_index])));

#if 1 // TODO: replace all IOCTL usage
				/* apply new scaling and offsets */
				(void)sprintf(str, "%s%u", GYRO_BASE_DEVICE_PATH, uorb_index);
				int fd = px4_open(str, 0);

				if (fd < 0) {
					failed = true;
					continue;
				}

				res = px4_ioctl(fd, GYROIOCSSCALE, (long unsigned int)&worker_data.gyro_scale[uorb_index]);
				px4_close(fd);

				if (res != PX4_OK) {
					calibration_log_critical(mavlink_log_pub, CAL_ERROR_APPLY_CAL_MSG);
				}

#endif
			}
		}

		if (failed) {
			calibration_log_critical(mavlink_log_pub, "ERROR: failed to set offset params");
			res = PX4_ERROR;
		}
	}

	/* if there is a any preflight-check system response, let the barrage of messages through */
	px4_usleep(200000);

	if (res == PX4_OK) {
		calibration_log_info(mavlink_log_pub, CAL_QGC_DONE_MSG, sensor_name);

	} else {
		calibration_log_info(mavlink_log_pub, CAL_QGC_FAILED_MSG, sensor_name);
	}

	orb_unsubscribe(worker_data.sensor_correction_sub);

	/* give this message enough time to propagate */
	px4_usleep(600000);

	return res;
}
