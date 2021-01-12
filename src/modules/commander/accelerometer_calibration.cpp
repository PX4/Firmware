/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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

#include "accelerometer_calibration.h"
#include "calibration_messages.h"
#include "calibration_routines.h"
#include "commander_helper.h"
#include "factory_calibration_storage.h"

#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/time.h>

#include <drivers/drv_hrt.h>
#include <lib/sensor_calibration/Accelerometer.hpp>
#include <lib/sensor_calibration/Utilities.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/ecl/geo/geo.h>
#include <matrix/math.hpp>
#include <lib/conversion/rotation.h>
#include <lib/parameters/param.h>
#include <lib/systemlib/err.h>
#include <lib/systemlib/mavlink_log.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionBlocking.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/vehicle_attitude.h>

using namespace matrix;
using namespace time_literals;

static constexpr char sensor_name[] {"accel"};
static constexpr unsigned MAX_ACCEL_SENS = 4;

/// Data passed to calibration worker routine
struct accel_worker_data_s {
	orb_advert_t	*mavlink_log_pub{nullptr};
	unsigned	done_count{0};
	float		accel_ref[MAX_ACCEL_SENS][detect_orientation_side_count][3] {};
};

// Read specified number of accelerometer samples, calculate average and dispersion.
static calibrate_return read_accelerometer_avg(float (&accel_avg)[MAX_ACCEL_SENS][detect_orientation_side_count][3],
		unsigned orient, unsigned samples_num)
{
	Vector3f accel_sum[MAX_ACCEL_SENS] {};
	unsigned counts[MAX_ACCEL_SENS] {};

	unsigned errcount = 0;

	// sensor thermal corrections
	uORB::Subscription sensor_correction_sub{ORB_ID(sensor_correction)};
	sensor_correction_s sensor_correction{};
	sensor_correction_sub.copy(&sensor_correction);

	uORB::SubscriptionBlocking<sensor_accel_s> accel_sub[MAX_ACCEL_SENS] {
		{ORB_ID(sensor_accel), 0, 0},
		{ORB_ID(sensor_accel), 0, 1},
		{ORB_ID(sensor_accel), 0, 2},
		{ORB_ID(sensor_accel), 0, 3},
	};

	/* use the first sensor to pace the readout, but do per-sensor counts */
	while (counts[0] < samples_num) {
		if (accel_sub[0].updatedBlocking(100000)) {
			for (unsigned accel_index = 0; accel_index < MAX_ACCEL_SENS; accel_index++) {
				sensor_accel_s arp;

				while (accel_sub[accel_index].update(&arp)) {
					// fetch optional thermal offset corrections in sensor/board frame
					Vector3f offset{0, 0, 0};
					sensor_correction_sub.update(&sensor_correction);

					if (sensor_correction.timestamp > 0 && arp.device_id != 0) {
						for (uint8_t correction_index = 0; correction_index < MAX_ACCEL_SENS; correction_index++) {
							if (sensor_correction.accel_device_ids[correction_index] == arp.device_id) {
								switch (correction_index) {
								case 0:
									offset = Vector3f{sensor_correction.accel_offset_0};
									break;
								case 1:
									offset = Vector3f{sensor_correction.accel_offset_1};
									break;
								case 2:
									offset = Vector3f{sensor_correction.accel_offset_2};
									break;
								case 3:
									offset = Vector3f{sensor_correction.accel_offset_3};
									break;
								}
							}
						}
					}

					accel_sum[accel_index] += Vector3f{arp.x, arp.y, arp.z} - offset;
					counts[accel_index]++;
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
	const Dcmf board_rotation = calibration::GetBoardRotationMatrix();

	for (unsigned s = 0; s < MAX_ACCEL_SENS; s++) {
		accel_sum[s] = board_rotation * accel_sum[s];
	}

	for (unsigned s = 0; s < MAX_ACCEL_SENS; s++) {
		const Vector3f avg{accel_sum[s] / counts[s]};
		avg.copyTo(accel_avg[s][orient]);
	}

	return calibrate_return_ok;
}

static calibrate_return accel_calibration_worker(detect_orientation_return orientation, void *data)
{
	static constexpr unsigned samples_num = 750;
	accel_worker_data_s *worker_data = (accel_worker_data_s *)(data);

	calibration_log_info(worker_data->mavlink_log_pub, "[cal] Hold still, measuring %s side",
			     detect_orientation_str(orientation));

	read_accelerometer_avg(worker_data->accel_ref, orientation, samples_num);

	// check accel
	for (unsigned accel_index = 0; accel_index < MAX_ACCEL_SENS; accel_index++) {
		switch (orientation) {
		case ORIENTATION_TAIL_DOWN:    // [ g, 0, 0 ]
			if (worker_data->accel_ref[accel_index][ORIENTATION_TAIL_DOWN][0] < 0.f) {
				calibration_log_emergency(worker_data->mavlink_log_pub, "[cal] accel %d invalid X-axis, check rotation", accel_index);
				return calibrate_return_error;
			}

			break;

		case ORIENTATION_NOSE_DOWN:    // [ -g, 0, 0 ]
			if (worker_data->accel_ref[accel_index][ORIENTATION_NOSE_DOWN][0] > 0.f) {
				calibration_log_emergency(worker_data->mavlink_log_pub, "[cal] accel %d invalid X-axis, check rotation", accel_index);
				return calibrate_return_error;
			}

			break;

		case ORIENTATION_LEFT:         // [ 0, g, 0 ]
			if (worker_data->accel_ref[accel_index][ORIENTATION_LEFT][1] < 0.f) {
				calibration_log_emergency(worker_data->mavlink_log_pub, "[cal] accel %d invalid Y-axis, check rotation", accel_index);
				return calibrate_return_error;
			}

			break;

		case ORIENTATION_RIGHT:        // [ 0, -g, 0 ]
			if (worker_data->accel_ref[accel_index][ORIENTATION_RIGHT][1] > 0.f) {
				calibration_log_emergency(worker_data->mavlink_log_pub, "[cal] accel %d invalid Y-axis, check rotation", accel_index);
				return calibrate_return_error;
			}

			break;

		case ORIENTATION_UPSIDE_DOWN:  // [ 0, 0, g ]
			if (worker_data->accel_ref[accel_index][ORIENTATION_UPSIDE_DOWN][2] < 0.f) {
				calibration_log_emergency(worker_data->mavlink_log_pub, "[cal] accel %d invalid Z-axis, check rotation", accel_index);
				return calibrate_return_error;
			}

			break;

		case ORIENTATION_RIGHTSIDE_UP: // [ 0, 0, -g ]
			if (worker_data->accel_ref[accel_index][ORIENTATION_RIGHTSIDE_UP][2] > 0.f) {
				calibration_log_emergency(worker_data->mavlink_log_pub, "[cal] accel %d invalid Z-axis, check rotation", accel_index);
				return calibrate_return_error;
			}

			break;

		default:
			break;
		}
	}

	calibration_log_info(worker_data->mavlink_log_pub, "[cal] %s side result: [%.3f %.3f %.3f]",
			     detect_orientation_str(orientation),
			     (double)worker_data->accel_ref[0][orientation][0],
			     (double)worker_data->accel_ref[0][orientation][1],
			     (double)worker_data->accel_ref[0][orientation][2]);

	worker_data->done_count++;
	calibration_log_info(worker_data->mavlink_log_pub, CAL_QGC_PROGRESS_MSG, 17 * worker_data->done_count);

	return calibrate_return_ok;
}

int do_accel_calibration(orb_advert_t *mavlink_log_pub)
{
	calibration_log_info(mavlink_log_pub, CAL_QGC_STARTED_MSG, sensor_name);

	calibration::Accelerometer calibrations[MAX_ACCEL_SENS] {};
	unsigned active_sensors = 0;

	for (uint8_t cur_accel = 0; cur_accel < MAX_ACCEL_SENS; cur_accel++) {
		uORB::SubscriptionData<sensor_accel_s> accel_sub{ORB_ID(sensor_accel), cur_accel};

		if (accel_sub.advertised() && (accel_sub.get().device_id != 0) && (accel_sub.get().timestamp > 0)) {
			calibrations[cur_accel].set_device_id(accel_sub.get().device_id);
			active_sensors++;

		} else {
			calibrations[cur_accel].Reset();
		}

		// reset calibration index to match uORB numbering
		calibrations[cur_accel].set_calibration_index(cur_accel);
	}

	if (active_sensors == 0) {
		calibration_log_critical(mavlink_log_pub, CAL_ERROR_SENSOR_MSG);
		return PX4_ERROR;
	}

	FactoryCalibrationStorage factory_storage;

	if (factory_storage.open() != PX4_OK) {
		calibration_log_critical(mavlink_log_pub, "ERROR: cannot open calibration storage");
		return PX4_ERROR;
	}

	/* measure and calculate offsets & scales */
	accel_worker_data_s worker_data{};
	worker_data.mavlink_log_pub = mavlink_log_pub;
	bool data_collected[detect_orientation_side_count] {};

	if (calibrate_from_orientation(mavlink_log_pub, data_collected, accel_calibration_worker, &worker_data,
				       false) == calibrate_return_ok) {

		const Dcmf board_rotation = calibration::GetBoardRotationMatrix();
		const Dcmf board_rotation_t = board_rotation.transpose();

		bool param_save = false;
		bool failed = true;

		for (unsigned i = 0; i < MAX_ACCEL_SENS; i++) {
			if (i < active_sensors) {
				// calculate offsets
				Vector3f offset{};

				// X offset: average X from TAIL_DOWN + NOSE_DOWN
				const Vector3f accel_tail_down{worker_data.accel_ref[i][ORIENTATION_TAIL_DOWN]};
				const Vector3f accel_nose_down{worker_data.accel_ref[i][ORIENTATION_NOSE_DOWN]};
				offset(0) = (accel_tail_down(0) + accel_nose_down(0)) * 0.5f;

				// Y offset: average Y from LEFT + RIGHT
				const Vector3f accel_left{worker_data.accel_ref[i][ORIENTATION_LEFT]};
				const Vector3f accel_right{worker_data.accel_ref[i][ORIENTATION_RIGHT]};
				offset(1) = (accel_left(1) + accel_right(1)) * 0.5f;

				// Z offset: average Z from UPSIDE_DOWN + RIGHTSIDE_UP
				const Vector3f accel_upside_down{worker_data.accel_ref[i][ORIENTATION_UPSIDE_DOWN]};
				const Vector3f accel_rightside_up{worker_data.accel_ref[i][ORIENTATION_RIGHTSIDE_UP]};
				offset(2) = (accel_upside_down(2) + accel_rightside_up(2)) * 0.5f;

				// transform matrix
				Matrix3f mat_A;
				mat_A.row(0) = accel_tail_down - offset;
				mat_A.row(1) = accel_left - offset;
				mat_A.row(2) = accel_upside_down - offset;

				// calculate inverse matrix for A: simplify matrices mult because b has only one non-zero element == g at index i
				const Matrix3f accel_T = mat_A.I() * CONSTANTS_ONE_G;

				// update calibration
				const Vector3f accel_offs_rotated{board_rotation_t *offset};
				calibrations[i].set_offset(accel_offs_rotated);

				const Matrix3f accel_T_rotated{board_rotation_t *accel_T * board_rotation};
				calibrations[i].set_scale(accel_T_rotated.diag());

#if defined(DEBUD_BUILD)
				PX4_INFO("accel %d: offset", i);
				offset.print();
				PX4_INFO("accel %d: bT * offset", i);
				accel_offs_rotated.print();

				PX4_INFO("accel %d: mat_A", i);
				mat_A.print();
				PX4_INFO("accel %d: accel_T", i);
				accel_T.print();
				PX4_INFO("accel %d: bT * accel_T * b", i);
				accel_T_rotated.print();
#endif // DEBUD_BUILD
				calibrations[i].PrintStatus();
			}

			// save all calibrations including empty slots
			if (calibrations[i].ParametersSave()) {
				param_save = true;
				failed = false;

			} else {
				failed = true;
				calibration_log_critical(mavlink_log_pub, "calibration save failed");
				break;
			}
		}

		if (!failed && factory_storage.store() != PX4_OK) {
			failed = true;
		}

		if (param_save) {
			param_notify_changes();
		}

		if (!failed) {
			calibration_log_info(mavlink_log_pub, CAL_QGC_DONE_MSG, sensor_name);
			px4_usleep(600000); // give this message enough time to propagate
			return PX4_OK;
		}
	}

	calibration_log_critical(mavlink_log_pub, CAL_QGC_FAILED_MSG, sensor_name);
	px4_usleep(600000); // give this message enough time to propagate
	return PX4_ERROR;
}

int do_accel_calibration_quick(orb_advert_t *mavlink_log_pub)
{
#if !defined(CONSTRAINED_FLASH)
	PX4_INFO("Accelerometer quick calibration");

	bool param_save = false;
	bool failed = true;

	FactoryCalibrationStorage factory_storage;

	if (factory_storage.open() != PX4_OK) {
		calibration_log_critical(mavlink_log_pub, "ERROR: cannot open calibration storage");
		return PX4_ERROR;
	}

	// sensor thermal corrections (optional)
	uORB::Subscription sensor_correction_sub{ORB_ID(sensor_correction)};
	sensor_correction_s sensor_correction{};
	sensor_correction_sub.copy(&sensor_correction);

	uORB::SubscriptionMultiArray<sensor_accel_s, MAX_ACCEL_SENS> accel_subs{ORB_ID::sensor_accel};

	/* use the first sensor to pace the readout, but do per-sensor counts */
	for (unsigned accel_index = 0; accel_index < MAX_ACCEL_SENS; accel_index++) {
		sensor_accel_s arp{};
		Vector3f accel_sum{};
		unsigned count = 0;

		while (accel_subs[accel_index].update(&arp)) {
			// fetch optional thermal offset corrections in sensor/board frame
			if ((arp.timestamp > 0) && (arp.device_id != 0)) {
				Vector3f offset{0, 0, 0};

				if (sensor_correction.timestamp > 0) {
					for (uint8_t correction_index = 0; correction_index < MAX_ACCEL_SENS; correction_index++) {
						if (sensor_correction.accel_device_ids[correction_index] == arp.device_id) {
							switch (correction_index) {
							case 0:
								offset = Vector3f{sensor_correction.accel_offset_0};
								break;
							case 1:
								offset = Vector3f{sensor_correction.accel_offset_1};
								break;
							case 2:
								offset = Vector3f{sensor_correction.accel_offset_2};
								break;
							case 3:
								offset = Vector3f{sensor_correction.accel_offset_3};
								break;
							}
						}
					}
				}

				const Vector3f accel{Vector3f{arp.x, arp.y, arp.z} - offset};

				if (count > 0) {
					const Vector3f diff{accel - (accel_sum / count)};

					if (diff.norm() < 1.f) {
						accel_sum += Vector3f{arp.x, arp.y, arp.z} - offset;
						count++;
					}

				} else {
					accel_sum = accel;
					count = 1;
				}
			}
		}

		if ((count > 0) && (arp.device_id != 0)) {

			bool calibrated = false;
			const Vector3f accel_avg = accel_sum / count;

			Vector3f offset{0.f, 0.f, 0.f};

			uORB::SubscriptionData<vehicle_attitude_s> attitude_sub{ORB_ID(vehicle_attitude)};
			attitude_sub.update();

			if (attitude_sub.advertised() && attitude_sub.get().timestamp != 0) {
				// use vehicle_attitude if available
				const vehicle_attitude_s &att = attitude_sub.get();
				const matrix::Quatf q{att.q};
				const Vector3f accel_ref = q.conjugate_inversed(Vector3f{0.f, 0.f, -CONSTANTS_ONE_G});

				// sanity check angle between acceleration vectors
				const float angle = AxisAnglef(Quatf(accel_avg, accel_ref)).angle();

				if (angle <= math::radians(10.f)) {
					offset = accel_avg - accel_ref;
					calibrated = true;
				}
			}

			if (!calibrated) {
				// otherwise simply normalize to gravity and remove offset
				Vector3f accel{accel_avg};
				accel.normalize();
				accel = accel * CONSTANTS_ONE_G;

				offset = accel_avg - accel;
				calibrated = true;
			}

			calibration::Accelerometer calibration{arp.device_id};

			// reset cal index to uORB
			calibration.set_calibration_index(accel_index);

			if (!calibrated || (offset.norm() > CONSTANTS_ONE_G)
			    || !PX4_ISFINITE(offset(0))
			    || !PX4_ISFINITE(offset(1))
			    || !PX4_ISFINITE(offset(2))) {

				PX4_ERR("accel %d quick calibrate failed", accel_index);

			} else {
				calibration.set_offset(offset);

				if (calibration.ParametersSave()) {
					calibration.PrintStatus();
					param_save = true;
					failed = false;

				} else {
					failed = true;
					calibration_log_critical(mavlink_log_pub, CAL_QGC_FAILED_MSG, "calibration save failed");
					break;
				}
			}
		}
	}

	if (!failed && factory_storage.store() != PX4_OK) {
		failed = true;
	}

	if (param_save) {
		param_notify_changes();
	}

	if (!failed) {
		return PX4_OK;
	}

#endif // !CONSTRAINED_FLASH

	return PX4_ERROR;
}
