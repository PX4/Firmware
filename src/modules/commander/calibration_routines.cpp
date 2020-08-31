/****************************************************************************
 *
 *   Copyright (c) 2012 PX4 Development Team. All rights reserved.
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
 * @file calibration_routines.cpp
 * Calibration routines implementations.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/time.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_tone_alarm.h>

#include <lib/ecl/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/systemlib/mavlink_log.h>
#include <matrix/math.hpp>

#include <uORB/Publication.hpp>
#include <uORB/SubscriptionBlocking.hpp>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

#include "calibration_routines.h"
#include "calibration_messages.h"
#include "commander_helper.h"

using namespace time_literals;

int run_lm_sphere_fit(const float x[], const float y[], const float z[], float &_fitness, float &_sphere_lambda,
		      unsigned int samples_collected, float *offset_x, float *offset_y, float *offset_z,
		      float *sphere_radius, float *diag_x, float *diag_y, float *diag_z, float *offdiag_x, float *offdiag_y, float *offdiag_z)
{
	// Run Sphere Fit using Levenberg Marquardt LSq Fit
	const float lma_damping = 10.f;
	float fitness = _fitness;
	float fit1 = 0.f;
	float fit2 = 0.f;

	matrix::SquareMatrix<float, 4> JTJ{};
	matrix::SquareMatrix<float, 4> JTJ2{};
	float JTFI[4] {};
	float residual = 0.0f;

	// Gauss Newton Part common for all kind of extensions including LM
	for (uint16_t k = 0; k < samples_collected; k++) {

		float sphere_jacob[4];
		//Calculate Jacobian
		float A = (*diag_x    * (x[k] - *offset_x)) + (*offdiag_x * (y[k] - *offset_y)) + (*offdiag_y * (z[k] - *offset_z));
		float B = (*offdiag_x * (x[k] - *offset_x)) + (*diag_y    * (y[k] - *offset_y)) + (*offdiag_z * (z[k] - *offset_z));
		float C = (*offdiag_y * (x[k] - *offset_x)) + (*offdiag_z * (y[k] - *offset_y)) + (*diag_z    * (z[k] - *offset_z));
		float length = sqrtf(A * A + B * B + C * C);

		// 0: partial derivative (radius wrt fitness fn) fn operated on sample
		sphere_jacob[0] = 1.0f;
		// 1-3: partial derivative (offsets wrt fitness fn) fn operated on sample
		sphere_jacob[1] = 1.0f * (((*diag_x    * A) + (*offdiag_x * B) + (*offdiag_y * C)) / length);
		sphere_jacob[2] = 1.0f * (((*offdiag_x * A) + (*diag_y    * B) + (*offdiag_z * C)) / length);
		sphere_jacob[3] = 1.0f * (((*offdiag_y * A) + (*offdiag_z * B) + (*diag_z    * C)) / length);
		residual = *sphere_radius - length;

		for (uint8_t i = 0; i < 4; i++) {
			// compute JTJ
			for (uint8_t j = 0; j < 4; j++) {
				JTJ(i, j) += sphere_jacob[i] * sphere_jacob[j];
				JTJ2(i, j) += sphere_jacob[i] * sphere_jacob[j]; //a backup JTJ for LM
			}

			JTFI[i] += sphere_jacob[i] * residual;
		}
	}


	//------------------------Levenberg-Marquardt-part-starts-here---------------------------------//
	// refer: http://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm#Choice_of_damping_parameter
	float fit1_params[4] = {*sphere_radius, *offset_x, *offset_y, *offset_z};
	float fit2_params[4];
	memcpy(fit2_params, fit1_params, sizeof(fit1_params));

	for (uint8_t i = 0; i < 4; i++) {
		JTJ(i, i) += _sphere_lambda;
		JTJ2(i, i) += _sphere_lambda / lma_damping;
	}

	if (!JTJ.I(JTJ)) {
		return -1;
	}

	if (!JTJ2.I(JTJ2)) {
		return -1;
	}

	for (uint8_t row = 0; row < 4; row++) {
		for (uint8_t col = 0; col < 4; col++) {
			fit1_params[row] -= JTFI[col] * JTJ(row, col);
			fit2_params[row] -= JTFI[col] * JTJ2(row, col);
		}
	}

	// Calculate mean squared residuals
	for (uint16_t k = 0; k < samples_collected; k++) {
		float A = (*diag_x    * (x[k] - fit1_params[1])) + (*offdiag_x * (y[k] - fit1_params[2])) + (*offdiag_y *
				(z[k] + fit1_params[3]));
		float B = (*offdiag_x * (x[k] - fit1_params[1])) + (*diag_y    * (y[k] - fit1_params[2])) + (*offdiag_z *
				(z[k] + fit1_params[3]));
		float C = (*offdiag_y * (x[k] - fit1_params[1])) + (*offdiag_z * (y[k] - fit1_params[2])) + (*diag_z    *
				(z[k] - fit1_params[3]));
		float length = sqrtf(A * A + B * B + C * C);
		residual = fit1_params[0] - length;
		fit1 += residual * residual;

		A = (*diag_x    * (x[k] - fit2_params[1])) + (*offdiag_x * (y[k] - fit2_params[2])) + (*offdiag_y *
				(z[k] - fit2_params[3]));
		B = (*offdiag_x * (x[k] - fit2_params[1])) + (*diag_y    * (y[k] - fit2_params[2])) + (*offdiag_z *
				(z[k] - fit2_params[3]));
		C = (*offdiag_y * (x[k] - fit2_params[1])) + (*offdiag_z * (y[k] - fit2_params[2])) + (*diag_z    *
				(z[k] - fit2_params[3]));
		length = sqrtf(A * A + B * B + C * C);
		residual = fit2_params[0] - length;
		fit2 += residual * residual;
	}

	fit1 = sqrtf(fit1) / samples_collected;
	fit2 = sqrtf(fit2) / samples_collected;

	if (fit1 > _fitness && fit2 > _fitness) {
		_sphere_lambda *= lma_damping;

	} else if (fit2 < _fitness && fit2 < fit1) {
		_sphere_lambda /= lma_damping;
		memcpy(fit1_params, fit2_params, sizeof(fit1_params));
		fitness = fit2;

	} else if (fit1 < _fitness) {
		fitness = fit1;
	}

	//--------------------Levenberg-Marquardt-part-ends-here--------------------------------//

	if (PX4_ISFINITE(fitness) && fitness < _fitness) {
		_fitness = fitness;
		*sphere_radius = fit1_params[0];
		*offset_x = fit1_params[1];
		*offset_y = fit1_params[2];
		*offset_z = fit1_params[3];
		return 0;

	} else {
		return -1;
	}
}

int run_lm_ellipsoid_fit(const float x[], const float y[], const float z[], float &_fitness, float &_sphere_lambda,
			 unsigned int samples_collected, float *offset_x, float *offset_y, float *offset_z,
			 float *sphere_radius, float *diag_x, float *diag_y, float *diag_z, float *offdiag_x, float *offdiag_y, float *offdiag_z)
{
	// Run Sphere Fit using Levenberg Marquardt LSq Fit
	const float lma_damping = 10.0f;
	float fitness = _fitness;
	float fit1 = 0.0f;
	float fit2 = 0.0f;

	float JTJ[81] {};
	float JTJ2[81] {};
	float JTFI[9] {};
	float residual = 0.0f;
	float ellipsoid_jacob[9];

	// Gauss Newton Part common for all kind of extensions including LM
	for (uint16_t k = 0; k < samples_collected; k++) {

		// Calculate Jacobian
		float A = (*diag_x    * (x[k] - *offset_x)) + (*offdiag_x * (y[k] - *offset_y)) + (*offdiag_y * (z[k] - *offset_z));
		float B = (*offdiag_x * (x[k] - *offset_x)) + (*diag_y    * (y[k] - *offset_y)) + (*offdiag_z * (z[k] - *offset_z));
		float C = (*offdiag_y * (x[k] - *offset_x)) + (*offdiag_z * (y[k] - *offset_y)) + (*diag_z    * (z[k] - *offset_z));
		float length = sqrtf(A * A + B * B + C * C);
		residual = *sphere_radius - length;
		fit1 += residual * residual;
		// 0-2: partial derivative (offset wrt fitness fn) fn operated on sample
		ellipsoid_jacob[0] = 1.0f * (((*diag_x    * A) + (*offdiag_x * B) + (*offdiag_y * C)) / length);
		ellipsoid_jacob[1] = 1.0f * (((*offdiag_x * A) + (*diag_y    * B) + (*offdiag_z * C)) / length);
		ellipsoid_jacob[2] = 1.0f * (((*offdiag_y * A) + (*offdiag_z * B) + (*diag_z    * C)) / length);
		// 3-5: partial derivative (diag offset wrt fitness fn) fn operated on sample
		ellipsoid_jacob[3] = -1.0f * ((x[k] - *offset_x) * A) / length;
		ellipsoid_jacob[4] = -1.0f * ((y[k] - *offset_y) * B) / length;
		ellipsoid_jacob[5] = -1.0f * ((z[k] - *offset_z) * C) / length;
		// 6-8: partial derivative (off-diag offset wrt fitness fn) fn operated on sample
		ellipsoid_jacob[6] = -1.0f * (((y[k] - *offset_y) * A) + ((x[k] - *offset_x) * B)) / length;
		ellipsoid_jacob[7] = -1.0f * (((z[k] - *offset_z) * A) + ((x[k] - *offset_x) * C)) / length;
		ellipsoid_jacob[8] = -1.0f * (((z[k] - *offset_z) * B) + ((y[k] - *offset_y) * C)) / length;

		for (uint8_t i = 0; i < 9; i++) {
			// compute JTJ
			for (uint8_t j = 0; j < 9; j++) {
				JTJ[i * 9 + j] += ellipsoid_jacob[i] * ellipsoid_jacob[j];
				JTJ2[i * 9 + j] += ellipsoid_jacob[i] * ellipsoid_jacob[j]; // a backup JTJ for LM
			}

			JTFI[i] += ellipsoid_jacob[i] * residual;
		}
	}


	//------------------------Levenberg-Marquardt-part-starts-here---------------------------------//
	// refer: http://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm#Choice_of_damping_parameter
	float fit1_params[9] = {*offset_x, *offset_y, *offset_z, *diag_x, *diag_y, *diag_z, *offdiag_x, *offdiag_y, *offdiag_z};
	float fit2_params[9];
	memcpy(fit2_params, fit1_params, sizeof(fit1_params));

	for (uint8_t i = 0; i < 9; i++) {
		JTJ[i * 9 + i] += _sphere_lambda;
		JTJ2[i * 9 + i] += _sphere_lambda / lma_damping;
	}


	if (!mat_inverse(JTJ, JTJ, 9)) {
		return -1;
	}

	if (!mat_inverse(JTJ2, JTJ2, 9)) {
		return -1;
	}

	for (uint8_t row = 0; row < 9; row++) {
		for (uint8_t col = 0; col < 9; col++) {
			fit1_params[row] -= JTFI[col] * JTJ[row * 9 + col];
			fit2_params[row] -= JTFI[col] * JTJ2[row * 9 + col];
		}
	}

	// Calculate mean squared residuals
	for (uint16_t k = 0; k < samples_collected; k++) {
		float A = (fit1_params[3]    * (x[k] - fit1_params[0])) + (fit1_params[6] * (y[k] - fit1_params[1])) + (fit1_params[7] *
				(z[k] - fit1_params[2]));
		float B = (fit1_params[6] * (x[k] - fit1_params[0])) + (fit1_params[4]   * (y[k] - fit1_params[1])) + (fit1_params[8] *
				(z[k] - fit1_params[2]));
		float C = (fit1_params[7] * (x[k] - fit1_params[0])) + (fit1_params[8] * (y[k] - fit1_params[1])) + (fit1_params[5]    *
				(z[k] - fit1_params[2]));
		float length = sqrtf(A * A + B * B + C * C);
		residual = *sphere_radius - length;
		fit1 += residual * residual;

		A = (fit2_params[3]    * (x[k] - fit2_params[0])) + (fit2_params[6] * (y[k] - fit2_params[1])) + (fit2_params[7] *
				(z[k] - fit2_params[2]));
		B = (fit2_params[6] * (x[k] - fit2_params[0])) + (fit2_params[4]   * (y[k] - fit2_params[1])) + (fit2_params[8] *
				(z[k] - fit2_params[2]));
		C = (fit2_params[7] * (x[k] - fit2_params[0])) + (fit2_params[8] * (y[k] - fit2_params[1])) + (fit2_params[5]    *
				(z[k] - fit2_params[2]));
		length = sqrtf(A * A + B * B + C * C);
		residual = *sphere_radius - length;
		fit2 += residual * residual;
	}

	fit1 = sqrtf(fit1) / samples_collected;
	fit2 = sqrtf(fit2) / samples_collected;

	if (fit1 > _fitness && fit2 > _fitness) {
		_sphere_lambda *= lma_damping;

	} else if (fit2 < _fitness && fit2 < fit1) {
		_sphere_lambda /= lma_damping;
		memcpy(fit1_params, fit2_params, sizeof(fit1_params));
		fitness = fit2;

	} else if (fit1 < _fitness) {
		fitness = fit1;
	}

	//--------------------Levenberg-Marquardt-part-ends-here--------------------------------//
	if (PX4_ISFINITE(fitness) && fitness < _fitness) {
		_fitness = fitness;
		*offset_x = fit1_params[0];
		*offset_y = fit1_params[1];
		*offset_z = fit1_params[2];
		*diag_x = fit1_params[3];
		*diag_y = fit1_params[4];
		*diag_z = fit1_params[5];
		*offdiag_x = fit1_params[6];
		*offdiag_y = fit1_params[7];
		*offdiag_z = fit1_params[8];
		return 0;

	} else {
		return -1;
	}
}

enum detect_orientation_return detect_orientation(orb_advert_t *mavlink_log_pub, bool lenient_still_position)
{
	static constexpr unsigned ndim = 3;

	float accel_ema[ndim] {};                       // exponential moving average of accel
	float accel_disp[3] {};                         // max-hold dispersion of accel
	static constexpr float ema_len = 0.5f;          // EMA time constant in seconds
	static constexpr float normal_still_thr = 0.25; // normal still threshold
	float still_thr2 = powf(lenient_still_position ? (normal_still_thr * 3) : normal_still_thr, 2);
	static constexpr float accel_err_thr = 5.0f;    // set accel error threshold to 5m/s^2
	const hrt_abstime still_time = lenient_still_position ? 500000 : 1300000; // still time required in us

	/* set timeout to 90s */
	static constexpr hrt_abstime timeout = 90_s;

	const hrt_abstime t_start = hrt_absolute_time();
	hrt_abstime t_timeout = t_start + timeout;
	hrt_abstime t = t_start;
	hrt_abstime t_prev = t_start;
	hrt_abstime t_still = 0;

	unsigned poll_errcount = 0;

	// Setup subscriptions to onboard accel sensor
	uORB::SubscriptionBlocking<vehicle_acceleration_s> vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};

	while (true) {
		vehicle_acceleration_s accel;

		if (vehicle_acceleration_sub.updateBlocking(accel, 100000)) {
			t = hrt_absolute_time();
			float dt = (t - t_prev) / 1000000.0f;
			t_prev = t;
			float w = dt / ema_len;

			for (unsigned i = 0; i < ndim; i++) {

				float di = accel.xyz[i];

				float d = di - accel_ema[i];
				accel_ema[i] += d * w;
				d = d * d;
				accel_disp[i] = accel_disp[i] * (1.0f - w);

				if (d > still_thr2 * 8.0f) {
					d = still_thr2 * 8.0f;
				}

				if (d > accel_disp[i]) {
					accel_disp[i] = d;
				}
			}

			/* still detector with hysteresis */
			if (accel_disp[0] < still_thr2 &&
			    accel_disp[1] < still_thr2 &&
			    accel_disp[2] < still_thr2) {

				/* is still now */
				if (t_still == 0) {
					/* first time */
					calibration_log_info(mavlink_log_pub, "[cal] detected rest position, hold still...");
					t_still = t;
					t_timeout = t + timeout;

				} else {
					/* still since t_still */
					if (t > t_still + still_time) {
						/* vehicle is still, exit from the loop to detection of its orientation */
						break;
					}
				}

			} else if (accel_disp[0] > still_thr2 * 4.0f ||
				   accel_disp[1] > still_thr2 * 4.0f ||
				   accel_disp[2] > still_thr2 * 4.0f) {
				/* not still, reset still start time */
				if (t_still != 0) {
					calibration_log_info(mavlink_log_pub, "[cal] detected motion, hold still...");
					px4_usleep(200000);
					t_still = 0;
				}
			}

		} else {
			poll_errcount++;
		}

		if (t > t_timeout) {
			poll_errcount++;
		}

		if (poll_errcount > 1000) {
			calibration_log_critical(mavlink_log_pub, CAL_ERROR_SENSOR_MSG);
			return ORIENTATION_ERROR;
		}
	}

	if (fabsf(accel_ema[0] - CONSTANTS_ONE_G) < accel_err_thr &&
	    fabsf(accel_ema[1]) < accel_err_thr &&
	    fabsf(accel_ema[2]) < accel_err_thr) {
		return ORIENTATION_TAIL_DOWN;        // [ g, 0, 0 ]
	}

	if (fabsf(accel_ema[0] + CONSTANTS_ONE_G) < accel_err_thr &&
	    fabsf(accel_ema[1]) < accel_err_thr &&
	    fabsf(accel_ema[2]) < accel_err_thr) {
		return ORIENTATION_NOSE_DOWN;        // [ -g, 0, 0 ]
	}

	if (fabsf(accel_ema[0]) < accel_err_thr &&
	    fabsf(accel_ema[1] - CONSTANTS_ONE_G) < accel_err_thr &&
	    fabsf(accel_ema[2]) < accel_err_thr) {
		return ORIENTATION_LEFT;        // [ 0, g, 0 ]
	}

	if (fabsf(accel_ema[0]) < accel_err_thr &&
	    fabsf(accel_ema[1] + CONSTANTS_ONE_G) < accel_err_thr &&
	    fabsf(accel_ema[2]) < accel_err_thr) {
		return ORIENTATION_RIGHT;        // [ 0, -g, 0 ]
	}

	if (fabsf(accel_ema[0]) < accel_err_thr &&
	    fabsf(accel_ema[1]) < accel_err_thr &&
	    fabsf(accel_ema[2] - CONSTANTS_ONE_G) < accel_err_thr) {
		return ORIENTATION_UPSIDE_DOWN;        // [ 0, 0, g ]
	}

	if (fabsf(accel_ema[0]) < accel_err_thr &&
	    fabsf(accel_ema[1]) < accel_err_thr &&
	    fabsf(accel_ema[2] + CONSTANTS_ONE_G) < accel_err_thr) {
		return ORIENTATION_RIGHTSIDE_UP;        // [ 0, 0, -g ]
	}

	calibration_log_critical(mavlink_log_pub, "ERROR: invalid orientation");

	return ORIENTATION_ERROR;	// Can't detect orientation
}

const char *detect_orientation_str(enum detect_orientation_return orientation)
{
	static const char *rgOrientationStrs[] = {
		"back",		// tail down
		"front",	// nose down
		"left",
		"right",
		"up",		// upside-down
		"down",		// right-side up
		"error"
	};

	return rgOrientationStrs[orientation];
}

calibrate_return calibrate_from_orientation(orb_advert_t *mavlink_log_pub,
		bool side_data_collected[detect_orientation_side_count], calibration_from_orientation_worker_t calibration_worker,
		void *worker_data, bool lenient_still_position)
{
	const hrt_abstime calibration_started = hrt_absolute_time();
	calibrate_return result = calibrate_return_ok;

	unsigned orientation_failures = 0;

	// Rotate through all requested orientation
	while (true) {
		if (calibrate_cancel_check(mavlink_log_pub, calibration_started)) {
			result = calibrate_return_cancelled;
			break;
		}

		if (orientation_failures > 4) {
			result = calibrate_return_error;
			calibration_log_critical(mavlink_log_pub, CAL_QGC_FAILED_MSG, "timeout: no motion");
			break;
		}

		unsigned int side_complete_count = 0;

		// Update the number of completed sides
		for (unsigned i = 0; i < detect_orientation_side_count; i++) {
			if (side_data_collected[i]) {
				side_complete_count++;
			}
		}

		if (side_complete_count == detect_orientation_side_count) {
			// We have completed all sides, move on
			break;
		}

		/* inform user which orientations are still needed */
		char pendingStr[80];
		pendingStr[0] = 0;

		for (unsigned int cur_orientation = 0; cur_orientation < detect_orientation_side_count; cur_orientation++) {
			if (!side_data_collected[cur_orientation]) {
				strncat(pendingStr, " ", sizeof(pendingStr) - 1);
				strncat(pendingStr, detect_orientation_str((enum detect_orientation_return)cur_orientation), sizeof(pendingStr) - 1);
			}
		}

		calibration_log_info(mavlink_log_pub, "[cal] pending:%s", pendingStr);
		px4_usleep(20000);
		calibration_log_info(mavlink_log_pub, "[cal] hold vehicle still on a pending side");
		px4_usleep(20000);
		enum detect_orientation_return orient = detect_orientation(mavlink_log_pub, lenient_still_position);

		if (orient == ORIENTATION_ERROR) {
			orientation_failures++;
			calibration_log_info(mavlink_log_pub, "[cal] detected motion, hold still...");
			px4_usleep(20000);
			continue;
		}

		/* inform user about already handled side */
		if (side_data_collected[orient]) {
			orientation_failures++;
			set_tune(TONE_NOTIFY_NEGATIVE_TUNE);
			calibration_log_info(mavlink_log_pub, "[cal] %s side already completed", detect_orientation_str(orient));
			px4_usleep(20000);
			continue;
		}

		calibration_log_info(mavlink_log_pub, CAL_QGC_ORIENTATION_DETECTED_MSG, detect_orientation_str(orient));
		px4_usleep(20000);
		calibration_log_info(mavlink_log_pub, CAL_QGC_ORIENTATION_DETECTED_MSG, detect_orientation_str(orient));
		px4_usleep(20000);
		orientation_failures = 0;

		// Call worker routine
		result = calibration_worker(orient, worker_data);

		if (result != calibrate_return_ok) {
			break;
		}

		calibration_log_info(mavlink_log_pub, CAL_QGC_SIDE_DONE_MSG, detect_orientation_str(orient));
		px4_usleep(20000);
		calibration_log_info(mavlink_log_pub, CAL_QGC_SIDE_DONE_MSG, detect_orientation_str(orient));
		px4_usleep(20000);

		// Note that this side is complete
		side_data_collected[orient] = true;

		// output neutral tune
		set_tune(TONE_NOTIFY_NEUTRAL_TUNE);

		// temporary priority boost for the white blinking led to come trough
		rgbled_set_color_and_mode(led_control_s::COLOR_WHITE, led_control_s::MODE_BLINK_FAST, 3, 1);
		px4_usleep(200000);
	}

	return result;
}

bool calibrate_cancel_check(orb_advert_t *mavlink_log_pub, const hrt_abstime &calibration_started)
{
	bool ret = false;

	uORB::Subscription vehicle_command_sub{ORB_ID(vehicle_command)};
	vehicle_command_s cmd;

	while (vehicle_command_sub.update(&cmd)) {
		if (cmd.command == vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION) {
			// only handle commands sent after calibration started from external sources
			if ((cmd.timestamp > calibration_started) && cmd.from_external) {

				vehicle_command_ack_s command_ack{};

				if ((int)cmd.param1 == 0 &&
				    (int)cmd.param2 == 0 &&
				    (int)cmd.param3 == 0 &&
				    (int)cmd.param4 == 0 &&
				    (int)cmd.param5 == 0 &&
				    (int)cmd.param6 == 0) {

					command_ack.result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;
					mavlink_log_critical(mavlink_log_pub, CAL_QGC_CANCELLED_MSG);
					tune_positive(true);
					ret = true;

				} else {
					command_ack.result = vehicle_command_s::VEHICLE_CMD_RESULT_DENIED;
					mavlink_log_critical(mavlink_log_pub, "command denied during calibration: %u", cmd.command);
					tune_negative(true);
					ret = false;
				}

				command_ack.command = cmd.command;
				command_ack.target_system = cmd.source_system;
				command_ack.target_component = cmd.source_component;
				command_ack.timestamp = hrt_absolute_time();

				uORB::PublicationQueued<vehicle_command_ack_s> command_ack_pub{ORB_ID(vehicle_command_ack)};
				command_ack_pub.publish(command_ack);

				return ret;
			}
		}
	}

	return ret;
}
