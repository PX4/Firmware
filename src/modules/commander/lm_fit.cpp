/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "lm_fit.hpp"

struct iteration_result {
	float gradient_damping;
	float cost;
	enum class STATUS {
		SUCCESS, FAILURE
	} result = STATUS::SUCCESS;
};

void lm_sphere_fit_iteration(const float x[], const float y[], const float z[],
			     unsigned int samples_collected, sphere_params &params, iteration_result &result)
{
	// Run Sphere Fit using Levenberg Marquardt LSq Fit
	const float lma_damping = 10.f;
	float fitness = result.cost;
	float fit1 = 0.f;
	float fit2 = 0.f;

	matrix::SquareMatrix<float, 4> JTJ;
	float JTFI[4] {};
	float residual = 0.0f;

	// Gauss Newton Part common for all kind of extensions including LM
	for (uint16_t k = 0; k < samples_collected; k++) {

		float sphere_jacob[4];
		//Calculate Jacobian
		float A = (params.diag(0)    * (x[k] - params.offset(0))) + (params.offdiag(0) * (y[k] - params.offset(1))) +
			  (params.offdiag(1) * (z[k] - params.offset(2)));
		float B = (params.offdiag(0) * (x[k] - params.offset(0))) + (params.diag(1)    * (y[k] - params.offset(1))) +
			  (params.offdiag(2) * (z[k] - params.offset(2)));
		float C = (params.offdiag(1) * (x[k] - params.offset(0))) + (params.offdiag(2) * (y[k] - params.offset(1))) +
			  (params.diag(2)    * (z[k] - params.offset(2)));
		float length = sqrtf(A * A + B * B + C * C);

		// 0: partial derivative (radius wrt fitness fn) fn operated on sample
		sphere_jacob[0] = 1.0f;
		// 1-3: partial derivative (offsets wrt fitness fn) fn operated on sample
		sphere_jacob[1] = 1.0f * (((params.diag(0)    * A) + (params.offdiag(0) * B) + (params.offdiag(1) * C)) / length);
		sphere_jacob[2] = 1.0f * (((params.offdiag(0) * A) + (params.diag(1)    * B) + (params.offdiag(2) * C)) / length);
		sphere_jacob[3] = 1.0f * (((params.offdiag(1) * A) + (params.offdiag(2) * B) + (params.diag(2)    * C)) / length);
		residual = params.radius - length;

		for (uint8_t i = 0; i < 4; i++) {
			// compute JTJ
			for (uint8_t j = 0; j < 4; j++) {
				JTJ(i, j) += sphere_jacob[i] * sphere_jacob[j];
			}

			JTFI[i] += sphere_jacob[i] * residual;
		}
	}


	//------------------------Levenberg-Marquardt-part-starts-here---------------------------------//
	// refer: http://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm#Choice_of_damping_parameter
	float fit1_params[4] = {params.radius, params.offset(0), params.offset(1), params.offset(2)};
	float fit2_params[4];
	memcpy(fit2_params, fit1_params, sizeof(fit1_params));
	JTJ = (JTJ + JTJ.transpose()) * 0.5f; // fix numerical issues
	matrix::SquareMatrix<float, 4> JTJ2 = JTJ;

	for (uint8_t i = 0; i < 4; i++) {
		JTJ(i, i) += result.gradient_damping;
		JTJ2(i, i) += result.gradient_damping / lma_damping;
	}

	if (!JTJ.I(JTJ)) {
		result.result = iteration_result::STATUS::FAILURE;
		return;
	}

	if (!JTJ2.I(JTJ2)) {
		result.result = iteration_result::STATUS::FAILURE;
		return;

	}

	for (uint8_t row = 0; row < 4; row++) {
		for (uint8_t col = 0; col < 4; col++) {
			fit1_params[row] -= JTFI[col] * JTJ(row, col);
			fit2_params[row] -= JTFI[col] * JTJ2(row, col);
		}
	}

	// Calculate mean squared residuals
	for (uint16_t k = 0; k < samples_collected; k++) {
		float A = (params.diag(0)    * (x[k] - fit1_params[1])) + (params.offdiag(0) * (y[k] - fit1_params[2])) +
			  (params.offdiag(1) *
			   (z[k] + fit1_params[3]));
		float B = (params.offdiag(0) * (x[k] - fit1_params[1])) + (params.diag(1)    * (y[k] - fit1_params[2])) +
			  (params.offdiag(2) *
			   (z[k] + fit1_params[3]));
		float C = (params.offdiag(1) * (x[k] - fit1_params[1])) + (params.offdiag(2) * (y[k] - fit1_params[2])) + (params.diag(
					2)    *
				(z[k] - fit1_params[3]));
		float length = sqrtf(A * A + B * B + C * C);
		residual = fit1_params[0] - length;
		fit1 += residual * residual;

		A = (params.diag(0)    * (x[k] - fit2_params[1])) + (params.offdiag(0) * (y[k] - fit2_params[2])) + (params.offdiag(1) *
				(z[k] - fit2_params[3]));
		B = (params.offdiag(0) * (x[k] - fit2_params[1])) + (params.diag(1)    * (y[k] - fit2_params[2])) + (params.offdiag(2) *
				(z[k] - fit2_params[3]));
		C = (params.offdiag(1) * (x[k] - fit2_params[1])) + (params.offdiag(2) * (y[k] - fit2_params[2])) + (params.diag(2)    *
				(z[k] - fit2_params[3]));
		length = sqrtf(A * A + B * B + C * C);
		residual = fit2_params[0] - length;
		fit2 += residual * residual;
	}

	fit1 = sqrtf(fit1) / samples_collected;
	fit2 = sqrtf(fit2) / samples_collected;

	if (fit1 > result.cost && fit2 > result.cost) {
		result.gradient_damping *= lma_damping;

	} else if (fit2 < result.cost && fit2 < fit1) {
		result.gradient_damping /= lma_damping;
		memcpy(fit1_params, fit2_params, sizeof(fit1_params));
		fitness = fit2;

	} else if (fit1 < result.cost) {
		fitness = fit1;
	}

	//--------------------Levenberg-Marquardt-part-ends-here--------------------------------//

	if (PX4_ISFINITE(fitness) && fitness <= result.cost) {
		result.cost = fitness;
		params.radius = fit1_params[0];
		params.offset(0) = fit1_params[1];
		params.offset(1) = fit1_params[2];
		params.offset(2) = fit1_params[3];
		result.result = iteration_result::STATUS::SUCCESS;

	} else {
		result.result = iteration_result::STATUS::FAILURE;
	}
}

void lm_ellipsoid_fit_iteration(const float x[], const float y[], const float z[],
				unsigned int samples_collected, sphere_params &params, iteration_result &result)
{
	// Run Sphere Fit using Levenberg Marquardt LSq Fit
	const float lma_damping = 10.0f;
	float fitness = result.cost;
	float fit1 = 0.0f;
	float fit2 = 0.0f;

	matrix::SquareMatrix<float, 9> JTJ;
	float JTFI[9] {};
	float residual = 0.0f;
	float ellipsoid_jacob[9];

	// Gauss Newton Part common for all kind of extensions including LM
	for (uint16_t k = 0; k < samples_collected; k++) {

		// Calculate Jacobian
		float A = (params.diag(0)    * (x[k] - params.offset(0))) + (params.offdiag(0) * (y[k] - params.offset(1))) +
			  (params.offdiag(1) * (z[k] - params.offset(2)));
		float B = (params.offdiag(0) * (x[k] - params.offset(0))) + (params.diag(1)    * (y[k] - params.offset(1))) +
			  (params.offdiag(2) * (z[k] - params.offset(2)));
		float C = (params.offdiag(1) * (x[k] - params.offset(0))) + (params.offdiag(2) * (y[k] - params.offset(1))) +
			  (params.diag(2)    * (z[k] - params.offset(2)));
		float length = sqrtf(A * A + B * B + C * C);
		residual = params.radius - length;
		fit1 += residual * residual;
		// 0-2: partial derivative (offset wrt fitness fn) fn operated on sample
		ellipsoid_jacob[0] = 1.0f * (((params.diag(0)    * A) + (params.offdiag(0) * B) + (params.offdiag(1) * C)) / length);
		ellipsoid_jacob[1] = 1.0f * (((params.offdiag(0) * A) + (params.diag(1)    * B) + (params.offdiag(2) * C)) / length);
		ellipsoid_jacob[2] = 1.0f * (((params.offdiag(1) * A) + (params.offdiag(2) * B) + (params.diag(2)    * C)) / length);
		// 3-5: partial derivative (diag offset wrt fitness fn) fn operated on sample
		ellipsoid_jacob[3] = -1.0f * ((x[k] - params.offset(0)) * A) / length;
		ellipsoid_jacob[4] = -1.0f * ((y[k] - params.offset(1)) * B) / length;
		ellipsoid_jacob[5] = -1.0f * ((z[k] - params.offset(2)) * C) / length;
		// 6-8: partial derivative (off-diag offset wrt fitness fn) fn operated on sample
		ellipsoid_jacob[6] = -1.0f * (((y[k] - params.offset(1)) * A) + ((x[k] - params.offset(0)) * B)) / length;
		ellipsoid_jacob[7] = -1.0f * (((z[k] - params.offset(2)) * A) + ((x[k] - params.offset(0)) * C)) / length;
		ellipsoid_jacob[8] = -1.0f * (((z[k] - params.offset(2)) * B) + ((y[k] - params.offset(1)) * C)) / length;

		for (uint8_t i = 0; i < 9; i++) {
			// compute JTJ
			for (uint8_t j = 0; j < 9; j++) {
				JTJ(i, j) += ellipsoid_jacob[i] * ellipsoid_jacob[j];
			}

			JTFI[i] += ellipsoid_jacob[i] * residual;
		}
	}


	//------------------------Levenberg-Marquardt-part-starts-here---------------------------------//
	// refer: http://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm#Choice_of_damping_parameter
	float fit1_params[9] = {params.offset(0), params.offset(1), params.offset(2), params.diag(0), params.diag(1), params.diag(2), params.offdiag(0), params.offdiag(1), params.offdiag(2)};
	float fit2_params[9];
	memcpy(fit2_params, fit1_params, sizeof(fit1_params));
	matrix::SquareMatrix<float, 9> JTJ2 = JTJ;

	for (uint8_t i = 0; i < 9; i++) {
		JTJ(i, i) += result.gradient_damping;
		JTJ2(i, i) += result.gradient_damping / lma_damping;
	}


	if (!JTJ.I(JTJ)) {
		result.result = iteration_result::STATUS::FAILURE;
		return;
	}

	if (!JTJ2.I(JTJ2)) {
		result.result = iteration_result::STATUS::FAILURE;
		return;
	}

	for (uint8_t row = 0; row < 9; row++) {
		for (uint8_t col = 0; col < 9; col++) {
			fit1_params[row] -= JTFI[col] * JTJ(row, col);
			fit2_params[row] -= JTFI[col] * JTJ2(row, col);
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
		residual = params.radius - length;
		fit1 += residual * residual;

		A = (fit2_params[3]    * (x[k] - fit2_params[0])) + (fit2_params[6] * (y[k] - fit2_params[1])) + (fit2_params[7] *
				(z[k] - fit2_params[2]));
		B = (fit2_params[6] * (x[k] - fit2_params[0])) + (fit2_params[4]   * (y[k] - fit2_params[1])) + (fit2_params[8] *
				(z[k] - fit2_params[2]));
		C = (fit2_params[7] * (x[k] - fit2_params[0])) + (fit2_params[8] * (y[k] - fit2_params[1])) + (fit2_params[5]    *
				(z[k] - fit2_params[2]));
		length = sqrtf(A * A + B * B + C * C);
		residual = params.radius - length;
		fit2 += residual * residual;
	}

	fit1 = sqrtf(fit1) / samples_collected;
	fit2 = sqrtf(fit2) / samples_collected;

	if (fit1 > result.cost && fit2 > result.cost) {
		result.gradient_damping *= lma_damping;

	} else if (fit2 < result.cost && fit2 < fit1) {
		result.gradient_damping /= lma_damping;
		memcpy(fit1_params, fit2_params, sizeof(fit1_params));
		fitness = fit2;

	} else if (fit1 < result.cost) {
		fitness = fit1;
	}

	//--------------------Levenberg-Marquardt-part-ends-here--------------------------------//
	if (PX4_ISFINITE(fitness) && fitness <= result.cost) {
		result.cost = fitness;
		params.offset(0) = fit1_params[0];
		params.offset(1) = fit1_params[1];
		params.offset(2) = fit1_params[2];
		params.diag(0) = fit1_params[3];
		params.diag(1) = fit1_params[4];
		params.diag(2) = fit1_params[5];
		params.offdiag(0) = fit1_params[6];
		params.offdiag(1) = fit1_params[7];
		params.offdiag(2) = fit1_params[8];
		result.result = iteration_result::STATUS::SUCCESS;

	} else {
		result.result = iteration_result::STATUS::FAILURE;
	}
}



int lm_mag_fit(const float x[], const float y[], const float z[], unsigned int samples_collected, sphere_params &params,
	       bool full_ellipsoid)
{

	const int max_iterations = 100;
	const int min_iterations = 10;
	const float cost_threshold = 0.01;
	const float step_threshold = 0.001;

	const float min_radius = 0.2;
	const float max_radius = 0.7;

	iteration_result iter;
	iter.cost = 1e30f;
	iter.gradient_damping = 1;

	bool success = false;

	for (int i = 0; i < max_iterations; i++) {
		if (full_ellipsoid) {
			lm_ellipsoid_fit_iteration(x, y, z, samples_collected, params, iter);

		} else {
			lm_sphere_fit_iteration(x, y, z, samples_collected, params, iter);
		}

		if (iter.result == iteration_result::STATUS::SUCCESS
		    && min_radius < params.radius && params.radius < max_radius
		    && i > min_iterations && (iter.cost < cost_threshold || iter.gradient_damping < step_threshold)) {
			success = true;
			break;
		}
	}

	if (success) {
		return PX4_OK;
	}

	return 1;
}

