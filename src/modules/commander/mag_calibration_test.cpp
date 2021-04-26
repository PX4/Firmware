/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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
 * Test code for the Magnetometer calibration routine
 * Run this test only using make tests TESTFILTER=mag_calibration
 *
 * @author Mathieu Bresciani <mathieu@auterion.com>
 */

#include <gtest/gtest.h>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/defines.h>

#include "lm_fit.hpp"
#include "mag_calibration_test_data.h"

using matrix::Vector3f;

class MagCalTest : public ::testing::Test
{
public:
	void generate2SidesMagData(float *x, float *y, float *z, unsigned int n_samples, float mag_str);

	/* Generate regularly spaced data on a sphere
	 * Ref.: How to generate equidistributed points on the surface of a sphere, Markus Deserno, 2004
	 */
	void generateRegularData(float *x, float *y, float *z, unsigned int n_samples, float mag_str);

	void modifyOffsetScale(float *x, float *y, float *z, unsigned int n_samples, Vector3f offsets, Vector3f scale_factors);
};

void MagCalTest::generate2SidesMagData(float *x, float *y, float *z, unsigned int n_samples, float mag_str)
{
	float psi = 0.f;
	float theta = 0.f;
	const float d_angle = 2.f * M_PI_F / float(n_samples / 2);

	for (int i = 0; i < int(n_samples / 2); i++) {
		x[i] = mag_str * sinf(psi);
		y[i] = mag_str * cosf(psi);
		z[i] = 0.f;
		psi += d_angle;
	}

	for (int i = int(n_samples / 2); i < int(n_samples); i++) {
		x[i] = mag_str * sinf(theta);
		y[i] = 0.f;
		z[i] = mag_str * cosf(theta);
		theta += d_angle;
	}
}

void MagCalTest::generateRegularData(float *x, float *y, float *z, unsigned int n_samples, float mag_str)
{
	const float a = 4.f * M_PI_F * mag_str * mag_str / n_samples;
	const float d = sqrtf(a);
	const int m_theta = static_cast<int>(M_PI_F / d);
	const float d_theta = M_PI_F / static_cast<float>(m_theta);
	const float d_phi = a / d_theta;

	unsigned int n_count = 0;

	for (int m = 0; m < m_theta; m++) {
		const float theta = M_PI_F * (m + 0.5f) / static_cast<float>(m_theta);
		const int m_phi = static_cast<int>(2.f * M_PI_F * sinf(theta / d_phi));

		for (int n = 0; n < m_phi; n++) {
			const float phi = 2.f * M_PI_F * n / static_cast<float>(m_phi);
			x[n_count] = mag_str * sinf(theta) * cosf(phi);
			y[n_count] = mag_str * sinf(theta) * sinf(phi);
			z[n_count] = mag_str * cosf(theta);
			n_count++;
		}
	}

	if (n_count > n_samples) {
		printf("Error placing samples, n = %d\n", n_count);
		return;
	}

	// Padd with constant data
	while (n_count < n_samples) {
		x[n_count] = x[n_count - 1];
		y[n_count] = y[n_count - 1];
		z[n_count] = z[n_count - 1];
		n_count++;
	}
}

void MagCalTest::modifyOffsetScale(float *x, float *y, float *z, unsigned int n_samples, Vector3f offsets,
				   Vector3f scale_factors)
{
	for (unsigned int k = 0; k < n_samples; k++) {
		x[k] = x[k] * scale_factors(0) + offsets(0);
		y[k] = y[k] * scale_factors(1) + offsets(1);
		z[k] = z[k] * scale_factors(2) + offsets(2);
	}
}

TEST_F(MagCalTest, sphere2Sides)
{
	// GIVEN: a dataset of points located on two orthogonal circles
	// perfectly centered on the origin
	static constexpr unsigned int N_SAMPLES = 240;

	const float mag_str_true = 0.4f;
	const Vector3f offset_true;
	const Vector3f scale_true = {1.f, 1.f, 1.f};

	float x[N_SAMPLES];
	float y[N_SAMPLES];
	float z[N_SAMPLES];

	generate2SidesMagData(x, y, z, N_SAMPLES, mag_str_true);

	// WHEN: fitting a sphere with the data and given a wrong initial radius
	sphere_params sphere;
	sphere.diag = {1.f, 1.f, 1.f};
	sphere.radius = 0.2;
	int success = lm_mag_fit(x, y, z, N_SAMPLES, sphere, false);

	// THEN: the algorithm should converge in a single step
	EXPECT_EQ(success, PX4_OK);
	EXPECT_NEAR(sphere.radius, mag_str_true, 0.001f) << "radius: " << sphere.radius;
	EXPECT_NEAR(sphere.offset(0), offset_true(0), 0.001f) << "offset X: " << sphere.offset(0);
	EXPECT_NEAR(sphere.offset(1), offset_true(1), 0.001f) << "offset Y: " << sphere.offset(1);
	EXPECT_NEAR(sphere.offset(2), offset_true(2), 0.001f) << "offset Z: " << sphere.offset(2);
	EXPECT_NEAR(sphere.diag(0), scale_true(0), 0.001f) << "scale X: " << sphere.diag(0);
	EXPECT_NEAR(sphere.diag(1), scale_true(1), 0.001f) << "scale Y: " << sphere.diag(1);
	EXPECT_NEAR(sphere.diag(2), scale_true(2), 0.001f) << "scale Z: " << sphere.diag(2);
}

TEST_F(MagCalTest, sphereRegularlySpaced)
{
	// GIVEN: a dataset of regularly spaced points
	// on a perfect sphere but not centered on the origin
	static constexpr unsigned int N_SAMPLES = 240;

	const float mag_str_true = 0.4f;
	const Vector3f offset_true = {-1.07f, 0.35f, -0.78f};
	const Vector3f scale_true = {1.f, 1.f, 1.f};

	float x[N_SAMPLES];
	float y[N_SAMPLES];
	float z[N_SAMPLES];
	generateRegularData(x, y, z, N_SAMPLES, mag_str_true);
	modifyOffsetScale(x, y, z, N_SAMPLES, offset_true, scale_true);

	// WHEN: fitting a sphere to the data
	sphere_params sphere;
	sphere.diag = {1.f, 1.f, 1.f};
	sphere.radius = 0.2;
	int success = lm_mag_fit(x, y, z, N_SAMPLES, sphere, false);

	// THEN: the algorithm should converge in a few iterations and
	// find the correct parameters
	EXPECT_EQ(success, PX4_OK);
	EXPECT_NEAR(sphere.radius, mag_str_true, 0.001f) << "radius: " << sphere.radius;
	EXPECT_NEAR(sphere.offset(0), offset_true(0), 0.001f) << "offset X: " << sphere.offset(0);
	EXPECT_NEAR(sphere.offset(1), offset_true(1), 0.001f) << "offset Y: " << sphere.offset(1);
	EXPECT_NEAR(sphere.offset(2), offset_true(2), 0.001f) << "offset Z: " << sphere.offset(2);
	EXPECT_NEAR(sphere.diag(0), scale_true(0), 0.001f) << "scale X: " << scale_true(0);
	EXPECT_NEAR(sphere.diag(1), scale_true(1), 0.001f) << "scale Y: " << scale_true(1);
	EXPECT_NEAR(sphere.diag(2), scale_true(2), 0.001f) << "scale Z: " << scale_true(2);
}

TEST_F(MagCalTest, replayTestData)
{
	// GIVEN: a real test dataset with large offsets
	// and where the two first iterations of the LM algorithm
	// produces a negative radius and a constant fitness value
	constexpr unsigned int N_SAMPLES = 231;

	const float mag_str_true = 0.4f;
	const Vector3f offset_true = {-0.18f, 0.05f, -0.58f};

	// WHEN: fitting a sphere to the data
	sphere_params sphere;
	sphere.diag = {1.f, 1.f, 1.f};
	sphere.radius = 0.2;
	int sphere_success = lm_mag_fit(mag_data1_x, mag_data1_y, mag_data1_z, N_SAMPLES, sphere, false);

	// THEN: the algorithm should converge and find the correct parameters
	EXPECT_EQ(sphere_success, PX4_OK);
	EXPECT_NEAR(sphere.radius, mag_str_true, 0.1f) << "radius: " << sphere.radius;
	EXPECT_NEAR(sphere.offset(0), offset_true(0), 0.01f) << "offset X: " << sphere.offset(0);
	EXPECT_NEAR(sphere.offset(1), offset_true(1), 0.01f) << "offset Y: " << sphere.offset(1);
	EXPECT_NEAR(sphere.offset(2), offset_true(2), 0.01f) << "offset Z: " << sphere.offset(2);

	printf("Ellipsoid fit\n");
	sphere_params ellipsoid;
	ellipsoid.diag = {1.f, 1.f, 1.f};
	ellipsoid.radius = 0.2;
	int ellipsoid_step_1_success = lm_mag_fit(mag_data1_x, mag_data1_y, mag_data1_z, N_SAMPLES, ellipsoid, false);
	int ellipsoid_success = lm_mag_fit(mag_data1_x, mag_data1_y, mag_data1_z, N_SAMPLES, ellipsoid, true);
	const Vector3f scale_true = {1.f, 1.06f, 0.94f};

	EXPECT_EQ(ellipsoid_step_1_success, PX4_OK);
	EXPECT_EQ(ellipsoid_success, PX4_OK);
	EXPECT_NEAR(ellipsoid.radius, mag_str_true, 0.1f) << "radius: " << sphere.radius;
	EXPECT_NEAR(ellipsoid.offset(0), offset_true(0), 0.01f) << "offset X: " << ellipsoid.offset(0);
	EXPECT_NEAR(ellipsoid.offset(1), offset_true(1), 0.01f) << "offset Y: " << ellipsoid.offset(1);
	EXPECT_NEAR(ellipsoid.offset(2), offset_true(2), 0.01f) << "offset Z: " << ellipsoid.offset(2);
	EXPECT_NEAR(ellipsoid.diag(0), scale_true(0), 0.01f) << "scale X: " << ellipsoid.diag(0);
	EXPECT_NEAR(ellipsoid.diag(1), scale_true(1), 0.01f) << "scale Y: " << ellipsoid.diag(1);
	EXPECT_NEAR(ellipsoid.diag(2), scale_true(2), 0.01f) << "scale Z: " << ellipsoid.diag(2);
}
