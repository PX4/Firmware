/****************************************************************************
 *
 *  Copyright (C) 2020 PX4 Development Team. All rights reserved.
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
 * @file RotationTest.cpp
 * Tests for rotations.
 */

#include <gtest/gtest.h>

#include "rotation.h"


TEST(Rotations, matrix_vs_3f)
{
	//iterate through all defined rotations
	for (size_t i = 0; i < (size_t)Rotation::ROTATION_MAX; i++) {

		// GIVEN: an initial vector and a rotation
		const matrix::Vector3f original = {1.f, 2.f, 3.f};
		const enum Rotation	rotation = static_cast<Rotation>(i);

		// WHEN: we transform the vector using the rotate_3f function and the rotation matrix
		matrix::Vector3f transformed_3f = original;
		rotate_3f(rotation, transformed_3f(0), transformed_3f(1), transformed_3f(2));

		matrix::Dcmf matrix = get_rot_matrix(rotation);
		matrix::Vector3f transformed_mat = matrix * original;

		// THEN: the results should be the same
		EXPECT_NEAR(transformed_mat(0), transformed_3f(0), 10e-6);
		EXPECT_NEAR(transformed_mat(1), transformed_3f(1), 10e-6);
		EXPECT_NEAR(transformed_mat(2), transformed_3f(2), 10e-6);
	}
}

TEST(Rotations, duplicates)
{
	// find all identical rotations to skip (needs to be kept in sync with mag calibration auto rotation)
	for (size_t i = 0; i < (size_t)Rotation::ROTATION_MAX; i++) {

		// GIVEN: an initial vector and a rotation
		const matrix::Vector3f original = {1.f, 2.f, 3.f};
		const enum Rotation rotation_1 = static_cast<Rotation>(i);

		// WHEN: we transform the vector using the rotate_3f function and the rotation matrix
		matrix::Vector3f transformed_1 = original;
		rotate_3f(rotation_1, transformed_1(0), transformed_1(1), transformed_1(2));

		for (size_t j = 0; j < (size_t)Rotation::ROTATION_MAX; j++) {

			const enum Rotation rotation_2 = static_cast<Rotation>(j);

			matrix::Vector3f transformed_2 = original;
			rotate_3f(rotation_2, transformed_2(0), transformed_2(1), transformed_2(2));

			if (i != j) {
				// ROTATION_PITCH_180_YAW_90 (26) = ROTATION_ROLL_180_YAW_270 (14)
				if (i == ROTATION_ROLL_180_YAW_270 && j == ROTATION_PITCH_180_YAW_90) { continue; }

				if (j == ROTATION_ROLL_180_YAW_270 && i == ROTATION_PITCH_180_YAW_90) { continue; }


				// ROTATION_ROLL_180_YAW_90 (10) = ROTATION_PITCH_180_YAW_270 (27)
				if (i == ROTATION_ROLL_180_YAW_90 && j == ROTATION_PITCH_180_YAW_270) { continue; }

				if (j == ROTATION_ROLL_180_YAW_90 && i == ROTATION_PITCH_180_YAW_270) { continue; }


				// otherwise all rotations should be different
				ASSERT_GT((transformed_1 - transformed_2).norm(), 0) << "Rotation " << i << " and " << j << " equal";
			}
		}
	}
}
