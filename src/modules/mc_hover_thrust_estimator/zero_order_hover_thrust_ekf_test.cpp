/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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
 * Test code for the Zero Order Hover Thrust Estimator
 * Run this test only using make tests TESTFILTER=zero_order_hover_thrust_ekf
 */

#include <gtest/gtest.h>
#include <matrix/matrix/math.hpp>
#include <random>

#include "zero_order_hover_thrust_ekf.hpp"

using namespace matrix;

class ZeroOrderHoverThrustEkfTest : public ::testing::Test
{
public:
	struct Status {
		float hover_thrust;
		float hover_thrust_var;
		float innov;
		float innov_var;
		float innov_test_ratio;
		float accel_noise_var;
	};

	ZeroOrderHoverThrustEkfTest()
	{
		_random_generator.seed(42);
	}
	float computeAccelFromThrustAndHoverThrust(float thrust, float hover_thrust);
	Status runEkf(float hover_thrust_true, float thrust, float time, float accel_noise = 0.f,
		      float thr_noise = 0.f);

private:
	ZeroOrderHoverThrustEkf _ekf{};
	static constexpr float _dt = 0.02f;

	std::normal_distribution<float> _standard_normal_distribution;
	std::default_random_engine _random_generator; // Pseudo-random generator with constant seed

protected:
	static constexpr float _accel_noise_var_min = 1.f; // Constrained in the implementation
};

float ZeroOrderHoverThrustEkfTest::computeAccelFromThrustAndHoverThrust(float thrust, float hover_thrust)
{
	return CONSTANTS_ONE_G * thrust / hover_thrust - CONSTANTS_ONE_G;
}

ZeroOrderHoverThrustEkfTest::Status ZeroOrderHoverThrustEkfTest::runEkf(float hover_thrust_true, float thrust,
		float time,
		float accel_noise, float thr_noise)
{
	Status status{};

	for (float t = 0.f; t <= time; t += _dt) {
		_ekf.predict(_dt);
		float noisy_thrust =  thrust + thr_noise * _standard_normal_distribution(_random_generator);
		float accel_theory = computeAccelFromThrustAndHoverThrust(thrust, hover_thrust_true);
		float noisy_accel =  accel_theory + accel_noise * _standard_normal_distribution(_random_generator);
		_ekf.fuseAccZ(noisy_accel, noisy_thrust);
	}

	status.hover_thrust = _ekf.getHoverThrustEstimate();
	status.hover_thrust_var = _ekf.getHoverThrustEstimateVar();

	status.innov = _ekf.getInnovation();
	status.innov_var = _ekf.getInnovationVar();
	status.innov_test_ratio = _ekf.getInnovationTestRatio();
	status.accel_noise_var = _ekf.getAccelNoiseVar();

	return status;
}

TEST_F(ZeroOrderHoverThrustEkfTest, testStaticCase)
{
	// GIVEN: a vehicle at hover, (the estimator starting at the true value)
	const float thrust = 0.5f;
	const float hover_thrust_true = 0.5f;

	// WHEN: we input noiseless data and run the filter
	ZeroOrderHoverThrustEkfTest::Status status = runEkf(hover_thrust_true, thrust, 2.f);

	// THEN: The estimate should not move and its variance decrease quickly
	EXPECT_NEAR(status.hover_thrust, hover_thrust_true, 1e-4f);
	EXPECT_NEAR(status.hover_thrust_var, 0.f, 1e-3f);
	EXPECT_NEAR(status.accel_noise_var, _accel_noise_var_min,
		    1.f); // The noise learning is slow and takes more time to go to zero
}

TEST_F(ZeroOrderHoverThrustEkfTest, testStaticConvergence)
{
	// GIVEN: a vehicle at hover, but the estimator is starting at hover_thrust = 0.5
	const float thrust = 0.72f;
	const float hover_thrust_true = 0.72f;

	// WHEN: we input noiseless data and run the filter
	ZeroOrderHoverThrustEkfTest::Status status = runEkf(hover_thrust_true, thrust, 2.f);

	// THEN: the state should converge to the true value and its variance decrease
	EXPECT_NEAR(status.hover_thrust, hover_thrust_true, 1e-2f);
	EXPECT_NEAR(status.hover_thrust_var, 0.f, 1e-3f);
	EXPECT_NEAR(status.accel_noise_var, _accel_noise_var_min,
		    1.f); // The noise learning is slow and takes more time to go to zero
}

TEST_F(ZeroOrderHoverThrustEkfTest, testStaticConvergenceWithNoise)
{
	// GIVEN: a vehicle at hover, the estimator starts with the wrong estimate and the measurements are noisy
	const float sigma_noise = 3.f;
	const float noise_var = sigma_noise * sigma_noise;
	const float thrust = 0.72f;
	const float hover_thrust_true = 0.72f;
	const float t_sim = 10.f;

	// WHEN: we input noisy accel data and run the filter
	ZeroOrderHoverThrustEkfTest::Status status = runEkf(hover_thrust_true, thrust, t_sim, sigma_noise);

	// THEN: the estimate should converge and the accel noise variance should be close to the true noise value
	EXPECT_NEAR(status.hover_thrust, hover_thrust_true, 5e-2f);
	EXPECT_NEAR(status.hover_thrust_var, 0.f, 1e-3f);
	EXPECT_NEAR(status.accel_noise_var, noise_var, 0.2f * noise_var);
}

TEST_F(ZeroOrderHoverThrustEkfTest, testLargeAccelNoiseAndBias)
{
	// GIVEN: a vehicle descending, the estimator starts with the wrong estimate, the measurements are really noisy
	const float sigma_noise = 7.f;
	const float noise_var = sigma_noise * sigma_noise;
	const float thrust = 0.4f; // Below hover thrust
	const float hover_thrust_true = 0.72f;
	const float t_sim = 15.f;

	// WHEN: we input noisy accel data and run the filter
	ZeroOrderHoverThrustEkfTest::Status status = runEkf(hover_thrust_true, thrust, t_sim, sigma_noise);

	// THEN: the estimate should converge and the accel noise variance should be close to the true noise value
	EXPECT_NEAR(status.hover_thrust, hover_thrust_true, 7e-2);
	EXPECT_NEAR(status.hover_thrust_var, 0.f, 1e-3f);
	EXPECT_NEAR(status.accel_noise_var, noise_var, 0.2f * noise_var);
}

TEST_F(ZeroOrderHoverThrustEkfTest, testThrustAndAccelNoise)
{
	// GIVEN: a vehicle climbing, the estimator starts with the wrong estimate, the measurements
	// and the input thrust are noisy
	const float accel_noise = 2.f;
	const float accel_var = accel_noise * accel_noise;
	const float thr_noise = 0.01f;
	const float thrust = 0.72f; // Above hover thrust
	const float hover_thrust_true = 0.6f;
	const float t_sim = 15.f;

	// WHEN: we input noisy accel and thrust data, and run the filter
	ZeroOrderHoverThrustEkfTest::Status status = runEkf(hover_thrust_true, thrust, t_sim, accel_noise, thr_noise);

	// THEN: the estimate should converge and the accel noise variance should be close to the true noise value
	EXPECT_NEAR(status.hover_thrust, hover_thrust_true, 5e-2f);
	EXPECT_NEAR(status.hover_thrust_var, 0.f, 1e-3f);
	// Because of the nonlinear measurment model and the thust noise, the accel noise estimation is a bit worse
	EXPECT_NEAR(status.accel_noise_var, accel_var, 0.4f * accel_var);
}

TEST_F(ZeroOrderHoverThrustEkfTest, testHoverThrustJump)
{
	// GIVEN: a vehicle hovering, the estimator starts with the wrong estimate, the measurements
	// and the input thrust are noisy
	const float accel_noise = 2.f;
	const float accel_var = accel_noise * accel_noise;
	const float thr_noise = 0.01f;
	float thrust = 0.8; // At hover
	float hover_thrust_true = 0.8f;
	float t_sim = 10.f;

	// WHEN: we input noisy accel and thrust data, and run the filter
	ZeroOrderHoverThrustEkfTest::Status status = runEkf(hover_thrust_true, thrust, t_sim, accel_noise, thr_noise);
	// THEN: change the hover thrust and the current thrust (the velocity controller responds quickly)
	// Note that this is an extreme jump in hover thrust
	thrust = 0.3;
	hover_thrust_true = 0.3f;
	t_sim = 10.f;
	status = runEkf(hover_thrust_true, thrust, t_sim, accel_noise, thr_noise);

	// THEN: the estimate should converge to the new hover thrust and the accel noise variance should
	// be close to the true noise value
	EXPECT_NEAR(status.hover_thrust, hover_thrust_true, 5e-2f);
	EXPECT_NEAR(status.hover_thrust_var, 0.f, 1e-3f);
	// After a recovery, the noise variance estimate takes more time to converge back to the true value
	EXPECT_NEAR(status.accel_noise_var, accel_var, 2.f * accel_var);
}
