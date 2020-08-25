/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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

/*
 * @file KalmanFilter.h
 * Simple Kalman Filter for variable gain low-passing
 *
 * Constant velocity model. Prediction according to
 * x_{k+1} = A * x_{k}
 * with A = [1 dt; 0 1]
 *
 * Update with a direct measurement of the first state:
 * H = [1 0]
 *
 * @author Nicolas de Palezieux (Sunflower Labs) <ndepal@gmail.com>
 *
 */

#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <matrix/Matrix.hpp>
#include <matrix/Vector.hpp>

#pragma once

namespace landing_target_estimator
{
class KalmanFilter
{
public:
	/**
	 * Default constructor, state not initialized
	 */
	KalmanFilter() {};

	/**
	 * Constructor, initialize state
	 */
	KalmanFilter(matrix::Vector<float, 2> &initial, matrix::Matrix<float, 2, 2> &covInit);

	/**
	 * Default desctructor
	 */
	virtual ~KalmanFilter() {};

	/**
	 * Initialize filter state
	 * @param initial initial state
	 * @param covInit initial covariance
	 */
	void init(matrix::Vector<float, 2> &initial, matrix::Matrix<float, 2, 2> &covInit);

	/**
	 * Initialize filter state, only specifying diagonal covariance elements
	 * @param initial0  first initial state
	 * @param initial1  second initial state
	 * @param covInit00 initial variance of first state
	 * @param covinit11 initial variance of second state
	 */
	void init(float initial0, float initial1, float covInit00, float covInit11);

	/**
	 * Predict the state with an external acceleration estimate
	 * @param dt            Time delta in seconds since last state change
	 * @param acc           Acceleration estimate
	 * @param acc_unc       Variance of acceleration estimate
	 */
	void predict(float dt, float acc, float acc_unc);

	/**
	 * Update the state estimate with a measurement
	 * @param meas    state measeasurement
	 * @param measUnc measurement uncertainty
	 * @return update success (measurement not rejected)
	 */
	bool update(float meas, float measUnc);

	/**
	 * Get the current filter state
	 * @param x1 State
	 */
	void getState(matrix::Vector<float, 2> &state);

	/**
	 * Get the current filter state
	 * @param state0 First state
	 * @param state1 Second state
	 */
	void getState(float &state0, float &state1);

	/**
	 * Get state covariance
	 * @param covariance Covariance of the state
	 */
	void getCovariance(matrix::Matrix<float, 2, 2> &covariance);

	/**
	 * Get state variances (diagonal elements)
	 * @param cov00 Variance of first state
	 * @param cov11 Variance of second state
	 */
	void getCovariance(float &cov00, float &cov11);

	/**
	 * Get measurement innovation and covariance of last update call
	 * @param innov Measurement innovation
	 * @param innovCov Measurement innovation covariance
	 */
	void getInnovations(float &innov, float &innovCov);

private:
	matrix::Vector<float, 2> _x; // state

	matrix::Matrix<float, 2, 2> _covariance; // state covariance

	float _residual{0.0f}; // residual of last measurement update

	float _innovCov{0.0f}; // innovation covariance of last measurement update
};
} // namespace landing_target_estimator
