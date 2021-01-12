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
 * @file LandingTargetEstimator.h
 * Landing target position estimator. Filter and publish the position of a landing target on the ground as observed by an onboard sensor.
 *
 * @author Nicolas de Palezieux (Sunflower Labs) <ndepal@gmail.com>
 * @author Mohammed Kabir <kabir@uasys.io>
 *
 */

#pragma once

#include <px4_platform_common/workqueue.h>
#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/irlock_report.h>
#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/landing_target_innovations.h>
#include <uORB/topics/parameter_update.h>
#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <matrix/Matrix.hpp>
#include "KalmanFilter.h"

using namespace time_literals;

namespace landing_target_estimator
{

class LandingTargetEstimator
{
public:

	LandingTargetEstimator();
	virtual ~LandingTargetEstimator() = default;

	/*
	 * Get new measurements and update the state estimate
	 */
	void update();

protected:

	/*
	 * Update uORB topics.
	 */
	void _update_topics();

	/*
	 * Update parameters.
	 */
	void _update_params();

	/* timeout after which filter is reset if target not seen */
	static constexpr uint32_t landing_target_estimator_TIMEOUT_US = 2000000;

	uORB::Publication<landing_target_pose_s> _targetPosePub{ORB_ID(landing_target_pose)};
	landing_target_pose_s _target_pose{};

	uORB::Publication<landing_target_innovations_s> _targetInnovationsPub{ORB_ID(landing_target_innovations)};
	landing_target_innovations_s _target_innovations{};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

private:

	enum class TargetMode {
		Moving = 0,
		Stationary
	};

	/**
	* Handles for parameters
	**/
	struct {
		param_t acc_unc;
		param_t meas_unc;
		param_t pos_unc_init;
		param_t vel_unc_init;
		param_t mode;
		param_t scale_x;
		param_t scale_y;
	} _paramHandle;

	struct {
		float acc_unc;
		float meas_unc;
		float pos_unc_init;
		float vel_unc_init;
		TargetMode mode;
		float scale_x;
		float scale_y;
	} _params;

	uORB::Subscription _vehicleLocalPositionSub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _attitudeSub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};
	uORB::Subscription _irlockReportSub{ORB_ID(irlock_report)};

	vehicle_local_position_s	_vehicleLocalPosition{};
	vehicle_attitude_s		_vehicleAttitude{};
	vehicle_acceleration_s		_vehicle_acceleration{};
	irlock_report_s			_irlockReport{};

	// keep track of which topics we have received
	bool _vehicleLocalPosition_valid{false};
	bool _vehicleAttitude_valid{false};
	bool _vehicle_acceleration_valid{false};
	bool _new_irlockReport{false};
	bool _estimator_initialized{false};
	// keep track of whether last measurement was rejected
	bool _faulty{false};

	matrix::Dcmf _R_att;
	matrix::Vector2f _rel_pos;
	KalmanFilter _kalman_filter_x;
	KalmanFilter _kalman_filter_y;
	hrt_abstime _last_predict{0}; // timestamp of last filter prediction
	hrt_abstime _last_update{0}; // timestamp of last filter update (used to check timeout)

	void _check_params(const bool force);

	void _update_state();
};


} // namespace landing_target_estimator
