/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#pragma once

#include <lib/mathlib/math/Limits.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_gps_position.h>

using namespace time_literals;

namespace sensors
{
class VehicleGPSPosition : public ModuleParams, public px4::ScheduledWorkItem
{
public:

	VehicleGPSPosition();
	~VehicleGPSPosition() override;

	bool Start();
	void Stop();

	void PrintStatus();

private:
	void Run() override;

	void ParametersUpdate(bool force = false);
	void Publish(const sensor_gps_s &gps);

	/*
	 * Update the internal state estimate for a blended GPS solution that is a weighted average of the phsyical
	 * receiver solutions. This internal state cannot be used directly by estimators because if physical receivers
	 * have significant position differences, variation in receiver estimated accuracy will cause undesirable
	 * variation in the position solution.
	*/
	bool blend_gps_data();

	/*
	 * Calculate internal states used to blend GPS data from multiple receivers using weightings calculated
	 * by calc_blend_weights()
	 * States are written to _gps_state and _gps_blended_state class variables
	 */
	void update_gps_blend_states();

	/*
	 * The location in _gps_blended_state will move around as the relative accuracy changes.
	 * To mitigate this effect a low-pass filtered offset from each GPS location to the blended location is
	 * calculated.
	*/
	void update_gps_offsets();

	/*
	 * Apply the steady state physical receiver offsets calculated by update_gps_offsets().
	*/
	void apply_gps_offsets();

	/*
	 Calculate GPS output that is a blend of the offset corrected physical receiver data
	*/
	void calc_gps_blend_output();

	// defines used to specify the mask position for use of different accuracy metrics in the GPS blending algorithm
	static constexpr uint8_t BLEND_MASK_USE_SPD_ACC  = 1;
	static constexpr uint8_t BLEND_MASK_USE_HPOS_ACC = 2;
	static constexpr uint8_t BLEND_MASK_USE_VPOS_ACC = 4;

	// define max number of GPS receivers supported and 0 base instance used to access virtual 'blended' GPS solution
	static constexpr int GPS_MAX_RECEIVERS = 2;
	static constexpr int GPS_BLENDED_INSTANCE = GPS_MAX_RECEIVERS;

	// Set the GPS timeout to 2s, after which a receiver will be ignored
	static constexpr hrt_abstime GPS_TIMEOUT_US = 2_s;
	static constexpr float GPS_TIMEOUT_S = (GPS_TIMEOUT_US / 1e6f);

	uORB::Publication<vehicle_gps_position_s> _vehicle_gps_position_pub{ORB_ID(vehicle_gps_position)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionCallbackWorkItem _sensor_gps_sub[GPS_MAX_RECEIVERS] {	/**< sensor data subscription */
		{this, ORB_ID(sensor_gps), 0},
		{this, ORB_ID(sensor_gps), 1},
	};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	// GPS blending and switching
	sensor_gps_s _gps_state[GPS_MAX_RECEIVERS] {}; ///< internal state data for the physical GPS
	sensor_gps_s _gps_blended_state{};		///< internal state data for the blended GPS
	sensor_gps_s _gps_output[GPS_MAX_RECEIVERS + 1] {}; ///< output state data for the physical and blended GPS
	matrix::Vector2f _NE_pos_offset_m[GPS_MAX_RECEIVERS] {}; ///< Filtered North,East position offset from GPS instance to blended solution in _output_state.location (m)
	float _hgt_offset_mm[GPS_MAX_RECEIVERS] {};	///< Filtered height offset from GPS instance relative to blended solution in _output_state.location (mm)
	matrix::Vector3f _blended_antenna_offset{};		///< blended antenna offset
	float _blend_weights[GPS_MAX_RECEIVERS] {};	///< blend weight for each GPS. The blend weights must sum to 1.0 across all instances.
	uint64_t _time_prev_us[GPS_MAX_RECEIVERS] {};	///< the previous value of time_us for that GPS instance - used to detect new data.
	uint8_t _gps_best_index{0};			///< index of the physical receiver with the lowest reported error
	uint8_t _gps_select_index{0};			///< 0 = GPS1, 1 = GPS2, 2 = blended
	uint8_t _gps_time_ref_index{0};			///< index of the receiver that is used as the timing reference for the blending update
	uint8_t _gps_oldest_index{0};			///< index of the physical receiver with the oldest data
	uint8_t _gps_newest_index{0};			///< index of the physical receiver with the newest data
	uint8_t _gps_slowest_index{0};			///< index of the physical receiver with the slowest update rate
	float _gps_dt[GPS_MAX_RECEIVERS] {};		///< average time step in seconds.
	bool  _gps_new_output_data{false};		///< true if there is new output data for the EKF

	DEFINE_PARAMETERS(
		// GPS blending
		(ParamInt<px4::params::SENS_GPS_MASK>)
		_param_sens_gps_mask, ///< mask defining when GPS accuracy metrics are used to calculate the blend ratio
		(ParamFloat<px4::params::SENS_GPS_TAU>)
		_param_sens_gps_tau ///< time constant controlling how rapidly the offset used to bring GPS solutions together is allowed to change (sec)
	)
};
}; // namespace sensors
