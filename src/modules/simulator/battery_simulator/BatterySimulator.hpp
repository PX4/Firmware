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

#include <lib/battery/battery.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class BatterySimulator : public ModuleBase<BatterySimulator>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	BatterySimulator();
	~BatterySimulator() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	uORB::Publication<battery_status_s> _battery_pub{ORB_ID(battery_status)};

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	class SimulatorBattery : public Battery
	{
	public:
		static constexpr uint32_t SIMLATOR_BATTERY_SAMPLE_FREQUENCY_HZ = 100; // Hz
		static constexpr uint32_t SIMLATOR_BATTERY_SAMPLE_INTERVAL_US = 1_s / SIMLATOR_BATTERY_SAMPLE_FREQUENCY_HZ;

		SimulatorBattery() : Battery(1, nullptr, SIMLATOR_BATTERY_SAMPLE_INTERVAL_US) {}

		virtual void updateParams() override
		{
			Battery::updateParams();
			_params.v_empty = 3.5f;
			_params.v_charged = 4.05f;
			_params.n_cells = 4;
			_params.capacity = 10.0f;
			_params.v_load_drop = 0.0f;
			_params.r_internal = 0.0f;
			_params.low_thr = 0.15f;
			_params.crit_thr = 0.07f;
			_params.emergen_thr = 0.05f;
			_params.source = 0;
		}
	} _battery;

	uint64_t _last_integration_us{0};
	float _battery_percentage{1.f};
	bool _armed{false};

	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::SIM_BAT_DRAIN>) _param_sim_bat_drain, ///< battery drain interval
		(ParamFloat<px4::params::SIM_BAT_MIN_PCT>) _param_bat_min_pct //< minimum battery percentage
	)
};
