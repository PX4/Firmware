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

#include "FlightTask.hpp"
#include "FlightTasks_generated.hpp"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/landing_gear.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/takeoff_status.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>

#include <new>

enum class FlightTaskError : int {
	NoError = 0,
	InvalidTask = -1,
	ActivationFailed = -2
};

class FlightModeManager : public ModuleBase<FlightModeManager>, public ModuleParams, public px4::WorkItem
{
public:
	FlightModeManager();
	~FlightModeManager() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	bool init();

private:
	void Run() override;
	void updateParams() override;
	void start_flight_task();
	void check_failure(bool task_failure, uint8_t nav_state);
	void send_vehicle_cmd_do(uint8_t nav_state);
	void handleCommand();
	void generateTrajectorySetpoint(const float dt, const vehicle_local_position_s &vehicle_local_position);
	void limitAltitude(vehicle_local_position_setpoint_s &setpoint, const vehicle_local_position_s &vehicle_local_position);

	/**
	 * Switch to a specific task (for normal usage)
	 * @param task index to switch to
	 * @return 0 on success, <0 on error
	 */
	FlightTaskError switchTask(FlightTaskIndex new_task_index);
	FlightTaskError switchTask(int new_task_index);

	/**
	 * Call this method to get the description of a task error.
	 */
	const char *errorToString(const FlightTaskError error);

	/**
	 * Check if any task is active
	 * @return true if a task is active, false if not
	 */
	bool isAnyTaskActive() const { return _current_task.task; }

	// generated
	int _initTask(FlightTaskIndex task_index);
	FlightTaskIndex switchVehicleCommand(const int command);

	static constexpr int NUM_FAILURE_TRIES = 10; ///< number of tries before switching to a failsafe flight task

	/**
	 * Union with all existing tasks: we use it to make sure that only the memory of the largest existing
	 * task is needed, and to avoid using dynamic memory allocations.
	 */
	TaskUnion _task_union; /**< storage for the currently active task */

	struct flight_task_t {
		FlightTask *task{nullptr};
		FlightTaskIndex index{FlightTaskIndex::None};
	} _current_task{};

	WeatherVane *_wv_controller{nullptr};
	int8_t _old_landing_gear_position{landing_gear_s::GEAR_KEEP};
	uint8_t _takeoff_state{takeoff_status_s::TAKEOFF_STATE_UNINITIALIZED};
	int _task_failure_count{0};
	uint8_t _last_vehicle_nav_state{0};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")}; ///< loop duration performance counter
	hrt_abstime _time_stamp_last_loop{0}; ///< time stamp of last loop iteration

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _takeoff_status_sub{ORB_ID(takeoff_status)};
	uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription _vehicle_local_position_setpoint_sub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::SubscriptionData<home_position_s> _home_position_sub{ORB_ID(home_position)};
	uORB::SubscriptionData<vehicle_control_mode_s> _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::SubscriptionData<vehicle_land_detected_s> _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::SubscriptionCallbackWorkItem _vehicle_local_position_sub{this, ORB_ID(vehicle_local_position)};

	uORB::SubscriptionData<vehicle_status_s> _vehicle_status_sub{ORB_ID(vehicle_status)};

	uORB::Publication<landing_gear_s> _landing_gear_pub{ORB_ID(landing_gear)};
	uORB::Publication<vehicle_local_position_setpoint_s> _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};
	uORB::Publication<vehicle_command_ack_s> _vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};
	uORB::Publication<vehicle_constraints_s> _vehicle_constraints_pub{ORB_ID(vehicle_constraints)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MPC_POS_MODE>) _param_mpc_pos_mode
	);
};
