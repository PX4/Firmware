/****************************************************************************
 *
 *   Copyright (c) 2018-20 PX4 Development Team. All rights reserved.
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
 * @file heater.h
 *
 * @author Mark Sauder <mcsauder@gmail.com>
 * @author Alex Klimaj <alexklimaj@gmail.com>
 * @author Jake Dahl <dahl.jakejacob@gmail.com>
 * @author Jacob Crabill <jacob@flyvoly.com>
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/heater_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>

#include <mathlib/mathlib.h>

using namespace time_literals;

#define CONTROLLER_PERIOD_DEFAULT 100000

class Heater : public ModuleBase<Heater>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	Heater();

	virtual ~Heater();

	/**
	 * @see ModuleBase::custom_command().
	 * @brief main Main entry point to the module that should be
	 *        called directly from the module's main method.
	 * @param argc The input argument count.
	 * @param argv Pointer to the input argument array.
	 * @return Returns 0 iff successful, -1 otherwise.
	 */
	static int custom_command(int argc, char *argv[]);

	/**
	 * @see ModuleBase::print_usage().
	 * @brief Prints the module usage to the nuttshell console.
	 * @param reason The requested reason for printing to console.
	 */
	static int print_usage(const char *reason = nullptr);

	/**
	 * @see ModuleBase::task_spawn().
	 * @brief Initializes the class in the same context as the work queue
	 *        and starts the background listener.
	 * @param argv Pointer to the input argument array.
	 * @return Returns 0 iff successful, -1 otherwise.
	 */
	static int task_spawn(int argc, char *argv[]);

	/**
	 * @brief Initiates the heater driver work queue, starts a new background task,
	 *        and fails if it is already running.
	 * @return Returns 1 iff start was successful.
	 */
	int start();

private:

	/** @brief Called once to initialize uORB topics. */
	bool initialize_topics();

	/** @brief Calculates the heater element on/off time and schedules the next cycle. */
	void Run() override;

	/**
	 * @brief Updates and checks for updated uORB parameters.
	 * @param force Boolean to determine if an update check should be forced.
	 */
	void update_params(const bool force = false);

	/** Enables / configures the heater (either by GPIO or PX4IO). */
	void heater_initialize();

	/** Disnables the heater (either by GPIO or PX4IO). */
	void heater_disable();

	/** Turns the heater on (either by GPIO or PX4IO). */
	void heater_on();

	/** Turns the heater off (either by GPIO or PX4IO). */
	void heater_off();

	/** Work queue struct for the scheduler. */
	static struct work_s _work;

	/** File descriptor for PX4IO for heater ioctl's */
#if defined(HEATER_PX4IO)
	int _io_fd_ {-1};
#endif

	bool _heater_on = false;

	int _controller_period_usec = CONTROLLER_PERIOD_DEFAULT;
	int _controller_time_on_usec = 0;

	float _integrator_value   = 0.0f;
	float _proportional_value = 0.0f;

	uORB::Publication<heater_status_s> _heater_status_pub{ORB_ID(heater_status)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _sensor_accel_sub{ORB_ID(sensor_accel)};

	uint32_t _sensor_device_id{0};

	float _temperature_last{NAN};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::SENS_IMU_TEMP_FF>) _param_sens_imu_temp_ff,
		(ParamFloat<px4::params::SENS_IMU_TEMP_I>)  _param_sens_imu_temp_i,
		(ParamFloat<px4::params::SENS_IMU_TEMP_P>)  _param_sens_imu_temp_p,
		(ParamInt<px4::params::SENS_TEMP_ID>)       _param_sens_temp_id,
		(ParamFloat<px4::params::SENS_IMU_TEMP>)    _param_sens_imu_temp
	)
};
