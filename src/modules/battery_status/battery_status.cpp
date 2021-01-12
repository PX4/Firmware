/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file sensors.cpp
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Julian Oes <julian@oes.ch>
 * @author Thomas Gubler <thomas@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/log.h>
#include <lib/mathlib/mathlib.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_adc.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <lib/battery/battery.h>
#include <lib/conversion/rotation.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/adc_report.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include "analog_battery.h"

using namespace time_literals;

/**
 * The channel definitions (e.g., ADC_BATTERY_VOLTAGE_CHANNEL, ADC_BATTERY_CURRENT_CHANNEL, and ADC_AIRSPEED_VOLTAGE_CHANNEL) are defined in board_config.h
 */

#ifndef BOARD_NUMBER_BRICKS
#error "battery_status module requires power bricks"
#endif

#if BOARD_NUMBER_BRICKS == 0
#error "battery_status module requires power bricks"
#endif

class BatteryStatus : public ModuleBase<BatteryStatus>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	BatteryStatus();
	~BatteryStatus() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	uORB::Subscription	_actuator_ctrl_0_sub{ORB_ID(actuator_controls_0)};		/**< attitude controls sub */
	uORB::SubscriptionInterval	_parameter_update_sub{ORB_ID(parameter_update), 1_s};				/**< notification of parameter updates */
	uORB::SubscriptionCallbackWorkItem _adc_report_sub{this, ORB_ID(adc_report)};

	static constexpr uint32_t SAMPLE_FREQUENCY_HZ = 100;
	static constexpr uint32_t SAMPLE_INTERVAL_US  = 1_s / SAMPLE_FREQUENCY_HZ;

	AnalogBattery _battery1;

#if BOARD_NUMBER_BRICKS > 1
	AnalogBattery _battery2;
#endif

	AnalogBattery *_analogBatteries[BOARD_NUMBER_BRICKS] {
		&_battery1,
#if BOARD_NUMBER_BRICKS > 1
		&_battery2,
#endif
	}; // End _analogBatteries

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	/**
	 * Check for changes in parameters.
	 */
	void 		parameter_update_poll(bool forced = false);

	/**
	 * Poll the ADC and update readings to suit.
	 *
	 * @param raw			Combined sensor data structure into which
	 *				data should be returned.
	 */
	void		adc_poll();
};

BatteryStatus::BatteryStatus() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_battery1(1, this, SAMPLE_INTERVAL_US),
#if BOARD_NUMBER_BRICKS > 1
	_battery2(2, this, SAMPLE_INTERVAL_US),
#endif
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME))
{
	updateParams();
}

BatteryStatus::~BatteryStatus()
{
	ScheduleClear();
}

void
BatteryStatus::parameter_update_poll(bool forced)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || forced) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}
}

void
BatteryStatus::adc_poll()
{
	/* For legacy support we publish the battery_status for the Battery that is
	* associated with the Brick that is the selected source for VDD_5V_IN
	* Selection is done in HW ala a LTC4417 or similar, or may be hard coded
	* Like in the FMUv4
	*/

	/* Per Brick readings with default unread channels at 0 */
	float bat_current_adc_readings[BOARD_NUMBER_BRICKS] {};
	float bat_voltage_adc_readings[BOARD_NUMBER_BRICKS] {};

	int selected_source = -1;

	adc_report_s adc_report;

	if (_adc_report_sub.update(&adc_report)) {

		/* Read add channels we got */
		for (unsigned i = 0; i < PX4_MAX_ADC_CHANNELS; ++i) {
			for (int b = 0; b < BOARD_NUMBER_BRICKS; b++) {

				/* Once we have subscriptions, Do this once for the lowest (highest priority
				 * supply on power controller) that is valid.
				 */
				if (selected_source < 0 && _analogBatteries[b]->is_valid()) {
					/* Indicate the lowest brick (highest priority supply on power controller)
					 * that is valid as the one that is the selected source for the
					 * VDD_5V_IN
					 */
					selected_source = b;
				}

				/* look for specific channels and process the raw voltage to measurement data */

				if (adc_report.channel_id[i] == _analogBatteries[b]->get_voltage_channel()) {
					/* Voltage in volts */
					bat_voltage_adc_readings[b] = adc_report.raw_data[i] *
								      adc_report.v_ref /
								      adc_report.resolution;

				} else if (adc_report.channel_id[i] == _analogBatteries[b]->get_current_channel()) {
					bat_current_adc_readings[b] = adc_report.raw_data[i] *
								      adc_report.v_ref /
								      adc_report.resolution;
				}

			}
		}

		for (int b = 0; b < BOARD_NUMBER_BRICKS; b++) {

			actuator_controls_s ctrl{};
			_actuator_ctrl_0_sub.copy(&ctrl);

			_analogBatteries[b]->updateBatteryStatusADC(
				hrt_absolute_time(),
				bat_voltage_adc_readings[b],
				bat_current_adc_readings[b],
				battery_status_s::BATTERY_SOURCE_POWER_MODULE,
				b,
				ctrl.control[actuator_controls_s::INDEX_THROTTLE]
			);
		}
	}
}

void
BatteryStatus::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	/* check parameters for updates */
	parameter_update_poll();

	/* check battery voltage */
	adc_poll();

	perf_end(_loop_perf);
}

int
BatteryStatus::task_spawn(int argc, char *argv[])
{
	BatteryStatus *instance = new BatteryStatus();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

bool
BatteryStatus::init()
{
	return _adc_report_sub.registerCallback();
}

int BatteryStatus::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int BatteryStatus::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

The provided functionality includes:
- Read the output from the ADC driver (via ioctl interface) and publish `battery_status`.


### Implementation
It runs in its own thread and polls on the currently selected gyro topic.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("battery_status", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int battery_status_main(int argc, char *argv[])
{
	return BatteryStatus::main(argc, argv);
}
