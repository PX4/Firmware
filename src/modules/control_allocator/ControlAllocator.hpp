/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file ControlAllocator.hpp
 *
 * Control allocator.
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#pragma once

#include <ActuatorEffectiveness.hpp>
#include <ActuatorEffectivenessMultirotor.hpp>
#include <ActuatorEffectivenessStandardVTOL.hpp>
#include <ActuatorEffectivenessTiltrotorVTOL.hpp>

#include <ControlAllocation.hpp>
#include <ControlAllocationPseudoInverse.hpp>
#include <ControlAllocationSequentialDesaturation.hpp>

#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/output_control.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_actuator_setpoint.h>
#include <uORB/topics/vehicle_status.h>

class ControlAllocator : public ModuleBase<ControlAllocator>, public ModuleParams, public px4::WorkItem
{
public:
	ControlAllocator();

	virtual ~ControlAllocator();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	void Run() override;

	bool init();

	static constexpr uint8_t NUM_ACTUATORS = ControlAllocation::NUM_ACTUATORS;
	static constexpr uint8_t NUM_AXES = ControlAllocation::NUM_AXES;

private:

	/**
	 * initialize some vectors/matrices from parameters
	 */
	void parameters_updated();

	void update_allocation_method();
	void update_effectiveness_source();

	void update_effectiveness_matrix_if_needed();

	void publish_actuator_setpoint();
	void publish_control_allocator_status();

	void publish_legacy_actuator_controls();
	void publish_legacy_multirotor_motor_limits();

	enum class AllocationMethod {
		NONE = -1,
		PSEUDO_INVERSE = 0,
		SEQUENTIAL_DESATURATION = 1,
	};

	AllocationMethod _allocation_method_id{AllocationMethod::NONE};
	ControlAllocation *_control_allocation{nullptr}; 	///< class for control allocation calculations

	enum class EffectivenessSource {
		NONE = -1,
		MULTIROTOR = 0,
		STANDARD_VTOL = 1,
		TILTROTOR_VTOL = 2,
	};

	EffectivenessSource _effectiveness_source_id{EffectivenessSource::NONE};
	ActuatorEffectiveness *_actuator_effectiveness{nullptr}; 	///< class providing actuator effectiveness

	// Inputs
	uORB::SubscriptionCallbackWorkItem _vehicle_torque_setpoint_sub{this, ORB_ID(vehicle_torque_setpoint)};  /**< vehicle torque setpoint subscription */
	uORB::SubscriptionCallbackWorkItem _vehicle_thrust_setpoint_sub{this, ORB_ID(vehicle_thrust_setpoint)};	 /**< vehicle thrust setpoint subscription */

	// Outputs
	uORB::Publication<vehicle_actuator_setpoint_s>	_vehicle_actuator_setpoint_pub{ORB_ID(vehicle_actuator_setpoint)};	/**< actuator setpoint publication */
	uORB::Publication<control_allocator_status_s>	_control_allocator_status_pub{ORB_ID(control_allocator_status)};	/**< actuator setpoint publication */
	uORB::Publication<output_control_s>             _output_control_pub{ORB_ID(output_control_ca)}; /// TODO: Merge with vehicle_actuator_setpoint

	// actuator_controls publication handles (temporary hack to plug actuator_setpoint into the mixer system)
	uORB::Publication<actuator_controls_s>	_actuator_controls_4_pub{ORB_ID(actuator_controls_4)};	/**< actuator controls 4 publication */
	uORB::Publication<actuator_controls_s>	_actuator_controls_5_pub{ORB_ID(actuator_controls_5)};	/**< actuator controls 5 publication */

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _airspeed_sub{ORB_ID(airspeed)};				/**< airspeed subscription */
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	matrix::Vector3f _torque_sp;
	matrix::Vector3f _thrust_sp;

	// float _battery_scale_factor{1.0f};
	// float _airspeed_scale_factor{1.0f};

	perf_counter_t	_loop_perf;			/**< loop duration performance counter */

	hrt_abstime _last_run{0};
	hrt_abstime _timestamp_sample{0};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::CA_AIRFRAME>) _param_ca_airframe,
		(ParamInt<px4::params::CA_METHOD>) _param_ca_method,
		(ParamBool<px4::params::CA_BAT_SCALE_EN>) _param_ca_bat_scale_en,
		(ParamBool<px4::params::CA_AIR_SCALE_EN>) _param_ca_air_scale_en,
		(ParamFloat<px4::params::CA_ACT0_MIN>) _param_ca_act0_min,
		(ParamFloat<px4::params::CA_ACT1_MIN>) _param_ca_act1_min,
		(ParamFloat<px4::params::CA_ACT2_MIN>) _param_ca_act2_min,
		(ParamFloat<px4::params::CA_ACT3_MIN>) _param_ca_act3_min,
		(ParamFloat<px4::params::CA_ACT4_MIN>) _param_ca_act4_min,
		(ParamFloat<px4::params::CA_ACT5_MIN>) _param_ca_act5_min,
		(ParamFloat<px4::params::CA_ACT6_MIN>) _param_ca_act6_min,
		(ParamFloat<px4::params::CA_ACT7_MIN>) _param_ca_act7_min,
		(ParamFloat<px4::params::CA_ACT8_MIN>) _param_ca_act8_min,
		(ParamFloat<px4::params::CA_ACT9_MIN>) _param_ca_act9_min,
		(ParamFloat<px4::params::CA_ACT10_MIN>) _param_ca_act10_min,
		(ParamFloat<px4::params::CA_ACT11_MIN>) _param_ca_act11_min,
		(ParamFloat<px4::params::CA_ACT12_MIN>) _param_ca_act12_min,
		(ParamFloat<px4::params::CA_ACT13_MIN>) _param_ca_act13_min,
		(ParamFloat<px4::params::CA_ACT14_MIN>) _param_ca_act14_min,
		(ParamFloat<px4::params::CA_ACT15_MIN>) _param_ca_act15_min,
		(ParamFloat<px4::params::CA_ACT0_MAX>) _param_ca_act0_max,
		(ParamFloat<px4::params::CA_ACT1_MAX>) _param_ca_act1_max,
		(ParamFloat<px4::params::CA_ACT2_MAX>) _param_ca_act2_max,
		(ParamFloat<px4::params::CA_ACT3_MAX>) _param_ca_act3_max,
		(ParamFloat<px4::params::CA_ACT4_MAX>) _param_ca_act4_max,
		(ParamFloat<px4::params::CA_ACT5_MAX>) _param_ca_act5_max,
		(ParamFloat<px4::params::CA_ACT6_MAX>) _param_ca_act6_max,
		(ParamFloat<px4::params::CA_ACT7_MAX>) _param_ca_act7_max,
		(ParamFloat<px4::params::CA_ACT8_MAX>) _param_ca_act8_max,
		(ParamFloat<px4::params::CA_ACT9_MAX>) _param_ca_act9_max,
		(ParamFloat<px4::params::CA_ACT10_MAX>) _param_ca_act10_max,
		(ParamFloat<px4::params::CA_ACT11_MAX>) _param_ca_act11_max,
		(ParamFloat<px4::params::CA_ACT12_MAX>) _param_ca_act12_max,
		(ParamFloat<px4::params::CA_ACT13_MAX>) _param_ca_act13_max,
		(ParamFloat<px4::params::CA_ACT14_MAX>) _param_ca_act14_max,
		(ParamFloat<px4::params::CA_ACT15_MAX>) _param_ca_act15_max
	)

};
