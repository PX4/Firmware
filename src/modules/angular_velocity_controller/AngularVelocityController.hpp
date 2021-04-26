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

#pragma once

#include <AngularVelocityControl.hpp>

#include <lib/mathlib/math/filter/LowPassFilter2pVector3f.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_angular_acceleration.h>
#include <uORB/topics/vehicle_angular_acceleration_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

using namespace time_literals;

class AngularVelocityController : public ModuleBase<AngularVelocityController>, public ModuleParams,
	public px4::WorkItem
{
public:
	AngularVelocityController();

	virtual ~AngularVelocityController();

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

private:

	/**
	 * initialize some vectors/matrices from parameters
	 */
	void		parameters_updated();

	void		vehicle_status_poll();

	void		publish_angular_acceleration_setpoint();
	void		publish_torque_setpoint();
	void		publish_thrust_setpoint();
	void		publish_actuator_controls();

	AngularVelocityControl _control; 	///< class for control calculations

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _control_allocator_status_sub{ORB_ID(control_allocator_status)};				/**< motor limits subscription */
	uORB::Subscription _vehicle_angular_acceleration_sub{ORB_ID(vehicle_angular_acceleration)}; 	/**< vehicle angular acceleration subscription */
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};			/**< vehicle control mode subscription */
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};			/**< vehicle land detected subscription */
	uORB::Subscription _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)};			/**< vehicle rates setpoint subscription */
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};					/**< vehicle status subscription */
	uORB::Subscription _hover_thrust_estimate_sub{ORB_ID(hover_thrust_estimate)};

	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

	uORB::Publication<rate_ctrl_status_s>			   _rate_ctrl_status_pub{ORB_ID(rate_ctrl_status)};						/**< controller status publication */
	uORB::Publication<vehicle_angular_acceleration_setpoint_s> _vehicle_angular_acceleration_setpoint_pub{ORB_ID(vehicle_angular_acceleration_setpoint)};	/**< angular acceleration setpoint publication */
	uORB::Publication<vehicle_thrust_setpoint_s>		   _vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};				/**< thrust setpoint publication */
	uORB::Publication<vehicle_torque_setpoint_s>		   _vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};				/**< torque setpoint publication */

	vehicle_control_mode_s		_vehicle_control_mode{};
	vehicle_status_s		_vehicle_status{};

	bool _landed{true};
	bool _maybe_landed{true};

	float _battery_status_scale{0.0f};

	perf_counter_t	_loop_perf;			/**< loop duration performance counter */

	matrix::Vector3f _angular_velocity_sp;		/**< angular velocity setpoint */
	matrix::Vector3f _angular_acceleration;		/**< angular acceleration (estimated) */
	matrix::Vector3f _thrust_sp;			/**< thrust setpoint */

	float _hover_thrust{0.5f};			/**< Normalized hover thrust **/

	bool _gear_state_initialized{false};		/**< true if the gear state has been initialized */

	hrt_abstime _task_start{hrt_absolute_time()};
	hrt_abstime _last_run{0};
	hrt_abstime _timestamp_sample{0};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::AVC_X_P>) _param_avc_x_p,
		(ParamFloat<px4::params::AVC_X_I>) _param_avc_x_i,
		(ParamFloat<px4::params::AVC_X_I_LIM>) _param_avc_x_i_lim,
		(ParamFloat<px4::params::AVC_X_D>) _param_avc_x_d,
		(ParamFloat<px4::params::AVC_X_FF>) _param_avc_x_ff,
		(ParamFloat<px4::params::AVC_X_K>) _param_avc_x_k,

		(ParamFloat<px4::params::AVC_Y_P>) _param_avc_y_p,
		(ParamFloat<px4::params::AVC_Y_I>) _param_avc_y_i,
		(ParamFloat<px4::params::AVC_Y_I_LIM>) _param_avc_y_i_lim,
		(ParamFloat<px4::params::AVC_Y_D>) _param_avc_y_d,
		(ParamFloat<px4::params::AVC_Y_FF>) _param_avc_y_ff,
		(ParamFloat<px4::params::AVC_Y_K>) _param_avc_y_k,

		(ParamFloat<px4::params::AVC_Z_P>) _param_avc_z_p,
		(ParamFloat<px4::params::AVC_Z_I>) _param_avc_z_i,
		(ParamFloat<px4::params::AVC_Z_I_LIM>) _param_avc_z_i_lim,
		(ParamFloat<px4::params::AVC_Z_D>) _param_avc_z_d,
		(ParamFloat<px4::params::AVC_Z_FF>) _param_avc_z_ff,
		(ParamFloat<px4::params::AVC_Z_K>) _param_avc_z_k,

		(ParamFloat<px4::params::VM_MASS>) _param_vm_mass,
		(ParamFloat<px4::params::VM_INERTIA_XX>) _param_vm_inertia_xx,
		(ParamFloat<px4::params::VM_INERTIA_YY>) _param_vm_inertia_yy,
		(ParamFloat<px4::params::VM_INERTIA_ZZ>) _param_vm_inertia_zz,
		(ParamFloat<px4::params::VM_INERTIA_XY>) _param_vm_inertia_xy,
		(ParamFloat<px4::params::VM_INERTIA_XZ>) _param_vm_inertia_xz,
		(ParamFloat<px4::params::VM_INERTIA_YZ>) _param_vm_inertia_yz,

		(ParamFloat<px4::params::MPC_THR_HOVER>) _param_mpc_thr_hover,
		(ParamBool<px4::params::MPC_USE_HTE>) _param_mpc_use_hte
	)

};
