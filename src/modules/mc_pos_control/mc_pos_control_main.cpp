/****************************************************************************
 *
 *   Copyright (c) 2013 - 2019 PX4 Development Team. All rights reserved.
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
 * @file mc_pos_control_main.cpp
 * Multicopter position controller.
 */

#include <commander/px4_custom_mode.h>
#include <drivers/drv_hrt.h>
#include <lib/controllib/blocks.hpp>
#include <lib/flight_tasks/FlightTasks.hpp>
#include <lib/hysteresis/hysteresis.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/systemlib/mavlink_log.h>
#include <lib/weather_vane/WeatherVane.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/home_position.h>
#include <uORB/topics/landing_gear.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_trajectory_waypoint.h>
#include <uORB/topics/hover_thrust_estimate.h>

#include "PositionControl/PositionControl.hpp"
#include "Takeoff/Takeoff.hpp"

#include <float.h>

using namespace time_literals;
using namespace matrix;

/**
 * Multicopter position control app start / stop handling function
 */
extern "C" __EXPORT int mc_pos_control_main(int argc, char *argv[]);

class MulticopterPositionControl : public ModuleBase<MulticopterPositionControl>, public control::SuperBlock,
	public ModuleParams, public px4::WorkItem
{
public:
	MulticopterPositionControl(bool vtol = false);
	~MulticopterPositionControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:
	void Run() override;

	Takeoff _takeoff; /**< state machine and ramp to bring the vehicle off the ground without jumps */

	uORB::Publication<vehicle_attitude_setpoint_s>	_vehicle_attitude_setpoint_pub;
	uORB::PublicationQueued<vehicle_command_s> _pub_vehicle_command{ORB_ID(vehicle_command)};	 /**< vehicle command publication */
	orb_advert_t _mavlink_log_pub{nullptr};

	uORB::Publication<landing_gear_s>			_landing_gear_pub{ORB_ID(landing_gear)};
	uORB::Publication<vehicle_local_position_setpoint_s>	_local_pos_sp_pub{ORB_ID(vehicle_local_position_setpoint)};	/**< vehicle local position setpoint publication */
	uORB::Publication<vehicle_local_position_setpoint_s>	_traj_sp_pub{ORB_ID(trajectory_setpoint)};			/**< trajectory setpoints publication */

	uORB::SubscriptionCallbackWorkItem _local_pos_sub{this, ORB_ID(vehicle_local_position)};	/**< vehicle local position */

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};			/**< vehicle status subscription */
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};	/**< vehicle land detected subscription */
	uORB::Subscription _control_mode_sub{ORB_ID(vehicle_control_mode)};		/**< vehicle control mode subscription */
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};		/**< notification of parameter updates */
	uORB::Subscription _home_pos_sub{ORB_ID(home_position)}; 			/**< home position */
	uORB::Subscription _hover_thrust_estimate_sub{ORB_ID(hover_thrust_estimate)};

	hrt_abstime	_time_stamp_last_loop{0};		/**< time stamp of last loop iteration */

	int _task_failure_count{0};         /**< counter for task failures */

	vehicle_status_s 			_vehicle_status{};		/**< vehicle status */
	/**< vehicle-land-detection: initialze to landed */
	vehicle_land_detected_s _vehicle_land_detected = {
		.timestamp = 0,
		.alt_max = -1.0f,
		.freefall = false,
		.ground_contact = true,
		.maybe_landed = true,
		.landed = true,
	};

	vehicle_control_mode_s	_control_mode{};		/**< vehicle control mode */
	vehicle_local_position_s _local_pos{};			/**< vehicle local position */
	home_position_s	_home_pos{};			/**< home position */
	landing_gear_s _landing_gear{};
	int8_t _old_landing_gear_position{landing_gear_s::GEAR_KEEP};

	DEFINE_PARAMETERS(
		// Position Control
		(ParamFloat<px4::params::MPC_XY_P>) _param_mpc_xy_p,
		(ParamFloat<px4::params::MPC_Z_P>) _param_mpc_z_p,
		(ParamFloat<px4::params::MPC_XY_VEL_P_ACC>) _param_mpc_xy_vel_p_acc,
		(ParamFloat<px4::params::MPC_XY_VEL_I_ACC>) _param_mpc_xy_vel_i_acc,
		(ParamFloat<px4::params::MPC_XY_VEL_D_ACC>) _param_mpc_xy_vel_d_acc,
		(ParamFloat<px4::params::MPC_Z_VEL_P_ACC>) _param_mpc_z_vel_p_acc,
		(ParamFloat<px4::params::MPC_Z_VEL_I_ACC>) _param_mpc_z_vel_i_acc,
		(ParamFloat<px4::params::MPC_Z_VEL_D_ACC>) _param_mpc_z_vel_d_acc,
		(ParamFloat<px4::params::MPC_XY_VEL_MAX>) _param_mpc_xy_vel_max,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_UP>) _param_mpc_z_vel_max_up,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_DN>) _param_mpc_z_vel_max_dn,
		(ParamFloat<px4::params::MPC_TILTMAX_AIR>) _param_mpc_tiltmax_air,
		(ParamFloat<px4::params::MPC_THR_HOVER>) _param_mpc_thr_hover,
		(ParamBool<px4::params::MPC_USE_HTE>) _param_mpc_use_hte,

		// Takeoff / Land
		(ParamFloat<px4::params::MPC_SPOOLUP_TIME>) _param_mpc_spoolup_time, /**< time to let motors spool up after arming */
		(ParamFloat<px4::params::MPC_TKO_RAMP_T>) _param_mpc_tko_ramp_t, /**< time constant for smooth takeoff ramp */
		(ParamFloat<px4::params::MPC_TKO_SPEED>) _param_mpc_tko_speed,
		(ParamFloat<px4::params::MPC_LAND_SPEED>) _param_mpc_land_speed,

		(ParamFloat<px4::params::MPC_VEL_MANUAL>) _param_mpc_vel_manual,
		(ParamFloat<px4::params::MPC_XY_CRUISE>) _param_mpc_xy_cruise,
		(ParamFloat<px4::params::MPC_LAND_ALT2>) _param_mpc_land_alt2, /**< downwards speed limited below this altitude */
		(ParamInt<px4::params::MPC_POS_MODE>) _param_mpc_pos_mode,
		(ParamInt<px4::params::MPC_ALT_MODE>) _param_mpc_alt_mode,
		(ParamFloat<px4::params::MPC_TILTMAX_LND>) _param_mpc_tiltmax_lnd, /**< maximum tilt for landing and smooth takeoff */
		(ParamFloat<px4::params::MPC_THR_MIN>) _param_mpc_thr_min,
		(ParamFloat<px4::params::MPC_THR_MAX>) _param_mpc_thr_max
	);

	control::BlockDerivative _vel_x_deriv; /**< velocity derivative in x */
	control::BlockDerivative _vel_y_deriv; /**< velocity derivative in y */
	control::BlockDerivative _vel_z_deriv; /**< velocity derivative in z */

	FlightTasks _flight_tasks; /**< class generating position controller setpoints depending on vehicle task */
	PositionControl _control; /**< class for core PID position control */
	PositionControlStates _states{}; /**< structure containing vehicle state information for position control */

	hrt_abstime _last_warn = 0; /**< timer when the last warn message was sent out */

	bool _in_failsafe = false; /**< true if failsafe was entered within current cycle */

	bool _hover_thrust_initialized{false};

	/** Timeout in us for trajectory data to get considered invalid */
	static constexpr uint64_t TRAJECTORY_STREAM_TIMEOUT_US = 500_ms;
	/** number of tries before switching to a failsafe flight task */
	static constexpr int NUM_FAILURE_TRIES = 10;
	/** If Flighttask fails, keep 0.2 seconds the current setpoint before going into failsafe land */
	static constexpr uint64_t LOITER_TIME_BEFORE_DESCEND = 200_ms;
	/** During smooth-takeoff, below ALTITUDE_THRESHOLD the yaw-control is turned off ant tilt is limited */
	static constexpr float ALTITUDE_THRESHOLD = 0.3f;

	static constexpr float MAX_SAFE_TILT_DEG = 89.f; // Numerical issues above this value due to tanf

	systemlib::Hysteresis _failsafe_land_hysteresis{false}; /**< becomes true if task did not update correctly for LOITER_TIME_BEFORE_DESCEND */

	WeatherVane *_wv_controller{nullptr};
	Vector3f _wv_dcm_z_sp_prev{0, 0, 1};

	perf_counter_t _cycle_perf;

	/**
	 * Update our local parameter cache.
	 * Parameter update can be forced when argument is true.
	 * @param force forces parameter update.
	 */
	int parameters_update(bool force);

	/**
	 * Check for changes in subscribed topics.
	 */
	void poll_subscriptions();

	/**
	 * Check for validity of positon/velocity states.
	 * @param vel_sp_z velocity setpoint in z-direction
	 */
	void set_vehicle_states(const float &vel_sp_z);

	/**
	 * Limit altitude based on land-detector.
	 * @param setpoint needed to detect vehicle intention.
	 */
	void limit_altitude(vehicle_local_position_setpoint_s &setpoint);

	/**
	 * Adjust the setpoint during landing.
	 * Thrust is adjusted to support the land-detector during detection.
	 * @param setpoint gets adjusted based on land-detector state
	 */
	void limit_thrust_during_landing(vehicle_attitude_setpoint_s &setpoint);

	/**
	 * Start flightasks based on navigation state.
	 * This methods activates a task based on the navigation state.
	 */
	void start_flight_task();

	/**
	 * Failsafe.
	 * If flighttask fails for whatever reason, then do failsafe. This could
	 * occur if the commander fails to switch to a mode in case of invalid states or
	 * setpoints. The failsafe will occur after LOITER_TIME_BEFORE_DESCEND. If force is set
	 * to true, the failsafe will be initiated immediately.
	 */
	void failsafe(const hrt_abstime &now, vehicle_local_position_setpoint_s &setpoint, const PositionControlStates &states,
		      const bool force, bool warn);

	/**
	 * Reset setpoints to NAN
	 */
	void reset_setpoint_to_nan(vehicle_local_position_setpoint_s &setpoint);

	/**
	 * check if task should be switched because of failsafe
	 */
	void check_failure(bool task_failure, uint8_t nav_state);

	/**
	 * send vehicle command to inform commander about failsafe
	 */
	void send_vehicle_cmd_do(uint8_t nav_state);
};

MulticopterPositionControl::MulticopterPositionControl(bool vtol) :
	SuperBlock(nullptr, "MPC"),
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_vehicle_attitude_setpoint_pub(vtol ? ORB_ID(mc_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),
	_vel_x_deriv(this, "VELD"),
	_vel_y_deriv(this, "VELD"),
	_vel_z_deriv(this, "VELD"),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time"))
{
	if (vtol) {
		// if vehicle is a VTOL we want to enable weathervane capabilities
		_wv_controller = new WeatherVane();
	}

	// fetch initial parameter values
	parameters_update(true);

	// set failsafe hysteresis
	_failsafe_land_hysteresis.set_hysteresis_time_from(false, LOITER_TIME_BEFORE_DESCEND);
}

MulticopterPositionControl::~MulticopterPositionControl()
{
	delete _wv_controller;

	perf_free(_cycle_perf);
}

bool
MulticopterPositionControl::init()
{
	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("vehicle_local_position callback registration failed!");
		return false;
	}

	// limit to every other vehicle_local_position update (50 Hz)
	_local_pos_sub.set_interval_us(20_ms);

	_time_stamp_last_loop = hrt_absolute_time();

	return true;
}

int
MulticopterPositionControl::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		ModuleParams::updateParams();
		SuperBlock::updateParams();

		if (_param_mpc_tiltmax_air.get() > MAX_SAFE_TILT_DEG) {
			_param_mpc_tiltmax_air.set(MAX_SAFE_TILT_DEG);
			_param_mpc_tiltmax_air.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Tilt constrained to safe value");
		}

		if (_param_mpc_tiltmax_lnd.get() > _param_mpc_tiltmax_air.get()) {
			_param_mpc_tiltmax_lnd.set(_param_mpc_tiltmax_air.get());
			_param_mpc_tiltmax_lnd.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Land tilt has been constrained by max tilt");
		}

		_control.setPositionGains(Vector3f(_param_mpc_xy_p.get(), _param_mpc_xy_p.get(), _param_mpc_z_p.get()));
		_control.setVelocityGains(
			Vector3f(_param_mpc_xy_vel_p_acc.get(), _param_mpc_xy_vel_p_acc.get(), _param_mpc_z_vel_p_acc.get()),
			Vector3f(_param_mpc_xy_vel_i_acc.get(), _param_mpc_xy_vel_i_acc.get(), _param_mpc_z_vel_i_acc.get()),
			Vector3f(_param_mpc_xy_vel_d_acc.get(), _param_mpc_xy_vel_d_acc.get(), _param_mpc_z_vel_d_acc.get()));
		_control.setVelocityLimits(_param_mpc_xy_vel_max.get(), _param_mpc_z_vel_max_up.get(), _param_mpc_z_vel_max_dn.get());
		_control.setThrustLimits(_param_mpc_thr_min.get(), _param_mpc_thr_max.get());
		_control.setTiltLimit(M_DEG_TO_RAD_F * _param_mpc_tiltmax_air.get()); // convert to radians!

		// Check that the design parameters are inside the absolute maximum constraints
		if (_param_mpc_xy_cruise.get() > _param_mpc_xy_vel_max.get()) {
			_param_mpc_xy_cruise.set(_param_mpc_xy_vel_max.get());
			_param_mpc_xy_cruise.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Cruise speed has been constrained by max speed");
		}

		if (_param_mpc_vel_manual.get() > _param_mpc_xy_vel_max.get()) {
			_param_mpc_vel_manual.set(_param_mpc_xy_vel_max.get());
			_param_mpc_vel_manual.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Manual speed has been constrained by max speed");
		}

		if (_param_mpc_thr_hover.get() > _param_mpc_thr_max.get() ||
		    _param_mpc_thr_hover.get() < _param_mpc_thr_min.get()) {
			_param_mpc_thr_hover.set(math::constrain(_param_mpc_thr_hover.get(), _param_mpc_thr_min.get(),
						 _param_mpc_thr_max.get()));
			_param_mpc_thr_hover.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Hover thrust has been constrained by min/max");
		}

		if (!_param_mpc_use_hte.get() || !_hover_thrust_initialized) {
			_control.setHoverThrust(_param_mpc_thr_hover.get());
			_hover_thrust_initialized = true;
		}

		_flight_tasks.handleParameterUpdate();

		// initialize vectors from params and enforce constraints
		_param_mpc_tko_speed.set(math::min(_param_mpc_tko_speed.get(), _param_mpc_z_vel_max_up.get()));
		_param_mpc_land_speed.set(math::min(_param_mpc_land_speed.get(), _param_mpc_z_vel_max_dn.get()));

		// set trigger time for takeoff delay
		_takeoff.setSpoolupTime(_param_mpc_spoolup_time.get());
		_takeoff.setTakeoffRampTime(_param_mpc_tko_ramp_t.get());
		_takeoff.generateInitialRampValue(_param_mpc_z_vel_p_acc.get());

		if (_wv_controller != nullptr) {
			_wv_controller->update_parameters();
		}
	}

	return OK;
}

void
MulticopterPositionControl::poll_subscriptions()
{
	_vehicle_status_sub.update(&_vehicle_status);
	_vehicle_land_detected_sub.update(&_vehicle_land_detected);
	_control_mode_sub.update(&_control_mode);
	_home_pos_sub.update(&_home_pos);

	if (_param_mpc_use_hte.get()) {
		hover_thrust_estimate_s hte;

		if (_hover_thrust_estimate_sub.update(&hte)) {
			if (hte.valid) {
				_control.updateHoverThrust(hte.hover_thrust);
			}
		}
	}
}

void
MulticopterPositionControl::limit_altitude(vehicle_local_position_setpoint_s &setpoint)
{
	if (_vehicle_land_detected.alt_max < 0.0f || !_home_pos.valid_alt || !_local_pos.v_z_valid) {
		// there is no altitude limitation present or the required information not available
		return;
	}

	// maximum altitude == minimal z-value (NED)
	const float min_z = _home_pos.z + (-_vehicle_land_detected.alt_max);

	if (_states.position(2) < min_z) {
		// above maximum altitude, only allow downwards flight == positive vz-setpoints (NED)
		setpoint.z = min_z;
		setpoint.vz = math::max(setpoint.vz, 0.f);
	}
}

void
MulticopterPositionControl::set_vehicle_states(const float &vel_sp_z)
{
	if (_local_pos.timestamp == 0) {
		return;
	}

	// only set position states if valid and finite
	if (PX4_ISFINITE(_local_pos.x) && PX4_ISFINITE(_local_pos.y) && _local_pos.xy_valid) {
		_states.position(0) = _local_pos.x;
		_states.position(1) = _local_pos.y;

	} else {
		_states.position(0) = _states.position(1) = NAN;
	}

	if (PX4_ISFINITE(_local_pos.z) && _local_pos.z_valid) {
		_states.position(2) = _local_pos.z;

	} else {
		_states.position(2) = NAN;
	}

	if (PX4_ISFINITE(_local_pos.vx) && PX4_ISFINITE(_local_pos.vy) && _local_pos.v_xy_valid) {
		_states.velocity(0) = _local_pos.vx;
		_states.velocity(1) = _local_pos.vy;
		_states.acceleration(0) = _vel_x_deriv.update(_states.velocity(0));
		_states.acceleration(1) = _vel_y_deriv.update(_states.velocity(1));

	} else {
		_states.velocity(0) = _states.velocity(1) = NAN;
		_states.acceleration(0) = _states.acceleration(1) = NAN;
		// reset derivatives to prevent acceleration spikes when regaining velocity
		_vel_x_deriv.reset();
		_vel_y_deriv.reset();
	}

	if (PX4_ISFINITE(_local_pos.vz) && _local_pos.v_z_valid) {
		_states.velocity(2) = _local_pos.vz;

		if (PX4_ISFINITE(vel_sp_z) && fabsf(vel_sp_z) > FLT_EPSILON && PX4_ISFINITE(_local_pos.z_deriv)) {
			// A change in velocity is demanded. Set velocity to the derivative of position
			// because it has less bias but blend it in across the landing speed range
			float weighting = fminf(fabsf(vel_sp_z) / _param_mpc_land_speed.get(), 1.0f);
			_states.velocity(2) = _local_pos.z_deriv * weighting + _local_pos.vz * (1.0f - weighting);
		}

		_states.acceleration(2) = _vel_z_deriv.update(-_states.velocity(2));

	} else {
		_states.velocity(2) = _states.acceleration(2) = NAN;
		// reset derivative to prevent acceleration spikes when regaining velocity
		_vel_z_deriv.reset();
	}

	if (PX4_ISFINITE(_local_pos.heading)) {
		_states.yaw = _local_pos.heading;
	}
}

int
MulticopterPositionControl::print_status()
{
	if (_flight_tasks.isAnyTaskActive()) {
		PX4_INFO("Running, active flight task: %i", _flight_tasks.getActiveTask());

	} else {
		PX4_INFO("Running, no flight task active");
	}

	perf_print_counter(_cycle_perf);

	return 0;
}

void
MulticopterPositionControl::Run()
{
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	if (_local_pos_sub.update(&_local_pos)) {

		poll_subscriptions();
		parameters_update(false);

		const hrt_abstime time_stamp_now = _local_pos.timestamp;
		const float dt = math::constrain(((time_stamp_now - _time_stamp_last_loop) * 1e-6f), 0.002f, 0.04f);
		_time_stamp_last_loop = time_stamp_now;

		// set _dt in controllib Block - the time difference since the last loop iteration in seconds
		setDt(dt);

		const bool was_in_failsafe = _in_failsafe;
		_in_failsafe = false;

		// activate the weathervane controller if required. If activated a flighttask can use it to implement a yaw-rate control strategy
		// that turns the nose of the vehicle into the wind
		if (_wv_controller != nullptr) {

			// in manual mode we just want to use weathervane if position is controlled as well
			// in mission, enabling wv is done in flight task
			if (_control_mode.flag_control_manual_enabled) {
				if (_control_mode.flag_control_position_enabled && _wv_controller->weathervane_enabled()) {
					_wv_controller->activate();

				} else {
					_wv_controller->deactivate();
				}
			}

			_wv_controller->update(_wv_dcm_z_sp_prev, _states.yaw);
		}

		// an update is necessary here because otherwise the takeoff state doesn't get skiped with non-altitude-controlled modes
		_takeoff.updateTakeoffState(_control_mode.flag_armed, _vehicle_land_detected.landed, false, 10.f,
					    !_control_mode.flag_control_climb_rate_enabled, time_stamp_now);

		// switch to the required flighttask
		start_flight_task();

		// check if any task is active
		if (_flight_tasks.isAnyTaskActive()) {
			// setpoints and constraints for the position controller from flighttask or failsafe
			vehicle_local_position_setpoint_s setpoint = FlightTask::empty_setpoint;
			vehicle_constraints_s constraints = FlightTask::empty_constraints;

			_flight_tasks.setYawHandler(_wv_controller);

			// update task
			if (!_flight_tasks.update()) {
				// FAILSAFE
				// Task was not able to update correctly. Do Failsafe.
				failsafe(time_stamp_now, setpoint, _states, false, !was_in_failsafe);

			} else {
				setpoint = _flight_tasks.getPositionSetpoint();
				constraints = _flight_tasks.getConstraints();

				_failsafe_land_hysteresis.set_state_and_update(false, time_stamp_now);
			}

			// publish trajectory setpoint
			_traj_sp_pub.publish(setpoint);

			landing_gear_s gear = _flight_tasks.getGear();

			// check if all local states are valid and map accordingly
			set_vehicle_states(setpoint.vz);

			// fix to prevent the takeoff ramp to ramp to a too high value or get stuck because of NAN
			// TODO: this should get obsolete once the takeoff limiting moves into the flight tasks
			if (!PX4_ISFINITE(constraints.speed_up) || (constraints.speed_up > _param_mpc_z_vel_max_up.get())) {
				constraints.speed_up = _param_mpc_z_vel_max_up.get();
			}

			// limit tilt during takeoff ramupup
			if (_takeoff.getTakeoffState() < TakeoffState::flight) {
				constraints.tilt = math::radians(_param_mpc_tiltmax_lnd.get());
				setpoint.acceleration[2] = NAN;
			}

			// limit altitude only if local position is valid
			if (PX4_ISFINITE(_states.position(2))) {
				limit_altitude(setpoint);
			}

			// handle smooth takeoff
			_takeoff.updateTakeoffState(_control_mode.flag_armed, _vehicle_land_detected.landed, constraints.want_takeoff,
						    constraints.speed_up, !_control_mode.flag_control_climb_rate_enabled, time_stamp_now);
			constraints.speed_up = _takeoff.updateRamp(dt, constraints.speed_up);

			const bool not_taken_off = _takeoff.getTakeoffState() < TakeoffState::rampup;
			const bool flying = _takeoff.getTakeoffState() >= TakeoffState::flight;
			const bool flying_but_ground_contact = flying && _vehicle_land_detected.ground_contact;

			if (flying) {
				_control.setThrustLimits(_param_mpc_thr_min.get(), _param_mpc_thr_max.get());

			} else {
				// allow zero thrust when taking off and landing
				_control.setThrustLimits(0.f, _param_mpc_thr_max.get());
			}

			if (not_taken_off || flying_but_ground_contact) {
				// we are not flying yet and need to avoid any corrections
				reset_setpoint_to_nan(setpoint);
				Vector3f(0.f, 0.f, 100.f).copyTo(setpoint.acceleration); // High downwards acceleration to make sure there's no thrust
				// set yaw-sp to current yaw
				// TODO: we need a clean way to disable yaw control
				setpoint.yaw = _states.yaw;
				setpoint.yawspeed = 0.f;
				// prevent any integrator windup
				_control.resetIntegral();
			}

			if (not_taken_off) {
				// reactivate the task which will reset the setpoint to current state
				_flight_tasks.reActivate();
			}

			// Run position control
			_control.setState(_states);
			_control.setConstraints(constraints);
			_control.setInputSetpoint(setpoint);

			if (!_control.update(dt)) {
				if ((time_stamp_now - _last_warn) > 1_s) {
					PX4_WARN("invalid setpoints");
					_last_warn = time_stamp_now;
				}

				failsafe(time_stamp_now, setpoint, _states, true, !was_in_failsafe);
				_control.setInputSetpoint(setpoint);
				constraints = FlightTask::empty_constraints;
				_control.update(dt);
			}

			// Fill local position, velocity and thrust setpoint.
			// This message contains setpoints where each type of setpoint is either the input to the PositionController
			// or was generated by the PositionController and therefore corresponds to the PositioControl internal states (states that were generated by P-PID).
			// Example:
			// If the desired setpoint is position-setpoint, _local_pos_sp will contain
			// position-, velocity- and thrust-setpoint where the velocity- and thrust-setpoint were generated by the PositionControlller.
			// If the desired setpoint has a velocity-setpoint only, then _local_pos_sp will contain valid velocity- and thrust-setpoint, but the position-setpoint
			// will remain NAN. Given that the PositionController cannot generate a position-setpoint, this type of setpoint is always equal to the input to the
			// PositionController.
			vehicle_local_position_setpoint_s local_pos_sp{};
			local_pos_sp.timestamp = time_stamp_now;
			_control.getLocalPositionSetpoint(local_pos_sp);

			// Publish local position setpoint
			// This message will be used by other modules (such as Landdetector) to determine vehicle intention.
			_local_pos_sp_pub.publish(local_pos_sp);

			// Inform FlightTask about the input and output of the velocity controller
			// This is used to properly initialize the velocity setpoint when onpening the position loop (position unlock)
			_flight_tasks.updateVelocityControllerIO(Vector3f(local_pos_sp.vx, local_pos_sp.vy, local_pos_sp.vz),
					Vector3f(local_pos_sp.acceleration));

			vehicle_attitude_setpoint_s attitude_setpoint{};
			attitude_setpoint.timestamp = time_stamp_now;
			_control.getAttitudeSetpoint(attitude_setpoint);

			// publish attitude setpoint
			// It's important to publish also when disarmed otheriwse the attitude setpoint stays uninitialized.
			// Not publishing when not running a flight task
			// in stabilized mode attitude setpoints get ignored
			// in offboard with attitude setpoints they come from MAVLink directly
			_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);

			_wv_dcm_z_sp_prev = Quatf(attitude_setpoint.q_d).dcm_z();

			// if there's any change in landing gear setpoint publish it
			if (gear.landing_gear != _old_landing_gear_position
			    && gear.landing_gear != landing_gear_s::GEAR_KEEP) {
				_landing_gear.timestamp = time_stamp_now;
				_landing_gear.landing_gear = gear.landing_gear;
				_landing_gear_pub.publish(_landing_gear);
			}

			_old_landing_gear_position = gear.landing_gear;

		} else {
			// reset the numerical derivatives to not generate d term spikes when coming from non-position controlled operation
			_vel_x_deriv.reset();
			_vel_y_deriv.reset();
			_vel_z_deriv.reset();
		}
	}

	perf_end(_cycle_perf);
}

void
MulticopterPositionControl::start_flight_task()
{
	bool task_failure = false;
	bool should_disable_task = true;
	int prev_failure_count = _task_failure_count;

	// Do not run any flight task for VTOLs in fixed-wing mode
	if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
		_flight_tasks.switchTask(FlightTaskIndex::None);
		return;
	}

	if (_vehicle_status.in_transition_mode) {
		should_disable_task = false;
		FlightTaskError error = _flight_tasks.switchTask(FlightTaskIndex::Transition);

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				PX4_WARN("Transition activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			// we want to be in this mode, reset the failure count
			_task_failure_count = 0;
		}

		return;
	}

	// offboard
	if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD
	    && (_control_mode.flag_control_altitude_enabled ||
		_control_mode.flag_control_position_enabled ||
		_control_mode.flag_control_climb_rate_enabled ||
		_control_mode.flag_control_velocity_enabled ||
		_control_mode.flag_control_acceleration_enabled)) {

		should_disable_task = false;
		FlightTaskError error = _flight_tasks.switchTask(FlightTaskIndex::Offboard);

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				PX4_WARN("Offboard activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			// we want to be in this mode, reset the failure count
			_task_failure_count = 0;
		}
	}

	// Auto-follow me
	if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET) {
		should_disable_task = false;
		FlightTaskError error = _flight_tasks.switchTask(FlightTaskIndex::AutoFollowMe);

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				PX4_WARN("Follow-Me activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			// we want to be in this mode, reset the failure count
			_task_failure_count = 0;
		}

	} else if (_control_mode.flag_control_auto_enabled) {
		// Auto related maneuvers
		should_disable_task = false;
		FlightTaskError error = FlightTaskError::NoError;

		error =  _flight_tasks.switchTask(FlightTaskIndex::AutoLineSmoothVel);

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				PX4_WARN("Auto activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			// we want to be in this mode, reset the failure count
			_task_failure_count = 0;
		}

	} else if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_DESCEND) {

		// Emergency descend
		should_disable_task = false;
		FlightTaskError error = FlightTaskError::NoError;

		error =  _flight_tasks.switchTask(FlightTaskIndex::Descend);

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				PX4_WARN("Descend activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			// we want to be in this mode, reset the failure count
			_task_failure_count = 0;
		}

	}

	// manual position control
	if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL || task_failure) {
		should_disable_task = false;
		FlightTaskError error = FlightTaskError::NoError;

		switch (_param_mpc_pos_mode.get()) {
		case 1:
			error =  _flight_tasks.switchTask(FlightTaskIndex::ManualPositionSmooth);
			break;

		case 3:
			error =  _flight_tasks.switchTask(FlightTaskIndex::ManualPositionSmoothVel);
			break;

		default:
			error =  _flight_tasks.switchTask(FlightTaskIndex::ManualPosition);
			break;
		}

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				PX4_WARN("Position-Ctrl activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			check_failure(task_failure, vehicle_status_s::NAVIGATION_STATE_POSCTL);
			task_failure = false;
		}
	}

	// manual altitude control
	if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ALTCTL || task_failure) {
		should_disable_task = false;
		FlightTaskError error = FlightTaskError::NoError;

		switch (_param_mpc_pos_mode.get()) {
		case 1:
			error =  _flight_tasks.switchTask(FlightTaskIndex::ManualAltitudeSmooth);
			break;

		case 3:
			error =  _flight_tasks.switchTask(FlightTaskIndex::ManualAltitudeSmoothVel);
			break;

		default:
			error =  _flight_tasks.switchTask(FlightTaskIndex::ManualAltitude);
			break;
		}

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				PX4_WARN("Altitude-Ctrl activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			check_failure(task_failure, vehicle_status_s::NAVIGATION_STATE_ALTCTL);
			task_failure = false;
		}
	}

	if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ORBIT) {
		should_disable_task = false;
	}

	// check task failure
	if (task_failure) {

		// for some reason no flighttask was able to start.
		// go into failsafe flighttask
		FlightTaskError error = _flight_tasks.switchTask(FlightTaskIndex::Failsafe);

		if (error != FlightTaskError::NoError) {
			// No task was activated.
			_flight_tasks.switchTask(FlightTaskIndex::None);
		}

	} else if (should_disable_task) {
		_flight_tasks.switchTask(FlightTaskIndex::None);
	}
}

void
MulticopterPositionControl::failsafe(const hrt_abstime &now, vehicle_local_position_setpoint_s &setpoint,
				     const PositionControlStates &states, const bool force, bool warn)
{
	// do not warn while we are disarmed, as we might not have valid setpoints yet
	if (!_control_mode.flag_armed) {
		warn = false;
	}

	_failsafe_land_hysteresis.set_state_and_update(true, now);

	if (!_failsafe_land_hysteresis.get_state() && !force) {
		// just keep current setpoint and don't do anything.

	} else {
		reset_setpoint_to_nan(setpoint);

		if (PX4_ISFINITE(_states.velocity(0)) && PX4_ISFINITE(_states.velocity(1))) {
			// don't move along xy
			setpoint.vx = setpoint.vy = 0.f;

			if (warn) {
				PX4_WARN("Failsafe: stop and wait");
			}

		} else {
			// descend with land speed since we can't stop
			setpoint.acceleration[0] = setpoint.acceleration[1] = 0.f;
			setpoint.vz = _param_mpc_land_speed.get();

			if (warn) {
				PX4_WARN("Failsafe: blind land");
			}
		}

		if (PX4_ISFINITE(_states.velocity(2))) {
			// don't move along z if we can stop in all dimensions
			if (!PX4_ISFINITE(setpoint.vz)) {
				setpoint.vz = 0.f;
			}

		} else {
			// emergency descend with a bit below hover thrust
			setpoint.vz = NAN;
			setpoint.acceleration[2] = .3f;

			if (warn) {
				PX4_WARN("Failsafe: blind descend");
			}
		}

		_in_failsafe = true;
	}
}

void
MulticopterPositionControl::reset_setpoint_to_nan(vehicle_local_position_setpoint_s &setpoint)
{
	setpoint.x = setpoint.y = setpoint.z = NAN;
	setpoint.vx = setpoint.vy = setpoint.vz = NAN;
	setpoint.yaw = setpoint.yawspeed = NAN;
	setpoint.acceleration[0] = setpoint.acceleration[1] = setpoint.acceleration[2] = NAN;
	setpoint.thrust[0] = setpoint.thrust[1] = setpoint.thrust[2] = NAN;
}

void MulticopterPositionControl::check_failure(bool task_failure, uint8_t nav_state)
{
	if (!task_failure) {
		// we want to be in this mode, reset the failure count
		_task_failure_count = 0;

	} else if (_task_failure_count > NUM_FAILURE_TRIES) {
		// tell commander to switch mode
		PX4_WARN("Previous flight task failed, switching to mode %d", nav_state);
		send_vehicle_cmd_do(nav_state);
		_task_failure_count = 0; // avoid immediate resending of a vehicle command in the next iteration
	}
}

void MulticopterPositionControl::send_vehicle_cmd_do(uint8_t nav_state)
{
	vehicle_command_s command{};
	command.timestamp = hrt_absolute_time();
	command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	command.param1 = (float)1; // base mode
	command.param3 = (float)0; // sub mode
	command.target_system = 1;
	command.target_component = 1;
	command.source_system = 1;
	command.source_component = 1;
	command.confirmation = false;
	command.from_external = false;

	// set the main mode
	switch (nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_STAB:
		command.param2 = (float)PX4_CUSTOM_MAIN_MODE_STABILIZED;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
		command.param2 = (float)PX4_CUSTOM_MAIN_MODE_ALTCTL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
		command.param2 = (float)PX4_CUSTOM_MAIN_MODE_AUTO;
		command.param3 = (float)PX4_CUSTOM_SUB_MODE_AUTO_LOITER;
		break;

	default: //vehicle_status_s::NAVIGATION_STATE_POSCTL
		command.param2 = (float)PX4_CUSTOM_MAIN_MODE_POSCTL;
		break;
	}

	// publish the vehicle command
	_pub_vehicle_command.publish(command);
}

int MulticopterPositionControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterPositionControl *instance = new MulticopterPositionControl(vtol);

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

int MulticopterPositionControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterPositionControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
The controller has two loops: a P loop for position error and a PID loop for velocity error.
Output of the velocity controller is thrust vector that is split to thrust direction
(i.e. rotation matrix for multicopter orientation) and thrust scalar (i.e. multicopter thrust itself).

The controller doesn't use Euler angles for its work, they are generated only for more human-friendly control and
logging.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int mc_pos_control_main(int argc, char *argv[])
{
	return MulticopterPositionControl::main(argc, argv);
}
