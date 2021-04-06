/****************************************************************************
 *
 *   Copyright (c) 2017, 2021 PX4 Development Team. All rights reserved.
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
 *
 * This module is a modification of the fixed wing module and it is designed for ground rovers.
 * It has been developed starting from the fw module, simplified and improved with dedicated items.
 *
 * All the acknowledgments and credits for the fw wing app are reported in those files.
 *
 * @author Marco Zorzi <mzorzi@student.ethz.ch>
 */


#include "RoverPositionControl.hpp"
#include <lib/ecl/geo/geo.h>

#define ACTUATOR_PUBLISH_PERIOD_MS 4

using namespace matrix;

/**
 * L1 control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int rover_pos_control_main(int argc, char *argv[]);

RoverPositionControl::RoverPositionControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED,  MODULE_NAME": cycle")) // TODO : do we even need these perf counters
{
}

RoverPositionControl::~RoverPositionControl()
{
	perf_free(_loop_perf);
}

bool
RoverPositionControl::init()
{
	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("vehicle attitude callback registration failed!");
		return false;
	}

	return true;
}

void RoverPositionControl::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();

		_gnd_control.set_l1_damping(_param_l1_damping.get());
		_gnd_control.set_l1_period(_param_l1_period.get());
		_gnd_control.set_l1_roll_limit(math::radians(0.0f));

		pid_init(&_speed_ctrl, PID_MODE_DERIVATIV_CALC, 0.01f);
		pid_set_parameters(&_speed_ctrl,
				   _param_speed_p.get(),
				   _param_speed_i.get(),
				   _param_speed_d.get(),
				   _param_speed_imax.get(),
				   _param_gndspeed_max.get());
	}
}

void
RoverPositionControl::vehicle_control_mode_poll()
{
	if (_control_mode_sub.updated()) {
		_control_mode_sub.copy(&_control_mode);
	}
}

void
RoverPositionControl::manual_control_setpoint_poll()
{
	if (_control_mode.flag_control_manual_enabled) {
		if (_manual_control_setpoint_sub.copy(&_manual_control_setpoint)) {
			float dt = math::constrain(hrt_elapsed_time(&_manual_setpoint_last_called) * 1e-6f,  0.0002f, 0.04f);

			if (!_control_mode.flag_control_climb_rate_enabled &&
			    !_control_mode.flag_control_offboard_enabled) {

				if (_control_mode.flag_control_attitude_enabled) {
					// STABILIZED mode generate the attitude setpoint from manual user inputs
					_att_sp.roll_body = 0.0;
					_att_sp.pitch_body = 0.0;

					/* reset yaw setpoint to current position if needed */
					if (_reset_yaw_sp) {
						const float vehicle_yaw = Eulerf(Quatf(_vehicle_att.q)).psi();
						_manual_yaw_sp = vehicle_yaw;
						_reset_yaw_sp = false;

					} else {
						const float yaw_rate = math::radians(_param_gnd_man_y_max.get());
						_att_sp.yaw_sp_move_rate = _manual_control_setpoint.y * yaw_rate;
						_manual_yaw_sp = wrap_pi(_manual_yaw_sp + _att_sp.yaw_sp_move_rate * dt);
					}

					_att_sp.yaw_body = _manual_yaw_sp;
					_att_sp.thrust_body[0] = _manual_control_setpoint.z;

					Quatf q(Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body));
					q.copyTo(_att_sp.q_d);

					_att_sp.timestamp = hrt_absolute_time();


					_attitude_sp_pub.publish(_att_sp);

				} else {
					_act_controls.control[actuator_controls_s::INDEX_ROLL] = 0.0f; // Nominally roll: _manual_control_setpoint.y;
					_act_controls.control[actuator_controls_s::INDEX_PITCH] = 0.0f; // Nominally pitch: -_manual_control_setpoint.x;
					// Set heading from the manual roll input channel
					_act_controls.control[actuator_controls_s::INDEX_YAW] =
						_manual_control_setpoint.y; // Nominally yaw: _manual_control_setpoint.r;
					// Set throttle from the manual throttle channel
					_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = _manual_control_setpoint.z;
					_reset_yaw_sp = true;
				}

			} else {
				_reset_yaw_sp = true;
			}

			_manual_setpoint_last_called = hrt_absolute_time();
		}
	}
}

void
RoverPositionControl::position_setpoint_triplet_poll()
{
	if (_pos_sp_triplet_sub.updated()) {
		_pos_sp_triplet_sub.copy(&_pos_sp_triplet);
	}
}

void
RoverPositionControl::attitude_setpoint_poll()
{
	if (_att_sp_sub.updated()) {
		_att_sp_sub.copy(&_att_sp);
	}
}

bool
RoverPositionControl::control_position(const matrix::Vector2d &current_position,
				       const matrix::Vector3f &ground_speed, const position_setpoint_triplet_s &pos_sp_triplet)
{
	float dt = 0.01; // Using non zero value to a avoid division by zero

	if (_control_position_last_called > 0) {
		dt = hrt_elapsed_time(&_control_position_last_called) * 1e-6f;
	}

	_control_position_last_called = hrt_absolute_time();

	bool setpoint = true;

	if ((_control_mode.flag_control_auto_enabled ||
	     _control_mode.flag_control_offboard_enabled) && pos_sp_triplet.current.valid) {
		/* AUTONOMOUS FLIGHT */

		_control_mode_current = UGV_POSCTRL_MODE_AUTO;

		/* get circle mode */
		//bool was_circle_mode = _gnd_control.circle_mode();

		/* current waypoint (the one currently heading for) */
		matrix::Vector2d curr_wp(pos_sp_triplet.current.lat, pos_sp_triplet.current.lon);

		/* previous waypoint */
		matrix::Vector2d prev_wp = curr_wp;

		if (pos_sp_triplet.previous.valid) {
			prev_wp(0) = pos_sp_triplet.previous.lat;
			prev_wp(1) = pos_sp_triplet.previous.lon;
		}

		matrix::Vector2f ground_speed_2d(ground_speed);

		float mission_throttle = _param_throttle_cruise.get();

		/* Just control the throttle */
		if (_param_speed_control_mode.get() == 1) {
			/* control the speed in closed loop */

			float mission_target_speed = _param_gndspeed_trim.get();

			if (PX4_ISFINITE(_pos_sp_triplet.current.cruising_speed) &&
			    _pos_sp_triplet.current.cruising_speed > 0.1f) {
				mission_target_speed = _pos_sp_triplet.current.cruising_speed;
			}

			// Velocity in body frame
			const Dcmf R_to_body(Quatf(_vehicle_att.q).inversed());
			const Vector3f vel = R_to_body * Vector3f(ground_speed(0), ground_speed(1), ground_speed(2));

			const float x_vel = vel(0);
			const float x_acc = _vehicle_acceleration_sub.get().xyz[0];

			// Compute airspeed control out and just scale it as a constant
			mission_throttle = _param_throttle_speed_scaler.get()
					   * pid_calculate(&_speed_ctrl, mission_target_speed, x_vel, x_acc, dt);

			// Constrain throttle between min and max
			mission_throttle = math::constrain(mission_throttle, _param_throttle_min.get(), _param_throttle_max.get());

		} else {
			/* Just control throttle in open loop */
			if (PX4_ISFINITE(_pos_sp_triplet.current.cruising_throttle) &&
			    _pos_sp_triplet.current.cruising_throttle > 0.01f) {

				mission_throttle = _pos_sp_triplet.current.cruising_throttle;
			}
		}

		float dist_target = get_distance_to_next_waypoint(_global_pos.lat, _global_pos.lon,
				    (double)curr_wp(0), (double)curr_wp(1)); // pos_sp_triplet.current.lat, pos_sp_triplet.current.lon);

		//PX4_INFO("Setpoint type %d", (int) pos_sp_triplet.current.type );
		//PX4_INFO(" State machine state %d", (int) _pos_ctrl_state);
		//PX4_INFO(" Setpoint Lat %f, Lon %f", (double) curr_wp(0), (double)curr_wp(1));
		//PX4_INFO(" Distance to target %f", (double) dist_target);

		switch (_pos_ctrl_state) {
		case GOTO_WAYPOINT: {
				if (dist_target < _param_nav_loiter_rad.get()) {
					_pos_ctrl_state = STOPPING;  // We are closer than loiter radius to waypoint, stop.

				} else {
					_gnd_control.navigate_waypoints(prev_wp, curr_wp, current_position, ground_speed_2d);

					_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = mission_throttle;

					float desired_r = ground_speed_2d.norm_squared() / math::abs_t(_gnd_control.nav_lateral_acceleration_demand());
					float desired_theta = (0.5f * M_PI_F) - atan2f(desired_r, _param_wheel_base.get());
					float control_effort = (desired_theta / _param_max_turn_angle.get()) * sign(
								       _gnd_control.nav_lateral_acceleration_demand());
					control_effort = math::constrain(control_effort, -1.0f, 1.0f);
					_act_controls.control[actuator_controls_s::INDEX_YAW] = control_effort;
				}
			}
			break;

		case STOPPING: {
				_act_controls.control[actuator_controls_s::INDEX_YAW] = 0.0f;
				_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;
				// Note _prev_wp is different to the local prev_wp which is related to a mission waypoint.
				float dist_between_waypoints = get_distance_to_next_waypoint((double)_prev_wp(0), (double)_prev_wp(1),
							       (double)curr_wp(0), (double)curr_wp(1));

				if (dist_between_waypoints > 0) {
					_pos_ctrl_state = GOTO_WAYPOINT; // A new waypoint has arrived go to it
				}

				//PX4_INFO(" Distance between prev and curr waypoints %f", (double)dist_between_waypoints);
			}
			break;

		default:
			PX4_ERR("Unknown Rover State");
			_pos_ctrl_state = STOPPING;
			break;
		}

		_prev_wp = curr_wp;

	} else {
		_control_mode_current = UGV_POSCTRL_MODE_OTHER;
		setpoint = false;
	}

	return setpoint;
}

void
RoverPositionControl::control_velocity(const matrix::Vector3f &current_velocity)
{
	const Vector3f desired_velocity{_trajectory_setpoint.velocity};
	float dt = 0.01; // Using non zero value to a avoid division by zero

	const float mission_throttle = _param_throttle_cruise.get();
	const float desired_speed = desired_velocity.norm();

	if (desired_speed > 0.01f) {
		const Dcmf R_to_body(Quatf(_vehicle_att.q).inversed());
		const Vector3f vel = R_to_body * Vector3f(current_velocity(0), current_velocity(1), current_velocity(2));

		const float x_vel = vel(0);
		const float x_acc = _vehicle_acceleration_sub.get().xyz[0];

		const float control_throttle = pid_calculate(&_speed_ctrl, desired_speed, x_vel, x_acc, dt);

		//Constrain maximum throttle to mission throttle
		_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = math::constrain(control_throttle, 0.0f, mission_throttle);

		Vector3f desired_body_velocity;

		if (_velocity_frame == VelocityFrame::NED) {
			desired_body_velocity = desired_velocity;

		} else {
			// If the frame of the velocity setpoint is unknown, assume it is in local frame
			desired_body_velocity = R_to_body * desired_velocity;
		}

		const float desired_theta = atan2f(desired_body_velocity(1), desired_body_velocity(0));
		float control_effort = desired_theta / _param_max_turn_angle.get();
		control_effort = math::constrain(control_effort, -1.0f, 1.0f);

		_act_controls.control[actuator_controls_s::INDEX_YAW] = control_effort;

	} else {

		_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;
		_act_controls.control[actuator_controls_s::INDEX_YAW] = 0.0f;
	}
}

void
RoverPositionControl::control_attitude(const vehicle_attitude_s &att, const vehicle_attitude_setpoint_s &att_sp)
{
	// quaternion attitude control law, qe is rotation from q to qd
	const Quatf qe = Quatf(att.q).inversed() * Quatf(att_sp.q_d);
	const Eulerf euler_sp = qe;

	float control_effort = euler_sp(2) / _param_max_turn_angle.get();
	control_effort = math::constrain(control_effort, -1.0f, 1.0f);

	_act_controls.control[actuator_controls_s::INDEX_YAW] = control_effort;

	const float control_throttle = att_sp.thrust_body[0];

	_act_controls.control[actuator_controls_s::INDEX_THROTTLE] =  math::constrain(control_throttle, 0.0f, 1.0f);

}

void
RoverPositionControl::Run()
{
	parameters_update(true);

	if (_vehicle_attitude_sub.update(&_vehicle_att)) {

		/* check vehicle control mode for changes to publication state */
		vehicle_control_mode_poll();
		attitude_setpoint_poll();
		manual_control_setpoint_poll();

		_vehicle_acceleration_sub.update();

		/* update parameters from storage */
		parameters_update();

		/* only run controller if position changed */
		if (_local_pos_sub.update(&_local_pos)) {

			/* load local copies */
			_global_pos_sub.update(&_global_pos);

			position_setpoint_triplet_poll();

			// Convert Local setpoints to global setpoints
			if (_control_mode.flag_control_offboard_enabled) {
				if (!map_projection_initialized(&_global_local_proj_ref)
				    || (_global_local_proj_ref.timestamp != _local_pos.ref_timestamp)) {

					map_projection_init_timestamped(&_global_local_proj_ref, _local_pos.ref_lat, _local_pos.ref_lon,
									_local_pos.ref_timestamp);

					_global_local_alt0 = _local_pos.ref_alt;
				}

				_trajectory_setpoint_sub.update(&_trajectory_setpoint);

				// local -> global
				map_projection_reproject(&_global_local_proj_ref,
							 _trajectory_setpoint.position[0], _trajectory_setpoint.position[1],
							 &_pos_sp_triplet.current.lat, &_pos_sp_triplet.current.lon);

				_pos_sp_triplet.current.alt = _global_local_alt0 - _trajectory_setpoint.position[2];
				_pos_sp_triplet.current.valid = true;
			}

			// update the reset counters in any case
			_pos_reset_counter = _global_pos.lat_lon_reset_counter;

			matrix::Vector3f ground_speed(_local_pos.vx, _local_pos.vy,  _local_pos.vz);
			matrix::Vector2d current_position(_global_pos.lat, _global_pos.lon);
			matrix::Vector3f current_velocity(_local_pos.vx, _local_pos.vy, _local_pos.vz);

			if (!_control_mode.flag_control_manual_enabled && _control_mode.flag_control_position_enabled) {

				if (control_position(current_position, ground_speed, _pos_sp_triplet)) {

					//TODO: check if radius makes sense here
					float turn_distance = _param_l1_distance.get(); //_gnd_control.switch_distance(100.0f);

					// publish status
					position_controller_status_s pos_ctrl_status{};

					pos_ctrl_status.nav_roll = 0.0f;
					pos_ctrl_status.nav_pitch = 0.0f;
					pos_ctrl_status.nav_bearing = _gnd_control.nav_bearing();

					pos_ctrl_status.target_bearing = _gnd_control.target_bearing();
					pos_ctrl_status.xtrack_error = _gnd_control.crosstrack_error();

					pos_ctrl_status.wp_dist = get_distance_to_next_waypoint(_global_pos.lat, _global_pos.lon,
								  _pos_sp_triplet.current.lat, _pos_sp_triplet.current.lon);

					pos_ctrl_status.acceptance_radius = turn_distance;
					pos_ctrl_status.yaw_acceptance = NAN;

					pos_ctrl_status.timestamp = hrt_absolute_time();

					_pos_ctrl_status_pub.publish(pos_ctrl_status);
				}

			} else if (!_control_mode.flag_control_manual_enabled && _control_mode.flag_control_velocity_enabled) {
				_trajectory_setpoint_sub.update(&_trajectory_setpoint);
				control_velocity(current_velocity);
			}
		}

		// Respond to an attitude update and run the attitude controller if enabled
		if (_control_mode.flag_control_attitude_enabled
		    && !_control_mode.flag_control_position_enabled
		    && !_control_mode.flag_control_velocity_enabled) {
			control_attitude(_vehicle_att, _att_sp);

		}

		/* Only publish if any of the proper modes are enabled */
		if (_control_mode.flag_control_velocity_enabled ||
		    _control_mode.flag_control_attitude_enabled ||
		    _control_mode.flag_control_position_enabled ||
		    _control_mode.flag_control_manual_enabled) {
			// timestamp and publish controls
			_act_controls.timestamp = hrt_absolute_time();
			_actuator_controls_pub.publish(_act_controls);
		}
	}
}

int RoverPositionControl::task_spawn(int argc, char *argv[])
{
	RoverPositionControl *instance = new RoverPositionControl();

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

int RoverPositionControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RoverPositionControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Controls the position of a ground rover using an L1 controller.

Publishes `actuator_controls_0` messages at a constant 250Hz.

### Implementation
Currently, this implementation supports only a few modes:

 * Full manual: Throttle and yaw controls are passed directly through to the actuators
 * Auto mission: The rover runs missions
 * Loiter: The rover will navigate to within the loiter radius, then stop the motors

### Examples
CLI usage example:
$ rover_pos_control start
$ rover_pos_control status
$ rover_pos_control stop

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rover_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start")
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int rover_pos_control_main(int argc, char *argv[])
{
	return RoverPositionControl::main(argc, argv);
}
