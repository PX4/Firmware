/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "rover position control")) // TODO : do we even need these perf counters
{
}

RoverPositionControl::~RoverPositionControl()
{
	perf_free(_loop_perf);
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
				   _param_speed_d.get(),
				   _param_speed_i.get(),
				   _param_speed_imax.get(),
				   _param_gndspeed_max.get());
	}
}

void
RoverPositionControl::vehicle_control_mode_poll()
{
	bool updated;
	orb_check(_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
	}
}

void
RoverPositionControl::manual_control_setpoint_poll()
{
	bool manual_updated;
	orb_check(_manual_control_sub, &manual_updated);

	if (manual_updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sub, &_manual);
	}
}

void
RoverPositionControl::position_setpoint_triplet_poll()
{
	bool pos_sp_triplet_updated;
	orb_check(_pos_sp_triplet_sub, &pos_sp_triplet_updated);

	if (pos_sp_triplet_updated) {
		orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);
	}
}

void
RoverPositionControl::attitude_setpoint_poll()
{
	bool att_sp_updated;
	orb_check(_att_sp_sub, &att_sp_updated);

	if (att_sp_updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub, &_att_sp);
	}
}

void
RoverPositionControl::vehicle_attitude_poll()
{
	bool att_updated;
	orb_check(_vehicle_attitude_sub, &att_updated);

	if (att_updated) {
		orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &_vehicle_att);
	}
}

bool
RoverPositionControl::control_position(const matrix::Vector2f &current_position,
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
		matrix::Vector2f curr_wp((float)pos_sp_triplet.current.lat, (float)pos_sp_triplet.current.lon);

		/* previous waypoint */
		matrix::Vector2f prev_wp = curr_wp;

		if (pos_sp_triplet.previous.valid) {
			prev_wp(0) = (float)pos_sp_triplet.previous.lat;
			prev_wp(1) = (float)pos_sp_triplet.previous.lon;
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

		float dist = get_distance_to_next_waypoint(_global_pos.lat, _global_pos.lon,
				pos_sp_triplet.current.lat, pos_sp_triplet.current.lon);

		bool should_idle = true;

		if (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER) {
			// Because of noise in measurements, if the rover was always trying to reach an exact point, it would
			// move around when it should be parked. So, I try to get the rover within loiter_radius/2, but then
			// once I reach that point, I don't move until I'm outside of loiter_radius.
			// TODO: Find out if there's a better measurement to use than loiter_radius.
			if (dist > pos_sp_triplet.current.loiter_radius) {
				_waypoint_reached = false;

			} else if (dist <= pos_sp_triplet.current.loiter_radius / 2) {
				_waypoint_reached = true;
			}

			should_idle = _waypoint_reached;

		} else if (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_POSITION ||
			   pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF ||
			   pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
			should_idle = false;
		}


		if (should_idle) {
			_act_controls.control[actuator_controls_s::INDEX_YAW] = 0.0f;
			_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;

		} else {

			/* waypoint is a plain navigation waypoint or the takeoff waypoint, does not matter */
			_gnd_control.navigate_waypoints(prev_wp, curr_wp, current_position, ground_speed_2d);

			_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = mission_throttle;

			float desired_r = ground_speed_2d.norm_squared() / math::abs_t(_gnd_control.nav_lateral_acceleration_demand());
			float desired_theta = (0.5f * M_PI_F) - atan2f(desired_r, _param_wheel_base.get());
			float control_effort = (desired_theta / _param_max_turn_angle.get()) * sign(
						       _gnd_control.nav_lateral_acceleration_demand());
			control_effort = math::constrain(control_effort, -1.0f, 1.0f);
			_act_controls.control[actuator_controls_s::INDEX_YAW] = control_effort;

		}

	} else {
		_control_mode_current = UGV_POSCTRL_MODE_OTHER;
		setpoint = false;
	}

	return setpoint;
}

void
RoverPositionControl::control_velocity(const matrix::Vector3f &current_velocity,
				       const position_setpoint_triplet_s &pos_sp_triplet)
{

	float dt = 0.01; // Using non zero value to a avoid division by zero

	const float mission_throttle = _param_throttle_cruise.get();
	const matrix::Vector3f desired_velocity{pos_sp_triplet.current.vx, pos_sp_triplet.current.vy, pos_sp_triplet.current.vz};
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

		if (pos_sp_triplet.current.velocity_frame == position_setpoint_s::VELOCITY_FRAME_BODY_NED) {
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

	const float control_throttle = math::constrain(att_sp.thrust_body[0], -1.0f, 1.0f);

	if (control_throttle >= 0.0f) {
		_act_controls.control[actuator_controls_s::INDEX_YAW] = control_effort;

	} else {
		// reverse steering, if driving backwards
		_act_controls.control[actuator_controls_s::INDEX_YAW] = -control_effort;
	}

	_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = control_throttle;

}

void
RoverPositionControl::run()
{
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_manual_control_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));

	_vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));

	/* rate limit control mode updates to 5Hz */
	orb_set_interval(_control_mode_sub, 200);

	/* rate limit position updates to 50 Hz */
	orb_set_interval(_global_pos_sub, 20);
	orb_set_interval(_local_pos_sub, 20);

	parameters_update(true);

	/* wakeup source(s) */
	px4_pollfd_struct_t fds[4];

	/* Setup of loop */
	fds[0].fd = _global_pos_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _manual_control_sub;
	fds[1].events = POLLIN;
	fds[2].fd = _sensor_combined_sub;
	fds[2].events = POLLIN;
	fds[3].fd = _vehicle_attitude_sub;
	fds[3].events = POLLIN;

	while (!should_exit()) {

		/* wait for up to 500ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 500);

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		/* check vehicle control mode for changes to publication state */
		vehicle_control_mode_poll();
		attitude_setpoint_poll();
		//manual_control_setpoint_poll();

		_vehicle_acceleration_sub.update();

		/* update parameters from storage */
		parameters_update();

		bool manual_mode = _control_mode.flag_control_manual_enabled;

		/* only run controller if position changed */
		if (fds[0].revents & POLLIN) {
			perf_begin(_loop_perf);

			/* load local copies */
			orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
			orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);

			position_setpoint_triplet_poll();

			//Convert Local setpoints to global setpoints
			if (_control_mode.flag_control_offboard_enabled) {
				if (!globallocalconverter_initialized()) {
					globallocalconverter_init(_local_pos.ref_lat, _local_pos.ref_lon,
								  _local_pos.ref_alt, _local_pos.ref_timestamp);

				} else {
					globallocalconverter_toglobal(_pos_sp_triplet.current.x, _pos_sp_triplet.current.y, _pos_sp_triplet.current.z,
								      &_pos_sp_triplet.current.lat, &_pos_sp_triplet.current.lon, &_pos_sp_triplet.current.alt);

				}
			}

			// update the reset counters in any case
			_pos_reset_counter = _global_pos.lat_lon_reset_counter;

			matrix::Vector3f ground_speed(_local_pos.vx, _local_pos.vy,  _local_pos.vz);
			matrix::Vector2f current_position((float)_global_pos.lat, (float)_global_pos.lon);
			matrix::Vector3f current_velocity(_local_pos.vx, _local_pos.vy, _local_pos.vz);

			if (!manual_mode && _control_mode.flag_control_position_enabled) {

				if (control_position(current_position, ground_speed, _pos_sp_triplet)) {

					//TODO: check if radius makes sense here
					float turn_distance = _param_l1_distance.get(); //_gnd_control.switch_distance(100.0f);

					// publish status
					position_controller_status_s pos_ctrl_status = {};

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

			} else if (!manual_mode && _control_mode.flag_control_velocity_enabled) {

				control_velocity(current_velocity, _pos_sp_triplet);

			}


			perf_end(_loop_perf);
		}

		if (fds[3].revents & POLLIN) {

			vehicle_attitude_poll();

			if (!manual_mode && _control_mode.flag_control_attitude_enabled
			    && !_control_mode.flag_control_position_enabled
			    && !_control_mode.flag_control_velocity_enabled) {

				control_attitude(_vehicle_att, _att_sp);

			}

		}

		if (fds[1].revents & POLLIN) {

			// This should be copied even if not in manual mode. Otherwise, the poll(...) call will keep
			// returning immediately and this loop will eat up resources.
			orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sub, &_manual);

			if (manual_mode) {
				/* manual/direct control */
				//PX4_INFO("Manual mode!");
				_act_controls.control[actuator_controls_s::INDEX_ROLL] = _manual.y;
				_act_controls.control[actuator_controls_s::INDEX_PITCH] = -_manual.x;
				_act_controls.control[actuator_controls_s::INDEX_YAW] = _manual.r; //TODO: Readd yaw scale param
				_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = _manual.z;
			}
		}

		if (fds[2].revents & POLLIN) {

			orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &_sensor_combined);

			//orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &_vehicle_att);
			_act_controls.timestamp = hrt_absolute_time();

			/* Only publish if any of the proper modes are enabled */
			if (_control_mode.flag_control_velocity_enabled ||
			    _control_mode.flag_control_attitude_enabled ||
			    manual_mode) {
				/* publish the actuator controls */
				_actuator_controls_pub.publish(_act_controls);
			}
		}

	}

	orb_unsubscribe(_control_mode_sub);
	orb_unsubscribe(_global_pos_sub);
	orb_unsubscribe(_local_pos_sub);
	orb_unsubscribe(_manual_control_sub);
	orb_unsubscribe(_pos_sp_triplet_sub);
	orb_unsubscribe(_vehicle_attitude_sub);
	orb_unsubscribe(_sensor_combined_sub);

	warnx("exiting.\n");
}

int RoverPositionControl::task_spawn(int argc, char *argv[])
{
	/* start the task */
	_task_id = px4_task_spawn_cmd("rover_pos_ctrl",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_POSITION_CONTROL,
				      1700,
				      (px4_main_t)&RoverPositionControl::run_trampoline,
				      nullptr);

	if (_task_id < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

RoverPositionControl *RoverPositionControl::instantiate(int argc, char *argv[])
{

	if (argc > 0) {
		PX4_WARN("Command 'start' takes no arguments.");
		return nullptr;
	}

	RoverPositionControl *instance = new RoverPositionControl();

	if (instance == nullptr) {
		PX4_ERR("Failed to instantiate RoverPositionControl object");
	}

	return instance;
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
