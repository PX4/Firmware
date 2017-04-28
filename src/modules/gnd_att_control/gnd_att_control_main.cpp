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
 * All the ackowledgments and credits for the fw wing app are reported in those files.
 *
 * @author Marco Zorzi <mzorzi@student.ethz.ch>
 */

#include "gnd_att_control.hpp"

/**
 * GroundRover attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int gnd_att_control_main(int argc, char *argv[]);

namespace att_gnd_control
{
GroundRoverAttitudeControl	*g_control = nullptr;
}

GroundRoverAttitudeControl::GroundRoverAttitudeControl() :

	_task_should_exit(false),
	_task_running(false),
	_control_task(-1),

	/* subscriptions */
	_att_sp_sub(-1),
	_battery_status_sub(-1),
	_ctrl_state_sub(-1),
	_global_pos_sub(-1),
	_manual_sub(-1),
	_params_sub(-1),
	_vcontrol_mode_sub(-1),
	_vehicle_land_detected_sub(-1),
	_vehicle_status_sub(-1),

	/* publications */
	_rate_sp_pub(nullptr),
	_attitude_sp_pub(nullptr),
	_actuators_0_pub(nullptr),
	_actuators_2_pub(nullptr),

	_rates_sp_id(nullptr),
	_actuators_id(nullptr),
	_attitude_setpoint_id(nullptr),

	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "fwa_dt")),
#if 0
	_nonfinite_input_perf(perf_alloc(PC_COUNT, "fwa_nani")),
	_nonfinite_output_perf(perf_alloc(PC_COUNT, "fwa_nano")),
#else
	_nonfinite_input_perf(nullptr),
	_nonfinite_output_perf(nullptr),
#endif
	/* states */
	_setpoint_valid(false),
	_debug(false),
	_yaw(0.0f)
{
	/* safely initialize structs */
	_actuators = {};
	_actuators_rvrframe = {};
	_att_sp = {};
	_battery_status = {};
	_ctrl_state = {};
	_global_pos = {};
	_manual = {};
	_rates_sp = {};
	_vcontrol_mode = {};
	_vehicle_land_detected = {};
	_vehicle_status = {};

	_parameter_handles.w_tc = param_find("GND_WR_TC");
	_parameter_handles.w_p = param_find("GND_WR_P");
	_parameter_handles.w_i = param_find("GND_WR_I");
	_parameter_handles.w_d = param_find("GND_WR_D");
	_parameter_handles.w_ff = param_find("GND_WR_FF");
	_parameter_handles.w_integrator_max = param_find("GND_WR_IMAX");
	_parameter_handles.w_rmax = param_find("GND_W_RMAX");

	_parameter_handles.airspeed_min = param_find("GND_AIRSPD_MIN");
	_parameter_handles.airspeed_trim = param_find("GND_AIRSPD_TRIM");
	_parameter_handles.airspeed_max = param_find("GND_AIRSPD_MAX");

	_parameter_handles.gspd_scaling_trim = param_find("GND_GSPD_SP_TRIM");

	_parameter_handles.trim_yaw = param_find("TRIM_YAW");

	_parameter_handles.man_yaw_scale = param_find("GND_MAN_Y_SC");

	_parameter_handles.bat_scale_en = param_find("GND_BAT_SCALE_EN");

	/* fetch initial parameter values */
	parameters_update();
}

GroundRoverAttitudeControl::~GroundRoverAttitudeControl()
{
	if (_control_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	perf_free(_loop_perf);
	perf_free(_nonfinite_input_perf);
	perf_free(_nonfinite_output_perf);

	att_gnd_control::g_control = nullptr;
}

int
GroundRoverAttitudeControl::parameters_update()
{

	param_get(_parameter_handles.w_tc, &(_parameters.w_tc));
	param_get(_parameter_handles.w_p, &(_parameters.w_p));
	param_get(_parameter_handles.w_i, &(_parameters.w_i));
	param_get(_parameter_handles.w_d, &(_parameters.w_d));
	param_get(_parameter_handles.w_ff, &(_parameters.w_ff));
	param_get(_parameter_handles.w_integrator_max, &(_parameters.w_integrator_max));
	param_get(_parameter_handles.w_rmax, &(_parameters.w_rmax));

	param_get(_parameter_handles.airspeed_min, &(_parameters.airspeed_min));
	param_get(_parameter_handles.airspeed_trim, &(_parameters.airspeed_trim));
	param_get(_parameter_handles.airspeed_max, &(_parameters.airspeed_max));

	param_get(_parameter_handles.gspd_scaling_trim, &(_parameters.gspd_scaling_trim));

	param_get(_parameter_handles.trim_yaw, &(_parameters.trim_yaw));
	param_get(_parameter_handles.man_yaw_scale, &(_parameters.man_yaw_scale));

	param_get(_parameter_handles.bat_scale_en, &_parameters.bat_scale_en);


	/* wheel control parameters */
	_wheel_ctrl.set_time_constant(_parameters.w_tc);
	_wheel_ctrl.set_k_p(_parameters.w_p);
	_wheel_ctrl.set_k_i(_parameters.w_i);
	_wheel_ctrl.set_k_ff(_parameters.w_ff);
	_wheel_ctrl.set_integrator_max(_parameters.w_integrator_max);
	_wheel_ctrl.set_max_rate(math::radians(_parameters.w_rmax));

	/* Steering pid parameters*/
	pid_init(&_steering_ctrl, PID_MODE_DERIVATIV_SET, 0.01f);
	pid_set_parameters(&_steering_ctrl,
			   _parameters.w_p,
			   _parameters.w_i,
			   _parameters.w_d,
			   _parameters.w_integrator_max,
			   1.0f);

	return PX4_OK;
}

void
GroundRoverAttitudeControl::vehicle_control_mode_poll()
{
	bool vcontrol_mode_updated;

	/* Check if vehicle control mode has changed */
	orb_check(_vcontrol_mode_sub, &vcontrol_mode_updated);

	if (vcontrol_mode_updated) {

		orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &_vcontrol_mode);
	}
}

void
GroundRoverAttitudeControl::vehicle_manual_poll()
{
	bool manual_updated;

	/* get pilots inputs */
	orb_check(_manual_sub, &manual_updated);

	if (manual_updated) {

		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
	}
}

void
GroundRoverAttitudeControl::vehicle_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool att_sp_updated;
	orb_check(_att_sp_sub, &att_sp_updated);

	if (att_sp_updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub, &_att_sp);
		_setpoint_valid = true;
	}
}

void
GroundRoverAttitudeControl::global_pos_poll()
{
	/* check if there is a new global position */
	bool global_pos_updated;
	orb_check(_global_pos_sub, &global_pos_updated);

	if (global_pos_updated) {
		orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
	}
}

void
GroundRoverAttitudeControl::vehicle_status_poll()
{
	/* check if there is new status information */
	bool vehicle_status_updated;
	orb_check(_vehicle_status_sub, &vehicle_status_updated);

	if (vehicle_status_updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (!_rates_sp_id) {
			if (_vehicle_status.is_vtol) {
				_rates_sp_id = ORB_ID(fw_virtual_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_virtual_fw);
				_attitude_setpoint_id = ORB_ID(fw_virtual_attitude_setpoint);

			} else {
				_rates_sp_id = ORB_ID(vehicle_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_0);
				_attitude_setpoint_id = ORB_ID(vehicle_attitude_setpoint);
			}
		}
	}
}

void
GroundRoverAttitudeControl::vehicle_land_detected_poll()
{
	/* check if there is new status information */
	bool vehicle_land_detected_updated;
	orb_check(_vehicle_land_detected_sub, &vehicle_land_detected_updated);

	if (vehicle_land_detected_updated) {
		orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);
	}
}

void
GroundRoverAttitudeControl::battery_status_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_battery_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(battery_status), _battery_status_sub, &_battery_status);
	}
}

void
GroundRoverAttitudeControl::task_main_trampoline(int argc, char *argv[])
{
	att_gnd_control::g_control->task_main();
}

void
GroundRoverAttitudeControl::task_main()
{
	/*
	 * do subscriptions
	 */
	_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	_battery_status_sub = orb_subscribe(ORB_ID(battery_status));

	parameters_update();

	/* get an initial update for all sensor and status data */
	vehicle_setpoint_poll();
	vehicle_control_mode_poll();
	vehicle_manual_poll();
	vehicle_status_poll();
	vehicle_land_detected_poll();
	battery_status_poll();

	/* wakeup source */
	px4_pollfd_struct_t fds[2];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _ctrl_state_sub;
	fds[1].events = POLLIN;

	_task_running = true;

	while (!_task_should_exit) {
		static int loop_counter = 0;

		/* wait for up to 500ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		perf_begin(_loop_perf);

		/* only update parameters if they changed */
		if (fds[0].revents & POLLIN) {
			/* read from param to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

		/* only run controller if attitude changed */
		if (fds[1].revents & POLLIN) {
			static uint64_t last_run = 0;
			float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too large deltaT's */
			if (deltaT > 1.0f ||
			    fabsf(deltaT) < 0.00001f ||
			    !PX4_ISFINITE(deltaT)) {
				deltaT = 0.01f;
			}

			/* load local copies */
			orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);

			/* get current rotation matrix and euler angles from control state quaternions */
			math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
			_R = q_att.to_dcm();

			math::Vector<3> euler_angles;
			euler_angles = _R.to_euler();
			_yaw     = euler_angles(2);

			vehicle_setpoint_poll();

			vehicle_control_mode_poll();

			vehicle_manual_poll();

			global_pos_poll();

			vehicle_status_poll();

			vehicle_land_detected_poll();

			battery_status_poll();

			/* lock integrator until control is started */
			bool lock_integrator;

			if (_vcontrol_mode.flag_control_rates_enabled && !_vehicle_status.is_rotary_wing) {
				lock_integrator = false;

			} else {
				lock_integrator = true;
			}

			/* Simple handling of failsafe: stop motors */
			if (_vcontrol_mode.flag_control_termination_enabled) {
				_actuators_rvrframe.control[3] = 0.0f;
			}

			/* if we are in rotary wing mode or vehicle is vtol do nothing */
			if (_vehicle_status.is_rotary_wing || _vehicle_status.is_vtol) {
				continue;
			}

			/* decide if in stabilized or full manual control */
			if (_vcontrol_mode.flag_control_rates_enabled) {
				/* scale around tuning airspeed */
				float airspeed;

				bool nonfinite = !PX4_ISFINITE(_ctrl_state.airspeed);

				/* if airspeed is non-finite or not valid or if we are asked not to control it, we assume the normal average speed */
				if (nonfinite || !_ctrl_state.airspeed_valid) {
					airspeed = _parameters.airspeed_trim;

					if (nonfinite) {
						perf_count(_nonfinite_input_perf);
					}

				} else {
					/* prevent numerical drama by setting 0.01 m/s minimal speed */
					airspeed = math::max(0.01f, _ctrl_state.airspeed);
				}

				// calculate groundspeed
				float groundspeed = sqrtf(_global_pos.vel_n * _global_pos.vel_n +
							  _global_pos.vel_e * _global_pos.vel_e);

				// TODO: this should be changed: I don't want to reduce steering because of the speed,
				// I want to reduce the speed because of the desired steering angle.
				float groundspeed_scaler =  _parameters.gspd_scaling_trim;

				float yaw_sp = 0.0f;
				float yaw_manual = 0.0f;
				float throttle_sp = 0.0f;

				yaw_sp = _att_sp.yaw_body;
				throttle_sp = _att_sp.thrust;

				/* allow manual yaw in manual modes */
				if (_vcontrol_mode.flag_control_manual_enabled) {
					yaw_manual = _manual.r;
				}

				if (_att_sp.yaw_reset_integral
				    || _vehicle_land_detected.landed
				    || (_vehicle_status.is_rotary_wing && !_vehicle_status.in_transition_mode)) {

					_wheel_ctrl.reset_integrator();
				}

				/* Prepare data for attitude controllers */
				struct ECL_ControlData control_input = {};
				control_input.roll = 0.0f;//_roll;
				control_input.pitch = 0.0f;//_pitch;
				control_input.yaw = _yaw;
				control_input.body_x_rate = _ctrl_state.roll_rate;
				control_input.body_y_rate = _ctrl_state.pitch_rate;
				control_input.body_z_rate = _ctrl_state.yaw_rate;
				control_input.roll_setpoint = 0.0f;
				control_input.pitch_setpoint = 0.0f;
				control_input.yaw_setpoint = yaw_sp;
				control_input.airspeed_min = _parameters.airspeed_min;
				control_input.airspeed_max = _parameters.airspeed_max;
				control_input.airspeed = airspeed;
				control_input.scaler = 0.0f;//airspeed_scaling;
				control_input.lock_integrator = lock_integrator;
				control_input.groundspeed = groundspeed;
				control_input.groundspeed_scaler = groundspeed_scaler;

				/* Run attitude controllers */
				if (_vcontrol_mode.flag_control_attitude_enabled) {
					_wheel_ctrl.control_attitude(control_input);

					/* Calculate the control output for the steering as yaw */
					float yaw_u = pid_calculate(&_steering_ctrl, yaw_sp, _yaw, _ctrl_state.yaw_rate, deltaT);

					math::constrain(yaw_u, -1.0f, 1.0f);

					/* possibly useful debug*/
					// warnx("yaw_u: %.4f | yaw_sp: %.4f | _yaw: %.4f", (double)yaw_u, (double)yaw_sp, (double)_yaw);

					_actuators.control[actuator_controls_s::INDEX_YAW] = (PX4_ISFINITE(yaw_u)) ? yaw_u + _parameters.trim_yaw :
							_parameters.trim_yaw;

					/* add in manual steering control */
					_actuators.control[actuator_controls_s::INDEX_YAW] += yaw_manual;

					if (!PX4_ISFINITE(yaw_u)) {
						_wheel_ctrl.reset_integrator();
						perf_count(_nonfinite_output_perf);

						if (_debug && loop_counter % 10 == 0) {
							warnx("yaw_u %.4f", (double)yaw_u);
						}
					}

					/* throttle passed through if it is finite and if no engine failure was detected */
					_actuators.control[actuator_controls_s::INDEX_THROTTLE] = (PX4_ISFINITE(throttle_sp) &&
							!(_vehicle_status.engine_failure ||
							  _vehicle_status.engine_failure_cmd)) ?
							throttle_sp : 0.0f;

					/* scale effort by battery status */
					if (_parameters.bat_scale_en && _battery_status.scale > 0.0f &&
					    _actuators.control[actuator_controls_s::INDEX_THROTTLE] > 0.1f) {
						_actuators.control[actuator_controls_s::INDEX_THROTTLE] *= _battery_status.scale;
					}

					if (!PX4_ISFINITE(throttle_sp)) {
						if (_debug && loop_counter % 10 == 0) {
							warnx("throttle_sp %.4f", (double)throttle_sp);
						}
					}
				}

				/*
				 * Lazily publish the rate setpoint (for analysis, the actuators are published below)
				 * only once available
				 */
				_rates_sp.timestamp = hrt_absolute_time();

				if (_rate_sp_pub != nullptr) {
					/* publish the attitude rates setpoint */
					orb_publish(_rates_sp_id, _rate_sp_pub, &_rates_sp);

				} else if (_rates_sp_id) {
					/* advertise the attitude rates setpoint */
					_rate_sp_pub = orb_advertise(_rates_sp_id, &_rates_sp);
				}

			} else {
				/* manual/direct control */
				_actuators.control[actuator_controls_s::INDEX_ROLL] = _manual.y;
				_actuators.control[actuator_controls_s::INDEX_PITCH] = -_manual.x;
				_actuators.control[actuator_controls_s::INDEX_YAW] = _manual.r * _parameters.man_yaw_scale + _parameters.trim_yaw;
				_actuators.control[actuator_controls_s::INDEX_THROTTLE] = _manual.z;
			}

			/* lazily publish the setpoint only once available */
			_actuators.timestamp = hrt_absolute_time();
			_actuators.timestamp_sample = _ctrl_state.timestamp;
			_actuators_rvrframe.timestamp = hrt_absolute_time();
			_actuators_rvrframe.timestamp_sample = _ctrl_state.timestamp;

			/* Only publish if any of the proper modes are enabled */
			if (_vcontrol_mode.flag_control_rates_enabled ||
			    _vcontrol_mode.flag_control_attitude_enabled ||
			    _vcontrol_mode.flag_control_manual_enabled) {
				/* publish the actuator controls */
				if (_actuators_0_pub != nullptr) {
					orb_publish(_actuators_id, _actuators_0_pub, &_actuators);

				} else if (_actuators_id) {
					_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
				}

				if (_actuators_2_pub != nullptr) {
					/* publish the actuator controls*/
					orb_publish(ORB_ID(actuator_controls_2), _actuators_2_pub, &_actuators_rvrframe);

				} else {
					/* advertise and publish */
					_actuators_2_pub = orb_advertise(ORB_ID(actuator_controls_2), &_actuators_rvrframe);
				}
			}
		}

		loop_counter++;
		perf_end(_loop_perf);
	}

	warnx("exiting.\n");

	_control_task = -1;
	_task_running = false;
}

int
GroundRoverAttitudeControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("gnd_att_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1500,
					   (px4_main_t)&GroundRoverAttitudeControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return PX4_OK;
}

int gnd_att_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: gnd_att_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (att_gnd_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		att_gnd_control::g_control = new GroundRoverAttitudeControl;

		if (att_gnd_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (PX4_OK != att_gnd_control::g_control->start()) {
			delete att_gnd_control::g_control;
			att_gnd_control::g_control = nullptr;
			warn("start failed");
			return 1;
		}

		/* check if the waiting is necessary at all */
		if (att_gnd_control::g_control == nullptr || !att_gnd_control::g_control->task_running()) {

			/* avoid memory fragmentation by not exiting start handler until the task has fully started */
			while (att_gnd_control::g_control == nullptr || !att_gnd_control::g_control->task_running()) {
				usleep(50000);
				printf(".");
				fflush(stdout);
			}

			printf("\n");
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (att_gnd_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete att_gnd_control::g_control;
		att_gnd_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (att_gnd_control::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}
