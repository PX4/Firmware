/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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
 * @file mc_att_control_main.cpp
 * Multicopter attitude controller.
 *
 * @author Lorenz Meier		<lorenz@px4.io>
 * @author Anton Babushkin	<anton.babushkin@me.com>
 * @author Sander Smeets	<sander@droneslab.com>
 * @author Matthias Grob	<maetugr@gmail.com>
 * @author Beat Küng		<beat-kueng@gmx.net>
 *
 */

#include "mc_att_control.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;

MulticopterAttitudeControl::MulticopterAttitudeControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::att_pos_ctrl),
	_vehicle_attitude_setpoint_pub(vtol ? ORB_ID(mc_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	if (vtol) {
		int32_t vt_type = -1;

		if (param_get(param_find("VT_TYPE"), &vt_type) == PX4_OK) {
			_is_tailsitter = (static_cast<vtol_type>(vt_type) == vtol_type::TAILSITTER);
		}
	}

	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	/* initialize quaternions in messages to be valid */
	_v_att.q[0] = 1.f;
	_v_att_sp.q_d[0] = 1.f;

	parameters_updated();
}

MulticopterAttitudeControl::~MulticopterAttitudeControl()
{
	perf_free(_loop_perf);
}

bool
MulticopterAttitudeControl::init()
{
	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("vehicle_attitude callback registration failed!");
		return false;
	}

	return true;
}

void
MulticopterAttitudeControl::parameters_updated()
{
	// Store some of the parameters in a more convenient way & precompute often-used values
	_attitude_control.setProportionalGain(Vector3f(_param_mc_roll_p.get(), _param_mc_pitch_p.get(), _param_mc_yaw_p.get()));

	// angular rate limits
	using math::radians;
	_attitude_control.setRateLimit(Vector3f(radians(_param_mc_rollrate_max.get()), radians(_param_mc_pitchrate_max.get()),
						radians(_param_mc_yawrate_max.get())));

	_man_tilt_max = math::radians(_param_mpc_man_tilt_max.get());
}

float
MulticopterAttitudeControl::throttle_curve(float throttle_stick_input)
{
	float throttle_min = _vehicle_land_detected.landed ? 0.0f : _param_mpc_manthr_min.get();

	// throttle_stick_input is in range [0, 1]
	switch (_param_mpc_thr_curve.get()) {
	case 1: // no rescaling to hover throttle
		return throttle_min + throttle_stick_input * (_param_mpc_thr_max.get() - throttle_min);

	default: // 0 or other: rescale to hover throttle at 0.5 stick
		if (throttle_stick_input < 0.5f) {
			return (_param_mpc_thr_hover.get() - throttle_min) / 0.5f * throttle_stick_input +
			       throttle_min;

		} else {
			return (_param_mpc_thr_max.get() - _param_mpc_thr_hover.get()) / 0.5f * (throttle_stick_input - 1.0f) +
			       _param_mpc_thr_max.get();
		}
	}
}

void
MulticopterAttitudeControl::generate_attitude_setpoint(float dt, bool reset_yaw_sp)
{
	vehicle_attitude_setpoint_s attitude_setpoint{};
	const float yaw = Eulerf(Quatf(_v_att.q)).psi();

	/* reset yaw setpoint to current position if needed */
	if (reset_yaw_sp) {
		_man_yaw_sp = yaw;

	} else if (_manual_control_sp.z > 0.05f || _param_mc_airmode.get() == (int32_t)Mixer::Airmode::roll_pitch_yaw) {

		const float yaw_rate = math::radians(_param_mpc_man_y_max.get());
		attitude_setpoint.yaw_sp_move_rate = _manual_control_sp.r * yaw_rate;
		_man_yaw_sp = wrap_pi(_man_yaw_sp + attitude_setpoint.yaw_sp_move_rate * dt);
	}

	/*
	 * Input mapping for roll & pitch setpoints
	 * ----------------------------------------
	 * We control the following 2 angles:
	 * - tilt angle, given by sqrt(x*x + y*y)
	 * - the direction of the maximum tilt in the XY-plane, which also defines the direction of the motion
	 *
	 * This allows a simple limitation of the tilt angle, the vehicle flies towards the direction that the stick
	 * points to, and changes of the stick input are linear.
	 */
	const float x = _manual_control_sp.x * _man_tilt_max;
	const float y = _manual_control_sp.y * _man_tilt_max;

	// we want to fly towards the direction of (x, y), so we use a perpendicular axis angle vector in the XY-plane
	Vector2f v = Vector2f(y, -x);
	float v_norm = v.norm(); // the norm of v defines the tilt angle

	if (v_norm > _man_tilt_max) { // limit to the configured maximum tilt angle
		v *= _man_tilt_max / v_norm;
	}

	Quatf q_sp_rpy = AxisAnglef(v(0), v(1), 0.f);
	Eulerf euler_sp = q_sp_rpy;
	attitude_setpoint.roll_body = euler_sp(0);
	attitude_setpoint.pitch_body = euler_sp(1);
	// The axis angle can change the yaw as well (noticeable at higher tilt angles).
	// This is the formula by how much the yaw changes:
	//   let a := tilt angle, b := atan(y/x) (direction of maximum tilt)
	//   yaw = atan(-2 * sin(b) * cos(b) * sin^2(a/2) / (1 - 2 * cos^2(b) * sin^2(a/2))).
	attitude_setpoint.yaw_body = _man_yaw_sp + euler_sp(2);

	/* modify roll/pitch only if we're a VTOL */
	if (_vehicle_status.is_vtol) {
		// Construct attitude setpoint rotation matrix. Modify the setpoints for roll
		// and pitch such that they reflect the user's intention even if a large yaw error
		// (yaw_sp - yaw) is present. In the presence of a yaw error constructing a rotation matrix
		// from the pure euler angle setpoints will lead to unexpected attitude behaviour from
		// the user's view as the euler angle sequence uses the  yaw setpoint and not the current
		// heading of the vehicle.
		// However there's also a coupling effect that causes oscillations for fast roll/pitch changes
		// at higher tilt angles, so we want to avoid using this on multicopters.
		// The effect of that can be seen with:
		// - roll/pitch into one direction, keep it fixed (at high angle)
		// - apply a fast yaw rotation
		// - look at the roll and pitch angles: they should stay pretty much the same as when not yawing

		// calculate our current yaw error
		float yaw_error = wrap_pi(attitude_setpoint.yaw_body - yaw);

		// compute the vector obtained by rotating a z unit vector by the rotation
		// given by the roll and pitch commands of the user
		Vector3f zB = {0.0f, 0.0f, 1.0f};
		Dcmf R_sp_roll_pitch = Eulerf(attitude_setpoint.roll_body, attitude_setpoint.pitch_body, 0.0f);
		Vector3f z_roll_pitch_sp = R_sp_roll_pitch * zB;

		// transform the vector into a new frame which is rotated around the z axis
		// by the current yaw error. this vector defines the desired tilt when we look
		// into the direction of the desired heading
		Dcmf R_yaw_correction = Eulerf(0.0f, 0.0f, -yaw_error);
		z_roll_pitch_sp = R_yaw_correction * z_roll_pitch_sp;

		// use the formula z_roll_pitch_sp = R_tilt * [0;0;1]
		// R_tilt is computed from_euler; only true if cos(roll) not equal zero
		// -> valid if roll is not +-pi/2;
		attitude_setpoint.roll_body = -asinf(z_roll_pitch_sp(1));
		attitude_setpoint.pitch_body = atan2f(z_roll_pitch_sp(0), z_roll_pitch_sp(2));
	}

	/* copy quaternion setpoint to attitude setpoint topic */
	Quatf q_sp = Eulerf(attitude_setpoint.roll_body, attitude_setpoint.pitch_body, attitude_setpoint.yaw_body);
	q_sp.copyTo(attitude_setpoint.q_d);

	attitude_setpoint.thrust_body[2] = -throttle_curve(_manual_control_sp.z);
	attitude_setpoint.timestamp = hrt_absolute_time();

	_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);
}

/**
 * Attitude controller.
 * Input: 'vehicle_attitude_setpoint' topics (depending on mode)
 * Output: '_rates_sp' vector
 */
void
MulticopterAttitudeControl::control_attitude()
{
	_v_att_sp_sub.update(&_v_att_sp);
	_rates_sp = _attitude_control.update(Quatf(_v_att.q), Quatf(_v_att_sp.q_d), _v_att_sp.yaw_sp_move_rate);
}

void
MulticopterAttitudeControl::publish_rates_setpoint()
{
	vehicle_rates_setpoint_s v_rates_sp{};

	v_rates_sp.roll = _rates_sp(0);
	v_rates_sp.pitch = _rates_sp(1);
	v_rates_sp.yaw = _rates_sp(2);
	v_rates_sp.thrust_body[0] = _v_att_sp.thrust_body[0];
	v_rates_sp.thrust_body[1] = _v_att_sp.thrust_body[1];
	v_rates_sp.thrust_body[2] = _v_att_sp.thrust_body[2];
	v_rates_sp.timestamp = hrt_absolute_time();

	_v_rates_sp_pub.publish(v_rates_sp);
}

void
MulticopterAttitudeControl::Run()
{
	if (should_exit()) {
		_vehicle_attitude_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_params_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_params_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	// run controller on attitude updates
	const uint8_t prev_quat_reset_counter = _v_att.quat_reset_counter;

	if (_vehicle_attitude_sub.update(&_v_att)) {

		// Check for a heading reset
		if (prev_quat_reset_counter != _v_att.quat_reset_counter) {
			// we only extract the heading change from the delta quaternion
			_man_yaw_sp += Eulerf(Quatf(_v_att.delta_q_reset)).psi();
		}

		const hrt_abstime now = hrt_absolute_time();

		// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((now - _last_run) / 1e6f), 0.0002f, 0.02f);
		_last_run = now;

		/* check for updates in other topics */
		_manual_control_sp_sub.update(&_manual_control_sp);
		_v_control_mode_sub.update(&_v_control_mode);
		_vehicle_land_detected_sub.update(&_vehicle_land_detected);
		_vehicle_status_sub.update(&_vehicle_status);

		/* Check if we are in rattitude mode and the pilot is above the threshold on pitch
		* or roll (yaw can rotate 360 in normal att control). If both are true don't
		* even bother running the attitude controllers */
		if (_v_control_mode.flag_control_rattitude_enabled) {
			_v_control_mode.flag_control_attitude_enabled =
				fabsf(_manual_control_sp.y) <= _param_mc_ratt_th.get() &&
				fabsf(_manual_control_sp.x) <= _param_mc_ratt_th.get();
		}

		bool attitude_setpoint_generated = false;

		const bool is_hovering = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
					 && !_vehicle_status.in_transition_mode;

		// vehicle is a tailsitter in transition mode
		const bool is_tailsitter_transition = _vehicle_status.in_transition_mode && _is_tailsitter;

		bool run_att_ctrl = _v_control_mode.flag_control_attitude_enabled && (is_hovering || is_tailsitter_transition);

		if (run_att_ctrl) {
			// Generate the attitude setpoint from stick inputs if we are in Manual/Stabilized mode
			if (_v_control_mode.flag_control_manual_enabled &&
			    !_v_control_mode.flag_control_altitude_enabled &&
			    !_v_control_mode.flag_control_velocity_enabled &&
			    !_v_control_mode.flag_control_position_enabled) {

				generate_attitude_setpoint(dt, _reset_yaw_sp);
				attitude_setpoint_generated = true;
			}

			control_attitude();

			if (_v_control_mode.flag_control_yawrate_override_enabled) {
				/* Yaw rate override enabled, overwrite the yaw setpoint */
				_v_rates_sp_sub.update(&_v_rates_sp);
				const auto yawrate_reference = _v_rates_sp.yaw;
				_rates_sp(2) = yawrate_reference;
			}

			publish_rates_setpoint();
		}

		// reset yaw setpoint during transitions, tailsitter.cpp generates
		// attitude setpoint for the transition
		_reset_yaw_sp = (!attitude_setpoint_generated && !_v_control_mode.flag_control_rattitude_enabled) ||
				_vehicle_land_detected.landed ||
				(_vehicle_status.is_vtol && _vehicle_status.in_transition_mode);

	}

	perf_end(_loop_perf);
}

int MulticopterAttitudeControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterAttitudeControl *instance = new MulticopterAttitudeControl(vtol);

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

int MulticopterAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter attitude controller. It takes attitude
setpoints (`vehicle_attitude_setpoint`) as inputs and outputs a rate setpoint.

The controller has a P loop for angular error

Publication documenting the implemented Quaternion Attitude Control:
Nonlinear Quadrocopter Attitude Control (2013)
by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
Institute for Dynamic Systems and Control (IDSC), ETH Zurich

https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int mc_att_control_main(int argc, char *argv[])
{
	return MulticopterAttitudeControl::main(argc, argv);
}
