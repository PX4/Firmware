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

#include "FixedwingPositionControl.hpp"

#include <vtol_att_control/vtol_type.h>

FixedwingPositionControl::FixedwingPositionControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::att_pos_ctrl),
	_attitude_sp_pub(vtol ? ORB_ID(fw_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_launchDetector(this),
	_runway_takeoff(this)
{
	if (vtol) {
		_parameter_handles.airspeed_trans = param_find("VT_ARSP_TRANS");

		// VTOL parameter VTOL_TYPE
		int32_t vt_type = -1;
		param_get(param_find("VT_TYPE"), &vt_type);

		_vtol_tailsitter = (static_cast<vtol_type>(vt_type) == vtol_type::TAILSITTER);

	} else {
		_parameter_handles.airspeed_trans = PARAM_INVALID;
	}

	// limit to 50 Hz
	_global_pos_sub.set_interval_ms(20);

	_parameter_handles.l1_period = param_find("FW_L1_PERIOD");
	_parameter_handles.l1_damping = param_find("FW_L1_DAMPING");
	_parameter_handles.roll_slew_deg_sec = param_find("FW_L1_R_SLEW_MAX");

	_parameter_handles.airspeed_min = param_find("FW_AIRSPD_MIN");
	_parameter_handles.airspeed_trim = param_find("FW_AIRSPD_TRIM");
	_parameter_handles.airspeed_max = param_find("FW_AIRSPD_MAX");
	_parameter_handles.airspeed_disabled = param_find("FW_ARSP_MODE");

	_parameter_handles.pitch_limit_min = param_find("FW_P_LIM_MIN");
	_parameter_handles.pitch_limit_max = param_find("FW_P_LIM_MAX");
	_parameter_handles.roll_limit = param_find("FW_R_LIM");
	_parameter_handles.throttle_min = param_find("FW_THR_MIN");
	_parameter_handles.throttle_max = param_find("FW_THR_MAX");
	_parameter_handles.throttle_idle = param_find("FW_THR_IDLE");
	_parameter_handles.throttle_slew_max = param_find("FW_THR_SLEW_MAX");
	_parameter_handles.throttle_cruise = param_find("FW_THR_CRUISE");
	_parameter_handles.throttle_alt_scale = param_find("FW_THR_ALT_SCL");
	_parameter_handles.throttle_land_max = param_find("FW_THR_LND_MAX");
	_parameter_handles.man_roll_max_deg = param_find("FW_MAN_R_MAX");
	_parameter_handles.man_pitch_max_deg = param_find("FW_MAN_P_MAX");
	_parameter_handles.rollsp_offset_deg = param_find("FW_RSP_OFF");
	_parameter_handles.pitchsp_offset_deg = param_find("FW_PSP_OFF");

	_parameter_handles.land_slope_angle = param_find("FW_LND_ANG");
	_parameter_handles.land_H1_virt = param_find("FW_LND_HVIRT");
	_parameter_handles.land_flare_alt_relative = param_find("FW_LND_FLALT");
	_parameter_handles.land_flare_pitch_min_deg = param_find("FW_LND_FL_PMIN");
	_parameter_handles.land_flare_pitch_max_deg = param_find("FW_LND_FL_PMAX");
	_parameter_handles.land_thrust_lim_alt_relative = param_find("FW_LND_TLALT");
	_parameter_handles.land_heading_hold_horizontal_distance = param_find("FW_LND_HHDIST");
	_parameter_handles.land_use_terrain_estimate = param_find("FW_LND_USETER");
	_parameter_handles.land_early_config_change = param_find("FW_LND_EARLYCFG");
	_parameter_handles.land_airspeed_scale = param_find("FW_LND_AIRSPD_SC");
	_parameter_handles.land_throtTC_scale = param_find("FW_LND_THRTC_SC");

	_parameter_handles.time_const = param_find("FW_T_TIME_CONST");
	_parameter_handles.time_const_throt = param_find("FW_T_THRO_CONST");
	_parameter_handles.min_sink_rate = param_find("FW_T_SINK_MIN");
	_parameter_handles.max_sink_rate = param_find("FW_T_SINK_MAX");
	_parameter_handles.max_climb_rate = param_find("FW_T_CLMB_MAX");
	_parameter_handles.climbout_diff = param_find("FW_CLMBOUT_DIFF");
	_parameter_handles.throttle_damp = param_find("FW_T_THR_DAMP");
	_parameter_handles.integrator_gain = param_find("FW_T_INTEG_GAIN");
	_parameter_handles.vertical_accel_limit = param_find("FW_T_VERT_ACC");
	_parameter_handles.height_comp_filter_omega = param_find("FW_T_HGT_OMEGA");
	_parameter_handles.speed_comp_filter_omega = param_find("FW_T_SPD_OMEGA");
	_parameter_handles.roll_throttle_compensation = param_find("FW_T_RLL2THR");
	_parameter_handles.speed_weight = param_find("FW_T_SPDWEIGHT");
	_parameter_handles.pitch_damping = param_find("FW_T_PTCH_DAMP");
	_parameter_handles.heightrate_p = param_find("FW_T_HRATE_P");
	_parameter_handles.heightrate_ff = param_find("FW_T_HRATE_FF");
	_parameter_handles.speedrate_p = param_find("FW_T_SRATE_P");
	_parameter_handles.loiter_radius = param_find("NAV_LOITER_RAD");

	/* fetch initial parameter values */
	parameters_update();
}

FixedwingPositionControl::~FixedwingPositionControl()
{
	perf_free(_loop_perf);
}

bool
FixedwingPositionControl::init()
{
	if (!_global_pos_sub.registerCallback()) {
		PX4_ERR("vehicle global position callback registration failed!");
		return false;
	}

	return true;
}

int
FixedwingPositionControl::parameters_update()
{
	updateParams();

	param_get(_parameter_handles.airspeed_min, &(_parameters.airspeed_min));
	param_get(_parameter_handles.airspeed_trim, &(_parameters.airspeed_trim));
	param_get(_parameter_handles.airspeed_max, &(_parameters.airspeed_max));
	param_get(_parameter_handles.airspeed_disabled, &(_parameters.airspeed_disabled));

	param_get(_parameter_handles.pitch_limit_min, &(_parameters.pitch_limit_min));
	param_get(_parameter_handles.pitch_limit_max, &(_parameters.pitch_limit_max));
	param_get(_parameter_handles.throttle_min, &(_parameters.throttle_min));
	param_get(_parameter_handles.throttle_max, &(_parameters.throttle_max));
	param_get(_parameter_handles.throttle_idle, &(_parameters.throttle_idle));
	param_get(_parameter_handles.throttle_cruise, &(_parameters.throttle_cruise));
	param_get(_parameter_handles.throttle_alt_scale, &(_parameters.throttle_alt_scale));
	param_get(_parameter_handles.throttle_land_max, &(_parameters.throttle_land_max));

	param_get(_parameter_handles.man_roll_max_deg, &_parameters.man_roll_max_rad);
	_parameters.man_roll_max_rad = radians(_parameters.man_roll_max_rad);
	param_get(_parameter_handles.man_pitch_max_deg, &_parameters.man_pitch_max_rad);
	_parameters.man_pitch_max_rad = radians(_parameters.man_pitch_max_rad);

	param_get(_parameter_handles.rollsp_offset_deg, &_parameters.rollsp_offset_rad);
	_parameters.rollsp_offset_rad = radians(_parameters.rollsp_offset_rad);
	param_get(_parameter_handles.pitchsp_offset_deg, &_parameters.pitchsp_offset_rad);
	_parameters.pitchsp_offset_rad = radians(_parameters.pitchsp_offset_rad);

	param_get(_parameter_handles.climbout_diff, &(_parameters.climbout_diff));

	param_get(_parameter_handles.land_heading_hold_horizontal_distance,
		  &(_parameters.land_heading_hold_horizontal_distance));
	param_get(_parameter_handles.land_flare_pitch_min_deg, &(_parameters.land_flare_pitch_min_deg));
	param_get(_parameter_handles.land_flare_pitch_max_deg, &(_parameters.land_flare_pitch_max_deg));
	param_get(_parameter_handles.land_use_terrain_estimate, &(_parameters.land_use_terrain_estimate));
	param_get(_parameter_handles.land_early_config_change, &(_parameters.land_early_config_change));
	param_get(_parameter_handles.land_airspeed_scale, &(_parameters.land_airspeed_scale));
	param_get(_parameter_handles.land_throtTC_scale, &(_parameters.land_throtTC_scale));
	param_get(_parameter_handles.loiter_radius, &(_parameters.loiter_radius));

	// VTOL parameter VT_ARSP_TRANS
	if (_parameter_handles.airspeed_trans != PARAM_INVALID) {
		param_get(_parameter_handles.airspeed_trans, &_parameters.airspeed_trans);
	}


	float v = 0.0f;


	// L1 control parameters

	if (param_get(_parameter_handles.l1_damping, &v) == PX4_OK) {
		_l1_control.set_l1_damping(v);
	}

	if (param_get(_parameter_handles.l1_period, &v) == PX4_OK) {
		_l1_control.set_l1_period(v);
	}

	if (param_get(_parameter_handles.roll_limit, &v) == PX4_OK) {
		_l1_control.set_l1_roll_limit(radians(v));
	}

	if (param_get(_parameter_handles.roll_slew_deg_sec, &v) == PX4_OK) {
		_l1_control.set_roll_slew_rate(radians(v));
	}

	// TECS parameters

	param_get(_parameter_handles.max_climb_rate, &(_parameters.max_climb_rate));
	_tecs.set_max_climb_rate(_parameters.max_climb_rate);

	param_get(_parameter_handles.max_sink_rate, &(_parameters.max_sink_rate));
	_tecs.set_max_sink_rate(_parameters.max_sink_rate);

	param_get(_parameter_handles.speed_weight, &(_parameters.speed_weight));
	_tecs.set_speed_weight(_parameters.speed_weight);

	_tecs.set_indicated_airspeed_min(_parameters.airspeed_min);
	_tecs.set_indicated_airspeed_max(_parameters.airspeed_max);

	if (param_get(_parameter_handles.time_const_throt, &(_parameters.time_const_throt)) == PX4_OK) {
		_tecs.set_time_const_throt(_parameters.time_const_throt);
	}

	if (param_get(_parameter_handles.time_const, &v) == PX4_OK) {
		_tecs.set_time_const(v);
	}

	if (param_get(_parameter_handles.min_sink_rate, &v) == PX4_OK) {
		_tecs.set_min_sink_rate(v);
	}

	if (param_get(_parameter_handles.throttle_damp, &v) == PX4_OK) {
		_tecs.set_throttle_damp(v);
	}

	if (param_get(_parameter_handles.integrator_gain, &v) == PX4_OK) {
		_tecs.set_integrator_gain(v);
	}

	if (param_get(_parameter_handles.throttle_slew_max, &v) == PX4_OK) {
		_tecs.set_throttle_slewrate(v);
	}

	if (param_get(_parameter_handles.vertical_accel_limit, &v) == PX4_OK) {
		_tecs.set_vertical_accel_limit(v);
	}

	if (param_get(_parameter_handles.height_comp_filter_omega, &v) == PX4_OK) {
		_tecs.set_height_comp_filter_omega(v);
	}

	if (param_get(_parameter_handles.speed_comp_filter_omega, &v) == PX4_OK) {
		_tecs.set_speed_comp_filter_omega(v);
	}

	if (param_get(_parameter_handles.roll_throttle_compensation, &v) == PX4_OK) {
		_tecs.set_roll_throttle_compensation(v);
	}

	if (param_get(_parameter_handles.pitch_damping, &v) == PX4_OK) {
		_tecs.set_pitch_damping(v);
	}

	if (param_get(_parameter_handles.heightrate_p, &v) == PX4_OK) {
		_tecs.set_heightrate_p(v);
	}

	if (param_get(_parameter_handles.heightrate_ff, &v) == PX4_OK) {
		_tecs.set_heightrate_ff(v);
	}

	if (param_get(_parameter_handles.speedrate_p, &v) == PX4_OK) {
		_tecs.set_speedrate_p(v);
	}


	// Landing slope

	float land_slope_angle = 0.0f;
	param_get(_parameter_handles.land_slope_angle, &land_slope_angle);

	float land_flare_alt_relative = 0.0f;
	param_get(_parameter_handles.land_flare_alt_relative, &land_flare_alt_relative);

	float land_thrust_lim_alt_relative = 0.0f;
	param_get(_parameter_handles.land_thrust_lim_alt_relative, &land_thrust_lim_alt_relative);

	float land_H1_virt = 0.0f;
	param_get(_parameter_handles.land_H1_virt, &land_H1_virt);

	/* check if negative value for 2/3 of flare altitude is set for throttle cut */
	if (land_thrust_lim_alt_relative < 0.0f) {
		land_thrust_lim_alt_relative = 0.66f * land_flare_alt_relative;
	}

	_landingslope.update(radians(land_slope_angle), land_flare_alt_relative, land_thrust_lim_alt_relative, land_H1_virt);

	landing_status_publish();

	// sanity check parameters
	if ((_parameters.airspeed_max < _parameters.airspeed_min) ||
	    (_parameters.airspeed_max < 5.0f) ||
	    (_parameters.airspeed_min > 100.0f) ||
	    (_parameters.airspeed_trim < _parameters.airspeed_min) ||
	    (_parameters.airspeed_trim > _parameters.airspeed_max)) {

		mavlink_log_critical(&_mavlink_log_pub, "Airspeed parameters invalid");

		return PX4_ERROR;
	}

	return PX4_OK;
}

void
FixedwingPositionControl::vehicle_control_mode_poll()
{
	if (_control_mode_sub.updated()) {
		const bool was_armed = _control_mode.flag_armed;

		if (_control_mode_sub.copy(&_control_mode)) {

			// reset state when arming
			if (!was_armed && _control_mode.flag_armed) {
				reset_takeoff_state(true);
				reset_landing_state();
			}
		}
	}
}

void
FixedwingPositionControl::vehicle_command_poll()
{
	if (_vehicle_command_sub.updated()) {
		_vehicle_command_sub.copy(&_vehicle_command);
		handle_command();
	}
}

void
FixedwingPositionControl::airspeed_poll()
{
	bool airspeed_valid = _airspeed_valid;

	if (!_parameters.airspeed_disabled && _airspeed_validated_sub.update()) {

		const airspeed_validated_s &airspeed_validated = _airspeed_validated_sub.get();
		_eas2tas = 1.0f; //this is the default value, taken in case of invalid airspeed

		if (PX4_ISFINITE(airspeed_validated.equivalent_airspeed_m_s)
		    && PX4_ISFINITE(airspeed_validated.true_airspeed_m_s)
		    && (airspeed_validated.equivalent_airspeed_m_s > 0.0f)) {

			airspeed_valid = true;

			_airspeed_last_valid = airspeed_validated.timestamp;
			_airspeed = airspeed_validated.equivalent_airspeed_m_s;

			_eas2tas = constrain(airspeed_validated.true_airspeed_m_s / airspeed_validated.equivalent_airspeed_m_s, 0.9f, 2.0f);
		}

	} else {
		// no airspeed updates for one second
		if (airspeed_valid && (hrt_elapsed_time(&_airspeed_last_valid) > 1_s)) {
			airspeed_valid = false;
		}
	}

	// update TECS if validity changed
	if (airspeed_valid != _airspeed_valid) {
		_tecs.enable_airspeed(airspeed_valid);
		_airspeed_valid = airspeed_valid;
	}
}

void
FixedwingPositionControl::vehicle_attitude_poll()
{
	if (_vehicle_attitude_sub.update(&_att)) {
		/* set rotation matrix and euler angles */
		_R_nb = Quatf(_att.q);

		// if the vehicle is a tailsitter we have to rotate the attitude by the pitch offset
		// between multirotor and fixed wing flight
		if (_vtol_tailsitter) {
			Dcmf R_offset = Eulerf(0, M_PI_2_F, 0);
			_R_nb = _R_nb * R_offset;
		}

		const Eulerf euler_angles(_R_nb);
		_roll    = euler_angles(0);
		_pitch   = euler_angles(1);
		_yaw     = euler_angles(2);
	}
}

float
FixedwingPositionControl::get_demanded_airspeed()
{
	float altctrl_airspeed = 0;

	// neutral throttle corresponds to trim airspeed
	if (_manual.z < 0.5f) {
		// lower half of throttle is min to trim airspeed
		altctrl_airspeed = _parameters.airspeed_min +
				   (_parameters.airspeed_trim - _parameters.airspeed_min) *
				   _manual.z * 2;

	} else {
		// upper half of throttle is trim to max airspeed
		altctrl_airspeed = _parameters.airspeed_trim +
				   (_parameters.airspeed_max - _parameters.airspeed_trim) *
				   (_manual.z * 2 - 1);
	}

	return altctrl_airspeed;
}

float
FixedwingPositionControl::calculate_target_airspeed(float airspeed_demand, const Vector2f &ground_speed)
{
	/*
	 * Calculate accelerated stall airspeed factor from commanded bank angle and use it to increase minimum airspeed.
	 *
	 *  We don't know the stall speed of the aircraft, but assuming user defined
	 *  minimum airspeed (FW_AIRSPD_MIN) is slightly larger than stall speed
	 *  this is close enough.
	 *
	 * increase lift vector to balance additional weight in bank
	 *  cos(bank angle) = W/L = 1/n
	 *   n is the load factor
	 *
	 * lift is proportional to airspeed^2 so the increase in stall speed is
	 *  Vsacc = Vs * sqrt(n)
	 *
	 */
	float adjusted_min_airspeed = _parameters.airspeed_min;

	if (_airspeed_valid && PX4_ISFINITE(_att_sp.roll_body)) {

		adjusted_min_airspeed = constrain(_parameters.airspeed_min / sqrtf(cosf(_att_sp.roll_body)),
						  _parameters.airspeed_min, _parameters.airspeed_max);
	}

	// groundspeed undershoot
	if (!_l1_control.circle_mode()) {

		// rotate ground speed vector with current attitude
		Vector2f yaw_vector(_R_nb(0, 0), _R_nb(1, 0));
		yaw_vector.normalize();

		const float ground_speed_body = yaw_vector * ground_speed;

		/*
		 * This error value ensures that a plane (as long as its throttle capability is
		 * not exceeded) travels towards a waypoint (and is not pushed more and more away
		 * by wind). Not countering this would lead to a fly-away.
		 */
		if (ground_speed_body < _groundspeed_min.get()) {
			airspeed_demand += max(_groundspeed_min.get() - ground_speed_body, 0.0f);
		}
	}

	// add minimum ground speed undershoot (only non-zero in presence of sufficient wind)
	// sanity check: limit to range
	return constrain(airspeed_demand, adjusted_min_airspeed, _parameters.airspeed_max);
}

void
FixedwingPositionControl::tecs_status_publish()
{
	tecs_status_s t = {};

	switch (_tecs.tecs_mode()) {
	case TECS::ECL_TECS_MODE_NORMAL:
		t.mode = tecs_status_s::TECS_MODE_NORMAL;
		break;

	case TECS::ECL_TECS_MODE_UNDERSPEED:
		t.mode = tecs_status_s::TECS_MODE_UNDERSPEED;
		break;

	case TECS::ECL_TECS_MODE_BAD_DESCENT:
		t.mode = tecs_status_s::TECS_MODE_BAD_DESCENT;
		break;

	case TECS::ECL_TECS_MODE_CLIMBOUT:
		t.mode = tecs_status_s::TECS_MODE_CLIMBOUT;
		break;
	}

	t.altitude_sp = _tecs.hgt_setpoint_adj();
	t.altitude_filtered = _tecs.vert_pos_state();

	t.airspeed_sp = _tecs.TAS_setpoint_adj();
	t.airspeed_filtered = _tecs.tas_state();

	t.height_rate_setpoint = _tecs.hgt_rate_setpoint();
	t.height_rate = _tecs.vert_vel_state();

	t.airspeed_derivative_sp = _tecs.TAS_rate_setpoint();
	t.airspeed_derivative = _tecs.speed_derivative();

	t.total_energy_error = _tecs.STE_error();
	t.total_energy_rate_error = _tecs.STE_rate_error();

	t.energy_distribution_error = _tecs.SEB_error();
	t.energy_distribution_rate_error = _tecs.SEB_rate_error();

	t.throttle_integ = _tecs.throttle_integ_state();
	t.pitch_integ = _tecs.pitch_integ_state();

	t.timestamp = hrt_absolute_time();

	_tecs_status_pub.publish(t);
}

void
FixedwingPositionControl::status_publish()
{
	position_controller_status_s pos_ctrl_status = {};

	pos_ctrl_status.nav_roll = _att_sp.roll_body;
	pos_ctrl_status.nav_pitch = _att_sp.pitch_body;
	pos_ctrl_status.nav_bearing = _l1_control.nav_bearing();

	pos_ctrl_status.target_bearing = _l1_control.target_bearing();
	pos_ctrl_status.xtrack_error = _l1_control.crosstrack_error();

	pos_ctrl_status.wp_dist = get_distance_to_next_waypoint(_global_pos.lat, _global_pos.lon,
				  _pos_sp_triplet.current.lat, _pos_sp_triplet.current.lon);

	pos_ctrl_status.acceptance_radius = _l1_control.switch_distance(500.0f);

	pos_ctrl_status.yaw_acceptance = NAN;

	pos_ctrl_status.launch_detection_running = _launch_detection_state == LAUNCHDETECTION_RES_NONE
			&& _control_mode.flag_armed
			&& _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF
			&& _pos_sp_triplet.current.valid
			&& _launchDetector.launchDetectionEnabled();

	pos_ctrl_status.timestamp = hrt_absolute_time();

	_pos_ctrl_status_pub.publish(pos_ctrl_status);
}

void
FixedwingPositionControl::landing_status_publish()
{
	position_controller_landing_status_s pos_ctrl_landing_status = {};

	pos_ctrl_landing_status.slope_angle_rad = _landingslope.landing_slope_angle_rad();
	pos_ctrl_landing_status.horizontal_slope_displacement = _landingslope.horizontal_slope_displacement();
	pos_ctrl_landing_status.flare_length = _landingslope.flare_length();

	pos_ctrl_landing_status.abort_landing = _land_abort;

	pos_ctrl_landing_status.timestamp = hrt_absolute_time();

	_pos_ctrl_landing_status_pub.publish(pos_ctrl_landing_status);
}

void
FixedwingPositionControl::abort_landing(bool abort)
{
	// only announce changes
	if (abort && !_land_abort) {
		mavlink_log_critical(&_mavlink_log_pub, "Landing aborted");
	}

	_land_abort = abort;
	landing_status_publish();
}

void
FixedwingPositionControl::get_waypoint_heading_distance(float heading, position_setpoint_s &waypoint_prev,
		position_setpoint_s &waypoint_next, bool flag_init)
{
	position_setpoint_s temp_prev = waypoint_prev;
	position_setpoint_s temp_next = waypoint_next;

	if (flag_init) {
		// previous waypoint: HDG_HOLD_SET_BACK_DIST meters behind us
		waypoint_from_heading_and_distance(_global_pos.lat, _global_pos.lon, heading + radians(180.0f),
						   HDG_HOLD_SET_BACK_DIST, &temp_prev.lat, &temp_prev.lon);

		// next waypoint: HDG_HOLD_DIST_NEXT meters in front of us
		waypoint_from_heading_and_distance(_global_pos.lat, _global_pos.lon, heading,
						   HDG_HOLD_DIST_NEXT, &temp_next.lat, &temp_next.lon);

	} else {
		// use the existing flight path from prev to next

		// previous waypoint: shifted HDG_HOLD_REACHED_DIST + HDG_HOLD_SET_BACK_DIST
		create_waypoint_from_line_and_dist(waypoint_next.lat, waypoint_next.lon, waypoint_prev.lat, waypoint_prev.lon,
						   HDG_HOLD_REACHED_DIST + HDG_HOLD_SET_BACK_DIST, &temp_prev.lat, &temp_prev.lon);

		// next waypoint: shifted -(HDG_HOLD_DIST_NEXT + HDG_HOLD_REACHED_DIST)
		create_waypoint_from_line_and_dist(waypoint_next.lat, waypoint_next.lon, waypoint_prev.lat, waypoint_prev.lon,
						   -(HDG_HOLD_REACHED_DIST + HDG_HOLD_DIST_NEXT), &temp_next.lat, &temp_next.lon);
	}

	waypoint_prev = temp_prev;
	waypoint_prev.alt = _hold_alt;
	waypoint_prev.valid = true;

	waypoint_next = temp_next;
	waypoint_next.alt = _hold_alt;
	waypoint_next.valid = true;
}

float
FixedwingPositionControl::get_terrain_altitude_takeoff(float takeoff_alt,
		const vehicle_global_position_s &global_pos)
{
	if (PX4_ISFINITE(global_pos.terrain_alt) && global_pos.terrain_alt_valid) {
		return global_pos.terrain_alt;
	}

	return takeoff_alt;
}

bool
FixedwingPositionControl::update_desired_altitude(float dt)
{
	/*
	 * The complete range is -1..+1, so this is 6%
	 * of the up or down range or 3% of the total range.
	 */
	const float deadBand = 0.06f;

	/*
	 * The correct scaling of the complete range needs
	 * to account for the missing part of the slope
	 * due to the deadband
	 */
	const float factor = 1.0f - deadBand;

	/* Climbout mode sets maximum throttle and pitch up */
	bool climbout_mode = false;

	/*
	 * Reset the hold altitude to the current altitude if the uncertainty
	 * changes significantly.
	 * This is to guard against uncommanded altitude changes
	 * when the altitude certainty increases or decreases.
	 */

	if (fabsf(_althold_epv - _global_pos.epv) > ALTHOLD_EPV_RESET_THRESH) {
		_hold_alt = _global_pos.alt;
		_althold_epv = _global_pos.epv;
	}

	/*
	 * Manual control has as convention the rotation around
	 * an axis. Positive X means to rotate positively around
	 * the X axis in NED frame, which is pitching down
	 */
	if (_manual.x > deadBand) {
		/* pitching down */
		float pitch = -(_manual.x - deadBand) / factor;
		_hold_alt += (_parameters.max_sink_rate * dt) * pitch;
		_was_in_deadband = false;

	} else if (_manual.x < - deadBand) {
		/* pitching up */
		float pitch = -(_manual.x + deadBand) / factor;
		_hold_alt += (_parameters.max_climb_rate * dt) * pitch;
		_was_in_deadband = false;
		climbout_mode = (pitch > MANUAL_THROTTLE_CLIMBOUT_THRESH);

	} else if (!_was_in_deadband) {
		/* store altitude at which manual.x was inside deadBand
		 * The aircraft should immediately try to fly at this altitude
		 * as this is what the pilot expects when he moves the stick to the center */
		_hold_alt = _global_pos.alt;
		_althold_epv = _global_pos.epv;
		_was_in_deadband = true;
	}

	if (_vehicle_status.is_vtol) {
		if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING || _vehicle_status.in_transition_mode) {
			_hold_alt = _global_pos.alt;
		}
	}

	return climbout_mode;
}

bool
FixedwingPositionControl::in_takeoff_situation()
{
	// a VTOL does not need special takeoff handling
	if (_vehicle_status.is_vtol) {
		return false;
	}

	// in air for < 10s
	const hrt_abstime delta_takeoff = 10_s;

	return (hrt_elapsed_time(&_time_went_in_air) < delta_takeoff)
	       && (_global_pos.alt <= _takeoff_ground_alt + _parameters.climbout_diff);
}

void
FixedwingPositionControl::do_takeoff_help(float *hold_altitude, float *pitch_limit_min)
{
	/* demand "climbout_diff" m above ground if user switched into this mode during takeoff */
	if (in_takeoff_situation()) {
		*hold_altitude = _takeoff_ground_alt + _parameters.climbout_diff;
		*pitch_limit_min = radians(10.0f);
	}
}

bool
FixedwingPositionControl::control_position(const Vector2f &curr_pos, const Vector2f &ground_speed,
		const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr, const position_setpoint_s &pos_sp_next)
{
	float dt = 0.01f;

	if (_control_position_last_called > 0) {
		dt = hrt_elapsed_time(&_control_position_last_called) * 1e-6f;
	}

	_control_position_last_called = hrt_absolute_time();

	_l1_control.set_dt(dt);

	/* only run position controller in fixed-wing mode and during transitions for VTOL */
	if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && !_vehicle_status.in_transition_mode) {
		_control_mode_current = FW_POSCTRL_MODE_OTHER;
		return false;
	}

	bool setpoint = true;

	_att_sp.fw_control_yaw = false;		// by default we don't want yaw to be contoller directly with rudder
	_att_sp.apply_flaps = vehicle_attitude_setpoint_s::FLAPS_OFF;		// by default we don't use flaps

	Vector2f nav_speed_2d{ground_speed};

	if (_airspeed_valid) {
		// l1 navigation logic breaks down when wind speed exceeds max airspeed
		// compute 2D groundspeed from airspeed-heading projection
		const Vector2f air_speed_2d{_airspeed * cosf(_yaw), _airspeed * sinf(_yaw)};

		// angle between air_speed_2d and ground_speed
		const float air_gnd_angle = acosf((air_speed_2d * ground_speed) / (air_speed_2d.length() * ground_speed.length()));

		// if angle > 90 degrees or groundspeed is less than threshold, replace groundspeed with airspeed projection
		if ((fabsf(air_gnd_angle) > M_PI_2_F) || (ground_speed.length() < 3.0f)) {
			nav_speed_2d = air_speed_2d;
		}
	}

	/* no throttle limit as default */
	float throttle_max = 1.0f;

	/* save time when airplane is in air */
	if (!_was_in_air && !_vehicle_land_detected.landed) {
		_was_in_air = true;
		_time_went_in_air = hrt_absolute_time();
		_takeoff_ground_alt = _global_pos.alt;
	}

	/* reset flag when airplane landed */
	if (_vehicle_land_detected.landed) {
		_was_in_air = false;
	}

	/* Reset integrators if switching to this mode from a other mode in which posctl was not active */
	if (_control_mode_current == FW_POSCTRL_MODE_OTHER) {
		/* reset integrators */
		_tecs.reset_state();
	}

	if ((_control_mode.flag_control_auto_enabled || _control_mode.flag_control_offboard_enabled) && pos_sp_curr.valid) {
		/* AUTONOMOUS FLIGHT */

		_control_mode_current = FW_POSCTRL_MODE_AUTO;

		/* reset hold altitude */
		_hold_alt = _global_pos.alt;

		/* reset hold yaw */
		_hdg_hold_yaw = _yaw;

		/* get circle mode */
		bool was_circle_mode = _l1_control.circle_mode();

		/* restore TECS parameters, in case changed intermittently (e.g. in landing handling) */
		_tecs.set_speed_weight(_parameters.speed_weight);
		_tecs.set_time_const_throt(_parameters.time_const_throt);

		/* current waypoint (the one currently heading for) */
		Vector2f curr_wp((float)pos_sp_curr.lat, (float)pos_sp_curr.lon);

		/* Initialize attitude controller integrator reset flags to 0 */
		_att_sp.roll_reset_integral = false;
		_att_sp.pitch_reset_integral = false;
		_att_sp.yaw_reset_integral = false;

		/* previous waypoint */
		Vector2f prev_wp{0.0f, 0.0f};

		if (pos_sp_prev.valid) {
			prev_wp(0) = (float)pos_sp_prev.lat;
			prev_wp(1) = (float)pos_sp_prev.lon;

		} else {
			/*
			 * No valid previous waypoint, go for the current wp.
			 * This is automatically handled by the L1 library.
			 */
			prev_wp(0) = (float)pos_sp_curr.lat;
			prev_wp(1) = (float)pos_sp_curr.lon;
		}

		float mission_airspeed = _parameters.airspeed_trim;

		if (PX4_ISFINITE(pos_sp_curr.cruising_speed) &&
		    pos_sp_curr.cruising_speed > 0.1f) {

			mission_airspeed = pos_sp_curr.cruising_speed;
		}

		float mission_throttle = _parameters.throttle_cruise;

		if (PX4_ISFINITE(pos_sp_curr.cruising_throttle) &&
		    pos_sp_curr.cruising_throttle > 0.01f) {

			mission_throttle = pos_sp_curr.cruising_throttle;
		}

		if (pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_IDLE) {
			_att_sp.thrust_body[0] = 0.0f;
			_att_sp.roll_body = 0.0f;
			_att_sp.pitch_body = 0.0f;

		} else if (pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_POSITION) {
			/* waypoint is a plain navigation waypoint */
			_l1_control.navigate_waypoints(prev_wp, curr_wp, curr_pos, nav_speed_2d);
			_att_sp.roll_body = _l1_control.get_roll_setpoint();
			_att_sp.yaw_body = _l1_control.nav_bearing();

			tecs_update_pitch_throttle(pos_sp_curr.alt,
						   calculate_target_airspeed(mission_airspeed, ground_speed),
						   radians(_parameters.pitch_limit_min) - _parameters.pitchsp_offset_rad,
						   radians(_parameters.pitch_limit_max) - _parameters.pitchsp_offset_rad,
						   _parameters.throttle_min,
						   _parameters.throttle_max,
						   mission_throttle,
						   false,
						   radians(_parameters.pitch_limit_min));

		} else if (pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_LOITER) {

			/* waypoint is a loiter waypoint */
			float loiter_radius = pos_sp_curr.loiter_radius;
			uint8_t loiter_direction = pos_sp_curr.loiter_direction;

			if (pos_sp_curr.loiter_radius < 0.01f || pos_sp_curr.loiter_radius > -0.01f) {
				loiter_radius = _parameters.loiter_radius;
				loiter_direction = (loiter_radius > 0) ? 1 : -1;

			}

			_l1_control.navigate_loiter(curr_wp, curr_pos, loiter_radius, loiter_direction, nav_speed_2d);

			_att_sp.roll_body = _l1_control.get_roll_setpoint();
			_att_sp.yaw_body = _l1_control.nav_bearing();

			float alt_sp = pos_sp_curr.alt;

			if (pos_sp_next.type == position_setpoint_s::SETPOINT_TYPE_LAND && pos_sp_next.valid
			    && _l1_control.circle_mode() && _parameters.land_early_config_change == 1) {
				// We're in a loiter directly before a landing WP. Enable our landing configuration (flaps,
				// landing airspeed and potentially tighter throttle control) already such that we don't
				// have to do this switch (which can cause significant altitude errors) close to the ground.
				_tecs.set_time_const_throt(_parameters.land_throtTC_scale * _parameters.time_const_throt);
				mission_airspeed = _parameters.land_airspeed_scale * _parameters.airspeed_min;
				_att_sp.apply_flaps = true;
			}

			if (in_takeoff_situation()) {
				alt_sp = max(alt_sp, _takeoff_ground_alt + _parameters.climbout_diff);
				_att_sp.roll_body = constrain(_att_sp.roll_body, radians(-5.0f), radians(5.0f));
			}

			if (_land_abort) {
				if (pos_sp_curr.alt - _global_pos.alt  < _parameters.climbout_diff) {
					// aborted landing complete, normal loiter over landing point
					abort_landing(false);

				} else {
					// continue straight until vehicle has sufficient altitude
					_att_sp.roll_body = 0.0f;
				}

				_tecs.set_time_const_throt(_parameters.land_throtTC_scale * _parameters.time_const_throt);
			}

			tecs_update_pitch_throttle(alt_sp,
						   calculate_target_airspeed(mission_airspeed, ground_speed),
						   radians(_parameters.pitch_limit_min) - _parameters.pitchsp_offset_rad,
						   radians(_parameters.pitch_limit_max) - _parameters.pitchsp_offset_rad,
						   _parameters.throttle_min,
						   _parameters.throttle_max,
						   _parameters.throttle_cruise,
						   false,
						   radians(_parameters.pitch_limit_min));

		} else if (pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
			control_landing(curr_pos, ground_speed, pos_sp_prev, pos_sp_curr);

		} else if (pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF) {
			control_takeoff(curr_pos, ground_speed, pos_sp_prev, pos_sp_curr);
		}

		/* reset landing state */
		if (pos_sp_curr.type != position_setpoint_s::SETPOINT_TYPE_LAND) {
			reset_landing_state();
		}

		/* reset takeoff/launch state */
		if (pos_sp_curr.type != position_setpoint_s::SETPOINT_TYPE_TAKEOFF) {
			reset_takeoff_state();
		}

		if (was_circle_mode && !_l1_control.circle_mode()) {
			/* just kicked out of loiter, reset roll integrals */
			_att_sp.roll_reset_integral = true;
		}

	} else if (_control_mode.flag_control_velocity_enabled &&
		   _control_mode.flag_control_altitude_enabled) {
		/* POSITION CONTROL: pitch stick moves altitude setpoint, throttle stick sets airspeed,
		   heading is set to a distant waypoint */

		if (_control_mode_current != FW_POSCTRL_MODE_POSITION) {
			/* Need to init because last loop iteration was in a different mode */
			_hold_alt = _global_pos.alt;
			_hdg_hold_yaw = _yaw;
			_hdg_hold_enabled = false; // this makes sure the waypoints are reset below
			_yaw_lock_engaged = false;

			/* reset setpoints from other modes (auto) otherwise we won't
			 * level out without new manual input */
			_att_sp.roll_body = _manual.y * _parameters.man_roll_max_rad;
			_att_sp.yaw_body = 0;
		}

		_control_mode_current = FW_POSCTRL_MODE_POSITION;

		float altctrl_airspeed = get_demanded_airspeed();

		/* update desired altitude based on user pitch stick input */
		bool climbout_requested = update_desired_altitude(dt);

		// if we assume that user is taking off then help by demanding altitude setpoint well above ground
		// and set limit to pitch angle to prevent steering into ground
		// this will only affect planes and not VTOL
		float pitch_limit_min = _parameters.pitch_limit_min;
		do_takeoff_help(&_hold_alt, &pitch_limit_min);

		/* throttle limiting */
		throttle_max = _parameters.throttle_max;

		if (_vehicle_land_detected.landed && (fabsf(_manual.z) < THROTTLE_THRESH)) {
			throttle_max = 0.0f;
		}

		tecs_update_pitch_throttle(_hold_alt,
					   altctrl_airspeed,
					   radians(_parameters.pitch_limit_min),
					   radians(_parameters.pitch_limit_max),
					   _parameters.throttle_min,
					   throttle_max,
					   _parameters.throttle_cruise,
					   climbout_requested,
					   climbout_requested ? radians(10.0f) : pitch_limit_min,
					   tecs_status_s::TECS_MODE_NORMAL);

		/* heading control */
		if (fabsf(_manual.y) < HDG_HOLD_MAN_INPUT_THRESH &&
		    fabsf(_manual.r) < HDG_HOLD_MAN_INPUT_THRESH) {

			/* heading / roll is zero, lock onto current heading */
			if (fabsf(_vehicle_rates_sub.get().xyz[2]) < HDG_HOLD_YAWRATE_THRESH && !_yaw_lock_engaged) {
				// little yaw movement, lock to current heading
				_yaw_lock_engaged = true;

			}

			/* user tries to do a takeoff in heading hold mode, reset the yaw setpoint on every iteration
			  to make sure the plane does not start rolling
			*/
			if (in_takeoff_situation()) {
				_hdg_hold_enabled = false;
				_yaw_lock_engaged = true;
			}

			if (_yaw_lock_engaged) {

				/* just switched back from non heading-hold to heading hold */
				if (!_hdg_hold_enabled) {
					_hdg_hold_enabled = true;
					_hdg_hold_yaw = _yaw;

					get_waypoint_heading_distance(_hdg_hold_yaw, _hdg_hold_prev_wp, _hdg_hold_curr_wp, true);
				}

				/* we have a valid heading hold position, are we too close? */
				float dist = get_distance_to_next_waypoint(_global_pos.lat, _global_pos.lon, _hdg_hold_curr_wp.lat,
						_hdg_hold_curr_wp.lon);

				if (dist < HDG_HOLD_REACHED_DIST) {
					get_waypoint_heading_distance(_hdg_hold_yaw, _hdg_hold_prev_wp, _hdg_hold_curr_wp, false);
				}

				Vector2f prev_wp{(float)_hdg_hold_prev_wp.lat, (float)_hdg_hold_prev_wp.lon};
				Vector2f curr_wp{(float)_hdg_hold_curr_wp.lat, (float)_hdg_hold_curr_wp.lon};

				/* populate l1 control setpoint */
				_l1_control.navigate_waypoints(prev_wp, curr_wp, curr_pos, ground_speed);

				_att_sp.roll_body = _l1_control.get_roll_setpoint();
				_att_sp.yaw_body = _l1_control.nav_bearing();

				if (in_takeoff_situation()) {
					/* limit roll motion to ensure enough lift */
					_att_sp.roll_body = constrain(_att_sp.roll_body, radians(-15.0f), radians(15.0f));
				}
			}
		}

		if (!_yaw_lock_engaged || fabsf(_manual.y) >= HDG_HOLD_MAN_INPUT_THRESH ||
		    fabsf(_manual.r) >= HDG_HOLD_MAN_INPUT_THRESH) {

			_hdg_hold_enabled = false;
			_yaw_lock_engaged = false;
			_att_sp.roll_body = _manual.y * _parameters.man_roll_max_rad;
			_att_sp.yaw_body = 0;
		}

	} else if (_control_mode.flag_control_altitude_enabled) {
		/* ALTITUDE CONTROL: pitch stick moves altitude setpoint, throttle stick sets airspeed */

		if (_control_mode_current != FW_POSCTRL_MODE_POSITION && _control_mode_current != FW_POSCTRL_MODE_ALTITUDE) {
			/* Need to init because last loop iteration was in a different mode */
			_hold_alt = _global_pos.alt;
		}

		_control_mode_current = FW_POSCTRL_MODE_ALTITUDE;

		/* Get demanded airspeed */
		float altctrl_airspeed = get_demanded_airspeed();

		/* update desired altitude based on user pitch stick input */
		bool climbout_requested = update_desired_altitude(dt);

		// if we assume that user is taking off then help by demanding altitude setpoint well above ground
		// and set limit to pitch angle to prevent steering into ground
		// this will only affect planes and not VTOL
		float pitch_limit_min = _parameters.pitch_limit_min;
		do_takeoff_help(&_hold_alt, &pitch_limit_min);

		/* throttle limiting */
		throttle_max = _parameters.throttle_max;

		if (_vehicle_land_detected.landed && (fabsf(_manual.z) < THROTTLE_THRESH)) {
			throttle_max = 0.0f;
		}

		tecs_update_pitch_throttle(_hold_alt,
					   altctrl_airspeed,
					   radians(_parameters.pitch_limit_min),
					   radians(_parameters.pitch_limit_max),
					   _parameters.throttle_min,
					   throttle_max,
					   _parameters.throttle_cruise,
					   climbout_requested,
					   climbout_requested ? radians(10.0f) : pitch_limit_min,
					   tecs_status_s::TECS_MODE_NORMAL);

		_att_sp.roll_body = _manual.y * _parameters.man_roll_max_rad;
		_att_sp.yaw_body = 0;

	} else {
		_control_mode_current = FW_POSCTRL_MODE_OTHER;

		/* do not publish the setpoint */
		setpoint = false;

		// reset hold altitude
		_hold_alt = _global_pos.alt;

		/* reset landing and takeoff state */
		if (!_last_manual) {
			reset_landing_state();
			reset_takeoff_state();
		}
	}

	/* Copy thrust output for publication */
	if (_control_mode_current == FW_POSCTRL_MODE_AUTO && // launchdetector only available in auto
	    pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF &&
	    _launch_detection_state != LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS &&
	    !_runway_takeoff.runwayTakeoffEnabled()) {

		/* making sure again that the correct thrust is used,
		 * without depending on library calls for safety reasons.
		   the pre-takeoff throttle and the idle throttle normally map to the same parameter. */
		_att_sp.thrust_body[0] = _parameters.throttle_idle;

	} else if (_control_mode_current == FW_POSCTRL_MODE_AUTO &&
		   pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF &&
		   _runway_takeoff.runwayTakeoffEnabled()) {

		_att_sp.thrust_body[0] = _runway_takeoff.getThrottle(min(get_tecs_thrust(), throttle_max));

	} else if (_control_mode_current == FW_POSCTRL_MODE_AUTO &&
		   pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_IDLE) {

		_att_sp.thrust_body[0] = 0.0f;

	} else if (_control_mode_current == FW_POSCTRL_MODE_OTHER) {
		_att_sp.thrust_body[0] = min(_att_sp.thrust_body[0], _parameters.throttle_max);

	} else {
		/* Copy thrust and pitch values from tecs */
		if (_vehicle_land_detected.landed) {
			// when we are landed state we want the motor to spin at idle speed
			_att_sp.thrust_body[0] = min(_parameters.throttle_idle, throttle_max);

		} else {
			_att_sp.thrust_body[0] = min(get_tecs_thrust(), throttle_max);
		}
	}

	// decide when to use pitch setpoint from TECS because in some cases pitch
	// setpoint is generated by other means
	bool use_tecs_pitch = true;

	// auto runway takeoff
	use_tecs_pitch &= !(_control_mode_current == FW_POSCTRL_MODE_AUTO &&
			    pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF &&
			    (_launch_detection_state == LAUNCHDETECTION_RES_NONE || _runway_takeoff.runwayTakeoffEnabled()));

	// flaring during landing
	use_tecs_pitch &= !(pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_LAND && _land_noreturn_vertical);

	// manual attitude control
	use_tecs_pitch &= !(_control_mode_current == FW_POSCTRL_MODE_OTHER);

	if (use_tecs_pitch) {
		_att_sp.pitch_body = get_tecs_pitch();
	}

	if (_control_mode.flag_control_position_enabled) {
		_last_manual = false;

	} else {
		_last_manual = true;
	}

	return setpoint;
}

void
FixedwingPositionControl::control_takeoff(const Vector2f &curr_pos, const Vector2f &ground_speed,
		const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr)
{
	/* current waypoint (the one currently heading for) */
	Vector2f curr_wp((float)pos_sp_curr.lat, (float)pos_sp_curr.lon);
	Vector2f prev_wp{0.0f, 0.0f}; /* previous waypoint */

	if (pos_sp_prev.valid) {
		prev_wp(0) = (float)pos_sp_prev.lat;
		prev_wp(1) = (float)pos_sp_prev.lon;

	} else {
		/*
		 * No valid previous waypoint, go for the current wp.
		 * This is automatically handled by the L1 library.
		 */
		prev_wp(0) = (float)pos_sp_curr.lat;
		prev_wp(1) = (float)pos_sp_curr.lon;
	}

	// apply flaps for takeoff according to the corresponding scale factor set
	// via FW_FLAPS_TO_SCL
	_att_sp.apply_flaps = vehicle_attitude_setpoint_s::FLAPS_TAKEOFF;

	// continuously reset launch detection and runway takeoff until armed
	if (!_control_mode.flag_armed) {
		_launchDetector.reset();
		_launch_detection_state = LAUNCHDETECTION_RES_NONE;
		_launch_detection_notify = 0;
	}

	if (_runway_takeoff.runwayTakeoffEnabled()) {
		if (!_runway_takeoff.isInitialized()) {
			Eulerf euler(Quatf(_att.q));
			_runway_takeoff.init(euler.psi(), _global_pos.lat, _global_pos.lon);

			/* need this already before takeoff is detected
			 * doesn't matter if it gets reset when takeoff is detected eventually */
			_takeoff_ground_alt = _global_pos.alt;

			mavlink_log_info(&_mavlink_log_pub, "Takeoff on runway");
		}

		float terrain_alt = get_terrain_altitude_takeoff(_takeoff_ground_alt, _global_pos);

		// update runway takeoff helper
		_runway_takeoff.update(_airspeed, _global_pos.alt - terrain_alt,
				       _global_pos.lat, _global_pos.lon, &_mavlink_log_pub);

		/*
		 * Update navigation: _runway_takeoff returns the start WP according to mode and phase.
		 * If we use the navigator heading or not is decided later.
		 */
		_l1_control.navigate_waypoints(_runway_takeoff.getStartWP(), curr_wp, curr_pos, ground_speed);

		// update tecs
		const float takeoff_pitch_max_deg = _runway_takeoff.getMaxPitch(_parameters.pitch_limit_max);

		tecs_update_pitch_throttle(pos_sp_curr.alt,
					   calculate_target_airspeed(_runway_takeoff.getMinAirspeedScaling() * _parameters.airspeed_min, ground_speed),
					   radians(_parameters.pitch_limit_min),
					   radians(takeoff_pitch_max_deg),
					   _parameters.throttle_min,
					   _parameters.throttle_max, // XXX should we also set runway_takeoff_throttle here?
					   _parameters.throttle_cruise,
					   _runway_takeoff.climbout(),
					   radians(_runway_takeoff.getMinPitch(pos_sp_curr.pitch_min, 10.0f, _parameters.pitch_limit_min)),
					   tecs_status_s::TECS_MODE_TAKEOFF);

		// assign values
		_att_sp.roll_body = _runway_takeoff.getRoll(_l1_control.get_roll_setpoint());
		_att_sp.yaw_body = _runway_takeoff.getYaw(_l1_control.nav_bearing());
		_att_sp.fw_control_yaw = _runway_takeoff.controlYaw();
		_att_sp.pitch_body = _runway_takeoff.getPitch(get_tecs_pitch());

		// reset integrals except yaw (which also counts for the wheel controller)
		_att_sp.roll_reset_integral = _runway_takeoff.resetIntegrators();
		_att_sp.pitch_reset_integral = _runway_takeoff.resetIntegrators();

	} else {
		/* Perform launch detection */
		if (_launchDetector.launchDetectionEnabled() &&
		    _launch_detection_state != LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS) {

			if (_control_mode.flag_armed) {
				/* Perform launch detection */

				/* Inform user that launchdetection is running every 4s */
				if (hrt_elapsed_time(&_launch_detection_notify) > 4e6) {
					mavlink_log_critical(&_mavlink_log_pub, "Launch detection running");
					_launch_detection_notify = hrt_absolute_time();
				}

				/* Detect launch using body X (forward) acceleration */
				_launchDetector.update(_vehicle_acceleration_sub.get().xyz[0]);

				/* update our copy of the launch detection state */
				_launch_detection_state = _launchDetector.getLaunchDetected();
			}

		} else	{
			/* no takeoff detection --> fly */
			_launch_detection_state = LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS;
		}

		/* Set control values depending on the detection state */
		if (_launch_detection_state != LAUNCHDETECTION_RES_NONE) {
			/* Launch has been detected, hence we have to control the plane. */

			_l1_control.navigate_waypoints(prev_wp, curr_wp, curr_pos, ground_speed);
			_att_sp.roll_body = _l1_control.get_roll_setpoint();
			_att_sp.yaw_body = _l1_control.nav_bearing();

			/* Select throttle: only in LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS we want to use
			 * full throttle, otherwise we use idle throttle */
			float takeoff_throttle = _parameters.throttle_max;

			if (_launch_detection_state != LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS) {
				takeoff_throttle = _parameters.throttle_idle;
			}

			/* select maximum pitch: the launchdetector may impose another limit for the pitch
			 * depending on the state of the launch */
			const float takeoff_pitch_max_deg = _launchDetector.getPitchMax(_parameters.pitch_limit_max);
			const float altitude_error = pos_sp_curr.alt - _global_pos.alt;

			/* apply minimum pitch and limit roll if target altitude is not within climbout_diff meters */
			if (_parameters.climbout_diff > 0.0f && altitude_error > _parameters.climbout_diff) {
				/* enforce a minimum of 10 degrees pitch up on takeoff, or take parameter */
				tecs_update_pitch_throttle(pos_sp_curr.alt,
							   _parameters.airspeed_trim,
							   radians(_parameters.pitch_limit_min),
							   radians(takeoff_pitch_max_deg),
							   _parameters.throttle_min,
							   takeoff_throttle,
							   _parameters.throttle_cruise,
							   true,
							   max(radians(pos_sp_curr.pitch_min), radians(10.0f)),
							   tecs_status_s::TECS_MODE_TAKEOFF);

				/* limit roll motion to ensure enough lift */
				_att_sp.roll_body = constrain(_att_sp.roll_body, radians(-15.0f), radians(15.0f));

			} else {
				tecs_update_pitch_throttle(pos_sp_curr.alt,
							   calculate_target_airspeed(_parameters.airspeed_trim, ground_speed),
							   radians(_parameters.pitch_limit_min),
							   radians(_parameters.pitch_limit_max),
							   _parameters.throttle_min,
							   takeoff_throttle,
							   _parameters.throttle_cruise,
							   false,
							   radians(_parameters.pitch_limit_min));
			}

		} else {
			/* Tell the attitude controller to stop integrating while we are waiting
			 * for the launch */
			_att_sp.roll_reset_integral = true;
			_att_sp.pitch_reset_integral = true;
			_att_sp.yaw_reset_integral = true;

			/* Set default roll and pitch setpoints during detection phase */
			_att_sp.roll_body = 0.0f;
			_att_sp.pitch_body = max(radians(pos_sp_curr.pitch_min), radians(10.0f));
		}
	}
}

void
FixedwingPositionControl::control_landing(const Vector2f &curr_pos, const Vector2f &ground_speed,
		const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr)
{
	/* current waypoint (the one currently heading for) */
	Vector2f curr_wp((float)pos_sp_curr.lat, (float)pos_sp_curr.lon);
	Vector2f prev_wp{0.0f, 0.0f}; /* previous waypoint */

	if (pos_sp_prev.valid) {
		prev_wp(0) = (float)pos_sp_prev.lat;
		prev_wp(1) = (float)pos_sp_prev.lon;

	} else {
		/*
		 * No valid previous waypoint, go for the current wp.
		 * This is automatically handled by the L1 library.
		 */
		prev_wp(0) = (float)pos_sp_curr.lat;
		prev_wp(1) = (float)pos_sp_curr.lon;
	}

	// apply full flaps for landings. this flag will also trigger the use of flaperons
	// if they have been enabled using the corresponding parameter
	_att_sp.apply_flaps = vehicle_attitude_setpoint_s::FLAPS_LAND;

	// Enable tighter throttle control for landings
	_tecs.set_time_const_throt(_parameters.land_throtTC_scale * _parameters.time_const_throt);

	// save time at which we started landing and reset abort_landing
	if (_time_started_landing == 0) {
		reset_landing_state();
		_time_started_landing = hrt_absolute_time();
	}

	const float bearing_airplane_currwp = get_bearing_to_next_waypoint((double)curr_pos(0), (double)curr_pos(1),
					      (double)curr_wp(0), (double)curr_wp(1));

	float bearing_lastwp_currwp = bearing_airplane_currwp;

	if (pos_sp_prev.valid) {
		bearing_lastwp_currwp = get_bearing_to_next_waypoint((double)prev_wp(0), (double)prev_wp(1), (double)curr_wp(0),
					(double)curr_wp(1));
	}

	/* Horizontal landing control */
	/* switch to heading hold for the last meters, continue heading hold after */
	float wp_distance = get_distance_to_next_waypoint((double)curr_pos(0), (double)curr_pos(1), (double)curr_wp(0),
			    (double)curr_wp(1));

	/* calculate a waypoint distance value which is 0 when the aircraft is behind the waypoint */
	float wp_distance_save = wp_distance;

	if (fabsf(wrap_pi(bearing_airplane_currwp - bearing_lastwp_currwp)) >= radians(90.0f)) {
		wp_distance_save = 0.0f;
	}

	// create virtual waypoint which is on the desired flight path but
	// some distance behind landing waypoint. This will make sure that the plane
	// will always follow the desired flight path even if we get close or past
	// the landing waypoint
	if (pos_sp_prev.valid) {
		double lat = pos_sp_curr.lat;
		double lon = pos_sp_curr.lon;

		create_waypoint_from_line_and_dist(pos_sp_curr.lat, pos_sp_curr.lon,
						   pos_sp_prev.lat, pos_sp_prev.lon, -1000.0f, &lat, &lon);

		curr_wp(0) = (float)lat;
		curr_wp(1) = (float)lon;
	}

	// we want the plane to keep tracking the desired flight path until we start flaring
	// if we go into heading hold mode earlier then we risk to be pushed away from the runway by cross winds
	if ((_parameters.land_heading_hold_horizontal_distance > 0.0f) && !_land_noreturn_horizontal &&
	    ((wp_distance < _parameters.land_heading_hold_horizontal_distance) || _land_noreturn_vertical)) {

		if (pos_sp_prev.valid) {
			/* heading hold, along the line connecting this and the last waypoint */
			_target_bearing = bearing_lastwp_currwp;

		} else {
			_target_bearing = _yaw;
		}

		_land_noreturn_horizontal = true;
		mavlink_log_info(&_mavlink_log_pub, "Landing, heading hold");
	}

	if (_land_noreturn_horizontal) {
		// heading hold
		_l1_control.navigate_heading(_target_bearing, _yaw, ground_speed);

	} else {
		// normal navigation
		_l1_control.navigate_waypoints(prev_wp, curr_wp, curr_pos, ground_speed);
	}

	_att_sp.roll_body = _l1_control.get_roll_setpoint();
	_att_sp.yaw_body = _l1_control.nav_bearing();

	if (_land_noreturn_horizontal) {
		/* limit roll motion to prevent wings from touching the ground first */
		_att_sp.roll_body = constrain(_att_sp.roll_body, radians(-10.0f), radians(10.0f));
	}

	/* Vertical landing control */
	/* apply minimum pitch (flare) and limit roll if close to touch down, altitude error is negative (going down) */

	// default to no terrain estimation, just use landing waypoint altitude
	float terrain_alt = pos_sp_curr.alt;

	if (_parameters.land_use_terrain_estimate == 1) {
		if (_global_pos.terrain_alt_valid) {
			// all good, have valid terrain altitude
			terrain_alt = _global_pos.terrain_alt;
			_t_alt_prev_valid = terrain_alt;
			_time_last_t_alt = hrt_absolute_time();

		} else if (_time_last_t_alt == 0) {
			// we have started landing phase but don't have valid terrain
			// wait for some time, maybe we will soon get a valid estimate
			// until then just use the altitude of the landing waypoint
			if (hrt_elapsed_time(&_time_started_landing) < 10_s) {
				terrain_alt = pos_sp_curr.alt;

			} else {
				// still no valid terrain, abort landing
				terrain_alt = pos_sp_curr.alt;
				abort_landing(true);
			}

		} else if ((!_global_pos.terrain_alt_valid && hrt_elapsed_time(&_time_last_t_alt) < T_ALT_TIMEOUT)
			   || _land_noreturn_vertical) {
			// use previous terrain estimate for some time and hope to recover
			// if we are already flaring (land_noreturn_vertical) then just
			//  go with the old estimate
			terrain_alt = _t_alt_prev_valid;

		} else {
			// terrain alt was not valid for long time, abort landing
			terrain_alt = _t_alt_prev_valid;
			abort_landing(true);
		}
	}

	/* Check if we should start flaring with a vertical and a
	 * horizontal limit (with some tolerance)
	 * The horizontal limit is only applied when we are in front of the wp
	 */
	if ((_global_pos.alt < terrain_alt + _landingslope.flare_relative_alt()) ||
	    _land_noreturn_vertical) {  //checking for land_noreturn to avoid unwanted climb out

		/* land with minimal speed */

		/* force TECS to only control speed with pitch, altitude is only implicitly controlled now */
		// _tecs.set_speed_weight(2.0f);

		/* kill the throttle if param requests it */
		float throttle_max = _parameters.throttle_max;

		/* enable direct yaw control using rudder/wheel */
		if (_land_noreturn_horizontal) {
			_att_sp.yaw_body = _target_bearing;
			_att_sp.fw_control_yaw = true;
		}

		if (((_global_pos.alt < terrain_alt + _landingslope.motor_lim_relative_alt()) &&
		     (wp_distance_save < _landingslope.flare_length() + 5.0f)) || // Only kill throttle when close to WP
		    _land_motor_lim) {
			throttle_max = min(throttle_max, _parameters.throttle_land_max);

			if (!_land_motor_lim) {
				_land_motor_lim  = true;
				mavlink_log_info(&_mavlink_log_pub, "Landing, limiting throttle");
			}
		}

		float flare_curve_alt_rel = _landingslope.getFlareCurveRelativeAltitudeSave(wp_distance, bearing_lastwp_currwp,
					    bearing_airplane_currwp);

		/* avoid climbout */
		if ((_flare_curve_alt_rel_last < flare_curve_alt_rel && _land_noreturn_vertical) || _land_stayonground) {
			flare_curve_alt_rel = 0.0f; // stay on ground
			_land_stayonground = true;
		}

		const float airspeed_land = _parameters.land_airspeed_scale * _parameters.airspeed_min;
		const float throttle_land = _parameters.throttle_min + (_parameters.throttle_max - _parameters.throttle_min) * 0.1f;

		tecs_update_pitch_throttle(terrain_alt + flare_curve_alt_rel,
					   calculate_target_airspeed(airspeed_land, ground_speed),
					   radians(_parameters.land_flare_pitch_min_deg),
					   radians(_parameters.land_flare_pitch_max_deg),
					   0.0f,
					   throttle_max,
					   throttle_land,
					   false,
					   _land_motor_lim ? radians(_parameters.land_flare_pitch_min_deg) : radians(_parameters.pitch_limit_min),
					   _land_motor_lim ? tecs_status_s::TECS_MODE_LAND_THROTTLELIM : tecs_status_s::TECS_MODE_LAND);

		if (!_land_noreturn_vertical) {
			// just started with the flaring phase
			_flare_pitch_sp = 0.0f;
			_flare_height = _global_pos.alt - terrain_alt;
			mavlink_log_info(&_mavlink_log_pub, "Landing, flaring");
			_land_noreturn_vertical = true;

		} else {
			if (_global_pos.vel_d > 0.1f) {
				_flare_pitch_sp = radians(_parameters.land_flare_pitch_min_deg) *
						  constrain((_flare_height - (_global_pos.alt - terrain_alt)) / _flare_height, 0.0f, 1.0f);
			}

			// otherwise continue using previous _flare_pitch_sp
		}

		_att_sp.pitch_body = _flare_pitch_sp;
		_flare_curve_alt_rel_last = flare_curve_alt_rel;

	} else {

		/* intersect glide slope:
		 * minimize speed to approach speed
		 * if current position is higher than the slope follow the glide slope (sink to the
		 * glide slope)
		 * also if the system captures the slope it should stay
		 * on the slope (bool land_onslope)
		 * if current position is below the slope continue at previous wp altitude
		 * until the intersection with slope
		 * */

		float altitude_desired = terrain_alt;

		const float landing_slope_alt_rel_desired = _landingslope.getLandingSlopeRelativeAltitudeSave(wp_distance,
				bearing_lastwp_currwp, bearing_airplane_currwp);

		if (_global_pos.alt > terrain_alt + landing_slope_alt_rel_desired || _land_onslope) {
			/* stay on slope */
			altitude_desired = terrain_alt + landing_slope_alt_rel_desired;

			if (!_land_onslope) {
				mavlink_log_info(&_mavlink_log_pub, "Landing, on slope");
				_land_onslope = true;
			}

		} else {
			/* continue horizontally */
			if (pos_sp_prev.valid) {
				altitude_desired = pos_sp_prev.alt;

			} else {
				altitude_desired = _global_pos.alt;
			}
		}

		const float airspeed_approach = _parameters.land_airspeed_scale * _parameters.airspeed_min;

		tecs_update_pitch_throttle(altitude_desired,
					   calculate_target_airspeed(airspeed_approach, ground_speed),
					   radians(_parameters.pitch_limit_min),
					   radians(_parameters.pitch_limit_max),
					   _parameters.throttle_min,
					   _parameters.throttle_max,
					   _parameters.throttle_cruise,
					   false,
					   radians(_parameters.pitch_limit_min));
	}
}

float
FixedwingPositionControl::get_tecs_pitch()
{
	if (_is_tecs_running) {
		return _tecs.get_pitch_setpoint();
	}

	// return 0 to prevent stale tecs state when it's not running
	return 0.0f;
}

float
FixedwingPositionControl::get_tecs_thrust()
{
	if (_is_tecs_running) {
		return _tecs.get_throttle_setpoint();
	}

	// return 0 to prevent stale tecs state when it's not running
	return 0.0f;
}

void
FixedwingPositionControl::handle_command()
{
	if (_vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_GO_AROUND) {
		// only abort landing before point of no return (horizontal and vertical)
		if (_control_mode.flag_control_auto_enabled &&
		    _pos_sp_triplet.current.valid &&
		    _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {

			abort_landing(true);
		}
	}
}

void
FixedwingPositionControl::Run()
{
	if (should_exit()) {
		_global_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	/* only run controller if position changed */
	if (_global_pos_sub.update(&_global_pos)) {

		// check for parameter updates
		if (_parameter_update_sub.updated()) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			parameters_update();
		}

		_local_pos_sub.update(&_local_pos);

		// handle estimator reset events. we only adjust setpoins for manual modes
		if (_control_mode.flag_control_manual_enabled) {
			if (_control_mode.flag_control_altitude_enabled && _global_pos.alt_reset_counter != _alt_reset_counter) {
				_hold_alt += _global_pos.delta_alt;
				// make TECS accept step in altitude and demanded altitude
				_tecs.handle_alt_step(_global_pos.delta_alt, _global_pos.alt);
			}

			// adjust navigation waypoints in position control mode
			if (_control_mode.flag_control_altitude_enabled && _control_mode.flag_control_velocity_enabled
			    && _global_pos.lat_lon_reset_counter != _pos_reset_counter) {

				// reset heading hold flag, which will re-initialise position control
				_hdg_hold_enabled = false;
			}
		}

		// update the reset counters in any case
		_alt_reset_counter = _global_pos.alt_reset_counter;
		_pos_reset_counter = _global_pos.lat_lon_reset_counter;

		airspeed_poll();
		_manual_control_sub.update(&_manual);
		_pos_sp_triplet_sub.update(&_pos_sp_triplet);
		vehicle_attitude_poll();
		vehicle_command_poll();
		vehicle_control_mode_poll();
		_vehicle_land_detected_sub.update(&_vehicle_land_detected);
		_vehicle_status_sub.update(&_vehicle_status);
		_vehicle_acceleration_sub.update();
		_vehicle_rates_sub.update();

		Vector2f curr_pos((float)_global_pos.lat, (float)_global_pos.lon);
		Vector2f ground_speed(_global_pos.vel_n, _global_pos.vel_e);

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

		/*
		 * Attempt to control position, on success (= sensors present and not in manual mode),
		 * publish setpoint.
		 */
		if (control_position(curr_pos, ground_speed, _pos_sp_triplet.previous, _pos_sp_triplet.current, _pos_sp_triplet.next)) {
			_att_sp.timestamp = hrt_absolute_time();

			// add attitude setpoint offsets
			_att_sp.roll_body += _parameters.rollsp_offset_rad;
			_att_sp.pitch_body += _parameters.pitchsp_offset_rad;

			if (_control_mode.flag_control_manual_enabled) {
				_att_sp.roll_body = constrain(_att_sp.roll_body, -_parameters.man_roll_max_rad, _parameters.man_roll_max_rad);
				_att_sp.pitch_body = constrain(_att_sp.pitch_body, -_parameters.man_pitch_max_rad, _parameters.man_pitch_max_rad);
			}

			Quatf q(Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body));
			q.copyTo(_att_sp.q_d);
			_att_sp.q_d_valid = true;

			if (_control_mode.flag_control_offboard_enabled ||
			    _control_mode.flag_control_position_enabled ||
			    _control_mode.flag_control_velocity_enabled ||
			    _control_mode.flag_control_acceleration_enabled ||
			    _control_mode.flag_control_altitude_enabled) {

				_attitude_sp_pub.publish(_att_sp);

				// only publish status in full FW mode
				if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING
				    && !_vehicle_status.in_transition_mode) {
					status_publish();
				}
			}
		}

		perf_end(_loop_perf);
	}
}

void
FixedwingPositionControl::reset_takeoff_state(bool force)
{
	// only reset takeoff if !armed or just landed
	if (!_control_mode.flag_armed || (_was_in_air && _vehicle_land_detected.landed) || force) {

		_runway_takeoff.reset();

		_launchDetector.reset();
		_launch_detection_state = LAUNCHDETECTION_RES_NONE;
		_launch_detection_notify = 0;

	} else {
		_launch_detection_state = LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS;
	}
}

void
FixedwingPositionControl::reset_landing_state()
{
	_time_started_landing = 0;

	// reset terrain estimation relevant values
	_time_last_t_alt = 0;

	_land_noreturn_horizontal = false;
	_land_noreturn_vertical = false;
	_land_stayonground = false;
	_land_motor_lim = false;
	_land_onslope = false;

	// reset abort land, unless loitering after an abort
	if (_land_abort && (_pos_sp_triplet.current.type != position_setpoint_s::SETPOINT_TYPE_LOITER)) {

		abort_landing(false);
	}
}

void
FixedwingPositionControl::tecs_update_pitch_throttle(float alt_sp, float airspeed_sp,
		float pitch_min_rad, float pitch_max_rad,
		float throttle_min, float throttle_max, float throttle_cruise,
		bool climbout_mode, float climbout_pitch_min_rad,
		uint8_t mode)
{
	float dt = 0.01f; // prevent division with 0

	if (_last_tecs_update > 0) {
		dt = hrt_elapsed_time(&_last_tecs_update) * 1e-6;
	}

	_last_tecs_update = hrt_absolute_time();

	// do not run TECS if we are not in air
	bool run_tecs = !_vehicle_land_detected.landed;

	// do not run TECS if vehicle is a VTOL and we are in rotary wing mode or in transition
	// (it should also not run during VTOL blending because airspeed is too low still)
	if (_vehicle_status.is_vtol) {
		if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING || _vehicle_status.in_transition_mode) {
			run_tecs = false;
		}

		if (_vehicle_status.in_transition_mode) {
			// we're in transition
			_was_in_transition = true;

			// set this to transition airspeed to init tecs correctly
			if (_parameters.airspeed_disabled) {
				// some vtols fly without airspeed sensor
				_asp_after_transition = _parameters.airspeed_trans;

			} else {
				_asp_after_transition = _airspeed;
			}

			_asp_after_transition = constrain(_asp_after_transition, _parameters.airspeed_min, _parameters.airspeed_max);

		} else if (_was_in_transition) {
			// after transition we ramp up desired airspeed from the speed we had coming out of the transition
			_asp_after_transition += dt * 2; // increase 2m/s

			if (_asp_after_transition < airspeed_sp && _airspeed < airspeed_sp) {
				airspeed_sp = max(_asp_after_transition, _airspeed);

			} else {
				_was_in_transition = false;
				_asp_after_transition = 0;
			}
		}
	}

	_is_tecs_running = run_tecs;

	if (!run_tecs) {
		// next time we run TECS we should reinitialize states
		_reinitialize_tecs = true;
		return;
	}

	if (_reinitialize_tecs) {
		_tecs.reset_state();
		_reinitialize_tecs = false;
	}

	if (_vehicle_status.engine_failure) {
		/* Force the slow downwards spiral */
		pitch_min_rad = M_DEG_TO_RAD_F * -1.0f;
		pitch_max_rad = M_DEG_TO_RAD_F * 5.0f;
	}

	/* No underspeed protection in landing mode */
	_tecs.set_detect_underspeed_enabled(!(mode == tecs_status_s::TECS_MODE_LAND
					      || mode == tecs_status_s::TECS_MODE_LAND_THROTTLELIM));

	/* Using tecs library */
	float pitch_for_tecs = _pitch - _parameters.pitchsp_offset_rad;

	/* filter speed and altitude for controller */
	Vector3f accel_body(_vehicle_acceleration_sub.get().xyz);

	// tailsitters use the multicopter frame as reference, in fixed wing
	// we need to use the fixed wing frame
	if (_vtol_tailsitter) {
		float tmp = accel_body(0);
		accel_body(0) = -accel_body(2);
		accel_body(2) = tmp;
	}

	/* tell TECS to update its state, but let it know when it cannot actually control the plane */
	bool in_air_alt_control = (!_vehicle_land_detected.landed &&
				   (_control_mode.flag_control_auto_enabled ||
				    _control_mode.flag_control_offboard_enabled ||
				    _control_mode.flag_control_velocity_enabled ||
				    _control_mode.flag_control_altitude_enabled));

	/* update TECS vehicle state estimates */
	_tecs.update_vehicle_state_estimates(_airspeed, _R_nb,
					     accel_body, (_global_pos.timestamp > 0), in_air_alt_control,
					     _global_pos.alt, _local_pos.v_z_valid, _local_pos.vz, _local_pos.az);

	/* scale throttle cruise by baro pressure */
	if (_parameters.throttle_alt_scale > FLT_EPSILON) {
		sensor_baro_s baro{};

		if (_sensor_baro_sub.update(&baro)) {
			if (PX4_ISFINITE(baro.pressure) && PX4_ISFINITE(_parameters.throttle_alt_scale)) {
				// scale throttle as a function of sqrt(p0/p) (~ EAS -> TAS at low speeds and altitudes ignoring temperature)
				const float eas2tas = sqrtf(CONSTANTS_STD_PRESSURE_MBAR / baro.pressure);
				const float scale = constrain((eas2tas - 1.0f) * _parameters.throttle_alt_scale + 1.0f, 1.0f, 2.0f);

				throttle_max = constrain(throttle_max * scale, throttle_min, 1.0f);
				throttle_cruise = constrain(throttle_cruise * scale, throttle_min + 0.01f, throttle_max - 0.01f);
			}
		}
	}

	_tecs.update_pitch_throttle(_R_nb, pitch_for_tecs,
				    _global_pos.alt, alt_sp,
				    airspeed_sp, _airspeed, _eas2tas,
				    climbout_mode, climbout_pitch_min_rad,
				    throttle_min, throttle_max, throttle_cruise,
				    pitch_min_rad, pitch_max_rad);

	tecs_status_publish();
}

int FixedwingPositionControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	FixedwingPositionControl *instance = new FixedwingPositionControl(vtol);

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

int FixedwingPositionControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FixedwingPositionControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
fw_pos_control_l1 is the fixed wing position controller.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fw_pos_control_l1", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fw_pos_control_l1_main(int argc, char *argv[])
{
	return FixedwingPositionControl::main(argc, argv);
}
