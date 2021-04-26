/****************************************************************************
 *
 *   Copyright (c) 2015-2021 PX4 Development Team. All rights reserved.
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
 * @file camera_trigger.cpp
 *
 * External camera-IMU synchronisation and triggering, and support for
 * camera manipulation using PWM signals over FMU auxillary pins.
 *
 * @author Mohammed Kabir <kabir@uasys.io>
 * @author Kelly Steich <kelly.steich@wingtra.com>
 * @author Andreas Bircher <andreas@wingtra.com>
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdbool.h>
#include <poll.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <systemlib/err.h>
#include <parameters/param.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/camera_trigger.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_local_position.h>

#include <drivers/drv_hrt.h>

#include "interfaces/src/camera_interface.h"
#include "interfaces/src/gpio.h"
#include "interfaces/src/pwm.h"
#include "interfaces/src/seagull_map2.h"

typedef enum : int32_t {
	CAMERA_INTERFACE_MODE_NONE = 0,
	CAMERA_INTERFACE_MODE_GPIO,
	CAMERA_INTERFACE_MODE_SEAGULL_MAP2_PWM,
	CAMERA_INTERFACE_MODE_MAVLINK,
	CAMERA_INTERFACE_MODE_GENERIC_PWM
} camera_interface_mode_t;

typedef enum : int32_t {
	TRIGGER_MODE_NONE = 0,
	TRIGGER_MODE_INTERVAL_ON_CMD,
	TRIGGER_MODE_INTERVAL_ALWAYS_ON,
	TRIGGER_MODE_DISTANCE_ALWAYS_ON,
	TRIGGER_MODE_DISTANCE_ON_CMD
} trigger_mode_t;

#define commandParamToInt(n) static_cast<int>(n >= 0 ? n + 0.5f : n - 0.5f)

class CameraTrigger : public px4::ScheduledWorkItem
{
public:
	/**
	 * Constructor
	 */
	CameraTrigger();

	/**
	 * Destructor, also kills task.
	 */
	~CameraTrigger() override;

	/**
	 * Run intervalometer update
	 */
	void		update_intervalometer();

	/**
	 * Run distance-based trigger update
	 */
	void		update_distance();

	/**
	 * Trigger the camera just once
	 */
	void		shoot_once();

	/**
	 * Toggle keep camera alive functionality
	 */
	void		enable_keep_alive(bool on);

	/**
	 * Toggle camera power (on/off)
	 */
	void        	toggle_power();

	/**
	 * Start the task.
	 */
	bool		start();

	/**
	 * Stop the task.
	 */
	void		stop();

	/**
	 * Display status.
	 */
	void		status();

	/**
	 * Trigger one image
	 */
	void		test();

	/**
	 * adjusts pose between triggers in CAMPOS mode
	 */
	void		adjust_roll();

private:

	struct hrt_call _engagecall {};
	struct hrt_call _disengagecall {};
	struct hrt_call _engage_turn_on_off_call {};
	struct hrt_call _disengage_turn_on_off_call {};
	struct hrt_call _keepalivecall_up {};
	struct hrt_call _keepalivecall_down {};

	float			_activation_time{0.5f};
	float			_interval{100.f};
	float			_min_interval{1.f};
	float 			_distance{25.f};
	uint32_t 		_trigger_seq{0};
	hrt_abstime		_last_trigger_timestamp{0};
	bool			_trigger_enabled{false};
	bool			_trigger_paused{false};
	bool			_one_shot{false};
	bool			_test_shot{false};
	bool 			_turning_on{false};
	matrix::Vector2f	_last_shoot_position{0.f, 0.f};
	bool			_valid_position{false};

	//Camera Auto Mount Pivoting Oblique Survey (CAMPOS)
	uint32_t		_CAMPOS_num_poses{0};
	uint32_t		_CAMPOS_pose_counter{0};
	float			_CAMPOS_roll_angle{0.f};
	float			_CAMPOS_angle_interval{0.f};
	float			_CAMPOS_pitch_angle{-90.f};
	bool			_CAMPOS_updated_roll_angle{false};
	uint32_t		_target_system{0};
	uint32_t		_target_component{0};

	uORB::Subscription	_command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription	_lpos_sub{ORB_ID(vehicle_local_position)};

	orb_advert_t		_trigger_pub{nullptr};

	uORB::Publication<vehicle_command_ack_s>	_cmd_ack_pub{ORB_ID(vehicle_command_ack)};

	param_t			_p_mode;
	param_t			_p_activation_time;
	param_t			_p_interval;
	param_t			_p_min_interval;
	param_t			_p_distance;
	param_t			_p_interface;
	param_t 		_p_cam_cap_fback;

	trigger_mode_t		_trigger_mode{TRIGGER_MODE_NONE};
	int32_t _cam_cap_fback{0};

	camera_interface_mode_t	_camera_interface_mode{CAMERA_INTERFACE_MODE_GPIO};
	CameraInterface		*_camera_interface{nullptr};  ///< instance of camera interface

	/**
	 * Vehicle command handler
	 */
	void		Run() override;

	/**
	 * Fires trigger
	 */
	static void	engage(void *arg);

	/**
	 * Resets trigger
	 */
	static void	disengage(void *arg);

	/**
	 * Fires on/off
	 */
	static void engange_turn_on_off(void *arg);

	/**
	 * Resets  on/off
	 */
	static void disengage_turn_on_off(void *arg);

	/**
	 * Enables keep alive signal
	 */
	static void	keep_alive_up(void *arg);
	/**
	 * Disables keep alive signal
	 */
	static void	keep_alive_down(void *arg);

};

namespace camera_trigger
{
CameraTrigger	*g_camera_trigger;
}

CameraTrigger::CameraTrigger() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	// Initiate camera interface based on camera_interface_mode
	if (_camera_interface != nullptr) {
		delete (_camera_interface);
		// set to zero to ensure parser is not used while not instantiated
		_camera_interface = nullptr;
	}

	// Parameters
	_p_interval = param_find("TRIG_INTERVAL");
	_p_min_interval = param_find("TRIG_MIN_INTERVA");
	_p_distance = param_find("TRIG_DISTANCE");
	_p_activation_time = param_find("TRIG_ACT_TIME");
	_p_mode = param_find("TRIG_MODE");
	_p_interface = param_find("TRIG_INTERFACE");
	_p_cam_cap_fback = param_find("CAM_CAP_FBACK");

	param_get(_p_activation_time, &_activation_time);
	param_get(_p_interval, &_interval);
	param_get(_p_min_interval, &_min_interval);
	param_get(_p_distance, &_distance);
	param_get(_p_mode, (int32_t *)&_trigger_mode);
	param_get(_p_interface, (int32_t *)&_camera_interface_mode);
	param_get(_p_cam_cap_fback, (int32_t *)&_cam_cap_fback);

	switch (_camera_interface_mode) {
#ifdef __PX4_NUTTX

	case CAMERA_INTERFACE_MODE_GPIO:
		_camera_interface = new CameraInterfaceGPIO();
		break;

	case CAMERA_INTERFACE_MODE_GENERIC_PWM:
		_camera_interface = new CameraInterfacePWM();
		break;

	case CAMERA_INTERFACE_MODE_SEAGULL_MAP2_PWM:
		_camera_interface = new CameraInterfaceSeagull();
		break;

#endif

	case CAMERA_INTERFACE_MODE_MAVLINK:
		// start an interface that does nothing. Instead mavlink will listen to the camera_trigger uORB message
		_camera_interface = new CameraInterface();
		break;

	default:
		PX4_ERR("unknown camera interface mode: %i", (int)_camera_interface_mode);
		break;
	}

	// Enforce a lower bound on the activation interval in PWM modes to not miss
	// engage calls in-between 50Hz PWM pulses. (see PX4 PR #6973)
	if ((_activation_time < 40.0f) &&
	    (_camera_interface_mode == CAMERA_INTERFACE_MODE_GENERIC_PWM ||
	     _camera_interface_mode == CAMERA_INTERFACE_MODE_SEAGULL_MAP2_PWM)) {

		_activation_time = 40.0f;
		PX4_WARN("Trigger interval too low for PWM interface, setting to 40 ms");
		param_set_no_notification(_p_activation_time, &(_activation_time));
	}

	// Advertise critical publishers here, because we cannot advertise in interrupt context
	camera_trigger_s trigger{};

	if (!_cam_cap_fback) {
		_trigger_pub = orb_advertise(ORB_ID(camera_trigger), &trigger);

	} else {
		_trigger_pub = orb_advertise(ORB_ID(camera_trigger_secondary), &trigger);
	}
}

CameraTrigger::~CameraTrigger()
{
	if (_camera_interface != nullptr) {
		delete (_camera_interface);
	}

	camera_trigger::g_camera_trigger = nullptr;
}

void
CameraTrigger::update_intervalometer()
{
	// the actual intervalometer runs in interrupt context, so we only need to call
	// control_intervalometer once on enabling/disabling trigger to schedule the calls.

	if (_trigger_enabled && !_trigger_paused) {
		PX4_DEBUG("update intervalometer, trigger enabled: %d, trigger paused: %d", _trigger_enabled, _trigger_paused);

		// schedule trigger on and off calls
		hrt_call_every(&_engagecall, 0, (_interval * 1000), &CameraTrigger::engage, this);

		// schedule trigger on and off calls
		hrt_call_every(&_disengagecall, 0 + (_activation_time * 1000), (_interval * 1000), &CameraTrigger::disengage, this);

	}
}

void
CameraTrigger::update_distance()
{
	if (_turning_on || !_trigger_enabled || _trigger_paused) {
		return;
	}

	vehicle_local_position_s local{};
	_lpos_sub.copy(&local);

	if (local.xy_valid) {
		// Initialize position if not done yet
		matrix::Vector2f current_position(local.x, local.y);

		if (!_valid_position) {
			// First time valid position, take first shot
			_last_shoot_position = current_position;
			_valid_position = local.xy_valid;

			if (!_one_shot) {
				shoot_once();
			}
		}

		hrt_abstime now = hrt_absolute_time();

		if (!_CAMPOS_updated_roll_angle && _CAMPOS_num_poses > 0 && (now - _last_trigger_timestamp > _min_interval * 1000)) {
			adjust_roll();
			_CAMPOS_updated_roll_angle = true;
		}

		// Check that distance threshold is exceeded
		if (matrix::Vector2f(_last_shoot_position - current_position).length() >= _distance) {
			shoot_once();
			_CAMPOS_updated_roll_angle = false;
			_last_shoot_position = current_position;
		}
	}
}

void
CameraTrigger::enable_keep_alive(bool on)
{
	if (on) {
		PX4_DEBUG("keep alive enable");

		// schedule keep-alive up and down calls
		hrt_call_every(&_keepalivecall_up, 0, (60000 * 1000), &CameraTrigger::keep_alive_up, this);

		hrt_call_every(&_keepalivecall_down, 0 + (30000 * 1000), (60000 * 1000), &CameraTrigger::keep_alive_down, this);

	} else {
		PX4_DEBUG("keep alive disable");
		// cancel all calls
		hrt_cancel(&_keepalivecall_up);
		hrt_cancel(&_keepalivecall_down);
	}
}

void
CameraTrigger::toggle_power()
{
	PX4_DEBUG("toggle power");

	// schedule power toggle calls
	hrt_call_after(&_engage_turn_on_off_call, 0, &CameraTrigger::engange_turn_on_off, this);

	hrt_call_after(&_disengage_turn_on_off_call, 0 + (200 * 1000), &CameraTrigger::disengage_turn_on_off, this);
}

void
CameraTrigger::shoot_once()
{
	PX4_DEBUG("shoot once");

	// schedule trigger on and off calls
	hrt_call_after(&_engagecall, 0,
		       (hrt_callout)&CameraTrigger::engage, this);

	hrt_call_after(&_disengagecall, 0 + (_activation_time * 1000), &CameraTrigger::disengage, this);
}

bool
CameraTrigger::start()
{
	if (_camera_interface == nullptr) {
		if (camera_trigger::g_camera_trigger != nullptr) {
			delete (camera_trigger::g_camera_trigger);
			camera_trigger::g_camera_trigger = nullptr;

		}

		return false;
	}

	if ((_trigger_mode == TRIGGER_MODE_INTERVAL_ALWAYS_ON ||
	     _trigger_mode == TRIGGER_MODE_DISTANCE_ALWAYS_ON) &&
	    _camera_interface->has_power_control() &&
	    !_camera_interface->is_powered_on()) {

		// If in always-on mode and the interface supports it, enable power to the camera
		toggle_power();
		enable_keep_alive(true);

	} else {
		enable_keep_alive(false);
	}

	// enable immediately if configured that way
	if (_trigger_mode == TRIGGER_MODE_INTERVAL_ALWAYS_ON) {
		// enable and start triggering
		_trigger_enabled = true;
		update_intervalometer();

	} else if (_trigger_mode == TRIGGER_MODE_DISTANCE_ALWAYS_ON) {
		// just enable, but do not fire. actual trigger is based on distance covered
		_trigger_enabled = true;
	}

	// start to monitor at high rate for trigger enable command
	ScheduleNow();

	return true;
}

void
CameraTrigger::stop()
{
	ScheduleClear();

	hrt_cancel(&_engagecall);
	hrt_cancel(&_disengagecall);
	hrt_cancel(&_engage_turn_on_off_call);
	hrt_cancel(&_disengage_turn_on_off_call);
	hrt_cancel(&_keepalivecall_up);
	hrt_cancel(&_keepalivecall_down);

	if (camera_trigger::g_camera_trigger != nullptr) {
		delete (camera_trigger::g_camera_trigger);
		camera_trigger::g_camera_trigger = nullptr;
	}
}

void
CameraTrigger::test()
{
	vehicle_command_s vcmd{};
	vcmd.param5 = 1.0;
	vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_DIGICAM_CONTROL;
	vcmd.target_system = 1;
	vcmd.target_component = 1;

	uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
	vcmd.timestamp = hrt_absolute_time();
	vcmd_pub.publish(vcmd);
}

void
CameraTrigger::Run()
{
	// default loop polling interval
	int poll_interval_usec = 50000;

	vehicle_command_s cmd{};
	unsigned cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
	bool need_ack = false;

	// this flag is set when the polling loop is slowed down to allow the camera to power on
	_turning_on = false;

	// these flags are used to detect state changes in the command loop
	bool main_state = _trigger_enabled;
	bool pause_state = _trigger_paused;

	bool updated = _command_sub.update(&cmd);

	// Command handling
	if (updated) {
		if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_DIGICAM_CONTROL) {
			PX4_DEBUG("received DO_DIGICAM_CONTROL");

			need_ack = true;
			hrt_abstime now = hrt_absolute_time();

			if (now - _last_trigger_timestamp < _min_interval * 1000) {
				// triggering too fast, abort
				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;

			} else {
				if (commandParamToInt(cmd.param7) == 1) {
					// test shots are not logged or forwarded to GCS for geotagging
					_test_shot = true;
				}

				if (commandParamToInt((float)cmd.param5) == 1) {
					// Schedule shot
					_one_shot = true;
				}

				cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;
			}

		} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_TRIGGER_CONTROL) {
			PX4_DEBUG("received DO_TRIGGER_CONTROL");
			need_ack = true;

			if (commandParamToInt(cmd.param3) == 1) {
				// pause triggger
				_trigger_paused = true;

			} else if (commandParamToInt(cmd.param3) == 0) {
				_trigger_paused = false;
			}

			if (commandParamToInt(cmd.param2) == 1) {
				// reset trigger sequence
				_trigger_seq = 0;
			}

			if (commandParamToInt(cmd.param1) == 1) {
				_trigger_enabled = true;

			} else if (commandParamToInt(cmd.param1) == 0) {
				_trigger_enabled = false;
			}

			cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

		} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_SET_CAM_TRIGG_DIST) {
			PX4_DEBUG("received DO_SET_CAM_TRIGG_DIST");
			need_ack = true;

			/*
			 * TRANSITIONAL SUPPORT ADDED AS OF 11th MAY 2017 (v1.6 RELEASE)
			 */
			if (cmd.param1 > 0.0f) {
				_distance = cmd.param1;
				param_set_no_notification(_p_distance, &_distance);

				_trigger_enabled = true;
				_trigger_paused = false;
				_valid_position = false;

			} else if (commandParamToInt(cmd.param1) == 0) {
				_trigger_paused = true;

			} else if (commandParamToInt(cmd.param1) == -1) {
				_trigger_enabled = false;
			}

			// We can only control the shutter integration time of the camera in GPIO mode (for now)
			if (cmd.param2 > 0.0f) {
				if (_camera_interface_mode == CAMERA_INTERFACE_MODE_GPIO) {
					_activation_time = cmd.param2;
					param_set_no_notification(_p_activation_time, &(_activation_time));
				}
			}

			// Trigger once immediately if param is set
			if (cmd.param3 > 0.0f) {
				// Schedule shot
				_one_shot = true;
			}

			cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

		} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_SET_CAM_TRIGG_INTERVAL) {
			PX4_DEBUG("received DO_SET_CAM_TRIGG_INTERVAL");
			need_ack = true;

			if (cmd.param1 > 0.0f) {
				_interval = cmd.param1;
				param_set_no_notification(_p_interval, &(_interval));
			}

			// We can only control the shutter integration time of the camera in GPIO mode
			if (cmd.param2 > 0.0f) {
				if (_camera_interface_mode == CAMERA_INTERFACE_MODE_GPIO) {
					_activation_time = cmd.param2;
					param_set_no_notification(_p_activation_time, &_activation_time);
				}
			}

			cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

		} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_OBLIQUE_SURVEY) {
			PX4_INFO("received OBLIQUE_SURVEY");
			// Camera Auto Mount Pivoting Oblique Survey (CAMPOS)

			need_ack = true;

			if (cmd.param1 > 0.0f) {
				_distance = cmd.param1;
				param_set_no_notification(_p_distance, &_distance);

				_trigger_enabled = true;
				_trigger_paused = false;
				_valid_position = false;

			} else if (commandParamToInt(cmd.param1) == 0) {
				_trigger_paused = true;

			} else if (commandParamToInt(cmd.param1) == -1) {
				_trigger_enabled = false;
			}

			// We can only control the shutter integration time of the camera in GPIO mode (for now)
			if (cmd.param2 > 0.0f) {
				if (_camera_interface_mode == CAMERA_INTERFACE_MODE_GPIO) {
					_activation_time = cmd.param2;
					param_set_no_notification(_p_activation_time, &(_activation_time));
				}
			}

			// Set Param for minimum camera trigger interval
			if (cmd.param3 > 0.0f) {
				_min_interval = cmd.param3;
				param_set_no_notification(_p_min_interval, &(_min_interval));
			}

			if (cmd.param4 >= 2.0f) {
				_CAMPOS_num_poses = commandParamToInt(cmd.param4);
				_CAMPOS_roll_angle = cmd.param5;
				_CAMPOS_pitch_angle = cmd.param6;
				_CAMPOS_angle_interval = _CAMPOS_roll_angle * 2 / (_CAMPOS_num_poses - 1);
				_CAMPOS_pose_counter = 0;
				_CAMPOS_updated_roll_angle = false;

			} else {
				_CAMPOS_num_poses = 0;
			}

			cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

		} else {
			goto unknown_cmd;
		}

		_target_system = cmd.target_system;
		_target_component = cmd.target_component;
	}

unknown_cmd:

	// State change handling
	if ((main_state != _trigger_enabled) ||
	    (pause_state != _trigger_paused) ||
	    _one_shot) {

		if (_trigger_enabled || _one_shot) { // Just got enabled via a command

			// If camera isn't already powered on, we enable power to it
			if (!_camera_interface->is_powered_on() &&
			    _camera_interface->has_power_control()) {

				toggle_power();
				enable_keep_alive(true);

				// Give the camera time to turn on before starting to send trigger signals
				poll_interval_usec = 3000000;
				_turning_on = true;
			}

		} else if (!_trigger_enabled || _trigger_paused) { // Just got disabled/paused via a command

			// Power off the camera if we are disabled
			if (_camera_interface->is_powered_on() &&
			    _camera_interface->has_power_control() &&
			    !_trigger_enabled) {

				enable_keep_alive(false);
				toggle_power();
			}

			// cancel all calls for both disabled and paused
			hrt_cancel(&_engagecall);
			hrt_cancel(&_disengagecall);

			// ensure that the pin is off
			hrt_call_after(&_disengagecall, 0, (hrt_callout)&CameraTrigger::disengage, this);

			// reset distance counter if needed
			if (_trigger_mode == TRIGGER_MODE_DISTANCE_ON_CMD ||
			    _trigger_mode == TRIGGER_MODE_DISTANCE_ALWAYS_ON) {

				// this will force distance counter reinit on getting enabled/unpaused
				_valid_position = false;
			}
		}

		// only run on state changes, not every loop iteration
		if (_trigger_mode == TRIGGER_MODE_INTERVAL_ON_CMD) {
			// update intervalometer state and reset calls
			update_intervalometer();
		}
	}

	// order matters - one_shot has to be handled LAST
	// as the other trigger handlers will back off from it

	// run every loop iteration and trigger if needed
	if (_trigger_mode == TRIGGER_MODE_DISTANCE_ON_CMD ||
	    _trigger_mode == TRIGGER_MODE_DISTANCE_ALWAYS_ON) {

		// update distance counter and trigger
		update_distance();
	}

	// One shot command-based capture handling
	if (_one_shot && !_turning_on) {
		// One-shot trigger
		shoot_once();
		_one_shot = false;

		if (_test_shot) {
			_test_shot = false;
		}
	}

	// Command ACK handling
	if (updated && need_ack) {
		PX4_DEBUG("acknowledging command %d, result=%d", cmd.command, cmd_result);
		vehicle_command_ack_s command_ack{};
		command_ack.command = cmd.command;
		command_ack.result = (uint8_t)cmd_result;
		command_ack.target_system = cmd.source_system;
		command_ack.target_component = cmd.source_component;
		command_ack.timestamp = hrt_absolute_time();
		_cmd_ack_pub.publish(command_ack);
	}

	ScheduleDelayed(poll_interval_usec);
}

void
CameraTrigger::adjust_roll()
{
	vehicle_command_s vcmd{};

	vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONTROL;
	vcmd.target_system = _target_system;
	vcmd.target_component = _target_component;

	//param1 of VEHICLE_CMD_DO_MOUNT_CONTROL in VEHICLE_MOUNT_MODE_MAVLINK_TARGETING mode is pitch
	vcmd.param1 = _CAMPOS_pitch_angle;

	//param2 of VEHICLE_CMD_DO_MOUNT_CONTROL in VEHICLE_MOUNT_MODE_MAVLINK_TARGETING mode is roll
	if (++_CAMPOS_pose_counter == _CAMPOS_num_poses) {
		_CAMPOS_pose_counter = 0;
	}

	vcmd.param2 = _CAMPOS_angle_interval * _CAMPOS_pose_counter - _CAMPOS_roll_angle;

	vcmd.param7 = vehicle_command_s::VEHICLE_MOUNT_MODE_MAVLINK_TARGETING;

	uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
	vcmd.timestamp = hrt_absolute_time();
	vcmd_pub.publish(vcmd);
}

void
CameraTrigger::engage(void *arg)
{
	CameraTrigger *trig = static_cast<CameraTrigger *>(arg);

	hrt_abstime now = hrt_absolute_time();

	if ((trig->_last_trigger_timestamp > 0) && (now - trig->_last_trigger_timestamp < trig->_min_interval * 1000)) {
		return;
	}

	// Trigger the camera
	trig->_camera_interface->trigger(true);
	// set last timestamp
	trig->_last_trigger_timestamp = now;

	if (trig->_test_shot) {
		// do not send messages or increment frame count for test shots
		return;
	}

	// Send camera trigger message. This messages indicates that we sent
	// the camera trigger request. Does not guarantee capture.
	camera_trigger_s trigger{};

	timespec tv{};
	px4_clock_gettime(CLOCK_REALTIME, &tv);
	trigger.timestamp_utc = (uint64_t) tv.tv_sec * 1000000 + tv.tv_nsec / 1000;

	trigger.seq = trig->_trigger_seq;
	trigger.feedback = false;
	trigger.timestamp = hrt_absolute_time();

	if (!trig->_cam_cap_fback) {
		orb_publish(ORB_ID(camera_trigger), trig->_trigger_pub, &trigger);

	} else {
		orb_publish(ORB_ID(camera_trigger_secondary), trig->_trigger_pub, &trigger);
	}

	// increment frame count
	trig->_trigger_seq++;
}

void
CameraTrigger::disengage(void *arg)
{
	CameraTrigger *trig = static_cast<CameraTrigger *>(arg);

	trig->_camera_interface->trigger(false);
}

void
CameraTrigger::engange_turn_on_off(void *arg)
{
	CameraTrigger *trig = static_cast<CameraTrigger *>(arg);

	trig->_camera_interface->send_toggle_power(true);
}

void
CameraTrigger::disengage_turn_on_off(void *arg)
{
	CameraTrigger *trig = static_cast<CameraTrigger *>(arg);

	trig->_camera_interface->send_toggle_power(false);
}

void
CameraTrigger::keep_alive_up(void *arg)
{
	CameraTrigger *trig = static_cast<CameraTrigger *>(arg);

	trig->_camera_interface->send_keep_alive(true);
}

void
CameraTrigger::keep_alive_down(void *arg)
{
	CameraTrigger *trig = static_cast<CameraTrigger *>(arg);

	trig->_camera_interface->send_keep_alive(false);
}

void
CameraTrigger::status()
{
	PX4_INFO("main state : %s", _trigger_enabled ? "enabled" : "disabled");
	PX4_INFO("pause state : %s", _trigger_paused ? "paused" : "active");
	PX4_INFO("mode : %i", _trigger_mode);

	if (_trigger_mode == TRIGGER_MODE_INTERVAL_ALWAYS_ON ||
	    _trigger_mode == TRIGGER_MODE_INTERVAL_ON_CMD) {
		PX4_INFO("interval : %.2f [ms]", (double)_interval);

	} else if (_trigger_mode == TRIGGER_MODE_DISTANCE_ALWAYS_ON ||
		   _trigger_mode == TRIGGER_MODE_DISTANCE_ON_CMD) {
		PX4_INFO("distance : %.2f [m]", (double)_distance);
	}

	if (_camera_interface->has_power_control())	{
		PX4_INFO("camera power : %s", _camera_interface->is_powered_on() ? "ON" : "OFF");
	}

	PX4_INFO("activation time : %.2f [ms]", (double)_activation_time);
	_camera_interface->info();
}

static int usage()
{
	PX4_INFO("usage: camera_trigger {start|stop|status|test|test_power}\n");
	return 1;
}

extern "C" __EXPORT int camera_trigger_main(int argc, char *argv[])
{
	if (argc < 2) {
		return usage();
	}

	if (!strcmp(argv[1], "start")) {

		if (camera_trigger::g_camera_trigger != nullptr) {
			PX4_WARN("already running");
			return 0;
		}

		camera_trigger::g_camera_trigger = new CameraTrigger();

		if (camera_trigger::g_camera_trigger == nullptr) {
			PX4_WARN("alloc failed");
			return 1;
		}

		if (!camera_trigger::g_camera_trigger->start()) {
			PX4_WARN("failed to start camera trigger");
			return 1;
		}

		return 0;
	}

	if (camera_trigger::g_camera_trigger == nullptr) {
		PX4_WARN("not running");
		return 1;

	} else if (!strcmp(argv[1], "stop")) {
		camera_trigger::g_camera_trigger->stop();

	} else if (!strcmp(argv[1], "status")) {
		camera_trigger::g_camera_trigger->status();

	} else if (!strcmp(argv[1], "test")) {
		camera_trigger::g_camera_trigger->test();

	} else if (!strcmp(argv[1], "test_power")) {
		camera_trigger::g_camera_trigger->toggle_power();

	} else {
		return usage();
	}

	return 0;
}
