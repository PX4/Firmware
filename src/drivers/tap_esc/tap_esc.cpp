/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include <stdint.h>

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/posix.h>
#include <errno.h>

#include <math.h>	// NAN
#include <cstring>

#include <lib/mathlib/mathlib.h>
#include <lib/cdev/CDev.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/test_motor.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/parameter_update.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_mixer.h>
#include <lib/mixer/MixerGroup.hpp>

#include "tap_esc_common.h"

#include "drv_tap_esc.h"

#if !defined(BOARD_TAP_ESC_MODE)
#  define BOARD_TAP_ESC_MODE 0
#endif

#if !defined(DEVICE_ARGUMENT_MAX_LENGTH)
#  define DEVICE_ARGUMENT_MAX_LENGTH 32
#endif

// uorb update rate for control groups in miliseconds
#if !defined(TAP_ESC_CTRL_UORB_UPDATE_INTERVAL)
#  define TAP_ESC_CTRL_UORB_UPDATE_INTERVAL 2  // [ms] min: 2, max: 100
#endif

using namespace time_literals;

/*
 * This driver connects to TAP ESCs via serial.
 */
class TAP_ESC : public cdev::CDev, public ModuleBase<TAP_ESC>, public ModuleParams
{
public:
	TAP_ESC(char const *const device, uint8_t channels_count);
	virtual ~TAP_ESC();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static TAP_ESC *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	virtual int init();
	virtual int ioctl(cdev::file_t *filp, int cmd, unsigned long arg);
	void cycle();

private:
	char 			_device[DEVICE_ARGUMENT_MAX_LENGTH];
	int 			_uart_fd = -1;
	static const uint8_t 	_device_mux_map[TAP_ESC_MAX_MOTOR_NUM];
	static const uint8_t 	_device_dir_map[TAP_ESC_MAX_MOTOR_NUM];
	bool 			_is_armed = false;
	int			_armed_sub = -1;
	int 			_test_motor_sub = -1;

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	orb_advert_t        	_outputs_pub = nullptr;
	actuator_outputs_s      _outputs = {};
	actuator_armed_s	_armed = {};

	perf_counter_t	_perf_control_latency;

	int			_control_subs[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	actuator_controls_s 	_controls[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	orb_id_t		_control_topics[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	px4_pollfd_struct_t	_poll_fds[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	unsigned		_poll_fds_num = 0;

	orb_advert_t      _esc_feedback_pub = nullptr;
	orb_advert_t      _to_mixer_status = nullptr; 	///< mixer status flags
	esc_status_s      _esc_feedback = {};
	uint8_t    	  _channels_count = 0; 		///< nnumber of ESC channels
	uint8_t 	  _responding_esc = 0;

	MixerGroup	*_mixers = nullptr;
	uint32_t	_groups_required = 0;
	uint32_t	_groups_subscribed = 0;
	ESC_UART_BUF 	_uartbuf = {};
	EscPacket 	_packet = {};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MC_AIRMODE>) _param_mc_airmode   ///< multicopter air-mode
	)

	void subscribe();
	void send_esc_outputs(const uint16_t *pwm, const uint8_t motor_cnt);
	static int control_callback_trampoline(uintptr_t handle,
					       uint8_t control_group, uint8_t control_index, float &input);
	inline int control_callback(uint8_t control_group, uint8_t control_index, float &input);
};

const uint8_t TAP_ESC::_device_mux_map[TAP_ESC_MAX_MOTOR_NUM] = ESC_POS;
const uint8_t TAP_ESC::_device_dir_map[TAP_ESC_MAX_MOTOR_NUM] = ESC_DIR;

TAP_ESC::TAP_ESC(char const *const device, uint8_t channels_count):
	CDev(nullptr),
	ModuleParams(nullptr),
	_perf_control_latency(perf_alloc(PC_ELAPSED, "tap_esc control latency")),
	_channels_count(channels_count)
{
	strncpy(_device, device, sizeof(_device));
	_device[sizeof(_device) - 1] = '\0';  // Fix in case of overflow

	_control_topics[0] = ORB_ID(actuator_controls_0);
	_control_topics[1] = ORB_ID(actuator_controls_1);
	_control_topics[2] = ORB_ID(actuator_controls_2);
	_control_topics[3] = ORB_ID(actuator_controls_3);
	memset(_controls, 0, sizeof(_controls));
	memset(_poll_fds, 0, sizeof(_poll_fds));

	for (int i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; ++i) {
		_control_subs[i] = -1;
	}

	for (size_t i = 0; i < sizeof(_outputs.output) / sizeof(_outputs.output[0]); i++) {
		_outputs.output[i] = NAN;
	}

	_outputs.noutputs = 0;
}

TAP_ESC::~TAP_ESC()
{
	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (_control_subs[i] >= 0) {
			orb_unsubscribe(_control_subs[i]);
			_control_subs[i] = -1;
		}
	}

	orb_unsubscribe(_armed_sub);
	orb_unsubscribe(_test_motor_sub);

	orb_unadvertise(_outputs_pub);
	orb_unadvertise(_esc_feedback_pub);
	orb_unadvertise(_to_mixer_status);

	tap_esc_common::deinitialise_uart(_uart_fd);

	PX4_INFO("stopping");

	perf_free(_perf_control_latency);
}

/** @see ModuleBase */
TAP_ESC *TAP_ESC::instantiate(int argc, char *argv[])
{
	/* Parse arguments */
	const char *device = nullptr;
	uint8_t channels_count = 0;

	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	if (argc < 2) {
		print_usage("not enough arguments");
		return nullptr;
	}

	while ((ch = px4_getopt(argc, argv, "d:n:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			break;

		case 'n':
			channels_count = atoi(myoptarg);
			break;
		}
	}

	/* Sanity check on arguments */
	if (channels_count == 0) {
		print_usage("Channel count is invalid (0)");
		return nullptr;
	}

	if (device == nullptr || strlen(device) == 0) {
		print_usage("no device specified");
		return nullptr;
	}

	TAP_ESC *tap_esc = new TAP_ESC(device, channels_count);

	if (tap_esc == nullptr) {
		PX4_ERR("failed to instantiate module");
		return nullptr;
	}

	if (tap_esc->init() != 0) {
		PX4_ERR("failed to initialize module");
		delete tap_esc;
		return nullptr;
	}

	return tap_esc;
}

/** @see ModuleBase */
int TAP_ESC::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int TAP_ESC::init()
{
	int ret;

	ret = tap_esc_common::initialise_uart(_device, _uart_fd);

	if (ret != 0) {
		PX4_ERR("failed to initialise UART.");
		return ret;
	}

	/* Respect boot time required by the ESC FW */
	hrt_abstime uptime_us = hrt_absolute_time();

	if (uptime_us < MAX_BOOT_TIME_MS * 1000) {
		usleep((MAX_BOOT_TIME_MS * 1000) - uptime_us);
	}

	/* Issue Basic Config */
	EscPacket packet = {PACKET_HEAD, sizeof(ConfigInfoBasicRequest), ESCBUS_MSG_ID_CONFIG_BASIC};
	ConfigInfoBasicRequest   &config = packet.d.reqConfigInfoBasic;
	memset(&config, 0, sizeof(ConfigInfoBasicRequest));
	config.maxChannelInUse = _channels_count;
	/* Enable closed-loop control if supported by the board */
	config.controlMode = BOARD_TAP_ESC_MODE;

	/* Asign the id's to the ESCs to match the mux */
	for (uint8_t phy_chan_index = 0; phy_chan_index < _channels_count; phy_chan_index++) {
		config.channelMapTable[phy_chan_index] = _device_mux_map[phy_chan_index] &
				ESC_MASK_MAP_CHANNEL;
		config.channelMapTable[phy_chan_index] |= (_device_dir_map[phy_chan_index] << 4) &
				ESC_MASK_MAP_RUNNING_DIRECTION;
	}

	config.maxChannelValue = RPMMAX;
	config.minChannelValue = RPMMIN;

	ret = tap_esc_common::send_packet(_uart_fd, packet, 0);

	if (ret < 0) {
		return ret;
	}

	/* To Unlock the ESC from the Power up state we need to issue 10
	 * ESCBUS_MSG_ID_RUN request with all the values 0;
	 */
	EscPacket unlock_packet = {PACKET_HEAD, _channels_count, ESCBUS_MSG_ID_RUN};
	unlock_packet.len *= sizeof(unlock_packet.d.reqRun.rpm_flags[0]);
	memset(unlock_packet.d.bytes, 0, sizeof(unlock_packet.d.bytes));

	int unlock_times = 10;

	while (unlock_times--) {

		tap_esc_common::send_packet(_uart_fd, unlock_packet, -1);

		/* Min Packet to Packet time is 1 Ms so use 2 */
		usleep(2000);
	}

	/* do regular cdev init */
	ret = CDev::init();

	/* advertise the mixed control outputs, insist on the first group output */
	_outputs_pub = orb_advertise(ORB_ID(actuator_outputs), &_outputs);
	_esc_feedback_pub = orb_advertise(ORB_ID(esc_status), &_esc_feedback);
	multirotor_motor_limits_s multirotor_motor_limits = {};
	_to_mixer_status = orb_advertise(ORB_ID(multirotor_motor_limits), &multirotor_motor_limits);

	_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	_test_motor_sub = orb_subscribe(ORB_ID(test_motor));

	return ret;
}

void TAP_ESC::subscribe()
{
	/* subscribe/unsubscribe to required actuator control groups */
	uint32_t sub_groups = _groups_required & ~_groups_subscribed;
	uint32_t unsub_groups = _groups_subscribed & ~_groups_required;
	_poll_fds_num = 0;

	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (sub_groups & (1 << i)) {
			PX4_DEBUG("subscribe to actuator_controls_%d", i);
			_control_subs[i] = orb_subscribe(_control_topics[i]);
		}

		if (unsub_groups & (1 << i)) {
			PX4_DEBUG("unsubscribe from actuator_controls_%d", i);
			orb_unsubscribe(_control_subs[i]);
			_control_subs[i] = -1;
		}

		if (_control_subs[i] >= 0) {
			_poll_fds[_poll_fds_num].fd = _control_subs[i];
			_poll_fds[_poll_fds_num].events = POLLIN;
			_poll_fds_num++;
		}
	}
}

void TAP_ESC::send_esc_outputs(const uint16_t *pwm, const uint8_t motor_cnt)
{
	uint16_t rpm[TAP_ESC_MAX_MOTOR_NUM] = {};

	for (uint8_t i = 0; i < motor_cnt; i++) {
		rpm[i] = pwm[i];

		if (rpm[i] > RPMMAX) {
			rpm[i] = RPMMAX;

		} else if (rpm[i] < RPMSTOPPED) {
			rpm[i] = RPMSTOPPED;
		}
	}

	rpm[_responding_esc] |= (RUN_FEEDBACK_ENABLE_MASK | RUN_BLUE_LED_ON_MASK);

	EscPacket packet = {PACKET_HEAD, _channels_count, ESCBUS_MSG_ID_RUN};
	packet.len *= sizeof(packet.d.reqRun.rpm_flags[0]);

	for (uint8_t i = 0; i < _channels_count; i++) {
		packet.d.reqRun.rpm_flags[i] = rpm[i];
	}

	int ret = tap_esc_common::send_packet(_uart_fd, packet, _responding_esc);

	if (++_responding_esc >= _channels_count) {
		_responding_esc = 0;
	}

	if (ret < 1) {
		PX4_WARN("TX ERROR: ret: %d, errno: %d", ret, errno);
	}
}

void TAP_ESC::cycle()
{
	if (_groups_subscribed != _groups_required) {
		subscribe();
		_groups_subscribed = _groups_required;

		/* Set uorb update rate */
		for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
			if (_control_subs[i] >= 0) {
				orb_set_interval(_control_subs[i], TAP_ESC_CTRL_UORB_UPDATE_INTERVAL);
				PX4_DEBUG("New actuator update interval: %ums", TAP_ESC_CTRL_UORB_UPDATE_INTERVAL);
			}
		}
	}

	if (_mixers) {
		_mixers->set_airmode((Mixer::Airmode)_param_mc_airmode.get());
	}

	/* check if anything updated */
	int ret = px4_poll(_poll_fds, _poll_fds_num, 5);

	/* this would be bad... */
	if (ret < 0) {
		PX4_ERR("poll error %d", errno);

	} else { /* update even in the case of a timeout, to check for test_motor commands */

		/* get controls for required topics */
		unsigned poll_id = 0;

		for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
			if (_control_subs[i] >= 0) {
				if (_poll_fds[poll_id].revents & POLLIN) {
					orb_copy(_control_topics[i], _control_subs[i], &_controls[i]);

				}

				poll_id++;
			}
		}

		uint8_t num_outputs = _channels_count;

		/* can we mix? */
		if (_is_armed && _mixers != nullptr) {

			/* do mixing */
			num_outputs = _mixers->mix(&_outputs.output[0], num_outputs);
			_outputs.noutputs = num_outputs;

			/* publish mixer status */
			multirotor_motor_limits_s multirotor_motor_limits = {};
			multirotor_motor_limits.saturation_status = _mixers->get_saturation_status();

			orb_publish(ORB_ID(multirotor_motor_limits), _to_mixer_status, &multirotor_motor_limits);

			/* disable unused ports by setting their output to NaN */
			for (size_t i = num_outputs; i < sizeof(_outputs.output) / sizeof(_outputs.output[0]); i++) {
				_outputs.output[i] = NAN;
			}

			/* iterate actuators */
			for (unsigned i = 0; i < num_outputs; i++) {
				/* last resort: catch NaN, INF and out-of-band errors */
				if (i < _outputs.noutputs && PX4_ISFINITE(_outputs.output[i])
				    && !_armed.lockdown && !_armed.manual_lockdown) {
					/* scale for PWM output 1200 - 1900us */
					_outputs.output[i] = (RPMMAX + RPMMIN) / 2 + ((RPMMAX - RPMMIN) / 2) * _outputs.output[i];
					math::constrain(_outputs.output[i], (float)RPMMIN, (float)RPMMAX);

				} else {
					/*
					 * Value is NaN, INF, or we are in lockdown - stop the motor.
					 * This will be clearly visible on the servo status and will limit the risk of accidentally
					 * spinning motors. It would be deadly in flight.
					 */
					_outputs.output[i] = RPMSTOPPED;
				}
			}

		} else {

			_outputs.noutputs = num_outputs;
			_outputs.timestamp = hrt_absolute_time();

			/* check for motor test commands */
			bool test_motor_updated;
			orb_check(_test_motor_sub, &test_motor_updated);

			if (test_motor_updated) {
				struct test_motor_s test_motor;
				orb_copy(ORB_ID(test_motor), _test_motor_sub, &test_motor);

				if (test_motor.action == test_motor_s::ACTION_STOP) {
					_outputs.output[test_motor.motor_number] = RPMSTOPPED;

				} else {
					_outputs.output[test_motor.motor_number] = RPMSTOPPED + ((RPMMAX - RPMSTOPPED) * test_motor.value);
				}

				PX4_INFO("setting motor %i to %.1lf", test_motor.motor_number,
					 (double)_outputs.output[test_motor.motor_number]);
			}

			/* set the invalid values to the minimum */
			for (unsigned i = 0; i < num_outputs; i++) {
				if (!PX4_ISFINITE(_outputs.output[i])) {
					_outputs.output[i] = RPMSTOPPED;
				}
			}

			/* disable unused ports by setting their output to NaN */
			for (size_t i = num_outputs; i < sizeof(_outputs.output) / sizeof(_outputs.output[0]); i++) {
				_outputs.output[i] = NAN;
			}

		}

		uint16_t motor_out[TAP_ESC_MAX_MOTOR_NUM];

		// We need to remap from the system default to what PX4's normal
		// scheme is
		switch (num_outputs) {
		case 4:
			motor_out[0] = (uint16_t)_outputs.output[2];
			motor_out[1] = (uint16_t)_outputs.output[1];
			motor_out[2] = (uint16_t)_outputs.output[0];
			motor_out[3] = (uint16_t)_outputs.output[3];
			break;

		case 6:
			motor_out[0] = (uint16_t)_outputs.output[3];
			motor_out[1] = (uint16_t)_outputs.output[0];
			motor_out[2] = (uint16_t)_outputs.output[4];
			motor_out[3] = (uint16_t)_outputs.output[2];
			motor_out[4] = (uint16_t)_outputs.output[1];
			motor_out[5] = (uint16_t)_outputs.output[5];
			break;

		default:

			// Use the system defaults
			for (uint8_t i = 0; i < num_outputs; ++i) {
				motor_out[i] = (uint16_t)_outputs.output[i];
			}

			break;
		}

		// Set remaining motors to RPMSTOPPED to be on the safe side
		for (uint8_t i = num_outputs; i < TAP_ESC_MAX_MOTOR_NUM; ++i) {
			motor_out[i] = RPMSTOPPED;
		}

		_outputs.timestamp = hrt_absolute_time();

		send_esc_outputs(motor_out, num_outputs);
		tap_esc_common::read_data_from_uart(_uart_fd, &_uartbuf);

		if (!tap_esc_common::parse_tap_esc_feedback(&_uartbuf, &_packet)) {
			if (_packet.msg_id == ESCBUS_MSG_ID_RUN_INFO) {
				RunInfoRepsonse &feed_back_data = _packet.d.rspRunInfo;

				if (feed_back_data.channelID < esc_status_s::CONNECTED_ESC_MAX) {
					_esc_feedback.esc[feed_back_data.channelID].esc_rpm = feed_back_data.speed;
					_esc_feedback.esc[feed_back_data.channelID].esc_state = feed_back_data.ESCStatus;
					_esc_feedback.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_SERIAL;
					_esc_feedback.counter++;
					_esc_feedback.esc_count = num_outputs;
					_esc_feedback.esc_online_flags = (1 << num_outputs) - 1;
					_esc_feedback.esc_armed_flags = (1 << num_outputs) - 1;

					_esc_feedback.timestamp = hrt_absolute_time();

					orb_publish(ORB_ID(esc_status), _esc_feedback_pub, &_esc_feedback);
				}
			}
		}

		/* and publish for anyone that cares to see */
		orb_publish(ORB_ID(actuator_outputs), _outputs_pub, &_outputs);

		// use first valid timestamp_sample for latency tracking
		for (int i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
			const bool required = _groups_required & (1 << i);
			const hrt_abstime &timestamp_sample = _controls[i].timestamp_sample;

			if (required && (timestamp_sample > 0)) {
				perf_set_elapsed(_perf_control_latency, _outputs.timestamp - timestamp_sample);
				break;
			}
		}

	}

	bool updated;

	orb_check(_armed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);

		if (_is_armed != _armed.armed) {
			/* reset all outputs */
			for (size_t i = 0; i < sizeof(_outputs.output) / sizeof(_outputs.output[0]); i++) {
				_outputs.output[i] = NAN;
			}
		}

		_is_armed = _armed.armed;

	}

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}
}

int TAP_ESC::control_callback_trampoline(uintptr_t handle, uint8_t control_group, uint8_t control_index, float &input)
{
	TAP_ESC *obj = (TAP_ESC *)handle;
	return obj->control_callback(control_group, control_index, input);
}

int TAP_ESC::control_callback(uint8_t control_group, uint8_t control_index, float &input)
{
	input = _controls[control_group].control[control_index];

	/* limit control input */
	input = math::constrain(input, -1.f, 1.f);

	/* throttle not arming - mark throttle input as invalid */
	if (_armed.prearmed && !_armed.armed) {
		if ((control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE ||
		     control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE_ALTERNATE) &&
		    control_index == actuator_controls_s::INDEX_THROTTLE) {
			/* set the throttle to an invalid value */
			input = NAN;
		}
	}

	return 0;
}

int TAP_ESC::ioctl(cdev::file_t *filp, int cmd, unsigned long arg)
{
	int ret = OK;

	switch (cmd) {

	case MIXERIOCRESET:
		if (_mixers != nullptr) {
			delete _mixers;
			_mixers = nullptr;
			_groups_required = 0;
		}

		break;

	case MIXERIOCLOADBUF: {
			const char *buf = (const char *)arg;
			unsigned buflen = strlen(buf);

			if (_mixers == nullptr) {
				_mixers = new MixerGroup();
			}

			if (_mixers == nullptr) {
				_groups_required = 0;
				ret = -ENOMEM;

			} else {
				ret = _mixers->load_from_buf(control_callback_trampoline, (uintptr_t)this, buf, buflen);

				if (ret != 0) {
					PX4_DEBUG("mixer load failed with %d", ret);
					delete _mixers;
					_mixers = nullptr;
					_groups_required = 0;
					ret = -EINVAL;

				} else {

					_mixers->groups_required(_groups_required);
				}
			}

			break;
		}

	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}

/** @see ModuleBase */
void TAP_ESC::run()
{
	// Main loop
	while (!should_exit()) {
		cycle();
	}
}

/** @see ModuleBase */
int TAP_ESC::task_spawn(int argc, char *argv[])
{
	/* start the task */
	_task_id = px4_task_spawn_cmd("tap_esc",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_ACTUATOR_OUTPUTS,
				      1180,
				      (px4_main_t)&run_trampoline,
				      argv);

	if (_task_id < 0) {
		PX4_ERR("task start failed");
		_task_id = -1;
		return PX4_ERROR;
	}

	// wait until task is up & running
	if (wait_until_running() < 0) {
		_task_id = -1;
		return -1;
	}

	return PX4_OK;
}

/** @see ModuleBase */
int TAP_ESC::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module controls the TAP_ESC hardware via UART. It listens on the
actuator_controls topics, does the mixing and writes the PWM outputs.

### Implementation
Currently the module is implementd as a threaded version only, meaning that it
runs in its own thread instead of on the work queue.

### Example
The module is typically started with:
tap_esc start -d /dev/ttyS2 -n <1-8>

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("tap_esc", "driver");

	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<device>", "Device used to talk to ESCs", true);
	PRINT_MODULE_USAGE_PARAM_INT('n', 4, 0, 8, "Number of ESCs", true);
	return PX4_OK;
}

extern "C" __EXPORT int tap_esc_main(int argc, char *argv[])
{
	return TAP_ESC::main(argc, argv);
}
