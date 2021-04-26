/****************************************************************************
 *
 *   Copyright (c) 2012-2020 PX4 Development Team. All rights reserved.
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
 * @file px4io.cpp
 * Driver for the PX4IO board.
 *
 * PX4IO is connected via DMA enabled high-speed UART.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/sem.hpp>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <debug.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <crc32.h>



#include <drivers/device/device.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_sbus.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_io_heater.h>
#include <drivers/drv_mixer.h>

#include <rc/dsm.h>

#include <lib/mathlib/mathlib.h>
#include <lib/mixer/MixerGroup.hpp>
#include <lib/mixer/MultirotorMixer/MultirotorMixer.hpp>
#include <perf/perf_counter.h>
#include <systemlib/err.h>
#include <parameters/param.h>
#include <circuit_breaker/circuit_breaker.h>
#include <systemlib/mavlink_log.h>

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/safety.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/px4io_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/test_motor.h>

#include <debug.h>

#include <modules/px4iofirmware/protocol.h>

#include "uploader.h"

#include "modules/dataman/dataman.h"

#include "px4io_driver.h"

#define PX4IO_SET_DEBUG			_IOC(0xff00, 0)
#define PX4IO_INAIR_RESTART_ENABLE	_IOC(0xff00, 1)
#define PX4IO_REBOOT_BOOTLOADER		_IOC(0xff00, 2)
#define PX4IO_CHECK_CRC			_IOC(0xff00, 3)

#define ORB_CHECK_INTERVAL		200000		// 200 ms -> 5 Hz
#define IO_POLL_INTERVAL		20000		// 20 ms -> 50 Hz

using namespace time_literals;

/**
 * The PX4IO class.
 *
 * Encapsulates PX4FMU to PX4IO communications modeled as file operations.
 */
class PX4IO : public cdev::CDev
{
public:
	/**
	 * Constructor.
	 *
	 * Initialize all class variables.
	 */
	PX4IO(device::Device *interface);

	/**
	 * Destructor.
	 *
	 * Wait for worker thread to terminate.
	 */
	virtual ~PX4IO();

	/**
	 * Initialize the PX4IO class.
	 *
	 * Retrieve relevant initial system parameters. Initialize PX4IO registers.
	 */
	virtual int		init();

	/**
	 * Initialize the PX4IO class.
	 *
	 * Retrieve relevant initial system parameters. Initialize PX4IO registers.
	 *
	 * @param disable_rc_handling set to true to forbid override / RC handling on IO
	 * @param hitl_mode set to suppress publication of actuator_outputs - instead defer to pwm_out_sim
	 */
	int			init(bool disable_rc_handling, bool hitl_mode);

	/**
	 * Detect if a PX4IO is connected.
	 *
	 * Only validate if there is a PX4IO to talk to.
	 */
	virtual int		detect();

	/**
	 * IO Control handler.
	 *
	 * Handle all IOCTL calls to the PX4IO file descriptor.
	 *
	 * @param[in] filp file handle (not used). This function is always called directly through object reference
	 * @param[in] cmd the IOCTL command
	 * @param[in] the IOCTL command parameter (optional)
	 */
	virtual int		ioctl(file *filp, int cmd, unsigned long arg);

	/**
	 * Disable RC input handling
	 */
	int			disable_rc_handling();

	/**
	 * Print IO status.
	 *
	 * Print all relevant IO status information
	 *
	 * @param extended_status Shows more verbose information (in particular RC config)
	 */
	void			print_status(bool extended_status);

	/**
	 * Fetch and print debug console output.
	 */
	int			print_debug();

	/*
	 * To test what happens if IO stops receiving updates from FMU.
	 *
	 * @param is_fail	true for failure condition, false for normal operation.
	 */
	void			test_fmu_fail(bool is_fail)
	{
		_test_fmu_fail = is_fail;
	};

	inline uint16_t		system_status() const {return _status;}

private:
	device::Device		*_interface;

	unsigned		_hardware;		///< Hardware revision
	unsigned		_max_actuators;		///< Maximum # of actuators supported by PX4IO
	unsigned		_max_controls;		///< Maximum # of controls supported by PX4IO
	unsigned		_max_rc_input;		///< Maximum receiver channels supported by PX4IO
	unsigned		_max_relays;		///< Maximum relays supported by PX4IO
	unsigned		_max_transfer;		///< Maximum number of I2C transfers supported by PX4IO

	bool			_rc_handling_disabled;	///< If set, IO does not evaluate, but only forward the RC values
	unsigned		_rc_chan_count;		///< Internal copy of the last seen number of RC channels
	uint64_t		_rc_last_valid;		///< last valid timestamp

	volatile int		_task;			///< worker task id
	volatile bool		_task_should_exit;	///< worker terminate flag

	orb_advert_t		_mavlink_log_pub;	///< mavlink log pub

	perf_counter_t		_perf_update;		///< local performance counter for status updates
	perf_counter_t		_perf_write;		///< local performance counter for PWM control writes
	perf_counter_t		_perf_sample_latency;	///< total system latency (based on passed-through timestamp)

	/* cached IO state */
	uint16_t		_status{0};		///< Various IO status flags
	uint16_t		_alarms{0};		///< Various IO alarms
	uint16_t		_setup_arming{0};	///< last arming setup state
	uint16_t		_last_written_arming_s{0};	///< the last written arming state reg
	uint16_t		_last_written_arming_c{0};	///< the last written arming state reg

	/* subscribed topics */
	int			_t_actuator_controls_0;	///< actuator controls group 0 topic

	uORB::Subscription	_t_actuator_controls_1{ORB_ID(actuator_controls_1)};	///< actuator controls group 1 topic
	uORB::Subscription	_t_actuator_controls_2{ORB_ID(actuator_controls_2)};;	///< actuator controls group 2 topic
	uORB::Subscription	_t_actuator_controls_3{ORB_ID(actuator_controls_3)};;	///< actuator controls group 3 topic
	uORB::Subscription	_t_actuator_armed{ORB_ID(actuator_armed)};		///< system armed control topic
	uORB::Subscription 	_t_vehicle_control_mode{ORB_ID(vehicle_control_mode)};	///< vehicle control mode topic
	uORB::Subscription	_t_vehicle_command{ORB_ID(vehicle_command)};		///< vehicle command topic

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	hrt_abstime             _last_status_publish{0};

	bool			_param_update_force;	///< force a parameter update

	/* advertised topics */
	uORB::PublicationMulti<input_rc_s>			_to_input_rc{ORB_ID(input_rc)};
	uORB::PublicationMulti<actuator_outputs_s>		_to_outputs{ORB_ID(actuator_outputs)};
	uORB::PublicationMulti<multirotor_motor_limits_s>	_to_mixer_status{ORB_ID(multirotor_motor_limits)};
	uORB::Publication<px4io_status_s>			_px4io_status_pub{ORB_ID(px4io_status)};
	uORB::Publication<safety_s>				_to_safety{ORB_ID(safety)};

	safety_s _safety{};

	bool			_primary_pwm_device;	///< true if we are the default PWM output
	bool			_lockdown_override;	///< allow to override the safety lockdown
	bool			_armed;			///< wether the system is armed
	bool			_override_available;	///< true if manual reversion mode is enabled

	bool			_cb_flighttermination;	///< true if the flight termination circuit breaker is enabled
	bool 			_in_esc_calibration_mode;	///< do not send control outputs to IO (used for esc calibration)

	int32_t			_rssi_pwm_chan; ///< RSSI PWM input channel
	int32_t			_rssi_pwm_max; ///< max RSSI input on PWM channel
	int32_t			_rssi_pwm_min; ///< min RSSI input on PWM channel
	int32_t			_thermal_control; ///< thermal control state
	bool			_analog_rc_rssi_stable; ///< true when analog RSSI input is stable
	float			_analog_rc_rssi_volt; ///< analog RSSI voltage

	bool			_test_fmu_fail; ///< To test what happens if IO loses FMU

	struct MotorTest {
		uORB::Subscription test_motor_sub{ORB_ID(test_motor)};
		bool in_test_mode{false};
		hrt_abstime timeout{0};
	};
	MotorTest _motor_test;
	bool                    _hitl_mode;     ///< Hardware-in-the-loop simulation mode - don't publish actuator_outputs

	/**
	 * Trampoline to the worker task
	 */
	static int		task_main_trampoline(int argc, char *argv[]);

	/**
	 * worker task
	 */
	void			task_main();

	/**
	 * Send controls for one group to IO
	 */
	int			io_set_control_state(unsigned group);

	/**
	 * Send all controls to IO
	 */
	int			io_set_control_groups();

	/**
	 * Update IO's arming-related state
	 */
	int			io_set_arming_state();

	/**
	 * Push RC channel configuration to IO.
	 */
	int			io_set_rc_config();

	/**
	 * Fetch status and alarms from IO
	 *
	 * Also publishes battery voltage/current.
	 */
	int			io_get_status();

	/**
	 * Disable RC input handling
	 */
	int			io_disable_rc_handling();

	/**
	 * Fetch RC inputs from IO.
	 *
	 * @param input_rc	Input structure to populate.
	 * @return		OK if data was returned.
	 */
	int			io_get_raw_rc_input(input_rc_s &input_rc);

	/**
	 * Fetch and publish raw RC input data.
	 */
	int			io_publish_raw_rc();

	/**
	 * Fetch and publish the PWM servo outputs.
	 */
	int			io_publish_pwm_outputs();

	/**
	 * write register(s)
	 *
	 * @param page		Register page to write to.
	 * @param offset	Register offset to start writing at.
	 * @param values	Pointer to array of values to write.
	 * @param num_values	The number of values to write.
	 * @return		OK if all values were successfully written.
	 */
	int			io_reg_set(uint8_t page, uint8_t offset, const uint16_t *values, unsigned num_values);

	/**
	 * write a register
	 *
	 * @param page		Register page to write to.
	 * @param offset	Register offset to write to.
	 * @param value		Value to write.
	 * @return		OK if the value was written successfully.
	 */
	int			io_reg_set(uint8_t page, uint8_t offset, const uint16_t value);

	/**
	 * read register(s)
	 *
	 * @param page		Register page to read from.
	 * @param offset	Register offset to start reading from.
	 * @param values	Pointer to array where values should be stored.
	 * @param num_values	The number of values to read.
	 * @return		OK if all values were successfully read.
	 */
	int			io_reg_get(uint8_t page, uint8_t offset, uint16_t *values, unsigned num_values);

	/**
	 * read a register
	 *
	 * @param page		Register page to read from.
	 * @param offset	Register offset to start reading from.
	 * @return		Register value that was read, or _io_reg_get_error on error.
	 */
	uint32_t		io_reg_get(uint8_t page, uint8_t offset);
	static const uint32_t	_io_reg_get_error = 0x80000000;

	/**
	 * modify a register
	 *
	 * @param page		Register page to modify.
	 * @param offset	Register offset to modify.
	 * @param clearbits	Bits to clear in the register.
	 * @param setbits	Bits to set in the register.
	 */
	int			io_reg_modify(uint8_t page, uint8_t offset, uint16_t clearbits, uint16_t setbits);

	/**
	 * Send mixer definition text to IO
	 */
	int			mixer_send(const char *buf, unsigned buflen, unsigned retries = 3);

	/**
	 * Handle a status update from IO.
	 *
	 * Publish IO status information if necessary.
	 *
	 * @param status	The status register
	 */
	int			io_handle_status(uint16_t status);

	/**
	 * Handle issuing dsm bind ioctl to px4io.
	 *
	 * @param dsmMode	DSM2_BIND_PULSES, DSMX_BIND_PULSES, DSMX8_BIND_PULSES
	 */
	int			dsm_bind_ioctl(int dsmMode);

	/**
	 * Respond to a vehicle command with an ACK message
	 *
	 * @param cmd		The command that was executed or denied (inbound)
	 * @param result	The command result
	 */
	void			answer_command(const vehicle_command_s &cmd, uint8_t result);

	void update_params();

	/**
	 * check and handle test_motor topic updates
	 */
	void handle_motor_test();

	/* do not allow to copy this class due to ptr data members */
	PX4IO(const PX4IO &);
	PX4IO operator=(const PX4IO &);
};

namespace
{
PX4IO	*g_dev = nullptr;
}

#define PX4IO_DEVICE_PATH	"/dev/px4io"

PX4IO::PX4IO(device::Device *interface) :
	CDev(PX4IO_DEVICE_PATH),
	_interface(interface),
	_hardware(0),
	_max_actuators(0),
	_max_controls(0),
	_max_rc_input(0),
	_max_relays(0),
	_max_transfer(16),	/* sensible default */
	_rc_handling_disabled(false),
	_rc_chan_count(0),
	_rc_last_valid(0),
	_task(-1),
	_task_should_exit(false),
	_mavlink_log_pub(nullptr),
	_perf_update(perf_alloc(PC_ELAPSED, "io update")),
	_perf_write(perf_alloc(PC_ELAPSED, "io write")),
	_perf_sample_latency(perf_alloc(PC_ELAPSED, "io control latency")),
	_t_actuator_controls_0(-1),
	_param_update_force(false),
	_primary_pwm_device(false),
	_lockdown_override(false),
	_armed(false),
	_override_available(false),
	_cb_flighttermination(true),
	_in_esc_calibration_mode(false),
	_rssi_pwm_chan(0),
	_rssi_pwm_max(0),
	_rssi_pwm_min(0),
	_thermal_control(-1),
	_analog_rc_rssi_stable(false),
	_analog_rc_rssi_volt(-1.0f),
	_test_fmu_fail(false),
	_hitl_mode(false)
{
	/* we need this potentially before it could be set in task_main */
	g_dev = this;
}

PX4IO::~PX4IO()
{
	/* tell the task we want it to go away */
	_task_should_exit = true;

	/* spin waiting for the task to stop */
	for (unsigned i = 0; (i < 10) && (_task != -1); i++) {
		/* give it another 100ms */
		px4_usleep(100000);
	}

	/* well, kill it anyway, though this will probably crash */
	if (_task != -1) {
		task_delete(_task);
	}

	if (_interface != nullptr) {
		delete _interface;
	}

	/* deallocate perfs */
	perf_free(_perf_update);
	perf_free(_perf_write);
	perf_free(_perf_sample_latency);

	g_dev = nullptr;
}

int
PX4IO::detect()
{
	int ret;

	if (_task == -1) {

		/* do regular cdev init */
		ret = CDev::init();

		if (ret != OK) {
			return ret;
		}

		/* get some parameters */
		unsigned protocol = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_PROTOCOL_VERSION);

		if (protocol != PX4IO_PROTOCOL_VERSION) {
			if (protocol == _io_reg_get_error) {
				PX4_ERR("IO not installed");

			} else {
				PX4_ERR("IO version error");
				mavlink_log_emergency(&_mavlink_log_pub, "IO VERSION MISMATCH, PLEASE UPGRADE SOFTWARE!");
			}

			return -1;
		}
	}

	PX4_INFO("IO found");

	return 0;
}

int
PX4IO::init(bool rc_handling_disabled, bool hitl_mode)
{
	_rc_handling_disabled = rc_handling_disabled;
	_hitl_mode = hitl_mode;
	return init();
}

int
PX4IO::init()
{
	int ret;
	param_t sys_restart_param;
	int32_t sys_restart_val = DM_INIT_REASON_VOLATILE;

	sys_restart_param = param_find("SYS_RESTART_TYPE");

	if (sys_restart_param != PARAM_INVALID) {
		/* Indicate restart type is unknown */
		int32_t prev_val;
		param_get(sys_restart_param, &prev_val);

		if (prev_val != DM_INIT_REASON_POWER_ON) {
			param_set_no_notification(sys_restart_param, &sys_restart_val);
		}
	}

	/* do regular cdev init */
	ret = CDev::init();

	if (ret != OK) {
		return ret;
	}

	/* get some parameters */
	unsigned protocol;
	hrt_abstime start_try_time = hrt_absolute_time();

	do {
		px4_usleep(2000);
		protocol = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_PROTOCOL_VERSION);
	} while (protocol == _io_reg_get_error && (hrt_elapsed_time(&start_try_time) < 700U * 1000U));

	/* if the error still persists after timing out, we give up */
	if (protocol == _io_reg_get_error) {
		mavlink_log_emergency(&_mavlink_log_pub, "Failed to communicate with IO, abort.");
		return -1;
	}

	if (protocol != PX4IO_PROTOCOL_VERSION) {
		mavlink_log_emergency(&_mavlink_log_pub, "IO protocol/firmware mismatch, abort.");
		return -1;
	}

	_hardware      = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_HARDWARE_VERSION);
	_max_actuators = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_ACTUATOR_COUNT);
	_max_controls  = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_CONTROL_COUNT);
	_max_relays    = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_RELAY_COUNT);
	_max_transfer  = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_MAX_TRANSFER) - 2;
	_max_rc_input  = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_RC_INPUT_COUNT);

	if ((_max_actuators < 1) || (_max_actuators > 16) ||
	    (_max_relays > 32)   ||
	    (_max_transfer < 16) || (_max_transfer > 255)  ||
	    (_max_rc_input < 1)  || (_max_rc_input > 255)) {

		PX4_ERR("config read error");
		mavlink_log_emergency(&_mavlink_log_pub, "[IO] config read fail, abort.");

		// ask IO to reboot into bootloader as the failure may
		// be due to mismatched firmware versions and we want
		// the startup script to be able to load a new IO
		// firmware

		// If IO has already safety off it won't accept going into bootloader mode,
		// therefore we need to set safety on first.
		io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FORCE_SAFETY_ON, PX4IO_FORCE_SAFETY_MAGIC);

		// Now the reboot into bootloader mode should succeed.
		io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_REBOOT_BL, PX4IO_REBOOT_BL_MAGIC);
		return -1;
	}

	if (_max_rc_input > input_rc_s::RC_INPUT_MAX_CHANNELS) {
		_max_rc_input = input_rc_s::RC_INPUT_MAX_CHANNELS;
	}

	param_get(param_find("RC_RSSI_PWM_CHAN"), &_rssi_pwm_chan);
	param_get(param_find("RC_RSSI_PWM_MAX"), &_rssi_pwm_max);
	param_get(param_find("RC_RSSI_PWM_MIN"), &_rssi_pwm_min);

	/*
	 * Check for IO flight state - if FMU was flagged to be in
	 * armed state, FMU is recovering from an in-air reset.
	 * Read back status and request the commander to arm
	 * in this case.
	 */

	uint16_t reg;

	/* get IO's last seen FMU state */
	ret = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, &reg, sizeof(reg));

	if (ret != OK) {
		return ret;
	}

	/*
	 * in-air restart is only tried if the IO board reports it is
	 * already armed, and has been configured for in-air restart
	 */
	if ((reg & PX4IO_P_SETUP_ARMING_INAIR_RESTART_OK) &&
	    (reg & PX4IO_P_SETUP_ARMING_FMU_ARMED)) {

		/* get a status update from IO */
		io_get_status();

		mavlink_log_emergency(&_mavlink_log_pub, "RECOVERING FROM FMU IN-AIR RESTART");

		/* WARNING: COMMANDER app/vehicle status must be initialized.
		 * If this fails (or the app is not started), worst-case IO
		 * remains untouched (so manual override is still available).
		 */

		uORB::Subscription actuator_armed_sub{ORB_ID(actuator_armed)};

		/* fill with initial values, clear updated flag */
		actuator_armed_s actuator_armed{};
		uint64_t try_start_time = hrt_absolute_time();

		/* keep checking for an update, ensure we got a arming information,
		   not something that was published a long time ago. */
		do {
			if (actuator_armed_sub.update(&actuator_armed)) {
				// updated data, exit loop
				break;
			}

			/* wait 10 ms */
			px4_usleep(10000);

			/* abort after 5s */
			if ((hrt_absolute_time() - try_start_time) / 1000 > 3000) {
				mavlink_log_emergency(&_mavlink_log_pub, "Failed to recover from in-air restart (1), abort");
				return 1;
			}

		} while (true);

		/* send this to itself */
		param_t sys_id_param = param_find("MAV_SYS_ID");
		param_t comp_id_param = param_find("MAV_COMP_ID");

		int32_t sys_id;
		int32_t comp_id;

		if (param_get(sys_id_param, &sys_id)) {
			errx(1, "PRM SYSID");
		}

		if (param_get(comp_id_param, &comp_id)) {
			errx(1, "PRM CMPID");
		}

		/* prepare vehicle command */
		vehicle_command_s vcmd = {};
		vcmd.target_system = (uint8_t)sys_id;
		vcmd.target_component = (uint8_t)comp_id;
		vcmd.source_system = (uint8_t)sys_id;
		vcmd.source_component = (uint8_t)comp_id;
		vcmd.confirmation = true; /* ask to confirm command */

		if (reg & PX4IO_P_SETUP_ARMING_FORCE_FAILSAFE) {
			mavlink_log_emergency(&_mavlink_log_pub, "IO is in failsafe, force failsafe");
			/* send command to terminate flight via command API */
			vcmd.timestamp = hrt_absolute_time();
			vcmd.param1 = 1.0f; /* request flight termination */
			vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_FLIGHTTERMINATION;

			/* send command once */
			uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
			vcmd_pub.publish(vcmd);

			/* spin here until IO's state has propagated into the system */
			do {
				actuator_armed_sub.update(&actuator_armed);

				/* wait 50 ms */
				px4_usleep(50000);

				/* abort after 5s */
				if ((hrt_absolute_time() - try_start_time) / 1000 > 2000) {
					mavlink_log_emergency(&_mavlink_log_pub, "Failed to recover from in-air restart (3), abort");
					return 1;
				}

				/* re-send if necessary */
				if (!actuator_armed.force_failsafe) {
					vcmd_pub.publish(vcmd);
					PX4_WARN("re-sending flight termination cmd");
				}

				/* keep waiting for state change for 2 s */
			} while (!actuator_armed.force_failsafe);
		}

		/* send command to arm system via command API */
		vcmd.timestamp = hrt_absolute_time();
		vcmd.param1 = 1.0f; /* request arming */
		vcmd.param3 = 1234.f; /* mark the command coming from IO (for in-air restoring) */
		vcmd.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;

		/* send command once */
		uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
		vcmd_pub.publish(vcmd);

		/* spin here until IO's state has propagated into the system */
		do {
			actuator_armed_sub.update(&actuator_armed);

			/* wait 50 ms */
			px4_usleep(50000);

			/* abort after 5s */
			if ((hrt_absolute_time() - try_start_time) / 1000 > 2000) {
				mavlink_log_emergency(&_mavlink_log_pub, "Failed to recover from in-air restart (2), abort");
				return 1;
			}

			/* re-send if necessary */
			if (!actuator_armed.armed) {
				vcmd_pub.publish(vcmd);
				PX4_WARN("re-sending arm cmd");
			}

			/* keep waiting for state change for 2 s */
		} while (!actuator_armed.armed);

		/* Indicate restart type is in-flight */
		sys_restart_val = DM_INIT_REASON_IN_FLIGHT;
		int32_t prev_val;
		param_get(sys_restart_param, &prev_val);

		if (prev_val != sys_restart_val) {
			param_set(sys_restart_param, &sys_restart_val);
		}

		/* regular boot, no in-air restart, init IO */

	} else {

		/* dis-arm IO before touching anything */
		io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING,
			      PX4IO_P_SETUP_ARMING_FMU_ARMED |
			      PX4IO_P_SETUP_ARMING_INAIR_RESTART_OK |
			      PX4IO_P_SETUP_ARMING_MANUAL_OVERRIDE_OK |
			      PX4IO_P_SETUP_ARMING_ALWAYS_PWM_ENABLE |
			      PX4IO_P_SETUP_ARMING_LOCKDOWN, 0);

		if (_rc_handling_disabled) {
			ret = io_disable_rc_handling();

			if (ret != OK) {
				PX4_ERR("failed disabling RC handling");
				return ret;
			}

		} else {
			/* publish RC config to IO */
			ret = io_set_rc_config();

			if (ret != OK) {
				mavlink_log_critical(&_mavlink_log_pub, "IO RC config upload fail");
				return ret;
			}
		}

		/* Indicate restart type is power on */
		sys_restart_val = DM_INIT_REASON_POWER_ON;
		int32_t prev_val;
		param_get(sys_restart_param, &prev_val);

		if (prev_val != sys_restart_val) {
			param_set(sys_restart_param, &sys_restart_val);
		}

	}

	/* set safety to off if circuit breaker enabled */
	if (circuit_breaker_enabled("CBRK_IO_SAFETY", CBRK_IO_SAFETY_KEY)) {
		(void)io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FORCE_SAFETY_OFF, PX4IO_FORCE_SAFETY_MAGIC);
	}

	/* try to claim the generic PWM output device node as well - it's OK if we fail at this */
	ret = register_driver(PWM_OUTPUT0_DEVICE_PATH, &fops, 0666, (void *)this);

	if (ret == OK) {
		PX4_INFO("default PWM output device");
		_primary_pwm_device = true;
	}

	/* start the IO interface task */
	_task = px4_task_spawn_cmd("px4io",
				   SCHED_DEFAULT,
				   SCHED_PRIORITY_ACTUATOR_OUTPUTS,
				   1500,
				   (px4_main_t)&PX4IO::task_main_trampoline,
				   nullptr);

	if (_task < 0) {
		PX4_ERR("task start failed: %d", errno);
		return -errno;
	}

	return OK;
}

int
PX4IO::task_main_trampoline(int argc, char *argv[])
{
	g_dev->task_main();
	return 0;
}

void
PX4IO::task_main()
{
	hrt_abstime poll_last = 0;
	hrt_abstime orb_check_last = 0;

	/*
	 * Subscribe to the appropriate PWM output topic based on whether we are the
	 * primary PWM output or not.
	 */
	_t_actuator_controls_0 = orb_subscribe(ORB_ID(actuator_controls_0));
	orb_set_interval(_t_actuator_controls_0, 2);		/* default to 500Hz */

	if (_t_actuator_controls_0 < 0) {
		PX4_ERR("actuator subscription failed");
		goto out;
	}

	/* Fetch initial flight termination circuit breaker state */
	_cb_flighttermination = circuit_breaker_enabled("CBRK_FLIGHTTERM", CBRK_FLIGHTTERM_KEY);

	/* poll descriptor */
	pollfd fds[1];
	fds[0].fd = _t_actuator_controls_0;
	fds[0].events = POLLIN;

	_param_update_force = true;

	/* lock against the ioctl handler */
	lock();

	/* loop talking to IO */
	while (!_task_should_exit) {

		/* sleep waiting for topic updates, but no more than 20ms */
		unlock();
		int ret = ::poll(fds, 1, 20);
		lock();

		/* this would be bad... */
		if (ret < 0) {
			warnx("poll error %d", errno);
			continue;
		}

		perf_begin(_perf_update);
		hrt_abstime now = hrt_absolute_time();

		/* if we have new control data from the ORB, handle it */
		if (fds[0].revents & POLLIN) {

			/* we're not nice to the lower-priority control groups and only check them
			   when the primary group updated (which is now). */
			(void)io_set_control_groups();
		}

		if (!_armed && !_lockdown_override) {
			handle_motor_test();

		} else {
			_motor_test.in_test_mode = false;
		}

		if (now >= poll_last + IO_POLL_INTERVAL) {
			/* run at 50-250Hz */
			poll_last = now;

			/* pull status and alarms from IO */
			io_get_status();

			/* get raw R/C input from IO */
			io_publish_raw_rc();

			/* fetch PWM outputs from IO */
			io_publish_pwm_outputs();

			/* check updates on uORB topics and handle it */
			bool updated = false;

			/* arming state */
			updated = _t_actuator_armed.updated();

			if (!updated) {
				updated = _t_vehicle_control_mode.updated();
			}

			if (updated) {
				io_set_arming_state();
			}
		}

		if (!_armed && (now >= orb_check_last + ORB_CHECK_INTERVAL)) {
			/* run at 5Hz */
			orb_check_last = now;

			/* vehicle command */
			if (_t_vehicle_command.updated()) {
				vehicle_command_s cmd{};
				_t_vehicle_command.copy(&cmd);

				// Check for a DSM pairing command
				if (((unsigned int)cmd.command == vehicle_command_s::VEHICLE_CMD_START_RX_PAIR) && ((int)cmd.param1 == 0)) {
					int bind_arg;

					switch ((int)cmd.param2) {
					case 0:
						bind_arg = DSM2_BIND_PULSES;
						break;

					case 1:
						bind_arg = DSMX_BIND_PULSES;
						break;

					case 2:
					default:
						bind_arg = DSMX8_BIND_PULSES;
						break;
					}

					int dsm_ret = dsm_bind_ioctl(bind_arg);

					/* publish ACK */
					if (dsm_ret == OK) {
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);

					} else {
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_FAILED);
					}
				}
			}

			/*
			 * If parameters have changed, re-send RC mappings to IO
			 *
			 * XXX this may be a bit spammy
			 */

			// check for parameter updates
			if (_parameter_update_sub.updated() || _param_update_force) {
				// clear update
				parameter_update_s pupdate;
				_parameter_update_sub.copy(&pupdate);

				_param_update_force = false;

				if (!_rc_handling_disabled) {
					/* re-upload RC input config as it may have changed */
					io_set_rc_config();
				}

				/* send RC throttle failsafe value to IO */
				int32_t failsafe_param_val;
				param_t failsafe_param = param_find("RC_FAILS_THR");

				if (failsafe_param != PARAM_INVALID) {

					param_get(failsafe_param, &failsafe_param_val);

					if (failsafe_param_val > 0) {

						uint16_t failsafe_thr = failsafe_param_val;
						int pret = io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_RC_THR_FAILSAFE_US, &failsafe_thr, 1);

						if (pret != OK) {
							mavlink_log_critical(&_mavlink_log_pub, "failsafe upload failed, FS: %d us", (int)failsafe_thr);
						}
					}
				}

				/* Check if the IO safety circuit breaker has been updated */
				bool circuit_breaker_io_safety_enabled = circuit_breaker_enabled("CBRK_IO_SAFETY", CBRK_IO_SAFETY_KEY);
				/* Bypass IO safety switch logic by setting FORCE_SAFETY_OFF */
				(void)io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FORCE_SAFETY_OFF, circuit_breaker_io_safety_enabled);

				/* Check if the flight termination circuit breaker has been updated */
				_cb_flighttermination = circuit_breaker_enabled("CBRK_FLIGHTTERM", CBRK_FLIGHTTERM_KEY);
				/* Tell IO that it can terminate the flight if FMU is not responding or if a failure has been reported by the FailureDetector logic */
				(void)io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ENABLE_FLIGHTTERMINATION, !_cb_flighttermination);

				param_get(param_find("RC_RSSI_PWM_CHAN"), &_rssi_pwm_chan);
				param_get(param_find("RC_RSSI_PWM_MAX"), &_rssi_pwm_max);
				param_get(param_find("RC_RSSI_PWM_MIN"), &_rssi_pwm_min);

				param_t thermal_param = param_find("SENS_EN_THERMAL");

				if (thermal_param != PARAM_INVALID) {

					int32_t thermal_p;
					param_get(thermal_param, &thermal_p);

					if (thermal_p != _thermal_control || _param_update_force) {

						_thermal_control = thermal_p;
						/* set power management state for thermal */
						uint16_t tctrl;

						if (_thermal_control < 0) {
							tctrl = PX4IO_THERMAL_IGNORE;

						} else {
							tctrl = PX4IO_THERMAL_OFF;
						}

						ret = io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_THERMAL, tctrl);
					}
				}

				float param_val;
				param_t parm_handle;

				parm_handle = param_find("TRIM_ROLL");

				if (parm_handle != PARAM_INVALID) {
					param_get(parm_handle, &param_val);
					(void)io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_TRIM_ROLL, FLOAT_TO_REG(param_val));
				}

				parm_handle = param_find("TRIM_PITCH");

				if (parm_handle != PARAM_INVALID) {
					param_get(parm_handle, &param_val);
					(void)io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_TRIM_PITCH, FLOAT_TO_REG(param_val));
				}

				parm_handle = param_find("TRIM_YAW");

				if (parm_handle != PARAM_INVALID) {
					param_get(parm_handle, &param_val);
					(void)io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_TRIM_YAW, FLOAT_TO_REG(param_val));
				}

				parm_handle = param_find("FW_MAN_R_SC");

				if (parm_handle != PARAM_INVALID) {
					param_get(parm_handle, &param_val);
					(void)io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_SCALE_ROLL, FLOAT_TO_REG(param_val));
				}

				parm_handle = param_find("FW_MAN_P_SC");

				if (parm_handle != PARAM_INVALID) {
					param_get(parm_handle, &param_val);
					(void)io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_SCALE_PITCH, FLOAT_TO_REG(param_val));
				}

				parm_handle = param_find("FW_MAN_Y_SC");

				if (parm_handle != PARAM_INVALID) {
					param_get(parm_handle, &param_val);
					(void)io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_SCALE_YAW, FLOAT_TO_REG(param_val));
				}

				/* S.BUS output */
				int sbus_mode;
				parm_handle = param_find("PWM_SBUS_MODE");

				if (parm_handle != PARAM_INVALID) {
					param_get(parm_handle, &sbus_mode);

					if (sbus_mode == 1) {
						/* enable S.BUS 1 */
						(void)io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES, 0, PX4IO_P_SETUP_FEATURES_SBUS1_OUT);

					} else if (sbus_mode == 2) {
						/* enable S.BUS 2 */
						(void)io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES, 0, PX4IO_P_SETUP_FEATURES_SBUS2_OUT);

					} else {
						/* disable S.BUS */
						(void)io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES,
								    (PX4IO_P_SETUP_FEATURES_SBUS1_OUT | PX4IO_P_SETUP_FEATURES_SBUS2_OUT), 0);
					}
				}

				/* thrust to pwm modelling factor */
				parm_handle = param_find("THR_MDL_FAC");

				if (parm_handle != PARAM_INVALID) {
					param_get(parm_handle, &param_val);
					(void)io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_THR_MDL_FAC, FLOAT_TO_REG(param_val));
				}

				/* maximum motor pwm slew rate */
				parm_handle = param_find("MOT_SLEW_MAX");

				if (parm_handle != PARAM_INVALID) {
					param_get(parm_handle, &param_val);
					(void)io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_MOTOR_SLEW_MAX, FLOAT_TO_REG(param_val));
				}

				/* air-mode */
				parm_handle = param_find("MC_AIRMODE");

				if (parm_handle != PARAM_INVALID) {
					int32_t param_val_int;
					param_get(parm_handle, &param_val_int);
					(void)io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_AIRMODE, SIGNED_TO_REG(param_val_int));
				}

				update_params();
			}

		}

		perf_end(_perf_update);
	}

	unlock();

out:
	PX4_DEBUG("exiting");

	/* clean up the alternate device node */
	if (_primary_pwm_device) {
		unregister_driver(PWM_OUTPUT0_DEVICE_PATH);
	}

	/* tell the dtor that we are exiting */
	_task = -1;
	_exit(0);
}

void PX4IO::update_params()
{
	// skip update when armed
	if (_armed) {
		return;
	}

	int32_t pwm_min_default = PWM_DEFAULT_MIN;
	int32_t pwm_max_default = PWM_DEFAULT_MAX;
	int32_t pwm_disarmed_default = 0;
	int32_t pwm_rate_default = 50;

	const char *prefix = "PWM_MAIN";

	param_get(param_find("PWM_MAIN_MIN"), &pwm_min_default);
	param_get(param_find("PWM_MAIN_MAX"), &pwm_max_default);
	param_get(param_find("PWM_MAIN_DISARM"), &pwm_disarmed_default);
	param_get(param_find("PWM_MAIN_RATE"), &pwm_rate_default);

	char str[17];


	// PWM_MAIN_MINx
	{
		pwm_output_values pwm{};
		pwm.channel_count = _max_actuators;

		for (unsigned i = 0; i < _max_actuators; i++) {
			sprintf(str, "%s_MIN%u", prefix, i + 1);
			int32_t pwm_min = -1;

			if (param_get(param_find(str), &pwm_min) == PX4_OK) {
				if (pwm_min >= 0) {
					pwm.values[i] = math::constrain(pwm_min, PWM_LOWEST_MIN, PWM_HIGHEST_MIN);

					if (pwm_min != pwm.values[i]) {
						int32_t pwm_min_new = pwm.values[i];
						param_set(param_find(str), &pwm_min_new);
					}

				} else {
					pwm.values[i] = pwm_min_default;
				}
			}
		}

		io_reg_set(PX4IO_PAGE_CONTROL_MIN_PWM, 0, pwm.values, pwm.channel_count);
	}

	// PWM_MAIN_MAXx
	{
		pwm_output_values pwm{};
		pwm.channel_count = _max_actuators;

		for (unsigned i = 0; i < _max_actuators; i++) {
			sprintf(str, "%s_MAX%u", prefix, i + 1);
			int32_t pwm_max = -1;

			if (param_get(param_find(str), &pwm_max) == PX4_OK) {
				if (pwm_max >= 0) {
					pwm.values[i] = math::constrain(pwm_max, PWM_LOWEST_MAX, PWM_HIGHEST_MAX);

					if (pwm_max != pwm.values[i]) {
						int32_t pwm_max_new = pwm.values[i];
						param_set(param_find(str), &pwm_max_new);
					}

				} else {
					pwm.values[i] = pwm_max_default;
				}
			}
		}

		io_reg_set(PX4IO_PAGE_CONTROL_MAX_PWM, 0, pwm.values, pwm.channel_count);
	}

	// PWM_MAIN_FAILx
	{
		pwm_output_values pwm{};
		pwm.channel_count = _max_actuators;

		for (unsigned i = 0; i < _max_actuators; i++) {
			sprintf(str, "%s_FAIL%u", prefix, i + 1);
			int32_t pwm_fail = -1;

			if (param_get(param_find(str), &pwm_fail) == PX4_OK) {
				if (pwm_fail >= 0) {
					pwm.values[i] = math::constrain(pwm_fail, 0, PWM_HIGHEST_MAX);

					if (pwm_fail != pwm.values[i]) {
						int32_t pwm_fail_new = pwm.values[i];
						param_set(param_find(str), &pwm_fail_new);
					}
				}
			}
		}

		io_reg_set(PX4IO_PAGE_FAILSAFE_PWM, 0, pwm.values, pwm.channel_count);
	}

	// PWM_MAIN_DISx
	{
		pwm_output_values pwm{};
		pwm.channel_count = _max_actuators;

		for (unsigned i = 0; i < _max_actuators; i++) {
			sprintf(str, "%s_DIS%u", prefix, i + 1);
			int32_t pwm_dis = -1;

			if (param_get(param_find(str), &pwm_dis) == PX4_OK) {
				if (pwm_dis >= 0) {
					pwm.values[i] = math::constrain(pwm_dis, 0, PWM_HIGHEST_MAX);

					if (pwm_dis != pwm.values[i]) {
						int32_t pwm_dis_new = pwm.values[i];
						param_set(param_find(str), &pwm_dis_new);
					}

				} else {
					pwm.values[i] = pwm_disarmed_default;
				}
			}
		}

		io_reg_set(PX4IO_PAGE_DISARMED_PWM, 0, pwm.values, pwm.channel_count);
	}

	// PWM_MAIN_REVx
	{
		int16_t reverse_pwm_mask = 0;

		for (unsigned i = 0; i < _max_actuators; i++) {
			sprintf(str, "%s_REV%u", prefix, i + 1);
			int32_t pwm_rev = -1;

			if (param_get(param_find(str), &pwm_rev) == PX4_OK) {
				if (pwm_rev >= 1) {
					reverse_pwm_mask |= (1 << i);
				}

			}
		}

		io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_REVERSE, reverse_pwm_mask);
	}

	// PWM_MAIN_TRIMx
	{
		uint16_t values[8] {};

		for (unsigned i = 0; i < _max_actuators; i++) {
			sprintf(str, "%s_TRIM%u", prefix, i + 1);
			float pwm_trim = 0.f;

			if (param_get(param_find(str), &pwm_trim) == PX4_OK) {
				values[i] = (int16_t)(10000 * pwm_trim);
			}
		}

		// copy the trim values to the mixer offsets
		io_reg_set(PX4IO_PAGE_CONTROL_TRIM_PWM, 0, values, _max_actuators);
	}
}

int
PX4IO::io_set_control_groups()
{
	int ret = io_set_control_state(0);

	/* send auxiliary control groups */
	(void)io_set_control_state(1);
	(void)io_set_control_state(2);
	(void)io_set_control_state(3);

	return ret;
}

int
PX4IO::io_set_control_state(unsigned group)
{
	actuator_controls_s	controls{};	///< actuator outputs

	/* get controls */
	bool changed = false;

	switch (group) {
	case 0: {
			orb_check(_t_actuator_controls_0, &changed);

			if (changed) {
				orb_copy(ORB_ID(actuator_controls_0), _t_actuator_controls_0, &controls);
				perf_set_elapsed(_perf_sample_latency, hrt_elapsed_time(&controls.timestamp_sample));
			}
		}
		break;

	case 1:
		changed = _t_actuator_controls_1.update(&controls);
		break;

	case 2:
		changed = _t_actuator_controls_2.update(&controls);
		break;

	case 3:
		changed = _t_actuator_controls_3.update(&controls);
		break;

	}

	if (!changed && (!_in_esc_calibration_mode || group != 0)) {
		return -1;

	} else if (_in_esc_calibration_mode && group == 0) {
		/* modify controls to get max pwm (full thrust) on every esc */
		memset(&controls, 0, sizeof(controls));

		/* set maximum thrust */
		controls.control[3] = 1.0f;
	}

	uint16_t regs[sizeof(controls.control) / sizeof(controls.control[0])] = {};

	for (unsigned i = 0; (i < _max_controls) && (i < sizeof(controls.control) / sizeof(controls.control[0])); i++) {
		/* ensure FLOAT_TO_REG does not produce an integer overflow */
		const float ctrl = math::constrain(controls.control[i], -1.0f, 1.0f);

		if (!isfinite(ctrl)) {
			regs[i] = INT16_MAX;

		} else {
			regs[i] = FLOAT_TO_REG(ctrl);
		}

	}

	if (!_test_fmu_fail && !_motor_test.in_test_mode) {
		/* copy values to registers in IO */
		return io_reg_set(PX4IO_PAGE_CONTROLS, group * PX4IO_PROTOCOL_MAX_CONTROL_COUNT, regs, math::min(_max_controls,
				  sizeof(controls.control) / sizeof(controls.control[0])));

	} else {
		return OK;
	}
}

void
PX4IO::answer_command(const vehicle_command_s &cmd, uint8_t result)
{
	/* publish ACK */
	uORB::Publication<vehicle_command_ack_s> vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};
	vehicle_command_ack_s command_ack{};
	command_ack.command = cmd.command;
	command_ack.result = result;
	command_ack.target_system = cmd.source_system;
	command_ack.target_component = cmd.source_component;
	command_ack.timestamp = hrt_absolute_time();
	vehicle_command_ack_pub.publish(command_ack);
}

void
PX4IO::handle_motor_test()
{
	test_motor_s test_motor;

	while (_motor_test.test_motor_sub.update(&test_motor)) {
		if (test_motor.driver_instance != 0 ||
		    hrt_elapsed_time(&test_motor.timestamp) > 100_ms) {
			continue;
		}

		bool in_test_mode = test_motor.action == test_motor_s::ACTION_RUN;

		if (in_test_mode != _motor_test.in_test_mode) {
			// reset all outputs to disarmed on state change
			pwm_output_values pwm_disarmed;

			if (io_reg_get(PX4IO_PAGE_DISARMED_PWM, 0, pwm_disarmed.values, _max_actuators) == 0) {
				for (unsigned i = 0; i < _max_actuators; ++i) {
					io_reg_set(PX4IO_PAGE_DIRECT_PWM, i, pwm_disarmed.values[i]);
				}
			}
		}

		if (in_test_mode) {
			unsigned idx = test_motor.motor_number;

			if (idx < _max_actuators) {
				if (test_motor.value < 0.f) {
					pwm_output_values pwm_disarmed;

					if (io_reg_get(PX4IO_PAGE_DISARMED_PWM, 0, pwm_disarmed.values, _max_actuators) == 0) {
						io_reg_set(PX4IO_PAGE_DIRECT_PWM, idx, pwm_disarmed.values[idx]);
					}

				} else {
					pwm_output_values pwm_min;
					pwm_output_values pwm_max;

					if (io_reg_get(PX4IO_PAGE_CONTROL_MIN_PWM, 0, pwm_min.values, _max_actuators) == 0 &&
					    io_reg_get(PX4IO_PAGE_CONTROL_MAX_PWM, 0, pwm_max.values, _max_actuators) == 0) {

						uint16_t value = math::constrain<uint16_t>(pwm_min.values[idx] +
								 (uint16_t)((pwm_max.values[idx] - pwm_min.values[idx]) * test_motor.value),
								 pwm_min.values[idx], pwm_max.values[idx]);
						io_reg_set(PX4IO_PAGE_DIRECT_PWM, idx, value);
					}
				}
			}

			if (test_motor.timeout_ms > 0) {
				_motor_test.timeout = test_motor.timestamp + test_motor.timeout_ms * 1000;

			} else {
				_motor_test.timeout = 0;
			}
		}

		_motor_test.in_test_mode = in_test_mode;
	}

	// check for timeouts
	if (_motor_test.timeout != 0 && hrt_absolute_time() > _motor_test.timeout) {
		_motor_test.in_test_mode = false;
		_motor_test.timeout = 0;
	}
}

int
PX4IO::io_set_arming_state()
{
	uint16_t set = 0;
	uint16_t clear = 0;

	actuator_armed_s armed;

	if (_t_actuator_armed.copy(&armed)) {
		_in_esc_calibration_mode = armed.in_esc_calibration_mode;

		if (armed.armed || _in_esc_calibration_mode) {
			set |= PX4IO_P_SETUP_ARMING_FMU_ARMED;

		} else {
			clear |= PX4IO_P_SETUP_ARMING_FMU_ARMED;
		}

		_armed = armed.armed;

		if (armed.prearmed) {
			set |= PX4IO_P_SETUP_ARMING_FMU_PREARMED;

		} else {
			clear |= PX4IO_P_SETUP_ARMING_FMU_PREARMED;
		}

		if ((armed.lockdown || armed.manual_lockdown) && !_lockdown_override) {
			set |= PX4IO_P_SETUP_ARMING_LOCKDOWN;
			_lockdown_override = true;

		} else if (!(armed.lockdown || armed.manual_lockdown) && _lockdown_override) {
			clear |= PX4IO_P_SETUP_ARMING_LOCKDOWN;
			_lockdown_override = false;
		}

		if (armed.force_failsafe) {
			set |= PX4IO_P_SETUP_ARMING_FORCE_FAILSAFE;

		} else {
			clear |= PX4IO_P_SETUP_ARMING_FORCE_FAILSAFE;
		}

		// XXX this is for future support in the commander
		// but can be removed if unneeded
		// if (armed.termination_failsafe) {
		// 	set |= PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE;
		// } else {
		// 	clear |= PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE;
		// }

		if (armed.ready_to_arm) {
			set |= PX4IO_P_SETUP_ARMING_IO_ARM_OK;

		} else {
			clear |= PX4IO_P_SETUP_ARMING_IO_ARM_OK;
		}
	}

	vehicle_control_mode_s control_mode;

	if (_t_vehicle_control_mode.copy(&control_mode)) {
		if (control_mode.flag_external_manual_override_ok) {
			set |= PX4IO_P_SETUP_ARMING_MANUAL_OVERRIDE_OK;
			_override_available = true;

		} else {
			clear |= PX4IO_P_SETUP_ARMING_MANUAL_OVERRIDE_OK;
			_override_available = false;
		}
	}

	if (_last_written_arming_s != set || _last_written_arming_c != clear) {
		_last_written_arming_s = set;
		_last_written_arming_c = clear;
		return io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, clear, set);
	}

	return 0;
}

int
PX4IO::disable_rc_handling()
{
	_rc_handling_disabled = true;
	return io_disable_rc_handling();
}

int
PX4IO::io_disable_rc_handling()
{
	uint16_t set = PX4IO_P_SETUP_ARMING_RC_HANDLING_DISABLED;
	uint16_t clear = 0;

	return io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, clear, set);
}

int
PX4IO::io_set_rc_config()
{
	unsigned offset = 0;
	int input_map[_max_rc_input];
	int32_t ichan;
	int ret = OK;

	/*
	 * Generate the input channel -> control channel mapping table;
	 * assign RC_MAP_ROLL/PITCH/YAW/THROTTLE to the canonical
	 * controls.
	 */

	/* fill the mapping with an error condition triggering value */
	for (unsigned i = 0; i < _max_rc_input; i++) {
		input_map[i] = UINT8_MAX;
	}

	/*
	 * NOTE: The indices for mapped channels are 1-based
	 *       for compatibility reasons with existing
	 *       autopilots / GCS'.
	 */

	/* ROLL */
	param_get(param_find("RC_MAP_ROLL"), &ichan);

	if ((ichan > 0) && (ichan <= (int)_max_rc_input)) {
		input_map[ichan - 1] = 0;
	}

	/* PITCH */
	param_get(param_find("RC_MAP_PITCH"), &ichan);

	if ((ichan > 0) && (ichan <= (int)_max_rc_input)) {
		input_map[ichan - 1] = 1;
	}

	/* YAW */
	param_get(param_find("RC_MAP_YAW"), &ichan);

	if ((ichan > 0) && (ichan <= (int)_max_rc_input)) {
		input_map[ichan - 1] = 2;
	}

	/* THROTTLE */
	param_get(param_find("RC_MAP_THROTTLE"), &ichan);

	if ((ichan > 0) && (ichan <= (int)_max_rc_input)) {
		input_map[ichan - 1] = 3;
	}

	/* FLAPS */
	param_get(param_find("RC_MAP_FLAPS"), &ichan);

	if ((ichan > 0) && (ichan <= (int)_max_rc_input)) {
		input_map[ichan - 1] = 4;
	}

	/* AUX 1*/
	param_get(param_find("RC_MAP_AUX1"), &ichan);

	if ((ichan > 0) && (ichan <= (int)_max_rc_input)) {
		input_map[ichan - 1] = 5;
	}

	/* AUX 2*/
	param_get(param_find("RC_MAP_AUX2"), &ichan);

	if ((ichan > 0) && (ichan <= (int)_max_rc_input)) {
		input_map[ichan - 1] = 6;
	}

	/* AUX 3*/
	param_get(param_find("RC_MAP_AUX3"), &ichan);

	if ((ichan > 0) && (ichan <= (int)_max_rc_input)) {
		input_map[ichan - 1] = 7;
	}

	/* MAIN MODE SWITCH */
	param_get(param_find("RC_MAP_MODE_SW"), &ichan);

	if ((ichan > 0) && (ichan <= (int)_max_rc_input)) {
		/* use out of normal bounds index to indicate special channel */
		input_map[ichan - 1] = PX4IO_P_RC_CONFIG_ASSIGNMENT_MODESWITCH;
	}

	/*
	 * Iterate all possible RC inputs.
	 */
	for (unsigned i = 0; i < _max_rc_input; i++) {
		uint16_t regs[PX4IO_P_RC_CONFIG_STRIDE];
		char pname[16];
		float fval;

		/*
		 * RC params are floats, but do only
		 * contain integer values. Do not scale
		 * or cast them, let the auto-typeconversion
		 * do its job here.
		 * Channels: 500 - 2500
		 * Inverted flag: -1 (inverted) or 1 (normal)
		 */

		sprintf(pname, "RC%u_MIN", i + 1);
		param_get(param_find(pname), &fval);
		regs[PX4IO_P_RC_CONFIG_MIN] = fval;

		sprintf(pname, "RC%u_TRIM", i + 1);
		param_get(param_find(pname), &fval);
		regs[PX4IO_P_RC_CONFIG_CENTER] = fval;

		sprintf(pname, "RC%u_MAX", i + 1);
		param_get(param_find(pname), &fval);
		regs[PX4IO_P_RC_CONFIG_MAX] = fval;

		sprintf(pname, "RC%u_DZ", i + 1);
		param_get(param_find(pname), &fval);
		regs[PX4IO_P_RC_CONFIG_DEADZONE] = fval;

		regs[PX4IO_P_RC_CONFIG_ASSIGNMENT] = input_map[i];

		regs[PX4IO_P_RC_CONFIG_OPTIONS] = PX4IO_P_RC_CONFIG_OPTIONS_ENABLED;
		sprintf(pname, "RC%u_REV", i + 1);
		param_get(param_find(pname), &fval);

		/*
		 * This has been taken for the sake of compatibility
		 * with APM's setup / mission planner: normal: 1,
		 * inverted: -1
		 */
		if (fval < 0) {
			regs[PX4IO_P_RC_CONFIG_OPTIONS] |= PX4IO_P_RC_CONFIG_OPTIONS_REVERSE;
		}

		/* send channel config to IO */
		ret = io_reg_set(PX4IO_PAGE_RC_CONFIG, offset, regs, PX4IO_P_RC_CONFIG_STRIDE);

		if (ret != OK) {
			PX4_ERR("rc config upload failed");
			break;
		}

		/* check the IO initialisation flag */
		if (!(io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FLAGS) & PX4IO_P_STATUS_FLAGS_INIT_OK)) {
			mavlink_log_critical(&_mavlink_log_pub, "config for RC%u rejected by IO", i + 1);
			break;
		}

		offset += PX4IO_P_RC_CONFIG_STRIDE;
	}

	return ret;
}

int
PX4IO::io_handle_status(uint16_t status)
{
	int ret = 1;
	/**
	 * WARNING: This section handles in-air resets.
	 */

	/* check for IO reset - force it back to armed if necessary */
	if (_status & PX4IO_P_STATUS_FLAGS_SAFETY_OFF && !(status & PX4IO_P_STATUS_FLAGS_SAFETY_OFF)
	    && !(status & PX4IO_P_STATUS_FLAGS_ARM_SYNC)) {
		/* set the arming flag */
		ret = io_reg_modify(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FLAGS, 0,
				    PX4IO_P_STATUS_FLAGS_SAFETY_OFF | PX4IO_P_STATUS_FLAGS_ARM_SYNC);

		/* set new status */
		_status = status;
		_status &= PX4IO_P_STATUS_FLAGS_SAFETY_OFF;

	} else if (!(_status & PX4IO_P_STATUS_FLAGS_ARM_SYNC)) {

		/* set the sync flag */
		ret = io_reg_modify(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FLAGS, 0, PX4IO_P_STATUS_FLAGS_ARM_SYNC);
		/* set new status */
		_status = status;

	} else {
		ret = 0;

		/* set new status */
		_status = status;
	}

	/**
	 * Get and handle the safety status
	 */
	const bool safety_off = status & PX4IO_P_STATUS_FLAGS_SAFETY_OFF;
	const bool override_enabled = status & PX4IO_P_STATUS_FLAGS_OVERRIDE;

	// publish immediately on change, otherwise at 1 Hz
	if ((hrt_elapsed_time(&_safety.timestamp) >= 1_s)
	    || (_safety.safety_off != safety_off)
	    || (_safety.override_available != _override_available)
	    || (_safety.override_enabled != override_enabled)) {

		_safety.safety_switch_available = true;
		_safety.safety_off = safety_off;
		_safety.override_available = _override_available;
		_safety.override_enabled = override_enabled;
		_safety.timestamp = hrt_absolute_time();

		_to_safety.publish(_safety);
	}

	return ret;
}

int
PX4IO::dsm_bind_ioctl(int dsmMode)
{
	// Do not bind if invalid pulses are provided
	if (dsmMode != DSM2_BIND_PULSES &&
	    dsmMode != DSMX_BIND_PULSES &&
	    dsmMode != DSMX8_BIND_PULSES) {
		PX4_ERR("Unknown DSM mode: %d", dsmMode);
		return -EINVAL;
	}

	// Do not bind if armed
	bool armed = (_status & PX4IO_P_SETUP_ARMING_FMU_ARMED);

	if (armed) {
		PX4_ERR("Not binding DSM, system is armed.");
		return -EINVAL;
	}

	// Check if safety was off
	bool safety_off = (_status & PX4IO_P_STATUS_FLAGS_SAFETY_OFF);
	int ret = -1;

	// Put safety on
	if (safety_off) {
		// re-enable safety
		ret = io_reg_modify(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FLAGS, PX4IO_P_STATUS_FLAGS_SAFETY_OFF, 0);

		// set new status
		_status &= ~(PX4IO_P_STATUS_FLAGS_SAFETY_OFF);
	}

	PX4_INFO("Binding DSM%s RX", (dsmMode == DSM2_BIND_PULSES) ? "2" : ((dsmMode == DSMX_BIND_PULSES) ? "-X" : "-X8"));

	io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_DSM, dsm_bind_power_down);
	px4_usleep(500000);
	io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_DSM, dsm_bind_set_rx_out);
	io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_DSM, dsm_bind_power_up);
	px4_usleep(72000);
	io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_DSM, dsm_bind_send_pulses | (dsmMode << 4));
	px4_usleep(50000);
	io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_DSM, dsm_bind_reinit_uart);
	ret = OK;

	// Put safety back off
	if (safety_off) {
		ret = io_reg_modify(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FLAGS, 0,
				    PX4IO_P_STATUS_FLAGS_SAFETY_OFF);
		_status |= PX4IO_P_STATUS_FLAGS_SAFETY_OFF;
	}

	if (ret != OK) {
		PX4_INFO("Binding DSM failed");
	}

	return ret;
}

int
PX4IO::io_get_status()
{
	/* get
	 * STATUS_FLAGS, STATUS_ALARMS, STATUS_VBATT, STATUS_IBATT,
	 * STATUS_VSERVO, STATUS_VRSSI
	 * in that order */
	uint16_t regs[6] {};
	int ret = io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FLAGS, &regs[0], sizeof(regs) / sizeof(regs[0]));

	if (ret != OK) {
		return ret;
	}

	const uint16_t STATUS_FLAGS  = regs[0];
	const uint16_t STATUS_ALARMS = regs[1];
	const uint16_t STATUS_VSERVO = regs[4];
	const uint16_t STATUS_VRSSI  = regs[5];

	io_handle_status(STATUS_FLAGS);

	const float rssi_v = STATUS_VRSSI * 0.001f; // voltage is scaled to mV

	if (_analog_rc_rssi_volt < 0.f) {
		_analog_rc_rssi_volt = rssi_v;
	}

	_analog_rc_rssi_volt = _analog_rc_rssi_volt * 0.99f + rssi_v * 0.01f;

	if (_analog_rc_rssi_volt > 2.5f) {
		_analog_rc_rssi_stable = true;
	}

	const uint16_t SETUP_ARMING = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING);

	if ((hrt_elapsed_time(&_last_status_publish) >= 1_s)
	    || (_status != STATUS_FLAGS)
	    || (_alarms != STATUS_ALARMS)
	    || (_setup_arming != SETUP_ARMING)
	   ) {

		px4io_status_s status{};

		status.voltage_v = STATUS_VSERVO * 0.001f; // voltage is scaled to mV
		status.rssi_v = rssi_v;

		status.free_memory_bytes = io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FREEMEM);

		// PX4IO_P_STATUS_FLAGS
		status.status_outputs_armed   = STATUS_FLAGS & PX4IO_P_STATUS_FLAGS_OUTPUTS_ARMED;
		status.status_override        = STATUS_FLAGS & PX4IO_P_STATUS_FLAGS_OVERRIDE;
		status.status_rc_ok           = STATUS_FLAGS & PX4IO_P_STATUS_FLAGS_RC_OK;
		status.status_rc_ppm          = STATUS_FLAGS & PX4IO_P_STATUS_FLAGS_RC_PPM;
		status.status_rc_dsm          = STATUS_FLAGS & PX4IO_P_STATUS_FLAGS_RC_DSM;
		status.status_rc_sbus         = STATUS_FLAGS & PX4IO_P_STATUS_FLAGS_RC_SBUS;
		status.status_fmu_ok          = STATUS_FLAGS & PX4IO_P_STATUS_FLAGS_FMU_OK;
		status.status_raw_pwm         = STATUS_FLAGS & PX4IO_P_STATUS_FLAGS_RAW_PWM;
		status.status_mixer_ok        = STATUS_FLAGS & PX4IO_P_STATUS_FLAGS_MIXER_OK;
		status.status_arm_sync        = STATUS_FLAGS & PX4IO_P_STATUS_FLAGS_ARM_SYNC;
		status.status_init_ok         = STATUS_FLAGS & PX4IO_P_STATUS_FLAGS_INIT_OK;
		status.status_failsafe        = STATUS_FLAGS & PX4IO_P_STATUS_FLAGS_FAILSAFE;
		status.status_safety_off      = STATUS_FLAGS & PX4IO_P_STATUS_FLAGS_SAFETY_OFF;
		status.status_fmu_initialized = STATUS_FLAGS & PX4IO_P_STATUS_FLAGS_FMU_INITIALIZED;
		status.status_rc_st24         = STATUS_FLAGS & PX4IO_P_STATUS_FLAGS_RC_ST24;
		status.status_rc_sumd         = STATUS_FLAGS & PX4IO_P_STATUS_FLAGS_RC_SUMD;

		// PX4IO_P_STATUS_ALARMS
		status.alarm_vbatt_low     = STATUS_ALARMS & PX4IO_P_STATUS_ALARMS_VBATT_LOW;
		status.alarm_temperature   = STATUS_ALARMS & PX4IO_P_STATUS_ALARMS_TEMPERATURE;
		status.alarm_servo_current = STATUS_ALARMS & PX4IO_P_STATUS_ALARMS_SERVO_CURRENT;
		status.alarm_acc_current   = STATUS_ALARMS & PX4IO_P_STATUS_ALARMS_ACC_CURRENT;
		status.alarm_fmu_lost      = STATUS_ALARMS & PX4IO_P_STATUS_ALARMS_FMU_LOST;
		status.alarm_rc_lost       = STATUS_ALARMS & PX4IO_P_STATUS_ALARMS_RC_LOST;
		status.alarm_pwm_error     = STATUS_ALARMS & PX4IO_P_STATUS_ALARMS_PWM_ERROR;
		status.alarm_vservo_fault  = STATUS_ALARMS & PX4IO_P_STATUS_ALARMS_VSERVO_FAULT;

		// PX4IO_P_SETUP_ARMING
		status.arming_io_arm_ok            = SETUP_ARMING & PX4IO_P_SETUP_ARMING_IO_ARM_OK;
		status.arming_fmu_armed            = SETUP_ARMING & PX4IO_P_SETUP_ARMING_FMU_ARMED;
		status.arming_fmu_prearmed         = SETUP_ARMING & PX4IO_P_SETUP_ARMING_FMU_PREARMED;
		status.arming_manual_override_ok   = SETUP_ARMING & PX4IO_P_SETUP_ARMING_MANUAL_OVERRIDE_OK;
		status.arming_failsafe_custom      = SETUP_ARMING & PX4IO_P_SETUP_ARMING_FAILSAFE_CUSTOM;
		status.arming_inair_restart_ok     = SETUP_ARMING & PX4IO_P_SETUP_ARMING_INAIR_RESTART_OK;
		status.arming_always_pwm_enable    = SETUP_ARMING & PX4IO_P_SETUP_ARMING_ALWAYS_PWM_ENABLE;
		status.arming_rc_handling_disabled = SETUP_ARMING & PX4IO_P_SETUP_ARMING_RC_HANDLING_DISABLED;
		status.arming_lockdown             = SETUP_ARMING & PX4IO_P_SETUP_ARMING_LOCKDOWN;
		status.arming_force_failsafe       = SETUP_ARMING & PX4IO_P_SETUP_ARMING_FORCE_FAILSAFE;
		status.arming_termination_failsafe = SETUP_ARMING & PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE;
		status.arming_override_immediate   = SETUP_ARMING & PX4IO_P_SETUP_ARMING_OVERRIDE_IMMEDIATE;


		for (unsigned i = 0; i < _max_actuators; i++) {
			status.actuators[i] = static_cast<int16_t>(io_reg_get(PX4IO_PAGE_ACTUATORS, i));
			status.servos[i] = io_reg_get(PX4IO_PAGE_SERVOS, i);
		}

		uint16_t raw_inputs = io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_COUNT);

		for (unsigned i = 0; i < raw_inputs; i++) {
			status.raw_inputs[i] = io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_BASE + i);
		}

		status.timestamp = hrt_absolute_time();
		_px4io_status_pub.publish(status);

		_last_status_publish = status.timestamp;
	}

	_alarms = STATUS_ALARMS;
	_setup_arming = SETUP_ARMING;

	return ret;
}

int
PX4IO::io_get_raw_rc_input(input_rc_s &input_rc)
{
	uint32_t channel_count;
	int	ret;

	/* we don't have the status bits, so input_source has to be set elsewhere */
	input_rc.input_source = input_rc_s::RC_INPUT_SOURCE_UNKNOWN;

	const unsigned prolog = (PX4IO_P_RAW_RC_BASE - PX4IO_P_RAW_RC_COUNT);
	uint16_t regs[input_rc_s::RC_INPUT_MAX_CHANNELS + prolog];

	/*
	 * Read the channel count and the first 9 channels.
	 *
	 * This should be the common case (9 channel R/C control being a reasonable upper bound).
	 */
	ret = io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_COUNT, &regs[0], prolog + 9);

	if (ret != OK) {
		return ret;
	}

	/*
	 * Get the channel count any any extra channels. This is no more expensive than reading the
	 * channel count once.
	 */
	channel_count = regs[PX4IO_P_RAW_RC_COUNT];

	/* limit the channel count */
	if (channel_count > input_rc_s::RC_INPUT_MAX_CHANNELS) {
		channel_count = input_rc_s::RC_INPUT_MAX_CHANNELS;
	}

	_rc_chan_count = channel_count;

	input_rc.timestamp = hrt_absolute_time();

	input_rc.rc_ppm_frame_length = regs[PX4IO_P_RAW_RC_DATA];

	if (!_analog_rc_rssi_stable) {
		input_rc.rssi = regs[PX4IO_P_RAW_RC_NRSSI];

	} else {
		float rssi_analog = ((_analog_rc_rssi_volt - 0.2f) / 3.0f) * 100.0f;

		if (rssi_analog > 100.0f) {
			rssi_analog = 100.0f;
		}

		if (rssi_analog < 0.0f) {
			rssi_analog = 0.0f;
		}

		input_rc.rssi = rssi_analog;
	}

	input_rc.rc_failsafe = (regs[PX4IO_P_RAW_RC_FLAGS] & PX4IO_P_RAW_RC_FLAGS_FAILSAFE);
	input_rc.rc_lost = !(regs[PX4IO_P_RAW_RC_FLAGS] & PX4IO_P_RAW_RC_FLAGS_RC_OK);
	input_rc.rc_lost_frame_count = regs[PX4IO_P_RAW_LOST_FRAME_COUNT];
	input_rc.rc_total_frame_count = regs[PX4IO_P_RAW_FRAME_COUNT];
	input_rc.channel_count = channel_count;

	/* rc_lost has to be set before the call to this function */
	if (!input_rc.rc_lost && !input_rc.rc_failsafe) {
		_rc_last_valid = input_rc.timestamp;
	}

	input_rc.timestamp_last_signal = _rc_last_valid;

	/* FIELDS NOT SET HERE */
	/* input_rc.input_source is set after this call XXX we might want to mirror the flags in the RC struct */

	if (channel_count > 9) {
		ret = io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_BASE + 9, &regs[prolog + 9], channel_count - 9);

		if (ret != OK) {
			return ret;
		}
	}

	/* last thing set are the actual channel values as 16 bit values */
	for (unsigned i = 0; i < channel_count; i++) {
		input_rc.values[i] = regs[prolog + i];
	}

	/* zero the remaining fields */
	for (unsigned i = channel_count; i < (sizeof(input_rc.values) / sizeof(input_rc.values[0])); i++) {
		input_rc.values[i] = 0;
	}

	/* get RSSI from input channel */
	if (_rssi_pwm_chan > 0 && _rssi_pwm_chan <= input_rc_s::RC_INPUT_MAX_CHANNELS && _rssi_pwm_max - _rssi_pwm_min != 0) {
		int rssi = ((input_rc.values[_rssi_pwm_chan - 1] - _rssi_pwm_min) * 100) /
			   (_rssi_pwm_max - _rssi_pwm_min);
		rssi = rssi > 100 ? 100 : rssi;
		rssi = rssi < 0 ? 0 : rssi;
		input_rc.rssi = rssi;
	}

	return ret;
}

int
PX4IO::io_publish_raw_rc()
{

	/* fetch values from IO */
	input_rc_s	rc_val;

	/* set the RC status flag ORDER MATTERS! */
	rc_val.rc_lost = !(_status & PX4IO_P_STATUS_FLAGS_RC_OK);

	int ret = io_get_raw_rc_input(rc_val);

	if (ret != OK) {
		return ret;
	}

	/* sort out the source of the values */
	if (_status & PX4IO_P_STATUS_FLAGS_RC_PPM) {
		rc_val.input_source = input_rc_s::RC_INPUT_SOURCE_PX4IO_PPM;

	} else if (_status & PX4IO_P_STATUS_FLAGS_RC_DSM) {
		rc_val.input_source = input_rc_s::RC_INPUT_SOURCE_PX4IO_SPEKTRUM;

	} else if (_status & PX4IO_P_STATUS_FLAGS_RC_SBUS) {
		rc_val.input_source = input_rc_s::RC_INPUT_SOURCE_PX4IO_SBUS;

	} else if (_status & PX4IO_P_STATUS_FLAGS_RC_ST24) {
		rc_val.input_source = input_rc_s::RC_INPUT_SOURCE_PX4IO_ST24;

	} else {
		rc_val.input_source = input_rc_s::RC_INPUT_SOURCE_UNKNOWN;

		/* only keep publishing RC input if we ever got a valid input */
		if (_rc_last_valid == 0) {
			/* we have never seen valid RC signals, abort */
			return OK;
		}
	}

	_to_input_rc.publish(rc_val);

	return OK;
}

int
PX4IO::io_publish_pwm_outputs()
{
	if (_hitl_mode) {
		return OK;
	}

	/* get servo values from IO */
	uint16_t ctl[_max_actuators];
	int ret = io_reg_get(PX4IO_PAGE_SERVOS, 0, ctl, _max_actuators);

	if (ret != OK) {
		return ret;
	}

	actuator_outputs_s outputs = {};
	outputs.timestamp = hrt_absolute_time();
	outputs.noutputs = _max_actuators;

	/* convert from register format to float */
	for (unsigned i = 0; i < _max_actuators; i++) {
		outputs.output[i] = ctl[i];
	}

	_to_outputs.publish(outputs);

	/* get mixer status flags from IO */
	MultirotorMixer::saturation_status saturation_status;
	ret = io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_MIXER, &saturation_status.value, 1);

	if (ret != OK) {
		return ret;
	}

	/* publish mixer status */
	if (saturation_status.flags.valid) {
		multirotor_motor_limits_s motor_limits{};
		motor_limits.timestamp = hrt_absolute_time();
		motor_limits.saturation_status = saturation_status.value;

		_to_mixer_status.publish(motor_limits);
	}

	return OK;
}

int
PX4IO::io_reg_set(uint8_t page, uint8_t offset, const uint16_t *values, unsigned num_values)
{
	/* range check the transfer */
	if (num_values > ((_max_transfer) / sizeof(*values))) {
		PX4_DEBUG("io_reg_set: too many registers (%u, max %u)", num_values, _max_transfer / 2);
		return -EINVAL;
	}

	int ret =  _interface->write((page << 8) | offset, (void *)values, num_values);

	if (ret != (int)num_values) {
		PX4_DEBUG("io_reg_set(%hhu,%hhu,%u): error %d", page, offset, num_values, ret);
		return -1;
	}

	return OK;
}

int
PX4IO::io_reg_set(uint8_t page, uint8_t offset, uint16_t value)
{
	return io_reg_set(page, offset, &value, 1);
}

int
PX4IO::io_reg_get(uint8_t page, uint8_t offset, uint16_t *values, unsigned num_values)
{
	/* range check the transfer */
	if (num_values > ((_max_transfer) / sizeof(*values))) {
		PX4_DEBUG("io_reg_get: too many registers (%u, max %u)", num_values, _max_transfer / 2);
		return -EINVAL;
	}

	int ret = _interface->read((page << 8) | offset, reinterpret_cast<void *>(values), num_values);

	if (ret != (int)num_values) {
		PX4_DEBUG("io_reg_get(%hhu,%hhu,%u): data error %d", page, offset, num_values, ret);
		return -1;
	}

	return OK;
}

uint32_t
PX4IO::io_reg_get(uint8_t page, uint8_t offset)
{
	uint16_t value;

	if (io_reg_get(page, offset, &value, 1) != OK) {
		return _io_reg_get_error;
	}

	return value;
}

int
PX4IO::io_reg_modify(uint8_t page, uint8_t offset, uint16_t clearbits, uint16_t setbits)
{
	int ret;
	uint16_t value;

	ret = io_reg_get(page, offset, &value, 1);

	if (ret != OK) {
		return ret;
	}

	value &= ~clearbits;
	value |= setbits;

	return io_reg_set(page, offset, value);
}

int
PX4IO::print_debug()
{
#if defined(CONFIG_ARCH_BOARD_PX4_FMU_V2) || defined(CONFIG_ARCH_BOARD_PX4_FMU_V3)
	int io_fd = -1;

	if (io_fd <= 0) {
		io_fd = ::open("/dev/ttyS0", O_RDONLY | O_NONBLOCK | O_NOCTTY);
	}

	/* read IO's output */
	if (io_fd >= 0) {
		pollfd fds[1];
		fds[0].fd = io_fd;
		fds[0].events = POLLIN;

		px4_usleep(500);
		int pret = ::poll(fds, sizeof(fds) / sizeof(fds[0]), 0);

		if (pret > 0) {
			int count;
			char buf[65];

			do {
				count = ::read(io_fd, buf, sizeof(buf) - 1);

				if (count > 0) {
					/* enforce null termination */
					buf[count] = '\0';
					warnx("IO CONSOLE: %s", buf);
				}

			} while (count > 0);
		}

		::close(io_fd);
		return 0;
	}

#endif
	return 1;

}

int
PX4IO::mixer_send(const char *buf, unsigned buflen, unsigned retries)
{
	/* get debug level */
	int debuglevel = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_SET_DEBUG);

	uint8_t	frame[_max_transfer];

	do {

		px4io_mixdata *msg = (px4io_mixdata *)&frame[0];
		unsigned max_len = _max_transfer - sizeof(px4io_mixdata);

		msg->f2i_mixer_magic = F2I_MIXER_MAGIC;
		msg->action = F2I_MIXER_ACTION_RESET;

		do {
			unsigned count = buflen;

			if (count > max_len) {
				count = max_len;
			}

			if (count > 0) {
				memcpy(&msg->text[0], buf, count);
				buf += count;
				buflen -= count;

			} else {
				continue;
			}

			/*
			 * We have to send an even number of bytes.  This
			 * will only happen on the very last transfer of a
			 * mixer, and we are guaranteed that there will be
			 * space left to round up as _max_transfer will be
			 * even.
			 */
			unsigned total_len = sizeof(px4io_mixdata) + count;

			if (total_len % 2) {
				msg->text[count] = '\0';
				total_len++;
			}

			int ret;

			for (int i = 0; i < 30; i++) {
				/* failed, but give it a 2nd shot */
				ret = io_reg_set(PX4IO_PAGE_MIXERLOAD, 0, (uint16_t *)frame, total_len / 2);

				if (ret) {
					px4_usleep(333);

				} else {
					break;
				}
			}

			/* print mixer chunk */
			if (debuglevel > 5 || ret) {

				warnx("fmu sent: \"%s\"", msg->text);

				/* read IO's output */
				print_debug();
			}

			if (ret) {
				PX4_ERR("mixer send error %d", ret);
				return ret;
			}

			msg->action = F2I_MIXER_ACTION_APPEND;

		} while (buflen > 0);

		int ret;

		/* send the closing newline */
		msg->text[0] = '\n';
		msg->text[1] = '\0';

		for (int i = 0; i < 30; i++) {
			/* failed, but give it a 2nd shot */
			ret = io_reg_set(PX4IO_PAGE_MIXERLOAD, 0, (uint16_t *)frame, (sizeof(px4io_mixdata) + 2) / 2);

			if (ret) {
				px4_usleep(333);

			} else {
				break;
			}
		}

		if (ret == 0) {
			/* success, exit */
			break;
		}

		retries--;

	} while (retries > 0);

	if (retries == 0) {
		mavlink_log_info(&_mavlink_log_pub, "[IO] mixer upload fail");
		/* load must have failed for some reason */
		return -EINVAL;

	} else {
		/* all went well, set the mixer ok flag */
		return io_reg_modify(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FLAGS, 0, PX4IO_P_STATUS_FLAGS_MIXER_OK);
	}
}

void
PX4IO::print_status(bool extended_status)
{
	/* basic configuration */
	printf("protocol %u hardware %u bootloader %u buffer %uB crc 0x%04x%04x\n",
	       io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_PROTOCOL_VERSION),
	       io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_HARDWARE_VERSION),
	       io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_BOOTLOADER_VERSION),
	       io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_MAX_TRANSFER),
	       io_reg_get(PX4IO_PAGE_SETUP,  PX4IO_P_SETUP_CRC),
	       io_reg_get(PX4IO_PAGE_SETUP,  PX4IO_P_SETUP_CRC + 1));
	printf("%u controls %u actuators %u R/C inputs %u analog inputs %u relays\n",
	       io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_CONTROL_COUNT),
	       io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_ACTUATOR_COUNT),
	       io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_RC_INPUT_COUNT),
	       io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_ADC_INPUT_COUNT),
	       io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_RELAY_COUNT));

	/* status */
	uORB::SubscriptionData<px4io_status_s> status_sub{ORB_ID(px4io_status)};
	status_sub.update();

	print_message(status_sub.get());

	/* now clear alarms */
	io_reg_set(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_ALARMS, 0x0000);

	uint16_t pwm_invert_mask = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_REVERSE);

	printf("\n");
	printf("reversed outputs: [");

	for (unsigned i = 0; i < _max_actuators; i++) {
		printf("%s", (pwm_invert_mask & (1 << i)) ? "x" : "_");
	}

	printf("]");

	float trim_roll = REG_TO_FLOAT(io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_TRIM_ROLL));
	float trim_pitch = REG_TO_FLOAT(io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_TRIM_PITCH));
	float trim_yaw = REG_TO_FLOAT(io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_TRIM_YAW));

	printf(" trims: r: %8.4f p: %8.4f y: %8.4f\n",
	       (double)trim_roll, (double)trim_pitch, (double)trim_yaw);

	uint16_t raw_inputs = io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_COUNT);
	printf("%hu raw R/C inputs", raw_inputs);

	for (unsigned i = 0; i < raw_inputs; i++) {
		printf(" %u", io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_BASE + i));
	}

	printf("\n");

	uint16_t io_status_flags = io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FLAGS);
	uint16_t flags = io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_FLAGS);
	printf("R/C flags: 0x%04hx%s%s%s%s%s\n", flags,
	       (((io_status_flags & PX4IO_P_STATUS_FLAGS_RC_DSM) && (!(flags & PX4IO_P_RAW_RC_FLAGS_RC_DSM11))) ? " DSM10" : ""),
	       (((io_status_flags & PX4IO_P_STATUS_FLAGS_RC_DSM) && (flags & PX4IO_P_RAW_RC_FLAGS_RC_DSM11)) ? " DSM11" : ""),
	       ((flags & PX4IO_P_RAW_RC_FLAGS_FRAME_DROP) ? " FRAME_DROP" : ""),
	       ((flags & PX4IO_P_RAW_RC_FLAGS_FAILSAFE) ? " FAILSAFE" : ""),
	       ((flags & PX4IO_P_RAW_RC_FLAGS_MAPPING_OK) ? " MAPPING_OK" : "")
	      );

	if ((io_status_flags & PX4IO_P_STATUS_FLAGS_RC_PPM)) {
		int frame_len = io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_DATA);
		printf("RC data (PPM frame len) %d us\n", frame_len);

		if ((frame_len - raw_inputs * 2000 - 3000) < 0) {
			printf("WARNING  WARNING  WARNING! This RC receiver does not allow safe frame detection.\n");
		}
	}

	uint16_t mapped_inputs = io_reg_get(PX4IO_PAGE_RC_INPUT, PX4IO_P_RC_VALID);
	printf("mapped R/C inputs 0x%04hx", mapped_inputs);

	for (unsigned i = 0; i < _max_rc_input; i++) {
		if (mapped_inputs & (1 << i)) {
			printf(" %u:%hd", i, REG_TO_SIGNED(io_reg_get(PX4IO_PAGE_RC_INPUT, PX4IO_P_RC_BASE + i)));
		}
	}

	printf("\n");
	uint16_t adc_inputs = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_ADC_INPUT_COUNT);
	printf("ADC inputs");

	for (unsigned i = 0; i < adc_inputs; i++) {
		printf(" %u", io_reg_get(PX4IO_PAGE_RAW_ADC_INPUT, i));
	}

	printf("\n");

	/* setup and state */
	uint16_t features = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES);
	printf("features 0x%04hx%s%s%s%s\n", features,
	       ((features & PX4IO_P_SETUP_FEATURES_SBUS1_OUT) ? " S.BUS1_OUT" : ""),
	       ((features & PX4IO_P_SETUP_FEATURES_SBUS2_OUT) ? " S.BUS2_OUT" : ""),
	       ((features & PX4IO_P_SETUP_FEATURES_PWM_RSSI) ? " RSSI_PWM" : ""),
	       ((features & PX4IO_P_SETUP_FEATURES_ADC_RSSI) ? " RSSI_ADC" : "")
	      );

	printf("rates 0x%04x default %u alt %u sbus %u\n",
	       io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_RATES),
	       io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_DEFAULTRATE),
	       io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_ALTRATE),
	       io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_SBUS_RATE));
	printf("debuglevel %u\n", io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_SET_DEBUG));

	for (unsigned group = 0; group < 4; group++) {
		printf("controls %u:", group);

		for (unsigned i = 0; i < _max_controls; i++) {
			printf(" %hd", (int16_t) io_reg_get(PX4IO_PAGE_CONTROLS, group * PX4IO_PROTOCOL_MAX_CONTROL_COUNT + i));
		}

		printf("\n");
	}

	if (extended_status) {
		for (unsigned i = 0; i < _max_rc_input; i++) {
			unsigned base = PX4IO_P_RC_CONFIG_STRIDE * i;
			uint16_t options = io_reg_get(PX4IO_PAGE_RC_CONFIG, base + PX4IO_P_RC_CONFIG_OPTIONS);
			printf("input %u min %u center %u max %u deadzone %u assigned %u options 0x%04hx%s%s\n",
			       i,
			       io_reg_get(PX4IO_PAGE_RC_CONFIG, base + PX4IO_P_RC_CONFIG_MIN),
			       io_reg_get(PX4IO_PAGE_RC_CONFIG, base + PX4IO_P_RC_CONFIG_CENTER),
			       io_reg_get(PX4IO_PAGE_RC_CONFIG, base + PX4IO_P_RC_CONFIG_MAX),
			       io_reg_get(PX4IO_PAGE_RC_CONFIG, base + PX4IO_P_RC_CONFIG_DEADZONE),
			       io_reg_get(PX4IO_PAGE_RC_CONFIG, base + PX4IO_P_RC_CONFIG_ASSIGNMENT),
			       options,
			       ((options & PX4IO_P_RC_CONFIG_OPTIONS_ENABLED) ? " ENABLED" : ""),
			       ((options & PX4IO_P_RC_CONFIG_OPTIONS_REVERSE) ? " REVERSED" : ""));
		}
	}

	printf("failsafe");

	for (unsigned i = 0; i < _max_actuators; i++) {
		printf(" %u", io_reg_get(PX4IO_PAGE_FAILSAFE_PWM, i));
	}

	printf("\ndisarmed values");

	for (unsigned i = 0; i < _max_actuators; i++) {
		printf(" %u", io_reg_get(PX4IO_PAGE_DISARMED_PWM, i));
	}

	/* IMU heater (Pixhawk 2.1) */
	uint16_t heater_level = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_THERMAL);

	if (heater_level != UINT16_MAX) {
		if (heater_level == PX4IO_THERMAL_OFF) {
			printf("\nIMU heater off");

		} else {
			printf("\nIMU heater level %d", heater_level);
		}
	}

	if (_hitl_mode) {
		printf("\nHITL Mode");
	}

	printf("\n");
}

int
PX4IO::ioctl(file *filep, int cmd, unsigned long arg)
{
	SmartLock lock_guard(_lock);
	int ret = OK;

	/* regular ioctl? */
	switch (cmd) {
	case PWM_SERVO_ARM:
		/* set the 'armed' bit */
		ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, 0, PX4IO_P_SETUP_ARMING_FMU_ARMED);
		break;

	case PWM_SERVO_SET_ARM_OK:
		/* set the 'OK to arm' bit */
		ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, 0, PX4IO_P_SETUP_ARMING_IO_ARM_OK);
		break;

	case PWM_SERVO_CLEAR_ARM_OK:
		/* clear the 'OK to arm' bit */
		ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, PX4IO_P_SETUP_ARMING_IO_ARM_OK, 0);
		break;

	case PWM_SERVO_DISARM:
		/* clear the 'armed' bit */
		ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, PX4IO_P_SETUP_ARMING_FMU_ARMED, 0);
		break;

	case PWM_SERVO_GET_DEFAULT_UPDATE_RATE:
		/* get the default update rate */
		*(unsigned *)arg = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_DEFAULTRATE);
		break;

	case PWM_SERVO_SET_UPDATE_RATE:
		/* set the requested alternate rate */
		ret = io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_ALTRATE, arg);
		break;

	case PWM_SERVO_GET_UPDATE_RATE:
		/* get the alternative update rate */
		*(unsigned *)arg = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_ALTRATE);
		break;

	case PWM_SERVO_SET_SELECT_UPDATE_RATE: {

			/* blindly clear the PWM update alarm - might be set for some other reason */
			io_reg_set(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_ALARMS, PX4IO_P_STATUS_ALARMS_PWM_ERROR);

			/* attempt to set the rate map */
			io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_RATES, arg);

			/* check that the changes took */
			uint16_t alarms = io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_ALARMS);

			if (alarms & PX4IO_P_STATUS_ALARMS_PWM_ERROR) {
				ret = -EINVAL;
				io_reg_set(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_ALARMS, PX4IO_P_STATUS_ALARMS_PWM_ERROR);
				PX4_ERR("failed setting PWM rate on IO");
			}

			break;
		}

	case PWM_SERVO_GET_SELECT_UPDATE_RATE:

		*(unsigned *)arg = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_RATES);
		break;

	case PWM_SERVO_SET_FAILSAFE_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			if (pwm->channel_count > _max_actuators)
				/* fail with error */
			{
				return -E2BIG;
			}

			/* copy values to registers in IO */
			ret = io_reg_set(PX4IO_PAGE_FAILSAFE_PWM, 0, pwm->values, pwm->channel_count);
			break;
		}

	case PWM_SERVO_GET_FAILSAFE_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;
			pwm->channel_count = _max_actuators;

			ret = io_reg_get(PX4IO_PAGE_FAILSAFE_PWM, 0, pwm->values, _max_actuators);

			if (ret != OK) {
				ret = -EIO;
			}

			break;
		}

	case PWM_SERVO_SET_DISARMED_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			if (pwm->channel_count > _max_actuators)
				/* fail with error */
			{
				return -E2BIG;
			}

			/* copy values to registers in IO */
			ret = io_reg_set(PX4IO_PAGE_DISARMED_PWM, 0, pwm->values, pwm->channel_count);
			break;
		}

	case PWM_SERVO_GET_DISARMED_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;
			pwm->channel_count = _max_actuators;

			ret = io_reg_get(PX4IO_PAGE_DISARMED_PWM, 0, pwm->values, _max_actuators);

			if (ret != OK) {
				ret = -EIO;
			}

			break;
		}

	case PWM_SERVO_SET_MIN_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			if (pwm->channel_count > _max_actuators)
				/* fail with error */
			{
				return -E2BIG;
			}

			/* copy values to registers in IO */
			ret = io_reg_set(PX4IO_PAGE_CONTROL_MIN_PWM, 0, pwm->values, pwm->channel_count);
			break;
		}

	case PWM_SERVO_GET_MIN_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;
			pwm->channel_count = _max_actuators;

			ret = io_reg_get(PX4IO_PAGE_CONTROL_MIN_PWM, 0, pwm->values, _max_actuators);

			if (ret != OK) {
				ret = -EIO;
			}

			break;
		}

	case PWM_SERVO_SET_MAX_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			if (pwm->channel_count > _max_actuators)
				/* fail with error */
			{
				return -E2BIG;
			}

			/* copy values to registers in IO */
			ret = io_reg_set(PX4IO_PAGE_CONTROL_MAX_PWM, 0, pwm->values, pwm->channel_count);
			break;
		}

	case PWM_SERVO_GET_MAX_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;
			pwm->channel_count = _max_actuators;

			ret = io_reg_get(PX4IO_PAGE_CONTROL_MAX_PWM, 0, pwm->values, _max_actuators);

			if (ret != OK) {
				ret = -EIO;
			}
		}

		break;

	case PWM_SERVO_GET_TRIM_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;
			pwm->channel_count = _max_actuators;

			ret = io_reg_get(PX4IO_PAGE_CONTROL_TRIM_PWM, 0, pwm->values, _max_actuators);

			if (ret != OK) {
				ret = -EIO;
			}
		}

		break;

	case PWM_SERVO_GET_COUNT:
		*(unsigned *)arg = _max_actuators;
		break;

	case PWM_SERVO_SET_DISABLE_LOCKDOWN:
		_lockdown_override = arg;
		break;

	case PWM_SERVO_GET_DISABLE_LOCKDOWN:
		*(unsigned *)arg = _lockdown_override;
		break;

	case PWM_SERVO_SET_FORCE_SAFETY_OFF:
		/* force safety swith off */
		ret = io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FORCE_SAFETY_OFF, PX4IO_FORCE_SAFETY_MAGIC);
		break;

	case PWM_SERVO_SET_FORCE_SAFETY_ON:
		/* force safety switch on */
		ret = io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FORCE_SAFETY_ON, PX4IO_FORCE_SAFETY_MAGIC);
		break;

	case PWM_SERVO_SET_FORCE_FAILSAFE:

		/* force failsafe mode instantly */
		if (arg == 0) {
			/* clear force failsafe flag */
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, PX4IO_P_SETUP_ARMING_FORCE_FAILSAFE, 0);

		} else {
			/* set force failsafe flag */
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, 0, PX4IO_P_SETUP_ARMING_FORCE_FAILSAFE);
		}

		break;

	case PWM_SERVO_SET_TERMINATION_FAILSAFE:

		/* if failsafe occurs, do not allow the system to recover */
		if (arg == 0) {
			/* clear termination failsafe flag */
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE, 0);

		} else {
			/* set termination failsafe flag */
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, 0, PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE);
		}

		break;

	case PWM_SERVO_SET_OVERRIDE_IMMEDIATE:

		/* control whether override on FMU failure is
		   immediate or waits for override threshold on mode
		   switch */
		if (arg == 0) {
			/* clear override immediate flag */
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, PX4IO_P_SETUP_ARMING_OVERRIDE_IMMEDIATE, 0);

		} else {
			/* set override immediate flag */
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, 0, PX4IO_P_SETUP_ARMING_OVERRIDE_IMMEDIATE);
		}

		break;

	case DSM_BIND_START:
		/* bind a DSM receiver */
		ret = dsm_bind_ioctl(arg);
		break;

	case PWM_SERVO_SET(0) ... PWM_SERVO_SET(PWM_OUTPUT_MAX_CHANNELS - 1): {

			/* TODO: we could go lower for e.g. TurboPWM */
			unsigned channel = cmd - PWM_SERVO_SET(0);

			/* PWM needs to be either 0 or in the valid range. */
			if ((arg != 0) && ((channel >= _max_actuators) ||
					   (arg < PWM_LOWEST_MIN) ||
					   (arg > PWM_HIGHEST_MAX))) {
				ret = -EINVAL;

			} else {
				if (!_test_fmu_fail && _motor_test.in_test_mode) {
					/* send a direct PWM value */
					ret = io_reg_set(PX4IO_PAGE_DIRECT_PWM, channel, arg);

				} else {
					/* Just silently accept the ioctl without doing anything
					 * in test mode. */
					ret = OK;
				}
			}

			break;
		}

	case PWM_SERVO_GET(0) ... PWM_SERVO_GET(PWM_OUTPUT_MAX_CHANNELS - 1): {

			unsigned channel = cmd - PWM_SERVO_GET(0);

			if (channel >= _max_actuators) {
				ret = -EINVAL;

			} else {
				/* fetch a current PWM value */
				uint32_t value = io_reg_get(PX4IO_PAGE_SERVOS, channel);

				if (value == _io_reg_get_error) {
					ret = -EIO;

				} else {
					*(servo_position_t *)arg = value;
				}
			}

			break;
		}

	case PWM_SERVO_GET_RATEGROUP(0) ... PWM_SERVO_GET_RATEGROUP(PWM_OUTPUT_MAX_CHANNELS - 1): {

			unsigned channel = cmd - PWM_SERVO_GET_RATEGROUP(0);

			*(uint32_t *)arg = io_reg_get(PX4IO_PAGE_PWM_INFO, PX4IO_RATE_MAP_BASE + channel);

			if (*(uint32_t *)arg == _io_reg_get_error) {
				ret = -EIO;
			}

			break;
		}

	case PWM_SERVO_SET_MODE: {
			// reset all channels to disarmed when entering/leaving test mode, so that we don't
			// accidentially use values from previous tests
			pwm_output_values pwm_disarmed;

			if (io_reg_get(PX4IO_PAGE_DISARMED_PWM, 0, pwm_disarmed.values, _max_actuators) == 0) {
				for (unsigned i = 0; i < _max_actuators; ++i) {
					io_reg_set(PX4IO_PAGE_DIRECT_PWM, i, pwm_disarmed.values[i]);
				}
			}

			_motor_test.in_test_mode = (arg == PWM_SERVO_ENTER_TEST_MODE);
			ret = (arg == PWM_SERVO_ENTER_TEST_MODE || PWM_SERVO_EXIT_TEST_MODE) ? 0 : -EINVAL;
		}
		break;

	case MIXERIOCRESET:
		ret = 0;	/* load always resets */
		break;

	case MIXERIOCLOADBUF: {
			const char *buf = (const char *)arg;
			ret = mixer_send(buf, strlen(buf));
			break;
		}

	case PX4IO_SET_DEBUG:
		/* set the debug level */
		ret = io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_SET_DEBUG, arg);
		break;

	case PX4IO_REBOOT_BOOTLOADER:
		if (system_status() & PX4IO_P_SETUP_ARMING_FMU_ARMED) {
			PX4_ERR("not upgrading IO firmware, system is armed");
			return -EINVAL;

		}

		// re-enable safety
		ret = io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FORCE_SAFETY_ON, PX4IO_FORCE_SAFETY_MAGIC);

		if (ret != PX4_OK) {
			PX4_ERR("IO refused to re-enable safety");
		}

		// set new status
		_status &= ~(PX4IO_P_STATUS_FLAGS_SAFETY_OFF);

		/* reboot into bootloader - arg must be PX4IO_REBOOT_BL_MAGIC */
		usleep(1);
		ret = io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_REBOOT_BL, arg);

		if (ret != PX4_OK) {
			PX4_ERR("IO refused to reboot");
		}

		break;

	case PX4IO_CHECK_CRC: {
			/* check IO firmware CRC against passed value */
			uint32_t io_crc = 0;
			ret = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_CRC, (uint16_t *)&io_crc, 2);

			if (ret != OK) {
				return ret;
			}

			if (io_crc != arg) {
				PX4_DEBUG("Firmware CRC mismatch 0x%08x 0x%08lx", io_crc, arg);
				return -EINVAL;
			}

			break;
		}

	case PX4IO_INAIR_RESTART_ENABLE:

		/* set/clear the 'in-air restart' bit */
		if (arg) {
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, 0, PX4IO_P_SETUP_ARMING_INAIR_RESTART_OK);

		} else {
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, PX4IO_P_SETUP_ARMING_INAIR_RESTART_OK, 0);
		}

		break;

	case RC_INPUT_ENABLE_RSSI_ANALOG:

		if (arg) {
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES, 0, PX4IO_P_SETUP_FEATURES_ADC_RSSI);

		} else {
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES, PX4IO_P_SETUP_FEATURES_ADC_RSSI, 0);
		}

		break;

	case RC_INPUT_ENABLE_RSSI_PWM:

		if (arg) {
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES, 0, PX4IO_P_SETUP_FEATURES_PWM_RSSI);

		} else {
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES, PX4IO_P_SETUP_FEATURES_PWM_RSSI, 0);
		}

		break;

	case SBUS_SET_PROTO_VERSION:

		if (arg == 1) {
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES, 0, PX4IO_P_SETUP_FEATURES_SBUS1_OUT);

		} else if (arg == 2) {
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES, 0, PX4IO_P_SETUP_FEATURES_SBUS2_OUT);

		} else {
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES,
					    (PX4IO_P_SETUP_FEATURES_SBUS1_OUT | PX4IO_P_SETUP_FEATURES_SBUS2_OUT), 0);
		}

		break;

	case PX4IO_HEATER_CONTROL:
		if (arg == (unsigned long)HEATER_MODE_DISABLED) {
			io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_THERMAL, PX4IO_THERMAL_IGNORE);

		} else if (arg == 1) {
			io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_THERMAL, PX4IO_THERMAL_FULL);

		} else {
			io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_THERMAL, PX4IO_THERMAL_OFF);

		}

		break;

	default:
		/* see if the parent class can make any use of it */
		ret = CDev::ioctl(filep, cmd, arg);
		break;
	}

	return ret;
}

extern "C" __EXPORT int px4io_main(int argc, char *argv[]);

namespace
{

device::Device *
get_interface()
{
	device::Device *interface = nullptr;

#ifdef PX4IO_SERIAL_BASE
	interface = PX4IO_serial_interface();
#endif

	if (interface != nullptr) {
		goto got;
	}

	errx(1, "cannot alloc interface");

got:

	if (interface->init() != OK) {
		delete interface;
		errx(1, "interface init failed");
	}

	return interface;
}

void
start(int argc, char *argv[])
{
	if (g_dev != nullptr) {
		errx(0, "already loaded");
	}

	/* allocate the interface */
	device::Device *interface = get_interface();

	/* create the driver - it will set g_dev */
	(void)new PX4IO(interface);

	if (g_dev == nullptr) {
		delete interface;
		errx(1, "driver allocation failed");
	}

	bool rc_handling_disabled = false;
	bool hitl_mode = false;

	/* disable RC handling and/or actuator_output publication on request */
	for (int extra_args = 1; extra_args < argc; extra_args++) {
		if (!strcmp(argv[extra_args], "norc")) {
			rc_handling_disabled = true;

		} else if (!strcmp(argv[extra_args], "hil")) {
			hitl_mode = true;

		} else if (argv[extra_args][0] != '\0') {
			PX4_WARN("unknown argument: %s", argv[extra_args]);
		}
	}

	if (OK != g_dev->init(rc_handling_disabled, hitl_mode)) {
		delete g_dev;
		g_dev = nullptr;
		errx(1, "driver init failed");
	}

	exit(0);
}

void
detect(int argc, char *argv[])
{
	if (g_dev != nullptr) {
		errx(0, "already loaded");
	}

	/* allocate the interface */
	device::Device *interface = get_interface();

	/* create the driver - it will set g_dev */
	(void)new PX4IO(interface);

	if (g_dev == nullptr) {
		errx(1, "driver allocation failed");
	}

	int ret = g_dev->detect();

	delete g_dev;
	g_dev = nullptr;

	if (ret) {
		/* nonzero, error */
		exit(1);

	} else {
		exit(0);
	}
}

void
checkcrc(int argc, char *argv[])
{
	bool keep_running = false;

	if (g_dev == nullptr) {
		/* allocate the interface */
		device::Device *interface = get_interface();

		/* create the driver - it will set g_dev */
		(void)new PX4IO(interface);

		if (g_dev == nullptr) {
			PX4_ERR("driver allocation failed");
			exit(1);
		}

	} else {
		/* its already running, don't kill the driver */
		keep_running = true;
	}

	/*
	  check IO CRC against CRC of a file
	 */
	if (argc < 2) {
		PX4_WARN("usage: px4io checkcrc filename");
		exit(1);
	}

	int fd = open(argv[1], O_RDONLY);

	if (fd == -1) {
		PX4_ERR("open of %s failed: %d", argv[1], errno);
		exit(1);
	}

	const uint32_t app_size_max = 0xf000;
	uint32_t fw_crc = 0;
	uint32_t nbytes = 0;

	while (true) {
		uint8_t buf[16];
		int n = read(fd, buf, sizeof(buf));

		if (n <= 0) { break; }

		fw_crc = crc32part(buf, n, fw_crc);
		nbytes += n;
	}

	close(fd);

	while (nbytes < app_size_max) {
		uint8_t b = 0xff;
		fw_crc = crc32part(&b, 1, fw_crc);
		nbytes++;
	}

	int ret = g_dev->ioctl(nullptr, PX4IO_CHECK_CRC, fw_crc);

	if (!keep_running) {
		delete g_dev;
		g_dev = nullptr;
	}

	if (ret != OK) {
		PX4_ERR("check CRC failed: %d, CRC: %u", ret, fw_crc);
		exit(1);

	} else {
		PX4_INFO("IO FW CRC match");
	}

	exit(0);
}

void
bind(int argc, char *argv[])
{
	int pulses;

	if (g_dev == nullptr) {
		errx(1, "px4io must be started first");
	}

	if (argc < 3) {
		errx(0, "needs argument, use dsm2, dsmx or dsmx8");
	}

	if (!strcmp(argv[2], "dsm2")) {
		pulses = DSM2_BIND_PULSES;

	} else if (!strcmp(argv[2], "dsmx")) {
		pulses = DSMX_BIND_PULSES;

	} else if (!strcmp(argv[2], "dsmx8")) {
		pulses = DSMX8_BIND_PULSES;

	} else {
		errx(1, "unknown parameter %s, use dsm2, dsmx or dsmx8", argv[2]);
	}

	// Test for custom pulse parameter
	if (argc > 3) {
		pulses = atoi(argv[3]);
	}

	g_dev->ioctl(nullptr, DSM_BIND_START, pulses);

	exit(0);

}

void
monitor(void)
{
	/* clear screen */
	printf("\033[2J");

	unsigned cancels = 2;

	for (;;) {
		pollfd fds[1];

		fds[0].fd = 0;
		fds[0].events = POLLIN;

		if (poll(fds, 1, 2000) < 0) {
			errx(1, "poll fail");
		}

		if (fds[0].revents == POLLIN) {
			/* control logic is to cancel with any key */
			char c;
			(void)read(0, &c, 1);

			if (cancels-- == 0) {
				printf("\033[2J\033[H"); /* move cursor home and clear screen */
				exit(0);
			}
		}

		if (g_dev != nullptr) {

			printf("\033[2J\033[H"); /* move cursor home and clear screen */
			(void)g_dev->print_status(false);
			(void)g_dev->print_debug();
			printf("\n\n\n[ Use 'px4io debug <N>' for more output. Hit <enter> three times to exit monitor mode ]\n");

		} else {
			errx(1, "driver not loaded, exiting");
		}
	}
}

void
lockdown(int argc, char *argv[])
{
	if (g_dev != nullptr) {

		if (argc > 2 && !strcmp(argv[2], "disable")) {

			warnx("WARNING: ACTUATORS WILL BE LIVE IN HIL! PROCEED?");
			warnx("Press 'y' to enable, any other key to abort.");

			/* check if user wants to abort */
			char c;

			struct pollfd fds;
			int ret;
			hrt_abstime start = hrt_absolute_time();
			const unsigned long timeout = 5000000;

			while (hrt_elapsed_time(&start) < timeout) {
				fds.fd = 0; /* stdin */
				fds.events = POLLIN;
				ret = poll(&fds, 1, 0);

				if (ret > 0) {

					if (read(0, &c, 1) > 0) {

						if (c != 'y') {
							exit(0);

						} else if (c == 'y') {
							break;
						}
					}
				}

				px4_usleep(10000);
			}

			if (hrt_elapsed_time(&start) > timeout) {
				errx(1, "TIMEOUT! ABORTED WITHOUT CHANGES.");
			}

			(void)g_dev->ioctl(0, PWM_SERVO_SET_DISABLE_LOCKDOWN, 1);

			warnx("WARNING: ACTUATORS ARE NOW LIVE IN HIL!");

		} else {
			(void)g_dev->ioctl(0, PWM_SERVO_SET_DISABLE_LOCKDOWN, 0);
			warnx("ACTUATORS ARE NOW SAFE IN HIL.");
		}

	} else {
		errx(1, "driver not loaded, exiting");
	}

	exit(0);
}

} /* namespace */

int
px4io_main(int argc, char *argv[])
{
	/* check for sufficient number of arguments */
	if (argc < 2) {
		goto out;
	}

	if (!PX4_MFT_HW_SUPPORTED(PX4_MFT_PX4IO)) {
		errx(1, "PX4IO Not Supported");
	}

	if (!strcmp(argv[1], "start")) {
		start(argc - 1, argv + 1);
	}

	if (!strcmp(argv[1], "detect")) {
		detect(argc - 1, argv + 1);
	}

	if (!strcmp(argv[1], "checkcrc")) {
		checkcrc(argc - 1, argv + 1);
	}

	if (!strcmp(argv[1], "update")) {

		constexpr unsigned MAX_RETRIES = 5;
		unsigned retries = 0;
		int ret = PX4_ERROR;

		while (ret != OK && retries < MAX_RETRIES) {

			retries++;
			// Sleep 200 ms before the next attempt
			usleep(200 * 1000);

			if (g_dev == nullptr) {
				/* allocate the interface */
				device::Device *interface = get_interface();

				/* create the driver - it will set g_dev */
				(void)new PX4IO(interface);

				if (g_dev == nullptr) {
					delete interface;
					errx(1, "driver allocation failed");
				}
			}

			// Try to reboot
			ret = g_dev->ioctl(nullptr, PX4IO_REBOOT_BOOTLOADER, PX4IO_REBOOT_BL_MAGIC);
			// tear down the px4io instance
			delete g_dev;
			g_dev = nullptr;

			if (ret != OK) {
				PX4_ERR("reboot failed - %d, still attempting upgrade", ret);
			}

			PX4IO_Uploader *up;

			/* Assume we are using default paths */

			const char *fn[4] = PX4IO_FW_SEARCH_PATHS;

			/* Override defaults if a path is passed on command line */
			if (argc > 2) {
				fn[0] = argv[2];
				fn[1] = nullptr;
			}

			up = new PX4IO_Uploader;
			ret = up->upload(&fn[0]);
			delete up;
		}

		switch (ret) {
		case OK:
			break;

		case -ENOENT:
			PX4_ERR("PX4IO firmware file not found");
			break;

		case -EEXIST:
		case -EIO:
			PX4_ERR("error updating PX4IO - check that bootloader mode is enabled");
			break;

		case -EINVAL:
			PX4_ERR("verify failed - retry the update");
			break;

		case -ETIMEDOUT:
			PX4_ERR("timed out waiting for bootloader - power-cycle and try again");
			break;

		default:
			PX4_ERR("unexpected error %d", ret);
			break;
		}

		return ret;
	}

	/* commands below here require a started driver */

	if (g_dev == nullptr) {
		errx(1, "not started");
	}

	if (!strcmp(argv[1], "safety_off")) {
		int ret = g_dev->ioctl(NULL, PWM_SERVO_SET_FORCE_SAFETY_OFF, 0);

		if (ret != OK) {
			warnx("failed to disable safety");
			exit(1);
		}

		exit(0);
	}

	if (!strcmp(argv[1], "safety_on")) {
		int ret = g_dev->ioctl(NULL, PWM_SERVO_SET_FORCE_SAFETY_ON, 0);

		if (ret != OK) {
			warnx("failed to enable safety");
			exit(1);
		}

		exit(0);
	}

	if (!strcmp(argv[1], "recovery")) {

		/*
		 * Enable in-air restart support.
		 * We can cheat and call the driver directly, as it
		 * doesn't reference filp in ioctl()
		 */
		g_dev->ioctl(NULL, PX4IO_INAIR_RESTART_ENABLE, 1);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {

		/* stop the driver */
		delete g_dev;
		g_dev = nullptr;
		exit(0);
	}


	if (!strcmp(argv[1], "status")) {

		warnx("loaded");
		g_dev->print_status(true);

		exit(0);
	}

	if (!strcmp(argv[1], "debug")) {
		if (argc <= 2) {
			warnx("usage: px4io debug LEVEL");
			exit(1);
		}

		if (g_dev == nullptr) {
			warnx("not started");
			exit(1);
		}

		uint8_t level = atoi(argv[2]);
		/* we can cheat and call the driver directly, as it
		 * doesn't reference filp in ioctl()
		 */
		int ret = g_dev->ioctl(nullptr, PX4IO_SET_DEBUG, level);

		if (ret != 0) {
			warnx("SET_DEBUG failed: %d", ret);
			exit(1);
		}

		warnx("SET_DEBUG %hhu OK", level);
		exit(0);
	}

	if (!strcmp(argv[1], "rx_dsm") ||
	    !strcmp(argv[1], "rx_dsm_10bit") ||
	    !strcmp(argv[1], "rx_dsm_11bit") ||
	    !strcmp(argv[1], "rx_sbus") ||
	    !strcmp(argv[1], "rx_ppm")) {
		errx(0, "receiver type is automatically detected, option '%s' is deprecated", argv[1]);
	}

	if (!strcmp(argv[1], "monitor")) {
		monitor();
	}

	if (!strcmp(argv[1], "bind")) {
		bind(argc, argv);
	}

	if (!strcmp(argv[1], "lockdown")) {
		lockdown(argc, argv);
	}

	if (!strcmp(argv[1], "sbus1_out")) {
		/* we can cheat and call the driver directly, as it
		 * doesn't reference filp in ioctl()
		 */
		int ret = g_dev->ioctl(nullptr, SBUS_SET_PROTO_VERSION, 1);

		if (ret != 0) {
			errx(ret, "S.BUS v1 failed");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "sbus2_out")) {
		/* we can cheat and call the driver directly, as it
		 * doesn't reference filp in ioctl()
		 */
		int ret = g_dev->ioctl(nullptr, SBUS_SET_PROTO_VERSION, 2);

		if (ret != 0) {
			errx(ret, "S.BUS v2 failed");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "rssi_analog")) {
		/* we can cheat and call the driver directly, as it
		 * doesn't reference filp in ioctl()
		 */
		int ret = g_dev->ioctl(nullptr, RC_INPUT_ENABLE_RSSI_ANALOG, 1);

		if (ret != 0) {
			errx(ret, "RSSI analog failed");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "rssi_pwm")) {
		/* we can cheat and call the driver directly, as it
		 * doesn't reference filp in ioctl()
		 */
		int ret = g_dev->ioctl(nullptr, RC_INPUT_ENABLE_RSSI_PWM, 1);

		if (ret != 0) {
			errx(ret, "RSSI PWM failed");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "test_fmu_fail")) {
		if (g_dev != nullptr) {
			g_dev->test_fmu_fail(true);
			exit(0);

		} else {

			errx(1, "px4io must be started first");
		}
	}

	if (!strcmp(argv[1], "test_fmu_ok")) {
		if (g_dev != nullptr) {
			g_dev->test_fmu_fail(false);
			exit(0);

		} else {

			errx(1, "px4io must be started first");
		}
	}

out:
	errx(1, "need a command, try 'start', 'stop', 'status', 'monitor', 'debug <level>',\n"
	     "'recovery', 'bind', 'checkcrc', 'safety_on', 'safety_off',\n"
	     "'update', 'sbus1_out', 'sbus2_out', 'rssi_analog' or 'rssi_pwm',\n"
	     "'test_fmu_fail', 'test_fmu_ok'");
}
