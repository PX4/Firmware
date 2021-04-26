/****************************************************************************
 *
 *   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
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

#include <drivers/device/device.h>
#include <drivers/drv_input_capture.h>
#include <drivers/drv_mixer.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <uORB/topics/esc_status.h>

#include "DShotTelemetry.h"

using namespace time_literals;

#if !defined(BOARD_HAS_PWM)
#  error "board_config.h needs to define BOARD_HAS_PWM"
#endif

/** Dshot PWM frequency, Hz */
static constexpr unsigned int DSHOT150  =  150000u;
static constexpr unsigned int DSHOT300  =  300000u;
static constexpr unsigned int DSHOT600  =  600000u;
static constexpr unsigned int DSHOT1200 = 1200000u;

static constexpr int DSHOT_DISARM_VALUE = 0;
static constexpr int DSHOT_MIN_THROTTLE = 1;
static constexpr int DSHOT_MAX_THROTTLE = 1999;

class DShot : public cdev::CDev, public ModuleBase<DShot>, public OutputModuleInterface
{
public:
	DShot();
	virtual ~DShot();

	enum Mode {
		MODE_NONE = 0,
		MODE_1PWM,
		MODE_2PWM,
		MODE_2PWM2CAP,
		MODE_3PWM,
		MODE_3PWM1CAP,
		MODE_4PWM,
		MODE_4PWM1CAP,
		MODE_4PWM2CAP,
		MODE_5PWM,
		MODE_5PWM1CAP,
		MODE_6PWM,
		MODE_8PWM,
		MODE_12PWM,
		MODE_14PWM,
		MODE_4CAP,
		MODE_5CAP,
		MODE_6CAP,
	};

	/** Mode given via CLI */
	enum PortMode {
		PORT_MODE_UNSET = 0,
		PORT_FULL_GPIO,
		PORT_FULL_PWM,
		PORT_PWM14,
		PORT_PWM12,
		PORT_PWM8,
		PORT_PWM6,
		PORT_PWM5,
		PORT_PWM4,
		PORT_PWM3,
		PORT_PWM2,
		PORT_PWM1,
		PORT_PWM3CAP1,
		PORT_PWM4CAP1,
		PORT_PWM4CAP2,
		PORT_PWM5CAP1,
		PORT_PWM2CAP2,
		PORT_CAPTURE,
	};

	static void capture_trampoline(void *context, const uint32_t channel_index, const hrt_abstime edge_time,
				       const uint32_t edge_state, const uint32_t overflow);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	Mode get_mode() { return _mode; }

	virtual int init();

	virtual int ioctl(file *filp, int cmd, unsigned long arg);

	/** change the mode of the running module */
	static int module_new_mode(const PortMode new_mode);

	void mixerChanged() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	void retrieve_and_print_esc_info_thread_safe(const int motor_index);

	/**
	 * Send a dshot command to one or all motors
	 * This is expected to be called from another thread.
	 * @param num_repetitions number of times to repeat, set at least to 1
	 * @param motor_index index or -1 for all
	 * @return 0 on success, <0 error otherwise
	 */
	int send_command_thread_safe(const dshot_command_t command, const int num_repetitions, const int motor_index);

	int set_mode(const Mode new_mode);

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	bool telemetry_enabled() const { return _telemetry != nullptr; }

	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

private:

	/** Disallow copy construction and move assignment. */
	DShot(const DShot &) = delete;
	DShot operator=(const DShot &) = delete;

	enum class DShotConfig {
		Disabled  = 0,
		DShot150  = 150,
		DShot300  = 300,
		DShot600  = 600,
		DShot1200 = 1200,
	};

	struct Command {
		dshot_command_t command{};
		int num_repetitions{0};
		uint8_t motor_mask{0xff};
		bool valid() const { return num_repetitions > 0; }
		void clear() { num_repetitions = 0; }
	};

	struct Telemetry {
		DShotTelemetry handler{};
		uORB::PublicationData<esc_status_s> esc_status_pub{ORB_ID(esc_status)};
		int last_motor_index{-1};
	};

	void capture_callback(const uint32_t channel_index, const hrt_abstime edge_time,
			      const uint32_t edge_state, const uint32_t overflow);

	int capture_ioctl(file *filp, const int cmd, const unsigned long arg);

	void enable_dshot_outputs(const bool enabled);

	void init_telemetry(const char *device);

	void handle_new_telemetry_data(const int motor_index, const DShotTelemetry::EscData &data);

	int pwm_ioctl(file *filp, const int cmd, const unsigned long arg);

	int request_esc_info();

	void Run() override;

	void update_params();

	void update_telemetry_num_motors();

	MixingOutput _mixing_output{DIRECT_PWM_OUTPUT_CHANNELS, *this, MixingOutput::SchedulingPolicy::Auto, false, false};

	Telemetry *_telemetry{nullptr};

	static char _telemetry_device[20];
	static px4::atomic_bool _request_telemetry_init;

	px4::atomic<Command *> _new_command{nullptr};

	px4::atomic<DShotTelemetry::OutputBuffer *> _request_esc_info{nullptr};

	bool _outputs_initialized{false};
	bool _outputs_on{false};
	bool _waiting_for_esc_info{false};

	unsigned _num_outputs{0};
	uint32_t _output_mask{0};

	int _class_instance{-1};

	perf_counter_t	_cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	Command _current_command{};

	Mode _mode{MODE_NONE};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::DSHOT_CONFIG>)   _param_dshot_config,
		(ParamFloat<px4::params::DSHOT_MIN>)    _param_dshot_min,
		(ParamInt<px4::params::MOT_POLE_COUNT>) _param_mot_pole_count
	)
};
