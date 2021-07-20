/****************************************************************************
 *
 *   Copyright (c) 2012-2019, 2021 PX4 Development Team. All rights reserved.
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

#include <float.h>

#include <board_config.h>
#include <drivers/drv_adc.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_rc_input.h>
#include <lib/perf/perf_counter.h>
#include <lib/rc/crsf.h>
#include <lib/rc/ghst.hpp>
#include <lib/rc/dsm.h>
#include <lib/rc/sbus.h>
#include <lib/rc/st24.h>
#include <lib/rc/sumd.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/adc_report.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_status.h>

#include "crsf_telemetry.h"
#include "ghst_telemetry.hpp"

#ifdef HRT_PPM_CHANNEL
# include <systemlib/ppm_decode.h>
#endif

class RCInput : public ModuleBase<RCInput>, public ModuleParams, public px4::ScheduledWorkItem
{
public:

	RCInput(const char *device);
	virtual ~RCInput();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	int	init();

private:

	enum RC_SCAN {
		RC_SCAN_PPM = 0,
		RC_SCAN_SBUS,
		RC_SCAN_DSM,
		RC_SCAN_SUMD,
		RC_SCAN_ST24,
		RC_SCAN_CRSF,
		RC_SCAN_GHST
	} _rc_scan_state{RC_SCAN_SBUS};

	static constexpr char const *RC_SCAN_STRING[7] {
		"PPM",
		"SBUS",
		"DSM",
		"SUMD",
		"ST24",
		"CRSF",
		"GHST"
	};

	void Run() override;

#if defined(SPEKTRUM_POWER)
	bool bind_spektrum(int arg = DSMX8_BIND_PULSES) const;
#endif // SPEKTRUM_POWER

	void fill_rc_in(uint16_t raw_rc_count_local,
			uint16_t raw_rc_values_local[input_rc_s::RC_INPUT_MAX_CHANNELS],
			hrt_abstime now, bool frame_drop, bool failsafe,
			unsigned frame_drops, int rssi);

	void set_rc_scan_state(RC_SCAN _rc_scan_state);

	void rc_io_invert(bool invert);

	hrt_abstime _rc_scan_begin{0};

	bool _initialized{false};
	bool _rc_scan_locked{false};
	bool _report_lock{true};

	static constexpr unsigned	_current_update_interval{4000}; // 250 Hz

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription	_adc_report_sub{ORB_ID(adc_report)};
	uORB::Subscription	_vehicle_cmd_sub{ORB_ID(vehicle_command)};
	uORB::Subscription	_vehicle_status_sub{ORB_ID(vehicle_status)};

	input_rc_s	_rc_in{};

	float		_analog_rc_rssi_volt{-1.0f};
	bool		_analog_rc_rssi_stable{false};

	bool _armed{false};


	uORB::PublicationMulti<input_rc_s>	_to_input_rc{ORB_ID(input_rc)};

	int		_rcs_fd{-1};
	char		_device[20] {};					///< device / serial port path

	static constexpr size_t RC_MAX_BUFFER_SIZE{SBUS_BUFFER_SIZE};
	uint8_t _rcs_buf[RC_MAX_BUFFER_SIZE] {};

	uint16_t _raw_rc_values[input_rc_s::RC_INPUT_MAX_CHANNELS] {};
	uint16_t _raw_rc_count{};

	CRSFTelemetry *_crsf_telemetry{nullptr};
	GHSTTelemetry *_ghst_telemetry{nullptr};

	perf_counter_t	_cycle_perf;
	perf_counter_t	_publish_interval_perf;
	uint32_t	_bytes_rx{0};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::RC_RSSI_PWM_CHAN>) _param_rc_rssi_pwm_chan,
		(ParamInt<px4::params::RC_RSSI_PWM_MIN>) _param_rc_rssi_pwm_min,
		(ParamInt<px4::params::RC_RSSI_PWM_MAX>) _param_rc_rssi_pwm_max
	)
};
