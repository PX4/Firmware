/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file ghst_telemetry.cpp
 *
 * IRC Ghost (Immersion RC Ghost) telemetry.
 *
 * @author Igor Misic <igy1000mb@gmail.com>
 * @author Juraj Ciberlin <jciberlin1@gmail.com>
 */

#pragma once

#include <uORB/Subscription.hpp>
#include <uORB/topics/battery_status.h>
#include <drivers/drv_hrt.h>

/**
 * High-level class that handles sending of GHST telemetry data
 */
class GHSTTelemetry
{
public:
	/**
	 * @param uart_fd file descriptor for the UART to use. It is expected to be configured
	 * already.
	 */
	explicit GHSTTelemetry(int uart_fd);

	~GHSTTelemetry() = default;

	/**
	 * Send telemetry data. Call this regularly (i.e. at 100Hz), it will automatically
	 * limit the sending rate.
	 * @return true if new data sent
	 */
	bool update(const hrt_abstime &now);

private:
	bool send_battery_status();

	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};

	int _uart_fd;
	hrt_abstime _last_update {0U};
	uint32_t _next_type {0U};

	static constexpr uint32_t NUM_DATA_TYPES {1U};	// number of different telemetry data types
	static constexpr uint32_t UPDATE_RATE_HZ {10U};	// update rate [Hz]

	// Factors that should be applied to get correct values
	static constexpr float FACTOR_VOLTS_TO_10MV {100.0F};
	static constexpr float FACTOR_AMPS_TO_10MA {100.0F};
	static constexpr float FACTOR_MAH_TO_10MAH {0.1F};

};
