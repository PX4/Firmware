/****************************************************************************
 *
 *   Copyright (c) 2014-2019 PX4 Development Team. All rights reserved.
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
 * @file LidarLitePWM.h
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Ban Siesta <bansiesta@gmail.com>
 *
 * Driver for the PulsedLight Lidar-Lite range finders connected via PWM.
 *
 * This driver accesses the pwm_input published by the pwm_input driver.
 */
#pragma once

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/topics/pwm_input.h>
#include <uORB/Subscription.hpp>
#include <board_config.h>
#include <drivers/device/device.h>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <perf/perf_counter.h>

using namespace time_literals;

// Device limits
static constexpr float LL40LS_MIN_DISTANCE{0.05f};
static constexpr float LL40LS_MAX_DISTANCE{25.00f};
static constexpr float LL40LS_MAX_DISTANCE_V2{35.00f};

// Normal conversion wait time.
static constexpr uint32_t LL40LS_CONVERSION_INTERVAL{50_ms};

// Maximum time to wait for a conversion to complete.
static constexpr uint32_t LL40LS_CONVERSION_TIMEOUT{100_ms};

#if DIRECT_PWM_OUTPUT_CHANNELS >= 6
#define GPIO_VDD_RANGEFINDER_EN_CHAN 5 // use pin 6
#define LIDAR_LITE_PWM_SUPPORTED

class LidarLitePWM : public px4::ScheduledWorkItem
{
public:
	LidarLitePWM(const uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
	virtual ~LidarLitePWM();

	int init();
	void start();
	void stop();

	void print_info();

protected:

	int collect();
	int measure();

	void Run() override;

private:
	uint32_t get_measure_interval() const { return LL40LS_CONVERSION_INTERVAL; };


	uORB::Subscription _sub_pwm_input{ORB_ID(pwm_input)};

	pwm_input_s _pwm{};

	PX4Rangefinder	_px4_rangefinder;

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, "ll40ls: comms errors")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, "ll40ls: read")};
	perf_counter_t _sensor_resets{perf_alloc(PC_COUNT, "ll40ls: resets")};
	perf_counter_t _sensor_zero_resets{perf_alloc(PC_COUNT, "ll40ls: zero resets")};
};

#endif
