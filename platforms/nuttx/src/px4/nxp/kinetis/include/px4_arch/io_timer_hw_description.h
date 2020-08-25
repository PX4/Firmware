/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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


#include <px4_arch/io_timer.h>
#include <px4_arch/hw_description.h>
#include <px4_platform_common/constexpr_util.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform/io_timer_init.h>
#include "hardware/kinetis_ftm.h"
#include "hardware/kinetis_sim.h"


static inline constexpr timer_io_channels_t initIOTimerChannel(const io_timers_t io_timers_conf[MAX_IO_TIMERS],
		Timer::TimerChannel timer, GPIO::GPIOPin pin)
{
	timer_io_channels_t ret{};

	uint32_t gpio_af = 0;

	switch (pin.port) {
	case GPIO::PortA:
		gpio_af = PIN_ALT3;
		break;

	case GPIO::PortB:
		if (pin.pin == GPIO::Pin12 || pin.pin == GPIO::Pin13) {
			gpio_af = PIN_ALT4;

		} else {
			gpio_af = PIN_ALT3;
		}

		break;

	case GPIO::PortC:
		if (pin.pin == GPIO::Pin5) {
			gpio_af = PIN_ALT7;

		} else if (pin.pin == GPIO::Pin8 || pin.pin == GPIO::Pin9 || pin.pin == GPIO::Pin10 || pin.pin == GPIO::Pin11) {
			gpio_af = PIN_ALT3;

		} else {
			gpio_af = PIN_ALT4;
		}

		break;

	case GPIO::PortD:
		gpio_af = PIN_ALT4;
		break;

	case GPIO::PortE:
		gpio_af = PIN_ALT6;
		break;

	default: break;
	}

	uint32_t gpio_pin_port = getGPIOPort(pin.port) | getGPIOPin(pin.pin);
	ret.gpio_in = gpio_af | gpio_pin_port;
	ret.gpio_out = gpio_af | gpio_pin_port;

	ret.timer_channel = (int)timer.channel + 1;

	// find timer index
	ret.timer_index = 0xff;
	const uint32_t timer_base = timerBaseRegister(timer.timer);

	for (int i = 0; i < MAX_IO_TIMERS; ++i) {
		if (io_timers_conf[i].base == timer_base) {
			ret.timer_index = i;
			break;
		}
	}

	constexpr_assert(ret.timer_index != 0xff, "Timer not found");

	return ret;
}

static inline constexpr io_timers_t initIOTimer(Timer::Timer timer)
{
	bool nuttx_config_timer_enabled = false;
	io_timers_t ret{};

	switch (timer) {
	case Timer::FTM0:
		ret.base = KINETIS_FTM0_BASE;
		ret.clock_register = KINETIS_SIM_SCGC6;
		ret.clock_bit = SIM_SCGC6_FTM0;
		ret.vectorno =  KINETIS_IRQ_FTM0;
#ifdef CONFIG_KINETIS_FTM0
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::FTM1:
		ret.base = KINETIS_FTM1_BASE;
		ret.clock_register = KINETIS_SIM_SCGC6;
		ret.clock_bit = SIM_SCGC6_FTM1;
		ret.vectorno =  KINETIS_IRQ_FTM1;
#ifdef CONFIG_KINETIS_FTM1
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::FTM2:
		ret.base = KINETIS_FTM2_BASE;
		ret.clock_register = KINETIS_SIM_SCGC3;
		ret.clock_bit = SIM_SCGC3_FTM2;
		ret.vectorno =  KINETIS_IRQ_FTM2;
#ifdef CONFIG_KINETIS_FTM2
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::FTM3:
		ret.base = KINETIS_FTM3_BASE;
		ret.clock_register = KINETIS_SIM_SCGC3;
		ret.clock_bit = SIM_SCGC3_FTM3;
		ret.vectorno =  KINETIS_IRQ_FTM3;
#ifdef CONFIG_KINETIS_FTM3
		nuttx_config_timer_enabled = true;
#endif
		break;
	}

	// This is not strictly required, but for consistency let's make sure NuttX timers are disabled
	constexpr_assert(!nuttx_config_timer_enabled, "IO Timer requires NuttX timer config to be disabled (KINETIS_FTMx)");

	return ret;
}


