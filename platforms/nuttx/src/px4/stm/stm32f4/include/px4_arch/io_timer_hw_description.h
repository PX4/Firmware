/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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


#include "../../../stm32_common/include/px4_arch/io_timer_hw_description.h"

static inline constexpr timer_io_channels_t initIOTimerGPIOInOut(Timer::TimerChannel timer, GPIO::GPIOPin pin)
{
	timer_io_channels_t ret{};
	uint32_t gpio_af = 0;

	switch (timer.timer) {
	case Timer::Timer1:
	case Timer::Timer2:
		gpio_af = GPIO_AF1;
		break;

	case Timer::Timer3:
	case Timer::Timer4:
	case Timer::Timer5:
		gpio_af = GPIO_AF2;
		break;

	case Timer::Timer6:
	case Timer::Timer7:
	case Timer::Timer8:
	case Timer::Timer9:
	case Timer::Timer10:
	case Timer::Timer11:
		gpio_af = GPIO_AF3;
		break;

	case Timer::Timer12:
	case Timer::Timer13:
	case Timer::Timer14:
		gpio_af = GPIO_AF9;
		break;
	}

	uint32_t pin_port = getGPIOPort(pin.port) | getGPIOPin(pin.pin);
	ret.gpio_in = gpio_af | (GPIO_ALT | GPIO_SPEED_50MHz | GPIO_FLOAT) | pin_port;
	ret.gpio_out = gpio_af | (GPIO_ALT | GPIO_SPEED_50MHz | GPIO_PUSHPULL) | pin_port;
	return ret;
}
