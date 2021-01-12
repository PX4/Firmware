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
#include "hardware/s32k1xx_ftm.h"
#include "hardware/s32k1xx_pcc.h"

/* TO DO: This mess only supports FTM 0-5, not 6-7 (only available on S32K148).
 * It's also hard to check if it is correct. *Should* work for S32K146, not confirmed for other S32K1XX MCUs. */

static inline constexpr timer_io_channels_t initIOTimerChannel(const io_timers_t io_timers_conf[MAX_IO_TIMERS],
		Timer::TimerChannel timer, GPIO::GPIOPin pin)
{
	timer_io_channels_t ret{};

	uint32_t gpio_af = 0;

	switch (pin.port) {
	case GPIO::PortA:
		if (pin.pin == GPIO::Pin0 || pin.pin == GPIO::Pin1 || pin.pin == GPIO::Pin2 || pin.pin == GPIO::Pin3 ||
		    pin.pin == GPIO::Pin10 || pin.pin == GPIO::Pin11 || pin.pin == GPIO::Pin12 || pin.pin == GPIO::Pin13 ||
		    pin.pin == GPIO::Pin15 || pin.pin == GPIO::Pin16 || pin.pin == GPIO::Pin17 || pin.pin == GPIO::Pin25 ||
		    pin.pin == GPIO::Pin26 || pin.pin == GPIO::Pin27 || pin.pin == GPIO::Pin28 || pin.pin == GPIO::Pin29 ||
		    pin.pin == GPIO::Pin30 || pin.pin == GPIO::Pin31) {
			gpio_af = PIN_ALT2;

		} else if (pin.pin == GPIO::Pin7) {
			gpio_af = PIN_ALT3;

		} else if (pin.pin == GPIO::Pin6) {
			gpio_af = PIN_ALT4;
		}

		break;

	case GPIO::PortB:
		if (pin.pin == GPIO::Pin2 || pin.pin == GPIO::Pin3 || pin.pin == GPIO::Pin4 || pin.pin == GPIO::Pin5 ||
		    pin.pin == GPIO::Pin8 || pin.pin == GPIO::Pin9 || pin.pin == GPIO::Pin10 || pin.pin == GPIO::Pin11 ||
		    pin.pin == GPIO::Pin12 || pin.pin == GPIO::Pin13 || pin.pin == GPIO::Pin14 || pin.pin == GPIO::Pin15 ||
		    pin.pin == GPIO::Pin16 || pin.pin == GPIO::Pin17 || pin.pin == GPIO::Pin18) {
			gpio_af = PIN_ALT2;

		} else if (pin.pin == GPIO::Pin0 || pin.pin == GPIO::Pin1) {
			gpio_af = PIN_ALT6;
		}

		break;

	case GPIO::PortC:
		if (pin.pin == GPIO::Pin0 || pin.pin == GPIO::Pin1) {
			if (timer.timer == 0) {
				gpio_af = PIN_ALT2;

			} else if (timer.timer == 1) {
				gpio_af = PIN_ALT6;
			}

		} else if (pin.pin == GPIO::Pin11 || pin.pin == GPIO::Pin12 || pin.pin == GPIO::Pin13) {
			if (timer.timer == 3) {
				gpio_af = PIN_ALT2;

			} else if (timer.timer == 2 || timer.timer == 4) {
				gpio_af = PIN_ALT3;
			}

		} else if (pin.pin == GPIO::Pin2 || pin.pin == GPIO::Pin3 || pin.pin == GPIO::Pin4 || pin.pin == GPIO::Pin5 ||
			   pin.pin == GPIO::Pin10 || pin.pin == GPIO::Pin14 || pin.pin == GPIO::Pin15 || pin.pin == GPIO::Pin27 ||
			   pin.pin == GPIO::Pin28 || pin.pin == GPIO::Pin29 || pin.pin == GPIO::Pin30 || pin.pin == GPIO::Pin31) {
			gpio_af = PIN_ALT2;

		} else if (pin.pin == GPIO::Pin6 || pin.pin == GPIO::Pin7 || pin.pin == GPIO::Pin8 || pin.pin == GPIO::Pin9) {
			gpio_af = PIN_ALT4;
		}

		break;

	case GPIO::PortD:
		if (pin.pin == GPIO::Pin0 || pin.pin == GPIO::Pin1) {
			if (timer.timer == 0) {
				gpio_af = PIN_ALT2;

			} else if (timer.timer == 2) {
				gpio_af = PIN_ALT4;
			}

		} else if (pin.pin == GPIO::Pin2 || pin.pin == GPIO::Pin3 || pin.pin == GPIO::Pin5 || pin.pin == GPIO::Pin10 ||
			   pin.pin == GPIO::Pin11 || pin.pin == GPIO::Pin12 || pin.pin == GPIO::Pin13 || pin.pin == GPIO::Pin14 ||
			   pin.pin == GPIO::Pin15 || pin.pin == GPIO::Pin16 || pin.pin == GPIO::Pin18) {
			gpio_af = PIN_ALT2;

		} else if (pin.pin == GPIO::Pin8 || pin.pin == GPIO::Pin9) {
			gpio_af = PIN_ALT6;
		}

		break;

	case GPIO::PortE:
		if (pin.pin == GPIO::Pin7 || pin.pin == GPIO::Pin8 || pin.pin == GPIO::Pin9 || pin.pin == GPIO::Pin13 ||
		    pin.pin == GPIO::Pin20 || pin.pin == GPIO::Pin21 || pin.pin == GPIO::Pin22 || pin.pin == GPIO::Pin23 ||
		    pin.pin == GPIO::Pin24 || pin.pin == GPIO::Pin25) {
			gpio_af = PIN_ALT2;

		} else if (pin.pin == GPIO::Pin2 || pin.pin == GPIO::Pin4 || pin.pin == GPIO::Pin5 || pin.pin == GPIO::Pin6 ||
			   pin.pin == GPIO::Pin10 || pin.pin == GPIO::Pin11 || pin.pin == GPIO::Pin15 || pin.pin == GPIO::Pin16) {
			gpio_af = PIN_ALT4;
		}

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
		ret.base = S32K1XX_FTM0_BASE;
		ret.clock_register = S32K1XX_PCC_FTM0;
		ret.clock_bit = PCC_CGC;
#if defined(CONFIG_ARCH_CHIP_S32K14X) /* S32K14X has a different vectornumber than S32K11X */
		ret.vectorno =
			S32K1XX_IRQ_FTM0_CH0_1; /* To Do: There are now multiple interrupts per FTM... Need to work around this somehow. */
#elif defined(CONFIG_ARCH_CHIP_S32K11X)
		ret.vectorno = S32K1XX_IRQ_FTM0_CH0_7;
#endif
#ifdef CONFIG_S32K1XX_FTM0
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::FTM1:
		ret.base = S32K1XX_FTM1_BASE;
		ret.clock_register = S32K1XX_PCC_FTM1;
		ret.clock_bit = PCC_CGC;
#if defined(CONFIG_ARCH_CHIP_S32K14X) /* S32K14X has a different vectornumber than S32K11X */
		ret.vectorno =
			S32K1XX_IRQ_FTM1_CH0_1; /* To Do: There are now multiple interrupts per FTM... Need to work around this somehow. */
#elif defined(CONFIG_ARCH_CHIP_S32K11X)
		ret.vectorno = S32K1XX_IRQ_FTM1_CH0_7;
#endif
#ifdef CONFIG_S32K1XX_FTM1
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::FTM2:
		ret.base = S32K1XX_FTM2_BASE;
		ret.clock_register = S32K1XX_PCC_FTM2;
		ret.clock_bit = PCC_CGC;
		ret.vectorno =
			S32K1XX_IRQ_FTM2_CH0_1; /* To Do: There are now multiple interrupts per FTM... Need to work around this somehow. */
#ifdef CONFIG_S32K1XX_FTM2
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::FTM3:
		ret.base = S32K1XX_FTM3_BASE;
		ret.clock_register = S32K1XX_PCC_FTM3;
		ret.clock_bit = PCC_CGC;
		ret.vectorno =
			S32K1XX_IRQ_FTM3_CH0_1; /* To Do: There are now multiple interrupts per FTM... Need to work around this somehow. */
#ifdef CONFIG_S32K1XX_FTM3
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::FTM4:
		ret.base = S32K1XX_FTM4_BASE;
		ret.clock_register = S32K1XX_PCC_FTM4;
		ret.clock_bit = PCC_CGC;
		ret.vectorno =
			S32K1XX_IRQ_FTM4_CH0_1; /* To Do: There are now multiple interrupts per FTM... Need to work around this somehow. */
#ifdef CONFIG_S32K1XX_FTM4
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::FTM5:
		ret.base = S32K1XX_FTM5_BASE;
		ret.clock_register = S32K1XX_PCC_FTM5;
		ret.clock_bit = PCC_CGC;
		ret.vectorno =
			S32K1XX_IRQ_FTM5_CH0_1; /* To Do: There are now multiple interrupts per FTM... Need to work around this somehow. */
#ifdef CONFIG_S32K1XX_FTM5
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::FTM6:
		ret.base = S32K1XX_FTM6_BASE;
		ret.clock_register = S32K1XX_PCC_FTM6;
		ret.clock_bit = PCC_CGC;
		ret.vectorno =
			S32K1XX_IRQ_FTM6_CH0_1; /* To Do: There are now multiple interrupts per FTM... Need to work around this somehow. */
#ifdef CONFIG_S32K1XX_FTM6
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::FTM7:
		ret.base = S32K1XX_FTM7_BASE;
		ret.clock_register = S32K1XX_PCC_FTM7;
		ret.clock_bit = PCC_CGC;
		ret.vectorno =
			S32K1XX_IRQ_FTM7_CH0_1; /* To Do: There are now multiple interrupts per FTM... Need to work around this somehow. */
#ifdef CONFIG_S32K1XX_FTM7
		nuttx_config_timer_enabled = true;
#endif
		break;
	}

	// This is not strictly required, but for consistency let's make sure NuttX timers are disabled
	constexpr_assert(!nuttx_config_timer_enabled, "IO Timer requires NuttX timer config to be disabled (S32K1XX_FTMx)");

	return ret;
}
