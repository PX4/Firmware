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

#include <stdint.h>

#include <stm32_gpio.h>
#include <stm32_tim.h>
#include <stm32_dma.h>

#include <px4_platform_common/constexpr_util.h>

/*
 * DMA
 */

struct DMA {
	enum Index { Invalid = 0, Index1 = 1, Index2 };
	Index index;
	enum Stream { Stream0, Stream1, Stream2, Stream3, Stream4, Stream5, Stream6, Stream7 };
	Stream stream;
	enum Channel { Channel0, Channel1, Channel2, Channel3, Channel4, Channel5, Channel6, Channel7 };
	Channel channel;
};

static inline constexpr uint32_t getDMABaseRegister(const DMA &dma)
{
	switch (dma.index) {
	case DMA::Index1: return STM32_DMA1_BASE;

	case DMA::Index2: return STM32_DMA2_BASE;

	case DMA::Invalid:
		constexpr_assert(false, "Trying to get DMA base register for invalid config");
		break;
	}

	return 0;
}


/*
 * Timers
 */

namespace Timer
{
enum Timer {
	Timer1 = 1,
	Timer2,
	Timer3,
	Timer4,
	Timer5,
	Timer6,
	Timer7,
	Timer8,
	Timer9,
	Timer10,
	Timer11,
	Timer12,
	Timer13,
	Timer14,
};
enum Channel {
	Channel1 = 1,
	Channel2,
	Channel3,
	Channel4,
};
struct TimerChannel {
	Timer timer;
	Channel channel;
};
}

static inline constexpr uint32_t timerBaseRegister(Timer::Timer timer)
{
	switch (timer) {
	case Timer::Timer1: return STM32_TIM1_BASE;

	case Timer::Timer2: return STM32_TIM2_BASE;

	case Timer::Timer3: return STM32_TIM3_BASE;

	case Timer::Timer4: return STM32_TIM4_BASE;

	case Timer::Timer5: return STM32_TIM5_BASE;

	case Timer::Timer6: return STM32_TIM6_BASE;

	case Timer::Timer7: return STM32_TIM7_BASE;

	case Timer::Timer8: return STM32_TIM8_BASE;

#ifdef STM32_TIM9_BASE // STM32F1 only goes up to TIM8

	case Timer::Timer9: return STM32_TIM9_BASE;
#endif

#ifdef STM32_TIM10_BASE

	case Timer::Timer10: return STM32_TIM10_BASE;
#endif

#ifdef STM32_TIM11_BASE

	case Timer::Timer11: return STM32_TIM11_BASE;
#endif

#ifdef STM32_TIM12_BASE

	case Timer::Timer12: return STM32_TIM12_BASE;
#endif

#ifdef STM32_TIM13_BASE

	case Timer::Timer13: return STM32_TIM13_BASE;
#endif

#ifdef STM32_TIM14_BASE

	case Timer::Timer14: return STM32_TIM14_BASE;
#endif

	default: break;
	}

	return 0;
}


/*
 * GPIO
 */

namespace GPIO
{
enum Port {
	PortInvalid = 0,
	PortA,
	PortB,
	PortC,
	PortD,
	PortE,
	PortF,
	PortG,
	PortH,
	PortI,
	PortJ,
	PortK,
};
enum Pin {
	Pin0 = 0,
	Pin1,
	Pin2,
	Pin3,
	Pin4,
	Pin5,
	Pin6,
	Pin7,
	Pin8,
	Pin9,
	Pin10,
	Pin11,
	Pin12,
	Pin13,
	Pin14,
	Pin15,
};
struct GPIOPin {
	Port port;
	Pin pin;
};
}

static inline constexpr uint32_t getGPIOPort(GPIO::Port port)
{
	switch (port) {
	case GPIO::PortA: return GPIO_PORTA;

	case GPIO::PortB: return GPIO_PORTB;

	case GPIO::PortC: return GPIO_PORTC;

	case GPIO::PortD: return GPIO_PORTD;

	case GPIO::PortE: return GPIO_PORTE;
#ifdef GPIO_PORTF

	case GPIO::PortF: return GPIO_PORTF;
#endif
#ifdef GPIO_PORTG

	case GPIO::PortG: return GPIO_PORTG;
#endif
#ifdef GPIO_PORTH

	case GPIO::PortH: return GPIO_PORTH;
#endif
#ifdef GPIO_PORTI

	case GPIO::PortI: return GPIO_PORTI;
#endif
#ifdef GPIO_PORTJ

	case GPIO::PortJ: return GPIO_PORTJ;
#endif
#ifdef GPIO_PORTK

	case GPIO::PortK: return GPIO_PORTK;
#endif

	default: break;
	}

	return 0;
}

static inline constexpr uint32_t getGPIOPin(GPIO::Pin pin)
{
	switch (pin) {
	case GPIO::Pin0: return GPIO_PIN0;

	case GPIO::Pin1: return GPIO_PIN1;

	case GPIO::Pin2: return GPIO_PIN2;

	case GPIO::Pin3: return GPIO_PIN3;

	case GPIO::Pin4: return GPIO_PIN4;

	case GPIO::Pin5: return GPIO_PIN5;

	case GPIO::Pin6: return GPIO_PIN6;

	case GPIO::Pin7: return GPIO_PIN7;

	case GPIO::Pin8: return GPIO_PIN8;

	case GPIO::Pin9: return GPIO_PIN9;

	case GPIO::Pin10: return GPIO_PIN10;

	case GPIO::Pin11: return GPIO_PIN11;

	case GPIO::Pin12: return GPIO_PIN12;

	case GPIO::Pin13: return GPIO_PIN13;

	case GPIO::Pin14: return GPIO_PIN14;

	case GPIO::Pin15: return GPIO_PIN15;
	}

	return 0;
}

namespace SPI
{

enum class Bus {
	SPI1 = 1,
	SPI2,
	SPI3,
	SPI4,
	SPI5,
	SPI6,
};

using CS = GPIO::GPIOPin; ///< chip-select pin
using DRDY = GPIO::GPIOPin; ///< data ready pin

struct bus_device_external_cfg_t {
	CS cs_gpio;
	DRDY drdy_gpio;
};

} // namespace SPI
