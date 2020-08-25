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

#include <px4_arch/io_timer.h>
#include <px4_arch/hw_description.h>
#include <px4_platform_common/constexpr_util.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform/io_timer_init.h>
#include <stm32_tim.h>

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
	case Timer::Timer12:
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



static inline constexpr timer_io_channels_t initIOTimerChannel(const io_timers_t io_timers_conf[MAX_IO_TIMERS],
		Timer::TimerChannel timer, GPIO::GPIOPin pin)
{
	timer_io_channels_t ret = initIOTimerGPIOInOut(timer, pin);

	switch (timer.channel) {
	case Timer::Channel1:
		ret.ccr_offset = STM32_GTIM_CCR1_OFFSET;
		ret.masks = GTIM_SR_CC1IF | GTIM_SR_CC1OF;
		ret.timer_channel = 1;
		break;

	case Timer::Channel2:
		ret.ccr_offset = STM32_GTIM_CCR2_OFFSET;
		ret.masks = GTIM_SR_CC2IF | GTIM_SR_CC2OF;
		ret.timer_channel = 2;
		break;

	case Timer::Channel3:
		ret.ccr_offset = STM32_GTIM_CCR3_OFFSET;
		ret.masks = GTIM_SR_CC3IF | GTIM_SR_CC3OF;
		ret.timer_channel = 3;
		break;

	case Timer::Channel4:
		ret.ccr_offset = STM32_GTIM_CCR4_OFFSET;
		ret.masks = GTIM_SR_CC4IF | GTIM_SR_CC4OF;
		ret.timer_channel = 4;
		break;
	}

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

static inline constexpr io_timers_t initIOTimer(Timer::Timer timer, DMA dshot_dma = {})
{
	bool nuttx_config_timer_enabled = false;
	io_timers_t ret{};

	switch (timer) {
	case Timer::Timer1:
		ret.base = STM32_TIM1_BASE;
		ret.clock_register = STM32_RCC_APB2ENR;
		ret.clock_bit = RCC_APB2ENR_TIM1EN;
		ret.clock_freq = STM32_APB2_TIM1_CLKIN;
		ret.vectorno =  STM32_IRQ_TIMCC;
#ifdef CONFIG_STM32_TIM1
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::Timer2:
		ret.base = STM32_TIM2_BASE;
		ret.clock_register = STM32_RCC_APB1LENR;
		ret.clock_bit = RCC_APB1LENR_TIM2EN;
		ret.clock_freq = STM32_APB1_TIM2_CLKIN;
		ret.vectorno =  STM32_IRQ_TIM2;
#ifdef CONFIG_STM32_TIM2
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::Timer3:
		ret.base = STM32_TIM3_BASE;
		ret.clock_register = STM32_RCC_APB1LENR;
		ret.clock_bit = RCC_APB1LENR_TIM3EN;
		ret.clock_freq = STM32_APB1_TIM3_CLKIN;
		ret.vectorno =  STM32_IRQ_TIM3;
#ifdef CONFIG_STM32_TIM3
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::Timer4:
		ret.base = STM32_TIM4_BASE;
		ret.clock_register = STM32_RCC_APB1LENR;
		ret.clock_bit = RCC_APB1LENR_TIM4EN;
		ret.clock_freq = STM32_APB1_TIM4_CLKIN;
		ret.vectorno =  STM32_IRQ_TIM4;
#ifdef CONFIG_STM32_TIM4
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::Timer5:
		ret.base = STM32_TIM5_BASE;
		ret.clock_register = STM32_RCC_APB1LENR;
		ret.clock_bit = RCC_APB1LENR_TIM5EN;
		ret.clock_freq = STM32_APB1_TIM5_CLKIN;
		ret.vectorno =  STM32_IRQ_TIM5;
#ifdef CONFIG_STM32_TIM5
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::Timer6:
		ret.base = STM32_TIM6_BASE;
		ret.clock_register = STM32_RCC_APB1LENR;
		ret.clock_bit = RCC_APB1LENR_TIM6EN;
		ret.clock_freq = STM32_APB1_TIM6_CLKIN;
		ret.vectorno =  STM32_IRQ_TIM6;
#ifdef CONFIG_STM32_TIM6
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::Timer7:
		ret.base = STM32_TIM7_BASE;
		ret.clock_register = STM32_RCC_APB1LENR;
		ret.clock_bit = RCC_APB1LENR_TIM7EN;
		ret.clock_freq = STM32_APB1_TIM7_CLKIN;
		ret.vectorno =  STM32_IRQ_TIM7;
#ifdef CONFIG_STM32_TIM7
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::Timer8:
		ret.base = STM32_TIM8_BASE;
		ret.clock_register = STM32_RCC_APB2ENR;
		ret.clock_bit = RCC_APB2ENR_TIM8EN;
		ret.clock_freq = STM32_APB2_TIM8_CLKIN;
		ret.vectorno =  STM32_IRQ_TIM8CC;
#ifdef CONFIG_STM32_TIM8
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::Timer9:
	case Timer::Timer10:
	case Timer::Timer11:
		constexpr_assert(false, "Invalid Timer");
		break;

	case Timer::Timer12:
		ret.base = STM32_TIM12_BASE;
		ret.clock_register = STM32_RCC_APB1LENR;
		ret.clock_bit = RCC_APB1LENR_TIM12EN;
		ret.clock_freq = STM32_APB1_TIM12_CLKIN;
		ret.vectorno =  STM32_IRQ_TIM12;
#ifdef CONFIG_STM32_TIM12
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::Timer13:
		ret.base = STM32_TIM13_BASE;
		ret.clock_register = STM32_RCC_APB1LENR;
		ret.clock_bit = RCC_APB1LENR_TIM13EN;
		ret.clock_freq = STM32_APB1_TIM13_CLKIN;
		ret.vectorno =  STM32_IRQ_TIM13;
#ifdef CONFIG_STM32_TIM13
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::Timer14:
		ret.base = STM32_TIM14_BASE;
		ret.clock_register = STM32_RCC_APB1LENR;
		ret.clock_bit = RCC_APB1LENR_TIM14EN;
		ret.clock_freq = STM32_APB1_TIM14_CLKIN;
		ret.vectorno =  STM32_IRQ_TIM14;
#ifdef CONFIG_STM32_TIM14
		nuttx_config_timer_enabled = true;
#endif
		break;
	}

	// This is not strictly required, but for consistency let's make sure NuttX timers are disabled
	constexpr_assert(!nuttx_config_timer_enabled, "IO Timer requires NuttX timer config to be disabled (STM32_TIMx)");

	// DShot
	if (dshot_dma.index != DMA::Invalid) {
		ret.dshot.dma_base = getDMABaseRegister(dshot_dma);
		ret.dshot.dmamap = getTimerUpdateDMAMap(timer, dshot_dma);
	}

	return ret;
}
