/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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

#include <px4_arch/io_timer_hw_description.h>

/* Timer allocation
 *
 * TIM5_CH4  T FMU_CH1
 * TIM5_CH3  T FMU_CH2
 * TIM5_CH2  T FMU_CH3
 * TIM5_CH1  T FMU_CH4
 *
 * TIM4_CH2  T FMU_CH5
 * TIM4_CH3  T FMU_CH6
 *
 * TIM12_CH1 T FMU_CH7
 * TIM12_CH2 T FMU_CH8
 *
 * TIM1_CH2  T FMU_CAP1               < Capture
 * TIM1_CH3  T SPI2_DRDY2_ISM330_INT2 < Capture or GPIO INT
 * TIM1_CH1  T SPIX_SYNC              > Pulse or GPIO strobe
 *
 * TIM2_CH3  T HEATER                 > PWM OUT or GPIO
 *
 * TIM14_CH1 T BUZZER_1              - Driven by other driver
 * TIM8_CH1_IN T FMU_PPM_INPUT       - Sampled byt HRT by other driver
 */

constexpr io_timers_t io_timers[MAX_IO_TIMERS] = {
	initIOTimer(Timer::Timer5, DMA{DMA::Index1}),
	initIOTimer(Timer::Timer4, DMA{DMA::Index1}),
	initIOTimer(Timer::Timer12),
	initIOTimer(Timer::Timer1),
	initIOTimer(Timer::Timer2),
};

constexpr timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
	initIOTimerChannel(io_timers, {Timer::Timer5, Timer::Channel4}, {GPIO::PortI, GPIO::Pin0}),
	initIOTimerChannel(io_timers, {Timer::Timer5, Timer::Channel3}, {GPIO::PortH, GPIO::Pin12}),
	initIOTimerChannel(io_timers, {Timer::Timer5, Timer::Channel2}, {GPIO::PortH, GPIO::Pin11}),
	initIOTimerChannel(io_timers, {Timer::Timer5, Timer::Channel1}, {GPIO::PortH, GPIO::Pin10}),
	initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel2}, {GPIO::PortD, GPIO::Pin13}),
	initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel3}, {GPIO::PortD, GPIO::Pin14}),
	initIOTimerChannel(io_timers, {Timer::Timer12, Timer::Channel1}, {GPIO::PortH, GPIO::Pin6}),
	initIOTimerChannel(io_timers, {Timer::Timer12, Timer::Channel2}, {GPIO::PortH, GPIO::Pin9}),
};

constexpr io_timers_channel_mapping_t io_timers_channel_mapping =
	initIOTimerChannelMapping(io_timers, timer_io_channels);
