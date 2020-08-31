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


#include "../../../stm32_common/include/px4_arch/hw_description.h"

static inline constexpr uint32_t getTimerUpdateDMAMap(Timer::Timer timer, const DMA &dma)
{
	uint32_t dma_map = 0;

	switch (timer) {
	case Timer::Timer1:
		if (dma.index == DMA::Index2 && dma.stream == DMA::Stream5 && dma.channel == DMA::Channel6) { dma_map = DMAMAP_TIM1_UP; }

		break;

	case Timer::Timer2:
		if (dma.index == DMA::Index1 && dma.stream == DMA::Stream1 && dma.channel == DMA::Channel3) { dma_map = DMAMAP_TIM2_UP_1; }

		if (dma.index == DMA::Index1 && dma.stream == DMA::Stream7 && dma.channel == DMA::Channel3) { dma_map = DMAMAP_TIM2_UP_2; }

		break;

	case Timer::Timer3:
		if (dma.index == DMA::Index1 && dma.stream == DMA::Stream2 && dma.channel == DMA::Channel5) { dma_map = DMAMAP_TIM3_UP; }

		break;

	case Timer::Timer4:
		if (dma.index == DMA::Index1 && dma.stream == DMA::Stream6 && dma.channel == DMA::Channel2) { dma_map = DMAMAP_TIM4_UP; }

		break;

	case Timer::Timer5:
		if (dma.index == DMA::Index1 && dma.stream == DMA::Stream0 && dma.channel == DMA::Channel6) { dma_map = DMAMAP_TIM5_UP_1; }

		if (dma.index == DMA::Index1 && dma.stream == DMA::Stream6 && dma.channel == DMA::Channel6) { dma_map = DMAMAP_TIM5_UP_2; }

		break;

	case Timer::Timer6:
		if (dma.index == DMA::Index1 && dma.stream == DMA::Stream1 && dma.channel == DMA::Channel7) { dma_map = DMAMAP_TIM6_UP; }

		break;

	case Timer::Timer7:
		if (dma.index == DMA::Index1 && dma.stream == DMA::Stream2 && dma.channel == DMA::Channel1) { dma_map = DMAMAP_TIM7_UP_1; }

		if (dma.index == DMA::Index1 && dma.stream == DMA::Stream4 && dma.channel == DMA::Channel1) { dma_map = DMAMAP_TIM7_UP_2; }

		break;

	case Timer::Timer8:
		if (dma.index == DMA::Index2 && dma.stream == DMA::Stream1 && dma.channel == DMA::Channel7) { dma_map = DMAMAP_TIM8_UP; }

		break;

	case Timer::Timer9:
	case Timer::Timer10:
	case Timer::Timer11:
	case Timer::Timer12:
	case Timer::Timer13:
	case Timer::Timer14:
		break;
	}

	constexpr_assert(dma_map != 0, "Invalid DMA config for given timer");
	return dma_map;
}
