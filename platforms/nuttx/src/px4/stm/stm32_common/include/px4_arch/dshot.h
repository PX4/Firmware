/****************************************************************************
 *
 * Copyright (C) 2019 PX4 Development Team. All rights reserved.
 * Author: Igor Misic <igy1000mb@gmail.com>
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

#include <drivers/drv_pwm_output.h>
#include <stm32_dma.h>

#define DSHOT_MOTOR_PWM_BIT_WIDTH		20u

/* Configuration for each timer to setup DShot. Some timers have only one while others have two choices for the stream.
 *
 * DMAMAP_TIM1_UP	- DMA2, Channel6, Stream5
 * DMAMAP_TIM2_UP_1	- DMA1, Channel3, Stream1
 * DMAMAP_TIM2_UP_2	- DMA1, Channel3, Stream7
 * DMAMAP_TIM3_UP	- DMA1, Channel5, Stream2
 * DMAMAP_TIM4_UP	- DMA1, Channel2, Stream6
 * DMAMAP_TIM5_UP_1	- DMA1, Channel6, Stream0
 * DMAMAP_TIM5_UP_2	- DMA1, Channel6, Stream6
 * DMAMAP_TIM6_UP	- DMA1, Channel7, Stream1
 * DMAMAP_TIM7_UP_1	- DMA1, Channel1, Stream2
 * DMAMAP_TIM7_UP_2	- DMA1, Channel1, Stream4
 * DMAMAP_TIM8_UP	- DMA2, Channel7, Stream1
 */

/* The structure which contains configuration for DShot
 */
typedef struct dshot_conf_t {
	uint32_t			dma_base;
	uint32_t			dmamap;
} dshot_conf_t;
