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

/**
 * @file drv_dshot.h
 */

#pragma once

#include <drivers/drv_pwm_output.h>

#define DSHOT_MOTOR_PWM_BIT_WIDTH		20u

/* Configuration for each timer to setup DShot. Some timers have only one while others have two choices for the stream.
 *
 * TIM1UP - DMA2, Channel6, Stream5
 * TIM2UP - DMA1, Channel3, Stream1 or Stream7
 * TIM3UP - DMA1, Channel5, Stream2
 * TIM4UP - DMA1, Channel2, Stream6
 * TIM5UP - DMA1, Channel6, Stream0 or Stream6
 * TIM6UP - DMA1, Channel7, Stream1
 * TIM7UP - DMA1, Channel1, Stream2 or Stream4
 * TIM8UP - DMA2, Channel7, Stream1
 */

#define DSHOT_DMA1_BASE		STM32_DMA1_BASE
#define DSHOT_DMA2_BASE		STM32_DMA2_BASE

typedef enum dshot_dma_channel_t {
	DShot_Channel0	= 0u,
	DShot_Channel1	= 1u,
	DShot_Channel2	= 2u,
	DShot_Channel3	= 3u,
	DShot_Channel4	= 4u,
	DShot_Channel5	= 5u,
	DShot_Channel6	= 6u,
	DShot_Channel7	= 7u
}dshot_dma_channel_t;

typedef enum dshot_dma_stream_t {
	DShot_Stream0	= 0u,
	DShot_Stream1	= 1u,
	DShot_Stream2	= 2u,
	DShot_Stream3	= 3u,
	DShot_Stream4	= 4u,
	DShot_Stream5	= 5u,
	DShot_Stream6	= 6u,
	DShot_Stream7	= 7u
}dshot_dma_stream_t;

typedef enum {
    DShot_cmd_motor_stop = 0,
    DShot_cmd_beacon1,
    DShot_cmd_beacon2,
    DShot_cmd_beacon3,
    DShot_cmd_beacon4,
    DShot_cmd_beacon5,
    DShot_cmd_esc_info, // V2 includes settings
    DShot_cmd_spin_direction_1,
    DShot_cmd_spin_direction_2,
    DShot_cmd_3d_mode_off,
    DShot_cmd_3d_mode_on,
    DShot_cmd_settings_request, // Currently not implemented
    DShot_cmd_save_settings,
    DShot_cmd_spin_direction_normal = 20,
    DShot_cmd_spin_direction_reversed = 21,
    DShot_cmd_led0_on, // BLHeli32 only
    DShot_cmd_led1_on, // BLHeli32 only
    DShot_cmd_led2_on, // BLHeli32 only
    DShot_cmd_led3_on, // BLHeli32 only
    DShot_cmd_led0_off, // BLHeli32 only
    DShot_cmd_led1_off, // BLHeli32 only
    DShot_cmd_led2_off, // BLHeli32 only
    DShot_cmd_led4_off, // BLHeli32 only
    DShot_cmd_audio_stream_mode_on_off = 30, // KISS audio Stream mode on/off
    DShot_cmd_silent_mode_on_off = 31, // KISS silent Mode on/off
    DShot_cmd_signal_line_telemeetry_disable = 32,
    DShot_cmd_signal_line_continuous_erpm_telemetry = 33,
    DShot_cmd_MAX = 47
	// >47 are throttle values
} dshot_command_t;


/* The structure which contains configuration for DShot
 */
typedef struct dshot_conf_t {
	uint32_t			dma_base;
	dshot_dma_channel_t	channel;
	dshot_dma_stream_t	stream;
	uint32_t			start_ccr_register;
	uint8_t				channels_number;
} dshot_conf_t;
