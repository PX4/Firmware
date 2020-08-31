/****************************************************************************
 *
 *   Copyright (C) 2012, 2019 PX4 Development Team. All rights reserved.
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
 * @file drv_adc.h
 *
 * ADC driver interface.
 *
 */

#pragma once

#include <stdint.h>
#include <sys/ioctl.h>
#include <systemlib/px4_macros.h>
#include <uORB/topics/adc_report.h>

/* Define the PX4 low level format ADC and the maximum
 * number of channels that can be returned by a lowlevel
 * ADC driver. Drivers may return less than PX4_MAX_ADC_CHANNELS
 * but no more than PX4_MAX_ADC_CHANNELS.
 *
 */
#define PX4_MAX_ADC_CHANNELS arraySize(adc_report_s::channel_id)
typedef struct __attribute__((packed)) px4_adc_msg_t {
	uint8_t      am_channel;               /* The 8-bit ADC Channel */
	int32_t      am_data;                  /* ADC convert result (4 bytes) */
} px4_adc_msg_t;


#define BUILTIN_ADC_DEVID	0xffffffff	// TODO: integrate into existing ID management


__BEGIN_DECLS

/**
 * Initialize ADC hardware
 * @param base_address architecture-specific address to specify the ADC
 * @return 0 on success, <0 error otherwise
 */
int px4_arch_adc_init(uint32_t base_address);

/**
 * Uninitialize ADC hardware
 * @param base_address architecture-specific address to specify the ADC
 */
void px4_arch_adc_uninit(uint32_t base_address);

/**
 * Read a sample from the ADC
 * @param base_address architecture-specific address to specify the ADC
 * @param channel specify the channel
 * @return sample, 0xffffffff on error
 */
uint32_t px4_arch_adc_sample(uint32_t base_address, unsigned channel);

/**
 * Get the ADC positive reference voltage
 * N.B This assume that all ADC channels share the same vref.
 * @return v_ref
 */
float px4_arch_adc_reference_v(void);

/**
 * Get the temperature sensor channel bitmask
 */
uint32_t px4_arch_adc_temp_sensor_mask(void);

/**
 * Get the adc digital number full count
 */
uint32_t px4_arch_adc_dn_fullcount(void);

__END_DECLS
