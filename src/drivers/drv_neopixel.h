/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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

namespace neopixel
{
class NeoLEDData
{
public:
	enum eRGB {
		eB = 0,
		eR = 1,
		eG = 2
	};

	typedef union {
		uint8_t  grb[3];
		uint32_t l;
	} led_data_t;

	led_data_t  data{};
	NeoLEDData() {data.l = 0;}
	NeoLEDData(NeoLEDData &r) {data.l = r.data.l;}

	uint8_t &R() {return data.grb[eR];};
	uint8_t &G() {return data.grb[eG];};
	uint8_t &B() {return data.grb[eB];};
};
};

__BEGIN_DECLS

int neopixel_init(neopixel::NeoLEDData *led_data, int number_of_packages);
int neopixel_write(neopixel::NeoLEDData *led_data, int number_of_packages);
int neopixel_deinit(void);
int neopixel_write_no_dma(uint8_t r, uint8_t g, uint8_t b, uint8_t led_count);
__END_DECLS
