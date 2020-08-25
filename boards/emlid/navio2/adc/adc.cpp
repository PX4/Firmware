/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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

#include <board_config.h>

#include <drivers/drv_adc.h>

#include <px4_platform_common/log.h>

#include <fcntl.h>
#include <stdint.h>
#include <unistd.h>

#define ADC_SYSFS_PATH "/sys/kernel/rcio/adc"
#define ADC_MAX_CHAN 6
int _channels_fd[ADC_MAX_CHAN];

int px4_arch_adc_init(uint32_t base_address)
{
	for (int i = 0; i < ADC_MAX_CHAN; i++) {
		_channels_fd[i] = -1;
	}

	return 0;
}

void px4_arch_adc_uninit(uint32_t base_address)
{
	for (int i = 0; i < ADC_MAX_CHAN; i++) {
		::close(_channels_fd[i]);
		_channels_fd[i] = -1;
	}
}

uint32_t px4_arch_adc_sample(uint32_t base_address, unsigned channel)
{
	if (channel > ADC_MAX_CHAN) {
		PX4_ERR("channel %d out of range: %d", channel, ADC_MAX_CHAN);
		return UINT32_MAX; // error
	}

	// open channel if necessary
	if (_channels_fd[channel] == -1) {
		// ADC_SYSFS_PATH
		char channel_path[strlen(ADC_SYSFS_PATH) + 5] {};

		if (sprintf(channel_path, "%s/ch%d", ADC_SYSFS_PATH, channel) == -1) {
			PX4_ERR("adc channel: %d\n", channel);
			return UINT32_MAX; // error
		}

		_channels_fd[channel] = ::open(channel_path, O_RDONLY);
	}

	char buffer[10] {};

	if (::pread(_channels_fd[channel], buffer, sizeof(buffer), 0) < 0) {
		PX4_ERR("read channel %d failed", channel);
		return UINT32_MAX; // error
	}

	return atoi(buffer);
}

float px4_arch_adc_reference_v()
{
	return BOARD_ADC_POS_REF_V;
}

uint32_t px4_arch_adc_dn_fullcount()
{
	return 1 << 12; // 12 bit ADC
}
