/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "../../../stm32_common/include/px4_arch/spi_hw_description.h"

constexpr bool validateSPIConfig(const px4_spi_bus_t spi_busses_conf[SPI_BUS_MAX_BUS_ITEMS])
{
	const bool nuttx_enabled_spi_buses[] = {
#ifdef CONFIG_STM32_SPI1
		true,
#else
		false,
#endif
#ifdef CONFIG_STM32_SPI2
		true,
#else
		false,
#endif
#ifdef CONFIG_STM32_SPI3
		true,
#else
		false,
#endif
#ifdef CONFIG_STM32_SPI4
		true,
#else
		false,
#endif
#ifdef CONFIG_STM32_SPI5
		true,
#else
		false,
#endif
#ifdef CONFIG_STM32_SPI6
		true,
#else
		false,
#endif
	};

	for (unsigned i = 0; i < sizeof(nuttx_enabled_spi_buses) / sizeof(nuttx_enabled_spi_buses[0]); ++i) {
		bool found_bus = false;

		for (int j = 0; j < SPI_BUS_MAX_BUS_ITEMS; ++j) {
			if (spi_busses_conf[j].bus == (int)i + 1) {
				found_bus = true;
			}
		}

		// Either the bus is enabled in NuttX and configured in spi_busses_conf, or disabled and not configured
		constexpr_assert(found_bus == nuttx_enabled_spi_buses[i], "SPI bus config mismatch (CONFIG_STM32_SPIx)");
	}

	return false;
}

