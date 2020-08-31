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

#include <board_config.h>

#define I2C_BUS_MAX_BUS_ITEMS PX4_NUMBER_I2C_BUSES

struct px4_i2c_bus_t {
	int bus{-1}; ///< physical bus number (1, ...) (-1 means this is unused)
	bool is_external; ///< static external configuration. Use px4_i2c_bus_external() to check if a bus is really external
};

__EXPORT extern const px4_i2c_bus_t px4_i2c_buses[I2C_BUS_MAX_BUS_ITEMS]; ///< board-specific I2C bus configuration

/**
 * runtime-check if a board has a specific bus as external.
 * This can be overridden by a board to add run-time checks.
 */
__EXPORT bool px4_i2c_bus_external(const px4_i2c_bus_t &bus);

/**
 * runtime-check if a board has a specific bus as external.
 */
static inline bool px4_i2c_bus_external(int bus)
{
	for (int i = 0; i < I2C_BUS_MAX_BUS_ITEMS; ++i) {
		if (px4_i2c_buses[i].bus == bus) {
			return px4_i2c_bus_external(px4_i2c_buses[i]);
		}
	}

	return true;
}


/**
 * @class I2CBusIterator
 * Iterate over configured I2C buses by the board
 */
class I2CBusIterator
{
public:
	enum class FilterType {
		All, ///< specific or all buses
		InternalBus, ///< specific or all internal buses
		ExternalBus, ///< specific or all external buses
	};

	/**
	 * @param bus specify bus: starts with 1, -1=all. Internal: arch-specific bus numbering is used,
	 *             external: n-th external bus
	 */
	I2CBusIterator(FilterType filter, int bus = -1)
		: _filter(filter), _bus(bus) {}

	bool next();

	const px4_i2c_bus_t &bus() const { return px4_i2c_buses[_index]; }

	int externalBusIndex() const { return _external_bus_counter; }

	bool external() const { return px4_i2c_bus_external(bus()); }

private:
	const FilterType _filter;
	const int _bus;
	int _index{-1};
	int _external_bus_counter{0};
};
