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

#include <stdint.h>
#include <board_config.h>

/*
 * Helper macros to handle device ID's. They are used to match drivers against SPI buses and chip-select signals.
 * They match with corresponding definitions in NuttX.
 * 'type' is typically PX4_SPI_DEVICE_ID for PX4 drivers, and 'index' is used for the driver (DRV_*) or chip-select index.
 */
#define PX4_SPIDEV_ID(type, index)  ((((type) & 0xffff) << 16) | ((index) & 0xffff))
#define PX4_SPI_DEVICE_ID         (1 << 12)
#define PX4_SPI_DEV_ID(devid)     ((devid) & 0xffff)
#define PX4_SPIDEVID_TYPE(devid) (((uint32_t)(devid) >> 16) & 0xffff)

typedef uint32_t spi_drdy_gpio_t;

#define SPI_BUS_MAX_DEVICES 6
struct px4_spi_bus_device_t {
	uint32_t cs_gpio; ///< chip-select GPIO (0 if this device is not used)
	spi_drdy_gpio_t drdy_gpio; ///< data ready GPIO (0 if not set)
	uint32_t devid; ///< SPIDEV_ID(type,index). For PX4 devices on NuttX: index is the device type, and for external buses the CS index
	uint16_t devtype_driver; ///< driver device type, e.g. DRV_IMU_DEVTYPE_ICM20689 (on NuttX: PX4_SPI_DEV_ID(devid) == devtype_driver)
};

struct px4_spi_bus_devices_t {
	px4_spi_bus_device_t devices[SPI_BUS_MAX_DEVICES];
};

struct px4_spi_bus_t {
	px4_spi_bus_device_t devices[SPI_BUS_MAX_DEVICES];
	int bus{-1}; ///< physical bus number (1, ...) (-1 means this is unused)
	uint32_t power_enable_gpio{0}; ///< GPIO (if non-zero) to control the power of the attached devices on this bus (0 means power is off)
	bool is_external; ///< static external configuration. Use px4_spi_bus_external() to check if a bus is really external
	bool requires_locking; ///< whether the bus should be locked during transfers (true if NuttX drivers access the bus)
};


struct px4_spi_bus_all_hw_t {
	px4_spi_bus_t buses[SPI_BUS_MAX_BUS_ITEMS];
	int board_hw_version{-1}; ///< 0=default, >0 for a specific revision (see board_get_hw_version), -1=unused
};

#if BOARD_NUM_SPI_CFG_HW_VERSIONS > 1
/**
 * initialze px4_spi_buses from px4_spi_buses_all_hw and the hardware version.
 * Call this on early boot before anything else accesses px4_spi_buses (e.g. from stm32_spiinitialize).
 */
__EXPORT void px4_set_spi_buses_from_hw_version();

__EXPORT extern const px4_spi_bus_all_hw_t
px4_spi_buses_all_hw[BOARD_NUM_SPI_CFG_HW_VERSIONS]; ///< board-specific SPI bus configuration all hw versions

__EXPORT extern const px4_spi_bus_t *px4_spi_buses; ///< board-specific SPI bus configuration for current board revision
#else
static inline void px4_set_spi_buses_from_hw_version() {}
__EXPORT extern const px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS]; ///< board-specific SPI bus configuration
#endif


/**
 * Find a SPI bus given a device ID. Note: only internal buses are checked.
 * @return the bus or -1
 */
__EXPORT int px4_find_spi_bus(uint32_t devid);

/**
 * Check if a bus requires locking during a SPI transfer (because it is potentially accessed by different threads)
 */
__EXPORT bool px4_spi_bus_requires_locking(int bus);

/**
 * runtime-check if a board has a specific bus as external.
 * This can be overridden by a board to add run-time checks.
 */
__EXPORT bool px4_spi_bus_external(const px4_spi_bus_t &bus);

/**
 * runtime-check if a board has a specific bus as external.
 */
static inline bool px4_spi_bus_external(int bus)
{
	for (int i = 0; i < SPI_BUS_MAX_BUS_ITEMS; ++i) {
		if (px4_spi_buses[i].bus == bus) {
			return px4_spi_bus_external(px4_spi_buses[i]);
		}
	}

	return true;
}


/**
 * @class SPIBusIterator
 * Iterate over configured SPI buses by the board
 */
class SPIBusIterator
{
public:
	enum class FilterType {
		InternalBus, ///< specific or all internal buses
		ExternalBus, ///< specific external bus + CS index
	};

	/**
	 * Constructor
	 * Note: only for devices of type PX4_SPI_DEVICE_ID
	 * @param filter
	 * @param devid_driver_index DRV_* or chip-select index starting from 1
	 * @param bus starts with 1 (-1=all, but only for internal). Numbering for internal is arch-specific, for external
	 *            it is the n-th external bus.
	 */
	SPIBusIterator(FilterType filter, uint16_t devid_driver_index, int bus = -1)
		: _filter(filter), _devid_driver_index(devid_driver_index),
		  _bus(filter == FilterType::ExternalBus && bus == -1 ? 1 : bus) {}

	bool next();

	const px4_spi_bus_t &bus() const { return px4_spi_buses[_index]; }
	spi_drdy_gpio_t DRDYGPIO() const { return px4_spi_buses[_index].devices[_bus_device_index].drdy_gpio; }

	uint32_t devid() const { return px4_spi_buses[_index].devices[_bus_device_index].devid; }

	int externalBusIndex() const { return _external_bus_counter; }

	bool external() const { return px4_spi_bus_external(bus()); }

private:
	const FilterType _filter;
	const uint16_t _devid_driver_index;
	const int _bus;
	int _index{-1};
	int _external_bus_counter{0};
	int _bus_device_index{0};
};
