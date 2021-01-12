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

#include <board_config.h>
#include <systemlib/px4_macros.h>
#include <px4_platform_common/spi.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <unistd.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>
#include <arm_arch.h>
#include <chip.h>
#include <stm32_gpio.h>

// wrapper for stm32f7
#ifdef CONFIG_STM32F7_SPI1
#define CONFIG_STM32_SPI1
#endif
#ifdef CONFIG_STM32F7_SPI2
#define CONFIG_STM32_SPI2
#endif
#ifdef CONFIG_STM32F7_SPI3
#define CONFIG_STM32_SPI3
#endif
#ifdef CONFIG_STM32F7_SPI4
#define CONFIG_STM32_SPI4
#endif
#ifdef CONFIG_STM32F7_SPI5
#define CONFIG_STM32_SPI5
#endif
#ifdef CONFIG_STM32F7_SPI6
#define CONFIG_STM32_SPI6
#endif

// wrapper for stm32h7
#ifdef CONFIG_STM32H7_SPI1
#define CONFIG_STM32_SPI1
#endif
#ifdef CONFIG_STM32H7_SPI2
#define CONFIG_STM32_SPI2
#endif
#ifdef CONFIG_STM32H7_SPI3
#define CONFIG_STM32_SPI3
#endif
#ifdef CONFIG_STM32H7_SPI4
#define CONFIG_STM32_SPI4
#endif
#ifdef CONFIG_STM32H7_SPI5
#define CONFIG_STM32_SPI5
#endif
#ifdef CONFIG_STM32H7_SPI6
#define CONFIG_STM32_SPI6
#endif

static const px4_spi_bus_t *_spi_bus1;
static const px4_spi_bus_t *_spi_bus2;
static const px4_spi_bus_t *_spi_bus3;
static const px4_spi_bus_t *_spi_bus4;
static const px4_spi_bus_t *_spi_bus5;
static const px4_spi_bus_t *_spi_bus6;

static void spi_bus_configgpio_cs(const px4_spi_bus_t *bus)
{
	for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
		if (bus->devices[i].cs_gpio != 0) {
			px4_arch_configgpio(bus->devices[i].cs_gpio);
		}
	}
}

__EXPORT void stm32_spiinitialize()
{
	px4_set_spi_buses_from_hw_version();
	board_control_spi_sensors_power_configgpio();
	board_control_spi_sensors_power(true, 0xffff);

	for (int i = 0; i < SPI_BUS_MAX_BUS_ITEMS; ++i) {
		switch (px4_spi_buses[i].bus) {
		case 1: _spi_bus1 = &px4_spi_buses[i]; break;

		case 2: _spi_bus2 = &px4_spi_buses[i]; break;

		case 3: _spi_bus3 = &px4_spi_buses[i]; break;

		case 4: _spi_bus4 = &px4_spi_buses[i]; break;

		case 5: _spi_bus5 = &px4_spi_buses[i]; break;

		case 6: _spi_bus6 = &px4_spi_buses[i]; break;
		}
	}

#ifdef CONFIG_STM32_SPI1
	ASSERT(_spi_bus1);

	if (board_has_bus(BOARD_SPI_BUS, 1)) {
		spi_bus_configgpio_cs(_spi_bus1);
	}

#endif // CONFIG_STM32_SPI1


#if defined(CONFIG_STM32_SPI2)
	ASSERT(_spi_bus2);

	if (board_has_bus(BOARD_SPI_BUS, 2)) {
		spi_bus_configgpio_cs(_spi_bus2);
	}

#endif // CONFIG_STM32_SPI2

#ifdef CONFIG_STM32_SPI3
	ASSERT(_spi_bus3);

	if (board_has_bus(BOARD_SPI_BUS, 3)) {
		spi_bus_configgpio_cs(_spi_bus3);
	}

#endif // CONFIG_STM32_SPI3

#ifdef CONFIG_STM32_SPI4
	ASSERT(_spi_bus4);

	if (board_has_bus(BOARD_SPI_BUS, 4)) {
		spi_bus_configgpio_cs(_spi_bus4);
	}

#endif // CONFIG_STM32_SPI4


#ifdef CONFIG_STM32_SPI5
	ASSERT(_spi_bus5);

	if (board_has_bus(BOARD_SPI_BUS, 5)) {
		spi_bus_configgpio_cs(_spi_bus5);
	}

#endif // CONFIG_STM32_SPI5


#ifdef CONFIG_STM32_SPI6
	ASSERT(_spi_bus6);

	if (board_has_bus(BOARD_SPI_BUS, 6)) {
		spi_bus_configgpio_cs(_spi_bus6);
	}

#endif // CONFIG_STM32_SPI6
}

static inline void stm32_spixselect(const px4_spi_bus_t *bus, struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
		if (bus->devices[i].cs_gpio == 0) {
			break;
		}

		if (devid == bus->devices[i].devid) {
			// SPI select is active low, so write !selected to select the device
			stm32_gpiowrite(bus->devices[i].cs_gpio, !selected);
		}
	}
}


/************************************************************************************
 * Name: stm32_spi1select and stm32_spi1status
 *
 * Description:
 *   Called by stm32 spi driver on bus 1.
 *
 ************************************************************************************/
#ifdef CONFIG_STM32_SPI1

__EXPORT void stm32_spi1select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	stm32_spixselect(_spi_bus1, dev, devid, selected);
}

__EXPORT uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif // CONFIG_STM32_SPI1

/************************************************************************************
 * Name: stm32_spi2select and stm32_spi2status
 *
 * Description:
 *   Called by stm32 spi driver on bus 2.
 *
 ************************************************************************************/
#if defined(CONFIG_STM32_SPI2)
__EXPORT void stm32_spi2select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	stm32_spixselect(_spi_bus2, dev, devid, selected);
}

__EXPORT uint8_t stm32_spi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif // CONFIG_STM32_SPI2

/************************************************************************************
 * Name: stm32_spi3select and stm32_spi3status
 *
 * Description:
 *   Called by stm32 spi driver on bus 3.
 *
 ************************************************************************************/
#if defined(CONFIG_STM32_SPI3)
__EXPORT void stm32_spi3select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	stm32_spixselect(_spi_bus3, dev, devid, selected);
}

__EXPORT uint8_t stm32_spi3status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif // CONFIG_STM32_SPI3

/************************************************************************************
 * Name: stm32_spi4select and stm32_spi4status
 *
 * Description:
 *   Called by stm32 spi driver on bus 4.
 *
 ************************************************************************************/
#ifdef CONFIG_STM32_SPI4

__EXPORT void stm32_spi4select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	stm32_spixselect(_spi_bus4, dev, devid, selected);
}

__EXPORT uint8_t stm32_spi4status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif // CONFIG_STM32_SPI4

/************************************************************************************
 * Name: stm32_spi5select and stm32_spi5status
 *
 * Description:
 *   Called by stm32 spi driver on bus 5.
 *
 ************************************************************************************/
#ifdef CONFIG_STM32_SPI5

__EXPORT void stm32_spi5select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	stm32_spixselect(_spi_bus5, dev, devid, selected);
}

__EXPORT uint8_t stm32_spi5status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif // CONFIG_STM32_SPI5

/************************************************************************************
 * Name: stm32_spi6select and stm32_spi6status
 *
 * Description:
 *   Called by stm32 spi driver on bus 6.
 *
 ************************************************************************************/
#ifdef CONFIG_STM32_SPI6

__EXPORT void stm32_spi6select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	stm32_spixselect(_spi_bus6, dev, devid, selected);
}

__EXPORT uint8_t stm32_spi6status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif // CONFIG_STM32_SPI6


void board_control_spi_sensors_power(bool enable_power, int bus_mask)
{
	const px4_spi_bus_t *buses = px4_spi_buses;
	// this might be called very early on boot where we have not yet determined the hw version
	// (we expect all versions to have the same power GPIO)
#if BOARD_NUM_SPI_CFG_HW_VERSIONS > 1

	if (!buses) {
		buses = &px4_spi_buses_all_hw[0].buses[0];
	}

#endif

	for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
		if (buses[bus].bus == -1) {
			break;
		}

		const bool bus_matches = bus_mask & (1 << (buses[bus].bus - 1));

		if (buses[bus].power_enable_gpio == 0 ||
		    !board_has_bus(BOARD_SPI_BUS, buses[bus].bus) ||
		    !bus_matches) {
			continue;
		}

		px4_arch_gpiowrite(buses[bus].power_enable_gpio, enable_power ? 1 : 0);
	}
}

void board_control_spi_sensors_power_configgpio()
{
	const px4_spi_bus_t *buses = px4_spi_buses;
	// this might be called very early on boot where we have yet not determined the hw version
	// (we expect all versions to have the same power GPIO)
#if BOARD_NUM_SPI_CFG_HW_VERSIONS > 1

	if (!buses) {
		buses = &px4_spi_buses_all_hw[0].buses[0];
	}

#endif

	for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
		if (buses[bus].bus == -1) {
			break;
		}

		if (buses[bus].power_enable_gpio == 0 ||
		    !board_has_bus(BOARD_SPI_BUS, buses[bus].bus)) {
			continue;
		}

		px4_arch_configgpio(buses[bus].power_enable_gpio);
	}
}

#define _PIN_OFF(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz))

__EXPORT void board_spi_reset(int ms, int bus_mask)
{
	bool has_power_enable = false;

	// disable SPI bus
	for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
		if (px4_spi_buses[bus].bus == -1) {
			break;
		}

		const bool bus_requested = bus_mask & (1 << (px4_spi_buses[bus].bus - 1));

		if (px4_spi_buses[bus].power_enable_gpio == 0 ||
		    !board_has_bus(BOARD_SPI_BUS, px4_spi_buses[bus].bus) ||
		    !bus_requested) {
			continue;
		}

		has_power_enable = true;

		for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
			if (px4_spi_buses[bus].devices[i].cs_gpio != 0) {
				px4_arch_configgpio(_PIN_OFF(px4_spi_buses[bus].devices[i].cs_gpio));
			}

			if (px4_spi_buses[bus].devices[i].drdy_gpio != 0) {
				px4_arch_configgpio(_PIN_OFF(px4_spi_buses[bus].devices[i].drdy_gpio));
			}
		}

#if defined(CONFIG_STM32_SPI1)

		if (px4_spi_buses[bus].bus == 1) {
			px4_arch_configgpio(_PIN_OFF(GPIO_SPI1_SCK));
			px4_arch_configgpio(_PIN_OFF(GPIO_SPI1_MISO));
			px4_arch_configgpio(_PIN_OFF(GPIO_SPI1_MOSI));
		}

#endif
#if defined(CONFIG_STM32_SPI2)

		if (px4_spi_buses[bus].bus == 2) {
			px4_arch_configgpio(_PIN_OFF(GPIO_SPI2_SCK));
			px4_arch_configgpio(_PIN_OFF(GPIO_SPI2_MISO));
			px4_arch_configgpio(_PIN_OFF(GPIO_SPI2_MOSI));
		}

#endif
#if defined(CONFIG_STM32_SPI3)

		if (px4_spi_buses[bus].bus == 3) {
			px4_arch_configgpio(_PIN_OFF(GPIO_SPI3_SCK));
			px4_arch_configgpio(_PIN_OFF(GPIO_SPI3_MISO));
			px4_arch_configgpio(_PIN_OFF(GPIO_SPI3_MOSI));
		}

#endif
#if defined(CONFIG_STM32_SPI4)

		if (px4_spi_buses[bus].bus == 4) {
			px4_arch_configgpio(_PIN_OFF(GPIO_SPI4_SCK));
			px4_arch_configgpio(_PIN_OFF(GPIO_SPI4_MISO));
			px4_arch_configgpio(_PIN_OFF(GPIO_SPI4_MOSI));
		}

#endif
#if defined(CONFIG_STM32_SPI5)

		if (px4_spi_buses[bus].bus == 5) {
			px4_arch_configgpio(_PIN_OFF(GPIO_SPI5_SCK));
			px4_arch_configgpio(_PIN_OFF(GPIO_SPI5_MISO));
			px4_arch_configgpio(_PIN_OFF(GPIO_SPI5_MOSI));
		}

#endif
#if defined(CONFIG_STM32_SPI6)

		if (px4_spi_buses[bus].bus == 6) {
			px4_arch_configgpio(_PIN_OFF(GPIO_SPI6_SCK));
			px4_arch_configgpio(_PIN_OFF(GPIO_SPI6_MISO));
			px4_arch_configgpio(_PIN_OFF(GPIO_SPI6_MOSI));
		}

#endif
	}

	if (!has_power_enable) {
		// board does not have power control over any of the sensor buses
		return;
	}

	// set the sensor rail(s) off
	board_control_spi_sensors_power(false, bus_mask);

	// wait for the sensor rail to reach GND
	usleep(ms * 1000);
	syslog(LOG_DEBUG, "reset done, %d ms\n", ms);

	/* re-enable power */

	// switch the sensor rail back on
	board_control_spi_sensors_power(true, bus_mask);

	/* wait a bit before starting SPI, different times didn't influence results */
	usleep(100);

	/* reconfigure the SPI pins */
	for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
		if (px4_spi_buses[bus].bus == -1) {
			break;
		}

		const bool bus_requested = bus_mask & (1 << (px4_spi_buses[bus].bus - 1));

		if (px4_spi_buses[bus].power_enable_gpio == 0 ||
		    !board_has_bus(BOARD_SPI_BUS, px4_spi_buses[bus].bus) ||
		    !bus_requested) {
			continue;
		}

		for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
			if (px4_spi_buses[bus].devices[i].cs_gpio != 0) {
				px4_arch_configgpio(px4_spi_buses[bus].devices[i].cs_gpio);
			}

			if (px4_spi_buses[bus].devices[i].drdy_gpio != 0) {
				px4_arch_configgpio(px4_spi_buses[bus].devices[i].drdy_gpio);
			}
		}

#if defined(CONFIG_STM32_SPI1)

		if (px4_spi_buses[bus].bus == 1) {
			px4_arch_configgpio(GPIO_SPI1_SCK);
			px4_arch_configgpio(GPIO_SPI1_MISO);
			px4_arch_configgpio(GPIO_SPI1_MOSI);
		}

#endif
#if defined(CONFIG_STM32_SPI2)

		if (px4_spi_buses[bus].bus == 2) {
			px4_arch_configgpio(GPIO_SPI2_SCK);
			px4_arch_configgpio(GPIO_SPI2_MISO);
			px4_arch_configgpio(GPIO_SPI2_MOSI);
		}

#endif
#if defined(CONFIG_STM32_SPI3)

		if (px4_spi_buses[bus].bus == 3) {
			px4_arch_configgpio(GPIO_SPI3_SCK);
			px4_arch_configgpio(GPIO_SPI3_MISO);
			px4_arch_configgpio(GPIO_SPI3_MOSI);
		}

#endif
#if defined(CONFIG_STM32_SPI4)

		if (px4_spi_buses[bus].bus == 4) {
			px4_arch_configgpio(GPIO_SPI4_SCK);
			px4_arch_configgpio(GPIO_SPI4_MISO);
			px4_arch_configgpio(GPIO_SPI4_MOSI);
		}

#endif
#if defined(CONFIG_STM32_SPI5)

		if (px4_spi_buses[bus].bus == 5) {
			px4_arch_configgpio(GPIO_SPI5_SCK);
			px4_arch_configgpio(GPIO_SPI5_MISO);
			px4_arch_configgpio(GPIO_SPI5_MOSI);
		}

#endif
#if defined(CONFIG_STM32_SPI6)

		if (px4_spi_buses[bus].bus == 6) {
			px4_arch_configgpio(GPIO_SPI6_SCK);
			px4_arch_configgpio(GPIO_SPI6_MISO);
			px4_arch_configgpio(GPIO_SPI6_MOSI);
		}

#endif
	}
}
