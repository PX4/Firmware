/************************************************************************************
 *
 *   Copyright (C) 2016, 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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
 ************************************************************************************/

#include <px4_arch/spi_hw_description.h>
#include <drivers/drv_sensor.h>
#include <nuttx/spi/spi.h>

#include <px4_platform_common/px4_config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
//#include <unistd.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "arm_arch.h"
#include "chip.h"
#include <kinetis.h>
#include "board_config.h"
#include <systemlib/px4_macros.h>

#if defined(CONFIG_KINETIS_SPI0) || defined(CONFIG_KINETIS_SPI1) || defined(CONFIG_KINETIS_SPI2)

constexpr px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {
	initSPIBus(SPI::Bus::SPI0, {
		initSPIDevice(SPIDEV_FLASH(0), SPI::CS{GPIO::PortC, GPIO::Pin2})
	}),
	initSPIBus(SPI::Bus::SPI1, {
		initSPIDevice(DRV_IMU_DEVTYPE_ICM42688P, SPI::CS{GPIO::PortB, GPIO::Pin10}, SPI::DRDY{GPIO::PortE, GPIO::Pin9}),
		initSPIDevice(DRV_GYR_DEVTYPE_BMI088, SPI::CS{GPIO::PortB, GPIO::Pin9}, SPI::DRDY{GPIO::PortD, GPIO::Pin12}),
		initSPIDevice(DRV_ACC_DEVTYPE_BMI088, SPI::CS{GPIO::PortE, GPIO::Pin6}),
		initSPIDevice(DRV_DEVTYPE_UNUSED, SPI::CS{GPIO::PortA, GPIO::Pin19}), // CAL Memory
	}, {GPIO::PortB, GPIO::Pin8}),
	initSPIBusExternal(SPI::Bus::SPI2, {
		initSPIConfigExternal(SPI::CS{GPIO::PortB, GPIO::Pin20}),
		initSPIConfigExternal(SPI::CS{GPIO::PortD, GPIO::Pin15}),
	}),
};

static constexpr bool unused = validateSPIConfig(px4_spi_buses);


#define PX4_MK_GPIO(pin_ftmx, io)    ((((uint32_t)(pin_ftmx)) & ~(_PIN_MODE_MASK | _PIN_OPTIONS_MASK)) |(io))

/************************************************************************************
 * Public Functions
 ************************************************************************************/

__EXPORT void board_spi_reset(int ms, int bus_mask)
{
	/* Goal not to back feed the chips on the bus via IO lines */

	/* Next Change CS to inputs with pull downs */
	for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
		if (px4_spi_buses[bus].bus == PX4_BUS_NUMBER_TO_PX4(1)) {
			for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
				if (px4_spi_buses[bus].devices[i].cs_gpio != 0) {
					kinetis_pinconfig(PX4_MK_GPIO(px4_spi_buses[bus].devices[i].cs_gpio, GPIO_PULLDOWN));
				}
			}
		}
	}

	/* Turn all the int inputs to inputs with pull down  */

	kinetis_pinconfig(PX4_MK_GPIO(GPIO_nSPI1_DRDY1_BMI1088_ACCEL_INT1, GPIO_PULLDOWN));
	kinetis_pinconfig(PX4_MK_GPIO(GPIO_nSPI1_DRDY2_BMI1088_GYRO_INT2, GPIO_PULLDOWN));
	kinetis_pinconfig(PX4_MK_GPIO(GPIO_nSPI1_DRDY3_ICM42688_INT1, GPIO_PULLDOWN));

	/* Power Down The Sensors */

	VDD_3V3_SENSORS_EN(false);
	up_mdelay(ms);

	/* Power Up The Sensors */
	VDD_3V3_SENSORS_EN(true);
	up_mdelay(2);

	/* Restore all the CS to ouputs inactive */

	for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
		if (px4_spi_buses[bus].bus == PX4_BUS_NUMBER_TO_PX4(1)) {
			for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
				if (px4_spi_buses[bus].devices[i].cs_gpio != 0) {
					kinetis_pinconfig(px4_spi_buses[bus].devices[i].cs_gpio);
				}
			}
		}
	}

	/* Restore all the int inputs to inputs */

	kinetis_pinconfig(GPIO_nSPI1_DRDY1_BMI1088_ACCEL_INT1);
	kinetis_pinconfig(GPIO_nSPI1_DRDY2_BMI1088_GYRO_INT2);
	kinetis_pinconfig(GPIO_nSPI1_DRDY3_ICM42688_INT1);
}

/************************************************************************************
 * Name: fmuk66_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the NXP FMUK66-E board.
 *
 ************************************************************************************/

void fmuk66_spidev_initialize(void)
{
	board_spi_reset(10, 0xffff);

	for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
		for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
			if (px4_spi_buses[bus].devices[i].cs_gpio != 0) {
				kinetis_pinconfig(px4_spi_buses[bus].devices[i].cs_gpio);
			}
		}
	}
}

/************************************************************************************
 * Name: kinetis_spi_bus_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the NXP FMUK66 v3 board.
 *
 ************************************************************************************/
static const px4_spi_bus_t *_spi_bus0;
static const px4_spi_bus_t *_spi_bus1;
static const px4_spi_bus_t *_spi_bus2;

__EXPORT int fmuk66_spi_bus_initialize(void)
{
	for (int i = 0; i < SPI_BUS_MAX_BUS_ITEMS; ++i) {
		switch (px4_spi_buses[i].bus) {
		case PX4_BUS_NUMBER_TO_PX4(0): _spi_bus0 = &px4_spi_buses[i]; break;

		case PX4_BUS_NUMBER_TO_PX4(1): _spi_bus1 = &px4_spi_buses[i]; break;

		case PX4_BUS_NUMBER_TO_PX4(2): _spi_bus2 = &px4_spi_buses[i]; break;
		}
	}

	/* Configure SPI-based devices */

	struct spi_dev_s *spi_sensors = px4_spibus_initialize(PX4_BUS_NUMBER_TO_PX4(1));

	if (!spi_sensors) {
		syslog(LOG_ERR, "[boot] FAILED to initialize SPI port %d\n", 1);
		return -ENODEV;
	}

	/* Default bus 1 to 1MHz and de-assert the known chip selects.
	 */

	SPI_SETFREQUENCY(spi_sensors, 1 * 1000 * 1000);
	SPI_SETBITS(spi_sensors, 8);
	SPI_SETMODE(spi_sensors, SPIDEV_MODE0);

	/* Get the SPI port for the Memory */

	struct spi_dev_s *spi_memory = px4_spibus_initialize(PX4_BUS_NUMBER_TO_PX4(0));

	if (!spi_memory) {
		syslog(LOG_ERR, "[boot] FAILED to initialize SPI port %d\n", 0);
		return -ENODEV;
	}

	/* Default bus 0 to 12MHz and de-assert the known chip selects.
	 */

	SPI_SETFREQUENCY(spi_memory, 12 * 1000 * 1000);
	SPI_SETBITS(spi_memory, 8);
	SPI_SETMODE(spi_memory, SPIDEV_MODE3);

	/* Configure EXTERNAL SPI-based devices */

	struct spi_dev_s *spi_ext = px4_spibus_initialize(PX4_BUS_NUMBER_TO_PX4(2));

	if (!spi_ext) {
		syslog(LOG_ERR, "[boot] FAILED to initialize SPI port %d\n", 2);
		return -ENODEV;
	}

	/* Default external bus to 1MHz and de-assert the known chip selects.
	 */

	SPI_SETFREQUENCY(spi_ext, 8 * 1000 * 1000);
	SPI_SETBITS(spi_ext, 8);
	SPI_SETMODE(spi_ext, SPIDEV_MODE3);

	/* deselect all */
	for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
		for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
			if (px4_spi_buses[bus].devices[i].cs_gpio != 0) {
				SPI_SELECT(spi_ext, px4_spi_buses[bus].devices[i].devid, false);
			}
		}
	}

	return OK;

}

/************************************************************************************
 * Name:  kinetis_spi[n]select, kinetis_spi[n]status, and kinetis_spi[n]cmddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic.  They are
 *   implementations of the select, status, and cmddata methods of the SPI interface
 *   defined by struct spi_ops_s (see include/nuttx/spi/spi.h). All other methods
 *   including kinetis_spibus_initialize()) are provided by common Kinetis logic.
 *   To use this common SPI logic on your board:
 *
 *   1. Provide logic in kinetis_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide kinetis_spi[n]select() and kinetis_spi[n]status() functions
 *      in your board-specific logic.  These functions will perform chip selection
 *      and status operations using GPIOs in the way your board is configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *      kinetis_spi[n]cmddata() functions in your board-specific logic.  These
 *      functions will perform cmd/data selection operations using GPIOs in the way
 *      your board is configured.
 *   3. Add a call to kinetis_spibus_initialize() in your low level application
 *      initialization logic
 *   4. The handle returned by kinetis_spibus_initialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ************************************************************************************/

static inline void kinetis_spixselect(const px4_spi_bus_t *bus, struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
		if (bus->devices[i].cs_gpio == 0) {
			break;
		}

		if (devid == bus->devices[i].devid) {
			// SPI select is active low, so write !selected to select the device
			kinetis_gpiowrite(bus->devices[i].cs_gpio, !selected);
		}
	}
}

void kinetis_spi0select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
	kinetis_spixselect(_spi_bus0, dev, devid, selected);
}

uint8_t kinetis_spi0status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}

void kinetis_spi1select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
	kinetis_spixselect(_spi_bus1, dev, devid, selected);
}

uint8_t kinetis_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}

void kinetis_spi2select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
	kinetis_spixselect(_spi_bus2, dev, devid, selected);
}

uint8_t kinetis_spi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}


#endif /* CONFIG_KINETIS_SPI0 || CONFIG_KINETIS_SPI1 || CONFIG_KINETIS_SPI2 */
