/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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
 * @file init.c
 *
 * omnibusf4sd-specific early startup code.  This file implements the
 * board_app_initialize() function that is called early by nsh during startup.
 *
 * Code here is run before the rcS script is invoked; it should start required
 * subsystems and perform board-specific initialization.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <syslog.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/mmcsd.h>
#include <nuttx/analog/adc.h>
#include <nuttx/mm/gran.h>

#include <stm32.h>
#include "board_config.h"
#include <stm32_uart.h>

#include <arch/board/board.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_board_led.h>

#include <systemlib/px4_macros.h>

#include <px4_arch/io_timer.h>
#include <px4_platform_common/init.h>
#include <px4_platform/board_dma_alloc.h>

# if defined(FLASH_BASED_PARAMS)
#  include <parameters/flashparams/flashfs.h>
#endif

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/*
 * Ideally we'd be able to get these from arm_internal.h,
 * but since we want to be able to disable the NuttX use
 * of leds for system indication at will and there is no
 * separate switch, we need to build independent of the
 * CONFIG_ARCH_LEDS configuration switch.
 */
__BEGIN_DECLS
extern void led_init(void);
extern void led_on(int led);
extern void led_off(int led);
__END_DECLS

/****************************************************************************
 * Protected Functions
 ****************************************************************************/
/****************************************************************************
 * Public Functions
 ****************************************************************************/
/************************************************************************************
 * Name: board_peripheral_reset
 *
 * Description:
 *
 ************************************************************************************/
__EXPORT void board_peripheral_reset(int ms)
{
	UNUSED(ms);
}

/************************************************************************************
 * Name: board_on_reset
 *
 * Description:
 * Optionally provided function called on entry to board_system_reset
 * It should perform any house keeping prior to the rest.
 *
 * status - 1 if resetting to boot loader
 *          0 if just resetting
 *
 ************************************************************************************/
__EXPORT void board_on_reset(int status)
{
	/* configure the GPIO pins to outputs and keep them low */
	for (int i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; ++i) {
		px4_arch_configgpio(io_timer_channel_get_gpio_output(i));
	}

	/* On resets invoked from system (not boot) insure we establish a low
	 * output state (discharge the pins) on PWM pins before they become inputs.
	 */

	if (status >= 0) {
		up_mdelay(400);
	}
}

/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the initialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

__EXPORT void
stm32_boardinitialize(void)
{
	/* Reset all PWM to Low outputs */

	board_on_reset(-1);

	/* configure LEDs */
	board_autoled_initialize();


	/* configure ADC pins */
	stm32_configgpio(GPIO_ADC1_IN11);	/* BATT_VOLTAGE_SENS */
	stm32_configgpio(GPIO_ADC1_IN13);	/* BATT_CURRENT_SENS */
	stm32_configgpio(GPIO_ADC1_IN12);	/* RSSI analog in (TX of UART4 instead) */

	// TODO: power peripherals
	///* configure power supply control/sense pins */
	//stm32_configgpio(GPIO_PERIPH_3V3_EN);
	//stm32_configgpio(GPIO_VDD_BRICK_VALID);
	//stm32_configgpio(GPIO_VDD_USB_VALID);

	// TODO: 3v3 Sensor?
	///* Start with Sensor voltage off We will enable it
	// * in board_app_initialize
	// */
	//stm32_configgpio(GPIO_VDD_3V3_SENSORS_EN);

	// TODO: SBUS inversion? SPEK power?
	//stm32_configgpio(GPIO_SBUS_INV);
	//stm32_configgpio(GPIO_SPEKTRUM_PWR_EN);

	// TODO: $$$ Unused?
	//stm32_configgpio(GPIO_8266_GPIO0);
	//stm32_configgpio(GPIO_8266_PD);
	//stm32_configgpio(GPIO_8266_RST);

	/* Safety - led don in led driver */

	// TODO: unused?
	//stm32_configgpio(GPIO_BTN_SAFETY);

	// TODO: RSSI
	//stm32_configgpio(GPIO_RSSI_IN);

	stm32_configgpio(GPIO_PPM_IN);

	/* configure SPI all interfaces GPIO */

	stm32_spiinitialize();

}

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initalization logic and the the
 *         matching application logic.  The value cold be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

static struct spi_dev_s *spi1;
static struct spi_dev_s *spi2;
static struct spi_dev_s *spi3;

__EXPORT int board_app_initialize(uintptr_t arg)
{
	px4_platform_init();

	/* configure the DMA allocator */

	if (board_dma_alloc_init() < 0) {
		syslog(LOG_ERR, "DMA alloc FAILED\n");
	}

	/* set up the serial DMA polling */
	static struct hrt_call serial_dma_call;
	struct timespec ts;

	/*
	 * Poll at 1ms intervals for received bytes that have not triggered
	 * a DMA event.
	 */
	ts.tv_sec = 0;
	ts.tv_nsec = 1000000;

	hrt_call_every(&serial_dma_call,
		       ts_to_abstime(&ts),
		       ts_to_abstime(&ts),
		       (hrt_callout)stm32_serial_dma_poll,
		       NULL);

	/* initial LED state */
	drv_led_start();
	led_off(LED_BLUE);

	if (board_hardfault_init(2, true) != 0) {
		led_on(LED_BLUE);
	}


	/* Configure SPI-based devices */

	// SPI1: MPU6000
	spi1 = stm32_spibus_initialize(1);

	if (!spi1) {
		syslog(LOG_ERR, "[boot] FAILED to initialize SPI port 1\n");
		led_on(LED_BLUE);
		return -ENODEV;
	}

	/* Default SPI1 to 1MHz and de-assert the known chip selects. */
	SPI_SETFREQUENCY(spi1, 10000000);
	SPI_SETBITS(spi1, 8);
	SPI_SETMODE(spi1, SPIDEV_MODE3);
	up_udelay(20);

	// SPI2: SDCard
	/* Get the SPI port for the microSD slot */
	spi2 = stm32_spibus_initialize(CONFIG_NSH_MMCSDSPIPORTNO);

	if (!spi2) {
		syslog(LOG_ERR, "[boot] FAILED to initialize SPI port %d\n", CONFIG_NSH_MMCSDSPIPORTNO);
		led_on(LED_BLUE);
		return -ENODEV;
	}

	/* Now bind the SPI interface to the MMCSD driver */
	int result = mmcsd_spislotinitialize(CONFIG_NSH_MMCSDMINOR, CONFIG_NSH_MMCSDSLOTNO, spi2);

	if (result != OK) {
		led_on(LED_BLUE);
		syslog(LOG_ERR, "[boot] FAILED to bind SPI port 2 to the MMCSD driver\n");
		return -ENODEV;
	}

	up_udelay(20);


	// SPI3: OSD / Baro
	spi3 = stm32_spibus_initialize(3);

	if (!spi3) {
		syslog(LOG_ERR, "[boot] FAILED to initialize SPI port 3\n");
		led_on(LED_BLUE);
		return -ENODEV;
	}

	/* Copied from fmu-v4
	 * Default SPI3 to 12MHz and de-assert the known chip selects.
	 * MS5611 has max SPI clock speed of 20MHz
	 */

	// BMP280 max SPI speed is 10 MHz
	SPI_SETFREQUENCY(spi3, 10 * 1000 * 1000);
	SPI_SETBITS(spi3, 8);
	SPI_SETMODE(spi3, SPIDEV_MODE3);
	up_udelay(20);

#if defined(FLASH_BASED_PARAMS)
	static sector_descriptor_t params_sector_map[] = {
		{1, 16 * 1024, 0x08004000},
		{0, 0, 0},
	};

	/* Initialize the flashfs layer to use heap allocated memory */
	result = parameter_flashfs_init(params_sector_map, NULL, 0);

	if (result != OK) {
		syslog(LOG_ERR, "[boot] FAILED to init params in FLASH %d\n", result);
		led_on(LED_AMBER);
		return -ENODEV;
	}

#endif

	/* Configure the HW based on the manifest */

	px4_platform_configure();

	return OK;
}
