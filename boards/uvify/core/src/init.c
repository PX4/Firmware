/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * UVify Core specific early startup code.  This file implements the
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

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sdio.h>
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

#include <px4_platform_common/init.h>
#include <px4_platform/board_dma_alloc.h>

#include <drivers/drv_pwm_output.h>
#include <px4_arch/io_timer.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/**
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
	// Set the peripheral rails off.
	stm32_configgpio(GPIO_PERIPH_3V3_EN);

	stm32_gpiowrite(GPIO_PERIPH_3V3_EN, 0);

	bool last = stm32_gpioread(GPIO_SPEKTRUM_PWR_EN);
	// Keep Spektum on to discharge rail.
	stm32_gpiowrite(GPIO_SPEKTRUM_PWR_EN, 1);

	// Wait for the peripheral rail to reach GND.
	usleep(ms * 1000);
	syslog(LOG_DEBUG, "reset done, %d ms\n", ms);

	// Re-enable power.
	// Switch the peripheral rail back on.
	stm32_gpiowrite(GPIO_SPEKTRUM_PWR_EN, last);
	stm32_gpiowrite(GPIO_PERIPH_3V3_EN, 1);

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
	// Configure the GPIO pins to outputs and keep them low.
	for (int i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; ++i) {
		px4_arch_configgpio(io_timer_channel_get_gpio_output(i));
	}

	/**
	 * On resets invoked from system (not boot) insure we establish a low
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
	// Reset all PWM to Low outputs.
	board_on_reset(-1);

	// Configure LEDs.
	board_autoled_initialize();


	// Configure ADC pins.
	stm32_configgpio(GPIO_ADC1_IN2);	/* BATT_VOLTAGE_SENS */
	stm32_configgpio(GPIO_ADC1_IN3);	/* BATT_CURRENT_SENS */
	stm32_configgpio(GPIO_ADC1_IN4);	/* VDD_5V_SENS */
	stm32_configgpio(GPIO_ADC1_IN11);	/* RSSI analog in */

	// Configure CAN interface
	stm32_configgpio(GPIO_CAN1_RX);
	stm32_configgpio(GPIO_CAN1_TX);

	// Configure power supply control/sense pins.
	stm32_configgpio(GPIO_PERIPH_3V3_EN);
	stm32_configgpio(GPIO_VDD_BRICK_VALID);
	stm32_configgpio(GPIO_VDD_USB_VALID);

	stm32_configgpio(GPIO_SBUS_INV);
	stm32_configgpio(GPIO_SPEKTRUM_PWR_EN);

	stm32_configgpio(GPIO_PB4);
	stm32_configgpio(GPIO_PE2);

	// Safety - led on in led driver.
	stm32_configgpio(GPIO_BTN_SAFETY);
	stm32_configgpio(GPIO_PPM_IN);

#if defined(CONFIG_STM32_SPI4)

	/* We have SPI4 is GPIO_PB4 pin 3 Low */
	if (stm32_gpioread(GPIO_PB4) != 0) {
#endif /* CONFIG_STM32_SPI4 */

		stm32_configgpio(GPIO_PE5);
		stm32_configgpio(GPIO_PE6);

#if defined(CONFIG_STM32_SPI4)
	}

#endif /* CONFIG_STM32_SPI4 */

	// Configure SPI all interfaces GPIO & enable power.
	stm32_spiinitialize();

	// Configure heater GPIO.
	stm32_configgpio(GPIO_HEATER_INPUT);
	stm32_configgpio(GPIO_HEATER_OUTPUT);
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
static struct sdio_dev_s *sdio;
#if defined(CONFIG_STM32_SPI4)
static struct spi_dev_s *spi4;
#endif

__EXPORT int board_app_initialize(uintptr_t arg)
{
	px4_platform_init();

	// Configure the DMA allocator.
	if (board_dma_alloc_init() < 0) {
		syslog(LOG_ERR, "DMA alloc FAILED\n");
	}

	// Set up the serial DMA polling.
	static struct hrt_call serial_dma_call;
	struct timespec ts;

	/**
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

	// Initial LED state.
	drv_led_start();
	led_off(LED_RED);
	led_off(LED_GREEN);
	led_off(LED_BLUE);

	if (board_hardfault_init(2, true) != 0) {
		led_on(LED_RED);
	}

	// Power down the heater.
	stm32_gpiowrite(GPIO_HEATER_OUTPUT, 0);

	// Configure SPI-based devices.
	spi1 = stm32_spibus_initialize(1);

	if (!spi1) {
		syslog(LOG_ERR, "[boot] FAILED to initialize SPI port 1\n");
		led_on(LED_RED);
		return -ENODEV;
	}


	// Default SPI1 to 1MHz and de-assert the known chip selects.
	SPI_SETFREQUENCY(spi1, 10000000);
	SPI_SETBITS(spi1, 8);
	SPI_SETMODE(spi1, SPIDEV_MODE3);
	up_udelay(20);

	// Get the SPI port for the FRAM.
	spi2 = stm32_spibus_initialize(2);

	if (!spi2) {
		syslog(LOG_ERR, "[boot] FAILED to initialize SPI port 2\n");
		led_on(LED_RED);
		return -ENODEV;
	}

	/**
	 * Default SPI2 to 12MHz and de-assert the known chip selects.
	 * MS5611 has max SPI clock speed of 20MHz.
	 */

	// XXX start with 10.4 MHz and go up to 20 once validated.
	SPI_SETFREQUENCY(spi2, 20 * 1000 * 1000);
	SPI_SETBITS(spi2, 8);
	SPI_SETMODE(spi2, SPIDEV_MODE3);

#if defined(CONFIG_STM32_SPI4)

	if (stm32_gpioread(GPIO_PB4) == 0) {
		syslog(LOG_INFO, "[boot] GPIO_PB4 - Low Initialize SPI port 4 \n");

		// Configure SPI-based devices.
		spi4 = stm32_spibus_initialize(4);

		if (!spi4) {
			syslog(LOG_ERR, "[boot] FAILED to initialize SPI port 4\n");

		} else {
			// Default SPI4 to 20 MHz and de-assert the known chip selects.
			SPI_SETFREQUENCY(spi4, 20 * 1000 * 1000);
			SPI_SETBITS(spi4, 8);
			SPI_SETMODE(spi4, SPIDEV_MODE3);
		}
	}

#endif /* defined(CONFIG_STM32_SPI4) */


#ifdef CONFIG_MMCSD

	// First, get an instance of the SDIO interface.
	sdio = sdio_initialize(CONFIG_NSH_MMCSDSLOTNO);

	if (!sdio) {
		led_on(LED_RED);
		syslog(LOG_ERR, "[boot] Failed to initialize SDIO slot %d\n",
		       CONFIG_NSH_MMCSDSLOTNO);
		return -ENODEV;
	}

	// Now bind the SDIO interface to the MMC/SD driver.
	int ret = mmcsd_slotinitialize(CONFIG_NSH_MMCSDMINOR, sdio);

	if (ret != OK) {
		led_on(LED_RED);
		syslog(LOG_ERR, "[boot] Failed to bind SDIO to the MMC/SD driver: %d\n", ret);
		return ret;
	}

	// Then let's guess and say that there is a card in the slot. There is no card detect GPIO.
	sdio_mediachange(sdio, true);

#endif

	/* Configure the HW based on the manifest */

	px4_platform_configure();

	return OK;
}
