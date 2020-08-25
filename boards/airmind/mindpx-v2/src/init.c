/****************************************************************************
 *
 *   Copyright (c) 2015, 2016 PX4 Development Team. All rights reserved.
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
 * @file mindpx2_init.c
 *
 * MINDPX-specific early startup code.  This file implements the
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

#include <px4_platform_common/init.h>
#include <px4_platform/board_dma_alloc.h>
#include <drivers/drv_pwm_output.h>
#include <px4_arch/io_timer.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/*
 * Ideally we'd be able to get these from up_internal.h,
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

	/**
	 * On resets invoked from system (not boot) insure we establish a low
	 * output state (discharge the pins) on PWM pins before they become inputs.
	 */

	if (status >= 0) {
		up_mdelay(400);
	}
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

__EXPORT void
stm32_boardinitialize(void)
{
	// Reset all PWM to Low outputs.

	board_on_reset(-1);

	/* configure LEDs */

	board_autoled_initialize();

	/* configure ADC pins */

	stm32_configgpio(GPIO_ADC1_IN4);	/* VDD_5V_SENS */
	stm32_configgpio(GPIO_ADC1_IN10);	/* BATT_CURRENT_SENS */
	stm32_configgpio(GPIO_ADC1_IN12);	/* BATT_VOLTAGE_SENS */
	stm32_configgpio(GPIO_ADC1_IN11);	/* RSSI analog in */
	stm32_configgpio(GPIO_ADC1_IN13);	/* FMU_AUX_ADC_1 */
	stm32_configgpio(GPIO_ADC1_IN14);	/* FMU_AUX_ADC_2 */
	stm32_configgpio(GPIO_ADC1_IN15);	/* PRESSURE_SENS */

	/* configure power supply control/sense pins */

	stm32_configgpio(GPIO_SBUS_INV);
	stm32_configgpio(GPIO_FRSKY_INV);

	/* configure CAN interface */

	stm32_configgpio(GPIO_CAN1_RX);
	stm32_configgpio(GPIO_CAN1_TX);

	/* configure SPI interfaces */

	stm32_spiinitialize();

	stm32_configgpio(GPIO_I2C2_SCL);
	stm32_configgpio(GPIO_I2C2_SDA);

	stm32_configgpio(GPIO_I2C1_SCL);
	stm32_configgpio(GPIO_I2C1_SDA);

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
static struct spi_dev_s *spi4;
static struct sdio_dev_s *sdio;

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
	led_off(LED_AMBER);

	if (board_hardfault_init(2, true) != 0) {
		led_on(LED_AMBER);
	}

	/* Configure SPI-based devices */

	spi4 = px4_spibus_initialize(4);

	if (!spi4) {
		syslog(LOG_ERR, "[boot] FAILED to initialize SPI port 4\n");
		board_autoled_on(LED_AMBER);
		return -ENODEV;
	}

	/* Default SPI4 to 10MHz and de-assert the known chip selects. */
	SPI_SETFREQUENCY(spi4, 10000000);
	SPI_SETBITS(spi4, 8);
	SPI_SETMODE(spi4, SPIDEV_MODE3);
	SPI_SELECT(spi4, PX4_SPIDEV_GYRO, false);
	SPI_SELECT(spi4, PX4_SPIDEV_ACCEL_MAG, false);
	SPI_SELECT(spi4, PX4_SPIDEV_BARO, false);
	SPI_SELECT(spi4, PX4_SPIDEV_MPU, false);
	up_udelay(20);

	/* Get the SPI port for the FRAM */

	spi1 = stm32_spibus_initialize(1);

	if (!spi1) {
		syslog(LOG_ERR, "[boot] FAILED to initialize SPI port 1\n");
		board_autoled_on(LED_AMBER);
		return -ENODEV;
	}

	/* Default SPI1 to 37.5 MHz (40 MHz rounded to nearest valid divider, F4 max)
	 * and de-assert the known chip selects. */

	// XXX start with 10.4 MHz in FRAM usage and go up to 37.5 once validated
	SPI_SETFREQUENCY(spi1, 24 * 1000 * 1000);
	SPI_SETBITS(spi1, 8);
	SPI_SETMODE(spi1, SPIDEV_MODE3);
	SPI_SELECT(spi1, SPIDEV_FLASH(0), false);


	spi2 = px4_spibus_initialize(2);

	/* Default SPI2 to 10MHz and de-assert the known chip selects. */
	SPI_SETFREQUENCY(spi2, 10000000);
	SPI_SETBITS(spi2, 8);
	SPI_SETMODE(spi2, SPIDEV_MODE3);
	SPI_SELECT(spi2, PX4_SPIDEV_EXT0, false);


#ifdef CONFIG_MMCSD
	/* First, get an instance of the SDIO interface */

	sdio = sdio_initialize(CONFIG_NSH_MMCSDSLOTNO);

	if (!sdio) {
		syslog(LOG_ERR, "[boot] Failed to initialize SDIO slot %d\n",
		       CONFIG_NSH_MMCSDSLOTNO);
		return -ENODEV;
	}

	/* Now bind the SDIO interface to the MMC/SD driver */
	int ret = mmcsd_slotinitialize(CONFIG_NSH_MMCSDMINOR, sdio);

	if (ret != OK) {
		syslog(LOG_ERR, "[boot] Failed to bind SDIO to the MMC/SD driver: %d\n", ret);
		return ret;
	}

	/* Then let's guess and say that there is a card in the slot. There is no card detect GPIO. */
	sdio_mediachange(sdio, true);

#endif

	return OK;
}
