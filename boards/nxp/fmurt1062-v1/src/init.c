/****************************************************************************
 *
 *   Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
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
 * NXP imxrt1062-v1 specific early startup code.  This file implements the
 * board_app_initialize() function that is called early by nsh during startup.
 *
 * Code here is run before the rcS script is invoked; it should start required
 * subsystems and perform board-specific initialization.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "board_config.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/config.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>
#include <nuttx/analog/adc.h>
#include <nuttx/mm/gran.h>

#include "arm_arch.h"
#include "arm_internal.h"
#include "imxrt_flexspi_nor_boot.h"
#include "imxrt_iomuxc.h"
#include <chip.h>
#include "board_config.h"

#include <hardware/imxrt_lpuart.h>

#include <arch/board/board.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_board_led.h>
#include <systemlib/px4_macros.h>
#include <px4_arch/io_timer.h>
#include <px4_platform_common/init.h>
#include <px4_platform/gpio.h>
#include <px4_platform/board_determine_hw_info.h>
#include <px4_platform/board_dma_alloc.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

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

/************************************************************************************
 * Name: board_peripheral_reset
 *
 * Description:
 *
 ************************************************************************************/
__EXPORT void board_peripheral_reset(int ms)
{
	/* set the peripheral rails off */

	VDD_5V_PERIPH_EN(false);
	VDD_5V_HIPOWER_EN(false);

	/* wait for the peripheral rail to reach GND */
	usleep(ms * 1000);
	syslog(LOG_DEBUG, "reset done, %d ms", ms);

	/* re-enable power */

	/* switch the peripheral rail back on */
	VDD_5V_HIPOWER_EN(true);
	VDD_5V_PERIPH_EN(true);

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
	for (int i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; ++i) {
		px4_arch_configgpio(PX4_MAKE_GPIO_INPUT(io_timer_channel_get_gpio_output(i)));
	}

	if (status >= 0) {
		up_mdelay(6);
	}
}


/****************************************************************************
 * Name: imxrt_ocram_initialize
 *
 * Description:
 *   Called off reset vector to reconfigure the flexRAM
 *   and finish the FLASH to RAM Copy.
 *
 ****************************************************************************/

__EXPORT void imxrt_ocram_initialize(void)
{
	const uint32_t *src;
	uint32_t *dest;
	uint32_t regval;

	/* Reallocate 128K of Flex RAM from ITCM to OCRAM
	 * Final Confiduration is
	 *    128 DTCM
	 *
	 *    128 FlexRAM OCRAM  (202C:0000-202D:ffff)
	 *    256 FlexRAM OCRAM  (2028:0000-202B:ffff)
	 *    512 System  OCRAM2 (2020:0000-2027:ffff)
	 * */

	putreg32(0xaa555555, IMXRT_IOMUXC_GPR_GPR17);
	regval = getreg32(IMXRT_IOMUXC_GPR_GPR16);
	putreg32(regval | GPR_GPR16_FLEXRAM_BANK_CFG_SELF, IMXRT_IOMUXC_GPR_GPR16);

	for (src = (uint32_t *)(LOCATE_IN_SRC(g_boot_data.start) + g_boot_data.size),
	     dest = (uint32_t *)(g_boot_data.start + g_boot_data.size);
	     dest < (uint32_t *) &_etext;) {
		*dest++ = *src++;
	}
}

/****************************************************************************
 * Name: imxrt_boardinitialize
 *
 * Description:
 *   All i.MX RT architectures must provide the following entry point.  This
 *   entry point is called early in the initialization -- after clocking and
 *   memory have been configured but before caches have been enabled and
 *   before any devices have been initialized.
 *
 ****************************************************************************/

__EXPORT void imxrt_boardinitialize(void)
{

	board_on_reset(-1); /* Reset PWM first thing */

	/* configure LEDs */

	board_autoled_initialize();

	/* configure pins */

	const uint32_t gpio[] = PX4_GPIO_INIT_LIST;
	px4_gpio_init(gpio, arraySize(gpio));

	/* configure SPI interfaces */

	imxrt_spidev_initialize();

	imxrt_usb_initialize();

	fmurt1062_timer_initialize();
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

__EXPORT int board_app_initialize(uintptr_t arg)
{

	/* Power on Interfaces */


	VDD_3V3_SD_CARD_EN(true);
	VDD_5V_PERIPH_EN(true);
	VDD_5V_HIPOWER_EN(true);
	VDD_3V3_SENSORS_EN(true);
	VDD_3V3_SPEKTRUM_POWER_EN(true);

	board_spi_reset(10, 0xffff);

	if (OK == board_determine_hw_info()) {
		syslog(LOG_INFO, "[boot] Rev 0x%1x : Ver 0x%1x %s\n", board_get_hw_revision(), board_get_hw_version(),
		       board_get_hw_type_name());

	} else {
		syslog(LOG_ERR, "[boot] Failed to read HW revision and version\n");
	}


	px4_platform_init();

	/* configure the DMA allocator */

	if (board_dma_alloc_init() < 0) {
		syslog(LOG_ERR, "[boot] DMA alloc FAILED\n");
	}

	/* set up the serial DMA polling */
#ifdef SERIAL_HAVE_DMA
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
		       (hrt_callout)imxrt_serial_dma_poll,
		       NULL);
#endif

	/* initial LED state */
	drv_led_start();

	led_off(LED_RED);
	led_off(LED_GREEN);
	led_off(LED_BLUE);

#if defined(CONFIG_IMXRT_USDHC)
	int ret = fmurt1062_usdhc_initialize();

	if (ret != OK) {
		led_on(LED_RED);
		return ret;
	}

#endif

	/* Configure SPI-based devices */

	ret = imxrt1062_spi_bus_initialize();

	if (ret != OK) {
		led_on(LED_RED);
		return ret;
	}

	/* Configure the HW based on the manifest */

	px4_platform_configure();

	return OK;
}

// USB Stubs
#include <nuttx/usb/usbdev.h>
void arm_usbinitialize(void)
{
}

int usbdev_register(struct usbdevclass_driver_s *driver)
{
	return -EINVAL;
}
int usbdev_unregister(struct usbdevclass_driver_s *driver)
{
	return -EINVAL;
}
