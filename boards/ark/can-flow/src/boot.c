/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *       Author: Ben Dyer <ben_dyer@mac.com>
 *               Pavel Kirienko <pavel.kirienko@zubax.com>
 *               David Sidrane <david_s5@nscdg.com>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <px4_config.h>
#include <stdint.h>
#include "boot_config.h"
#include "board.h"

#include <debug.h>
#include <string.h>
#include <arch/board/board.h>

#include <nuttx/board.h>

__BEGIN_DECLS
extern void led_init(void);
extern void bootloader_led_on(int led);
extern void bootloader_led_off(int led);
__END_DECLS

/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the initialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

__EXPORT void stm32_boardinitialize(void)
{
	putreg32(getreg32(STM32_RCC_APB1ENR) | RCC_APB1ENR_CAN1EN, STM32_RCC_APB1ENR);
	stm32_configgpio(GPIO_CAN1_RX);
	stm32_configgpio(GPIO_CAN1_TX);
	stm32_configgpio(GPIO_CAN1_SILENT_S0);
	stm32_configgpio(GPIO_CAN1_TERMINATION);
	putreg32(getreg32(STM32_RCC_APB1RSTR) | RCC_APB1RSTR_CAN1RST, STM32_RCC_APB1RSTR);
	putreg32(getreg32(STM32_RCC_APB1RSTR) & ~RCC_APB1RSTR_CAN1RST, STM32_RCC_APB1RSTR);

	led_init();

#if defined(OPT_WAIT_FOR_GETNODEINFO_JUMPER_GPIO)
	stm32_configgpio(GPIO_GETNODEINFO_JUMPER);
#endif

}

/************************************************************************************
 * Name: board_deinitialize
 *
 * Description:
 *   This function is called by the bootloader code prior to booting
 *   the application. Is should place the HW into an benign initialized state.
 *
 ************************************************************************************/

void board_deinitialize(void)
{
	putreg32(getreg32(STM32_RCC_APB1RSTR) | RCC_APB1RSTR_CAN1RST, STM32_RCC_APB1RSTR);
}

/****************************************************************************
 * Name: board_get_product_name
 *
 * Description:
 *   Called to retrieve the product name. The returned value is a assumed
 *   to be written to a pascal style string that will be length prefixed
 *   and not null terminated
 *
 * Input Parameters:
 *    product_name - A pointer to a buffer to write the name.
 *    maxlen       - The maximum number of charter that can be written
 *
 * Returned Value:
 *   The length of characters written to the buffer.
 *
 ****************************************************************************/

uint8_t board_get_product_name(uint8_t *product_name, size_t maxlen)
{
	DEBUGASSERT(maxlen > UAVCAN_STRLEN(HW_UAVCAN_NAME));
	memcpy(product_name, HW_UAVCAN_NAME, UAVCAN_STRLEN(HW_UAVCAN_NAME));
	return UAVCAN_STRLEN(HW_UAVCAN_NAME);
}

/****************************************************************************
 * Name: board_get_hardware_version
 *
 * Description:
 *   Called to retrieve the hardware version information. The function
 *   will first initialize the the callers struct to all zeros.
 *
 * Input Parameters:
 *    hw_version - A pointer to a uavcan_hardwareversion_t.
 *
 * Returned Value:
 *   Length of the unique_id
 *
 ****************************************************************************/

size_t board_get_hardware_version(uavcan_HardwareVersion_t *hw_version)
{
	memset(hw_version, 0, sizeof(uavcan_HardwareVersion_t));

	hw_version->major = HW_VERSION_MAJOR;
	hw_version->minor = HW_VERSION_MINOR;

	return board_get_mfguid(*(mfguid_t *) hw_version->unique_id);
}

/****************************************************************************
 * Name: board_indicate
 *
 * Description:
 *   Provides User feedback to indicate the state of the bootloader
 *   on board specific  hardware.
 *
 * Input Parameters:
 *    indication - A member of the uiindication_t
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
void board_indicate(uiindication_t indication)
{
	if (indication == off) {
		bootloader_led_off(GPIO_nLED_RED);
		bootloader_led_off(GPIO_nLED_BLUE);

	} else if (indication == fw_update_start) {
		bootloader_led_on(GPIO_nLED_RED);
		bootloader_led_on(GPIO_nLED_BLUE);

	} else if ((indication == fw_update_erase_fail) || (indication == fw_update_invalid_response)
		   || (indication == fw_update_timeout) || (indication == fw_update_invalid_crc)) {
		bootloader_led_on(GPIO_nLED_RED);
		bootloader_led_off(GPIO_nLED_BLUE);

	} else if (indication == allocation_start) {
		bootloader_led_on(GPIO_nLED_RED);
		bootloader_led_off(GPIO_nLED_BLUE);

	} else {
		bootloader_led_off(GPIO_nLED_RED);
		bootloader_led_on(GPIO_nLED_BLUE);
	}
}
