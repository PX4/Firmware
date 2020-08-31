/************************************************************************************
 * configs/px4_fmu-v4pro/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2009, 2016 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
 *            Kevin Lopez Alvarez <kevin@drotek.com>
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

#ifndef __ARCH_BOARD_BOARD_H
#define __ARCH_BOARD_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/
#include "board_dma_map.h"

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

#include <stm32.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* The PX4FMUV4-PRO uses a 24MHz crystal connected to the HSE.
 *
 * This is the "standard" configuration as set up by arch/arm/src/stm32f40xx_rcc.c:
 *   System Clock source           : PLL (HSE)
 *   SYSCLK(Hz)                    : 180000000    Determined by PLL configuration
 *   HCLK(Hz)                      : 180000000    (STM32_RCC_CFGR_HPRE)
 *   AHB Prescaler                 : 1            (STM32_RCC_CFGR_HPRE)
 *   APB1 Prescaler                : 4            (STM32_RCC_CFGR_PPRE1)
 *   APB2 Prescaler                : 2            (STM32_RCC_CFGR_PPRE2)
 *   HSE Frequency(Hz)             : 24000000     (STM32_BOARD_XTAL)
 *   PLLM                          : 12           (STM32_PLLCFG_PLLM)
 *   PLLN                          : 180          (STM32_PLLCFG_PLLN)
 *   PLLP                          : 2            (STM32_PLLCFG_PLLP)
 *   PLLQ                          : 2            (STM32_PLLCFG_PPQ)
 *   Main regulator output voltage : Scale1 mode  Needed for high speed SYSCLK
 *   Flash Latency(WS)             : 5
 *   Prefetch Buffer               : OFF
 *   Instruction cache             : ON
 *   Data cache                    : ON
 *   Require 48MHz for USB OTG FS, : Use PLLSA1M
 *   SDIO and RNG clock
 */

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * HSE - On-board crystal frequency is 24MHz
 * LSE - not installed
 */

#define STM32_BOARD_XTAL        24000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL

/* Main PLL Configuration.
 *
 * PLL source is HSE
 * PLL_VCO = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 *         = (24,000,000 / 12) * 180
 *         = 369,000,000
 * SYSCLK  = PLL_VCO / PLLP
 *         = 360,000,000 / 2 = 180,000,000
 * USB OTG FS, SDIO and RNG Clock is from PLL SAIP
 *         = 48,000,000
 */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(12)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(180)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(2)
#define STM32_PLLCFG_PLLR       RCC_PLLCFG_PLLR(2)

/* Configure factors for  PLLSAI clock */

#define STM32_RCC_PLLSAICFGR_PLLSAIN    RCC_PLLSAICFGR_PLLSAIN(96)
#define STM32_RCC_PLLSAICFGR_PLLSAIP    RCC_PLLSAICFGR_PLLSAIP(4)
#define STM32_RCC_PLLSAICFGR_PLLSAIQ    RCC_PLLSAICFGR_PLLSAIQ(2)
#define STM32_RCC_PLLSAICFGR_PLLSAIR    RCC_PLLSAICFGR_PLLSAIR(2)

/* Configure Dedicated Clock Configuration Register */

#define STM32_RCC_DCKCFGR_PLLI2SDIVQ    RCC_DCKCFGR_PLLI2SDIVQ(1)
#define STM32_RCC_DCKCFGR_PLLSAIDIVQ    RCC_DCKCFGR_PLLSAIDIVQ(1)
#define STM32_RCC_DCKCFGR_PLLSAIDIVR    RCC_DCKCFGR_PLLSAIDIVR_DIV2
#define STM32_RCC_DCKCFGR_SAI1ASRC      RCC_DCKCFGR_SAI1ASRC_PLLSAI
#define STM32_RCC_DCKCFGR_SAI1BSRC      RCC_DCKCFGR_SAI1BSRC_PLLI2S
#define STM32_RCC_DCKCFGR_TIMPRE        0
#define STM32_RCC_DCKCFGR_48MSEL        RCC_DCKCFGR_48MSEL_PLLSAI
#define STM32_RCC_DCKCFGR_SDMMCSEL      RCC_DCKCFGR_SDMMCSEL_48MHZ
#define STM32_RCC_DCKCFGR_DSISEL        RCC_DCKCFGR_DSISEL_DSIPHY

/* Configure factors for  PLLI2S clock */

#define STM32_RCC_PLLI2SCFGR_PLLI2SN   RCC_PLLI2SCFGR_PLLI2SN(96)
#define STM32_RCC_PLLI2SCFGR_PLLI2SQ   RCC_PLLI2SCFGR_PLLI2SQ(4)
#define STM32_RCC_PLLI2SCFGR_PLLI2SR   RCC_PLLI2SCFGR_PLLI2SR(2)


#define STM32_SYSCLK_FREQUENCY  180000000ul

/* AHB clock (HCLK) is SYSCLK (180MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK  /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY  /* same as above, to satisfy compiler */

/* APB1 clock (PCLK1) is HCLK/4 (45MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd4     /* PCLK1 = HCLK / 4 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/4)

/* Timers driven from APB1 will be twice PCLK1 */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN  (2*STM32_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK/2 (90MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLKd2     /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers driven from APB2 will be twice PCLK2 */

#define STM32_APB2_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM9_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM10_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM11_CLKIN  (2*STM32_PCLK2_FREQUENCY)

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,8-11 are on APB2, others on APB1
 */

#define BOARD_TIM1_FREQUENCY    STM32_APB2_TIM1_CLKIN
#define BOARD_TIM2_FREQUENCY    STM32_APB1_TIM2_CLKIN
#define BOARD_TIM3_FREQUENCY    STM32_APB1_TIM3_CLKIN
#define BOARD_TIM4_FREQUENCY    STM32_APB1_TIM4_CLKIN
#define BOARD_TIM5_FREQUENCY    STM32_APB1_TIM5_CLKIN
#define BOARD_TIM6_FREQUENCY    STM32_APB1_TIM6_CLKIN
#define BOARD_TIM7_FREQUENCY    STM32_APB1_TIM7_CLKIN
#define BOARD_TIM8_FREQUENCY    STM32_APB2_TIM8_CLKIN
#define BOARD_TIM9_FREQUENCY    STM32_APB2_TIM9_CLKIN
#define BOARD_TIM10_FREQUENCY   STM32_APB2_TIM10_CLKIN
#define BOARD_TIM11_FREQUENCY   STM32_APB2_TIM11_CLKIN
#define BOARD_TIM12_FREQUENCY   STM32_APB1_TIM12_CLKIN
#define BOARD_TIM13_FREQUENCY   STM32_APB1_TIM13_CLKIN
#define BOARD_TIM14_FREQUENCY   STM32_APB1_TIM14_CLKIN

/* SDMMC dividers.  Note that slower clocking is required when DMA is disabled
 * in order to avoid RX overrun/TX underrun errors due to delayed responses
 * to service FIFOs in interrupt driven mode.  These values have not been
 * tuned!!!
 *
 * SDIOCLK=48MHz, SDMMC_CK=SDIOCLK/(118+2)=400 KHz
 */

/* Use the Falling edge of the SDIO_CLK clock to change the edge the
 * data and commands are change on
 */

#define SDIO_CLKCR_EDGE SDIO_CLKCR_NEGEDGE

#define SDIO_INIT_CLKDIV        (118 << SDIO_CLKCR_CLKDIV_SHIFT)

/* DMA ON:  SDIOCLK=48MHz, SDMMC_CK=SDIOCLK/(1+2)=16 MHz
 * DMA OFF: SDIOCLK=48MHz, SDMMC_CK=SDIOCLK/(2+2)=12 MHz
 */

#ifdef CONFIG_STM32_SDIO_DMA
#  define SDIO_MMCXFR_CLKDIV    (1 << SDIO_CLKCR_CLKDIV_SHIFT)
#else
#  define SDIO_MMCXFR_CLKDIV    (2 << SDIO_CLKCR_CLKDIV_SHIFT)
#endif

/* DMA ON:  SDIOCLK=48MHz, SDMMC_CK=SDIOCLK/(1+2)=16 MHz
 * DMA OFF: SDIOCLK=48MHz, SDMMC_CK=SDIOCLK/(2+2)=12 MHz
 */
//TODO #warning "Check Freq for 24mHz"

#ifdef CONFIG_STM32_SDIO_DMA
#  define SDIO_SDXFR_CLKDIV     (1 << SDIO_CLKCR_CLKDIV_SHIFT)
#else
#  define SDIO_SDXFR_CLKDIV     (2 << SDIO_CLKCR_CLKDIV_SHIFT)
#endif

/* FLASH wait states
 *
 *  --------- ---------- ----------- ---------
 *  VDD       MAX SYSCLK WAIT STATES OVERDRIVE
 *  --------- ---------- ----------- ---------
 *  1.7-2.1 V   168 MHz    8          OFF
 *  2.1-2.4 V   180 MHz    8          ON
 *  2.4-2.7 V   180 MHz    7          ON
 *  2.7-3.6 V   180 MHz    5          ON
 *  --------- ---------- ----------- --------
 *-
 */

#define BOARD_FLASH_WAITSTATES 5

/* Alternate function pin selections ************************************************/

/*
 * UARTs.
 */
#define GPIO_USART1_RX	GPIO_USART1_RX_2	/* ESP8266 */
#define GPIO_USART1_TX	GPIO_USART1_TX_2

#define GPIO_USART2_RX	GPIO_USART2_RX_2
#define GPIO_USART2_TX	GPIO_USART2_TX_2
#define GPIO_USART2_RTS	GPIO_USART2_RTS_2
#define GPIO_USART2_CTS	GPIO_USART2_CTS_2

#define GPIO_USART3_RX	GPIO_USART3_RX_3
#define GPIO_USART3_TX	GPIO_USART3_TX_3
#define GPIO_USART3_RTS	GPIO_USART3_RTS_2
#define GPIO_USART3_CTS	GPIO_USART3_CTS_2

#define GPIO_UART4_RX	GPIO_UART4_RX_1
#define GPIO_UART4_TX	GPIO_UART4_TX_1

#define GPIO_USART6_RX	GPIO_USART6_RX_1	/* RC_INPUT */
#define GPIO_USART6_TX	GPIO_USART6_TX_1

#define GPIO_UART7_RX	GPIO_UART7_RX_1
#define GPIO_UART7_TX	GPIO_UART7_TX_1

/* UART8 has no alternate pin config */


/*
 * CAN
 *
 * CAN1 and CAN2 is routed to the onboard transceiver.
 */
#define GPIO_CAN1_RX	GPIO_CAN1_RX_3
#define GPIO_CAN1_TX	GPIO_CAN1_TX_3

#define GPIO_CAN2_RX	GPIO_CAN2_RX_1
#define GPIO_CAN2_TX	GPIO_CAN2_TX_1

/*
 * I2C
 *
 * The optional _GPIO configurations allow the I2C driver to manually
 * reset the bus to clear stuck slaves.  They match the pin configuration,
 * but are normally-high GPIOs.
 */
#define GPIO_I2C1_SCL		GPIO_I2C1_SCL_2
#define GPIO_I2C1_SDA		GPIO_I2C1_SDA_2
#define GPIO_I2C1_SCL_GPIO	(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN8)
#define GPIO_I2C1_SDA_GPIO	(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN9)

#define GPIO_I2C2_SCL		GPIO_I2C2_SCL_2
#define GPIO_I2C2_SDA		GPIO_I2C2_SDA_2
#define GPIO_I2C2_SCL_GPIO	(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN1)
#define GPIO_I2C2_SDA_GPIO	(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN0)


/*
 * SPI
 *
 * There are sensors on SPI1, and SPI2 is connected to the FRAM.
 */
#define GPIO_SPI1_MISO  GPIO_SPI1_MISO_1
#define GPIO_SPI1_MOSI  GPIO_SPI1_MOSI_1
#define GPIO_SPI1_SCK   GPIO_SPI1_SCK_1

#define GPIO_SPI2_MISO  GPIO_SPI2_MISO_1
#define GPIO_SPI2_MOSI  GPIO_SPI2_MOSI_1
#define GPIO_SPI2_SCK   GPIO_SPI2_SCK_1

#define GPIO_SPI5_MISO  GPIO_SPI5_MISO_1
#define GPIO_SPI5_MOSI  GPIO_SPI5_MOSI_1
#define GPIO_SPI5_SCK   GPIO_SPI5_SCK_1
#define GPIO_SPI5_NSS   GPIO_SPI5_NSS_1

/* Board provides GPIO or other Hardware for signaling to timing analyzer */

#if defined(CONFIG_BOARD_USE_PROBES)
# define PROBE_N(n) (1<<((n)-1))
# define PROBE_1  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN14)
# define PROBE_2  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN13)
# define PROBE_3  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN11)
# define PROBE_4  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN9)
# define PROBE_5  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN13)
# define PROBE_6  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN14)

# define PROBE_INIT(mask) \
	do { \
		if ((mask)& PROBE_N(1)) { stm32_configgpio(PROBE_1); } \
		if ((mask)& PROBE_N(2)) { stm32_configgpio(PROBE_2); } \
		if ((mask)& PROBE_N(3)) { stm32_configgpio(PROBE_3); } \
		if ((mask)& PROBE_N(4)) { stm32_configgpio(PROBE_4); } \
		if ((mask)& PROBE_N(5)) { stm32_configgpio(PROBE_5); } \
		if ((mask)& PROBE_N(6)) { stm32_configgpio(PROBE_6); } \
	} while(0)

# define PROBE(n,s)  do {stm32_gpiowrite(PROBE_##n,(s));}while(0)
# define PROBE_MARK(n) PROBE(n,false);PROBE(n,true)
#else
# define PROBE_INIT(mask)
# define PROBE(n,s)
# define PROBE_MARK(n)
#endif


#endif  /* __ARCH_BOARD_BOARD_H */
