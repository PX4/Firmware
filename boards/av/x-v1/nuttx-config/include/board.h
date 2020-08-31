/************************************************************************************
 * nuttx-configs/av-x-v1/include/board.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#pragma once

/************************************************************************************
 * Included Files
 ************************************************************************************/
#include "board_dma_map.h"

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

#include "stm32_rcc.h"
#include "stm32_sdmmc.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* The av-x-v1  board provides the following clock sources:
 *
 *   X2:  16 MHz oscillator for STM32F777NI microcontroller and Ethernet PHY.
 *   X1:  32.768 KHz crystal for STM32F777NI embedded RTC
 *
 * So we have these clock source available within the STM32
 *
 *   HSI: 16 MHz RC factory-trimmed
 *   LSI: 32 KHz RC
 *   HSE: 16 MHz crystal for HSE
 */

#define STM32_BOARD_XTAL        16000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     0

/* Main PLL Configuration.
 *
 * PLL source is HSE = 25,000,000
 *
 * PLL_VCO = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 * Subject to:
 *
 *     2 <= PLLM <= 63
 *   192 <= PLLN <= 432
 *   192 MHz <= PLL_VCO <= 432MHz
 *
 * SYSCLK  = PLL_VCO / PLLP
 * Subject to
 *
 *   PLLP = {2, 4, 6, 8}
 *   SYSCLK <= 216 MHz
 *
 * SDMMC and RNG Clock = PLL_VCO / PLLQ
 * Subject to
 *   The SDMMC and the random number generator need a frequency lower than or equal
 *   to 48 MHz to work correctly.
 *
 * 2 <= PLLQ <= 15
 */

/* SDMMCCLK (= USB OTG FS clock = RNGCLK) should be <= 48MHz
 *
 * PLL_VCO = (16,000,000 / 8) * 216 = 432 MHz
 * SYSCLK  = 432 MHz / 2 = 216 MHz
 * SDMMC and RNG Clock = 432 MHz / 9 = 48 MHz
 */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(8)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(216)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(9)

#define STM32_VCO_FREQUENCY     ((STM32_HSE_FREQUENCY / 8) * 216)
#define STM32_SYSCLK_FREQUENCY  (STM32_VCO_FREQUENCY / 2)
#define STM32_OTGFS_FREQUENCY   (STM32_VCO_FREQUENCY / 9)

/* Configure factors for  PLLSAI clock */

#define CONFIG_STM32F7_PLLSAI 1
#define STM32_RCC_PLLSAICFGR_PLLSAIN    RCC_PLLSAICFGR_PLLSAIN(192)
#define STM32_RCC_PLLSAICFGR_PLLSAIP    RCC_PLLSAICFGR_PLLSAIP(8)
#define STM32_RCC_PLLSAICFGR_PLLSAIQ    RCC_PLLSAICFGR_PLLSAIQ(4)
#define STM32_RCC_PLLSAICFGR_PLLSAIR    RCC_PLLSAICFGR_PLLSAIR(2)

/* Configure Dedicated Clock Configuration Register */

#define STM32_RCC_DCKCFGR1_PLLI2SDIVQ  RCC_DCKCFGR1_PLLI2SDIVQ(1)
#define STM32_RCC_DCKCFGR1_PLLSAIDIVQ  RCC_DCKCFGR1_PLLSAIDIVQ(1)
#define STM32_RCC_DCKCFGR1_PLLSAIDIVR  RCC_DCKCFGR1_PLLSAIDIVR(0)
#define STM32_RCC_DCKCFGR1_SAI1SRC     RCC_DCKCFGR1_SAI1SEL(0)
#define STM32_RCC_DCKCFGR1_SAI2SRC     RCC_DCKCFGR1_SAI2SEL(0)
#define STM32_RCC_DCKCFGR1_TIMPRESRC   0
#define STM32_RCC_DCKCFGR1_DFSDM1SRC   0
#define STM32_RCC_DCKCFGR1_ADFSDM1SRC  0



/* Configure factors for  PLLI2S clock */

#define CONFIG_STM32F7_PLLI2S 1
#define STM32_RCC_PLLI2SCFGR_PLLI2SN   RCC_PLLI2SCFGR_PLLI2SN(192)
#define STM32_RCC_PLLI2SCFGR_PLLI2SP   RCC_PLLI2SCFGR_PLLI2SP(2)
#define STM32_RCC_PLLI2SCFGR_PLLI2SQ   RCC_PLLI2SCFGR_PLLI2SQ(2)
#define STM32_RCC_PLLI2SCFGR_PLLI2SR   RCC_PLLI2SCFGR_PLLI2SR(2)

/* Configure Dedicated Clock Configuration Register 2 */

#define STM32_RCC_DCKCFGR2_USART1SRC  RCC_DCKCFGR2_USART1SEL_APB
#define STM32_RCC_DCKCFGR2_USART2SRC  RCC_DCKCFGR2_USART2SEL_APB
#define STM32_RCC_DCKCFGR2_UART4SRC   RCC_DCKCFGR2_UART4SEL_APB
#define STM32_RCC_DCKCFGR2_UART5SRC   RCC_DCKCFGR2_UART5SEL_APB
#define STM32_RCC_DCKCFGR2_USART6SRC  RCC_DCKCFGR2_USART6SEL_APB
#define STM32_RCC_DCKCFGR2_UART7SRC   RCC_DCKCFGR2_UART7SEL_APB
#define STM32_RCC_DCKCFGR2_UART8SRC   RCC_DCKCFGR2_UART8SEL_APB
#define STM32_RCC_DCKCFGR2_I2C1SRC    RCC_DCKCFGR2_I2C1SEL_HSI
#define STM32_RCC_DCKCFGR2_I2C2SRC    RCC_DCKCFGR2_I2C2SEL_HSI
#define STM32_RCC_DCKCFGR2_I2C3SRC    RCC_DCKCFGR2_I2C3SEL_HSI
#define STM32_RCC_DCKCFGR2_I2C4SRC    RCC_DCKCFGR2_I2C4SEL_HSI
#define STM32_RCC_DCKCFGR2_LPTIM1SRC  RCC_DCKCFGR2_LPTIM1SEL_APB
#define STM32_RCC_DCKCFGR2_CECSRC     RCC_DCKCFGR2_CECSEL_HSI
#define STM32_RCC_DCKCFGR2_CK48MSRC   RCC_DCKCFGR2_CK48MSEL_PLL
#define STM32_RCC_DCKCFGR2_SDMMCSRC   RCC_DCKCFGR2_SDMMCSEL_48MHZ
#define STM32_RCC_DCKCFGR2_SDMMC2SRC  RCC_DCKCFGR2_SDMMC2SEL_48MHZ
#define STM32_RCC_DCKCFGR2_DSISRC     RCC_DCKCFGR2_DSISEL_PHY


/* Several prescalers allow the configuration of the two AHB buses, the
 * high-speed APB (APB2) and the low-speed APB (APB1) domains. The maximum
 * frequency of the two AHB buses is 216 MHz while the maximum frequency of
 * the high-speed APB domains is 108 MHz. The maximum allowed frequency of
 * the low-speed APB domain is 54 MHz.
 */

/* AHB clock (HCLK) is SYSCLK (216 MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK  /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY  /* same as above, to satisfy compiler */

/* APB1 clock (PCLK1) is HCLK/4 (54 MHz) */

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

/* APB2 clock (PCLK2) is HCLK/2 (108MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLKd2     /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers driven from APB2 will be twice PCLK2 */

#define STM32_APB2_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM9_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM10_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM11_CLKIN  (2*STM32_PCLK2_FREQUENCY)

/* SDMMC dividers.  Note that slower clocking is required when DMA is disabled
 * in order to avoid RX overrun/TX underrun errors due to delayed responses
 * to service FIFOs in interrupt driven mode.  These values have not been
 * tuned!!!
 *
 * SDMMCCLK=48MHz, SDMMC_CK=SDMMCCLK/(118+2)=400 KHz
 */

/* Use the Falling edge of the SDIO_CLK clock to change the edge the
 * data and commands are change on
 */

#define STM32_SDMMC_CLKCR_EDGE STM32_SDMMC_CLKCR_NEGEDGE

#define STM32_SDMMC_INIT_CLKDIV         (118 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)

/* DMA ON:  SDMMCCLK=48MHz, SDMMC_CK=SDMMCCLK/(1+2)=16 MHz
 * DMA OFF: SDMMCCLK=48MHz, SDMMC_CK=SDMMCCLK/(2+2)=12 MHz
 */

#ifdef CONFIG_STM32F7_SDMMC_DMA
#  define STM32_SDMMC_MMCXFR_CLKDIV     (1 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#else
#  define STM32_SDMMC_MMCXFR_CLKDIV     (2 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#endif

/* DMA ON:  SDMMCCLK=48MHz, SDMMC_CK=SDMMCCLK/(1+2)=16 MHz
 * DMA OFF: SDMMCCLK=48MHz, SDMMC_CK=SDMMCCLK/(2+2)=12 MHz
 */
//TODO #warning "Check Freq for 24mHz"

#ifdef CONFIG_STM32F7_SDMMC_DMA
#  define STM32_SDMMC_SDXFR_CLKDIV      (1 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#else
#  define STM32_SDMMC_SDXFR_CLKDIV      (2 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#endif

/* FLASH wait states
 *
 *  --------- ---------- -----------
 *  VDD       MAX SYSCLK WAIT STATES
 *  --------- ---------- -----------
 *  1.7-2.1 V   180 MHz    8
 *  2.1-2.4 V   216 MHz    9
 *  2.4-2.7 V   216 MHz    8
 *  2.7-3.6 V   216 MHz    7
 *  --------- ---------- -----------
 */

#define BOARD_FLASH_WAITSTATES 7

/* Alternate function pin selections ************************************************/

#define GPIO_USART1_RX   GPIO_USART1_RX_2   /* PB7 */
#define GPIO_USART1_TX   GPIO_USART1_TX_2   /* PB6 */

#define GPIO_USART2_RX   GPIO_USART2_RX_2   /* PD6 */
#define GPIO_USART2_TX   GPIO_USART2_TX_2   /* PD5 */

#define GPIO_USART3_RX   GPIO_USART3_RX_3   /* PD9 */
#define GPIO_USART3_TX   GPIO_USART3_TX_3   /* PD8 */

#define GPIO_UART4_RX    GPIO_UART4_RX_4    /* PD0 */
#define GPIO_UART4_TX    GPIO_UART4_TX_3    /* PA12 */
#define GPIO_UART4_RS485_DIR GPIO_UART4_RTS_2	/* PB14 */

#define GPIO_UART5_RX    GPIO_UART5_RX_4    /* PB8 */
#define GPIO_UART5_TX    GPIO_UART5_TX_4    /* PB9 */

#define GPIO_USART6_RX   GPIO_USART6_RX_2   /* PG9 */
#define GPIO_USART6_TX   GPIO_USART6_TX_2   /* PG14 */

#define GPIO_UART7_RX    GPIO_UART7_RX_1    /* PE8 */
#define GPIO_UART7_TX    GPIO_UART7_TX_1    /* PE7 */

/* USART8: has no remap
 *
 *      GPIO_UART8_RX                          PE0
 *      GPIO_UART8_TX                          PE1
 */

/* CAN
 *
 * CAN1 is routed to transceiver.
 */
#define GPIO_CAN1_RX     GPIO_CAN1_RX_5     /* PI9  */
#define GPIO_CAN1_TX     GPIO_CAN1_TX_3     /* PD1 */

/* SPI
 * SPI1-SPI4 sensors
 */

#define GPIO_SPI1_MISO   GPIO_SPI1_MISO_1   /* PA6 */
#define GPIO_SPI1_MOSI   GPIO_SPI1_MOSI_2   /* PB5 */
#define GPIO_SPI1_SCK    GPIO_SPI1_SCK_1    /* PA5 */

#define GPIO_SPI2_MISO   GPIO_SPI2_MISO_3   /* PI2 */
#define GPIO_SPI2_MOSI   GPIO_SPI2_MOSI_3   /* PI3 */
#define GPIO_SPI2_SCK    GPIO_SPI2_SCK_4    /* PD3 */

#define GPIO_SPI4_MISO   GPIO_SPI4_MISO_1   /* PE5 */
#define GPIO_SPI4_MOSI   GPIO_SPI4_MOSI_1   /* PE6 */
#define GPIO_SPI4_SCK    GPIO_SPI4_SCK_1    /* PE2 */

#define GPIO_SPI5_MISO   GPIO_SPI5_MISO_1   /* PF8 */
#define GPIO_SPI5_MOSI   GPIO_SPI5_MOSI_2   /* PF11 */
#define GPIO_SPI5_SCK    GPIO_SPI5_SCK_2    /* PH6 */

/* The STM32 F7 connects to a SMSC LAN8720A PHY using these pins:
 *
 *   STM32 F7 BOARD        LAN8720A
 *   GPIO     SIGNAL       PIN NAME
 *   -------- ------------ -------------
 *   PB11     RMII_TX_EN   TXEN
 *   PB12     RMII_TXD0    TXD0
 *   PB13     RMII_TXD1    TXD1
 *   PC4      RMII_RXD0    RXD0/MODE0
 *   PC5      RMII_RXD1    RXD1/MODE1
 *   PD5      RMII_RXER    RXER/PHYAD0
 *   PA7      RMII_CRS_DV  CRS_DV/MODE2
 *   PC1      RMII_MDC     MDC
 *   PA2      RMII_MDIO    MDIO
 *   N/A      NRST         nRST
 *   PA1      RMII_REF_CLK nINT/REFCLK0
 *   N/A      OSC_25M      XTAL1/CLKIN
 *
 * The PHY address is 0, since RMII_RXER/PHYAD0 features a pull down.
 * After reset, RMII_RXER/PHYAD0 switches to the RXER function,
 * receive errors can be detected using GPIO pin PD5
 */

#define GPIO_ETH_RMII_TX_EN   GPIO_ETH_RMII_TX_EN_1
#define GPIO_ETH_RMII_TXD0    GPIO_ETH_RMII_TXD0_1
#define GPIO_ETH_RMII_TXD1    GPIO_ETH_RMII_TXD1_1

/* I2C Mapping
 *
 */

#define GPIO_I2C2_SCL GPIO_I2C2_SCL_2       /* PF1 */
#define GPIO_I2C2_SDA GPIO_I2C2_SDA_2       /* PF0 */

#define GPIO_I2C2_SCL_GPIO                  (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET |GPIO_PORTF | GPIO_PIN1)
#define GPIO_I2C2_SDA_GPIO                  (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET |GPIO_PORTF | GPIO_PIN0)

#define GPIO_I2C3_SCL GPIO_I2C3_SCL_2       /* PH7 */
#define GPIO_I2C3_SDA GPIO_I2C3_SDA_2       /* PH8 */

#define GPIO_I2C3_SCL_GPIO                  (GPIO_OUTPUT |GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTH | GPIO_PIN7)
#define GPIO_I2C3_SDA_GPIO                  (GPIO_OUTPUT |GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTH | GPIO_PIN8)

#define GPIO_I2C4_SCL GPIO_I2C4_SCL_2       /* PF14 */
#define GPIO_I2C4_SDA GPIO_I2C4_SDA_2       /* PF15 */

#define GPIO_I2C4_SCL_GPIO                  (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTF | GPIO_PIN14)
#define GPIO_I2C4_SDA_GPIO                  (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTF | GPIO_PIN15)

/* SDMMC1
 *
 *      VDD 3.3
 *      GND
 *      SDMMC1_CK                           PC12
 *      SDMMC1_CMD                          PD2
 *      SDMMC1_D0                           PC8
 *      SDMMC1_D1                           PC9
 *      SDMMC1_D2                           PC10
 *      SDMMC1_D3                           PC11
 */

