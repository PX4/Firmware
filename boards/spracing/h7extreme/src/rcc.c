/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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
 * @file rcc.c
 *
 * Board-specific clock config functions.
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/
#include "board_config.h"

#include "stm32_pwr.h"
#include "hardware/stm32_axi.h"
#include "hardware/stm32_syscfg.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/
/* Allow up to 100 milliseconds for the high speed clock to become ready.
 * that is a very long delay, but if the clock does not become ready we are
 * hosed anyway.  Normally this is very fast, but I have seen at least one
 * board that required this long, long timeout for the HSE to be ready.
 */

#define HSERDY_TIMEOUT (100 * CONFIG_BOARD_LOOPSPERMSEC)

/* Same for HSI */

#define HSIRDY_TIMEOUT HSERDY_TIMEOUT

/* HSE divisor to yield ~1MHz RTC clock */

#define HSE_DIVISOR (STM32_HSE_FREQUENCY + 500000) / 1000000

/* Voltage output scale (default to Scale 1 mode) */

#ifndef STM32_PWR_VOS_SCALE
#	define STM32_PWR_VOS_SCALE PWR_D3CR_VOS_SCALE_1
#endif

#if !defined(BOARD_FLASH_PROGDELAY)
#	if STM32_PWR_VOS_SCALE == PWR_D3CR_VOS_SCALE_1
#		if STM32_SYSCLK_FREQUENCY <= 70000000 && BOARD_FLASH_WAITSTATES == 0
#			define BOARD_FLASH_PROGDELAY	0
#		elif STM32_SYSCLK_FREQUENCY <= 140000000 && BOARD_FLASH_WAITSTATES == 1
#			define BOARD_FLASH_PROGDELAY	10
#		elif STM32_SYSCLK_FREQUENCY <= 185000000 && BOARD_FLASH_WAITSTATES == 2
#			define BOARD_FLASH_PROGDELAY	1
#		elif STM32_SYSCLK_FREQUENCY <= 210000000 && BOARD_FLASH_WAITSTATES == 2
#			define BOARD_FLASH_PROGDELAY	2
#		elif STM32_SYSCLK_FREQUENCY <= 225000000 && BOARD_FLASH_WAITSTATES == 3
#			define BOARD_FLASH_PROGDELAY	2
#		else
#			define BOARD_FLASH_PROGDELAY	2
#		endif
#	endif
#endif

/* PLL are only enabled if the P,Q or R outputs are enabled. */

#undef USE_PLL1
#if STM32_PLLCFG_PLL1CFG & (RCC_PLLCFGR_DIVP1EN | RCC_PLLCFGR_DIVQ1EN | \
														RCC_PLLCFGR_DIVR1EN)
#	define USE_PLL1
#endif

#undef USE_PLL2
#if STM32_PLLCFG_PLL2CFG & (RCC_PLLCFGR_DIVP2EN | RCC_PLLCFGR_DIVQ2EN | \
														RCC_PLLCFGR_DIVR2EN)
#	define USE_PLL2
#endif

#undef USE_PLL3
#if STM32_PLLCFG_PLL3CFG & (RCC_PLLCFGR_DIVP3EN | RCC_PLLCFGR_DIVQ3EN | \
														RCC_PLLCFGR_DIVR3EN)
#	define USE_PLL3
#endif


/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#if defined(CONFIG_STM32H7_CUSTOM_CLOCKCONFIG)

extern uint32_t _start_itcm;
extern uint32_t _end_itcm;
extern uint32_t _load_itcm;

extern uint32_t _start_sram;
extern uint32_t _end_sram;
extern uint32_t _load_sram;

__ramfunc__ void stm32_board_clockconfig(void)
{
	volatile uint32_t regval = 0;
	volatile int32_t timeout;

	/* This is not the	best place for copying code to ITCM and RAM.
	* Since our code work to slow in External Flash, we need to copy it to ITCM and RAM.
	* This currently way to do it inside board specific code.
	*/

	const uint32_t *src;
	uint32_t *dest;

	for (src = &_load_itcm, dest = &_start_itcm; dest < &_end_itcm;) {
		*dest++ = *src++;
	}

	for (src = &_load_sram, dest = &_start_sram; dest < &_end_sram;) {
		*dest++ = *src++;
	}

#ifdef STM32_BOARD_USEHSI
	/* Enable Internal High-Speed Clock (HSI) */

	regval	= getreg32(STM32_RCC_CR);
	regval |= RCC_CR_HSION;					 /* Enable HSI */
	putreg32(regval, STM32_RCC_CR);

	/* Wait until the HSI is ready (or until a timeout elapsed) */

	for (timeout = HSIRDY_TIMEOUT; timeout > 0; timeout--) {
		/* Check if the HSIRDY flag is the set in the CR */

		if ((getreg32(STM32_RCC_CR) & RCC_CR_HSIRDY) != 0) {
			/* If so, then break-out with timeout > 0 */

			break;
		}
	}

#else /* if STM32_BOARD_USEHSE */
	/* Enable External High-Speed Clock (HSE) */

	regval	= getreg32(STM32_RCC_CR);
#ifdef STM32_HSEBYP_ENABLE					/* May be defined in board.h header file */
	regval |= RCC_CR_HSEBYP;					/* Enable HSE clock bypass */
#else
	regval &= ~RCC_CR_HSEBYP;				 /* Disable HSE clock bypass */
#endif
	regval |= RCC_CR_HSEON;					 /* Enable HSE */
	putreg32(regval, STM32_RCC_CR);

	/* Wait until the HSE is ready (or until a timeout elapsed) */

	for (timeout = HSERDY_TIMEOUT; timeout > 0; timeout--) {
		/* Check if the HSERDY flag is the set in the CR */

		if ((getreg32(STM32_RCC_CR) & RCC_CR_HSERDY) != 0) {
			/* If so, then break-out with timeout > 0 */

			break;
		}
	}

#endif

#define CONFIG_STM32H7_HSI48
#ifdef CONFIG_STM32H7_HSI48
	/* Enable HSI48 */

	regval	= getreg32(STM32_RCC_CR);
	regval |= RCC_CR_HSI48ON;
	putreg32(regval, STM32_RCC_CR);

	/* Wait until the HSI48 is ready */

	while ((getreg32(STM32_RCC_CR) & RCC_CR_HSI48RDY) == 0) {
	}

#endif

	/* Check for a timeout.	If this timeout occurs, then we are hosed.	We
	 * have no real back-up plan, although the following logic makes it look
	 * as though we do.
	 */

	if (timeout > 0) {
		/* Set the D1 domain Core prescaler and the HCLK source/divider */

		regval = getreg32(STM32_RCC_D1CFGR);
		regval &= ~(RCC_D1CFGR_HPRE_MASK | RCC_D1CFGR_D1CPRE_MASK);
		regval |= (STM32_RCC_D1CFGR_HPRE | STM32_RCC_D1CFGR_D1CPRE);
		putreg32(regval, STM32_RCC_D1CFGR);

		/* Set PCLK1 */

		regval = getreg32(STM32_RCC_D2CFGR);
		regval &= ~RCC_D2CFGR_D2PPRE2_MASK;
		regval |= STM32_RCC_D2CFGR_D2PPRE1;
		putreg32(regval, STM32_RCC_D2CFGR);

		/* Set PCLK2 */

		regval = getreg32(STM32_RCC_D2CFGR);
		regval &= ~RCC_D2CFGR_D2PPRE2_MASK;
		regval |= STM32_RCC_D2CFGR_D2PPRE2;
		putreg32(regval, STM32_RCC_D2CFGR);

		/* Set PCLK3 */

		regval = getreg32(STM32_RCC_D1CFGR);
		regval &= ~RCC_D1CFGR_D1PPRE_MASK;
		regval |= STM32_RCC_D1CFGR_D1PPRE;
		putreg32(regval, STM32_RCC_D1CFGR);

		/* Set PCLK4 */

		regval = getreg32(STM32_RCC_D3CFGR);
		regval &= ~RCC_D3CFGR_D3PPRE_MASK;
		regval |= STM32_RCC_D3CFGR_D3PPRE;
		putreg32(regval, STM32_RCC_D3CFGR);

#ifdef CONFIG_STM32H7_RTC_HSECLOCK
		/* Set the RTC clock divisor */

		regval = getreg32(STM32_RCC_CFGR);
		regval &= ~RCC_CFGR_RTCPRE_MASK;
		regval |= RCC_CFGR_RTCPRE(HSE_DIVISOR);
		putreg32(regval, STM32_RCC_CFGR);
#endif

		/* Configure PLL123 clock source and multipiers */

#ifdef STM32_BOARD_USEHSI
		regval = (RCC_PLLCKSELR_PLLSRC_HSI |
			  STM32_PLLCFG_PLL1M |
			  STM32_PLLCFG_PLL2M |
			  STM32_PLLCFG_PLL3M);
#else /* if STM32_BOARD_USEHSE */
		regval = (RCC_PLLCKSELR_PLLSRC_HSE |
			  STM32_PLLCFG_PLL1M |
			  STM32_PLLCFG_PLL2M |
			  STM32_PLLCFG_PLL3M);
#endif
		putreg32(regval, STM32_RCC_PLLCKSELR);

		/* Each PLL offers 3 outputs with post-dividers (PLLxP/PLLxQ/PLLxR) */

		/* Configure PLL1 dividers */

		regval = (STM32_PLLCFG_PLL1N |
			  STM32_PLLCFG_PLL1P |
			  STM32_PLLCFG_PLL1Q |
			  STM32_PLLCFG_PLL1R);
		putreg32(regval, STM32_RCC_PLL1DIVR);

		/* Configure PLL2 dividers */

		regval = (STM32_PLLCFG_PLL2N |
			  STM32_PLLCFG_PLL2P |
			  STM32_PLLCFG_PLL2Q |
			  STM32_PLLCFG_PLL2R);
		putreg32(regval, STM32_RCC_PLL2DIVR);

		/* Configure PLL3 dividers */

		regval = (STM32_PLLCFG_PLL3N |
			  STM32_PLLCFG_PLL3P |
			  STM32_PLLCFG_PLL3Q |
			  STM32_PLLCFG_PLL3R);
		putreg32(regval, STM32_RCC_PLL3DIVR);

		/* Configure PLLs */

		regval = (STM32_PLLCFG_PLL1CFG |
			  STM32_PLLCFG_PLL2CFG |
			  STM32_PLLCFG_PLL3CFG);
		putreg32(regval, STM32_RCC_PLLCFGR);

		regval = getreg32(STM32_RCC_CR);
#if defined(USE_PLL1)
		/* Enable the PLL1 */

		regval |= RCC_CR_PLL1ON;
#endif

#if defined(USE_PLL2)
		/* Enable the PLL2 */

		regval |= RCC_CR_PLL2ON;
#endif

#if defined(USE_PLL3)
		/* Enable the PLL3 */

		regval |= RCC_CR_PLL3ON;
#endif
		putreg32(regval, STM32_RCC_CR);

#if defined(USE_PLL1)
		/* Wait until the PLL1 is ready */

		while ((getreg32(STM32_RCC_CR) & RCC_CR_PLL1RDY) == 0) {
		}

#endif

#if defined(USE_PLL2)
		/* Wait until the PLL2 is ready */

		while ((getreg32(STM32_RCC_CR) & RCC_CR_PLL2RDY) == 0) {
		}

#endif
#if defined(USE_PLL3)
		/* Wait until the PLL3 is ready */

		while ((getreg32(STM32_RCC_CR) & RCC_CR_PLL3RDY) == 0) {
		}

#endif

		/* Ww must write the lower byte of the PWR_CR3 register is written once
		 * after POR and it shall be written before changing VOS level or ck_sys
		 * clock frequency. No limitation applies to the upper bytes.
		 *
		 * Programming data corresponding to an invalid combination of
		 * LDOEN and BYPASS bits will be ignored: data will not be written,
		 * the written-once mechanism will lock the register and any further
		 * write access will be ignored. The default supply configuration will
		 * be kept and the ACTVOSRDY bit in PWR control status register 1 (PWR_CSR1)
		 * will go on indicating invalid voltage levels.
		 *
		 * N.B. The system shall be power cycled before writing a new value.
		 */

		regval = getreg32(STM32_PWR_CR3);
		regval |= STM32_PWR_CR3_LDOEN | STM32_PWR_CR3_LDOESCUEN;
		putreg32(regval, STM32_PWR_CR3);

		/* Set the voltage output scale */

		regval = getreg32(STM32_PWR_D3CR);
		regval &= ~STM32_PWR_D3CR_VOS_MASK;
		regval |= STM32_PWR_VOS_SCALE;
		putreg32(regval, STM32_PWR_D3CR);

		while ((getreg32(STM32_PWR_D3CR) & STM32_PWR_D3CR_VOSRDY) == 0) {
		}

		/* Over-drive is needed if
		 *	- Voltage output scale 1 mode is selected and SYSCLK frequency is
		 *		over 400 Mhz.
		 */

		if ((STM32_PWR_VOS_SCALE == PWR_D3CR_VOS_SCALE_1) &&
		    STM32_SYSCLK_FREQUENCY > 400000000) {

			/* Enable System configuration controller clock to Enable ODEN */

			regval = getreg32(STM32_RCC_APB4ENR);
			regval |= RCC_APB4ENR_SYSCFGEN;
			putreg32(regval, STM32_RCC_APB4ENR);

			/* Enable Overdrive to extend the clock frequency up to 480 Mhz. */

			regval = getreg32(STM32_SYSCFG_PWRCR);
			regval |= SYSCFG_PWRCR_ODEN;
			putreg32(regval, STM32_SYSCFG_PWRCR);

			while ((getreg32(STM32_PWR_D3CR) & STM32_PWR_D3CR_VOSRDY) == 0) {
			}
		}

		/* Configure FLASH wait states */

		regval = FLASH_ACR_WRHIGHFREQ(BOARD_FLASH_PROGDELAY) |
			 FLASH_ACR_LATENCY(BOARD_FLASH_WAITSTATES);


		putreg32(regval, STM32_FLASH_ACR);

		/* Configure new QSPI source clock */

#if defined(STM32_RCC_D1CCIPR_QSPISEL)
		regval = getreg32(STM32_RCC_D1CCIPR);
		regval &= ~RCC_D1CCIPR_QSPISEL_MASK;
		regval |= STM32_RCC_D1CCIPR_QSPISEL;
		putreg32(regval, STM32_RCC_D1CCIPR);
#endif

		/* Select the PLL1P as system clock source */

		regval = getreg32(STM32_RCC_CFGR);
		regval &= ~RCC_CFGR_SW_MASK;
		regval |= RCC_CFGR_SW_PLL1;
		putreg32(regval, STM32_RCC_CFGR);

		/* Wait until the PLL source is used as the system clock source */

		while ((getreg32(STM32_RCC_CFGR) & RCC_CFGR_SWS_MASK) !=
		       RCC_CFGR_SWS_PLL1) {
		}

		/* Configure I2C source clock */

#if (STM32_RCC_D2CCIP2R_I2C123SRC == RCC_D2CCIP2R_I2C123SEL_HSI)
		/* Enable Internal High-Speed Clock (HSI) */

		regval	= getreg32(STM32_RCC_CR);
		regval |= RCC_CR_HSION;					 /* Enable HSI */
		regval |= RCC_CR_HSIDIV_4;		/* Set HSI to 16 MHz */
		putreg32(regval, STM32_RCC_CR);

		/* Wait until the HSI is ready (or until a timeout elapsed) */

		for (timeout = HSIRDY_TIMEOUT; timeout > 0; timeout--) {
			/* Check if the HSIRDY flag is the set in the CR */

			if ((getreg32(STM32_RCC_CR) & RCC_CR_HSIRDY) != 0) {
				/* If so, then break-out with timeout > 0 */

				break;
			}
		}

#endif

#if defined(STM32_RCC_D2CCIP2R_I2C123SRC)
		regval = getreg32(STM32_RCC_D2CCIP2R);
		regval &= ~RCC_D2CCIP2R_I2C123SEL_MASK;
		regval |= STM32_RCC_D2CCIP2R_I2C123SRC;
		putreg32(regval, STM32_RCC_D2CCIP2R);
#endif

#if defined(STM32_RCC_D3CCIPR_I2C4SRC)
		regval = getreg32(STM32_RCC_D3CCIPR);
		regval &= ~RCC_D3CCIPR_I2C4SEL_MASK;
		regval |= STM32_RCC_D3CCIPR_I2C4SRC;
		putreg32(regval, STM32_RCC_D3CCIPR);
#endif

		/* Configure SPI source clock */

#if defined(STM32_RCC_D2CCIP1R_SPI123SRC)
		regval = getreg32(STM32_RCC_D2CCIP1R);
		regval &= ~RCC_D2CCIP1R_SPI123SEL_MASK;
		regval |= STM32_RCC_D2CCIP1R_SPI123SRC;
		putreg32(regval, STM32_RCC_D2CCIP1R);
#endif

#if defined(STM32_RCC_D2CCIP1R_SPI45SRC)
		regval = getreg32(STM32_RCC_D2CCIP1R);
		regval &= ~RCC_D2CCIP1R_SPI45SEL_MASK;
		regval |= STM32_RCC_D2CCIP1R_SPI45SRC;
		putreg32(regval, STM32_RCC_D2CCIP1R);
#endif

#if defined(STM32_RCC_D3CCIPR_SPI6SRC)
		regval = getreg32(STM32_RCC_D3CCIPR);
		regval &= ~RCC_D3CCIPR_SPI6SEL_MASK;
		regval |= STM32_RCC_D3CCIPR_SPI6SRC;
		putreg32(regval, STM32_RCC_D3CCIPR);
#endif

		/* Configure USB source clock */

#if defined(STM32_RCC_D2CCIP2R_USBSRC)
		regval = getreg32(STM32_RCC_D2CCIP2R);
		regval &= ~RCC_D2CCIP2R_USBSEL_MASK;
		regval |= STM32_RCC_D2CCIP2R_USBSRC;
		putreg32(regval, STM32_RCC_D2CCIP2R);
#endif

		/* Configure ADC source clock */

#if defined(STM32_RCC_D3CCIPR_ADCSEL)
		regval = getreg32(STM32_RCC_D3CCIPR);
		regval &= ~RCC_D3CCIPR_ADCSEL_MASK;
		regval |= STM32_RCC_D3CCIPR_ADCSEL;
		putreg32(regval, STM32_RCC_D3CCIPR);
#endif

#if defined(CONFIG_STM32H7_IWDG) || defined(CONFIG_STM32H7_RTC_LSICLOCK)
		/* Low speed internal clock source LSI */

		stm32_rcc_enablelsi();
#endif

#if defined(CONFIG_STM32H7_RTC_LSECLOCK)
		/* Low speed external clock source LSE
		 *
		 * TODO: There is another case where the LSE needs to
		 * be enabled: if the MCO1 pin selects LSE as source.
		 */

		stm32_rcc_enablelse();
#endif
	}

}

#endif //#if defined(CONFIG_STM32H7_CUSTOM_CLOCKCONFIG)
