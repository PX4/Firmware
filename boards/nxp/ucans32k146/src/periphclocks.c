/****************************************************************************
 * boards/arm/s32k1xx/ucans32k146/src/s32k1xx_periphclks.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimer in
 *  the documentation and/or other materials provided with the
 *  distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *  used to endorse or promote products derived from this software
 *  without specific prior written permission.
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
 * Most of the settings within this file derives from NXP sample code for
 * the S32K1XX MCUs.  That sample code has this licensing information:
 *
 *   Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 *   Copyright 2016-2018 NXP
 *   All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "s32k1xx_periphclocks.h"
#include "board_config.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Each S32K1XX board must provide the following initialized structure.
 * This is needed to establish the initial peripheral clocking.
 */

const struct peripheral_clock_config_s g_peripheral_clockconfig0[] = {
	{
		.clkname = FLEXCAN0_CLK,
#ifdef CONFIG_S32K1XX_FLEXCAN0
		.clkgate = true,
#else
		.clkgate = false,
#endif
	},
	{
		.clkname = FLEXCAN1_CLK,
#ifdef CONFIG_S32K1XX_FLEXCAN1
		.clkgate = true,
#else
		.clkgate = false,
#endif
	},
	{
		.clkname = LPI2C0_CLK,
#ifdef CONFIG_S32K1XX_LPI2C0
		.clkgate = true,
#else
		.clkgate = false,
#endif
		.clksrc  = CLK_SRC_SIRC_DIV2,
	},
	{
		.clkname = LPSPI0_CLK,
#ifdef CONFIG_S32K1XX_LPSPI0
		.clkgate = true,
#else
		.clkgate = false,
#endif
		.clksrc  = CLK_SRC_SIRC_DIV2,
	},
	{
		.clkname = LPUART0_CLK,
#ifdef CONFIG_S32K1XX_LPUART0
		.clkgate = true,
#else
		.clkgate = false,
#endif
		.clksrc  = CLK_SRC_SIRC_DIV2,
	},
	{
		.clkname = LPUART1_CLK,
#ifdef CONFIG_S32K1XX_LPUART1
		.clkgate = true,
#else
		.clkgate = false,
#endif
		.clksrc  = CLK_SRC_SIRC_DIV2,
	},
	{
		.clkname    = RTC0_CLK,
#ifdef CONFIG_S32K1XX_RTC
		.clkgate = true,
#else
		.clkgate = false,
#endif
		.clksrc     = CLK_SRC_OFF,
		.frac       = MULTIPLY_BY_ONE,
		.divider    = 1,
	},
	{
		.clkname    = FTM0_CLK,
		.clkgate    = true,
		.clksrc     = CLK_SRC_SOSC_DIV1,
	},
	{
		.clkname    = FTM1_CLK,
		.clkgate    = true,
		.clksrc     = CLK_SRC_SOSC_DIV1,
	},
	{
		.clkname    = FTM2_CLK,
		.clkgate    = true,
		.clksrc     = CLK_SRC_SOSC_DIV1,
	},
	{
		.clkname    = FTM3_CLK,
		.clkgate    = true,
		.clksrc     = CLK_SRC_SOSC_DIV1,
	},
	{
		.clkname    = FTM4_CLK,
		.clkgate    = true,
		.clksrc     = CLK_SRC_SOSC_DIV1,
	},
	{
		.clkname    = FTM5_CLK,
		.clkgate    = true,
		.clksrc     = CLK_SRC_SOSC_DIV1,
	},
	{
		.clkname = PORTA_CLK,
		.clkgate = true,
	},
	{
		.clkname = PORTB_CLK,
		.clkgate = true,
	},
	{
		.clkname = PORTC_CLK,
		.clkgate = true,
	},
	{
		.clkname = PORTD_CLK,
		.clkgate = true,
	},
	{
		.clkname = PORTE_CLK,
		.clkgate = true,
	},
};
