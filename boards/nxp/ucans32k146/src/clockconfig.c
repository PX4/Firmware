/****************************************************************************
 * boards/arm/s32k1xx/ucans32k146/src/s32k1xx_clockconfig.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <stdint.h>
#include <stdbool.h>

#include "s32k1xx_clockconfig.h"
#include "s32k1xx_start.h"
#include "board_config.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Each S32K1XX board must provide the following initialized structure.
 * This is needed to establish the initial board clocking.
 */

const struct clock_configuration_s g_initial_clkconfig = {
	.scg =
	{
		.sirc          =
		{
			.range       = SCG_SIRC_RANGE_HIGH,              /* RANGE - High range (8 MHz) */
			.div1        = SCG_ASYNC_CLOCK_DIV_BY_1,         /* SIRCDIV1 */
			.div2        = SCG_ASYNC_CLOCK_DIV_BY_1,         /* SIRCDIV2 */
			.initialize  = true,                             /* Initialize */
			.stopmode    = false,                            /* SIRCSTEN */
			.lowpower    = true,                             /* SIRCLPEN */
			.locked      = false,                            /* LK */
		},
		.firc          =
		{
			.range       = SCG_FIRC_RANGE_48M,               /* RANGE */
			.div1        = SCG_ASYNC_CLOCK_DIV_BY_1,         /* FIRCDIV1 */
			.div2        = SCG_ASYNC_CLOCK_DIV_BY_1,         /* FIRCDIV2 */
			.initialize  = true,                             /* Initialize */
			.regulator   = true,                             /* FIRCREGOFF */
			.locked      = false,                            /* LK */
		},
		.sosc          =
		{
			.mode        = SCG_SOSC_MONITOR_DISABLE,         /* SOSCCM */
			.gain        = SCG_SOSC_GAIN_LOW,                /* HGO */
			.range       = SCG_SOSC_RANGE_HIGH,              /* RANGE */
			.extref      = SCG_SOSC_REF_OSC,                 /* EREFS */
			.div1        = SCG_ASYNC_CLOCK_DIV_BY_1,         /* SOSCDIV1 */
			.div2        = SCG_ASYNC_CLOCK_DIV_BY_1,         /* SOSCDIV2 */
			.initialize  = true,                             /* Initialize */
			.locked      = false,                            /* LK */
		},
		.spll          =
		{
			.mode        = SCG_SPLL_MONITOR_DISABLE,         /* SPLLCM */
			.div1        = SCG_ASYNC_CLOCK_DIV_BY_1,         /* SPLLDIV1 */
			.div2        = SCG_ASYNC_CLOCK_DIV_BY_1,         /* SPLLDIV2 */
			.prediv      = 1,                                /* PREDIV */
			.mult        = 40,                               /* MULT */
			.src         = 0,                                /* SOURCE */
			.initialize  = true,                             /* Initialize */
			.locked      = false,                            /* LK */
		},
		.rtc           =
		{
			.initialize  = true,                             /* Initialize */
			.clkin       = 0,                                /* RTC_CLKIN */
		},
		.clockout      =
		{
			.source      = SCG_CLOCKOUT_SRC_FIRC,            /* SCG CLKOUTSEL */
			.initialize  = true,                             /* Initialize */
		},
		.clockmode     =
		{
			.rccr        =                                   /* RCCR - Run Clock Control Register */
			{
				.src       = SCG_SYSTEM_CLOCK_SRC_SYS_PLL,     /* SCS */
				.divslow   = 3,                                /* DIVSLOW, range 1..16 */
				.divbus    = 2,                                /* DIVBUS, range 1..16 */
				.divcore   = 2,                                /* DIVCORE, range 1..16 */
			},
			.vccr        =                                   /* VCCR - VLPR Clock Control Register */
			{
				.src       = SCG_SYSTEM_CLOCK_SRC_SIRC,        /* SCS */
				.divslow   = 4,                                /* DIVSLOW, range 1..16 */
				.divbus    = 1,                                /* DIVBUS, range 1..16 */
				.divcore   = 2,                                /* DIVCORE, range 1..16 */
			},
			.hccr        =
			{
				.src       = SCG_SYSTEM_CLOCK_SRC_SYS_PLL,     /* SCS */
				.divslow   = 3,                                /* DIVSLOW, range 1..16 */
				.divbus    = 2,                                /* DIVBUS, range 1..16 */
				.divcore   = 2,                                /* DIVCORE, range 1..16 */
			},
			.initialize  = true,                             /* Initialize */
		},
	},
	.sim             =
	{
		.clockout      =                                   /* Clock Out configuration. */
		{
			.source      = SIM_CLKOUT_SEL_SYSTEM_SCG_CLKOUT, /* CLKOUTSEL */
			.divider     = 1,                                /* CLKOUTDIV, range 1..8 */
			.initialize  = true,                             /* Initialize */
			.enable      = false,                            /* CLKOUTEN */
		},
		.lpoclk        =                                   /* Low Power Clock configuration. */
		{
			.rtc_source  = SIM_RTCCLK_SEL_LPO_32K,           /* RTCCLKSEL */
			.lpo_source  = SIM_LPO_CLK_SEL_LPO_128K,         /* LPOCLKSEL */
			.initialize  = true,                             /* Initialize */
			.lpo32k      = true,                             /* LPO32KCLKEN */
			.lpo1k       = true,                             /* LPO1KCLKEN */
		},
		.tclk          =                                   /* TCLK CLOCK configuration. */
		{
			.tclkfreq[0] = 0,                                /* TCLK0 */
			.tclkfreq[1] = 0,                                /* TCLK1 */
			.tclkfreq[2] = 0,                                /* TCLK2 */
			.initialize  = true,                             /* Initialize */
		},
		.platgate      =                                   /* Platform Gate Clock configuration. */
		{
			.initialize  = true,                             /* Initialize */
			.mscm        = true,                             /* CGCMSCM */
			.mpu         = true,                             /* CGCMPU */
			.dma         = true,                             /* CGCDMA */
			.erm         = true,                             /* CGCERM */
			.eim         = true,                             /* CGCEIM */
		},
		.traceclk      =                                   /* Debug trace Clock Configuration. */
		{
			.source      = CLOCK_TRACE_SRC_CORE_CLK,         /* TRACECLK_SEL */
			.divider     = 1,                                /* TRACEDIV, range 1..8 */
			.initialize  = true,                             /* Initialize */
			.enable      = true,                             /* TRACEDIVEN */
			.fraction    = false,                            /* TRACEFRAC */
		},
	},
	.pcc             =
	{
		.count         = NUM_OF_PERIPHERAL_CLOCKS_0,       /* Number peripheral clock configurations */
		.pclks         = g_peripheral_clockconfig0,        /* Peripheral clock configurations */
	},
	.pmc             =
	{
		.lpoclk        =                                   /* Low Power Clock configuration. */
		{
			.trim        = 0,                                /* Trimming value for LPO */
			.initialize  = true,                             /* Initialize */
			.enable      = true,                             /* Enable/disable LPO */
		},
	},
};
