/****************************************************************************
 *
 *   Copyright (C) 2018-2019 PX4 Development Team. All rights reserved.
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
 * @file adc.cpp
 *
 * Driver for the imxrt ADC.
 *
 * This is a low-rate driver, designed for sampling things like voltages
 * and so forth. It avoids the gross complexity of the NuttX ADC driver.
 */

#include <board_config.h>
#include <stdint.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_adc.h>
#include <px4_arch/adc.h>

#include <hardware/imxrt_adc.h>
#include <imxrt_periphclks.h>

typedef uint32_t 	adc_chan_t;
#define ADC_TOTAL_CHANNELS 		16

#define _REG(_addr)	(*(volatile uint32_t *)(_addr))

/* ADC register accessors */

#define REG(base_address, _reg)	_REG((base_address) + (_reg))

#define rHC0(base_address)  REG(base_address, IMXRT_ADC_HC0_OFFSET)  /* Control register for hardware triggers */
#define rHC1(base_address)  REG(base_address, IMXRT_ADC_HC1_OFFSET)  /* Control register for hardware triggers */
#define rHC2(base_address)  REG(base_address, IMXRT_ADC_HC2_OFFSET)  /* Control register for hardware triggers */
#define rHC3(base_address)  REG(base_address, IMXRT_ADC_HC3_OFFSET)  /* Control register for hardware triggers */
#define rHC4(base_address)  REG(base_address, IMXRT_ADC_HC4_OFFSET)  /* Control register for hardware triggers */
#define rHC5(base_address)  REG(base_address, IMXRT_ADC_HC5_OFFSET)  /* Control register for hardware triggers */
#define rHC6(base_address)  REG(base_address, IMXRT_ADC_HC6_OFFSET)  /* Control register for hardware triggers */
#define rHC7(base_address)  REG(base_address, IMXRT_ADC_HC7_OFFSET)  /* Control register for hardware triggers */
#define rHS(base_address)   REG(base_address, IMXRT_ADC_HS_OFFSET)   /* Status register for HW triggers */
#define rR0(base_address)   REG(base_address, IMXRT_ADC_R0_OFFSET)   /* Data result register for HW triggers */
#define rR1(base_address)   REG(base_address, IMXRT_ADC_R1_OFFSET)   /* Data result register for HW triggers */
#define rR2(base_address)   REG(base_address, IMXRT_ADC_R2_OFFSET)   /* Data result register for HW triggers */
#define rR3(base_address)   REG(base_address, IMXRT_ADC_R3_OFFSET)   /* Data result register for HW triggers */
#define rR4(base_address)   REG(base_address, IMXRT_ADC_R4_OFFSET)   /* Data result register for HW triggers */
#define rR5(base_address)   REG(base_address, IMXRT_ADC_R5_OFFSET)   /* Data result register for HW triggers */
#define rR6(base_address)   REG(base_address, IMXRT_ADC_R6_OFFSET)   /* Data result register for HW triggers */
#define rR7(base_address)   REG(base_address, IMXRT_ADC_R7_OFFSET)   /* Data result register for HW triggers */
#define rCFG(base_address)  REG(base_address, IMXRT_ADC_CFG_OFFSET)  /* Configuration register */
#define rGC(base_address)   REG(base_address, IMXRT_ADC_GC_OFFSET)   /* General control register */
#define rGS(base_address)   REG(base_address, IMXRT_ADC_GS_OFFSET)   /* General status register */
#define rCV(base_address)   REG(base_address, IMXRT_ADC_CV_OFFSET)   /* Compare value register */
#define rOFS(base_address)  REG(base_address, IMXRT_ADC_OFS_OFFSET)  /* Offset correction value register */
#define rCAL(base_address)  REG(base_address, IMXRT_ADC_CAL_OFFSET)  /* Calibration value register */


int px4_arch_adc_init(uint32_t base_address)
{
	static bool once = false;

	if (!once) {

		once = true;

		/* Input is Buss Clock 56 Mhz We will use /8 for 7 Mhz */

		irqstate_t flags = px4_enter_critical_section();

		imxrt_clockall_adc1();

		rCFG(base_address) = ADC_CFG_ADICLK_IPGDIV2 | ADC_CFG_MODE_12BIT | \
				     ADC_CFG_ADIV_DIV8 | ADC_CFG_ADLSMP | ADC_CFG_ADSTS_7_21 | \
				     ADC_CFG_AVGS_4SMPL | ADC_CFG_OVWREN;
		px4_leave_critical_section(flags);

		/* Clear the CALF and begin the calibration */

		rGS(base_address) = ADC_GS_CALF;
		rGC(base_address) = ADC_GC_CAL;
		uint32_t guard = 100;

		while (guard != 0 && (rGS(base_address) & ADC_GC_CAL) == 0) {
			guard--;
			usleep(1);
		}

		while ((rGS(base_address) & ADC_GC_CAL) == ADC_GC_CAL) {

			usleep(100);

			if (rGS(base_address) & ADC_GS_CALF) {
				return -1;
			}
		}

		if ((rHS(base_address) & ADC_HS_COCO0) == 0) {
			return -2;
		}

		if (rGS(base_address) & ADC_GS_CALF) {
			return -3;
		}

		/* dummy read to clear COCO of calibration */

		int32_t r = rR0(base_address);
		UNUSED(r);

		/* kick off a sample and wait for it to complete */
		hrt_abstime now = hrt_absolute_time();
		rGC(base_address) = ADC_GC_AVGE;
		rHC0(base_address) =  0xd; // VREFSH = internal channel, for ADC self-test, hard connected to VRH internally

		while (!(rHS(base_address) & ADC_HS_COCO0)) {

			/* don't wait for more than 500us, since that means something broke -
			 * should reset here if we see this
			 */

			if ((hrt_absolute_time() - now) > 500) {
				return -4;
			}
		}

		r = rR0(base_address);
	} // once

	return 0;
}
void px4_arch_adc_uninit(uint32_t base_address)
{
	imxrt_clockoff_adc1();
}

uint32_t px4_arch_adc_sample(uint32_t base_address, unsigned channel)
{

	/* clear any previous COCO0 */

	uint16_t result = rR0(base_address);

	rHC0(base_address) =  channel;

	/* wait for the conversion to complete */
	hrt_abstime now = hrt_absolute_time();

	while (!(rHS(base_address) & ADC_HS_COCO0)) {
		/* don't wait for more than 50us, since that means something broke
		 *  should reset here if we see this
		 */
		if ((hrt_absolute_time() - now) > 50) {
			return 0xffff;
		}
	}

	/* read the result and clear  COCO0 */
	result  = rR0(base_address);
	return result;
}

float px4_arch_adc_reference_v()
{
	return BOARD_ADC_POS_REF_V;	// TODO: provide true vref
}

uint32_t px4_arch_adc_temp_sensor_mask()
{
	return 0;
}

uint32_t px4_arch_adc_dn_fullcount(void)
{
	return 1 << 12; // 12 bit ADC
}
