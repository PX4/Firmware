/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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

#include <board_config.h>
#include <drivers/drv_adc.h>
#include <drivers/drv_hrt.h>
#include <px4_arch/adc.h>

#include <nuttx/analog/adc.h>

#include <hardware/s32k1xx_sim.h>

//todo S32K add ADC fior now steal the kinetis one
#include <kinetis.h>
#include <hardware/kinetis_adc.h>


#define _REG(_addr)	(*(volatile uint32_t *)(_addr))

/* ADC register accessors */

#define REG(a, _reg)	_REG(KINETIS_ADC##a##_BASE + (_reg))

#define rSC1A(adc)  REG(adc, KINETIS_ADC_SC1A_OFFSET) /* ADC status and control registers 1 */
#define rSC1B(adc)  REG(adc, KINETIS_ADC_SC1B_OFFSET) /* ADC status and control registers 1 */
#define rCFG1(adc)  REG(adc, KINETIS_ADC_CFG1_OFFSET) /* ADC configuration register 1 */
#define rCFG2(adc)  REG(adc, KINETIS_ADC_CFG2_OFFSET) /* Configuration register 2 */
#define rRA(adc)    REG(adc, KINETIS_ADC_RA_OFFSET)   /* ADC data result register */
#define rRB(adc)    REG(adc, KINETIS_ADC_RB_OFFSET)   /* ADC data result register */
#define rCV1(adc)   REG(adc, KINETIS_ADC_CV1_OFFSET)  /* Compare value registers */
#define rCV2(adc)   REG(adc, KINETIS_ADC_CV2_OFFSET)  /* Compare value registers */
#define rSC2(adc)   REG(adc, KINETIS_ADC_SC2_OFFSET)  /* Status and control register 2 */
#define rSC3(adc)   REG(adc, KINETIS_ADC_SC3_OFFSET)  /* Status and control register 3 */
#define rOFS(adc)   REG(adc, KINETIS_ADC_OFS_OFFSET)  /* ADC offset correction register */
#define rPG(adc)    REG(adc, KINETIS_ADC_PG_OFFSET)   /* ADC plus-side gain register */
#define rMG(adc)    REG(adc, KINETIS_ADC_MG_OFFSET)   /* ADC minus-side gain register */
#define rCLPD(adc)  REG(adc, KINETIS_ADC_CLPD_OFFSET) /* ADC plus-side general calibration value register */
#define rCLPS(adc)  REG(adc, KINETIS_ADC_CLPS_OFFSET) /* ADC plus-side general calibration value register */
#define rCLP4(adc)  REG(adc, KINETIS_ADC_CLP4_OFFSET) /* ADC plus-side general calibration value register */
#define rCLP3(adc)  REG(adc, KINETIS_ADC_CLP3_OFFSET) /* ADC plus-side general calibration value register */
#define rCLP2(adc)  REG(adc, KINETIS_ADC_CLP2_OFFSET) /* ADC plus-side general calibration value register */
#define rCLP1(adc)  REG(adc, KINETIS_ADC_CLP1_OFFSET) /* ADC plus-side general calibration value register */
#define rCLP0(adc)  REG(adc, KINETIS_ADC_CLP0_OFFSET) /* ADC plus-side general calibration value register */
#define rCLMD(adc)  REG(adc, KINETIS_ADC_CLMD_OFFSET) /* ADC minus-side general calibration value register */
#define rCLMS(adc)  REG(adc, KINETIS_ADC_CLMS_OFFSET) /* ADC minus-side general calibration value register */
#define rCLM4(adc)  REG(adc, KINETIS_ADC_CLM4_OFFSET) /* ADC minus-side general calibration value register */
#define rCLM3(adc)  REG(adc, KINETIS_ADC_CLM3_OFFSET) /* ADC minus-side general calibration value register */
#define rCLM2(adc)  REG(adc, KINETIS_ADC_CLM2_OFFSET) /* ADC minus-side general calibration value register */
#define rCLM1(adc)  REG(adc, KINETIS_ADC_CLM1_OFFSET) /* ADC minus-side general calibration value register */
#define rCLM0(adc)  REG(adc, KINETIS_ADC_CLM0_OFFSET) /* ADC minus-side general calibration value register */

int px4_arch_adc_init(uint32_t base_address)
{
	/* Input is Buss Clock 56 Mhz We will use /8 for 7 Mhz */

	irqstate_t flags = px4_enter_critical_section();

	_REG(KINETIS_SIM_SCGC3) |= SIM_SCGC3_ADC1;
	rCFG1(1) = ADC_CFG1_ADICLK_BUSCLK | ADC_CFG1_MODE_1213BIT | ADC_CFG1_ADIV_DIV8;
	rCFG2(1) = 0;
	rSC2(1) = ADC_SC2_REFSEL_DEFAULT;

	px4_leave_critical_section(flags);

	/* Clear the CALF and begin the calibration */

	rSC3(1) = ADC_SC3_CAL | ADC_SC3_CALF;

	while ((rSC1A(1) & ADC_SC1_COCO) == 0) {
		usleep(100);

		if (rSC3(1) & ADC_SC3_CALF) {
			return -1;
		}
	}

	/* dummy read to clear COCO of calibration */

	int32_t r = rRA(1);

	/* Check the state of CALF at the end of calibration */

	if (rSC3(1) & ADC_SC3_CALF) {
		return -1;
	}

	/* Calculate the calibration values for single ended positive */

	r = rCLP0(1) + rCLP1(1)  + rCLP2(1)  + rCLP3(1)  + rCLP4(1)  + rCLPS(1) ;
	r = 0x8000U | (r >> 1U);
	rPG(1) = r;

	/* Calculate the calibration values for double ended Negitive */

	r = rCLM0(1) + rCLM1(1)  + rCLM2(1)  + rCLM3(1)  + rCLM4(1)  + rCLMS(1) ;
	r = 0x8000U | (r >> 1U);
	rMG(1) = r;

	/* kick off a sample and wait for it to complete */
	hrt_abstime now = hrt_absolute_time();

	rSC1A(1) =  ADC_SC1_ADCH(ADC_SC1_ADCH_TEMP);

	while (!(rSC1A(1) & ADC_SC1_COCO)) {

		/* don't wait for more than 500us, since that means something broke - should reset here if we see this */
		if ((hrt_absolute_time() - now) > 500) {
			return -1;
		}
	}

	return 0;
}

void px4_arch_adc_uninit(uint32_t base_address)
{
	irqstate_t flags = px4_enter_critical_section();
	_REG(KINETIS_SIM_SCGC3) &= ~SIM_SCGC3_ADC1;
	px4_leave_critical_section(flags);
}

uint32_t px4_arch_adc_sample(uint32_t base_address, unsigned channel)
{
	irqstate_t flags = px4_enter_critical_section();

	/* clear any previous COCC */
	rRA(1);

	/* run a single conversion right now - should take about 35 cycles (5 microseconds) max */
	rSC1A(1) = ADC_SC1_ADCH(channel);

	/* wait for the conversion to complete */
	const hrt_abstime now = hrt_absolute_time();

	while (!(rSC1A(1) & ADC_SC1_COCO)) {

		/* don't wait for more than 10us, since that means something broke - should reset here if we see this */
		if ((hrt_absolute_time() - now) > 10) {
			px4_leave_critical_section(flags);
			return 0xffff;
		}
	}

	/* read the result and clear EOC */
	uint32_t result = rRA(1);

	px4_leave_critical_section(flags);

	return result;
}

float px4_arch_adc_reference_v()
{
	return BOARD_ADC_POS_REF_V;	// TODO: provide true vref
}

uint32_t px4_arch_adc_temp_sensor_mask()
{
	return 1 << (ADC_SC1_ADCH_TEMP >> ADC_SC1_ADCH_SHIFT);
}

uint32_t px4_arch_adc_dn_fullcount()
{
	return 1 << 12; // 12 bit ADC
}
