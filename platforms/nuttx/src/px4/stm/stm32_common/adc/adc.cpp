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
#include <stdint.h>
#include <drivers/drv_adc.h>
#include <drivers/drv_hrt.h>
#include <px4_arch/adc.h>

#include <stm32_adc.h>
#include <stm32_gpio.h>

/*
 * Register accessors.
 * For now, no reason not to just use ADC1.
 */
#define REG(base, _reg) (*(volatile uint32_t *)((base) + (_reg)))

#define rSR(base)    REG((base), STM32_ADC_SR_OFFSET)
#define rCR1(base)   REG((base), STM32_ADC_CR1_OFFSET)
#define rCR2(base)   REG((base), STM32_ADC_CR2_OFFSET)
#define rSMPR1(base) REG((base), STM32_ADC_SMPR1_OFFSET)
#define rSMPR2(base) REG((base), STM32_ADC_SMPR2_OFFSET)
#define rJOFR1(base) REG((base), STM32_ADC_JOFR1_OFFSET)
#define rJOFR2(base) REG((base), STM32_ADC_JOFR2_OFFSET)
#define rJOFR3(base) REG((base), STM32_ADC_JOFR3_OFFSET)
#define rJOFR4(base) REG((base), STM32_ADC_JOFR4_OFFSET)
#define rHTR(base)   REG((base), STM32_ADC_HTR_OFFSET)
#define rLTR(base)   REG((base), STM32_ADC_LTR_OFFSET)
#define rSQR1(base)  REG((base), STM32_ADC_SQR1_OFFSET)
#define rSQR2(base)  REG((base), STM32_ADC_SQR2_OFFSET)
#define rSQR3(base)  REG((base), STM32_ADC_SQR3_OFFSET)
#define rJSQR(base)  REG((base), STM32_ADC_JSQR_OFFSET)
#define rJDR1(base)  REG((base), STM32_ADC_JDR1_OFFSET)
#define rJDR2(base)  REG((base), STM32_ADC_JDR2_OFFSET)
#define rJDR3(base)  REG((base), STM32_ADC_JDR3_OFFSET)
#define rJDR4(base)  REG((base), STM32_ADC_JDR4_OFFSET)
#define rDR(base)    REG((base), STM32_ADC_DR_OFFSET)



#ifdef STM32_ADC_CCR
# define rCCR(base)		REG((STM32_ADCCMN_BASE), STM32_ADC_CCR_OFFSET)

/* Assuming VDC 2.4 - 3.6 */

#define ADC_MAX_FADC 36000000

#  if STM32_PCLK2_FREQUENCY/2 <= ADC_MAX_FADC
#    define ADC_CCR_ADCPRE_DIV     ADC_CCR_ADCPRE_DIV2
#  elif STM32_PCLK2_FREQUENCY/4 <= ADC_MAX_FADC
#    define ADC_CCR_ADCPRE_DIV     ADC_CCR_ADCPRE_DIV4
#  elif STM32_PCLK2_FREQUENCY/6 <= ADC_MAX_FADC
#   define ADC_CCR_ADCPRE_DIV     ADC_CCR_ADCPRE_DIV6
#  elif STM32_PCLK2_FREQUENCY/8 <= ADC_MAX_FADC
#   define ADC_CCR_ADCPRE_DIV     ADC_CCR_ADCPRE_DIV8
#  else
#    error "ADC PCLK2 too high - no divisor found "
#  endif
#endif


int px4_arch_adc_init(uint32_t base_address)
{
	/* Perform ADC init once per ADC */

	static uint32_t once[SYSTEM_ADC_COUNT] {};

	uint32_t *free = nullptr;

	for (uint32_t i = 0; i < SYSTEM_ADC_COUNT; i++) {
		if (once[i] == base_address) {

			/* This one was done already */

			return OK;
		}

		/* Use first free slot */

		if (free == nullptr && once[i] == 0) {
			free = &once[i];
		}
	}

	if (free == nullptr) {

		/* ADC misconfigured SYSTEM_ADC_COUNT too small */;

		PANIC();
	}

	*free = base_address;

	/* do calibration if supported */
#ifdef ADC_CR2_CAL
	rCR2(base_address) |= ADC_CR2_CAL;
	px4_usleep(100);

	if (rCR2(base_address) & ADC_CR2_CAL) {
		return -1;
	}

#endif

	/* arbitrarily configure all channels for 55 cycle sample time */
	rSMPR1(base_address) = 0b00000011011011011011011011011011;
	rSMPR2(base_address) = 0b00011011011011011011011011011011;

	/* XXX for F2/4, might want to select 12-bit mode? */
	rCR1(base_address) = 0;

	/* enable the temperature sensor / Vrefint channel if supported*/
	rCR2(base_address) =
#ifdef ADC_CR2_TSVREFE
		/* enable the temperature sensor in CR2 */
		ADC_CR2_TSVREFE |
#endif
		0;

	/* Soc have CCR */
#ifdef STM32_ADC_CCR
#  ifdef ADC_CCR_TSVREFE
	/* enable temperature sensor in CCR */
	rCCR(base_address) = ADC_CCR_TSVREFE | ADC_CCR_ADCPRE_DIV;
#  else
	rCCR(base_address) = ADC_CCR_ADCPRE_DIV;
#  endif
#endif

	/* configure for a single-channel sequence */
	rSQR1(base_address) = 0;
	rSQR2(base_address) = 0;
	rSQR3(base_address) = 0;	/* will be updated with the channel each tick */

	/* power-cycle the ADC and turn it on */
	rCR2(base_address) &= ~ADC_CR2_ADON;
	px4_usleep(10);
	rCR2(base_address) |= ADC_CR2_ADON;
	px4_usleep(10);
	rCR2(base_address) |= ADC_CR2_ADON;
	px4_usleep(10);

	/* kick off a sample and wait for it to complete */
	hrt_abstime now = hrt_absolute_time();
	rCR2(base_address) |= ADC_CR2_SWSTART;

	while (!(rSR(base_address) & ADC_SR_EOC)) {

		/* don't wait for more than 500us, since that means something broke - should reset here if we see this */
		if ((hrt_absolute_time() - now) > 500) {
			return -1;
		}
	}

	/* Read out result, clear EOC */

	(void) rDR(base_address);

	return 0;
}

void px4_arch_adc_uninit(uint32_t base_address)
{
	// nothing to do
}

uint32_t px4_arch_adc_sample(uint32_t base_address, unsigned channel)
{
	irqstate_t flags = px4_enter_critical_section();

	/* clear any previous EOC */
	if (rSR(base_address) & ADC_SR_EOC) {
		rSR(base_address) &= ~ADC_SR_EOC;
	}

	/* run a single conversion right now - should take about 60 cycles (a few microseconds) max */
	rSQR3(base_address) = channel;
	rCR2(base_address) |= ADC_CR2_SWSTART;

	/* wait for the conversion to complete */
	const hrt_abstime now = hrt_absolute_time();

	while (!(rSR(base_address) & ADC_SR_EOC)) {

		/* don't wait for more than 50us, since that means something broke - should reset here if we see this */
		if ((hrt_absolute_time() - now) > 50) {
			px4_leave_critical_section(flags);
			return UINT32_MAX;
		}
	}

	/* read the result and clear EOC */
	uint32_t result = rDR(base_address);

	px4_leave_critical_section(flags);

	return result;
}

float px4_arch_adc_reference_v()
{
	return BOARD_ADC_POS_REF_V;	// TODO: provide true vref
}

uint32_t px4_arch_adc_temp_sensor_mask()
{

	return 1 << PX4_ADC_INTERNAL_TEMP_SENSOR_CHANNEL;

}

uint32_t px4_arch_adc_dn_fullcount()
{
	return 1 << 12; // 12 bit ADC
}
