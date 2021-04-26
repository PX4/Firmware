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
 *  If there is only one ADC in use in PX4 and it is not
 *  ADC3 we still need ADC3 for temperature sensing.
 */
#if SYSTEM_ADC_COUNT == 1 && SYSTEM_ADC_BASE != STM32_ADC3_BASE
#  undef SYSTEM_ADC_COUNT
#  define SYSTEM_ADC_COUNT 2
#endif

/*
 * Register accessors.
 * For now, no reason not to just use ADC1.
 */
#define REG(base, _reg) (*(volatile uint32_t *)((base) + (_reg)))

#define rCR(base)    REG((base), STM32_ADC_CR_OFFSET)
#define rISR(base)   REG((base), STM32_ADC_ISR_OFFSET)
#define rSMPR1(base) REG((base), STM32_ADC_SMPR1_OFFSET)
#define rSMPR2(base) REG((base), STM32_ADC_SMPR2_OFFSET)
#define rPCSEL(base) REG((base), STM32_ADC_PCSEL_OFFSET)
#define rCFG(base)   REG((base), STM32_ADC_CFGR_OFFSET)
#define rCFG2(base)  REG((base), STM32_ADC_CFGR2_OFFSET)
#define rCCR()       REG((STM32_ADC1_BASE), (STM32_ADC_CCR_OFFSET))
#define rSQR1(base)  REG((base), STM32_ADC_SQR1_OFFSET)
#define rSQR2(base)  REG((base), STM32_ADC_SQR2_OFFSET)
#define rSQR3(base)  REG((base), STM32_ADC_SQR3_OFFSET)
#define rSQR4(base)  REG((base), STM32_ADC_SQR4_OFFSET)
#define rDR(base)    REG((base), STM32_ADC_DR_OFFSET)

#define ADC_SMPR_DEFAULT    ADC_SMPR_64p5 // 64.5 +7.5 * 24 Mhz is 3 uS
#define ADC_SMPR1_DEFAULT   ((ADC_SMPR_DEFAULT << ADC_SMPR1_SMP0_SHIFT) | \
			     (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP1_SHIFT) | \
			     (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP2_SHIFT) | \
			     (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP3_SHIFT) | \
			     (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP4_SHIFT) | \
			     (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP5_SHIFT) | \
			     (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP6_SHIFT) | \
			     (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP7_SHIFT) | \
			     (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP8_SHIFT) | \
			     (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP9_SHIFT))
#define ADC_SMPR2_DEFAULT   ((ADC_SMPR_DEFAULT << ADC_SMPR2_SMP10_SHIFT) | \
			     (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP11_SHIFT) | \
			     (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP12_SHIFT) | \
			     (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP13_SHIFT) | \
			     (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP14_SHIFT) | \
			     (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP15_SHIFT) | \
			     (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP16_SHIFT) | \
			     (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP17_SHIFT) | \
			     (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP18_SHIFT) | \
			     (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP19_SHIFT))


/* Assuming VDC 2.4 - 3.6 */

#define ADC_MAX_FADC 36000000

#if STM32_PLL2P_FREQUENCY     <= ADC_MAX_FADC
#  define ADC_CCR_PRESC_DIV     ADC_CCR_PRESC_NOT_DIV
#elif STM32_PLL2P_FREQUENCY/2 <= ADC_MAX_FADC
#  define ADC_CCR_PRESC_DIV     ADC_CCR_PRESC_DIV2
#elif STM32_PLL2P_FREQUENCY/4 <= ADC_MAX_FADC
#  define ADC_CCR_PRESC_DIV     ADC_CCR_PRESC_DIV4
#elif STM32_PLL2P_FREQUENCY/6 <= ADC_MAX_FADC
# define ADC_CCR_PRESC_DIV     ADC_CCR_PRESC_DIV6
#elif STM32_PLL2P_FREQUENCY/8 <= ADC_MAX_FADC
# define ADC_CCR_PRESC_DIV     ADC_CCR_PRESC_DIV8
#elif STM32_PLL2P_FREQUENCY/10 <= ADC_MAX_FADC
# define ADC_CCR_PRESC_DIV     ADC_CCR_PRESC_DIV10
#elif STM32_PLL2P_FREQUENCY/12 <= ADC_MAX_FADC
# define ADC_CCR_PRESC_DIV     ADC_CCR_PRESC_DIV12
#elif STM32_PLL2P_FREQUENCY/16 <= ADC_MAX_FADC
# define ADC_CCR_PRESC_DIV     ADC_CCR_PRESC_DIV16
#elif STM32_PLL2P_FREQUENCY/32 <= ADC_MAX_FADC
# define ADC_CCR_PRESC_DIV     ADC_CCR_PRESC_DIV32
#elif STM32_PLL2P_FREQUENCY/64 <= ADC_MAX_FADC
# define ADC_CCR_PRESC_DIV     ADC_CCR_PRESC_DIV64
#elif STM32_PLL2P_FREQUENCY/128 <= ADC_MAX_FADC
# define ADC_CCR_PRESC_DIV     ADC_CCR_PRESC_DIV128
#elif STM32_PLL2P_FREQUENCY/256 <= ADC_MAX_FADC
# define ADC_CCR_PRESC_DIV     ADC_CCR_PRESC_DIV256
#else
#  error "ADC STM32_PLL2P_FREQUENCY too high - no divisor found "
#endif

#define ADC3_INTERNAL_TEMP_SENSOR_CHANNEL 18 //define to map the internal temperature channel.

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

	rCR(base_address)  = ADC_CR_ADVREGEN | ADC_CR_BOOST;

	/* Wait for voltage regulator to power up */

	up_udelay(20);

	/* enable the temperature sensor, VREFINT channel and VBAT */

	rCCR() = (ADC_CCR_VREFEN | ADC_CCR_VSENSEEN | ADC_CCR_VBATEN |
		  ADC_CCR_CKMODE_ASYCH | ADC_CCR_PRESC_DIV);

	/* Enable ADC calibration. ADCALDIF == 0 so this is only for
	 * single-ended conversions, not for differential ones.
	 * Do Liner Cal first
	 */

	rCR(base_address)  |= ADC_CR_ADCALLIN;
	rCR(base_address)  |= ADC_CR_ADCAL;

	/* Wait for calibration to complete */

	hrt_abstime now = hrt_absolute_time();

	while ((rCR(base_address) & ADC_CR_ADCAL)) {

		/* don't wait for more than 15000us, since that means something broke
		 * should reset here if we see this */
		if ((hrt_absolute_time() - now) > 15000) {
			return -1;
		}
	}

	rCR(base_address)  &= ~ADC_CR_ADCALLIN;

	rCR(base_address)  |= ADC_CR_ADCAL;

	/* Wait for calibration to complete */

	now = hrt_absolute_time();

	while ((rCR(base_address) & ADC_CR_ADCAL)) {

		/* don't wait for more than 500us, since that means something broke
		 * should reset here if we see this */
		if ((hrt_absolute_time() - now) > 500) {
			return -2;
		}
	}


	/* Enable ADC
	 * Note: ADEN bit cannot be set during ADCAL=1 and 4 ADC clock cycle
	 * after the ADCAL bit is cleared by hardware. If we are using SYSCLK
	 * as ADC clock source, this is the same as time taken to execute 4
	 * ARM instructions.
	 */

	rCR(base_address) |= ADC_CR_ADEN;

	now = hrt_absolute_time();

	/* Wait for hardware to be ready for conversions */

	while ((rISR(base_address) & ADC_INT_ADRDY) == 0) {

		/* don't wait for more than 500us, since that means something broke
		 * should reset here if we see this */
		if ((hrt_absolute_time() - now) > 500) {
			return -3;
		}
	}


	/* arbitrarily configure all channels for 64.5 cycle sample time */

	rSMPR1(base_address) = ADC_SMPR1_DEFAULT;
	rSMPR2(base_address) = ADC_SMPR2_DEFAULT;


	/* Set CFGR configuration
	 * Set the resolution of the conversion.
	 * Disable external trigger for regular channels
	 */

	rCFG(base_address) = (ADC_CFGR_RES_16BIT | ADC_CFGR_EXTEN_NONE);

	/* Set CFGR2 configuration to align right no oversample */

	rCFG2(base_address) = 0;

	/* configure for a single-channel sequence */

	rSQR1(base_address) = 0;
	rSQR2(base_address) = 0;
	rSQR3(base_address) = 0;
	rSQR4(base_address) = 0;

	/* kick off a sample and wait for it to complete */
	now = hrt_absolute_time();
	rCR(base_address) |= ADC_CR_ADSTART;

	while (!(rISR(base_address) & ADC_INT_EOC)) {

		/* don't wait for more than 50us, since that means something broke - should reset here if we see this */
		if ((hrt_absolute_time() - now) > 50) {
			return -4;
		}
	}

	/* Read out result, clear EOC */

	(void) rDR(base_address);

	return OK;
}

void px4_arch_adc_uninit(uint32_t base_address)
{
	// nothing to do
}

uint32_t px4_arch_adc_sample(uint32_t base_address, unsigned channel)
{
	irqstate_t flags = px4_enter_critical_section();

	/* Add a channel mapping for ADC3 on the H7 */

	if (channel == PX4_ADC_INTERNAL_TEMP_SENSOR_CHANNEL) {
		static bool once = false;
		channel = ADC3_INTERNAL_TEMP_SENSOR_CHANNEL;
		base_address = STM32_ADC3_BASE;

		// Init it once (px4_arch_adc_init does this as well, but this is less cycles)
		if (!once) {
			once = true;
			px4_arch_adc_init(base_address);
		}
	}


	/* clear any previous EOC */

	if (rISR(base_address) & ADC_INT_EOC) {
		rISR(base_address) &= ~ADC_INT_EOC;
	}

	/* run a single conversion right now - should take about 64.5 cycles (34 microseconds) max */

	rPCSEL(base_address) |= 1 << channel;
	rSQR1(base_address) = channel << ADC_SQR1_SQ_OFFSET;
	rCR(base_address) |= ADC_CR_ADSTART;

	/* wait for the conversion to complete */
	const hrt_abstime now = hrt_absolute_time();

	while (!(rISR(base_address) & ADC_INT_EOC)) {

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
	return 1 << 16; // 16 bit ADC
}
