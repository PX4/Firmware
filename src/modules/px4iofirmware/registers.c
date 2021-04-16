/****************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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
 * @file registers.c
 *
 * Implementation of the PX4IO register space.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include <px4_platform_common/px4_config.h>

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>
#include <stm32_pwr.h>
#include <rc/dsm.h>
#include <rc/sbus.h>

#include "px4io.h"
#include "protocol.h"

static int	registers_set_one(uint8_t page, uint8_t offset, uint16_t value);
static void	pwm_configure_rates(uint16_t map, uint16_t defaultrate, uint16_t altrate);

bool update_mc_thrust_param;
bool update_trims;

/**
 * PAGE 0
 *
 * Static configuration parameters.
 */
static const uint16_t	r_page_config[] = {
	[PX4IO_P_CONFIG_PROTOCOL_VERSION]	= PX4IO_PROTOCOL_VERSION,
	[PX4IO_P_CONFIG_HARDWARE_VERSION]	= 2,
	[PX4IO_P_CONFIG_BOOTLOADER_VERSION]	= PX4IO_BL_VERSION,
	[PX4IO_P_CONFIG_MAX_TRANSFER]		= PX4IO_MAX_TRANSFER_LEN,
	[PX4IO_P_CONFIG_CONTROL_COUNT]		= PX4IO_CONTROL_CHANNELS,
	[PX4IO_P_CONFIG_ACTUATOR_COUNT]		= PX4IO_SERVO_COUNT,
	[PX4IO_P_CONFIG_RC_INPUT_COUNT]		= PX4IO_RC_INPUT_CHANNELS,
	[PX4IO_P_CONFIG_ADC_INPUT_COUNT]	= PX4IO_ADC_CHANNEL_COUNT,
};

/**
 * PAGE 1
 *
 * Status values.
 */
volatile uint16_t	r_page_status[] = {
	[PX4IO_P_STATUS_FREEMEM]		= 0,
	[PX4IO_P_STATUS_CPULOAD]		= 0,
	[PX4IO_P_STATUS_FLAGS]			= 0,
	[PX4IO_P_STATUS_ALARMS]			= 0,
	[PX4IO_P_STATUS_VSERVO]			= 0,
	[PX4IO_P_STATUS_VRSSI]			= 0,
};

/**
 * PAGE 3
 *
 * Servo PWM values
 */
uint16_t		r_page_servos[PX4IO_SERVO_COUNT];

/**
 * PAGE 4
 *
 * Raw RC input
 */
uint16_t		r_page_raw_rc_input[] = {
	[PX4IO_P_RAW_RC_COUNT]			= 0,
	[PX4IO_P_RAW_RC_FLAGS]			= 0,
	[PX4IO_P_RAW_RC_NRSSI]			= 0,
	[PX4IO_P_RAW_RC_DATA]			= 0,
	[PX4IO_P_RAW_FRAME_COUNT]		= 0,
	[PX4IO_P_RAW_LOST_FRAME_COUNT]		= 0,
	[PX4IO_P_RAW_RC_BASE ...(PX4IO_P_RAW_RC_BASE + PX4IO_RC_INPUT_CHANNELS)] = 0
};

/**
 * Scratch page; used for registers that are constructed as-read.
 *
 * PAGE 6 Raw ADC input.
 * PAGE 7 PWM rate maps.
 */
uint16_t		r_page_scratch[32];

/**
 * PAGE 8
 *
 * RAW PWM values
 */
uint16_t		r_page_direct_pwm[PX4IO_SERVO_COUNT];

/**
 * PAGE 100
 *
 * Setup registers
 */
volatile uint16_t	r_page_setup[] = {
	/* default to RSSI ADC functionality */
	[PX4IO_P_SETUP_FEATURES]		= PX4IO_P_SETUP_FEATURES_ADC_RSSI,
	[PX4IO_P_SETUP_ARMING]			= 0,
	[PX4IO_P_SETUP_PWM_RATES]		= 0,
	[PX4IO_P_SETUP_PWM_DEFAULTRATE]		= 50,
	[PX4IO_P_SETUP_PWM_ALTRATE]		= 200,
	[PX4IO_P_SETUP_SBUS_RATE]		= 72,
	[PX4IO_P_SETUP_VSERVO_SCALE]		= 10000,
	[PX4IO_P_SETUP_SET_DEBUG]		= 0,
	[PX4IO_P_SETUP_REBOOT_BL]		= 0,
	[PX4IO_P_SETUP_CRC ...(PX4IO_P_SETUP_CRC + 1)] = 0,
	[PX4IO_P_SETUP_THERMAL] = PX4IO_THERMAL_IGNORE,
	[PX4IO_P_SETUP_ENABLE_FLIGHTTERMINATION] = 0
};

#define PX4IO_P_SETUP_FEATURES_VALID	(PX4IO_P_SETUP_FEATURES_SBUS1_OUT | PX4IO_P_SETUP_FEATURES_SBUS2_OUT | PX4IO_P_SETUP_FEATURES_ADC_RSSI)

#define PX4IO_P_SETUP_ARMING_VALID	(PX4IO_P_SETUP_ARMING_FMU_ARMED | \
		PX4IO_P_SETUP_ARMING_FMU_PREARMED | \
		PX4IO_P_SETUP_ARMING_IO_ARM_OK | \
		PX4IO_P_SETUP_ARMING_FAILSAFE_CUSTOM | \
		PX4IO_P_SETUP_ARMING_LOCKDOWN | \
		PX4IO_P_SETUP_ARMING_FORCE_FAILSAFE | \
		PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE)
#define PX4IO_P_SETUP_RATES_VALID	((1 << PX4IO_SERVO_COUNT) - 1)

/**
 * PAGE 103
 *
 * R/C channel input configuration.
 */
uint16_t		r_page_rc_input_config[PX4IO_RC_INPUT_CHANNELS * PX4IO_P_RC_CONFIG_STRIDE];

/* valid options */
#define PX4IO_P_RC_CONFIG_OPTIONS_VALID	(PX4IO_P_RC_CONFIG_OPTIONS_REVERSE | PX4IO_P_RC_CONFIG_OPTIONS_ENABLED)

/*
 * PAGE 104 uses r_page_servos.
 */

/**
 * PAGE 105
 *
 * Failsafe servo PWM values
 *
 * Disable pulses as default.
 */
uint16_t		r_page_servo_failsafe[PX4IO_SERVO_COUNT] = { 0, 0, 0, 0, 0, 0, 0, 0 };

/**
 * PAGE 109
 *
 * disarmed PWM values for difficult ESCs
 *
 */
uint16_t		r_page_servo_disarmed[PX4IO_SERVO_COUNT] = { 0, 0, 0, 0, 0, 0, 0, 0 };

int
registers_set(uint8_t page, uint8_t offset, const uint16_t *values, unsigned num_values)
{
	switch (page) {
	/* handle raw PWM input */
	case PX4IO_PAGE_DIRECT_PWM:

		/* copy channel data */
		while ((offset < PX4IO_CONTROL_CHANNELS) && (num_values > 0)) {

			/* XXX range-check value? */
			if (*values != PWM_IGNORE_THIS_CHANNEL) {
				r_page_direct_pwm[offset] = *values;
			}

			offset++;
			num_values--;
			values++;
		}

		system_state.fmu_data_received_time = hrt_absolute_time();
		r_status_flags |= PX4IO_P_STATUS_FLAGS_RAW_PWM;

		/* Trigger all timer's channels in Oneshot mode to fire
		 * the oneshots with updated values.
		 */
		up_pwm_update();

		break;

	/* handle setup for servo failsafe values */
	case PX4IO_PAGE_FAILSAFE_PWM:

		/* copy channel data */
		while ((offset < PX4IO_SERVO_COUNT) && (num_values > 0)) {

			if (*values == 0) {
				/* ignore 0 */
			} else if (*values < PWM_LOWEST_MIN) {
				r_page_servo_failsafe[offset] = PWM_LOWEST_MIN;

			} else if (*values > PWM_HIGHEST_MAX) {
				r_page_servo_failsafe[offset] = PWM_HIGHEST_MAX;

			} else {
				r_page_servo_failsafe[offset] = *values;
			}

			/* flag the failsafe values as custom */
			r_setup_arming |= PX4IO_P_SETUP_ARMING_FAILSAFE_CUSTOM;

			offset++;
			num_values--;
			values++;
		}

		break;

	case PX4IO_PAGE_DISARMED_PWM: {
			/* copy channel data */
			while ((offset < PX4IO_SERVO_COUNT) && (num_values > 0)) {
				if (*values == 0) {
					/* 0 means disabling always PWM */
					r_page_servo_disarmed[offset] = 0;

				} else if (*values < PWM_LOWEST_MIN) {
					r_page_servo_disarmed[offset] = PWM_LOWEST_MIN;

				} else if (*values > PWM_HIGHEST_MAX) {
					r_page_servo_disarmed[offset] = PWM_HIGHEST_MAX;

				} else {
					r_page_servo_disarmed[offset] = *values;
				}

				offset++;
				num_values--;
				values++;
			}
		}
		break;

	default:

		/* avoid offset wrap */
		if ((offset + num_values) > 255) {
			num_values = 255 - offset;
		}

		/* iterate individual registers, set each in turn */
		while (num_values--) {
			if (registers_set_one(page, offset, *values)) {
				return -1;
			}

			offset++;
			values++;
		}

		break;
	}

	return 0;
}

static int
registers_set_one(uint8_t page, uint8_t offset, uint16_t value)
{
	switch (page) {

	case PX4IO_PAGE_STATUS:
		switch (offset) {
		case PX4IO_P_STATUS_ALARMS:
			/* clear bits being written */
			r_status_alarms &= ~value;
			break;

		case PX4IO_P_STATUS_FLAGS:

			/*
			 * Allow FMU override of arming state (to allow in-air restores),
			 * but only if the arming state is not in sync on the IO side.
			 */
			if (!(r_status_flags & PX4IO_P_STATUS_FLAGS_ARM_SYNC)) {
				r_status_flags = value;

			}

			break;

		default:
			/* just ignore writes to other registers in this page */
			break;
		}

		break;

	case PX4IO_PAGE_SETUP:
		switch (offset) {
		case PX4IO_P_SETUP_FEATURES:

			value &= PX4IO_P_SETUP_FEATURES_VALID;

			/* some of the options conflict - give S.BUS out precedence, then ADC RSSI, then PWM RSSI */

			/* switch S.Bus output pin as needed */
#ifdef ENABLE_SBUS_OUT
			ENABLE_SBUS_OUT(value & (PX4IO_P_SETUP_FEATURES_SBUS1_OUT | PX4IO_P_SETUP_FEATURES_SBUS2_OUT));

			/* disable the conflicting options with SBUS 1 */
			if (value & (PX4IO_P_SETUP_FEATURES_SBUS1_OUT)) {
				value &= ~(PX4IO_P_SETUP_FEATURES_ADC_RSSI | PX4IO_P_SETUP_FEATURES_SBUS2_OUT);
			}

			/* disable the conflicting options with SBUS 2 */
			if (value & (PX4IO_P_SETUP_FEATURES_SBUS2_OUT)) {
				value &= ~(PX4IO_P_SETUP_FEATURES_ADC_RSSI | PX4IO_P_SETUP_FEATURES_SBUS1_OUT);
			}

#endif

			/* disable the conflicting options with ADC RSSI */
			if (value & (PX4IO_P_SETUP_FEATURES_ADC_RSSI)) {
				value &= ~(PX4IO_P_SETUP_FEATURES_SBUS1_OUT | PX4IO_P_SETUP_FEATURES_SBUS2_OUT);
			}

			/* apply changes */
			r_setup_features = value;

			break;

		case PX4IO_P_SETUP_ARMING:

			value &= PX4IO_P_SETUP_ARMING_VALID;

			/*
			 * If the failsafe termination flag is set, do not allow the autopilot to unset it
			 */
			value |= (r_setup_arming & PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE);

			/*
			 * If failsafe termination is enabled and force failsafe bit is set, do not allow
			 * the autopilot to clear it.
			 */
			if (r_setup_arming & PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE) {
				value |= (r_setup_arming & PX4IO_P_SETUP_ARMING_FORCE_FAILSAFE);
			}

			r_setup_arming = value;

			break;

		case PX4IO_P_SETUP_PWM_RATES:
			value &= PX4IO_P_SETUP_RATES_VALID;
			pwm_configure_rates(value, r_setup_pwm_defaultrate, r_setup_pwm_altrate);
			break;

		case PX4IO_P_SETUP_PWM_DEFAULTRATE:
			if (value < 25) {
				value = 25;
			}

			if (value > 400) {
				value = 400;
			}

			pwm_configure_rates(r_setup_pwm_rates, value, r_setup_pwm_altrate);
			break;

		case PX4IO_P_SETUP_PWM_ALTRATE:

			/* For PWM constrain to [25,400]Hz
			 * For Oneshot there is no rate, 0 is therefore used to select Oneshot mode
			 */
			if (value != 0) {
				if (value < 25) {
					value = 25;
				}

				if (value > 400) {
					value = 400;
				}
			}

			pwm_configure_rates(r_setup_pwm_rates, r_setup_pwm_defaultrate, value);
			break;

		case PX4IO_P_SETUP_SET_DEBUG:
			r_page_setup[PX4IO_P_SETUP_SET_DEBUG] = value;
			isr_debug(0, "set debug %u\n", (unsigned)r_page_setup[PX4IO_P_SETUP_SET_DEBUG]);
			break;

		case PX4IO_P_SETUP_REBOOT_BL:
			if (r_status_flags & PX4IO_P_STATUS_FLAGS_SAFETY_OFF) {
				// don't allow reboot while armed
				break;
			}

			// check the magic value
			if (value != PX4IO_REBOOT_BL_MAGIC) {
				break;
			}

			// we schedule a reboot rather than rebooting
			// immediately to allow the IO board to ACK
			// the reboot command
			schedule_reboot(100000);
			break;

		case PX4IO_P_SETUP_DSM:
			dsm_bind(value & 0x0f, (value >> 4) & 0xF);
			break;

		case PX4IO_P_SETUP_FORCE_SAFETY_ON:
			if (value == PX4IO_FORCE_SAFETY_MAGIC) {
				r_status_flags &= ~PX4IO_P_STATUS_FLAGS_SAFETY_OFF;

			} else {
				return -1;
			}

			break;

		case PX4IO_P_SETUP_FORCE_SAFETY_OFF:
			if (value == PX4IO_FORCE_SAFETY_MAGIC) {
				r_status_flags |= PX4IO_P_STATUS_FLAGS_SAFETY_OFF;

			} else {
				return -1;
			}

			break;

		case PX4IO_P_SETUP_SBUS_RATE:
			r_page_setup[offset] = value;
			sbus1_set_output_rate_hz(value);
			break;

		case PX4IO_P_SETUP_THERMAL:
		case PX4IO_P_SETUP_ENABLE_FLIGHTTERMINATION:
			r_page_setup[offset] = value;
			break;

		default:
			return -1;
		}

		break;

	case PX4IO_PAGE_RC_CONFIG: {

			/**
			 * do not allow a RC config change while safety is off
			 */
			if (r_status_flags & PX4IO_P_STATUS_FLAGS_SAFETY_OFF) {
				break;
			}

			unsigned channel = offset / PX4IO_P_RC_CONFIG_STRIDE;
			unsigned index = offset - channel * PX4IO_P_RC_CONFIG_STRIDE;
			uint16_t *conf = &r_page_rc_input_config[channel * PX4IO_P_RC_CONFIG_STRIDE];

			if (channel >= PX4IO_RC_INPUT_CHANNELS) {
				return -1;
			}

			/* disable the channel until we have a chance to sanity-check it */
			conf[PX4IO_P_RC_CONFIG_OPTIONS] &= PX4IO_P_RC_CONFIG_OPTIONS_ENABLED;

			switch (index) {
			case PX4IO_P_RC_CONFIG_ASSIGNMENT:
				conf[index] = value;
				break;

			case PX4IO_P_RC_CONFIG_OPTIONS:
				value &= PX4IO_P_RC_CONFIG_OPTIONS_VALID;
				r_status_flags |= PX4IO_P_STATUS_FLAGS_INIT_OK;

				/* set all options except the enabled option */
				conf[index] = value & ~PX4IO_P_RC_CONFIG_OPTIONS_ENABLED;

				/* should the channel be enabled? */
				/* this option is normally set last */
				if (value & PX4IO_P_RC_CONFIG_OPTIONS_ENABLED) {
					uint8_t count = 0;
					bool disabled = false;

					if (conf[PX4IO_P_RC_CONFIG_ASSIGNMENT] == UINT8_MAX) {
						disabled = true;

					} else if ((conf[PX4IO_P_RC_CONFIG_ASSIGNMENT] >= PX4IO_RC_MAPPED_CONTROL_CHANNELS) &&
						   (conf[PX4IO_P_RC_CONFIG_ASSIGNMENT] != PX4IO_P_RC_CONFIG_ASSIGNMENT_KILLSWITCH)) {
						count++;
					}

					/* sanity checks pass, enable channel */
					if (count) {
						isr_debug(0, "ERROR: %d config error(s) for RC%d.\n", count, (channel + 1));
						r_status_flags &= ~PX4IO_P_STATUS_FLAGS_INIT_OK;

					} else if (!disabled) {
						conf[index] |= PX4IO_P_RC_CONFIG_OPTIONS_ENABLED;
					}
				}

				break;
				/* inner switch: case PX4IO_P_RC_CONFIG_OPTIONS */

			}

			break;
			/* case PX4IO_RC_PAGE_CONFIG */
		}

	case PX4IO_PAGE_TEST:
		switch (offset) {
		case PX4IO_P_TEST_LED:
			LED_AMBER(value & 1);
			break;
		}

		break;

	default:
		return -1;
	}

	return 0;
}

uint8_t last_page;
uint8_t last_offset;

int
registers_get(uint8_t page, uint8_t offset, uint16_t **values, unsigned *num_values)
{
#define SELECT_PAGE(_page_name)							\
	do {									\
		*values = (uint16_t *)&_page_name[0];				\
		*num_values = sizeof(_page_name) / sizeof(_page_name[0]);	\
	} while(0)

	switch (page) {

	/*
	 * Handle pages that are updated dynamically at read time.
	 */
	case PX4IO_PAGE_STATUS:
		/* PX4IO_P_STATUS_FREEMEM */

		/* XXX PX4IO_P_STATUS_CPULOAD */

		/* PX4IO_P_STATUS_FLAGS maintained externally */

		/* PX4IO_P_STATUS_ALARMS maintained externally */

#ifdef ADC_VSERVO
		/* PX4IO_P_STATUS_VSERVO */
		{
			unsigned counts = adc_measure(ADC_VSERVO);

			if (counts != 0xffff) {
				// use 3:1 scaling on 3.3V ADC input
				unsigned mV = counts * 9900 / 4096;
				r_page_status[PX4IO_P_STATUS_VSERVO] = mV;
			}
		}

#endif
#ifdef ADC_RSSI
		/* PX4IO_P_STATUS_VRSSI */
		{
			unsigned counts = adc_measure(ADC_RSSI);

			if (counts != 0xffff) {
				// use 1:1 scaling on 3.3V ADC input
				unsigned mV = counts * 3300 / 4096;
				r_page_status[PX4IO_P_STATUS_VRSSI] = mV;
			}
		}
#endif
		/* XXX PX4IO_P_STATUS_PRSSI */

		SELECT_PAGE(r_page_status);
		break;

	case PX4IO_PAGE_RAW_ADC_INPUT:
		memset(r_page_scratch, 0, sizeof(r_page_scratch));

#ifdef ADC_VSERVO
		r_page_scratch[0] = adc_measure(ADC_VSERVO);
#endif
#ifdef ADC_RSSI
		r_page_scratch[1] = adc_measure(ADC_RSSI);
#endif
		SELECT_PAGE(r_page_scratch);
		break;

	case PX4IO_PAGE_PWM_INFO:
		memset(r_page_scratch, 0, sizeof(r_page_scratch));

		for (unsigned i = 0; i < PX4IO_SERVO_COUNT; i++) {
			r_page_scratch[PX4IO_RATE_MAP_BASE + i] = up_pwm_servo_get_rate_group(i);
		}

		SELECT_PAGE(r_page_scratch);
		break;

	/*
	 * Pages that are just a straight read of the register state.
	 */

	/* status pages */
	case PX4IO_PAGE_CONFIG:
		SELECT_PAGE(r_page_config);
		break;

	case PX4IO_PAGE_SERVOS:
		SELECT_PAGE(r_page_servos);
		break;

	case PX4IO_PAGE_RAW_RC_INPUT:
		SELECT_PAGE(r_page_raw_rc_input);
		break;

	/* readback of input pages */
	case PX4IO_PAGE_SETUP:
		SELECT_PAGE(r_page_setup);
		break;

	case PX4IO_PAGE_RC_CONFIG:
		SELECT_PAGE(r_page_rc_input_config);
		break;

	case PX4IO_PAGE_DIRECT_PWM:
		SELECT_PAGE(r_page_direct_pwm);
		break;

	case PX4IO_PAGE_FAILSAFE_PWM:
		SELECT_PAGE(r_page_servo_failsafe);
		break;

	case PX4IO_PAGE_DISARMED_PWM:
		SELECT_PAGE(r_page_servo_disarmed);
		break;

	default:
		return -1;
	}

#undef SELECT_PAGE
#undef COPY_PAGE

	last_page = page;
	last_offset = offset;

	/* if the offset is at or beyond the end of the page, we have no data */
	if (offset >= *num_values) {
		return -1;
	}

	/* correct the data pointer and count for the offset */
	*values += offset;
	*num_values -= offset;

	return 0;
}

/*
 * Helper function to handle changes to the PWM rate control registers.
 */
static void
pwm_configure_rates(uint16_t map, uint16_t defaultrate, uint16_t altrate)
{
	for (unsigned pass = 0; pass < 2; pass++) {
		for (unsigned group = 0; group < PX4IO_SERVO_COUNT; group++) {

			/* get the channel mask for this rate group */
			uint32_t mask = up_pwm_servo_get_rate_group(group);

			if (mask == 0) {
				continue;
			}

			/* all channels in the group must be either default or alt-rate */
			uint32_t alt = map & mask;

			if (pass == 0) {
				/* preflight */
				if ((alt != 0) && (alt != mask)) {
					/* not a legal map, bail with an alarm */
					r_status_alarms |= PX4IO_P_STATUS_ALARMS_PWM_ERROR;
					return;
				}

			} else {
				/* set it - errors here are unexpected */
				if (alt != 0) {
					if (up_pwm_servo_set_rate_group_update(group, r_setup_pwm_altrate) != OK) {
						r_status_alarms |= PX4IO_P_STATUS_ALARMS_PWM_ERROR;
					}

				} else {
					if (up_pwm_servo_set_rate_group_update(group, r_setup_pwm_defaultrate) != OK) {
						r_status_alarms |= PX4IO_P_STATUS_ALARMS_PWM_ERROR;
					}
				}
			}
		}
	}

	r_setup_pwm_rates = map;
	r_setup_pwm_defaultrate = defaultrate;
	r_setup_pwm_altrate = altrate;
}
