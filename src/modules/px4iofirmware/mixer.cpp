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
 * @file mixer.cpp
 *
 * Control channel input/output mixer and failsafe.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include <px4_platform_common/px4_config.h>
#include <syslog.h>

#include <sys/types.h>
#include <stdbool.h>
#include <float.h>
#include <string.h>
#include <math.h>

#include <drivers/drv_pwm_output.h>
#include <drivers/drv_hrt.h>

#include <rc/sbus.h>

extern "C" {
	/* #define DEBUG */
#include "px4io.h"
}

/*
 * Maximum interval in us before FMU signal is considered lost
 */
#define FMU_INPUT_DROP_LIMIT_US		500000

/* current servo arm/disarm state */
static volatile bool mixer_servos_armed = false;
static volatile bool should_arm = false;
static volatile bool should_arm_nothrottle = false;
static volatile bool should_always_enable_pwm = false;
static volatile bool in_mixer = false;

static bool new_fmu_data = false;
static uint64_t last_fmu_update = 0;

extern int _sbus_fd;

/* selected control values and count for mixing */
enum mixer_source {
	MIX_NONE,
	MIX_DISARMED,
	MIX_FAILSAFE,
};

void
mixer_tick()
{
	/* check that we are receiving fresh data from the FMU */
	irqstate_t irq_flags = enter_critical_section();
	const hrt_abstime fmu_data_received_time = system_state.fmu_data_received_time;
	leave_critical_section(irq_flags);

	if ((fmu_data_received_time == 0) ||
	    hrt_elapsed_time(&fmu_data_received_time) > FMU_INPUT_DROP_LIMIT_US) {

		/* too long without FMU input, time to go to failsafe */
		if (r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK) {
			isr_debug(1, "AP RX timeout");
		}

		atomic_modify_clear(&r_status_flags, PX4IO_P_STATUS_FLAGS_FMU_OK);

	} else {
		atomic_modify_or(&r_status_flags, PX4IO_P_STATUS_FLAGS_FMU_OK);

		/* this flag is never cleared once OK */
		atomic_modify_or(&r_status_flags, PX4IO_P_STATUS_FLAGS_FMU_INITIALIZED);

		if (fmu_data_received_time > last_fmu_update) {
			new_fmu_data = true;
			last_fmu_update = fmu_data_received_time;
		}
	}

	/* default to disarmed mixing */
	mixer_source source = MIX_DISARMED;

	/*
	 * Decide which set of controls we're using.
	 */

	/* Do not mix if we have raw PWM and FMU is ok. */
	if ((r_status_flags & PX4IO_P_STATUS_FLAGS_RAW_PWM) &&
	    (r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK)) {

		/* don't actually mix anything - copy values from r_page_direct_pwm */
		source = MIX_NONE;
		memcpy(r_page_servos, r_page_direct_pwm, sizeof(uint16_t)*PX4IO_SERVO_COUNT);
	}

	/*
	 * Decide whether the servos should be armed right now.
	 *
	 * We must be armed, and we must have a PWM source; either raw from
	 * FMU or from the mixer.
	 *
	 */
	should_arm = ((r_status_flags & PX4IO_P_STATUS_FLAGS_INIT_OK)				/* IO initialised without error */
		      && (r_status_flags & PX4IO_P_STATUS_FLAGS_SAFETY_OFF)			/* and IO is armed */
		      && ((r_setup_arming & PX4IO_P_SETUP_ARMING_FMU_ARMED)		/* and FMU is armed */
			  || (r_status_flags & PX4IO_P_STATUS_FLAGS_RAW_PWM)		/* or direct PWM is set */
			 )
		     );

	should_arm_nothrottle = ((r_status_flags & PX4IO_P_STATUS_FLAGS_INIT_OK)              /* IO initialised without error */
				 && (r_status_flags & PX4IO_P_STATUS_FLAGS_SAFETY_OFF)        /* and IO is armed */
				 && ((r_setup_arming & PX4IO_P_SETUP_ARMING_FMU_PREARMED)     /* and FMU is prearmed */
				     || (r_status_flags & PX4IO_P_STATUS_FLAGS_RAW_PWM)       /* or direct PWM is set */
				    ));

	/* we enable PWM output always on the IO side if FMU is up and running
	 * as zero-outputs can be controlled by FMU by sending a 0 PWM command
	 */
	should_always_enable_pwm = ((r_status_flags & PX4IO_P_STATUS_FLAGS_INIT_OK)
				    && (r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK));

	/*
	 * Check if FMU is still alive, if not, terminate the flight
	 */
	if (REG_TO_BOOL(r_setup_flighttermination) && 			/* Flight termination is allowed */
	    (source == MIX_DISARMED) && 				/* and if we ended up not changing the default mixer */
	    should_arm && 						/* and we should be armed, so we intended to provide outputs */
	    (r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_INITIALIZED)) { 	/* and FMU is initialized */
		atomic_modify_or(&r_setup_arming, PX4IO_P_SETUP_ARMING_FORCE_FAILSAFE); /* then FMU is dead -> terminate flight */
	}

	/*
	 * Check if we should force failsafe - and do it if we have to
	 */
	if (r_setup_arming & PX4IO_P_SETUP_ARMING_FORCE_FAILSAFE) {
		source = MIX_FAILSAFE;
	}

	/*
	 * Set failsafe status flag depending on mixing source
	 */
	if (source == MIX_FAILSAFE) {
		atomic_modify_or(&r_status_flags, PX4IO_P_STATUS_FLAGS_FAILSAFE);

	} else {
		atomic_modify_clear(&r_status_flags, (PX4IO_P_STATUS_FLAGS_FAILSAFE));
	}

	/*
	 * Run the mixers.
	 */
	if (source == MIX_FAILSAFE) {
		/* copy failsafe values to the servo outputs */
		for (unsigned i = 0; i < PX4IO_SERVO_COUNT; i++) {
			r_page_servos[i] = r_page_servo_failsafe[i];
		}

	} else if (source == MIX_DISARMED) {
		/* copy disarmed values to the servo outputs */
		for (unsigned i = 0; i < PX4IO_SERVO_COUNT; i++) {
			r_page_servos[i] = r_page_servo_disarmed[i];
		}
	}

	/* set arming */
	bool needs_to_arm = (should_arm || should_arm_nothrottle || should_always_enable_pwm);

	/* lockdown means to send a valid pulse which disables the outputs */
	if (r_setup_arming & PX4IO_P_SETUP_ARMING_LOCKDOWN) {
		needs_to_arm = true;
	}

	if (needs_to_arm && !mixer_servos_armed) {
		/* need to arm, but not armed */
		up_pwm_servo_arm(true);
		mixer_servos_armed = true;
		atomic_modify_or(&r_status_flags, PX4IO_P_STATUS_FLAGS_OUTPUTS_ARMED);
		isr_debug(5, "> PWM enabled");

	} else if (!needs_to_arm && mixer_servos_armed) {
		/* armed but need to disarm */
		up_pwm_servo_arm(false);
		mixer_servos_armed = false;
		atomic_modify_clear(&r_status_flags, (PX4IO_P_STATUS_FLAGS_OUTPUTS_ARMED));
		isr_debug(5, "> PWM disabled");
	}

	if (mixer_servos_armed
	    && (should_arm || should_arm_nothrottle || (source == MIX_FAILSAFE))
	    && !(r_setup_arming & PX4IO_P_SETUP_ARMING_LOCKDOWN)) {
		/* update the servo outputs. */
		for (unsigned i = 0; i < PX4IO_SERVO_COUNT; i++) {
			up_pwm_servo_set(i, r_page_servos[i]);
		}

		/* set S.BUS1 or S.BUS2 outputs */
		if (r_setup_features & PX4IO_P_SETUP_FEATURES_SBUS2_OUT) {
			sbus2_output(_sbus_fd, r_page_servos, PX4IO_SERVO_COUNT);

		} else if (r_setup_features & PX4IO_P_SETUP_FEATURES_SBUS1_OUT) {
			sbus1_output(_sbus_fd, r_page_servos, PX4IO_SERVO_COUNT);
		}

	} else if (mixer_servos_armed && (should_always_enable_pwm || (r_setup_arming & PX4IO_P_SETUP_ARMING_LOCKDOWN))) {
		/* set the disarmed servo outputs. */
		for (unsigned i = 0; i < PX4IO_SERVO_COUNT; i++) {
			up_pwm_servo_set(i, r_page_servo_disarmed[i]);
			/* copy values into reporting register */
			r_page_servos[i] = r_page_servo_disarmed[i];
		}

		/* set S.BUS1 or S.BUS2 outputs */
		if (r_setup_features & PX4IO_P_SETUP_FEATURES_SBUS1_OUT) {
			sbus1_output(_sbus_fd, r_page_servo_disarmed, PX4IO_SERVO_COUNT);
		}

		if (r_setup_features & PX4IO_P_SETUP_FEATURES_SBUS2_OUT) {
			sbus2_output(_sbus_fd, r_page_servo_disarmed, PX4IO_SERVO_COUNT);
		}
	}
}
