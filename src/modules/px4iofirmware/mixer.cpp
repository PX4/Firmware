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

#include <lib/mixer/MixerGroup.hpp>
#include <output_limit/output_limit.h>
#include <rc/sbus.h>

#include <uORB/topics/actuator_controls.h>

#include "mixer.h"

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
	MIX_FMU,
	MIX_OVERRIDE,
	MIX_FAILSAFE,
	MIX_OVERRIDE_FMU_OK
};

static volatile mixer_source source;

static int mixer_callback(uintptr_t handle, uint8_t control_group, uint8_t control_index, float &control);
static int mixer_mix_threadsafe(float *outputs, volatile uint16_t *limits);

static MixerGroup mixer_group;

int mixer_mix_threadsafe(float *outputs, volatile uint16_t *limits)
{
	/* poor mans mutex */
	if ((r_status_flags & PX4IO_P_STATUS_FLAGS_MIXER_OK) == 0) {
		return 0;
	}

	in_mixer = true;
	int mixcount = mixer_group.mix(&outputs[0], PX4IO_SERVO_COUNT);
	*limits = mixer_group.get_saturation_status();
	in_mixer = false;

	return mixcount;
}

void
mixer_tick()
{
	/* check if the mixer got modified */
	mixer_handle_text_create_mixer();

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

		atomic_modify_clear(&r_status_flags, (PX4IO_P_STATUS_FLAGS_FMU_OK));
		atomic_modify_or(&r_status_alarms, PX4IO_P_STATUS_ALARMS_FMU_LOST);

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
	source = MIX_DISARMED;

	/*
	 * Decide which set of controls we're using.
	 */

	/* Do not mix if we have raw PWM and FMU is ok. */
	if ((r_status_flags & PX4IO_P_STATUS_FLAGS_RAW_PWM) &&
	    (r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK)) {

		if ((r_status_flags & PX4IO_P_STATUS_FLAGS_OVERRIDE) > 0) {
			/* a channel based override has been
			 * triggered, with FMU active */
			source = MIX_OVERRIDE_FMU_OK;

		} else {
			/* don't actually mix anything - copy values from r_page_direct_pwm */
			source = MIX_NONE;
			memcpy(r_page_servos, r_page_direct_pwm, sizeof(uint16_t)*PX4IO_SERVO_COUNT);
		}

	} else {

		if (!(r_status_flags & PX4IO_P_STATUS_FLAGS_OVERRIDE) &&
		    (r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK) &&
		    (r_status_flags & PX4IO_P_STATUS_FLAGS_MIXER_OK)) {

			/* mix from FMU controls */
			source = MIX_FMU;
		}

		else if (r_status_flags & PX4IO_P_STATUS_FLAGS_OVERRIDE) {

			if (r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK) {

				/* if allowed, mix from RC inputs directly up to available rc channels */
				source = MIX_OVERRIDE_FMU_OK;

			} else {
				/* if allowed, mix from RC inputs directly */
				source = MIX_OVERRIDE;
			}
		}
	}

	/*
	 * Decide whether the servos should be armed right now.
	 *
	 * We must be armed, and we must have a PWM source; either raw from
	 * FMU or from the mixer.
	 *
	 */
	should_arm = (
			     (r_status_flags & PX4IO_P_STATUS_FLAGS_INIT_OK)				/* IO initialised without error */
			     && (r_status_flags & PX4IO_P_STATUS_FLAGS_SAFETY_OFF)			/* and IO is armed */
			     && (
				     ((r_setup_arming & PX4IO_P_SETUP_ARMING_FMU_ARMED)		/* and FMU is armed */
				      && (r_status_flags & PX4IO_P_STATUS_FLAGS_MIXER_OK))	/* and there is valid input via or mixer */
				     || (r_status_flags & PX4IO_P_STATUS_FLAGS_RAW_PWM)		/* or direct PWM is set */
			     )
		     );

	should_arm_nothrottle = (
					(r_status_flags & PX4IO_P_STATUS_FLAGS_INIT_OK)                /* IO initialised without error */
					&& (r_status_flags & PX4IO_P_STATUS_FLAGS_SAFETY_OFF)        /* and IO is armed */
					&& (((r_setup_arming & PX4IO_P_SETUP_ARMING_FMU_PREARMED)    /* and FMU is prearmed */
					     && (r_status_flags & PX4IO_P_STATUS_FLAGS_MIXER_OK))     /* and there is valid input via or mixer */
					    || (r_status_flags & PX4IO_P_STATUS_FLAGS_RAW_PWM)        /* or direct PWM is set */
					   )
				);

	should_always_enable_pwm = (
					   (r_setup_arming & PX4IO_P_SETUP_ARMING_ALWAYS_PWM_ENABLE)
					   && (r_status_flags & PX4IO_P_STATUS_FLAGS_INIT_OK)
					   && (r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK)
				   );

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
	 * Set simple mixer trim values. If the OK flag is set the mixer is fully loaded.
	 */
	if (update_trims && r_status_flags & PX4IO_P_STATUS_FLAGS_MIXER_OK) {
		update_trims = false;
		mixer_group.set_trims(r_page_servo_control_trim, PX4IO_SERVO_COUNT);
	}

	/*
	 * Update air-mode parameter
	 */
	mixer_group.set_airmode((Mixer::Airmode)REG_TO_SIGNED(r_setup_airmode));


	/*
	 * Run the mixers.
	 */
	if (source == MIX_FAILSAFE) {

		/* copy failsafe values to the servo outputs */
		for (unsigned i = 0; i < PX4IO_SERVO_COUNT; i++) {
			r_page_servos[i] = r_page_servo_failsafe[i];

			/* safe actuators for FMU feedback */
			r_page_actuators[i] = FLOAT_TO_REG((r_page_servos[i] - 1500) / 600.0f);
		}

	} else if (source == MIX_DISARMED) {

		/* copy disarmed values to the servo outputs */
		for (unsigned i = 0; i < PX4IO_SERVO_COUNT; i++) {
			r_page_servos[i] = r_page_servo_disarmed[i];

			/* safe actuators for FMU feedback */
			r_page_actuators[i] = FLOAT_TO_REG((r_page_servos[i] - 1500) / 600.0f);
		}

	} else if (source != MIX_NONE && (r_status_flags & PX4IO_P_STATUS_FLAGS_MIXER_OK)
		   && !(r_setup_arming & PX4IO_P_SETUP_ARMING_LOCKDOWN)) {

		float	outputs[PX4IO_SERVO_COUNT];
		unsigned mixed;

		if (REG_TO_FLOAT(r_setup_slew_max) > FLT_EPSILON) {
			/*  maximum value the outputs of the multirotor mixer are allowed to change in this cycle
			 * factor 2 is needed because actuator outputs are in the range [-1,1]
			 */
			float delta_out_max = 2.0f * 1000.0f * dt / (r_page_servo_control_max[0] - r_page_servo_control_min[0]) / REG_TO_FLOAT(
						      r_setup_slew_max);
			mixer_group.set_max_delta_out_once(delta_out_max);
		}

		/*  set dt to be used in simple mixer for slew rate limiting */
		mixer_group.set_dt_once(dt);

		/* update parameter for mc thrust model if it updated */
		if (update_mc_thrust_param) {
			mixer_group.set_thrust_factor(REG_TO_FLOAT(r_setup_thr_fac));
			update_mc_thrust_param = false;
		}

		/* mix */
		mixed = mixer_mix_threadsafe(&outputs[0], &r_mixer_limits);

		/* the pwm limit call takes care of out of band errors */
		output_limit_calc(should_arm, should_arm_nothrottle, mixed, r_setup_pwm_reverse, r_page_servo_disarmed,
				  r_page_servo_control_min, r_page_servo_control_max, outputs, r_page_servos, &pwm_limit);

		/* clamp unused outputs to zero */
		for (unsigned i = mixed; i < PX4IO_SERVO_COUNT; i++) {
			r_page_servos[i] = 0;
			outputs[i] = 0.0f;
		}

		/* store normalized outputs */
		for (unsigned i = 0; i < PX4IO_SERVO_COUNT; i++) {
			r_page_actuators[i] = FLOAT_TO_REG(outputs[i]);
		}


		if (mixed  && new_fmu_data) {
			new_fmu_data = false;

			/* Trigger all timer's channels in Oneshot mode to fire
			 * the oneshots with updated values.
			 */

			up_pwm_update();
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

	} else if (mixer_servos_armed && (should_always_enable_pwm
					  || (r_setup_arming & PX4IO_P_SETUP_ARMING_LOCKDOWN))) {
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

static int
mixer_callback(uintptr_t handle,
	       uint8_t control_group,
	       uint8_t control_index,
	       float &control)
{
	control = 0.0f;

	if (control_group >= PX4IO_CONTROL_GROUPS) {
		return -1;
	}

	switch (source) {
	case MIX_FMU:
		if (control_index < PX4IO_CONTROL_CHANNELS && control_group < PX4IO_CONTROL_GROUPS) {
			if (r_page_controls[CONTROL_PAGE_INDEX(control_group, control_index)] == INT16_MAX) {
				//catch NAN values encoded as INT16 max for disarmed outputs
				control = NAN;

			} else {
				control = REG_TO_FLOAT(r_page_controls[CONTROL_PAGE_INDEX(control_group, control_index)]);
			}

			break;
		}

		return -1;

	case MIX_OVERRIDE:
		if (r_page_rc_input[PX4IO_P_RC_VALID] & (1 << CONTROL_PAGE_INDEX(control_group, control_index))) {
			control = REG_TO_FLOAT(r_page_rc_input[PX4IO_P_RC_BASE + control_index]);
			break;
		}

		return -1;

	case MIX_OVERRIDE_FMU_OK:

		/* FMU is ok but we are in override mode, use direct rc control for the available rc channels. The remaining channels are still controlled by the fmu */
		if (r_page_rc_input[PX4IO_P_RC_VALID] & (1 << CONTROL_PAGE_INDEX(control_group, control_index))) {
			control = REG_TO_FLOAT(r_page_rc_input[PX4IO_P_RC_BASE + control_index]);
			break;

		} else if (control_index < PX4IO_CONTROL_CHANNELS && control_group < PX4IO_CONTROL_GROUPS) {
			control = REG_TO_FLOAT(r_page_controls[CONTROL_PAGE_INDEX(control_group, control_index)]);
			break;
		}

		return -1;

	case MIX_DISARMED:
	case MIX_FAILSAFE:
	case MIX_NONE:
		control = 0.0f;
		return -1;
	}

	/* apply trim offsets for override channels */
	if (source == MIX_OVERRIDE || source == MIX_OVERRIDE_FMU_OK) {
		if (control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE &&
		    control_index == actuator_controls_s::INDEX_ROLL) {
			control *= REG_TO_FLOAT(r_setup_scale_roll);
			control += REG_TO_FLOAT(r_setup_trim_roll);

		} else if (control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE &&
			   control_index == actuator_controls_s::INDEX_PITCH) {
			control *= REG_TO_FLOAT(r_setup_scale_pitch);
			control += REG_TO_FLOAT(r_setup_trim_pitch);

		} else if (control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE &&
			   control_index == actuator_controls_s::INDEX_YAW) {
			control *= REG_TO_FLOAT(r_setup_scale_yaw);
			control += REG_TO_FLOAT(r_setup_trim_yaw);
		}
	}

	/* limit output */
	if (control > 1.0f) {
		control = 1.0f;

	} else if (control < -1.0f) {
		control = -1.0f;
	}

	/* motor spinup phase - lock throttle to zero */
	if ((pwm_limit.state == OUTPUT_LIMIT_STATE_RAMP) || (should_arm_nothrottle && !should_arm)) {
		if ((control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE ||
		     control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE_ALTERNATE) &&
		    control_index == actuator_controls_s::INDEX_THROTTLE) {
			/* limit the throttle output to zero during motor spinup,
			 * as the motors cannot follow any demand yet
			 */
			control = 0.0f;
		}
	}

	/* only safety off, but not armed - set throttle as invalid */
	if (should_arm_nothrottle && !should_arm) {
		if ((control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE ||
		     control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE_ALTERNATE) &&
		    control_index == actuator_controls_s::INDEX_THROTTLE) {
			/* mark the throttle as invalid */
			control = NAN;
		}
	}

	return 0;
}

/*
 * XXX error handling here should be more aggressive; currently it is
 * possible to get STATUS_FLAGS_MIXER_OK set even though the mixer has
 * not loaded faithfully.
 */

static char mixer_text[PX4IO_MAX_MIXER_LENGTH];		/* large enough for one mixer */
static unsigned mixer_text_length = 0;
static bool mixer_update_pending = false;

int
mixer_handle_text_create_mixer()
{
	/* only run on update */
	if (!mixer_update_pending) {
		return 0;
	}

	/* do not allow a mixer change while safety off and FMU armed */
	if ((r_status_flags & PX4IO_P_STATUS_FLAGS_SAFETY_OFF) &&
	    (r_setup_arming & PX4IO_P_SETUP_ARMING_FMU_ARMED)) {
		return 1;
	}

	/* abort if we're in the mixer - it will be tried again in the next iteration */
	if (in_mixer) {
		return 1;
	}

	/* process the text buffer, adding new mixers as their descriptions can be parsed */
	unsigned resid = mixer_text_length;
	mixer_group.load_from_buf(mixer_callback, 0, &mixer_text[0], resid);

	/* if anything was parsed */
	if (resid != mixer_text_length) {

		isr_debug(2, "used %u", mixer_text_length - resid);

		/* copy any leftover text to the base of the buffer for re-use */
		if (resid > 0) {
			memmove(&mixer_text[0], &mixer_text[mixer_text_length - resid], resid);
			/* enforce null termination */
			mixer_text[resid] = '\0';
		}

		mixer_text_length = resid;
	}

	mixer_update_pending = false;

	return 0;
}

int
mixer_handle_text(const void *buffer, size_t length)
{
	/* do not allow a mixer change while safety off and FMU armed */
	if ((r_status_flags & PX4IO_P_STATUS_FLAGS_SAFETY_OFF) &&
	    (r_setup_arming & PX4IO_P_SETUP_ARMING_FMU_ARMED)) {
		return 1;
	}

	/* disable mixing, will be enabled once load is complete */
	atomic_modify_clear(&r_status_flags, PX4IO_P_STATUS_FLAGS_MIXER_OK);

	/* set the update flags to dirty so we reload those values after a mixer change */
	update_trims = true;
	update_mc_thrust_param = true;

	/* abort if we're in the mixer - the caller is expected to retry */
	if (in_mixer) {
		return 1;
	}

	px4io_mixdata	*msg = (px4io_mixdata *)buffer;

	isr_debug(2, "mix txt %u", length);

	if (length < sizeof(px4io_mixdata)) {
		return 0;
	}

	unsigned text_length = length - sizeof(px4io_mixdata);

	switch (msg->action) {
	case F2I_MIXER_ACTION_RESET:
		isr_debug(2, "reset");

		/* THEN actually delete it */
		mixer_group.reset();
		mixer_text_length = 0;

	/* FALLTHROUGH */
	case F2I_MIXER_ACTION_APPEND:
		isr_debug(2, "append %d", length);

		/* check for overflow - this would be really fatal */
		if ((mixer_text_length + text_length + 1) > sizeof(mixer_text)) {
			atomic_modify_clear(&r_status_flags, PX4IO_P_STATUS_FLAGS_MIXER_OK);
			return 0;
		}

		/* check if the last item has been processed - bail out if not */
		if (mixer_update_pending) {
			return 1;
		}

		/* append mixer text and nul-terminate, guard against overflow */
		memcpy(&mixer_text[mixer_text_length], msg->text, text_length);
		mixer_text_length += text_length;
		mixer_text[mixer_text_length] = '\0';
		isr_debug(2, "buflen %u", mixer_text_length);

		/* flag the buffer as ready */
		mixer_update_pending = true;

		break;
	}

	return 0;
}

void
mixer_set_failsafe()
{
	/*
	 * Check if a custom failsafe value has been written,
	 * or if the mixer is not ok and bail out.
	 */

	if ((r_setup_arming & PX4IO_P_SETUP_ARMING_FAILSAFE_CUSTOM) ||
	    !(r_status_flags & PX4IO_P_STATUS_FLAGS_MIXER_OK)) {
		return;
	}

	/* set failsafe defaults to the values for all inputs = 0 */
	float	outputs[PX4IO_SERVO_COUNT];
	unsigned mixed;

	if (REG_TO_FLOAT(r_setup_slew_max) > FLT_EPSILON) {
		/* maximum value the outputs of the multirotor mixer are allowed to change in this cycle
		 * factor 2 is needed because actuator outputs are in the range [-1,1]
		 */
		float delta_out_max = 2.0f * 1000.0f * dt / (r_page_servo_control_max[0] - r_page_servo_control_min[0]) / REG_TO_FLOAT(
					      r_setup_slew_max);
		mixer_group.set_max_delta_out_once(delta_out_max);
	}

	/*  set dt to be used in simple mixer for slew rate limiting */
	mixer_group.set_dt_once(dt);

	/* update parameter for mc thrust model if it updated */
	if (update_mc_thrust_param) {
		mixer_group.set_thrust_factor(REG_TO_FLOAT(r_setup_thr_fac));
		update_mc_thrust_param = false;
	}

	/* mix */
	mixed = mixer_mix_threadsafe(&outputs[0], &r_mixer_limits);

	/* scale to PWM and update the servo outputs as required */
	for (unsigned i = 0; i < mixed; i++) {

		/* scale to servo output */
		r_page_servo_failsafe[i] = (outputs[i] * 600.0f) + 1500;

	}

	/* disable the rest of the outputs */
	for (unsigned i = mixed; i < PX4IO_SERVO_COUNT; i++) {
		r_page_servo_failsafe[i] = 0;
	}

}
