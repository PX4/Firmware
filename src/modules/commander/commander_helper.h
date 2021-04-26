/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file commander_helper.h
 * Commander helper functions definitions
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <julian@oes.ch>
 */

#ifndef COMMANDER_HELPER_H_
#define COMMANDER_HELPER_H_

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <drivers/drv_led.h>
#include <drivers/drv_board_led.h>


bool is_multirotor(const vehicle_status_s &current_status);
bool is_rotary_wing(const vehicle_status_s &current_status);
bool is_vtol(const vehicle_status_s &current_status);
bool is_vtol_tailsitter(const vehicle_status_s &current_status);
bool is_fixed_wing(const vehicle_status_s &current_status);
bool is_ground_rover(const vehicle_status_s &current_status);

int buzzer_init(void);
void buzzer_deinit(void);

void set_tune_override(int tune);
void set_tune(int tune);
void tune_home_set(bool use_buzzer);
void tune_mission_ok(bool use_buzzer);
void tune_mission_fail(bool use_buzzer);
void tune_positive(bool use_buzzer);
void tune_neutral(bool use_buzzer);
void tune_negative(bool use_buzzer);
void tune_failsafe(bool use_buzzer);

int blink_msg_state();

/* methods to control the onboard LED(s) */
int led_init(void);
void led_deinit(void);
int led_toggle(int led);
int led_on(int led);
int led_off(int led);

/**
 * set the LED color & mode
 * @param color @see led_control_s::COLOR_*
 * @param mode @see led_control_s::MODE_*
 */
void rgbled_set_color_and_mode(uint8_t color, uint8_t mode);

void rgbled_set_color_and_mode(uint8_t color, uint8_t mode, uint8_t blinks, uint8_t prio);

#endif /* COMMANDER_HELPER_H_ */
