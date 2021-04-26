/****************************************************************************
 *
 *   Copyright (c) 2015-2021 PX4 Development Team. All rights reserved.
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
 * @file camera_trigger_params.c
 * Camera trigger parameters
 *
 * @author Mohammed Kabir <kabir@uasys.io>
 * @author Andreas Bircher <andreas@wingtra.com>
 * @author Lorenz Meier <lorenz@px4.io>
 */

/**
* Camera trigger Interface
*
* Selects the trigger interface
*
* @value 1 GPIO
* @value 2 Seagull MAP2 (over PWM)
* @value 3 MAVLink (forward via MAV_CMD_IMAGE_START_CAPTURE)
* @value 4 Generic PWM (IR trigger, servo)
*
* @reboot_required true
* @group Camera trigger
*/
PARAM_DEFINE_INT32(TRIG_INTERFACE, 4);

/**
 * Camera trigger interval
 *
 * This parameter sets the time between two consecutive trigger events
 *
 * @unit ms
 * @min 4.0
 * @max 10000.0
 * @decimal 1
 * @reboot_required true
 * @group Camera trigger
 */
PARAM_DEFINE_FLOAT(TRIG_INTERVAL, 40.0f);

/**
 * Minimum camera trigger interval
 *
 * This parameter sets the minimum time between two consecutive trigger events
 * the specific camera setup is supporting.
 *
 * @unit ms
 * @min 1.0
 * @max 10000.0
 * @decimal 1
 * @reboot_required true
 * @group Camera trigger
 */
PARAM_DEFINE_FLOAT(TRIG_MIN_INTERVA, 1.0f);

/**
 * Camera trigger polarity
 *
 * This parameter sets the polarity of the trigger (0 = active low, 1 = active high )
 *
 * @value 0 Active low
 * @value 1 Active high
 * @min 0
 * @max 1
 * @reboot_required true
 * @group Camera trigger
 */
PARAM_DEFINE_INT32(TRIG_POLARITY, 0);

/**
 * Camera trigger activation time
 *
 * This parameter sets the time the trigger needs to pulled high or low.
 *
 * @unit ms
 * @min 0.1
 * @max 3000
 * @decimal 1
 * @reboot_required true
 * @group Camera trigger
 */
PARAM_DEFINE_FLOAT(TRIG_ACT_TIME, 40.0f);

/**
 * Camera trigger mode
 *
 * @value 0 Disable
 * @value 1 Time based, on command
 * @value 2 Time based, always on
 * @value 3 Distance based, always on
 * @value 4 Distance based, on command (Survey mode)
 * @min 0
 * @max 4
 * @reboot_required true
 * @group Camera trigger
 */
PARAM_DEFINE_INT32(TRIG_MODE, 0);

/**
 * Camera trigger pin
 *
 * Selects which FMU pin is used (range: AUX1-AUX8 on Pixhawk controllers with an I/O board,
 * MAIN1-MAIN8 on controllers without an I/O board. The PWM interface takes two pins per camera, while relay
 * triggers on every pin individually. Example: Value 56 would trigger on pins 5 and 6.
 * For GPIO mode Pin 6 will be triggered followed by 5. With a value of 65 pin 5 will
 * be triggered followed by 6. Pins may be non contiguous. I.E. 16 or 61.
 * In GPIO mode the delay pin to pin is < .2 uS.
 *
 * Note: only with a value of 56 or 78 it is possible to use the lower pins for
 * actuator outputs (e.g. ESC's).
 *
 * @min 1
 * @max 12345678
 * @decimal 0
 * @reboot_required true
 * @group Camera trigger
 */
PARAM_DEFINE_INT32(TRIG_PINS, 56);

/**
 * Camera trigger distance
 *
 * Sets the distance at which to trigger the camera.
 *
 * @unit m
 * @min 0
 * @increment 1
 * @decimal 1
 * @reboot_required true
 * @group Camera trigger
 */
PARAM_DEFINE_FLOAT(TRIG_DISTANCE, 25.0f);

/**
 * PWM output to trigger shot.
 *
 * @min 1000
 * @max 2000
 * @unit us
 * @group Camera trigger
 * @reboot_required true
 */
PARAM_DEFINE_INT32(TRIG_PWM_SHOOT, 1900);


/**
 * PWM neutral output on trigger pin.
 *
 * @min 1000
 * @max 2000
 * @unit us
 * @group Camera trigger
 * @reboot_required true
 */
PARAM_DEFINE_INT32(TRIG_PWM_NEUTRAL, 1500);

