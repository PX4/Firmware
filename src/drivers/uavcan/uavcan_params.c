/****************************************************************************
 *
 *   Copyright (c) 2014-2017 PX4 Development Team. All rights reserved.
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
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <px4_config.h>
#include <parameters/param.h>

/**
 * UAVCAN mode
 *
 *  0 - UAVCAN disabled.
 *  1 - Enables support for UAVCAN sensors without dynamic node ID allocation and firmware update.
 *  2 - Enables support for UAVCAN sensors with dynamic node ID allocation and firmware update.
 *  3 - Enables support for UAVCAN sensors and actuators with dynamic node ID allocation and firmware update. Also sets the motor control outputs to UAVCAN.
 *
 * @min 0
 * @max 3
 * @value 0 Disabled
 * @value 1 Sensors Manual Config
 * @value 2 Sensors Automatic Config
 * @value 3 Sensors and Actuators (ESCs) Automatic Config
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_ENABLE, 0);

/**
 * UAVCAN Node ID.
 *
 * Read the specs at http://uavcan.org to learn more about Node ID.
 *
 * @min 1
 * @max 125
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_NODE_ID, 1);

/**
 * UAVCAN CAN bus bitrate.
 *
 * @unit bit/s
 * @min 20000
 * @max 1000000
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_BITRATE, 1000000);

/**
 * UAVCAN ESC will spin at idle throttle when armed, even if the mixer outputs zero setpoints.
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_ESC_IDLT, 1);

/******************************************************************************
*                                UAVCAN_RATE                                 *
******************************************************************************/

/**
 * Set the UAVCAN output frequency for the actuator UAVCAN message publication
 *
 * @reboot_required true
 *
 * @min 0
 * @max 200
 * @unit Hz
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_RATE, 50);

/**
 * Set the maximum deflection/throttle command value for the actuator UAVCAN messages.
 *
 * For ESC or servo depending on UAVCAN_MAP_CHx parameter
 *
 * This value is used only if the channel specific (UAVCAN_MAXx) value is not set
 *
 * Set to 1.0 for full positive range.
 *
 * @reboot_required true
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @unit normalized
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_MAX, 1.0);

/**
 * Set the minimum deflection/throttle command value for the actuator UAVCAN messages.
 *
 * For ESC or servo depending on UAVCAN_MAP_CHx parameter
 *
 * This value is used only if the channel specific (UAVCAN_MINx) value is not set
 *
 * Set to -1.0 for full negative range.
 *
 * @reboot_required true
 *
 * @min -1.0
 * @max 0.0
 * @decimal 2
 * @unit normalized
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_MIN, -1.0);

/**
 * Set the disarmed deflection/throttle command value for the actuator UAVCAN messages
 *
 * This is the deflection/throttle value that the autopilot is outputting if not armed.
 *
 * The main use of this parameter is to silence ESCs/servos when they are disarmed, and to trim control surfaces
 * if the control surfaces are not symetrical around the value set by the UAVCAN_TRIMx parameter.
 *
 * This value is used only if the channel specific (UAVCAN_DISARMEDx) value is not set
 *
 * @reboot_required true
 *
 * @min -1.0
 * @max 1.0
 * @decimal 2
 * @unit normalized
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_DISARMED, 0.0);

/**
 * Set the disarmed deflection/throttle command value for the actuator UAVCAN messages for channel 1
 *
 * This is the deflection/throttle value that the autopilot is outputting if not armed.
 *
 * The main use of this parameter is to silence ESCs/servos when they are disarmed and to trim control surfaces
 * if the control surfaces are not symetrical around the value set by the UAVCAN_TRIMx parameter.
 *
 * @reboot_required true
 *
 * @min -1.0
 * @max 1.0
 * @decimal 2
 * @unit normalized
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_DIS1, 0.0);

/**
 * Set the disarmed deflection/throttle command value for the actuator UAVCAN messages for channel 2
 *
 * This is the deflection/throttle value that the autopilot is outputting if not armed.
 *
 * The main use of this parameter is to silence ESCs/servos when they are disarmed and to trim control surfaces
 * if the control surfaces are not symetrical around the value set by the UAVCAN_TRIMx parameter.
 *
 * @reboot_required true
 *
 * @min -1.0
 * @max 1.0
 * @decimal 2
 * @unit normalized
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_DIS2, 0.0);

/**
 * Set the disarmed deflection/throttle command value for the actuator UAVCAN messages for channel 3
 *
 * This is the deflection/throttle value that the autopilot is outputting if not armed.
 *
 * The main use of this parameter is to silence ESCs/servos when they are disarmed and to trim control surfaces
 * if the control surfaces are not symetrical around the value set by the UAVCAN_TRIMx parameter.
 *
 * @reboot_required true
 *
 * @min -1.0
 * @max 1.0
 * @decimal 2
 * @unit normalized
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_DIS3, 0.0);

/**
 * Set the disarmed deflection/throttle command value for the actuator UAVCAN messages for channel 4
 *
 * This is the deflection/throttle value that the autopilot is outputting if not armed.
 *
 * The main use of this parameter is to silence ESCs/servos when they are disarmed and to trim control surfaces
 * if the control surfaces are not symetrical around the value set by the UAVCAN_TRIMx parameter.
 *
 * @reboot_required true
 *
 * @min -1.0
 * @max 1.0
 * @decimal 2
 * @unit normalized
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_DIS4, 0.0);

/**
 * Set the disarmed deflection/throttle command value for the actuator UAVCAN messages for channel 5
 *
 * This is the deflection/throttle value that the autopilot is outputting if not armed.
 *
 * The main use of this parameter is to silence ESCs/servos when they are disarmed and to trim control surfaces
 * if the control surfaces are not symetrical around the value set by the UAVCAN_TRIMx parameter.
 *
 * @reboot_required true
 *
 * @min -1.0
 * @max 1.0
 * @decimal 2
 * @unit normalized
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_DIS5, 0.0);

/**
 * Set the disarmed deflection/throttle command value for the actuator UAVCAN messages for channel 6
 *
 * This is the deflection/throttle value that the autopilot is outputting if not armed.
 *
 * The main use of this parameter is to silence ESCs/servos when they are disarmed and to trim control surfaces
 * if the control surfaces are not symetrical around the value set by the UAVCAN_TRIMx parameter.
 *
 * @reboot_required true
 *
 * @min -1.0
 * @max 1.0
 * @decimal 2
 * @unit normalized
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_DIS6, 0.0);

/**
 * Set the disarmed deflection/throttle command value for the actuator UAVCAN messages for channel 7
 *
 * This is the deflection/throttle value that the autopilot is outputting if not armed.
 *
 * The main use of this parameter is to silence ESCs/servos when they are disarmed and to trim control surfaces
 * if the control surfaces are not symetrical around the value set by the UAVCAN_TRIMx parameter.
 *
 * @reboot_required true
 *
 * @min -1.0
 * @max 1.0
 * @decimal 2
 * @unit normalized
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_DIS7, 0.0);

/**
 * Set the disarmed deflection/throttle command value for the actuator UAVCAN messages for channel 8
 *
 * This is the deflection/throttle value that the autopilot is outputting if not armed.
 *
 * The main use of this parameter is to silence ESCs/servos when they are disarmed and to trim control surfaces
 * if the control surfaces are not symetrical around the value set by the UAVCAN_TRIMx parameter.
 *
 * @reboot_required true
 *
 * @min -1.0
 * @max 1.0
 * @decimal 2
 * @unit normalized
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_DIS8, 0.0);


/**
 * Set the maximum deflection/throttle command value for the actuator UAVCAN message for channel 1.
 *
 * For ESC or servo depending on UAVCAN_MAP_CH1 parameter
 *
 * Set to 1.0 for full positive range.
 *
 * @reboot_required true
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @unit normalized
 * @group UAVCAN
 */

PARAM_DEFINE_FLOAT(UAVCAN_MAX1, 1.0);

/**
 * Set the maximum deflection/throttle command value for the actuator UAVCAN message for channel 2.
 *
 * For ESC or servo depending on UAVCAN_MAP_CH2 parameter
 *
 * Set to 1.0 for full positive range.
 *
 * @reboot_required true
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @unit normalized
 * @group UAVCAN
 */

PARAM_DEFINE_FLOAT(UAVCAN_MAX2, 1.0);

/**
 * Set the maximum deflection/throttle command value for the actuator UAVCAN message for channel 3.
 *
 * For ESC or servo depending on UAVCAN_MAP_CH3 parameter
 *
 * Set to 1.0 for full positive range.
 *
 * @reboot_required true
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @unit normalized
 * @group UAVCAN
 */

PARAM_DEFINE_FLOAT(UAVCAN_MAX3, 1.0);

/**
 * Set the maximum deflection/throttle command value for the actuator UAVCAN message for channel 4.
 *
 * For ESC or servo depending on UAVCAN_MAP_CH4 parameter
 *
 * Set to 1.0 for full positive range.
 *
 * @reboot_required true
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @unit normalized
 * @group UAVCAN
 */

PARAM_DEFINE_FLOAT(UAVCAN_MAX4, 1.0);

/**
 * Set the maximum deflection/throttle command value for the actuator UAVCAN message for channel 5.
 *
 * For ESC or servo depending on UAVCAN_MAP_CH5 parameter
 *
 * Set to 1.0 for full positive range.
 *
 * @reboot_required true
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @unit normalized
 * @group UAVCAN
 */

PARAM_DEFINE_FLOAT(UAVCAN_MAX5, 1.0);

/**
 * Set the maximum deflection/throttle command value for the actuator UAVCAN message for channel 6.
 *
 * For ESC or servo depending on UAVCAN_MAP_CH6 parameter
 *
 * Set to 1.0 for full positive range.
 *
 * @reboot_required true
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @unit normalized
 * @group UAVCAN
 */

PARAM_DEFINE_FLOAT(UAVCAN_MAX6, 1.0);

/**
 * Set the maximum deflection/throttle command value for the actuator UAVCAN message for channel 7.
 *
 * For ESC or servo depending on UAVCAN_MAP_CH7 parameter
 *
 * Set to 1.0 for full positive range.
 *
 * @reboot_required true
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @unit normalized
 * @group UAVCAN
 */

PARAM_DEFINE_FLOAT(UAVCAN_MAX7, 1.0);

/**
 * Set the maximum deflection/throttle command value for the actuator UAVCAN message for channel 8.
 *
 * For ESC or servo depending on UAVCAN_MAP_CH8 parameter
 *
 * Set to 1.0 for full positive range.
 *
 * @reboot_required true
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @unit normalized
 * @group UAVCAN
 */

PARAM_DEFINE_FLOAT(UAVCAN_MAX8, 1.0);

/**
 * Set the minimum deflection/throttle command value for the actuator UAVCAN message for channel 1.
 *
 * For ESC or servo depending on UAVCAN_MAP_CH1 parameter
 *
 * Set to -1.0 for full negative range.
 *
 * @reboot_required true
 *
 * @min -1.0
 * @max 0.0
 * @decimal 2
 * @unit normalized
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_MIN1, -1.0);

/**
 * Set the minimum deflection/throttle command value for the actuator UAVCAN message for channel 2.
 *
 * For ESC or servo depending on UAVCAN_MAP_CH2 parameter
 *
 * Set to -1.0 for full negative range.
 *
 * @reboot_required true
 *
 * @min -1.0
 * @max 0.0
 * @decimal 2
 * @unit normalized
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_MIN2, -1.0);

/**
 * Set the minimum deflection/throttle command value for the actuator UAVCAN message for channel 3.
 *
 * For ESC or servo depending on UAVCAN_MAP_CH3 parameter
 *
 * Set to -1.0 for full negative range.
 *
 * @reboot_required true
 *
 * @min -1.0
 * @max 0.0
 * @decimal 2
 * @unit normalized
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_MIN3, -1.0);

/**
 * Set the minimum deflection/throttle command value for the actuator UAVCAN message for channel 4.
 *
 * For ESC or servo depending on UAVCAN_MAP_CH4 parameter
 *
 * Set to -1.0 for full negative range.
 *
 * @reboot_required true
 *
 * @min -1.0
 * @max 0.0
 * @decimal 2
 * @unit normalized
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_MIN4, -1.0);

/**
 * Set the minimum deflection/throttle command value for the actuator UAVCAN message for channel 5.
 *
 * For ESC or servo depending on UAVCAN_MAP_CH5 parameter
 *
 * Set to -1.0 for full negative range.
 *
 * @reboot_required true
 *
 * @min -1.0
 * @max 0.0
 * @decimal 2
 * @unit normalized
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_MIN5, -1.0);

/**
 * Set the minimum deflection/throttle command value for the actuator UAVCAN message for channel 6.
 *
 * For ESC or servo depending on UAVCAN_MAP_CH6 parameter
 *
 * Set to -1.0 for full negative range.
 *
 * @reboot_required true
 *
 * @min -1.0
 * @max 0.0
 * @decimal 2
 * @unit normalized
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_MIN6, -1.0);

/**
 * Set the minimum deflection/throttle command value for the actuator UAVCAN message for channel 7.
 *
 * For ESC or servo depending on UAVCAN_MAP_CH7 parameter
 *
 * Set to -1.0 for full negative range.
 *
 * @reboot_required true
 *
 * @min -1.0
 * @max 0.0
 * @decimal 2
 * @unit normalized
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_MIN7, -1.0);

/**
 * Set the minimum deflection/throttle command value for the actuator UAVCAN message for channel 8.
 *
 * For ESC or servo depending on UAVCAN_MAP_CH8 parameter
 *
 * Set to -1.0 for full negative range.
 *
 * @reboot_required true
 *
 * @min -1.0
 * @max 0.0
 * @decimal 2
 * @unit normalized
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_MIN8, -1.0);

/**
 * Scale value for uavcan output channel 1
 *
 * Set to scale value.  scale + max trim = 1; -scale + min trim = -1
 *
 * @min 0.75
 * @max 1.0
 * @decimal 2
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_SCALE1, 1.0);

/**
 * Scale value for uavcan output channel 2
 *
 * Set to scale value.  scale + max trim = 1; -scale + min trim = -1
 *
 * @min 0.75
 * @max 1.0
 * @decimal 2
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_SCALE2, 1.0);

/**
 * Scale value for uavcan output channel 3
 *
 * Set to scale value.  scale + max trim = 1; -scale + min trim = -1
 *
 * @min 0.75
 * @max 1.0
 * @decimal 2
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_SCALE3, 1.0);

/**
 * Scale value for uavcan output channel 4
 *
 * Set to scale value.  scale + max trim = 1; -scale + min trim = -1
 *
 * @min 0.75
 * @max 1.0
 * @decimal 2
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_SCALE4, 1.0);

/**
 * Scale value for uavcan output channel 5
 *
 * Set to scale value.  scale + max trim = 1; -scale + min trim = -1
 *
 * @min 0.75
 * @max 1.0
 * @decimal 2
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_SCALE5, 1.0);

/**
 * Scale value for uavcan output channel 6
 *
 * Set to scale value.  scale + max trim = 1; -scale + min trim = -1
 *
 * @min 0.75
 * @max 1.0
 * @decimal 2
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_SCALE6, 1.0);

/**
 * Scale value for uavcan output channel 7
 *
 * Set to scale value.  scale + max trim = 1; -scale + min trim = -1
 *
 * @min 0.75
 * @max 1.0
 * @decimal 2
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_SCALE7, 1.0);

/**
 * Scale value for uavcan output channel 8
 *
 * Set to scale value.  scale + max trim = 1; -scale + min trim = -1
 *
 * @min 0.75
 * @max 1.0
 * @decimal 2
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_SCALE8, 1.0);

/**
 * Trim value for uavcan output channel 1
 *
 * Set to normalized offset
 *
 * @min -0.25
 * @max 0.25
 * @decimal 2
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_TRIM1, 0.0);

/**
 * Trim value for uavcan output channel 2
 *
 * Set to normalized offset
 *
 * @min -0.25
 * @max 0.25
 * @decimal 2
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_TRIM2, 0.0);

/**
 * Trim value for uavcan output channel 3
 *
 * Set to normalized offset
 *
 * @min -0.25
 * @max 0.25
 * @decimal 2
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_TRIM3, 0.0);

/**
 * Trim value for uavcan output channel 4
 *
 * Set to normalized offset
 *
 * @min -0.25
 * @max 0.25
 * @decimal 2
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_TRIM4, 0.0);

/**
 * Trim value for uavcan output channel 5
 *
 * Set to normalized offset
 *
 * @min -0.25
 * @max 0.25
 * @decimal 2
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_TRIM5, 0.0);

/**
 * Trim value for uavcan output channel 6
 *
 * Set to normalized offset
 *
 * @min -0.25
 * @max 0.25
 * @decimal 2
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_TRIM6, 0.0);

/**
 * Trim value for uavcan output channel 7
 *
 * Set to normalized offset
 *
 * @min -0.25
 * @max 0.25
 * @decimal 2
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_TRIM7, 0.0);

/**
 * Trim value for uavcan output channel 8
 *
 * Set to normalized offset
 *
 * @min -0.25
 * @max 0.25
 * @decimal 2
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_TRIM8, 0.0);

/**
 * UAVCAN mixer channel 1 to actuator type mapping
 *
 *  0 - AUX Mixer channel 1 not mapped to an actuator type (default)
 *  1 - AUX Mixer channel 1 mapped to ESC (esc::RawCommand)
 *  2 - AUX Mixer channel 1 mapped to Servo motor (actuator::ArrayCommand)
 *  3 - AUX Mixer channel 1 mapped to both ESC and Servo motor
 *
 * @min 0
 * @max 3
 * @value 0 AUX Mixer channel 1 disabled
 * @value 1 AUX Mixer channel 1 to ESC
 * @value 2 AUX Mixer channel 1 to Servo
 * @value 3 AUX Mixer Channel 1 to ESC and Servo
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_MIX_CH1, 0);

/**
 * UAVCAN mixer channel 2 to actuator type mapping
 *
 *  0 - AUX Mixer channel 2 not mapped to an actuator type (default)
 *  1 - AUX Mixer channel 2 mapped to ESC (esc::RawCommand)
 *  2 - AUX Mixer channel 2 mapped to Servo motor (actuator::ArrayCommand)
 *  3 - AUX Mixer channel 2 mapped to both ESC and Servo motor
 *
 * @min 0
 * @max 3
 * @value 0 AUX Mixer channel 2 disabled
 * @value 1 AUX Mixer channel 2 to ESC
 * @value 2 AUX Mixer channel 2 to Servo
 * @value 3 AUX Mixer Channel 2 to ESC and Servo
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_MIX_CH2, 1);

/**
 * UAVCAN mixer channel 3 to actuator type mapping
 *
 *  0 - AUX Mixer channel 3 not mapped to an actuator type (default)
 *  1 - AUX Mixer channel 3 mapped to ESC (esc::RawCommand)
 *  2 - AUX Mixer channel 3 mapped to Servo motor (actuator::ArrayCommand)
 *  3 - AUX Mixer channel 3 mapped to both ESC and Servo motor
 *
 * @min 0
 * @max 3
 * @value 0 AUX Mixer channel 3 disabled
 * @value 1 AUX Mixer channel 3 to ESC
 * @value 2 AUX Mixer channel 3 to Servo
 * @value 3 AUX Mixer Channel 3 to ESC and Servo
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_MIX_CH3, 1);

/**
 * UAVCAN mixer channel 4 to actuator type mapping
 *
 *  0 - AUX Mixer channel 4 not mapped to an actuator type (default)
 *  1 - AUX Mixer channel 4 mapped to ESC (esc::RawCommand)
 *  2 - AUX Mixer channel 4 mapped to Servo motor (actuator::ArrayCommand)
 *  3 - AUX Mixer channel 4 mapped to both ESC and Servo motor
 *
 * @min 0
 * @max 3
 * @value 0 AUX Mixer channel 4 disabled
 * @value 1 AUX Mixer channel 4 to ESC
 * @value 2 AUX Mixer channel 4 to Servo
 * @value 3 AUX Mixer Channel 4 to ESC and Servo
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_MIX_CH4, 0);

/**
 * UAVCAN mixer channel 5 to actuator type mapping
 *
 *  0 - AUX Mixer channel 5 not mapped to an actuator type (default)
 *  1 - AUX Mixer channel 5 mapped to ESC (esc::RawCommand)
 *  2 - AUX Mixer channel 5 mapped to Servo motor (actuator::ArrayCommand)
 *  3 - AUX Mixer channel 5 mapped to both ESC and Servo motor
 *
 * @min 0
 * @max 3
 * @value 0 AUX Mixer channel 5 disabled
 * @value 1 AUX Mixer channel 5 to ESC
 * @value 2 AUX Mixer channel 5 to Servo
 * @value 3 AUX Mixer Channel 5 to ESC and Servo
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_MIX_CH5, 0);

/**
 * UAVCAN mixer channel 6 to actuator type mapping
 *
 *  0 - AUX Mixer channel 6 not mapped to an actuator type (default)
 *  1 - AUX Mixer channel 6 mapped to ESC (esc::RawCommand)
 *  2 - AUX Mixer channel 6 mapped to Servo motor (actuator::ArrayCommand)
 *  3 - AUX Mixer channel 6 mapped to both ESC and Servo motor
 *
 * @min 0
 * @max 3
 * @value 0 AUX Mixer channel 6 disabled
 * @value 1 AUX Mixer channel 6 to ESC
 * @value 2 AUX Mixer channel 6 to Servo
 * @value 3 AUX Mixer Channel 6 to ESC and Servo
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_MIX_CH6, 0);

/**
 * UAVCAN mixer channel 7 to actuator type mapping
 *
 *  0 - AUX Mixer channel 7 not mapped to an actuator type (default)
 *  1 - AUX Mixer channel 7 mapped to ESC (esc::RawCommand)
 *  2 - AUX Mixer channel 7 mapped to Servo motor (actuator::ArrayCommand)
 *  3 - AUX Mixer channel 7 mapped to both ESC and Servo motor
 *
 * @min 0
 * @max 3
 * @value 0 AUX Mixer channel 7 disabled
 * @value 1 AUX Mixer channel 7 to ESC
 * @value 2 AUX Mixer channel 7 to Servo
 * @value 3 AUX Mixer Channel 7 to ESC and Servo
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_MIX_CH7, 0);

/**
 * UAVCAN mixer channel 8 to actuator type mapping
 *
 *  0 - AUX Mixer channel 8 not mapped to an actuator type (default)
 *  1 - AUX Mixer channel 8 mapped to ESC (esc::RawCommand)
 *  2 - AUX Mixer channel 8 mapped to Servo motor (actuator::ArrayCommand)
 *  3 - AUX Mixer channel 8 mapped to both ESC and Servo motor
 *
 * @min 0
 * @max 3
 * @value 0 AUX Mixer channel 8 disabled
 * @value 1 AUX Mixer channel 8 to ESC
 * @value 2 AUX Mixer channel 8 to Servo
 * @value 3 AUX Mixer Channel 8 to ESC and Servo
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_MIX_CH8, 0);
