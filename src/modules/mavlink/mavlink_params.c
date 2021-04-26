/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * MAVLink system ID
 * @group MAVLink
 * @min 1
 * @max 250
 * @reboot_required true
 */
PARAM_DEFINE_INT32(MAV_SYS_ID, 1);

/**
 * MAVLink component ID
 * @group MAVLink
 * @min 1
 * @max 250
 * @reboot_required true
 */
PARAM_DEFINE_INT32(MAV_COMP_ID, 1);

/**
 * MAVLink protocol version
 * @group MAVLink
 * @value 0 Default to 1, switch to 2 if GCS sends version 2
 * @value 1 Always use version 1
 * @value 2 Always use version 2
 */
PARAM_DEFINE_INT32(MAV_PROTO_VER, 0);

/**
 * MAVLink SiK Radio ID
 *
 * When non-zero the MAVLink app will attempt to configure the
 * SiK radio to this ID and re-set the parameter to 0. If the value
 * is negative it will reset the complete radio config to
 * factory defaults. Only applies if this mavlink instance is going through a SiK radio
 *
 * @group MAVLink
 * @min -1
 * @max 240
 */
PARAM_DEFINE_INT32(MAV_SIK_RADIO_ID, 0);

/**
 * MAVLink airframe type
 *
 * @min 1
 * @max 27
 * @value 0 Generic micro air vehicle
 * @value 1 Fixed wing aircraft
 * @value 2 Quadrotor
 * @value 3 Coaxial helicopter
 * @value 4 Normal helicopter with tail rotor
 * @value 5 Ground installation
 * @value 6 Operator control unit / ground control station
 * @value 7 Airship, controlled
 * @value 8 Free balloon, uncontrolled
 * @value 9 Rocket
 * @value 10 Ground rover
 * @value 11 Surface vessel, boat, ship
 * @value 12 Submarine
 * @value 13 Hexarotor
 * @value 14 Octorotor
 * @value 15 Tricopter
 * @value 16 Flapping wing
 * @value 17 Kite
 * @value 18 Onboard companion controller
 * @value 19 Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter.
 * @value 20 Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter.
 * @value 21 Tiltrotor VTOL
 * @value 22 VTOL reserved 2
 * @value 23 VTOL reserved 3
 * @value 24 VTOL reserved 4
 * @value 25 VTOL reserved 5
 * @value 26 Onboard gimbal
 * @value 27 Onboard ADSB peripheral
 * @group MAVLink
 */
PARAM_DEFINE_INT32(MAV_TYPE, 2);

/**
 * Use/Accept HIL GPS message even if not in HIL mode
 *
 * If set to 1 incoming HIL GPS messages are parsed.
 *
 * @boolean
 * @group MAVLink
 */
PARAM_DEFINE_INT32(MAV_USEHILGPS, 0);

/**
 * Forward external setpoint messages
 *
 * If set to 1 incoming external setpoint messages will be directly forwarded
 * to the controllers if in offboard control mode
 *
 * @boolean
 * @group MAVLink
 */
PARAM_DEFINE_INT32(MAV_FWDEXTSP, 1);

/**
 * Parameter hash check.
 *
 * Disabling the parameter hash check functionality will make the mavlink instance
 * stream parameters continuously.
 *
 * @boolean
 * @group MAVLink
 */
PARAM_DEFINE_INT32(MAV_HASH_CHK_EN, 1);

/**
 * Hearbeat message forwarding.
 *
 * The mavlink hearbeat message will not be forwarded if this parameter is set to 'disabled'.
 * The main reason for disabling heartbeats to be forwarded is because they confuse dronekit.
 *
 * @boolean
 * @group MAVLink
 */
PARAM_DEFINE_INT32(MAV_HB_FORW_EN, 1);

/**
 * Activate ODOMETRY loopback.
 *
 * If set, it gets the data from 'vehicle_visual_odometry' instead of 'vehicle_odometry'
 * serving as a loopback of the received ODOMETRY messages on the Mavlink receiver.
 *
 * @boolean
 * @group MAVLink
 */
PARAM_DEFINE_INT32(MAV_ODOM_LP, 0);

/**
 * Timeout in seconds for the RADIO_STATUS reports coming in
 *
 * If the connected radio stops reporting RADIO_STATUS for a certain time,
 * a warning is triggered and, if MAV_X_RADIO_CTL is enabled, the software-flow
 * control is reset.
 *
 * @group MAVLink
 * @unit s
 * @min 1
 * @max 250
 */
PARAM_DEFINE_INT32(MAV_RADIO_TOUT, 5);
