/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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
 * UAVCAN v1
 *
 *  0 - UAVCAN disabled.
 *  1 - Enables UAVCANv1
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN v1
 */
PARAM_DEFINE_INT32(UAVCAN_V1_ENABLE, 0);

/**
 * UAVCAN v1 Node ID.
 *
 * Read the specs at http://uavcan.org to learn more about Node ID.
 *
 * @min 1
 * @max 125
 * @reboot_required true
 * @group UAVCANv1
 */
PARAM_DEFINE_INT32(UAVCAN_V1_ID, 1);

/**
 * UAVCAN/CAN v1 bus bitrate.
 *
 * @unit bit/s
 * @min 20000
 * @max 1000000
 * @reboot_required true
 * @group UAVCAN v1
 */
PARAM_DEFINE_INT32(UAVCAN_V1_BAUD, 1000000);

/**
 * UAVCAN v1 battery mode.
 *
 * @value 0 Disable
 * @value 1 Receive BMSStatus
 * @value 2 Send BMSStatus
 * @reboot_required true
 * @group UAVCAN v1
 */
PARAM_DEFINE_INT32(UAVCAN_V1_BAT_MD, 0);

/**
 * UAVCAN v1 battery port ID.
 *
 * @min 1
 * @max 32767
 * @group UAVCAN v1
 */
PARAM_DEFINE_INT32(UAVCAN_V1_BAT_ID, 4242);

// Subscription Port IDs
PARAM_DEFINE_INT32(UCAN1_ESC0_PID, 0);
PARAM_DEFINE_INT32(UCAN1_GPS0_PID, 0);
PARAM_DEFINE_INT32(UCAN1_GPS1_PID, 0);
//PARAM_DEFINE_INT32(UCAN1_BMS0_PID, 0);
//PARAM_DEFINE_INT32(UCAN1_BMS1_PID, 0);

PARAM_DEFINE_INT32(UCAN1_BMS_ES_PID, 0);
PARAM_DEFINE_INT32(UCAN1_BMS_BS_PID, 0);
PARAM_DEFINE_INT32(UCAN1_BMS_BP_PID, 0);

PARAM_DEFINE_INT32(UCAN1_LG_BMS_PID, 0);

// Publication Port IDs
PARAM_DEFINE_INT32(UCAN1_ESC_PUB, 0);
PARAM_DEFINE_INT32(UCAN1_GPS_PUB, 0);
PARAM_DEFINE_INT32(UCAN1_SERVO_PUB, 0);
