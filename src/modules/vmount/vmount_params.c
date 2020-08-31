/****************************************************************************
*
*   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
* @file vmount_params.c
* @author Leon Müller (thedevleon)
* @author Matthew Edwards (mje-nz)
*
*/

/**
* Mount input mode
*
* RC uses the AUX input channels (see MNT_MAN_* parameters),
* MAVLINK_ROI uses the MAV_CMD_DO_SET_ROI Mavlink message, and MAVLINK_DO_MOUNT the
* MAV_CMD_DO_MOUNT_CONFIGURE and MAV_CMD_DO_MOUNT_CONTROL messages to control a mount.
*
* @value -1 DISABLED
* @value 0 AUTO
* @value 1 RC
* @value 2 MAVLINK_ROI
* @value 3 MAVLINK_DO_MOUNT
* @min -1
* @max 3
* @group Mount
* @reboot_required true
*/
PARAM_DEFINE_INT32(MNT_MODE_IN, -1);

/**
* Mount output mode
*
* AUX uses the mixer output Control Group #2.
* MAVLINK uses the MAV_CMD_DO_MOUNT_CONFIGURE and MAV_CMD_DO_MOUNT_CONTROL MavLink messages
* to control a mount (set MNT_MAV_SYSID & MNT_MAV_COMPID)
*
* @value 0 AUX
* @value 1 MAVLINK
* @min 0
* @max 1
* @group Mount
*/
PARAM_DEFINE_INT32(MNT_MODE_OUT, 0);

/**
* Mavlink System ID of the mount
*
* If MNT_MODE_OUT is MAVLINK, mount configure/control commands will be sent with this target ID.
*
* @group Mount
*/
PARAM_DEFINE_INT32(MNT_MAV_SYSID, 1);

/**
* Mavlink Component ID of the mount
*
* If MNT_MODE_OUT is MAVLINK, mount configure/control commands will be sent with this component ID.
*
* @group Mount
*/
PARAM_DEFINE_INT32(MNT_MAV_COMPID, 154);

/**
* Mixer value for selecting normal mode
* if required by the gimbal (only in AUX output mode)
*
* @min -1.0
* @max 1.0
* @decimal 3
* @group Mount
*/
PARAM_DEFINE_FLOAT(MNT_OB_NORM_MODE, -1.0f);

/**
* Mixer value for selecting a locking mode
* if required for the gimbal (only in AUX output mode)
*
* @min -1.0
* @max 1.0
* @decimal 3
* @group Mount
*/
PARAM_DEFINE_FLOAT(MNT_OB_LOCK_MODE, 0.0f);

/**
* Auxiliary channel to control roll (in AUX input or manual mode).
*
* @value 0 Disable
* @value 1 AUX1
* @value 2 AUX2
* @value 3 AUX3
* @value 4 AUX4
* @value 5 AUX5
* @value 6 AUX6
* @min 0
* @max 6
* @group Mount
*/
PARAM_DEFINE_INT32(MNT_MAN_ROLL, 0);

/**
* Auxiliary channel to control pitch (in AUX input or manual mode).
*
* @value 0 Disable
* @value 1 AUX1
* @value 2 AUX2
* @value 3 AUX3
* @value 4 AUX4
* @value 5 AUX5
* @value 6 AUX6
* @min 0
* @max 6
* @group Mount
*/
PARAM_DEFINE_INT32(MNT_MAN_PITCH, 0);

/**
* Auxiliary channel to control yaw (in AUX input or manual mode).
*
* @value 0 Disable
* @value 1 AUX1
* @value 2 AUX2
* @value 3 AUX3
* @value 4 AUX4
* @value 5 AUX5
* @value 6 AUX6
* @min 0
* @max 6
* @group Mount
*/
PARAM_DEFINE_INT32(MNT_MAN_YAW, 0);

/**
* Stabilize the mount (set to true for servo gimbal, false for passthrough).
* Does not affect MAVLINK_ROI input.
*
* @boolean
* @group Mount
*/
PARAM_DEFINE_INT32(MNT_DO_STAB, 0);

/**
* Range of pitch channel output in degrees (only in AUX output mode).
*
* @min 1.0
* @max 720.0
* @decimal 1
* @group Mount
*/
PARAM_DEFINE_FLOAT(MNT_RANGE_PITCH, 360.0f);

/**
* Range of roll channel output in degrees (only in AUX output mode).
*
* @min 1.0
* @max 720.0
* @decimal 1
* @group Mount
*/
PARAM_DEFINE_FLOAT(MNT_RANGE_ROLL, 360.0f);

/**
* Range of yaw channel output in degrees (only in AUX output mode).
*
* @min 1.0
* @max 720.0
* @decimal 1
* @group Mount
*/
PARAM_DEFINE_FLOAT(MNT_RANGE_YAW, 360.0f);

/**
* Offset for pitch channel output in degrees.
*
* @min -360.0
* @max 360.0
* @decimal 1
* @group Mount
*/
PARAM_DEFINE_FLOAT(MNT_OFF_PITCH, 0.0f);

/**
* Offset for roll channel output in degrees.
*
* @min -360.0
* @max 360.0
* @decimal 1
* @group Mount
*/
PARAM_DEFINE_FLOAT(MNT_OFF_ROLL, 0.0f);

/**
* Offset for yaw channel output in degrees.
*
* @min -360.0
* @max 360.0
* @decimal 1
* @group Mount
*/
PARAM_DEFINE_FLOAT(MNT_OFF_YAW, 0.0f);
