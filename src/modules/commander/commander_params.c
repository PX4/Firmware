/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file commander_params.c
 *
 * Parameters defined by the sensors task.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Thomas Gubler <thomas@px4.io>
 * @author Julian Oes <julian@px4.io>
 */

/**
 * Roll trim
 *
 * The trim value is the actuator control value the system needs
 * for straight and level flight. It can be calibrated by
 * flying manually straight and level using the RC trims and
 * copying them using the GCS.
 *
 * @group Radio Calibration
 * @min -0.25
 * @max 0.25
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(TRIM_ROLL, 0.0f);

/**
 * Pitch trim
 *
 * The trim value is the actuator control value the system needs
 * for straight and level flight. It can be calibrated by
 * flying manually straight and level using the RC trims and
 * copying them using the GCS.
 *
 * @group Radio Calibration
 * @min -0.25
 * @max 0.25
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(TRIM_PITCH, 0.0f);

/**
 * Yaw trim
 *
 * The trim value is the actuator control value the system needs
 * for straight and level flight. It can be calibrated by
 * flying manually straight and level using the RC trims and
 * copying them using the GCS.
 *
 * @group Radio Calibration
 * @min -0.25
 * @max 0.25
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(TRIM_YAW, 0.0f);

/**
 * Datalink loss time threshold
 *
 * After this amount of seconds without datalink the data link lost mode triggers
 *
 * @group Commander
 * @unit s
 * @min 5
 * @max 300
 * @decimal 1
 * @increment 1
 */
PARAM_DEFINE_INT32(COM_DL_LOSS_T, 10);

/**
 * High Latency Datalink loss time threshold
 *
 * After this amount of seconds without datalink the data link lost mode triggers
 *
 * @group Commander
 * @unit s
 * @min 60
 * @max 3600
 */
PARAM_DEFINE_INT32(COM_HLDL_LOSS_T, 120);

/**
 * High Latency Datalink regain time threshold
 *
 * After a data link loss: after this number of seconds with a healthy datalink the 'datalink loss'
 * flag is set back to false
 *
 * @group Commander
 * @unit s
 * @min 0
 * @max 60
 */
PARAM_DEFINE_INT32(COM_HLDL_REG_T, 0);

/**
 * Engine Failure Throttle Threshold
 *
 * Engine failure triggers only above this throttle value
 *
 * @group Commander
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(COM_EF_THROT, 0.5f);

/**
 * Engine Failure Current/Throttle Threshold
 *
 * Engine failure triggers only below this current value
 *
 * @group Commander
 * @min 0.0
 * @max 50.0
 * @unit A/%
 * @decimal 2
 * @increment 1
 */
PARAM_DEFINE_FLOAT(COM_EF_C2T, 5.0f);

/**
 * Engine Failure Time Threshold
 *
 * Engine failure triggers only if the throttle threshold and the
 * current to throttle threshold are violated for this time
 *
 * @group Commander
 * @unit s
 * @min 0.0
 * @max 60.0
 * @decimal 1
 * @increment 1
 */
PARAM_DEFINE_FLOAT(COM_EF_TIME, 10.0f);

/**
 * RC loss time threshold
 *
 * After this amount of seconds without RC connection it's considered lost and not used anymore
 *
 * @group Commander
 * @unit s
 * @min 0
 * @max 35
 * @decimal 1
 * @increment 0.1
 */
PARAM_DEFINE_FLOAT(COM_RC_LOSS_T, 0.5f);

/**
 * Delay between RC loss and configured reaction
 *
 * RC signal not updated -> still use data for COM_RC_LOSS_T seconds
 * Consider RC signal lost -> wait COM_RCL_ACT_T seconds on the spot waiting to regain signal
 * React with failsafe action NAV_RCL_ACT
 *
 * A zero value disables the delay.
 *
 * @group Commander
 * @unit s
 * @min 0.0
 * @max 25.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(COM_RCL_ACT_T, 15.0f);

/**
 * Home set horizontal threshold
 *
 * The home position will be set if the estimated positioning accuracy is below the threshold.
 *
 * @group Commander
 * @unit m
 * @min 2
 * @max 15
 * @decimal 2
 * @increment 0.5
 */
PARAM_DEFINE_FLOAT(COM_HOME_H_T, 5.0f);

/**
 * Home set vertical threshold
 *
 * The home position will be set if the estimated positioning accuracy is below the threshold.
 *
 * @group Commander
 * @unit m
 * @min 5
 * @max 25
 * @decimal 2
 * @increment 0.5
 */
PARAM_DEFINE_FLOAT(COM_HOME_V_T, 10.0f);

/**
 * Allows setting the home position after takeoff
 *
 * If set to true, the autopilot is allowed to set its home position after takeoff
 * The true home position is back-computed if a local position is estimate if available.
 * If no local position is available, home is set to the current position.
 *
 * @boolean
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_HOME_IN_AIR, 0);

/**
 * RC control input mode
 *
 * The default value of 0 requires a valid RC transmitter setup.
 * Setting this to 1 allows joystick control and disables RC input handling and the associated checks. A value of
 * 2 will generate RC control data from manual input received via MAVLink instead
 * of directly forwarding the manual input data.
 *
 * @group Commander
 * @min 0
 * @max 2
 * @value 0 RC Transmitter
 * @value 1 Joystick/No RC Checks
 * @value 2 Virtual RC by Joystick
 */
PARAM_DEFINE_INT32(COM_RC_IN_MODE, 0);

/**
 * RC input arm/disarm command duration
 *
 * The default value of 1000 requires the stick to be held in the arm or disarm position for 1 second.
 *
 * @group Commander
 * @min 100
 * @max 1500
 * @unit ms
 */
PARAM_DEFINE_INT32(COM_RC_ARM_HYST, 1000);

/**
 * Time-out for auto disarm after landing
 *
 * A non-zero, positive value specifies the time-out period in seconds after which the vehicle will be
 * automatically disarmed in case a landing situation has been detected during this period.
 *
 * A zero or negative value means that automatic disarming triggered by landing detection is disabled.
 *
 * @group Commander
 * @unit s
 * @decimal 2
 */

PARAM_DEFINE_FLOAT(COM_DISARM_LAND, 2.0f);

/**
 * Time-out for auto disarm if not taking off
 *
 * A non-zero, positive value specifies the time in seconds, within which the
 * vehicle is expected to take off after arming. In case the vehicle didn't takeoff
 * within the timout it disamrs again.
 *
 * A negative value disables autmoatic disarming triggered by a pre-takeoff timeout.
 *
 * @group Commander
 * @unit s
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(COM_DISARM_PRFLT, 10.0f);


/**
 * Allow arming without GPS
 *
 * The default allows the vehicle to arm without GPS signal.
 *
 * @group Commander
 * @value 0 Require GPS lock to arm
 * @value 1 Allow arming without GPS
 */
PARAM_DEFINE_INT32(COM_ARM_WO_GPS, 1);

/**
 * Arm switch is a momentary button
 *
 * 0: Arming/disarming triggers on switch transition.
 * 1: Arming/disarming triggers when holding the momentary button down
 * for COM_RC_ARM_HYST like the stick gesture.
 *
 * @group Commander
 * @boolean
 */
PARAM_DEFINE_INT32(COM_ARM_SWISBTN, 0);

/**
 * Battery failsafe mode
 *
 * Action the system takes at critical battery. See also BAT_CRIT_THR and BAT_EMERGEN_THR
 * for definition of battery states.
 *
 * @group Commander
 * @value 0 Warning
 * @value 2 Land mode
 * @value 3 Return at critical level, land at emergency level
 * @decimal 0
 * @increment 1
 */
PARAM_DEFINE_INT32(COM_LOW_BAT_ACT, 0);

/**
 * Time-out to wait when offboard connection is lost before triggering offboard lost action.
 *
 * See COM_OBL_ACT and COM_OBL_RC_ACT to configure action.
 *
 * @group Commander
 * @unit s
 * @min 0
 * @max 60
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(COM_OF_LOSS_T, 1.0f);

/**
 * Set offboard loss failsafe mode
 *
 * The offboard loss failsafe will only be entered after a timeout,
 * set by COM_OF_LOSS_T in seconds.
 *
 * @value -1 Disabled
 * @value  0 Land mode
 * @value  1 Hold mode
 * @value  2 Return mode
 * @value  3 Terminate
 * @value  4 Lockdown
 *
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_OBL_ACT, 0);

/**
 * Set offboard loss failsafe mode when RC is available
 *
 * The offboard loss failsafe will only be entered after a timeout,
 * set by COM_OF_LOSS_T in seconds.
 *
 * @value -1 Disabled
 * @value  0 Position mode
 * @value  1 Altitude mode
 * @value  2 Manual
 * @value  3 Return mode
 * @value  4 Land mode
 * @value  5 Hold mode
 * @value  6 Terminate
 * @value  7 Lockdown
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_OBL_RC_ACT, 0);

/**
 * Time-out to wait when onboard computer connection is lost before warning about loss connection.
 *
 * @group Commander
 * @unit s
 * @min 0
 * @max 60
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(COM_OBC_LOSS_T, 5.0f);

/**
 * First flightmode slot (1000-1160)
 *
 * If the main switch channel is in this range the
 * selected flight mode will be applied.
 *
 * @value -1 Unassigned
 * @value 0 Manual
 * @value 1 Altitude
 * @value 2 Position
 * @value 3 Mission
 * @value 4 Hold
 * @value 10 Takeoff
 * @value 11 Land
 * @value 5 Return
 * @value 6 Acro
 * @value 7 Offboard
 * @value 8 Stabilized
 * @value 12 Follow Me
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_FLTMODE1, -1);

/**
 * Second flightmode slot (1160-1320)
 *
 * If the main switch channel is in this range the
 * selected flight mode will be applied.
 *
 * @value -1 Unassigned
 * @value 0 Manual
 * @value 1 Altitude
 * @value 2 Position
 * @value 3 Mission
 * @value 4 Hold
 * @value 10 Takeoff
 * @value 11 Land
 * @value 5 Return
 * @value 6 Acro
 * @value 7 Offboard
 * @value 8 Stabilized
 * @value 12 Follow Me
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_FLTMODE2, -1);

/**
 * Third flightmode slot (1320-1480)
 *
 * If the main switch channel is in this range the
 * selected flight mode will be applied.
 *
 * @value -1 Unassigned
 * @value 0 Manual
 * @value 1 Altitude
 * @value 2 Position
 * @value 3 Mission
 * @value 4 Hold
 * @value 10 Takeoff
 * @value 11 Land
 * @value 5 Return
 * @value 6 Acro
 * @value 7 Offboard
 * @value 8 Stabilized
 * @value 12 Follow Me
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_FLTMODE3, -1);

/**
 * Fourth flightmode slot (1480-1640)
 *
 * If the main switch channel is in this range the
 * selected flight mode will be applied.
 *
 * @value -1 Unassigned
 * @value 0 Manual
 * @value 1 Altitude
 * @value 2 Position
 * @value 3 Mission
 * @value 4 Hold
 * @value 10 Takeoff
 * @value 11 Land
 * @value 5 Return
 * @value 6 Acro
 * @value 7 Offboard
 * @value 8 Stabilized
 * @value 12 Follow Me
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_FLTMODE4, -1);

/**
 * Fifth flightmode slot (1640-1800)
 *
 * If the main switch channel is in this range the
 * selected flight mode will be applied.
 *
 * @value -1 Unassigned
 * @value 0 Manual
 * @value 1 Altitude
 * @value 2 Position
 * @value 3 Mission
 * @value 4 Hold
 * @value 10 Takeoff
 * @value 11 Land
 * @value 5 Return
 * @value 6 Acro
 * @value 7 Offboard
 * @value 8 Stabilized
 * @value 12 Follow Me
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_FLTMODE5, -1);

/**
 * Sixth flightmode slot (1800-2000)
 *
 * If the main switch channel is in this range the
 * selected flight mode will be applied.
 *
 * @value -1 Unassigned
 * @value 0 Manual
 * @value 1 Altitude
 * @value 2 Position
 * @value 3 Mission
 * @value 4 Hold
 * @value 10 Takeoff
 * @value 11 Land
 * @value 5 Return
 * @value 6 Acro
 * @value 7 Offboard
 * @value 8 Stabilized
 * @value 12 Follow Me
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_FLTMODE6, -1);

/**
 * Maximum EKF position innovation test ratio that will allow arming
 *
 * @group Commander
 * @min 0.1
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 */
PARAM_DEFINE_FLOAT(COM_ARM_EKF_POS, 0.5f);

/**
 * Maximum EKF velocity innovation test ratio that will allow arming
 *
 * @group Commander
 * @min 0.1
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 */
PARAM_DEFINE_FLOAT(COM_ARM_EKF_VEL, 0.5f);

/**
 * Maximum EKF height innovation test ratio that will allow arming
 *
 * @group Commander
 * @min 0.1
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 */
PARAM_DEFINE_FLOAT(COM_ARM_EKF_HGT, 1.0f);

/**
 * Maximum EKF yaw innovation test ratio that will allow arming
 *
 * @group Commander
 * @min 0.1
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 */
PARAM_DEFINE_FLOAT(COM_ARM_EKF_YAW, 0.5f);

/**
 * Maximum accelerometer inconsistency between IMU units that will allow arming
 *
 * @group Commander
 * @unit m/s^2
 * @min 0.1
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 */
PARAM_DEFINE_FLOAT(COM_ARM_IMU_ACC, 0.7f);

/**
 * Maximum rate gyro inconsistency between IMU units that will allow arming
 *
 * @group Commander
 * @unit rad/s
 * @min 0.02
 * @max 0.3
 * @decimal 3
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(COM_ARM_IMU_GYR, 0.25f);

/**
 * Maximum magnetic field inconsistency between units that will allow arming
 *
 * Set -1 to disable the check.
 *
 * @group Commander
 * @unit deg
 * @min 3
 * @max 180
 */
PARAM_DEFINE_INT32(COM_ARM_MAG_ANG, 45);

/**
 * Enable mag strength preflight check
 *
 * Deny arming if the estimator detects a strong magnetic
 * disturbance (check enabled by EKF2_MAG_CHECK)
 *
 * @boolean
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_ARM_MAG_STR, 1);

/**
 * Rearming grace period
 *
 * Re-arming grace allows to rearm the drone with manual command without running prearmcheck during 5 s after disarming.
 *
 * @group Commander
 * @boolean
 */
PARAM_DEFINE_INT32(COM_REARM_GRACE, 1);

/**
 * Enable RC stick override of auto and/or offboard modes
 *
 * When RC stick override is enabled, moving the RC sticks more than COM_RC_STICK_OV from
 * their center position immediately gives control back to the pilot by switching to Position mode.
 * Note: Only has an effect on multicopters, and VTOLs in multicopter mode.
 *
 * @min 0
 * @max 7
 * @bit 0 Enable override during auto modes (except for in critical battery reaction)
 * @bit 1 Enable override during offboard mode
 * @bit 2 Ignore throttle stick
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_RC_OVERRIDE, 1);

/**
 * RC stick override threshold
 *
 * If COM_RC_OVERRIDE is enabled and the joystick input is moved more than this threshold
 * the autopilot the pilot takes over control.
 *
 * @group Commander
 * @unit %
 * @min 5
 * @max 80
 * @decimal 0
 * @increment 0.05
 */
PARAM_DEFINE_FLOAT(COM_RC_STICK_OV, 30.0f);

/**
 * Require valid mission to arm
 *
 * The default allows to arm the vehicle without a valid mission.
 *
 * @group Commander
 * @boolean
 */
PARAM_DEFINE_INT32(COM_ARM_MIS_REQ, 0);

/**
 * Position control navigation loss response.
 *
 * This sets the flight mode that will be used if navigation accuracy is no longer adequate for position control.
 * Navigation accuracy checks can be disabled using the CBRK_VELPOSERR parameter, but doing so will remove protection for all flight modes.
 *
 * @value 0 Altitude/Manual. Assume use of remote control after fallback. Switch to Altitude mode if a height estimate is available, else switch to MANUAL.
 * @value 1 Land/Terminate. Assume no use of remote control after fallback. Switch to Land mode if a height estimate is available, else switch to TERMINATION.
 *
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_POSCTL_NAVL, 0);

/**
 * Require arm authorization to arm
 *
 * By default off. The default allows to arm the vehicle without a arm authorization.
 *
 * @group Commander
 * @boolean
 */
PARAM_DEFINE_INT32(COM_ARM_AUTH_REQ, 0);

/**
 * Arm authorizer system id
 *
 * Used if arm authorization is requested by COM_ARM_AUTH_REQ.
 *
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_ARM_AUTH_ID, 10);

/**
 * Arm authorization method
 *
 * Methods:
 * - one arm: request authorization and arm when authorization is received
 * - two step arm: 1st arm command request an authorization and
 *                 2nd arm command arm the drone if authorized
 *
 * Used if arm authorization is requested by COM_ARM_AUTH_REQ.
 *
 * @group Commander
 * @value 0 one arm
 * @value 1 two step arm
 */
PARAM_DEFINE_INT32(COM_ARM_AUTH_MET, 0);

/**
 * Arm authorization timeout
 *
 * Timeout for authorizer answer.
 * Used if arm authorization is requested by COM_ARM_AUTH_REQ.
 *
 * @group Commander
 * @unit s
 * @decimal 1
 * @increment 0.1
 */
PARAM_DEFINE_FLOAT(COM_ARM_AUTH_TO, 1);

/**
 * Loss of position failsafe activation delay.
 *
 * This sets number of seconds that the position checks need to be failed before the failsafe will activate.
 * The default value has been optimised for rotary wing applications. For fixed wing applications, a larger value between 5 and 10 should be used.
 *
 * @unit s
 * @reboot_required true
 * @group Commander
 * @min 1
 * @max 100
 */
PARAM_DEFINE_INT32(COM_POS_FS_DELAY, 1);

/**
 * Loss of position probation delay at takeoff.
 *
 * The probation delay is the number of seconds that the EKF innovation checks need to pass for the position to be declared good after it has been declared bad.
 * The probation delay will be reset to this parameter value when takeoff is detected.
 * After takeoff, if position checks are passing, the probation delay will reduce by one second for every lapsed second of valid position down to a minimum of 1 second.
 * If position checks are failing, the probation delay will increase by COM_POS_FS_GAIN seconds for every lapsed second up to a maximum of 100 seconds.
 * The default value has been optimised for rotary wing applications. For fixed wing applications, a value of 1 should be used.
 *
 * @unit s
 * @reboot_required true
 * @group Commander
 * @min 1
 * @max 100
 */
PARAM_DEFINE_INT32(COM_POS_FS_PROB, 30);

/**
 * Loss of position probation gain factor.
 *
 * This sets the rate that the loss of position probation time grows when position checks are failing.
 * The default value has been optimised for rotary wing applications. For fixed wing applications a value of 0 should be used.
 *
 * @reboot_required true
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_POS_FS_GAIN, 10);

/**
 * Horizontal position error threshold.
 *
 * This is the horizontal position error (EPH) threshold that will trigger a failsafe. The default is appropriate for a multicopter. Can be increased for a fixed-wing.
 *
 * @unit m
 * @group Commander
 */
PARAM_DEFINE_FLOAT(COM_POS_FS_EPH, 5);

/**
 * Vertical position error threshold.
 *
 * This is the vertical position error (EPV) threshold that will trigger a failsafe. The default is appropriate for a multicopter. Can be increased for a fixed-wing.
 *
 * @unit m
 * @group Commander
 */
PARAM_DEFINE_FLOAT(COM_POS_FS_EPV, 10);

/**
 * Horizontal velocity error threshold.
 *
 * This is the horizontal velocity error (EVH) threshold that will trigger a failsafe. The default is appropriate for a multicopter. Can be increased for a fixed-wing.
 *
 * @unit m/s
 * @group Commander
 */
PARAM_DEFINE_FLOAT(COM_VEL_FS_EVH, 1);

/**
 * Next flight UUID
 *
 * This number is incremented automatically after every flight on
 * disarming in order to remember the next flight UUID.
 * The first flight is 0.
 *
 * @group Commander
 * @category system
 * @volatile
 * @min 0
 */
PARAM_DEFINE_INT32(COM_FLIGHT_UUID, 0);

/**
 * Action after TAKEOFF has been accepted.
 *
 * The mode transition after TAKEOFF has completed successfully.
 *
 * @value 0 Hold
 * @value 1 Mission (if valid)
 * @group Mission
 */
PARAM_DEFINE_INT32(COM_TAKEOFF_ACT, 0);

/**
 * Set data link loss failsafe mode
 *
 * The data link loss failsafe will only be entered after a timeout,
 * set by COM_DL_LOSS_T in seconds. Once the timeout occurs the selected
 * action will be executed.
 *
 * @value 0 Disabled
 * @value 1 Hold mode
 * @value 2 Return mode
 * @value 3 Land mode
 * @value 5 Terminate
 * @value 6 Lockdown
 *
 * @group Mission
 */
PARAM_DEFINE_INT32(NAV_DLL_ACT, 0);

/**
 * Set RC loss failsafe mode
 *
 * The RC loss failsafe will only be entered after a timeout,
 * set by COM_RC_LOSS_T in seconds. If RC input checks have been disabled
 * by setting the COM_RC_IN_MODE param it will not be triggered.
 *
 * @value 0 Disabled
 * @value 1 Hold mode
 * @value 2 Return mode
 * @value 3 Land mode
 * @value 5 Terminate
 * @value 6 Lockdown
 *
 * @group Mission
 */
PARAM_DEFINE_INT32(NAV_RCL_ACT, 2);

/**
 * Flag to enable obstacle avoidance.
 *
 * @boolean
 * @group Mission
 */
PARAM_DEFINE_INT32(COM_OBS_AVOID, 0);

/**
 * User Flight Profile
 *
 * Describes the intended use of the vehicle.
 * Can be used by ground control software or log post processing.
 * This param does not influence the behavior within the firmware. This means for example the control logic is independent of the setting of this param (but depends on other params).
 *
 * @value 0 Default
 * @value 100 Pro User
 * @value 200 Flight Tester
 * @value 300 Developer
 *
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_FLT_PROFILE, 0);

/**
 * Enable checks on ESCs that report telemetry.
 *
 * If this parameter is set, the system will check ESC's online status and failures.
 * This param is specific for ESCs reporting status. It shall be used only if ESCs support telemetry.
 *
 * @group Commander
 * @boolean
 */
PARAM_DEFINE_INT32(COM_ARM_CHK_ESCS, 0);

/**
 * Condition to enter prearmed mode
 *
 * Condition to enter the prearmed state, an intermediate state between disarmed and armed
 * in which non-throttling actuators are active.
 *
 * @value 0 Disabled
 * @value 1 Safety button
 * @value 2 Always
 *
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_PREARM_MODE, 0);

/**
 * Enable Motor Testing
 *
 * If set, enables the motor test interface via MAVLink (DO_MOTOR_TEST), that
 * allows spinning the motors for testing purposes.
 *
 * @boolean
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_MOT_TEST_EN, 1);

/**
 * Timeout value for disarming when kill switch is engaged
 *
 * @group Commander
 * @unit s
 * @min 0.0
 * @max 30.0
 * @increment 0.1
 */
PARAM_DEFINE_FLOAT(COM_KILL_DISARM, 5.0f);

/**
 * Maximum allowed CPU load to still arm
 *
 * A negative value disables the check.
 *
 * @group Commander
 * @unit %
 * @min -1
 * @max 100
 * @increment 1
 */
PARAM_DEFINE_FLOAT(COM_CPU_MAX, 90.0f);

/**
 * Required number of redundant power modules
 *
 * This configures a check to verify the expected number of 5V rail power supplies are present. By default only one is expected.
 * Note: CBRK_SUPPLY_CHK disables all power checks including this one.
 *
 * @group Commander
 * @min 0
 * @max 4
 */
PARAM_DEFINE_INT32(COM_POWER_COUNT, 1);

/**
 * Timeout for detecting a failure after takeoff
 *
 * A non-zero, positive value specifies the timeframe in seconds within failure detector is allowed to put the vehicle into
 * a lockdown state if attitude exceeds the limits defined in FD_FAIL_P and FD_FAIL_R.
 * The check is not executed for flight modes that do support acrobatic maneuvers, e.g: Acro (MC/FW) and Manual (FW).
 * A zero or negative value means that the check is disabled.
 *
 * @group Commander
 * @unit s
 * @min -1.0
 * @max 5.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(COM_LKDOWN_TKO, 3.0f);

/**
* Enable preflight check for maximal allowed airspeed when arming.
*
* Deny arming if the current airspeed measurement is greater than half the stall speed (ASPD_STALL).
* Excessive airspeed measurements on ground are either caused by wind or bad airspeed calibration.
*
* @group Commander
* @value 0 Disabled
* @value 1 Enabled
*/
PARAM_DEFINE_INT32(COM_ARM_ARSP_EN, 1);

/**
 * Enable FMU SD card detection check
 *
 * This check detects if the FMU SD card is missing.
 * Depending on the value of the parameter, the check can be
 * disabled, warn only or deny arming.
 *
 * @group Commander
 * @value 0 Disabled
 * @value 1 Warning only
 * @value 2 Enforce SD card presence
 */
PARAM_DEFINE_INT32(COM_ARM_SDCARD, 1);
