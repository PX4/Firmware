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
 * After a data link loss: after this this amount of seconds with a healthy datalink the 'datalink loss'
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
 * After this amount of seconds without RC connection the rc lost flag is set to true
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
 * RC stick override threshold
 *
 * If an RC stick is moved more than by this amount the system will interpret this as
 * override request by the pilot.
 *
 * @group Commander
 * @unit %
 * @min 5
 * @max 40
 * @decimal 0
 * @increment 0.05
 */
PARAM_DEFINE_FLOAT(COM_RC_STICK_OV, 12.0f);

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
 * Time-out for auto disarm if too slow to takeoff
 *
 * A non-zero, positive value specifies the time after arming, in seconds, within which the
 * vehicle must take off (after which it will automatically disarm).
 *
 * A zero or negative value means that automatic disarming triggered by a pre-takeoff timeout is disabled.
 *
 * @group Commander
 * @unit s
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(COM_DISARM_PRFLT,  10.0f);


/**
 * Allow arming without GPS
 *
 * The default allows to arm the vehicle without GPS signal.
 *
 * @group Commander
 * @boolean
 */
PARAM_DEFINE_INT32(COM_ARM_WO_GPS, 1);

/**
 * Arm switch is only a button
 *
 * The default uses the arm switch as real switch.
 * If parameter set button gets handled like stick arming.
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
 * See COM_OBL_ACT and COM_OBL_RC_ACT to configure action.
 *
 * @group Commander
 * @unit s
 * @min 0
 * @max 60
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(COM_OF_LOSS_T, 0.5f);

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
 * @value 9 Rattitude
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
 * @value 9 Rattitude
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
 * @value 9 Rattitude
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
 * @value 9 Rattitude
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
 * @value 9 Rattitude
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
 * @value 9 Rattitude
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
 * Maximum value of EKF accelerometer delta velocity bias estimate that will allow arming.
 * Note: ekf2 will limit the delta velocity bias estimate magnitude to be less than EKF2_ABL_LIM * FILTER_UPDATE_PERIOD_MS * 0.001 so this parameter must be less than that to be useful.
 *
 * @group Commander
 * @unit m/s
 * @min 0.001
 * @max 0.01
 * @decimal 4
 * @increment 0.0001
 */
PARAM_DEFINE_FLOAT(COM_ARM_EKF_AB, 1.73e-3f);

/**
 * Maximum value of EKF gyro delta angle bias estimate that will allow arming
 *
 * @group Commander
 * @unit rad
 * @min 0.0001
 * @max 0.0017
 * @decimal 5
 * @increment 0.0001
 */
PARAM_DEFINE_FLOAT(COM_ARM_EKF_GB, 8.7e-4f);

/**
 * Maximum accelerometer inconsistency between IMU units that will allow arming
 *
 * @group Commander
 * @unit m/s/s
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
 * Set -1 to disable the check.
 *
 * @group Commander
 * @unit deg
 * @min 3
 * @max 180
 */
PARAM_DEFINE_INT32(COM_ARM_MAG_ANG, 30);

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
 * Enable RC stick override of auto and/or offboard modes
 *
 * When RC stick override is enabled, moving the RC sticks immediately gives control back
 * to the pilot (switches to manual position mode):
 * bit 0: Enable for auto modes (except for in critical battery reaction),
 * bit 1: Enable for offboard mode.
 *
 * Only has an effect on multicopters, and VTOLS in multicopter mode.
 *
 * @min 0
 * @max 3
 * @bit 0  Enable override in auto modes
 * @bit 1  Enable override in offboard mode
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_RC_OVERRIDE, 1);

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
 * @value 1 Land/Terminate.  Assume no use of remote control after fallback. Switch to Land mode if a height estimate is available, else switch to TERMINATION.
 *
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_POSCTL_NAVL, 0);

/**
 * Arm authorization parameters, this uint32_t will be split between starting from the LSB:
 * - 8bits to authorizer system id
 * - 16bits to authentication method parameter, this will be used to store a timeout for the first 2 methods but can be used to another parameter for other new authentication methods.
 * - 7bits to authentication method
 * 		- one arm = 0
 * 		- two step arm = 1
 * * the MSB bit is not used to avoid problems in the conversion between int and uint
 *
 * Default value: (10 << 0 | 1000 << 8 | 0 << 24) = 256010
 * - authorizer system id = 10
 * - authentication method parameter = 10000msec of timeout
 * - authentication method = during arm
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_ARM_AUTH, 256010);

/**
 * Require arm authorization to arm
 *
 * The default allows to arm the vehicle without a arm authorization.
 *
 * @group Commander
 * @boolean
 */
PARAM_DEFINE_INT32(COM_ARM_AUTH_REQ, 0);

/**
 * Loss of position failsafe activation delay.
 *
 * This sets number of seconds that the position checks need to be failed before the failsafe will activate.
 * The default value has been optimised for rotary wing applications. For fixed wing applications, a larger value between 5 and 10 should be used.
 *
 * @unit sec
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
 * @unit sec
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
 * Require all the ESCs to be detected to arm.
 *
 * This param is specific for ESCs reporting status. Normal ESCs configurations are not affected by the change of this param.
 *
 * @group Commander
 * @boolean
 */
PARAM_DEFINE_INT32(COM_ARM_CHK_ESCS, 1);

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
PARAM_DEFINE_INT32(COM_PREARM_MODE, 1);

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
