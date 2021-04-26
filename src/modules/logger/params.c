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
 * UTC offset (unit: min)
 *
 * the difference in hours and minutes from Coordinated
 * Universal Time (UTC) for a your place and date.
 *
 * for example, In case of South Korea(UTC+09:00),
 * UTC offset is 540 min (9*60)
 *
 * refer to https://en.wikipedia.org/wiki/List_of_UTC_time_offsets
 *
 * @unit min
 * @min -1000
 * @max  1000
 * @group SD Logging
 */
PARAM_DEFINE_INT32(SDLOG_UTC_OFFSET, 0);

/**
 * Logging Mode
 *
 * Determines when to start and stop logging. By default, logging is started
 * when arming the system, and stopped when disarming.
 *
 * @value -1 disabled
 * @value 0 when armed until disarm (default)
 * @value 1 from boot until disarm
 * @value 2 from boot until shutdown
 * @value 3 depending on AUX1 RC channel
 *
 * @reboot_required true
 * @group SD Logging
 */
PARAM_DEFINE_INT32(SDLOG_MODE, 0);

/**
 * Battery-only Logging
 *
 * When enabled, logging will not start from boot if battery power is not detected
 * (e.g. powered via USB on a test bench). This prevents extraneous flight logs from
 * being created during bench testing.
 *
 * Note that this only applies to log-from-boot modes. This has no effect on arm-based
 * modes.
 *
 * @boolean
 * @group SD Logging
 */
PARAM_DEFINE_INT32(SDLOG_BOOT_BAT, 0);

/**
 * Mission Log
 *
 * If enabled, a small additional "mission" log file will be written to the SD card.
 * The log contains just those messages that are useful for tasks like
 * generating flight statistics and geotagging.
 *
 * The different modes can be used to further reduce the logged data
 * (and thus the log file size). For example, choose geotagging mode to
 * only log data required for geotagging.

 * Note that the normal/full log is still created, and contains all
 * the data in the mission log (and more).
 *
 * @value 0 Disabled
 * @value 1 All mission messages
 * @value 2 Geotagging messages
 *
 * @reboot_required true
 * @group SD Logging
 */
PARAM_DEFINE_INT32(SDLOG_MISSION, 0);

/**
 * Logging topic profile (integer bitmask).
 *
 * This integer bitmask controls the set and rates of logged topics.
 * The default allows for general log analysis while keeping the
 * log file size reasonably small.
 *
 * Enabling multiple sets leads to higher bandwidth requirements and larger log
 * files.
 *
 * Set bits true to enable:
 * 0 : Default set (used for general log analysis)
 * 1 : Full rate estimator (EKF2) replay topics
 * 2 : Topics for thermal calibration (high rate raw IMU and Baro sensor data)
 * 3 : Topics for system identification (high rate actuator control and IMU data)
 * 4 : Full rates for analysis of fast maneuvers (RC, attitude, rates and actuators)
 * 5 : Debugging topics (debug_*.msg topics, for custom code)
 * 6 : Topics for sensor comparison (low rate raw IMU, Baro and Magnetomer data)
 * 7 : Topics for computer vision and collision avoidance
 * 8 : Raw FIFO high-rate IMU (Gyro)
 * 9 : Raw FIFO high-rate IMU (Accel)
 *
 * @min 0
 * @max 1023
 * @bit 0 Default set (general log analysis)
 * @bit 1 Estimator replay (EKF2)
 * @bit 2 Thermal calibration
 * @bit 3 System identification
 * @bit 4 High rate
 * @bit 5 Debug
 * @bit 6 Sensor comparison
 * @bit 7 Computer Vision and Avoidance
 * @bit 8 Raw FIFO high-rate IMU (Gyro)
 * @bit 9 Raw FIFO high-rate IMU (Accel)
 * @reboot_required true
 * @group SD Logging
 */
PARAM_DEFINE_INT32(SDLOG_PROFILE, 1);

/**
 * Maximum number of log directories to keep
 *
 * If there are more log directories than this value,
 * the system will delete the oldest directories during startup.
 *
 * In addition, the system will delete old logs if there is not enough free space left.
 * The minimum amount is 300 MB.
 *
 * If this is set to 0, old directories will only be removed if the free space falls below
 * the minimum.
 *
 * Note: this does not apply to mission log files.
 *
 * @min 0
 * @max 1000
 * @reboot_required true
 * @group SD Logging
 */
PARAM_DEFINE_INT32(SDLOG_DIRS_MAX, 0);

/**
 * Log UUID
 *
 * If set to 1, add an ID to the log, which uniquely identifies the vehicle
 *
 * @boolean
 * @group SD Logging
 */
PARAM_DEFINE_INT32(SDLOG_UUID, 1);
