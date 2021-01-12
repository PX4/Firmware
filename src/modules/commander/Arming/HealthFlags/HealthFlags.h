/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file HealthFlags.h
 *
 * Contains helper functions to efficiently set the system health flags from commander and preflight check.
 *
 * @author Philipp Oettershagen (philipp.oettershagen@mavt.ethz.ch)
 */

#pragma once

#include <px4_platform_common/log.h>
#include <uORB/topics/vehicle_status.h>

struct subsystem_info_s {
	// keep in sync with mavlink MAV_SYS_STATUS_SENSOR
	static constexpr uint64_t SUBSYSTEM_TYPE_GYRO = 1 << 0;
	static constexpr uint64_t SUBSYSTEM_TYPE_ACC = 1 << 1;
	static constexpr uint64_t SUBSYSTEM_TYPE_MAG = 1 << 2;
	static constexpr uint64_t SUBSYSTEM_TYPE_ABSPRESSURE = 1 << 3;
	static constexpr uint64_t SUBSYSTEM_TYPE_DIFFPRESSURE = 1 << 4;
	static constexpr uint64_t SUBSYSTEM_TYPE_GPS = 1 << 5;
	static constexpr uint64_t SUBSYSTEM_TYPE_OPTICALFLOW = 1 << 6;
	static constexpr uint64_t SUBSYSTEM_TYPE_CVPOSITION = 1 << 7;
	static constexpr uint64_t SUBSYSTEM_TYPE_LASERPOSITION = 1 << 8;
	static constexpr uint64_t SUBSYSTEM_TYPE_EXTERNALGROUNDTRUTH = 1 << 9;
	static constexpr uint64_t SUBSYSTEM_TYPE_ANGULARRATECONTROL = 1 << 10;
	static constexpr uint64_t SUBSYSTEM_TYPE_ATTITUDESTABILIZATION = 1 << 11;
	static constexpr uint64_t SUBSYSTEM_TYPE_YAWPOSITION = 1 << 12;
	static constexpr uint64_t SUBSYSTEM_TYPE_ALTITUDECONTROL = 1 << 13;
	static constexpr uint64_t SUBSYSTEM_TYPE_POSITIONCONTROL = 1 << 14;
	static constexpr uint64_t SUBSYSTEM_TYPE_MOTORCONTROL = 1 << 15;
	static constexpr uint64_t SUBSYSTEM_TYPE_RCRECEIVER = 1 << 16;
	static constexpr uint64_t SUBSYSTEM_TYPE_GYRO2 = 1 << 17;
	static constexpr uint64_t SUBSYSTEM_TYPE_ACC2 = 1 << 18;
	static constexpr uint64_t SUBSYSTEM_TYPE_MAG2 = 1 << 19;
	static constexpr uint64_t SUBSYSTEM_TYPE_GEOFENCE = 1 << 20;
	static constexpr uint64_t SUBSYSTEM_TYPE_AHRS = 1 << 21;
	static constexpr uint64_t SUBSYSTEM_TYPE_TERRAIN = 1 << 22;
	static constexpr uint64_t SUBSYSTEM_TYPE_REVERSEMOTOR = 1 << 23;
	static constexpr uint64_t SUBSYSTEM_TYPE_LOGGING = 1 << 24;
	static constexpr uint64_t SUBSYSTEM_TYPE_SENSORBATTERY = 1 << 25;
	static constexpr uint64_t SUBSYSTEM_TYPE_SENSORPROXIMITY = 1 << 26;
	static constexpr uint64_t SUBSYSTEM_TYPE_SATCOM = 1 << 27;
	static constexpr uint64_t SUBSYSTEM_TYPE_PREARM_CHECK = 1 << 28;
	static constexpr uint64_t SUBSYSTEM_TYPE_OBSTACLE_AVOIDANCE = 1 << 29;
};

void set_health_flags(uint64_t subsystem_type, bool present, bool enabled, bool ok, vehicle_status_s &status);
void set_health_flags_present_healthy(uint64_t subsystem_type, bool present, bool healthy, vehicle_status_s &status);
void set_health_flags_healthy(uint64_t subsystem_type, bool healthy, vehicle_status_s &status);
void print_health_flags(const vehicle_status_s &status);
