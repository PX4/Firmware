/****************************************************************************
*
*   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
* @file PreflightCheck.cpp
*
* Preflight check for main system components
*
* @author Lorenz Meier <lorenz@px4.io>
* @author Johan Jansen <jnsn.johan@gmail.com>
*/

#include <px4_config.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>

#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/rc_check.h>
#include <systemlib/mavlink_log.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_airspeed.h>

#include <uORB/topics/airspeed.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/ekf2_innovations.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/sensor_combined.h>

#include "PreflightCheck.h"

#include "DevMgr.hpp"

using namespace DriverFramework;

namespace Commander
{

static int check_calibration(DevHandle &h, const char* param_template, int &devid)
{
	bool calibration_found;

	/* new style: ask device for calibration state */
	int ret = h.ioctl(SENSORIOCCALTEST, 0);

	calibration_found = (ret == OK);

	devid = h.ioctl(DEVIOCGDEVICEID, 0);

	char s[20];
	int instance = 0;

	/* old style transition: check param values */
	while (!calibration_found) {
		sprintf(s, param_template, instance);
		param_t parm = param_find(s);

		/* if the calibration param is not present, abort */
		if (parm == PARAM_INVALID) {
			break;
		}

		/* if param get succeeds */
		int calibration_devid;
		if (!param_get(parm, &(calibration_devid))) {

			/* if the devid matches, exit early */
			if (devid == calibration_devid) {
				calibration_found = true;
				break;
			}
		}
		instance++;
	}

	return !calibration_found;
}

static bool magnometerCheck(orb_advert_t *mavlink_log_pub, unsigned instance, bool optional, int &device_id, bool report_fail)
{
	bool success = true;

	char s[30];
	sprintf(s, "%s%u", MAG_BASE_DEVICE_PATH, instance);
	DevHandle h;
	DevMgr::getHandle(s, h);

	if (!h.isValid()) {
		if (!optional) {
			if (report_fail) {
				mavlink_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: NO MAG SENSOR #%u", instance);
			}
		}

		return false;
	}

	int ret = check_calibration(h, "CAL_MAG%u_ID", device_id);

	if (ret) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: MAG #%u UNCALIBRATED", instance);
		}
		success = false;
		goto out;
	}

	ret = h.ioctl(MAGIOCSELFTEST, 0);

	if (ret != OK) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: MAG #%u SELFTEST FAILED", instance);
		}
		success = false;
		goto out;
	}

out:
	DevMgr::releaseHandle(h);
	return success;
}

static bool imuConsistencyCheck(orb_advert_t *mavlink_log_pub, bool checkAcc, bool checkGyro, bool report_status)
{
	// get the sensor combined data
	int sensors_sub = orb_subscribe(ORB_ID(sensor_combined));
	struct sensor_combined_s sensors = {};
	orb_copy(ORB_ID(sensor_combined), sensors_sub, &sensors);
	px4_close(sensors_sub);

	// Use the difference between IMU's to detect a bad calibration. If a single IMU is fitted, the value being checked will be zero so this check will always pass.
	// Fail if accel difference greater than 0.7 m/s/s and notify if greater than 0.35 m/s/s
	bool success = true;
	float test_limit;
	param_get(param_find("COM_ARM_IMU_ACC"), &test_limit);
	if (checkAcc) {
		if (sensors.accel_inconsistency_m_s_s > test_limit) {
			if (report_status) {
				mavlink_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: ACCEL SENSORS INCONSISTENT - CHECK CALIBRATION");
			}
			success = false;
			goto out;

		} else if (sensors.accel_inconsistency_m_s_s > test_limit * 0.5f) {
			if (report_status) {
				mavlink_log_info(mavlink_log_pub, "PREFLIGHT ADVICE: ACCEL SENSORS INCONSISTENT - CHECK CALIBRATION");

			}
		}
	}
	// Fail if gyro difference greater than 5 deg/sec and notify if greater than 2.5 deg/sec
	param_get(param_find("COM_ARM_IMU_GYR"), &test_limit);
	if (checkGyro) {
		if (sensors.gyro_inconsistency_rad_s > test_limit) {
			if (report_status) {
				mavlink_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: GYRO SENSORS INCONSISTENT - CHECK CALIBRATION");
			}
			success = false;
			goto out;

		} else if (sensors.gyro_inconsistency_rad_s > test_limit * 0.5f) {
			if (report_status) {
				mavlink_log_info(mavlink_log_pub, "PREFLIGHT ADVICE: GYRO SENSORS INCONSISTENT - CHECK CALIBRATION");

			}
		}
	}

out:
	return success;
}

static bool accelerometerCheck(orb_advert_t *mavlink_log_pub, unsigned instance, bool optional, bool dynamic, int &device_id, bool report_fail)
{
	bool success = true;

	char s[30];
	sprintf(s, "%s%u", ACCEL_BASE_DEVICE_PATH, instance);
	DevHandle h;
	DevMgr::getHandle(s, h);

	if (!h.isValid()) {
		if (!optional) {
			if (report_fail) {
				mavlink_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: NO ACCEL SENSOR #%u", instance);
			}
		}

		return false;
	}

	int ret = check_calibration(h, "CAL_ACC%u_ID", device_id);

	if (ret) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: ACCEL #%u UNCALIBRATED", instance);
		}
		success = false;
		goto out;
	}

	ret = h.ioctl(ACCELIOCSELFTEST, 0);

	if (ret != OK) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: ACCEL #%u TEST FAILED: %d", instance, ret);
		}
		success = false;
		goto out;
	}

#ifdef __PX4_NUTTX
	if (dynamic) {
		/* check measurement result range */
		struct accel_report acc;
		ret = h.read(&acc, sizeof(acc));

		if (ret == sizeof(acc)) {
			/* evaluate values */
			float accel_magnitude = sqrtf(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);

			if (accel_magnitude < 4.0f || accel_magnitude > 15.0f /* m/s^2 */) {
				if (report_fail) {
					mavlink_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: ACCEL RANGE, hold still on arming");
				}
				/* this is frickin' fatal */
				success = false;
				goto out;
			}
		} else {
			if (report_fail) {
				mavlink_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: ACCEL READ");
			}
			/* this is frickin' fatal */
			success = false;
			goto out;
		}
	}
#endif

out:
	DevMgr::releaseHandle(h);
	return success;
}

static bool gyroCheck(orb_advert_t *mavlink_log_pub, unsigned instance, bool optional, int &device_id, bool report_fail)
{
	bool success = true;

	char s[30];
	sprintf(s, "%s%u", GYRO_BASE_DEVICE_PATH, instance);
	DevHandle h;
	DevMgr::getHandle(s, h);

	if (!h.isValid()) {
		if (!optional) {
			if (report_fail) {
				mavlink_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: NO GYRO SENSOR #%u", instance);
			}
		}

		return false;
	}

	int ret = check_calibration(h, "CAL_GYRO%u_ID", device_id);

	if (ret) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: GYRO #%u UNCALIBRATED", instance);
		}
		success = false;
		goto out;
	}

	ret = h.ioctl(GYROIOCSELFTEST, 0);

	if (ret != OK) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: GYRO #%u SELFTEST FAILED", instance);
		}
		success = false;
		goto out;
	}

out:
	DevMgr::releaseHandle(h);
	return success;
}

static bool baroCheck(orb_advert_t *mavlink_log_pub, unsigned instance, bool optional, int &device_id, bool report_fail)
{
	bool success = true;

	char s[30];
	sprintf(s, "%s%u", BARO_BASE_DEVICE_PATH, instance);
	DevHandle h;
	DevMgr::getHandle(s, h);

	if (!h.isValid()) {
		if (!optional) {
			if (report_fail) {
				mavlink_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: NO BARO SENSOR #%u", instance);
			}
		}

		return false;
	}

	device_id = -1000;

	// TODO: There is no baro calibration yet, since no external baros exist
	// int ret = check_calibration(fd, "CAL_BARO%u_ID");

	// if (ret) {
	// 	mavlink_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: BARO #%u UNCALIBRATED", instance);
	// 	success = false;
	// 	goto out;
	// }

//out:

	DevMgr::releaseHandle(h);
	return success;
}

static bool airspeedCheck(orb_advert_t *mavlink_log_pub, bool optional, bool report_fail)
{
	bool success = true;
	int ret;
	int fd = orb_subscribe(ORB_ID(airspeed));

	struct airspeed_s airspeed;

	if ((ret = orb_copy(ORB_ID(airspeed), fd, &airspeed)) ||
	    (hrt_elapsed_time(&airspeed.timestamp) > (500 * 1000))) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: AIRSPEED SENSOR MISSING");
		}
		success = false;
		goto out;
	}

	if (fabsf(airspeed.confidence) < 0.99f) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: AIRSPEED SENSOR COMM ERROR");
		}
		success = false;
		goto out;
	}

	if (fabsf(airspeed.indicated_airspeed_m_s) > 6.0f) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "AIRSPEED WARNING: WIND OR CALIBRATION ISSUE");
		}
		// XXX do not make this fatal yet
	}

out:
	px4_close(fd);
	return success;
}

static bool gnssCheck(orb_advert_t *mavlink_log_pub, bool report_fail)
{
	bool success = true;

	int gpsSub = orb_subscribe(ORB_ID(vehicle_gps_position));

	//Wait up to 2000ms to allow the driver to detect a GNSS receiver module
	px4_pollfd_struct_t fds[1];
	fds[0].fd = gpsSub;
	fds[0].events = POLLIN;
	if(px4_poll(fds, 1, 2000) <= 0) {
		success = false;
	}
	else {
		struct vehicle_gps_position_s gps;
		if ( (OK != orb_copy(ORB_ID(vehicle_gps_position), gpsSub, &gps)) ||
		    (hrt_elapsed_time(&gps.timestamp) > 1000000)) {
			success = false;
		}
	}

	//Report failure to detect module
	if (!success) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: GPS RECEIVER MISSING");
		}
	}

	px4_close(gpsSub);
	return success;
}

static bool ekfCheck(orb_advert_t *mavlink_log_pub, bool optional, bool report_fail)
{
	bool success = true;

	int fd1 = orb_subscribe(ORB_ID(ekf2_innovations));
	struct ekf2_innovations_s innovations;
	orb_copy(ORB_ID(ekf2_innovations), fd1, &innovations);
	px4_close(fd1);

	int fd2 = orb_subscribe(ORB_ID(estimator_status));
	struct estimator_status_s status;
	orb_copy(ORB_ID(estimator_status), fd1, &status);
	px4_close(fd2);

	float test_limit;
	param_get(param_find("COM_ARM_EKF_VD"), &test_limit);
	// check vertical velocity innovation
	if (fabsf(innovations.vel_pos_innov[2]) > test_limit) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: EKF VERT VEL INNOVATIONS");
		}
		success = false;
		goto out;
	}

	// check vertical position innovation
	param_get(param_find("COM_ARM_EKF_PD"), &test_limit);
	if (fabsf(innovations.vel_pos_innov[5]) > test_limit) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: EKF VERT POS INNOVATIONS");
		}
		success = false;
		goto out;
	}

	// check horizontal velocity innovations
	param_get(param_find("COM_ARM_EKF_VH"), &test_limit);
	if ((innovations.vel_pos_innov[0]*innovations.vel_pos_innov[0] + innovations.vel_pos_innov[1]*innovations.vel_pos_innov[1]) > test_limit) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: EKF HORIZ VEL INNOVATIONS");
		}
		success = false;
		goto out;
	}

	// check horizontal position innovations
	param_get(param_find("COM_ARM_EKF_PH"), &test_limit);
	if ((innovations.vel_pos_innov[3]*innovations.vel_pos_innov[3] + innovations.vel_pos_innov[4]*innovations.vel_pos_innov[4]) > test_limit) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: EKF HORIZ POS INNOVATIONS");
		}
		success = false;
		goto out;
	}

	// check yaw innovation
	param_get(param_find("COM_ARM_EKF_YAW"), &test_limit);
	if (fabsf(innovations.heading_innov) > test_limit) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: EKF YAW INNOVATION");
		}
		success = false;
		goto out;
	}

	// check accelerometer delta velocity bias estimates
	param_get(param_find("COM_ARM_IMU_AB"), &test_limit);
	if (fabsf(status.states[13]) > test_limit ||  fabsf(status.states[14]) > test_limit || fabsf(status.states[15]) > test_limit) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: EKF HIGH IMU ACCEL BIAS");
		}
		success = false;
		goto out;
	}

	// check gyro delta angle bias estimates
	param_get(param_find("COM_ARM_IMU_GB"), &test_limit);
	if (fabsf(status.states[10]) > test_limit ||  fabsf(status.states[11]) > test_limit || fabsf(status.states[12]) > test_limit) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "PREFLIGHT FAIL: EKF HIGH IMU GYRO BIAS");
		}
		success = false;
		goto out;
	}

out:
	return success;
}

bool preflightCheck(orb_advert_t *mavlink_log_pub, bool checkMag, bool checkAcc, bool checkGyro,
		    bool checkBaro, bool checkAirspeed, bool checkRC, bool checkGNSS, bool checkDynamic, bool isVTOL, bool reportFailures)
{

#ifdef __PX4_QURT
	// WARNING: Preflight checks are important and should be added back when
	// all the sensors are supported
	PX4_WARN("Preflight checks always pass on Snapdragon.");
	return true;
#elif defined(__PX4_POSIX_RPI)
	PX4_WARN("Preflight checks always pass on RPI.");
	return true;
#elif defined(__PX4_POSIX_BEBOP)
	PX4_WARN("Preflight checks always pass on Bebop.");
	return true;
#endif

	bool failed = false;

	/* ---- MAG ---- */
	if (checkMag) {
		bool prime_found = false;
		int32_t prime_id = 0;
		param_get(param_find("CAL_MAG_PRIME"), &prime_id);

		/* check all sensors, but fail only for mandatory ones */
		for (unsigned i = 0; i < max_optional_mag_count; i++) {
			bool required = (i < max_mandatory_mag_count);
			int device_id = -1;

			if (!magnometerCheck(mavlink_log_pub, i, !required, device_id, reportFailures) && required) {
				failed = true;
			}

			if (device_id == prime_id) {
				prime_found = true;
			}
		}

		/* check if the primary device is present */
		if (!prime_found && prime_id != 0) {
			if (reportFailures) {
				mavlink_log_critical(mavlink_log_pub, "Warning: Primary compass not found");
			}
			failed = true;
		}
	}

	/* ---- ACCEL ---- */
	if (checkAcc) {
		bool prime_found = false;
		int32_t prime_id = 0;
		param_get(param_find("CAL_ACC_PRIME"), &prime_id);

		/* check all sensors, but fail only for mandatory ones */
		for (unsigned i = 0; i < max_optional_accel_count; i++) {
			bool required = (i < max_mandatory_accel_count);
			int device_id = -1;

			if (!accelerometerCheck(mavlink_log_pub, i, !required, checkDynamic, device_id, reportFailures) && required) {
				failed = true;
			}

			if (device_id == prime_id) {
				prime_found = true;
			}
		}

		/* check if the primary device is present */
		if (!prime_found && prime_id != 0) {
			if (reportFailures) {
				mavlink_log_critical(mavlink_log_pub, "Warning: Primary accelerometer not found");
			}
			failed = true;
		}
	}

	/* ---- GYRO ---- */
	if (checkGyro) {
		bool prime_found = false;
		int32_t prime_id = 0;
		param_get(param_find("CAL_GYRO_PRIME"), &prime_id);

		/* check all sensors, but fail only for mandatory ones */
		for (unsigned i = 0; i < max_optional_gyro_count; i++) {
			bool required = (i < max_mandatory_gyro_count);
			int device_id = -1;

			if (!gyroCheck(mavlink_log_pub, i, !required, device_id, reportFailures) && required) {
				failed = true;
			}

			if (device_id == prime_id) {
				prime_found = true;
			}
		}

		/* check if the primary device is present */
		if (!prime_found && prime_id != 0) {
			if (reportFailures) {
				mavlink_log_critical(mavlink_log_pub, "Warning: Primary gyro not found");
			}
			failed = true;
		}
	}

	/* ---- BARO ---- */
	if (checkBaro) {
		bool prime_found = false;
		int32_t prime_id = 0;
		param_get(param_find("CAL_BARO_PRIME"), &prime_id);

		/* check all sensors, but fail only for mandatory ones */
		for (unsigned i = 0; i < max_optional_baro_count; i++) {
			bool required = (i < max_mandatory_baro_count);
			int device_id = -1;

			if (!baroCheck(mavlink_log_pub, i, !required, device_id, reportFailures) && required) {
				failed = true;
			}

			if (device_id == prime_id) {
				prime_found = true;
			}
		}

		// TODO there is no logic in place to calibrate the primary baro yet
		// // check if the primary device is present
		if (!prime_found && prime_id != 0) {
			if (reportFailures) {
				mavlink_log_critical(mavlink_log_pub, "warning: primary barometer not operational");
			}
			failed = true;
		}
	}

	/* ---- IMU CONSISTENCY ---- */
	imuConsistencyCheck(mavlink_log_pub, checkAcc, checkGyro, reportFailures);

	/* ---- AIRSPEED ---- */
	if (checkAirspeed) {
		if (!airspeedCheck(mavlink_log_pub, true, reportFailures)) {
			failed = true;
		}
	}

	/* ---- RC CALIBRATION ---- */
	if (checkRC) {
		if (rc_calibration_check(mavlink_log_pub, reportFailures, isVTOL) != OK) {
			if (reportFailures) {
				mavlink_log_critical(mavlink_log_pub, "RC calibration check failed");
			}
			failed = true;
		}
	}

	/* ---- Global Navigation Satellite System receiver ---- */
	if (checkGNSS) {
		if (!gnssCheck(mavlink_log_pub, reportFailures)) {
			failed = true;
		}
	}

	/* ---- Navigation EKF ---- */
	if (checkGNSS) {
		if (!ekfCheck(mavlink_log_pub, true, reportFailures)) {
			failed = true;
		}
	}

	/* Report status */
	return !failed;
}

}
