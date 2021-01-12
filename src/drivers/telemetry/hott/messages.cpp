/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Simon Wilks <sjwilks@gmail.com>
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
 * @file messages.c
 *
 */

#include "messages.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <lib/ecl/geo/geo.h>
#include <unistd.h>
#include <px4_platform_common/defines.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_gps_position.h>

#include <drivers/drv_hrt.h>

/* The board is very roughly 5 deg warmer than the surrounding air */
#define BOARD_TEMP_OFFSET_DEG 5

static int _battery_sub = -1;
static int _gps_sub = -1;
static int _home_sub = -1;
static int _airdata_sub = -1;
static int _airspeed_sub = -1;
static int _esc_sub = -1;

static orb_advert_t _esc_pub = nullptr;

static bool _home_position_set = false;
static double _home_lat = 0.0d;
static double _home_lon = 0.0d;

void
init_sub_messages(void)
{
	_battery_sub = orb_subscribe(ORB_ID(battery_status));
	_gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	_home_sub = orb_subscribe(ORB_ID(home_position));
	_airdata_sub = orb_subscribe(ORB_ID(vehicle_air_data));
	_airspeed_sub = orb_subscribe(ORB_ID(airspeed));
	_esc_sub = orb_subscribe(ORB_ID(esc_status));
}

void
init_pub_messages(void)
{
}

void
build_gam_request(uint8_t *buffer, size_t *size)
{
	struct gam_module_poll_msg msg;
	*size = sizeof(msg);
	memset(&msg, 0, *size);

	msg.mode = BINARY_MODE_REQUEST_ID;
	msg.id = GAM_SENSOR_ID;

	memcpy(buffer, &msg, *size);
}

void
publish_gam_message(const uint8_t *buffer)
{
	struct gam_module_msg msg;
	size_t size = sizeof(msg);
	memset(&msg, 0, size);
	memcpy(&msg, buffer, size);
	struct esc_status_s esc;
	memset(&esc, 0, sizeof(esc));

	// Publish it.
	esc.timestamp = hrt_absolute_time();
	esc.esc_count = 1;
	esc.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_PPM;

	esc.esc[0].esc_rpm = (uint16_t)((msg.rpm_H << 8) | (msg.rpm_L & 0xff)) * 10;
	esc.esc[0].esc_temperature = msg.temperature1 - 20;
	esc.esc[0].esc_voltage = static_cast<float>((msg.main_voltage_H << 8) | (msg.main_voltage_L & 0xff)) * 0.1F;
	esc.esc[0].esc_current = static_cast<float>((msg.current_H << 8) | (msg.current_L & 0xff)) * 0.1F;

	/* announce the esc if needed, just publish else */
	if (_esc_pub != nullptr) {
		orb_publish(ORB_ID(esc_status), _esc_pub, &esc);

	} else {
		_esc_pub = orb_advertise(ORB_ID(esc_status), &esc);
	}
}

void
build_eam_response(uint8_t *buffer, size_t *size)
{
	/* get a local copy of the current sensor values */
	vehicle_air_data_s airdata = {};
	orb_copy(ORB_ID(vehicle_air_data), _airdata_sub, &airdata);

	/* get a local copy of the battery data */
	struct battery_status_s battery;
	memset(&battery, 0, sizeof(battery));
	orb_copy(ORB_ID(battery_status), _battery_sub, &battery);

	struct eam_module_msg msg;
	*size = sizeof(msg);
	memset(&msg, 0, *size);

	msg.start = START_BYTE;
	msg.eam_sensor_id = EAM_SENSOR_ID;
	msg.sensor_text_id = EAM_SENSOR_TEXT_ID;

	msg.temperature1 = (uint8_t)(airdata.baro_temp_celcius + 20);
	msg.temperature2 = msg.temperature1 - BOARD_TEMP_OFFSET_DEG;

	msg.main_voltage_L = (uint8_t)(battery.voltage_v * 10);

	uint16_t alt = (uint16_t)(airdata.baro_alt_meter + 500);
	msg.altitude_L = (uint8_t)alt & 0xff;
	msg.altitude_H = (uint8_t)(alt >> 8) & 0xff;

	/* get a local copy of the airspeed data */
	struct airspeed_s airspeed;
	memset(&airspeed, 0, sizeof(airspeed));
	orb_copy(ORB_ID(airspeed), _airspeed_sub, &airspeed);

	uint16_t speed = (uint16_t)(airspeed.indicated_airspeed_m_s * 3.6f);
	msg.speed_L = (uint8_t)speed & 0xff;
	msg.speed_H = (uint8_t)(speed >> 8) & 0xff;

	msg.stop = STOP_BYTE;
	memcpy(buffer, &msg, *size);
}

void
build_gam_response(uint8_t *buffer, size_t *size)
{
	/* get a local copy of the ESC Status values */
	struct esc_status_s esc;
	memset(&esc, 0, sizeof(esc));
	orb_copy(ORB_ID(esc_status), _esc_sub, &esc);

	struct gam_module_msg msg;
	*size = sizeof(msg);
	memset(&msg, 0, *size);

	msg.start = START_BYTE;
	msg.gam_sensor_id = GAM_SENSOR_ID;
	msg.sensor_text_id = GAM_SENSOR_TEXT_ID;

	msg.temperature1 = esc.esc[0].esc_temperature + 20;
	msg.temperature2 = 20;  // 0 deg. C.

	const uint16_t voltage = (uint16_t)(esc.esc[0].esc_voltage * 10.0F);
	msg.main_voltage_L = (uint8_t)voltage & 0xff;
	msg.main_voltage_H = (uint8_t)(voltage >> 8) & 0xff;

	const uint16_t current = (uint16_t)(esc.esc[0].esc_current * 10.0F);
	msg.current_L = (uint8_t)current & 0xff;
	msg.current_H = (uint8_t)(current >> 8) & 0xff;

	const uint16_t rpm = (uint16_t)(esc.esc[0].esc_rpm * 0.1f);
	msg.rpm_L = (uint8_t)rpm & 0xff;
	msg.rpm_H = (uint8_t)(rpm >> 8) & 0xff;

	msg.stop = STOP_BYTE;
	memcpy(buffer, &msg, *size);
}

void
build_gps_response(uint8_t *buffer, size_t *size)
{
	/* get a local copy of the battery data */
	struct vehicle_gps_position_s gps;
	memset(&gps, 0, sizeof(gps));
	orb_copy(ORB_ID(vehicle_gps_position), _gps_sub, &gps);

	struct gps_module_msg msg;
	*size = sizeof(msg);
	memset(&msg, 0, *size);

	msg.start = START_BYTE;
	msg.sensor_id = GPS_SENSOR_ID;
	msg.sensor_text_id = GPS_SENSOR_TEXT_ID;

	msg.gps_num_sat = gps.satellites_used;

	/* The GPS fix type: 0 = none, 2 = 2D, 3 = 3D */
	msg.gps_fix_char = (uint8_t)(gps.fix_type + 48);
	msg.gps_fix = (uint8_t)(gps.fix_type + 48);

	/* No point collecting more data if we don't have a 3D fix yet */
	if (gps.fix_type > 2) {
		/* Current flight direction */
		msg.flight_direction = (uint8_t)(gps.cog_rad * M_RAD_TO_DEG_F);

		/* GPS speed */
		uint16_t speed = (uint16_t)(gps.vel_m_s * 3.6f);
		msg.gps_speed_L = (uint8_t)speed & 0xff;
		msg.gps_speed_H = (uint8_t)(speed >> 8) & 0xff;

		/* Get latitude in degrees, minutes and seconds */
		double lat = ((double)(gps.lat)) * 1e-7d;

		/* Set the N or S specifier */
		msg.latitude_ns = 0;

		if (lat < 0) {
			msg.latitude_ns = 1;
			lat = abs(lat);
		}

		int deg;
		int min;
		int sec;
		convert_to_degrees_minutes_seconds(lat, &deg, &min, &sec);

		uint16_t lat_min = (uint16_t)(deg * 100 + min);
		msg.latitude_min_L = (uint8_t)lat_min & 0xff;
		msg.latitude_min_H = (uint8_t)(lat_min >> 8) & 0xff;
		uint16_t lat_sec = (uint16_t)(sec);
		msg.latitude_sec_L = (uint8_t)lat_sec & 0xff;
		msg.latitude_sec_H = (uint8_t)(lat_sec >> 8) & 0xff;

		/* Get longitude in degrees, minutes and seconds */
		double lon = ((double)(gps.lon)) * 1e-7d;

		/* Set the E or W specifier */
		msg.longitude_ew = 0;

		if (lon < 0) {
			msg.longitude_ew = 1;
			lon = abs(lon);
		}

		convert_to_degrees_minutes_seconds(lon, &deg, &min, &sec);

		uint16_t lon_min = (uint16_t)(deg * 100 + min);
		msg.longitude_min_L = (uint8_t)lon_min & 0xff;
		msg.longitude_min_H = (uint8_t)(lon_min >> 8) & 0xff;
		uint16_t lon_sec = (uint16_t)(sec);
		msg.longitude_sec_L = (uint8_t)lon_sec & 0xff;
		msg.longitude_sec_H = (uint8_t)(lon_sec >> 8) & 0xff;

		/* Altitude */
		uint16_t alt = (uint16_t)(gps.alt * 1e-3f + 500.0f);
		msg.altitude_L = (uint8_t)alt & 0xff;
		msg.altitude_H = (uint8_t)(alt >> 8) & 0xff;

		/* Get any (and probably only ever one) _home_sub position report */
		bool updated;
		orb_check(_home_sub, &updated);

		if (updated) {
			/* get a local copy of the home position data */
			struct home_position_s home;
			memset(&home, 0, sizeof(home));
			orb_copy(ORB_ID(home_position), _home_sub, &home);

			if (home.valid_hpos) {
				_home_lat = home.lat;
				_home_lon = home.lon;
				_home_position_set = true;
			}
		}

		/* Distance from home */
		if (_home_position_set) {
			uint16_t dist = (uint16_t)get_distance_to_next_waypoint(_home_lat, _home_lon, lat, lon);

			msg.distance_L = (uint8_t)dist & 0xff;
			msg.distance_H = (uint8_t)(dist >> 8) & 0xff;

			/* Direction back to home */
			uint16_t bearing = (uint16_t)(get_bearing_to_next_waypoint(_home_lat, _home_lon, lat, lon) * M_RAD_TO_DEG_F);
			msg.home_direction = (uint8_t)bearing >> 1;
		}
	}

	msg.stop = STOP_BYTE;
	memcpy(buffer, &msg, *size);
}

void
convert_to_degrees_minutes_seconds(double val, int *deg, int *min, int *sec)
{
	*deg = (int)val;

	double delta = val - *deg;
	const double min_d = delta * 60.0d;
	*min = (int)min_d;
	delta = min_d - *min;
	*sec = (int)(delta * 10000.0d);
}
