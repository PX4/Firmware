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
 * @file frsky_data.c
 *
 * @author Stefan Rado <px4@sradonia.net>
 *
 * FrSky telemetry implementation.
 *
 */

#include "frsky_data.h"
#include "common.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <lib/ecl/geo/geo.h>
#include <stdbool.h>
#include <math.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_status.h>

#include <drivers/drv_hrt.h>

#define frac(f) (f - (int)f)

struct frsky_subscription_data_s {

	uORB::SubscriptionData<battery_status_s> battery_status_sub{ORB_ID(battery_status)};
	uORB::SubscriptionData<vehicle_acceleration_s> vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};
	uORB::SubscriptionData<vehicle_air_data_s> vehicle_air_data_sub{ORB_ID(vehicle_air_data)};
	uORB::SubscriptionData<vehicle_local_position_s> vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::SubscriptionData<vehicle_global_position_s> vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::SubscriptionData<vehicle_gps_position_s> vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};
	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
};

static struct frsky_subscription_data_s *subscription_data = nullptr;

/**
 * Initializes the uORB subscriptions.
 */
bool frsky_init()
{
	subscription_data = new frsky_subscription_data_s();

	if (!subscription_data) {
		return false;
	}

	return true;
}

void frsky_deinit()
{
	if (subscription_data) {
		delete subscription_data;
		subscription_data = nullptr;
	}
}

/**
 * Sends a 0x5E start/stop byte.
 */
static void frsky_send_startstop(int uart)
{
	static const uint8_t c = 0x5E;
	write(uart, &c, sizeof(c));
}

/**
 * Sends one byte, performing byte-stuffing if necessary.
 */
static void frsky_send_byte(int uart, uint8_t value)
{
	const uint8_t x5E[] = { 0x5D, 0x3E };
	const uint8_t x5D[] = { 0x5D, 0x3D };

	switch (value) {
	case 0x5E:
		write(uart, x5E, sizeof(x5E));
		break;

	case 0x5D:
		write(uart, x5D, sizeof(x5D));
		break;

	default:
		write(uart, &value, sizeof(value));
		break;
	}
}

/**
 * Sends one data id/value pair.
 */
static void frsky_send_data(int uart, uint8_t id, int16_t data)
{
	/* Cast data to unsigned, because signed shift might behave incorrectly */
	uint16_t udata = data;

	frsky_send_startstop(uart);

	frsky_send_byte(uart, id);
	frsky_send_byte(uart, udata);      /* LSB */
	frsky_send_byte(uart, udata >> 8); /* MSB */
}

void frsky_update_topics()
{
	subscription_data->battery_status_sub.update();
	subscription_data->vehicle_acceleration_sub.update();
	subscription_data->vehicle_air_data_sub.update();
	subscription_data->vehicle_local_position_sub.update();
	subscription_data->vehicle_global_position_sub.update();
	subscription_data->vehicle_gps_position_sub.update();
	subscription_data->vehicle_status_sub.update();
}

/**
 * Sends frame 1 (every 200ms):
 *   acceleration values, barometer altitude, temperature, battery voltage & current
 */
void frsky_send_frame1(int uart)
{
	/* send formatted frame */
	const vehicle_acceleration_s &vehicle_acceleration = subscription_data->vehicle_acceleration_sub.get();
	frsky_send_data(uart, FRSKY_ID_ACCEL_X, roundf(vehicle_acceleration.xyz[0] * 1000.0f));
	frsky_send_data(uart, FRSKY_ID_ACCEL_Y, roundf(vehicle_acceleration.xyz[1] * 1000.0f));
	frsky_send_data(uart, FRSKY_ID_ACCEL_Z, roundf(vehicle_acceleration.xyz[2] * 1000.0f));

	const vehicle_air_data_s &air_data = subscription_data->vehicle_air_data_sub.get();
	frsky_send_data(uart, FRSKY_ID_BARO_ALT_BP, air_data.baro_alt_meter);
	frsky_send_data(uart, FRSKY_ID_BARO_ALT_AP, roundf(frac(air_data.baro_alt_meter) * 100.0f));

	const battery_status_s &bat = subscription_data->battery_status_sub.get();
	frsky_send_data(uart, FRSKY_ID_VFAS, roundf(bat.voltage_v * 10.0f));
	frsky_send_data(uart, FRSKY_ID_CURRENT, (bat.current_a < 0) ? 0 : roundf(bat.current_a * 10.0f));

	int16_t telem_flight_mode = get_telemetry_flight_mode(subscription_data->vehicle_status_sub.get().nav_state);
	frsky_send_data(uart, FRSKY_ID_TEMP1, telem_flight_mode); // send flight mode as TEMP1. This matches with OpenTX & APM

	const vehicle_gps_position_s &gps = subscription_data->vehicle_gps_position_sub.get();
	frsky_send_data(uart, FRSKY_ID_TEMP2, gps.satellites_used * 10 + gps.fix_type);

	frsky_send_startstop(uart);
}

/**
 * Formats the decimal latitude/longitude to the required degrees/minutes.
 */
static float frsky_format_gps(float dec)
{
	float dm_deg = (int) dec;
	return (dm_deg * 100.0f) + (dec - dm_deg) * 60;
}

/**
 * Sends frame 2 (every 1000ms):
 *   GPS course, latitude, longitude, ground speed, GPS altitude, remaining battery level
 */
void frsky_send_frame2(int uart)
{
	const vehicle_global_position_s &gpos = subscription_data->vehicle_global_position_sub.get();
	const vehicle_local_position_s &lpos = subscription_data->vehicle_local_position_sub.get();
	const battery_status_s &battery_status = subscription_data->battery_status_sub.get();
	const vehicle_gps_position_s &gps = subscription_data->vehicle_gps_position_sub.get();

	/* send formatted frame */
	float course = 0, lat = 0, lon = 0, speed = 0, alt = 0;
	char lat_ns = 0, lon_ew = 0;
	int sec = 0;

	if (gpos.timestamp != 0 && hrt_absolute_time() < gpos.timestamp + 20000) {
		course = lpos.heading / M_PI_F * 180.0f;

		if (course < 0.f) { // course is in range [0, 360], 0=north, CW
			course += 360.f;
		}

		lat    = frsky_format_gps(fabsf(gpos.lat));
		lat_ns = (gpos.lat < 0) ? 'S' : 'N';
		lon    = frsky_format_gps(fabsf(gpos.lon));
		lon_ew = (gpos.lon < 0) ? 'W' : 'E';
		speed  = sqrtf(lpos.vx * lpos.vx + lpos.vy * lpos.vy) * 25.0f / 46.0f;
		alt    = gpos.alt;
	}

	if (gps.timestamp != 0 && hrt_absolute_time() < gps.timestamp + 20000) {
		time_t time_gps = gps.time_utc_usec / 1000000ULL;
		struct tm *tm_gps = gmtime(&time_gps);

		sec    = tm_gps->tm_sec;
	}

	frsky_send_data(uart, FRSKY_ID_GPS_COURS_BP, course);
	frsky_send_data(uart, FRSKY_ID_GPS_COURS_AP, frac(course) * 1000.0f);

	frsky_send_data(uart, FRSKY_ID_GPS_LAT_BP, lat);
	frsky_send_data(uart, FRSKY_ID_GPS_LAT_AP, frac(lat) * 10000.0f);
	frsky_send_data(uart, FRSKY_ID_GPS_LAT_NS, lat_ns);

	frsky_send_data(uart, FRSKY_ID_GPS_LONG_BP, lon);
	frsky_send_data(uart, FRSKY_ID_GPS_LONG_AP, frac(lon) * 10000.0f);
	frsky_send_data(uart, FRSKY_ID_GPS_LONG_EW, lon_ew);

	frsky_send_data(uart, FRSKY_ID_GPS_SPEED_BP, speed);
	frsky_send_data(uart, FRSKY_ID_GPS_SPEED_AP, frac(speed) * 100.0f);

	frsky_send_data(uart, FRSKY_ID_GPS_ALT_BP, alt);
	frsky_send_data(uart, FRSKY_ID_GPS_ALT_AP, frac(alt) * 100.0f);

	frsky_send_data(uart, FRSKY_ID_FUEL, roundf(battery_status.remaining * 100.0f));

	frsky_send_data(uart, FRSKY_ID_GPS_SEC, sec);

	frsky_send_startstop(uart);
}

/**
 * Sends frame 3 (every 5000ms):
 *   GPS date & time
 */
void frsky_send_frame3(int uart)
{
	/* send formatted frame */
	time_t time_gps = subscription_data->vehicle_gps_position_sub.get().time_utc_usec / 1000000ULL;

	struct tm *tm_gps = gmtime(&time_gps);
	uint16_t hour_min = (tm_gps->tm_min << 8) | (tm_gps->tm_hour & 0xff);
	frsky_send_data(uart, FRSKY_ID_GPS_DAY_MONTH, tm_gps->tm_mday);
	frsky_send_data(uart, FRSKY_ID_GPS_YEAR, tm_gps->tm_year);
	frsky_send_data(uart, FRSKY_ID_GPS_HOUR_MIN, hour_min);
	frsky_send_data(uart, FRSKY_ID_GPS_SEC, tm_gps->tm_sec);

	frsky_send_startstop(uart);
}

/* parse 11 byte frames */
bool frsky_parse_host(uint8_t *sbuf, int nbytes, struct adc_linkquality *v)
{
	bool data_ready = false;
	static int dcount = 0;
	static uint8_t type = 0;
	static uint8_t data[11];
	static enum {
		HEADER = 0,
		TYPE,
		DATA,
		TRAILER
	} state = HEADER;

	for (int i = 0; i < nbytes; i++) {
		switch (state) {
		case HEADER:
			if (sbuf[i] == 0x7E) {
				state = TYPE;
			}

			break;

		case TYPE:
			if (sbuf[i] != 0x7E) {
				state = DATA;
				type = sbuf[i];
				dcount = 0;
			}

			break;

		case DATA:

			/* read 8 data bytes */
			if (dcount < 7) {
				data[dcount++] = sbuf[i];

			} else {
				/* received all data bytes */
				state = TRAILER;
			}

			break;

		case TRAILER:
			state = HEADER;

			if (sbuf[i] != 0x7E) {
//				warnx("host packet error: %x", sbuf[i]);

			} else {
				data_ready = true;

				if (type == 0xFE) {
					/* this is an adc_linkquality packet */
					v->ad1 = data[0];
					v->ad2 = data[1];
					v->linkq = data[2];
				}
			}

			break;
		}
	}

	return data_ready;
}
