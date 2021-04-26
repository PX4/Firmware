/****************************************************************************
 *
 *   Copyright (c) 2014, 2015 PX4 Development Team. All rights reserved.
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
 * @file gnss.cpp
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 * @author Andrew Chambers <achamber@gmail.com>
 *
 */

#include "gnss.hpp"

#include <cstdint>

#include <drivers/drv_hrt.h>
#include <systemlib/err.h>
#include <mathlib/mathlib.h>

using namespace time_literals;

const char *const UavcanGnssBridge::NAME = "gnss";

UavcanGnssBridge::UavcanGnssBridge(uavcan::INode &node) :
	UavcanSensorBridgeBase("uavcan_gnss", ORB_ID(sensor_gps)),
	_node(node),
	_sub_auxiliary(node),
	_sub_fix(node),
	_sub_fix2(node),
	_pub_rtcm(node),
	_channel_using_fix2(new bool[_max_channels]),
	_rtcm_perf(perf_alloc(PC_INTERVAL, "uavcan: gnss: rtcm pub"))
{
	for (uint8_t i = 0; i < _max_channels; i++) {
		_channel_using_fix2[i] = false;
	}

	set_device_type(DRV_GPS_DEVTYPE_UAVCAN);
}

UavcanGnssBridge::~UavcanGnssBridge()
{
	delete [] _channel_using_fix2;
	perf_free(_rtcm_perf);
}

int
UavcanGnssBridge::init()
{
	int res = _sub_auxiliary.start(AuxiliaryCbBinder(this, &UavcanGnssBridge::gnss_auxiliary_sub_cb));

	if (res < 0) {
		PX4_WARN("GNSS auxiliary sub failed %i", res);
		return res;
	}

	res = _sub_fix.start(FixCbBinder(this, &UavcanGnssBridge::gnss_fix_sub_cb));

	if (res < 0) {
		PX4_WARN("GNSS fix sub failed %i", res);
		return res;
	}

	res = _sub_fix2.start(Fix2CbBinder(this, &UavcanGnssBridge::gnss_fix2_sub_cb));

	if (res < 0) {
		PX4_WARN("GNSS fix2 sub failed %i", res);
		return res;
	}

	_pub_rtcm.setPriority(uavcan::TransferPriority::OneHigherThanLowest);

	return res;
}

void
UavcanGnssBridge::gnss_auxiliary_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Auxiliary> &msg)
{
	// store latest hdop and vdop for use in process_fixx();
	_last_gnss_auxiliary_timestamp = hrt_absolute_time();
	_last_gnss_auxiliary_hdop = msg.hdop;
	_last_gnss_auxiliary_vdop = msg.vdop;
}

void
UavcanGnssBridge::gnss_fix_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Fix> &msg)
{
	// Check to see if this node is also publishing a Fix2 message.
	// If so, ignore the old "Fix" message for this node.
	const int8_t ch = get_channel_index_for_node(msg.getSrcNodeID().get());

	if (ch > -1 && _channel_using_fix2[ch]) {
		return;
	}

	uint8_t fix_type = msg.status;

	const bool valid_pos_cov = !msg.position_covariance.empty();
	const bool valid_vel_cov = !msg.velocity_covariance.empty();

	float pos_cov[9];
	msg.position_covariance.unpackSquareMatrix(pos_cov);

	float vel_cov[9];
	msg.velocity_covariance.unpackSquareMatrix(vel_cov);

	process_fixx(msg, fix_type, pos_cov, vel_cov, valid_pos_cov, valid_vel_cov);
}

void
UavcanGnssBridge::gnss_fix2_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Fix2> &msg)
{
	using uavcan::equipment::gnss::Fix2;

	const int8_t ch = get_channel_index_for_node(msg.getSrcNodeID().get());

	if (ch > -1 && !_channel_using_fix2[ch]) {
		PX4_WARN("GNSS Fix2 msg detected for ch %d; disabling Fix msg for this node", ch);
		_channel_using_fix2[ch] = true;
	}

	uint8_t fix_type = msg.status;

	switch (msg.mode) {
	case Fix2::MODE_DGPS:
		fix_type = 4; // RTCM code differential
		break;

	case Fix2::MODE_RTK:
		switch (msg.sub_mode) {
		case Fix2::SUB_MODE_RTK_FLOAT:
			fix_type = 5; // RTK float
			break;

		case Fix2::SUB_MODE_RTK_FIXED:
			fix_type = 6; // RTK fixed
			break;
		}

		break;
	}

	float pos_cov[9] {};
	float vel_cov[9] {};
	bool valid_covariances = true;

	switch (msg.covariance.size()) {
	case 1: {
			// Scalar matrix
			const auto x = msg.covariance[0];

			pos_cov[0] = x;
			pos_cov[4] = x;
			pos_cov[8] = x;

			vel_cov[0] = x;
			vel_cov[4] = x;
			vel_cov[8] = x;
		}
		break;

	case 6: {
			// Diagonal matrix (the most common case)
			pos_cov[0] = msg.covariance[0];
			pos_cov[4] = msg.covariance[1];
			pos_cov[8] = msg.covariance[2];

			vel_cov[0] = msg.covariance[3];
			vel_cov[4] = msg.covariance[4];
			vel_cov[8] = msg.covariance[5];

		}
		break;


	case 21: {
			// Upper triangular matrix.
			// This code has been carefully optimized by hand. We could use unpackSquareMatrix(), but it's slow.
			// Sub-matrix indexes (empty squares contain velocity-position covariance data):
			// 0  1  2
			// 1  6  7
			// 2  7 11
			//         15 16 17
			//         16 18 19
			//         17 19 20
			pos_cov[0] = msg.covariance[0];
			pos_cov[1] = msg.covariance[1];
			pos_cov[2] = msg.covariance[2];
			pos_cov[3] = msg.covariance[1];
			pos_cov[4] = msg.covariance[6];
			pos_cov[5] = msg.covariance[7];
			pos_cov[6] = msg.covariance[2];
			pos_cov[7] = msg.covariance[7];
			pos_cov[8] = msg.covariance[11];

			vel_cov[0] = msg.covariance[15];
			vel_cov[1] = msg.covariance[16];
			vel_cov[2] = msg.covariance[17];
			vel_cov[3] = msg.covariance[16];
			vel_cov[4] = msg.covariance[18];
			vel_cov[5] = msg.covariance[19];
			vel_cov[6] = msg.covariance[17];
			vel_cov[7] = msg.covariance[19];
			vel_cov[8] = msg.covariance[20];
		}

	/* FALLTHROUGH */
	case 36: {
			// Full matrix 6x6.
			// This code has been carefully optimized by hand. We could use unpackSquareMatrix(), but it's slow.
			// Sub-matrix indexes (empty squares contain velocity-position covariance data):
			//  0  1  2
			//  6  7  8
			// 12 13 14
			//          21 22 23
			//          27 28 29
			//          33 34 35
			pos_cov[0] = msg.covariance[0];
			pos_cov[1] = msg.covariance[1];
			pos_cov[2] = msg.covariance[2];
			pos_cov[3] = msg.covariance[6];
			pos_cov[4] = msg.covariance[7];
			pos_cov[5] = msg.covariance[8];
			pos_cov[6] = msg.covariance[12];
			pos_cov[7] = msg.covariance[13];
			pos_cov[8] = msg.covariance[14];

			vel_cov[0] = msg.covariance[21];
			vel_cov[1] = msg.covariance[22];
			vel_cov[2] = msg.covariance[23];
			vel_cov[3] = msg.covariance[27];
			vel_cov[4] = msg.covariance[28];
			vel_cov[5] = msg.covariance[29];
			vel_cov[6] = msg.covariance[33];
			vel_cov[7] = msg.covariance[34];
			vel_cov[8] = msg.covariance[35];
		}

	/* FALLTHROUGH */
	default: {
			// Either empty or invalid sized, interpret as zero matrix
			valid_covariances = false;
			break;	// Nothing to do
		}
	}

	process_fixx(msg, fix_type, pos_cov, vel_cov, valid_covariances, valid_covariances);
}

template <typename FixType>
void UavcanGnssBridge::process_fixx(const uavcan::ReceivedDataStructure<FixType> &msg,
				    uint8_t fix_type,
				    const float (&pos_cov)[9], const float (&vel_cov)[9],
				    const bool valid_pos_cov, const bool valid_vel_cov)
{
	sensor_gps_s report{};
	report.device_id = get_device_id();

	/*
	 * FIXME HACK
	 * There used to be the following line of code:
	 * 	report.timestamp_position = msg.getMonotonicTimestamp().toUSec();
	 * It stopped working when the time sync feature has been introduced, because it caused libuavcan
	 * to use an independent time source (based on hardware TIM5) instead of HRT.
	 * The proper solution is to be developed.
	 */
	report.timestamp = hrt_absolute_time();

	report.lat           = msg.latitude_deg_1e8 / 10;
	report.lon           = msg.longitude_deg_1e8 / 10;
	report.alt           = msg.height_msl_mm;
	report.alt_ellipsoid = msg.height_ellipsoid_mm;

	if (valid_pos_cov) {
		// Horizontal position uncertainty
		const float horizontal_pos_variance = math::max(pos_cov[0], pos_cov[4]);
		report.eph = (horizontal_pos_variance > 0) ? sqrtf(horizontal_pos_variance) : -1.0F;

		// Vertical position uncertainty
		report.epv = (pos_cov[8] > 0) ? sqrtf(pos_cov[8]) : -1.0F;

	} else {
		report.eph = -1.0F;
		report.epv = -1.0F;
	}

	if (valid_vel_cov) {
		report.s_variance_m_s = math::max(math::max(vel_cov[0], vel_cov[4]), vel_cov[8]);

		/* There is a nonlinear relationship between the velocity vector and the heading.
		 * Use Jacobian to transform velocity covariance to heading covariance
		 *
		 * Nonlinear equation:
		 * heading = atan2(vel_e_m_s, vel_n_m_s)
		 * For math, see http://en.wikipedia.org/wiki/Atan2#Derivative
		 *
		 * To calculate the variance of heading from the variance of velocity,
		 * cov(heading) = J(velocity)*cov(velocity)*J(velocity)^T
		 */
		float vel_n = msg.ned_velocity[0];
		float vel_e = msg.ned_velocity[1];
		float vel_n_sq = vel_n * vel_n;
		float vel_e_sq = vel_e * vel_e;
		report.c_variance_rad =
			(vel_e_sq * vel_cov[0] +
			 -2 * vel_n * vel_e * vel_cov[1] +	// Covariance matrix is symmetric
			 vel_n_sq * vel_cov[4]) / ((vel_n_sq + vel_e_sq) * (vel_n_sq + vel_e_sq));

	} else {
		report.s_variance_m_s = -1.0F;
		report.c_variance_rad = -1.0F;
	}

	report.fix_type = fix_type;

	report.vel_n_m_s = msg.ned_velocity[0];
	report.vel_e_m_s = msg.ned_velocity[1];
	report.vel_d_m_s = msg.ned_velocity[2];
	report.vel_m_s = sqrtf(report.vel_n_m_s * report.vel_n_m_s +
			       report.vel_e_m_s * report.vel_e_m_s +
			       report.vel_d_m_s * report.vel_d_m_s);
	report.cog_rad = atan2f(report.vel_e_m_s, report.vel_n_m_s);
	report.vel_ned_valid = true;

	report.timestamp_time_relative = 0;

	const uint64_t gnss_ts_usec = uavcan::UtcTime(msg.gnss_timestamp).toUSec();

	switch (msg.gnss_time_standard) {
	case FixType::GNSS_TIME_STANDARD_UTC:
		report.time_utc_usec = gnss_ts_usec;
		break;

	case FixType::GNSS_TIME_STANDARD_GPS:
		if (msg.num_leap_seconds > 0) {
			report.time_utc_usec = gnss_ts_usec - msg.num_leap_seconds + 9;
		}

		break;

	case FixType::GNSS_TIME_STANDARD_TAI:
		if (msg.num_leap_seconds > 0) {
			report.time_utc_usec = gnss_ts_usec - msg.num_leap_seconds - 10;
		}

		break;

	default:
		break;
	}

	// If we haven't already done so, set the system clock using GPS data
	if (valid_pos_cov && !_system_clock_set) {
		timespec ts{};

		// get the whole microseconds
		ts.tv_sec = report.time_utc_usec / 1000000ULL;

		// get the remainder microseconds and convert to nanoseconds
		ts.tv_nsec = (report.time_utc_usec % 1000000ULL) * 1000;

		px4_clock_settime(CLOCK_REALTIME, &ts);

		_system_clock_set = true;
	}

	report.satellites_used = msg.sats_used;

	if (hrt_elapsed_time(&_last_gnss_auxiliary_timestamp) < 2_s) {
		report.hdop = _last_gnss_auxiliary_hdop;
		report.vdop = _last_gnss_auxiliary_vdop;

	} else {
		// Using PDOP for HDOP and VDOP
		// Relevant discussion: https://github.com/PX4/Firmware/issues/5153
		report.hdop = msg.pdop;
		report.vdop = msg.pdop;
	}

	report.heading = NAN;
	report.heading_offset = NAN;

	publish(msg.getSrcNodeID().get(), &report);
}

void UavcanGnssBridge::update()
{
	handleInjectDataTopic();
}

// Partially taken from src/drivers/gps/gps.cpp
// This listens on the gps_inject_data uORB topic for RTCM data
// sent from a GCS (usually over MAVLINK GPS_RTCM_DATA).
// Forwarding this data to the UAVCAN bus enables DGPS/RTK GPS
// to work.
void UavcanGnssBridge::handleInjectDataTopic()
{
	bool updated = false;

	// Limit maximum number of GPS injections to 6 since usually
	// GPS injections should consist of 1-4 packets (GPS, Glonass, BeiDou, Galileo).
	// Looking at 6 packets thus guarantees, that at least a full injection
	// data set is evaluated.
	const size_t max_num_injections = 6;
	size_t num_injections = 0;

	do {
		num_injections++;
		updated = _orb_inject_data_sub.updated();

		if (updated) {
			gps_inject_data_s msg;

			if (_orb_inject_data_sub.copy(&msg)) {

				/* Write the message to the gps device. Note that the message could be fragmented.
				 * But as we don't write anywhere else to the device during operation, we don't
				 * need to assemble the message first.
				 */
				injectData(msg.data, msg.len);
			}
		}
	} while (updated && num_injections < max_num_injections);
}

bool UavcanGnssBridge::injectData(const uint8_t *const data, const size_t data_len)
{
	using uavcan::equipment::gnss::RTCMStream;

	perf_count(_rtcm_perf);

	RTCMStream msg;
	msg.protocol_id = RTCMStream::PROTOCOL_ID_RTCM3;

	const size_t capacity = msg.data.capacity();
	size_t written = 0;
	bool result = true;

	while (result && written < data_len) {
		size_t chunk_size = data_len - written;

		if (chunk_size > capacity) {
			chunk_size = capacity;
		}

		for (size_t i = 0; i < chunk_size; ++i) {
			msg.data.push_back(data[written]);
			written += 1;
		}

		result = _pub_rtcm.broadcast(msg) >= 0;
		msg.data.clear();
	}

	return result;
}

void UavcanGnssBridge::print_status() const
{
	UavcanSensorBridgeBase::print_status();
	perf_print_counter(_rtcm_perf);
}
