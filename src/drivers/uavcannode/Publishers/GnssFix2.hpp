/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#pragma once

#include "UavcanPublisherBase.hpp"

#include <uavcan/equipment/gnss/Fix2.hpp>

#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/sensor_gps.h>

namespace uavcannode
{

class GnssFix2 :
	public UavcanPublisherBase,
	public uORB::SubscriptionCallbackWorkItem,
	private uavcan::Publisher<uavcan::equipment::gnss::Fix2>
{
public:
	GnssFix2(px4::WorkItem *work_item, uavcan::INode &node) :
		UavcanPublisherBase(uavcan::equipment::gnss::Fix2::DefaultDataTypeID),
		uORB::SubscriptionCallbackWorkItem(work_item, ORB_ID(sensor_gps)),
		uavcan::Publisher<uavcan::equipment::gnss::Fix2>(node)
	{}

	void PrintInfo() override
	{
		if (uORB::SubscriptionCallbackWorkItem::advertised()) {
			printf("\t%s -> %s:%d\n",
			       uORB::SubscriptionCallbackWorkItem::get_topic()->o_name,
			       uavcan::equipment::gnss::Fix2::getDataTypeFullName(),
			       id());
		}
	}

	void BroadcastAnyUpdates() override
	{
		// sensor_gps -> uavcan::equipment::gnss::Fix2
		sensor_gps_s gps;

		if (uORB::SubscriptionCallbackWorkItem::update(&gps)) {
			uavcan::equipment::gnss::Fix2 fix2{};

			fix2.gnss_time_standard = fix2.GNSS_TIME_STANDARD_UTC;
			fix2.gnss_timestamp.usec = gps.time_utc_usec;
			fix2.latitude_deg_1e8 = (int64_t)gps.lat * 10;
			fix2.longitude_deg_1e8 = (int64_t)gps.lon * 10;
			fix2.height_msl_mm = gps.alt;
			fix2.height_ellipsoid_mm = gps.alt_ellipsoid;
			fix2.status = gps.fix_type;
			fix2.ned_velocity[0] = gps.vel_n_m_s;
			fix2.ned_velocity[1] = gps.vel_e_m_s;
			fix2.ned_velocity[2] = gps.vel_d_m_s;
			fix2.pdop = gps.hdop > gps.vdop ? gps.hdop :
				    gps.vdop; // Use pdop for both hdop and vdop since uavcan v0 spec does not support them
			fix2.sats_used = gps.satellites_used;

			// Diagonal matrix
			// position variances -- Xx, Yy, Zz
			fix2.covariance.push_back(gps.eph);
			fix2.covariance.push_back(gps.eph);
			fix2.covariance.push_back(gps.epv);
			// velocity variance -- Vxx, Vyy, Vzz
			fix2.covariance.push_back(gps.s_variance_m_s);
			fix2.covariance.push_back(gps.s_variance_m_s);
			fix2.covariance.push_back(gps.s_variance_m_s);

			uavcan::Publisher<uavcan::equipment::gnss::Fix2>::broadcast(fix2);

			// ensure callback is registered
			uORB::SubscriptionCallbackWorkItem::registerCallback();
		}
	}
};
} // namespace uavcannode
