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

/**
 * @file sensor_gps.hpp
 *
 * Defines uORB over UAVCANv1 sensor_gps subscriber
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 */

#pragma once

#include <uORB/topics/sensor_gps.h>
#include <uORB/PublicationMulti.hpp>

#include "../DynamicPortSubscriber.hpp"

class UORB_over_UAVCAN_sensor_gps_Subscriber : public UavcanDynamicPortSubscriber
{
public:
	UORB_over_UAVCAN_sensor_gps_Subscriber(CanardInstance &ins, UavcanParamManager &pmgr, uint8_t instance = 0) :
		UavcanDynamicPortSubscriber(ins, pmgr, "uorb.sensor_gps", instance) { };

	void subscribe() override
	{
		// Subscribe to messages uORB sensor_gps payload over UAVCAN
		canardRxSubscribe(&_canard_instance,
				  CanardTransferKindMessage,
				  _subj_sub._canard_sub.port_id,
				  sizeof(struct sensor_gps_s),
				  CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC * 10000,
				  &_subj_sub._canard_sub);
	};

	void callback(const CanardTransfer &receive) override
	{
		//PX4_INFO("uORB sensor_gps Callback");

		if (receive.payload_size == sizeof(struct sensor_gps_s)) {
			sensor_gps_s *gps_msg = (sensor_gps_s *)receive.payload;
			gps_msg->timestamp = hrt_absolute_time();

			/* As long as we don't have timesync between nodes we set the timestamp to the current time */

			_sensor_gps_pub.publish(*gps_msg);

		} else {
			PX4_ERR("uORB over UAVCAN %s playload size mismatch got %d expected %d",
				_subj_sub._subject_name, receive.payload_size, sizeof(struct sensor_gps_s));
		}
	};

private:
	uORB::PublicationMulti<sensor_gps_s> _sensor_gps_pub{ORB_ID(sensor_gps)};

};
