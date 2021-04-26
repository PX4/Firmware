/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @author RJ Gritter <rjgritter657@gmail.com>
 */

#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include "rangefinder.hpp"
#include <math.h>

const char *const UavcanRangefinderBridge::NAME = "rangefinder";

UavcanRangefinderBridge::UavcanRangefinderBridge(uavcan::INode &node) :
	UavcanSensorBridgeBase("uavcan_rangefinder", ORB_ID(distance_sensor)),
	_sub_range_data(node)
{ }

int UavcanRangefinderBridge::init()
{
	// Initialize min/max range from params
	param_get(param_find("UAVCAN_RNG_MIN"), &_range_min_m);
	param_get(param_find("UAVCAN_RNG_MAX"), &_range_max_m);

	int res = _sub_range_data.start(RangeCbBinder(this, &UavcanRangefinderBridge::range_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

void UavcanRangefinderBridge::range_sub_cb(const
		uavcan::ReceivedDataStructure<uavcan::equipment::range_sensor::Measurement> &msg)
{
	uavcan_bridge::Channel *channel = get_channel_for_node(msg.getSrcNodeID().get());

	if (channel == nullptr || channel->instance < 0) {
		// Something went wrong - no channel to publish on; return
		return;
	}

	// Cast our generic CDev pointer to the sensor-specific driver class
	PX4Rangefinder *rangefinder = (PX4Rangefinder *)channel->h_driver;

	if (rangefinder == nullptr) {
		return;
	}

	if (!_inited) {

		uint8_t rangefinder_type = 0;

		switch (msg.sensor_type) {
		case uavcan::equipment::range_sensor::Measurement::SENSOR_TYPE_SONAR:
			rangefinder_type = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;
			break;

		case uavcan::equipment::range_sensor::Measurement::SENSOR_TYPE_RADAR:
			rangefinder_type = distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR;
			break;

		case uavcan::equipment::range_sensor::Measurement::SENSOR_TYPE_LIDAR:
		case uavcan::equipment::range_sensor::Measurement::SENSOR_TYPE_UNDEFINED:
		default:
			rangefinder_type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
			break;
		}

		rangefinder->set_rangefinder_type(rangefinder_type);
		rangefinder->set_fov(msg.field_of_view);
		rangefinder->set_min_distance(_range_min_m);
		rangefinder->set_max_distance(_range_max_m);

		_inited = true;
	}

	/*
	 * FIXME HACK
	 * This code used to rely on msg.getMonotonicTimestamp().toUSec() instead of HRT.
	 * It stopped working when the time sync feature has been introduced, because it caused libuavcan
	 * to use an independent time source (based on hardware TIM5) instead of HRT.
	 * The proper solution is to be developed.
	 */
	rangefinder->update(hrt_absolute_time(), msg.range);
}

int UavcanRangefinderBridge::init_driver(uavcan_bridge::Channel *channel)
{
	// update device id as we now know our device node_id
	DeviceId device_id{_device_id};

	device_id.devid_s.devtype = DRV_DIST_DEVTYPE_UAVCAN;
	device_id.devid_s.address = static_cast<uint8_t>(channel->node_id);

	channel->h_driver = new PX4Rangefinder(device_id.devid, distance_sensor_s::ROTATION_DOWNWARD_FACING);

	if (channel->h_driver == nullptr) {
		return PX4_ERROR;
	}

	PX4Rangefinder *rangefinder = (PX4Rangefinder *)channel->h_driver;

	channel->instance = rangefinder->get_instance();

	if (channel->instance < 0) {
		PX4_ERR("UavcanRangefinder: Unable to get an instance");
		delete rangefinder;
		channel->h_driver = nullptr;
		return PX4_ERROR;
	}

	return PX4_OK;
}
