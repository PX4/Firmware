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
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <containers/List.hpp>
#include <uavcan/uavcan.hpp>
#include <drivers/device/device.h>
#include <drivers/drv_orb_dev.h>
#include <lib/cdev/CDev.hpp>
#include <uORB/uORB.h>

/**
 * A sensor bridge class must implement this interface.
 */
class IUavcanSensorBridge : uavcan::Noncopyable, public ListNode<IUavcanSensorBridge *>
{
public:
	static constexpr unsigned MAX_NAME_LEN = 20;

	virtual ~IUavcanSensorBridge() = default;

	/**
	 * Returns ASCII name of the bridge.
	 */
	virtual const char *get_name() const = 0;

	/**
	 * Starts the bridge.
	 * @return Non-negative value on success, negative on error.
	 */
	virtual int init() = 0;

	/**
	 * Returns number of active redundancy channels.
	 */
	virtual unsigned get_num_redundant_channels() const = 0;

	/**
	 * Prints current status in a human readable format to stdout.
	 */
	virtual void print_status() const = 0;

	/**
	 * Sensor bridge factory.
	 * Creates all known sensor bridges and puts them in the linked list.
	 */
	static void make_all(uavcan::INode &node, List<IUavcanSensorBridge *> &list);
};

namespace uavcan_bridge
{
struct Channel {
	int node_id              = -1;
	orb_advert_t orb_advert  = nullptr;
	int class_instance       = -1;
	int orb_instance         = -1;
	cdev::CDev *h_driver     = nullptr;
};
}

/**
 * This is the base class for redundant sensors with an independent ORB topic per each redundancy channel.
 * For example, sensor_mag0, sensor_mag1, etc.
 */
class UavcanCDevSensorBridgeBase : public IUavcanSensorBridge, public device::CDev
{
	const char *const _class_devname;
	const orb_id_t _orb_topic;
	uavcan_bridge::Channel *const _channels;
	bool _out_of_channels = false;

protected:
	static constexpr unsigned DEFAULT_MAX_CHANNELS = ORB_MULTI_MAX_INSTANCES;
	const unsigned _max_channels;

	UavcanCDevSensorBridgeBase(const char *name, const char *devname, const char *class_devname,
				   const orb_id_t orb_topic_sensor,
				   const unsigned max_channels = DEFAULT_MAX_CHANNELS) :
		device::CDev(name, devname),
		_class_devname(class_devname),
		_orb_topic(orb_topic_sensor),
		_channels(new uavcan_bridge::Channel[max_channels]),
		_max_channels(max_channels)
	{
		_device_id.devid_s.bus_type = DeviceBusType_UAVCAN;
		_device_id.devid_s.bus = 0;
	}

	/**
	 * Sends one measurement into appropriate ORB topic.
	 * New redundancy channels will be registered automatically.
	 * @param node_id Sensor's Node ID
	 * @param report  Pointer to ORB message object
	 */
	void publish(const int node_id, const void *report);

	/**
	 * Init the sensor driver for this channel.
	 * Implementation depends on sensor type being constructed.
	 * @param channel Channel pointer for which h_driver should be initialized.
	 */
	virtual int init_driver(uavcan_bridge::Channel *channel) { return PX4_OK; };

	uavcan_bridge::Channel *get_channel_for_node(int node_id);

public:
	virtual ~UavcanCDevSensorBridgeBase();

	unsigned get_num_redundant_channels() const override;

	int8_t get_channel_index_for_node(int node_id);

	void print_status() const override;
};
