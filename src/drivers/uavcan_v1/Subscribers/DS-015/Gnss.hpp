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
 * @file Gnss.hpp
 *
 * Defines basic functionality of UAVCAN v1 GNSS subscription
 *
 * @author Jacob Crabill <jacob@flyvoly.com>
 */

#pragma once

// DS-15 Specification Messages
#include <reg/drone/physics/kinematics/geodetic/Point_0_1.h>

#include "../DynamicPortSubscriber.hpp"

class UavcanGnssSubscriber : public UavcanDynamicPortSubscriber
{
public:
	UavcanGnssSubscriber(CanardInstance &ins, UavcanParamManager &pmgr, uint8_t instance = 0) :
		UavcanDynamicPortSubscriber(ins, pmgr, "gps", instance) { };

	void subscribe() override
	{
		// Subscribe to messages reg.drone.physics.kinematics.geodetic.Point.0.1
		canardRxSubscribe(&_canard_instance,
				  CanardTransferKindMessage,
				  _subj_sub._canard_sub.port_id,
				  reg_drone_physics_kinematics_geodetic_Point_0_1_EXTENT_BYTES_,
				  CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
				  &_subj_sub._canard_sub);

		/** TODO: Add additional GPS-data messages: (reg.drone.service.gnss._.0.1.uavcan):
		 * # A compliant implementation of this service should publish the following subjects:
		 * #
		 * #   PUBLISHED SUBJECT NAME      SUBJECT TYPE                                            TYP. RATE [Hz]
		 * #   point_kinematics            reg.drone.physics.kinematics.geodetic.PointStateVarTs   1...100
		 * #   time                        reg.drone.service.gnss.Time                             1...10
		 * #   heartbeat                   reg.drone.service.gnss.Heartbeat                        ~1
		 * #   sensor_status               reg.drone.service.sensor.Status                         ~1
		 *
		 * Not mentioned, but should also be included: Dilution of Precision
		 *   (reg.drone.service.gnss.DilutionOfPrecision.0.1.uavcan)
		 * For PX4, only the PointStateVarTs, DilutionOfPrecision, and perhaps Time would be needed
		 * to publish 'sensor_gps'
		 */
	};

	void callback(const CanardTransfer &receive) override
	{
		// Test with Yakut:
		// export YAKUT_TRANSPORT="pyuavcan.transport.can.CANTransport(pyuavcan.transport.can.media.slcan.SLCANMedia('/dev/serial/by-id/usb-Zubax_Robotics_Zubax_Babel_23002B000E514E413431302000000000-if00', 8, 115200), 42)"
		// yakut pub 1500.reg.drone.physics.kinematics.geodetic.Point.0.1 '{latitude: 1.234, longitude: 2.34, altitude: {meter: 0.5}}'
		PX4_INFO("GpsCallback");

		reg_drone_physics_kinematics_geodetic_Point_0_1 geo {};
		size_t geo_size_in_bits = receive.payload_size;
		reg_drone_physics_kinematics_geodetic_Point_0_1_deserialize_(&geo, (const uint8_t *)receive.payload, &geo_size_in_bits);

		double lat = geo.latitude;
		double lon = geo.longitude;
		double alt = geo.altitude.meter;
		PX4_INFO("Latitude: %f, Longitude: %f, Altitude: %f", lat, lon, alt);
		/// do something with the data
	};

};
