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
 * Defines basic functionality of UAVCAN v1 GNSS publisher
 *
 * @author Jacob Crabill <jacob@flyvoly.com>
 */

#pragma once

// DS-15 Specification Messages
#include <reg/drone/physics/kinematics/geodetic/Point_0_1.h>
#include <reg/drone/service/gnss/DilutionOfPrecision_0_1.h>

#include "Publisher.hpp"

class UavcanGnssPublisher : public UavcanPublisher
{
public:
	UavcanGnssPublisher(CanardInstance &ins, UavcanParamManager &pmgr, uint8_t instance = 0) :
		UavcanPublisher(ins, pmgr, "gps", instance)
	{

	};

	// Update the uORB Subscription and broadcast a UAVCAN message
	virtual void update() override
	{
		if (_gps_sub.updated() && _port_id != CANARD_PORT_ID_UNSET) {
			sensor_gps_s gps {};
			_gps_sub.update(&gps);

			reg_drone_physics_kinematics_geodetic_Point_0_1 geo {};
			geo.latitude = gps.lat;
			geo.longitude = gps.lon;
			geo.altitude = uavcan_si_unit_length_WideScalar_1_0 { .meter = static_cast<double>(gps.alt) };

			uint8_t geo_payload_buffer[reg_drone_physics_kinematics_geodetic_Point_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

			CanardTransfer transfer = {
				.timestamp_usec = hrt_absolute_time() + PUBLISHER_DEFAULT_TIMEOUT_USEC,
				.priority       = CanardPriorityNominal,
				.transfer_kind  = CanardTransferKindMessage,
				.port_id        = _port_id, // This is the subject-ID.
				.remote_node_id = CANARD_NODE_ID_UNSET,
				.transfer_id    = _transfer_id,
				.payload_size   = reg_drone_physics_kinematics_geodetic_Point_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_,
				.payload        = &geo_payload_buffer,
			};

			int32_t result = reg_drone_physics_kinematics_geodetic_Point_0_1_serialize_(&geo, geo_payload_buffer,
					 &transfer.payload_size);

			if (result == 0) {
				// set the data ready in the buffer and chop if needed
				++_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
				result = canardTxPush(&_canard_instance, &transfer);
			}

			/// TODO: Also publish DilutionOfPrecision, ...?
			reg_drone_service_gnss_DilutionOfPrecision_0_1 dop {
				.geometric = NAN,
				.position = NAN,
				.horizontal = gps.hdop,
				.vertical = gps.vdop,
				.time = NAN,
				.northing = NAN,
				.easting = NAN,
			};

			uint8_t dop_payload_buffer[reg_drone_service_gnss_DilutionOfPrecision_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

			CanardPortID _port_id_2 = static_cast<CanardPortID>((uint16_t)_port_id + 1U);

			CanardTransfer transfer2 = {
				.timestamp_usec = hrt_absolute_time() + PUBLISHER_DEFAULT_TIMEOUT_USEC,
				.priority       = CanardPriorityNominal,
				.transfer_kind  = CanardTransferKindMessage,
				.port_id        = _port_id_2, // This is the subject-ID.
				.remote_node_id = CANARD_NODE_ID_UNSET,
				.transfer_id    = _transfer_id_2,
				.payload_size   = reg_drone_service_gnss_DilutionOfPrecision_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_,
				.payload        = &dop_payload_buffer,
			};

			result = reg_drone_service_gnss_DilutionOfPrecision_0_1_serialize_(&dop, dop_payload_buffer, &transfer2.payload_size);

			if (result == 0) {
				// set the data ready in the buffer and chop if needed
				++_transfer_id_2;  // The transfer-ID shall be incremented after every transmission on this subject.
				result = canardTxPush(&_canard_instance, &transfer2);
			}
		}
	};

private:

	/// TODO: Allow >1 instance
	uORB::Subscription _gps_sub{ORB_ID(sensor_gps)};
	CanardTransferID _transfer_id_2 {0};
};
