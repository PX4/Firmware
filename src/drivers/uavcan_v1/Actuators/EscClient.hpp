/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
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
 * @file EscClient.hpp
 *
 * Client-side implementation of DS-15 specification ESC service
 *
 * Publishes the following UAVCAN v1 messages:
 *   reg.drone.service.actuator.common.sp.Value8.0.1
 *   reg.drone.service.common.Readiness.0.1
 *
 * Subscribes to the following UAVCAN v1 messages:
 *   reg.drone.service.actuator.common.Feedback.0.1
 *   reg.drone.service.actuator.common.Status.0.1
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 * @author Jacob Crabill <jacob@flyvoly.com>
 */

#pragma once

#include <lib/perf/perf_counter.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/esc_status.h>
#include <drivers/drv_hrt.h>
#include <lib/mixer_module/mixer_module.hpp>

// DS-15 Specification Messages
#include <reg/drone/service/actuator/common/sp/Vector31_0_1.h>
#include <reg/drone/service/common/Readiness_0_1.h>

/// TODO: Allow derived class of Subscription at same time, to handle ESC Feedback/Status
class UavcanEscController : public UavcanPublisher
{
public:
	static constexpr int MAX_ACTUATORS = MixingOutput::MAX_ACTUATORS;

	UavcanEscController(CanardInstance &ins, UavcanParamManager &pmgr) :
		UavcanPublisher(ins, pmgr, "esc") { };

	~UavcanEscController() {};

	void update() override
	{
		if (_armed_sub.updated()) {
			actuator_armed_s new_arming;
			_armed_sub.update(&new_arming);

			if (new_arming.armed != _armed.armed) {
				_armed = new_arming;

				// Only publish if we have a valid publication ID set
				if (_port_id == 0) {
					return;
				}

				reg_drone_service_common_Readiness_0_1 msg_arming {};

				if (_armed.armed) {
					msg_arming.value = reg_drone_service_common_Readiness_0_1_ENGAGED;

				} else if (_armed.prearmed) {
					msg_arming.value = reg_drone_service_common_Readiness_0_1_STANDBY;

				} else {
					msg_arming.value = reg_drone_service_common_Readiness_0_1_SLEEP;
				}

				uint8_t arming_payload_buffer[reg_drone_service_common_Readiness_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

				CanardPortID arming_pid = static_cast<CanardPortID>(static_cast<uint32_t>(_port_id) + 1);
				CanardTransfer transfer = {
					.timestamp_usec = hrt_absolute_time() + PUBLISHER_DEFAULT_TIMEOUT_USEC,
					.priority       = CanardPriorityNominal,
					.transfer_kind  = CanardTransferKindMessage,
					.port_id        = arming_pid,                // This is the subject-ID.
					.remote_node_id = CANARD_NODE_ID_UNSET,      // Messages cannot be unicast, so use UNSET.
					.transfer_id    = _arming_transfer_id,
					.payload_size   = reg_drone_service_common_Readiness_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_,
					.payload        = &arming_payload_buffer,
				};

				int result = reg_drone_service_common_Readiness_0_1_serialize_(&msg_arming, arming_payload_buffer,
						&transfer.payload_size);

				if (result == 0) {
					// set the data ready in the buffer and chop if needed
					++_arming_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
					result = canardTxPush(&_canard_instance, &transfer);
				}
			}
		}
	};

	void update_outputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs)
	{
		if (_port_id > 0) {
			reg_drone_service_actuator_common_sp_Vector31_0_1 msg_sp {0};

			for (uint8_t i = 0; i < MAX_ACTUATORS; i++) {
				if (i < num_outputs) {
					msg_sp.value[i] = static_cast<float>(outputs[i]);

				} else {
					// "unset" values published as NaN
					msg_sp.value[i] = NAN;
				}
			}

			PX4_INFO("Publish %d values %f, %f, %f, %f", num_outputs, (double)msg_sp.value[0], (double)msg_sp.value[1],
				 (double)msg_sp.value[2], (double)msg_sp.value[3]);

			uint8_t esc_sp_payload_buffer[reg_drone_service_actuator_common_sp_Vector31_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

			CanardTransfer transfer = {
				.timestamp_usec = hrt_absolute_time() + PUBLISHER_DEFAULT_TIMEOUT_USEC,
				.priority       = CanardPriorityNominal,
				.transfer_kind  = CanardTransferKindMessage,
				.port_id        = _port_id,                // This is the subject-ID.
				.remote_node_id = CANARD_NODE_ID_UNSET,       // Messages cannot be unicast, so use UNSET.
				.transfer_id    = _transfer_id,
				.payload_size   = reg_drone_service_actuator_common_sp_Vector31_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_,
				.payload        = &esc_sp_payload_buffer,
			};

			int result = reg_drone_service_actuator_common_sp_Vector31_0_1_serialize_(&msg_sp, esc_sp_payload_buffer,
					&transfer.payload_size);

			if (result == 0) {
				// set the data ready in the buffer and chop if needed
				++_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
				result = canardTxPush(&_canard_instance, &transfer);
			}
		}
	};

	/**
	 * Sets the number of rotors
	 */
	void set_rotor_count(uint8_t count) { _rotor_count = count; }

private:
	/**
	 * ESC status message reception will be reported via this callback.
	 */
	void esc_status_sub_cb(const CanardTransfer &msg);

	uint8_t _rotor_count {0};

	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};
	actuator_armed_s _armed {};

	CanardTransferID _arming_transfer_id;
};
