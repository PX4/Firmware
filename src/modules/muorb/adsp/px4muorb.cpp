/****************************************************************************
 *
 * Copyright (C) 2015 Mark Charlebois. All rights reserved.
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

#include "px4muorb.hpp"
#include "uORBFastRpcChannel.hpp"
#include "uORBManager.hpp"

#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <dspal_platform.h>
#include "uORB.h"
#include <parameters/param.h>
#include <px4_platform_common/shmem.h>
#include <px4_platform_common/log.h>

__BEGIN_DECLS
extern int dspal_main(int argc, char *argv[]);
__END_DECLS

int px4muorb_orb_initialize()
{
	HAP_power_request(100, 100, 1000);
	shmem_info_p = NULL;

	// The uORB Manager needs to be initialized first up, otherwise the instance is nullptr.
	uORB::Manager::initialize();
	// Register the fastrpc muorb with uORBManager.
	uORB::Manager::get_instance()->set_uorb_communicator(
		uORB::FastRpcChannel::GetInstance());

	// Now continue with the usual dspal startup.
	const char *argv[] = { "dspal", "start" };
	int argc = 2;
	int rc;
	rc = dspal_main(argc, (char **) argv);

	return rc;
}

int px4muorb_set_absolute_time_offset(int32_t time_diff_us)
{
	return hrt_set_absolute_time_offset(time_diff_us);
}

int px4muorb_get_absolute_time(uint64_t *time_us)
{
	*time_us = hrt_absolute_time();
	return 0;
}

/*update value and param's change bit in shared memory*/
int px4muorb_param_update_to_shmem(uint32_t param, const uint8_t *value,
				   int data_len_in_bytes)
{
	unsigned int byte_changed, bit_changed;
	union param_value_u *param_value = (union param_value_u *) value;

	if (!shmem_info_p) {
		init_shared_memory();
	}

	if (get_shmem_lock(__FILE__, __LINE__) != 0) {
		PX4_INFO("Could not get shmem lock\n");
		return -1;
	}

	shmem_info_p->params_val[param] = *param_value;

	byte_changed = param / 8;
	bit_changed = 1 << param % 8;
	shmem_info_p->krait_changed_index[byte_changed] |= bit_changed;

	release_shmem_lock(__FILE__, __LINE__);

	return 0;
}

int px4muorb_param_update_index_from_shmem(unsigned char *data, int data_len_in_bytes)
{
	if (!shmem_info_p) {
		return -1;
	}

	if (get_shmem_lock(__FILE__, __LINE__) != 0) {
		PX4_INFO("Could not get shmem lock\n");
		return -1;
	}

	for (int i = 0; i < data_len_in_bytes; i++) {
		data[i] = shmem_info_p->adsp_changed_index[i];
	}

	release_shmem_lock(__FILE__, __LINE__);

	return 0;
}

int px4muorb_param_update_value_from_shmem(uint32_t param, const uint8_t *value,
		int data_len_in_bytes)
{
	unsigned int byte_changed, bit_changed;
	union param_value_u *param_value = (union param_value_u *) value;

	if (!shmem_info_p) {
		return -1;
	}

	if (get_shmem_lock(__FILE__, __LINE__) != 0) {
		PX4_INFO("Could not get shmem lock\n");
		return -1;
	}

	*param_value = shmem_info_p->params_val[param];

	/*also clear the index since we are holding the lock*/
	byte_changed = param / 8;
	bit_changed = 1 << param % 8;
	shmem_info_p->adsp_changed_index[byte_changed] &= ~bit_changed;

	release_shmem_lock(__FILE__, __LINE__);

	return 0;
}

int px4muorb_topic_advertised(const char *topic_name)
{
	int rc = 0;
	PX4_INFO("TEST px4muorb_topic_advertised of [%s] on remote side...", topic_name);
	uORB::FastRpcChannel *channel = uORB::FastRpcChannel::GetInstance();
	uORBCommunicator::IChannelRxHandler *rxHandler = channel->GetRxHandler();

	if (rxHandler != nullptr) {
		rc = rxHandler->process_remote_topic(topic_name, 1);

	} else {
		rc = -1;
	}

	return rc;
}

int px4muorb_topic_unadvertised(const char *topic_name)
{
	int rc = 0;
	PX4_INFO("TEST px4muorb_topic_unadvertised of [%s] on remote side...", topic_name);
	uORB::FastRpcChannel *channel = uORB::FastRpcChannel::GetInstance();
	uORBCommunicator::IChannelRxHandler *rxHandler = channel->GetRxHandler();

	if (rxHandler != nullptr) {
		rc = rxHandler->process_remote_topic(topic_name, 0);

	} else {
		rc = -1;
	}

	return rc;
}

int px4muorb_add_subscriber(const char *name)
{
	int rc = 0;
	uORB::FastRpcChannel *channel = uORB::FastRpcChannel::GetInstance();
	channel->AddRemoteSubscriber(name);
	uORBCommunicator::IChannelRxHandler *rxHandler = channel->GetRxHandler();

	if (rxHandler != nullptr) {
		rc = rxHandler->process_add_subscription(name, 0);

		if (rc != OK) {
			channel->RemoveRemoteSubscriber(name);
		}

	} else {
		rc = -1;
	}

	return rc;
}

int px4muorb_remove_subscriber(const char *name)
{
	int rc = 0;
	uORB::FastRpcChannel *channel = uORB::FastRpcChannel::GetInstance();
	channel->RemoveRemoteSubscriber(name);
	uORBCommunicator::IChannelRxHandler *rxHandler = channel->GetRxHandler();

	if (rxHandler != nullptr) {
		rc = rxHandler->process_remove_subscription(name);

	} else {
		rc = -1;
	}

	return rc;
}

int px4muorb_send_topic_data(const char *name, const uint8_t *data,
			     int data_len_in_bytes)
{
	int rc = 0;
	uORB::FastRpcChannel *channel = uORB::FastRpcChannel::GetInstance();
	uORBCommunicator::IChannelRxHandler *rxHandler = channel->GetRxHandler();

	if (rxHandler != nullptr) {
		rc = rxHandler->process_received_message(name, data_len_in_bytes,
				(uint8_t *) data);

	} else {
		rc = -1;
	}

	return rc;
}

int px4muorb_is_subscriber_present(const char *topic_name, int *status)
{
	int rc = 0;
	int32_t local_status = 0;
	uORB::FastRpcChannel *channel = uORB::FastRpcChannel::GetInstance();
	rc = channel->is_subscriber_present(topic_name, &local_status);

	if (rc == 0) {
		*status = (int) local_status;
	}

	return rc;
}

int px4muorb_receive_msg(int *msg_type, char *topic_name, int topic_name_len,
			 uint8_t *data, int data_len_in_bytes, int *bytes_returned)
{
	int rc = 0;
	int32_t local_msg_type = 0;
	int32_t local_bytes_returned = 0;
	uORB::FastRpcChannel *channel = uORB::FastRpcChannel::GetInstance();
	//PX4_DEBUG( "topic_namePtr: [0x%p] dataPtr: [0x%p]", topic_name, data );
	rc = channel->get_data(&local_msg_type, topic_name, topic_name_len, data,
			       data_len_in_bytes, &local_bytes_returned);
	*msg_type = (int) local_msg_type;
	*bytes_returned = (int) local_bytes_returned;
	return rc;
}

int px4muorb_receive_bulk_data(uint8_t *bulk_transfer_buffer,
			       int max_size_in_bytes, int *returned_length_in_bytes, int *topic_count)
{
	int rc = 0;
	int32_t local_bytes_returned = 0;
	int32_t local_topic_count = 0;
	uORB::FastRpcChannel *channel = uORB::FastRpcChannel::GetInstance();
	//PX4_DEBUG( "topic_namePtr: [0x%p] dataPtr: [0x%p]", topic_name, data );
	rc = channel->get_bulk_data(bulk_transfer_buffer, max_size_in_bytes,
				    &local_bytes_returned, &local_topic_count);
	*returned_length_in_bytes = (int) local_bytes_returned;
	*topic_count = (int) local_topic_count;
	return rc;
}

int px4muorb_unblock_recieve_msg(void)
{
	int rc = 0;
	uORB::FastRpcChannel *channel = uORB::FastRpcChannel::GetInstance();
	rc = channel->unblock_get_data_method();
	return rc;
}
