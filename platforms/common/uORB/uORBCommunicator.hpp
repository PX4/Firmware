/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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

#ifndef _uORBCommunicator_hpp_
#define _uORBCommunicator_hpp_

#include <stdint.h>


namespace uORBCommunicator
{
class IChannel;
class IChannelRxHandler;
}

/**
 * Interface to enable remote subscriptions.  The implementor of this interface
 * shall manage the communication channel. It can be fastRPC or tcp or ip.
 */

class uORBCommunicator::IChannel
{
public:

	//=========================================================================
	//     INTERFACES FOR Control messages over a channel.
	//=========================================================================

	/**
	 * @brief Interface to notify the remote entity of a topic being advertised.
	 *
	 * @param messageName
	 * 	This represents the uORB message name(aka topic); This message name should be
	 * 	globally unique.
	 * @return
	 * 	0 = success; This means the messages is successfully sent to the receiver
	 * 		Note: This does not mean that the receiver as received it.
	 *  otherwise = failure.
	 */
	virtual int16_t topic_advertised(const char *messageName) = 0;

	/**
	 * @brief Interface to notify the remote entity of a topic being unadvertised
	 * and is no longer publishing messages.
	 *
	 * @param messageName
	 * 	This represents the uORB message name(aka topic); This message name should be
	 * 	globally unique.
	 * @return
	 * 	0 = success; This means the messages is successfully sent to the receiver
	 * 		Note: This does not mean that the receiver as received it.
	 *  otherwise = failure.
	 */
	//virtual int16_t topic_unadvertised(const char *messageName) = 0;

	/**
	 * @brief Interface to notify the remote entity of interest of a
	 * subscription for a message.
	 *
	 * @param messageName
	 * 	This represents the uORB message name; This message name should be
	 * 	globally unique.
	 * @param msgRate
	 * 	The max rate at which the subscriber can accept the messages.
	 * @return
	 * 	0 = success; This means the messages is successfully sent to the receiver
	 * 		Note: This does not mean that the receiver as received it.
	 *  otherwise = failure.
	 */

	virtual int16_t add_subscription(const char *messageName, int32_t msgRateInHz) = 0;



	/**
	 * @brief Interface to notify the remote entity of removal of a subscription
	 *
	 * @param messageName
	 * 	This represents the uORB message name; This message name should be
	 * 	globally unique.
	 * @return
	 * 	0 = success; This means the messages is successfully sent to the receiver
	 * 		Note: This does not necessarily mean that the receiver as received it.
	 *  otherwise = failure.
	 */

	virtual int16_t remove_subscription(const char *messageName) = 0;


	/**
	 * Register Message Handler.  This is internal for the IChannel implementer*
	 */
	virtual int16_t register_handler(uORBCommunicator::IChannelRxHandler *handler) = 0;


	//=========================================================================
	//     INTERFACES FOR Data messages
	//=========================================================================

	/**
	 * @brief Sends the data message over the communication link.
	 * @param messageName
	 * 	This represents the uORB message name; This message name should be
	 * 	globally unique.
	 * @param length
	 * 	The length of the data buffer to be sent.
	 * @param data
	 * 	The actual data to be sent.
	 * @return
	 *  0 = success; This means the messages is successfully sent to the receiver
	 * 		Note: This does not mean that the receiver as received it.
	 *  otherwise = failure.
	 */

	virtual int16_t send_message(const char *messageName, int32_t length, uint8_t *data) = 0;

};

/**
 * Class passed to the communication link implement to provide callback for received
 * messages over a channel.
 */
class uORBCommunicator::IChannelRxHandler
{
public:

	/**
	 * Interface to process a received topic from remote.
	 * @param topic_name
	 * 	This represents the uORB message Name (topic); This message Name should be
	 * 	globally unique.
	 * @param isAdvertisement
	 * 	Represents if the topic has been advertised or is no longer avialable.
	 * @return
	 *  0 = success; This means the messages is successfully handled in the
	 *  	handler.
	 *  otherwise = failure.
	 */

	virtual int16_t process_remote_topic(const char *topic_name, bool isAdvertisement) = 0;

	/**
	 * Interface to process a received AddSubscription from remote.
	 * @param messageName
	 * 	This represents the uORB message Name; This message Name should be
	 * 	globally unique.
	 * @param msgRate
	 * 	The max rate at which the subscriber can accept the messages.
	 * @return
	 *  0 = success; This means the messages is successfully handled in the
	 *  	handler.
	 *  otherwise = failure.
	 */

	virtual int16_t process_add_subscription(const char *messageName, int32_t msgRateInHz) = 0;


	/**
	 * Interface to process a received control msg to remove subscription
	 * @param messageName
	 * 	This represents the uORB message Name; This message Name should be
	 * 	globally unique.
	 * @return
	 *  0 = success; This means the messages is successfully handled in the
	 *  	handler.
	 *  otherwise = failure.
	 */

	virtual int16_t process_remove_subscription(const char *messageName) = 0;


	/**
	 * Interface to process the received data message.
	 * @param messageName
	 * 	This represents the uORB message Name; This message Name should be
	 * 	globally unique.
	 * @param length
	 * 	The length of the data buffer to be sent.
	 * @param data
	 * 	The actual data to be sent.
	 * @return
	 *  0 = success; This means the messages is successfully handled in the
	 *  	handler.
	 *  otherwise = failure.
	 */

	virtual int16_t process_received_message(const char *messageName, int32_t length, uint8_t *data) = 0;

};

#endif /* _uORBCommunicator_hpp_ */
