@###############################################
@#
@# EmPy template for generating <msg>_uRTPS_UART.cpp file
@#
@###############################################
@# Start of Template
@#
@# Context:
@#  - msgs (List) list of all msg files
@#  - multi_topics (List) list of all multi-topic names
@#  - ids (List) list of all RTPS msg ids
@###############################################
@{
from packaging import version
import genmsg.msgs

from px_generate_uorb_topic_helper import * # this is in Tools/

topic = alias if alias else spec.short_name
try:
    ros2_distro = ros2_distro.decode("utf-8")
except AttributeError:
    pass
}@
/****************************************************************************
 *
 * Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
 * Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*!
 * @@file @(topic)_Subscriber.h
 * This header file contains the declaration of the subscriber functions.
 *
 * This file was adapted from the fastcdrgen tool.
 */


#ifndef _@(topic)__SUBSCRIBER_H_
#define _@(topic)__SUBSCRIBER_H_

#include <fastrtps/fastrtps_fwd.h>
#include <fastrtps/subscriber/SubscriberListener.h>
#include <fastrtps/subscriber/SampleInfo.h>
@[if version.parse(fastrtps_version) <= version.parse('1.7.2')]@
#include "@(topic)_PubSubTypes.h"
@[else]@
#include "@(topic)PubSubTypes.h"
@[end if]@

#include <atomic>
#include <condition_variable>
#include <queue>

using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;

@[if version.parse(fastrtps_version) <= version.parse('1.7.2')]@
@[    if ros2_distro]@
using @(topic)_msg_t = @(package)::msg::dds_::@(topic)_;
using @(topic)_msg_datatype = @(package)::msg::dds_::@(topic)_PubSubType;
@[    else]@
using @(topic)_msg_t = @(topic)_;
using @(topic)_msg_datatype = @(topic)_PubSubType;
@[    end if]@
@[else]@
@[    if ros2_distro]@
using @(topic)_msg_t = @(package)::msg::@(topic);
using @(topic)_msg_datatype = @(package)::msg::@(topic)PubSubType;
@[    else]@
using @(topic)_msg_t = @(topic);
using @(topic)_msg_datatype = @(topic)PubSubType;
@[    end if]@
@[end if]@

class @(topic)_Subscriber
{
public:
    @(topic)_Subscriber();
    virtual ~@(topic)_Subscriber();
    bool init(uint8_t topic_ID, std::condition_variable* t_send_queue_cv, std::mutex* t_send_queue_mutex, std::queue<uint8_t>* t_send_queue, const std::string& ns);
    void run();
    bool hasMsg();
    @(topic)_msg_t getMsg();
    void unlockMsg();

private:
    Participant *mp_participant;
    Subscriber *mp_subscriber;

    class SubListener : public SubscriberListener
    {
    public:
        SubListener() : n_matched(0), n_msg(0), has_msg(false){};
        ~SubListener(){};
        void onSubscriptionMatched(Subscriber* sub, MatchingInfo& info);
        void onNewDataMessage(Subscriber* sub);
        SampleInfo_t m_info;
        int n_matched;
        int n_msg;
        @(topic)_msg_t msg;
        std::atomic_bool has_msg;
        uint8_t topic_ID;
        std::condition_variable* t_send_queue_cv;
        std::mutex* t_send_queue_mutex;
        std::queue<uint8_t>* t_send_queue;
        std::condition_variable has_msg_cv;
        std::mutex has_msg_mutex;

    } m_listener;
    @(topic)_msg_datatype @(topic)DataType;
};

#endif // _@(topic)__SUBSCRIBER_H_
