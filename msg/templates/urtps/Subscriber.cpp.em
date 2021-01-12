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
 * @@file @(topic)_Subscriber.cpp
 * This file contains the implementation of the subscriber functions.
 *
 * This file was adapted from the fastcdrgen tool.
 */

#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/subscriber/Subscriber.h>
#include <fastrtps/attributes/SubscriberAttributes.h>

#include <fastrtps/Domain.h>

#include "@(topic)_Subscriber.h"

@(topic)_Subscriber::@(topic)_Subscriber()
    : mp_participant(nullptr),
      mp_subscriber(nullptr)
{ }

@(topic)_Subscriber::~@(topic)_Subscriber()
{
    Domain::removeParticipant(mp_participant);
}

bool @(topic)_Subscriber::init(uint8_t topic_ID, std::condition_variable* t_send_queue_cv, std::mutex* t_send_queue_mutex, std::queue<uint8_t>* t_send_queue, const std::string& ns)
{
    m_listener.topic_ID = topic_ID;
    m_listener.t_send_queue_cv = t_send_queue_cv;
    m_listener.t_send_queue_mutex = t_send_queue_mutex;
    m_listener.t_send_queue = t_send_queue;

    // Create RTPSParticipant
    ParticipantAttributes PParam;
@[if version.parse(fastrtps_version) < version.parse('2.0')]@
    PParam.rtps.builtin.domainId = 0;
@[else]@
    PParam.domainId = 0;
@[end if]@
@[if version.parse(fastrtps_version) <= version.parse('1.8.4')]@
    PParam.rtps.builtin.leaseDuration = c_TimeInfinite;
@[else]@
    PParam.rtps.builtin.discovery_config.leaseDuration = c_TimeInfinite;
@[end if]@
    std::string nodeName = ns;
    nodeName.append("@(topic)_subscriber");
    PParam.rtps.setName(nodeName.c_str());
    mp_participant = Domain::createParticipant(PParam);
    if(mp_participant == nullptr)
            return false;

    //Register the type
    Domain::registerType(mp_participant, static_cast<TopicDataType*>(&@(topic)DataType));

    // Create Subscriber
    SubscriberAttributes Rparam;
    Rparam.topic.topicKind = NO_KEY;
    Rparam.topic.topicDataType = @(topic)DataType.getName();
@[if ros2_distro]@
@[    if ros2_distro == "ardent"]@
    Rparam.qos.m_partition.push_back("rt");
    std::string topicName = ns;
    topicName.append("@(topic)_PubSubTopic");
    Rparam.topic.topicName = topicName;
@[    else]@
    std::string topicName = "rt/";
    topicName.append(ns);
    topicName.append("@(topic)_PubSubTopic");
    Rparam.topic.topicName = topicName;
@[    end if]@
@[else]@
    std::string topicName = ns;
    topicName.append("@(topic)PubSubTopic");
    Rparam.topic.topicName = topicName;
@[end if]@
    mp_subscriber = Domain::createSubscriber(mp_participant, Rparam, static_cast<SubscriberListener*>(&m_listener));
    if(mp_subscriber == nullptr)
        return false;
    return true;
}

void @(topic)_Subscriber::SubListener::onSubscriptionMatched(Subscriber* sub, MatchingInfo& info)
{
@# Since the time sync runs on the bridge itself, it is required that there is a
@# match between two topics of the same entity
@[if topic != 'Timesync' and topic != 'timesync']@
    // The first 6 values of the ID guidPrefix of an entity in a DDS-RTPS Domain
    // are the same for all its subcomponents (publishers, subscribers)
    bool is_different_endpoint = false;
    for (size_t i = 0; i < 6; i++) {
        if (sub->getGuid().guidPrefix.value[i] != info.remoteEndpointGuid.guidPrefix.value[i]) {
            is_different_endpoint = true;
            break;
        }
    }

    // If the matching happens for the same entity, do not make a match
    if (is_different_endpoint) {
        if (info.status == MATCHED_MATCHING) {
            n_matched++;
            std::cout << "\033[0;37m[   micrortps_agent   ]\t@(topic) subscriber matched\033[0m" << std::endl;
        } else {
            n_matched--;
            std::cout << "\033[0;37m[   micrortps_agent   ]\t@(topic) subscriber unmatched\033[0m" << std::endl;
        }
    }
@[else]@
    (void)sub;

    if (info.status == MATCHED_MATCHING) {
        n_matched++;
    } else {
        n_matched--;
    }
@[end if]@
}

void @(topic)_Subscriber::SubListener::onNewDataMessage(Subscriber* sub)
{
    if (n_matched > 0) {
        std::unique_lock<std::mutex> has_msg_lock(has_msg_mutex);
        if(has_msg.load() == true) // Check if msg has been fetched
        {
            has_msg_cv.wait(has_msg_lock); // Wait till msg has been fetched
        }
        has_msg_lock.unlock();

        // Take data
        if(sub->takeNextData(&msg, &m_info))
        {
            if(m_info.sampleKind == ALIVE)
            {
                std::unique_lock<std::mutex> lk(*t_send_queue_mutex);

                ++n_msg;
                has_msg = true;

                t_send_queue->push(topic_ID);
                lk.unlock();
                t_send_queue_cv->notify_one();

            }
        }
    }
}

bool @(topic)_Subscriber::hasMsg()
{
    if (m_listener.n_matched > 0) {
        return m_listener.has_msg.load();
    }

    return false;
}

@(topic)_msg_t @(topic)_Subscriber::getMsg()
{
    return m_listener.msg;
}

void @(topic)_Subscriber::unlockMsg()
{
    if (m_listener.n_matched > 0) {
        std::unique_lock<std::mutex> has_msg_lock(m_listener.has_msg_mutex);
        m_listener.has_msg = false;
        has_msg_lock.unlock();
        m_listener.has_msg_cv.notify_one();
    }
}
