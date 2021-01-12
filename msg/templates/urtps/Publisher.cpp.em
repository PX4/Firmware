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
 * @@file @(topic)_Publisher.cpp
 * This file contains the implementation of the publisher functions.
 *
 * This file was adapted from the fastcdrgen tool.
 */


#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/publisher/Publisher.h>
#include <fastrtps/attributes/PublisherAttributes.h>

#include <fastrtps/Domain.h>

#include "@(topic)_Publisher.h"

@(topic)_Publisher::@(topic)_Publisher()
    : mp_participant(nullptr),
      mp_publisher(nullptr)
{ }

@(topic)_Publisher::~@(topic)_Publisher()
{
    Domain::removeParticipant(mp_participant);
}

bool @(topic)_Publisher::init(const std::string& ns)
{
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
    nodeName.append("@(topic)_publisher");
    PParam.rtps.setName(nodeName.c_str());
    mp_participant = Domain::createParticipant(PParam);
    if(mp_participant == nullptr)
        return false;

    // Register the type
    Domain::registerType(mp_participant, static_cast<TopicDataType*>(&@(topic)DataType));

    // Create Publisher
    PublisherAttributes Wparam;
    Wparam.topic.topicKind = NO_KEY;
    Wparam.topic.topicDataType = @(topic)DataType.getName();
@[if ros2_distro]@
@[    if ros2_distro == "ardent"]@
    Wparam.qos.m_partition.push_back("rt");
    std::string topicName = ns;
    topicName.append("@(topic)_PubSubTopic");
    Wparam.topic.topicName = topicName;
@[    else]@
    std::string topicName = "rt/";
    topicName.append(ns);
    topicName.append("@(topic)_PubSubTopic");
    Wparam.topic.topicName = topicName;
@[    end if]@
@[else]@
    std::string topicName = ns;
    topicName.append("@(topic)PubSubTopic");
    Wparam.topic.topicName = topicName;
@[end if]@
    mp_publisher = Domain::createPublisher(mp_participant, Wparam, static_cast<PublisherListener*>(&m_listener));
    if(mp_publisher == nullptr)
        return false;
    return true;
}

void @(topic)_Publisher::PubListener::onPublicationMatched(Publisher* pub, MatchingInfo& info)
{
    // The first 6 values of the ID guidPrefix of an entity in a DDS-RTPS Domain
    // are the same for all its subcomponents (publishers, subscribers)
    bool is_different_endpoint = false;
    for (size_t i = 0; i < 6; i++) {
        if (pub->getGuid().guidPrefix.value[i] != info.remoteEndpointGuid.guidPrefix.value[i]) {
            is_different_endpoint = true;
            break;
        }
    }

    // If the matching happens for the same entity, do not make a match
    if (is_different_endpoint) {
        if (info.status == MATCHED_MATCHING) {
            n_matched++;
            std::cout << "\033[0;37m[   micrortps_agent   ]\t@(topic) publisher matched\033[0m" << std::endl;
        } else {
            n_matched--;
            std::cout << "\033[0;37m[   micrortps_agent   ]\t@(topic) publisher unmatched\033[0m" << std::endl;
        }
    }
}

void @(topic)_Publisher::publish(@(topic)_msg_t* st)
{
    mp_publisher->write(st);
}
