@###############################################
@#
@# EmPy template for generating RtpsTopics.cpp file
@#
@###############################################
@# Start of Template
@#
@# Context:
@#  - msgs (List) list of all msg files
@#  - ids (List) list of all RTPS msg ids
@###############################################
@{
import os

import genmsg.msgs

from px_generate_uorb_topic_helper import * # this is in Tools/
from px_generate_uorb_topic_files import MsgScope # this is in Tools/

send_topics = [(alias[idx] if alias[idx] else s.short_name) for idx, s in enumerate(spec) if scope[idx] == MsgScope.SEND]
recv_topics = [(alias[idx] if alias[idx] else s.short_name) for idx, s in enumerate(spec) if scope[idx] == MsgScope.RECEIVE]
package = package[0]
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

#include "RtpsTopics.h"

bool RtpsTopics::init(std::condition_variable* t_send_queue_cv, std::mutex* t_send_queue_mutex, std::queue<uint8_t>* t_send_queue, const std::string& ns)
{
@[if recv_topics]@
    // Initialise subscribers
    std::cout << "\033[0;36m---   Subscribers   ---\033[0m" << std::endl;
@[for topic in recv_topics]@
    if (_@(topic)_sub.init(@(rtps_message_id(ids, topic)), t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
        std::cout << "- @(topic) subscriber started" << std::endl;
    } else {
        std::cerr << "Failed starting @(topic) subscriber" << std::endl;
        return false;
    }
@[end for]@
    std::cout << "\033[0;36m-----------------------\033[0m" << std::endl << std::endl;
@[end if]@
@[if send_topics]@
    // Initialise publishers
    std::cout << "\033[0;36m----   Publishers  ----\033[0m" << std::endl;
@[for topic in send_topics]@
    if (_@(topic)_pub.init(ns)) {
        std::cout << "- @(topic) publisher started" << std::endl;
@[    if topic == 'Timesync' or topic == 'timesync']@
        _timesync->start(&_@(topic)_pub);
@[    end if]@
    } else {
        std::cerr << "ERROR starting @(topic) publisher" << std::endl;
        return false;
    }
@[end for]@
    std::cout << "\033[0;36m-----------------------\033[0m" << std::endl;
@[end if]@
    return true;
}

@[if send_topics]@
void RtpsTopics::publish(uint8_t topic_ID, char data_buffer[], size_t len)
{
    switch (topic_ID)
    {
@[for topic in send_topics]@
        case @(rtps_message_id(ids, topic)): // @(topic)
        {
            @(topic)_msg_t st;
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
@[    if topic == 'Timesync' or topic == 'timesync']@
            _timesync->processTimesyncMsg(&st);

            if (getMsgSysID(&st) == 1) {
@[    end if]@
            // apply timestamp offset
            uint64_t timestamp = getMsgTimestamp(&st);
            _timesync->subtractOffset(timestamp);
            setMsgTimestamp(&st, timestamp);
            _@(topic)_pub.publish(&st);
@[    if topic == 'Timesync' or topic == 'timesync']@
            }
@[    end if]@
        }
        break;
@[end for]@
        default:
            printf("\033[1;33m[   micrortps_agent   ]\tUnexpected topic ID '%hhu' to publish Please make sure the agent is capable of parsing the message associated to the topic ID '%hhu'\033[0m\n", topic_ID, topic_ID);
        break;
    }
}
@[end if]@
@[if recv_topics]@

bool RtpsTopics::getMsg(const uint8_t topic_ID, eprosima::fastcdr::Cdr &scdr)
{
    bool ret = false;
    switch (topic_ID)
    {
@[for topic in recv_topics]@
        case @(rtps_message_id(ids, topic)): // @(topic)
            if (_@(topic)_sub.hasMsg())
            {
                @(topic)_msg_t msg = _@(topic)_sub.getMsg();
@[    if topic == 'Timesync' or topic == 'timesync']@
                if (getMsgSysID(&msg) == 0) {
@[    end if]@
                // apply timestamps offset
                uint64_t timestamp = getMsgTimestamp(&msg);
                uint64_t timestamp_sample = getMsgTimestampSample(&msg);
                _timesync->addOffset(timestamp);
                setMsgTimestamp(&msg, timestamp);
                _timesync->addOffset(timestamp_sample);
                setMsgTimestampSample(&msg, timestamp_sample);
                msg.serialize(scdr);
                ret = true;
@[    if topic == 'Timesync' or topic == 'timesync']@
                }
@[    end if]@
                _@(topic)_sub.unlockMsg();
            }
        break;
@[end for]@
        default:
            printf("\033[1;33m[   micrortps_agent   ]\tUnexpected topic ID '%hhu' to getMsg. Please make sure the agent is capable of parsing the message associated to the topic ID '%hhu'\033[0m\n", topic_ID, topic_ID);
        break;
    }

    return ret;
}
@[end if]@
