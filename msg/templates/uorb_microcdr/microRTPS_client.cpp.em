@###############################################
@#
@# EmPy template for generating microRTPS_client.cpp file
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
import os

import genmsg.msgs

from px_generate_uorb_topic_helper import * # this is in Tools/
from px_generate_uorb_topic_files import MsgScope # this is in Tools/

topic_names = [s.short_name for s in spec]
send_topics = [(alias[idx] if alias[idx] else s.short_name) for idx, s in enumerate(spec) if scope[idx] == MsgScope.SEND]
send_base_types = [s.short_name for idx, s in enumerate(spec) if scope[idx] == MsgScope.SEND]
recv_topics = [(alias[idx] if alias[idx] else s.short_name) for idx, s in enumerate(spec) if scope[idx] == MsgScope.RECEIVE]
receive_base_types = [s.short_name for idx, s in enumerate(spec) if scope[idx] == MsgScope.RECEIVE]
}@
/****************************************************************************
 *
 * Copyright (c) 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#include "microRTPS_transport.h"
#include "microRTPS_client.h"

#include <inttypes.h>
#include <cstdio>
#include <ctime>
#include <pthread.h>

#include <ucdr/microcdr.h>
#include <px4_time.h>
#include <uORB/uORB.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
@[for topic in list(set(topic_names))]@
#include <uORB/topics/@(topic).h>
#include <uORB_microcdr/topics/@(topic).h>
@[end for]@

void* send(void *data);

uint8_t last_remote_msg_seq = 0;
uint8_t last_msg_seq = 0;

@[if recv_topics]@
// Publishers for received messages
struct RcvTopicsPubs
{
@[    for idx, topic in enumerate(recv_topics)]@
    uORB::Publication<@(receive_base_types[idx])_s> @(topic)_pub{ORB_ID(@(topic))};
@[    end for]@
};
@[end if]@

@[if send_topics]@
// Subscribers for messages to send
struct SendTopicsSubs
{
@[    for idx, topic in enumerate(send_topics)]@
    uORB::Subscription @(topic)_sub{ORB_ID(@(topic))};
@[    end for]@
};
@[end if]@


@[if send_topics]@
void* send(void* /*unused*/)
{
    char data_buffer[BUFFER_SIZE] = {};
    uint64_t sent = 0, total_sent = 0;
    int loop = 0, read = 0;
    uint32_t length = 0;
    size_t header_length = 0;
    SendTopicsSubs *subs = new SendTopicsSubs();

    // ucdrBuffer to serialize using the user defined buffer
    ucdrBuffer writer;
    header_length = transport_node->get_header_length();
    ucdr_init_buffer(&writer, reinterpret_cast<uint8_t*>(&data_buffer[header_length]), BUFFER_SIZE - header_length);

    struct timespec begin;
    px4_clock_gettime(CLOCK_REALTIME, &begin);

    while (!_should_exit_task)
    {
@[for idx, topic in enumerate(send_topics)]@
        {
            @(send_base_types[idx])_s @(topic)_data;
            if (subs->@(topic)_sub.update(&@(topic)_data)) {
@[if topic == 'Timesync' or topic == 'timesync']@
                if(@(topic)_data.sys_id == 0 && @(topic)_data.seq != last_remote_msg_seq && @(topic)_data.tc1 == 0) {
                    last_remote_msg_seq = @(topic)_data.seq;

                    @(topic)_data.timestamp = hrt_absolute_time();
                    @(topic)_data.sys_id = 1;
                    @(topic)_data.seq = last_msg_seq;
                    @(topic)_data.tc1 = hrt_absolute_time() * 1000ULL;
                    @(topic)_data.ts1 = @(topic)_data.ts1;

                    last_msg_seq++;
@[end if]@
                    // copy raw data into local buffer. Payload is shifted by header length to make room for header
                    serialize_@(send_base_types[idx])(&writer, &@(topic)_data, &data_buffer[header_length], &length);
                    if (0 < (read = transport_node->write(static_cast<char>(@(rtps_message_id(ids, topic))), data_buffer, length)))
                    {
                        total_sent += read;
                        ++sent;
                    }
@[if topic == 'Timesync' or topic == 'timesync']@
                }
@[end if]@
             }
        }
@[end for]@
        px4_usleep(_options.sleep_ms * 1000);
        ++loop;
    }

    struct timespec end;
    px4_clock_gettime(CLOCK_REALTIME, &end);
    double elapsed_secs = end.tv_sec - begin.tv_sec + (end.tv_nsec - begin.tv_nsec) / 1e9;
    PX4_INFO("SENT: %" PRIu64 " messages in %d LOOPS, %" PRIu64 " bytes in %.03f seconds - %.02fKB/s",
                sent, loop, total_sent, elapsed_secs, total_sent / (1e3 * elapsed_secs));

    delete subs;

    return nullptr;
}

static int launch_send_thread(pthread_t &sender_thread)
{
    pthread_attr_t sender_thread_attr;
    pthread_attr_init(&sender_thread_attr);
    pthread_attr_setstacksize(&sender_thread_attr, PX4_STACK_ADJUSTED(2250));
    struct sched_param param;
    (void)pthread_attr_getschedparam(&sender_thread_attr, &param);
    param.sched_priority = SCHED_PRIORITY_DEFAULT;
    (void)pthread_attr_setschedparam(&sender_thread_attr, &param);
    pthread_create(&sender_thread, &sender_thread_attr, send, nullptr);
    if (pthread_setname_np(sender_thread, "micrortps_client_send"))
    {
        PX4_ERR("Could not set pthread name (%d)", errno);
    }
    pthread_attr_destroy(&sender_thread_attr);

    return 0;
}
@[end if]@

void micrortps_start_topics(struct timespec &begin, uint64_t &total_read, uint64_t &received, int &loop)
{
@[if recv_topics]@
    char data_buffer[BUFFER_SIZE] = {};
    int read = 0;
    uint8_t topic_ID = 255;
    RcvTopicsPubs *pubs = new RcvTopicsPubs();

    // Set the main task name to 'micrortps_client_rcv' in case there is
    // data to receive
    px4_prctl(PR_SET_NAME, "micrortps_client_rcv", px4_getpid());

    // ucdrBuffer to deserialize using the user defined buffer
    ucdrBuffer reader;
    ucdr_init_buffer(&reader, reinterpret_cast<uint8_t*>(data_buffer), BUFFER_SIZE);
@[end if]@

    px4_clock_gettime(CLOCK_REALTIME, &begin);
    _should_exit_task = false;
@[if send_topics]@

    // create a thread for sending data to the simulator
    pthread_t sender_thread;
    launch_send_thread(sender_thread);
@[end if]@

    while (!_should_exit_task)
    {
@[if recv_topics]@
        while (0 < (read = transport_node->read(&topic_ID, data_buffer, BUFFER_SIZE)))
        {
            total_read += read;
            uint64_t read_time = hrt_absolute_time();
            switch (topic_ID)
            {
@[for idx, topic in enumerate(recv_topics)]@
                case @(rtps_message_id(ids, topic)):
                {
                    @(receive_base_types[idx])_s @(topic)_data;
                    deserialize_@(receive_base_types[idx])(&reader, &@(topic)_data, data_buffer);
                    if (@(topic)_data.timestamp > read_time)
                    {
                        // don't allow timestamps from the future
                        @(topic)_data.timestamp = read_time;
                    }
                    pubs->@(topic)_pub.publish(@(topic)_data);
                    ++received;
                }
                break;
@[end for]@
                default:
                    PX4_WARN("Unexpected topic ID '%hhu' to getMsg. Please make sure the client is capable of parsing the message associated to the topic ID '%hhu'", topic_ID, topic_ID);
                break;
            }
        }
@[end if]@

        // loop forever if informed loop number is negative
        if (_options.loops >= 0 && loop >= _options.loops) break;

        px4_usleep(_options.sleep_ms * 1000);
        ++loop;
    }
@[if recv_topics]@
    delete pubs;
@[end if]@
@[if send_topics]@
    _should_exit_task = true;
    pthread_join(sender_thread, nullptr);
@[end if]@
}
