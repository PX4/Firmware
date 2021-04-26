@###############################################
@#
@# EmPy template for generating RtpsTopics.h file
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
from packaging import version

import genmsg.msgs

from px_generate_uorb_topic_helper import * # this is in Tools/
from px_generate_uorb_topic_files import MsgScope # this is in Tools/

send_topics = [(alias[idx] if alias[idx] else s.short_name) for idx, s in enumerate(spec) if scope[idx] == MsgScope.SEND]
recv_topics = [(alias[idx] if alias[idx] else s.short_name) for idx, s in enumerate(spec) if scope[idx] == MsgScope.RECEIVE]
package = package[0]
fastrtps_version = fastrtps_version[0]
try:
    ros2_distro = ros2_distro[0].decode("utf-8")
except AttributeError:
    ros2_distro = ros2_distro[0]
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

#include <fastcdr/Cdr.h>
#include <condition_variable>
#include <queue>
#include <type_traits>

#include "microRTPS_timesync.h"

@[for topic in send_topics]@
#include "@(topic)_Publisher.h"
@[end for]@
@[for topic in recv_topics]@
#include "@(topic)_Subscriber.h"
@[end for]@


@[for topic in (recv_topics + send_topics)]@
@[    if version.parse(fastrtps_version) <= version.parse('1.7.2')]@
@[        if ros2_distro]@
using @(topic)_msg_t = @(package)::msg::dds_::@(topic)_;
@[        else]@
using @(topic)_msg_t = @(topic)_;
@[        end if]@
@[    else]@
@[        if ros2_distro]@
using @(topic)_msg_t = @(package)::msg::@(topic);
@[        else]@
using @(topic)_msg_t = @(topic);
@[        end if]@
@[    end if]@
@[end for]@

class RtpsTopics {
public:
    bool init(std::condition_variable* t_send_queue_cv, std::mutex* t_send_queue_mutex, std::queue<uint8_t>* t_send_queue, const std::string& ns);
    void set_timesync(const std::shared_ptr<TimeSync>& timesync) { _timesync = timesync; };
@[if send_topics]@
    void publish(uint8_t topic_ID, char data_buffer[], size_t len);
@[end if]@
@[if recv_topics]@
    bool getMsg(const uint8_t topic_ID, eprosima::fastcdr::Cdr &scdr);
@[end if]@

private:
@[if send_topics]@
    /** Publishers **/
@[for topic in send_topics]@
    @(topic)_Publisher _@(topic)_pub;
@[end for]@
@[end if]@

@[if recv_topics]@
    /** Subscribers **/
@[for topic in recv_topics]@
    @(topic)_Subscriber _@(topic)_sub;
@[end for]@
@[end if]@

    // SFINAE
    template<typename T> struct hasTimestampSample{
    private:
      static void detect(...);
      template<typename U> static decltype(std::declval<U>().timestamp_sample()) detect(const U&);
    public:
      static constexpr bool value = std::is_same<uint64_t, decltype(detect(std::declval<T>()))>::value;
    };

    template<typename T>
    inline typename std::enable_if<!hasTimestampSample<T>::value, uint64_t>::type
    getMsgTimestampSample_impl(const T*) { return 0; }

    /** Msg metada Getters **/
@[if version.parse(fastrtps_version) <= version.parse('1.7.2') or not ros2_distro]@
    template <class T>
    inline uint64_t getMsgTimestamp(const T* msg) { return msg->timestamp_(); }

    template<typename T>
    inline typename std::enable_if<hasTimestampSample<T>::value, uint64_t>::type
    getMsgTimestampSample_impl(const T* msg) { return msg->timestamp_sample_(); }

    template <class T>
    inline uint8_t getMsgSysID(const T* msg) { return msg->sys_id_(); }

    template <class T>
    inline uint8_t getMsgSeq(const T* msg) { return msg->seq_(); }
@[elif ros2_distro]@
    template <class T>
    inline uint64_t getMsgTimestamp(const T* msg) { return msg->timestamp(); }

    template<typename T>
    inline typename std::enable_if<hasTimestampSample<T>::value, uint64_t>::type
    getMsgTimestampSample_impl(const T* msg) { return msg->timestamp_sample(); }

    template <class T>
    inline uint8_t getMsgSysID(const T* msg) { return msg->sys_id(); }

    template <class T>
    inline uint8_t getMsgSeq(const T* msg) { return msg->seq(); }
@[end if]@

    template <class T>
    inline uint64_t getMsgTimestampSample(const T* msg) { return getMsgTimestampSample_impl(msg); }

    template<typename T>
    inline typename std::enable_if<!hasTimestampSample<T>::value, void>::type
    setMsgTimestampSample_impl(T*, const uint64_t&) {}

    /** Msg metadata Setters **/
@[if version.parse(fastrtps_version) <= version.parse('1.7.2') or not ros2_distro]@
    template <class T>
    inline void setMsgTimestamp(T* msg, const uint64_t& timestamp) { msg->timestamp_() = timestamp; }

    template <class T>
    inline typename std::enable_if<hasTimestampSample<T>::value, void>::type
    setMsgTimestampSample_impl(T* msg, const uint64_t& timestamp_sample) { msg->timestamp_sample_() = timestamp_sample; }

    template <class T>
    inline void setMsgSysID(T* msg, const uint8_t& sys_id) { msg->sys_id_() = sys_id; }

    template <class T>
    inline void setMsgSeq(T* msg, const uint8_t& seq) { msg->seq_() = seq; }
@[elif ros2_distro]@
    template <class T>
    inline void setMsgTimestamp(T* msg, const uint64_t& timestamp) { msg->timestamp() = timestamp; }

    template <class T>
    inline typename std::enable_if<hasTimestampSample<T>::value, void>::type
    setMsgTimestampSample_impl(T* msg, const uint64_t& timestamp_sample) { msg->timestamp_sample() = timestamp_sample; }

    template <class T>
    inline void setMsgSysID(T* msg, const uint8_t& sys_id) { msg->sys_id() = sys_id; }

    template <class T>
    inline void setMsgSeq(T* msg, const uint8_t& seq) { msg->seq() = seq; }
@[end if]@

    template <class T>
    inline void setMsgTimestampSample(T* msg, const uint64_t& timestamp_sample) { setMsgTimestampSample_impl(msg, timestamp_sample); }

    /**
     * @@brief Timesync object ptr.
     *         This object is used to compuyte and apply the time offsets to the
     *         messages timestamps.
     */
    std::shared_ptr<TimeSync> _timesync;
};
