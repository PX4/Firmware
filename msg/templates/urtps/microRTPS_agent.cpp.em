@###############################################
@#
@# EmPy template for generating microRTPS_agent.cpp file
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
import genmsg.msgs

from px_generate_uorb_topic_helper import * # this is in Tools/
from px_generate_uorb_topic_files import MsgScope # this is in Tools/

send_topics = [(alias[idx] if alias[idx] else s.short_name) for idx, s in enumerate(spec) if scope[idx] == MsgScope.SEND]
recv_topics = [(alias[idx] if alias[idx] else s.short_name) for idx, s in enumerate(spec) if scope[idx] == MsgScope.RECEIVE]
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

#include <thread>
#include <atomic>
#include <unistd.h>
#include <poll.h>
#include <chrono>
#include <ctime>
#include <csignal>
#include <termios.h>
#include <condition_variable>
#include <queue>

#include <fastcdr/Cdr.h>
#include <fastcdr/FastCdr.h>
#include <fastcdr/exceptions/Exception.h>
#include <fastrtps/Domain.h>

#include "microRTPS_transport.h"
#include "microRTPS_timesync.h"
#include "RtpsTopics.h"

// Default values
#define DEVICE "/dev/ttyACM0"
#define SLEEP_US 1
#define BAUDRATE 460800
#define POLL_MS 1
#define WAIT_CNST 2
#define DEFAULT_RECV_PORT 2020
#define DEFAULT_SEND_PORT 2019
#define DEFAULT_IP "127.0.0.1"

using namespace eprosima;
using namespace eprosima::fastrtps;

volatile sig_atomic_t running = 1;
Transport_node *transport_node = nullptr;
RtpsTopics topics;
uint32_t total_sent = 0, sent = 0;

struct options {
    enum class eTransports
    {
        UART,
        UDP
    };
    eTransports transport = options::eTransports::UART;
    char device[64] = DEVICE;
    int sleep_us = SLEEP_US;
    uint32_t baudrate = BAUDRATE;
    int poll_ms = POLL_MS;
    uint16_t recv_port = DEFAULT_RECV_PORT;
    uint16_t send_port = DEFAULT_SEND_PORT;
    char ip[16] = DEFAULT_IP;
    bool sw_flow_control = false;
    bool hw_flow_control = false;
    bool verbose_debug = false;
    std::string ns = "";
} _options;

static void usage(const char *name)
{
    printf("usage: %s [options]\n\n"
             "  -b <baudrate>           UART device baudrate. Default 460800\n"
             "  -d <device>             UART device. Default /dev/ttyACM0\n"
             "  -f <sw flow control>    Activates UART link SW flow control\n"
             "  -h <hw flow control>    Activates UART link HW flow control\n"
             "  -i <ip_address>         Target IP for UDP. Default 127.0.0.1\n"
             "  -n <namespace>          ROS 2 topics namespace. Identifies the vehicle in a multi-agent network\n"
             "  -p <poll_ms>            Time in ms to poll over UART. Default 1ms\n"
             "  -r <reception port>     UDP port for receiving. Default 2019\n"
             "  -s <sending port>       UDP port for sending. Default 2020\n"
             "  -t <transport>          [UART|UDP] Default UART\n"
             "  -v <debug verbosity>    Add more verbosity\n"
             "  -w <sleep_time_us>      Time in us for which each iteration sleep. Default 1ms\n",
             name);
}

static int parse_options(int argc, char **argv)
{
    int ch;

    while ((ch = getopt(argc, argv, "t:d:w:b:p:r:s:i:fhvn:")) != EOF)
    {
        switch (ch)
        {
            case 't': _options.transport      = strcmp(optarg, "UDP") == 0?
                                                 options::eTransports::UDP
                                                :options::eTransports::UART;    break;
            case 'd': if (nullptr != optarg) strcpy(_options.device, optarg);   break;
            case 'w': _options.sleep_us        = strtol(optarg, nullptr, 10);   break;
            case 'b': _options.baudrate        = strtoul(optarg, nullptr, 10);  break;
            case 'p': _options.poll_ms         = strtol(optarg, nullptr, 10);   break;
            case 'r': _options.recv_port       = strtoul(optarg, nullptr, 10);  break;
            case 's': _options.send_port       = strtoul(optarg, nullptr, 10);  break;
            case 'i': if (nullptr != optarg) strcpy(_options.ip, optarg);       break;
            case 'f': _options.sw_flow_control = true;                          break;
            case 'h': _options.hw_flow_control = true;                          break;
            case 'v': _options.verbose_debug = true;                            break;
            case 'n': if (nullptr != optarg) _options.ns = std::string(optarg) + "/"; break;
            default:
                usage(argv[0]);
                return -1;
        }
    }

    if (_options.poll_ms < 1) {
            _options.poll_ms = 1;
            printf("\033[1;33m[   micrortps_agent   ]\tPoll timeout too low, using 1 ms\033[0m");
    }

    if (_options.hw_flow_control && _options.sw_flow_control) {
            printf("\033[0;31m[   micrortps_agent   ]\tHW and SW flow control set. Please set only one or another\033[0m");
            return -1;
    }

    return 0;
}

void signal_handler(int signum)
{
   printf("\033[1;33m[   micrortps_agent   ]\tInterrupt signal (%d) received.\033[0m\n", signum);
   running = 0;
   transport_node->close();
}

@[if recv_topics]@
std::atomic<bool> exit_sender_thread(false);
std::condition_variable t_send_queue_cv;
std::mutex t_send_queue_mutex;
std::queue<uint8_t> t_send_queue;

void t_send(void*)
{
    char data_buffer[BUFFER_SIZE] = {};
    uint32_t length = 0;

    while (running && !exit_sender_thread.load())
    {
        std::unique_lock<std::mutex> lk(t_send_queue_mutex);
        while (t_send_queue.empty() && !exit_sender_thread.load())
        {
            t_send_queue_cv.wait(lk);
        }
        uint8_t topic_ID = t_send_queue.front();
        t_send_queue.pop();
        lk.unlock();

        size_t header_length = transport_node->get_header_length();
        /* make room for the header to fill in later */
        eprosima::fastcdr::FastBuffer cdrbuffer(&data_buffer[header_length], sizeof(data_buffer)-header_length);
        eprosima::fastcdr::Cdr scdr(cdrbuffer);

        if (topics.getMsg(topic_ID, scdr))
        {
            length = scdr.getSerializedDataLength();
            if (0 < (length = transport_node->write(topic_ID, data_buffer, length)))
            {
                total_sent += length;
                ++sent;
            }
        }
    }
}
@[end if]@

int main(int argc, char** argv)
{
    if (-1 == parse_options(argc, argv))
    {
        printf("\033[1;33m[   micrortps_agent   ]\tEXITING...\033[0m\n");
        return -1;
    }

    // register signal SIGINT and signal handler
    signal(SIGINT, signal_handler);

    printf("\033[0;37m--- MicroRTPS Agent ---\033[0m\n");
    printf("[   micrortps_agent   ]\tStarting link...\n");

    switch (_options.transport)
    {
        case options::eTransports::UART:
        {
            transport_node = new UART_node(_options.device, _options.baudrate, _options.poll_ms,
                   _options.sw_flow_control, _options.hw_flow_control, _options.verbose_debug);
            printf("[   micrortps_agent   ]\tUART transport: device: %s; baudrate: %d; sleep: %dus; poll: %dms; flow_control: %s\n",
                   _options.device, _options.baudrate, _options.sleep_us, _options.poll_ms,
                   _options.sw_flow_control ? "SW enabled" : (_options.hw_flow_control ? "HW enabled" : "No"));
        }
        break;
        case options::eTransports::UDP:
        {
            transport_node = new UDP_node(_options.ip, _options.recv_port, _options.send_port, _options.verbose_debug);
            printf("[   micrortps_agent   ]\tUDP transport: ip address: %s; recv port: %u; send port: %u; sleep: %dus\n",
                    _options.ip, _options.recv_port, _options.send_port, _options.sleep_us);
        }
        break;
        default:
            printf("\033[0;37m[   micrortps_agent   ]\tEXITING...\033[0m\n");
        return -1;
    }

    if (0 > transport_node->init())
    {
        printf("\033[0;37m[   micrortps_agent   ]\tEXITING...\033[0m\n");
        return -1;
    }

    sleep(1);

@[if send_topics]@
    char data_buffer[BUFFER_SIZE] = {};
    int received = 0, loop = 0;
    int length = 0, total_read = 0;
    bool receiving = false;
    uint8_t topic_ID = 255;
    std::chrono::time_point<std::chrono::steady_clock> start, end;
@[end if]@

    // Init timesync
    std::shared_ptr<TimeSync> timeSync = std::make_shared<TimeSync>(_options.verbose_debug);

    topics.set_timesync(timeSync);

@[if recv_topics]@
    topics.init(&t_send_queue_cv, &t_send_queue_mutex, &t_send_queue, _options.ns);
@[end if]@

    running = true;
@[if recv_topics]@
    std::thread sender_thread(t_send, nullptr);
@[end if]@

    while (running)
    {
@[if send_topics]@
        ++loop;
        if (!receiving) start = std::chrono::steady_clock::now();
        // Publish messages received from UART
        while (0 < (length = transport_node->read(&topic_ID, data_buffer, BUFFER_SIZE)))
        {
            topics.publish(topic_ID, data_buffer, sizeof(data_buffer));
            ++received;
            total_read += length;
            receiving = true;
            end = std::chrono::steady_clock::now();
        }

        if ((receiving && std::chrono::duration<double>(std::chrono::steady_clock::now() - end).count() > WAIT_CNST) ||
            (!running  && loop > 1))
        {
            std::chrono::duration<double>  elapsed_secs = end - start;
            printf("[   micrortps_agent   ]\tSENT:     %lumessages \t- %lubytes\n", (unsigned long)sent, (unsigned long)total_sent);
            printf("[   micrortps_agent   ]\tRECEIVED: %dmessages \t- %dbytes; %d LOOPS - %.03f seconds - %.02fKB/s\n",
                    received, total_read, loop, elapsed_secs.count(), (double)total_read/(1000*elapsed_secs.count()));
            received = sent = total_read = total_sent = 0;
            receiving = false;
        }
@[else]@
        usleep(_options.sleep_us);
@[end if]@
    }
@[if recv_topics]@
    exit_sender_thread = true;
    t_send_queue_cv.notify_one();
    sender_thread.join();
@[end if]@
    delete transport_node;
    transport_node = nullptr;

    timeSync->stop();
    timeSync->reset();

    return 0;
}
