@###############################################
@#
@# EmPy template for generating microRTPS_timesync.cpp file
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

package = package[0]
fastrtps_version = fastrtps_version[0]
try:
    ros2_distro = ros2_distro[0].decode("utf-8")
except AttributeError:
    ros2_distro = ros2_distro[0]
}@
/****************************************************************************
 *
 * Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @@file microRTPS_timesync.cpp
 * @@brief source code for time sync implementation
 */

#include <time.h>
#include <cmath>
#include <iostream>

#include "microRTPS_timesync.h"

TimeSync::TimeSync()
    : _offset_ns(-1),
      _skew_ns_per_sync(0.0),
      _num_samples(0),
      _request_reset_counter(0),
      _last_msg_seq(0),
      _last_remote_msg_seq(0)
{ }

TimeSync::~TimeSync() { stop(); }

void TimeSync::start(const TimesyncPublisher* pub) {
	stop();

	_timesync_pub = (*pub);

	auto run = [this]() {
		while (!_request_stop) {
			timesync_msg_t msg = newTimesyncMsg();

			_timesync_pub.publish(&msg);

			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	};
	_request_stop = false;
	_send_timesync_thread.reset(new std::thread(run));
}

void TimeSync::stop() {
	_request_stop = true;
	if (_send_timesync_thread && _send_timesync_thread->joinable()) _send_timesync_thread->join();
	_send_timesync_thread.reset();
}

void TimeSync::reset() {
	_num_samples = 0;
	_request_reset_counter = 0;
}

bool TimeSync::addMeasurement(int64_t local_t1_ns, int64_t remote_t2_ns, int64_t local_t3_ns) {
	int64_t rtti = local_t3_ns - local_t1_ns;

	// assume rtti is evenly split both directions
	int64_t remote_t3_ns = remote_t2_ns + rtti / 2ll;

	int64_t measurement_offset = remote_t3_ns - local_t3_ns;

	if (_request_reset_counter > REQUEST_RESET_COUNTER_THRESHOLD) {
		reset();
		std::cout << std::endl << "Timesync clock changed, resetting" << std::endl;
	}

        if (_num_samples == 0) {
                updateOffset(measurement_offset);
                _skew_ns_per_sync = 0;
        }

	if (_num_samples >= WINDOW_SIZE) {
		if (std::abs(measurement_offset - _offset_ns.load()) > TRIGGER_RESET_THRESHOLD_NS) {
			_request_reset_counter++;
			std::cout << std::endl << "Timesync offset outlier, discarding" << std::endl;
			return false;
		} else {
			_request_reset_counter = 0;
		}
	}

	// ignore if rtti > 10ms
	if (rtti > 15ll * 1000ll * 1000ll) {
		std::cout << std::endl
			  << "RTTI too high for timesync: " << rtti / (1000ll * 1000ll) << "ms" << std::endl;
		return false;
	}

	double alpha = ALPHA_FINAL;
	double beta = BETA_FINAL;

	if (_num_samples < WINDOW_SIZE) {
		double schedule = (double)_num_samples / WINDOW_SIZE;
		double s = 1. - exp(.5 * (1. - 1. / (1. - schedule)));
		alpha = (1. - s) * ALPHA_INITIAL + s * ALPHA_FINAL;
		beta = (1. - s) * BETA_INITIAL + s * BETA_FINAL;
	}

	int64_t offset_prev = _offset_ns.load();
	updateOffset(static_cast<int64_t>((_skew_ns_per_sync + _offset_ns.load()) * (1. - alpha) +
					  measurement_offset * alpha));
	_skew_ns_per_sync =
	    static_cast<int64_t>(beta * (_offset_ns.load() - offset_prev) + (1. - beta) * _skew_ns_per_sync);

	_num_samples++;

	return true;
}

void TimeSync::processTimesyncMsg(timesync_msg_t * msg) {
	if (getMsgSysID(msg) == 1 && getMsgSeq(msg) != _last_remote_msg_seq) {
                _last_remote_msg_seq = getMsgSeq(msg);

		if (getMsgTC1(msg) > 0) {
			if (!addMeasurement(getMsgTS1(msg), getMsgTC1(msg), getMonoRawTimeNSec())) {
				std::cerr << "Offset not updated" << std::endl;
			}

		} else if (getMsgTC1(msg) == 0) {
			setMsgTimestamp(msg, getMonoTimeUSec());
			setMsgSysID(msg, 0);
			setMsgSeq(msg, getMsgSeq(msg) + 1);
			setMsgTC1(msg, getMonoRawTimeNSec());

			_timesync_pub.publish(msg);
		}
	}
}

timesync_msg_t TimeSync::newTimesyncMsg() {
	timesync_msg_t msg{};

	setMsgTimestamp(&msg, getMonoTimeUSec());
	setMsgSysID(&msg, 0);
	setMsgSeq(&msg, _last_msg_seq);
	setMsgTC1(&msg, 0);
	setMsgTS1(&msg, getMonoRawTimeNSec());

	_last_msg_seq++;

	return msg;
}
