/****************************************************************************
 *
 *   Copyright (c) 2016-2019 PX4 Development Team. All rights reserved.
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

#pragma once

#include "Replay.hpp"

namespace px4
{

/**
 * @class ReplayEkf2
 * replay specialization for Ekf2 replay
 */
class ReplayEkf2 : public Replay
{
public:
protected:

	void onEnterMainLoop() override;
	void onExitMainLoop() override;

	/**
	 * handle ekf2 topic publication in ekf2 replay mode
	 * @param sub
	 * @param data
	 * @param replay_file file currently replayed (file seek position should be considered arbitrary after this call)
	 * @return true if published, false otherwise
	 */
	bool handleTopicUpdate(Subscription &sub, void *data, std::ifstream &replay_file) override;

	void onSubscriptionAdded(Subscription &sub, uint16_t msg_id) override;

	uint64_t getTimestampOffset() override
	{
		// avoid offsetting timestamps as we use them to compare against the log
		return 0;
	}
private:

	bool publishEkf2Topics(const ekf2_timestamps_s &ekf2_timestamps, std::ifstream &replay_file);

	/**
	 * find the next message for a subscription that matches a given timestamp and publish it
	 * @param timestamp in 0.1 ms
	 * @param msg_id
	 * @param replay_file file currently replayed (file seek position should be considered arbitrary after this call)
	 * @return true if timestamp found and published
	 */
	bool findTimestampAndPublish(uint64_t timestamp, uint16_t msg_id, std::ifstream &replay_file);

	static constexpr uint16_t msg_id_invalid = 0xffff;

	uint16_t _airspeed_msg_id = msg_id_invalid;
	uint16_t _distance_sensor_msg_id = msg_id_invalid;
	uint16_t _optical_flow_msg_id = msg_id_invalid;
	uint16_t _sensor_combined_msg_id = msg_id_invalid;
	uint16_t _vehicle_air_data_msg_id = msg_id_invalid;
	uint16_t _vehicle_magnetometer_msg_id = msg_id_invalid;
	uint16_t _vehicle_visual_odometry_msg_id = msg_id_invalid;
};

} //namespace px4
