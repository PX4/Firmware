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

#include <fstream>
#include <map>
#include <vector>
#include <set>
#include <string>

#include "definitions.hpp"

#include <px4_platform_common/module.h>
#include <uORB/topics/uORBTopics.hpp>
#include <uORB/topics/ekf2_timestamps.h>

namespace px4
{

/**
 * @class Replay
 * Parses an ULog file and replays it in 'real-time'. The timestamp of each replayed message is offset
 * to match the starting time of replay. It keeps a stream for each subscription to find the next message
 * to replay. This is necessary because data messages from different subscriptions don't need to be in
 * monotonic increasing order.
 */
class Replay : public ModuleBase<Replay>
{
public:
	Replay() = default;

	virtual ~Replay();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static Replay *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/**
	 * Apply the parameters from the log
	 * @param quiet do not print an error if true and no log file given via ENV
	 * @return 0 on success
	 */
	static int applyParams(bool quiet);

	/**
	 * Tell the replay module that we want to use replay mode.
	 * After that, only 'replay start' must be executed (typically the last step after startup).
	 * @param file_name file name of the used log replay file. Will be copied.
	 */
	static void setupReplayFile(const char *file_name);

	static bool isSetup() { return _replay_file; }

protected:

	/**
	 * @class Compatibility base class to convert topics to an updated format
	 */
	class CompatBase
	{
	public:
		virtual ~CompatBase() = default;

		/**
		 * apply compatibility to a topic
		 * @param data input topic (can be modified in place)
		 * @return new topic data
		 */
		virtual void *apply(void *data) = 0;
	};

	class CompatSensorCombinedDtType : public CompatBase
	{
	public:
		CompatSensorCombinedDtType(int gyro_integral_dt_offset_log, int gyro_integral_dt_offset_intern,
					   int accelerometer_integral_dt_offset_log, int accelerometer_integral_dt_offset_intern);

		void *apply(void *data) override;
	private:
		int _gyro_integral_dt_offset_log;
		int _gyro_integral_dt_offset_intern;
		int _accelerometer_integral_dt_offset_log;
		int _accelerometer_integral_dt_offset_intern;
	};

	struct Subscription {

		const orb_metadata *orb_meta = nullptr; ///< if nullptr, this subscription is invalid
		orb_advert_t orb_advert = nullptr;
		uint8_t multi_id;
		int timestamp_offset; ///< marks the field of the timestamp

		bool ignored = false; ///< if true, it will not be considered for publication in the main loop

		std::streampos next_read_pos;
		uint64_t next_timestamp; ///< timestamp of the file

		CompatBase *compat = nullptr;

		// statistics
		int error_counter = 0;
		int publication_counter = 0;
	};

	/**
	 * Find the offset & field size in bytes for a given field name
	 * @param format format string, as specified by ULog
	 * @param field_name search for this field
	 * @param offset returned offset
	 * @param field_size returned field size
	 * @return true if found, false otherwise
	 */
	static bool findFieldOffset(const std::string &format, const std::string &field_name, int &offset, int &field_size);

	/**
	 * publish an orb topic
	 * @param sub
	 * @param data
	 * @return true if published, false otherwise
	 */
	bool publishTopic(Subscription &sub, void *data);

	/**
	 * called when entering the main replay loop
	 */
	virtual void onEnterMainLoop() {}

	/**
	 * called when exiting the main replay loop
	 */
	virtual void onExitMainLoop() {}

	/**
	 * called when a new subscription is added
	 */
	virtual void onSubscriptionAdded(Subscription &sub, uint16_t msg_id) {}

	/**
	 * handle delay until topic can be published.
	 * @param next_file_timestamp timestamp of next message to publish
	 * @param timestamp_offset offset between file start time and replay start time
	 * @return timestamp that the message to publish should have
	 */
	virtual uint64_t handleTopicDelay(uint64_t next_file_time, uint64_t timestamp_offset);

	/**
	 * handle the publication of a topic update
	 * @return true if published, false otherwise
	 */
	virtual bool handleTopicUpdate(Subscription &sub, void *data, std::ifstream &replay_file);

	/**
	 * read a topic from the file (offset given by the subscription) into _read_buffer
	 */
	void readTopicDataToBuffer(const Subscription &sub, std::ifstream &replay_file);

	/**
	 * Find next data message for this subscription, starting with the stored file offset.
	 * Skip the first message, and if found, read the timestamp and store the new file offset.
	 * This also takes care of new subscriptions and parameter updates. When reaching EOF,
	 * the subscription is set to invalid.
	 * File seek position is arbitrary after this call.
	 * @return false on file error
	 */
	bool nextDataMessage(std::ifstream &file, Subscription &subscription, int msg_id);

	virtual uint64_t getTimestampOffset()
	{
		//we update the timestamps from the file by a constant offset to match
		//the current replay time
		return _replay_start_time - _file_start_time;
	}

	std::vector<Subscription *> _subscriptions;
	std::vector<uint8_t> _read_buffer;

	float _speed_factor{1.f}; ///< from PX4_SIM_SPEED_FACTOR env variable (set to 0 to avoid usleep = unlimited rate)

private:
	std::set<std::string> _overridden_params;
	std::map<std::string, std::string> _file_formats; ///< all formats we read from the file

	uint64_t _file_start_time;
	uint64_t _replay_start_time;
	std::streampos _data_section_start; ///< first ADD_LOGGED_MSG message

	/** keep track of file position to avoid adding a subscription multiple times. */
	std::streampos _subscription_file_pos = 0;

	int64_t _read_until_file_position = 1ULL << 60; ///< read limit if log contains appended data

	float _accumulated_delay{0.f};

	bool readFileHeader(std::ifstream &file);

	/**
	 * Read definitions section: check formats, apply parameters and store
	 * the start of the data section.
	 * @return true on success
	 */
	bool readFileDefinitions(std::ifstream &file);

	///file parsing methods. They return false, when further parsing should be aborted.
	bool readFormat(std::ifstream &file, uint16_t msg_size);
	bool readAndAddSubscription(std::ifstream &file, uint16_t msg_size);
	bool readFlagBits(std::ifstream &file, uint16_t msg_size);

	/**
	 * Read the file header and definitions sections. Apply the parameters from this section
	 * and apply user-defined overridden parameters.
	 * @return true on success
	 */
	bool readDefinitionsAndApplyParams(std::ifstream &file);

	/**
	 * Read and handle additional messages starting at current file position, while position < end_position.
	 * This handles dropout and parameter update messages.
	 * We need to handle these separately, because they have no timestamp. We look at the file position instead.
	 * @return false on file error
	 */
	bool readAndHandleAdditionalMessages(std::ifstream &file, std::streampos end_position);
	bool readDropout(std::ifstream &file, uint16_t msg_size);
	bool readAndApplyParameter(std::ifstream &file, uint16_t msg_size);

	static const orb_metadata *findTopic(const std::string &name);

	/** get the array size from a type. eg. float[3] -> return float */
	static std::string extractArraySize(const std::string &type_name_full, int &array_size);

	/** get the size of a type that is not an array */
	static size_t sizeOfType(const std::string &type_name);

	/** get the size of a type that can be an array */
	static size_t sizeOfFullType(const std::string &type_name_full);

	void setUserParams(const char *filename);

	static char *_replay_file;
};

} //namespace px4
