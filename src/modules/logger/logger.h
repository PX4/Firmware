/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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

#include "log_writer.h"
#include "messages.h"
#include <containers/Array.hpp>
#include "util.h"
#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>
#include <version/version.h>
#include <parameters/param.h>
#include <px4_platform_common/printload.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/logger_status.h>
#include <uORB/topics/log_message.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/parameter_update.h>

extern "C" __EXPORT int logger_main(int argc, char *argv[]);

using namespace time_literals;

static constexpr hrt_abstime TRY_SUBSCRIBE_INTERVAL{20_ms};	// interval in microseconds at which we try to subscribe to a topic
// if we haven't succeeded before

namespace px4
{
namespace logger
{

static constexpr uint8_t MSG_ID_INVALID = UINT8_MAX;

struct LoggerSubscription : public uORB::SubscriptionInterval {
	LoggerSubscription() = default;

	LoggerSubscription(ORB_ID id, uint32_t interval_ms = 0, uint8_t instance = 0) :
		uORB::SubscriptionInterval(id, interval_ms * 1000, instance)
	{}

	uint8_t msg_id{MSG_ID_INVALID};
};

class Logger : public ModuleBase<Logger>, public ModuleParams
{
public:
	enum class LogMode {
		while_armed = 0,
		boot_until_disarm,
		boot_until_shutdown,
		rc_aux1
	};

	Logger(LogWriter::Backend backend, size_t buffer_size, uint32_t log_interval, const char *poll_topic_name,
	       LogMode log_mode, bool log_name_timestamp);

	~Logger();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static Logger *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	/**
	 * Tell the logger that we're in replay mode. This must be called
	 * before starting the logger.
	 * @param file_name file name of the used log replay file. Will be copied.
	 */
	void setReplayFile(const char *file_name);

	/**
	 * request the logger thread to stop (this method does not block).
	 * @return true if the logger is stopped, false if (still) running
	 */
	static bool request_stop_static();

	void print_statistics(LogType type);

	void set_arm_override(bool override) { _manually_logging_override = override; }

private:

	enum class PrintLoadReason {
		Preflight,
		Postflight,
		Watchdog
	};

	static constexpr int		MAX_MISSION_TOPICS_NUM = 5; /**< Maximum number of mission topics */
	static constexpr unsigned	MAX_NO_LOGFILE = 999;	/**< Maximum number of log files */
	static constexpr const char	*LOG_ROOT[(int)LogType::Count] = {
		PX4_STORAGEDIR "/log",
		PX4_STORAGEDIR "/mission_log"
	};

	struct LogFileName {
		char log_dir[12];           ///< e.g. "2018-01-01" or "sess001"
		int sess_dir_index{1};      ///< search starting index for 'sess<i>' directory name
		char log_file_name[31];     ///< e.g. "log001.ulg" or "12_09_00_replayed.ulg"
		bool has_log_dir{false};
	};

	struct Statistics {
		hrt_abstime start_time_file{0};				///< Time when logging started, file backend (not the logger thread)
		hrt_abstime dropout_start{0};				///< start of current dropout (0 = no dropout)
		float max_dropout_duration{0.0f};			///< max duration of dropout [s]
		size_t write_dropouts{0};				///< failed buffer writes due to buffer overflow
		size_t high_water{0};					///< maximum used write buffer
	};

	struct MissionSubscription {
		unsigned min_delta_ms{0};        ///< minimum time between 2 topic writes [ms]
		unsigned next_write_time{0};     ///< next time to write in 0.1 seconds
	};

	/**
	 * @brief Updates and checks for updated uORB parameters.
	 */
	void update_params();

	/**
	 * Write an ADD_LOGGED_MSG to the log for a all current subscriptions and instances
	 */
	void write_all_add_logged_msg(LogType type);

	/**
	 * Write an ADD_LOGGED_MSG to the log for a given subscription and instance.
	 * _writer.lock() must be held when calling this.
	 */
	void write_add_logged_msg(LogType type, LoggerSubscription &subscription);

	/**
	 * Create logging directory
	 * @param type
	 * @param tt if not null, use it for the directory name
	 * @param log_dir returned log directory path
	 * @param log_dir_len log_dir buffer length
	 * @return string length of log_dir (excluding terminating null-char), <0 on error
	 */
	int create_log_dir(LogType type, tm *tt, char *log_dir, int log_dir_len);

	/**
	 * Get log file name with directory (create it if necessary)
	 */
	int get_log_file_name(LogType type, char *file_name, size_t file_name_size);

	void start_log_file(LogType type);

	void stop_log_file(LogType type);

	void start_log_mavlink();

	void stop_log_mavlink();

	/** check if mavlink logging can be started */
	bool can_start_mavlink_log() const
	{
		return !_writer.is_started(LogType::Full, LogWriter::BackendMavlink)
		       && (_writer.backend() & LogWriter::BackendMavlink) != 0;
	}

	/** get the configured backend as string */
	const char *configured_backend_mode() const;

	/**
	 * write the file header with file magic and timestamp.
	 */
	void write_header(LogType type);

	/// Array to store written formats for nested definitions (only)
	using WrittenFormats = Array < const orb_metadata *, 20 >;

	void write_format(LogType type, const orb_metadata &meta, WrittenFormats &written_formats, ulog_message_format_s &msg,
			  int subscription_index, int level = 1);
	void write_formats(LogType type);

	/**
	 * write performance counters
	 * @param preflight preflight if true, postflight otherwise
	 */
	void write_perf_data(bool preflight);

	/**
	 * write bootup console output
	 */
	void write_console_output();

	/**
	 * callback to write the performance counters
	 */
	static void perf_iterate_callback(perf_counter_t handle, void *user);

	/**
	 * callback for print_load_buffer() to print the process load
	 */
	static void print_load_callback(void *user);

	void write_version(LogType type);

	void write_info(LogType type, const char *name, const char *value);
	void write_info_multiple(LogType type, const char *name, const char *value, bool is_continued);
	void write_info(LogType type, const char *name, int32_t value);
	void write_info(LogType type, const char *name, uint32_t value);

	/** generic common template method for write_info variants */
	template<typename T>
	void write_info_template(LogType type, const char *name, T value, const char *type_str);

	void write_parameters(LogType type);
	void write_parameter_defaults(LogType type);

	void write_changed_parameters(LogType type);

	inline bool copy_if_updated(int sub_idx, void *buffer, bool try_to_subscribe);

	/**
	 * Write exactly one ulog message to the logger and handle dropouts.
	 * Must be called with _writer.lock() held.
	 * @return true if data written, false otherwise (on overflow)
	 */
	bool write_message(LogType type, void *ptr, size_t size);

	/**
	 * Add topic subscriptions from SD file if it exists, otherwise add topics based on the configured profile.
	 * This must be called before start_log() (because it does not write an ADD_LOGGED_MSG message).
	 * @return true on success
	 */
	bool initialize_topics();

	/**
	 * Determines if log-from-boot should be disabled, based on the value of SDLOG_BOOT_BAT and the battery status.
	 * @return true if log-from-boot should be disabled
	 */
	bool get_disable_boot_logging();

	/**
	 * check current arming state or aux channel and start/stop logging if state changed and according to
	 * configured params.
	 * @return true if log started
	 */
	bool start_stop_logging();

	void handle_vehicle_command_update();
	void ack_vehicle_command(vehicle_command_s *cmd, uint32_t result);

	/**
	 * initialize the output for the process load, so that ~1 second later it will be written to the log
	 */
	void initialize_load_output(PrintLoadReason reason);

	/**
	 * write the process load, which was previously initialized with initialize_load_output()
	 */
	void write_load_output();

	/**
	 * Regularly print the buffer fill state (only if DBGPRINT is set)
	 * @param total_bytes total written bytes (to the full file), will be reset on each print
	 * @param timer_start time since last print
	 */
	inline void debug_print_buffer(uint32_t &total_bytes, hrt_abstime &timer_start);

	void publish_logger_status();

	uint8_t						*_msg_buffer{nullptr};
	int						_msg_buffer_len{0};

	LogFileName					_file_name[(int)LogType::Count];

	bool						_prev_state{false}; ///< previous state depending on logging mode (arming or aux1 state)
	bool						_manually_logging_override{false};

	Statistics					_statistics[(int)LogType::Count];
	hrt_abstime					_last_sync_time{0}; ///< last time a sync msg was sent

	LogMode						_log_mode;
	const bool					_log_name_timestamp;

	LoggerSubscription	 			*_subscriptions{nullptr}; ///< all subscriptions for full & mission log (in front)
	int						_num_subscriptions{0};
	MissionSubscription 				_mission_subscriptions[MAX_MISSION_TOPICS_NUM] {}; ///< additional data for mission subscriptions
	int						_num_mission_subs{0};

	LogWriter					_writer;
	uint32_t					_log_interval{0};
	const orb_metadata				*_polling_topic_meta{nullptr}; ///< if non-null, poll on this topic instead of sleeping
	orb_advert_t					_mavlink_log_pub{nullptr};
	uint8_t						_next_topic_id{0}; ///< id of next subscribed ulog topic
	char						*_replay_file_name{nullptr};
	bool						_should_stop_file_log{false}; /**< if true _next_load_print is set and file logging
											will be stopped after load printing (for the full log) */
	print_load_s					_load{}; ///< process load data
	hrt_abstime					_next_load_print{0}; ///< timestamp when to print the process load
	PrintLoadReason					_print_load_reason {PrintLoadReason::Preflight};

	uORB::PublicationMulti<logger_status_s>		_logger_status_pub[(int)LogType::Count] { ORB_ID(logger_status), ORB_ID(logger_status) };

	hrt_abstime					_logger_status_last {0};
	int						_lockstep_component{-1};

	uint32_t					_message_gaps{0};

	uORB::Subscription				_manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription				_vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription				_vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::SubscriptionInterval			_log_message_sub{ORB_ID(log_message), 20};
	uORB::SubscriptionInterval			_parameter_update_sub{ORB_ID(parameter_update), 1_s};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SDLOG_UTC_OFFSET>) _param_sdlog_utc_offset,
		(ParamInt<px4::params::SDLOG_DIRS_MAX>) _param_sdlog_dirs_max,
		(ParamInt<px4::params::SDLOG_PROFILE>) _param_sdlog_profile,
		(ParamInt<px4::params::SDLOG_MISSION>) _param_sdlog_mission,
		(ParamBool<px4::params::SDLOG_BOOT_BAT>) _param_sdlog_boot_bat,
		(ParamBool<px4::params::SDLOG_UUID>) _param_sdlog_uuid
	)
};

} //namespace logger
} //namespace px4
