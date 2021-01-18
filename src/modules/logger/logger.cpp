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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/console_buffer.h>
#include "logged_topics.h"
#include "logger.h"
#include "messages.h"
#include "watchdog.h"

#include <dirent.h>
#include <sys/stat.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/uORBTopics.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/battery_status.h>

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>
#include <px4_platform/cpuload.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/sem.h>
#include <px4_platform_common/shutdown.h>
#include <px4_platform_common/tasks.h>
#include <systemlib/mavlink_log.h>
#include <replay/definitions.hpp>
#include <version/version.h>

//#define DBGPRINT //write status output every few seconds

#if defined(DBGPRINT)
// needed for mallinfo
#if defined(__PX4_POSIX) && !defined(__PX4_DARWIN)
#include <malloc.h>
#endif /* __PX4_POSIX */

// struct mallinfo not available on OSX?
#if defined(__PX4_DARWIN)
#undef DBGPRINT
#endif /* defined(__PX4_DARWIN) */
#endif /* defined(DBGPRINT) */

using namespace px4::logger;
using namespace time_literals;


struct timer_callback_data_s {
	px4_sem_t semaphore;

	watchdog_data_t watchdog_data;
	volatile bool watchdog_triggered = false;
};

/* This is used to schedule work for the logger (periodic scan for updated topics) */
static void timer_callback(void *arg)
{
	/* Note: we are in IRQ context here (on NuttX) */

	timer_callback_data_s *data = (timer_callback_data_s *)arg;

	if (watchdog_update(data->watchdog_data)) {
		data->watchdog_triggered = true;
	}

	/* check the value of the semaphore: if the logger cannot keep up with running it's main loop as fast
	 * as the timer_callback here increases the semaphore count, the counter would increase unbounded,
	 * leading to an overflow at some point. This case we want to avoid here, so we check the current
	 * value against a (somewhat arbitrary) threshold, and avoid calling sem_post() if it's exceeded.
	 * (it's not a problem if the threshold is a bit too large, it just means the logger will do
	 * multiple iterations at once, the next time it's scheduled). */
	int semaphore_value;

	if (px4_sem_getvalue(&data->semaphore, &semaphore_value) == 0 && semaphore_value > 1) {
		return;
	}

	px4_sem_post(&data->semaphore);

}


int logger_main(int argc, char *argv[])
{
	// logger currently assumes little endian
	int num = 1;

	if (*(char *)&num != 1) {
		PX4_ERR("Logger only works on little endian!\n");
		return 1;
	}

	return Logger::main(argc, argv);
}

namespace px4
{
namespace logger
{

constexpr const char *Logger::LOG_ROOT[(int)LogType::Count];

int Logger::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("logger not running");
		return 1;
	}

	if (!strcmp(argv[0], "on")) {
		get_instance()->set_arm_override(true);
		return 0;
	}

	if (!strcmp(argv[0], "off")) {
		get_instance()->set_arm_override(false);
		return 0;
	}

	return print_usage("unknown command");
}

int Logger::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("logger",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_LOG_CAPTURE,
				      3700,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

int Logger::print_status()
{
	PX4_INFO("Running in mode: %s", configured_backend_mode());
	PX4_INFO("Number of subscriptions: %i (%i bytes)", _num_subscriptions,
		 (int)(_num_subscriptions * sizeof(LoggerSubscription)));

	bool is_logging = false;

	if (_writer.is_started(LogType::Full, LogWriter::BackendFile)) {
		PX4_INFO("Full File Logging Running:");
		print_statistics(LogType::Full);
		is_logging = true;
	}

	if (_writer.is_started(LogType::Mission, LogWriter::BackendFile)) {
		PX4_INFO("Mission File Logging Running:");
		print_statistics(LogType::Mission);
		is_logging = true;
	}

	if (_writer.is_started(LogType::Full, LogWriter::BackendMavlink)) {
		PX4_INFO("Mavlink Logging Running (Full log)");
		is_logging = true;
	}

	if (!is_logging) {
		PX4_INFO("Not logging");
	}

	return 0;
}

void Logger::print_statistics(LogType type)
{
	if (!_writer.is_started(type, LogWriter::BackendFile)) { //currently only statistics for file logging
		return;
	}

	Statistics &stats = _statistics[(int)type];

	/* this is only for the file backend */
	float kibibytes = _writer.get_total_written_file(type) / 1024.0f;
	float mebibytes = kibibytes / 1024.0f;
	float seconds = ((float)(hrt_absolute_time() - stats.start_time_file)) / 1000000.0f;

	PX4_INFO("Log file: %s/%s/%s", LOG_ROOT[(int)type], _file_name[(int)type].log_dir, _file_name[(int)type].log_file_name);

	if (mebibytes < 0.1f) {
		PX4_INFO("Wrote %4.2f KiB (avg %5.2f KiB/s)", (double)kibibytes, (double)(kibibytes / seconds));

	} else {
		PX4_INFO("Wrote %4.2f MiB (avg %5.2f KiB/s)", (double)mebibytes, (double)(kibibytes / seconds));
	}

	PX4_INFO("Since last status: dropouts: %zu (max len: %.3f s), max used buffer: %zu / %zu B",
		 stats.write_dropouts, (double)stats.max_dropout_duration, stats.high_water, _writer.get_buffer_size_file(type));
	stats.high_water = 0;
	stats.write_dropouts = 0;
	stats.max_dropout_duration = 0.f;
}

Logger *Logger::instantiate(int argc, char *argv[])
{
	uint32_t log_interval = 3500;
	int log_buffer_size = 12 * 1024;
	Logger::LogMode log_mode = Logger::LogMode::while_armed;
	bool error_flag = false;
	bool log_name_timestamp = false;
	LogWriter::Backend backend = LogWriter::BackendAll;
	const char *poll_topic = nullptr;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "r:b:etfm:p:x", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'r': {
				unsigned long r = strtoul(myoptarg, nullptr, 10);

				if (r <= 0) {
					r = 1e6;
				}

				log_interval = 1e6 / r;
			}
			break;

		case 'x':
			log_mode = Logger::LogMode::rc_aux1;
			break;

		case 'e':
			if (log_mode != Logger::LogMode::boot_until_shutdown) {
				//setting boot_until_shutdown can't lower mode to boot_until_disarm
				log_mode = Logger::LogMode::boot_until_disarm;

			}

			break;

		case 'f':
			log_mode = Logger::LogMode::boot_until_shutdown;
			break;

		case 'b': {
				unsigned long s = strtoul(myoptarg, nullptr, 10);

				if (s < 1) {
					s = 1;
				}

				log_buffer_size = 1024 * s;
			}
			break;

		case 't':
			log_name_timestamp = true;
			break;


		case 'm':
			if (!strcmp(myoptarg, "file")) {
				backend = LogWriter::BackendFile;

			} else if (!strcmp(myoptarg, "mavlink")) {
				backend = LogWriter::BackendMavlink;

			} else if (!strcmp(myoptarg, "all")) {
				backend = LogWriter::BackendAll;

			} else {
				PX4_ERR("unknown mode: %s", myoptarg);
				error_flag = true;
			}

			break;

		case 'p':
			poll_topic = myoptarg;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	Logger *logger = new Logger(backend, log_buffer_size, log_interval, poll_topic, log_mode, log_name_timestamp);

#if defined(DBGPRINT) && defined(__PX4_NUTTX)
	struct mallinfo alloc_info = mallinfo();
	PX4_INFO("largest free chunk: %d bytes", alloc_info.mxordblk);
	PX4_INFO("remaining free heap: %d bytes", alloc_info.fordblks);
#endif /* DBGPRINT */

	if (logger == nullptr) {
		PX4_ERR("alloc failed");

	} else {
#ifndef __PX4_NUTTX
		//check for replay mode
		const char *logfile = getenv(px4::replay::ENV_FILENAME);

		if (logfile) {
			logger->setReplayFile(logfile);
		}

#endif /* __PX4_NUTTX */

	}

	return logger;
}

Logger::Logger(LogWriter::Backend backend, size_t buffer_size, uint32_t log_interval, const char *poll_topic_name,
	       LogMode log_mode, bool log_name_timestamp) :
	ModuleParams(nullptr),
	_log_mode(log_mode),
	_log_name_timestamp(log_name_timestamp),
	_writer(backend, buffer_size),
	_log_interval(log_interval)
{
	if (poll_topic_name) {
		const orb_metadata *const *topics = orb_get_topics();

		for (size_t i = 0; i < orb_topics_count(); i++) {
			if (strcmp(poll_topic_name, topics[i]->o_name) == 0) {
				_polling_topic_meta = topics[i];
				break;
			}
		}

		if (!_polling_topic_meta) {
			PX4_ERR("Failed to find topic %s", poll_topic_name);
		}
	}
}

Logger::~Logger()
{
	if (_replay_file_name) {
		free(_replay_file_name);
	}

	delete[](_msg_buffer);
	delete[](_subscriptions);
}

void Logger::update_params()
{
	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		ModuleParams::updateParams();
	}
}

bool Logger::request_stop_static()
{
	if (is_running()) {
		get_instance()->request_stop();
		return false;
	}

	return true;
}

bool Logger::copy_if_updated(int sub_idx, void *buffer, bool try_to_subscribe)
{
	LoggerSubscription &sub = _subscriptions[sub_idx];

	bool updated = false;

	if (sub.valid()) {
		if (sub.get_interval_us() == 0) {
			// record gaps in full rate (no interval) messages
			const unsigned last_generation = sub.get_last_generation();
			updated = sub.update(buffer);

			if (updated && (sub.get_last_generation() != last_generation + 1)) {
				// error, missed a message
				_message_gaps++;
			}

		} else {
			updated = sub.update(buffer);
		}

	} else if (try_to_subscribe) {
		if (sub.subscribe()) {
			write_add_logged_msg(LogType::Full, sub);

			if (sub_idx < _num_mission_subs) {
				write_add_logged_msg(LogType::Mission, sub);
			}

			// copy first data
			updated = sub.copy(buffer);
		}
	}

	return updated;
}

const char *Logger::configured_backend_mode() const
{
	switch (_writer.backend()) {
	case LogWriter::BackendFile: return "file";

	case LogWriter::BackendMavlink: return "mavlink";

	case LogWriter::BackendAll: return "all";

	default: return "several";
	}
}

bool Logger::initialize_topics()
{
	// get the logging profile
	SDLogProfileMask sdlog_profile = (SDLogProfileMask)_param_sdlog_profile.get();

	if ((int32_t)sdlog_profile == 0) {
		PX4_WARN("No logging profile selected. Using default set");
		sdlog_profile = SDLogProfileMask::DEFAULT;
	}

	LoggedTopics logged_topics;

	// initialize mission topics
	logged_topics.initialize_mission_topics((MissionLogType)_param_sdlog_mission.get());
	_num_mission_subs = logged_topics.numMissionSubscriptions();

	if (_num_mission_subs > 0) {
		if (_num_mission_subs >= MAX_MISSION_TOPICS_NUM) {
			PX4_ERR("Max num mission topics exceeded (%i)", _num_mission_subs);
			_num_mission_subs = MAX_MISSION_TOPICS_NUM;
		}

		for (int i = 0; i < _num_mission_subs; ++i) {
			_mission_subscriptions[i].min_delta_ms = logged_topics.subscriptions().sub[i].interval_ms;
			_mission_subscriptions[i].next_write_time = 0;
		}

		int mkdir_ret = mkdir(LOG_ROOT[(int)LogType::Mission], S_IRWXU | S_IRWXG | S_IRWXO);

		if (mkdir_ret != 0 && errno != EEXIST) {
			PX4_ERR("failed creating log root dir: %s (%i)", LOG_ROOT[(int)LogType::Mission], errno);
		}
	}

	if (!logged_topics.initialize_logged_topics(sdlog_profile)) {
		return false;
	}

	if ((sdlog_profile & SDLogProfileMask::RAW_IMU_ACCEL_FIFO) || (sdlog_profile & SDLogProfileMask::RAW_IMU_GYRO_FIFO)) {
		// if we are logging high-rate FIFO, reduce the logging interval & increase process priority to avoid missing samples
		PX4_INFO("Logging FIFO data: increasing task prio and logging rate");
		_log_interval = 800;
		sched_param param{};
		param.sched_priority = SCHED_PRIORITY_ATTITUDE_CONTROL;
		int ret = pthread_setschedparam(pthread_self(), SCHED_DEFAULT, &param);

		if (ret != 0) {
			PX4_ERR("pthread_setschedparam failed (%i)", ret);
		}
	}

	delete[](_subscriptions);
	_subscriptions = nullptr;

	if (logged_topics.subscriptions().count > 0) {
		_subscriptions = new LoggerSubscription[logged_topics.subscriptions().count];

		if (!_subscriptions) {
			PX4_ERR("alloc failed");
			return false;
		}

		for (int i = 0; i < logged_topics.subscriptions().count; ++i) {
			const LoggedTopics::RequestedSubscription &sub = logged_topics.subscriptions().sub[i];
			_subscriptions[i] = LoggerSubscription(sub.id, sub.interval_ms, sub.instance);
			_subscriptions[i].subscribe();
		}
	}

	_num_subscriptions = logged_topics.subscriptions().count;
	return true;
}

void Logger::run()
{
	PX4_INFO("logger started (mode=%s)", configured_backend_mode());

	if (_writer.backend() & LogWriter::BackendFile) {
		int mkdir_ret = mkdir(LOG_ROOT[(int)LogType::Full], S_IRWXU | S_IRWXG | S_IRWXO);

		if (mkdir_ret == 0) {
			PX4_INFO("log root dir created: %s", LOG_ROOT[(int)LogType::Full]);

		} else if (errno != EEXIST) {
			PX4_ERR("failed creating log root dir: %s (%i)", LOG_ROOT[(int)LogType::Full], errno);

			if ((_writer.backend() & ~LogWriter::BackendFile) == 0) {
				return;
			}
		}

		if (util::check_free_space(LOG_ROOT[(int)LogType::Full], _param_sdlog_dirs_max.get(), _mavlink_log_pub,
					   _file_name[(int)LogType::Full].sess_dir_index) == 1) {
			return;
		}
	}

	uORB::Subscription parameter_update_sub(ORB_ID(parameter_update));

	if (!initialize_topics()) {
		return;
	}

	//all topics added. Get required message buffer size
	int max_msg_size = 0;

	for (int sub = 0; sub < _num_subscriptions; ++sub) {
		//use o_size, because that's what orb_copy will use
		if (_subscriptions[sub].get_topic()->o_size > max_msg_size) {
			max_msg_size = _subscriptions[sub].get_topic()->o_size;
		}
	}

	max_msg_size += sizeof(ulog_message_data_header_s);

	if (sizeof(ulog_message_logging_s) > (size_t)max_msg_size) {
		max_msg_size = sizeof(ulog_message_logging_s);
	}

	if (_polling_topic_meta && _polling_topic_meta->o_size > max_msg_size) {
		max_msg_size = _polling_topic_meta->o_size;
	}

	if (max_msg_size > _msg_buffer_len) {
		if (_msg_buffer) {
			delete[](_msg_buffer);
		}

		_msg_buffer_len = max_msg_size;
		_msg_buffer = new uint8_t[_msg_buffer_len];

		if (!_msg_buffer) {
			PX4_ERR("failed to alloc message buffer");
			return;
		}
	}


	if (!_writer.init()) {
		PX4_ERR("writer init failed");
		return;
	}

	/* debug stats */
	hrt_abstime	timer_start = 0;
	uint32_t	total_bytes = 0;

	px4_register_shutdown_hook(&Logger::request_stop_static);

	const bool disable_boot_logging = get_disable_boot_logging();

	if ((_log_mode == LogMode::boot_until_disarm || _log_mode == LogMode::boot_until_shutdown) && !disable_boot_logging) {
		start_log_file(LogType::Full);
	}

	/* init the update timer */
	struct hrt_call timer_call {};
	timer_callback_data_s timer_callback_data;
	px4_sem_init(&timer_callback_data.semaphore, 0, 0);
	/* timer_semaphore use case is a signal */
	px4_sem_setprotocol(&timer_callback_data.semaphore, SEM_PRIO_NONE);

	int polling_topic_sub = -1;

	if (_polling_topic_meta) {
		polling_topic_sub = orb_subscribe(_polling_topic_meta);

		if (polling_topic_sub < 0) {
			PX4_ERR("Failed to subscribe (%i)", errno);
		}

	} else {

		if (_writer.backend() & LogWriter::BackendFile) {

			const pid_t pid_self = getpid();
			const pthread_t writer_thread = _writer.thread_id_file();

			// sched_note_start is already called from pthread_create and task_create,
			// which means we can expect to find the tasks in system_load.tasks, as required in watchdog_initialize
			watchdog_initialize(pid_self, writer_thread, timer_callback_data.watchdog_data);
		}

		hrt_call_every(&timer_call, _log_interval, _log_interval, timer_callback, &timer_callback_data);
	}

	// check for new subscription data
	hrt_abstime next_subscribe_check = 0;
	int next_subscribe_topic_index = -1; // this is used to distribute the checks over time

	if (polling_topic_sub >= 0) {
		_lockstep_component = px4_lockstep_register_component();
	}

	while (!should_exit()) {
		// Start/stop logging (depending on logging mode, by default when arming/disarming)
		const bool logging_started = start_stop_logging();

		if (logging_started) {
#ifdef DBGPRINT
			timer_start = hrt_absolute_time();
			total_bytes = 0;
#endif /* DBGPRINT */
		}

		/* check for logging command from MAVLink (start/stop streaming) */
		handle_vehicle_command_update();

		if (timer_callback_data.watchdog_triggered) {
			timer_callback_data.watchdog_triggered = false;
			initialize_load_output(PrintLoadReason::Watchdog);
		}


		const hrt_abstime loop_time = hrt_absolute_time();

		if (_writer.is_started(LogType::Full)) { // mission log only runs when full log is also started

			/* check if we need to output the process load */
			if (_next_load_print != 0 && loop_time >= _next_load_print) {
				_next_load_print = 0;
				write_load_output();

				if (_should_stop_file_log) {
					_should_stop_file_log = false;
					stop_log_file(LogType::Full);
					continue; // skip to next loop iteration
				}
			}

			/* Check if parameters have changed */
			if (!_should_stop_file_log) { // do not record param changes after disarming
				if (parameter_update_sub.updated()) {
					// clear update
					parameter_update_s pupdate;
					parameter_update_sub.copy(&pupdate);

					write_changed_parameters(LogType::Full);
				}
			}

			/* wait for lock on log buffer */
			_writer.lock();

			for (int sub_idx = 0; sub_idx < _num_subscriptions; ++sub_idx) {
				LoggerSubscription &sub = _subscriptions[sub_idx];
				/* if this topic has been updated, copy the new data into the message buffer
				 * and write a message to the log
				 */
				const bool try_to_subscribe = (sub_idx == next_subscribe_topic_index);

				if (copy_if_updated(sub_idx, _msg_buffer + sizeof(ulog_message_data_header_s), try_to_subscribe)) {
					// each message consists of a header followed by an orb data object
					const size_t msg_size = sizeof(ulog_message_data_header_s) + sub.get_topic()->o_size_no_padding;
					const uint16_t write_msg_size = static_cast<uint16_t>(msg_size - ULOG_MSG_HEADER_LEN);
					const uint16_t write_msg_id = sub.msg_id;

					//write one byte after another (necessary because of alignment)
					_msg_buffer[0] = (uint8_t)write_msg_size;
					_msg_buffer[1] = (uint8_t)(write_msg_size >> 8);
					_msg_buffer[2] = static_cast<uint8_t>(ULogMessageType::DATA);
					_msg_buffer[3] = (uint8_t)write_msg_id;
					_msg_buffer[4] = (uint8_t)(write_msg_id >> 8);

					// PX4_INFO("topic: %s, size = %zu, out_size = %zu", sub.get_topic()->o_name, sub.get_topic()->o_size, msg_size);

					// full log
					if (write_message(LogType::Full, _msg_buffer, msg_size)) {

#ifdef DBGPRINT
						total_bytes += msg_size;
#endif /* DBGPRINT */
					}

					// mission log
					if (sub_idx < _num_mission_subs) {
						if (_writer.is_started(LogType::Mission)) {
							if (_mission_subscriptions[sub_idx].next_write_time < (loop_time / 100000)) {
								unsigned delta_time = _mission_subscriptions[sub_idx].min_delta_ms;

								if (delta_time > 0) {
									_mission_subscriptions[sub_idx].next_write_time = (loop_time / 100000) + delta_time / 100;
								}

								write_message(LogType::Mission, _msg_buffer, msg_size);
							}
						}
					}
				}
			}

			// check for new logging message(s)
			log_message_s log_message;

			if (_log_message_sub.update(&log_message)) {
				const char *message = (const char *)log_message.text;
				int message_len = strlen(message);

				if (message_len > 0) {
					uint16_t write_msg_size = sizeof(ulog_message_logging_s) - sizeof(ulog_message_logging_s::message)
								  - ULOG_MSG_HEADER_LEN + message_len;
					_msg_buffer[0] = (uint8_t)write_msg_size;
					_msg_buffer[1] = (uint8_t)(write_msg_size >> 8);
					_msg_buffer[2] = static_cast<uint8_t>(ULogMessageType::LOGGING);
					_msg_buffer[3] = log_message.severity + '0';
					memcpy(_msg_buffer + 4, &log_message.timestamp, sizeof(ulog_message_logging_s::timestamp));
					strncpy((char *)(_msg_buffer + 12), message, sizeof(ulog_message_logging_s::message));

					write_message(LogType::Full, _msg_buffer, write_msg_size + ULOG_MSG_HEADER_LEN);
				}
			}

			// Add sync magic
			if (loop_time - _last_sync_time > 500_ms) {
				uint16_t write_msg_size = static_cast<uint16_t>(sizeof(ulog_message_sync_s) - ULOG_MSG_HEADER_LEN);
				_msg_buffer[0] = (uint8_t)write_msg_size;
				_msg_buffer[1] = (uint8_t)(write_msg_size >> 8);
				_msg_buffer[2] = static_cast<uint8_t>(ULogMessageType::SYNC);

				// sync byte sequence
				_msg_buffer[3] = 0x2F;
				_msg_buffer[4] = 0x73;
				_msg_buffer[5] = 0x13;
				_msg_buffer[6] = 0x20;
				_msg_buffer[7] = 0x25;
				_msg_buffer[8] = 0x0C;
				_msg_buffer[9] = 0xBB;
				_msg_buffer[10] = 0x12;

				write_message(LogType::Full, _msg_buffer, write_msg_size + ULOG_MSG_HEADER_LEN);
				_last_sync_time = loop_time;
			}

			// update buffer statistics
			for (int i = 0; i < (int)LogType::Count; ++i) {
				if (!_statistics[i].dropout_start && (_writer.get_buffer_fill_count_file((LogType)i) > _statistics[i].high_water)) {
					_statistics[i].high_water = _writer.get_buffer_fill_count_file((LogType)i);
				}
			}

			publish_logger_status();

			/* release the log buffer */
			_writer.unlock();

			/* notify the writer thread */
			_writer.notify();

			/* subscription update */
			if (next_subscribe_topic_index != -1) {
				if (++next_subscribe_topic_index >= _num_subscriptions) {
					next_subscribe_topic_index = -1;
					next_subscribe_check = loop_time + TRY_SUBSCRIBE_INTERVAL;
				}

			} else if (loop_time > next_subscribe_check) {
				next_subscribe_topic_index = 0;
			}

			debug_print_buffer(total_bytes, timer_start);

		} else { // not logging

			// try to subscribe to new topics, even if we don't log, so that:
			// - we avoid subscribing to many topics at once, when logging starts
			// - we'll get the data immediately once we start logging (no need to wait for the next subscribe timeout)
			if (next_subscribe_topic_index != -1) {
				if (!_subscriptions[next_subscribe_topic_index].valid()) {
					_subscriptions[next_subscribe_topic_index].subscribe();
				}

				if (++next_subscribe_topic_index >= _num_subscriptions) {
					next_subscribe_topic_index = -1;
					next_subscribe_check = loop_time + TRY_SUBSCRIBE_INTERVAL;
				}

			} else if (loop_time > next_subscribe_check) {
				next_subscribe_topic_index = 0;
			}
		}

		update_params();

		// wait for next loop iteration...
		if (polling_topic_sub >= 0) {
			px4_lockstep_progress(_lockstep_component);

			px4_pollfd_struct_t fds[1];
			fds[0].fd = polling_topic_sub;
			fds[0].events = POLLIN;
			int pret = px4_poll(fds, 1, 20);

			if (pret < 0) {
				PX4_ERR("poll failed (%i)", pret);

			} else if (pret != 0) {
				if (fds[0].revents & POLLIN) {
					// need to to an orb_copy so that the next poll will not return immediately
					orb_copy(_polling_topic_meta, polling_topic_sub, _msg_buffer);
				}
			}

		} else {
			/*
			 * We wait on the semaphore, which periodically gets updated by a high-resolution timer.
			 * The simpler alternative would be:
			 *   usleep(max(300, _log_interval - elapsed_time_since_loop_start));
			 * And on linux this is quite accurate as well, but under NuttX it is not accurate,
			 * because usleep() has only a granularity of CONFIG_MSEC_PER_TICK (=1ms).
			 */
			while (px4_sem_wait(&timer_callback_data.semaphore) != 0) {}
		}
	}

	px4_lockstep_unregister_component(_lockstep_component);

	stop_log_file(LogType::Full);
	stop_log_file(LogType::Mission);

	hrt_cancel(&timer_call);
	px4_sem_destroy(&timer_callback_data.semaphore);

	// stop the writer thread
	_writer.thread_stop();

	if (polling_topic_sub >= 0) {
		orb_unsubscribe(polling_topic_sub);
	}

	if (_mavlink_log_pub) {
		orb_unadvertise(_mavlink_log_pub);
		_mavlink_log_pub = nullptr;
	}

	px4_unregister_shutdown_hook(&Logger::request_stop_static);
}

void Logger::debug_print_buffer(uint32_t &total_bytes, hrt_abstime &timer_start)
{
#ifdef DBGPRINT
	double deltat = (double)(hrt_absolute_time() - timer_start)  * 1e-6;

	if (deltat > 4.0) {
		struct mallinfo alloc_info = mallinfo();
		double throughput = total_bytes / deltat;
		PX4_INFO("%8.1f kB/s, %zu highWater,  %d dropouts, %5.3f sec max, free heap: %d",
			 throughput / 1.e3, _statistics[0].high_water, _statistics[0].write_dropouts,
			 (double)_statistics[0].max_dropout_duration, alloc_info.fordblks);

		_statistics[0].high_water = 0;
		_statistics[0].max_dropout_duration = 0.f;
		total_bytes = 0;
		timer_start = hrt_absolute_time();
	}

#endif /* DBGPRINT */
}

void Logger::publish_logger_status()
{
	if (hrt_elapsed_time(&_logger_status_last) >= 1_s) {
		for (int i = 0; i < (int)LogType::Count; ++i) {

			const LogType log_type = static_cast<LogType>(i);

			if (_writer.is_started(log_type)) {

				const size_t buffer_fill_count_file = _writer.get_buffer_fill_count_file(log_type);

				const float kb_written = _writer.get_total_written_file(log_type) / 1024.0f;
				const float seconds = hrt_elapsed_time(&_statistics[i].start_time_file) * 1e-6f;

				logger_status_s status;
				status.type = i;
				status.backend = _writer.backend();
				status.total_written_kb = kb_written;
				status.write_rate_kb_s = kb_written / seconds;
				status.dropouts = _statistics[i].write_dropouts;
				status.message_gaps = _message_gaps;
				status.buffer_used_bytes = buffer_fill_count_file;
				status.buffer_size_bytes = _writer.get_buffer_size_file(log_type);
				status.num_messages = _num_subscriptions;
				status.timestamp = hrt_absolute_time();
				_logger_status_pub[i].publish(status);
			}
		}

		_logger_status_last = hrt_absolute_time();
	}
}

bool Logger::get_disable_boot_logging()
{
	if (_param_sdlog_boot_bat.get()) {

		uORB::SubscriptionMultiArray<battery_status_s, battery_status_s::MAX_INSTANCES> battery_status_subs{ORB_ID::battery_status};

		for (auto &sub : battery_status_subs) {
			battery_status_s battery_status;

			if (sub.copy(&battery_status)) {
				if ((hrt_elapsed_time(&battery_status.timestamp) < 2_s) && (battery_status.voltage_v > 0.f)) {
					return true;
				}
			}
		}

		PX4_WARN("battery_status not available. Logging anyway");
	}

	return false;
}

bool Logger::start_stop_logging()
{
	bool updated = false;
	bool desired_state = false;

	if (_log_mode == LogMode::rc_aux1) {
		// aux1-based logging
		manual_control_setpoint_s manual_control_setpoint;

		if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {

			desired_state = (manual_control_setpoint.aux1 > 0.3f);
			updated = true;
		}

	} else if (_log_mode != LogMode::boot_until_shutdown) {
		// arming-based logging
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.update(&vehicle_status)) {

			desired_state = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
			updated = true;
		}
	}

	desired_state = desired_state || _manually_logging_override;

	// only start/stop if this is a state transition
	if (updated && _prev_state != desired_state) {
		_prev_state = desired_state;

		if (desired_state) {
			if (_should_stop_file_log) { // happens on quick stop/start toggling
				_should_stop_file_log = false;
				stop_log_file(LogType::Full);
			}

			start_log_file(LogType::Full);

			if ((MissionLogType)_param_sdlog_mission.get() != MissionLogType::Disabled) {
				start_log_file(LogType::Mission);
			}

			return true;

		} else {
			// delayed stop: we measure the process loads and then stop
			initialize_load_output(PrintLoadReason::Postflight);
			_should_stop_file_log = true;

			if ((MissionLogType)_param_sdlog_mission.get() != MissionLogType::Disabled) {
				stop_log_file(LogType::Mission);
			}
		}
	}

	return false;
}

void Logger::handle_vehicle_command_update()
{
	vehicle_command_s command;

	if (_vehicle_command_sub.update(&command)) {

		if (command.command == vehicle_command_s::VEHICLE_CMD_LOGGING_START) {

			if ((int)(command.param1 + 0.5f) != 0) {
				ack_vehicle_command(&command, vehicle_command_s::VEHICLE_CMD_RESULT_UNSUPPORTED);

			} else if (can_start_mavlink_log()) {
				ack_vehicle_command(&command, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);
				start_log_mavlink();

			} else {
				ack_vehicle_command(&command, vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED);
			}

		} else if (command.command == vehicle_command_s::VEHICLE_CMD_LOGGING_STOP) {
			stop_log_mavlink();
			ack_vehicle_command(&command, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);
		}
	}
}

bool Logger::write_message(LogType type, void *ptr, size_t size)
{
	Statistics &stats = _statistics[(int)type];

	if (_writer.write_message(type, ptr, size, stats.dropout_start) != -1) {

		if (stats.dropout_start) {
			float dropout_duration = (float)(hrt_elapsed_time(&stats.dropout_start) / 1000) / 1.e3f;

			if (dropout_duration > stats.max_dropout_duration) {
				stats.max_dropout_duration = dropout_duration;
			}

			stats.dropout_start = 0;
		}

		return true;
	}

	if (!stats.dropout_start) {
		stats.dropout_start = hrt_absolute_time();
		++stats.write_dropouts;
		stats.high_water = 0;
	}

	return false;
}

int Logger::create_log_dir(LogType type, tm *tt, char *log_dir, int log_dir_len)
{
	LogFileName &file_name = _file_name[(int)type];

	/* create dir on sdcard if needed */
	int n = snprintf(log_dir, log_dir_len, "%s/", LOG_ROOT[(int)type]);

	if (n >= log_dir_len) {
		PX4_ERR("log path too long");
		return -1;
	}

	if (tt) {
		strftime(file_name.log_dir, sizeof(LogFileName::log_dir), "%Y-%m-%d", tt);
		strncpy(log_dir + n, file_name.log_dir, log_dir_len - n);
		int mkdir_ret = mkdir(log_dir, S_IRWXU | S_IRWXG | S_IRWXO);

		if (mkdir_ret != OK && errno != EEXIST) {
			PX4_ERR("failed creating new dir: %s", log_dir);
			return -1;
		}

	} else {
		uint16_t dir_number = file_name.sess_dir_index;

		if (file_name.has_log_dir) {
			strncpy(log_dir + n, file_name.log_dir, log_dir_len - n);
		}

		/* look for the next dir that does not exist */
		while (!file_name.has_log_dir) {
			/* format log dir: e.g. /fs/microsd/log/sess001 */
			int n2 = snprintf(file_name.log_dir, sizeof(LogFileName::log_dir), "sess%03u", dir_number);

			if (n2 >= (int)sizeof(LogFileName::log_dir)) {
				PX4_ERR("log path too long (%i)", n);
				return -1;
			}

			strncpy(log_dir + n, file_name.log_dir, log_dir_len - n);
			int mkdir_ret = mkdir(log_dir, S_IRWXU | S_IRWXG | S_IRWXO);

			if (mkdir_ret == 0) {
				PX4_DEBUG("log dir created: %s", log_dir);
				file_name.has_log_dir = true;

			} else if (errno != EEXIST) {
				PX4_ERR("failed creating new dir: %s (%i)", log_dir, errno);
				return -1;
			}

			/* dir exists already */
			dir_number++;
		}

		file_name.has_log_dir = true;
	}

	return strlen(log_dir);
}

int Logger::get_log_file_name(LogType type, char *file_name, size_t file_name_size)
{
	tm tt = {};
	bool time_ok = false;

	if (_log_name_timestamp) {
		/* use RTC time for log file naming, e.g. /fs/microsd/log/2014-01-19/19_37_52.ulg */
		time_ok = util::get_log_time(&tt, _param_sdlog_utc_offset.get() * 60, false);
	}

	const char *replay_suffix = "";

	if (_replay_file_name) {
		replay_suffix = "_replayed";
	}

	char *log_file_name = _file_name[(int)type].log_file_name;

	if (time_ok) {
		int n = create_log_dir(type, &tt, file_name, file_name_size);

		if (n < 0) {
			return -1;
		}

		char log_file_name_time[16] = "";
		strftime(log_file_name_time, sizeof(log_file_name_time), "%H_%M_%S", &tt);
		snprintf(log_file_name, sizeof(LogFileName::log_file_name), "%s%s.ulg", log_file_name_time, replay_suffix);
		snprintf(file_name + n, file_name_size - n, "/%s", log_file_name);

	} else {
		int n = create_log_dir(type, nullptr, file_name, file_name_size);

		if (n < 0) {
			return -1;
		}

		uint16_t file_number = 1; // start with file log001

		/* look for the next file that does not exist */
		while (file_number <= MAX_NO_LOGFILE) {
			/* format log file path: e.g. /fs/microsd/log/sess001/log001.ulg */
			snprintf(log_file_name, sizeof(LogFileName::log_file_name), "log%03u%s.ulg", file_number, replay_suffix);
			snprintf(file_name + n, file_name_size - n, "/%s", log_file_name);

			if (!util::file_exist(file_name)) {
				break;
			}

			file_number++;
		}

		if (file_number > MAX_NO_LOGFILE) {
			/* we should not end up here, either we have more than MAX_NO_LOGFILE on the SD card, or another problem */
			return -1;
		}
	}

	return 0;
}

void Logger::setReplayFile(const char *file_name)
{
	if (_replay_file_name) {
		free(_replay_file_name);
	}

	_replay_file_name = strdup(file_name);
}

void Logger::start_log_file(LogType type)
{
	if (_writer.is_started(type, LogWriter::BackendFile) || (_writer.backend() & LogWriter::BackendFile) == 0) {
		return;
	}

	if (type == LogType::Full) {
		// initialize cpu load as early as possible to get more data
		initialize_load_output(PrintLoadReason::Preflight);
	}

	PX4_INFO("Start file log (type: %s)", log_type_str(type));

	char file_name[LOG_DIR_LEN] = "";

	if (get_log_file_name(type, file_name, sizeof(file_name))) {
		PX4_ERR("failed to get log file name");
		return;
	}

	if (type == LogType::Full) {
		/* print logging path, important to find log file later */
		mavlink_log_info(&_mavlink_log_pub, "[logger] %s", file_name);
	}

	_writer.start_log_file(type, file_name);
	_writer.select_write_backend(LogWriter::BackendFile);
	_writer.set_need_reliable_transfer(true);
	write_header(type);
	write_version(type);
	write_formats(type);

	if (type == LogType::Full) {
		write_parameters(type);
		write_perf_data(true);
		write_console_output();
	}

	write_all_add_logged_msg(type);
	_writer.set_need_reliable_transfer(false);
	_writer.unselect_write_backend();
	_writer.notify();

	if (type == LogType::Full) {
		/* reset performance counters to get in-flight min and max values in post flight log */
		perf_reset_all();
	}

	_statistics[(int)type].start_time_file = hrt_absolute_time();

}

void Logger::stop_log_file(LogType type)
{
	if (!_writer.is_started(type, LogWriter::BackendFile)) {
		return;
	}

	if (type == LogType::Full) {
		_writer.set_need_reliable_transfer(true);
		write_perf_data(false);
		_writer.set_need_reliable_transfer(false);
	}

	_writer.stop_log_file(type);
}

void Logger::start_log_mavlink()
{
	if (!can_start_mavlink_log()) {
		return;
	}

	// initialize cpu load as early as possible to get more data
	initialize_load_output(PrintLoadReason::Preflight);

	PX4_INFO("Start mavlink log");

	_writer.start_log_mavlink();
	_writer.select_write_backend(LogWriter::BackendMavlink);
	_writer.set_need_reliable_transfer(true);
	write_header(LogType::Full);
	write_version(LogType::Full);
	write_formats(LogType::Full);
	write_parameters(LogType::Full);
	write_perf_data(true);
	write_console_output();
	write_all_add_logged_msg(LogType::Full);
	_writer.set_need_reliable_transfer(false);
	_writer.unselect_write_backend();
	_writer.notify();
}

void Logger::stop_log_mavlink()
{
	// don't write perf data since a client does not expect more data after a stop command
	PX4_INFO("Stop mavlink log");
	_writer.stop_log_mavlink();
}

struct perf_callback_data_t {
	Logger *logger;
	int counter;
	bool preflight;
	char *buffer;
};

void Logger::perf_iterate_callback(perf_counter_t handle, void *user)
{
	perf_callback_data_t *callback_data = (perf_callback_data_t *)user;
	const int buffer_length = 220;
	char buffer[buffer_length];
	const char *perf_name;

	perf_print_counter_buffer(buffer, buffer_length, handle);

	if (callback_data->preflight) {
		perf_name = "perf_counter_preflight";

	} else {
		perf_name = "perf_counter_postflight";
	}

	callback_data->logger->write_info_multiple(LogType::Full, perf_name, buffer, callback_data->counter != 0);
	++callback_data->counter;
}

void Logger::write_perf_data(bool preflight)
{
	perf_callback_data_t callback_data = {};
	callback_data.logger = this;
	callback_data.counter = 0;
	callback_data.preflight = preflight;

	// write the perf counters
	perf_iterate_all(perf_iterate_callback, &callback_data);
}


void Logger::print_load_callback(void *user)
{
	perf_callback_data_t *callback_data = (perf_callback_data_t *)user;
	const char *perf_name;

	if (!callback_data->buffer) {
		return;
	}

	switch (callback_data->logger->_print_load_reason) {
	case PrintLoadReason::Preflight:
		perf_name = "perf_top_preflight";
		break;

	case PrintLoadReason::Postflight:
		perf_name = "perf_top_postflight";
		break;

	case PrintLoadReason::Watchdog:
		perf_name = "perf_top_watchdog";
		break;

	default:
		perf_name = "perf_top";
		break;
	}

	callback_data->logger->write_info_multiple(LogType::Full, perf_name, callback_data->buffer,
			callback_data->counter != 0);
	++callback_data->counter;
}

void Logger::initialize_load_output(PrintLoadReason reason)
{
	init_print_load(&_load);
	_next_load_print = hrt_absolute_time() + 1_s;
	_print_load_reason = reason;
}

void Logger::write_load_output()
{
	if (_print_load_reason == PrintLoadReason::Watchdog) {
		PX4_ERR("Writing watchdog data"); // this is just that we see it easily in the log
	}

	perf_callback_data_t callback_data = {};
	char buffer[140];
	callback_data.logger = this;
	callback_data.counter = 0;
	callback_data.buffer = buffer;
	_writer.set_need_reliable_transfer(true);
	// TODO: maybe we should restrict the output to a selected backend (eg. when file logging is running
	// and mavlink log is started, this will be added to the file as well)
	print_load_buffer(buffer, sizeof(buffer), print_load_callback, &callback_data, &_load);
	cpuload_monitor_stop();
	_writer.set_need_reliable_transfer(false);
}

void Logger::write_console_output()
{
	const int buffer_length = 220;
	char buffer[buffer_length];
	int size = px4_console_buffer_size();
	int offset = -1;
	bool first = true;

	while (size > 0) {
		int read_size = px4_console_buffer_read(buffer, buffer_length - 1, &offset);

		if (read_size <= 0) { break; }

		buffer[math::min(read_size, size)] = '\0';
		write_info_multiple(LogType::Full, "boot_console_output", buffer, !first);

		size -= read_size;
		first = false;
	}

}

void Logger::write_format(LogType type, const orb_metadata &meta, WrittenFormats &written_formats,
			  ulog_message_format_s &msg, int subscription_index, int level)
{
	if (level > 3) {
		// precaution: limit recursion level. If we land here it's either a bug or nested topic definitions. In the
		// latter case, increase the maximum level.
		PX4_ERR("max recursion level reached (%i)", level);
		return;
	}

	// check if we already wrote the format: either if at a previous _subscriptions index or in written_formats
	for (const auto &written_format : written_formats) {
		if (written_format == &meta) {
			PX4_DEBUG("already added: %s", meta.o_name);
			return;
		}
	}

	for (int i = 0; i < subscription_index; ++i) {
		if (_subscriptions[i].get_topic() == &meta) {
			PX4_DEBUG("already in _subscriptions: %s", meta.o_name);
			return;
		}
	}

	PX4_DEBUG("writing format for %s", meta.o_name);

	// Write the current format (we don't need to check if we already added it to written_formats)
	int format_len = snprintf(msg.format, sizeof(msg.format), "%s:%s", meta.o_name, meta.o_fields);
	size_t msg_size = sizeof(msg) - sizeof(msg.format) + format_len;
	msg.msg_size = msg_size - ULOG_MSG_HEADER_LEN;

	write_message(type, &msg, msg_size);

	if (level > 1 && !written_formats.push_back(&meta)) {
		PX4_ERR("Array too small");
	}

	// Now go through the fields and check for nested type usages.
	// o_fields looks like this for example: "uint64_t timestamp;uint8_t[5] array;"
	const char *fmt = meta.o_fields;

	while (fmt && *fmt) {
		// extract the type name
		char type_name[64];
		const char *space = strchr(fmt, ' ');

		if (!space) {
			PX4_ERR("invalid format %s", fmt);
			break;
		}

		const char *array_start = strchr(fmt, '['); // check for an array

		int type_length;

		if (array_start && array_start < space) {
			type_length = array_start - fmt;

		} else {
			type_length = space - fmt;
		}

		if (type_length >= (int)sizeof(type_name)) {
			PX4_ERR("buf len too small");
			break;
		}

		memcpy(type_name, fmt, type_length);
		type_name[type_length] = '\0';

		// ignore built-in types
		if (strcmp(type_name, "int8_t") != 0 &&
		    strcmp(type_name, "uint8_t") != 0 &&
		    strcmp(type_name, "int16_t") != 0 &&
		    strcmp(type_name, "uint16_t") != 0 &&
		    strcmp(type_name, "int16_t") != 0 &&
		    strcmp(type_name, "uint16_t") != 0 &&
		    strcmp(type_name, "int32_t") != 0 &&
		    strcmp(type_name, "uint32_t") != 0 &&
		    strcmp(type_name, "int64_t") != 0 &&
		    strcmp(type_name, "uint64_t") != 0 &&
		    strcmp(type_name, "float") != 0 &&
		    strcmp(type_name, "double") != 0 &&
		    strcmp(type_name, "bool") != 0 &&
		    strcmp(type_name, "char") != 0) {

			// find orb meta for type
			const orb_metadata *const *topics = orb_get_topics();
			const orb_metadata *found_topic = nullptr;

			for (size_t i = 0; i < orb_topics_count(); i++) {
				if (strcmp(topics[i]->o_name, type_name) == 0) {
					found_topic = topics[i];
				}
			}

			if (found_topic) {

				write_format(type, *found_topic, written_formats, msg, subscription_index, level + 1);

			} else {
				PX4_ERR("No definition for topic %s found", fmt);
			}
		}

		fmt = strchr(fmt, ';');

		if (fmt) { ++fmt; }
	}
}

void Logger::write_formats(LogType type)
{
	_writer.lock();

	// both of these are large and thus we need to be careful in terms of stack size requirements
	ulog_message_format_s msg;
	WrittenFormats written_formats;

	// write all subscribed formats
	int sub_count = _num_subscriptions;

	if (type == LogType::Mission) {
		sub_count = _num_mission_subs;
	}

	for (int i = 0; i < sub_count; ++i) {
		const LoggerSubscription &sub = _subscriptions[i];
		write_format(type, *sub.get_topic(), written_formats, msg, i);
	}

	_writer.unlock();
}

void Logger::write_all_add_logged_msg(LogType type)
{
	_writer.lock();

	int sub_count = _num_subscriptions;

	if (type == LogType::Mission) {
		sub_count = _num_mission_subs;
	}

	bool added_subscriptions = false;

	for (int i = 0; i < sub_count; ++i) {
		LoggerSubscription &sub = _subscriptions[i];

		if (sub.valid()) {
			write_add_logged_msg(type, sub);
			added_subscriptions = true;
		}
	}

	_writer.unlock();

	if (!added_subscriptions) {
		PX4_ERR("No subscriptions added"); // this results in invalid log files
	}
}

void Logger::write_add_logged_msg(LogType type, LoggerSubscription &subscription)
{
	ulog_message_add_logged_s msg;

	if (subscription.msg_id == MSG_ID_INVALID) {
		if (_next_topic_id == MSG_ID_INVALID) {
			// if we land here an uint8 is too small -> switch to uint16
			PX4_ERR("limit for _next_topic_id reached");
			return;
		}

		subscription.msg_id = _next_topic_id++;
	}

	msg.msg_id = subscription.msg_id;
	msg.multi_id = subscription.get_instance();

	int message_name_len = strlen(subscription.get_topic()->o_name);

	memcpy(msg.message_name, subscription.get_topic()->o_name, message_name_len);

	size_t msg_size = sizeof(msg) - sizeof(msg.message_name) + message_name_len;
	msg.msg_size = msg_size - ULOG_MSG_HEADER_LEN;

	bool prev_reliable = _writer.need_reliable_transfer();
	_writer.set_need_reliable_transfer(true);
	write_message(type, &msg, msg_size);
	_writer.set_need_reliable_transfer(prev_reliable);
}

void Logger::write_info(LogType type, const char *name, const char *value)
{
	_writer.lock();
	ulog_message_info_header_s msg = {};
	uint8_t *buffer = reinterpret_cast<uint8_t *>(&msg);
	msg.msg_type = static_cast<uint8_t>(ULogMessageType::INFO);

	/* construct format key (type and name) */
	size_t vlen = strlen(value);
	msg.key_len = snprintf(msg.key, sizeof(msg.key), "char[%zu] %s", vlen, name);
	size_t msg_size = sizeof(msg) - sizeof(msg.key) + msg.key_len;

	/* copy string value directly to buffer */
	if (vlen < (sizeof(msg) - msg_size)) {
		memcpy(&buffer[msg_size], value, vlen);
		msg_size += vlen;

		msg.msg_size = msg_size - ULOG_MSG_HEADER_LEN;

		write_message(type, buffer, msg_size);
	}

	_writer.unlock();
}

void Logger::write_info_multiple(LogType type, const char *name, const char *value, bool is_continued)
{
	_writer.lock();
	ulog_message_info_multiple_header_s msg;
	uint8_t *buffer = reinterpret_cast<uint8_t *>(&msg);
	msg.msg_type = static_cast<uint8_t>(ULogMessageType::INFO_MULTIPLE);
	msg.is_continued = is_continued;

	/* construct format key (type and name) */
	size_t vlen = strlen(value);
	msg.key_len = snprintf(msg.key, sizeof(msg.key), "char[%zu] %s", vlen, name);
	size_t msg_size = sizeof(msg) - sizeof(msg.key) + msg.key_len;

	/* copy string value directly to buffer */
	if (vlen < (sizeof(msg) - msg_size)) {
		memcpy(&buffer[msg_size], value, vlen);
		msg_size += vlen;

		msg.msg_size = msg_size - ULOG_MSG_HEADER_LEN;

		write_message(type, buffer, msg_size);

	} else {
		PX4_ERR("info_multiple str too long (%i), key=%s", msg.key_len, msg.key);
	}

	_writer.unlock();
}

void Logger::write_info(LogType type, const char *name, int32_t value)
{
	write_info_template<int32_t>(type, name, value, "int32_t");
}

void Logger::write_info(LogType type, const char *name, uint32_t value)
{
	write_info_template<uint32_t>(type, name, value, "uint32_t");
}


template<typename T>
void Logger::write_info_template(LogType type, const char *name, T value, const char *type_str)
{
	_writer.lock();
	ulog_message_info_header_s msg = {};
	uint8_t *buffer = reinterpret_cast<uint8_t *>(&msg);
	msg.msg_type = static_cast<uint8_t>(ULogMessageType::INFO);

	/* construct format key (type and name) */
	msg.key_len = snprintf(msg.key, sizeof(msg.key), "%s %s", type_str, name);
	size_t msg_size = sizeof(msg) - sizeof(msg.key) + msg.key_len;

	/* copy string value directly to buffer */
	memcpy(&buffer[msg_size], &value, sizeof(T));
	msg_size += sizeof(T);

	msg.msg_size = msg_size - ULOG_MSG_HEADER_LEN;

	write_message(type, buffer, msg_size);

	_writer.unlock();
}

void Logger::write_header(LogType type)
{
	ulog_file_header_s header = {};
	header.magic[0] = 'U';
	header.magic[1] = 'L';
	header.magic[2] = 'o';
	header.magic[3] = 'g';
	header.magic[4] = 0x01;
	header.magic[5] = 0x12;
	header.magic[6] = 0x35;
	header.magic[7] = 0x01; //file version 1
	header.timestamp = hrt_absolute_time();
	_writer.lock();
	write_message(type, &header, sizeof(header));

	// write the Flags message: this MUST be written right after the ulog header
	ulog_message_flag_bits_s flag_bits{};

	flag_bits.msg_size = sizeof(flag_bits) - ULOG_MSG_HEADER_LEN;
	flag_bits.msg_type = static_cast<uint8_t>(ULogMessageType::FLAG_BITS);

	write_message(type, &flag_bits, sizeof(flag_bits));

	_writer.unlock();
}

void Logger::write_version(LogType type)
{
	write_info(type, "ver_sw", px4_firmware_version_string());
	write_info(type, "ver_sw_release", px4_firmware_version());
	uint32_t vendor_version = px4_firmware_vendor_version();

	if (vendor_version > 0) {
		write_info(type, "ver_vendor_sw_release", vendor_version);
	}

	write_info(type, "ver_hw", px4_board_name());
	const char *board_sub_type = px4_board_sub_type();

	if (board_sub_type && board_sub_type[0]) {
		write_info(type, "ver_hw_subtype", board_sub_type);
	}

	write_info(type, "sys_name", "PX4");
	write_info(type, "sys_os_name", px4_os_name());
	const char *os_version = px4_os_version_string();

	const char *git_branch = px4_firmware_git_branch();

	if (git_branch && git_branch[0]) {
		write_info(type, "ver_sw_branch", git_branch);
	}

	if (os_version) {
		write_info(type, "sys_os_ver", os_version);
	}

	write_info(type, "sys_os_ver_release", px4_os_version());
	write_info(type, "sys_toolchain", px4_toolchain_name());
	write_info(type, "sys_toolchain_ver", px4_toolchain_version());

	const char *ecl_version = px4_ecl_lib_version_string();

	if (ecl_version && ecl_version[0]) {
		write_info(type, "sys_lib_ecl_ver", ecl_version);
	}

	char revision = 'U';
	const char *chip_name = nullptr;

	if (board_mcu_version(&revision, &chip_name, nullptr) >= 0) {
		char mcu_ver[64];
		snprintf(mcu_ver, sizeof(mcu_ver), "%s, rev. %c", chip_name, revision);
		write_info(type, "sys_mcu", mcu_ver);
	}

	// data versioning: increase this on every larger data change (format/semantic)
	// 1: switch to FIFO drivers (disabled on-chip DLPF)
	write_info(type, "ver_data_format", 1);

#ifndef BOARD_HAS_NO_UUID

	/* write the UUID if enabled */
	if (_param_sdlog_uuid.get() == 1) {
		char px4_uuid_string[PX4_GUID_FORMAT_SIZE];
		board_get_px4_guid_formated(px4_uuid_string, sizeof(px4_uuid_string));
		write_info(type, "sys_uuid", px4_uuid_string);
	}

#endif /* BOARD_HAS_NO_UUID */

	write_info(type, "time_ref_utc", _param_sdlog_utc_offset.get() * 60);

	if (_replay_file_name) {
		write_info(type, "replay", _replay_file_name);
	}

	if (type == LogType::Mission) {
		write_info(type, "log_type", "mission");
	}
}

void Logger::write_parameters(LogType type)
{
	_writer.lock();
	ulog_message_parameter_header_s msg = {};
	uint8_t *buffer = reinterpret_cast<uint8_t *>(&msg);

	msg.msg_type = static_cast<uint8_t>(ULogMessageType::PARAMETER);
	int param_idx = 0;
	param_t param = 0;

	do {
		// skip over all parameters which are not invalid and not used
		do {
			param = param_for_index(param_idx);
			++param_idx;
		} while (param != PARAM_INVALID && !param_used(param));

		// save parameters which are valid AND used
		if (param != PARAM_INVALID) {
			// get parameter type and size
			const char *type_str;
			param_type_t ptype = param_type(param);
			size_t value_size = 0;

			switch (ptype) {
			case PARAM_TYPE_INT32:
				type_str = "int32_t";
				value_size = sizeof(int32_t);
				break;

			case PARAM_TYPE_FLOAT:
				type_str = "float";
				value_size = sizeof(float);
				break;

			default:
				continue;
			}

			// format parameter key (type and name)
			msg.key_len = snprintf(msg.key, sizeof(msg.key), "%s %s", type_str, param_name(param));
			size_t msg_size = sizeof(msg) - sizeof(msg.key) + msg.key_len;

			// copy parameter value directly to buffer
			switch (ptype) {
			case PARAM_TYPE_INT32:
				param_get(param, (int32_t *)&buffer[msg_size]);
				break;

			case PARAM_TYPE_FLOAT:
				param_get(param, (float *)&buffer[msg_size]);
				break;

			default:
				continue;
			}

			msg_size += value_size;

			msg.msg_size = msg_size - ULOG_MSG_HEADER_LEN;

			write_message(type, buffer, msg_size);
		}
	} while ((param != PARAM_INVALID) && (param_idx < (int) param_count()));

	_writer.unlock();
	_writer.notify();
}

void Logger::write_changed_parameters(LogType type)
{
	_writer.lock();
	ulog_message_parameter_header_s msg = {};
	uint8_t *buffer = reinterpret_cast<uint8_t *>(&msg);

	msg.msg_type = static_cast<uint8_t>(ULogMessageType::PARAMETER);
	int param_idx = 0;
	param_t param = 0;

	do {
		// skip over all parameters which are not invalid and not used
		do {
			param = param_for_index(param_idx);
			++param_idx;
		} while (param != PARAM_INVALID && !param_used(param));

		// log parameters which are valid AND used AND unsaved
		if ((param != PARAM_INVALID) && param_value_unsaved(param)) {

			// get parameter type and size
			const char *type_str;
			param_type_t ptype = param_type(param);
			size_t value_size = 0;

			switch (ptype) {
			case PARAM_TYPE_INT32:
				type_str = "int32_t";
				value_size = sizeof(int32_t);
				break;

			case PARAM_TYPE_FLOAT:
				type_str = "float";
				value_size = sizeof(float);
				break;

			default:
				continue;
			}

			// format parameter key (type and name)
			msg.key_len = snprintf(msg.key, sizeof(msg.key), "%s %s", type_str, param_name(param));
			size_t msg_size = sizeof(msg) - sizeof(msg.key) + msg.key_len;

			// copy parameter value directly to buffer
			switch (ptype) {
			case PARAM_TYPE_INT32:
				param_get(param, (int32_t *)&buffer[msg_size]);
				break;

			case PARAM_TYPE_FLOAT:
				param_get(param, (float *)&buffer[msg_size]);
				break;

			default:
				continue;
			}

			msg_size += value_size;

			// msg_size is now 1 (msg_type) + 2 (msg_size) + 1 (key_len) + key_len + value_size
			msg.msg_size = msg_size - ULOG_MSG_HEADER_LEN;

			write_message(type, buffer, msg_size);
		}
	} while ((param != PARAM_INVALID) && (param_idx < (int) param_count()));

	_writer.unlock();
	_writer.notify();
}

void Logger::ack_vehicle_command(vehicle_command_s *cmd, uint32_t result)
{
	vehicle_command_ack_s vehicle_command_ack = {};
	vehicle_command_ack.timestamp = hrt_absolute_time();
	vehicle_command_ack.command = cmd->command;
	vehicle_command_ack.result = (uint8_t)result;
	vehicle_command_ack.target_system = cmd->source_system;
	vehicle_command_ack.target_component = cmd->source_component;

	uORB::Publication<vehicle_command_ack_s> cmd_ack_pub{ORB_ID(vehicle_command_ack)};
	cmd_ack_pub.publish(vehicle_command_ack);
}

int Logger::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
System logger which logs a configurable set of uORB topics and system printf messages
(`PX4_WARN` and `PX4_ERR`) to ULog files. These can be used for system and flight performance evaluation,
tuning, replay and crash analysis.

It supports 2 backends:
- Files: write ULog files to the file system (SD card)
- MAVLink: stream ULog data via MAVLink to a client (the client must support this)

Both backends can be enabled and used at the same time.

The file backend supports 2 types of log files: full (the normal log) and a mission
log. The mission log is a reduced ulog file and can be used for example for geotagging or
vehicle management. It can be enabled and configured via SDLOG_MISSION parameter.
The normal log is always a superset of the mission log.

### Implementation
The implementation uses two threads:
- The main thread, running at a fixed rate (or polling on a topic if started with -p) and checking for
  data updates
- The writer thread, writing data to the file

In between there is a write buffer with configurable size (and another fixed-size buffer for
the mission log). It should be large to avoid dropouts.

### Examples
Typical usage to start logging immediately:
$ logger start -e -t

Or if already running:
$ logger on
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("logger", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('m', "all", "file|mavlink|all", "Backend mode", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('x', "Enable/disable logging via Aux1 RC channel", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('e', "Enable logging right after start until disarm (otherwise only when armed)", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Log until shutdown (implies -e)", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('t', "Use date/time for naming log directories and files", true);
	PRINT_MODULE_USAGE_PARAM_INT('r', 280, 0, 8000, "Log rate in Hz, 0 means unlimited rate", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', 12, 4, 10000, "Log buffer size in KiB", true);
	PRINT_MODULE_USAGE_PARAM_STRING('p', nullptr, "<topic_name>",
					 "Poll on a topic instead of running with fixed rate (Log rate and topic intervals are ignored if this is set)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("on", "start logging now, override arming (logger must be running)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("off", "stop logging now, override arming (logger must be running)");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

} // namespace logger
} // namespace px4
