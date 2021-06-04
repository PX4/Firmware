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

#include <px4_platform_common/defines.h>
#include <stdint.h>
#include <pthread.h>
#include <drivers/drv_hrt.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/crypto.h>

namespace px4
{
namespace logger
{

/**
 * @enum LogType
 * Defines different log (file) types
 */
enum class LogType {
	Full = 0, //!< Normal, full size log
	Mission,  //!< reduced mission log (e.g. for geotagging)

	Count
};

const char *log_type_str(LogType type);

/**
 * @class LogWriterFile
 * Writes logging data to a file
 */
class LogWriterFile
{
public:
	LogWriterFile(size_t buffer_size);
	~LogWriterFile();

	bool init();

	/**
	 * start the thread
	 * @return 0 on success, error number otherwise (@see pthread_create)
	 */
	int thread_start();

	void thread_stop();

	void start_log(LogType type, const char *filename);

	void stop_log(LogType type);

	bool is_started(LogType type) const { return _buffers[(int)type]._should_run; }

	/** @see LogWriter::write_message() */
	int write_message(LogType type, void *ptr, size_t size, uint64_t dropout_start = 0);

	void lock()
	{
		pthread_mutex_lock(&_mtx);
	}

	void unlock()
	{
		pthread_mutex_unlock(&_mtx);
	}

	void notify()
	{
		pthread_cond_broadcast(&_cv);
	}

	size_t get_total_written(LogType type) const
	{
		return _buffers[(int)type].total_written();
	}

	size_t get_buffer_size(LogType type) const
	{
		return _buffers[(int)type].buffer_size();
	}

	size_t get_buffer_fill_count(LogType type) const
	{
		return _buffers[(int)type].count();
	}

	void set_need_reliable_transfer(bool need_reliable)
	{
		_need_reliable_transfer = need_reliable;
	}

	bool need_reliable_transfer() const
	{
		return _need_reliable_transfer;
	}

	pthread_t thread_id() const { return _thread; }

private:
	static void *run_helper(void *);

	void run();

	/**
	 * permanently store the ulog file name for the hardfault crash handler, so that it can
	 * append crash logs to the last ulog file.
	 * @param log_file path to the log file
	 * @return 0 on success, <0 errno otherwise
	 */
	int hardfault_store_filename(const char *log_file);

	/**
	 * write w/o waiting/blocking
	 */
	int write(LogType type, void *ptr, size_t size, uint64_t dropout_start);

	/* 512 didn't seem to work properly, 4096 should match the FAT cluster size */
	static constexpr size_t	_min_write_chunk = 4096;

	class LogFileBuffer
	{
	public:
		LogFileBuffer(size_t log_buffer_size, perf_counter_t perf_write, perf_counter_t perf_fsync);

		~LogFileBuffer();

		bool start_log(const char *filename);

		void close_file();

		size_t get_read_ptr(void **ptr, bool *is_part);

		/**
		 * Write to the buffer but assuming there is enough space
		 */
		inline void write_no_check(void *ptr, size_t size);

		size_t available() const { return _buffer_size - _count; }

		int fd() const { return _fd; }

		inline ssize_t write_to_file(const void *buffer, size_t size, bool call_fsync) const;

		inline void fsync() const;

		void mark_read(size_t n) { _count -= n; _total_written += n; }

		size_t total_written() const { return _total_written; }
		size_t buffer_size() const { return _buffer_size; }
		size_t count() const { return _count; }

		bool _should_run = false;
	private:
		const size_t _buffer_size;
		int	_fd = -1;
		uint8_t *_buffer = nullptr;
		size_t _head = 0; ///< next position to write to
		size_t _count = 0; ///< number of bytes in _buffer to be written
		size_t _total_written = 0;
		perf_counter_t _perf_write;
		perf_counter_t _perf_fsync;
	};

	LogFileBuffer _buffers[(int)LogType::Count];

	bool 		_exit_thread = false;
	bool		_need_reliable_transfer = false;
	pthread_mutex_t		_mtx;
	pthread_cond_t		_cv;
	pthread_t _thread = 0;
#if defined(PX4_CRYPTO)
	bool init_logfile_encryption(const char *filename);
	PX4Crypto *_crypto;
#endif

};

}
}
