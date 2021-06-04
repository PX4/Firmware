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

#include "log_writer_file.h"
#include "messages.h"

#include <fcntl.h>
#include <string.h>
#include <errno.h>

#include <mathlib/mathlib.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/crypto.h>
#ifdef __PX4_NUTTX
#include <systemlib/hardfault_log.h>
#endif /* __PX4_NUTTX */

// TODO: how to define the key slots/product?
// this depends on the keystore as well
#define LOGGER_CRYPTO_KEY_IDX 2

using namespace time_literals;


namespace px4
{
namespace logger
{
constexpr size_t LogWriterFile::_min_write_chunk;

LogWriterFile::LogWriterFile(size_t buffer_size)
	: _buffers{
	//We always write larger chunks (orb messages) to the buffer, so the buffer
	//needs to be larger than the minimum write chunk (300 is somewhat arbitrary)
	{
		math::max(buffer_size, _min_write_chunk + 300),
		perf_alloc(PC_ELAPSED, "logger_sd_write"), perf_alloc(PC_ELAPSED, "logger_sd_fsync")},

	{
		300, // buffer size for the mission log (can be kept fairly small)
		perf_alloc(PC_ELAPSED, "logger_sd_write_mission"), perf_alloc(PC_ELAPSED, "logger_sd_fsync_mission")}
}
{
	pthread_mutex_init(&_mtx, nullptr);
	pthread_cond_init(&_cv, nullptr);
}

bool LogWriterFile::init()
{
	return true;
}

LogWriterFile::~LogWriterFile()
{
	pthread_mutex_destroy(&_mtx);
	pthread_cond_destroy(&_cv);
}

#if defined(PX4_CRYPTO)
bool LogWriterFile::init_logfile_encryption(const char *filename)
{
	// Generate a crypto key and store it to the keystore.

	// TODO: use parameter for alogrithm? Or product specific macro?
	// TODO: how to define key index to the keystore? Product macro? parameter?
	if (!_crypto->generate_key(LOGGER_CRYPTO_KEY_IDX, false)) {
		PX4_ERR("Can't generate crypto key");
		return false;
	}

	// Get the generated key, encrypted with RSA_OAEP. Open another temporary session for this.
	PX4Crypto *rsa_crypto = PX4Crypto::px4_crypto_open(CRYPTO_RSA_OAEP);

	if (rsa_crypto == nullptr) {
		return false;
	}

	// TODO: define the public key used for key exchange. Now hardcoded to 1

	/* Get the size of an encrypted key and nonce */

	size_t key_size;
	size_t nonce_size;

	if (!rsa_crypto->get_encrypted_key(LOGGER_CRYPTO_KEY_IDX,
					   NULL,
					   &key_size,
					   1) ||
	    !_crypto->get_nonce(NULL, &nonce_size) ||
	    key_size == 0) {
		PX4Crypto::px4_crypto_close(rsa_crypto);
		return false;
	}

	/* Allocate space and get key + nonce */
	uint8_t *key = (uint8_t *)malloc(key_size + nonce_size);

	if (!key ||
	    !rsa_crypto->get_encrypted_key(
		    LOGGER_CRYPTO_KEY_IDX,
		    key,
		    &key_size,
		    1) ||
	    !_crypto->get_nonce(
		    key + key_size, &nonce_size)) {
		PX4_ERR("Can't get & encrypt the key");
		free(key);
		PX4Crypto::px4_crypto_close(rsa_crypto);
		return false;
	}

	PX4Crypto::px4_crypto_close(rsa_crypto);

	// Write the encrypted key to the disk

	size_t fnlen = strlen(filename);
	char *tmp_buf = (char *)malloc(fnlen + 1);

	if (!tmp_buf) {
		PX4_ERR("out of memory");
		return false;
	}

	memcpy(tmp_buf, filename, fnlen + 1);
	tmp_buf[fnlen - 1] = 'k';

	int key_fd = ::open((const char *)tmp_buf, O_CREAT | O_WRONLY, PX4_O_MODE_666);

	if (key_fd < 0) {
		PX4_ERR("Can't open key file %s, errno: %d", tmp_buf, errno);
		free(tmp_buf);
		return false;
	}

	free(tmp_buf);

	size_t written = ::write(key_fd, key, key_size + nonce_size);

	// Free temporary memory allocations
	free(key);
	::close(key_fd);

	// Check that writing to the disk succeeded
	if (written != key_size + nonce_size) {
		PX4_ERR("Writing the encryption key to disk fail");
		return false;
	}

	return true;
}
#endif // PX4_CRYPTO


void LogWriterFile::start_log(LogType type, const char *filename)
{
	// At this point we don't expect the file to be open, but it can happen for very fast consecutive stop & start
	// calls. In that case we wait for the thread to close the file first.
	lock();

	while (_buffers[(int)type].fd() >= 0) {
		unlock();
		system_usleep(5000);
		lock();
	}

	unlock();

	if (type == LogType::Full) {
		// register the current file with the hardfault handler: if the system crashes,
		// the hardfault handler will append the crash log to that file on the next reboot.
		// Note that we don't deregister it when closing the log, so that crashes after disarming
		// are appended as well (the same holds for crashes before arming, which can be a bit misleading)
		int ret = hardfault_store_filename(filename);

		if (ret) {
			PX4_ERR("Failed to register ULog file to the hardfault handler (%i)", ret);
		}
	}

#if PX4_CRYPTO

	// open a crypto session // TODO: use a parameter for algorithm
	_crypto = PX4Crypto::px4_crypto_open(CRYPTO_XCHACHA20);

	if (_crypto == nullptr) {
		return;
	}

	// TODO: use parameter to check if logfile crypto is enabled
	bool enc_init = init_logfile_encryption(filename);

	if (!enc_init) {
		PX4_ERR("Failed to start encrypted logging");
		PX4Crypto::px4_crypto_close(_crypto);
		return;
	}

#endif

	if (_buffers[(int)type].start_log(filename)) {
		PX4_INFO("Opened %s log file: %s", log_type_str(type), filename);
		notify();
	}
}

int LogWriterFile::hardfault_store_filename(const char *log_file)
{
#ifdef __PX4_NUTTX
	int fd = open(HARDFAULT_ULOG_PATH, O_TRUNC | O_WRONLY | O_CREAT);

	if (fd < 0) {
		return -errno;
	}

	int n = strlen(log_file);

	if (n >= HARDFAULT_MAX_ULOG_FILE_LEN) {
		PX4_ERR("ULog file name too long (%s, %i>=%i)\n", log_file, n, HARDFAULT_MAX_ULOG_FILE_LEN);
		return -EINVAL;
	}

	if (n + 1 != ::write(fd, log_file, n + 1)) {
		close(fd);
		return -errno;
	}

	int ret = close(fd);

	if (ret != 0) {
		return -errno;
	}

#endif /* __PX4_NUTTX */

	return 0;
}

void LogWriterFile::stop_log(LogType type)
{
	_buffers[(int)type]._should_run = false;
	notify();
}

int LogWriterFile::thread_start()
{
	pthread_attr_t thr_attr;
	pthread_attr_init(&thr_attr);

	sched_param param;
	/* low priority, as this is expensive disk I/O */
	param.sched_priority = SCHED_PRIORITY_DEFAULT - 40;
	(void)pthread_attr_setschedparam(&thr_attr, &param);

	pthread_attr_setstacksize(&thr_attr, PX4_STACK_ADJUSTED(1170));

	int ret = pthread_create(&_thread, &thr_attr, &LogWriterFile::run_helper, this);
	pthread_attr_destroy(&thr_attr);

	return ret;
}

void LogWriterFile::thread_stop()
{
	// this will terminate the main loop of the writer thread
	_exit_thread = true;
	_buffers[0]._should_run = _buffers[1]._should_run = false;

	notify();

	// wait for thread to complete
	int ret = pthread_join(_thread, nullptr);

	if (ret) {
		PX4_WARN("join failed: %d", ret);
	}
}

void *LogWriterFile::run_helper(void *context)
{
	px4_prctl(PR_SET_NAME, "log_writer_file", px4_getpid());

	static_cast<LogWriterFile *>(context)->run();
	return nullptr;
}

void LogWriterFile::run()
{
	while (!_exit_thread) {
		// Outer endless loop
		// Wait for _should_run flag
		while (!_exit_thread) {
			bool start = false;
			pthread_mutex_lock(&_mtx);
			pthread_cond_wait(&_cv, &_mtx);
			start = _buffers[0]._should_run || _buffers[1]._should_run;
			pthread_mutex_unlock(&_mtx);

			if (start) {
				break;
			}
		}

		if (_exit_thread) {
			break;
		}

		int poll_count = 0;
		int written = 0;
		hrt_abstime last_fsync = hrt_absolute_time();

		pthread_mutex_lock(&_mtx);

		while (true) {

			const hrt_abstime now = hrt_absolute_time();

			/* call fsync periodically to minimize potential loss of data */
			const bool call_fsync = ++poll_count >= 100 || now - last_fsync > 1_s;

			if (call_fsync) {
				last_fsync = now;
				poll_count = 0;
			}

			constexpr size_t min_available[(int)LogType::Count] = {
				_min_write_chunk,
				1 // For the mission log, write as soon as there is data available
			};

			/* Check all buffers for available data. Mission log is first to avoid drops */
			int i = (int)LogType::Count - 1;

			while (i >= 0) {
				void *read_ptr;
				bool is_part;
				LogFileBuffer &buffer = _buffers[i];
				size_t available = buffer.get_read_ptr(&read_ptr, &is_part);

#if defined(PX4_CRYPTO)
				// Split into 64-byte chunks, so it is good for encrypting in pieces
				available = (available / 64) * 64;
#endif

				/* if sufficient data available or partial read or terminating, write data */
				if (available >= min_available[i] || is_part || (!buffer._should_run && available > 0)) {
					pthread_mutex_unlock(&_mtx);

#if defined(PX4_CRYPTO)
					/* This makes the following assumtions:
					 * - the chipher size is always the
					     same as the input size
					 * - the encryption can be done in
					     place. This is always taken care
					     by the px4 crypto interfaces
					 */

					// TODO: use parameter to check if logfile crypto is enabled
					size_t out = available;
					_crypto->encrypt_data(
						LOGGER_CRYPTO_KEY_IDX, // key
						(uint8_t *)read_ptr,
						available,
						(uint8_t *)read_ptr,
						&out);
#endif
					// TODO: check that available == out? it would be a fatal error, logfile can't be written.
					written = buffer.write_to_file(read_ptr, available, call_fsync);

					/* buffer.mark_read() requires _mtx to be locked */
					pthread_mutex_lock(&_mtx);

					if (written >= 0) {
						/* subtract bytes written from number in buffer (count -= written) */
						buffer.mark_read(written);

						if (!buffer._should_run && written == static_cast<int>(available) && !is_part) {
							/* Stop only when all data written */
							buffer.close_file();
						}

					} else {
						PX4_ERR("write failed (%i)", errno);
						buffer._should_run = false;
						buffer.close_file();
					}

				} else if (call_fsync && buffer._should_run) {
					pthread_mutex_unlock(&_mtx);
					buffer.fsync();
					pthread_mutex_lock(&_mtx);

				} else if (available == 0 && !buffer._should_run) {
					buffer.close_file();
				}

				/* if split into 2 parts, write the second part immediately as well */
				if (!is_part) {
					--i;
				}
			}


			if (_buffers[0].fd() < 0 && _buffers[1].fd() < 0) {
				// stop when both files are closed
#if defined(PX4_CRYPTO)
				/* close the crypto session */

				PX4Crypto::px4_crypto_close(_crypto);
#endif

				break;
			}

			/* Wait for a call to notify(), which indicates new data is available.
			 * Note that at this point there could already be new data available (because of a longer write),
			 * and calling pthread_cond_wait() will still wait for the next notify(). But this is generally
			 * not an issue because notify() is called regularly.
			 * If the logger was switched off in the meantime, do not wait for data, instead run this loop
			 * once more to write remaining data and close the file. */
			if (_buffers[0]._should_run) {
				pthread_cond_wait(&_cv, &_mtx);
			}
		}

		// go back to idle
		pthread_mutex_unlock(&_mtx);
	}
}

int LogWriterFile::write_message(LogType type, void *ptr, size_t size, uint64_t dropout_start)
{
	if (_need_reliable_transfer) {
		int ret;

		// if there's a dropout, write it first (because we might split the message)
		if (dropout_start) {
			while ((ret = write(type, ptr, 0, dropout_start)) == -1) {
				unlock();
				notify();
				px4_usleep(3000);
				lock();
			}
		}

		uint8_t *uptr = (uint8_t *)ptr;

		do {
			// Split into several blocks if the data is longer than the write buffer
			size_t write_size = math::min(size, _buffers[(int)type].buffer_size());

			while ((ret = write(type, uptr, write_size, 0)) == -1) {
				unlock();
				notify();
				px4_usleep(3000);
				lock();
			}

			uptr += write_size;
			size -= write_size;
		} while (size > 0);

		return ret;
	}

	return write(type, ptr, size, dropout_start);
}

int LogWriterFile::write(LogType type, void *ptr, size_t size, uint64_t dropout_start)
{
	if (!is_started(type)) {
		return 0;
	}

	// Bytes available to write
	size_t available = _buffers[(int)type].available();
	size_t dropout_size = 0;

	if (dropout_start) {
		dropout_size = sizeof(ulog_message_dropout_s);
	}

	if (size + dropout_size > available) {
		// buffer overflow
		return -1;
	}

	if (dropout_start) {
		//write dropout msg
		ulog_message_dropout_s dropout_msg;
		dropout_msg.duration = (uint16_t)(hrt_elapsed_time(&dropout_start) / 1000);
		_buffers[(int)type].write_no_check(&dropout_msg, sizeof(dropout_msg));
	}

	_buffers[(int)type].write_no_check(ptr, size);
	return 0;
}

const char *log_type_str(LogType type)
{
	switch (type) {
	case LogType::Full: return "full";

	case LogType::Mission: return "mission";

	case LogType::Count: break;
	}

	return "unknown";
}

LogWriterFile::LogFileBuffer::LogFileBuffer(size_t log_buffer_size, perf_counter_t perf_write,
		perf_counter_t perf_fsync)
	: _buffer_size(log_buffer_size), _perf_write(perf_write), _perf_fsync(perf_fsync)
{
}

LogWriterFile::LogFileBuffer::~LogFileBuffer()
{
	if (_fd >= 0) {
		close(_fd);
	}

	free(_buffer);

	perf_free(_perf_write);
	perf_free(_perf_fsync);
}

void LogWriterFile::LogFileBuffer::write_no_check(void *ptr, size_t size)
{
	size_t n = _buffer_size - _head;	// bytes to end of the buffer

	uint8_t *buffer_c = static_cast<uint8_t *>(ptr);

	if (size > n) {
		// Message goes over the end of the buffer
		memcpy(&(_buffer[_head]), buffer_c, n);
		_head = 0;

	} else {
		n = 0;
	}

	// now: n = bytes already written
	size_t p = size - n;	// number of bytes to write

	memcpy(&(_buffer[_head]), &(buffer_c[n]), p);
	_head = (_head + p) % _buffer_size;
	_count += size;
}

size_t LogWriterFile::LogFileBuffer::get_read_ptr(void **ptr, bool *is_part)
{
	// bytes available to read
	int read_ptr = _head - _count;

	if (read_ptr < 0) {
		read_ptr += _buffer_size;
		*ptr = &_buffer[read_ptr];
		*is_part = true;
		return _buffer_size - read_ptr;

	} else {
		*ptr = &_buffer[read_ptr];
		*is_part = false;
		return _count;
	}
}

bool LogWriterFile::LogFileBuffer::start_log(const char *filename)
{
	_fd = ::open(filename, O_CREAT | O_WRONLY, PX4_O_MODE_666);

	if (_fd < 0) {
		PX4_ERR("Can't open log file %s, errno: %d", filename, errno);
		return false;
	}

	if (_buffer == nullptr) {
		_buffer = (uint8_t *) px4_cache_aligned_alloc(_buffer_size);

		if (_buffer == nullptr) {
			PX4_ERR("Can't create log buffer");
			::close(_fd);
			_fd = -1;
			return false;
		}
	}

	// Clear buffer and counters
	_head = 0;
	_count = 0;
	_total_written = 0;

	_should_run = true;

	return true;
}

void LogWriterFile::LogFileBuffer::fsync() const
{
	perf_begin(_perf_fsync);
	::fsync(_fd);
	perf_end(_perf_fsync);
}

ssize_t LogWriterFile::LogFileBuffer::write_to_file(const void *buffer, size_t size, bool call_fsync) const
{
	perf_begin(_perf_write);
	ssize_t ret = ::write(_fd, buffer, size);
	perf_end(_perf_write);

	if (call_fsync) {
		fsync();
	}

	return ret;
}

void LogWriterFile::LogFileBuffer::close_file()
{
	_head = 0;
	_count = 0;

	if (_fd >= 0) {
		int res = close(_fd);
		_fd = -1;

		if (res) {
			PX4_WARN("closing log file failed (%i)", errno);

		} else {
			PX4_INFO("closed logfile, bytes written: %zu", _total_written);
		}
	}
}

}
}
