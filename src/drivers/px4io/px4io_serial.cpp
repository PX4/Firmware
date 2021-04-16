/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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

/**
 * @file px4io_serial.cpp
 *
 * Serial interface for PX4IO
 */

#include "px4io_driver.h"

#include <px4_arch/px4io_serial.h>

static PX4IO_serial *g_interface;

device::Device
*PX4IO_serial_interface()
{
	return new ArchPX4IOSerial();
}

PX4IO_serial::PX4IO_serial() :
	Device("PX4IO_serial"),
	_pc_txns(perf_alloc(PC_ELAPSED, MODULE_NAME": txns")),
	_pc_retries(perf_alloc(PC_COUNT, MODULE_NAME": retries")),
	_pc_timeouts(perf_alloc(PC_COUNT, MODULE_NAME": timeouts")),
	_pc_crcerrs(perf_alloc(PC_COUNT, MODULE_NAME": crcerrs")),
	_pc_protoerrs(perf_alloc(PC_COUNT, MODULE_NAME": protoerrs")),
	_pc_uerrs(perf_alloc(PC_COUNT, MODULE_NAME": uarterrs")),
	_pc_idle(perf_alloc(PC_COUNT, MODULE_NAME": idle")),
	_pc_badidle(perf_alloc(PC_COUNT, MODULE_NAME": badidle")),
	_bus_semaphore(SEM_INITIALIZER(0))
{
	g_interface = this;
}

PX4IO_serial::~PX4IO_serial()
{
	/* kill our semaphores */
	px4_sem_destroy(&_bus_semaphore);

	perf_free(_pc_txns);
	perf_free(_pc_retries);
	perf_free(_pc_timeouts);
	perf_free(_pc_crcerrs);
	perf_free(_pc_protoerrs);
	perf_free(_pc_uerrs);
	perf_free(_pc_idle);
	perf_free(_pc_badidle);

	if (g_interface == this) {
		g_interface = nullptr;
	}
}

int
PX4IO_serial::init(IOPacket *io_buffer)
{
	_io_buffer_ptr = io_buffer;
	/* create semaphores */
	// in case the sub-class impl fails, the semaphore is cleaned up by destructor.
	px4_sem_init(&_bus_semaphore, 0, 1);

	return 0;
}

int
PX4IO_serial::write(unsigned address, void *data, unsigned count)
{
	uint8_t page = address >> 8;
	uint8_t offset = address & 0xff;
	const uint16_t *values = reinterpret_cast<const uint16_t *>(data);

	if (count > PKT_MAX_REGS) {
		return -EINVAL;
	}

	px4_sem_wait(&_bus_semaphore);

	int result;

	for (unsigned retries = 0; retries < 3; retries++) {
		_io_buffer_ptr->count_code = count | PKT_CODE_WRITE;
		_io_buffer_ptr->page = page;
		_io_buffer_ptr->offset = offset;
		memcpy((void *)&_io_buffer_ptr->regs[0], (void *)values, (2 * count));

		for (unsigned i = count; i < PKT_MAX_REGS; i++) {
			_io_buffer_ptr->regs[i] = 0x55aa;
		}

		_io_buffer_ptr->crc = 0;
		_io_buffer_ptr->crc = crc_packet(_io_buffer_ptr);

		/* start the transaction and wait for it to complete */
		result = _bus_exchange(_io_buffer_ptr);

		/* successful transaction? */
		if (result == OK) {

			/* check result in packet */
			if (PKT_CODE(*_io_buffer_ptr) == PKT_CODE_ERROR) {

				/* IO didn't like it - no point retrying */
				result = -EINVAL;
				perf_count(_pc_protoerrs);
			}

			break;
		}

		perf_count(_pc_retries);
	}

	px4_sem_post(&_bus_semaphore);

	if (result == OK) {
		result = count;
	}

	return result;
}

int
PX4IO_serial::read(unsigned address, void *data, unsigned count)
{
	uint8_t page = address >> 8;
	uint8_t offset = address & 0xff;
	uint16_t *values = reinterpret_cast<uint16_t *>(data);

	if (count > PKT_MAX_REGS) {
		return -EINVAL;
	}

	px4_sem_wait(&_bus_semaphore);

	int result;

	for (unsigned retries = 0; retries < 3; retries++) {

		_io_buffer_ptr->count_code = count | PKT_CODE_READ;
		_io_buffer_ptr->page = page;
		_io_buffer_ptr->offset = offset;

		_io_buffer_ptr->crc = 0;
		_io_buffer_ptr->crc = crc_packet(_io_buffer_ptr);

		/* start the transaction and wait for it to complete */
		result = _bus_exchange(_io_buffer_ptr);

		/* successful transaction? */
		if (result == OK) {

			/* check result in packet */
			if (PKT_CODE(*_io_buffer_ptr) == PKT_CODE_ERROR) {

				/* IO didn't like it - no point retrying */
				result = -EINVAL;
				perf_count(_pc_protoerrs);

				/* compare the received count with the expected count */

			} else if (PKT_COUNT(*_io_buffer_ptr) != count) {

				/* IO returned the wrong number of registers - no point retrying */
				result = -EIO;
				perf_count(_pc_protoerrs);

				/* successful read */

			} else {

				/* copy back the result */
				memcpy(values, &_io_buffer_ptr->regs[0], (2 * count));
			}

			break;
		}

		perf_count(_pc_retries);
	}

	px4_sem_post(&_bus_semaphore);

	if (result == OK) {
		result = count;
	}

	return result;
}
