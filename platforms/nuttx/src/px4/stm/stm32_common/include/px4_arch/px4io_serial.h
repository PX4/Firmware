/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file px4io_serial.h
 *
 * Serial Interface definition for PX4IO
 */

#pragma once

#include <board_config.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/micro_hal.h>

#include <perf/perf_counter.h>

#include <drivers/device/device.h>
#include <modules/px4iofirmware/protocol.h>

class PX4IO_serial : public device::Device
{
public:
	PX4IO_serial();
	virtual ~PX4IO_serial();

	virtual int	init() = 0;
	virtual int	read(unsigned offset, void *data, unsigned count = 1);
	virtual int	write(unsigned address, void *data, unsigned count = 1);

protected:
	/**
	 * Does the PX4IO_serial instance initialization.
	 * @param io_buffer The IO buffer that should be used for transfers.
	 * @return 0 on success.
	 */
	int		init(IOPacket *io_buffer);

	/**
	 * Start the transaction with IO and wait for it to complete.
	 */
	virtual int	_bus_exchange(IOPacket *_packet) = 0;

	/**
	 * Performance counters.
	 */
	perf_counter_t		_pc_txns;
	perf_counter_t		_pc_retries;
	perf_counter_t		_pc_timeouts;
	perf_counter_t		_pc_crcerrs;
	perf_counter_t		_pc_protoerrs;
	perf_counter_t		_pc_uerrs;
	perf_counter_t		_pc_idle;
	perf_counter_t		_pc_badidle;
private:
	/*
	 * XXX tune this value
	 *
	 * At 1.5Mbps each register takes 13.3µs, and we always transfer a full packet.
	 * Packet overhead is 26µs for the four-byte header.
	 *
	 * 32 registers = 451µs
	 *
	 * Maybe we can just send smaller packets (e.g. 8 regs) and loop for larger (less common)
	 * transfers? Could cause issues with any regs expecting to be written atomically...
	 */
	IOPacket		*_io_buffer_ptr;

	/** bus-ownership lock */
	px4_sem_t			_bus_semaphore;

	/* do not allow top copying this class */
	PX4IO_serial(PX4IO_serial &);
	PX4IO_serial &operator = (const PX4IO_serial &);
};


#include <stm32_dma.h>


class ArchPX4IOSerial : public PX4IO_serial
{
public:
	ArchPX4IOSerial();
	ArchPX4IOSerial(ArchPX4IOSerial &) = delete;
	ArchPX4IOSerial &operator = (const ArchPX4IOSerial &) = delete;
	~ArchPX4IOSerial();

	virtual int	init();
	virtual int	ioctl(unsigned operation, unsigned &arg);

protected:
	/**
	 * Start the transaction with IO and wait for it to complete.
	 */
	int		_bus_exchange(IOPacket *_packet);

private:
	DMA_HANDLE		_tx_dma;
	DMA_HANDLE		_rx_dma;

	IOPacket *_current_packet;

	/** saved DMA status */
	static const unsigned	_dma_status_inactive = 0x80000000;	// low bits overlap DMA_STATUS_* values
	static const unsigned	_dma_status_waiting  = 0x00000000;
	volatile unsigned	_rx_dma_status;

	/** client-waiting lock/signal */
	px4_sem_t			_completion_semaphore;

	/**
	 * DMA completion handler.
	 */
	static void		_dma_callback(DMA_HANDLE handle, uint8_t status, void *arg);
	void			_do_rx_dma_callback(unsigned status);

	/**
	 * Serial interrupt handler.
	 */
	static int		_interrupt(int vector, void *context, void *arg);
	void			_do_interrupt();

	/**
	 * Cancel any DMA in progress with an error.
	 */
	void			_abort_dma();

	/**
	 * Performance counters.
	 */
	perf_counter_t		_pc_dmasetup;
	perf_counter_t		_pc_dmaerrs;

	/**
	 * IO Buffer storage
	 */
	static uint8_t _io_buffer_storage[] px4_cache_aligned_data();
};

