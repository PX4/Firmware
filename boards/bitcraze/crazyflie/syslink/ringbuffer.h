/****************************************************************************
 *
 *   Copyright (C) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file ringbuffer.h
 *
 * A flexible ringbuffer class.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

namespace ringbuffer __EXPORT
{

class RingBuffer
{
public:
	RingBuffer(unsigned num_items, size_t item_size);
	virtual ~RingBuffer();

	/**
	 * Put an item into the buffer.
	 *
	 * @param val		Item to put
	 * @return		true if the item was put, false if the buffer is full
	 */
	bool			put(const void *val, size_t val_size = 0);

	bool			put(int8_t val);
	bool			put(uint8_t val);
	bool			put(int16_t val);
	bool			put(uint16_t val);
	bool			put(int32_t val);
	bool			put(uint32_t val);
	bool			put(int64_t val);
	bool			put(uint64_t val);
	bool			put(float val);
	bool			put(double val);

	/**
	 * Force an item into the buffer, discarding an older item if there is not space.
	 *
	 * @param val		Item to put
	 * @return		true if an item was discarded to make space
	 */
	bool			force(const void *val, size_t val_size = 0);

	bool			force(int8_t val);
	bool			force(uint8_t val);
	bool			force(int16_t val);
	bool			force(uint16_t val);
	bool			force(int32_t val);
	bool			force(uint32_t val);
	bool			force(int64_t val);
	bool			force(uint64_t val);
	bool			force(float val);
	bool			force(double val);

	/**
	 * Get an item from the buffer.
	 *
	 * @param val		Item that was gotten
	 * @return		true if an item was got, false if the buffer was empty.
	 */
	bool			get(void *val, size_t val_size = 0);

	bool			get(int8_t &val);
	bool			get(uint8_t &val);
	bool			get(int16_t &val);
	bool			get(uint16_t &val);
	bool			get(int32_t &val);
	bool			get(uint32_t &val);
	bool			get(int64_t &val);
	bool			get(uint64_t &val);
	bool			get(float &val);
	bool			get(double &val);

	/*
	 * Get the number of slots free in the buffer.
	 *
	 * @return		The number of items that can be put into the buffer before
	 *			it becomes full.
	 */
	unsigned		space(void);

	/*
	 * Get the number of items in the buffer.
	 *
	 * @return		The number of items that can be got from the buffer before
	 *			it becomes empty.
	 */
	unsigned		count(void);

	/*
	 * Returns true if the buffer is empty.
	 */
	bool			empty();

	/*
	 * Returns true if the buffer is full.
	 */
	bool			full();

	/*
	 * Returns the capacity of the buffer, or zero if the buffer could
	 * not be allocated.
	 */
	unsigned		size();

	/*
	 * Empties the buffer.
	 */
	void			flush();

	/*
	 * resize the buffer. This is unsafe to be called while
	 * a producer or consuming is running. Caller is responsible
	 * for any locking needed
	 *
	 * @param new_size	new size for buffer
	 * @return		true if the resize succeeds, false if
	 * 			not (allocation error)
	 */
	bool			resize(unsigned new_size);

	/*
	 * printf() some info on the buffer
	 */
	void			print_info(const char *name);

private:
	unsigned		_num_items;
	const size_t		_item_size;
	char			*_buf;
	volatile unsigned	_head;	/**< insertion point in _item_size units */
	volatile unsigned	_tail;	/**< removal point in _item_size units */

	unsigned		_next(unsigned index);

	/* we don't want this class to be copied */
	RingBuffer(const RingBuffer &);
	RingBuffer operator=(const RingBuffer &);
};

} // namespace ringbuffer
