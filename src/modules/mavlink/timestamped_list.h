/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file timestamped list.h
 * Fixed size list with timestamps.
 *
 * The list has a fixed size that is set at instantiation and is based
 * on timestamps. If a new value is put into a full list, the oldest value
 * is overwritten.
 *
 * @author Julian Oes <julian@oes.ch>
 */

#pragma once

#include <drivers/drv_hrt.h>

/**
 * @class TimestampedList
 */
template <class T>
class TimestampedList
{
public:
	TimestampedList(int num_items)
	{
		_list = new item_s[num_items];
		_list_len = num_items;
	}

	~TimestampedList()
	{
		delete[] _list;
	}

	/**
	 * Insert a value into the list, overwrite the oldest entry if full.
	 */
	void put(const T &new_value)
	{
		hrt_abstime now = hrt_absolute_time();

		// Insert it wherever there is a free space.
		for (int i = 0; i < _list_len; ++i) {
			if (_list[i].timestamp_us == 0) {
				_list[i].timestamp_us = now;
				_list[i].value = new_value;
				return;
			}
		}

		// Find oldest entry.
		int oldest_i = 0;

		for (int i = 1; i < _list_len; ++i) {
			if (_list[i].timestamp_us < _list[oldest_i].timestamp_us) {
				oldest_i = i;
			}
		}

		// And overwrite oldest.
		_list[oldest_i].timestamp_us = now;
		_list[oldest_i].value = new_value;
	}

	/**
	 * Before iterating using get_next(), reset to start.
	 */
	void reset_to_start()
	{
		_current_i = -1;
	}

	/**
	 * Iterate through all active values (not sorted).
	 * Return nullptr if at end of list.
	 *
	 * This is basically a poor man's iterator.
	 */
	T *get_next()
	{
		// Increment first, then leave it until called again.
		++_current_i;

		for (int i = _current_i; i < _list_len; ++i) {
			if (_list[i].timestamp_us != 0) {
				_current_i = i;
				return &_list[i].value;
			}
		}

		return nullptr;
	}

	/**
	 * Disable the last item that we have gotten.
	 */
	void drop_current()
	{
		if (_current_i < _list_len) {
			_list[_current_i].timestamp_us = 0;
		}
	}

	/**
	 * Update the timestamp of the item we have gotten.
	 */
	void update_current()
	{
		if (_current_i < _list_len) {
			_list[_current_i].timestamp = hrt_absolute_time();
		}
	}

	/* do not allow copying or assigning this class */
	TimestampedList(const TimestampedList &) = delete;
	TimestampedList operator=(const TimestampedList &) = delete;

private:
	struct item_s {
		hrt_abstime timestamp_us = 0; // 0 signals inactive.
		T value;
	};

	item_s *_list = nullptr;
	int _list_len = 0;
	int _current_i = -1;
};
