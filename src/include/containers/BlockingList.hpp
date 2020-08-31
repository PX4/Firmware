/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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
 * @file BlockingList.hpp
 *
 * A blocking intrusive linked list.
 */

#pragma once

#include "IntrusiveSortedList.hpp"
#include "LockGuard.hpp"

#include <pthread.h>
#include <stdlib.h>

template<class T>
class BlockingList : public IntrusiveSortedList<T>
{
public:

	~BlockingList()
	{
		pthread_mutex_destroy(&_mutex);
		pthread_cond_destroy(&_cv);
	}

	void add(T newNode)
	{
		LockGuard lg{_mutex};
		IntrusiveSortedList<T>::add(newNode);
	}

	bool remove(T removeNode)
	{
		LockGuard lg{_mutex};
		return IntrusiveSortedList<T>::remove(removeNode);
	}

	size_t size()
	{
		LockGuard lg{_mutex};
		return IntrusiveSortedList<T>::size();
	}

	void clear()
	{
		LockGuard lg{_mutex};
		IntrusiveSortedList<T>::clear();
	}

	pthread_mutex_t &mutex() { return _mutex; }

private:

	pthread_mutex_t	_mutex = PTHREAD_MUTEX_INITIALIZER;
	pthread_cond_t	_cv = PTHREAD_COND_INITIALIZER;

};
