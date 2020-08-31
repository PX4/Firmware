/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan_stm32h7/build_config.hpp>

#if UAVCAN_STM32H7_NUTTX
# include <nuttx/config.h>
# include <nuttx/fs/fs.h>
# include <poll.h>
# include <errno.h>
# include <cstdio>
# include <ctime>
# include <cstring>
#else
# error "Unknown OS"
#endif

#include <uavcan/uavcan.hpp>

namespace uavcan_stm32h7
{

class CanDriver;

#if UAVCAN_STM32H7_NUTTX

/**
 * All bus events are reported as POLLIN.
 */
class BusEvent : uavcan::Noncopyable
{
	using SignalCallbackHandler = void(*)();

	SignalCallbackHandler signal_cb_{nullptr};
	sem_t sem_;

public:
	BusEvent(CanDriver &can_driver);
	~BusEvent();

	void registerSignalCallback(SignalCallbackHandler handler) { signal_cb_ = handler; }

	bool wait(uavcan::MonotonicDuration duration);

	void signalFromInterrupt();
};

class Mutex
{
	pthread_mutex_t mutex_;

public:
	Mutex()
	{
		init();
	}

	int init()
	{
		return pthread_mutex_init(&mutex_, UAVCAN_NULLPTR);
	}

	int deinit()
	{
		return pthread_mutex_destroy(&mutex_);
	}

	void lock()
	{
		(void)pthread_mutex_lock(&mutex_);
	}

	void unlock()
	{
		(void)pthread_mutex_unlock(&mutex_);
	}
};
#endif


class MutexLocker
{
	Mutex &mutex_;

public:
	MutexLocker(Mutex &mutex)
		: mutex_(mutex)
	{
		mutex_.lock();
	}
	~MutexLocker()
	{
		mutex_.unlock();
	}
};

}
