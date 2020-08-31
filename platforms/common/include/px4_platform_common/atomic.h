/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file atomic.h
 *
 * Provides atomic integers and counters. Each method is executed atomically and thus
 * can be used to prevent data races and add memory synchronization between threads.
 *
 * In addition to the atomicity, each method serves as a memory barrier (sequential
 * consistent ordering). This means all operations that happen before and could
 * potentially have visible side-effects in other threads will happen before
 * the method is executed.
 *
 * The implementation uses the built-in methods from GCC (supported by Clang as well).
 * @see https://gcc.gnu.org/onlinedocs/gcc/_005f_005fatomic-Builtins.html.
 *
 * @note: on ARM, the instructions LDREX and STREX might be emitted. To ensure correct
 * behavior, the exclusive monitor needs to be cleared on a task switch (via CLREX).
 * This happens automatically e.g. on ARMv7-M as part of an exception entry or exit
 * sequence.
 */

#pragma once

#ifdef __cplusplus

#include <stdbool.h>
#include <stdint.h>

namespace px4
{

template <typename T>
class atomic
{
public:

#ifdef __PX4_NUTTX
	// Ensure that all operations are lock-free, so that 'atomic' can be used from
	// IRQ handlers. This might not be required everywhere though.
	static_assert(__atomic_always_lock_free(sizeof(T), 0), "atomic is not lock-free for the given type T");
#endif

	atomic() = default;
	explicit atomic(T value) : _value(value) {}

	/**
	 * Atomically read the current value
	 */
	inline T load() const
	{
#ifdef __PX4_QURT
		return _value;
#else
		return __atomic_load_n(&_value, __ATOMIC_SEQ_CST);
#endif
	}

	/**
	 * Atomically store a value
	 */
	inline void store(T value)
	{
#ifdef __PX4_QURT
		_value = value;
#else
		__atomic_store(&_value, &value, __ATOMIC_SEQ_CST);
#endif
	}

	/**
	 * Atomically add a number and return the previous value.
	 * @return value prior to the addition
	 */
	inline T fetch_add(T num)
	{
#ifdef __PX4_QURT
		// TODO: fix
		return _value++;
#else
		return __atomic_fetch_add(&_value, num, __ATOMIC_SEQ_CST);
#endif
	}

	/**
	 * Atomically substract a number and return the previous value.
	 * @return value prior to the substraction
	 */
	inline T fetch_sub(T num)
	{
		return __atomic_fetch_sub(&_value, num, __ATOMIC_SEQ_CST);
	}

	/**
	 * Atomic AND with a number
	 * @return value prior to the operation
	 */
	inline T fetch_and(T num)
	{
		return __atomic_fetch_and(&_value, num, __ATOMIC_SEQ_CST);
	}

	/**
	 * Atomic XOR with a number
	 * @return value prior to the operation
	 */
	inline T fetch_xor(T num)
	{
		return __atomic_fetch_xor(&_value, num, __ATOMIC_SEQ_CST);
	}

	/**
	 * Atomic OR with a number
	 * @return value prior to the operation
	 */
	inline T fetch_or(T num)
	{
		return __atomic_fetch_or(&_value, num, __ATOMIC_SEQ_CST);
	}

	/**
	 * Atomic NAND (~(_value & num)) with a number
	 * @return value prior to the operation
	 */
	inline T fetch_nand(T num)
	{
		return __atomic_fetch_nand(&_value, num, __ATOMIC_SEQ_CST);
	}

	/**
	 * Atomic compare and exchange operation.
	 * This compares the contents of _value with the contents of *expected. If
	 * equal, the operation is a read-modify-write operation that writes desired
	 * into _value. If they are not equal, the operation is a read and the current
	 * contents of _value are written into *expected.
	 * @return If desired is written into _value then true is returned
	 */
	inline bool compare_exchange(T *expected, T desired)
	{
		return __atomic_compare_exchange(&_value, expected, &desired, false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST);
	}

private:
#ifdef __PX4_QURT
	// It seems that __atomic_store  and __atomic_load are not supported on Qurt,
	// so the best that we can do is to use volatile.
	volatile T _value{};
#else
	T _value {};
#endif
};

using atomic_int = atomic<int>;
using atomic_int32_t = atomic<int32_t>;
using atomic_bool = atomic<bool>;

} /* namespace px4 */

#endif /* __cplusplus */
