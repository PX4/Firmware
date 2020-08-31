/****************************************************************************
 *
 *  Copyright (C) 2020 PX4 Development Team. All rights reserved.
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
 * @file test_microbench_atomic.cpp
 * Microbenchmark atomic operations.
 */

#include <unit_test.h>

#include <time.h>
#include <stdlib.h>
#include <unistd.h>

#include <drivers/drv_hrt.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/micro_hal.h>
#include <px4_platform_common/atomic.h>

#ifdef __PX4_NUTTX
#include <nuttx/irq.h>
#endif

namespace MicroBenchAtomic
{

#define PERF(name, op, count) do { \
		px4_usleep(100); \
		reset(); \
		perf_counter_t p = perf_alloc(PC_ELAPSED, name); \
		for (int i = 0; i < count; i++) { \
			px4_usleep(1); \
			lock(); \
			perf_begin(p); \
			op; \
			perf_end(p); \
			unlock(); \
			reset(); \
		} \
		perf_print_counter(p); \
		perf_free(p); \
	} while (0)

class MicroBenchAtomic : public UnitTest
{
public:
	bool run_tests() override;

private:

	bool time_atomic_bool();
	bool time_atomic_int8();
	bool time_atomic_int16();
	bool time_atomic_int32();
	bool time_atomic_uint32();
	bool time_atomic_float();

	void reset();

	void lock()
	{
#ifdef __PX4_NUTTX
		_flags = px4_enter_critical_section();
#endif
	}

	void unlock()
	{
#ifdef __PX4_NUTTX
		px4_leave_critical_section(_flags);
#endif
	}

#ifdef __PX4_NUTTX
	irqstate_t _flags {};
#endif

	px4::atomic<bool> _atomic_bool{false};
	px4::atomic<bool> _atomic_bool_storage{false};
	bool _test_load_bool{};

	px4::atomic<int8_t> _atomic_int8{0};
	px4::atomic<int8_t> _atomic_int8_storage{0};
	int8_t _test_load_int8{};

	px4::atomic<int16_t> _atomic_int16{0};
	px4::atomic<int16_t> _atomic_int16_storage{0};
	int16_t _test_load_int16{};

	px4::atomic<int32_t> _atomic_int32{0};
	px4::atomic<int32_t> _atomic_int32_storage{0};
	int32_t _test_load_int32{};

	px4::atomic<uint32_t> _atomic_uint32{0};
	px4::atomic<uint32_t> _atomic_uint32_storage{0};
	uint32_t _test_load_uint32{};

	px4::atomic<float> _atomic_float{0};
	px4::atomic<float> _atomic_float_storage{0};
	float _test_load_float{};
};

bool MicroBenchAtomic::run_tests()
{
	ut_run_test(time_atomic_bool);
	ut_run_test(time_atomic_int8);
	ut_run_test(time_atomic_int16);
	ut_run_test(time_atomic_int32);
	ut_run_test(time_atomic_uint32);
	ut_run_test(time_atomic_float);

	return (_tests_failed == 0);
}

void MicroBenchAtomic::reset()
{
	srand(time(nullptr));

	_atomic_bool.store(rand());
	_atomic_bool_storage.store(rand());
	_test_load_bool = rand();

	_atomic_int8.store(rand());
	_atomic_int8_storage.store(rand());
	_test_load_int8 = rand();

	_atomic_int16.store(rand());
	_atomic_int16_storage.store(rand());
	_test_load_int16 = rand();

	_atomic_int32.store(rand());
	_atomic_int32_storage.store(rand());
	_test_load_int32 = rand();

	_atomic_uint32.store(rand());
	_atomic_uint32_storage.store(rand());
	_test_load_uint32 = rand();

	_atomic_float.store(rand());
	_atomic_float_storage.store(rand());
	_test_load_float = rand();
}

ut_declare_test_c(test_microbench_atomic, MicroBenchAtomic)

bool MicroBenchAtomic::time_atomic_bool()
{
	PERF("atomic bool load", _test_load_bool = _atomic_bool.load(), 100);
	PERF("atomic bool store", _atomic_bool.store(_test_load_bool), 100);
	PERF("atomic bool load and store", _atomic_bool_storage.store(_atomic_bool.load()), 100);

	bool expected = true;
	PERF("atomic bool compare exchange (same)",
	     volatile bool compare_exchange = _atomic_bool.compare_exchange(&expected, true), 100);
	PERF("atomic bool compare exchange (different)",
	     volatile bool compare_exchange = _atomic_bool.compare_exchange(&expected, false), 100);

	return true;
}

bool MicroBenchAtomic::time_atomic_int8()
{
	PERF("atomic int8 load", _test_load_int8 = _atomic_int8.load(), 100);
	PERF("atomic int8 store", _atomic_int8.store(_test_load_int8), 100);
	PERF("atomic int8 load and store", _atomic_int8_storage.store(_atomic_int8.load()), 100);
	PERF("atomic int8 fetch add", _test_load_int8 = _atomic_int8.fetch_add(_test_load_int8), 100);
	PERF("atomic int8 fetch sub", _test_load_int8 = _atomic_int8.fetch_sub(_test_load_int8), 100);
	PERF("atomic int8 fetch and", _test_load_int8 = _atomic_int8.fetch_and(_test_load_int8), 100);
	PERF("atomic int8 fetch xor", _test_load_int8 = _atomic_int8.fetch_xor(_test_load_int8), 100);
	PERF("atomic int8 fetch or", _test_load_int8 = _atomic_int8.fetch_or(_test_load_int8), 100);
	PERF("atomic int8 fetch nand", _test_load_int8 = _atomic_int8.fetch_nand(_test_load_int8), 100);

	int8_t expected = 42;
	PERF("atomic int8 compare exchange (same)",
	     volatile bool compare_exchange = _atomic_int8.compare_exchange(&expected, 42), 100);
	PERF("atomic int8 compare exchange (different)",
	     volatile bool compare_exchange = _atomic_int8.compare_exchange(&expected, 0), 100);

	return true;
}

bool MicroBenchAtomic::time_atomic_int16()
{
	PERF("atomic int16 load", _test_load_int16 = _atomic_int16.load(), 100);
	PERF("atomic int16 store", _atomic_int16.store(_test_load_int16), 100);
	PERF("atomic int16 load and store", _atomic_int16_storage.store(_atomic_int16.load()), 100);
	PERF("atomic int16 fetch add", _test_load_int16 = _atomic_int16.fetch_add(_test_load_int16), 100);
	PERF("atomic int16 fetch sub", _test_load_int16 = _atomic_int16.fetch_sub(_test_load_int16), 100);
	PERF("atomic int16 fetch and", _test_load_int16 = _atomic_int16.fetch_and(_test_load_int16), 100);
	PERF("atomic int16 fetch xor", _test_load_int16 = _atomic_int16.fetch_xor(_test_load_int16), 100);
	PERF("atomic int16 fetch or", _test_load_int16 = _atomic_int16.fetch_or(_test_load_int16), 100);
	PERF("atomic int16 fetch nand", _test_load_int16 = _atomic_int16.fetch_nand(_test_load_int16), 100);

	int16_t expected = 42;
	PERF("atomic int16 compare exchange (same)",
	     volatile bool compare_exchange = _atomic_int16.compare_exchange(&expected, 42), 100);
	PERF("atomic int16 compare exchange (different)",
	     volatile bool compare_exchange = _atomic_int16.compare_exchange(&expected, 0), 100);

	return true;
}

bool MicroBenchAtomic::time_atomic_int32()
{
	PERF("atomic int32 load", _test_load_int32 = _atomic_int32.load(), 100);
	PERF("atomic int32 store", _atomic_int32.store(_test_load_int32), 100);
	PERF("atomic int32 load and store", _atomic_int32_storage.store(_atomic_int32.load()), 100);
	PERF("atomic int32 fetch add", _test_load_int32 = _atomic_int32.fetch_add(_test_load_int32), 100);
	PERF("atomic int32 fetch sub", _test_load_int32 = _atomic_int32.fetch_sub(_test_load_int32), 100);
	PERF("atomic int32 fetch and", _test_load_int32 = _atomic_int32.fetch_and(_test_load_int32), 100);
	PERF("atomic int32 fetch xor", _test_load_int32 = _atomic_int32.fetch_xor(_test_load_int32), 100);
	PERF("atomic int32 fetch or", _test_load_int32 = _atomic_int32.fetch_or(_test_load_int32), 100);
	PERF("atomic int32 fetch nand", _test_load_int32 = _atomic_int32.fetch_nand(_test_load_int32), 100);

	int32_t expected = 42;
	PERF("atomic int32 compare exchange (same)",
	     volatile bool compare_exchange = _atomic_int32.compare_exchange(&expected, 42), 100);
	PERF("atomic int32 compare exchange (different)",
	     volatile bool compare_exchange = _atomic_int32.compare_exchange(&expected, 0), 100);

	return true;
}

bool MicroBenchAtomic::time_atomic_uint32()
{
	PERF("atomic uint32 load", _test_load_uint32 = _atomic_uint32.load(), 100);
	PERF("atomic uint32 store", _atomic_uint32_storage.store(_test_load_uint32), 100);
	PERF("atomic uint32 load and store", _atomic_uint32_storage.store(_atomic_uint32.load()), 100);
	PERF("atomic uint32 fetch add", _test_load_uint32 = _atomic_uint32.fetch_add(_test_load_uint32), 100);
	PERF("atomic uint32 fetch sub", _test_load_uint32 = _atomic_uint32.fetch_sub(_test_load_uint32), 100);
	PERF("atomic uint32 fetch and", _test_load_uint32 = _atomic_uint32.fetch_and(_test_load_uint32), 100);
	PERF("atomic uint32 fetch xor", _test_load_uint32 = _atomic_uint32.fetch_xor(_test_load_uint32), 100);
	PERF("atomic uint32 fetch or", _test_load_uint32 = _atomic_uint32.fetch_or(_test_load_uint32), 100);
	PERF("atomic uint32 fetch nand", _test_load_uint32 = _atomic_uint32.fetch_nand(_test_load_uint32), 100);

	uint32_t expected = 42;
	PERF("atomic uint32 compare exchange (same)",
	     volatile bool compare_exchange = _atomic_uint32.compare_exchange(&expected, 42), 100);
	PERF("atomic uint32 compare exchange (different)",
	     volatile bool compare_exchange = _atomic_uint32.compare_exchange(&expected, 0), 100);

	return true;
}

bool MicroBenchAtomic::time_atomic_float()
{
	//PERF("atomic float load", volatile float test_load = _atomic_float.load(), 100);
	PERF("atomic float store", _atomic_float.store(_test_load_float), 100);
	//PERF("atomic float load and store", _atomic_float_storage.store(_atomic_float.load()), 100);

	float expected = 42;
	PERF("atomic float compare exchange (same)",
	     volatile bool compare_exchange = _atomic_float.compare_exchange(&expected, 42), 100);
	PERF("atomic float compare exchange (different)",
	     volatile bool compare_exchange = _atomic_float.compare_exchange(&expected, 0), 100);

	return true;
}

} // namespace MicroBenchAtomic
