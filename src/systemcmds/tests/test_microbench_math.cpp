/****************************************************************************
 *
 *  Copyright (C) 2018-2019 PX4 Development Team. All rights reserved.
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
 * @file test_microbench_math.cpp
 * Tests for the microbench math library.
 */

#include <unit_test.h>

#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

#include <drivers/drv_hrt.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/micro_hal.h>

namespace MicroBenchMath
{

#ifdef __PX4_NUTTX
#include <nuttx/irq.h>
static irqstate_t flags;
#endif

void lock()
{
#ifdef __PX4_NUTTX
	flags = px4_enter_critical_section();
#endif
}

void unlock()
{
#ifdef __PX4_NUTTX
	px4_leave_critical_section(flags);
#endif
}

#define PERF(name, op, count) do { \
		px4_usleep(1000); \
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

class MicroBenchMath : public UnitTest
{
public:
	virtual bool run_tests();

private:
	bool time_single_precision_float();
	bool time_single_precision_float_trig();

	bool time_double_precision_float();
	bool time_double_precision_float_trig();

	bool time_8bit_integers();
	bool time_16bit_integers();
	bool time_32bit_integers();
	bool time_64bit_integers();

	void reset();

	volatile float f32;
	volatile float f32_out;

	volatile double f64;
	volatile double f64_out;

	volatile uint8_t i_8;
	volatile uint8_t i_8_out;

	volatile uint16_t i_16;
	volatile uint16_t i_16_out;

	volatile uint32_t i_32;
	volatile uint32_t i_32_out;

	volatile int64_t i_64;
	volatile int64_t i_64_out;

	volatile uint64_t u_64;
	volatile uint64_t u_64_out;
};

bool MicroBenchMath::run_tests()
{
	ut_run_test(time_single_precision_float);
	ut_run_test(time_single_precision_float_trig);
	ut_run_test(time_double_precision_float);
	ut_run_test(time_double_precision_float_trig);
	ut_run_test(time_8bit_integers);
	ut_run_test(time_16bit_integers);
	ut_run_test(time_32bit_integers);
	ut_run_test(time_64bit_integers);

	return (_tests_failed == 0);
}

template<typename T>
T random(T min, T max)
{
	const T scale = rand() / (T) RAND_MAX; /* [0, 1.0] */
	return min + scale * (max - min);      /* [min, max] */
}

void MicroBenchMath::reset()
{
	srand(time(nullptr));

	// initialize with random data
	f32 = random(-2.0f * M_PI, 2.0f * M_PI);		// somewhat representative range for angles in radians
	f32_out = random(-2.0f * M_PI, 2.0f * M_PI);

	f64 = random(-2.0 * M_PI, 2.0 * M_PI);
	f64_out = random(-2.0 * M_PI, 2.0 * M_PI);

	i_8 = rand();
	i_8_out = rand();

	i_16 = rand();
	i_16_out = rand();

	i_32 = rand();
	i_32_out = rand();

	i_64 = rand();
	i_64_out = rand();

	u_64 = rand();
	u_64_out = rand();
}

ut_declare_test_c(test_microbench_math, MicroBenchMath)

bool MicroBenchMath::time_single_precision_float()
{
	PERF("float add", f32_out += f32, 100);
	PERF("float sub", f32_out -= f32, 100);
	PERF("float mul", f32_out *= f32, 100);
	PERF("float div", f32_out /= f32, 100);
	PERF("float sqrt", f32_out = sqrtf(f32), 100);

	return true;
}

bool MicroBenchMath::time_single_precision_float_trig()
{
	PERF("sinf()", f32_out = sinf(f32), 100);
	PERF("cosf()", f32_out = cosf(f32), 100);
	PERF("tanf()", f32_out = tanf(f32), 100);

	PERF("acosf()", f32_out = acosf(f32), 100);
	PERF("asinf()", f32_out = asinf(f32), 100);
	PERF("atan2f()", f32_out = atan2f(f32, 2.0f * f32), 100);

	return true;
}

bool MicroBenchMath::time_double_precision_float()
{
	PERF("double add", f64_out += f64, 100);
	PERF("double sub", f64_out -= f64, 100);
	PERF("double mul", f64_out *= f64, 100);
	PERF("double div", f64_out /= f64, 100);
	PERF("double sqrt", f64_out = sqrt(f64), 100);

	return true;
}

bool MicroBenchMath::time_double_precision_float_trig()
{
	PERF("sin()", f64_out = sin(f64), 100);
	PERF("cos()", f64_out = cos(f64), 100);
	PERF("tan()", f64_out = tan(f64), 100);

	PERF("acos()", f64_out = acos(f64 * 0.5), 100);
	PERF("asin()", f64_out = asin(f64 * 0.6), 100);
	PERF("atan2()", f64_out = atan2(f64 * 0.7, f64 * 0.8), 100);

	PERF("sqrt()", f64_out = sqrt(f64), 100);

	return true;
}


bool MicroBenchMath::time_8bit_integers()
{
	PERF("int8 add", i_8_out += i_8, 100);
	PERF("int8 sub", i_8_out -= i_8, 100);
	PERF("int8 mul", i_8_out *= i_8, 100);
	PERF("int8 div", i_8_out /= i_8, 100);

	return true;
}

bool MicroBenchMath::time_16bit_integers()
{
	PERF("int16 add", i_16_out += i_16, 100);
	PERF("int16 sub", i_16_out -= i_16, 100);
	PERF("int16 mul", i_16_out *= i_16, 100);
	PERF("int16 div", i_16_out /= i_16, 100);

	return true;
}

bool MicroBenchMath::time_32bit_integers()
{
	PERF("int32 add", i_32_out += i_32, 100);
	PERF("int32 sub", i_32_out -= i_32, 100);
	PERF("int32 mul", i_32_out *= i_32, 100);
	PERF("int32 div", i_32_out /= i_32, 100);

	return true;
}

bool MicroBenchMath::time_64bit_integers()
{
	PERF("int64 add", i_64_out += i_64, 100);
	PERF("int64 sub", i_64_out -= i_64, 100);
	PERF("int64 mul", i_64_out *= i_64, 100);
	PERF("int64 div", i_64_out /= i_64, 100);

	return true;
}

} // namespace MicroBenchMath
