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
 * @file test_microbench_matrix.cpp
 * Tests for the microbench matrix math library.
 */

#include <unit_test.h>

#include <time.h>
#include <stdlib.h>
#include <unistd.h>

#include <drivers/drv_hrt.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/micro_hal.h>

#include <matrix/math.hpp>

namespace MicroBenchMatrix
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

class MicroBenchMatrix : public UnitTest
{
public:
	virtual bool run_tests();

private:

	bool time_matrix_euler();
	bool time_matrix_quaternion();
	bool time_matrix_dcm();
	bool time_matrix_pseduo_inverse();

	void reset();

	matrix::Quatf q;
	matrix::Eulerf e;
	matrix::Dcmf d;
	matrix::Matrix<float, 16, 6> A16;
	matrix::Matrix<float, 6, 16> B16;
	matrix::Matrix<float, 6, 16> B16_4;
};

bool MicroBenchMatrix::run_tests()
{
	ut_run_test(time_matrix_euler);
	ut_run_test(time_matrix_quaternion);
	ut_run_test(time_matrix_dcm);
	ut_run_test(time_matrix_pseduo_inverse);

	return (_tests_failed == 0);
}

template<typename T>
T random(T min, T max)
{
	const T scale = rand() / (T) RAND_MAX; /* [0, 1.0] */
	return min + scale * (max - min);      /* [min, max] */
}

void MicroBenchMatrix::reset()
{
	srand(time(nullptr));

	// initialize with random data
	q = matrix::Quatf(rand(), rand(), rand(), rand());
	e = matrix::Eulerf(random(-2.0 * M_PI, 2.0 * M_PI), random(-2.0 * M_PI, 2.0 * M_PI), random(-2.0 * M_PI, 2.0 * M_PI));
	d = q;

	for (size_t j = 0; j < 6; j++) {
		for (size_t i = 0; i < 16; i++) {
			B16(j, i) = random(-10.0, 10.0);
		}

		for (size_t i = 0; i < 4; i++) {
			B16_4(j, i) = random(-10.0, 10.0);
		}
	}
}

bool MicroBenchMatrix::time_matrix_euler()
{
	PERF("matrix Euler from Quaternion", e = q, 100);
	PERF("matrix Euler from Dcm", e = d, 100);
	return true;
}

bool MicroBenchMatrix::time_matrix_quaternion()
{
	PERF("matrix Quaternion from Euler", q = e, 100);
	PERF("matrix Quaternion from Dcm", q = d, 100);
	return true;
}

bool MicroBenchMatrix::time_matrix_dcm()
{
	PERF("matrix Dcm from Euler", d = e, 100);
	PERF("matrix Dcm from Quaternion", d = q, 100);
	return true;
}

bool MicroBenchMatrix::time_matrix_pseduo_inverse()
{
	PERF("matrix 6x16 pseudo inverse (all non-zero columns)", A16 = matrix::geninv(B16), 100);
	PERF("matrix 6x16 pseudo inverse (4 non-zero columns)", A16 = matrix::geninv(B16_4), 100);
	return true;
}

ut_declare_test_c(test_microbench_matrix, MicroBenchMatrix)

} // namespace MicroBenchMatrix
