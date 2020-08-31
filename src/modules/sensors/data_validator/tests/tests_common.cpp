/****************************************************************************
 *
 *   Copyright (c) 2019 Todd Stellanova. All rights reserved.
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be  used to endorse or promote products derived
 *    from this software without specific prior written permission.
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

#include <stdio.h>
#include "tests_common.h"


void insert_values_around_mean(DataValidator *validator, const float mean, uint32_t count, float *rms_err,
			       uint64_t *timestamp_io)
{
	uint64_t timestamp = *timestamp_io;
	uint64_t timestamp_incr = 5;
	const uint32_t error_count = 0;
	const uint8_t priority = 50;
	const float swing = 1E-2f;
	double sum_dev_squares = 0.0f;

	//insert a series of values that swing around the mean
	for (uint32_t i = 0; i < count;  i++) {
		float iter_swing = (0 == (i % 2)) ? swing : -swing;
		float iter_val = mean + iter_swing;
		float iter_dev = iter_val - mean;
		sum_dev_squares += (iter_dev * iter_dev);
		timestamp += timestamp_incr;
		validator->put(timestamp, iter_val, error_count, priority);
	}

	double rms = sqrt(sum_dev_squares / (double)count);
	//note: this should be approximately equal to "swing"
	*rms_err = (float)rms;
	*timestamp_io = timestamp;
}

void dump_validator_state(DataValidator *validator)
{
	uint32_t state = validator->state();
	printf("state: 0x%x no_data: %d stale: %d timeout:%d\n",
	       validator->state(),
	       DataValidator::ERROR_FLAG_NO_DATA & state,
	       DataValidator::ERROR_FLAG_STALE_DATA & state,
	       DataValidator::ERROR_FLAG_TIMEOUT & state
	      );
	validator->print();
	printf("\n");
}

void fill_validator_with_samples(DataValidator *validator,
				 const float incr_value,
				 float *value_io,
				 uint64_t *timestamp_io)
{
	uint64_t timestamp = *timestamp_io;
	const uint64_t timestamp_incr = 5; //usec
	const uint32_t timeout_usec = 2000;//derived from class-private value

	float val = *value_io;
	const uint32_t error_count = 0;
	const uint8_t priority = 50; //"medium" priority
	const int equal_value_count = 100; //default is private VALUE_EQUAL_COUNT_DEFAULT

	validator->set_equal_value_threshold(equal_value_count);
	validator->set_timeout(timeout_usec);

	//put a bunch of values that are all different
	for (int i = 0; i < equal_value_count; i++, val += incr_value) {
		timestamp += timestamp_incr;
		validator->put(timestamp, val, error_count, priority);
	}

	*timestamp_io = timestamp;
	*value_io = val;

}
