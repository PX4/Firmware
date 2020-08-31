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
/**
 * @file test_data_validator.cpp
 * Testing the DataValidator class
 *
 * @author Todd Stellanova
 */

#include <stdint.h>
#include <cassert>
#include <cstdlib>
#include <stdio.h>
#include <math.h>
#include <validation/data_validator.h>
#include <validation/tests/tests_common.h>


void test_init()
{
	printf("\n--- test_init ---\n");

	uint64_t fake_timestamp = 666;
	const uint32_t timeout_usec = 2000;//from original private value

	DataValidator *validator = new DataValidator;
	// initially there should be no siblings
	assert(nullptr == validator->sibling());
	// initially we should have zero confidence
	assert(0.0f == validator->confidence(fake_timestamp));
	// initially the error count should be zero
	assert(0 == validator->error_count());
	// initially unused
	assert(!validator->used());
	// initially no priority
	assert(0 == validator->priority());
	validator->set_timeout(timeout_usec);
	assert(validator->get_timeout() == timeout_usec);


	DataValidator *sibling_validator = new DataValidator;
	validator->setSibling(sibling_validator);
	assert(sibling_validator == validator->sibling());

	//verify that with no data, confidence is zero and error mask is set
	assert(0.0f == validator->confidence(fake_timestamp + 1));
	uint32_t state = validator->state();
	assert(DataValidator::ERROR_FLAG_NO_DATA == (DataValidator::ERROR_FLAG_NO_DATA & state));

	//verify that calling print doesn't crash tests
	validator->print();

	delete validator; //force delete
}

void test_put()
{
	printf("\n--- test_put ---\n");

	uint64_t timestamp = 500;
	const uint32_t timeout_usec = 2000;//derived from class-private value
	float val = 3.14159f;
	//derived from class-private value: this is min change needed to avoid stale detection
	const float sufficient_incr_value = (1.1f * 1E-6f);

	DataValidator *validator = new DataValidator;
	fill_validator_with_samples(validator, sufficient_incr_value, &val, &timestamp);

	assert(validator->used());
	//verify that the last value we inserted is the current validator value
	float last_val = val - sufficient_incr_value;
	assert(validator->value()[0] == last_val);

	// we've just provided a bunch of valid data: should be fully confident
	float conf = validator->confidence(timestamp);

	if (1.0f != conf) {
		printf("conf: %f\n", (double)conf);
		dump_validator_state(validator);
	}

	assert(1.0f == conf);
	// should be no errors
	assert(0 == validator->state());

	//now check confidence much beyond the timeout window-- should timeout
	conf = validator->confidence(timestamp + (1.1 * timeout_usec));

	if (0.0f != conf) {
		printf("conf: %f\n", (double)conf);
		dump_validator_state(validator);
	}

	assert(0.0f == conf);
	assert(DataValidator::ERROR_FLAG_TIMEOUT == (DataValidator::ERROR_FLAG_TIMEOUT & validator->state()));

	delete validator; //force delete
}

/**
 * Verify that the DataValidator detects sensor data that does not vary sufficiently
 */
void test_stale_detector()
{
	printf("\n--- test_stale_detector ---\n");

	uint64_t timestamp = 500;
	float val = 3.14159f;
	//derived from class-private value, this is insufficient to avoid stale detection:
	const float insufficient_incr_value = (0.99 * 1E-6f);

	DataValidator *validator = new DataValidator;
	fill_validator_with_samples(validator, insufficient_incr_value, &val, &timestamp);

	// data is stale: should have no confidence
	assert(0.0f == validator->confidence(timestamp));

	// should be a stale error
	uint32_t state = validator->state();

	if (DataValidator::ERROR_FLAG_STALE_DATA != state) {
		dump_validator_state(validator);
	}

	assert(DataValidator::ERROR_FLAG_STALE_DATA == (DataValidator::ERROR_FLAG_STALE_DATA & state));

	delete validator; //force delete
}

/**
 * Verify the RMS error calculated by the DataValidator for a series of samples
 */
void test_rms_calculation()
{
	printf("\n--- test_rms_calculation ---\n");
	const int equal_value_count = 100; //default is private VALUE_EQUAL_COUNT_DEFAULT
	const float mean_value = 3.14159f;
	const uint32_t sample_count = 1000;
	float expected_rms_err = 0.0f;
	uint64_t timestamp = 500;

	DataValidator *validator = new DataValidator;
	validator->set_equal_value_threshold(equal_value_count);

	insert_values_around_mean(validator, mean_value, sample_count, &expected_rms_err, &timestamp);
	float *rms = validator->rms();
	assert(nullptr != rms);
	float calc_rms_err = rms[0];
	float diff = fabsf(calc_rms_err - expected_rms_err);
	float diff_frac = (diff / expected_rms_err);
	printf("rms: %f expect: %f diff: %f frac: %f\n", (double)calc_rms_err, (double)expected_rms_err,
	       (double)diff, (double)diff_frac);
	assert(diff_frac < 0.03f);

	delete validator; //force delete
}

/**
 * Verify error tracking performed by DataValidator::put
 */
void test_error_tracking()
{
	printf("\n--- test_error_tracking ---\n");
	uint64_t timestamp = 500;
	uint64_t timestamp_incr = 5;
	const uint32_t timeout_usec = 2000;//from original private value
	float val = 3.14159f;
	uint32_t error_count = 0;
	int expected_error_density = 0;
	uint8_t priority = 50;
	//from private value: this is min change needed to avoid stale detection
	const float sufficient_incr_value = (1.1f * 1E-6f);
	//default is private VALUE_EQUAL_COUNT_DEFAULT
	const int equal_value_count = 50000;
	//should be less than equal_value_count: ensure this is less than NORETURN_ERRCOUNT
	const int total_iterations = 1000;

	DataValidator *validator = new DataValidator;
	validator->set_timeout(timeout_usec);
	validator->set_equal_value_threshold(equal_value_count);

	//put a bunch of values that are all different
	for (int i = 0; i < total_iterations;  i++, val += sufficient_incr_value) {
		timestamp += timestamp_incr;

		//up to a 50% random error rate appears to pass the error density filter
		if ((((float)rand() / (float)RAND_MAX)) < 0.500f) {
			error_count += 1;
			expected_error_density += 1;

		} else if (expected_error_density > 0) {
			expected_error_density -= 1;
		}

		validator->put(timestamp, val, error_count, priority);
	}

	assert(validator->used());
	//at this point, error_count should be less than NORETURN_ERRCOUNT
	assert(validator->error_count() == error_count);

	// we've just provided a bunch of valid data with some errors:
	// confidence should be reduced by the number of errors
	float conf = validator->confidence(timestamp);
	printf("error_count: %u validator confidence: %f\n", (uint32_t)error_count, (double)conf);
	assert(1.0f != conf);  //we should not be fully confident
	assert(0.0f != conf);  //neither should we be completely unconfident
	// should be no errors, even if confidence is reduced, since we didn't exceed NORETURN_ERRCOUNT
	assert(0 == validator->state());

	// the error density will reduce the confidence by 1 - (error_density / ERROR_DENSITY_WINDOW)
	// ERROR_DENSITY_WINDOW is currently private, but == 100.0f
	float reduced_conf = 1.0f - ((float)expected_error_density / 100.0f);
	double diff = fabs(reduced_conf - conf);

	if (reduced_conf != conf) {
		printf("conf: %f reduced_conf: %f diff: %f\n",
		       (double)conf, (double)reduced_conf, diff);
		dump_validator_state(validator);
	}

	assert(diff < 1E-6f);

	//Now, insert a series of errors and ensure we trip the error detector
	for (int i = 0; i < 250;  i++, val += sufficient_incr_value) {
		timestamp += timestamp_incr;
		//100% error rate
		error_count += 1;
		expected_error_density += 1;
		validator->put(timestamp, val, error_count, priority);
	}

	conf = validator->confidence(timestamp);
	assert(0.0f == conf);  // should we be completely unconfident
	// we should have triggered the high error density detector
	assert(DataValidator::ERROR_FLAG_HIGH_ERRDENSITY == (DataValidator::ERROR_FLAG_HIGH_ERRDENSITY & validator->state()));


	validator->reset_state();

	//Now insert so many errors that we exceed private NORETURN_ERRCOUNT
	for (int i = 0; i < 10000;  i++, val += sufficient_incr_value) {
		timestamp += timestamp_incr;
		//100% error rate
		error_count += 1;
		expected_error_density += 1;
		validator->put(timestamp, val, error_count, priority);
	}

	conf = validator->confidence(timestamp);
	assert(0.0f == conf);  // should we be completely unconfident
	// we should have triggered the high error count detector
	assert(DataValidator::ERROR_FLAG_HIGH_ERRCOUNT == (DataValidator::ERROR_FLAG_HIGH_ERRCOUNT & validator->state()));

	delete validator; //force delete

}

int main(int argc, char *argv[])
{
	(void)argc; // unused
	(void)argv; // unused

	srand(666);
	test_init();
	test_put();
	test_stale_detector();
	test_rms_calculation();
	test_error_tracking();

	return 0; //passed
}
