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
 * @file test_data_validator_group.cpp
 * Testing the DataValidatorGroup class
 *
 * @author Todd Stellanova
 */

#include <stdint.h>
#include <cassert>
#include <cstdlib>
#include <stdio.h>
#include <math.h>
#include <validation/data_validator.h>
#include <validation/data_validator_group.h>
#include <validation/tests/tests_common.h>


const uint32_t base_timeout_usec = 2000;//from original private value
const int equal_value_count = 100; //default is private VALUE_EQUAL_COUNT_DEFAULT
const uint64_t base_timestamp = 666;
const unsigned base_num_siblings = 4;


/**
 * Initialize a DataValidatorGroup with some common settings;
 * @param sibling_count (out) the number of siblings initialized
 */
DataValidatorGroup  *setup_base_group(unsigned *sibling_count)
{
	unsigned num_siblings = base_num_siblings;

	DataValidatorGroup *group = new DataValidatorGroup(num_siblings);
	assert(nullptr != group);
	//verify that calling print doesn't crash the tests
	group->print();
	printf("\n");

	//should be no failovers yet
	assert(0 == group->failover_count());
	assert(DataValidator::ERROR_FLAG_NO_ERROR == group->failover_state());
	assert(-1 == group->failover_index());

	//this sets the timeout on all current members of the group, as well as members added later
	group->set_timeout(base_timeout_usec);
	//the following sets the threshold on all CURRENT members of the group, but not any added later
	//TODO This is likely a bug in DataValidatorGroup
	group->set_equal_value_threshold(equal_value_count);

	//return values
	*sibling_count = num_siblings;

	return group;
}

/**
 * Fill one DataValidator with samples, by index.
 *
 * @param group
 * @param val1_idx Index of the validator to fill with samples
 * @param num_samples
 */
void fill_one_with_valid_data(DataValidatorGroup *group, int val1_idx,  uint32_t num_samples)
{
	uint64_t timestamp = base_timestamp;
	uint32_t error_count = 0;
	float last_best_val = 0.0f;

	for (uint32_t i = 0; i < num_samples; i++) {
		float val = ((float) rand() / (float) RAND_MAX);
		float data[DataValidator::dimensions] = {val};
		group->put(val1_idx, timestamp, data, error_count, 100);
		last_best_val = val;
	}

	int best_idx = 0;
	float *best_data = group->get_best(timestamp, &best_idx);
	assert(last_best_val == best_data[0]);
	assert(best_idx == val1_idx);
}



/**
 * Fill two validators in the group with samples, by index.
 * Both validators will be filled with the same data, but
 * the priority of the first validator will be higher than the second.
 *
 * @param group
 * @param val1_idx index of the first validator to fill
 * @param val2_idx index of the second validator to fill
 * @param num_samples
 */
void fill_two_with_valid_data(DataValidatorGroup *group, int val1_idx, int val2_idx, uint32_t num_samples)
{
	uint64_t timestamp = base_timestamp;
	uint32_t error_count = 0;
	float last_best_val = 0.0f;

	for (uint32_t i = 0; i < num_samples; i++) {
		float val = ((float) rand() / (float) RAND_MAX);
		float data[DataValidator::dimensions] = {val};
		//two sensors with identical values, but different priorities
		group->put(val1_idx, timestamp, data, error_count, 100);
		group->put(val2_idx, timestamp, data, error_count, 10);
		last_best_val = val;
	}

	int best_idx = 0;
	float *best_data = group->get_best(timestamp, &best_idx);
	assert(last_best_val == best_data[0]);
	assert(best_idx == val1_idx);

}

/**
 * Dynamically add a validator to the group after construction
 * @param group
 * @return
 */
DataValidator *add_validator_to_group(DataValidatorGroup *group)
{
	DataValidator *validator = group->add_new_validator();
	//verify the previously set timeout applies to the new group member
	assert(validator->get_timeout() == base_timeout_usec);
	//for testing purposes, ensure this newly added member is consistent with the rest of the group
	//TODO this is likely a bug in DataValidatorGroup
	validator->set_equal_value_threshold(equal_value_count);

	return validator;
}

/**
 * Create a DataValidatorGroup and tack on two additional DataValidators
 *
 * @param validator1_handle (out) first DataValidator added to the group
 * @param validator2_handle (out) second DataValidator added to the group
 * @param sibling_count (in/out) in: number of initial siblings to create, out: total
 * @return
 */
DataValidatorGroup *setup_group_with_two_validator_handles(
	DataValidator **validator1_handle,
	DataValidator **validator2_handle,
	unsigned *sibling_count)
{
	DataValidatorGroup *group = setup_base_group(sibling_count);

	//now we add validators
	*validator1_handle = add_validator_to_group(group);
	*validator2_handle = add_validator_to_group(group);
	*sibling_count += 2;

	return group;
}


void test_init()
{
	unsigned num_siblings = 0;

	DataValidatorGroup *group = setup_base_group(&num_siblings);

	//should not yet be any best value
	int best_index = -1;
	assert(nullptr == group->get_best(base_timestamp, &best_index));

	delete group; //force cleanup
}


/**
 * Happy path test of put method -- ensure the "best" sensor selected is the one with highest priority
 */
void test_put()
{
	unsigned num_siblings = 0;
	DataValidator *validator1 = nullptr;
	DataValidator *validator2 = nullptr;

	uint64_t timestamp = base_timestamp;

	DataValidatorGroup *group = setup_group_with_two_validator_handles(&validator1, &validator2, &num_siblings);
	printf("num_siblings: %d \n", num_siblings);
	unsigned val1_idx = num_siblings - 2;
	unsigned val2_idx = num_siblings - 1;

	fill_two_with_valid_data(group, val1_idx, val2_idx, 500);
	int best_idx = -1;
	float *best_data = group->get_best(timestamp, &best_idx);
	assert(nullptr != best_data);
	float best_val = best_data[0];

	float *cur_val1 = validator1->value();
	assert(nullptr != cur_val1);
	//printf("cur_val1 %p \n", cur_val1);
	assert(best_val == cur_val1[0]);

	float *cur_val2 = validator2->value();
	assert(nullptr != cur_val2);
	//printf("cur_val12 %p \n", cur_val2);
	assert(best_val == cur_val2[0]);

	delete group; //force cleanup
}


/**
 * Verify that the DataValidatorGroup will select the sensor with the latest higher priority as "best".
 */
void test_priority_switch()
{
	unsigned num_siblings = 0;
	DataValidator *validator1 = nullptr;
	DataValidator *validator2 = nullptr;

	uint64_t timestamp = base_timestamp;

	DataValidatorGroup *group = setup_group_with_two_validator_handles(&validator1, &validator2, &num_siblings);
	//printf("num_siblings: %d \n",num_siblings);
	int val1_idx = (int)num_siblings - 2;
	int val2_idx = (int)num_siblings - 1;
	uint32_t error_count = 0;

	fill_two_with_valid_data(group, val1_idx, val2_idx, 100);

	int best_idx = -1;
	float *best_data = nullptr;
	//now, switch the priorities, which switches "best" but doesn't trigger a failover
	float new_best_val = 3.14159f;
	float data[DataValidator::dimensions] = {new_best_val};
	//a single sample insertion should be sufficient to trigger a priority switch
	group->put(val1_idx, timestamp, data, error_count, 1);
	group->put(val2_idx, timestamp, data, error_count, 100);
	best_data = group->get_best(timestamp, &best_idx);
	assert(new_best_val == best_data[0]);
	//the new best sensor should now be the sensor with the higher priority
	assert(best_idx == val2_idx);
	//should not have detected a real failover
	assert(0 == group->failover_count());

	delete  group; //cleanup
}

/**
 * Verify that the DataGroupValidator will prefer a sensor with no errors over a sensor with high errors
 */
void test_simple_failover()
{
	unsigned num_siblings = 0;
	DataValidator *validator1 = nullptr;
	DataValidator *validator2 = nullptr;

	uint64_t timestamp = base_timestamp;

	DataValidatorGroup *group = setup_group_with_two_validator_handles(&validator1, &validator2, &num_siblings);
	//printf("num_siblings: %d \n",num_siblings);
	int val1_idx = (int)num_siblings - 2;
	int val2_idx = (int)num_siblings - 1;


	fill_two_with_valid_data(group, val1_idx, val2_idx, 100);

	int best_idx = -1;
	float *best_data = nullptr;

	//trigger a real failover
	float new_best_val = 3.14159f;
	float data[DataValidator::dimensions] = {new_best_val};
	//trigger a bunch of errors on the previous best sensor
	unsigned val1_err_count = 0;

	for (int i = 0; i < 25; i++) {
		group->put(val1_idx, timestamp, data, ++val1_err_count, 100);
		group->put(val2_idx, timestamp, data, 0, 10);
	}

	assert(validator1->error_count() == val1_err_count);

	//since validator1 is experiencing errors, we should see a failover to validator2
	best_data = group->get_best(timestamp + 1, &best_idx);
	assert(nullptr != best_data);
	assert(new_best_val == best_data[0]);
	assert(best_idx == val2_idx);
	//should have detected a real failover
	printf("failover_count: %d \n", group->failover_count());
	assert(1 == group->failover_count());

	//even though validator1 has encountered a bunch of errors, it hasn't failed
	assert(DataValidator::ERROR_FLAG_NO_ERROR == validator1->state());

	// although we failed over from one sensor to another, this is not the same thing tracked by failover_index
	int fail_idx = group->failover_index();
	assert(-1 == fail_idx);//no failed sensor

	//since no sensor has actually hard-failed, the group failover state is NO_ERROR
	assert(DataValidator::ERROR_FLAG_NO_ERROR == group->failover_state());


	delete  group; //cleanup
}

/**
 * Force once sensor to fail and ensure that we detect it
 */
void test_sensor_failure()
{
	unsigned num_siblings = 0;
	uint64_t timestamp = base_timestamp;
	const float sufficient_incr_value = (1.1f * 1E-6f);
	const uint32_t timeout_usec = 2000;//derived from class-private value

	float val = 3.14159f;

	DataValidatorGroup *group =  setup_base_group(&num_siblings);

	//now we add validators
	DataValidator *validator  = add_validator_to_group(group);
	assert(nullptr != validator);
	num_siblings++;
	int val_idx = num_siblings - 1;

	fill_validator_with_samples(validator,  sufficient_incr_value, &val, &timestamp);
	//the best should now be the one validator we've filled with samples

	int best_idx = -1;
	float *best_data = group->get_best(timestamp, &best_idx);
	assert(nullptr != best_data);
	//printf("best_idx: %d val_idx: %d\n", best_idx, val_idx);
	assert(best_idx == val_idx);

	//now force a timeout failure in the one validator, by checking confidence long past timeout
	validator->confidence(timestamp + (1.1 * timeout_usec));
	assert(DataValidator::ERROR_FLAG_TIMEOUT == (DataValidator::ERROR_FLAG_TIMEOUT & validator->state()));

	//now that the one sensor has failed, the group should detect this as well
	int fail_idx = group->failover_index();
	assert(val_idx == fail_idx);

	delete  group;
}

int main(int argc, char *argv[])
{
	(void)argc; // unused
	(void)argv; // unused

	test_init();
	test_put();
	test_simple_failover();
	test_priority_switch();
	test_sensor_failure();

	return 0; //passed
}
