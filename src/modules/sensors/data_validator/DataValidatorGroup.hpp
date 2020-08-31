/****************************************************************************
 *
 *   Copyright (c) 2015-2020 PX4 Development Team. All rights reserved.
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
 * @file DataValidatorGroup.hpp
 *
 * A data validation group to identify anomalies in data streams
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#pragma once

#include "DataValidator.hpp"

class DataValidatorGroup
{
public:
	/**
	 * @param siblings initial number of DataValidator's. Must be > 0.
	 */
	DataValidatorGroup(unsigned siblings);
	~DataValidatorGroup();

	/**
	 * Create a new Validator (with index equal to the number of currently existing validators)
	 * @return the newly created DataValidator or nullptr on error
	 */
	DataValidator *add_new_validator();

	/**
	 * Put an item into the validator group.
	 *
	 * @param index		Sensor index
	 * @param timestamp	The timestamp of the measurement
	 * @param val		The 3D vector
	 * @param error_count	The current error count of the sensor
	 * @param priority	The priority of the sensor
	 */
	void put(unsigned index, uint64_t timestamp, const float val[3], uint32_t error_count, uint8_t priority);

	/**
	 * Get the best data triplet of the group
	 *
	 * @return		pointer to the array of best values
	 */
	float *get_best(uint64_t timestamp, int *index);

	/**
	 * Get the number of failover events
	 *
	 * @return		the number of failovers
	 */
	unsigned failover_count() const { return _toggle_count; }

	/**
	 * Get the index of the failed sensor in the group
	 *
	 * @return		index of the failed sensor
	 */
	int failover_index();

	/**
	 * Get the error state of the failed sensor in the group
	 *
	 * @return		bitmask with erro states of the failed sensor
	 */
	uint32_t failover_state();

	/**
	 * Get the error state of the sensor with the specified index
	 *
	 * @return		bitmask with error states of the sensor
	 */
	uint32_t get_sensor_state(unsigned index);

	/**
	 * Print the validator value
	 *
	 */
	void print();

	/**
	 * Set the timeout value for the whole group
	 *
	 * @param timeout_interval_us The timeout interval in microseconds
	 */
	void set_timeout(uint32_t timeout_interval_us);

	/**
	 * Set the equal count threshold for the whole group
	 *
	 * @param threshold The number of equal values before considering the sensor stale
	 */
	void set_equal_value_threshold(uint32_t threshold);

private:
	DataValidator *_first{nullptr}; /**< first node in the group */
	DataValidator *_last{nullptr};  /**< last node in the group */

	uint32_t _timeout_interval_us{0}; /**< currently set timeout */

	int _curr_best{-1}; /**< currently best index */
	int _prev_best{-1}; /**< the previous best index */

	uint64_t _first_failover_time{0}; /**< timestamp where the first failover occured or zero if none occured */

	unsigned _toggle_count{0}; /**< number of back and forth switches between two sensors */

	static constexpr float MIN_REGULAR_CONFIDENCE = 0.9f;

	/* we don't want this class to be copied */
	DataValidatorGroup(const DataValidatorGroup &);
	DataValidatorGroup operator=(const DataValidatorGroup &);
};
