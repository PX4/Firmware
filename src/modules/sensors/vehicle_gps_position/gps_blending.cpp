/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file gps_blending.cpp
 */

#include "gps_blending.hpp"


void GpsBlending::update(uint64_t hrt_now_us)
{
	_is_new_output_data_available = false;

	// blend multiple receivers if available
	if (!blend_gps_data(hrt_now_us)) {
		// Only use selected receiver data if it has been updated
		uint8_t gps_select_index = 0;

		// Find the single "best" GPS from the data we have
		// First, find the GPS(s) with the best fix
		uint8_t best_fix = 0;

		for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
			if (_gps_state[i].fix_type > best_fix) {
				best_fix = _gps_state[i].fix_type;
			}
		}

		// Second, compare GPS's with best fix and take the one with most satellites
		uint8_t max_sats = 0;

		for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
			if (_gps_state[i].fix_type == best_fix && _gps_state[i].satellites_used > max_sats) {
				max_sats = _gps_state[i].satellites_used;
				gps_select_index = i;
			}
		}

		// Check for new data on selected GPS, and clear blend offsets
		for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
			_NE_pos_offset_m[i].zero();
			_hgt_offset_mm[i] = 0.0f;
		}

		// Only use a secondary instance if the fallback is allowed
		if ((_primary_instance > -1)
		    && (gps_select_index != _primary_instance)
		    && !_fallback_allowed) {
			gps_select_index = _primary_instance;
		}

		_selected_gps = gps_select_index;
		_is_new_output_data_available =  _gps_updated[gps_select_index];

		for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
			_gps_updated[gps_select_index] = false;
		}
	}
}

bool GpsBlending::blend_gps_data(uint64_t hrt_now_us)
{
	/*
	 * If both receivers have the same update rate, use the oldest non-zero time.
	 * If two receivers with different update rates are used, use the slowest.
	 * If time difference is excessive, use newest to prevent a disconnected receiver
	 * from blocking updates.
	 */

	// Calculate the time step for each receiver with some filtering to reduce the effects of jitter
	// Find the largest and smallest time step.
	float dt_max = 0.0f;
	float dt_min = GPS_TIMEOUT_S;
	_np_gps_suitable_for_blending = 0;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
		const float raw_dt = 1e-6f * (float)(_gps_state[i].timestamp - _time_prev_us[i]);
		const float present_dt = 1e-6f * (float)(hrt_now_us - _gps_state[i].timestamp);

		if (raw_dt > 0.0f && raw_dt < GPS_TIMEOUT_S) {
			_gps_dt[i] = 0.1f * raw_dt + 0.9f * _gps_dt[i];

		} else if ((present_dt >= GPS_TIMEOUT_S)
			   && (_gps_state[i].timestamp > 0)) {
			// Timed out - kill the stored fix for this receiver and don't track its (stale) gps_dt
			_gps_state[i].timestamp = 0;
			_gps_state[i].fix_type = 0;
			_gps_state[i].satellites_used = 0;
			_gps_state[i].vel_ned_valid = 0;

			if (i == _primary_instance) {
				// Allow using a secondary instance when the primary
				// receiver has timed out
				_fallback_allowed = true;
			}

			continue;
		}

		// Only count GPSs with at least a 2D fix for blending purposes
		if (_gps_state[i].fix_type < 2) {
			continue;
		}

		if (_gps_dt[i] > dt_max) {
			dt_max = _gps_dt[i];
			_gps_slowest_index = i;
		}

		if (_gps_dt[i] < dt_min) {
			dt_min = _gps_dt[i];
		}

		_np_gps_suitable_for_blending++;
	}

	// Find the receiver that is last be updated
	uint64_t max_us = 0; // newest non-zero system time of arrival of a GPS message
	uint64_t min_us = -1; // oldest non-zero system time of arrival of a GPS message

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
		// Find largest and smallest times
		if (_gps_state[i].timestamp > max_us) {
			max_us = _gps_state[i].timestamp;
			_gps_newest_index = i;
		}

		if ((_gps_state[i].timestamp < min_us) && (_gps_state[i].timestamp > 0)) {
			min_us = _gps_state[i].timestamp;
		}
	}

	if (_np_gps_suitable_for_blending < 2) {
		// Less than 2 receivers left, so fall out of blending
		return false;
	}

	/*
	 * If the largest dt is less than 20% greater than the smallest, then we have  receivers
	 * running at the same rate then we wait until we have two messages with an arrival time
	 * difference that is less than 50% of the smallest time step and use the time stamp from
	 * the newest data.
	 * Else we have two receivers at different update rates and use the slowest receiver
	 * as the timing reference.
	 */
	bool gps_new_output_data = false;

	if ((dt_max - dt_min) < 0.2f * dt_min) {
		// both receivers assumed to be running at the same rate
		if ((max_us - min_us) < (uint64_t)(5e5f * dt_min)) {
			// data arrival within a short time window enables the two measurements to be blended
			_gps_time_ref_index = _gps_newest_index;
			gps_new_output_data = true;
		}

	} else {
		// both receivers running at different rates
		_gps_time_ref_index = _gps_slowest_index;

		if (_gps_state[_gps_time_ref_index].timestamp > _time_prev_us[_gps_time_ref_index]) {
			// blend data at the rate of the slower receiver
			gps_new_output_data = true;
		}
	}

	if (gps_new_output_data) {
		// calculate the sum squared speed accuracy across all GPS sensors
		float speed_accuracy_sum_sq = 0.0f;

		if (_blend_use_spd_acc) {
			for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
				if (_gps_state[i].fix_type >= 3 && _gps_state[i].s_variance_m_s > 0.0f) {
					speed_accuracy_sum_sq += _gps_state[i].s_variance_m_s * _gps_state[i].s_variance_m_s;
				}
			}
		}

		// calculate the sum squared horizontal position accuracy across all GPS sensors
		float horizontal_accuracy_sum_sq = 0.0f;

		if (_blend_use_hpos_acc) {
			for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
				if (_gps_state[i].fix_type >= 2 && _gps_state[i].eph > 0.0f) {
					horizontal_accuracy_sum_sq += _gps_state[i].eph * _gps_state[i].eph;
				}
			}
		}

		// calculate the sum squared vertical position accuracy across all GPS sensors
		float vertical_accuracy_sum_sq = 0.0f;

		if (_blend_use_vpos_acc) {
			for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
				if (_gps_state[i].fix_type >= 3 && _gps_state[i].epv > 0.0f) {
					vertical_accuracy_sum_sq += _gps_state[i].epv * _gps_state[i].epv;
				}
			}
		}

		// Check if we can do blending using reported accuracy
		bool can_do_blending = (horizontal_accuracy_sum_sq > 0.0f || vertical_accuracy_sum_sq > 0.0f
					|| speed_accuracy_sum_sq > 0.0f);

		// if we can't do blending using reported accuracy, return false and hard switch logic will be used instead
		if (!can_do_blending) {
			return false;
		}

		float sum_of_all_weights = 0.0f;

		// calculate a weighting using the reported speed accuracy
		float spd_blend_weights[GPS_MAX_RECEIVERS_BLEND] {};

		if (speed_accuracy_sum_sq > 0.0f && _blend_use_spd_acc) {
			// calculate the weights using the inverse of the variances
			float sum_of_spd_weights = 0.0f;

			for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
				if (_gps_state[i].fix_type >= 3 && _gps_state[i].s_variance_m_s >= 0.001f) {
					spd_blend_weights[i] = 1.0f / (_gps_state[i].s_variance_m_s * _gps_state[i].s_variance_m_s);
					sum_of_spd_weights += spd_blend_weights[i];
				}
			}

			// normalise the weights
			if (sum_of_spd_weights > 0.0f) {
				for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
					spd_blend_weights[i] = spd_blend_weights[i] / sum_of_spd_weights;
				}

				sum_of_all_weights += 1.0f;
			}
		}

		// calculate a weighting using the reported horizontal position
		float hpos_blend_weights[GPS_MAX_RECEIVERS_BLEND] {};

		if (horizontal_accuracy_sum_sq > 0.0f && _blend_use_hpos_acc) {
			// calculate the weights using the inverse of the variances
			float sum_of_hpos_weights = 0.0f;

			for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
				if (_gps_state[i].fix_type >= 2 && _gps_state[i].eph >= 0.001f) {
					hpos_blend_weights[i] = horizontal_accuracy_sum_sq / (_gps_state[i].eph * _gps_state[i].eph);
					sum_of_hpos_weights += hpos_blend_weights[i];
				}
			}

			// normalise the weights
			if (sum_of_hpos_weights > 0.0f) {
				for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
					hpos_blend_weights[i] = hpos_blend_weights[i] / sum_of_hpos_weights;
				}

				sum_of_all_weights += 1.0f;
			}
		}

		// calculate a weighting using the reported vertical position accuracy
		float vpos_blend_weights[GPS_MAX_RECEIVERS_BLEND] = {};

		if (vertical_accuracy_sum_sq > 0.0f && _blend_use_vpos_acc) {
			// calculate the weights using the inverse of the variances
			float sum_of_vpos_weights = 0.0f;

			for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
				if (_gps_state[i].fix_type >= 3 && _gps_state[i].epv >= 0.001f) {
					vpos_blend_weights[i] = vertical_accuracy_sum_sq / (_gps_state[i].epv * _gps_state[i].epv);
					sum_of_vpos_weights += vpos_blend_weights[i];
				}
			}

			// normalise the weights
			if (sum_of_vpos_weights > 0.0f) {
				for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
					vpos_blend_weights[i] = vpos_blend_weights[i] / sum_of_vpos_weights;
				}

				sum_of_all_weights += 1.0f;
			};
		}

		// blend weight for each GPS. The blend weights must sum to 1.0 across all instances.
		float blend_weights[GPS_MAX_RECEIVERS_BLEND];

		// calculate an overall weight
		for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
			blend_weights[i] = (hpos_blend_weights[i] + vpos_blend_weights[i] + spd_blend_weights[i]) / sum_of_all_weights;
		}

		// With updated weights we can calculate a blended GPS solution and
		// offsets for each physical receiver
		sensor_gps_s gps_blended_state = gps_blend_states(blend_weights);

		update_gps_offsets(gps_blended_state);

		// calculate a blended output from the offset corrected receiver data
		// publish if blending was successful
		calc_gps_blend_output(gps_blended_state, blend_weights);

		_gps_blended_state = gps_blended_state;
		_selected_gps = GPS_MAX_RECEIVERS_BLEND;
		_is_new_output_data_available = true;
	}

	return true;
}

sensor_gps_s GpsBlending::gps_blend_states(float blend_weights[GPS_MAX_RECEIVERS_BLEND]) const
{
	// initialise the blended states so we can accumulate the results using the weightings for each GPS receiver.
	sensor_gps_s gps_blended_state{};
	gps_blended_state.eph = FLT_MAX;
	gps_blended_state.epv = FLT_MAX;
	gps_blended_state.s_variance_m_s = FLT_MAX;
	gps_blended_state.vel_ned_valid = true;
	gps_blended_state.hdop = FLT_MAX;
	gps_blended_state.vdop = FLT_MAX;

	// combine the the GPS states into a blended solution using the weights calculated in calc_blend_weights()
	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
		// blend the timing data
		gps_blended_state.timestamp += (uint64_t)((double)_gps_state[i].timestamp * (double)blend_weights[i]);

		// use the highest status
		if (_gps_state[i].fix_type > gps_blended_state.fix_type) {
			gps_blended_state.fix_type = _gps_state[i].fix_type;
		}

		// Assume blended error magnitude, DOP and sat count is equal to the best value from contributing receivers
		// If any receiver contributing has an invalid velocity, then report blended velocity as invalid
		if (blend_weights[i] > 0.0f) {

			// calculate a blended average speed and velocity vector
			gps_blended_state.vel_m_s += _gps_state[i].vel_m_s * blend_weights[i];
			gps_blended_state.vel_n_m_s += _gps_state[i].vel_n_m_s * blend_weights[i];
			gps_blended_state.vel_e_m_s += _gps_state[i].vel_e_m_s * blend_weights[i];
			gps_blended_state.vel_d_m_s += _gps_state[i].vel_d_m_s * blend_weights[i];

			if (_gps_state[i].eph > 0.0f
			    && _gps_state[i].eph < gps_blended_state.eph) {
				gps_blended_state.eph = _gps_state[i].eph;
			}

			if (_gps_state[i].epv > 0.0f
			    && _gps_state[i].epv < gps_blended_state.epv) {
				gps_blended_state.epv = _gps_state[i].epv;
			}

			if (_gps_state[i].s_variance_m_s > 0.0f
			    && _gps_state[i].s_variance_m_s < gps_blended_state.s_variance_m_s) {
				gps_blended_state.s_variance_m_s = _gps_state[i].s_variance_m_s;
			}

			if (_gps_state[i].hdop > 0
			    && _gps_state[i].hdop < gps_blended_state.hdop) {
				gps_blended_state.hdop = _gps_state[i].hdop;
			}

			if (_gps_state[i].vdop > 0
			    && _gps_state[i].vdop < gps_blended_state.vdop) {
				gps_blended_state.vdop = _gps_state[i].vdop;
			}

			if (_gps_state[i].satellites_used > 0
			    && _gps_state[i].satellites_used > gps_blended_state.satellites_used) {
				gps_blended_state.satellites_used = _gps_state[i].satellites_used;
			}

			if (!_gps_state[i].vel_ned_valid) {
				gps_blended_state.vel_ned_valid = false;
			}
		}

		// TODO read parameters for individual GPS antenna positions and blend
		// Vector3f temp_antenna_offset = _antenna_offset[i];
		// temp_antenna_offset *= blend_weights[i];
		// _blended_antenna_offset += temp_antenna_offset;
	}

	/*
	 * Calculate the instantaneous weighted average location using  available GPS instances and store in  _gps_state.
	 * This is statistically the most likely location, but may not be stable enough for direct use by the EKF.
	*/

	// Use the GPS with the highest weighting as the reference position
	float best_weight = 0.0f;

	// index of the physical receiver with the lowest reported error
	uint8_t gps_best_index = 0;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
		if (blend_weights[i] > best_weight) {
			best_weight = blend_weights[i];
			gps_best_index = i;
			gps_blended_state.lat = _gps_state[i].lat;
			gps_blended_state.lon = _gps_state[i].lon;
			gps_blended_state.alt = _gps_state[i].alt;
		}
	}

	// Convert each GPS position to a local NEU offset relative to the reference position
	Vector2f blended_NE_offset_m{0, 0};
	float blended_alt_offset_mm = 0.0f;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
		if ((blend_weights[i] > 0.0f) && (i != gps_best_index)) {
			// calculate the horizontal offset
			Vector2f horiz_offset{};
			get_vector_to_next_waypoint((gps_blended_state.lat / 1.0e7), (gps_blended_state.lon / 1.0e7),
						    (_gps_state[i].lat / 1.0e7), (_gps_state[i].lon / 1.0e7),
						    &horiz_offset(0), &horiz_offset(1));

			// sum weighted offsets
			blended_NE_offset_m += horiz_offset * blend_weights[i];

			// calculate vertical offset
			float vert_offset = (float)(_gps_state[i].alt - gps_blended_state.alt);

			// sum weighted offsets
			blended_alt_offset_mm += vert_offset * blend_weights[i];
		}
	}

	// Add the sum of weighted offsets to the reference position to obtain the blended position
	const double lat_deg_now = (double)gps_blended_state.lat * 1.0e-7;
	const double lon_deg_now = (double)gps_blended_state.lon * 1.0e-7;
	double lat_deg_res = 0;
	double lon_deg_res = 0;
	add_vector_to_global_position(lat_deg_now, lon_deg_now,
				      blended_NE_offset_m(0), blended_NE_offset_m(1),
				      &lat_deg_res, &lon_deg_res);
	gps_blended_state.lat = (int32_t)(1.0E7 * lat_deg_res);
	gps_blended_state.lon = (int32_t)(1.0E7 * lon_deg_res);
	gps_blended_state.alt += (int32_t)blended_alt_offset_mm;

	// Take GPS heading from the highest weighted receiver that is publishing a valid .heading value
	uint8_t gps_best_yaw_index = 0;
	float best_yaw_weight = 0.0f;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
		if (PX4_ISFINITE(_gps_state[i].heading) && (blend_weights[i] > best_yaw_weight)) {
			best_yaw_weight = blend_weights[i];
			gps_best_yaw_index = i;
		}
	}

	gps_blended_state.heading = _gps_state[gps_best_yaw_index].heading;
	gps_blended_state.heading_offset = _gps_state[gps_best_yaw_index].heading_offset;

	return gps_blended_state;
}

void GpsBlending::update_gps_offsets(const sensor_gps_s &gps_blended_state)
{
	// Calculate filter coefficients to be applied to the offsets for each GPS position and height offset
	// A weighting of 1 will make the offset adjust the slowest, a weighting of 0 will make it adjust with zero filtering
	float alpha[GPS_MAX_RECEIVERS_BLEND] {};
	float omega_lpf = 1.0f / fmaxf(_blending_time_constant, 1.0f);

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
		if (_gps_state[i].timestamp - _time_prev_us[i] > 0) {
			// calculate the filter coefficient that achieves the time constant specified by the user adjustable parameter
			alpha[i] = constrain(omega_lpf * 1e-6f * (float)(_gps_state[i].timestamp - _time_prev_us[i]),
					     0.0f, 1.0f);

			_time_prev_us[i] = _gps_state[i].timestamp;
		}
	}

	// Calculate a filtered position delta for each GPS relative to the blended solution state
	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
		Vector2f offset;
		get_vector_to_next_waypoint((_gps_state[i].lat / 1.0e7), (_gps_state[i].lon / 1.0e7),
					    (gps_blended_state.lat / 1.0e7), (gps_blended_state.lon / 1.0e7),
					    &offset(0), &offset(1));

		_NE_pos_offset_m[i] = offset * alpha[i] + _NE_pos_offset_m[i] * (1.0f - alpha[i]);

		_hgt_offset_mm[i] = (float)(gps_blended_state.alt - _gps_state[i].alt) *  alpha[i] +
				    _hgt_offset_mm[i] * (1.0f - alpha[i]);
	}

	// calculate offset limits from the largest difference between receivers
	Vector2f max_ne_offset{};
	float max_alt_offset = 0;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
		for (uint8_t j = i; j < GPS_MAX_RECEIVERS_BLEND; j++) {
			if (i != j) {
				Vector2f offset;
				get_vector_to_next_waypoint((_gps_state[i].lat / 1.0e7), (_gps_state[i].lon / 1.0e7),
							    (_gps_state[j].lat / 1.0e7), (_gps_state[j].lon / 1.0e7),
							    &offset(0), &offset(1));
				max_ne_offset(0) = fmaxf(max_ne_offset(0), fabsf(offset(0)));
				max_ne_offset(1) = fmaxf(max_ne_offset(1), fabsf(offset(1)));
				max_alt_offset = fmaxf(max_alt_offset, fabsf((float)(_gps_state[i].alt - _gps_state[j].alt)));
			}
		}
	}

	// apply offset limits
	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
		_NE_pos_offset_m[i](0) = constrain(_NE_pos_offset_m[i](0), -max_ne_offset(0), max_ne_offset(0));
		_NE_pos_offset_m[i](1) = constrain(_NE_pos_offset_m[i](1), -max_ne_offset(1), max_ne_offset(1));
		_hgt_offset_mm[i] = constrain(_hgt_offset_mm[i], -max_alt_offset, max_alt_offset);
	}
}

void GpsBlending::calc_gps_blend_output(sensor_gps_s &gps_blended_state,
					float blend_weights[GPS_MAX_RECEIVERS_BLEND]) const
{
	// Convert each GPS position to a local NEU offset relative to the reference position
	// which is defined as the positon of the blended solution calculated from non offset corrected data
	Vector2f blended_NE_offset_m{0, 0};
	float blended_alt_offset_mm = 0.0f;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
		if (blend_weights[i] > 0.0f) {

			// Add the sum of weighted offsets to the reference position to obtain the blended position
			const double lat_deg_orig = (double)_gps_state[i].lat * 1.0e-7;
			const double lon_deg_orig = (double)_gps_state[i].lon * 1.0e-7;
			double lat_deg_offset_res = 0;
			double lon_deg_offset_res = 0;
			add_vector_to_global_position(lat_deg_orig, lon_deg_orig,
						      _NE_pos_offset_m[i](0), _NE_pos_offset_m[i](1),
						      &lat_deg_offset_res, &lon_deg_offset_res);

			float alt_offset = _gps_state[i].alt + (int32_t)_hgt_offset_mm[i];


			// calculate the horizontal offset
			Vector2f horiz_offset{};
			get_vector_to_next_waypoint((gps_blended_state.lat / 1.0e7), (gps_blended_state.lon / 1.0e7),
						    lat_deg_offset_res, lon_deg_offset_res,
						    &horiz_offset(0), &horiz_offset(1));

			// sum weighted offsets
			blended_NE_offset_m += horiz_offset * blend_weights[i];

			// calculate vertical offset
			float vert_offset = alt_offset - gps_blended_state.alt;

			// sum weighted offsets
			blended_alt_offset_mm += vert_offset * blend_weights[i];
		}
	}

	// Add the sum of weighted offsets to the reference position to obtain the blended position
	const double lat_deg_now = (double)gps_blended_state.lat * 1.0e-7;
	const double lon_deg_now = (double)gps_blended_state.lon * 1.0e-7;
	double lat_deg_res = 0;
	double lon_deg_res = 0;
	add_vector_to_global_position(lat_deg_now, lon_deg_now,
				      blended_NE_offset_m(0), blended_NE_offset_m(1),
				      &lat_deg_res, &lon_deg_res);

	gps_blended_state.lat = (int32_t)(1.0E7 * lat_deg_res);
	gps_blended_state.lon = (int32_t)(1.0E7 * lon_deg_res);
	gps_blended_state.alt = gps_blended_state.alt + (int32_t)blended_alt_offset_mm;
}
