/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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
 * Test code for the GPS blending logic
 * Run this test only using make tests TESTFILTER=GpsBlending
 *
 * @author Mathieu Bresciani <mathieu@auterion.com>
 */

#include <gtest/gtest.h>
#include <matrix/matrix/math.hpp>

#include "gps_blending.hpp"

using matrix::Vector3f;

class GpsBlendingTest : public ::testing::Test
{
public:
	sensor_gps_s getDefaultGpsData();
	void runSeconds(float duration_s, GpsBlending &gps_blending, sensor_gps_s &gps_data, int instance);
	void runSeconds(float duration_s, GpsBlending &gps_blending, sensor_gps_s &gps_data0, sensor_gps_s &gps_data1);

	uint64_t _time_now_us{1000000};
};

sensor_gps_s GpsBlendingTest::getDefaultGpsData()
{
	sensor_gps_s gps_data{};
	gps_data.timestamp = _time_now_us - 10e3;
	gps_data.time_utc_usec = 0;
	gps_data.lat = 47e7;
	gps_data.lon = 9e7;
	gps_data.alt = 800e3;
	gps_data.alt_ellipsoid = 800e3;
	gps_data.s_variance_m_s = 0.2f;
	gps_data.c_variance_rad = 0.5f;
	gps_data.eph = 0.7f;
	gps_data.epv = 1.2f;
	gps_data.hdop = 1.f;
	gps_data.vdop = 1.f;
	gps_data.noise_per_ms = 20;
	gps_data.jamming_indicator = 40;
	gps_data.vel_m_s = 1.f;
	gps_data.vel_n_m_s = 1.f;
	gps_data.vel_e_m_s = 1.f;
	gps_data.vel_d_m_s = 1.f;
	gps_data.cog_rad = 0.f;
	gps_data.timestamp_time_relative = 0;
	gps_data.heading = NAN;
	gps_data.heading_offset = 0.f;
	gps_data.fix_type = 4;
	gps_data.vel_ned_valid = true;
	gps_data.satellites_used = 8;

	return gps_data;
}

void GpsBlendingTest::runSeconds(float duration_s, GpsBlending &gps_blending, sensor_gps_s &gps_data, int instance)
{
	const float dt = 0.1;
	const uint64_t dt_us = static_cast<uint64_t>(dt * 1e6f);

	for (int k = 0; k < static_cast<int>(duration_s / dt); k++) {
		gps_blending.setGpsData(gps_data, instance);
		gps_blending.update(_time_now_us);

		_time_now_us += dt_us;
		gps_data.timestamp += dt_us;
	}
}

void GpsBlendingTest::runSeconds(float duration_s, GpsBlending &gps_blending, sensor_gps_s &gps_data0,
				 sensor_gps_s &gps_data1)
{
	const float dt = 0.1;
	const uint64_t dt_us = static_cast<uint64_t>(dt * 1e6f);

	for (int k = 0; k < static_cast<int>(duration_s / dt); k++) {
		gps_blending.setGpsData(gps_data0, 0);
		gps_blending.setGpsData(gps_data1, 1);

		gps_blending.update(_time_now_us);

		_time_now_us += dt_us;
		gps_data0.timestamp += dt_us;
		gps_data1.timestamp += dt_us;
	}
}

TEST_F(GpsBlendingTest, noData)
{
	GpsBlending gps_blending;

	EXPECT_EQ(gps_blending.getSelectedGps(), 0);
	EXPECT_FALSE(gps_blending.isNewOutputDataAvailable());

	gps_blending.update(_time_now_us);

	EXPECT_EQ(gps_blending.getSelectedGps(), 0);
	EXPECT_FALSE(gps_blending.isNewOutputDataAvailable());
}

TEST_F(GpsBlendingTest, singleReceiver)
{
	GpsBlending gps_blending;

	gps_blending.setPrimaryInstance(-1);
	sensor_gps_s gps_data = getDefaultGpsData();

	gps_blending.setGpsData(gps_data, 1);
	gps_blending.update(_time_now_us);

	_time_now_us += 200e3;
	gps_data.timestamp = _time_now_us - 10e3;
	gps_blending.setGpsData(gps_data, 1);
	gps_blending.update(_time_now_us);

	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_EQ(gps_blending.getNumberOfGpsSuitableForBlending(), 1);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());

	// BUT IF: a second update is called without data
	gps_blending.update(_time_now_us);

	// THEN: no new data should be available
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_EQ(gps_blending.getNumberOfGpsSuitableForBlending(), 1);
	EXPECT_FALSE(gps_blending.isNewOutputDataAvailable());
}

TEST_F(GpsBlendingTest, dualReceiverNoBlending)
{
	GpsBlending gps_blending;

	// GIVEN: two receivers with the same prioity
	gps_blending.setPrimaryInstance(-1);
	sensor_gps_s gps_data0 = getDefaultGpsData();
	sensor_gps_s gps_data1 = getDefaultGpsData();

	gps_data1.satellites_used = gps_data0.satellites_used + 2; // gps1 has more satellites than gps0
	gps_blending.setGpsData(gps_data0, 0);
	gps_blending.setGpsData(gps_data1, 1);
	gps_blending.update(_time_now_us);

	// THEN: gps1 should be selected because it has more satellites
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_EQ(gps_blending.getNumberOfGpsSuitableForBlending(), 2);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());

	gps_data1.satellites_used = gps_data0.satellites_used - 2; // gps1 has less satellites than gps0
	gps_blending.setGpsData(gps_data0, 0);
	gps_blending.setGpsData(gps_data1, 1);
	gps_blending.update(_time_now_us);

	// THEN: gps0 should be selected because it has more satellites
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);
	EXPECT_EQ(gps_blending.getNumberOfGpsSuitableForBlending(), 2);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());
}

TEST_F(GpsBlendingTest, dualReceiverBlendingHPos)
{
	GpsBlending gps_blending;

	sensor_gps_s gps_data0 = getDefaultGpsData();
	sensor_gps_s gps_data1 = getDefaultGpsData();

	gps_blending.setBlendingUseHPosAccuracy(true);

	gps_data1.eph = gps_data0.eph / 2.f;
	gps_blending.setGpsData(gps_data0, 0);
	gps_blending.setGpsData(gps_data1, 1);
	gps_blending.update(_time_now_us);

	// THEN: the blended instance should be selected (2)
	// and the eph should be adjusted
	EXPECT_EQ(gps_blending.getSelectedGps(), 2);
	EXPECT_EQ(gps_blending.getNumberOfGpsSuitableForBlending(), 2);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());
	EXPECT_LT(gps_blending.getOutputGpsData().eph, gps_data0.eph);
	EXPECT_FLOAT_EQ(gps_blending.getOutputGpsData().eph, gps_data1.eph); // TODO: should be greater than
}

TEST_F(GpsBlendingTest, dualReceiverFailover)
{
	GpsBlending gps_blending;

	// GIVEN: a dual GPS setup with the first instance (0)
	// set as primary
	gps_blending.setPrimaryInstance(0);
	gps_blending.setBlendingUseSpeedAccuracy(false);
	gps_blending.setBlendingUseHPosAccuracy(false);
	gps_blending.setBlendingUseVPosAccuracy(false);

	// WHEN: only the secondary receiver is available
	sensor_gps_s gps_data1 = getDefaultGpsData();

	const float duration_s = 10.f;
	runSeconds(duration_s, gps_blending, gps_data1, 1);

	// THEN: the primary instance should be selected even if
	// not available. No data is then available
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);
	EXPECT_EQ(gps_blending.getNumberOfGpsSuitableForBlending(), 1);
	EXPECT_FALSE(gps_blending.isNewOutputDataAvailable());

	// BUT WHEN: the data of the primary receiver is avaialbe
	sensor_gps_s gps_data0 = getDefaultGpsData();
	gps_blending.setGpsData(gps_data0, 0);
	gps_blending.update(_time_now_us);

	// THEN: the primary instance is selected and the data
	// is available
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);
	EXPECT_EQ(gps_blending.getNumberOfGpsSuitableForBlending(), 2);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());

	runSeconds(duration_s, gps_blending, gps_data0, gps_data1);

	EXPECT_EQ(gps_blending.getSelectedGps(), 0);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());

	// BUT WHEN: the primary receiver isn't available anymore
	runSeconds(duration_s, gps_blending, gps_data1, 1);

	// THEN: the data of the secondary receiver can be used
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());

	// AND IF: the primary receiver is available again and has
	// better metrics than the secondary one
	gps_data0.timestamp = gps_data1.timestamp;
	gps_data0.satellites_used = gps_data1.satellites_used + 2;

	runSeconds(1.f, gps_blending, gps_data0, gps_data1);

	// THEN: the primary receiver should be used again
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());
}
