/****************************************************************************
 *
 *  Copyright (C) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file test_mathlib.cpp
 * Tests for the PX4 math library.
 */

#include <unit_test.h>

#include <errno.h>
#include <fcntl.h>
#include <float.h>
#include <math.h>
#include <px4_platform_common/px4_config.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>

#include <px4_platform_common/log.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <mathlib/mathlib.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <matrix/math.hpp>

#include "tests_main.h"

class MathlibTest : public UnitTest
{
public:
	virtual bool run_tests();

private:
	bool testVector2();
	bool testVector3();
	bool testVector10();
	bool testMatrix3x3();
	bool testMatrix10x10();
	bool testMatrixNonsymmetric();
	bool testRotationMatrixQuaternion();
	bool testQuaternionfrom_dcm();
	bool testQuaternionfrom_euler();
	bool testQuaternionRotate();
	bool testFinite();
};

#define TEST_OP(_title, _op) { unsigned int n = 30000; hrt_abstime t0, t1; t0 = hrt_absolute_time(); for (unsigned int j = 0; j < n; j++) { _op; }; t1 = hrt_absolute_time(); PX4_INFO(_title ": %.6fus", (double)(t1 - t0) / n); }

using namespace math;

bool MathlibTest::testVector2()
{
	{
		matrix::Vector2f v;
		matrix::Vector2f v1(1.0f, 2.0f);
		matrix::Vector2f v2(1.0f, -1.0f);
		float data[2] = {1.0f, 2.0f};
		TEST_OP("Constructor matrix::Vector2f()", matrix::Vector2f v3);
		TEST_OP("Constructor matrix::Vector2f(matrix::Vector2f)", matrix::Vector2f v3(v1); ut_assert_true(v3 == v1); v3.zero());
		TEST_OP("Constructor matrix::Vector2f(float[])", matrix::Vector2f v3(data));
		TEST_OP("Constructor matrix::Vector2f(float, float)", matrix::Vector2f v3(1.0f, 2.0f));
		TEST_OP("matrix::Vector2f = matrix::Vector2f", v = v1);
		TEST_OP("matrix::Vector2f + matrix::Vector2f", v + v1);
		TEST_OP("matrix::Vector2f - matrix::Vector2f", v - v1);
		TEST_OP("matrix::Vector2f += matrix::Vector2f", v += v1);
		TEST_OP("matrix::Vector2f -= matrix::Vector2f", v -= v1);
		TEST_OP("matrix::Vector2f * matrix::Vector2f", v * v1);
		TEST_OP("matrix::Vector2f %% matrix::Vector2f", v1 % v2);
	}
	return true;
}

bool MathlibTest::testVector3()
{

	{
		matrix::Vector3f v;
		matrix::Vector3f v1(1.0f, 2.0f, 0.0f);
		matrix::Vector3f v2(1.0f, -1.0f, 2.0f);
		float data[3] = {1.0f, 2.0f, 3.0f};
		TEST_OP("Constructor matrix::Vector3f()", matrix::Vector3f v3);
		TEST_OP("Constructor matrix::Vector3f(matrix::Vector3f)", matrix::Vector3f v3(v1); ut_assert_true(v3 == v1); v3.zero());
		TEST_OP("Constructor matrix::Vector3f(float[])", matrix::Vector3f v3(data));
		TEST_OP("Constructor matrix::Vector3f(float, float, float)", matrix::Vector3f v3(1.0f, 2.0f, 3.0f));
		TEST_OP("matrix::Vector3f = matrix::Vector3f", v = v1);
		TEST_OP("matrix::Vector3f + matrix::Vector3f", v + v1);
		TEST_OP("matrix::Vector3f - matrix::Vector3f", v - v1);
		TEST_OP("matrix::Vector3f += matrix::Vector3f", v += v1);
		TEST_OP("matrix::Vector3f -= matrix::Vector3f", v -= v1);
		TEST_OP("matrix::Vector3f * float", v1 * 2.0f);
		TEST_OP("matrix::Vector3f / float", v1 / 2.0f);
		TEST_OP("matrix::Vector3f *= float", v1 *= 2.0f);
		TEST_OP("matrix::Vector3f /= float", v1 /= 2.0f);
		TEST_OP("matrix::Vector3f * matrix::Vector3f", v * v1);
		TEST_OP("matrix::Vector3f %% matrix::Vector3f", v1 % v2);
		TEST_OP("matrix::Vector3f length", v1.length());
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
		// Need pragma here instead of moving variable out of TEST_OP and just reference because
		// TEST_OP measures performance of vector operations.
		TEST_OP("matrix::Vector3f element read", volatile float a = v1(0));
#pragma GCC diagnostic pop
		TEST_OP("matrix::Vector3f element write", v1(0) = 1.0f);
	}
	return true;
}

bool MathlibTest::testMatrix3x3()
{
	{
		matrix::Matrix3f m1;
		m1.identity();
		matrix::Matrix3f m2;
		m2.identity();
		matrix::Vector3f v1(1.0f, 2.0f, 0.0f);
		TEST_OP("matrix::Matrix3f * matrix::Vector3f", m1 * v1);
		TEST_OP("matrix::Matrix3f + matrix::Matrix3f", m1 + m2);
		TEST_OP("matrix::Matrix3f * matrix::Matrix3f", m1 * m2);
	}
	return true;
}

bool MathlibTest::testMatrixNonsymmetric()
{
	int rc = true;
	{
		//PX4_INFO("Nonsymmetric matrix operations test");
		// test nonsymmetric +, -, +=, -=

		float data1[2][3] = {{1, 2, 3}, {4, 5, 6}};
		float data2[2][3] = {{2, 4, 6}, {8, 10, 12}};
		float data3[2][3] = {{3, 6, 9}, {12, 15, 18}};

		matrix::Matrix<float, 2, 3> m1(data1);
		matrix::Matrix<float, 2, 3> m2(data2);
		matrix::Matrix<float, 2, 3> m3(data3);

		if (m1 + m2 != m3) {
			PX4_ERR("matrix::Matrix<float, 2, 3> + matrix::Matrix<float, 2, 3> failed!");
			(m1 + m2).print();
			printf("!=\n");
			m3.print();
			rc = false;
		}

		ut_assert("m1 + m2 == m3", m1 + m2 == m3);

		if (m3 - m2 != m1) {
			PX4_ERR("matrix::Matrix<float, 2, 3> - matrix::Matrix<float, 2, 3> failed!");
			(m3 - m2).print();
			printf("!=\n");
			m1.print();
			rc = false;
		}

		ut_assert("m3 - m2 == m1", m3 - m2 == m1);

		m1 += m2;

		if (m1 != m3) {
			PX4_ERR("matrix::Matrix<float, 2, 3> += matrix::Matrix<float, 2, 3> failed!");
			m1.print();
			printf("!=\n");
			m3.print();
			rc = false;
		}

		ut_assert("m1 == m3", m1 == m3);

		m1 -= m2;
		matrix::Matrix<float, 2, 3> m1_orig(data1);

		if (m1 != m1_orig) {
			PX4_ERR("matrix::Matrix<float, 2, 3> -= matrix::Matrix<float, 2, 3> failed!");
			m1.print();
			printf("!=\n");
			m1_orig.print();
			rc = false;
		}

		ut_assert("m1 == m1_orig", m1 == m1_orig);

	}
	return rc;
}

bool MathlibTest::testRotationMatrixQuaternion()
{
	// test conversion rotation matrix to quaternion and back
	matrix::Dcmf R_orig;
	matrix::Dcmf R;
	matrix::Quatf q;
	float diff = 0.2f;
	float tol = 0.00001f;

	//PX4_INFO("Quaternion transformation methods test.");

	for (float roll = -M_PI_F; roll <= M_PI_F; roll += diff) {
		for (float pitch = -M_PI_2_F; pitch <= M_PI_2_F; pitch += diff) {
			for (float yaw = -M_PI_F; yaw <= M_PI_F; yaw += diff) {
				R_orig = matrix::Eulerf(roll, pitch, yaw);
				q = R_orig;
				R = q;

				for (int i = 0; i < 3; i++) {
					for (int j = 0; j < 3; j++) {
						ut_assert("matrix::Quatf method 'from_dcm' or 'to_dcm' outside tolerance!", fabsf(R_orig(i, j) - R(i, j)) < tol);
					}
				}
			}
		}
	}

	return true;
}


bool MathlibTest::testQuaternionfrom_dcm()
{
	// test against some known values
	float tol = 0.0001f;
	matrix::Quatf q_true = {1.0f, 0.0f, 0.0f, 0.0f};

	matrix::Matrix3f R_orig;
	R_orig.identity();

	matrix::Quatf q(R_orig);

	for (unsigned i = 0; i < 4; i++) {
		ut_assert("matrix::Quatf method 'from_dcm()' outside tolerance!", fabsf(q(i) - q_true(i)) < tol);
	}

	return true;
}

bool MathlibTest::testQuaternionfrom_euler()
{
	float tol = 0.0001f;
	matrix::Quatf q_true = {1.0f, 0.0f, 0.0f, 0.0f};

	matrix::Matrix3f R_orig;
	R_orig.identity();

	matrix::Quatf q(R_orig);

	q_true = matrix::Eulerf(0.3f, 0.2f, 0.1f);
	q = {0.9833f, 0.1436f, 0.1060f, 0.0343f};

	for (unsigned i = 0; i < 4; i++) {
		ut_assert("matrix::Quatf method 'from_euler()' outside tolerance!", fabsf(q(i) - q_true(i)) < tol);
	}

	q_true = matrix::Eulerf(-1.5f, -0.2f, 0.5f);
	q = {0.7222f, -0.6391f, -0.2386f, 0.1142f};

	for (unsigned i = 0; i < 4; i++) {
		ut_assert("matrix::Quatf method 'from_euler()' outside tolerance!", fabsf(q(i) - q_true(i)) < tol);
	}

	q_true = matrix::Eulerf(M_PI_2_F, -M_PI_2_F, -M_PI_F / 3);
	q = {0.6830f, 0.1830f, -0.6830f, 0.1830f};

	for (unsigned i = 0; i < 4; i++) {
		ut_assert("matrix::Quatf method 'from_euler()' outside tolerance!", fabsf(q(i) - q_true(i)) < tol);
	}

	return true;
}

bool MathlibTest::testQuaternionRotate()
{
	// test quaternion method "rotate" (rotate vector by quaternion)
	matrix::Vector3f vector = {1.0f, 1.0f, 1.0f};
	matrix::Vector3f vector_q;
	matrix::Vector3f vector_r;
	matrix::Quatf q;
	matrix::Dcmf R;
	float diff = 0.2f;
	float tol = 0.00001f;

	//PX4_INFO("matrix::Quatf vector rotation method test.");

	for (float roll = -M_PI_F; roll <= M_PI_F; roll += diff) {
		for (float pitch = -M_PI_2_F; pitch <= M_PI_2_F; pitch += diff) {
			for (float yaw = -M_PI_F; yaw <= M_PI_F; yaw += diff) {
				R = matrix::Eulerf(roll, pitch, yaw);
				q = matrix::Eulerf(roll, pitch, yaw);
				vector_r = R * vector;
				vector_q = q.conjugate(vector);

				for (int i = 0; i < 3; i++) {
					ut_assert("matrix::Quatf method 'rotate' outside tolerance", fabsf(vector_r(i) - vector_q(i)) < tol);
				}
			}
		}
	}


	// test some values calculated with matlab
	tol = 0.0001f;
	q = matrix::Eulerf(M_PI_2_F, 0.0f, 0.0f);
	vector_q = q.conjugate(vector);
	matrix::Vector3f vector_true = {1.00f, -1.00f, 1.00f};

	for (unsigned i = 0; i < 3; i++) {
		ut_assert("matrix::Quatf method 'rotate' outside tolerance", fabsf(vector_true(i) - vector_q(i)) < tol);
	}

	q = matrix::Eulerf(0.3f, 0.2f, 0.1f);
	vector_q = q.conjugate(vector);
	vector_true = {1.1566, 0.7792, 1.0273};

	for (unsigned i = 0; i < 3; i++) {
		ut_assert("matrix::Quatf method 'rotate' outside tolerance", fabsf(vector_true(i) - vector_q(i)) < tol);
	}

	q = matrix::Eulerf(-1.5f, -0.2f, 0.5f);
	vector_q = q.conjugate(vector);
	vector_true = {0.5095, 1.4956, -0.7096};

	for (unsigned i = 0; i < 3; i++) {
		ut_assert("matrix::Quatf method 'rotate' outside tolerance", fabsf(vector_true(i) - vector_q(i)) < tol);
	}

	q = matrix::Eulerf(M_PI_2_F, -M_PI_2_F, -M_PI_F / 3.0f);
	vector_q = q.conjugate(vector);
	vector_true = { -1.3660, 0.3660, 1.0000};

	for (unsigned i = 0; i < 3; i++) {
		ut_assert("matrix::Quatf method 'rotate' outside tolerance", fabsf(vector_true(i) - vector_q(i)) < tol);
	}

	return true;
}

bool MathlibTest::testFinite()
{
	ut_assert("PX4_ISFINITE(0.0f)", PX4_ISFINITE(0.0f) == true);
	ut_assert("PX4_ISFINITE(-0.0f)", PX4_ISFINITE(-0.0f) == true);
	ut_assert("PX4_ISFINITE(1.0f)", PX4_ISFINITE(1.0f) == true);
	ut_assert("PX4_ISFINITE(-1.0f)", PX4_ISFINITE(-1.0f) == true);

	ut_assert("PX4_ISFINITE(NAN)", PX4_ISFINITE(NAN) == false);
	ut_assert("PX4_ISFINITE(1/0)", PX4_ISFINITE(1.0f / 0.0f) == false);
	ut_assert("PX4_ISFINITE(0/0)", PX4_ISFINITE(0.0f / 0.0f) == false);
	ut_assert("PX4_ISFINITE(INFINITY)", PX4_ISFINITE(INFINITY) == false);
	ut_assert("PX4_ISFINITE(NAN * INFINITY)", PX4_ISFINITE(NAN * INFINITY) == false);
	ut_assert("PX4_ISFINITE(NAN * 1.0f)", PX4_ISFINITE(NAN * 1.0f) == false);
	ut_assert("PX4_ISFINITE(INFINITY * 2.0f)", PX4_ISFINITE(INFINITY * 2.0f) == false);

	return true;
}

bool MathlibTest::run_tests()
{
	ut_run_test(testVector2);
	ut_run_test(testVector3);
	ut_run_test(testMatrix3x3);
	ut_run_test(testMatrixNonsymmetric);
	ut_run_test(testRotationMatrixQuaternion);
	ut_run_test(testQuaternionfrom_dcm);
	ut_run_test(testQuaternionfrom_euler);
	ut_run_test(testQuaternionRotate);
	ut_run_test(testFinite);

	return (_tests_failed == 0);
}

ut_declare_test_c(test_mathlib, MathlibTest)
