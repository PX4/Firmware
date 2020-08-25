/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include <unit_test.h>

#include <px4_platform_common/atomic_bitset.h>

class AtomicBitsetTest : public UnitTest
{
public:
	virtual bool run_tests();

private:
	bool constructTest();
	bool setAllTest();
	bool setRandomTest();

};

bool AtomicBitsetTest::run_tests()
{
	ut_run_test(constructTest);
	ut_run_test(setAllTest);
	ut_run_test(setRandomTest);

	return (_tests_failed == 0);
}


ut_declare_test_c(test_atomic_bitset, AtomicBitsetTest)

bool AtomicBitsetTest::constructTest()
{
	px4::AtomicBitset<10> test_bitset1;

	ut_compare("bitset size 10", test_bitset1.size(), 10);
	ut_compare("bitset init count 0", test_bitset1.count(), 0);

	for (int i = 0; i < test_bitset1.size(); i++) {
		ut_compare("bitset not set by default", test_bitset1[i], false);
	}

	return true;
}

bool AtomicBitsetTest::setAllTest()
{
	px4::AtomicBitset<100> test_bitset2;

	ut_compare("bitset size 100", test_bitset2.size(), 100);
	ut_compare("bitset init count 0", test_bitset2.count(), 0);

	for (int i = 0; i < test_bitset2.size(); i++) {
		ut_compare("bitset not set by default", test_bitset2[i], false);
	}

	// set all
	for (int i = 0; i < test_bitset2.size(); i++) {
		test_bitset2.set(i, true);
	}

	// check count
	ut_compare("bitset count", test_bitset2.count(), 100);

	// verify all set
	for (int i = 0; i < test_bitset2.size(); i++) {
		ut_compare("bitset not true", test_bitset2[i], true);
	}

	// set all back to false
	for (int i = 0; i < test_bitset2.size(); i++) {
		test_bitset2.set(i, false);
	}

	// check count
	ut_compare("bitset count", test_bitset2.count(), 0);

	// verify all no longer set
	for (int i = 0; i < test_bitset2.size(); i++) {
		ut_compare("bitset not false", test_bitset2[i], false);
	}

	return true;
}

bool AtomicBitsetTest::setRandomTest()
{
	px4::AtomicBitset<999> test_bitset3;

	ut_compare("bitset size 999", test_bitset3.size(), 999);
	ut_compare("bitset init count 0", test_bitset3.count(), 0);

	for (int i = 0; i < test_bitset3.size(); i++) {
		ut_compare("bitset not set by default", test_bitset3[i], false);
	}

	// random set and verify 100 elements
	const int random_test_size = 5;
	int random_array[random_test_size] = { 3, 1, 4, 5, 9 };

	// set random elements
	for (auto x : random_array) {
		test_bitset3.set(x, true);
		ut_less_than("invalid test element range", x, test_bitset3.size());
	}

	// check count
	ut_compare("bitset count", test_bitset3.count(), random_test_size);

	// check that only random elements are set
	for (int i = 0; i < test_bitset3.size(); i++) {

		// is i in the random test array
		// if so it should be set
		bool i_in_random = false;

		for (auto x : random_array) {
			if (i == x) {
				i_in_random = true;
			}
		}

		if (i_in_random) {
			ut_compare("bitset true", test_bitset3[i], true);

		} else {
			ut_compare("bitset false", test_bitset3[i], false);
		}
	}

	// set all back to false
	for (int i = 0; i < test_bitset3.size(); i++) {
		test_bitset3.set(i, false);
	}

	// check count
	ut_compare("bitset count", test_bitset3.count(), 0);

	// verify all no longer set
	for (int i = 0; i < test_bitset3.size(); i++) {
		ut_compare("bitset not false", test_bitset3[i], false);
	}

	return true;
}
