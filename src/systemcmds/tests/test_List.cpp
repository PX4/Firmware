/****************************************************************************
 *
 *  Copyright (C) 2019 PX4 Development Team. All rights reserved.
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
 * @file test_List.cpp
 * Tests the List container.
 */

#include <unit_test.h>
#include <containers/List.hpp>
#include <float.h>
#include <math.h>

class testContainer : public ListNode<testContainer *>
{
public:
	int i{0};
};

class ListTest : public UnitTest
{
public:
	virtual bool run_tests();

	bool test_add();
	bool test_remove();
	bool test_range_based_for();
	bool test_reinsert();

};

bool ListTest::run_tests()
{
	ut_run_test(test_add);
	ut_run_test(test_remove);
	ut_run_test(test_range_based_for);
	ut_run_test(test_reinsert);

	return (_tests_failed == 0);
}

bool ListTest::test_add()
{
	List<testContainer *> list1;

	// size should be 0 initially
	ut_compare("size initially 0", list1.size(), 0);
	ut_assert_true(list1.empty());

	// insert 100
	for (int i = 0; i < 100; i++) {
		testContainer *t = new testContainer();
		t->i = i;
		list1.add(t);

		ut_compare("size increasing with i", list1.size(), i + 1);
		ut_assert_true(!list1.empty());
	}

	// verify full size (100)
	ut_assert_true(list1.size() == 100);

	int i = 0;

	for (auto t : list1) {
		// verify all elements were inserted in order
		ut_compare("stored i", i, t->i);
		i++;
	}

	// delete all elements
	list1.clear();

	// verify list has been cleared
	ut_assert_true(list1.empty());
	ut_compare("size 0", list1.size(), 0);

	return true;
}

bool ListTest::test_remove()
{
	List<testContainer *> list1;

	// size should be 0 initially
	ut_compare("size initially 0", list1.size(), 0);
	ut_assert_true(list1.empty());

	// insert 100
	for (int i = 0; i < 100; i++) {
		testContainer *t = new testContainer();
		t->i = i;
		list1.add(t);

		ut_compare("size increasing with i", list1.size(), i + 1);
		ut_assert_true(!list1.empty());
	}

	// verify full size (100)
	ut_assert_true(list1.size() == 100);

	// test removing elements
	for (int remove_i = 0; remove_i < 100; remove_i++) {

		// find node with i == remove_i
		testContainer *removed = nullptr;

		for (auto t : list1) {
			if (t->i == remove_i) {
				ut_assert_true(list1.remove(t));
				removed = t;
			}
		}

		delete removed;

		// iterate list again to verify removal
		for (auto t : list1) {
			ut_assert_true(t->i != remove_i);
		}

		ut_assert_true(list1.size() == 100 - remove_i - 1);
	}

	// list should now be empty
	ut_assert_true(list1.empty());
	ut_compare("size 0", list1.size(), 0);

	// delete all elements (should be safe on empty list)
	list1.clear();

	// verify list has been cleared
	ut_assert_true(list1.empty());
	ut_compare("size 0", list1.size(), 0);

	return true;
}

bool ListTest::test_range_based_for()
{
	List<testContainer *> list1;

	// size should be 0 initially
	ut_compare("size initially 0", list1.size(), 0);
	ut_assert_true(list1.empty());

	// insert 100 elements in order
	for (int i = 99; i >= 0; i--) {
		testContainer *t = new testContainer();
		t->i = i;

		list1.add(t);

		ut_assert_true(!list1.empty());
	}

	// first element should be 99 (first added)
	ut_compare("first 0", list1.getHead()->i, 99);

	// verify all elements were inserted in order
	int i = 99;
	auto t1 = list1.getHead();

	while (t1 != nullptr) {
		ut_compare("check count", i, t1->i);
		t1 = t1->getSibling();
		i--;
	}

	// verify full size (100)
	ut_compare("size check", list1.size(), 100);

	i = 99;

	for (auto t2 : list1) {
		ut_compare("range based for i", i, t2->i);
		i--;
	}

	// verify full size (100)
	ut_compare("size check", list1.size(), 100);

	// delete all elements
	list1.clear();

	// verify list has been cleared
	ut_assert_true(list1.empty());
	ut_compare("size check", list1.size(), 0);

	return true;
}

bool ListTest::test_reinsert()
{
	List<testContainer *> l1;

	// size should be 0 initially
	ut_compare("size initially 0", l1.size(), 0);
	ut_assert_true(l1.empty());

	// insert 100
	for (int i = 0; i < 100; i++) {
		testContainer *t = new testContainer();
		t->i = i;
		l1.add(t);

		ut_compare("size increasing with i", l1.size(), i + 1);
		ut_assert_true(!l1.empty());
	}

	// verify full size (100)
	ut_assert_true(l1.size() == 100);
	ut_assert_false(l1.empty());

	// test removing elements
	for (int remove_i = 0; remove_i < 100; remove_i++) {

		ut_assert_false(l1.empty());

		// find node with i == remove_i
		testContainer *removed = nullptr;

		for (auto t : l1) {
			if (t->i == remove_i) {
				ut_assert_true(l1.remove(t));
				removed = t;
			}
		}

		// l1 shouldn't be empty until the very last iteration
		ut_assert_false(l1.empty());

		// iterate list again to verify removal
		for (auto t : l1) {
			ut_assert_true(t->i != remove_i);
		}

		// size temporarily reduced by 1
		ut_assert_true(l1.size() == 100 - 1);

		// now re-insert the removed node
		const size_t sz1 = l1.size();
		l1.add(removed);
		const size_t sz2 = l1.size();

		// verify the size increase
		ut_assert_true(sz2 > sz1);

		// size restored to 100
		ut_assert_true(l1.size() == 100);
	}

	// queue shouldn't be empty
	ut_assert_false(l1.empty());
	ut_compare("size still 100", l1.size(), 100);

	// delete all elements
	l1.clear();

	// verify list has been cleared
	ut_assert_true(l1.empty());
	ut_compare("size 0", l1.size(), 0);

	return true;
}

ut_declare_test_c(test_List, ListTest)
