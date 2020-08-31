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

#include <unit_test.h>
#include <containers/IntrusiveQueue.hpp>
#include <float.h>
#include <math.h>

class testContainer : public IntrusiveQueueNode<testContainer *>
{
public:
	int i{0};
};

class IntrusiveQueueTest : public UnitTest
{
public:
	virtual bool run_tests();

	bool test_push();
	bool test_pop();
	bool test_push_duplicate();
	bool test_remove();
	bool test_reinsert();

};

bool IntrusiveQueueTest::run_tests()
{
	ut_run_test(test_push);
	ut_run_test(test_pop);
	ut_run_test(test_push_duplicate);
	ut_run_test(test_remove);
	ut_run_test(test_reinsert);

	return (_tests_failed == 0);
}

bool IntrusiveQueueTest::test_push()
{
	IntrusiveQueue<testContainer *> q1;

	// size should be 0 initially
	ut_compare("size initially 0", q1.size(), 0);
	ut_assert_true(q1.empty());

	// insert 100
	for (int i = 0; i < 100; i++) {
		testContainer *t = new testContainer();
		t->i = i;

		ut_compare("size increasing with i", q1.size(), i);
		q1.push(t);
		ut_compare("size increasing with i", q1.size(), i + 1);

		ut_assert_false(q1.empty());
	}

	// verify full size (100)
	ut_compare("size 100", q1.size(), 100);

	// pop all elements
	for (int i = 0; i < 100; i++) {
		auto node = q1.front();


		ut_assert_false(q1.empty());

		ut_assert_true(q1.pop() == node);
		delete node;

		bool expect_empty = (i == 100 - 1);
		ut_compare("empty", q1.empty(), expect_empty);
	}

	// verify list has been cleared
	ut_compare("size 0", q1.size(), 0);
	ut_assert_true(q1.empty());

	return true;
}

bool IntrusiveQueueTest::test_pop()
{
	IntrusiveQueue<testContainer *> q1;

	// size should be 0 initially
	ut_compare("size initially 0", q1.size(), 0);
	ut_assert_true(q1.empty());

	// insert 100
	for (int i = 0; i < 100; i++) {
		testContainer *t = new testContainer();
		t->i = i;
		q1.push(t);
	}

	// verify full size (100)
	ut_assert_true(q1.size() == 100);
	ut_assert_false(q1.empty());

	for (int i = 0; i < 100; i++) {
		if (i < 100 - 1) {
			ut_assert_false(q1.empty());
		}

		auto node = q1.front();
		ut_compare("stored i", i, node->i);

		ut_compare("size check", q1.size(), 100 - i);
		q1.pop();

		if (i < 100 - 1) {
			ut_assert_false(q1.empty());

		} else {
			ut_assert_true(q1.empty());
		}

		ut_compare("size check", q1.size(), 100 - i - 1);

		delete node;

		ut_compare("size check", q1.size(), 100 - i - 1);
	}

	// verify list has been cleared
	ut_assert_true(q1.empty());
	ut_compare("size check", q1.size(), 0);

	// pop an empty queue
	auto T = q1.pop();
	ut_assert_true(T == nullptr);
	ut_assert_true(q1.empty());
	ut_compare("size check", q1.size(), 0);

	return true;
}

bool IntrusiveQueueTest::test_push_duplicate()
{
	IntrusiveQueue<testContainer *> q1;

	// size should be 0 initially
	ut_compare("size initially 0", q1.size(), 0);
	ut_assert_true(q1.empty());

	// insert 100
	for (int i = 0; i < 100; i++) {
		testContainer *t = new testContainer();
		t->i = i;

		ut_compare("size increasing with i", q1.size(), i);
		q1.push(t);
		ut_compare("size increasing with i", q1.size(), i + 1);

		ut_assert_true(!q1.empty());
	}

	// verify full size (100)
	ut_compare("size 100", q1.size(), 100);


	// attempt to insert front again
	const auto q1_front = q1.front();
	const auto q1_front_i = q1_front->i; // copy i value

	const auto q1_back = q1.back();
	const auto q1_back_i = q1_back->i; // copy i value

	// push front and back aagain
	q1.push(q1_front);
	q1.push(q1_back);

	// verify no change
	ut_compare("size 100", q1.size(), 100);
	ut_compare("q front not reinserted", q1.front()->i, q1_front->i);
	ut_compare("q back not reinserted", q1.back()->i, q1_back->i);


	// pop the head
	const auto q1_head = q1.pop();

	// verfify size should now be 99
	ut_compare("size 99", q1.size(), 99);

	// push back on
	q1.push(q1_head);

	// verify size now 100 again
	ut_compare("size 100", q1.size(), 100);


	// pop all elements
	for (int i = 0; i < 100; i++) {
		auto node = q1.front();
		q1.pop();
		delete node;
	}

	// verify list has been cleared
	ut_compare("size 0", q1.size(), 0);
	ut_assert_true(q1.empty());

	return true;
}

bool IntrusiveQueueTest::test_remove()
{
	IntrusiveQueue<testContainer *> q1;

	// size should be 0 initially
	ut_compare("size initially 0", q1.size(), 0);
	ut_assert_true(q1.empty());

	// insert 100
	for (int i = 0; i < 100; i++) {
		testContainer *t = new testContainer();
		t->i = i;
		q1.push(t);

		ut_compare("size increasing with i", q1.size(), i + 1);
		ut_assert_true(!q1.empty());
	}

	// verify full size (100)
	ut_assert_true(q1.size() == 100);
	ut_assert_false(q1.empty());

	// test removing elements
	for (int remove_i = 0; remove_i < 100; remove_i++) {

		ut_assert_false(q1.empty());

		// find node with i == remove_i
		testContainer *removed = nullptr;

		for (auto t : q1) {
			if (t->i == remove_i) {
				ut_assert_true(q1.remove(t));
				removed = t;
			}
		}

		// q1 shouldn't be empty until the very last iteration
		if (remove_i < 100 - 1) {
			ut_assert_false(q1.empty());

		} else {
			ut_assert_true(q1.empty());
		}

		delete removed;

		// iterate list again to verify removal
		for (auto t : q1) {
			ut_assert_true(t->i != remove_i);
		}

		ut_assert_true(q1.size() == 100 - remove_i - 1);
	}

	// list should now be empty
	ut_assert_true(q1.empty());
	ut_compare("size 0", q1.size(), 0);

	// delete all elements (should be safe on empty list)
	while (!q1.empty()) {
		q1.pop();
	}

	// verify list has been cleared
	ut_assert_true(q1.empty());
	ut_compare("size 0", q1.size(), 0);

	return true;
}

bool IntrusiveQueueTest::test_reinsert()
{
	IntrusiveQueue<testContainer *> q1;

	// size should be 0 initially
	ut_compare("size initially 0", q1.size(), 0);
	ut_assert_true(q1.empty());

	// insert 100
	for (int i = 0; i < 100; i++) {
		testContainer *t = new testContainer();
		t->i = i;
		q1.push(t);

		ut_compare("size increasing with i", q1.size(), i + 1);
		ut_assert_true(!q1.empty());
	}

	// verify full size (100)
	ut_assert_true(q1.size() == 100);
	ut_assert_false(q1.empty());

	// test removing elements
	for (int remove_i = 0; remove_i < 100; remove_i++) {

		ut_assert_false(q1.empty());

		// find node with i == remove_i
		testContainer *removed = nullptr;

		for (auto t : q1) {
			if (t->i == remove_i) {
				ut_assert_true(q1.remove(t));
				removed = t;
			}
		}

		// q1 shouldn't be empty until the very last iteration
		ut_assert_false(q1.empty());

		// iterate list again to verify removal
		for (auto t : q1) {
			ut_assert_true(t->i != remove_i);
		}

		// size temporarily reduced by 1
		ut_assert_true(q1.size() == 100 - 1);

		// now re-insert the removed node
		const size_t sz1 = q1.size();
		q1.push(removed);
		const size_t sz2 = q1.size();

		// verify the size increase
		ut_assert_true(sz2 > sz1);

		// size restored to 100
		ut_assert_true(q1.size() == 100);
	}

	// queue shouldn't be empty
	ut_assert_false(q1.empty());
	ut_compare("size still 100", q1.size(), 100);

	// delete all elements
	while (!q1.empty()) {
		auto t = q1.pop();
		delete t;
	}

	// verify list has been cleared
	ut_assert_true(q1.empty());
	ut_compare("size 0", q1.size(), 0);

	return true;
}

ut_declare_test_c(test_IntrusiveQueue, IntrusiveQueueTest)
