/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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

#include "uORBTest_UnitTest.hpp"
#include "../uORBCommon.hpp"
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/time.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <lib/cdev/CDev.hpp>

uORBTest::UnitTest &uORBTest::UnitTest::instance()
{
	static uORBTest::UnitTest t;
	return t;
}

int uORBTest::UnitTest::pubsublatency_main()
{
	/* poll on test topic and output latency */
	float latency_integral = 0.0f;

	/* wakeup source(s) */
	px4_pollfd_struct_t fds[3];

	int test_multi_sub = orb_subscribe_multi(ORB_ID(orb_test), 0);
	int test_multi_sub_medium = orb_subscribe_multi(ORB_ID(orb_test_medium), 0);
	int test_multi_sub_large = orb_subscribe_multi(ORB_ID(orb_test_large), 0);

	orb_test_large_s t{};

	/* clear all ready flags */
	orb_copy(ORB_ID(orb_test), test_multi_sub, &t);
	orb_copy(ORB_ID(orb_test_medium), test_multi_sub_medium, &t);
	orb_copy(ORB_ID(orb_test_large), test_multi_sub_large, &t);

	fds[0].fd = test_multi_sub;
	fds[0].events = POLLIN;
	fds[1].fd = test_multi_sub_medium;
	fds[1].events = POLLIN;
	fds[2].fd = test_multi_sub_large;
	fds[2].events = POLLIN;

	const unsigned maxruns = 1000;
	unsigned timingsgroup = 0;
	int current_value = t.val;
	int num_missed = 0;

	// timings has to be on the heap to keep frame size below 2048 bytes
	unsigned *timings = new unsigned[maxruns] {};
	unsigned timing_min = 9999999, timing_max = 0;

	for (unsigned i = 0; i < maxruns; i++) {
		/* wait for up to 500ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 500);

		if (fds[0].revents & POLLIN) {
			orb_copy(ORB_ID(orb_test), test_multi_sub, &t);
			timingsgroup = 0;

		} else if (fds[1].revents & POLLIN) {
			orb_copy(ORB_ID(orb_test_medium), test_multi_sub_medium, &t);
			timingsgroup = 1;

		} else if (fds[2].revents & POLLIN) {
			orb_copy(ORB_ID(orb_test_large), test_multi_sub_large, &t);
			timingsgroup = 2;
		}

		if (pret < 0) {
			PX4_ERR("poll error %d, %d", pret, errno);
			continue;
		}

		num_missed += t.val - current_value - 1;
		current_value = t.val;

		unsigned elt = (unsigned)hrt_elapsed_time_atomic(&t.timestamp);
		latency_integral += elt;
		timings[i] = elt;

		if (elt > timing_max) {
			timing_max = elt;
		}

		if (elt < timing_min) {
			timing_min = elt;
		}
	}

	orb_unsubscribe(test_multi_sub);
	orb_unsubscribe(test_multi_sub_medium);
	orb_unsubscribe(test_multi_sub_large);

	if (pubsubtest_print) {
		char fname[32] {};
		sprintf(fname, PX4_STORAGEDIR"/uorb_timings%u.txt", timingsgroup);
		FILE *f = fopen(fname, "w");

		if (f == nullptr) {
			PX4_ERR("Error opening file!");
			delete[] timings;
			return PX4_ERROR;
		}

		for (unsigned i = 0; i < maxruns; i++) {
			fprintf(f, "%u\n", timings[i]);
		}

		fclose(f);
	}


	float std_dev = 0.f;
	float mean = latency_integral / maxruns;

	for (unsigned i = 0; i < maxruns; i++) {
		float diff = (float)timings[i] - mean;
		std_dev += diff * diff;
	}

	delete[] timings;

	PX4_INFO("mean:    %8.4f us", static_cast<double>(mean));
	PX4_INFO("std dev: %8.4f us", static_cast<double>(sqrtf(std_dev / (maxruns - 1))));
	PX4_INFO("min:     %3i us", timing_min);
	PX4_INFO("max:     %3i us", timing_max);
	PX4_INFO("missed topic updates: %i", num_missed);

	pubsubtest_passed = true;

	if (static_cast<float>(latency_integral / maxruns) > 100.0f) {
		pubsubtest_res = PX4_ERROR;

	} else {
		pubsubtest_res = PX4_OK;
	}

	return pubsubtest_res;
}

int uORBTest::UnitTest::test()
{
	int ret = test_single();

	if (ret != OK) {
		return ret;
	}

	ret = test_multi();

	if (ret != OK) {
		return ret;
	}

	ret = test_multi_reversed();

	if (ret != OK) {
		return ret;
	}

	ret = test_unadvertise();

	if (ret != OK) {
		return ret;
	}

	ret = test_multi2();

	if (ret != OK) {
		return ret;
	}

	ret = test_queue();

	if (ret != OK) {
		return ret;
	}

	return test_queue_poll_notify();
}

int uORBTest::UnitTest::test_unadvertise()
{
	test_note("Testing unadvertise");

	//we still have the advertisements from the previous test_multi calls.
	for (int i = 0; i < 4; ++i) {
		int ret = orb_unadvertise(_pfd[i]);

		if (ret != PX4_OK) {
			return test_fail("orb_unadvertise failed (%i)", ret);
		}
	}

	//try to advertise and see whether we get the right instance
	int instance_test[4] {};
	orb_test_s t{};

	for (int i = 0; i < 4; ++i) {
		_pfd[i] = orb_advertise_multi(ORB_ID(orb_multitest), &t, &instance_test[i]);

		if (instance_test[i] != i) {
			return test_fail("got wrong instance (should be %i, is %i)", i, instance_test[i]);
		}
	}

	for (int i = 0; i < 4; ++i) {
		orb_unadvertise(_pfd[i]);
	}

	return test_note("PASS unadvertise");
}


int uORBTest::UnitTest::info()
{
	return OK;
}

int uORBTest::UnitTest::test_single()
{
	test_note("try single-topic support");

	orb_test_s t{};
	orb_test_s u{};
	int sfd = -1;
	orb_advert_t ptopic{};
	bool updated{false};

	t.val = 0;
	ptopic = orb_advertise(ORB_ID(orb_test), &t);

	if (ptopic == nullptr) {
		return test_fail("advertise failed: %d", errno);
	}

	test_note("publish handle %p", ptopic);
	sfd = orb_subscribe(ORB_ID(orb_test));

	if (sfd < 0) {
		return test_fail("subscribe failed: %d", errno);
	}

	test_note("subscribe fd %d", sfd);
	u.val = 1;

	if (PX4_OK != orb_copy(ORB_ID(orb_test), sfd, &u)) {
		return test_fail("copy(1) failed: %d", errno);
	}

	if (u.val != t.val) {
		return test_fail("copy(1) mismatch: %d expected %d", u.val, t.val);
	}

	if (PX4_OK != orb_check(sfd, &updated)) {
		return test_fail("check(1) failed");
	}

	if (updated) {
		return test_fail("spurious updated flag");
	}

	t.val = 2;
	test_note("try publish");

	if (PX4_OK != orb_publish(ORB_ID(orb_test), ptopic, &t)) {
		return test_fail("publish failed");
	}

	if (PX4_OK != orb_check(sfd, &updated)) {
		return test_fail("check(2) failed");
	}

	if (!updated) {
		return test_fail("missing updated flag");
	}

	if (PX4_OK != orb_copy(ORB_ID(orb_test), sfd, &u)) {
		return test_fail("copy(2) failed: %d", errno);
	}

	if (u.val != t.val) {
		return test_fail("copy(2) mismatch: %d expected %d", u.val, t.val);
	}

	orb_unsubscribe(sfd);

	int ret = orb_unadvertise(ptopic);

	if (ret != PX4_OK) {
		return test_fail("orb_unadvertise failed: %i", ret);
	}

	return test_note("PASS single-topic test");
}

int uORBTest::UnitTest::test_multi()
{
	/* this routine tests the multi-topic support */
	test_note("try multi-topic support");

	orb_test_s t{};
	orb_test_s u{};

	int instance0;
	_pfd[0] = orb_advertise_multi(ORB_ID(orb_multitest), &t, &instance0);

	test_note("advertised");

	int instance1;
	_pfd[1] = orb_advertise_multi(ORB_ID(orb_multitest), &t, &instance1);

	if (instance0 != 0) {
		return test_fail("mult. id0: %d", instance0);
	}

	if (instance1 != 1) {
		return test_fail("mult. id1: %d", instance1);
	}

	t.val = 103;

	if (PX4_OK != orb_publish(ORB_ID(orb_multitest), _pfd[0], &t)) {
		return test_fail("mult. pub0 fail");
	}

	test_note("published");

	t.val = 203;

	if (PX4_OK != orb_publish(ORB_ID(orb_multitest), _pfd[1], &t)) {
		return test_fail("mult. pub1 fail");
	}

	/* subscribe to both topics and ensure valid data is received */
	int sfd0 = orb_subscribe_multi(ORB_ID(orb_multitest), 0);

	if (PX4_OK != orb_copy(ORB_ID(orb_multitest), sfd0, &u)) {
		return test_fail("sub #0 copy failed: %d", errno);
	}

	if (u.val != 103) {
		return test_fail("sub #0 val. mismatch: %d", u.val);
	}

	int sfd1 = orb_subscribe_multi(ORB_ID(orb_multitest), 1);

	if (PX4_OK != orb_copy(ORB_ID(orb_multitest), sfd1, &u)) {
		return test_fail("sub #1 copy failed: %d", errno);
	}

	if (u.val != 203) {
		return test_fail("sub #1 val. mismatch: %d", u.val);
	}

	if (PX4_OK != latency_test<orb_test_s>(ORB_ID(orb_test), false)) {
		return test_fail("latency test failed");
	}

	orb_unsubscribe(sfd0);
	orb_unsubscribe(sfd1);

	return test_note("PASS multi-topic test");
}

int uORBTest::UnitTest::pub_test_multi2_entry(int argc, char *argv[])
{
	uORBTest::UnitTest &t = uORBTest::UnitTest::instance();
	return t.pub_test_multi2_main();
}

int uORBTest::UnitTest::pub_test_multi2_main()
{
	int data_next_idx = 0;
	const int num_instances = 3;
	orb_advert_t orb_pub[num_instances] {};
	orb_test_medium_s data_topic{};

	for (int i = 0; i < num_instances; ++i) {
		orb_advert_t &pub = orb_pub[i];
		int idx = i;
//		PX4_WARN("advertise %i, t=%" PRIu64, i, hrt_absolute_time());
		pub = orb_advertise_multi(ORB_ID(orb_test_medium_multi), &data_topic, &idx);

		if (idx != i) {
			_thread_should_exit = true;
			PX4_ERR("Got wrong instance! should be: %i, but is %i", i, idx);
			return -1;
		}
	}

	px4_usleep(100 * 1000);

	int message_counter = 0;
	int num_messages = 50 * num_instances;

	while (message_counter++ < num_messages) {
		px4_usleep(2); //make sure the timestamps are different
		orb_advert_t &pub = orb_pub[data_next_idx];

		data_topic.timestamp = hrt_absolute_time();
		data_topic.val = data_next_idx;

		orb_publish(ORB_ID(orb_test_medium_multi), pub, &data_topic);
//		PX4_WARN("publishing msg (idx=%i, t=%" PRIu64 ")", data_next_idx, data_topic.time);

		data_next_idx = (data_next_idx + 1) % num_instances;

		if (data_next_idx == 0) {
			px4_usleep(50 * 1000);
		}
	}

	px4_usleep(100 * 1000);
	_thread_should_exit = true;

	for (int i = 0; i < num_instances; ++i) {
		orb_unadvertise(orb_pub[i]);
	}

	return 0;
}

int uORBTest::UnitTest::test_multi2()
{
	test_note("Testing multi-topic 2 test (queue simulation)");
	//test: first subscribe, then advertise

	_thread_should_exit = false;
	const int num_instances = 3;
	int orb_data_fd[num_instances] {-1, -1, -1};
	int orb_data_next = 0;

	for (int i = 0; i < num_instances; ++i) {
//		PX4_WARN("subscribe %i, t=%" PRIu64, i, hrt_absolute_time());
		orb_data_fd[i] = orb_subscribe_multi(ORB_ID(orb_test_medium_multi), i);
	}

	char *const args[1] = { nullptr };
	int pubsub_task = px4_task_spawn_cmd("uorb_test_multi",
					     SCHED_DEFAULT,
					     SCHED_PRIORITY_MAX - 5,
					     3000,
					     (px4_main_t)&uORBTest::UnitTest::pub_test_multi2_entry,
					     args);

	if (pubsub_task < 0) {
		return test_fail("failed launching task");
	}

	hrt_abstime last_time = 0;

	while (!_thread_should_exit) {

		px4_usleep(1000);

		bool updated = false;
		int orb_data_cur_fd = orb_data_fd[orb_data_next];
		orb_check(orb_data_cur_fd, &updated);

		if (updated) {
			orb_test_medium_s msg{};
			orb_copy(ORB_ID(orb_test_medium_multi), orb_data_cur_fd, &msg);

			if (last_time >= msg.timestamp && last_time != 0) {
				return test_fail("Timestamp not increasing! (%" PRIu64 " >= %" PRIu64 ")", last_time, msg.timestamp);
			}

			last_time = msg.timestamp;

			PX4_DEBUG("got message (val=%i, idx=%i, t=%" PRIu64 ")", msg.val, orb_data_next, msg.timestamp);
			orb_data_next = (orb_data_next + 1) % num_instances;
		}
	}

	for (int i = 0; i < num_instances; ++i) {
		orb_unsubscribe(orb_data_fd[i]);
	}

	return test_note("PASS multi-topic 2 test (queue simulation)");
}

int uORBTest::UnitTest::test_multi_reversed()
{
	test_note("try multi-topic support subscribing before publishing");

	/* For these tests 0 and 1 instances are taken from before, therefore continue with 2 and 3. */

	/* Subscribe first and advertise afterwards. */
	int sfd2 = orb_subscribe_multi(ORB_ID(orb_multitest), 2);

	if (sfd2 < 0) {
		return test_fail("sub. id2: ret: %d", sfd2);
	}

	orb_test_s t{};
	orb_test_s u{};

	t.val = 0;

	int instance2;
	_pfd[2] = orb_advertise_multi(ORB_ID(orb_multitest), &t, &instance2);

	int instance3;
	_pfd[3] = orb_advertise_multi(ORB_ID(orb_multitest), &t, &instance3);

	test_note("advertised");

	if (instance2 != 2) {
		return test_fail("mult. id2: %d", instance2);
	}

	if (instance3 != 3) {
		return test_fail("mult. id3: %d", instance3);
	}

	t.val = 204;

	if (PX4_OK != orb_publish(ORB_ID(orb_multitest), _pfd[2], &t)) {
		return test_fail("mult. pub0 fail");
	}


	t.val = 304;

	if (PX4_OK != orb_publish(ORB_ID(orb_multitest), _pfd[3], &t)) {
		return test_fail("mult. pub1 fail");
	}

	test_note("published");

	if (PX4_OK != orb_copy(ORB_ID(orb_multitest), sfd2, &u)) {
		return test_fail("sub #2 copy failed: %d", errno);
	}

	if (u.val != 204) {
		return test_fail("sub #3 val. mismatch: %d", u.val);
	}

	int sfd3 = orb_subscribe_multi(ORB_ID(orb_multitest), 3);

	if (PX4_OK != orb_copy(ORB_ID(orb_multitest), sfd3, &u)) {
		return test_fail("sub #3 copy failed: %d", errno);
	}

	if (u.val != 304) {
		return test_fail("sub #3 val. mismatch: %d", u.val);
	}

	return test_note("PASS multi-topic reversed");
}

int uORBTest::UnitTest::test_queue()
{
	test_note("Testing orb queuing");

	orb_test_medium_s t{};
	orb_test_medium_s u{};
	orb_advert_t ptopic{nullptr};
	bool updated{false};

	int sfd = orb_subscribe(ORB_ID(orb_test_medium_queue));

	if (sfd < 0) {
		return test_fail("subscribe failed: %d", errno);
	}

	const int queue_size = 11;
	t.val = 0;
	ptopic = orb_advertise_queue(ORB_ID(orb_test_medium_queue), &t, queue_size);

	if (ptopic == nullptr) {
		return test_fail("advertise failed: %d", errno);
	}

	orb_check(sfd, &updated);

	if (!updated) {
		return test_fail("update flag not set");
	}

	if (PX4_OK != orb_copy(ORB_ID(orb_test_medium_queue), sfd, &u)) {
		return test_fail("copy(1) failed: %d", errno);
	}

	if (u.val != t.val) {
		return test_fail("copy(1) mismatch: %d expected %d", u.val, t.val);
	}

	orb_check(sfd, &updated);

	if (updated) {
		return test_fail("spurious updated flag");
	}

#define CHECK_UPDATED(element) \
	orb_check(sfd, &updated); \
	if (!updated) { \
		return test_fail("update flag not set, element %i", element); \
	}
#define CHECK_NOT_UPDATED(element) \
	orb_check(sfd, &updated); \
	if (updated) { \
		return test_fail("update flag set, element %i", element); \
	}
#define CHECK_COPY(i_got, i_correct) \
	orb_copy(ORB_ID(orb_test_medium_queue), sfd, &u); \
	if (i_got != i_correct) { \
		return test_fail("got wrong element from the queue (got %i, should be %i)", i_got, i_correct); \
	}

	//no messages in the queue anymore

	test_note("  Testing to write some elements...");

	for (int i = 0; i < queue_size - 2; ++i) {
		t.val = i;
		orb_publish(ORB_ID(orb_test_medium_queue), ptopic, &t);
	}

	for (int i = 0; i < queue_size - 2; ++i) {
		CHECK_UPDATED(i);
		CHECK_COPY(u.val, i);
	}

	CHECK_NOT_UPDATED(queue_size);

	test_note("  Testing overflow...");
	int overflow_by = 3;

	for (int i = 0; i < queue_size + overflow_by; ++i) {
		t.val = i;
		orb_publish(ORB_ID(orb_test_medium_queue), ptopic, &t);
	}

	for (int i = 0; i < queue_size; ++i) {
		CHECK_UPDATED(i);
		CHECK_COPY(u.val, i + overflow_by);
	}

	CHECK_NOT_UPDATED(queue_size);

	test_note("  Testing underflow...");

	for (int i = 0; i < queue_size; ++i) {
		CHECK_NOT_UPDATED(i);
		CHECK_COPY(u.val, queue_size + overflow_by - 1);
	}

	t.val = 943;
	orb_publish(ORB_ID(orb_test_medium_queue), ptopic, &t);
	CHECK_UPDATED(-1);
	CHECK_COPY(u.val, t.val);

#undef CHECK_COPY
#undef CHECK_UPDATED
#undef CHECK_NOT_UPDATED

	orb_unadvertise(ptopic);

	return test_note("PASS orb queuing");
}


int uORBTest::UnitTest::pub_test_queue_entry(int argc, char *argv[])
{
	uORBTest::UnitTest &t = uORBTest::UnitTest::instance();
	return t.pub_test_queue_main();
}

int uORBTest::UnitTest::pub_test_queue_main()
{
	orb_test_medium_s t{};
	orb_advert_t ptopic{nullptr};
	const int queue_size = 50;

	if ((ptopic = orb_advertise_queue(ORB_ID(orb_test_medium_queue_poll), &t, queue_size)) == nullptr) {
		_thread_should_exit = true;
		return test_fail("advertise failed: %d", errno);
	}

	int message_counter = 0;
	int num_messages = 20 * queue_size;

	++t.val;

	while (message_counter < num_messages) {

		//simulate burst
		int burst_counter = 0;

		while (burst_counter++ < queue_size / 2 + 7) { //make interval non-boundary aligned
			orb_publish(ORB_ID(orb_test_medium_queue_poll), ptopic, &t);
			++t.val;
		}

		message_counter += burst_counter;
		px4_usleep(20 * 1000); //give subscriber a chance to catch up
	}

	_num_messages_sent = t.val;
	px4_usleep(100 * 1000);
	_thread_should_exit = true;
	orb_unadvertise(ptopic);

	return 0;
}

int uORBTest::UnitTest::test_queue_poll_notify()
{
	test_note("Testing orb queuing (poll & notify)");

	orb_test_medium_s t{};
	int sfd = -1;

	if ((sfd = orb_subscribe(ORB_ID(orb_test_medium_queue_poll))) < 0) {
		return test_fail("subscribe failed: %d", errno);
	}

	_thread_should_exit = false;

	char *const args[1] = { nullptr };
	int pubsub_task = px4_task_spawn_cmd("uorb_test_queue",
					     SCHED_DEFAULT,
					     SCHED_PRIORITY_MIN + 5,
					     3000,
					     (px4_main_t)&uORBTest::UnitTest::pub_test_queue_entry,
					     args);

	if (pubsub_task < 0) {
		return test_fail("failed launching task");
	}

	int next_expected_val = 0;
	px4_pollfd_struct_t fds[1] {};
	fds[0].fd = sfd;
	fds[0].events = POLLIN;

	while (!_thread_should_exit) {

		int poll_ret = px4_poll(fds, 1, 500);

		if (poll_ret == 0) {
			if (_thread_should_exit) {
				break;
			}

			return test_fail("poll timeout");

		} else if (poll_ret < 0) {
			return test_fail("poll error (%d, %d)", poll_ret, errno);
		}

		if (fds[0].revents & POLLIN) {
			orb_copy(ORB_ID(orb_test_medium_queue_poll), sfd, &t);

			if (next_expected_val != t.val) {
				return test_fail("copy mismatch: %d expected %d", t.val, next_expected_val);
			}

			++next_expected_val;
		}
	}

	if (_num_messages_sent != next_expected_val) {
		return test_fail("number of sent and received messages mismatch (sent: %i, received: %i)",
				 _num_messages_sent, next_expected_val);
	}

	return test_note("PASS orb queuing (poll & notify), got %i messages", next_expected_val);
}


int uORBTest::UnitTest::test_fail(const char *fmt, ...)
{
	va_list ap;

	fprintf(stderr, "uORB FAIL: ");
	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	fprintf(stderr, "\n");
	fflush(stderr);

	return PX4_ERROR;
}

int uORBTest::UnitTest::test_note(const char *fmt, ...)
{
	va_list ap;

	fprintf(stderr, "uORB note: ");
	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	fprintf(stderr, "\n");
	fflush(stderr);
	return OK;
}

int uORBTest::UnitTest::pubsubtest_threadEntry(int argc, char *argv[])
{
	uORBTest::UnitTest &t = uORBTest::UnitTest::instance();
	return t.pubsublatency_main();
}
