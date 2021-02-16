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

#ifndef _uORBTest_UnitTest_hpp_
#define _uORBTest_UnitTest_hpp_

#include <uORB/uORB.h>
#include <uORB/uORBDeviceMaster.hpp>
#include <uORB/uORBDeviceNode.hpp>
#include <uORB/uORBManager.hpp>
#include <uORB/topics/orb_test.h>
#include <uORB/topics/orb_test_medium.h>
#include <uORB/topics/orb_test_large.h>

#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/tasks.h>

#include <drivers/drv_hrt.h>

#include <errno.h>
#include <unistd.h>

namespace uORBTest
{
class UnitTest;
}

class uORBTest::UnitTest
{
public:

	// Singleton pattern
	static uORBTest::UnitTest &instance();
	~UnitTest() = default;

	int test();
	int latency_test(bool print);
	int info();

	// Disallow copy
	UnitTest(const uORBTest::UnitTest & /*unused*/) = delete;

	// Assist in testing the wrap-around situation
	static void set_generation(uORB::DeviceNode &node, unsigned generation)
	{
		node._generation.store(generation);
	}

private:
	UnitTest() = default;

	static int pubsubtest_threadEntry(int argc, char *argv[]);
	int pubsublatency_main();

	static int pub_test_multi2_entry(int argc, char *argv[]);
	int pub_test_multi2_main();

	volatile bool _thread_should_exit;

	bool pubsubtest_passed{false};
	bool pubsubtest_print{false};
	int pubsubtest_res = OK;

	orb_advert_t _pfd[4] {}; ///< used for test_multi and test_multi_reversed

	int test_single();
	int test_single_unadvertise();

	/* These 3 depend on each other and must be called in this order */
	int test_multi();
	int test_multi_reversed();
	int test_unadvertise();

	int test_multi2();

	int test_wrap_around();

	int test_SubscriptionMulti();

	/* queuing tests */
	int test_queue();
	static int pub_test_queue_entry(int argc, char *argv[]);
	int pub_test_queue_main();
	int test_queue_poll_notify();
	volatile int _num_messages_sent = 0;

	int test_fail(const char *fmt, ...);
	int test_note(const char *fmt, ...);
};

#endif // _uORBTest_UnitTest_hpp_
