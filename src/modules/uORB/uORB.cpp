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

/**
 * @file uORB.cpp
 * A lightweight object broker.
 */

#include "uORB.h"
#include "uORBManager.hpp"
#include "uORBCommon.hpp"

orb_advert_t orb_advertise(const struct orb_metadata *meta, const void *data)
{
	return uORB::Manager::get_instance()->orb_advertise(meta, data);
}

orb_advert_t orb_advertise_queue(const struct orb_metadata *meta, const void *data, unsigned int queue_size)
{
	return uORB::Manager::get_instance()->orb_advertise(meta, data, queue_size);
}

orb_advert_t orb_advertise_multi(const struct orb_metadata *meta, const void *data, int *instance)
{
	return uORB::Manager::get_instance()->orb_advertise_multi(meta, data, instance);
}

orb_advert_t orb_advertise_multi_queue(const struct orb_metadata *meta, const void *data, int *instance,
				       unsigned int queue_size)
{
	return uORB::Manager::get_instance()->orb_advertise_multi(meta, data, instance, queue_size);
}

int orb_unadvertise(orb_advert_t handle)
{
	return uORB::Manager::get_instance()->orb_unadvertise(handle);
}

int  orb_publish(const struct orb_metadata *meta, orb_advert_t handle, const void *data)
{
	return uORB::Manager::get_instance()->orb_publish(meta, handle, data);
}

int  orb_subscribe(const struct orb_metadata *meta)
{
	return uORB::Manager::get_instance()->orb_subscribe(meta);
}

int  orb_subscribe_multi(const struct orb_metadata *meta, unsigned instance)
{
	return uORB::Manager::get_instance()->orb_subscribe_multi(meta, instance);
}

int  orb_unsubscribe(int handle)
{
	return uORB::Manager::get_instance()->orb_unsubscribe(handle);
}

int  orb_copy(const struct orb_metadata *meta, int handle, void *buffer)
{
	return uORB::Manager::get_instance()->orb_copy(meta, handle, buffer);
}

int  orb_check(int handle, bool *updated)
{
	return uORB::Manager::get_instance()->orb_check(handle, updated);
}

int  orb_exists(const struct orb_metadata *meta, int instance)
{
	return uORB::Manager::get_instance()->orb_exists(meta, instance);
}

int  orb_group_count(const struct orb_metadata *meta)
{
	unsigned instance = 0;

	while (uORB::Manager::get_instance()->orb_exists(meta, instance) == OK) {
		++instance;
	};

	return instance;
}

int orb_set_interval(int handle, unsigned interval)
{
	return uORB::Manager::get_instance()->orb_set_interval(handle, interval);
}

int orb_get_interval(int handle, unsigned *interval)
{
	return uORB::Manager::get_instance()->orb_get_interval(handle, interval);
}
