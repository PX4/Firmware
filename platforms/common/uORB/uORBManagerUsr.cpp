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

#include <sys/types.h>
#include <sys/stat.h>
#include <stdarg.h>
#include <fcntl.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>

#include <sys/boardctl.h>

#include "uORBDeviceNode.hpp"
#include "uORBUtils.hpp"
#include "uORBManager.hpp"

uORB::Manager *uORB::Manager::_Instance = nullptr;

bool uORB::Manager::initialize()
{
	if (_Instance == nullptr) {
		_Instance = new uORB::Manager();
	}

	return _Instance != nullptr;
}

bool uORB::Manager::terminate()
{
	if (_Instance != nullptr) {
		delete _Instance;
		_Instance = nullptr;
		return true;
	}

	return false;
}

uORB::Manager::Manager()
{
}

uORB::Manager::~Manager()
{
}

int uORB::Manager::orb_exists(const struct orb_metadata *meta, int instance)
{
	// instance valid range: [0, ORB_MULTI_MAX_INSTANCES)
	if ((instance < 0) || (instance > (ORB_MULTI_MAX_INSTANCES - 1))) {
		return PX4_ERROR;
	}

	orbiocdevexists_t data = {static_cast<ORB_ID>(meta->o_id), static_cast<uint8_t>(instance), true, PX4_ERROR};
	boardctl(ORBIOCDEVEXISTS, reinterpret_cast<unsigned long>(&data));

	return data.ret;
}

orb_advert_t uORB::Manager::orb_advertise_multi(const struct orb_metadata *meta, const void *data, int *instance,
		unsigned int queue_size)
{
	/* open the node as an advertiser */
	int fd = node_open(meta, true, instance);

	if (fd == PX4_ERROR) {
		PX4_ERR("%s advertise failed (%i)", meta->o_name, errno);
		return nullptr;
	}

	/* Set the queue size. This must be done before the first publication; thus it fails if
	 * this is not the first advertiser.
	 */
	int result = px4_ioctl(fd, ORBIOCSETQUEUESIZE, (unsigned long)queue_size);

	if (result < 0 && queue_size > 1) {
		PX4_WARN("orb_advertise_multi: failed to set queue size");
	}

	/* get the advertiser handle and close the node */
	orb_advert_t advertiser;

	result = px4_ioctl(fd, ORBIOCGADVERTISER, (unsigned long)&advertiser);
	px4_close(fd);

	if (result == PX4_ERROR) {
		PX4_WARN("px4_ioctl ORBIOCGADVERTISER failed. fd = %d", fd);
		return nullptr;
	}

	/* the advertiser may perform an initial publish to initialise the object */
	if (data != nullptr) {
		result = orb_publish(meta, advertiser, data);

		if (result == PX4_ERROR) {
			PX4_ERR("orb_publish failed %s", meta->o_name);
			return nullptr;
		}
	}

	return advertiser;
}

int uORB::Manager::orb_unadvertise(orb_advert_t handle)
{
	orbiocdevunadvertise_t data = {handle, PX4_ERROR};
	boardctl(ORBIOCDEVUNADVERTISE, reinterpret_cast<unsigned long>(&data));

	return data.ret;
}

int uORB::Manager::orb_subscribe(const struct orb_metadata *meta)
{
	return node_open(meta, false);
}

int uORB::Manager::orb_subscribe_multi(const struct orb_metadata *meta, unsigned instance)
{
	int inst = instance;
	return node_open(meta, false, &inst);
}

int uORB::Manager::orb_unsubscribe(int fd)
{
	return px4_close(fd);
}

int uORB::Manager::orb_publish(const struct orb_metadata *meta, orb_advert_t handle, const void *data)
{
	orbiocdevpublish_t d = {meta, handle, data, PX4_ERROR};
	boardctl(ORBIOCDEVPUBLISH, reinterpret_cast<unsigned long>(&d));

	return d.ret;
}

int uORB::Manager::orb_copy(const struct orb_metadata *meta, int handle, void *buffer)
{
	int ret;

	ret = px4_read(handle, buffer, meta->o_size);

	if (ret < 0) {
		return PX4_ERROR;
	}

	if (ret != (int)meta->o_size) {
		errno = EIO;
		return PX4_ERROR;
	}

	return PX4_OK;
}

int uORB::Manager::orb_check(int handle, bool *updated)
{
	/* Set to false here so that if `px4_ioctl` fails to false. */
	*updated = false;
	return px4_ioctl(handle, ORBIOCUPDATED, (unsigned long)(uintptr_t)updated);
}

int uORB::Manager::orb_set_interval(int handle, unsigned interval)
{
	return px4_ioctl(handle, ORBIOCSETINTERVAL, interval * 1000);
}

int uORB::Manager::orb_get_interval(int handle, unsigned *interval)
{
	int ret = px4_ioctl(handle, ORBIOCGETINTERVAL, (unsigned long)interval);
	*interval /= 1000;
	return ret;
}

int uORB::Manager::node_open(const struct orb_metadata *meta, bool advertiser, int *instance)
{
	char path[orb_maxpath];
	int fd = -1;
	int ret = PX4_ERROR;

	/*
	 * If meta is null, the object was not defined, i.e. it is not
	 * known to the system.  We can't advertise/subscribe such a thing.
	 */
	if (nullptr == meta) {
		errno = ENOENT;
		return PX4_ERROR;
	}

	/* if we have an instance and are an advertiser, we will generate a new node and set the instance,
	 * so we do not need to open here */
	if (!instance || !advertiser) {
		/*
		 * Generate the path to the node and try to open it.
		 */
		ret = uORB::Utils::node_mkpath(path, meta, instance);

		if (ret != OK) {
			errno = -ret;
			return PX4_ERROR;
		}

		/* open the path as either the advertiser or the subscriber */
		fd = px4_open(path, advertiser ? PX4_F_WRONLY : PX4_F_RDONLY);

	} else {
		*instance = 0;
	}

	/* we may need to advertise the node... */
	if (fd < 0) {
		ret = PX4_ERROR;

		orbiocdevadvertise_t data = {meta, advertiser, instance, PX4_ERROR};
		boardctl(ORBIOCDEVADVERTISE, (unsigned long)&data);
		ret = data.ret;

		/* it's OK if it already exists */
		if ((ret != PX4_OK) && (EEXIST == errno)) {
			ret = PX4_OK;
		}

		if (ret == PX4_OK) {
			/* update the path, as it might have been updated during the node advertise call */
			ret = uORB::Utils::node_mkpath(path, meta, instance);

			/* on success, try to open again */
			if (ret == PX4_OK) {
				fd = px4_open(path, (advertiser) ? PX4_F_WRONLY : PX4_F_RDONLY);

			} else {
				errno = -ret;
				return PX4_ERROR;
			}
		}
	}

	if (fd < 0) {
		errno = EIO;
		return PX4_ERROR;
	}

	/* everything has been OK, we can return the handle now */
	return fd;
}

bool uORB::Manager::orb_device_node_exists(ORB_ID orb_id, uint8_t instance)
{
	orbiocdevexists_t data = {orb_id, instance, false, 0};
	boardctl(ORBIOCDEVEXISTS, reinterpret_cast<unsigned long>(&data));

	return data.ret == PX4_OK ? true : false;
}

void *uORB::Manager::orb_add_internal_subscriber(ORB_ID orb_id, uint8_t instance, unsigned *initial_generation)
{
	orbiocdevaddsubscriber_t data = {orb_id, instance, initial_generation, nullptr};
	boardctl(ORBIOCDEVADDSUBSCRIBER, reinterpret_cast<unsigned long>(&data));

	return data.handle;
}

void uORB::Manager::orb_remove_internal_subscriber(void *node_handle)
{
	boardctl(ORBIOCDEVREMSUBSCRIBER, reinterpret_cast<unsigned long>(node_handle));
}

uint8_t uORB::Manager::orb_get_queue_size(const void *node_handle)
{
	orbiocdevqueuesize_t data = {node_handle, 0};
	boardctl(ORBIOCDEVQUEUESIZE, reinterpret_cast<unsigned long>(&data));

	return data.size;
}

bool uORB::Manager::orb_data_copy(void *node_handle, void *dst, unsigned &generation)
{
	orbiocdevdatacopy_t data = {node_handle, dst, generation, false};
	boardctl(ORBIOCDEVDATACOPY, reinterpret_cast<unsigned long>(&data));
	generation = data.generation;

	return data.ret;
}

bool uORB::Manager::register_callback(void *node_handle, SubscriptionCallback *callback_sub)
{
	orbiocdevregcallback_t data = {node_handle, callback_sub, false};
	boardctl(ORBIOCDEVREGCALLBACK, reinterpret_cast<unsigned long>(&data));

	return data.registered;
}

void uORB::Manager::unregister_callback(void *node_handle, SubscriptionCallback *callback_sub)
{
	orbiocdevunregcallback_t data = {node_handle, callback_sub};
	boardctl(ORBIOCDEVUNREGCALLBACK, reinterpret_cast<unsigned long>(&data));
}

uint8_t uORB::Manager::orb_get_instance(const void *node_handle)
{
	orbiocdevgetinstance_t data = {node_handle, 0};
	boardctl(ORBIOCDEVGETINSTANCE, reinterpret_cast<unsigned long>(&data));

	return data.instance;
}

void uORB::Manager::device_master_cmd(orbiocdevmastercmd_t cmd)
{
	boardctl(ORBIOCDEVMASTERCMD, cmd);
}
