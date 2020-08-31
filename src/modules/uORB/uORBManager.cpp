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
#ifdef ORB_USE_PUBLISHER_RULES
	const char *file_name = PX4_STORAGEDIR"/orb_publisher.rules";
	int ret = readPublisherRulesFromFile(file_name, _publisher_rule);

	if (ret == PX4_OK) {
		_has_publisher_rules = true;
		PX4_INFO("Using orb rules from %s", file_name);

	} else {
		PX4_ERR("Failed to read publisher rules file %s (%s)", file_name, strerror(-ret));
	}

#endif /* ORB_USE_PUBLISHER_RULES */

}

uORB::Manager::~Manager()
{
	delete _device_master;
}

uORB::DeviceMaster *uORB::Manager::get_device_master()
{
	if (!_device_master) {
		_device_master = new DeviceMaster();

		if (_device_master == nullptr) {
			PX4_ERR("Failed to allocate DeviceMaster");
			errno = ENOMEM;
		}
	}

	return _device_master;
}

int uORB::Manager::orb_exists(const struct orb_metadata *meta, int instance)
{
	int ret = PX4_ERROR;

	// instance valid range: [0, ORB_MULTI_MAX_INSTANCES)
	if ((instance < 0) || (instance > (ORB_MULTI_MAX_INSTANCES - 1))) {
		return ret;
	}

	if (get_device_master()) {
		uORB::DeviceNode *node = _device_master->getDeviceNode(meta, instance);

		if (node != nullptr) {
			if (node->is_advertised()) {
				return PX4_OK;
			}
		}
	}

#ifdef ORB_COMMUNICATOR

	/*
	 * Generate the path to the node and try to open it.
	 */
	char path[orb_maxpath];
	int inst = instance;

	ret = uORB::Utils::node_mkpath(path, meta, &inst);

	if (ret != OK) {
		errno = -ret;
		return PX4_ERROR;
	}

	ret = px4_access(path, F_OK);

	if (ret == -1 && meta != nullptr && !_remote_topics.empty()) {
		ret = (_remote_topics.find(meta->o_name) != _remote_topics.end()) ? OK : PX4_ERROR;
	}

	if (ret == 0) {
		// we know the topic exists, but it's not necessarily advertised/published yet (for example
		// if there is only a subscriber)
		// The open() will not lead to memory allocations.
		int fd = px4_open(path, 0);

		if (fd >= 0) {
			unsigned long is_advertised;

			if (px4_ioctl(fd, ORBIOCISADVERTISED, (unsigned long)&is_advertised) == 0) {
				if (!is_advertised) {
					ret = PX4_ERROR;
				}
			}

			px4_close(fd);
		}
	}

#endif /* ORB_COMMUNICATOR */

	return ret;
}

orb_advert_t uORB::Manager::orb_advertise_multi(const struct orb_metadata *meta, const void *data, int *instance,
		unsigned int queue_size)
{
#ifdef ORB_USE_PUBLISHER_RULES

	// check publisher rule
	if (_has_publisher_rules) {
		const char *prog_name = px4_get_taskname();

		if (strcmp(_publisher_rule.module_name, prog_name) == 0) {
			if (_publisher_rule.ignore_other_topics) {
				if (!findTopic(_publisher_rule, meta->o_name)) {
					PX4_DEBUG("not allowing %s to publish topic %s", prog_name, meta->o_name);
					return (orb_advert_t)_Instance;
				}
			}

		} else {
			if (findTopic(_publisher_rule, meta->o_name)) {
				PX4_DEBUG("not allowing %s to publish topic %s", prog_name, meta->o_name);
				return (orb_advert_t)_Instance;
			}
		}
	}

#endif /* ORB_USE_PUBLISHER_RULES */

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

#ifdef ORB_COMMUNICATOR
	// For remote systems call over and inform them
	uORB::DeviceNode::topic_advertised(meta);
#endif /* ORB_COMMUNICATOR */

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
#ifdef ORB_USE_PUBLISHER_RULES

	if (handle == _Instance) {
		return PX4_OK; //pretend success
	}

#endif /* ORB_USE_PUBLISHER_RULES */

	return uORB::DeviceNode::unadvertise(handle);
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
#ifdef ORB_USE_PUBLISHER_RULES

	if (handle == _Instance) {
		return PX4_OK; //pretend success
	}

#endif /* ORB_USE_PUBLISHER_RULES */

	return uORB::DeviceNode::publish(meta, handle, data);
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

		if (get_device_master()) {
			ret = _device_master->advertise(meta, advertiser, instance);
		}

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

#ifdef ORB_COMMUNICATOR
void uORB::Manager::set_uorb_communicator(uORBCommunicator::IChannel *channel)
{
	_comm_channel = channel;

	if (_comm_channel != nullptr) {
		_comm_channel->register_handler(this);
	}
}

uORBCommunicator::IChannel *uORB::Manager::get_uorb_communicator()
{
	return _comm_channel;
}

int16_t uORB::Manager::process_remote_topic(const char *topic_name, bool isAdvertisement)
{
	int16_t rc = 0;

	if (isAdvertisement) {
		_remote_topics.insert(topic_name);

	} else {
		_remote_topics.erase(topic_name);
	}

	return rc;
}

int16_t uORB::Manager::process_add_subscription(const char *messageName, int32_t msgRateInHz)
{
	PX4_DEBUG("entering Manager_process_add_subscription: name: %s", messageName);

	int16_t rc = 0;
	_remote_subscriber_topics.insert(messageName);
	char nodepath[orb_maxpath];
	int ret = uORB::Utils::node_mkpath(nodepath, messageName);
	DeviceMaster *device_master = get_device_master();

	if (ret == OK && device_master) {
		uORB::DeviceNode *node = device_master->getDeviceNode(nodepath);

		if (node == nullptr) {
			PX4_DEBUG("DeviceNode(%s) not created yet", messageName);

		} else {
			// node is present.
			node->process_add_subscription(msgRateInHz);
		}

	} else {
		rc = -1;
	}

	return rc;
}

int16_t uORB::Manager::process_remove_subscription(const char *messageName)
{
	int16_t rc = -1;
	_remote_subscriber_topics.erase(messageName);
	char nodepath[orb_maxpath];
	int ret = uORB::Utils::node_mkpath(nodepath, messageName);
	DeviceMaster *device_master = get_device_master();

	if (ret == OK && device_master) {
		uORB::DeviceNode *node = device_master->getDeviceNode(nodepath);

		// get the node name.
		if (node == nullptr) {
			PX4_DEBUG("[posix-uORB::Manager::process_remove_subscription(%d)]Error No existing subscriber found for message: [%s]",
				  __LINE__, messageName);

		} else {
			// node is present.
			node->process_remove_subscription();
			rc = 0;
		}
	}

	return rc;
}

int16_t uORB::Manager::process_received_message(const char *messageName, int32_t length, uint8_t *data)
{
	int16_t rc = -1;
	char nodepath[orb_maxpath];
	int ret = uORB::Utils::node_mkpath(nodepath, messageName);
	DeviceMaster *device_master = get_device_master();

	if (ret == OK && device_master) {
		uORB::DeviceNode *node = device_master->getDeviceNode(nodepath);

		// get the node name.
		if (node == nullptr) {
			PX4_DEBUG("No existing subscriber found for message: [%s] nodepath:[%s]", messageName, nodepath);

		} else {
			// node is present.
			node->process_received_message(length, data);
			rc = 0;
		}
	}

	return rc;
}

bool uORB::Manager::is_remote_subscriber_present(const char *messageName)
{
#ifdef __PX4_NUTTX
	return _remote_subscriber_topics.find(messageName);
#else
	return (_remote_subscriber_topics.find(messageName) != _remote_subscriber_topics.end());
#endif
}
#endif /* ORB_COMMUNICATOR */

#ifdef ORB_USE_PUBLISHER_RULES

bool uORB::Manager::startsWith(const char *pre, const char *str)
{
	size_t lenpre = strlen(pre),
	       lenstr = strlen(str);
	return lenstr < lenpre ? false : strncmp(pre, str, lenpre) == 0;
}

bool uORB::Manager::findTopic(const PublisherRule &rule, const char *topic_name)
{
	const char **topics_ptr = rule.topics;

	while (*topics_ptr) {
		if (strcmp(*topics_ptr, topic_name) == 0) {
			return true;
		}

		++topics_ptr;
	}

	return false;
}

void uORB::Manager::strTrim(const char **str)
{
	while (**str == ' ' || **str == '\t') { ++(*str); }
}

int uORB::Manager::readPublisherRulesFromFile(const char *file_name, PublisherRule &rule)
{
	FILE *fp;
	static const int line_len = 1024;
	int ret = PX4_OK;
	char *line = new char[line_len];

	if (!line) {
		return -ENOMEM;
	}

	fp = fopen(file_name, "r");

	if (fp == NULL) {
		delete[](line);
		return -errno;
	}

	const char *restrict_topics_str = "restrict_topics:";
	const char *module_str = "module:";
	const char *ignore_others = "ignore_others:";

	rule.ignore_other_topics = false;
	rule.module_name = nullptr;
	rule.topics = nullptr;

	while (fgets(line, line_len, fp) && ret == PX4_OK) {

		if (strlen(line) < 2 || line[0] == '#') {
			continue;
		}

		if (startsWith(restrict_topics_str, line)) {
			//read topics list
			char *start = line + strlen(restrict_topics_str);
			strTrim((const char **)&start);
			char *topics = strdup(start);
			int topic_len = 0, num_topics = 0;

			for (int i = 0; topics[i]; ++i) {
				if (topics[i] == ',' || topics[i] == '\n') {
					if (topic_len > 0) {
						topics[i] = 0;
						++num_topics;
					}

					topic_len = 0;

				} else {
					++topic_len;
				}
			}

			if (num_topics > 0) {
				rule.topics = new const char *[num_topics + 1];
				int topic = 0;
				strTrim((const char **)&topics);
				rule.topics[topic++] = topics;

				while (topic < num_topics) {
					if (*topics == 0) {
						++topics;
						strTrim((const char **)&topics);
						rule.topics[topic++] = topics;

					} else {
						++topics;
					}
				}

				rule.topics[num_topics] = nullptr;
			}

		} else if (startsWith(module_str, line)) {
			//read module name
			char *start = line + strlen(module_str);
			strTrim((const char **)&start);
			int len = strlen(start);

			if (len > 0 && start[len - 1] == '\n') {
				start[len - 1] = 0;
			}

			rule.module_name = strdup(start);

		} else if (startsWith(ignore_others, line)) {
			const char *start = line + strlen(ignore_others);
			strTrim(&start);

			if (startsWith("true", start)) {
				rule.ignore_other_topics = true;
			}

		} else {
			PX4_ERR("orb rules file: wrong format: %s", line);
			ret = -EINVAL;
		}
	}

	if (ret == PX4_OK && (!rule.module_name || !rule.topics)) {
		PX4_ERR("Wrong format in orb publisher rules file");
		ret = -EINVAL;
	}

	delete[](line);
	fclose(fp);
	return ret;
}
#endif /* ORB_USE_PUBLISHER_RULES */
