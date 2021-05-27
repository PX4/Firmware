/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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

#include "uORBDeviceNode.hpp"

#include "uORBUtils.hpp"
#include "uORBManager.hpp"

#include "SubscriptionCallback.hpp"

#ifdef ORB_COMMUNICATOR
#include "uORBCommunicator.hpp"
#endif /* ORB_COMMUNICATOR */

static uORB::SubscriptionInterval *filp_to_subscription(cdev::file_t *filp) { return static_cast<uORB::SubscriptionInterval *>(filp->f_priv); }

// Determine the data range
static inline bool is_in_range(unsigned left, unsigned value, unsigned right)
{
	if (right > left) {
		return (left <= value) && (value <= right);

	} else {  // Maybe the data overflowed and a wraparound occurred
		return (left <= value) || (value <= right);
	}
}

// round up to nearest power of two
// Such as 0 => 1, 1 => 1, 2 => 2 ,3 => 4, 10 => 16, 60 => 64, 65...255 => 128
// Note: When the input value > 128, the output is always 128
static inline uint8_t round_pow_of_two_8(uint8_t n)
{
	if (n == 0) {
		return 1;
	}

	// Avoid is already a power of 2
	uint8_t value = n - 1;

	// Fill 1
	value |= value >> 1U;
	value |= value >> 2U;
	value |= value >> 4U;

	// Unable to round-up, take the value of round-down
	if (value == UINT8_MAX) {
		value >>= 1U;
	}

	return value + 1;
}

uORB::DeviceNode::DeviceNode(const struct orb_metadata *meta, const uint8_t instance, const char *path,
			     uint8_t queue_size) :
	CDev(path),
	_meta(meta),
	_instance(instance),
	_queue_size(round_pow_of_two_8(queue_size))
{
}

uORB::DeviceNode::~DeviceNode()
{
	delete[] _data;

	CDev::unregister_driver_and_memory();
}

int
uORB::DeviceNode::open(cdev::file_t *filp)
{
	/* is this a publisher? */
	if (filp->f_oflags == PX4_F_WRONLY) {

		lock();
		mark_as_advertised();
		unlock();

		/* now complete the open */
		return CDev::open(filp);
	}

	/* is this a new subscriber? */
	if (filp->f_oflags == PX4_F_RDONLY) {

		/* allocate subscriber data */
		SubscriptionInterval *sd = new SubscriptionInterval(_meta, 0, _instance);

		if (nullptr == sd) {
			return -ENOMEM;
		}

		filp->f_priv = (void *)sd;

		int ret = CDev::open(filp);

		if (ret != PX4_OK) {
			PX4_ERR("CDev::open failed");
			delete sd;
		}

		return ret;
	}

	if (filp->f_oflags == 0) {
		return CDev::open(filp);
	}

	/* can only be pub or sub, not both */
	return -EINVAL;
}

int
uORB::DeviceNode::close(cdev::file_t *filp)
{
	if (filp->f_oflags == PX4_F_RDONLY) { /* subscriber */
		SubscriptionInterval *sd = filp_to_subscription(filp);
		delete sd;
	}

	return CDev::close(filp);
}

bool
uORB::DeviceNode::copy(void *dst, unsigned &generation)
{
	if ((dst != nullptr) && (_data != nullptr)) {
		if (_queue_size == 1) {
			ATOMIC_ENTER;
			memcpy(dst, _data, _meta->o_size);
			generation = _generation.load();
			ATOMIC_LEAVE;
			return true;

		} else {
			ATOMIC_ENTER;
			const unsigned current_generation = _generation.load();

			if (current_generation == generation) {
				/* The subscriber already read the latest message, but nothing new was published yet.
				* Return the previous message
				*/
				--generation;
			}

			// Compatible with normal and overflow conditions
			if (!is_in_range(current_generation - _queue_size, generation, current_generation - 1)) {
				// Reader is too far behind: some messages are lost
				generation = current_generation - _queue_size;
			}

			memcpy(dst, _data + (_meta->o_size * (generation % _queue_size)), _meta->o_size);
			ATOMIC_LEAVE;

			++generation;

			return true;
		}
	}

	return false;
}

ssize_t
uORB::DeviceNode::read(cdev::file_t *filp, char *buffer, size_t buflen)
{
	/* if the caller's buffer is the wrong size, that's an error */
	if (buflen != _meta->o_size) {
		return -EIO;
	}

	return filp_to_subscription(filp)->copy(buffer) ? _meta->o_size : 0;
}

ssize_t
uORB::DeviceNode::write(cdev::file_t *filp, const char *buffer, size_t buflen)
{
	/*
	 * Writes are legal from interrupt context as long as the
	 * object has already been initialised from thread context.
	 *
	 * Writes outside interrupt context will allocate the object
	 * if it has not yet been allocated.
	 *
	 * Note that filp will usually be NULL.
	 */
	if (nullptr == _data) {

#ifdef __PX4_NUTTX

		if (!up_interrupt_context()) {
#endif /* __PX4_NUTTX */

			lock();

			/* re-check size */
			if (nullptr == _data) {
				_data = new uint8_t[_meta->o_size * _queue_size];
			}

			unlock();

#ifdef __PX4_NUTTX
		}

#endif /* __PX4_NUTTX */

		/* failed or could not allocate */
		if (nullptr == _data) {
			return -ENOMEM;
		}
	}

	/* If write size does not match, that is an error */
	if (_meta->o_size != buflen) {
		return -EIO;
	}

	/* Perform an atomic copy. */
	ATOMIC_ENTER;
	/* wrap-around happens after ~49 days, assuming a publisher rate of 1 kHz */
	unsigned generation = _generation.fetch_add(1);

	memcpy(_data + (_meta->o_size * (generation % _queue_size)), buffer, _meta->o_size);

	// callbacks
	for (auto item : _callbacks) {
		item->call();
	}

	/* Mark at least one data has been published */
	_data_valid = true;

	ATOMIC_LEAVE;

	/* notify any poll waiters */
	poll_notify(POLLIN);

	return _meta->o_size;
}

int
uORB::DeviceNode::ioctl(cdev::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case ORBIOCUPDATED: {
			ATOMIC_ENTER;
			*(bool *)arg = filp_to_subscription(filp)->updated();
			ATOMIC_LEAVE;
			return PX4_OK;
		}

	case ORBIOCSETINTERVAL:
		filp_to_subscription(filp)->set_interval_us(arg);
		return PX4_OK;

	case ORBIOCGADVERTISER:
		*(uintptr_t *)arg = (uintptr_t)this;
		return PX4_OK;

	case ORBIOCSETQUEUESIZE: {
			lock();
			int ret = update_queue_size(arg);
			unlock();
			return ret;
		}

	case ORBIOCGETINTERVAL:
		*(unsigned *)arg = filp_to_subscription(filp)->get_interval_us();
		return PX4_OK;

	case ORBIOCISADVERTISED:
		*(unsigned long *)arg = _advertised;

		return PX4_OK;

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}

ssize_t
uORB::DeviceNode::publish(const orb_metadata *meta, orb_advert_t handle, const void *data)
{
	uORB::DeviceNode *devnode = (uORB::DeviceNode *)handle;
	int ret;

	/* check if the device handle is initialized and data is valid */
	if ((devnode == nullptr) || (meta == nullptr) || (data == nullptr)) {
		errno = EFAULT;
		return PX4_ERROR;
	}

	/* check if the orb meta data matches the publication */
	if (devnode->_meta->o_id != meta->o_id) {
		errno = EINVAL;
		return PX4_ERROR;
	}

	/* call the devnode write method with no file pointer */
	ret = devnode->write(nullptr, (const char *)data, meta->o_size);

	if (ret < 0) {
		errno = -ret;
		return PX4_ERROR;
	}

	if (ret != (int)meta->o_size) {
		errno = EIO;
		return PX4_ERROR;
	}

#ifdef ORB_COMMUNICATOR
	/*
	 * if the write is successful, send the data over the Multi-ORB link
	 */
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();

	if (ch != nullptr) {
		if (ch->send_message(meta->o_name, meta->o_size, (uint8_t *)data) != 0) {
			PX4_ERR("Error Sending [%s] topic data over comm_channel", meta->o_name);
			return PX4_ERROR;
		}
	}

#endif /* ORB_COMMUNICATOR */

	return PX4_OK;
}

int uORB::DeviceNode::unadvertise(orb_advert_t handle)
{
	if (handle == nullptr) {
		return -EINVAL;
	}

	uORB::DeviceNode *devnode = (uORB::DeviceNode *)handle;

	/*
	 * We are cheating a bit here. First, with the current implementation, we can only
	 * have multiple publishers for instance 0. In this case the caller will have
	 * instance=nullptr and _published has no effect at all. Thus no unadvertise is
	 * necessary.
	 * In case of multiple instances, we have at most 1 publisher per instance and
	 * we can signal an instance as 'free' by setting _published to false.
	 * We never really free the DeviceNode, for this we would need reference counting
	 * of subscribers and publishers. But we also do not have a leak since future
	 * publishers reuse the same DeviceNode object.
	 */
	devnode->_advertised = false;

	return PX4_OK;
}

#ifdef ORB_COMMUNICATOR
int16_t uORB::DeviceNode::topic_advertised(const orb_metadata *meta)
{
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();

	if (ch != nullptr && meta != nullptr) {
		return ch->topic_advertised(meta->o_name);
	}

	return -1;
}

/*
//TODO: Check if we need this since we only unadvertise when things all shutdown and it doesn't actually remove the device
int16_t uORB::DeviceNode::topic_unadvertised(const orb_metadata *meta)
{
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();
	if (ch != nullptr && meta != nullptr) {
		return ch->topic_unadvertised(meta->o_name);
	}
	return -1;
}
*/
#endif /* ORB_COMMUNICATOR */

px4_pollevent_t
uORB::DeviceNode::poll_state(cdev::file_t *filp)
{
	// If the topic appears updated to the subscriber, say so.
	return filp_to_subscription(filp)->updated() ? POLLIN : 0;
}

void
uORB::DeviceNode::poll_notify_one(px4_pollfd_struct_t *fds, px4_pollevent_t events)
{
	// If the topic looks updated to the subscriber, go ahead and notify them.
	if (filp_to_subscription((cdev::file_t *)fds->priv)->updated()) {
		CDev::poll_notify_one(fds, events);
	}
}

bool
uORB::DeviceNode::print_statistics(int max_topic_length)
{
	if (!_advertised) {
		return false;
	}

	lock();

	const uint8_t instance = get_instance();
	const int8_t sub_count = subscriber_count();
	const uint8_t queue_size = get_queue_size();

	unlock();

	PX4_INFO_RAW("%-*s %2i %4i %2i %4i %s\n", max_topic_length, get_meta()->o_name, (int)instance, (int)sub_count,
		     queue_size, get_meta()->o_size, get_devname());

	return true;
}

void uORB::DeviceNode::add_internal_subscriber()
{
	lock();
	_subscriber_count++;

#ifdef ORB_COMMUNICATOR
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();

	if (ch != nullptr && _subscriber_count > 0) {
		unlock(); //make sure we cannot deadlock if add_subscription calls back into DeviceNode
		ch->add_subscription(_meta->o_name, 1);

	} else
#endif /* ORB_COMMUNICATOR */

	{
		unlock();
	}
}

void uORB::DeviceNode::remove_internal_subscriber()
{
	lock();
	_subscriber_count--;

#ifdef ORB_COMMUNICATOR
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();

	if (ch != nullptr && _subscriber_count == 0) {
		unlock(); //make sure we cannot deadlock if remove_subscription calls back into DeviceNode
		ch->remove_subscription(_meta->o_name);

	} else
#endif /* ORB_COMMUNICATOR */
	{
		unlock();
	}
}

#ifdef ORB_COMMUNICATOR
int16_t uORB::DeviceNode::process_add_subscription(int32_t rateInHz)
{
	// if there is already data in the node, send this out to
	// the remote entity.
	// send the data to the remote entity.
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();

	if (_data != nullptr && ch != nullptr) { // _data will not be null if there is a publisher.
		ch->send_message(_meta->o_name, _meta->o_size, _data);
	}

	return PX4_OK;
}

int16_t uORB::DeviceNode::process_remove_subscription()
{
	return PX4_OK;
}

int16_t uORB::DeviceNode::process_received_message(int32_t length, uint8_t *data)
{
	int16_t ret = -1;

	if (length != (int32_t)(_meta->o_size)) {
		PX4_ERR("Received '%s' with DataLength[%d] != ExpectedLen[%d]", _meta->o_name, (int)length, (int)_meta->o_size);
		return PX4_ERROR;
	}

	/* call the devnode write method with no file pointer */
	ret = write(nullptr, (const char *)data, _meta->o_size);

	if (ret < 0) {
		return PX4_ERROR;
	}

	if (ret != (int)_meta->o_size) {
		errno = EIO;
		return PX4_ERROR;
	}

	return PX4_OK;
}
#endif /* ORB_COMMUNICATOR */

int uORB::DeviceNode::update_queue_size(unsigned int queue_size)
{
	if (_queue_size == queue_size) {
		return PX4_OK;
	}

	//queue size is limited to 255 for the single reason that we use uint8 to store it
	if (_data || _queue_size > queue_size || queue_size > 255) {
		return PX4_ERROR;
	}

	_queue_size = round_pow_of_two_8(queue_size);
	return PX4_OK;
}

unsigned uORB::DeviceNode::get_initial_generation()
{
	ATOMIC_ENTER;

	// If there any previous publications allow the subscriber to read them
	unsigned generation = _generation.load() - (_data_valid ? 1 : 0);

	ATOMIC_LEAVE;

	return generation;
}

bool
uORB::DeviceNode::register_callback(uORB::SubscriptionCallback *callback_sub)
{
	if (callback_sub != nullptr) {
		ATOMIC_ENTER;

		// prevent duplicate registrations
		for (auto existing_callbacks : _callbacks) {
			if (callback_sub == existing_callbacks) {
				ATOMIC_LEAVE;
				return true;
			}
		}

		_callbacks.add(callback_sub);
		ATOMIC_LEAVE;
		return true;
	}

	return false;
}

void
uORB::DeviceNode::unregister_callback(uORB::SubscriptionCallback *callback_sub)
{
	ATOMIC_ENTER;
	_callbacks.remove(callback_sub);
	ATOMIC_LEAVE;
}
