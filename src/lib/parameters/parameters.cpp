/****************************************************************************
 *
 *   Copyright (c) 2012-2021 PX4 Development Team. All rights reserved.
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
 * @file parameters.cpp
 *
 * Global parameter store.
 *
 * Note that it might make sense to convert this into a driver.  That would
 * offer some interesting options regarding state for e.g. ORB advertisements
 * and background parameter saving.
 */

#define PARAM_IMPLEMENTATION
#include "param.h"
#include "param_translation.h"
#include <parameters/px4_parameters.hpp>
#include "tinybson/tinybson.h"

#include <crc32.h>
#include <float.h>
#include <math.h>

#include <containers/Bitset.hpp>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/atomic_bitset.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/sem.h>
#include <px4_platform_common/shutdown.h>
#include "uthash/utarray.h"

using namespace time_literals;

#include "uORB/uORB.h"
#include "uORB/topics/parameter_update.h"
#include <uORB/topics/actuator_armed.h>
#include <uORB/Subscription.hpp>

#if defined(FLASH_BASED_PARAMS)
#include "flashparams/flashparams.h"
static const char *param_default_file = nullptr; // nullptr means to store to FLASH
#else
inline static int flash_param_save(bool only_unsaved, param_filter_func filter) { return -1; }
inline static int flash_param_load() { return -1; }
inline static int flash_param_import() { return -1; }
static const char *param_default_file = PX4_ROOTFSDIR"/eeprom/parameters";
#endif

static char *param_user_file = nullptr;

#ifdef __PX4_QURT
#define PARAM_OPEN	px4_open
#define PARAM_CLOSE	px4_close
#else
#define PARAM_OPEN	open
#define PARAM_CLOSE	close
#endif

#include <px4_platform_common/workqueue.h>
/* autosaving variables */
static hrt_abstime last_autosave_timestamp = 0;
static struct work_s autosave_work {};
static px4::atomic<bool> autosave_scheduled{false};
static bool autosave_disabled = false;

static constexpr uint16_t param_info_count = sizeof(px4::parameters) / sizeof(param_info_s);
static px4::AtomicBitset<param_info_count> params_active;  // params found
static px4::AtomicBitset<param_info_count> params_changed; // params non-default
static px4::Bitset<param_info_count> params_custom_default; // params with runtime default value

// Storage for modified parameters.
struct param_wbuf_s {
	union param_value_u val;
	param_t             param;
	bool                unsaved;
};

/** flexible array holding modified parameter values */
UT_array *param_values{nullptr};
UT_array *param_custom_default_values{nullptr};

const UT_icd param_icd = {sizeof(param_wbuf_s), nullptr, nullptr, nullptr};

/** parameter update topic handle */
static orb_advert_t param_topic = nullptr;
static unsigned int param_instance = 0;

// the following implements an RW-lock using 2 semaphores (used as mutexes). It gives
// priority to readers, meaning a writer could suffer from starvation, but in our use-case
// we only have short periods of reads and writes are rare.
static px4_sem_t param_sem; ///< this protects against concurrent access to param_values
static int reader_lock_holders = 0;
static px4_sem_t reader_lock_holders_lock; ///< this protects against concurrent access to reader_lock_holders

static perf_counter_t param_export_perf;
static perf_counter_t param_find_perf;
static perf_counter_t param_get_perf;
static perf_counter_t param_set_perf;

static px4_sem_t param_sem_save; ///< this protects against concurrent param saves (file or flash access).
///< we use a separate lock to allow concurrent param reads and saves.
///< a param_set could still be blocked by a param save, because it
///< needs to take the reader lock

/** lock the parameter store for read access */
static void
param_lock_reader()
{
	do {} while (px4_sem_wait(&reader_lock_holders_lock) != 0);

	++reader_lock_holders;

	if (reader_lock_holders == 1) {
		// the first reader takes the lock, the next ones are allowed to just continue
		do {} while (px4_sem_wait(&param_sem) != 0);
	}

	px4_sem_post(&reader_lock_holders_lock);
}

/** lock the parameter store for write access */
static void
param_lock_writer()
{
	do {} while (px4_sem_wait(&param_sem) != 0);
}

/** unlock the parameter store */
static void
param_unlock_reader()
{
	do {} while (px4_sem_wait(&reader_lock_holders_lock) != 0);

	--reader_lock_holders;

	if (reader_lock_holders == 0) {
		// the last reader releases the lock
		px4_sem_post(&param_sem);
	}

	px4_sem_post(&reader_lock_holders_lock);
}

/** unlock the parameter store */
static void
param_unlock_writer()
{
	px4_sem_post(&param_sem);
}

/** assert that the parameter store is locked */
static void
param_assert_locked()
{
	/* XXX */
}

void
param_init()
{
	px4_sem_init(&param_sem, 0, 1);
	px4_sem_init(&param_sem_save, 0, 1);
	px4_sem_init(&reader_lock_holders_lock, 0, 1);

	param_export_perf = perf_alloc(PC_ELAPSED, "param: export");
	param_find_perf = perf_alloc(PC_COUNT, "param: find");
	param_get_perf = perf_alloc(PC_COUNT, "param: get");
	param_set_perf = perf_alloc(PC_ELAPSED, "param: set");
}

/**
 * Test whether a param_t is value.
 *
 * @param param			The parameter handle to test.
 * @return			True if the handle is valid.
 */
static constexpr bool handle_in_range(param_t param) { return (param < param_info_count); }

/**
 * Compare two modified parameter structures to determine ordering.
 *
 * This function is suitable for passing to qsort or bsearch.
 */
static int
param_compare_values(const void *a, const void *b)
{
	struct param_wbuf_s *pa = (struct param_wbuf_s *)a;
	struct param_wbuf_s *pb = (struct param_wbuf_s *)b;

	if (pa->param < pb->param) {
		return -1;
	}

	if (pa->param > pb->param) {
		return 1;
	}

	return 0;
}

/**
 * Locate the modified parameter structure for a parameter, if it exists.
 *
 * @param param			The parameter being searched.
 * @return			The structure holding the modified value, or
 *				nullptr if the parameter has not been modified.
 */
static param_wbuf_s *
param_find_changed(param_t param)
{
	param_assert_locked();

	if (params_changed[param] && (param_values != nullptr)) {
		param_wbuf_s key{};
		key.param = param;
		return (param_wbuf_s *)utarray_find(param_values, &key, param_compare_values);
	}

	return nullptr;
}

void
param_notify_changes()
{
	parameter_update_s pup{};
	pup.instance = param_instance++;
	pup.get_count = perf_event_count(param_get_perf);
	pup.set_count = perf_event_count(param_set_perf);
	pup.find_count = perf_event_count(param_find_perf);
	pup.export_count = perf_event_count(param_export_perf);
	pup.active = params_active.count();
	pup.changed = params_changed.count();
	pup.custom_default = params_custom_default.count();
	pup.timestamp = hrt_absolute_time();

	if (param_topic == nullptr) {
		param_topic = orb_advertise(ORB_ID(parameter_update), &pup);

	} else {
		orb_publish(ORB_ID(parameter_update), param_topic, &pup);
	}
}

static param_t param_find_internal(const char *name, bool notification)
{
	perf_count(param_find_perf);

	param_t middle;
	param_t front = 0;
	param_t last = param_info_count;

	/* perform a binary search of the known parameters */

	while (front <= last) {
		middle = front + (last - front) / 2;
		int ret = strcmp(name, param_name(middle));

		if (ret == 0) {
			if (notification) {
				param_set_used(middle);
			}

			perf_end(param_find_perf);
			return middle;

		} else if (middle == front) {
			/* An end point has been hit, but there has been no match */
			break;

		} else if (ret < 0) {
			last = middle;

		} else {
			front = middle;
		}
	}

	/* not found */
	return PARAM_INVALID;
}

param_t param_find(const char *name)
{
	return param_find_internal(name, true);
}

param_t param_find_no_notification(const char *name)
{
	return param_find_internal(name, false);
}

unsigned param_count()
{
	return param_info_count;
}

unsigned param_count_used()
{
	return params_active.count();
}

param_t param_for_index(unsigned index)
{
	if (index < param_info_count) {
		return (param_t)index;
	}

	return PARAM_INVALID;
}

param_t param_for_used_index(unsigned index)
{
	// walk all params and count used params
	if (index < param_info_count) {
		unsigned used_count = 0;

		for (int i = 0; i < params_active.size(); i++) {
			if (params_active[i]) {
				// we found the right used count,
				//  return the param value
				if (index == used_count) {
					return static_cast<param_t>(i);
				}

				used_count++;
			}
		}
	}

	return PARAM_INVALID;
}

int param_get_index(param_t param)
{
	if (handle_in_range(param)) {
		return (unsigned)param;
	}

	return -1;
}

int param_get_used_index(param_t param)
{
	/* this tests for out of bounds and does a constant time lookup */
	if (!param_used(param)) {
		return -1;
	}

	/* walk all params and count, now knowing that it has a valid index */
	int used_count = 0;

	for (int i = 0; i < params_active.size(); i++) {
		if (params_active[i]) {

			if (param == i) {
				return used_count;
			}

			used_count++;
		}
	}

	return -1;
}

const char *param_name(param_t param)
{
	return handle_in_range(param) ? px4::parameters[param].name : nullptr;
}

param_type_t param_type(param_t param)
{
	return handle_in_range(param) ? px4::parameters_type[param] : PARAM_TYPE_UNKNOWN;
}

bool param_is_volatile(param_t param)
{
	if (handle_in_range(param)) {
		for (const auto &p : px4::parameters_volatile) {
			if (static_cast<px4::params>(param) == p) {
				return true;
			}
		}
	}

	return false;
}

bool
param_value_unsaved(param_t param)
{
	struct param_wbuf_s *s;
	param_lock_reader();
	s = param_find_changed(param);
	bool ret = s && s->unsaved;
	param_unlock_reader();
	return ret;
}

size_t param_size(param_t param)
{
	if (handle_in_range(param)) {
		switch (param_type(param)) {
		case PARAM_TYPE_INT32:
		case PARAM_TYPE_FLOAT:
			return 4;

		default:
			return 0;
		}
	}

	return 0;
}

/**
 * Obtain a pointer to the storage allocated for a parameter.
 *
 * @param param			The parameter whose storage is sought.
 * @return			A pointer to the parameter value, or nullptr
 *				if the parameter does not exist.
 */
static const void *
param_get_value_ptr(param_t param)
{
	param_assert_locked();

	if (handle_in_range(param)) {
		/* work out whether we're fetching the default or a written value */
		struct param_wbuf_s *s = param_find_changed(param);

		if (s != nullptr) {
			return &s->val;

		} else {
			if (params_custom_default[param] && param_custom_default_values) {
				// get default from custom default storage
				param_wbuf_s key{};
				key.param = param;
				param_wbuf_s *pbuf = (param_wbuf_s *)utarray_find(param_custom_default_values, &key, param_compare_values);

				if (pbuf != nullptr) {
					return &pbuf->val;
				}
			}

			// otherwise return static default value
			switch (param_type(param)) {
			case PARAM_TYPE_INT32:
				return &px4::parameters[param].val.i;

			case PARAM_TYPE_FLOAT:
				return &px4::parameters[param].val.f;
			}
		}
	}

	return nullptr;
}

int
param_get(param_t param, void *val)
{
	perf_count(param_get_perf);

	if (!handle_in_range(param)) {
		PX4_ERR("get: param %d invalid", param);
		return PX4_ERROR;
	}

	if (!params_active[param]) {
		PX4_DEBUG("get: param %d (%s) not active", param, param_name(param));
	}

	int result = PX4_ERROR;

	if (val) {
		param_lock_reader();

		const void *v = param_get_value_ptr(param);

		if (v) {
			memcpy(val, v, param_size(param));
			result = PX4_OK;
		}

		param_unlock_reader();
	}

	return result;
}

int
param_get_default_value_internal(param_t param, void *default_val)
{
	if (!handle_in_range(param)) {
		PX4_ERR("get default value: param %d invalid", param);
		return PX4_ERROR;
	}

	if (default_val) {
		if (params_custom_default[param] && param_custom_default_values) {
			// get default from custom default storage
			param_wbuf_s key{};
			key.param = param;
			param_wbuf_s *pbuf = (param_wbuf_s *)utarray_find(param_custom_default_values, &key, param_compare_values);

			if (pbuf != nullptr) {
				memcpy(default_val, &pbuf->val, param_size(param));
				return PX4_OK;
			}
		}

		// otherwise return static default value
		switch (param_type(param)) {
		case PARAM_TYPE_INT32:
			memcpy(default_val, &px4::parameters[param].val.i, param_size(param));
			return PX4_OK;

		case PARAM_TYPE_FLOAT:
			memcpy(default_val, &px4::parameters[param].val.f, param_size(param));
			return PX4_OK;
		}
	}

	return PX4_ERROR;
}

int
param_get_default_value(param_t param, void *default_val)
{
	param_lock_reader();
	int ret = param_get_default_value_internal(param, default_val);
	param_unlock_reader();
	return ret;
}

bool param_value_is_default(param_t param)
{
	// the param_values dynamic array might carry things that have been set
	// back to default, so we don't rely on the params_changed bitset here
	if (handle_in_range(param)) {
		switch (param_type(param)) {
		case PARAM_TYPE_INT32: {
				param_lock_reader();
				int32_t default_value = 0;

				if (param_get_default_value_internal(param, &default_value) == PX4_OK) {
					const void *v = param_get_value_ptr(param);

					if (v) {
						int32_t current_value;
						memcpy(&current_value, v, param_size(param));
						param_unlock_reader();
						return (current_value == default_value);
					}
				}

				param_unlock_reader();
			}
			break;

		case PARAM_TYPE_FLOAT: {
				param_lock_reader();
				float default_value = 0;

				if (param_get_default_value_internal(param, &default_value) == PX4_OK) {
					const void *v = param_get_value_ptr(param);

					if (v) {
						float current_value;
						memcpy(&current_value, v, param_size(param));
						param_unlock_reader();
						return (fabsf(current_value - default_value) <= FLT_EPSILON);
					}
				}

				param_unlock_reader();
			}
			break;
		}
	}

	return false;
}

int
param_get_system_default_value(param_t param, void *default_val)
{
	if (!handle_in_range(param)) {
		return PX4_ERROR;
	}

	int ret = PX4_OK;

	switch (param_type(param)) {
	case PARAM_TYPE_INT32:
		memcpy(default_val, &px4::parameters[param].val.i, param_size(param));
		break;

	case PARAM_TYPE_FLOAT:
		memcpy(default_val, &px4::parameters[param].val.f, param_size(param));
		break;

	default:
		ret = PX4_ERROR;
		break;
	}

	return ret;
}

/**
 * worker callback method to save the parameters
 * @param arg unused
 */
static void
autosave_worker(void *arg)
{
	bool disabled = false;

	if (!param_get_default_file()) {
		// In case we save to FLASH, defer param writes until disarmed,
		// as writing to FLASH can stall the entire CPU (in rare cases around 300ms on STM32F7)
		uORB::SubscriptionData<actuator_armed_s> armed_sub{ORB_ID(actuator_armed)};

		if (armed_sub.get().armed) {
			work_queue(LPWORK, &autosave_work, (worker_t)&autosave_worker, nullptr, USEC2TICK(1_s));
			return;
		}
	}

	param_lock_writer();
	last_autosave_timestamp = hrt_absolute_time();
	autosave_scheduled.store(false);
	disabled = autosave_disabled;
	param_unlock_writer();

	if (disabled) {
		return;
	}

	PX4_DEBUG("Autosaving params");
	int ret = param_save_default();

	if (ret != 0) {
		PX4_ERR("param auto save failed (%i)", ret);
	}
}

/**
 * Automatically save the parameters after a timeout and limited rate.
 *
 * This needs to be called with the writer lock held (it's not necessary that it's the writer lock, but it
 * needs to be the same lock as autosave_worker() and param_control_autosave() use).
 */
static void
param_autosave()
{
	if (autosave_scheduled.load() || autosave_disabled) {
		return;
	}

	// wait at least 300ms before saving, because:
	// - tasks often call param_set() for multiple params, so this avoids unnecessary save calls
	// - the logger stores changed params. He gets notified on a param change via uORB and then
	//   looks at all unsaved params.
	hrt_abstime delay = 300_ms;

	static constexpr const hrt_abstime rate_limit = 2_s; // rate-limit saving to 2 seconds
	const hrt_abstime last_save_elapsed = hrt_elapsed_time(&last_autosave_timestamp);

	if (last_save_elapsed < rate_limit && rate_limit > last_save_elapsed + delay) {
		delay = rate_limit - last_save_elapsed;
	}

	autosave_scheduled.store(true);
	work_queue(LPWORK, &autosave_work, (worker_t)&autosave_worker, nullptr, USEC2TICK(delay));
}

void
param_control_autosave(bool enable)
{
	param_lock_writer();

	if (!enable && autosave_scheduled.load()) {
		work_cancel(LPWORK, &autosave_work);
		autosave_scheduled.store(false);
	}

	autosave_disabled = !enable;
	param_unlock_writer();
}

static int
param_set_internal(param_t param, const void *val, bool mark_saved, bool notify_changes)
{
	if (!handle_in_range(param)) {
		PX4_ERR("set invalid param %d", param);
		return PX4_ERROR;
	}

	if (val == nullptr) {
		PX4_ERR("set invalid value");
		return PX4_ERROR;
	}

	int result = -1;
	bool param_changed = false;

	param_lock_writer();
	perf_begin(param_set_perf);

	// create the parameter store if it doesn't exist
	if (param_values == nullptr) {
		utarray_new(param_values, &param_icd);

		// mark all parameters unchanged (default)
		for (int i = 0; i < params_changed.size(); i++) {
			params_changed.set(i, false);
		}
	}

	if (param_values == nullptr) {
		PX4_ERR("failed to allocate modified values array");
		goto out;

	} else {
		param_wbuf_s *s = param_find_changed(param);

		if (s == nullptr) {
			/* construct a new parameter */
			param_wbuf_s buf{};
			buf.param = param;

			param_changed = true;

			/* add it to the array and sort */
			utarray_push_back(param_values, &buf);
			utarray_sort(param_values, param_compare_values);
			params_changed.set(param, true);

			/* find it after sorting */
			s = param_find_changed(param);
		}

		if (s != nullptr) {
			/* update the changed value */
			switch (param_type(param)) {
			case PARAM_TYPE_INT32:
				param_changed = param_changed || s->val.i != *(int32_t *)val;
				s->val.i = *(int32_t *)val;
				s->unsaved = !mark_saved;
				params_changed.set(param, true);
				result = PX4_OK;
				break;

			case PARAM_TYPE_FLOAT:
				param_changed = param_changed || fabsf(s->val.f - * (float *)val) > FLT_EPSILON;
				s->val.f = *(float *)val;
				s->unsaved = !mark_saved;
				params_changed.set(param, true);
				result = PX4_OK;
				break;

			default:
				break;
			}
		}

		if ((result == PX4_OK) && !mark_saved) { // this is false when importing parameters
			param_autosave();
		}
	}

out:
	perf_end(param_set_perf);
	param_unlock_writer();

	/*
	 * If we set something, now that we have unlocked, go ahead and advertise that
	 * a thing has been set.
	 */
	if ((result == PX4_OK) && param_changed && notify_changes) {
		param_notify_changes();
	}

	return result;
}

#if defined(FLASH_BASED_PARAMS)
int param_set_external(param_t param, const void *val, bool mark_saved, bool notify_changes)
{
	return param_set_internal(param, val, mark_saved, notify_changes);
}

const void *param_get_value_ptr_external(param_t param)
{
	return param_get_value_ptr(param);
}
#endif

int param_set(param_t param, const void *val)
{
	return param_set_internal(param, val, false, true);
}

int param_set_no_notification(param_t param, const void *val)
{
	return param_set_internal(param, val, false, false);
}

bool param_used(param_t param)
{
	if (handle_in_range(param)) {
		return params_active[param];
	}

	return false;
}

void param_set_used(param_t param)
{
	if (handle_in_range(param)) {
		params_active.set(param, true);
	}
}

int param_set_default_value(param_t param, const void *val)
{
	if (!handle_in_range(param)) {
		PX4_ERR("set default value invalid param %d", param);
		return PX4_ERROR;
	}

	if (val == nullptr) {
		PX4_ERR("set default value invalid value");
		return PX4_ERROR;
	}

	int result = PX4_ERROR;

	param_lock_writer();

	if (param_custom_default_values == nullptr) {
		utarray_new(param_custom_default_values, &param_icd);

		// mark all parameters unchanged (default)
		for (int i = 0; i < params_custom_default.size(); i++) {
			params_custom_default.set(i, false);
		}
	}

	if (param_custom_default_values == nullptr) {
		PX4_ERR("failed to allocate custom default values array");
		param_unlock_writer();
		return PX4_ERROR;
	}

	// check if param being set to default value
	bool setting_to_static_default = false;

	switch (param_type(param)) {
	case PARAM_TYPE_INT32:
		setting_to_static_default = (px4::parameters[param].val.i == *(int32_t *)val);
		break;

	case PARAM_TYPE_FLOAT:
		setting_to_static_default = (fabsf(px4::parameters[param].val.f - * (float *)val) < FLT_EPSILON);
		break;
	}

	// find if custom default value is already set
	param_wbuf_s *s = nullptr;

	{
		param_wbuf_s key{};
		key.param = param;
		s = (param_wbuf_s *)utarray_find(param_custom_default_values, &key, param_compare_values);
	}

	if (setting_to_static_default) {
		if (s != nullptr) {
			// param in memory and set to non-default value, clear
			int pos = utarray_eltidx(param_custom_default_values, s);
			utarray_erase(param_custom_default_values, pos, 1);
		}

		// do nothing if param not already set and being set to default
		params_custom_default.set(param, false);
		result = PX4_OK;

	} else {
		if (s == nullptr) {
			// construct a new parameter default value
			param_wbuf_s buf{};
			buf.param = param;

			// add it to the array and sort
			utarray_push_back(param_custom_default_values, &buf);
			utarray_sort(param_custom_default_values, param_compare_values);

			// find it after sorting
			s = (param_wbuf_s *)utarray_find(param_custom_default_values, &buf, param_compare_values);
		}

		if (s != nullptr) {
			// update the default value
			switch (param_type(param)) {
			case PARAM_TYPE_INT32:
				s->val.i = *(int32_t *)val;
				params_custom_default.set(param, true);
				result = PX4_OK;
				break;

			case PARAM_TYPE_FLOAT:
				s->val.f = *(float *)val;
				params_custom_default.set(param, true);
				result = PX4_OK;
				break;

			default:
				break;
			}
		}
	}

	param_unlock_writer();

	if ((result == PX4_OK) && param_used(param)) {
		// send notification if param is already in use
		param_notify_changes();
	}

	return result;
}

static int param_reset_internal(param_t param, bool notify = true)
{
	param_wbuf_s *s = nullptr;
	bool param_found = false;

	param_lock_writer();

	if (handle_in_range(param)) {
		/* look for a saved value */
		s = param_find_changed(param);

		/* if we found one, erase it */
		if (s != nullptr) {
			int pos = utarray_eltidx(param_values, s);
			utarray_erase(param_values, pos, 1);
		}

		params_changed.set(param, false);

		param_found = true;
	}

	param_autosave();

	param_unlock_writer();

	if (s != nullptr && notify) {
		param_notify_changes();
	}

	return (!param_found);
}

int param_reset(param_t param) { return param_reset_internal(param, true); }
int param_reset_no_notification(param_t param) { return param_reset_internal(param, false); }

static void
param_reset_all_internal(bool auto_save)
{
	param_lock_writer();

	if (param_values != nullptr) {
		utarray_free(param_values);

		for (int i = 0; i < params_changed.size(); i++) {
			params_changed.set(i, false);
		}
	}

	/* mark as reset / deleted */
	param_values = nullptr;

	if (auto_save) {
		param_autosave();
	}

	param_unlock_writer();

	param_notify_changes();
}

void
param_reset_all()
{
	param_reset_all_internal(true);
}

void
param_reset_excludes(const char *excludes[], int num_excludes)
{
	for (param_t param = 0; handle_in_range(param); param++) {
		const char *name = param_name(param);
		bool exclude = false;

		for (int index = 0; index < num_excludes; index ++) {
			int len = strlen(excludes[index]);

			if ((excludes[index][len - 1] == '*'
			     && strncmp(name, excludes[index], len - 1) == 0)
			    || strcmp(name, excludes[index]) == 0) {

				exclude = true;
				break;
			}
		}

		if (!exclude) {
			param_reset(param);
		}
	}
}

void
param_reset_specific(const char *resets[], int num_resets)
{
	for (param_t param = 0; handle_in_range(param); param++) {
		const char *name = param_name(param);
		bool reset = false;

		for (int index = 0; index < num_resets; index++) {
			int len = strlen(resets[index]);

			if ((resets[index][len - 1] == '*'
			     && strncmp(name, resets[index], len - 1) == 0)
			    || strcmp(name, resets[index]) == 0) {

				reset = true;
				break;
			}
		}

		if (reset) {
			param_reset(param);
		}
	}
}

int
param_set_default_file(const char *filename)
{
#ifdef FLASH_BASED_PARAMS
	// the default for flash-based params is always the FLASH
	(void)filename;
#else

	if (param_user_file != nullptr) {
		// we assume this is not in use by some other thread
		free(param_user_file);
		param_user_file = nullptr;
	}

	if (filename) {
		param_user_file = strdup(filename);
	}

#endif /* FLASH_BASED_PARAMS */

	return 0;
}

const char *
param_get_default_file()
{
	return (param_user_file != nullptr) ? param_user_file : param_default_file;
}

int param_save_default()
{
	int res = PX4_ERROR;

	const char *filename = param_get_default_file();

	if (!filename) {
		param_lock_writer();
		perf_begin(param_export_perf);
		res = flash_param_save(false, nullptr);
		perf_end(param_export_perf);
		param_unlock_writer();
		return res;
	}

	int attempts = 5;

	while (res != OK && attempts > 0) {
		// write parameters to file
		int fd = PARAM_OPEN(filename, O_WRONLY | O_CREAT, PX4_O_MODE_666);

		if (fd > -1) {
			res = param_export(fd, false, nullptr);
			PARAM_CLOSE(fd);

			if (res != PX4_OK) {
				PX4_ERR("param_export failed, retrying %d", attempts);
			}

		} else {
			PX4_ERR("failed to open param file %s, retrying %d", filename, attempts);
		}

		attempts--;
	}

	if (res != OK) {
		PX4_ERR("failed to write parameters to file: %s", filename);
	}

	return res;
}

/**
 * @return 0 on success, 1 if all params have not yet been stored, -1 if device open failed, -2 if writing parameters failed
 */
int
param_load_default()
{
	int res = 0;
	const char *filename = param_get_default_file();

	if (!filename) {
		return flash_param_load();
	}

	int fd_load = PARAM_OPEN(filename, O_RDONLY);

	if (fd_load < 0) {
		/* no parameter file is OK, otherwise this is an error */
		if (errno != ENOENT) {
			PX4_ERR("open '%s' for reading failed", filename);
			return -1;
		}

		return 1;
	}

	int result = param_load(fd_load);
	PARAM_CLOSE(fd_load);

	if (result != 0) {
		PX4_ERR("error reading parameters from '%s'", filename);
		return -2;
	}

	return res;
}

int
param_export(int fd, bool only_unsaved, param_filter_func filter)
{
	int	result = -1;
	perf_begin(param_export_perf);

	if (fd < 0) {
		param_lock_writer();
		// flash_param_save() will take the shutdown lock
		result = flash_param_save(only_unsaved, filter);
		param_unlock_writer();
		perf_end(param_export_perf);
		return result;
	}

	param_wbuf_s *s = nullptr;
	struct bson_encoder_s encoder;

	int shutdown_lock_ret = px4_shutdown_lock();

	if (shutdown_lock_ret) {
		PX4_ERR("px4_shutdown_lock() failed (%i)", shutdown_lock_ret);
	}

	// take the file lock
	do {} while (px4_sem_wait(&param_sem_save) != 0);

	param_lock_reader();

	uint8_t bson_buffer[256];
	bson_encoder_init_buf_file(&encoder, fd, &bson_buffer, sizeof(bson_buffer));

	/* no modified parameters -> we are done */
	if (param_values == nullptr) {
		result = 0;
		goto out;
	}

	while ((s = (struct param_wbuf_s *)utarray_next(param_values, s)) != nullptr) {
		/*
		 * If we are only saving values changed since last save, and this
		 * one hasn't, then skip it
		 */
		if (only_unsaved && !s->unsaved) {
			continue;
		}

		if (filter && !filter(s->param)) {
			continue;
		}

		// don't export default values
		switch (param_type(s->param)) {
		case PARAM_TYPE_INT32: {
				int32_t default_value = 0;
				param_get_default_value_internal(s->param, &default_value);

				if (s->val.i == default_value) {
					PX4_DEBUG("skipping %s %d export", param_name(s->param), default_value);
					continue;
				}
			}
			break;

		case PARAM_TYPE_FLOAT: {
				float default_value = 0;
				param_get_default_value_internal(s->param, &default_value);

				if (fabsf(s->val.f - default_value) <= FLT_EPSILON) {
					PX4_DEBUG("skipping %s %.3f export", param_name(s->param), (double)default_value);
					continue;
				}
			}
			break;
		}

		s->unsaved = false;

		const char *name = param_name(s->param);
		const size_t size = param_size(s->param);

		/* append the appropriate BSON type object */
		switch (param_type(s->param)) {
		case PARAM_TYPE_INT32: {
				const int32_t i = s->val.i;
				PX4_DEBUG("exporting: %s (%d) size: %lu val: %d", name, s->param, (long unsigned int)size, i);

				if (bson_encoder_append_int(&encoder, name, i) != 0) {
					PX4_ERR("BSON append failed for '%s'", name);
					goto out;
				}
			}
			break;

		case PARAM_TYPE_FLOAT: {
				const double f = (double)s->val.f;
				PX4_DEBUG("exporting: %s (%d) size: %lu val: %.3f", name, s->param, (long unsigned int)size, (double)f);

				if (bson_encoder_append_double(&encoder, name, f) != 0) {
					PX4_ERR("BSON append failed for '%s'", name);
					goto out;
				}
			}
			break;

		default:
			PX4_ERR("%s unrecognized parameter type %d, skipping export", name, param_type(s->param));
		}
	}

	result = 0;

out:

	if (result == 0) {
		if (bson_encoder_fini(&encoder) != PX4_OK) {
			PX4_ERR("BSON encoder finialize failed");
			result = -1;
		}
	}

	param_unlock_reader();

	px4_sem_post(&param_sem_save);

	if (shutdown_lock_ret == 0) {
		px4_shutdown_unlock();
	}

	perf_end(param_export_perf);

	return result;
}

struct param_import_state {
	bool mark_saved;
};

static int
param_import_callback(bson_decoder_t decoder, void *priv, bson_node_t node)
{
	float f = 0.0f;
	int32_t i = 0;
	void *v = nullptr;
	int result = -1;
	param_import_state *state = (param_import_state *)priv;

	/*
	 * EOO means the end of the parameter object. (Currently not supporting
	 * nested BSON objects).
	 */
	if (node->type == BSON_EOO) {
		PX4_DEBUG("end of parameters");
		return 0;
	}

	param_modify_on_import(node);

	/*
	 * Find the parameter this node represents.  If we don't know it,
	 * ignore the node.
	 */
	param_t param = param_find_no_notification(node->name);

	if (param == PARAM_INVALID) {
		PX4_ERR("ignoring unrecognised parameter '%s'", node->name);
		return 1;
	}

	/*
	 * Handle setting the parameter from the node
	 */

	switch (node->type) {
	case BSON_INT32: {
			if (param_type(param) != PARAM_TYPE_INT32) {
				PX4_WARN("unexpected type for %s", node->name);
				result = 1; // just skip this entry
				goto out;
			}

			i = node->i;
			v = &i;

			PX4_DEBUG("Imported %s with value %d", param_name(param), i);
		}
		break;

	case BSON_DOUBLE: {
			if (param_type(param) != PARAM_TYPE_FLOAT) {
				PX4_WARN("unexpected type for %s", node->name);
				result = 1; // just skip this entry
				goto out;
			}

			f = node->d;
			v = &f;

			PX4_DEBUG("Imported %s with value %f", param_name(param), (double)f);
		}
		break;

	default:
		PX4_DEBUG("unrecognised node type");
		goto out;
	}

	if (param_set_internal(param, v, state->mark_saved, true)) {
		PX4_DEBUG("error setting value for '%s'", node->name);
		goto out;
	}

	/* don't return zero, that means EOF */
	result = 1;

out:
	return result;
}

static int
param_import_internal(int fd, bool mark_saved)
{
	bson_decoder_s decoder;
	param_import_state state;
	int result = -1;

	if (bson_decoder_init_file(&decoder, fd, param_import_callback, &state)) {
		PX4_ERR("decoder init failed");
		return PX4_ERROR;
	}

	state.mark_saved = mark_saved;

	do {
		result = bson_decoder_next(&decoder);

	} while (result > 0);

	PX4_INFO("BSON document size %d bytes, decoded %d bytes", decoder.total_document_size, decoder.total_decoded_size);

	return result;
}

int
param_import(int fd, bool mark_saved)
{
	if (fd < 0) {
		return flash_param_import();
	}

	return param_import_internal(fd, mark_saved);
}

int
param_load(int fd)
{
	if (fd < 0) {
		return flash_param_load();
	}

	param_reset_all_internal(false);
	return param_import_internal(fd, true);
}

void
param_foreach(void (*func)(void *arg, param_t param), void *arg, bool only_changed, bool only_used)
{
	param_t	param;

	for (param = 0; handle_in_range(param); param++) {

		/* if requested, skip unchanged values */
		if (only_changed && (param_find_changed(param) == nullptr)) {
			continue;
		}

		if (only_used && !param_used(param)) {
			continue;
		}

		func(arg, param);
	}
}

uint32_t param_hash_check()
{
	uint32_t param_hash = 0;

	param_lock_reader();

	/* compute the CRC32 over all string param names and 4 byte values */
	for (param_t param = 0; handle_in_range(param); param++) {
		if (!param_used(param) || param_is_volatile(param)) {
			continue;
		}

		const char *name = param_name(param);
		const void *val = param_get_value_ptr(param);
		param_hash = crc32part((const uint8_t *)name, strlen(name), param_hash);
		param_hash = crc32part((const uint8_t *)val, param_size(param), param_hash);
	}

	param_unlock_reader();

	return param_hash;
}

void param_print_status()
{
	PX4_INFO("summary: %d/%d (used/total)", param_count_used(), param_count());

#ifndef FLASH_BASED_PARAMS
	const char *filename = param_get_default_file();

	if (filename != nullptr) {
		PX4_INFO("file: %s", param_get_default_file());
	}

#endif /* FLASH_BASED_PARAMS */

	if (param_values != nullptr) {
		PX4_INFO("storage array: %d/%d elements (%zu bytes total)",
			 utarray_len(param_values), param_values->n, param_values->n * sizeof(UT_icd));
	}

	if (param_custom_default_values != nullptr) {
		PX4_INFO("storage array (custom defaults): %d/%d elements (%zu bytes total)",
			 utarray_len(param_custom_default_values), param_custom_default_values->n,
			 param_custom_default_values->n * sizeof(UT_icd));
	}

	PX4_INFO("auto save: %s", autosave_disabled ? "off" : "on");

	if (!autosave_disabled && (last_autosave_timestamp > 0)) {
		PX4_INFO("last auto save: %.3f seconds ago", hrt_elapsed_time(&last_autosave_timestamp) * 1e-6);
	}

	perf_print_counter(param_export_perf);
	perf_print_counter(param_find_perf);
	perf_print_counter(param_get_perf);
	perf_print_counter(param_set_perf);
}
