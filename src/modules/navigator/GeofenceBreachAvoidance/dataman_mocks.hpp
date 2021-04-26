/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file dataman_mocks.h
 * Provides a minimal dataman implementation to compile against for testing
 *
 * @author Roman Bapst
 * @author Julian Kent
 */
#pragma once

#include <dataman/dataman.h>
extern "C" {
	__EXPORT ssize_t
	dm_read(
		dm_item_t item,			/* The item type to retrieve */
		unsigned index,			/* The index of the item */
		void *buffer,			/* Pointer to caller data buffer */
		size_t buflen			/* Length in bytes of data to retrieve */
	) {return 0;};

	/** write to the data manager store */
	__EXPORT ssize_t
	dm_write(
		dm_item_t  item,		/* The item type to store */
		unsigned index,			/* The index of the item */
		dm_persitence_t persistence,	/* The persistence level of this item */
		const void *buffer,		/* Pointer to caller data buffer */
		size_t buflen			/* Length in bytes of data to retrieve */
	) {return 0;};

	/**
	 * Lock all items of a type. Can be used for atomic updates of multiple items (single items are always updated
	 * atomically).
	 * Note that this lock is independent from dm_read & dm_write calls.
	 * @return 0 on success and lock taken, -1 on error (lock not taken, errno set)
	 */
	__EXPORT int
	dm_lock(
		dm_item_t item			/* The item type to lock */
	) {return 0;};

	/**
	 * Try to lock all items of a type (@see sem_trywait()).
	 * @return 0 if lock is taken, -1 otherwise (on error or if already locked. errno is set accordingly)
	 */
	__EXPORT int
	dm_trylock(
		dm_item_t item			/* The item type to lock */
	) {return 0;};

	/** Unlock all items of a type */
	__EXPORT void
	dm_unlock(
		dm_item_t item			/* The item type to unlock */
	) {};

	/** Erase all items of this type */
	__EXPORT int
	dm_clear(
		dm_item_t item			/* The item type to clear */
	) {return 0;};

	/** Tell the data manager about the type of the last reset */
	__EXPORT int
	dm_restart(
		dm_reset_reason restart_type	/* The last reset type */
	) {return 0;};
}

