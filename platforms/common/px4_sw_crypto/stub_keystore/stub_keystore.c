/****************************************************************************
 *
 *   Copyright (c) 2021 Technology Innovation Institute. All rights reserved.
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

#include <px4_platform_common/crypto_algorithms.h>
#include <string.h>
#include <stdbool.h>
#include "public_key.h"
#include "keystore_backend_definitions.h"
/*
 * For now, this is just a dummy up/down counter for tracking open/close calls
 */
static int keystore_open_count = 0;

void keystore_init(void)
{
}

keystore_session_handle_t keystore_open(void)
{
	keystore_session_handle_t ret;
	ret.handle = ++keystore_open_count;
	return ret;
}

void keystore_close(keystore_session_handle_t *handle)
{
	keystore_open_count--;
	handle->handle = 0;
}

size_t keystore_get_key(keystore_session_handle_t handle, uint8_t idx, uint8_t *key_buf, size_t key_buf_size)
{
	size_t ret = 0;

	if (idx < NPERSISTENT_KEYS) {
		ret = public_keys[idx].key_size;

		if (key_buf) {
			if (key_buf_size >= ret) {
				memcpy(key_buf, public_keys[idx].key, ret);

			} else {
				ret = 0;
			}
		}
	}

	return ret;
}

bool keystore_put_key(keystore_session_handle_t handle, uint8_t idx, uint8_t *key, size_t key_size)
{
	return false;
}
