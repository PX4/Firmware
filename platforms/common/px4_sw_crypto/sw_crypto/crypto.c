/****************************************************************************
 *
 *   Copyright (c) 2020 Technology Innovation Institute. All rights reserved.
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
 * @file crypto.c
 *
 * Wrapper for the monocypher crypto
 *
 */

#include <inttypes.h>
#include <stdbool.h>

#include <px4_platform_common/crypto_backend.h>
#include <lib/crypto/monocypher/src/optional/monocypher-ed25519.h>
#include <tomcrypt.h>

extern void libtomcrypt_init(void);

/* room for 16 keys */
#define KEY_CACHE_LEN 16

#ifndef SECMEM_ALLOC
#define SECMEM_ALLOC malloc
#endif

#ifndef SECMEM_FREE
#define SECMEM_FREE free
#endif

/*
 * For now, this is just a dummy up/down counter for tracking open/close calls
 */
static int crypto_open_count = 0;

typedef struct {
	size_t key_size;
	uint8_t *key;
} volatile_key_t;

static volatile_key_t key_cache[KEY_CACHE_LEN];

typedef struct {
	uint8_t nonce[24];
	uint64_t ctr;
} chacha20_context_t;

/* Clear key cache */
static void clear_key_cache(void)
{
	for (int i = 0; i < KEY_CACHE_LEN ; i++) {
		SECMEM_FREE(key_cache[i].key);
		key_cache[i].key = NULL;
		key_cache[i].key_size = 0;
	}
}

/* Retrieve a direct pointer to the cached temporary/public key */
static const uint8_t *crypto_get_key_ptr(keystore_session_handle_t handle, uint8_t key_idx,
		size_t *len)
{
	uint8_t *ret;

	if (key_idx >= KEY_CACHE_LEN) {
		*len = 0;
		return NULL;
	}

	ret = key_cache[key_idx].key;

	/* if the key doesn't exist in the key cache, try to read it in there from keystore */
	if (ret == NULL) {

		/* First check if the key exists in the keystore and retrieve its length */
		*len = keystore_get_key(handle, key_idx, NULL, 0);

		if (*len > 0) {

			/* Allocate memory for the key in the cache */
			ret = SECMEM_ALLOC(*len);

			/* Retrieve the key from the keystore */
			if (ret) {
				if (keystore_get_key(handle, key_idx, ret, *len) > 0) {
					/* Success, store the key in cache */
					key_cache[key_idx].key_size = *len;
					key_cache[key_idx].key = ret;

				} else {
					/* key retrieval failed, free the memory */
					SECMEM_FREE(ret);
				}
			}
		}
	}

	*len = key_cache[key_idx].key_size;

	return ret;
}


void crypto_init()
{
	keystore_init();
	clear_key_cache();
	libtomcrypt_init();
}

crypto_session_handle_t crypto_open(px4_crypto_algorithm_t algorithm)
{
	crypto_session_handle_t ret;
	ret.keystore_handle = keystore_open();

	if (keystore_session_handle_valid(ret.keystore_handle)) {
		ret.algorithm = algorithm;
		ret.handle = ++crypto_open_count;

	} else {
		ret.handle = 0;
		ret.context = NULL;
		ret.algorithm = CRYPTO_NONE;
		return ret;
	}

	switch (algorithm) {
	case CRYPTO_XCHACHA20: {
			chacha20_context_t *context = malloc(sizeof(chacha20_context_t));

			if (!context) {
				ret.handle = 0;
				crypto_open_count--;

			} else {
				ret.context = context;
				px4_get_secure_random(context->nonce, sizeof(context->nonce));
				context->ctr = 0;
			}
		}
		break;

	default:
		ret.context = NULL;
	}

	return ret;
}

void crypto_close(crypto_session_handle_t *handle)
{
	crypto_open_count--;
	handle->handle = 0;
	keystore_close(&handle->keystore_handle);
	free(handle->context);
	handle->context = NULL;
}

bool crypto_signature_check(crypto_session_handle_t handle,
			    uint8_t  key_index,
			    const uint8_t  *signature,
			    const uint8_t *message,
			    size_t message_size)
{
	bool ret = false;
	size_t keylen = 0;
	const uint8_t *public_key;

	if (crypto_session_handle_valid(handle)) {
		public_key = crypto_get_key_ptr(handle.keystore_handle, key_index, &keylen);
	}

	if (keylen == 0) {
		return false;
	}

	switch (handle.algorithm) {
	case CRYPTO_ED25519:
		ret = crypto_ed25519_check(signature, public_key, message, message_size) == 0;
		break;

	default:
		ret = false;
	}

	return ret;
}

bool crypto_encrypt_data(crypto_session_handle_t handle,
			 uint8_t  key_idx,
			 const uint8_t *message,
			 size_t message_size,
			 uint8_t *cipher,
			 size_t *cipher_size)
{

	bool ret = false;

	switch (handle.algorithm) {
	case CRYPTO_NONE:
		if (*cipher_size >= message_size) {
			memcpy(cipher, message, message_size);
			*cipher_size = message_size;
			ret = true;
		}

		break;

	case CRYPTO_XCHACHA20: {
			size_t key_sz;
			uint8_t *key = (uint8_t *)crypto_get_key_ptr(handle.keystore_handle, key_idx, &key_sz);
			chacha20_context_t *context = handle.context;

			if (key_sz == 32) {
				context->ctr = crypto_xchacha20_ctr(cipher, message, *cipher_size, key, context->nonce, context->ctr);
				ret = true;
			}
		}
		break;

	case CRYPTO_RSA_OAEP: {
			rsa_key key;
			size_t key_sz;
			unsigned long outlen = *cipher_size;
			uint8_t *public_key = (uint8_t *)crypto_get_key_ptr(handle.keystore_handle, key_idx, &key_sz);

			if (public_key &&
			    rsa_import(public_key, key_sz, &key) == CRYPT_OK &&
			    outlen >= ltc_mp.unsigned_size(key.N)) {

				unsigned long modulus_bitlen = ltc_mp.count_bits(key.N);

				if (pkcs_1_oaep_encode(message, message_size,
						       NULL, 0,
						       modulus_bitlen, NULL,
						       0, 0,
						       cipher, &outlen) == CRYPT_OK &&
				    rsa_exptmod(cipher, outlen, cipher, &outlen, PK_PUBLIC, &key) == CRYPT_OK) {
					*cipher_size = outlen;
					ret = true;

				} else {
					*cipher_size = 0;
				}
			}
		}
		break;

	default:
		break;
	}

	return ret;
}

bool crypto_get_encrypted_key(crypto_session_handle_t handle,
			      uint8_t key_idx,
			      uint8_t *key,
			      size_t *max_len,
			      uint8_t encryption_key_idx)
{
	// Retrieve the plaintext key
	bool ret = true;
	size_t key_sz;
	const uint8_t *plain_key = crypto_get_key_ptr(handle.keystore_handle, key_idx, &key_sz);

	if (key_sz == 0) {
		return false;
	}

	// Caclulate the key size, when encrypted
	switch (handle.algorithm) {
	case CRYPTO_RSA_OAEP: {
			rsa_key enc_key;
			unsigned pub_key_sz;
			uint8_t *pub_key = (uint8_t *)crypto_get_key_ptr(handle.keystore_handle, encryption_key_idx, &pub_key_sz);

			if (pub_key &&
			    rsa_import(pub_key, pub_key_sz, &enc_key) == CRYPT_OK) {
				*max_len = ltc_mp.unsigned_size(enc_key.N);

			} else {
				*max_len = 0;
			}
		}

		break;

	default:
		*max_len = key_sz;
	}

	// Encrypt it
	if (key != NULL) {
		ret = crypto_encrypt_data(handle,
					  encryption_key_idx,
					  plain_key,
					  key_sz,
					  key,
					  max_len);
	}

	return ret;
}


bool crypto_get_nonce(crypto_session_handle_t handle,
		      uint8_t *nonce,
		      size_t *nonce_len)
{
	switch (handle.algorithm) {
	case CRYPTO_XCHACHA20: {
			chacha20_context_t *context = handle.context;

			if (nonce != NULL && context != NULL) {
				memcpy(nonce, context->nonce, sizeof(context->nonce));
			}

			*nonce_len = sizeof(context->nonce);
		}
		break;

	default:
		*nonce_len = 0;
	}

	return true;
}
