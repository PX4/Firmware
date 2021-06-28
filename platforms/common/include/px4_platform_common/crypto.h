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

#pragma once

#ifdef PX4_CRYPTO

#include <stdbool.h>
#include <stdint.h>

#include <px4_random.h>
#include <px4_platform_common/crypto_algorithms.h>
#include <px4_platform_common/sem.h>
#include "crypto_backend_definitions.h"

/*
 * Crypto API interface class
 */

class PX4Crypto
{
public:

	/*
	 * Crypto api initialization, called only once at boot
	 */

	static void px4_crypto_init(void);

	/*
	 * Class function to open and return a pointer to crypto api
	 */

	static PX4Crypto *px4_crypto_open(px4_crypto_algorithm_t algorithm);

	static void px4_crypto_close(PX4Crypto *&crypto);

	/*
	 * Keystore access functions
	 */

	/*
	 * Generate a single random key for symmetric-key encryption
	 *
	 * algorithm: generate a suitable key for this algorithm
	 * idx: the index in keystore where the key will be stored
	 * persistent: whether the key need to be stored persistently
	 * returns true on success, false on failure
	 */

	bool generate_key(uint8_t idx,
			  bool persistent);

	/*
	 * Generate a key pair for asymmetric-key encryption
	 *
	 * algorithm: the key type
	 * key_size: size of the key in bytes
	 * private_idx: the private key will be stored in this index in the keystore
	 * public_idx: the public key will be stored in this index in the keystore
	 * persistent: whether the keys need to be stored persistently
	 * returns true on success, false on failure
	*/

	bool generate_keypair(px4_crypto_algorithm_t algorithm,
			      size_t key_size,
			      uint8_t private_idx,
			      uint8_t public_idx,
			      bool persistent);


	/*
	 * Re-create or set nonce.
	 *
	 * A nonce or intialization vector value for the selected algortithm is
	 * automatically generated when the crypto session is opened. If needed, the
	 * nonce can be set by this function.
	 * If this is called with NULL pointer, a new nonce is automatically random
	 * generated
	 */

	bool renew_nonce(const uint8_t *nonce, size_t nonce_size);


	/*
	 * Get current crypto session nonce
	 *
	 * This function returns the current nonce for the session
	 * If the "nonce" is NULL, only nonse legth will be provided
	 * nonce: pointer to the buffer where the nonce will be written
	 * nonce_len: length of the current nonce vector for the session
	 * returns true on success, false on failure
	 */

	bool get_nonce(uint8_t *nonce,
		       size_t *nonce_len);

	/*
	 * Store a key into keystore
	 *
	 * encryption_idx: The key index in keystore to be used in decrypting and
	 *   authenticating the key before storing
	 * key: The pointer to the key
	 * key_idx: Index where the key will be stored in keystore
	*/


	bool set_key(uint8_t encryption_idx,
		     const uint8_t *key,
		     uint8_t key_idx);

	/*
	 * Get a key from keystore. Key can be encrypted
	 *
	 * key_idx: Index of the requested key in the keystore
	 * key: The provided buffer to the key. If NULL, the function only provides
	 *          the length of the key.
	 * key_len: input: the size of the provided "key" buffer.
	            output: the actual size of the key
	 * encryption_key_idx: The key index in keystore to be used for encrypting
	 * returns true on success, false on failure
	 *
	 */

	bool get_encrypted_key(uint8_t key_idx,
			       uint8_t *key,
			       size_t *key_len,
			       uint8_t encryption_key_idx);

	/*
	 * PX4 Crypto API functions
	 */

	/*
	 * Verify signature
	 *
	 * key_index: public key index in keystore
	 * signature: pointer to the signature
	 * message: pointer to the data to be verified
	 * message_size: size of the message in bytes
	*/

	bool signature_check(uint8_t  key_index,
			     const uint8_t *signature,
			     const uint8_t *message,
			     size_t message_size);


	/*
	 * Encrypt data
	 *
	 * key_index: key index in keystore
	 * message: pointer to the message
	 * message_size: size of the message in bytes
	 * cipher: pointer to a buffer for encrypted data
	 * cipher_size: size of the buffer reserved for cipher and actual cipher length
	 *   after the encryption
	 * returns true on success, false on failure
	 */

	bool encrypt_data(uint8_t  key_index,
			  const uint8_t *message,
			  size_t message_size,
			  uint8_t *cipher,
			  size_t *cipher_size);

private:
	/* Instances are created and deleted through px4_crypto_open and px4_crypto_close */
	PX4Crypto();
	PX4Crypto(const crypto_session_handle_t &handle);
	~PX4Crypto();

	crypto_session_handle_t crypto_handle;

	static px4_sem_t _lock;
	static bool _initialized;
	static void lock() { do {} while (px4_sem_wait(&PX4Crypto::_lock) != 0); }
	static void unlock() { px4_sem_post(&PX4Crypto::_lock); }
};

#endif
