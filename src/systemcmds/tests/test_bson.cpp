/****************************************************************************
 *
 *  Copyright (C) 2018-2019 PX4 Development Team. All rights reserved.
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
 * @file test_bson.cpp
 * Tests for the bson en/decoder
 */

#include <px4_platform_common/defines.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <limits.h>
#include <math.h>

#include <systemlib/err.h>
#include <parameters/tinybson/tinybson.h>

#include "tests_main.h"

static const bool sample_bool = true;
static const int32_t sample_small_int = 123;
static const int64_t sample_big_int = (int64_t)INT_MAX + 123LL;
static const double sample_double = 2.5f;
static const char *sample_string = "this is a test";
static const uint8_t sample_data[256] = {0};
//static const char *sample_filename = "/fs/microsd/bson.test";

static int
encode(bson_encoder_t encoder)
{
	if (bson_encoder_append_bool(encoder, "bool1", sample_bool) != 0) {
		PX4_ERR("FAIL: encoder: append bool failed");
		return 1;
	}

	if (bson_encoder_append_int(encoder, "int1", sample_small_int) != 0) {
		PX4_ERR("FAIL: encoder: append int failed");
		return 1;
	}

	if (bson_encoder_append_int(encoder, "int2", sample_big_int) != 0) {
		PX4_ERR("FAIL: encoder: append int failed");
		return 1;
	}

	if (bson_encoder_append_double(encoder, "double1", sample_double) != 0) {
		PX4_ERR("FAIL: encoder: append double failed");
		return 1;
	}

	if (bson_encoder_append_string(encoder, "string1", sample_string) != 0) {
		PX4_ERR("FAIL: encoder: append string failed");
		return 1;
	}

	if (bson_encoder_append_binary(encoder, "data1", BSON_BIN_BINARY, sizeof(sample_data), sample_data) != 0) {
		PX4_ERR("FAIL: encoder: append data failed");
		return 1;
	}

	bson_encoder_fini(encoder);

	return 0;
}

static int
decode_callback(bson_decoder_t decoder, bson_node_t node)
{
	unsigned len;

	if (!strcmp(node->name, "bool1")) {
		if (node->type != BSON_BOOL) {
			PX4_ERR("FAIL: decoder: bool1 type %d, expected %d", node->type, BSON_BOOL);
			return 1;
		}

		if (node->b != sample_bool) {
			PX4_ERR("FAIL: decoder: bool1 value %s, expected %s",
				(node->b ? "true" : "false"),
				(sample_bool ? "true" : "false"));
			return 1;
		}

		PX4_INFO("PASS: decoder: bool1");
		return 1;
	}

	if (!strcmp(node->name, "int1")) {
		if (node->type != BSON_INT32) {
			PX4_ERR("FAIL: decoder: int1 type %d, expected %d", node->type, BSON_INT32);
			return 1;
		}

		if (node->i != sample_small_int) {
			PX4_ERR("FAIL: decoder: int1 value %" PRIu64 ", expected %" PRIi32 "", node->i, sample_small_int);
			return 1;
		}

		warnx("PASS: decoder: int1");
		return 1;
	}

	if (!strcmp(node->name, "int2")) {
		if (node->type != BSON_INT64) {
			PX4_ERR("FAIL: decoder: int2 type %d, expected %d", node->type, BSON_INT64);
			return 1;
		}

		if (node->i != sample_big_int) {
			PX4_ERR("FAIL: decoder: int2 value %" PRIu64 ", expected %" PRIu64, node->i, sample_big_int);
			return 1;
		}

		warnx("PASS: decoder: int2");
		return 1;
	}

	if (!strcmp(node->name, "double1")) {
		if (node->type != BSON_DOUBLE) {
			PX4_ERR("FAIL: decoder: double1 type %d, expected %d", node->type, BSON_DOUBLE);
			return 1;
		}

		if (fabs(node->d - sample_double) > 1e-12) {
			PX4_ERR("FAIL: decoder: double1 value %f, expected %f", node->d, sample_double);
			return 1;
		}

		warnx("PASS: decoder: double1");
		return 1;
	}

	if (!strcmp(node->name, "string1")) {
		if (node->type != BSON_STRING) {
			PX4_ERR("FAIL: decoder: string1 type %d, expected %d", node->type, BSON_STRING);
			return 1;
		}

		len = bson_decoder_data_pending(decoder);

		if (len != strlen(sample_string) + 1) {
			PX4_ERR("FAIL: decoder: string1 length %d wrong, expected %zd", len, strlen(sample_string) + 1);
			return 1;
		}

		char sbuf[len];

		if (bson_decoder_copy_data(decoder, sbuf)) {
			PX4_ERR("FAIL: decoder: string1 copy failed");
			return 1;
		}

		if (bson_decoder_data_pending(decoder) != 0) {
			PX4_ERR("FAIL: decoder: string1 copy did not exhaust all data");
			return 1;
		}

		if (sbuf[len - 1] != '\0') {
			PX4_ERR("FAIL: decoder: string1 not 0-terminated");
			return 1;
		}

		if (strcmp(sbuf, sample_string) != 0) {
			PX4_ERR("FAIL: decoder: string1 value '%s', expected '%s'", sbuf, sample_string);
			return 1;
		}

		warnx("PASS: decoder: string1");
		return 1;
	}

	if (!strcmp(node->name, "data1")) {
		if (node->type != BSON_BINDATA) {
			PX4_ERR("FAIL: decoder: data1 type %d, expected %d", node->type, BSON_BINDATA);
			return 1;
		}

		len = bson_decoder_data_pending(decoder);

		if (len != sizeof(sample_data)) {
			PX4_ERR("FAIL: decoder: data1 length %d, expected %zu", len, sizeof(sample_data));
			return 1;
		}

		if (node->subtype != BSON_BIN_BINARY) {
			PX4_ERR("FAIL: decoder: data1 subtype %d, expected %d", node->subtype, BSON_BIN_BINARY);
			return 1;
		}

		uint8_t dbuf[len];

		if (bson_decoder_copy_data(decoder, dbuf)) {
			PX4_ERR("FAIL: decoder: data1 copy failed");
			return 1;
		}

		if (bson_decoder_data_pending(decoder) != 0) {
			PX4_ERR("FAIL: decoder: data1 copy did not exhaust all data");
			return 1;
		}

		if (memcmp(sample_data, dbuf, len) != 0) {
			PX4_ERR("FAIL: decoder: data1 compare fail");
			return 1;
		}

		PX4_INFO("PASS: decoder: data1");
		return 1;
	}

	if (node->type != BSON_EOO) {
		PX4_ERR("FAIL: decoder: unexpected node name '%s'", node->name);
	}

	return 1;
}

static void
decode(bson_decoder_t decoder)
{
	int result;

	do {
		result = bson_decoder_next(decoder);
	} while (result > 0);
}

int
test_bson(int argc, char *argv[])
{
	struct bson_encoder_s encoder;
	struct bson_decoder_s decoder;
	void *buf;
	int len;

	/* encode data to a memory buffer */
	if (bson_encoder_init_buf(&encoder, nullptr, 0)) {
		PX4_ERR("FAIL: bson_encoder_init_buf");
		return 1;
	}

	encode(&encoder);
	len = bson_encoder_buf_size(&encoder);

	if (len <= 0) {
		PX4_ERR("FAIL: bson_encoder_buf_len");
		return 1;
	}

	buf = bson_encoder_buf_data(&encoder);

	if (buf == nullptr) {
		PX4_ERR("FAIL: bson_encoder_buf_data");
		return 1;
	}

	/* now test-decode it */
	if (bson_decoder_init_buf(&decoder, buf, len, decode_callback)) {
		PX4_ERR("FAIL: bson_decoder_init_buf");
		return 1;
	}

	decode(&decoder);
	free(buf);

	return PX4_OK;
}
