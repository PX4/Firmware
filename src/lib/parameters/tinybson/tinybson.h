/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
* @file tinybson.h
*
* A simple subset SAX-style BSON parser and generator. See http://bsonspec.org
*
* Some types and defines taken from the standalone BSON parser/generator
* in the Mongo C connector.
*/

#ifndef _TINYBSON_H
#define _TINYBSON_H

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

/** subset of the BSON node types we might care about */
typedef enum {
	BSON_EOO = 0,
	BSON_DOUBLE = 1,
	BSON_STRING = 2,
	BSON_OBJECT = 3,
	BSON_ARRAY = 4,
	BSON_BINDATA = 5,
	BSON_UNDEFINED = 6,
	BSON_BOOL = 8,
	BSON_DATE = 9,
	BSON_nullptr = 10,
	BSON_INT32 = 16,
	BSON_INT64 = 18
} bson_type_t;

typedef enum bson_binary_subtype {
	BSON_BIN_BINARY = 0,
	BSON_BIN_USER = 128
} bson_binary_subtype_t;

/**
 * Maximum node name length.
 */
#define BSON_MAXNAME		32

/**
 * Buffer growth increment when writing to a buffer.
 */
#define BSON_BUF_INCREMENT	128

/**
 * Node structure passed to the callback.
 */
typedef struct bson_node_s {
	char			name[BSON_MAXNAME];
	bson_type_t		type;
	bson_binary_subtype_t	subtype;
	union {
		int64_t		i;
		double		d;
		bool		b;
	};
} *bson_node_t;

typedef struct bson_decoder_s *bson_decoder_t;

/**
 * Node callback.
 *
 * The node callback function's return value is returned by bson_decoder_next.
 */
typedef int	(* bson_decoder_callback)(bson_decoder_t decoder, void *priv, bson_node_t node);

struct bson_decoder_s {
	/* file reader state */
	int			fd;

	/* buffer reader state */
	uint8_t			*buf;
	size_t			bufsize;
	unsigned		bufpos;

	bool			dead;
	bson_decoder_callback	callback;
	void			*priv;
	unsigned		nesting;
	struct bson_node_s	node;
	int32_t			pending;

	int32_t                 total_document_size;
	int32_t                 total_decoded_size;
};

/**
 * Initialise the decoder to read from a file.
 *
 * @param decoder		Decoder state structure to be initialised.
 * @param fd			File to read BSON data from.
 * @param callback		Callback to be invoked by bson_decoder_next
 * @param priv          Callback private data, stored in node.
 * @return			Zero on success.
 */
__EXPORT int bson_decoder_init_file(bson_decoder_t decoder, int fd, bson_decoder_callback callback, void *priv);

/**
 * Initialise the decoder to read from a buffer in memory.
 *
 * @param decoder		Decoder state structure to be initialised.
 * @param buf			Buffer to read from.
 * @param bufsize		Size of the buffer (BSON object may be smaller).  May be
 *				passed as zero if the buffer size should be extracted from the
 *				BSON header only.
 * @param callback		Callback to be invoked by bson_decoder_next
 * @param priv		Callback private data, stored in node.
 * @return			Zero on success.
 */
__EXPORT int bson_decoder_init_buf(bson_decoder_t decoder, void *buf, unsigned bufsize, bson_decoder_callback callback,
				   void *priv);

/**
 * Process the next node from the stream and invoke the callback.
 *
 * @param decoder		Decoder state, must have been initialised with bson_decoder_init.
 * @return			-1 if parsing encountered an error, 0 if the BSON stream has ended,
 *				otherwise the return value from the callback.
 */
__EXPORT int bson_decoder_next(bson_decoder_t decoder);

/**
 * Copy node data.
 *
 * @param decoder		Decoder state, must have been initialised with bson_decoder_init.
 */
__EXPORT int bson_decoder_copy_data(bson_decoder_t decoder, void *buf);

/**
 * Report copyable data size.
 *
 * @param decoder		Decoder state, must have been initialised with bson_decoder_init.
 */
__EXPORT size_t bson_decoder_data_pending(bson_decoder_t decoder);

/**
 * Encoder state structure.
 */
typedef struct bson_encoder_s {
	/* file writer state */
	int		fd;

	/* buffer writer state */
	uint8_t		*buf;
	unsigned	bufsize;
	unsigned	bufpos;

	bool		realloc_ok;
	bool		dead;

	int32_t        total_document_size;

} *bson_encoder_t;

/**
 * Initialze the encoder for writing to a file.
 *
 * @param encoder		Encoder state structure to be initialised.
 * @param fd			File to write to.
 * @return			Zero on success.
 */
__EXPORT int bson_encoder_init_file(bson_encoder_t encoder, int fd);

/**
 * Initialze the encoder for writing to a file.
 *
 * @param encoder		Encoder state structure to be initialised.
 * @param fd			File to write to.
 * @param buf			Buffer pointer to use, can't be nullptr
 * @param bufsize		Supplied buffer size
 * @return			Zero on success.
 */
__EXPORT int bson_encoder_init_buf_file(bson_encoder_t encoder, int fd, void *buf, unsigned bufsize);

/**
 * Initialze the encoder for writing to a buffer.
 *
 * @param encoder		Encoder state structure to be initialised.
 * @param buf			Buffer pointer to use, or nullptr if the buffer
 *				should be allocated by the encoder.
 * @param bufsize		Maximum buffer size, or zero for no limit. If
 *				the buffer is supplied, the size of the supplied buffer.
 * @return			Zero on success.
 */
__EXPORT int bson_encoder_init_buf(bson_encoder_t encoder, void *buf, unsigned bufsize);

/**
 * Finalise the encoded stream.
 *
 * @param encoder		The encoder to finalise.
 */
__EXPORT int bson_encoder_fini(bson_encoder_t encoder);

/**
 * Fetch the size of the encoded object; only valid for buffer operations.
 */
__EXPORT int bson_encoder_buf_size(bson_encoder_t encoder);

/**
 * Get a pointer to the encoded object buffer.
 *
 * Note that if the buffer was allocated by the encoder, it is the caller's responsibility
 * to free this buffer.
 */
__EXPORT void *bson_encoder_buf_data(bson_encoder_t encoder);

/**
 * Append a boolean to the encoded stream.
 *
 * @param encoder		Encoder state.
 * @param name			Node name.
 * @param value			Value to be encoded.
 */
__EXPORT int bson_encoder_append_bool(bson_encoder_t encoder, const char *name, bool value);

/**
 * Append an integer to the encoded stream.
 *
 * @param encoder		Encoder state.
 * @param name			Node name.
 * @param value			Value to be encoded.
 */
__EXPORT int bson_encoder_append_int(bson_encoder_t encoder, const char *name, int64_t value);

/**
 * Append a double to the encoded stream
 *
 * @param encoder		Encoder state.
 * @param name			Node name.
 * @param value			Value to be encoded.
 */
__EXPORT int bson_encoder_append_double(bson_encoder_t encoder, const char *name, double value);

/**
 * Append a string to the encoded stream.
 *
 * @param encoder		Encoder state.
 * @param name			Node name.
 * @param string		Nul-terminated C string.
 */
__EXPORT int bson_encoder_append_string(bson_encoder_t encoder, const char *name, const char *string);

/**
 * Append a binary blob to the encoded stream.
 *
 * @param encoder		Encoder state.
 * @param name			Node name.
 * @param subtype		Binary data subtype.
 * @param size			Data size.
 * @param data			Buffer containing data to be encoded.
 */
__EXPORT int bson_encoder_append_binary(bson_encoder_t encoder, const char *name, bson_binary_subtype_t subtype,
					size_t size, const void *data);


#endif
