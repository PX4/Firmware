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
 * @file tinybson.c
 *
 * A simple subset SAX-style BSON parser and generator.
 */

#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <systemlib/err.h>

#include "tinybson.h"

#if 0
# define debug(fmt, args...)		do { PX4_INFO("BSON: " fmt, ##args); } while(0)
#else
# define debug(fmt, args...)		do { } while(0)
#endif

#define CODER_CHECK(_c)		do { if (_c->dead) { debug("coder dead"); return -1; }} while(0)
#define CODER_KILL(_c, _reason)	do { debug("killed: %s", _reason); _c->dead = true; return -1; } while(0)

static int
read_x(bson_decoder_t decoder, void *p, size_t s)
{
	CODER_CHECK(decoder);

	if (decoder->fd > -1) {
		int ret = ::read(decoder->fd, p, s);

		if (ret == s) {
			decoder->total_decoded_size += ret;
			return 0;
		}

		return -1;
	}

	if (decoder->buf != nullptr) {
		/* staged operations to avoid integer overflow for corrupt data */
		if (s >= decoder->bufsize) {
			CODER_KILL(decoder, "buffer too small for read");
		}

		if ((decoder->bufsize - s) < decoder->bufpos) {
			CODER_KILL(decoder, "not enough data for read");
		}

		memcpy(p, (decoder->buf + decoder->bufpos), s);
		decoder->bufpos += s;
		decoder->total_decoded_size += s;
		return 0;
	}

	debug("no source");
	return -1;
}

static int
read_int8(bson_decoder_t decoder, int8_t *b)
{
	return read_x(decoder, b, sizeof(*b));
}

static int
read_int32(bson_decoder_t decoder, int32_t *i)
{
	return read_x(decoder, i, sizeof(*i));
}

static int
read_int64(bson_decoder_t decoder, int64_t *i)
{
	return read_x(decoder, i, sizeof(*i));
}

static int
read_double(bson_decoder_t decoder, double *d)
{
	return read_x(decoder, d, sizeof(*d));
}

int
bson_decoder_init_file(bson_decoder_t decoder, int fd, bson_decoder_callback callback, void *priv)
{
	decoder->fd = fd;
	decoder->buf = nullptr;
	decoder->dead = false;
	decoder->callback = callback;
	decoder->priv = priv;
	decoder->nesting = 1;
	decoder->pending = 0;
	decoder->node.type = BSON_UNDEFINED;
	decoder->total_decoded_size = 0;

	// read document size
	decoder->total_document_size = 0;

	if (read_int32(decoder, &decoder->total_document_size)) {
		CODER_KILL(decoder, "failed reading length");
	}

	debug("total document size = %d", decoder->total_document_size);

	/* ready for decoding */
	return 0;
}

int
bson_decoder_init_buf(bson_decoder_t decoder, void *buf, unsigned bufsize, bson_decoder_callback callback,
		      void *priv)
{
	/* argument sanity */
	if ((buf == nullptr) || (callback == nullptr)) {
		return -1;
	}

	decoder->fd = -1;
	decoder->buf = (uint8_t *)buf;
	decoder->dead = false;

	if (bufsize == 0) {
		decoder->bufsize = *(uint32_t *)buf;
		debug("auto-detected %u byte object", decoder->bufsize);

	} else {
		decoder->bufsize = bufsize;
	}

	decoder->bufpos = 0;
	decoder->callback = callback;
	decoder->priv = priv;
	decoder->nesting = 1;
	decoder->pending = 0;
	decoder->node.type = BSON_UNDEFINED;
	decoder->total_decoded_size = 0;

	// read document size
	decoder->total_document_size = 0;

	if (read_int32(decoder, &decoder->total_document_size)) {
		CODER_KILL(decoder, "failed reading length");
	}

	debug("total document size = %d", decoder->total_document_size);

	if ((decoder->total_document_size > 0) && (decoder->total_document_size > (int)decoder->bufsize)) {
		CODER_KILL(decoder, "document length larger than buffer");
	}

	/* ready for decoding */
	return 0;
}

int
bson_decoder_next(bson_decoder_t decoder)
{
	int8_t	tbyte;
	int32_t tint;
	unsigned nlen;

	CODER_CHECK(decoder);

	/* if the previous node was EOO, pop a nesting level */
	if (decoder->node.type == BSON_EOO) {
		if (decoder->nesting > 0) {
			decoder->nesting--;
		}

		/* if the nesting level is now zero, the top-level document is done */
		if (decoder->nesting == 0) {
			/* like kill but not an error */
			debug("nesting is zero, document is done");
			decoder->fd = -1;

			/* return end-of-file to the caller */
			return 0;
		}
	}

	/* if there are unread bytes pending in the stream, discard them */
	while (decoder->pending > 0) {
		if (read_int8(decoder, &tbyte)) {
			CODER_KILL(decoder, "read error discarding pending bytes");
		}

		decoder->pending--;
	}

	/* get the type byte */
	if (read_int8(decoder, &tbyte)) {
		CODER_KILL(decoder, "read error on type byte");
	}

	decoder->node.type = (bson_type_t)tbyte;
	decoder->pending = 0;

	debug("got type byte 0x%02x", decoder->node.type);

	/* EOO is special; it has no name/data following */
	if (decoder->node.type == BSON_EOO) {
		decoder->node.name[0] = '\0';

	} else {

		/* get the node name */
		nlen = 0;

		for (;;) {
			if (nlen >= BSON_MAXNAME) {
				CODER_KILL(decoder, "node name overflow");
			}

			if (read_int8(decoder, (int8_t *)&decoder->node.name[nlen])) {
				CODER_KILL(decoder, "read error on node name");
			}

			if (decoder->node.name[nlen] == '\0') {
				break;
			}

			nlen++;
		}

		debug("got name '%s'", decoder->node.name);

		switch (decoder->node.type) {
		case BSON_BOOL:
			if (read_int8(decoder, &tbyte)) {
				CODER_KILL(decoder, "read error on BSON_BOOL");
			}

			decoder->node.b = (tbyte != 0);
			break;

		case BSON_INT32:
			if (read_int32(decoder, &tint)) {
				CODER_KILL(decoder, "read error on BSON_INT");
			}

			decoder->node.i = tint;
			break;

		case BSON_INT64:
			if (read_int64(decoder, &decoder->node.i)) {
				CODER_KILL(decoder, "read error on BSON_INT");
			}

			break;

		case BSON_DOUBLE:
			if (read_double(decoder, &decoder->node.d)) {
				CODER_KILL(decoder, "read error on BSON_DOUBLE");
			}

			break;

		case BSON_STRING:
			if (read_int32(decoder, &decoder->pending)) {
				CODER_KILL(decoder, "read error on BSON_STRING length");
			}

			break;

		case BSON_BINDATA:
			if (read_int32(decoder, &decoder->pending)) {
				CODER_KILL(decoder, "read error on BSON_BINDATA size");
			}

			if (read_int8(decoder, &tbyte)) {
				CODER_KILL(decoder, "read error on BSON_BINDATA subtype");
			}

			decoder->node.subtype = (bson_binary_subtype_t)tbyte;
			break;

		/* XXX currently not supporting other types */
		default:
			CODER_KILL(decoder, "unsupported node type");
		}
	}

	/* call the callback and pass its results back */
	return decoder->callback(decoder, decoder->priv, &decoder->node);
}

int
bson_decoder_copy_data(bson_decoder_t decoder, void *buf)
{
	int result;

	CODER_CHECK(decoder);

	/* copy data */
	result = read_x(decoder, buf, decoder->pending);

	if (result != 0) {
		CODER_KILL(decoder, "read error on copy_data");
	}

	/* pending count is discharged */
	decoder->pending = 0;
	return 0;
}

size_t
bson_decoder_data_pending(bson_decoder_t decoder)
{
	return decoder->pending;
}

static int
write_x(bson_encoder_t encoder, const void *p, size_t s)
{
	CODER_CHECK(encoder);

	/* bson file encoder (non-buffered) */
	if (encoder->fd > -1 && encoder->buf == nullptr) {
		int ret = ::write(encoder->fd, p, s);

		if (ret == s) {
			encoder->total_document_size += ret;
			return 0;
		}

		return -1;
	}

	/* do we need to extend the buffer? */
	while ((encoder->bufpos + s) > encoder->bufsize) {

		/* bson buffered file encoder */
		if (encoder->fd > -1) {
			// write to disk
			debug("writing buffer (%d) to disk", encoder->bufpos);
			int ret = ::write(encoder->fd, encoder->buf, encoder->bufpos);

			if (ret == (int)encoder->bufpos) {
				// reset buffer to beginning and continue
				encoder->bufpos = 0;
				encoder->total_document_size += ret;

				if ((encoder->bufpos + s) > encoder->bufsize) {
					CODER_KILL(encoder, "fixed-size buffer overflow");
				}

				break;

			} else {
				CODER_KILL(encoder, "file write error");
			}
		}

		if (!encoder->realloc_ok) {
			CODER_KILL(encoder, "fixed-size buffer overflow");
		}

		uint8_t *newbuf = (uint8_t *)realloc(encoder->buf, encoder->bufsize + BSON_BUF_INCREMENT);

		if (newbuf == nullptr) {
			CODER_KILL(encoder, "could not grow buffer");
		}

		encoder->buf = newbuf;
		encoder->bufsize += BSON_BUF_INCREMENT;
		debug("allocated %d bytes", BSON_BUF_INCREMENT);
	}

	memcpy(encoder->buf + encoder->bufpos, p, s);
	encoder->bufpos += s;
	debug("appended %d bytes", s);

	return 0;
}

static int
write_int8(bson_encoder_t encoder, int8_t b)
{
	return write_x(encoder, &b, sizeof(b));
}

static int
write_int32(bson_encoder_t encoder, int32_t i)
{
	return write_x(encoder, &i, sizeof(i));
}

static int
write_int64(bson_encoder_t encoder, int64_t i)
{
	return write_x(encoder, &i, sizeof(i));
}

static int
write_double(bson_encoder_t encoder, double d)
{
	return write_x(encoder, &d, sizeof(d));
}

static int
write_name(bson_encoder_t encoder, const char *name)
{
	size_t len = strlen(name);

	if (len > BSON_MAXNAME) {
		CODER_KILL(encoder, "node name too long");
	}

	return write_x(encoder, name, len + 1);
}

int
bson_encoder_init_file(bson_encoder_t encoder, int fd)
{
	encoder->fd = fd;
	encoder->buf = nullptr;
	encoder->dead = false;
	encoder->total_document_size = 0;

	if (write_int32(encoder, 0)) {
		CODER_KILL(encoder, "write error on document length");
	}

	return 0;
}

int
bson_encoder_init_buf_file(bson_encoder_t encoder, int fd, void *buf, unsigned bufsize)
{
	encoder->fd = fd;
	encoder->buf = (uint8_t *)buf;
	encoder->bufpos = 0;
	encoder->bufsize = bufsize;
	encoder->dead = false;
	encoder->realloc_ok = false;
	encoder->total_document_size = 0;

	if (write_int32(encoder, 0)) {
		CODER_KILL(encoder, "write error on document length");
	}

	return 0;
}

int
bson_encoder_init_buf(bson_encoder_t encoder, void *buf, unsigned bufsize)
{
	encoder->fd = -1;
	encoder->buf = (uint8_t *)buf;
	encoder->bufpos = 0;
	encoder->dead = false;

	if (encoder->buf == nullptr) {
		encoder->bufsize = 0;
		encoder->realloc_ok = true;

	} else {
		encoder->bufsize = bufsize;
		encoder->realloc_ok = false;
	}

	encoder->total_document_size = 0;

	if (write_int32(encoder, 0)) {
		CODER_KILL(encoder, "write error on document length");
	}

	return 0;
}

int
bson_encoder_fini(bson_encoder_t encoder)
{
	CODER_CHECK(encoder);

	if (write_int8(encoder, BSON_EOO)) {
		CODER_KILL(encoder, "write error on document terminator");
	}

	if (encoder->fd > -1 && encoder->buf != nullptr && encoder->bufpos > 0) {
		/* write final buffer to disk */
		int ret = ::write(encoder->fd, encoder->buf, encoder->bufpos);

		if (ret == (int)encoder->bufpos) {
			encoder->total_document_size += ret;

		} else {
			CODER_KILL(encoder, "write error");
		}
	}

	// record document size
	if (encoder->fd > -1) {
		if (lseek(encoder->fd, 0, SEEK_SET) == 0) {
			debug("writing document size %d to beginning of file", encoder->total_document_size);

			if (::write(encoder->fd, &encoder->total_document_size,
				    sizeof(encoder->total_document_size)) != sizeof(encoder->total_document_size)) {

				CODER_KILL(encoder, "write error on document length");
			}
		}

		::fsync(encoder->fd);

	} else if (encoder->buf != nullptr) {
		/* update buffer length */
		int32_t len = bson_encoder_buf_size(encoder);
		memcpy(encoder->buf, &len, sizeof(len));
	}

	return 0;
}

int
bson_encoder_buf_size(bson_encoder_t encoder)
{
	CODER_CHECK(encoder);

	if (encoder->fd > -1) {
		return -1;
	}

	return encoder->bufpos;
}

void *
bson_encoder_buf_data(bson_encoder_t encoder)
{
	/* note, no CODER_CHECK here as the caller has to clean up dead buffers */

	if (encoder->fd > -1) {
		return nullptr;
	}

	return encoder->buf;
}

int bson_encoder_append_bool(bson_encoder_t encoder, const char *name, bool value)
{
	CODER_CHECK(encoder);

	if (write_int8(encoder, BSON_BOOL) ||
	    write_name(encoder, name) ||
	    write_int8(encoder, value ? 1 : 0)) {
		CODER_KILL(encoder, "write error on BSON_BOOL");
	}

	return 0;
}

int
bson_encoder_append_int(bson_encoder_t encoder, const char *name, int64_t value)
{
	bool result;

	CODER_CHECK(encoder);

	/* use the smallest encoding that will hold the value */
	if (value == (int32_t)value) {
		debug("encoding %lld as int32", value);
		result = write_int8(encoder, BSON_INT32) ||
			 write_name(encoder, name) ||
			 write_int32(encoder, value);

	} else {
		debug("encoding %lld as int64", value);
		result = write_int8(encoder, BSON_INT64) ||
			 write_name(encoder, name) ||
			 write_int64(encoder, value);
	}

	if (result) {
		CODER_KILL(encoder, "write error on BSON_INT");
	}

	return 0;
}

int
bson_encoder_append_double(bson_encoder_t encoder, const char *name, double value)
{
	CODER_CHECK(encoder);

	if (write_int8(encoder, BSON_DOUBLE) ||
	    write_name(encoder, name) ||
	    write_double(encoder, value)) {
		CODER_KILL(encoder, "write error on BSON_DOUBLE");
	}


	return 0;
}

int
bson_encoder_append_string(bson_encoder_t encoder, const char *name, const char *string)
{
	size_t len;

	CODER_CHECK(encoder);

	len = strlen(string) + 1;	/* include trailing nul */

	if (write_int8(encoder, BSON_STRING) ||
	    write_name(encoder, name) ||
	    write_int32(encoder, len) ||
	    write_x(encoder, string, len)) {
		CODER_KILL(encoder, "write error on BSON_STRING");
	}

	return 0;
}

int
bson_encoder_append_binary(bson_encoder_t encoder, const char *name, bson_binary_subtype_t subtype, size_t size,
			   const void *data)
{
	CODER_CHECK(encoder);

	if (write_int8(encoder, BSON_BINDATA) ||
	    write_name(encoder, name) ||
	    write_int32(encoder, size) ||
	    write_int8(encoder, subtype) ||
	    write_x(encoder, data, size)) {
		CODER_KILL(encoder, "write error on BSON_BINDATA");
	}

	return 0;
}
