/****************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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
 * @file sbus.c
 *
 * Serial protocol decoder for the Futaba S.bus protocol.
 */

#include <px4_platform_common/px4_config.h>

#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#ifdef TIOCSSINGLEWIRE
#include <sys/ioctl.h>
#endif

#include "sbus.h"
#include "common_rc.h"
#include <drivers/drv_hrt.h>

using namespace time_literals;

#define SBUS_DEBUG_LEVEL 	0 /* Set debug output level */

#if defined(__PX4_LINUX)
#include <sys/ioctl.h>
#include <asm-generic/termbits.h>
#else
#include <termios.h>
#endif

#define SBUS_START_SYMBOL	0x0f

#define SBUS_INPUT_CHANNELS	16
#define SBUS_FLAGS_BYTE		23
#define SBUS_FAILSAFE_BIT	3
#define SBUS_FRAMELOST_BIT	2

// testing with a SBUS->PWM adapter shows that
// above 300Hz SBUS becomes unreliable. 333 would
// be the theoretical achievable, but at 333Hz some
// frames are lost
#define SBUS1_MAX_RATE_HZ	300
#define SBUS1_MIN_RATE_HZ	50

// this is the rate of the old code
#define SBUS1_DEFAULT_RATE_HZ	72

/*
  Measured values with Futaba FX-30/R6108SB:
    -+100% on TX:  PCM 1.100/1.520/1.950ms -> SBus raw values: 350/1024/1700  (100% ATV)
    -+140% on TX:  PCM 0.930/1.520/2.112ms -> SBus raw values:  78/1024/1964  (140% ATV)
    -+152% on TX:  PCM 0.884/1.520/2.160ms -> SBus raw values:   1/1024/2047  (140% ATV plus dirty tricks)
*/

/* define range mapping here, -+100% -> 1000..2000 */
#define SBUS_RANGE_MIN 200.0f
#define SBUS_RANGE_MAX 1800.0f

#define SBUS_TARGET_MIN 1000.0f
#define SBUS_TARGET_MAX 2000.0f

#if defined(SBUS_DEBUG_LEVEL) && SBUS_DEBUG_LEVEL > 0
#  include <stdio.h>
#endif

/* pre-calculate the floating point stuff as far as possible at compile time */
#define SBUS_SCALE_FACTOR ((SBUS_TARGET_MAX - SBUS_TARGET_MIN) / (SBUS_RANGE_MAX - SBUS_RANGE_MIN))
#define SBUS_SCALE_OFFSET (int)(SBUS_TARGET_MIN - (SBUS_SCALE_FACTOR * SBUS_RANGE_MIN + 0.5f))

static hrt_abstime last_rx_time;
static hrt_abstime last_txframe_time = 0;

#define SBUS2_FRAME_SIZE_RX_VOLTAGE	3
#define SBUS2_FRAME_SIZE_GPS_DIGIT	3

static enum SBUS2_DECODE_STATE {
	SBUS2_DECODE_STATE_DESYNC = 0xFFF,
	SBUS2_DECODE_STATE_SBUS_START = 0x2FF,
	SBUS2_DECODE_STATE_SBUS1_SYNC = 0x00,
	SBUS2_DECODE_STATE_SBUS2_SYNC = 0x1FF,
	SBUS2_DECODE_STATE_SBUS2_RX_VOLTAGE = 0x04,
	SBUS2_DECODE_STATE_SBUS2_GPS = 0x14,
	SBUS2_DECODE_STATE_SBUS2_DATA1 = 0x24,
	SBUS2_DECODE_STATE_SBUS2_DATA2 = 0x34
} sbus_decode_state = SBUS2_DECODE_STATE_DESYNC;

static sbus_frame_t	 &sbus_frame = rc_decode_buf.sbus_frame;

static unsigned partial_frame_count;
static unsigned sbus1_frame_delay = (1000U * 1000U) / SBUS1_DEFAULT_RATE_HZ;

static unsigned sbus_frame_drops;

unsigned
sbus_dropped_frames()
{
	return sbus_frame_drops;
}

static bool
sbus_decode(uint64_t frame_time, uint8_t *frame, uint16_t *values, uint16_t *num_values,
	    bool *sbus_failsafe, bool *sbus_frame_drop, uint16_t max_values);

int
sbus_init(const char *device, bool singlewire)
{
	int sbus_fd = open(device, O_RDWR | O_NONBLOCK);

	int ret = sbus_config(sbus_fd, singlewire);

	if (!ret) {
		return sbus_fd;

	} else {
		return -1;
	}
}

int
sbus_config(int sbus_fd, bool singlewire)
{
	int ret = -1;

#if defined(__PX4_LINUX)

	struct termios2 tio = {};

	if (0 != ioctl(sbus_fd, TCGETS2, &tio)) {
		return ret;
	}

	/**
	 * Setting serial port,8E2, non-blocking.100Kbps
	 */
	tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL
			 | IXON);
	tio.c_iflag |= (INPCK | IGNPAR);
	tio.c_oflag &= ~OPOST;
	tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tio.c_cflag &= ~(CSIZE | CRTSCTS | PARODD | CBAUD);
	/**
	 * use BOTHER to specify speed directly in c_[io]speed member
	 */
	tio.c_cflag |= (CS8 | CSTOPB | CLOCAL | PARENB | BOTHER | CREAD);
	tio.c_ispeed = 100000;
	tio.c_ospeed = 100000;
	tio.c_cc[VMIN] = 25;
	tio.c_cc[VTIME] = 0;

	if (0 != ioctl(sbus_fd, TCSETS2, &tio)) {
		return ret;
	}

	ret = 0;
#else

	if (sbus_fd >= 0) {
		struct termios t;

		/* 100000bps, even parity, two stop bits */
		tcgetattr(sbus_fd, &t);
		cfsetspeed(&t, 100000);
		t.c_cflag |= (CSTOPB | PARENB);
		tcsetattr(sbus_fd, TCSANOW, &t);

		if (singlewire) {
			/* only defined in configs capable of IOCTL
			 * Note It is never turned off
			 */
#ifdef TIOCSSINGLEWIRE
			ioctl(sbus_fd, TIOCSSINGLEWIRE, SER_SINGLEWIRE_ENABLED);
#endif
		}

		ret = 0;
	}

#endif
	/* initialise the decoder */
	partial_frame_count = 0;
	last_rx_time = hrt_absolute_time();
	sbus_frame_drops = 0;

	return ret;
}

void
sbus1_output(int sbus_fd, uint16_t *values, uint16_t num_values)
{
	uint8_t byteindex = 1; /*Data starts one byte into the sbus frame. */
	uint8_t offset = 0;
	uint16_t value;
	hrt_abstime now;

	now = hrt_absolute_time();

	if ((now - last_txframe_time) > sbus1_frame_delay) {
		last_txframe_time = now;
		uint8_t	oframe[SBUS_FRAME_SIZE] = { 0x0f };

		/* 16 is sbus number of servos/channels minus 2 single bit channels.
		* currently ignoring single bit channels.  */

		for (unsigned i = 0; (i < num_values) && (i < 16); ++i) {
			value = (uint16_t)(((values[i] - SBUS_SCALE_OFFSET) / SBUS_SCALE_FACTOR) + .5f);

			/*protect from out of bounds values and limit to 11 bits*/
			if (value > 0x07ff) {
				value = 0x07ff;
			}

			while (offset >= 8) {
				++byteindex;
				offset -= 8;
			}

			oframe[byteindex] |= (value << (offset)) & 0xff;
			oframe[byteindex + 1] |= (value >> (8 - offset)) & 0xff;
			oframe[byteindex + 2] |= (value >> (16 - offset)) & 0xff;
			offset += 11;
		}

		write(sbus_fd, oframe, SBUS_FRAME_SIZE);
	}
}
void
sbus2_output(int sbus_fd, uint16_t *values, uint16_t num_values)
{
	sbus1_output(sbus_fd, values, num_values);
}

bool
sbus_input(int sbus_fd, uint16_t *values, uint16_t *num_values, bool *sbus_failsafe, bool *sbus_frame_drop,
	   uint16_t max_channels)
{
	/*
	 * Fetch bytes, but no more than we would need to complete
	 * a complete frame.
	 */
	uint8_t buf[SBUS_FRAME_SIZE * 2];

	int ret = read(sbus_fd, &buf[0], SBUS_FRAME_SIZE);

	/* if the read failed for any reason, just give up here */
	if (ret < 1) {
		return false;
	}

	const hrt_abstime now = hrt_absolute_time();
#ifdef __PX4_NUTTX /* limit time-based hardening to RTOS's where we have reliable timing */

	/*
	 * The S.BUS protocol doesn't provide reliable framing,
	 * so we detect frame boundaries by the inter-frame delay.
	 *
	 * The minimum frame spacing is 7ms; with 25 bytes at 100000bps
	 * frame transmission time is 3ms.
	 *
	 * We expect to only be called when bytes arrive for processing,
	 * and if an interval of more than 3ms passes between calls,
	 * the first byte we read will be the first byte of a frame.
	 *
	 * In the case where byte(s) are dropped from a frame, this also
	 * provides a degree of protection. Of course, it would be better
	 * if we didn't drop bytes...
	 */
	if (now - last_rx_time > 3_ms) {
		if (partial_frame_count > 0) {
			partial_frame_count = 0;
			sbus_decode_state = SBUS2_DECODE_STATE_DESYNC;
#if defined(SBUS_DEBUG_LEVEL) && SBUS_DEBUG_LEVEL > 0
			printf("SBUS: RESET (TIME LIM)\n");
#endif
		}
	}

	if (partial_frame_count == 0 && buf[0] != SBUS_START_SYMBOL) {
		/* don't bother going through the buffer if we don't get the
		 * expected start symbol as a first byte */
		sbus_decode_state = SBUS2_DECODE_STATE_DESYNC;
		return false;
	}

#endif /* __PX4_NUTTX */

	/*
	 * Try to decode something with what we got
	 */
	return sbus_parse(now, &buf[0], ret, values, num_values, sbus_failsafe,
			  sbus_frame_drop, &sbus_frame_drops, max_channels);
}

bool
sbus_parse(uint64_t now, uint8_t *frame, unsigned len, uint16_t *values,
	   uint16_t *num_values, bool *sbus_failsafe, bool *sbus_frame_drop, unsigned *frame_drops, uint16_t max_channels)
{

	last_rx_time = now;

	/* this is set by the decoding state machine and will default to false
	 * once everything that was decodable has been decoded.
	 */
	bool decode_ret = false;

	/* keep decoding until we have consumed the buffer */
	for (unsigned d = 0; d < len; d++) {

		/* overflow check */
		if (partial_frame_count == sizeof(sbus_frame) / sizeof(sbus_frame[0])) {
			partial_frame_count = 0;
			sbus_decode_state = SBUS2_DECODE_STATE_DESYNC;
#if defined(SBUS_DEBUG_LEVEL) && SBUS_DEBUG_LEVEL > 0
			printf("SBUS2: RESET (BUF LIM)\n");
#endif
		}

		if (partial_frame_count == SBUS_FRAME_SIZE) {
			partial_frame_count = 0;
			sbus_decode_state = SBUS2_DECODE_STATE_DESYNC;
#if defined(SBUS_DEBUG_LEVEL) && SBUS_DEBUG_LEVEL > 0
			printf("SBUS2: RESET (PACKET LIM)\n");
#endif
		}

#if defined(SBUS_DEBUG_LEVEL) && SBUS_DEBUG_LEVEL > 1
		printf("sbus state: %s%s%s%s%s%s, count: %d, val: %02x\n",
		       (sbus_decode_state == SBUS2_DECODE_STATE_DESYNC) ? "SBUS2_DECODE_STATE_DESYNC" : "",
		       (sbus_decode_state == SBUS2_DECODE_STATE_SBUS_START) ? "SBUS2_DECODE_STATE_SBUS_START" : "",
		       (sbus_decode_state == SBUS2_DECODE_STATE_SBUS1_SYNC) ? "SBUS2_DECODE_STATE_SBUS1_SYNC" : "",
		       (sbus_decode_state == SBUS2_DECODE_STATE_SBUS2_SYNC) ? "SBUS2_DECODE_STATE_SBUS2_SYNC" : "",
		       (sbus_decode_state == SBUS2_DECODE_STATE_SBUS2_RX_VOLTAGE) ? "SBUS2_DECODE_STATE_SBUS2_RX_VOLTAGE" : "",
		       (sbus_decode_state == SBUS2_DECODE_STATE_SBUS2_GPS) ? "SBUS2_DECODE_STATE_SBUS2_GPS" : "",
		       partial_frame_count,
		       (unsigned)frame[d]);
#endif

		switch (sbus_decode_state) {
		case SBUS2_DECODE_STATE_DESYNC:

			/* we are de-synced and only interested in the frame marker */
			if (frame[d] == SBUS_START_SYMBOL) {
				sbus_decode_state = SBUS2_DECODE_STATE_SBUS_START;
				partial_frame_count = 0;
				sbus_frame[partial_frame_count++] = frame[d];
			}

			break;

		/* fall through */
		case SBUS2_DECODE_STATE_SBUS_START:
		case SBUS2_DECODE_STATE_SBUS1_SYNC:

		/* fall through */
		case SBUS2_DECODE_STATE_SBUS2_SYNC: {
				sbus_frame[partial_frame_count++] = frame[d];

				/* decode whatever we got and expect */
				if (partial_frame_count < SBUS_FRAME_SIZE) {
					break;
				}

				/*
				 * Great, it looks like we might have a frame. Go ahead and decode it.
				 */
				decode_ret = sbus_decode(now, sbus_frame, values, num_values, sbus_failsafe, sbus_frame_drop, max_channels);

				/*
				 * Offset recovery: If decoding failed, check if there is a second
				 * start marker in the packet.
				 */
				unsigned start_index = 0;

				if (!decode_ret && sbus_decode_state == SBUS2_DECODE_STATE_DESYNC) {

					for (unsigned i = 1; i < partial_frame_count; i++) {
						if (sbus_frame[i] == SBUS_START_SYMBOL) {
							start_index = i;
							break;
						}
					}

					/* we found a second start marker */
					if (start_index != 0) {
						/* shift everything in the buffer and reset the state machine */
						for (unsigned i = 0; i < partial_frame_count - start_index; i++) {
							sbus_frame[i] = sbus_frame[i + start_index];
						}

						partial_frame_count -= start_index;
						sbus_decode_state = SBUS2_DECODE_STATE_SBUS_START;

#if defined(SBUS_DEBUG_LEVEL) && SBUS_DEBUG_LEVEL > 0
						printf("DECODE RECOVERY: %d\n", start_index);
#endif
					}
				}

				/* if there has been no successful attempt at saving a failed
				 * decoding run, reset the frame count for successful and
				 * unsuccessful decode runs.
				 */
				if (start_index == 0) {
					partial_frame_count = 0;
				}

			}
			break;

		case SBUS2_DECODE_STATE_SBUS2_RX_VOLTAGE: {
				sbus_frame[partial_frame_count++] = frame[d];

				if (partial_frame_count == 1 && sbus_frame[0] == SBUS_START_SYMBOL) {
					/* this slot is unused and in fact S.BUS2 sync */
					sbus_decode_state = SBUS2_DECODE_STATE_SBUS2_SYNC;
				}

				if (partial_frame_count < SBUS2_FRAME_SIZE_RX_VOLTAGE) {
					break;
				}

				/* find out which payload we're dealing with in this slot */
				switch (sbus_frame[0]) {
				case 0x03: {
						// Observed values:
						// (frame[0] == 0x3 && frame[1] == 0x84 && frame[2] == 0x0)
						// (frame[0] == 0x3 && frame[1] == 0xc4 && frame[2] == 0x0)
						// (frame[0] == 0x3 && frame[1] == 0x80 && frame[2] == 0x2f)
						// (frame[0] == 0x3 && frame[1] == 0xc0 && frame[2] == 0x2f)
#if defined(SBUS_DEBUG_LEVEL) && SBUS_DEBUG_LEVEL > 2
						uint16_t rx_voltage = (sbus_frame[1] << 8) | sbus_frame[2];
						printf("rx_voltage %d\n", (int)rx_voltage);
#endif
					}

					partial_frame_count = 0;
					break;

				default:
					/* this is not what we expect it to be, go back to sync */
					sbus_decode_state = SBUS2_DECODE_STATE_DESYNC;
					sbus_frame_drops++;
				}

			}
			break;

		case SBUS2_DECODE_STATE_SBUS2_GPS: {
				sbus_frame[partial_frame_count++] = frame[d];

				if (partial_frame_count == 1 && sbus_frame[0] == SBUS_START_SYMBOL) {
					/* this slot is unused and in fact S.BUS2 sync */
					sbus_decode_state = SBUS2_DECODE_STATE_SBUS2_SYNC;
				}

				if (partial_frame_count < 24) {
					break;
				}

				/* find out which payload we're dealing with in this slot */
				switch (sbus_frame[0]) {
				case 0x13: {
#if defined(SBUS_DEBUG_LEVEL) && SBUS_DEBUG_LEVEL > 0
						uint16_t gps_something = (frame[1] << 8) | frame[2];
						printf("gps_something %d\n", (int)gps_something);
#endif
					}

					partial_frame_count = 0;
					break;

				default:
					/* this is not what we expect it to be, go back to sync */
					sbus_decode_state = SBUS2_DECODE_STATE_DESYNC;
					sbus_frame_drops++;
					/* throw unknown bytes away */
				}
			}
			break;

		default:
#if defined(SBUS_DEBUG_LEVEL) && SBUS_DEBUG_LEVEL > 0
			printf("UNKNOWN PROTO STATE");
#endif
			decode_ret = false;
		}


	}

	if (frame_drops) {
		*frame_drops = sbus_frame_drops;
	}

	/* return false as default */
	return decode_ret;
}

/*
 * S.bus decoder matrix.
 *
 * Each channel value can come from up to 3 input bytes. Each row in the
 * matrix describes up to three bytes, and each entry gives:
 *
 * - byte offset in the data portion of the frame
 * - right shift applied to the data byte
 * - mask for the data byte
 * - left shift applied to the result into the channel value
 */
struct sbus_bit_pick {
	uint8_t byte;
	uint8_t rshift;
	uint8_t mask;
	uint8_t lshift;
};
static const struct sbus_bit_pick sbus_decoder[SBUS_INPUT_CHANNELS][3] = {
	/*  0 */ { { 0, 0, 0xff, 0}, { 1, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
	/*  1 */ { { 1, 3, 0x1f, 0}, { 2, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
	/*  2 */ { { 2, 6, 0x03, 0}, { 3, 0, 0xff, 2}, { 4, 0, 0x01, 10} },
	/*  3 */ { { 4, 1, 0x7f, 0}, { 5, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
	/*  4 */ { { 5, 4, 0x0f, 0}, { 6, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
	/*  5 */ { { 6, 7, 0x01, 0}, { 7, 0, 0xff, 1}, { 8, 0, 0x03,  9} },
	/*  6 */ { { 8, 2, 0x3f, 0}, { 9, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
	/*  7 */ { { 9, 5, 0x07, 0}, {10, 0, 0xff, 3}, { 0, 0, 0x00,  0} },
	/*  8 */ { {11, 0, 0xff, 0}, {12, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
	/*  9 */ { {12, 3, 0x1f, 0}, {13, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
	/* 10 */ { {13, 6, 0x03, 0}, {14, 0, 0xff, 2}, {15, 0, 0x01, 10} },
	/* 11 */ { {15, 1, 0x7f, 0}, {16, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
	/* 12 */ { {16, 4, 0x0f, 0}, {17, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
	/* 13 */ { {17, 7, 0x01, 0}, {18, 0, 0xff, 1}, {19, 0, 0x03,  9} },
	/* 14 */ { {19, 2, 0x3f, 0}, {20, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
	/* 15 */ { {20, 5, 0x07, 0}, {21, 0, 0xff, 3}, { 0, 0, 0x00,  0} }
};

bool
sbus_decode(uint64_t frame_time, uint8_t *frame, uint16_t *values, uint16_t *num_values,
	    bool *sbus_failsafe, bool *sbus_frame_drop, uint16_t max_values)
{

	/* check frame boundary markers to avoid out-of-sync cases */
	if (frame[0] != SBUS_START_SYMBOL) {
		sbus_frame_drops++;
#if defined(SBUS_DEBUG_LEVEL) && SBUS_DEBUG_LEVEL > 0
		printf("DECODE FAIL: ");

		for (unsigned i = 0; i < SBUS_FRAME_SIZE; i++) {
			printf("%0x ", frame[i]);
		}

		printf("\n");
#endif
		sbus_decode_state = SBUS2_DECODE_STATE_DESYNC;
		return false;
	}

	/* the last byte in the frame indicates what frame will follow after this one */
	switch (frame[24]) {
	case 0x00:
		/* this is S.BUS 1 */
		sbus_decode_state = SBUS2_DECODE_STATE_SBUS1_SYNC;
		break;

	case 0x04:
		/* receiver voltage */
		sbus_decode_state = SBUS2_DECODE_STATE_SBUS2_RX_VOLTAGE;
		break;

	case 0x14:
		/* GPS / baro */
		sbus_decode_state = SBUS2_DECODE_STATE_SBUS2_GPS;
		break;

	case 0x24:
		/* Unknown SBUS2 data */
		sbus_decode_state = SBUS2_DECODE_STATE_SBUS2_SYNC;
		break;

	case 0x34:
		/* Unknown SBUS2 data */
		sbus_decode_state = SBUS2_DECODE_STATE_SBUS2_SYNC;
		break;

	default:
#if defined(SBUS_DEBUG_LEVEL) && SBUS_DEBUG_LEVEL > 0
		printf("DECODE FAIL: END MARKER\n");
#endif
		sbus_decode_state = SBUS2_DECODE_STATE_DESYNC;
		return false;
	}

	/* we have received something we think is a frame */
	unsigned chancount = (max_values > SBUS_INPUT_CHANNELS) ?
			     SBUS_INPUT_CHANNELS : max_values;

	/* use the decoder matrix to extract channel data */
	for (unsigned channel = 0; channel < chancount; channel++) {
		unsigned value = 0;

		for (unsigned pick = 0; pick < 3; pick++) {
			const struct sbus_bit_pick *decode = &sbus_decoder[channel][pick];

			if (decode->mask != 0) {
				unsigned piece = frame[1 + decode->byte];
				piece >>= decode->rshift;
				piece &= decode->mask;
				piece <<= decode->lshift;

				value |= piece;
			}
		}


		/* convert 0-2048 values to 1000-2000 ppm encoding in a not too sloppy fashion */
		values[channel] = (uint16_t)(value * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
	}

	/* decode switch channels if data fields are wide enough */
	if (max_values > 17 && chancount > 15) {
		chancount = 18;

		/* channel 17 (index 16) */
		values[16] = ((frame[SBUS_FLAGS_BYTE] & (1 << 0)) ? 1000 : 0) + 998;
		/* channel 18 (index 17) */
		values[17] = ((frame[SBUS_FLAGS_BYTE] & (1 << 1)) ? 1000 : 0) + 998;
	}

	/* note the number of channels decoded */
	*num_values = chancount;

	/* decode and handle failsafe and frame-lost flags */
	if (frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FAILSAFE_BIT)) { /* failsafe */
		/* report that we failed to read anything valid off the receiver */
		*sbus_failsafe = true;
		*sbus_frame_drop = true;

	} else if (frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FRAMELOST_BIT)) { /* a frame was lost */
		/* set a special warning flag
		 *
		 * Attention! This flag indicates a skipped frame only, not a total link loss! Handling this
		 * condition as fail-safe greatly reduces the reliability and range of the radio link,
		 * e.g. by prematurely issuing return-to-launch!!! */

		*sbus_failsafe = false;
		*sbus_frame_drop = true;

	} else {
		*sbus_failsafe = false;
		*sbus_frame_drop = false;
	}

	return true;
}

/*
  set output rate of SBUS in Hz
 */
void sbus1_set_output_rate_hz(uint16_t rate_hz)
{
	if (rate_hz > SBUS1_MAX_RATE_HZ) {
		rate_hz = SBUS1_MAX_RATE_HZ;
	}

	if (rate_hz < SBUS1_MIN_RATE_HZ) {
		rate_hz = SBUS1_MIN_RATE_HZ;
	}

	sbus1_frame_delay = (1000U * 1000U) / rate_hz;
}
