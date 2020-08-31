/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *       Author: Ben Dyer <ben_dyer@mac.com>
 *               Pavel Kirienko <pavel.kirienko@zubax.com>
 *               David Sidrane <david_s5@nscdg.com>
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

#include <nuttx/config.h>
#include "boot_config.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <limits.h>

#include "chip.h"
#include "stm32.h"
#include <hardware/stm32_can.h>
#include "nvic.h"

#include "board.h"
#include "px4_macros.h"
#include "can.h"
#include "crc.h"
#include "timer.h"

#include <arch/board/board.h>


#define INAK_TIMEOUT          65535

#define CAN_TX_TIMEOUT_MS     (200 /(1000/(1000000/CONFIG_USEC_PER_TICK)))

#define SJW_POS               24
#define BS1_POS               16
#define BS2_POS               20

#define CAN_TSR_RQCP_SHFTS    8

#define FILTER_ID             1
#define FILTER_MASK           2

#if STM32_PCLK1_FREQUENCY == 45000000 || STM32_PCLK1_FREQUENCY == 36000000
/* Sample 88.9 % */
# define QUANTA 9
# define BS1_VALUE 6
# define BS2_VALUE 0
#elif STM32_PCLK1_FREQUENCY == 42000000
/* Sample 85.7 % */
# define QUANTA 14
# define BS1_VALUE 10
# define BS2_VALUE 1
#else
# warning Undefined QUANTA bsed on Clock Rate
/* Sample 85.7 % */
# define QUANTA 14
# define BS1_VALUE 10
# define BS2_VALUE 1
#endif

#define CAN_1MBAUD_SJW 0
#define CAN_1MBAUD_BS1 BS1_VALUE
#define CAN_1MBAUD_BS2 BS2_VALUE
#define CAN_1MBAUD_PRESCALER (STM32_PCLK1_FREQUENCY/1000000/QUANTA)

#define CAN_500KBAUD_SJW 0
#define CAN_500KBAUD_BS1 BS1_VALUE
#define CAN_500KBAUD_BS2 BS2_VALUE
#define CAN_500KBAUD_PRESCALER (STM32_PCLK1_FREQUENCY/500000/QUANTA)

#define CAN_250KBAUD_SJW 0
#define CAN_250KBAUD_BS1 BS1_VALUE
#define CAN_250KBAUD_BS2 BS2_VALUE
#define CAN_250KBAUD_PRESCALER (STM32_PCLK1_FREQUENCY/250000/QUANTA)

#define CAN_125KBAUD_SJW 0
#define CAN_125KBAUD_BS1 BS1_VALUE
#define CAN_125KBAUD_BS2 BS2_VALUE
#define CAN_125KBAUD_PRESCALER (STM32_PCLK1_FREQUENCY/125000/QUANTA)

#define CAN_BTR_LBK_SHIFT 30

// Number of CPU cycles for a single bit time at the supported speeds
#define CAN_125KBAUD_BIT_CYCLES (8*(TIMER_HRT_CYCLES_PER_US))

#define CAN_BAUD_TIME_IN_MS             200
#define CAN_BAUD_SAMPLES_NEEDED         32
#define CAN_BAUD_SAMPLES_DISCARDED      8

static inline uint32_t read_msr_rx(void)
{
	return getreg32(STM32_CAN1_MSR) & CAN_MSR_RX;
}

static uint32_t read_msr(time_hrt_cycles_t  *now)
{
	__asm__ __volatile__("\tcpsid  i\n");
	*now = timer_hrt_read();
	uint32_t msr = read_msr_rx();
	__asm__ __volatile__("\tcpsie  i\n");
	return msr;
}

static int read_bits_times(time_hrt_cycles_t *times, size_t max)
{
	uint32_t samplecnt = 0;
	bl_timer_id ab_timer = timer_allocate(modeTimeout | modeStarted, CAN_BAUD_TIME_IN_MS, 0);
	time_ref_t ab_ref =  timer_ref(ab_timer);
	uint32_t msr;
	uint32_t last_msr = read_msr(times);

	while (samplecnt < max && !timer_ref_expired(ab_ref)) {
		do {
			msr = read_msr(&times[samplecnt]);
		} while (!(msr ^ last_msr) && !timer_ref_expired(ab_ref));

		last_msr = msr;
		samplecnt++;
	}

	timer_free(ab_timer);
	return samplecnt;
}

/****************************************************************************
 * Name: can_speed2freq
 *
 * Description:
 *   This function maps a can_speed_t to a bit rate in Hz
 *
 * Input Parameters:
 *   can_speed - A can_speed_t from CAN_125KBAUD to CAN_1MBAUD
 *
 * Returned value:
 *   Bit rate in Hz
 *
 ****************************************************************************/
int can_speed2freq(can_speed_t speed)

{
	return 1000000 >> (CAN_1MBAUD - speed);
}

/****************************************************************************
 * Name: can_speed2freq
 *
 * Description:
 *   This function maps a frequency in Hz to a can_speed_t in the range
 *   CAN_125KBAUD to CAN_1MBAUD.
 *
 * Input Parameters:
 *   freq - Bit rate in Hz
 *
 * Returned value:
 *   A can_speed_t from CAN_125KBAUD to CAN_1MBAUD
 *
 ****************************************************************************/
can_speed_t can_freq2speed(int freq)
{
	return (freq == 1000000u ? CAN_1MBAUD : freq == 500000u ? CAN_500KBAUD : freq == 250000u ? CAN_250KBAUD : CAN_125KBAUD);
}

/****************************************************************************
 * Name: can_tx
 *
 * Description:
 *   This function is called to transmit a CAN frame using the supplied
 *   mailbox. It will busy wait on the mailbox if not available.
 *
 * Input Parameters:
 *   message_id - The CAN message's EXID field
 *   length     - The number of bytes of data - the DLC field
 *   message    - A pointer to 8 bytes of data to be sent (all 8 bytes will be
 *                loaded into the CAN transmitter but only length bytes will
 *                be sent.
 *   mailbox    - A can_fifo_mailbox_t MBxxx value to choose the outgoing
 *                mailbox.
 *
 * Returned value:
 *   The CAN_OK of the data sent or CAN_ERROR if a time out occurred
 *
 ****************************************************************************/
uint8_t can_tx(uint32_t message_id, size_t length, const uint8_t *message, uint8_t mailbox)
{
	uint32_t data[2];

	memcpy(data, message, sizeof(data));

	/*
	 * Just block while waiting for the mailbox.
	 */

	uint32_t mask = CAN_TSR_TME0 << mailbox;

	/* Reset the indication of timer expired */

	timer_hrt_clear_wrap();
	uint32_t cnt = CAN_TX_TIMEOUT_MS;

	while ((getreg32(STM32_CAN1_TSR) & mask) == 0) {
		if (timer_hrt_wrap()) {
			timer_hrt_clear_wrap();

			if (--cnt == 0) {
				return CAN_ERROR;
			}
		}
	}

	/*
	 * To allow detection of completion  - Set the LEC to
	 * 'No error' state off all 1s
	 */

	putreg32(CAN_ESR_LEC_MASK, STM32_CAN1_ESR);

	putreg32(length & CAN_TDTR_DLC_MASK, STM32_CAN1_TDTR(mailbox));
	putreg32(data[0], STM32_CAN1_TDLR(mailbox));
	putreg32(data[1], STM32_CAN1_TDHR(mailbox));
	putreg32((message_id << CAN_TIR_EXID_SHIFT) | CAN_TIR_IDE | CAN_TIR_TXRQ,
		 STM32_CAN1_TIR(mailbox));
	return CAN_OK;
}

/****************************************************************************
 * Name: can_rx
 *
 * Description:
 *   This function is called to receive a CAN frame from a supplied fifo.
 *   It does not block if there is not available, but returns 0
 *
 * Input Parameters:
 *   message_id - A pointer to return the CAN message's EXID field
 *   length     - A pointer to return the number of bytes of data - the DLC field
 *   message    - A pointer to return 8 bytes of data to be sent (all 8 bytes will
 *                be written from the CAN receiver but only length bytes will be sent.
 *   fifo         A can_fifo_mailbox_t fifixxx value to choose the incoming fifo.
 *
 * Returned value:
 *   The length of the data read or 0 if the fifo was empty
 *
 ****************************************************************************/
uint8_t can_rx(uint32_t *message_id, size_t *length, uint8_t *message, uint8_t fifo)
{
	uint32_t data[2];
	uint8_t rv = 0;
	const uint32_t fifos[] = { STM32_CAN1_RF0R, STM32_CAN1_RF1R };

	if (getreg32(fifos[fifo & 1]) & CAN_RFR_FMP_MASK) {

		rv = 1;
		/* If so, process it */

		*message_id = (getreg32(STM32_CAN1_RIR(fifo)) & CAN_RIR_EXID_MASK) >>
			      CAN_RIR_EXID_SHIFT;
		*length = (getreg32(STM32_CAN1_RDTR(fifo)) & CAN_RDTR_DLC_MASK) >>
			  CAN_RDTR_DLC_SHIFT;
		data[0] = getreg32(STM32_CAN1_RDLR(fifo));
		data[1] = getreg32(STM32_CAN1_RDHR(fifo));

		putreg32(CAN_RFR_RFOM, fifos[fifo & 1]);

		memcpy(message, data, sizeof(data));
	}

	return rv;
}

/****************************************************************************
 * Name: can_autobaud
 *
 * Description:
 *   This function will attempt to detect the bit rate in use on the CAN
 *   interface until the timeout provided expires or the successful detection
 *   occurs.
 *
 *   It will initialize the CAN block for a given bit rate
 *   to test that a message can be received. The CAN interface is left
 *   operating at the detected bit rate and in CAN_Mode_Normal mode.
 *
 * Input Parameters:
 *   can_speed - A pointer to return detected can_speed_t from CAN_UNKNOWN to
 *               CAN_1MBAUD
 *   timeout   - The timer id of a timer to use as the maximum time to wait for
 *               successful bit rate detection. This timer may be not running
 *               in which case the auto baud code will try indefinitely to
 *               detect the bit rate.
 *
 * Returned value:
 *   CAN_OK - on Success or a CAN_BOOT_TIMEOUT
 *
 ****************************************************************************/
int can_autobaud(can_speed_t *can_speed, bl_timer_id timeout)
{
	*can_speed = CAN_UNKNOWN;

	volatile int attempt = 0;
	/* Threshold are at 1.5 Bit times */

	/*
	 * We are here because there was a reset or the app invoked
	 * the bootloader with no bit rate set.
	 */

	time_hrt_cycles_t bit_time;
	time_hrt_cycles_t min_cycles;
	int sample;
	can_speed_t speed = CAN_125KBAUD;

	time_hrt_cycles_t samples[128];

	while (1) {


		while (1) {

			min_cycles = ULONG_MAX;
			int samplecnt = read_bits_times(samples, arraySize(samples));

			if (timer_expired(timeout)) {
				return CAN_BOOT_TIMEOUT;
			}

			if ((getreg32(STM32_CAN1_RF0R) | getreg32(STM32_CAN1_RF1R)) &
			    CAN_RFR_FMP_MASK) {
				*can_speed = speed;
				can_init(speed, CAN_Mode_Normal);
				return CAN_OK;
			}

			if (samplecnt < CAN_BAUD_SAMPLES_NEEDED) {
				continue;
			}

			for (sample = 0; sample < samplecnt; sample += 2) {
				bit_time = samples[sample] = timer_hrt_elapsed(samples[sample], samples[sample + 1]);

				if (sample > CAN_BAUD_SAMPLES_DISCARDED && bit_time < min_cycles) {
					min_cycles = bit_time;
				}
			}

			break;
		}

		uint32_t bit34 = CAN_125KBAUD_BIT_CYCLES - CAN_125KBAUD_BIT_CYCLES / 4;
		samples[1] = min_cycles;
		speed = CAN_125KBAUD;

		while (min_cycles < bit34 && speed < CAN_1MBAUD) {
			speed++;
			bit34 /= 2;
		}

		attempt++;
		can_init(speed, CAN_Mode_Silent);


	} /* while(1) */

	return CAN_OK;
}

/****************************************************************************
 * Name: can_init
 *
 * Description:
 *   This function is used to initialize the CAN block for a given bit rate and
 *   mode.
 *
 * Input Parameters:
 *   speed - A can_speed_t from CAN_125KBAUD to CAN_1MBAUD
 *   mode  - One of the can_mode_t of Normal, LoopBack and Silent or
 *           combination thereof.
 *
 * Returned value:
 *   OK - on Success or a negate errno value
 *
 ****************************************************************************/
int can_init(can_speed_t speed, can_mode_t mode)
{
	int speedndx = speed - 1;
	/*
	 * TODO: use full-word writes to reduce the number of loads/stores.
	 *
	 * Also consider filter use -- maybe set filters for all the message types we
	 * want. */
	const uint32_t bitrates[] = {
		(CAN_125KBAUD_SJW << SJW_POS) |
		(CAN_125KBAUD_BS1 << BS1_POS) |
		(CAN_125KBAUD_BS2 << BS2_POS) | (CAN_125KBAUD_PRESCALER - 1),

		(CAN_250KBAUD_SJW << SJW_POS) |
		(CAN_250KBAUD_BS1 << BS1_POS) |
		(CAN_250KBAUD_BS2 << BS2_POS) | (CAN_250KBAUD_PRESCALER - 1),

		(CAN_500KBAUD_SJW << SJW_POS) |
		(CAN_500KBAUD_BS1 << BS1_POS) |
		(CAN_500KBAUD_BS2 << BS2_POS) | (CAN_500KBAUD_PRESCALER - 1),

		(CAN_1MBAUD_SJW   << SJW_POS) |
		(CAN_1MBAUD_BS1   << BS1_POS) |
		(CAN_1MBAUD_BS2   << BS2_POS) | (CAN_1MBAUD_PRESCALER - 1)
	};

	/* Remove Unknow Offset */
	if (speedndx < 0 || speedndx > (int)arraySize(bitrates)) {
		return -EINVAL;
	}

	uint32_t timeout;
	/*
	 *  Reset state is 0x0001 0002 CAN_MCR_DBF|CAN_MCR_SLEEP
	 *  knock down Sleep and raise CAN_MCR_INRQ
	 */

	putreg32(CAN_MCR_DBF | CAN_MCR_INRQ, STM32_CAN1_MCR);

	/* Wait until initialization mode is acknowledged */

	for (timeout = INAK_TIMEOUT; timeout > 0; timeout--) {
		if ((getreg32(STM32_CAN1_MSR) & CAN_MSR_INAK) != 0) {
			/* We are in initialization mode */

			break;
		}
	}

	if (timeout < 1) {
		/*
		 * Initialization failed, not much we can do now other than try a normal
		 * startup. */
		return -ETIME;
	}


	putreg32(bitrates[speedndx] | mode << CAN_BTR_LBK_SHIFT, STM32_CAN1_BTR);
	putreg32(CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_DBF | CAN_MCR_TXFP, STM32_CAN1_MCR);

	for (timeout = INAK_TIMEOUT; timeout > 0; timeout--) {
		if ((getreg32(STM32_CAN1_MSR) & CAN_MSR_INAK) == 0) {
			/* We are in initialization mode */

			break;
		}
	}

	if (timeout < 1) {
		return -ETIME;
	}

	/*
	 * CAN filter initialization -- accept everything on RX FIFO 0, and only
	 * GetNodeInfo requests on RX FIFO 1. */

	/* Set FINIT - Initialization mode for the filters- */
	putreg32(CAN_FMR_FINIT, STM32_CAN1_FMR);

	putreg32(0, STM32_CAN1_FA1R); /* Disable all filters */

	putreg32(3, STM32_CAN1_FS1R); /* Single 32-bit scale configuration for filters 0 and 1 */

	/* Filter 0 masks -- DTIDGetNodeInfo requests only */

	uavcan_protocol_t protocol;

	protocol.id.u32 = 0;
	protocol.ser.type_id = DTIDReqGetNodeInfo;
	protocol.ser.service_not_message = true;
	protocol.ser.request_not_response = true;

	/* Set the Filter ID */
	putreg32(protocol.id.u32 << CAN_RIR_EXID_SHIFT, STM32_CAN1_FIR(0, FILTER_ID));

	/* Set the Filter Mask */
	protocol.ser.type_id = BIT_MASK(LengthUavCanServiceTypeID);

	putreg32(protocol.id.u32 << CAN_RIR_EXID_SHIFT, STM32_CAN1_FIR(0, FILTER_MASK));

	/* Filter 1 masks -- everything is don't-care */
	putreg32(0, STM32_CAN1_FIR(1, FILTER_ID));
	putreg32(0, STM32_CAN1_FIR(1, FILTER_MASK));

	putreg32(0, STM32_CAN1_FM1R);   /* Mask mode for all filters */
	putreg32(1, STM32_CAN1_FFA1R);  /* FIFO 1 for filter 0, FIFO 0 for the
                                         * rest of filters */
	putreg32(3, STM32_CAN1_FA1R);   /* Enable filters 0 and 1 */

	/* Clear FINIT - Active mode for the filters- */

	putreg32(0, STM32_CAN1_FMR);

	return OK;
}

/****************************************************************************
 * Name: can_cancel_on_error
 *
 * Description:
 *   This function will test for transition completion or any error.
 *   If the is a error the the transmit will be aborted.
 *
 * Input Parameters:
 *   mailbox    - A can_fifo_mailbox_t MBxxx value to choose the outgoing
 *                mailbox.
 *
 * Returned value:
 *   CAN_OK - on Success or a CAN_ERROR if the cancellation was needed
 *
 ****************************************************************************/
void can_cancel_on_error(uint8_t mailbox)
{
	uint32_t rvalue;

	/* Wait for completion the all 1's wat set in the tx code*/
	while (CAN_ESR_LEC_MASK == ((rvalue = (getreg32(STM32_CAN1_ESR) & CAN_ESR_LEC_MASK))));

	/* Any errors */
	if (rvalue) {

		putreg32(0, STM32_CAN1_ESR);

		/* Abort the the TX in case of collision wuth NART false  */

		putreg32(CAN_TSR_ABRQ0 << (mailbox * CAN_TSR_RQCP_SHFTS), STM32_CAN1_TSR);
	}
}
