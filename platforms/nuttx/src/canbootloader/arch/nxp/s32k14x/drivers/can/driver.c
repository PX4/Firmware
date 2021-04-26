/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *       Author: David Sidrane <david.sidrane@nscdg.com>
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
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <limits.h>

#include <hardware/s32k1xx_flexcan.h>
#include "nvic.h"

#include "board.h"
#include <systemlib/px4_macros.h>
#include "can.h"
#include "timer.h"

#include <arch/board/board.h>
#include "flexcan.h"

#include <lib/systemlib/crc.h>


#define CAN_TX_TIMEOUT_MS     (200 /(1000/(1000000/CONFIG_USEC_PER_TICK)))

#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))

/**
 * CANx register sets
 */

/*   Address of the CAN registers */

CanType *CAN0 = (CanType *) S32K1XX_FLEXCAN0_BASE;   // CAN0
enum { NumTxMesgBuffers = 6 };
enum { NumFilters = 16 };

#define CLK_FREQ     80000000
#define CLOCKSEL     0
#define useFIFO      1
#define numberFIFOFilters  CAN_CTRL2_RFFN_16MB
#define FIFO_IFLAG1 (CAN_FIFO_NE | CAN_FIFO_WARN | CAN_FIFO_OV)


typedef struct Timings {
	uint8_t prescaler;
	uint8_t rjw;
	uint8_t pseg1;
	uint8_t pseg2;
	uint8_t propseg;
} Timings;

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
	timer_hrt_clear_wrap();
	uint32_t cnt = CAN_TX_TIMEOUT_MS;

	uint32_t mbi = NumMBinFiFoAndFilters + mailbox;
	uint32_t mb_bit = 1 << mbi;
	MessageBufferType *mb = &CAN0->MB[mbi].mb;

	/* Wait while all the MB are busy */

	while ((CAN0->ESR2 & (CAN_ESR2_IMB | CAN_ESR2_VPS)) == (CAN_ESR2_VPS)) {
		if (timer_hrt_wrap()) {
			timer_hrt_clear_wrap();

			if (--cnt == 0) {
				return CAN_ERROR;
			}
		}
	}

	/* Wait it the requested mailbox is busy */

	while ((CAN0->IFLAG1 & mb_bit) == 0) {

		/* Not indicated, but it may be:
		 * Inactive, aborted,
		 */

		if (mb->CS.code != TxMbDataOrRemote) {
			break;
		}

		if (timer_hrt_wrap()) {
			timer_hrt_clear_wrap();

			if (--cnt == 0) {
				return CAN_ERROR;
			}
		}
	}

	/* Reset the flag */

	CAN0->IFLAG1 = mb_bit;

	/* Construct the frame */

	MBcsType cs;
	cs.code = TxMbDataOrRemote;
	mb->CS.code = TxMbInactive;

	cs.ide = 1;
	mb->ID.ext = message_id;
	cs.rtr = 0;
	cs.dlc = length;
	mb->data.l = __builtin_bswap32(*(uint32_t *)&message[0]);
	mb->data.h = __builtin_bswap32(*(uint32_t *)&message[4]);
	mb->CS = cs; // Go.
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
	uint8_t rv = 0;

	uint32_t flags = CAN0->IFLAG1 & FIFO_IFLAG1;

	if ((flags & FIFO_IFLAG1)) {

		if (flags & CAN_FIFO_OV) {
			CAN0->IFLAG1 = CAN_FIFO_OV;
		}

		if (flags & CAN_FIFO_WARN) {
			CAN0->IFLAG1 = CAN_FIFO_WARN;
		}

		if (flags & CAN_FIFO_NE) {
			const RxFiFoType *rf = &CAN0->MB[FiFo].fifo;
			/*
			 * Read the frame contents
			 */
			*message_id = rf->ID.ext;
			*length     = rf->CS.dlc;
			*(uint32_t *)&message[0] = __builtin_bswap32(rf->data.l);
			*(uint32_t *)&message[4] = __builtin_bswap32(rf->data.h);
			(void)CAN0->TIMER;
			CAN0->IFLAG1 = CAN_FIFO_NE;
			rv = 1;
		}
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
	uint32_t message_id;
	size_t length;
	uint8_t message[8];
	int rv = CAN_ERROR;

	while (rv == CAN_ERROR) {
		for (can_speed_t speed = CAN_125KBAUD; rv == CAN_ERROR  && speed <= CAN_1MBAUD; speed++) {

			can_init(speed, CAN_Mode_Silent);

			bl_timer_id baudtimer = timer_allocate(modeTimeout | modeStarted, 600, 0);

			do {

				if (timer_expired(timeout)) {
					rv = CAN_BOOT_TIMEOUT;
					break;
				}

				if (can_rx(&message_id, &length, message, 0)) {
					*can_speed = speed;
					can_init(speed, CAN_Mode_Normal);
					rv = CAN_OK;
					break;
				}

			} while (!timer_expired(baudtimer));

			timer_free(baudtimer);
		}
	}

	return rv;
}


static bool waitMCRChange(uint32_t mask, bool target_state)
{
	const unsigned Timeout = 1000;

	for (unsigned wait_ack = 0; wait_ack < Timeout; wait_ack++) {
		const bool state = (CAN0->MCR & mask) != 0;

		if (state == target_state) {
			return true;
		}

		up_udelay(10);
	}

	return false;
}

static void setMCR(uint32_t mask, bool target_state)
{
	if (target_state) {
		CAN0->MCR |= mask;

	} else {
		CAN0->MCR &= ~mask;
	}
}

static bool waitFreezeAckChange(bool target_state)
{
	return waitMCRChange(CAN_MCR_FRZACK, target_state);
}

static void setFreeze(bool freeze_true)
{
	{
		if (freeze_true) {
			CAN0->MCR |= CAN_MCR_FRZ;
			CAN0->MCR |= CAN_MCR_HALT;

		} else {
			CAN0->MCR &= ~CAN_MCR_HALT;
			CAN0->MCR &= ~CAN_MCR_FRZ;
		}
	}
}

static bool setEnable(bool enable_true)
{
	setMCR(CAN_MCR_MDIS, !enable_true);
	return waitMCRChange(CAN_MCR_LPMACK, !enable_true);
}

static int16_t doReset(Timings timings)
{
	setMCR(CAN_MCR_SOFTRST, true);

	if (!waitMCRChange(CAN_MCR_SOFTRST, false)) {
		return -ETIME;
	}

	uint8_t tasd = 25 - ((2 * (HWMaxMB + 1)) + 4 / ((1 + timings.pseg1 + timings.pseg2 + timings.propseg) *
			     timings.prescaler));

	setMCR(CAN_MCR_SUPV, false);

	for (int i = 0; i < HWMaxMB; i++) {
		CAN0->MB[i].mb.CS.w = 0x0;
		CAN0->MB[i].mb.ID.w = 0x0;
		CAN0->MB[i].mb.data.l = 0x0;
		CAN0->MB[i].mb.data.h = 0x0;
	}

	setMCR((useFIFO ? CAN_MCR_RFEN : 0) | CAN_MCR_SLFWAK | CAN_MCR_WRNEN | CAN_MCR_SRXDIS | CAN_MCR_IRMQ |
	       CAN_MCR_AEN | (((HWMaxMB - 1) << CAN_MCR_MAXMB_SHIFT) & CAN_MCR_MAXMB_MASK), true);

	CAN0->CTRL2 = numberFIFOFilters | ((tasd << CAN_CTRL2_TASD_SHIFT) & CAN_CTRL2_TASD_MASK) | CAN_CTRL2_RRS |
		      CAN_CTRL2_EACEN;

	for (int i = 0; i < HWMaxMB; i++) {
		CAN0->RXIMR[i].w = 0x0;
	}

	CAN0->RX14MASK = 0x3FFFFFFF;
	CAN0->RX15MASK = 0x3FFFFFFF;
	CAN0->RXMGMASK = 0x3FFFFFFF;
	CAN0->RXFGMASK = 0x0;
	return 0;
}

static int computeTimings(const uint32_t target_bitrate, Timings *out_timings)
{
	if (target_bitrate < 1) {
		return -EINVAL;
	}

	/*
	 * From FlexCAN Bit Timing Calculation by: Petr Stancik TIC
	 *  buadrate = 1 / tNBT  -The tNBT represents a period of the Nominal Bit Time (NBT).
	 *                  The NBT is separated into four non-overlaping segments,
	 *                  SYNC_SEG, PROP_SEG,PHASE_SEG1 and PHASE_SEG2. Each of
	 *                  these segments is an integer multiple of Time Quantum tQ
	 *  tNBT= tQ+PROP_SEG* tQ + PHASE_SEG1* tQ + PHASE_SEG2* tQ = NBT * tQ
	 *  tQ = 1/bitrate = 1/[BOARD_EXTAL_FREQ /(PRESDIV+1)].
	 *   NBT is 8..25
	 *  SYNC_SEG  = 1 tQ
	 *  PROP_SEG  = 1..8 tQ
	 *  PHASE1_SEG  = 1,2..8 tQ     1..8 for 1 Sample per bit (Spb), 2..8 for 3 Spb
	 *  PHASE2_SEG  = 1..8
	 *
	 */
	/*
	 * Hardware configuration
	 */

	/* Maximize NBT
	 * NBT * Prescaler = BOARD_EXTAL_FREQ / baud rate.
	 *
	 */

	const uint32_t nbt_prescaler = CLK_FREQ / target_bitrate;
	const int max_quanta_per_bit = 17;

	out_timings->prescaler = 0;

	/*
	 * Searching for such prescaler value so that the number of quanta per bit is highest.
	 */

	/* tNBT - tQ = PROP_SEG* tQ + PHASE_SEG1* tQ + PHASE_SEG2* tQ = NBT * tQ - tQ */

	for (uint32_t prescaler = 1; prescaler < 256; prescaler++) {
		if (prescaler > nbt_prescaler) {
			return -EINVAL;             // No solution
		}

		if ((0 == nbt_prescaler % prescaler) && (nbt_prescaler / prescaler) < max_quanta_per_bit) {
			out_timings->prescaler = prescaler;
			break;
		}
	}

	const uint32_t NBT = nbt_prescaler / out_timings->prescaler;

	/* Choose a reasonable and some what arbitrary value for Propseg  */

	out_timings->propseg = 5;


	/* Ideal sampling point  = 87.5% given by (SYNC_SEG + PROP_SEG + PHASE_SEG1) / (PHASE_SEG2) */

	uint32_t sp = (7 * NBT) / 8;

	out_timings->pseg1 = sp - 1 - out_timings->propseg;
	out_timings->pseg2 = NBT - (1 + out_timings->pseg1 + out_timings->propseg);
	out_timings->rjw = MIN(4, out_timings->pseg2);

	return ((out_timings->pseg1 <= 8) && (out_timings->pseg2 <= 8) && (out_timings->propseg <= 8)) ?  0 :
	       -EINVAL;
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

	/* Set the module disabled */

	if (!setEnable(false)) {
		return -ETIME;
	}

	/* Set the Clock */

	CAN0->CTRL1 |= CAN_CTRL1_CLKSRC;

	/* Set the module enabled */

	if (!setEnable(true)) {
		return -ETIME;
	}

	/*
	 * CAN timings for this bitrate
	 */
	Timings timings;
	const int timings_res = computeTimings(can_speed2freq(speed), &timings);

	if (timings_res < 0) {
		setEnable(false);
		return timings_res;
	}

	/*
	 * Hardware initialization from reset state
	 */

	int16_t rv = doReset(timings);

	if (rv != 0) {
		return -rv;
	}

	uint32_t ctl1 = 0;
	ctl1 |= CAN_CTRL1_PRESDIV(timings.prescaler - 1);
	ctl1 |= CAN_CTRL1_RJW(timings.rjw - 1);
	ctl1 |= CAN_CTRL1_PSEG1(timings.pseg1 - 1);
	ctl1 |= CAN_CTRL1_PSEG2(timings.pseg2 - 1);
	ctl1 |= ((mode == CAN_Mode_Silent) ? CAN_CTRL1_LOM : 0);
	ctl1 |= CAN_CTRL1_PROPSEG(timings.propseg - 1);
	CAN0->CTRL1 = ctl1;

	/*
	 * Default filter configuration
	 */
	volatile FilterType *filterBase = (FilterType *) &CAN0->MB[FirstFilter].mb;

	for (uint32_t i = 0; i < NumHWFilters; i++) {
		volatile FilterType *filter = &filterBase[i];
		filter->w = 0; // All bits do not care
	}

	CAN0->RXFGMASK = 0; // All bits do not care

	for (uint32_t mb = 0; mb < HWMaxMB; mb++) {
		CAN0->RXIMR[mb].w = 0; // All bits do not care
	}

	CAN0->IFLAG1 = FIFO_IFLAG1 | TXMBMask;
	CAN0->IMASK1 = FIFO_IFLAG1;

	setFreeze(false);
	return waitFreezeAckChange(false) ? 0 : -ETIME;
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
	/* Wait for completion the all 1's wat set in the tx code*/
	while (CAN_ESR1_TX == (CAN0->ESR1 & CAN_ESR1_TX));

	volatile uint32_t esr1 = CAN0->ESR1 & (CAN_ESR1_STFERR | CAN_ESR1_FRMERR |
					       CAN_ESR1_CRCERR | CAN_ESR1_ACKERR |
					       CAN_ESR1_BIT0ERR | CAN_ESR1_BIT1ERR | CAN_ESR1_TXWRN);

	/* Any errors */
	if (esr1) {

		uint32_t mbi = NumMBinFiFoAndFilters + mailbox;
		uint32_t mb_bit = 1 << mbi;
		MessageBufferType *mb = &CAN0->MB[mbi].mb;

		CAN0->ESR1 = esr1;
		CAN0->IFLAG1 = mb_bit;
		mb->CS.code = TxMbAbort;
	}

}
