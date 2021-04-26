/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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
 * @file uc_stm32h7_can.cpp
 *
 * STM32H7 FDCAN driver for libuavcan
 *
 * @author Jacob Crabill <jacob@flyvoly.com>
 */

/*
 * Original libuavcan driver for stm32:
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cassert>
#include <cstring>
#include <uavcan_stm32h7/can.hpp>
#include <uavcan_stm32h7/clock.hpp>
#include "internal.hpp"

#if UAVCAN_STM32H7_NUTTX
# include <nuttx/arch.h>
# include <nuttx/irq.h>
# include <arch/board/board.h>
#else
# error "Unknown OS"
#endif

#define WORD_LENGTH 4U
#define FIFO_ELEMENT_SIZE 4U // size in words of a FIFO element in message RAM

// Rx FIFO element definition for classic CAN frame
// RM0433 page 2536
typedef struct __attribute__((__packed__))
{
	// word R0
	uint32_t id : 29;
	uint32_t RTR : 1;
	uint32_t XTD : 1;
	uint32_t ESI : 1;

	// word R1
	uint16_t RXTS;
	uint8_t DLC : 4;  // Data Length Code
	uint8_t BRS : 1;  // Bit Rate Switching
	uint8_t FDF : 1;  // CAN-FD Flag
	uint8_t RES : 2;  // Reserved
	uint8_t FIDX : 7; // Filter Index
	uint8_t ANMF : 1; // Accepted non-matching frame

	// words R2, R3
	uint8_t data[8];

} RxFifoElement;

// Tx FIFO element definition for classic CAN frame
// RM0433 page 2538
typedef struct {
	// word T0
	uint32_t id : 29;
	uint32_t RTR : 1;
	uint32_t XTD : 1;
	uint32_t ESI : 1;

	// word T1
	uint16_t _reserved;
	uint8_t DLC : 4;
	uint8_t BRS : 1;
	uint8_t FDF : 1;
	uint8_t RES : 1;
	uint8_t EFC : 1;
	uint8_t MM;

	// words T2, T3
	uint8_t data[8];

} TxFifoElement;

extern "C"
{
	static int can1_irq(const int irq, void *, void *);
#if UAVCAN_STM32H7_NUM_IFACES > 1
	static int can2_irq(const int irq, void *, void *);
#endif
}

namespace uavcan_stm32h7
{
namespace
{

CanIface *ifaces[UAVCAN_STM32H7_NUM_IFACES] = {
	UAVCAN_NULLPTR
#if UAVCAN_STM32H7_NUM_IFACES > 1
	, UAVCAN_NULLPTR
#endif
};

inline void handleTxInterrupt(uavcan::uint8_t iface_index)
{
	UAVCAN_ASSERT(iface_index < UAVCAN_STM32H7_NUM_IFACES);

	if (ifaces[iface_index] == UAVCAN_NULLPTR) {
		fdcan::Can[iface_index]->IR = FDCAN_IR_TC;
		UAVCAN_ASSERT(0);
		return;
	}

	if (fdcan::Can[iface_index]->IR & FDCAN_IR_TC) {
		fdcan::Can[iface_index]->IR = FDCAN_IR_TC;
		uavcan::uint64_t utc_usec = clock::getUtcUSecFromCanInterrupt();

		if (utc_usec > 0) {
			utc_usec--;
		}

		ifaces[iface_index]->handleTxInterrupt(utc_usec);
	}
}

inline void handleRxInterrupt(uavcan::uint8_t iface_index)
{
	UAVCAN_ASSERT(iface_index < UAVCAN_STM32H7_NUM_IFACES);

	/*
	 * Rx-Related Interrupt Definitions
	 *
	 * FDCAN_IR_RFxN - FIFOx New message       -- Enabled
	 * FDCAN_IR_RFxW - FIFOx Watermark Reached
	 * FDCAN_IR_RFxF - FIFOx Full              -- Enabled
	 * FDCAN_IR_RFxL - FIFOx Message Lost
	 */

	if (ifaces[iface_index] == UAVCAN_NULLPTR) {
		// Bad interface - reset flags and return
		fdcan::Can[iface_index]->IR = FDCAN_IR_RF0N;
		fdcan::Can[iface_index]->IR = FDCAN_IR_RF1N;
		fdcan::Can[iface_index]->IR = FDCAN_IR_RF0F;
		fdcan::Can[iface_index]->IR = FDCAN_IR_RF1F;
		UAVCAN_ASSERT(0);
		return;
	}

	const uint32_t IR = fdcan::Can[iface_index]->IR;

	// Check for our enabled interrupts: FIFO 0
	if ((IR & (FDCAN_IR_RF0N | FDCAN_IR_RF0F)) > 0) {
		fdcan::Can[iface_index]->IR = (FDCAN_IR_RF0N | FDCAN_IR_RF0F);
		ifaces[iface_index]->handleRxInterrupt(0);

	} else if ((IR & (FDCAN_IR_RF1N | FDCAN_IR_RF1F)) > 0) {
		fdcan::Can[iface_index]->IR = (FDCAN_IR_RF1N | FDCAN_IR_RF1F);
		ifaces[iface_index]->handleRxInterrupt(1);

	} else {
		UAVCAN_ASSERT(0);
	}
}

} // namespace

/*
 * CanIface::RxQueue
 */
void CanIface::RxQueue::registerOverflow()
{
	if (overflow_cnt_ < 0xFFFFFFFF) {
		overflow_cnt_++;
	}
}

void CanIface::RxQueue::push(const uavcan::CanFrame &frame, const uint64_t &utc_usec, uavcan::CanIOFlags flags)
{
	buf_[in_].frame    = frame;
	buf_[in_].utc_usec = utc_usec;
	buf_[in_].flags    = flags;
	in_++;

	if (in_ >= capacity_) {
		in_ = 0;
	}

	len_++;

	if (len_ > capacity_) {
		len_ = capacity_;
		registerOverflow();
		out_++;

		if (out_ >= capacity_) {
			out_ = 0;
		}
	}
}

void CanIface::RxQueue::pop(uavcan::CanFrame &out_frame, uavcan::uint64_t &out_utc_usec, uavcan::CanIOFlags &out_flags)
{
	if (len_ > 0) {
		out_frame    = buf_[out_].frame;
		out_utc_usec = buf_[out_].utc_usec;
		out_flags    = buf_[out_].flags;
		out_++;

		if (out_ >= capacity_) {
			out_ = 0;
		}

		len_--;

	} else { UAVCAN_ASSERT(0); }
}

void CanIface::RxQueue::reset()
{
	in_ = 0;
	out_ = 0;
	len_ = 0;
	overflow_cnt_ = 0;
}

/*
 * CanIface
 */

int CanIface::computeTimings(const uavcan::uint32_t target_bitrate, Timings &out_timings)
{
	if (target_bitrate < 1) {
		return -ErrInvalidBitRate;
	}

	/*
	 * Hardware configuration
	 */
	const uavcan::uint32_t pclk = STM32_FDCANCLK;

	static const int MaxBS1 = 16;
	static const int MaxBS2 = 8;

	/*
	 * Ref. "Automatic Baudrate Detection in CANopen Networks", U. Koppe, MicroControl GmbH & Co. KG
	 *      CAN in Automation, 2003
	 *
	 * According to the source, optimal quanta per bit are:
	 *   Bitrate        Optimal Maximum
	 *   1000 kbps      8       10
	 *   500  kbps      16      17
	 *   250  kbps      16      17
	 *   125  kbps      16      17
	 */
	const int max_quanta_per_bit = (target_bitrate >= 1000000) ? 10 : 17;

	UAVCAN_ASSERT(max_quanta_per_bit <= (MaxBS1 + MaxBS2));

	static const int MaxSamplePointLocation = 900;

	/*
	 * Computing (prescaler * BS):
	 *   BITRATE = 1 / (PRESCALER * (1 / PCLK) * (1 + BS1 + BS2))       -- See the Reference Manual
	 *   BITRATE = PCLK / (PRESCALER * (1 + BS1 + BS2))                 -- Simplified
	 * let:
	 *   BS = 1 + BS1 + BS2                                             -- Number of time quanta per bit
	 *   PRESCALER_BS = PRESCALER * BS
	 * ==>
	 *   PRESCALER_BS = PCLK / BITRATE
	 */
	const uavcan::uint32_t prescaler_bs = pclk / target_bitrate;

	/*
	 * Searching for such prescaler value so that the number of quanta per bit is highest.
	 */
	uavcan::uint8_t bs1_bs2_sum = uavcan::uint8_t(max_quanta_per_bit - 1);

	while ((prescaler_bs % (1 + bs1_bs2_sum)) != 0) {
		if (bs1_bs2_sum <= 2) {
			return -ErrInvalidBitRate;          // No solution
		}

		bs1_bs2_sum--;
	}

	const uavcan::uint32_t prescaler = prescaler_bs / (1 + bs1_bs2_sum);

	if ((prescaler < 1U) || (prescaler > 1024U)) {
		return -ErrInvalidBitRate;              // No solution
	}

	/*
	 * Now we have a constraint: (BS1 + BS2) == bs1_bs2_sum.
	 * We need to find the values so that the sample point is as close as possible to the optimal value.
	 *
	 *   Solve[(1 + bs1)/(1 + bs1 + bs2) == 7/8, bs2]  (* Where 7/8 is 0.875, the recommended sample point location *)
	 *   {{bs2 -> (1 + bs1)/7}}
	 *
	 * Hence:
	 *   bs2 = (1 + bs1) / 7
	 *   bs1 = (7 * bs1_bs2_sum - 1) / 8
	 *
	 * Sample point location can be computed as follows:
	 *   Sample point location = (1 + bs1) / (1 + bs1 + bs2)
	 *
	 * Since the optimal solution is so close to the maximum, we prepare two solutions, and then pick the best one:
	 *   - With rounding to nearest
	 *   - With rounding to zero
	 */
	struct BsPair {
		uavcan::uint8_t bs1;
		uavcan::uint8_t bs2;
		uavcan::uint16_t sample_point_permill;

		BsPair() :
			bs1(0),
			bs2(0),
			sample_point_permill(0)
		{ }

		BsPair(uavcan::uint8_t bs1_bs2_sum, uavcan::uint8_t arg_bs1) :
			bs1(arg_bs1),
			bs2(uavcan::uint8_t(bs1_bs2_sum - bs1)),
			sample_point_permill(uavcan::uint16_t(1000 * (1 + bs1) / (1 + bs1 + bs2)))
		{
			UAVCAN_ASSERT(bs1_bs2_sum > arg_bs1);
		}

		bool isValid() const { return (bs1 >= 1) && (bs1 <= MaxBS1) && (bs2 >= 1) && (bs2 <= MaxBS2); }
	};

	// First attempt with rounding to nearest
	BsPair solution(bs1_bs2_sum, uavcan::uint8_t(((7 * bs1_bs2_sum - 1) + 4) / 8));

	if (solution.sample_point_permill > MaxSamplePointLocation) {
		// Second attempt with rounding to zero
		solution = BsPair(bs1_bs2_sum, uavcan::uint8_t((7 * bs1_bs2_sum - 1) / 8));
	}

	/*
	 * Final validation
	 * Helpful Python:
	 * def sample_point_from_btr(x):
	 *     assert 0b0011110010000000111111000000000 & x == 0
	 *     ts2,ts1,brp = (x>>20)&7, (x>>16)&15, x&511
	 *     return (1+ts1+1)/(1+ts1+1+ts2+1)
	 *
	 */
	if ((target_bitrate != (pclk / (prescaler * (1 + solution.bs1 + solution.bs2)))) || !solution.isValid()) {
		UAVCAN_ASSERT(0);
		return -ErrLogic;
	}

	UAVCAN_STM32H7_LOG("Timings: quanta/bit: %d, sample point location: %.1f%%",
			   int(1 + solution.bs1 + solution.bs2), double(solution.sample_point_permill) / 10.);

	out_timings.prescaler = uavcan::uint16_t(prescaler - 1U);
	out_timings.sjw = 0;                                        // Which means one
	out_timings.bs1 = uavcan::uint8_t(solution.bs1 - 1);
	out_timings.bs2 = uavcan::uint8_t(solution.bs2 - 1);
	return 0;
}

uavcan::int16_t CanIface::send(const uavcan::CanFrame &frame, uavcan::MonotonicTime tx_deadline,
			       uavcan::CanIOFlags flags)
{
	if (frame.isErrorFrame() || frame.dlc > 8) {
		return -ErrUnsupportedFrame;
	}

	/*
	 * Normally we should perform the same check as in @ref canAcceptNewTxFrame(), because
	 * it is possible that the highest-priority frame between select() and send() could have been
	 * replaced with a lower priority one due to TX timeout. But we don't do this check because:
	 *
	 *  - It is a highly unlikely scenario.
	 *
	 *  - Frames do not timeout on a properly functioning bus. Since frames do not timeout, the new
	 *    frame can only have higher priority, which doesn't break the logic.
	 *
	 *  - If high-priority frames are timing out in the TX FIFO, there's probably a lot of other
	 *    issues to take care of before this one becomes relevant.
	 *
	 *  - It takes CPU time. Not just CPU time, but critical section time, which is expensive.
	 */
	CriticalSectionLocker lock;

	// First, check if there are any slots available in the FIFO
	if ((can_->TXFQS & FDCAN_TXFQS_TFQF) > 0) {
		// Tx FIFO is full
		return 0;
	}

	// Next, get the next available FIFO index from the controller
	const uint8_t index = (can_->TXFQS & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos;

	// Now, we can copy the CAN frame to the FIFO (in message RAM)
	uint32_t *txbuf  = (uint32_t *)(message_ram_.TxFIFOSA + (index * FIFO_ELEMENT_SIZE * WORD_LENGTH));

	// Copy the ID; special case for standard ID frames
	if (frame.isExtended()) {
		txbuf[0] = (frame.id & fdcan::EXID_MASK) | fdcan::IDE;

	} else {
		// Standard ID frames must be entered into bits [28:18]
		txbuf[0] = (frame.id << fdcan::T0_STID_Pos) & fdcan::STID_MASK;
	}

	if (frame.isRemoteTransmissionRequest()) {
		txbuf[0] |= fdcan::RTR;
	}

	if (frame.isErrorFrame()) {
		txbuf[0] |= fdcan::ESI;
	}

	txbuf[1] = (frame.dlc << fdcan::T1_DLC_Pos);

	txbuf[1] &= ~(1 << fdcan::T1_FDF_Pos);   // Classic CAN frame, not CAN-FD
	txbuf[1] &= ~(1 << fdcan::T1_BRS_Pos);   // No bitrate switching
	txbuf[1] &= ~(1 << fdcan::T1_EFC_Pos);   // Don't store Tx events
	txbuf[1] |= (index << fdcan::T1_MM_Pos); // Marker for our use; just give it the FIFO index

	// Store the data bytes
	txbuf[2] = (uavcan::uint32_t(frame.data[3]) << 24) |
		   (uavcan::uint32_t(frame.data[2]) << 16) |
		   (uavcan::uint32_t(frame.data[1]) << 8)  |
		   (uavcan::uint32_t(frame.data[0]) << 0);
	txbuf[3] = (uavcan::uint32_t(frame.data[7]) << 24) |
		   (uavcan::uint32_t(frame.data[6]) << 16) |
		   (uavcan::uint32_t(frame.data[5]) << 8)  |
		   (uavcan::uint32_t(frame.data[4]) << 0);

	// Submit the transmission request for this element
	can_->TXBAR = 1 << index;

	/*
	 * Registering the pending transmission so we can track its deadline and loopback it as needed
	 */
	TxItem &txi = pending_tx_[index];
	txi.deadline       = tx_deadline;
	txi.frame          = frame;
	txi.loopback       = (flags & uavcan::CanIOFlagLoopback) != 0;
	txi.abort_on_error = (flags & uavcan::CanIOFlagAbortOnError) != 0;
	txi.index          = index;
	txi.pending        = true;

	return 1;
}

uavcan::int16_t CanIface::receive(uavcan::CanFrame &out_frame, uavcan::MonotonicTime &out_ts_monotonic,
				  uavcan::UtcTime &out_ts_utc, uavcan::CanIOFlags &out_flags)
{
	out_ts_monotonic = clock::getMonotonic();  // High precision is not required for monotonic timestamps
	uavcan::uint64_t utc_usec = 0;
	{
		CriticalSectionLocker lock;

		if (rx_queue_.getLength() == 0) {
			return 0;
		}

		rx_queue_.pop(out_frame, utc_usec, out_flags);
	}
	out_ts_utc = uavcan::UtcTime::fromUSec(utc_usec);
	return 1;
}

uavcan::int16_t CanIface::configureFilters(const uavcan::CanFilterConfig *filter_configs,
		uavcan::uint16_t num_configs)
{
	/*
	 * The FDCAN controller handles standard ID and extended ID filters separately.
	 * We must scan through the requested filter configurations, and group them by
	 * ID type.  We can then setup the filters and assign message RAM.
	 */

	// Filter config registers are protected; write access is only available
	// when bit CCE and bit INIT of FDCAN_CCCR register are set to 1.

	// CriticalSectionLocker lock;

	// // Request Init mode, then wait for completion
	// can_->CCCR |= FDCAN_CCCR_INIT;
	// while ((can_->CCCR & FDCAN_CCCR_INIT) == 0) {}

	// // Configuration Chane Enable
	// can_->CCCR |= FDCAN_CCCR_CCE;

	// /// TODO -------------------------

	// // Leave Init mode
	// can_->CCCR &= ~FDCAN_CCCR_INIT;

	return 0;
}

bool CanIface::waitCCCRBitStateChange(uint32_t mask, bool target_state)
{
#if UAVCAN_STM32_NUTTX
	const unsigned Timeout = 1000;
#else
	const unsigned Timeout = 2000000;
#endif

	for (unsigned wait_ack = 0; wait_ack < Timeout; wait_ack++) {
		const bool state = (can_->CCCR & mask) != 0;

		if (state == target_state) {
			return true;
		}

#if UAVCAN_STM32_NUTTX
		::usleep(1000);
#endif
	}

	return false;
}

int CanIface::init(const uavcan::uint32_t bitrate, const OperatingMode mode)
{
	/*
	 * Wake up the device and enable configuration changes
	 */
	{
		CriticalSectionLocker lock;

		// Exit Power-down / Sleep mode, then wait for acknowledgement
		can_->CCCR &= ~FDCAN_CCCR_CSR;

		if (!waitCCCRBitStateChange(FDCAN_CCCR_CSA, false)) {
			UAVCAN_STM32H7_LOG("CCCR CCCR_CSA not cleared");
			can_->CCCR |= FDCAN_CCCR_INIT;
			return -ErrCCCrCSANotCleared;
		}


		// Request Init mode, then wait for completion
		can_->CCCR |= FDCAN_CCCR_INIT;

		if (!waitCCCRBitStateChange(FDCAN_CCCR_INIT, true)) {
			UAVCAN_STM32H7_LOG("CCCR FDCAN_CCCR_INIT not set");
			return -ErrCCCrINITNotSet;
		}

		// Configuration Changes Enable.  Can only be set during Init mode;
		// cleared when INIT bit is cleared.
		can_->CCCR |= FDCAN_CCCR_CCE;

		// Disable interrupts while we configure the hardware
		can_->IE = 0;

	} // End Critcal section

	/*
	 * Object state - interrupts are disabled, so it's safe to modify it now
	 */
	rx_queue_.reset();
	error_cnt_ = 0;
	served_aborts_cnt_ = 0;
	uavcan::fill_n(pending_tx_, NumTxMailboxes, TxItem());
	peak_tx_mailbox_index_ = 0;
	had_activity_ = false;

	/*
	 * CAN timings for this bitrate
	 */
	Timings timings;
	const int timings_res = computeTimings(bitrate, timings);

	if (timings_res < 0) {
		can_->CCCR &= ~FDCAN_CCCR_INIT;
		return timings_res;
	}

	UAVCAN_STM32H7_LOG("Timings: presc=%u sjw=%u bs1=%u bs2=%u",
			   unsigned(timings.prescaler), unsigned(timings.sjw), unsigned(timings.bs1), unsigned(timings.bs2));

	/*
	 * Set bit timings and prescalers (Nominal and Data bitrates)
	 */

	//  We're not using CAN-FD, so set same timings for both
	can_->NBTP = ((timings.sjw << FDCAN_NBTP_NSJW_Pos)   |
		      (timings.bs1 << FDCAN_NBTP_NTSEG1_Pos) |
		      (timings.bs2 << FDCAN_NBTP_TSEG2_Pos)  |
		      (timings.prescaler << FDCAN_NBTP_NBRP_Pos));

	can_->DBTP = ((timings.sjw << FDCAN_DBTP_DSJW_Pos)   |
		      (timings.bs1 << FDCAN_DBTP_DTSEG1_Pos) |
		      (timings.bs2 << FDCAN_DBTP_DTSEG2_Pos)  |
		      (timings.prescaler << FDCAN_DBTP_DBRP_Pos));

	/*
	 * Operation Configuration
	 */

	// Disable CAN-FD communications ("classic" CAN only)
	can_->CCCR &= ~FDCAN_CCCR_FDOE;

	// Disable Time Triggered (TT) operation -- TODO (must use TTCAN_TypeDef)
	//ttcan_->TTOCF &= ~FDCAN_TTOCF_OM

	/*
	 * Configure Interrupts
	 */

	// Clear all interrupt flags
	// Note: A flag is cleared by writing a 1 to the corresponding bit position
	can_->IR = 0xFFFFFFFF; //FDCAN_IR_MASK;

	// Enable relevant interrupts
	can_->IE = FDCAN_IE_TCE     // Transmit Complete
		   | FDCAN_IE_RF0NE   // Rx FIFO 0 new message
		   | FDCAN_IE_RF0FE   // Rx FIFO 0 FIFO full
		   | FDCAN_IE_RF1NE   // Rx FIFO 1 new message
		   | FDCAN_IE_RF1FE;  // Rx FIFO 1 FIFO full

	// Keep Rx interrupts on Line 0; move Tx to Line 1
	can_->ILS = FDCAN_ILS_TCL;  // TC interrupt on line 1

	// Enable Tx buffer transmission interrupt
	can_->TXBTIE = FDCAN_TXBTIE_TIE;

	// Enable both interrupt lines
	can_->ILE = FDCAN_ILE_EINT0 | FDCAN_ILE_EINT1;

	/*
	 * Configure Message RAM
	 *
	 * The available 2560 words (10 kiB) of RAM are shared between both FDCAN
	 * interfaces. It is up to us to ensure each interface has its own non-
	 * overlapping region of RAM assigned to it by properly assignin the start and
	 * end addresses for all regions of RAM.
	 *
	 * We will give each interface half of the available RAM.
	 *
	 * Rx buffers are only used in conjunction with acceptance filters; we don't
	 * have any specific need for this, so we will only use Rx FIFOs.
	 *
	 * Each FIFO can hold up to 64 elements, where each element (for a classic CAN
	 * 2.0B frame) is up to 4 words long (8 bytes data + header bits)
	 *
	 * Let's make use of the full 64 FIFO elements for FIFO0.  We have no need to
	 * separate messages between FIFO0 and FIFO1, so ignore FIFO1 for simplicity.
	 *
	 * Note that the start addresses given to FDCAN are in terms of _words_, not
	 * bytes, so when we go to read/write to/from the message RAM, there will be a
	 * factor of 4 necessary in the address relative to the SA register values.
	 */

	// Location of this interface's message RAM - address in CPU memory address
	// and relative address (in words) used for configuration
	const uint32_t iface_ram_base = (2560 / 2) * self_index_;
	const uint32_t gl_ram_base = SRAMCAN_BASE;
	uint32_t ram_offset = iface_ram_base;

	// Standard ID Filters: Allow space for 128 filters (128 words)
	const uint8_t n_stdid = 128;
	message_ram_.StdIdFilterSA = gl_ram_base + ram_offset * WORD_LENGTH;
	can_->SIDFC = ((n_stdid << FDCAN_SIDFC_LSS_Pos)
		       | ram_offset << FDCAN_SIDFC_FLSSA_Pos);
	ram_offset += n_stdid;

	// Extended ID Filters: Allow space for 128 filters (128 words)
	const uint8_t n_extid = 128;
	message_ram_.ExtIdFilterSA = gl_ram_base + ram_offset * WORD_LENGTH;
	can_->XIDFC = ((n_extid << FDCAN_XIDFC_LSE_Pos)
		       | ram_offset << FDCAN_XIDFC_FLESA_Pos);
	ram_offset += n_extid;

	// Set size of each element in the Rx/Tx buffers and FIFOs
	can_->RXESC = 0; // 8 byte space for every element (Rx buffer, FIFO1, FIFO0)
	can_->TXESC = 0; // 8 byte space for every element (Tx buffer)

	// Rx FIFO0 (64 elements max)
	const uint8_t n_fifo0 = 64;
	message_ram_.RxFIFO0SA = gl_ram_base + ram_offset * WORD_LENGTH;
	can_->RXF0C = ram_offset << FDCAN_RXF0C_F0SA_Pos;
	can_->RXF0C |= n_fifo0 << FDCAN_RXF0C_F0S_Pos;
	ram_offset += n_fifo0 * FIFO_ELEMENT_SIZE;

	// Set Tx FIFO size (32 elements max)
	message_ram_.TxFIFOSA = gl_ram_base + ram_offset * WORD_LENGTH;
	can_->TXBC = 32U << FDCAN_TXBC_TFQS_Pos;
	can_->TXBC &= ~FDCAN_TXBC_TFQM; // Use FIFO
	can_->TXBC |= ram_offset << FDCAN_TXBC_TBSA_Pos;

	/*
	 * Default filter configuration
	 *
	 * Accept all messages into Rx FIFO0 by default
	 */
	can_->GFC &= ~FDCAN_GFC_ANFS;  // Accept non-matching stdid frames into FIFO0
	can_->GFC &= ~FDCAN_GFC_ANFE;  // Accept non-matching extid frames into FIFO0

	/*
	 * Exit Initialization mode
	 */
	can_->CCCR &= ~FDCAN_CCCR_INIT;

	return 0;
}

void CanIface::handleTxInterrupt(const uavcan::uint64_t utc_usec)
{
	// Update counters for successful transmissions
	// Process loopback messages
	for (uint8_t i = 0; i < NumTxMailboxes; i++) {
		if ((can_->TXBTO & (1 << i)) > 0) {
			// Transmission Occurred in buffer i
			TxItem &txi = pending_tx_[i];

			if (txi.loopback && txi.pending) {
				rx_queue_.push(txi.frame, utc_usec, uavcan::CanIOFlagLoopback);
			}

			txi.pending = false;
		}
	}

	pollErrorFlagsFromISR();
}

void CanIface::handleRxInterrupt(uavcan::uint8_t fifo_index)
{
	UAVCAN_ASSERT(fifo_index < 2);

	// Bitwise register definitions are the same for FIFO 0/1
	constexpr uint32_t FDCAN_RXFnC_FnS      = FDCAN_RXF0C_F0S;  // Rx FIFO Size
	constexpr uint32_t FDCAN_RXFnS_RFnL     = FDCAN_RXF0S_RF0L; // Rx Message Lost
	constexpr uint32_t FDCAN_RXFnS_FnFL     = FDCAN_RXF0S_F0FL; // Rx FIFO Fill Level
	constexpr uint32_t FDCAN_RXFnS_FnGI     = FDCAN_RXF0S_F0GI; // Rx FIFO Get Index
	constexpr uint32_t FDCAN_RXFnS_FnGI_Pos = FDCAN_RXF0S_F0GI_Pos;
	//constexpr uint32_t FDCAN_RXFnS_FnPI     = FDCAN_RXF0S_F0PI; // Rx FIFO Put Index
	//constexpr uint32_t FDCAN_RXFnS_FnPI_Pos = FDCAN_RXF0S_F0PI_Pos;
	//constexpr uint32_t FDCAN_RXFnS_FnF      = FDCAN_RXF0S_F0F; // Rx FIFO Full

	volatile uint32_t *const RXFnC = (fifo_index == 0) ? &(can_->RXF0C) : &(can_->RXF1C);
	volatile uint32_t *const RXFnS = (fifo_index == 0) ? &(can_->RXF0S) : &(can_->RXF1S);
	volatile uint32_t *const RXFnA = (fifo_index == 0) ? &(can_->RXF0A) : &(can_->RXF1A);

	bool had_error = false;

	// Check number of elements in message RAM allocated to this FIFO
	if ((*RXFnC & FDCAN_RXFnC_FnS) == 0) {
		UAVCAN_ASSERT(0);
		return;
	}

	// Check for message lost; count an error
	if ((*RXFnS & FDCAN_RXFnS_RFnL) != 0) {
		error_cnt_++;
	}

	// Check number of elements available (fill level)
	const uint8_t n_elem = (*RXFnS & FDCAN_RXFnS_FnFL);

	if (n_elem == 0) {
		// No elements available?
		UAVCAN_ASSERT(0);
		return;
	}

	while ((*RXFnS & FDCAN_RXFnS_FnFL) > 0) {
		// Copy the message from message RAM
		uavcan::uint64_t utc_usec = clock::getUtcUSecFromCanInterrupt();
		uavcan::CanFrame frame;

		const uint8_t index = (*RXFnS & FDCAN_RXFnS_FnGI) >> FDCAN_RXFnS_FnGI_Pos;

		RxFifoElement *rx = (RxFifoElement *)(message_ram_.RxFIFO0SA + (index * FIFO_ELEMENT_SIZE * WORD_LENGTH));

		// Note that we must shift Standard IDs to the lowest bit range of ID
		if (rx->XTD) {
			frame.id = (rx->id & fdcan::EXID_MASK) & uavcan::CanFrame::MaskExtID;
			frame.id |= uavcan::CanFrame::FlagEFF;

		} else {
			frame.id = (rx->id >> 18) & uavcan::CanFrame::MaskStdID;
		}

		if (rx->RTR) {
			frame.id |= uavcan::CanFrame::FlagRTR;
		}

		if (rx->ESI) {
			frame.id |= uavcan::CanFrame::FlagERR;
		}

		frame.dlc = rx->DLC;

		for (uint8_t i = 0; i < 8; i++) {
			frame.data[i] = rx->data[i];
		}

		// Acknowledge receipt of this FIFO element
		*RXFnA = index;

		// Push the frame into the application queue
		rx_queue_.push(frame, utc_usec, 0);

		had_activity_ = true;
	}

	if (had_error) {
		error_cnt_++;
	}

	update_event_.signalFromInterrupt();

	pollErrorFlagsFromISR();
}

void CanIface::pollErrorFlagsFromISR()
{
	// Read CAN Error Logging counter (This also resets the error counter)
	const uavcan::uint8_t cel = ((can_->ECR & FDCAN_ECR_CEL) >> FDCAN_ECR_CEL_Pos);

	if (cel > 0) {

		// Serve abort requests
		for (uint8_t i = 0; i < NumTxMailboxes; i++) {
			TxItem &txi = pending_tx_[i];

			if (txi.abort_on_error && ((1 << txi.index) & can_->TXBRP)) {
				// Request to Cancel Tx item
				can_->TXBCR = (1 << txi.index);
				txi.pending = false;
				error_cnt_++;
				served_aborts_cnt_++;
			}
		}
	}
}

void CanIface::discardTimedOutTxMailboxes(uavcan::MonotonicTime current_time)
{
	CriticalSectionLocker lock;

	for (uint8_t i = 0; i < NumTxMailboxes; i++) {
		TxItem &txi = pending_tx_[i];

		if (((1 << txi.index) & can_->TXBRP) && txi.deadline < current_time) {
			// Request to Cancel Tx item
			can_->TXBCR = (1 << txi.index);
			txi.pending = false;
			error_cnt_++;
		}
	}
}

bool CanIface::canAcceptNewTxFrame(const uavcan::CanFrame &frame) const
{
	// Check that we even _have_ a Tx FIFO allocated
	if ((can_->TXBC & FDCAN_TXBC_TFQS) == 0) {
		// Your FIFO size is 0, you did something wrong
		return false;
	}

	// Check if the Tx FIFO is full
	if ((can_->TXFQS & FDCAN_TXFQS_TFQF) == FDCAN_TXFQS_TFQF) {
		// Sorry, out of room, try back later
		return false;
	}

	return true;
}

bool CanIface::isRxBufferEmpty() const
{
	CriticalSectionLocker lock;
	return rx_queue_.getLength() == 0;
}

uavcan::uint64_t CanIface::getErrorCount() const
{
	CriticalSectionLocker lock;
	return error_cnt_ + rx_queue_.getOverflowCount();
}

unsigned CanIface::getRxQueueLength() const
{
	CriticalSectionLocker lock;
	return rx_queue_.getLength();
}

bool CanIface::hadActivity()
{
	CriticalSectionLocker lock;
	const bool ret = had_activity_;
	had_activity_ = false;
	return ret;
}

/*
 * CanDriver
 */
uavcan::CanSelectMasks CanDriver::makeSelectMasks(const uavcan::CanFrame * (& pending_tx)[uavcan::MaxCanIfaces]) const
{
	uavcan::CanSelectMasks msk;

	for (uint8_t i = 0; i < num_ifaces_; i++) {
		msk.read |= (ifaces[i]->isRxBufferEmpty() ? 0 : 1) << i;

		if (pending_tx[i] != UAVCAN_NULLPTR) {
			msk.write |= (ifaces[i]->canAcceptNewTxFrame(*pending_tx[i]) ? 1 : 0) << i;
		}
	}

	return msk;
}

bool CanDriver::hasReadableInterfaces() const
{
	for (uint8_t i = 0; i < num_ifaces_; i++) {
		if (!ifaces[i]->isRxBufferEmpty()) {
			return true;
		}
	}

	return false;
}

uavcan::int16_t CanDriver::select(uavcan::CanSelectMasks &inout_masks,
				  const uavcan::CanFrame * (& pending_tx)[uavcan::MaxCanIfaces],
				  const uavcan::MonotonicTime blocking_deadline)
{
	const uavcan::CanSelectMasks in_masks = inout_masks;
	const uavcan::MonotonicTime time = clock::getMonotonic();

	if0_.discardTimedOutTxMailboxes(time);              // Check TX timeouts - this may release some TX slots
	{
		CriticalSectionLocker cs_locker;
		if0_.pollErrorFlagsFromISR();
	}

#if UAVCAN_STM32H7_NUM_IFACES > 1
	if1_.discardTimedOutTxMailboxes(time);
	{
		CriticalSectionLocker cs_locker;
		if1_.pollErrorFlagsFromISR();
	}
#endif

	inout_masks = makeSelectMasks(pending_tx);          // Check if we already have some of the requested events

	if ((inout_masks.read  & in_masks.read)  != 0 ||
	    (inout_masks.write & in_masks.write) != 0) {
		return 1;
	}

	(void)update_event_.wait(blocking_deadline - time); // Block until timeout expires or any iface updates
	inout_masks = makeSelectMasks(pending_tx);  // Return what we got even if none of the requested events are set
	return 1;                                   // Return value doesn't matter as long as it is non-negative
}


void CanDriver::initOnce()
{
	/*
	 * FDCAN1, FDCAN2 - Enable peripheral, clock
	 */
	{
		CriticalSectionLocker lock;
#if UAVCAN_STM32H7_NUTTX
		modifyreg32(STM32_RCC_APB1HENR,  0, RCC_APB1HENR_FDCANEN);
		modifyreg32(STM32_RCC_APB1HRSTR, 0, RCC_APB1HRSTR_FDCANRST);
		modifyreg32(STM32_RCC_APB1HRSTR, RCC_APB1HRSTR_FDCANRST, 0);
#else
		RCC->APB1HENR  |=  RCC_APB1HENR_FDCANEN;
		RCC->APB1HRSTR |=  RCC_APB1HRSTR_FDCANRST;
		RCC->APB1HRSTR &= ~RCC_APB1HRSTR_FDCANRST;
#endif
	}

	/*
	 * IRQ
	 */
#if UAVCAN_STM32H7_NUTTX
# define IRQ_ATTACH(irq, handler)                          \
	{                                                      \
		const int res = irq_attach(irq, handler, NULL);    \
		(void)res;                                         \
		assert(res >= 0);                                  \
		up_enable_irq(irq);                                \
	}
	IRQ_ATTACH(STM32_IRQ_FDCAN1_0, can1_irq);
	IRQ_ATTACH(STM32_IRQ_FDCAN1_1, can1_irq);
# if UAVCAN_STM32H7_NUM_IFACES > 1
	IRQ_ATTACH(STM32_IRQ_FDCAN2_0, can2_irq);
	IRQ_ATTACH(STM32_IRQ_FDCAN2_1, can2_irq);
# endif
# undef IRQ_ATTACH
#endif
}

int CanDriver::init(const uavcan::uint32_t bitrate, const CanIface::OperatingMode mode,
		    const uavcan::uint32_t enabledInterfaces)
{
	int res = 0;

	UAVCAN_STM32H7_LOG("Bitrate %lu mode %d", static_cast<unsigned long>(bitrate), static_cast<int>(mode));

	static bool initialized_once = false;

	if (!initialized_once) {
		initialized_once = true;
		UAVCAN_STM32H7_LOG("First initialization");
		initOnce();
	}

	/*
	 * FDCAN1
	 */
	if (enabledInterfaces & 1) {
		UAVCAN_STM32H7_LOG("Initing iface 0...");
		ifaces[0] = &if0_;                          // This link must be initialized first,
		res = if0_.init(bitrate, mode);             // otherwise an IRQ may fire while the interface is not linked yet;

		if (res < 0) {                              // a typical race condition.
			UAVCAN_STM32H7_LOG("Iface 0 init failed %i", res);
			ifaces[0] = UAVCAN_NULLPTR;
			goto fail;
		}
	}

	/*
	 * FDCAN2
	 */
#if UAVCAN_STM32H7_NUM_IFACES > 1

	if (enabledInterfaces & 2) {
		UAVCAN_STM32H7_LOG("Initing iface 1...");
		ifaces[1] = &if1_;                          // Same thing here.
		res = if1_.init(bitrate, mode);

		if (res < 0) {
			UAVCAN_STM32H7_LOG("Iface 1 init failed %i", res);
			ifaces[1] = UAVCAN_NULLPTR;
			goto fail;
		}
	}

#endif

	UAVCAN_STM32H7_LOG("CAN drv init OK");
	UAVCAN_ASSERT(res >= 0);
	return res;

fail:
	UAVCAN_STM32H7_LOG("CAN drv init failed %i", res);
	UAVCAN_ASSERT(res < 0);
	return res;
}

CanIface *CanDriver::getIface(uavcan::uint8_t iface_index)
{
	if (iface_index < UAVCAN_STM32H7_NUM_IFACES) {
		return ifaces[iface_index];
	}

	return UAVCAN_NULLPTR;
}

bool CanDriver::hadActivity()
{
	bool ret = if0_.hadActivity();
#if UAVCAN_STM32H7_NUM_IFACES > 1
	ret |= if1_.hadActivity();
#endif
	return ret;
}

} // namespace uavcan_stm32

/*
 * Interrupt handlers
 */
extern "C"
{

#if UAVCAN_STM32H7_NUTTX

	static int can1_irq(const int irq, void *, void *)
	{
		if (irq == STM32_IRQ_FDCAN1_0) {
			// We've put only Rx interrupts on Line 0
			uavcan_stm32h7::handleRxInterrupt(0);

		} else if (irq == STM32_IRQ_FDCAN1_1) {
			// And only Tx interrupts on Line 1
			uavcan_stm32h7::handleTxInterrupt(0);

		} else {
			PANIC();
		}

		return 0;
	}

# if UAVCAN_STM32H7_NUM_IFACES > 1

	static int can2_irq(const int irq, void *, void *)
	{
		if (irq == STM32_IRQ_FDCAN2_0) {
			// We've put only Rx interrupts on Line 0
			uavcan_stm32h7::handleRxInterrupt(1);

		} else if (irq == STM32_IRQ_FDCAN2_1) {
			// And only Tx interrupts on Line 1
			uavcan_stm32h7::handleTxInterrupt(1);

		} else {
			PANIC();
		}

		return 0;
	}

# endif

#else // UAVCAN_STM32H7_NUTTX

#if !defined(FDCAN1_IT0_IRQHandler) ||\
    !defined(FDCAN1_IT1_IRQHandler)
# error "Misconfigured build"
#endif

	UAVCAN_STM32H7_IRQ_HANDLER(FDCAN1_IT0_IRQHandler);
	UAVCAN_STM32H7_IRQ_HANDLER(FDCAN1_IT0_IRQHandler)
	{
		UAVCAN_STM32H7_IRQ_PROLOGUE();
		uavcan_stm32h7::handleInterrupt(0, 0);
		UAVCAN_STM32H7_IRQ_EPILOGUE();
	}

	UAVCAN_STM32H7_IRQ_HANDLER(FDCAN1_IT1_IRQHandler);
	UAVCAN_STM32H7_IRQ_HANDLER(FDCAN1_IT1_IRQHandler)
	{
		UAVCAN_STM32H7_IRQ_PROLOGUE();
		uavcan_stm32h7::handleInterrupt(0, 1);
		UAVCAN_STM32H7_IRQ_EPILOGUE();
	}

# if UAVCAN_STM32H7_NUM_IFACES > 1

#if !defined(FDCAN2_IT0_IRQHandler) || \
	!defined(FDCAN2_IT1_IRQHandler)
# error "Misconfigured build"
#endif

	UAVCAN_STM32H7_IRQ_HANDLER(FDCAN2_IT0_IRQHandler);
	UAVCAN_STM32H7_IRQ_HANDLER(FDCAN2_IT0_IRQHandler)
	{
		UAVCAN_STM32H7_IRQ_PROLOGUE();
		uavcan_stm32h7::handleInterrupt(1, 0);
		UAVCAN_STM32H7_IRQ_EPILOGUE();
	}

	UAVCAN_STM32H7_IRQ_HANDLER(FDCAN2_IT1_IRQHandler);
	UAVCAN_STM32H7_IRQ_HANDLER(FDCAN2_IT1_IRQHandler)
	{
		UAVCAN_STM32H7_IRQ_PROLOGUE();
		uavcan_stm32h7::handleInterrupt(1, 1);
		UAVCAN_STM32H7_IRQ_EPILOGUE();
	}

# endif
#endif // UAVCAN_STM32H7_NUTTX

} // extern "C"
