/*
 * Copyright (C) 2014, 2018 Pavel Kirienko <pavel.kirienko@gmail.com>
 * Kinetis Port Author David Sidrane <david_s5@nscdg.com>
 */

#include <cassert>
#include <cstring>
#include <uavcan_kinetis/can.hpp>
#include <uavcan_kinetis/clock.hpp>
#include "internal.hpp"

#if UAVCAN_KINETIS_NUTTX
# include <nuttx/arch.h>
# include <nuttx/irq.h>
# include <arch/board/board.h>
#else
# error "Unknown OS"
#endif


namespace uavcan_kinetis
{
extern "C" {
	static int can0_irq(const int irq, void *, void *args);
#if UAVCAN_KINETIS_NUM_IFACES > 1
	static int can1_irq(const int irq, void *, void *args);
#endif
}
namespace flexcan
{
const uavcan::uint32_t OSCERCLK = BOARD_EXTAL_FREQ;
const uavcan::uint32_t busclck  = BOARD_BUS_FREQ;
const uavcan::uint8_t CLOCKSEL  = 0;     // Select OSCERCLK
const uavcan::uint32_t canclk   = CLOCKSEL ? busclck : OSCERCLK;     // Is Clock
const uavcan::uint8_t useFIFO = 1;     // Fifo is enabled
const uavcan::uint32_t numberFIFOFilters = flexcan::CTRL2_RFFN_16MB;     // 16 Rx FIFO filters
}
namespace
{

CanIface *ifaces[UAVCAN_KINETIS_NUM_IFACES] = {
	UAVCAN_NULLPTR
#if UAVCAN_KINETIS_NUM_IFACES > 1
	, UAVCAN_NULLPTR
#endif
};

inline void handleTxInterrupt(CanIface *can, uavcan::uint32_t iflags1)
{
	UAVCAN_ASSERT(iface_index < UAVCAN_KINETIS_NUM_IFACES);
	uavcan::uint64_t utc_usec = clock::getUtcUSecFromCanInterrupt();

	if (utc_usec > 0) {
		utc_usec--;
	}

	can->handleTxInterrupt(iflags1, utc_usec);
}

inline void handleRxInterrupt(CanIface *can, uavcan::uint32_t iflags1)
{
	uavcan::uint64_t utc_usec = clock::getUtcUSecFromCanInterrupt();

	if (utc_usec > 0) {
		utc_usec--;
	}

	can->handleRxInterrupt(iflags1, utc_usec);
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

	} else {
		UAVCAN_ASSERT(0);
	}
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
	 * From FlexCAN Bit Timing Calculation by: Petr Stancik TIC
	 *  buadrate = 1 / tNBT  -The tNBT represents a period of the Nominal Bit Time (NBT).
	 *                  The NBT is separated into four non-overlaping segments,
	 *                  SYNC_SEG, PROP_SEG,PHASE_SEG1 and PHASE_SEG2. Each of
	 *                  these segments is an integer multiple of Time Quantum tQ
	 *  tNBT= tQ+PROP_SEG* tQ + PHASE_SEG1* tQ + PHASE_SEG2* tQ = NBT * tQ
	 *  tQ = 1/bitrate = 1/[canclk /(PRESDIV+1)].
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
	 * NBT * Prescaler = canclk / baud rate.
	 *
	 */

	const uavcan::uint32_t nbt_prescaler = flexcan::canclk / target_bitrate;
	const int max_quanta_per_bit = 17;

	/*
	 * Searching for such prescaler value so that the number of quanta per bit is highest.
	 */

	/* tNBT - tQ = PROP_SEG* tQ + PHASE_SEG1* tQ + PHASE_SEG2* tQ = NBT * tQ - tQ */

	for (uavcan::uint32_t prescaler = 1; prescaler < 256; prescaler++) {
		if (prescaler > nbt_prescaler) {
			return -ErrInvalidBitRate;             // No solution
		}

		if ((0 == nbt_prescaler % prescaler) && (nbt_prescaler / prescaler) < max_quanta_per_bit) {
			out_timings.prescaler = prescaler;
			break;
		}
	}

	const uavcan::uint32_t NBT = nbt_prescaler / out_timings.prescaler;

	/* Choose a reasonable and some what arbitrary value for Propseg  */

	out_timings.propseg = 5;


	/* Ideal sampling point  = 87.5% given by (SYNC_SEG + PROP_SEG + PHASE_SEG1) / (PHASE_SEG2) */

	uavcan::uint32_t sp = (7 * NBT) / 8;

	out_timings.pseg1 = sp - 1 - out_timings.propseg;
	out_timings.pseg2 = NBT - (1 + out_timings.pseg1 + out_timings.propseg);
	out_timings.rjw = uavcan::min((uavcan::uint8_t) 4, out_timings.pseg2);

	return ((out_timings.pseg1 <= 8) && (out_timings.pseg2 <= 8) && (out_timings.propseg <= 8)) ?  0 :
	       -ErrInvalidBitRate;
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
	 *  - If high-priority frames are timing out in the TX queue, there's probably a lot of other
	 *    issues to take care of before this one becomes relevant.
	 *
	 *  - It takes CPU time. Not just CPU time, but critical section time, which is expensive.
	 */
	CriticalSectionLocker lock;

	/*
	 * Seeking for an empty slot
	 */

	uavcan::uint32_t mbi = 0;

	if ((can_->ESR2 & (flexcan::ESR2_IMB | flexcan::ESR2_VPS)) == (flexcan::ESR2_IMB | flexcan::ESR2_VPS)) {
		mbi = (can_->ESR2 & flexcan::ESR2_LPTM_MASK) >> flexcan::ESR2_LPTM_SHIFT;
		mbi -= flexcan::NumMBinFiFoAndFilters;
	}

	uavcan::uint32_t mb_bit = 1 << (flexcan::NumMBinFiFoAndFilters + mbi);

	while (mbi < NumTxMesgBuffers) {
		if (can_->MB[flexcan::NumMBinFiFoAndFilters + mbi].mb.CS.code != flexcan::TxMbDataOrRemote) {
			can_->IFLAG1 = mb_bit;
			break;
		}

		mb_bit <<= 1;
		mbi++;
	}

	if (mbi == NumTxMesgBuffers) {
		return 0;       // No transmission for you!
	}

	peak_tx_mailbox_index_ = uavcan::max(peak_tx_mailbox_index_, (uavcan::uint8_t) mbi);   // Statistics

	flexcan::MBcsType cs;
	cs.code = flexcan::TxMbDataOrRemote;
	flexcan::MessageBufferType &mb = can_->MB[flexcan::NumMBinFiFoAndFilters + mbi].mb;
	mb.CS.code = flexcan::TxMbInactive;

	if (frame.isExtended()) {
		cs.ide = 1;
		mb.ID.ext = frame.id & uavcan::CanFrame::MaskExtID;

	} else {
		mb.ID.std = frame.id & uavcan::CanFrame::MaskStdID;
	}

	cs.rtr = frame.isRemoteTransmissionRequest();

	cs.dlc = frame.dlc;
	mb.data.b0 = frame.data[0];
	mb.data.b1 = frame.data[1];
	mb.data.b2 = frame.data[2];
	mb.data.b3 = frame.data[3];
	mb.data.b4 = frame.data[4];
	mb.data.b5 = frame.data[5];
	mb.data.b6 = frame.data[6];
	mb.data.b7 = frame.data[7];

	/*
	 * Registering the pending transmission so we can track its deadline and loopback it as needed
	 */
	TxItem &txi = pending_tx_[mbi];
	txi.deadline       = tx_deadline;
	txi.frame          = frame;
	txi.loopback       = (flags & uavcan::CanIOFlagLoopback) != 0;
	txi.abort_on_error = (flags & uavcan::CanIOFlagAbortOnError) != 0;
	txi.pending        = TxItem::busy;

	mb.CS = cs; // Go.
	can_->IMASK1 |= mb_bit;
	return 1;
}

uavcan::int16_t CanIface::receive(uavcan::CanFrame &out_frame, uavcan::MonotonicTime &out_ts_monotonic,
				  uavcan::UtcTime &out_ts_utc, uavcan::CanIOFlags &out_flags)
{
	out_ts_monotonic = clock::getMonotonic();     // High precision is not required for monotonic timestamps
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
	if (num_configs <= NumFilters) {
		CriticalSectionLocker lock;
		setFreeze(true);

		if (!waitFreezeAckChange(true)) {
			return -ErrMcrFRZACKAckNotSet;
		}

		volatile flexcan::FilterType *filterBase = reinterpret_cast<volatile
				flexcan::FilterType *>(&can_->MB[flexcan::
						       FirstFilter].mb);

		if (num_configs == 0) {
			// No filters except all
			for (uint8_t i = 0; i < NumFilters; i++) {
				volatile flexcan::FilterType &filter = filterBase[i];
				filter.w = 0; // All Do not care
				can_->RXIMR[i].w  = 0;// All Do not care
			}

			can_->RXFGMASK = 0; // All Do not care

		} else {
			for (uint8_t i = 0; i < NumFilters; i++) {
				volatile flexcan::FilterType &filter = filterBase[i];
				volatile flexcan::FilterType &mask = can_->RXIMR[i];

				if (i < num_configs) {

					const uavcan::CanFilterConfig *const cfg = filter_configs + i;

					if ((cfg->id & uavcan::CanFrame::FlagEFF) || !(cfg->mask & uavcan::CanFrame::FlagEFF)) {
						filter.ide = 1;
						filter.ext = cfg->id & uavcan::CanFrame::MaskExtID;
						mask.ext = cfg->mask & uavcan::CanFrame::MaskExtID;

					} else {
						filter.ide = 0;
						filter.std = cfg->id & uavcan::CanFrame::MaskStdID;
						mask.std = cfg->mask & uavcan::CanFrame::MaskStdID;
					}

					filter.rtr = cfg->id & uavcan::CanFrame::FlagRTR ? 1 : 0;

					mask.rtr = cfg->mask & uavcan::CanFrame::FlagRTR ?  1 : 0;
					mask.ide = cfg->mask & uavcan::CanFrame::FlagEFF ?  1 : 0;

				} else {
					/* We can not really disable, with out effecting the memory map
					 * of the mail boxes, so set all bits to 1's and all to care
					 */
					filter.w = 0xffffffff;
					mask.w = 0xffffffff;
				}
			}
		}

		setFreeze(false);
		return waitFreezeAckChange(false) ? 0 : -ErrMcrFRZACKAckNotCleared;
	}

	return -ErrFilterNumConfigs;
}

bool CanIface::waitMCRChange(uavcan::uint32_t mask, bool target_state)
{
	const unsigned Timeout = 1000;

	for (unsigned wait_ack = 0; wait_ack < Timeout; wait_ack++) {
		const bool state = (can_->MCR & mask) != 0;

		if (state == target_state) {
			return true;
		}

		::up_udelay(10);
	}

	return false;
}

void CanIface::setMCR(uavcan::uint32_t mask, bool target_state)
{
	if (target_state) {
		can_->MCR |= mask;

	} else {
		can_->MCR &= ~mask;
	}

}

bool CanIface::waitFreezeAckChange(bool target_state)
{
	return waitMCRChange(flexcan::MCR_FRZACK, target_state);
}

void CanIface::setFreeze(bool freeze_true)
{
	{
		CriticalSectionLocker lock;

		if (freeze_true) {
			can_->MCR |= flexcan::MCR_FRZ;
			can_->MCR |= flexcan::MCR_HALT;

		} else {
			can_->MCR &= ~flexcan::MCR_HALT;
			can_->MCR &= ~flexcan::MCR_FRZ;

		}
	}
}


bool CanIface::setEnable(bool enable_true)
{
	setMCR(flexcan::MCR_MDIS, !enable_true);
	return waitMCRChange(flexcan::MCR_LPMACK, true);
}
uavcan::int16_t CanIface::doReset(Timings timings)

{
	setMCR(flexcan::MCR_SOFTRST, true);

	if (!waitMCRChange(flexcan::MCR_SOFTRST, false)) {
		return -ErrMcrSOFTRSTNotCleared;
	}

	uavcan::uint8_t tasd = 25 - (flexcan::canclk * (flexcan::HWMaxMB + 3  -
				     (flexcan::numberFIFOFilters * 8) -
				     (flexcan::useFIFO * flexcan::numberFIFOFilters * 2)) * 2) / \
			       (flexcan::busclck * (1 + timings.pseg1 + timings.pseg2 + timings.propseg)
				* timings.prescaler);

	setMCR(flexcan::MCR_SUPV, false);

	for (int i = 0; i < flexcan::HWMaxMB; i++) {
		can_->MB[i].mb.CS.w = 0x0;
		can_->MB[i].mb.ID.w = 0x0;
		can_->MB[i].mb.data.l = 0x0;
		can_->MB[i].mb.data.h = 0x0;
	}

	setMCR((flexcan::useFIFO ? flexcan::MCR_RFEN : 0) | flexcan::MCR_SLFWAK | flexcan::MCR_WRNEN | flexcan::MCR_WAKSRC
	       | flexcan::MCR_SRXDIS | flexcan::MCR_IRMQ | flexcan::MCR_AEN |
	       (((flexcan::HWMaxMB - 1) << flexcan::MCR_MAXMB_SHIFT) & flexcan::MCR_MAXMB_MASK), true);
	can_->CTRL2 = flexcan::numberFIFOFilters |
		      ((tasd << flexcan::CTRL2_TASD_SHIFT) & flexcan::CTRL2_TASD_MASK) |
		      flexcan::CTRL2_RRS |
		      flexcan::CTRL2_EACEN;

	for (int i = 0; i < flexcan::HWMaxMB; i++) {
		can_->RXIMR[i].w = 0x0;
	}

	can_->RX14MASK = 0x3FFFFFFF;
	can_->RX15MASK = 0x3FFFFFFF;
	can_->RXMGMASK = 0x3FFFFFFF;
	can_->RXFGMASK = 0x0;
	return 0;
}

int CanIface::init(const uavcan::uint32_t bitrate, const OperatingMode mode)
{

	/* Set the module disabled */

	if (!setEnable(false)) {
		UAVCAN_KINETIS_LOG("MCR MCR_LPMACK not set");
		return -ErrMcrLPMAckNotSet;
	}

	/* Set the Clock to OSCERCLK or ) */

	can_->CTRL1 &= ~flexcan::CTRL1_CLKSRC;
	can_->CTRL1 |= flexcan::CLOCKSEL;

	/* Set the module enabled */

	if (!setEnable(true)) {
		UAVCAN_KINETIS_LOG("MCR MCR_LPMACK not cleared");
		return -ErrMcrLPMAckNotCleared;
	}

	/*
	 * Object state - interrupts are disabled, so it's safe to modify it now
	 */

	rx_queue_.reset();
	error_cnt_ = 0;
	served_aborts_cnt_ = 0;
	uavcan::fill_n(pending_tx_, NumTxMesgBuffers, TxItem());
	peak_tx_mailbox_index_ = 0;
	had_activity_ = false;

	/*
	 * CAN timings for this bitrate
	 */
	Timings timings;
	const int timings_res = computeTimings(bitrate, timings);

	if (timings_res < 0) {
		setEnable(false);
		return timings_res;
	}

	UAVCAN_KINETIS_LOG("Timings: prescaler=%u rjw=%u propseg=%u pseg1=%u pseg2=%u",
			   unsigned(timings.prescaler), unsigned(timings.rjw),
			   unsigned(timings.propseg), unsigned(timings.pseg1),
			   unsigned(timings.pseg2));
	/*
	 * Hardware initialization from reset state
	 */


	uavcan::int16_t rv = doReset(timings);

	if (rv != 0) {
		UAVCAN_KINETIS_LOG("doReset returned %d", rv);
		return -rv;
	}

	uavcan::uint32_t ctl1 = can_->CTRL1;
	ctl1 |= (timings.prescaler - 1) << flexcan::CTRL1_PRESDIV_SHIFT;
	ctl1 |= (timings.rjw - 1) << flexcan::CTRL1_RJW_SHIFT;
	ctl1 |= (timings.pseg1 - 1) << flexcan::CTRL1_PSEG1_SHIFT;
	ctl1 |= (timings.pseg2 - 1) << flexcan::CTRL1_PSEG2_SHIFT;
	ctl1 |= flexcan::CTRL1_ERRMSK;
	ctl1 |= flexcan::CTRL1_TWRNMSK;
	ctl1 |= flexcan::CTRL1_RWRNMSK;
	ctl1 |= ((mode == SilentMode) ? flexcan::CTRL1_LOM : 0);
	ctl1 |= ((timings.propseg - 1) << flexcan::CTRL1_ROPSEG_SHIFT);
	can_->CTRL1 = ctl1;

	/*
	 * Default filter configuration
	 */
	volatile flexcan::FilterType *filterBase = reinterpret_cast<volatile
			flexcan::FilterType *>(&can_->MB[flexcan::FirstFilter].
					       mb);

	for (uavcan::uint32_t i = 0; i < flexcan::NumHWFilters; i++) {
		volatile flexcan::FilterType &filter = filterBase[i];
		filter.w = 0; // All bits do not care
	}

	can_->RXFGMASK = 0; // All bits do not care

	for (uavcan::uint32_t mb = 0; mb < flexcan::HWMaxMB; mb++) {
		can_->RXIMR[mb].w = 0; // All bits do not care
	}

	can_->IFLAG1 = FIFO_IFLAG1 | flexcan::TXMBMask;
	can_->IMASK1 = FIFO_IFLAG1;

	setFreeze(false);
	return waitFreezeAckChange(false) ? 0 : -ErrMcrFRZACKAckNotCleared;
}

void CanIface::handleTxMailboxInterrupt(uavcan::uint8_t mailbox_index, bool txok, const uavcan::uint64_t utc_usec)
{
	UAVCAN_ASSERT(mailbox_index < NumTxMesgBuffers);

	had_activity_ = had_activity_ || txok;

	TxItem &txi = pending_tx_[mailbox_index];

	if (txi.loopback && txok && txi.pending == TxItem::busy) {
		rx_queue_.push(txi.frame, utc_usec, uavcan::CanIOFlagLoopback);
	}

	txi.pending = TxItem::free;
}

void CanIface::handleTxInterrupt(uavcan::uint32_t tx_iflags, const uavcan::uint64_t utc_usec)
{

	/* First Process aborts */

	uavcan::uint32_t aborts = pending_aborts_ & tx_iflags;

	if (aborts) {
		uavcan::uint32_t moreAborts = aborts;
		uavcan::uint32_t bit = 1 << flexcan::NumMBinFiFoAndFilters;

		for (uavcan::uint32_t mb = 0; moreAborts && mb < NumTxMesgBuffers; mb++) {
			if (moreAborts & bit) {
				moreAborts &= ~bit;
				can_->IFLAG1 = bit;
				pending_tx_[mb].pending = TxItem::free;
				served_aborts_cnt_++;
			}

			bit <<= 1;
		}

		tx_iflags &= ~aborts;
		pending_aborts_ &= ~aborts;

	}

	/* Now Process TX completions */

	uavcan::uint32_t mbBit = 1 << flexcan::NumMBinFiFoAndFilters;

	for (uavcan::uint32_t mbi = 0; tx_iflags && mbi < NumTxMesgBuffers; mbi++) {
		if (tx_iflags & mbBit) {
			can_->IFLAG1 = mbBit;
			tx_iflags &= ~mbBit;
			const bool txok = can_->MB[flexcan::NumMBinFiFoAndFilters + mbi].mb.CS.code != flexcan::TxMbAbort;
			handleTxMailboxInterrupt(mbi, txok, utc_usec);
		}

		mbBit <<= 1;
	}

	update_event_.signalFromInterrupt();

	pollErrorFlagsFromISR();

}

void CanIface::handleRxInterrupt(uavcan::uint32_t rx_iflags, uavcan::uint64_t utc_usec)
{
	UAVCAN_ASSERT(fifo_index < 2);

	uavcan::CanFrame frame;

	if ((rx_iflags & FIFO_IFLAG1) == 0) {
		UAVCAN_ASSERT(0);  // Weird, IRQ is here but no data to read
		return;
	}

	if (rx_iflags & flexcan::CAN_FIFO_OV) {
		error_cnt_++;
		can_->IFLAG1 = flexcan::CAN_FIFO_OV;
	}

	if (rx_iflags & flexcan::CAN_FIFO_WARN) {
		fifo_warn_cnt_++;
		can_->IFLAG1 = flexcan::CAN_FIFO_WARN;
	}

	if (rx_iflags & flexcan::CAN_FIFO_NE) {
		const flexcan::RxFiFoType &rf = can_->MB[flexcan::FiFo].fifo;

		/*
		 * Read the frame contents
		 */

		if (rf.CS.ide) {
			frame.id = uavcan::CanFrame::MaskExtID & rf.ID.ext;
			frame.id |= uavcan::CanFrame::FlagEFF;

		} else {
			frame.id = uavcan::CanFrame::MaskStdID & rf.ID.std;
		}

		if (rf.CS.rtr) {
			frame.id |= uavcan::CanFrame::FlagRTR;
		}

		frame.dlc = rf.CS.dlc;

		frame.data[0] = rf.data.b0;
		frame.data[1] = rf.data.b1;
		frame.data[2] = rf.data.b2;
		frame.data[3] = rf.data.b3;
		frame.data[4] = rf.data.b4;
		frame.data[5] = rf.data.b5;
		frame.data[6] = rf.data.b6;
		frame.data[7] = rf.data.b7;

		volatile uavcan::uint32_t idhit = can_->RXFIR;

		(void)can_->TIMER;
		can_->IFLAG1 = flexcan::CAN_FIFO_NE;

		/*
		 * Store with timeout into the FIFO buffer and signal update event
		 */
		rx_queue_.push(frame, utc_usec, 0);
		had_activity_ = true;
		update_event_.signalFromInterrupt();

	}

	pollErrorFlagsFromISR();
}

void CanIface::pollErrorFlagsFromISR()
{
	volatile uavcan::uint32_t esr1 = can_->ESR1 & (flexcan::ESR1_STFERR | flexcan::ESR1_FRMERR |
					 flexcan::ESR1_CRCERR | flexcan::ESR1_ACKERR |
					 flexcan::ESR1_BIT0ERR | flexcan::ESR1_BIT1ERR);

	if (esr1 != 0) {
		can_->ESR1 = esr1;
		error_cnt_++;
		uavcan::uint32_t pending_aborts = 0;

		// Begin abort requests

		for (int i = 0; i < NumTxMesgBuffers; i++) {
			TxItem &txi = pending_tx_[i];
			uavcan::uint32_t mbi = flexcan::NumMBinFiFoAndFilters + i;
			uavcan::uint32_t iflag1 = 1 << mbi;

			if (txi.pending == TxItem::busy && txi.abort_on_error) {
				txi.pending = TxItem::abort;
				can_->IFLAG1 = iflag1;
				pending_aborts |= iflag1;;
				can_->MB[mbi].mb.CS.code = flexcan::TxMbAbort;
			}
		}

		pending_aborts_ = pending_aborts;
	}

}

void CanIface::discardTimedOutTxMailboxes(uavcan::MonotonicTime current_time)
{
	CriticalSectionLocker lock;

	for (int mbi = 0; mbi < NumTxMesgBuffers; mbi++) {
		TxItem &txi = pending_tx_[mbi];

		if (txi.pending == TxItem::busy && txi.deadline < current_time) {
			can_->MB[flexcan::NumMBinFiFoAndFilters + mbi].mb.CS.code = flexcan::TxMbAbort;
			txi.pending = TxItem::abort;
			error_cnt_++;
		}
	}
}

bool CanIface::canAcceptNewTxFrame(const uavcan::CanFrame &frame) const
{
	/*
	 * We can accept more frames only if the following conditions are satisfied:
	 *  - There is at least one TX mailbox free (obvious enough);
	 *  - The priority of the new frame is higher than priority of all TX mailboxes.
	 */
	{
		if ((can_->ESR2 & (flexcan::ESR2_IMB | flexcan::ESR2_VPS)) == flexcan::ESR2_VPS) {
			// No Free
			return false;
		}

	}

	/*
	 * The second condition requires a critical section.
	 */
	CriticalSectionLocker lock;

	for (int mbi = 0; mbi < NumTxMesgBuffers; mbi++) {
		if (pending_tx_[mbi].pending == TxItem::busy && !frame.priorityHigherThan(pending_tx_[mbi].frame)) {
			return false;       // There's a mailbox whose priority is higher or equal the priority of the new frame.
		}
	}

	return true;                // This new frame will be added to a free TX mailbox in the next @ref send().
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
uavcan::CanSelectMasks CanDriver::makeSelectMasks(const uavcan::CanFrame * (&pending_tx)[uavcan::MaxCanIfaces]) const
{
	uavcan::CanSelectMasks msk;

	// Iface 0
	msk.read  = if0_.isRxBufferEmpty() ? 0 : 1;

	if (pending_tx[0] != UAVCAN_NULLPTR) {
		msk.write = if0_.canAcceptNewTxFrame(*pending_tx[0]) ? 1 : 0;
	}

	// Iface 1
#if UAVCAN_KINETIS_NUM_IFACES > 1

	if (!if1_.isRxBufferEmpty()) {
		msk.read |= 1 << 1;
	}

	if (pending_tx[1] != UAVCAN_NULLPTR) {
		if (if1_.canAcceptNewTxFrame(*pending_tx[1])) {
			msk.write |= 1 << 1;
		}
	}

#endif
	return msk;
}

bool CanDriver::hasReadableInterfaces() const
{
#if UAVCAN_KINETIS_NUM_IFACES == 1
	return !if0_.isRxBufferEmpty();
#elif UAVCAN_KINETIS_NUM_IFACES == 2
	return !if0_.isRxBufferEmpty() || !if1_.isRxBufferEmpty();
#else
# error UAVCAN_KINETIS_NUM_IFACES
#endif
}

uavcan::int16_t CanDriver::select(uavcan::CanSelectMasks &inout_masks,
				  const uavcan::CanFrame * (&pending_tx)[uavcan::MaxCanIfaces],
				  const uavcan::MonotonicTime blocking_deadline)
{
	const uavcan::CanSelectMasks in_masks = inout_masks;
	const uavcan::MonotonicTime time = clock::getMonotonic();

	if0_.discardTimedOutTxMailboxes(time);              // Check TX timeouts - this may release some TX slots
	{
		CriticalSectionLocker cs_locker;
		if0_.pollErrorFlagsFromISR();
	}

#if UAVCAN_KINETIS_NUM_IFACES > 1
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

	(void)update_event_.wait(blocking_deadline - time);     // Block until timeout expires or any iface updates
	inout_masks = makeSelectMasks(pending_tx);     // Return what we got even if none of the requested events are set
	return 1;                                   // Return value doesn't matter as long as it is non-negative
}


void CanDriver::initOnce()
{
	/*
	 * CAN1, CAN2
	 */
	{
		CriticalSectionLocker lock;
		modifyreg32(KINETIS_SIM_SCGC6, 0, SIM_SCGC6_FLEXCAN0);
#if UAVCAN_KINETIS_NUM_IFACES > 1
		modifyreg32(KINETIS_SIM_SCGC3, 0, SIM_SCGC3_FLEXCAN1);
#endif
	}

	/*
	 * IRQ
	 */
#define IRQ_ATTACH(irq, handler)                          \
	{                                                      \
		const int res = irq_attach(irq, handler, NULL);    \
		(void)res;                                         \
		assert(res >= 0);                                  \
		up_enable_irq(irq);                                \
	}
	IRQ_ATTACH(KINETIS_IRQ_CAN0MB,  can0_irq);
#if UAVCAN_KINETIS_NUM_IFACES > 1
	IRQ_ATTACH(KINETIS_IRQ_CAN1MB,  can1_irq);
#endif
#undef IRQ_ATTACH
}

int CanDriver::init(const uavcan::uint32_t bitrate, const CanIface::OperatingMode mode)
{
	int res = 0;

	UAVCAN_KINETIS_LOG("Bitrate %lu mode %d", static_cast<unsigned long>(bitrate), static_cast<int>(mode));

	static bool initialized_once = false;

	if (!initialized_once) {
		initialized_once = true;
		UAVCAN_KINETIS_LOG("First initialization");
		initOnce();
	}

	/*
	 * CAN1
	 */
	UAVCAN_KINETIS_LOG("Initing iface 0...");
	ifaces[0] = &if0_;                          // This link must be initialized first,
	res = if0_.init(bitrate, mode);             // otherwise an IRQ may fire while the interface is not linked yet;

	if (res < 0) {                              // a typical race condition.
		UAVCAN_KINETIS_LOG("Iface 0 init failed %i", res);
		ifaces[0] = UAVCAN_NULLPTR;
		goto fail;
	}

	/*
	 * CAN2
	 */
#if UAVCAN_KINETIS_NUM_IFACES > 1
	UAVCAN_KINETIS_LOG("Initing iface 1...");
	ifaces[1] = &if1_;                          // Same thing here.
	res = if1_.init(bitrate, mode);

	if (res < 0) {
		UAVCAN_KINETIS_LOG("Iface 1 init failed %i", res);
		ifaces[1] = UAVCAN_NULLPTR;
		goto fail;
	}

#endif

	UAVCAN_KINETIS_LOG("CAN drv init OK");
	UAVCAN_ASSERT(res >= 0);
	return res;

fail:
	UAVCAN_KINETIS_LOG("CAN drv init failed %i", res);
	UAVCAN_ASSERT(res < 0);
	return res;
}

CanIface *CanDriver::getIface(uavcan::uint8_t iface_index)
{
	if (iface_index < UAVCAN_KINETIS_NUM_IFACES) {
		return ifaces[iface_index];
	}

	return UAVCAN_NULLPTR;
}

bool CanDriver::hadActivity()
{
	bool ret = if0_.hadActivity();
#if UAVCAN_KINETIS_NUM_IFACES > 1
	ret |= if1_.hadActivity();
#endif
	return ret;
}


/*
 * Interrupt handlers
 */
extern "C"
{

	static int can0_irq(const int irq, void *, void *args)
	{

		CanIface *cif = ifaces[0];
		flexcan::CanType *flex = flexcan::Can[0];

		if (cif != UAVCAN_NULLPTR) {
			const uavcan::uint32_t FIFO_IFLAG1 = flexcan::CAN_FIFO_NE | flexcan::CAN_FIFO_WARN | flexcan::CAN_FIFO_OV;
			uavcan::uint32_t flags = flex->IFLAG1 & FIFO_IFLAG1;

			if (flags) {
				handleRxInterrupt(cif, flags);
			}

			const uavcan::uint32_t MB_IFLAG1 = flexcan::TXMBMask;
			flags = flex->IFLAG1 & MB_IFLAG1;

			if (flags) {
				handleTxInterrupt(cif, flags);
			}
		}

		return 0;
	}

#if UAVCAN_KINETIS_NUM_IFACES > 1

	static int can1_irq(const int irq, void *, void *)
	{
		CanIface *cif = ifaces[1];
		flexcan::CanType *flex = flexcan::Can[1];

		if (cif != UAVCAN_NULLPTR) {
			const uavcan::uint32_t FIFO_IFLAG1 = flexcan::CAN_FIFO_NE | flexcan::CAN_FIFO_WARN | flexcan::CAN_FIFO_OV;
			uavcan::uint32_t flags = flex->IFLAG1 & FIFO_IFLAG1;

			if (flags) {
				handleRxInterrupt(cif, flags);
			}

			const uavcan::uint32_t MB_IFLAG1 = flexcan::TXMBMask;
			flags = flex->IFLAG1 & MB_IFLAG1;

			if (flags) {
				handleTxInterrupt(cif, flags);
			}
		}

		return 0;
	}

#endif
} // extern "C"
} // namespace uavcan_kinetis
