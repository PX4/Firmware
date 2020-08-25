/*
 * Copyright (C) 2014, 2018 Pavel Kirienko <pavel.kirienko@gmail.com>
 * Kinetis Port Author David Sidrane <david_s5@nscdg.com>
 */

#pragma once

#include <uavcan_kinetis/build_config.hpp>
#include <uavcan_kinetis/thread.hpp>
#include <uavcan/driver/can.hpp>
#include <uavcan_kinetis/flexcan.hpp>

namespace uavcan_kinetis
{
/**
 * Driver error codes.
 * These values can be returned from driver functions negated.
 */
//static const uavcan::int16_t ErrUnknown               = 1000; ///< Reserved for future use
static const uavcan::int16_t ErrNotImplemented          = 1001; ///< Feature not implemented
static const uavcan::int16_t ErrInvalidBitRate          = 1002; ///< Bit rate not supported
static const uavcan::int16_t ErrLogic                   = 1003; ///< Internal logic error
static const uavcan::int16_t ErrUnsupportedFrame        = 1004; ///< Frame not supported (e.g. RTR, CAN FD, etc)
static const uavcan::int16_t ErrMcrLPMAckNotSet         = 1005; ///< MCR_LPMACK bit of the MCR register is not 1
static const uavcan::int16_t ErrMcrLPMAckNotCleared     = 1006; ///< MCR_LPMACK bit of the MCR register is not 0
static const uavcan::int16_t ErrMcrFRZACKAckNotSet      = 1007; ///< MCR_FRZACK bit of the MCR register is not 1
static const uavcan::int16_t ErrMcrFRZACKAckNotCleared  = 1008; ///< MCR_FRZACK bit of the MCR register is not 0
static const uavcan::int16_t ErrBitRateNotDetected      = 1009; ///< Auto bit rate detection could not be finished
static const uavcan::int16_t ErrFilterNumConfigs        = 1010; ///< Number of filters is more than supported
static const uavcan::int16_t ErrMcrSOFTRSTNotCleared    = 1011; ///< MCR_SOFTRST bit of the MCR register is not 0
/**
 * RX queue item.
 * The application shall not use this directly.
 */
struct CanRxItem {
	uavcan::uint64_t utc_usec;
	uavcan::CanFrame frame;
	uavcan::CanIOFlags flags;
	CanRxItem()
		: utc_usec(0),
		  flags(0)
	{
	}
};

/**
 * Single CAN iface.
 * The application shall not use this directly.
 */
class CanIface : public uavcan::ICanIface
	, uavcan::Noncopyable
{
	const uavcan::uint32_t FIFO_IFLAG1 = flexcan::CAN_FIFO_NE | flexcan::CAN_FIFO_WARN | flexcan::CAN_FIFO_OV;
	class RxQueue
	{
		CanRxItem *const buf_;
		const uavcan::uint8_t capacity_;
		uavcan::uint8_t in_;
		uavcan::uint8_t out_;
		uavcan::uint8_t len_;
		uavcan::uint32_t overflow_cnt_;

		void registerOverflow();

	public:
		RxQueue(CanRxItem *buf, uavcan::uint8_t capacity)
			: buf_(buf),
			  capacity_(capacity),
			  in_(0),
			  out_(0),
			  len_(0),
			  overflow_cnt_(0)
		{
		}

		void push(const uavcan::CanFrame &frame, const uint64_t &utc_usec, uavcan::CanIOFlags flags);
		void pop(uavcan::CanFrame &out_frame, uavcan::uint64_t &out_utc_usec, uavcan::CanIOFlags &out_flags);

		void reset();
		unsigned getLength() const
		{
			return len_;
		}

		uavcan::uint32_t getOverflowCount() const
		{
			return overflow_cnt_;
		}
	};

	struct Timings {
		uavcan::uint8_t prescaler;
		uavcan::uint8_t rjw;
		uavcan::uint8_t pseg1;
		uavcan::uint8_t pseg2;
		uavcan::uint8_t propseg;

		Timings()
			: prescaler(0),
			  rjw(0),
			  pseg1(0),
			  pseg2(0),
			  propseg(0)
		{
		}
	};

	struct TxItem {
		uavcan::MonotonicTime deadline;
		uavcan::CanFrame frame;
		enum {free = 0, busy, abort } pending;
		bool loopback;
		bool abort_on_error;

		TxItem()
			: pending(free),
			  loopback(false),
			  abort_on_error(false)
		{
		}
	};

	enum { NumTxMesgBuffers = 6 };
	enum { NumFilters = 16 };

	uavcan::uint8_t MaxMB;

	RxQueue rx_queue_;
	flexcan::CanType *const can_;
	uavcan::uint64_t error_cnt_;
	uavcan::uint64_t fifo_warn_cnt_;
	uavcan::uint32_t pending_aborts_;
	uavcan::uint32_t served_aborts_cnt_;
	BusEvent &update_event_;
	TxItem pending_tx_[NumTxMesgBuffers];
	uavcan::uint8_t peak_tx_mailbox_index_;
	const uavcan::uint8_t self_index_;
	bool had_activity_;

	int computeTimings(uavcan::uint32_t target_bitrate, Timings &out_timings);

	virtual uavcan::int16_t send(const uavcan::CanFrame &frame, uavcan::MonotonicTime tx_deadline,
				     uavcan::CanIOFlags flags);

	virtual uavcan::int16_t receive(uavcan::CanFrame &out_frame, uavcan::MonotonicTime &out_ts_monotonic,
					uavcan::UtcTime &out_ts_utc, uavcan::CanIOFlags &out_flags);

	virtual uavcan::int16_t configureFilters(const uavcan::CanFilterConfig *filter_configs,
			uavcan::uint16_t num_configs);

	virtual uavcan::uint16_t getNumFilters() const
	{
		return NumFilters;
	}

	void handleTxMailboxInterrupt(uavcan::uint8_t mailbox_index, bool txok, uavcan::uint64_t utc_usec);

	bool waitMCRChange(uavcan::uint32_t mask, bool target_state);
	void setMCR(uavcan::uint32_t mask, bool target_state);

	bool setEnable(bool enable_true);
	uavcan::int16_t doReset(Timings timings);
	bool waitFreezeAckChange(bool target_state);
	void setFreeze(bool freeze_true);

public:
	enum { MaxRxQueueCapacity = 254 };

	enum OperatingMode {
		NormalMode,
		SilentMode
	};

	CanIface(flexcan::CanType *can, BusEvent &update_event, uavcan::uint8_t self_index,
		 CanRxItem *rx_queue_buffer, uavcan::uint8_t rx_queue_capacity)
		: MaxMB(flexcan::HWMaxMB),
		  rx_queue_(rx_queue_buffer, rx_queue_capacity),
		  can_(can),
		  error_cnt_(0),
		  fifo_warn_cnt_(0),
		  pending_aborts_(0),
		  served_aborts_cnt_(0),
		  update_event_(update_event),
		  peak_tx_mailbox_index_(0),
		  self_index_(self_index),
		  had_activity_(false)
	{
		UAVCAN_ASSERT(self_index_ < UAVCAN_KINETIS_NUM_IFACES);
	}

	/**
	 * Initializes the hardware CAN controller.
	 * Assumes:
	 *   - Iface clock is enabled
	 *   - Iface has been resetted via RCC
	 *   - Caller will configure NVIC by itself
	 */
	int init(const uavcan::uint32_t bitrate, const OperatingMode mode);

	void handleTxInterrupt(uavcan::uint32_t tx_iflags, uavcan::uint64_t utc_usec);
	void handleRxInterrupt(uavcan::uint32_t rx_iflags, uavcan::uint64_t utc_usec);

	/**
	 * This method is used to count errors and abort transmission on error if necessary.
	 * This functionality used to be implemented in the SCE interrupt handler, but that approach was
	 * generating too much processing overhead, especially on disconnected interfaces.
	 *
	 * Should be called from RX ISR, TX ISR, and select(); interrupts must be enabled.
	 */
	void pollErrorFlagsFromISR();

	void discardTimedOutTxMailboxes(uavcan::MonotonicTime current_time);

	bool canAcceptNewTxFrame(const uavcan::CanFrame &frame) const;
	bool isRxBufferEmpty() const;

	/**
	 * Number of RX frames lost due to queue overflow.
	 * This is an atomic read, it doesn't require a critical section.
	 */
	uavcan::uint32_t getRxQueueOverflowCount() const
	{
		return rx_queue_.getOverflowCount();
	}

	/**
	 * Total number of hardware failures and other kinds of errors (e.g. queue overruns).
	 * May increase continuously if the interface is not connected to the bus.
	 */
	virtual uavcan::uint64_t getErrorCount() const;

	/**
	 * Number of times the driver exercised library's requirement to abort transmission on first error.
	 * This is an atomic read, it doesn't require a critical section.
	 * See @ref uavcan::CanIOFlagAbortOnError.
	 */
	uavcan::uint32_t getVoluntaryTxAbortCount() const
	{
		return served_aborts_cnt_;
	}

	/**
	 * Returns the number of frames pending in the RX queue.
	 * This is intended for debug use only.
	 */
	unsigned getRxQueueLength() const;

	/**
	 * Whether this iface had at least one successful IO since the previous call of this method.
	 * This is designed for use with iface activity LEDs.
	 */
	bool hadActivity();

	/**
	 * Peak number of TX mailboxes used concurrently since initialization.
	 * Range is [1, 3].
	 * Value of 3 suggests that priority inversion could be taking place.
	 */
	uavcan::uint8_t getPeakNumTxMailboxesUsed() const
	{
		return uavcan::uint8_t(peak_tx_mailbox_index_ + 1);
	}
};

/**
 * CAN driver, incorporates all available CAN ifaces.
 * Please avoid direct use, prefer @ref CanInitHelper instead.
 */
class CanDriver : public uavcan::ICanDriver
	, uavcan::Noncopyable
{
	BusEvent update_event_;
	CanIface if0_;
#if UAVCAN_KINETIS_NUM_IFACES > 1
	CanIface if1_;
#endif

	virtual uavcan::int16_t select(uavcan::CanSelectMasks &inout_masks,
				       const uavcan::CanFrame * (&pending_tx)[uavcan::MaxCanIfaces],
				       uavcan::MonotonicTime blocking_deadline);

	static void initOnce();

public:
	template <unsigned RxQueueCapacity>
	CanDriver(CanRxItem(&rx_queue_storage)[UAVCAN_KINETIS_NUM_IFACES][RxQueueCapacity])
		: update_event_(*this),
		  if0_(flexcan::Can[0], update_event_, 0, rx_queue_storage[0], RxQueueCapacity)
#if UAVCAN_KINETIS_NUM_IFACES > 1
		, if1_(flexcan::Can[1], update_event_, 1, rx_queue_storage[1], RxQueueCapacity)
#endif
	{
		uavcan::StaticAssert < (RxQueueCapacity <= CanIface::MaxRxQueueCapacity) >::check();
	}

	/**
	 * This function returns select masks indicating which interfaces are available for read/write.
	 */
	uavcan::CanSelectMasks makeSelectMasks(const uavcan::CanFrame * (&pending_tx)[uavcan::MaxCanIfaces]) const;

	/**
	 * Whether there's at least one interface where receive() would return a frame.
	 */
	bool hasReadableInterfaces() const;

	/**
	 * Returns zero if OK.
	 * Returns negative value if failed (e.g. invalid bitrate).
	 */
	int init(const uavcan::uint32_t bitrate, const CanIface::OperatingMode mode);

	virtual CanIface *getIface(uavcan::uint8_t iface_index);

	virtual uavcan::uint8_t getNumIfaces() const
	{
		return UAVCAN_KINETIS_NUM_IFACES;
	}

	/**
	 * Whether at least one iface had at least one successful IO since previous call of this method.
	 * This is designed for use with iface activity LEDs.
	 */
	bool hadActivity();

	BusEvent &updateEvent() { return update_event_; }
};

/**
 * Helper class.
 * Normally only this class should be used by the application.
 * 145 usec per Extended CAN frame @ 1 Mbps, e.g. 32 RX slots * 145 usec --> 4.6 msec before RX queue overruns.
 */
template <unsigned RxQueueCapacity = 128>
class CanInitHelper
{
	CanRxItem queue_storage_[UAVCAN_KINETIS_NUM_IFACES][RxQueueCapacity];

public:
	enum { BitRateAutoDetect = 0 };

	CanDriver driver;

	CanInitHelper(uint32_t unused = 0x7) :
		driver(queue_storage_)
	{
	}

	/**
	 * This overload simply configures the provided bitrate.
	 * Auto bit rate detection will not be performed.
	 * Bitrate value must be positive.
	 * @return  Negative value on error; non-negative on success. Refer to constants Err*.
	 */
	int init(uavcan::uint32_t bitrate)
	{
		return driver.init(bitrate, CanIface::NormalMode);
	}

	/**
	 * This function can either initialize the driver at a fixed bit rate, or it can perform
	 * automatic bit rate detection. For theory please refer to the CiA application note #801.
	 *
	 * @param delay_callable    A callable entity that suspends execution for strictly more than one second.
	 *                          The callable entity will be invoked without arguments.
	 *                          @ref getRecommendedListeningDelay().
	 *
	 * @param inout_bitrate     Fixed bit rate or zero. Zero invokes the bit rate detection process.
	 *                          If auto detection was used, the function will update the argument
	 *                          with established bit rate. In case of an error the value will be undefined.
	 *
	 * @return                  Negative value on error; non-negative on success. Refer to constants Err*.
	 */
	template <typename DelayCallable>
	int init(DelayCallable delay_callable, uavcan::uint32_t &inout_bitrate = BitRateAutoDetect)
	{
		if (inout_bitrate > 0) {
			return driver.init(inout_bitrate, CanIface::NormalMode);

		} else {
			static const uavcan::uint32_t StandardBitRates[] = {
				1000000,
				500000,
				250000,
				125000
			};

			for (uavcan::uint8_t br = 0; br < sizeof(StandardBitRates) / sizeof(StandardBitRates[0]); br++) {
				inout_bitrate = StandardBitRates[br];

				const int res = driver.init(inout_bitrate, CanIface::SilentMode);

				delay_callable();

				if (res >= 0) {
					for (uavcan::uint8_t iface = 0; iface < driver.getNumIfaces(); iface++) {
						if (!driver.getIface(iface)->isRxBufferEmpty()) {
							// Re-initializing in normal mode
							return driver.init(inout_bitrate, CanIface::NormalMode);
						}
					}
				}
			}

			return -ErrBitRateNotDetected;
		}
	}

	/**
	 * Use this value for listening delay during automatic bit rate detection.
	 */
	static uavcan::MonotonicDuration getRecommendedListeningDelay()
	{
		return uavcan::MonotonicDuration::fromMSec(1050);
	}
};

}
