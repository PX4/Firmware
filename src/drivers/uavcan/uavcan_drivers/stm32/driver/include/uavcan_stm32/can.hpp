/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan_stm32/build_config.hpp>
#include <uavcan_stm32/thread.hpp>
#include <uavcan/driver/can.hpp>
#include <uavcan_stm32/bxcan.hpp>

namespace uavcan_stm32
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
static const uavcan::int16_t ErrMsrInakNotSet           = 1005; ///< INAK bit of the MSR register is not 1
static const uavcan::int16_t ErrMsrInakNotCleared       = 1006; ///< INAK bit of the MSR register is not 0
static const uavcan::int16_t ErrBitRateNotDetected      = 1007; ///< Auto bit rate detection could not be finished
static const uavcan::int16_t ErrFilterNumConfigs        = 1008; ///< Number of filters is more than supported

/**
 * RX queue item.
 * The application shall not use this directly.
 */
struct CanRxItem {
	uavcan::uint64_t utc_usec;
	uavcan::CanFrame frame;
	uavcan::CanIOFlags flags;
	CanRxItem()
		: utc_usec(0)
		, flags(0)
	{ }
};

/**
 * Single CAN iface.
 * The application shall not use this directly.
 */
class CanIface : public uavcan::ICanIface, uavcan::Noncopyable
{
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
			: buf_(buf)
			, capacity_(capacity)
			, in_(0)
			, out_(0)
			, len_(0)
			, overflow_cnt_(0)
		{ }

		void push(const uavcan::CanFrame &frame, const uint64_t &utc_usec, uavcan::CanIOFlags flags);
		void pop(uavcan::CanFrame &out_frame, uavcan::uint64_t &out_utc_usec, uavcan::CanIOFlags &out_flags);

		void reset();

		unsigned getLength() const { return len_; }

		uavcan::uint32_t getOverflowCount() const { return overflow_cnt_; }
	};

	struct Timings {
		uavcan::uint16_t prescaler;
		uavcan::uint8_t sjw;
		uavcan::uint8_t bs1;
		uavcan::uint8_t bs2;

		Timings()
			: prescaler(0)
			, sjw(0)
			, bs1(0)
			, bs2(0)
		{ }
	};

	struct TxItem {
		uavcan::MonotonicTime deadline;
		uavcan::CanFrame frame;
		bool pending;
		bool loopback;
		bool abort_on_error;

		TxItem()
			: pending(false)
			, loopback(false)
			, abort_on_error(false)
		{ }
	};

	enum { NumTxMailboxes = 3 };
	enum { NumFilters = 14 };

	static const uavcan::uint32_t TSR_ABRQx[NumTxMailboxes];

	RxQueue rx_queue_;
	bxcan::CanType *const can_;
	uavcan::uint64_t error_cnt_;
	uavcan::uint32_t served_aborts_cnt_;
	BusEvent &update_event_;
	TxItem pending_tx_[NumTxMailboxes];
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

	virtual uavcan::uint16_t getNumFilters() const { return NumFilters; }

	void handleTxMailboxInterrupt(uavcan::uint8_t mailbox_index, bool txok, uavcan::uint64_t utc_usec);

	bool waitMsrINakBitStateChange(bool target_state);

public:
	enum { MaxRxQueueCapacity = 254 };

	enum OperatingMode {
		NormalMode,
		SilentMode
	};

	CanIface(bxcan::CanType *can, BusEvent &update_event, uavcan::uint8_t self_index,
		 CanRxItem *rx_queue_buffer, uavcan::uint8_t rx_queue_capacity)
		: rx_queue_(rx_queue_buffer, rx_queue_capacity)
		, can_(can)
		, error_cnt_(0)
		, served_aborts_cnt_(0)
		, update_event_(update_event)
		, peak_tx_mailbox_index_(0)
		, self_index_(self_index)
		, had_activity_(false)
	{
		UAVCAN_ASSERT(self_index_ < UAVCAN_STM32_NUM_IFACES);
	}

	/**
	 * Initializes the hardware CAN controller.
	 * Assumes:
	 *   - Iface clock is enabled
	 *   - Iface has been resetted via RCC
	 *   - Caller will configure NVIC by itself
	 */
	int init(const uavcan::uint32_t bitrate, const OperatingMode mode);

	void handleTxInterrupt(uavcan::uint64_t utc_usec);
	void handleRxInterrupt(uavcan::uint8_t fifo_index, uavcan::uint64_t utc_usec);

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
	uavcan::uint32_t getRxQueueOverflowCount() const { return rx_queue_.getOverflowCount(); }

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
	uavcan::uint32_t getVoluntaryTxAbortCount() const { return served_aborts_cnt_; }

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
	uavcan::uint8_t getPeakNumTxMailboxesUsed() const { return uavcan::uint8_t(peak_tx_mailbox_index_ + 1); }
};

/**
 * CAN driver, incorporates all available CAN ifaces.
 * Please avoid direct use, prefer @ref CanInitHelper instead.
 */
class CanDriver : public uavcan::ICanDriver, uavcan::Noncopyable
{
	BusEvent update_event_;
	CanIface if0_;
#if UAVCAN_STM32_NUM_IFACES > 1
	CanIface if1_;
#endif
	uint32_t enabledInterfaces_;

	virtual uavcan::int16_t select(uavcan::CanSelectMasks &inout_masks,
				       const uavcan::CanFrame * (& pending_tx)[uavcan::MaxCanIfaces],
				       uavcan::MonotonicTime blocking_deadline);

	static void initOnce();

public:
	template <unsigned RxQueueCapacity>
	CanDriver(CanRxItem(&rx_queue_storage)[UAVCAN_STM32_NUM_IFACES][RxQueueCapacity])
		: update_event_(*this)
		, if0_(bxcan::Can[0], update_event_, 0, rx_queue_storage[0], RxQueueCapacity)
#if UAVCAN_STM32_NUM_IFACES > 1
		, if1_(bxcan::Can[1], update_event_, 1, rx_queue_storage[1], RxQueueCapacity)
#endif
		, enabledInterfaces_(0x7)
	{
		uavcan::StaticAssert < (RxQueueCapacity <= CanIface::MaxRxQueueCapacity) >::check();
	}

	/**
	 * This function returns select masks indicating which interfaces are available for read/write.
	 */
	uavcan::CanSelectMasks makeSelectMasks(const uavcan::CanFrame * (& pending_tx)[uavcan::MaxCanIfaces]) const;

	/**
	 * Whether there's at least one interface where receive() would return a frame.
	 */
	bool hasReadableInterfaces() const;

	/**
	 * Returns zero if OK.
	 * Returns negative value if failed (e.g. invalid bitrate).
	 */
	int init(const uavcan::uint32_t bitrate, const CanIface::OperatingMode mode, const uavcan::uint32_t EnabledInterfaces);

	virtual CanIface *getIface(uavcan::uint8_t iface_index);

	virtual uavcan::uint8_t getNumIfaces() const { return UAVCAN_STM32_NUM_IFACES; }

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
	CanRxItem queue_storage_[UAVCAN_STM32_NUM_IFACES][RxQueueCapacity];

public:
	enum { BitRateAutoDetect = 0 };

	CanDriver driver;
	uint32_t enabledInterfaces_;

	CanInitHelper(const uavcan::uint32_t EnabledInterfaces = 0x7) :
		driver(queue_storage_),
		enabledInterfaces_(EnabledInterfaces)
	{ }

	/**
	 * This overload simply configures the provided bitrate.
	 * Auto bit rate detection will not be performed.
	 * Bitrate value must be positive.
	 * @return  Negative value on error; non-negative on success. Refer to constants Err*.
	 */
	int init(uavcan::uint32_t bitrate)
	{
		return driver.init(bitrate, CanIface::NormalMode, enabledInterfaces_);
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
			return driver.init(inout_bitrate, CanIface::NormalMode, enabledInterfaces_);

		} else {
			static const uavcan::uint32_t StandardBitRates[] = {
				1000000,
				500000,
				250000,
				125000
			};

			for (uavcan::uint8_t br = 0; br < sizeof(StandardBitRates) / sizeof(StandardBitRates[0]); br++) {
				inout_bitrate = StandardBitRates[br];

				const int res = driver.init(inout_bitrate, CanIface::SilentMode, enabledInterfaces_);

				delay_callable();

				if (res >= 0) {
					for (uavcan::uint8_t iface = 0; iface < driver.getNumIfaces(); iface++) {
						if (!driver.getIface(iface)->isRxBufferEmpty()) {
							// Re-initializing in normal mode
							return driver.init(inout_bitrate, CanIface::NormalMode, enabledInterfaces_);
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
