/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan_stm32h7/clock.hpp>
#include <uavcan_stm32h7/thread.hpp>
#include "internal.hpp"

#if UAVCAN_STM32H7_TIMER_NUMBER

#include <cassert>
#include <math.h>

/*
 * Timer instance
 */
# if UAVCAN_STM32H7_NUTTX
#  define TIMX                    UAVCAN_STM32H7_GLUE3(STM32_TIM, UAVCAN_STM32H7_TIMER_NUMBER, _BASE)
#  define  TMR_REG(o)              (TIMX + (o))
#  define TIMX_INPUT_CLOCK         UAVCAN_STM32H7_GLUE3(STM32_APB1_TIM, UAVCAN_STM32H7_TIMER_NUMBER, _CLKIN)

#  define TIMX_IRQn                UAVCAN_STM32H7_GLUE2(STM32_IRQ_TIM, UAVCAN_STM32H7_TIMER_NUMBER)
# endif

# if UAVCAN_STM32H7_TIMER_NUMBER >= 2 && UAVCAN_STM32H7_TIMER_NUMBER <= 7
#  define TIMX_RCC_ENR           RCC->APB1ENR
#  define TIMX_RCC_RSTR          RCC->APB1RSTR
#  define TIMX_RCC_ENR_MASK      UAVCAN_STM32H7_GLUE3(RCC_APB1ENR_TIM,  UAVCAN_STM32H7_TIMER_NUMBER, EN)
#  define TIMX_RCC_RSTR_MASK     UAVCAN_STM32H7_GLUE3(RCC_APB1RSTR_TIM, UAVCAN_STM32H7_TIMER_NUMBER, RST)
# else
#  error "This UAVCAN_STM32H7_TIMER_NUMBER is not supported yet"
# endif

/**
 * UAVCAN_STM32H7_TIMX_INPUT_CLOCK can be used to manually override the auto-detected timer clock speed.
 * This is useful at least with certain versions of ChibiOS which do not support the bit
 * RCC_DKCFGR.TIMPRE that is available in newer models of STM32. In that case, if TIMPRE is active,
 * the auto-detected value of TIMX_INPUT_CLOCK will be twice lower than the actual clock speed.
 * Read this for additional context: http://www.chibios.com/forum/viewtopic.php?f=35&t=3870
 * A normal way to use the override feature is to provide an alternative macro, e.g.:
 *
 *      -DUAVCAN_STM32H7_TIMX_INPUT_CLOCK=STM32_HCLK
 *
 * Alternatively, the new clock rate can be specified directly.
 */
# ifdef UAVCAN_STM32H7_TIMX_INPUT_CLOCK
#  undef TIMX_INPUT_CLOCK
#  define TIMX_INPUT_CLOCK      UAVCAN_STM32H7_TIMX_INPUT_CLOCK
# endif

extern "C" UAVCAN_STM32H7_IRQ_HANDLER(TIMX_IRQHandler);

namespace uavcan_stm32h7
{
namespace clock
{
namespace
{

const uavcan::uint32_t USecPerOverflow = 65536;

Mutex mutex;

bool initialized = false;

bool utc_set = false;
bool utc_locked = false;
uavcan::uint32_t utc_jump_cnt = 0;
UtcSyncParams utc_sync_params;
float utc_prev_adj = 0;
float utc_rel_rate_ppm = 0;
float utc_rel_rate_error_integral = 0;
uavcan::int32_t utc_accumulated_correction_nsec = 0;
uavcan::int32_t utc_correction_nsec_per_overflow = 0;
uavcan::MonotonicTime prev_utc_adj_at;

uavcan::uint64_t time_mono = 0;
uavcan::uint64_t time_utc = 0;

}


void init()
{
	CriticalSectionLocker lock;

	if (initialized) {
		return;
	}

	initialized = true;


# if UAVCAN_STM32H7_NUTTX

	// Attach IRQ
	irq_attach(TIMX_IRQn, &TIMX_IRQHandler, NULL);

	// Power-on and reset
	modifyreg32(STM32_RCC_APB1ENR, 0, TIMX_RCC_ENR_MASK);
	modifyreg32(STM32_RCC_APB1RSTR, 0, TIMX_RCC_RSTR_MASK);
	modifyreg32(STM32_RCC_APB1RSTR, TIMX_RCC_RSTR_MASK, 0);


	// Start the timer
	putreg32(0xFFFF, TMR_REG(STM32_BTIM_ARR_OFFSET));
	putreg16(((TIMX_INPUT_CLOCK / 1000000) - 1), TMR_REG(STM32_BTIM_PSC_OFFSET));
	putreg16(BTIM_CR1_URS, TMR_REG(STM32_BTIM_CR1_OFFSET));
	putreg16(0, TMR_REG(STM32_BTIM_SR_OFFSET));
	putreg16(BTIM_EGR_UG, TMR_REG(STM32_BTIM_EGR_OFFSET)); // Reload immediately
	putreg16(BTIM_DIER_UIE, TMR_REG(STM32_BTIM_DIER_OFFSET));
	putreg16(BTIM_CR1_CEN, TMR_REG(STM32_BTIM_CR1_OFFSET)); // Start

	// Prioritize and Enable  IRQ
// todo: Currently changing the NVIC_SYSH_HIGH_PRIORITY is HARD faulting
// need to investigate
//    up_prioritize_irq(TIMX_IRQn, NVIC_SYSH_HIGH_PRIORITY);
	up_enable_irq(TIMX_IRQn);

# endif
}

void setUtc(uavcan::UtcTime time)
{
	MutexLocker mlocker(mutex);
	UAVCAN_ASSERT(initialized);

	{
		CriticalSectionLocker locker;
		time_utc = time.toUSec();
	}

	utc_set = true;
	utc_locked = false;
	utc_jump_cnt++;
	utc_prev_adj = 0;
	utc_rel_rate_ppm = 0;
}

static uavcan::uint64_t sampleUtcFromCriticalSection()
{
# if UAVCAN_STM32H7_NUTTX

	UAVCAN_ASSERT(initialized);
	UAVCAN_ASSERT(getreg16(TMR_REG(STM32_BTIM_DIER_OFFSET)) & BTIM_DIER_UIE);

	volatile uavcan::uint64_t time = time_utc;
	volatile uavcan::uint32_t cnt = getreg16(TMR_REG(STM32_BTIM_CNT_OFFSET));

	if (getreg16(TMR_REG(STM32_BTIM_SR_OFFSET)) & BTIM_SR_UIF) {
		cnt = getreg16(TMR_REG(STM32_BTIM_CNT_OFFSET));
		const uavcan::int32_t add = uavcan::int32_t(USecPerOverflow) +
					    (utc_accumulated_correction_nsec + utc_correction_nsec_per_overflow) / 1000;
		time = uavcan::uint64_t(uavcan::int64_t(time) + add);
	}

	return time + cnt;
# endif
}

uavcan::uint64_t getUtcUSecFromCanInterrupt()
{
	return utc_set ? sampleUtcFromCriticalSection() : 0;
}

uavcan::MonotonicTime getMonotonic()
{
	uavcan::uint64_t usec = 0;
	// Scope Critical section
	{
		CriticalSectionLocker locker;

		volatile uavcan::uint64_t time = time_mono;

# if UAVCAN_STM32H7_NUTTX

		volatile uavcan::uint32_t cnt = getreg16(TMR_REG(STM32_BTIM_CNT_OFFSET));

		if (getreg16(TMR_REG(STM32_BTIM_SR_OFFSET)) & BTIM_SR_UIF) {
			cnt = getreg16(TMR_REG(STM32_BTIM_CNT_OFFSET));
# endif
			time += USecPerOverflow;
		}

		usec = time + cnt;

# ifndef NDEBUG
		static uavcan::uint64_t prev_usec = 0;      // Self-test
		UAVCAN_ASSERT(prev_usec <= usec);
		(void)prev_usec;
		prev_usec = usec;
# endif
	} // End Scope Critical section

	return uavcan::MonotonicTime::fromUSec(usec);
}

uavcan::UtcTime getUtc()
{
	if (utc_set) {
		uavcan::uint64_t usec = 0;
		{
			CriticalSectionLocker locker;
			usec = sampleUtcFromCriticalSection();
		}
		return uavcan::UtcTime::fromUSec(usec);
	}

	return uavcan::UtcTime();
}

static float lowpass(float xold, float xnew, float corner, float dt)
{
	const float tau = 1.F / corner;
	return (dt * xnew + tau * xold) / (dt + tau);
}

static void updateRatePID(uavcan::UtcDuration adjustment)
{
	const uavcan::MonotonicTime ts = getMonotonic();
	const float dt = float((ts - prev_utc_adj_at).toUSec()) / 1e6F;
	prev_utc_adj_at = ts;
	const float adj_usec = float(adjustment.toUSec());

	/*
	 * Target relative rate in PPM
	 * Positive to go faster
	 */
	const float target_rel_rate_ppm = adj_usec * utc_sync_params.offset_p;

	/*
	 * Current relative rate in PPM
	 * Positive if the local clock is faster
	 */
	const float new_rel_rate_ppm = (utc_prev_adj - adj_usec) / dt; // rate error in [usec/sec], which is PPM
	utc_prev_adj = adj_usec;
	utc_rel_rate_ppm = lowpass(utc_rel_rate_ppm, new_rel_rate_ppm, utc_sync_params.rate_error_corner_freq, dt);

	const float rel_rate_error = target_rel_rate_ppm - utc_rel_rate_ppm;

	if (dt > 10) {
		utc_rel_rate_error_integral = 0;

	} else {
		utc_rel_rate_error_integral += rel_rate_error * dt * utc_sync_params.rate_i;
		utc_rel_rate_error_integral =
			uavcan::max(utc_rel_rate_error_integral, -utc_sync_params.max_rate_correction_ppm);
		utc_rel_rate_error_integral =
			uavcan::min(utc_rel_rate_error_integral, utc_sync_params.max_rate_correction_ppm);
	}

	/*
	 * Rate controller
	 */
	float total_rate_correction_ppm = rel_rate_error + utc_rel_rate_error_integral;
	total_rate_correction_ppm = uavcan::max(total_rate_correction_ppm, -utc_sync_params.max_rate_correction_ppm);
	total_rate_correction_ppm = uavcan::min(total_rate_correction_ppm, utc_sync_params.max_rate_correction_ppm);

	utc_correction_nsec_per_overflow = uavcan::int32_t((USecPerOverflow * 1000) * (total_rate_correction_ppm / 1e6F));

//    syslog("$ adj=%f   rel_rate=%f   rel_rate_eint=%f   tgt_rel_rate=%f   ppm=%f\n",
//              adj_usec, utc_rel_rate_ppm, utc_rel_rate_error_integral, target_rel_rate_ppm,
// total_rate_correction_ppm);
}

void adjustUtc(uavcan::UtcDuration adjustment)
{
	MutexLocker mlocker(mutex);
	UAVCAN_ASSERT(initialized);

	if (adjustment.getAbs() > utc_sync_params.min_jump || !utc_set) {
		const uavcan::int64_t adj_usec = adjustment.toUSec();

		{
			CriticalSectionLocker locker;

			if ((adj_usec < 0) && uavcan::uint64_t(-adj_usec) > time_utc) {
				time_utc = 1;

			} else {
				time_utc = uavcan::uint64_t(uavcan::int64_t(time_utc) + adj_usec);
			}
		}

		utc_set = true;
		utc_locked = false;
		utc_jump_cnt++;
		utc_prev_adj = 0;
		utc_rel_rate_ppm = 0;

	} else {
		updateRatePID(adjustment);

		if (!utc_locked) {
			utc_locked =
				(std::abs(utc_rel_rate_ppm) < utc_sync_params.lock_thres_rate_ppm) &&
				(std::abs(utc_prev_adj) < float(utc_sync_params.lock_thres_offset.toUSec()));
		}
	}
}

float getUtcRateCorrectionPPM()
{
	MutexLocker mlocker(mutex);
	const float rate_correction_mult = float(utc_correction_nsec_per_overflow) / float(USecPerOverflow * 1000);
	return 1e6F * rate_correction_mult;
}

uavcan::uint32_t getUtcJumpCount()
{
	MutexLocker mlocker(mutex);
	return utc_jump_cnt;
}

bool isUtcLocked()
{
	MutexLocker mlocker(mutex);
	return utc_locked;
}

UtcSyncParams getUtcSyncParams()
{
	MutexLocker mlocker(mutex);
	return utc_sync_params;
}

void setUtcSyncParams(const UtcSyncParams &params)
{
	MutexLocker mlocker(mutex);
	// Add some sanity check
	utc_sync_params = params;
}

} // namespace clock

SystemClock &SystemClock::instance()
{
	static union SystemClockStorage {
		uavcan::uint8_t buffer[sizeof(SystemClock)];
		long long _aligner_1;
		long double _aligner_2;
	} storage;

	SystemClock *const ptr = reinterpret_cast<SystemClock *>(storage.buffer);

	if (!clock::initialized) {
		MutexLocker mlocker(clock::mutex);
		clock::init();
		new (ptr)SystemClock();
	}

	return *ptr;
}

} // namespace uavcan_stm32h7


/**
 * Timer interrupt handler
 */

extern "C"
UAVCAN_STM32H7_IRQ_HANDLER(TIMX_IRQHandler)
{
	UAVCAN_STM32H7_IRQ_PROLOGUE();

# if UAVCAN_STM32H7_NUTTX
	putreg16(0, TMR_REG(STM32_BTIM_SR_OFFSET));
# endif

	using namespace uavcan_stm32h7::clock;
	UAVCAN_ASSERT(initialized);

	time_mono += USecPerOverflow;

	if (utc_set) {
		time_utc += USecPerOverflow;
		utc_accumulated_correction_nsec += utc_correction_nsec_per_overflow;

		if (std::abs(utc_accumulated_correction_nsec) >= 1000) {
			time_utc = uavcan::uint64_t(uavcan::int64_t(time_utc) + utc_accumulated_correction_nsec / 1000);
			utc_accumulated_correction_nsec %= 1000;
		}

		// Correction decay - 1 nsec per 65536 usec
		if (utc_correction_nsec_per_overflow > 0) {
			utc_correction_nsec_per_overflow--;

		} else if (utc_correction_nsec_per_overflow < 0) {
			utc_correction_nsec_per_overflow++;

		} else {
			; // Zero
		}
	}

	UAVCAN_STM32H7_IRQ_EPILOGUE();
}

#endif
