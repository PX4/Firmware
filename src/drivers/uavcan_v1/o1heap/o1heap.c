// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
// and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions
// of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// Copyright (c) 2020 Pavel Kirienko
// Authors: Pavel Kirienko <pavel.kirienko@zubax.com>

#include "o1heap.h"
#include <assert.h>

// ---------------------------------------- BUILD CONFIGURATION OPTIONS ----------------------------------------

/// The assertion macro defaults to the standard assert().
/// It can be overridden to manually suppress assertion checks or use a different error handling policy.
#ifndef O1HEAP_ASSERT
// Intentional violation of MISRA: the assertion check macro cannot be replaced with a function definition.
#    define O1HEAP_ASSERT(x) assert(x)  // NOSONAR
#endif

/// Branch probability annotations are used to improve the worst case execution time (WCET). They are entirely optional.
/// A stock implementation is provided for some well-known compilers; for other compilers it defaults to nothing.
/// If you are using a different compiler, consider overriding this value.
#ifndef O1HEAP_LIKELY
#    if defined(__GNUC__) || defined(__clang__) || defined(__CC_ARM)
// Intentional violation of MISRA: branch hinting macro cannot be replaced with a function definition.
#        define O1HEAP_LIKELY(x) __builtin_expect((x), 1)  // NOSONAR
#    else
#        define O1HEAP_LIKELY(x) x
#    endif
#endif

/// This option is used for testing only. Do not use in production.
#if defined(O1HEAP_EXPOSE_INTERNALS) && O1HEAP_EXPOSE_INTERNALS
#    define O1HEAP_PRIVATE
#else
#    define O1HEAP_PRIVATE static inline
#endif

// ---------------------------------------- INTERNAL DEFINITIONS ----------------------------------------

#if !defined(__STDC_VERSION__) || (__STDC_VERSION__ < 199901L)
#    error "Unsupported language: ISO C99 or a newer version is required."
#endif

#if __STDC_VERSION__ < 201112L
// Intentional violation of MISRA: static assertion macro cannot be replaced with a function definition.
#    define static_assert(x, ...) typedef char _static_assert_gl(_static_assertion_, __LINE__)[(x) ? 1 : -1]  // NOSONAR
#    define _static_assert_gl(a, b) _static_assert_gl_impl(a, b)                                              // NOSONAR
// Intentional violation of MISRA: the paste operator ## cannot be avoided in this context.
#    define _static_assert_gl_impl(a, b) a##b  // NOSONAR
#endif

/// The overhead is at most O1HEAP_ALIGNMENT bytes large,
/// then follows the user data which shall keep the next fragment aligned.
#define FRAGMENT_SIZE_MIN (O1HEAP_ALIGNMENT * 2U)

/// This is risky, handle with care: if the allocation amount plus per-fragment overhead exceeds 2**(b-1),
/// where b is the pointer bit width, then ceil(log2(amount)) yields b; then 2**b causes an integer overflow.
/// To avoid this, we put a hard limit on fragment size (which is amount + per-fragment overhead): 2**(b-1)
#define FRAGMENT_SIZE_MAX ((SIZE_MAX >> 1U) + 1U)

/// Normally we should subtract log2(FRAGMENT_SIZE_MIN) but log2 is bulky to compute using the preprocessor only.
/// We will certainly end up with unused bins this way, but it is cheap to ignore.
#define NUM_BINS_MAX (sizeof(size_t) * 8U)

static_assert((O1HEAP_ALIGNMENT & (O1HEAP_ALIGNMENT - 1U)) == 0U, "Not a power of 2");
static_assert((FRAGMENT_SIZE_MIN & (FRAGMENT_SIZE_MIN - 1U)) == 0U, "Not a power of 2");
static_assert((FRAGMENT_SIZE_MAX & (FRAGMENT_SIZE_MAX - 1U)) == 0U, "Not a power of 2");

typedef struct Fragment Fragment;

typedef struct FragmentHeader {
	Fragment *next;
	Fragment *prev;
	size_t    size;
	bool      used;
} FragmentHeader;
static_assert(sizeof(FragmentHeader) <= O1HEAP_ALIGNMENT, "Memory layout error");

struct Fragment {
	FragmentHeader header;
	// Everything past the header may spill over into the allocatable space. The header survives across alloc/free.
	Fragment *next_free;  // Next free fragment in the bin; NULL in the last one.
	Fragment *prev_free;  // Same but points back; NULL in the first one.
};
static_assert(sizeof(Fragment) <= FRAGMENT_SIZE_MIN, "Memory layout error");

struct O1HeapInstance {
	Fragment *bins[NUM_BINS_MAX];  ///< Smallest fragments are in the bin at index 0.
	size_t    nonempty_bin_mask;   ///< Bit 1 represents a non-empty bin; bin at index 0 is for the smallest fragments.

	O1HeapHook critical_section_enter;
	O1HeapHook critical_section_leave;

	O1HeapDiagnostics diagnostics;
};

/// The amount of space allocated for the heap instance.
/// Its size is padded up to O1HEAP_ALIGNMENT to ensure correct alignment of the allocation arena that follows.
#define INSTANCE_SIZE_PADDED ((sizeof(O1HeapInstance) + O1HEAP_ALIGNMENT - 1U) & ~(O1HEAP_ALIGNMENT - 1U))

static_assert(INSTANCE_SIZE_PADDED >= sizeof(O1HeapInstance), "Invalid instance footprint computation");
static_assert((INSTANCE_SIZE_PADDED % O1HEAP_ALIGNMENT) == 0U, "Invalid instance footprint computation");

/// True if the argument is an integer power of two or zero.
O1HEAP_PRIVATE bool isPowerOf2(const size_t x);
O1HEAP_PRIVATE bool isPowerOf2(const size_t x)
{
	return (x & (x - 1U)) == 0U;
}

/// Special case: if the argument is zero, returns zero.
O1HEAP_PRIVATE uint8_t log2Floor(const size_t x);
O1HEAP_PRIVATE uint8_t log2Floor(const size_t x)
{
	size_t  tmp = x;
	uint8_t y   = 0;

	// This is currently the only exception to the statement "routines contain neither loops nor recursion".
	// It is unclear if there is a better way to compute the binary logarithm than this.
	while (tmp > 1U) {
		tmp >>= 1U;
		y++;
	}

	return y;
}

/// Special case: if the argument is zero, returns zero.
O1HEAP_PRIVATE uint8_t log2Ceil(const size_t x);
O1HEAP_PRIVATE uint8_t log2Ceil(const size_t x)
{
	return (uint8_t)(log2Floor(x) + (isPowerOf2(x) ? 0U : 1U));
}

/// Raise 2 into the specified power.
/// You might be tempted to do something like (1U << power). WRONG! We humans are prone to forgetting things.
/// If you forget to cast your 1U to size_t or ULL, you may end up with undefined behavior.
O1HEAP_PRIVATE size_t pow2(const uint8_t power);
O1HEAP_PRIVATE size_t pow2(const uint8_t power)
{
	return ((size_t) 1U) << power;
}

O1HEAP_PRIVATE void invoke(const O1HeapHook hook);
O1HEAP_PRIVATE void invoke(const O1HeapHook hook)
{
	if (hook != NULL) {
		hook();
	}
}

/// Links two fragments so that their next/prev pointers point to each other; left goes before right.
O1HEAP_PRIVATE void interlink(Fragment *const left, Fragment *const right);
O1HEAP_PRIVATE void interlink(Fragment *const left, Fragment *const right)
{
	if (O1HEAP_LIKELY(left != NULL)) {
		left->header.next = right;
	}

	if (O1HEAP_LIKELY(right != NULL)) {
		right->header.prev = left;
	}
}

/// Adds a new block into the appropriate bin and updates the lookup mask.
O1HEAP_PRIVATE void rebin(O1HeapInstance *const handle, Fragment *const fragment);
O1HEAP_PRIVATE void rebin(O1HeapInstance *const handle, Fragment *const fragment)
{
	O1HEAP_ASSERT(handle != NULL);
	O1HEAP_ASSERT(fragment != NULL);
	O1HEAP_ASSERT(fragment->header.size >= FRAGMENT_SIZE_MIN);
	O1HEAP_ASSERT((fragment->header.size % FRAGMENT_SIZE_MIN) == 0U);
	const uint8_t idx = log2Floor(fragment->header.size / FRAGMENT_SIZE_MIN);  // Round DOWN when inserting.
	O1HEAP_ASSERT(idx < NUM_BINS_MAX);
	// Add the new fragment to the beginning of the bin list.
	// I.e., each allocation will be returning the least-recently-used fragment -- good for caching.
	fragment->next_free = handle->bins[idx];
	fragment->prev_free = NULL;

	if (O1HEAP_LIKELY(handle->bins[idx] != NULL)) {
		handle->bins[idx]->prev_free = fragment;
	}

	handle->bins[idx] = fragment;
	handle->nonempty_bin_mask |= pow2(idx);
}

/// Removes the specified block from its bin.
O1HEAP_PRIVATE void unbin(O1HeapInstance *const handle, const Fragment *const fragment);
O1HEAP_PRIVATE void unbin(O1HeapInstance *const handle, const Fragment *const fragment)
{
	O1HEAP_ASSERT(handle != NULL);
	O1HEAP_ASSERT(fragment != NULL);
	O1HEAP_ASSERT(fragment->header.size >= FRAGMENT_SIZE_MIN);
	O1HEAP_ASSERT((fragment->header.size % FRAGMENT_SIZE_MIN) == 0U);
	const uint8_t idx = log2Floor(fragment->header.size / FRAGMENT_SIZE_MIN);  // Round DOWN when removing.
	O1HEAP_ASSERT(idx < NUM_BINS_MAX);

	// Remove the bin from the free fragment list.
	if (O1HEAP_LIKELY(fragment->next_free != NULL)) {
		fragment->next_free->prev_free = fragment->prev_free;
	}

	if (O1HEAP_LIKELY(fragment->prev_free != NULL)) {
		fragment->prev_free->next_free = fragment->next_free;
	}

	// Update the bin header.
	if (O1HEAP_LIKELY(handle->bins[idx] == fragment)) {
		O1HEAP_ASSERT(fragment->prev_free == NULL);
		handle->bins[idx] = fragment->next_free;

		if (O1HEAP_LIKELY(handle->bins[idx] == NULL)) {
			handle->nonempty_bin_mask &= ~pow2(idx);
		}
	}
}

// ---------------------------------------- PUBLIC API IMPLEMENTATION ----------------------------------------

O1HeapInstance *o1heapInit(void *const      base,
			   const size_t     size,
			   const O1HeapHook critical_section_enter,
			   const O1HeapHook critical_section_leave)
{
	O1HeapInstance *out = NULL;

	if ((base != NULL) && ((((size_t) base) % O1HEAP_ALIGNMENT) == 0U) &&
	    (size >= (INSTANCE_SIZE_PADDED + FRAGMENT_SIZE_MIN))) {
		// Allocate the core heap metadata structure in the beginning of the arena.
		O1HEAP_ASSERT(((size_t) base) % sizeof(O1HeapInstance *) == 0U);
		out                         = (O1HeapInstance *) base;
		out->nonempty_bin_mask      = 0U;
		out->critical_section_enter = critical_section_enter;
		out->critical_section_leave = critical_section_leave;

		for (size_t i = 0; i < NUM_BINS_MAX; i++) {
			out->bins[i] = NULL;
		}

		// Limit and align the capacity.
		size_t capacity = size - INSTANCE_SIZE_PADDED;

		if (capacity > FRAGMENT_SIZE_MAX) {
			capacity = FRAGMENT_SIZE_MAX;
		}

		while ((capacity % FRAGMENT_SIZE_MIN) != 0) {
			O1HEAP_ASSERT(capacity > 0U);
			capacity--;
		}

		O1HEAP_ASSERT((capacity % FRAGMENT_SIZE_MIN) == 0);
		O1HEAP_ASSERT((capacity >= FRAGMENT_SIZE_MIN) && (capacity <= FRAGMENT_SIZE_MAX));

		// Initialize the root fragment.
		Fragment *const frag = (Fragment *)(void *)(((uint8_t *) base) + INSTANCE_SIZE_PADDED);
		O1HEAP_ASSERT((((size_t) frag) % O1HEAP_ALIGNMENT) == 0U);
		frag->header.next = NULL;
		frag->header.prev = NULL;
		frag->header.size = capacity;
		frag->header.used = false;
		frag->next_free   = NULL;
		frag->prev_free   = NULL;
		rebin(out, frag);
		O1HEAP_ASSERT(out->nonempty_bin_mask != 0U);

		// Initialize the diagnostics.
		out->diagnostics.capacity          = capacity;
		out->diagnostics.allocated         = 0U;
		out->diagnostics.peak_allocated    = 0U;
		out->diagnostics.peak_request_size = 0U;
		out->diagnostics.oom_count         = 0U;
	}

	return out;
}

void *o1heapAllocate(O1HeapInstance *const handle, const size_t amount)
{
	O1HEAP_ASSERT(handle != NULL);
	O1HEAP_ASSERT(handle->diagnostics.capacity <= FRAGMENT_SIZE_MAX);
	void *out = NULL;

	// If the amount approaches approx. SIZE_MAX/2, an undetected integer overflow may occur.
	// To avoid that, we do not attempt allocation if the amount exceeds the hard limit.
	// We perform multiple redundant checks to account for a possible unaccounted overflow.
	if (O1HEAP_LIKELY((amount > 0U) && (amount <= (handle->diagnostics.capacity - O1HEAP_ALIGNMENT)))) {
		// Add the header size and align the allocation size to the power of 2.
		// See "Timing-Predictable Memory Allocation In Hard Real-Time Systems", Herter, page 27.
		const size_t fragment_size = pow2(log2Ceil(amount + O1HEAP_ALIGNMENT));
		O1HEAP_ASSERT(fragment_size <= FRAGMENT_SIZE_MAX);
		O1HEAP_ASSERT(fragment_size >= FRAGMENT_SIZE_MIN);
		O1HEAP_ASSERT(fragment_size >= amount + O1HEAP_ALIGNMENT);
		O1HEAP_ASSERT(isPowerOf2(fragment_size));

		const uint8_t optimal_bin_index = log2Ceil(fragment_size / FRAGMENT_SIZE_MIN);  // Use CEIL when fetching.
		O1HEAP_ASSERT(optimal_bin_index < NUM_BINS_MAX);
		const size_t candidate_bin_mask = ~(pow2(optimal_bin_index) - 1U);

		invoke(handle->critical_section_enter);

		// Find the smallest non-empty bin we can use.
		const size_t suitable_bins     = handle->nonempty_bin_mask & candidate_bin_mask;
		const size_t smallest_bin_mask = suitable_bins & ~(suitable_bins - 1U);  // Clear all bits but the lowest.

		if (O1HEAP_LIKELY(smallest_bin_mask != 0)) {
			O1HEAP_ASSERT(isPowerOf2(smallest_bin_mask));
			const uint8_t bin_index = log2Floor(smallest_bin_mask);
			O1HEAP_ASSERT(bin_index >= optimal_bin_index);
			O1HEAP_ASSERT(bin_index < NUM_BINS_MAX);

			// The bin we found shall not be empty, otherwise it's a state divergence (memory corruption?).
			Fragment *const frag = handle->bins[bin_index];
			O1HEAP_ASSERT(frag != NULL);
			O1HEAP_ASSERT(frag->header.size >= fragment_size);
			O1HEAP_ASSERT((frag->header.size % FRAGMENT_SIZE_MIN) == 0U);
			O1HEAP_ASSERT(!frag->header.used);
			unbin(handle, frag);

			// Split the fragment if it is too large.
			const size_t leftover = frag->header.size - fragment_size;
			frag->header.size     = fragment_size;
			O1HEAP_ASSERT(leftover < handle->diagnostics.capacity);  // Overflow check.
			O1HEAP_ASSERT(leftover % FRAGMENT_SIZE_MIN == 0U);       // Alignment check.

			if (O1HEAP_LIKELY(leftover >= FRAGMENT_SIZE_MIN)) {
				Fragment *const new_frag = (Fragment *)(void *)(((uint8_t *) frag) + fragment_size);
				O1HEAP_ASSERT(((size_t) new_frag) % O1HEAP_ALIGNMENT == 0U);
				new_frag->header.size = leftover;
				new_frag->header.used = false;
				interlink(new_frag, frag->header.next);
				interlink(frag, new_frag);
				rebin(handle, new_frag);
			}

			// Update the diagnostics.
			O1HEAP_ASSERT((handle->diagnostics.allocated % FRAGMENT_SIZE_MIN) == 0U);
			handle->diagnostics.allocated += fragment_size;
			O1HEAP_ASSERT(handle->diagnostics.allocated <= handle->diagnostics.capacity);

			if (O1HEAP_LIKELY(handle->diagnostics.peak_allocated < handle->diagnostics.allocated)) {
				handle->diagnostics.peak_allocated = handle->diagnostics.allocated;
			}

			// Finalize the fragment we just allocated.
			O1HEAP_ASSERT(frag->header.size >= amount + O1HEAP_ALIGNMENT);
			frag->header.used = true;

			out = ((uint8_t *) frag) + O1HEAP_ALIGNMENT;
		}

	} else {
		invoke(handle->critical_section_enter);
	}

	// Update the diagnostics.
	if (O1HEAP_LIKELY(handle->diagnostics.peak_request_size < amount)) {
		handle->diagnostics.peak_request_size = amount;
	}

	if (O1HEAP_LIKELY((out == NULL) && (amount > 0U))) {
		handle->diagnostics.oom_count++;
	}

	invoke(handle->critical_section_leave);
	return out;
}

void o1heapFree(O1HeapInstance *const handle, void *const pointer)
{
	O1HEAP_ASSERT(handle != NULL);
	O1HEAP_ASSERT(handle->diagnostics.capacity <= FRAGMENT_SIZE_MAX);

	if (O1HEAP_LIKELY(pointer != NULL)) { // NULL pointer is a no-op.
		Fragment *const frag = (Fragment *)(void *)(((uint8_t *) pointer) - O1HEAP_ALIGNMENT);

		// Check for heap corruption in debug builds.
		O1HEAP_ASSERT(((size_t) frag) % sizeof(Fragment *) == 0U);
		O1HEAP_ASSERT(((size_t) frag) >= (((size_t) handle) + INSTANCE_SIZE_PADDED));
		O1HEAP_ASSERT(((size_t) frag) <=
			      (((size_t) handle) + INSTANCE_SIZE_PADDED + handle->diagnostics.capacity - FRAGMENT_SIZE_MIN));
		O1HEAP_ASSERT(frag->header.used);  // Catch double-free
		O1HEAP_ASSERT(((size_t) frag->header.next) % sizeof(Fragment *) == 0U);
		O1HEAP_ASSERT(((size_t) frag->header.prev) % sizeof(Fragment *) == 0U);
		O1HEAP_ASSERT(frag->header.size >= FRAGMENT_SIZE_MIN);
		O1HEAP_ASSERT(frag->header.size <= handle->diagnostics.capacity);
		O1HEAP_ASSERT((frag->header.size % FRAGMENT_SIZE_MIN) == 0U);

		invoke(handle->critical_section_enter);

		// Even if we're going to drop the fragment later, mark it free anyway to prevent double-free.
		frag->header.used = false;

		// Update the diagnostics. It must be done before merging because it invalidates the fragment size information.
		O1HEAP_ASSERT(handle->diagnostics.allocated >= frag->header.size);  // Heap corruption check.
		handle->diagnostics.allocated -= frag->header.size;

		// Merge with siblings and insert the returned fragment into the appropriate bin and update metadata.
		Fragment *const prev       = frag->header.prev;
		Fragment *const next       = frag->header.next;
		const bool      join_left  = (prev != NULL) && (!prev->header.used);
		const bool      join_right = (next != NULL) && (!next->header.used);

		if (join_left && join_right) { // [ prev ][ this ][ next ] => [ ------- prev ------- ]
			unbin(handle, prev);
			unbin(handle, next);
			prev->header.size += frag->header.size + next->header.size;
			frag->header.size = 0;  // Invalidate the dropped fragment headers to prevent double-free.
			next->header.size = 0;
			O1HEAP_ASSERT((prev->header.size % FRAGMENT_SIZE_MIN) == 0U);
			interlink(prev, next->header.next);
			rebin(handle, prev);

		} else if (join_left) { // [ prev ][ this ][ next ] => [ --- prev --- ][ next ]
			unbin(handle, prev);
			prev->header.size += frag->header.size;
			frag->header.size = 0;
			O1HEAP_ASSERT((prev->header.size % FRAGMENT_SIZE_MIN) == 0U);
			interlink(prev, next);
			rebin(handle, prev);

		} else if (join_right) { // [ prev ][ this ][ next ] => [ prev ][ --- this --- ]
			unbin(handle, next);
			frag->header.size += next->header.size;
			next->header.size = 0;
			O1HEAP_ASSERT((frag->header.size % FRAGMENT_SIZE_MIN) == 0U);
			interlink(frag, next->header.next);
			rebin(handle, frag);

		} else {
			rebin(handle, frag);
		}

		invoke(handle->critical_section_leave);
	}
}

bool o1heapDoInvariantsHold(const O1HeapInstance *const handle)
{
	O1HEAP_ASSERT(handle != NULL);
	bool valid = true;

	invoke(handle->critical_section_enter);

	// Check the bin mask consistency.
	for (size_t i = 0; i < NUM_BINS_MAX; i++) { // Dear compiler, feel free to unroll this loop.
		const bool mask_bit_set = (handle->nonempty_bin_mask & pow2((uint8_t) i)) != 0U;
		const bool bin_nonempty = handle->bins[i] != NULL;
		valid                   = valid && (mask_bit_set == bin_nonempty);
	}

	// Create a local copy of the diagnostics struct to check later and release the critical section early.
	const O1HeapDiagnostics diag = handle->diagnostics;

	invoke(handle->critical_section_leave);

	// Capacity check.
	valid = valid && (diag.capacity <= FRAGMENT_SIZE_MAX) && (diag.capacity >= FRAGMENT_SIZE_MIN) &&
		((diag.capacity % FRAGMENT_SIZE_MIN) == 0U);

	// Allocation info check.
	valid = valid && (diag.allocated <= diag.capacity) && ((diag.allocated % FRAGMENT_SIZE_MIN) == 0U) &&
		(diag.peak_allocated <= diag.capacity) && (diag.peak_allocated >= diag.allocated) &&
		((diag.peak_allocated % FRAGMENT_SIZE_MIN) == 0U);

	// Peak request check
	valid = valid && ((diag.peak_request_size < diag.capacity) || (diag.oom_count > 0U));

	if (diag.peak_request_size == 0U) {
		valid = valid && (diag.peak_allocated == 0U) && (diag.allocated == 0U) && (diag.oom_count == 0U);

	} else {
		valid = valid &&  // Overflow on summation is possible but safe to ignore.
			(((diag.peak_request_size + O1HEAP_ALIGNMENT) <= diag.peak_allocated) || (diag.oom_count > 0U));
	}

	return valid;
}

O1HeapDiagnostics o1heapGetDiagnostics(const O1HeapInstance *const handle)
{
	O1HEAP_ASSERT(handle != NULL);
	invoke(handle->critical_section_enter);
	const O1HeapDiagnostics out = handle->diagnostics;
	invoke(handle->critical_section_leave);
	return out;
}
