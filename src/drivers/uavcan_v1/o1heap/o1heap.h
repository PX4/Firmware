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
//
// READ THE DOCUMENTATION IN README.md.

#ifndef O1HEAP_H_INCLUDED
#define O1HEAP_H_INCLUDED

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/// The semantic version number of this distribution.
#define O1HEAP_VERSION_MAJOR 1

/// The guaranteed alignment depends on the platform pointer width.
#define O1HEAP_ALIGNMENT (sizeof(void*) * 4U)

/// The definition is private, so the user code can only operate on pointers. This is done to enforce encapsulation.
typedef struct O1HeapInstance O1HeapInstance;

/// A hook function invoked by the allocator. NULL hooks are silently not invoked (not an error).
typedef void (*O1HeapHook)(void);

/// Runtime diagnostic information. This information can be used to facilitate runtime self-testing,
/// as required by certain safety-critical development guidelines.
/// If assertion checks are not disabled, the library will perform automatic runtime self-diagnostics that trigger
/// an assertion failure if a heap corruption is detected.
/// Health checks and validation can be done with @ref o1heapDoInvariantsHold().
typedef struct {
	/// The total amount of memory available for serving allocation requests (heap size).
	/// The maximum allocation size is (capacity - O1HEAP_ALIGNMENT).
	/// This parameter does not include the overhead used up by @ref O1HeapInstance and arena alignment.
	/// This parameter is constant.
	size_t capacity;

	/// The amount of memory that is currently allocated, including the per-fragment overhead and size alignment.
	/// For example, if the application requested a fragment of size 1 byte, the value reported here may be 32 bytes.
	size_t allocated;

	/// The maximum value of 'allocated' seen since initialization. This parameter is never decreased.
	size_t peak_allocated;

	/// The largest amount of memory that the allocator has attempted to allocate (perhaps unsuccessfully)
	/// since initialization (not including the rounding and the allocator's own per-fragment overhead,
	/// so the total is larger). This parameter is never decreased. The initial value is zero.
	size_t peak_request_size;

	/// The number of times an allocation request could not be completed due to the lack of memory or
	/// excessive fragmentation. OOM stands for "out of memory". This parameter is never decreased.
	uint64_t oom_count;
} O1HeapDiagnostics;

/// The arena base pointer shall be aligned at @ref O1HEAP_ALIGNMENT, otherwise NULL is returned.
///
/// The total heap capacity cannot exceed approx. (SIZE_MAX/2). If the arena size allows for a larger heap,
/// the excess will be silently truncated away (no error). This is not a realistic use case because a typical
/// application is unlikely to be able to dedicate that much of the address space for the heap.
///
/// The critical section enter/leave callbacks will be invoked when the allocator performs an atomic transaction.
/// There is at most one atomic transaction per allocation/deallocation.
/// Either or both of the callbacks may be NULL if locking is not needed (i.e., the heap is not shared).
/// It is guaranteed that a critical section will never be entered recursively.
/// It is guaranteed that 'enter' is invoked the same number of times as 'leave', unless either of them are NULL.
/// It is guaranteed that 'enter' is invoked before 'leave', unless either of them are NULL.
/// The callbacks are never invoked from the initialization function itself.
///
/// The function initializes a new heap instance allocated in the provided arena, taking some of its space for its
/// own needs (normally about 40..600 bytes depending on the architecture, but this parameter is not characterized).
/// A pointer to the newly initialized instance is returned.
///
/// If the provided space is insufficient, NULL is returned.
///
/// An initialized instance does not hold any resources. Therefore, if the instance is no longer needed,
/// it can be discarded without any de-initialization procedures.
///
/// The time complexity is unspecified.
O1HeapInstance *o1heapInit(void *const      base,
			   const size_t     size,
			   const O1HeapHook critical_section_enter,
			   const O1HeapHook critical_section_leave);

/// The semantics follows malloc() with additional guarantees the full list of which is provided below.
///
/// If the allocation request is served successfully, a pointer to the newly allocated memory fragment is returned.
/// The returned pointer is guaranteed to be aligned at @ref O1HEAP_ALIGNMENT.
///
/// If the allocation request cannot be served due to the lack of memory or its excessive fragmentation,
/// a NULL pointer is returned.
///
/// The function is executed in constant time (unless the critical section management hooks are used and are not
/// constant-time). The allocated memory is NOT zero-filled (because zero-filling is a variable-complexity operation).
///
/// The function may invoke critical_section_enter and critical_section_leave at most once each (NULL hooks ignored).
void *o1heapAllocate(O1HeapInstance *const handle, const size_t amount);

/// The semantics follows free() with additional guarantees the full list of which is provided below.
///
/// If the pointer does not point to a previously allocated block and is not NULL, the behavior is undefined.
/// Builds where assertion checks are enabled may trigger an assertion failure for some invalid inputs.
///
/// The function is executed in constant time (unless the critical section management hooks are used and are not
/// constant-time).
///
/// The function may invoke critical_section_enter and critical_section_leave at most once each (NULL hooks ignored).
void o1heapFree(O1HeapInstance *const handle, void *const pointer);

/// Performs a basic sanity check on the heap.
/// This function can be used as a weak but fast method of heap corruption detection.
/// It invokes critical_section_enter once (unless NULL) and then critical_section_leave once (unless NULL).
/// If the handle pointer is NULL, the behavior is undefined.
/// The time complexity is constant.
/// The return value is truth if the heap looks valid, falsity otherwise.
bool o1heapDoInvariantsHold(const O1HeapInstance *const handle);

/// Samples and returns a copy of the diagnostic information, see @ref O1HeapDiagnostics.
/// This function merely copies the structure from an internal storage, so it is fast to return.
/// It invokes critical_section_enter once (unless NULL) and then critical_section_leave once (unless NULL).
/// If the handle pointer is NULL, the behavior is undefined.
O1HeapDiagnostics o1heapGetDiagnostics(const O1HeapInstance *const handle);

#ifdef __cplusplus
}
#endif
#endif  // O1HEAP_H_INCLUDED
