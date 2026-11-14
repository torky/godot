/**************************************************************************/
/*  fp_deterministic.h                                                    */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#pragma once

// Enable flush-to-zero (FTZ) and denormals-are-zero (DAZ) for deterministic
// floating-point behavior. This improves performance and ensures consistent
// results across different CPUs.
//
// Call fp_deterministic_init() once at application startup (before any FP math).

#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
// x86/x64: Use SSE MXCSR register
// FTZ (bit 15): Flush denormal OUTPUTS to zero
// DAZ (bit 6): Treat denormal INPUTS as zero
// Both are needed for full determinism
#include <xmmintrin.h> // SSE: _MM_FLUSH_ZERO_ON
#include <pmmintrin.h> // SSE3: _MM_DENORMALS_ZERO_ON

inline void fp_deterministic_init() {
	_mm_setcsr(_mm_getcsr() | _MM_FLUSH_ZERO_ON | _MM_DENORMALS_ZERO_ON);
}

#elif defined(__aarch64__) || defined(_M_ARM64)
// ARM64: Use FPCR register
// FZ (bit 24): Flush-to-Zero for both inputs and outputs
// DN (bit 25): Default NaN mode - ensures consistent NaN propagation
// FZ16 (bit 19): Flush-to-Zero for FP16 (optional, set for completeness)

#if defined(_MSC_VER)
// MSVC ARM64
#include <intrin.h>
inline void fp_deterministic_init() {
	unsigned long long fpcr = _ReadStatusReg(0x5A20); // FPCR
	fpcr |= (1ULL << 24); // FZ bit
	fpcr |= (1ULL << 25); // DN bit (default NaN)
	fpcr |= (1ULL << 19); // FZ16 bit
	_WriteStatusReg(0x5A20, fpcr);
}
#else
// GCC/Clang ARM64
inline void fp_deterministic_init() {
	unsigned long fpcr;
	__asm__ __volatile__("mrs %0, fpcr" : "=r"(fpcr));
	fpcr |= (1UL << 24); // FZ bit
	fpcr |= (1UL << 25); // DN bit (default NaN)
	fpcr |= (1UL << 19); // FZ16 bit
	__asm__ __volatile__("msr fpcr, %0" : : "r"(fpcr));
}
#endif

#elif defined(__arm__) || defined(_M_ARM)
// ARM32: Use FPSCR register
// Bit 24 = FZ (Flush-to-Zero)

#if defined(_MSC_VER)
// MSVC ARM32
#include <intrin.h>
inline void fp_deterministic_init() {
	unsigned int fpscr;
	_controlfp_s(&fpscr, _DN_FLUSH, _MCW_DN);
}
#elif defined(__ARM_FP)
// GCC/Clang ARM32 with hardware FP
inline void fp_deterministic_init() {
	unsigned int fpscr;
	__asm__ __volatile__("vmrs %0, fpscr" : "=r"(fpscr));
	fpscr |= (1 << 24); // FZ bit
	__asm__ __volatile__("vmsr fpscr, %0" : : "r"(fpscr));
}
#else
// Soft float - no hardware FP control
inline void fp_deterministic_init() {
	// No-op for soft float
}
#endif

#elif defined(__EMSCRIPTEN__)
// WebAssembly: Denormal handling is implementation-defined
// Most browsers flush denormals by default, but we can't control it
inline void fp_deterministic_init() {
	// WebAssembly doesn't provide direct FP control word access
	// Browsers typically handle denormals efficiently already
}

#elif defined(__riscv)
// RISC-V: No standard denormal flush mechanism
inline void fp_deterministic_init() {
	// RISC-V doesn't have a standard FTZ mode
}

#elif defined(__PPC__) || defined(__ppc__) || defined(__powerpc__)
// PowerPC: FPSCR register
inline void fp_deterministic_init() {
	// PowerPC denormal handling is complex and varies by implementation
	// Most modern PPC handles denormals in hardware
}

#else
// Unknown architecture - no-op
inline void fp_deterministic_init() {
}

#endif
