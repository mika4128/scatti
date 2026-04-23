/********************************************************************
* Permission is hereby granted, free of charge, to any person obtaining
* a copy of this software and associated documentation files (the
* "Software"), to deal in the Software without restriction, including
* without limitation the rights to use, copy, modify, merge, publish,
* distribute, sublicense, and/or sell copies of the Software, and to
* permit persons to whom the Software is furnished to do so, subject
* to the following conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*
* Description: scatti - pure C99 jerk-limited real-time trajectory
*              generation. Port of Ruckig with embed-friendly
*              memory and math hooks.
* Author:      杨阳 (Yang Yang) <mika-net@outlook.com>
* Origin:      Based on Ruckig Community Edition by Lars Berscheid
*              (https://github.com/pantor/ruckig)
* License:     MIT (SPDX-License-Identifier: MIT)
* Copyright (c) 2026 杨阳 (Yang Yang)
********************************************************************/

#ifndef SCATTI_CONFIG_H
#define SCATTI_CONFIG_H

/*
 * scatti Performance Configuration
 *
 * These macros control optimization levels. They can be set via CMake options
 * or defined before including any scatti header.
 *
 * SCATTI_OPT_LEVEL:
 *   0 = Default (portable, no special flags)
 *   1 = Optimized (LTO + O3)
 *   2 = Aggressive (LTO + O3 + march=native + fast-math)
 */

/* Compiler detection */
#if defined(__GNUC__) || defined(__clang__)
#  define SCATTI_GCC_COMPAT 1
#else
#  define SCATTI_GCC_COMPAT 0
#endif

/* Branch prediction hints */
#if SCATTI_GCC_COMPAT
#  define SCATTI_LIKELY(x)   __builtin_expect(!!(x), 1)
#  define SCATTI_UNLIKELY(x) __builtin_expect(!!(x), 0)
#else
#  define SCATTI_LIKELY(x)   (x)
#  define SCATTI_UNLIKELY(x) (x)
#endif

/* Force inline for critical hot-path functions */
#if SCATTI_GCC_COMPAT
#  define SCATTI_FORCE_INLINE static inline __attribute__((always_inline))
#  define SCATTI_HOT __attribute__((hot))
#else
#  define SCATTI_FORCE_INLINE static inline
#  define SCATTI_HOT
#endif

/* restrict keyword (C99) */
#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 199901L
#  define SCATTI_RESTRICT restrict
#elif SCATTI_GCC_COMPAT
#  define SCATTI_RESTRICT __restrict__
#elif defined(_MSC_VER)
#  define SCATTI_RESTRICT __restrict
#else
#  define SCATTI_RESTRICT
#endif

/* Prefetch hint for upcoming memory access */
#if SCATTI_GCC_COMPAT
#  define SCATTI_PREFETCH(addr) __builtin_prefetch(addr, 0, 1)
#else
#  define SCATTI_PREFETCH(addr) ((void)0)
#endif

/*
 * ============================================================================
 * Host-environment overrides (memory + math)
 * ============================================================================
 * Defaults map to libc. Embedders (e.g. LinuxCNC kernel module) may predefine
 * these macros to redirect to RTAPI / kernel-side replacements BEFORE including
 * any scatti header. Example for LinuxCNC RTAPI:
 *
 *   #define SCATTI_MALLOC(sz)      rtapi_kmalloc(sz, RTAPI_GFP_KERNEL)
 *   #define SCATTI_CALLOC(n, sz)   rtapi_kzalloc((n) * (sz), RTAPI_GFP_KERNEL)
 *   #define SCATTI_REALLOC(p, sz)  rtapi_krealloc(p, sz, RTAPI_GFP_KERNEL)
 *   #define SCATTI_FREE(p)         rtapi_kfree(p)
 *   #define SCATTI_CBRT(x)         scatti_cbrt(x)
 *
 * Overrides can also be injected via the build system, e.g.
 *   cmake -DSCATTI_EMBED_LINUXCNC=ON
 */

#ifndef SCATTI_MALLOC
#  include <stdlib.h>
#  define SCATTI_MALLOC(sz)      malloc(sz)
#endif
#ifndef SCATTI_CALLOC
#  include <stdlib.h>
#  define SCATTI_CALLOC(n, sz)   calloc((n), (sz))
#endif
#ifndef SCATTI_REALLOC
#  include <stdlib.h>
#  define SCATTI_REALLOC(p, sz)  realloc((p), (sz))
#endif
#ifndef SCATTI_FREE
#  include <stdlib.h>
#  define SCATTI_FREE(p)         free(p)
#endif

/* Cube root: libm by default; embedders may supply a no-libm replacement. */
#ifndef SCATTI_CBRT
#  include <math.h>
#  define SCATTI_CBRT(x)         cbrt(x)
#endif

/*
 * ============================================================================
 * Optional "extra" solver functions preserved for reference / future use
 * ============================================================================
 * Define SCATTI_ENABLE_EXTRA_SOLVERS to compile additional solver variants
 * that are not wired into the current dispatcher (e.g. smooth-phase synchroni-
 * zation candidates). Default: disabled, so downstream builds (LinuxCNC) stay
 * warning-clean under -Wunused-function.
 *
 * See docs/linuxcnc_integration.md for the inventory.
 */
/* #define SCATTI_ENABLE_EXTRA_SOLVERS */

#endif /* SCATTI_CONFIG_H */
