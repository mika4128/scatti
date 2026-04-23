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

/*
 * Optional: polynomial-based cube root for environments without libm cbrt().
 *
 * Compile this file into scatti only when SCATTI_PROVIDE_FAST_CBRT is defined
 * (see CMake option SCATTI_PROVIDE_FAST_CBRT / -DSCATTI_PROVIDE_FAST_CBRT=ON).
 * Then, to actually use it instead of libm, define the SCATTI_CBRT macro to
 * point at scatti_cbrt() before including any scatti header — for example in
 * a LinuxCNC build:
 *
 *   #define SCATTI_CBRT(x) scatti_cbrt(x)
 *
 * Implementation adapted from musl libc / FreeBSD libmsun:
 *   polynomial approximation to 23 bits + one Newton step to 53 bits,
 *   error < 0.667 ulps.
 *
 * Copyright (c) 1993 Sun Microsystems, Inc.
 * Copyright (c) 2005-2020 Rich Felker, et al. (musl libc)  — MIT license.
 */

#include <scatti/scatti_config.h>

#ifdef SCATTI_PROVIDE_FAST_CBRT

double scatti_cbrt(double x) {
    static const unsigned B1 = 715094163;
    static const unsigned B2 = 696219795;
    static const double P0 =  1.87595182427177009643;
    static const double P1 = -1.88497979543377169875;
    static const double P2 =  1.621429720105354466140;
    static const double P3 = -0.758397934778766047437;
    static const double P4 =  0.145996192886612446982;
    union { double f; unsigned long long i; } u = {x};
    double r, s, t, w;
    unsigned hx = u.i >> 32 & 0x7fffffff;

    if (hx >= 0x7ff00000)
        return x + x;

    if (hx < 0x00100000) {
        u.f = x * 0x1p54;
        hx = u.i >> 32 & 0x7fffffff;
        if (hx == 0) return x;
        hx = hx / 3 + B2;
    } else {
        hx = hx / 3 + B1;
    }
    u.i &= 1ULL << 63;
    u.i |= (unsigned long long)hx << 32;
    t = u.f;

    r = (t * t) * (t / x);
    t = t * ((P0 + r * (P1 + r * P2)) + ((r * r) * r) * (P3 + r * P4));

    u.f = t;
    u.i = (u.i + 0x80000000) & 0xffffffffc0000000ULL;
    t = u.f;

    s = t * t;
    r = x / s;
    w = t + t;
    r = (r - t) / (w + r);
    t = t + t * r;
    return t;
}

#endif /* SCATTI_PROVIDE_FAST_CBRT */
