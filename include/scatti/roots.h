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

#ifndef SCATTI_ROOTS_H
#define SCATTI_ROOTS_H

#include <stddef.h>
#include <scatti/scatti_config.h>

/* A set of positive double roots, stored on the stack */
typedef struct {
    double data[4];
    size_t size;
} SCattiRootSet;

SCATTI_FORCE_INLINE void scatti_root_set_init(SCattiRootSet *s) {
    s->size = 0;
}

SCATTI_FORCE_INLINE void scatti_root_set_insert(SCattiRootSet *s, double value) {
    if (value >= 0.0) {
        s->data[s->size] = value;
        s->size++;
    }
}

/* Sort the root set (simple insertion sort for small N) */
void scatti_root_set_sort(SCattiRootSet *s);

/* Solve a*x^3 + b*x^2 + c*x + d = 0, returning positive roots */
SCattiRootSet scatti_roots_solve_cubic(double a, double b, double c, double d);

/* Solve resolvent equation, returns number of zeros */
int scatti_roots_solve_resolvent(double x[3], double a, double b, double c);

/* Solve monic quartic x^4 + a*x^3 + b*x^2 + c*x + d = 0 */
SCattiRootSet scatti_roots_solve_quart_monic(double a, double b, double c, double d);

/* Evaluate polynomial of order N at x. Coefficients in descending order: p[0]*x^(N-1) + ... + p[N-1] */
double scatti_roots_poly_eval(const double *p, size_t n, double x);

/* Calculate derivative coefficients */
void scatti_roots_poly_derivative(const double *coeffs, size_t n, double *deriv);

/* Safe Newton method: find root in [l, h] where p(l)*p(h) < 0 */
double scatti_roots_shrink_interval(const double *p, size_t n, double l, double h);

/*
 * Optional no-libm cube root. Compiled only when SCATTI_PROVIDE_FAST_CBRT is
 * defined at build time (see src/scatti_cbrt.c). To actually redirect internal
 * cbrt calls to this implementation, also set the SCATTI_CBRT macro:
 *   #define SCATTI_CBRT(x) scatti_cbrt(x)
 */
#ifdef SCATTI_PROVIDE_FAST_CBRT
double scatti_cbrt(double x);
#endif

#endif /* SCATTI_ROOTS_H */
