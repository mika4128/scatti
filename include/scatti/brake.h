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

#ifndef SCATTI_BRAKE_H
#define SCATTI_BRAKE_H

#include <stdbool.h>

/* Two-phase brake profile */
typedef struct {
    double duration;
    double t[2];
    double j[2];
    double a[2];
    double v[2];
    double p[2];
} SCattiBrakeProfile;

void scatti_brake_init(SCattiBrakeProfile *bp);

/* Calculate brake trajectories */
void scatti_brake_get_position_brake_trajectory(SCattiBrakeProfile *bp, double v0, double a0,
                                         double vMax, double vMin, double aMax, double aMin, double jMax);
void scatti_brake_get_second_order_position_brake_trajectory(SCattiBrakeProfile *bp, double v0,
                                                      double vMax, double vMin, double aMax, double aMin);
void scatti_brake_get_velocity_brake_trajectory(SCattiBrakeProfile *bp, double a0,
                                         double aMax, double aMin, double jMax);
void scatti_brake_get_second_order_velocity_brake_trajectory(SCattiBrakeProfile *bp);

/* Finalize by integrating */
void scatti_brake_finalize(SCattiBrakeProfile *bp, double *ps, double *vs, double *as);
void scatti_brake_finalize_second_order(SCattiBrakeProfile *bp, double *ps, double *vs, double *as);

#endif /* SCATTI_BRAKE_H */
