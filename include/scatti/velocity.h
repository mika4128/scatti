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

#ifndef SCATTI_VELOCITY_H
#define SCATTI_VELOCITY_H

#include <stdbool.h>
#include <scatti/profile.h>
#include <scatti/block.h>

/* ---- Third Order Step 1 ---- */
typedef struct {
    double a0, af;
    double _aMax, _aMin, _jMax;
    double vd;
    SCattiProfile valid_profiles[3];
} SCattiVelocityThirdOrderStep1;

void scatti_vel3_step1_init(SCattiVelocityThirdOrderStep1 *s,
                     double v0, double a0, double vf, double af,
                     double aMax, double aMin, double jMax);
bool scatti_vel3_step1_get_profile(SCattiVelocityThirdOrderStep1 *s,
                            const SCattiProfile *input, SCattiBlock *block);

/* ---- Third Order Step 2 ---- */
typedef struct {
    double a0, tf, af;
    double _aMax, _aMin, _jMax;
    double vd, ad;
} SCattiVelocityThirdOrderStep2;

void scatti_vel3_step2_init(SCattiVelocityThirdOrderStep2 *s,
                     double tf, double v0, double a0, double vf, double af,
                     double aMax, double aMin, double jMax);
bool scatti_vel3_step2_get_profile(SCattiVelocityThirdOrderStep2 *s, SCattiProfile *profile);

/* ---- Second Order Step 1 ---- */
typedef struct {
    double _aMax, _aMin;
    double vd;
} SCattiVelocitySecondOrderStep1;

void scatti_vel2_step1_init(SCattiVelocitySecondOrderStep1 *s,
                     double v0, double vf, double aMax, double aMin);
bool scatti_vel2_step1_get_profile(SCattiVelocitySecondOrderStep1 *s,
                            const SCattiProfile *input, SCattiBlock *block);

/* ---- Second Order Step 2 ---- */
typedef struct {
    double tf;
    double _aMax, _aMin;
    double vd;
} SCattiVelocitySecondOrderStep2;

void scatti_vel2_step2_init(SCattiVelocitySecondOrderStep2 *s,
                     double tf, double v0, double vf, double aMax, double aMin);
bool scatti_vel2_step2_get_profile(SCattiVelocitySecondOrderStep2 *s, SCattiProfile *profile);

#endif /* SCATTI_VELOCITY_H */
