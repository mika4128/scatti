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

#include <scatti/scatti_config.h>
#include <scatti/velocity.h>
#include <scatti/block.h>
#include <scatti/profile.h>

void scatti_vel2_step2_init(SCattiVelocitySecondOrderStep2 *s,
                     double tf, double v0, double vf, double aMax, double aMin)
{
    s->tf = tf;
    s->_aMax = aMax;
    s->_aMin = aMin;
    s->vd = vf - v0;
}

bool scatti_vel2_step2_get_profile(SCattiVelocitySecondOrderStep2 *s, SCattiProfile *profile)
{
    const double af = s->vd / s->tf;

    profile->t[0] = 0;
    profile->t[1] = s->tf;
    profile->t[2] = 0;
    profile->t[3] = 0;
    profile->t[4] = 0;
    profile->t[5] = 0;
    profile->t[6] = 0;

    if (scatti_profile_check_for_second_order_velocity_with_timing_full(profile, ControlSignsUDDU, ReachedLimitsNONE, s->tf, af, s->_aMax, s->_aMin)) {
        profile->pf = profile->p[7];
        return true;
    }

    return false;
}
