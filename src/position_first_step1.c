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
#include <math.h>
#include <float.h>

#include <scatti/position.h>
#include <scatti/block.h>
#include <scatti/profile.h>

void scatti_pos1_step1_init(SCattiPositionFirstOrderStep1 *s,
                     double p0, double pf, double vMax, double vMin)
{
    s->_vMax = vMax;
    s->_vMin = vMin;
    s->pd = pf - p0;
}

bool scatti_pos1_step1_get_profile(SCattiPositionFirstOrderStep1 *s,
                            const SCattiProfile *input, SCattiBlock *block)
{
    SCattiProfile *p = &block->p_min;
    scatti_profile_set_boundary_from_profile(p, input);

    const double vf = (s->pd > 0) ? s->_vMax : s->_vMin;
    p->t[0] = 0;
    p->t[1] = 0;
    p->t[2] = 0;
    p->t[3] = s->pd / vf;
    p->t[4] = 0;
    p->t[5] = 0;
    p->t[6] = 0;

    if (scatti_profile_check_for_first_order(p, ControlSignsUDDU, ReachedLimitsVEL, vf)) {
        block->t_min = p->t_sum[6] + p->brake.duration + p->accel.duration;
        return true;
    }
    return false;
}
