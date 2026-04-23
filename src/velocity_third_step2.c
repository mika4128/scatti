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
#include <stdbool.h>

#include <scatti/velocity.h>
#include <scatti/block.h>
#include <scatti/profile.h>

/* ---- Internal helper functions ---- */

static bool time_acc0(SCattiVelocityThirdOrderStep2 *s, SCattiProfile *profile,
                      double aMax, double aMin, double jMax)
{
    /* UD Solution 1/2 */
    {
        const double h1 = sqrt((-s->ad * s->ad + 2 * jMax * ((s->a0 + s->af) * s->tf - 2 * s->vd)) / (jMax * jMax) + s->tf * s->tf);

        profile->t[0] = s->ad / (2 * jMax) + (s->tf - h1) / 2;
        profile->t[1] = h1;
        profile->t[2] = s->tf - (profile->t[0] + h1);
        profile->t[3] = 0;
        profile->t[4] = 0;
        profile->t[5] = 0;
        profile->t[6] = 0;

        if (scatti_profile_check_for_velocity_with_timing(profile, ControlSignsUDDU, ReachedLimitsACC0, s->tf, jMax, aMax, aMin)) {
            profile->pf = profile->p[7];
            return true;
        }
    }

    /* UU Solution */
    {
        const double h1 = (-s->ad + jMax * s->tf);

        profile->t[0] = -s->ad * s->ad / (2 * jMax * h1) + (s->vd - s->a0 * s->tf) / h1;
        profile->t[1] = -s->ad / jMax + s->tf;
        profile->t[2] = 0;
        profile->t[3] = 0;
        profile->t[4] = 0;
        profile->t[5] = 0;
        profile->t[6] = s->tf - (profile->t[0] + profile->t[1]);

        if (scatti_profile_check_for_velocity_with_timing(profile, ControlSignsUDDU, ReachedLimitsACC0, s->tf, jMax, aMax, aMin)) {
            profile->pf = profile->p[7];
            return true;
        }
    }

    /* UU Solution - 2 step */
    {
        profile->t[0] = 0;
        profile->t[1] = -s->ad / jMax + s->tf;
        profile->t[2] = 0;
        profile->t[3] = 0;
        profile->t[4] = 0;
        profile->t[5] = 0;
        profile->t[6] = s->ad / jMax;

        if (scatti_profile_check_for_velocity_with_timing(profile, ControlSignsUDDU, ReachedLimitsACC0, s->tf, jMax, aMax, aMin)) {
            profile->pf = profile->p[7];
            return true;
        }
    }

    return false;
}

static bool time_none(SCattiVelocityThirdOrderStep2 *s, SCattiProfile *profile,
                      double aMax, double aMin, double jMax)
{
    if (fabs(s->a0) < DBL_EPSILON && fabs(s->af) < DBL_EPSILON && fabs(s->vd) < DBL_EPSILON) {
        profile->t[0] = 0;
        profile->t[1] = s->tf;
        profile->t[2] = 0;
        profile->t[3] = 0;
        profile->t[4] = 0;
        profile->t[5] = 0;
        profile->t[6] = 0;

        if (scatti_profile_check_for_velocity_with_timing(profile, ControlSignsUDDU, ReachedLimitsNONE, s->tf, jMax, aMax, aMin)) {
            profile->pf = profile->p[7];
            return true;
        }
    }

    /* UD Solution 1/2 */
    {
        const double h1 = 2 * (s->af * s->tf - s->vd);

        profile->t[0] = h1 / s->ad;
        profile->t[1] = s->tf - profile->t[0];
        profile->t[2] = 0;
        profile->t[3] = 0;
        profile->t[4] = 0;
        profile->t[5] = 0;
        profile->t[6] = 0;

        const double jf = s->ad * s->ad / h1;

        if (fabs(jf) < fabs(jMax) + 1e-12 && scatti_profile_check_for_velocity_with_timing(profile, ControlSignsUDDU, ReachedLimitsNONE, s->tf, jf, aMax, aMin)) {
            profile->pf = profile->p[7];
            return true;
        }
    }

    return false;
}

static bool check_all(SCattiVelocityThirdOrderStep2 *s, SCattiProfile *profile,
                      double aMax, double aMin, double jMax)
{
    return time_acc0(s, profile, aMax, aMin, jMax) || time_none(s, profile, aMax, aMin, jMax);
}


/* ---- Public interface ---- */

void scatti_vel3_step2_init(SCattiVelocityThirdOrderStep2 *s,
                     double tf, double v0, double a0, double vf, double af,
                     double aMax, double aMin, double jMax)
{
    s->a0 = a0;
    s->tf = tf;
    s->af = af;
    s->_aMax = aMax;
    s->_aMin = aMin;
    s->_jMax = jMax;
    s->vd = vf - v0;
    s->ad = af - a0;
}

bool scatti_vel3_step2_get_profile(SCattiVelocityThirdOrderStep2 *s, SCattiProfile *profile)
{
    /* Test all cases to get ones that match */
    /* However we should guess which one is correct and try them first... */
    if (s->vd > 0) {
        return check_all(s, profile, s->_aMax, s->_aMin, s->_jMax) || check_all(s, profile, s->_aMin, s->_aMax, -s->_jMax);
    }

    return check_all(s, profile, s->_aMin, s->_aMax, -s->_jMax) || check_all(s, profile, s->_aMax, s->_aMin, s->_jMax);
}
