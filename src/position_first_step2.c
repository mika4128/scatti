#include <scatti/scatti_config.h>
#include <math.h>
#include <float.h>

#include <scatti/position.h>
#include <scatti/block.h>
#include <scatti/profile.h>
#include <scatti/roots.h>

void scatti_pos1_step2_init(SCattiPositionFirstOrderStep2 *s,
                     double tf, double p0, double pf, double vMax, double vMin)
{
    s->tf = tf;
    s->_vMax = vMax;
    s->_vMin = vMin;
    s->pd = pf - p0;
}

bool scatti_pos1_step2_get_profile(SCattiPositionFirstOrderStep2 *s, SCattiProfile *profile)
{
    const double vf = s->pd / s->tf;

    profile->t[0] = 0;
    profile->t[1] = 0;
    profile->t[2] = 0;
    profile->t[3] = s->tf;
    profile->t[4] = 0;
    profile->t[5] = 0;
    profile->t[6] = 0;

    return scatti_profile_check_for_first_order_with_timing_full(profile, ControlSignsUDDU, ReachedLimitsNONE,
                                                          s->tf, vf, s->_vMax, s->_vMin);
}
