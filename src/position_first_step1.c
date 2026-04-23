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
