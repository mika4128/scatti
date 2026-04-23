#include <scatti/scatti_config.h>
#include <scatti/velocity.h>
#include <scatti/block.h>
#include <scatti/profile.h>

void scatti_vel2_step1_init(SCattiVelocitySecondOrderStep1 *s,
                     double v0, double vf, double aMax, double aMin)
{
    s->_aMax = aMax;
    s->_aMin = aMin;
    s->vd = vf - v0;
}

bool scatti_vel2_step1_get_profile(SCattiVelocitySecondOrderStep1 *s,
                            const SCattiProfile *input, SCattiBlock *block)
{
    SCattiProfile *p = &block->p_min;
    scatti_profile_set_boundary_from_profile(p, input);

    const double af = (s->vd > 0) ? s->_aMax : s->_aMin;
    p->t[0] = 0;
    p->t[1] = s->vd / af;
    p->t[2] = 0;
    p->t[3] = 0;
    p->t[4] = 0;
    p->t[5] = 0;
    p->t[6] = 0;

    if (scatti_profile_check_for_second_order_velocity(p, ControlSignsUDDU, ReachedLimitsACC0, af)) {
        block->t_min = p->t_sum[6] + p->brake.duration + p->accel.duration;
        return true;
    }
    return false;
}
