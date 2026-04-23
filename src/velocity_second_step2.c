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
