#ifndef SCATTI_UTILS_H
#define SCATTI_UTILS_H

#include <math.h>
#include <scatti/scatti_config.h>

SCATTI_FORCE_INLINE void scatti_integrate(double t, double p0, double v0, double a0, double j,
                                            double * SCATTI_RESTRICT p_out,
                                            double * SCATTI_RESTRICT v_out,
                                            double * SCATTI_RESTRICT a_out) {
    *p_out = p0 + t * (v0 + t * (a0 / 2.0 + t * j / 6.0));
    *v_out = v0 + t * (a0 + t * j / 2.0);
    *a_out = a0 + t * j;
}

SCATTI_FORCE_INLINE double scatti_pow2(double v) {
    return v * v;
}

#endif /* SCATTI_UTILS_H */
