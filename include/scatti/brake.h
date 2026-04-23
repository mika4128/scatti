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
