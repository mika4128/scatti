#ifndef SCATTI_PROFILE_H
#define SCATTI_PROFILE_H

#include <stdbool.h>
#include <stddef.h>
#include <scatti/brake.h>

/* Constants */
#define PROFILE_V_EPS    1e-12
#define PROFILE_A_EPS    1e-12
#define PROFILE_J_EPS    1e-12
#define PROFILE_P_PREC   1e-8
#define PROFILE_V_PREC   1e-8
#define PROFILE_A_PREC   1e-10
#define PROFILE_T_PREC   1e-12
#define PROFILE_T_MAX    1e12

typedef enum {
    ReachedLimitsACC0_ACC1_VEL = 0,
    ReachedLimitsVEL,
    ReachedLimitsACC0,
    ReachedLimitsACC1,
    ReachedLimitsACC0_ACC1,
    ReachedLimitsACC0_VEL,
    ReachedLimitsACC1_VEL,
    ReachedLimitsNONE
} SCattiReachedLimits;

typedef enum {
    DirectionUP = 0,
    DirectionDOWN
} SCattiDirection;

typedef enum {
    ControlSignsUDDU = 0,
    ControlSignsUDUD
} SCattiControlSigns;

/* Position extrema info */
typedef struct {
    double min, max;
    double t_min, t_max;
} SCattiBound;

/* Single-DOF kinematic profile */
typedef struct {
    double t[7];
    double t_sum[7];
    double j[7];
    double a[8];
    double v[8];
    double p[8];

    SCattiBrakeProfile brake;
    SCattiBrakeProfile accel;

    double pf, vf, af;

    SCattiReachedLimits limits;
    SCattiDirection direction;
    SCattiControlSigns control_signs;
} SCattiProfile;

void scatti_profile_init(SCattiProfile *p);

/* Set boundary conditions */
void scatti_profile_set_boundary(SCattiProfile *p, double p0, double v0, double a0,
                          double pf, double vf, double af);
void scatti_profile_set_boundary_from_profile(SCattiProfile *p, const SCattiProfile *src);
void scatti_profile_set_boundary_for_velocity(SCattiProfile *p, double p0, double v0, double a0,
                                       double vf, double af);

/* Third-order position check */
bool scatti_profile_check(SCattiProfile *p, SCattiControlSigns cs, SCattiReachedLimits lim,
                   bool set_limits, double jf, double vMax, double vMin, double aMax, double aMin);
bool scatti_profile_check_with_timing(SCattiProfile *p, SCattiControlSigns cs, SCattiReachedLimits lim,
                               double tf, double jf, double vMax, double vMin, double aMax, double aMin);
bool scatti_profile_check_with_timing_full(SCattiProfile *p, SCattiControlSigns cs, SCattiReachedLimits lim,
                                    double tf, double jf, double vMax, double vMin, double aMax, double aMin, double jMax);

/* Third-order velocity check */
bool scatti_profile_check_for_velocity(SCattiProfile *p, SCattiControlSigns cs, SCattiReachedLimits lim,
                                double jf, double aMax, double aMin);
bool scatti_profile_check_for_velocity_with_timing(SCattiProfile *p, SCattiControlSigns cs, SCattiReachedLimits lim,
                                            double tf, double jf, double aMax, double aMin);
bool scatti_profile_check_for_velocity_with_timing_full(SCattiProfile *p, SCattiControlSigns cs, SCattiReachedLimits lim,
                                                 double tf, double jf, double aMax, double aMin, double jMax);

/* Second-order position check */
bool scatti_profile_check_for_second_order(SCattiProfile *p, SCattiControlSigns cs, SCattiReachedLimits lim,
                                    double aUp, double aDown, double vMax, double vMin);
bool scatti_profile_check_for_second_order_with_timing(SCattiProfile *p, SCattiControlSigns cs, SCattiReachedLimits lim,
                                                double tf, double aUp, double aDown, double vMax, double vMin);
bool scatti_profile_check_for_second_order_with_timing_full(SCattiProfile *p, SCattiControlSigns cs, SCattiReachedLimits lim,
                                                     double tf, double aUp, double aDown, double vMax, double vMin,
                                                     double aMax, double aMin);

/* Second-order velocity check */
bool scatti_profile_check_for_second_order_velocity(SCattiProfile *p, SCattiControlSigns cs, SCattiReachedLimits lim,
                                             double aUp);
bool scatti_profile_check_for_second_order_velocity_with_timing(SCattiProfile *p, SCattiControlSigns cs, SCattiReachedLimits lim,
                                                         double tf, double aUp);
bool scatti_profile_check_for_second_order_velocity_with_timing_full(SCattiProfile *p, SCattiControlSigns cs, SCattiReachedLimits lim,
                                                              double tf, double aUp, double aMax, double aMin);

/* First-order position check */
bool scatti_profile_check_for_first_order(SCattiProfile *p, SCattiControlSigns cs, SCattiReachedLimits lim,
                                   double vUp);
bool scatti_profile_check_for_first_order_with_timing(SCattiProfile *p, SCattiControlSigns cs, SCattiReachedLimits lim,
                                               double tf, double vUp);
bool scatti_profile_check_for_first_order_with_timing_full(SCattiProfile *p, SCattiControlSigns cs, SCattiReachedLimits lim,
                                                    double tf, double vUp, double vMax, double vMin);

/* Position extrema */
SCattiBound scatti_profile_get_position_extrema(const SCattiProfile *p);

/* First time at position */
bool scatti_profile_get_first_state_at_position(const SCattiProfile *p, double pt, double *time, double time_after);

#endif /* SCATTI_PROFILE_H */
