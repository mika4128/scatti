#ifndef SCATTI_VELOCITY_H
#define SCATTI_VELOCITY_H

#include <stdbool.h>
#include <scatti/profile.h>
#include <scatti/block.h>

/* ---- Third Order Step 1 ---- */
typedef struct {
    double a0, af;
    double _aMax, _aMin, _jMax;
    double vd;
    SCattiProfile valid_profiles[3];
} SCattiVelocityThirdOrderStep1;

void scatti_vel3_step1_init(SCattiVelocityThirdOrderStep1 *s,
                     double v0, double a0, double vf, double af,
                     double aMax, double aMin, double jMax);
bool scatti_vel3_step1_get_profile(SCattiVelocityThirdOrderStep1 *s,
                            const SCattiProfile *input, SCattiBlock *block);

/* ---- Third Order Step 2 ---- */
typedef struct {
    double a0, tf, af;
    double _aMax, _aMin, _jMax;
    double vd, ad;
} SCattiVelocityThirdOrderStep2;

void scatti_vel3_step2_init(SCattiVelocityThirdOrderStep2 *s,
                     double tf, double v0, double a0, double vf, double af,
                     double aMax, double aMin, double jMax);
bool scatti_vel3_step2_get_profile(SCattiVelocityThirdOrderStep2 *s, SCattiProfile *profile);

/* ---- Second Order Step 1 ---- */
typedef struct {
    double _aMax, _aMin;
    double vd;
} SCattiVelocitySecondOrderStep1;

void scatti_vel2_step1_init(SCattiVelocitySecondOrderStep1 *s,
                     double v0, double vf, double aMax, double aMin);
bool scatti_vel2_step1_get_profile(SCattiVelocitySecondOrderStep1 *s,
                            const SCattiProfile *input, SCattiBlock *block);

/* ---- Second Order Step 2 ---- */
typedef struct {
    double tf;
    double _aMax, _aMin;
    double vd;
} SCattiVelocitySecondOrderStep2;

void scatti_vel2_step2_init(SCattiVelocitySecondOrderStep2 *s,
                     double tf, double v0, double vf, double aMax, double aMin);
bool scatti_vel2_step2_get_profile(SCattiVelocitySecondOrderStep2 *s, SCattiProfile *profile);

#endif /* SCATTI_VELOCITY_H */
