#ifndef SCATTI_POSITION_H
#define SCATTI_POSITION_H

#include <stdbool.h>
#include <scatti/profile.h>
#include <scatti/block.h>

/* ---- Third Order Step 1 ---- */
typedef struct {
    double v0, a0, vf, af;
    double _vMax, _vMin, _aMax, _aMin, _jMax;
    double pd;
    double v0_v0, vf_vf;
    double a0_a0, a0_p3, a0_p4;
    double af_af, af_p3, af_p4;
    double jMax_jMax;
    SCattiProfile valid_profiles[6];
} SCattiPositionThirdOrderStep1;

void scatti_pos3_step1_init(SCattiPositionThirdOrderStep1 *s,
                     double p0, double v0, double a0,
                     double pf, double vf, double af,
                     double vMax, double vMin, double aMax, double aMin, double jMax);
bool scatti_pos3_step1_get_profile(SCattiPositionThirdOrderStep1 *s,
                            const SCattiProfile *input, SCattiBlock *block);

/* ---- Third Order Step 2 ---- */
typedef struct {
    double v0, a0, tf, vf, af;
    double _vMax, _vMin, _aMax, _aMin, _jMax;
    double pd;
    double tf_tf, tf_p3, tf_p4;
    double vd, vd_vd;
    double ad, ad_ad;
    double v0_v0, vf_vf;
    double a0_a0, a0_p3, a0_p4, a0_p5, a0_p6;
    double af_af, af_p3, af_p4, af_p5, af_p6;
    double jMax_jMax;
    double g1, g2;
} SCattiPositionThirdOrderStep2;

void scatti_pos3_step2_init(SCattiPositionThirdOrderStep2 *s,
                     double tf, double p0, double v0, double a0,
                     double pf, double vf, double af,
                     double vMax, double vMin, double aMax, double aMin, double jMax);
bool scatti_pos3_step2_get_profile(SCattiPositionThirdOrderStep2 *s, SCattiProfile *profile);

/* ---- Second Order Step 1 ---- */
typedef struct {
    double v0, vf;
    double _vMax, _vMin, _aMax, _aMin;
    double pd;
    SCattiProfile valid_profiles[4];
} SCattiPositionSecondOrderStep1;

void scatti_pos2_step1_init(SCattiPositionSecondOrderStep1 *s,
                     double p0, double v0, double pf, double vf,
                     double vMax, double vMin, double aMax, double aMin);
bool scatti_pos2_step1_get_profile(SCattiPositionSecondOrderStep1 *s,
                            const SCattiProfile *input, SCattiBlock *block);

/* ---- Second Order Step 2 ---- */
typedef struct {
    double v0, tf, vf;
    double _vMax, _vMin, _aMax, _aMin;
    double pd, vd;
} SCattiPositionSecondOrderStep2;

void scatti_pos2_step2_init(SCattiPositionSecondOrderStep2 *s,
                     double tf, double p0, double v0, double pf, double vf,
                     double vMax, double vMin, double aMax, double aMin);
bool scatti_pos2_step2_get_profile(SCattiPositionSecondOrderStep2 *s, SCattiProfile *profile);

/* ---- First Order Step 1 ---- */
typedef struct {
    double _vMax, _vMin;
    double pd;
} SCattiPositionFirstOrderStep1;

void scatti_pos1_step1_init(SCattiPositionFirstOrderStep1 *s,
                     double p0, double pf, double vMax, double vMin);
bool scatti_pos1_step1_get_profile(SCattiPositionFirstOrderStep1 *s,
                            const SCattiProfile *input, SCattiBlock *block);

/* ---- First Order Step 2 ---- */
typedef struct {
    double tf;
    double _vMax, _vMin;
    double pd;
} SCattiPositionFirstOrderStep2;

void scatti_pos1_step2_init(SCattiPositionFirstOrderStep2 *s,
                     double tf, double p0, double pf, double vMax, double vMin);
bool scatti_pos1_step2_get_profile(SCattiPositionFirstOrderStep2 *s, SCattiProfile *profile);

#endif /* SCATTI_POSITION_H */
