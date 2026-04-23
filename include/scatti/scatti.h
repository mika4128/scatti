#ifndef SCATTI_H
#define SCATTI_H

#include <stdbool.h>
#include <stddef.h>

#include <scatti/result.h>
#include <scatti/input_parameter.h>
#include <scatti/output_parameter.h>
#include <scatti/trajectory.h>
#include <scatti/calculator.h>

/* Main scatti instance */
typedef struct {
    size_t degrees_of_freedom;
    double delta_time;
    size_t max_number_of_waypoints;

    CRuckigCalculator *calculator;
    CRuckigInputParameter *current_input;
    bool current_input_initialized;
} CRuckig;

/* Create and destroy (backward compatible: 0 waypoints) */
CRuckig* scatti_create(size_t dofs, double delta_time);

/* Create with waypoint support */
CRuckig* scatti_create_waypoints(size_t dofs, double delta_time, size_t max_waypoints);

void scatti_destroy(CRuckig *r);

/* Reset (force recalculation on next update) */
void scatti_reset(CRuckig *r);

/* Calculate trajectory (offline, auto-dispatches to waypoint calculator if needed) */
CRuckigResult scatti_calculate(CRuckig *r, const CRuckigInputParameter *input,
                                CRuckigTrajectory *trajectory);

/* Update (online, call every delta_time) */
CRuckigResult scatti_update(CRuckig *r, const CRuckigInputParameter *input,
                             CRuckigOutputParameter *output);

/* Validate input */
bool scatti_validate_input(const CRuckig *r, const CRuckigInputParameter *input,
                            bool check_current_within_limits,
                            bool check_target_within_limits);

#endif /* SCATTI_H */
