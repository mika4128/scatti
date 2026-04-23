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

    SCattiCalculator *calculator;
    SCattiInputParameter *current_input;
    bool current_input_initialized;
} SCatti;

/* Create and destroy (backward compatible: 0 waypoints) */
SCatti* scatti_create(size_t dofs, double delta_time);

/* Create with waypoint support */
SCatti* scatti_create_waypoints(size_t dofs, double delta_time, size_t max_waypoints);

void scatti_destroy(SCatti *r);

/* Reset (force recalculation on next update) */
void scatti_reset(SCatti *r);

/* Calculate trajectory (offline, auto-dispatches to waypoint calculator if needed) */
SCattiResult scatti_calculate(SCatti *r, const SCattiInputParameter *input,
                                SCattiTrajectory *trajectory);

/* Update (online, call every delta_time) */
SCattiResult scatti_update(SCatti *r, const SCattiInputParameter *input,
                             SCattiOutputParameter *output);

/* Validate input */
bool scatti_validate_input(const SCatti *r, const SCattiInputParameter *input,
                            bool check_current_within_limits,
                            bool check_target_within_limits);

#endif /* SCATTI_H */
