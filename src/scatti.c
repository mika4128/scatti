#define _POSIX_C_SOURCE 199309L

#include <scatti/scatti_config.h>
#include <scatti/scatti.h>

#include <stdlib.h>
#include <string.h>
#include <time.h>

static SCatti* scatti_create_internal(size_t dofs, double delta_time, size_t max_waypoints) {
    SCatti *r = (SCatti*)SCATTI_CALLOC(1, sizeof(SCatti));
    if (!r) return NULL;

    r->degrees_of_freedom = dofs;
    r->delta_time = delta_time;
    r->max_number_of_waypoints = max_waypoints;

    r->calculator = scatti_calculator_create(dofs);
    if (!r->calculator) {
        SCATTI_FREE(r);
        return NULL;
    }

    r->current_input = scatti_input_create(dofs);
    if (!r->current_input) {
        scatti_calculator_destroy(r->calculator);
        SCATTI_FREE(r);
        return NULL;
    }

    r->current_input_initialized = false;

    return r;
}

SCatti* scatti_create(size_t dofs, double delta_time) {
    return scatti_create_internal(dofs, delta_time, 0);
}

SCatti* scatti_create_waypoints(size_t dofs, double delta_time, size_t max_waypoints) {
    return scatti_create_internal(dofs, delta_time, max_waypoints);
}

void scatti_destroy(SCatti *r) {
    if (!r) return;
    scatti_calculator_destroy(r->calculator);
    scatti_input_destroy(r->current_input);
    SCATTI_FREE(r);
}

void scatti_reset(SCatti *r) {
    if (!r) return;
    r->current_input_initialized = false;
}

static inline bool use_waypoints(const SCattiInputParameter *input) {
    return input->num_intermediate_waypoints > 0 &&
           input->control_interface == SCattiPosition;
}

bool scatti_validate_input(const SCatti *r, const SCattiInputParameter *input,
                            bool check_current_within_limits,
                            bool check_target_within_limits)
{
    if (!r || !input) return false;

    if (!scatti_input_validate(input, check_current_within_limits, check_target_within_limits)) {
        return false;
    }

    if (r->delta_time <= 0.0 && input->duration_discretization != SCattiContinuous) {
        return false;
    }

    /* Validate waypoint count against max */
    if (input->num_intermediate_waypoints > r->max_number_of_waypoints &&
        r->max_number_of_waypoints > 0) {
        return false;
    }

    return true;
}

static SCattiResult dispatch_calculate(SCatti *r, const SCattiInputParameter *input,
                                        SCattiTrajectory *trajectory, bool *was_interrupted)
{
    if (use_waypoints(input)) {
        /* Ensure trajectory has enough capacity */
        size_t nsec = input->num_intermediate_waypoints + 1;
        if (!scatti_trajectory_resize(trajectory, nsec)) {
            return SCattiError;
        }
        return scatti_calculator_calculate_waypoints(r->calculator, input, trajectory,
                                                       r->delta_time, was_interrupted);
    } else {
        /* Single-segment: ensure single section */
        if (trajectory->num_sections != 1) {
            scatti_trajectory_resize(trajectory, 1);
        }
        return scatti_calculator_calculate(r->calculator, input, trajectory,
                                            r->delta_time, was_interrupted);
    }
}

SCattiResult scatti_calculate(SCatti *r, const SCattiInputParameter *input,
                                SCattiTrajectory *trajectory)
{
    if (!r || !input || !trajectory) return SCattiError;

    if (!scatti_validate_input(r, input, false, true)) {
        return SCattiErrorInvalidInput;
    }

    bool was_interrupted = false;
    return dispatch_calculate(r, input, trajectory, &was_interrupted);
}

static double get_time_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1e6 + (double)ts.tv_nsec / 1e3;
}

SCATTI_HOT
SCattiResult scatti_update(SCatti *r, const SCattiInputParameter *input,
                             SCattiOutputParameter *output)
{
    if (SCATTI_UNLIKELY(!r || !input || !output)) return SCattiError;

    double start_us = get_time_us();

    output->new_calculation = false;

    SCattiResult result = SCattiWorking;
    if (!r->current_input_initialized || !scatti_input_is_equal(input, r->current_input)) {
        if (!scatti_validate_input(r, input, false, true)) {
            return SCattiErrorInvalidInput;
        }

        result = dispatch_calculate(r, input, output->trajectory,
                                    &output->was_calculation_interrupted);
        if (result != SCattiWorking && result != SCattiErrorPositionalLimits) {
            return result;
        }

        scatti_input_copy(r->current_input, input);
        r->current_input_initialized = true;
        output->time = 0.0;
        output->new_section = 0;
        output->new_calculation = true;
    }

    size_t old_section = output->new_section;
    output->time += r->delta_time;
    scatti_trajectory_at_time(output->trajectory, output->time,
                               output->new_position, output->new_velocity,
                               output->new_acceleration, output->new_jerk,
                               &output->new_section);
    output->did_section_change = (output->new_section > old_section);

    double stop_us = get_time_us();
    output->calculation_duration = stop_us - start_us;

    scatti_output_pass_to_input(output, r->current_input);

    if (output->time > scatti_trajectory_get_duration(output->trajectory)) {
        return SCattiFinished;
    }

    return result;
}
