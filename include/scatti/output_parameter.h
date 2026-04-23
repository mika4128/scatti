#ifndef SCATTI_OUTPUT_PARAMETER_H
#define SCATTI_OUTPUT_PARAMETER_H

#include <stdbool.h>
#include <stddef.h>
#include <scatti/trajectory.h>
#include <scatti/input_parameter.h>

typedef struct {
    size_t degrees_of_freedom;

    SCattiTrajectory *trajectory;

    double *new_position;
    double *new_velocity;
    double *new_acceleration;
    double *new_jerk;

    double time;
    size_t new_section;
    bool did_section_change;
    bool new_calculation;
    bool was_calculation_interrupted;
    double calculation_duration; /* microseconds */
} SCattiOutputParameter;

SCattiOutputParameter* scatti_output_create(size_t dofs);
void scatti_output_destroy(SCattiOutputParameter *out);
void scatti_output_pass_to_input(const SCattiOutputParameter *out, SCattiInputParameter *inp);

#endif /* SCATTI_OUTPUT_PARAMETER_H */
