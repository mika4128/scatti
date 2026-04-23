#ifndef SCATTI_CALCULATOR_H
#define SCATTI_CALCULATOR_H

#include <stdbool.h>
#include <stddef.h>
#include <scatti/result.h>
#include <scatti/block.h>
#include <scatti/input_parameter.h>
#include <scatti/trajectory.h>

typedef struct {
    size_t degrees_of_freedom;

    double *new_phase_control;
    double *pd;
    double *possible_t_syncs;
    size_t *idx;

    SCattiBlock *blocks;
    double *inp_min_velocity;
    double *inp_min_acceleration;
    SCattiControlInterface *inp_per_dof_control_interface;
    SCattiSynchronization *inp_per_dof_synchronization;

    /* Scratch space for waypoint calculation */
    SCattiInputParameter *segment_input;  /* Reusable per-segment input */
} SCattiCalculator;

SCattiCalculator* scatti_calculator_create(size_t dofs);
void scatti_calculator_destroy(SCattiCalculator *calc);

/* Single-segment calculation (existing, backward compatible) */
SCattiResult scatti_calculator_calculate(SCattiCalculator *calc,
                                           const SCattiInputParameter *inp,
                                           SCattiTrajectory *traj,
                                           double delta_time,
                                           bool *was_interrupted);

/* Multi-segment waypoint calculation */
SCattiResult scatti_calculator_calculate_waypoints(SCattiCalculator *calc,
                                                     const SCattiInputParameter *inp,
                                                     SCattiTrajectory *traj,
                                                     double delta_time,
                                                     bool *was_interrupted);

/* Continue an interrupted calculation */
SCattiResult scatti_calculator_continue(SCattiCalculator *calc,
                                          const SCattiInputParameter *inp,
                                          SCattiTrajectory *traj,
                                          double delta_time,
                                          bool *was_interrupted);

#endif /* SCATTI_CALCULATOR_H */
