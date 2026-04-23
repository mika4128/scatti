/********************************************************************
* Permission is hereby granted, free of charge, to any person obtaining
* a copy of this software and associated documentation files (the
* "Software"), to deal in the Software without restriction, including
* without limitation the rights to use, copy, modify, merge, publish,
* distribute, sublicense, and/or sell copies of the Software, and to
* permit persons to whom the Software is furnished to do so, subject
* to the following conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*
* Description: scatti - pure C99 jerk-limited real-time trajectory
*              generation. Port of Ruckig with embed-friendly
*              memory and math hooks.
* Author:      杨阳 (Yang Yang) <mika-net@outlook.com>
* Origin:      Based on Ruckig Community Edition by Lars Berscheid
*              (https://github.com/pantor/ruckig)
* License:     MIT (SPDX-License-Identifier: MIT)
* Copyright (c) 2026 杨阳 (Yang Yang)
********************************************************************/

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
