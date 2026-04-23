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

#ifndef SCATTI_INPUT_PARAMETER_H
#define SCATTI_INPUT_PARAMETER_H

#include <stdbool.h>
#include <stddef.h>
#include <scatti/result.h>

typedef struct {
    size_t degrees_of_freedom;

    SCattiControlInterface control_interface;
    SCattiSynchronization synchronization;
    SCattiDurationDiscretization duration_discretization;

    /* Current state */
    double *current_position;
    double *current_velocity;
    double *current_acceleration;

    /* Target state */
    double *target_position;
    double *target_velocity;
    double *target_acceleration;

    /* Kinematic constraints */
    double *max_velocity;
    double *max_acceleration;
    double *max_jerk;

    /* Optional min limits (NULL = use -max) */
    double *min_velocity;     /* NULL or array of dofs */
    double *min_acceleration; /* NULL or array of dofs */

    /* Per-DOF enable flags */
    bool *enabled;

    /* Optional per-DOF control interface / synchronization (NULL = use global) */
    SCattiControlInterface *per_dof_control_interface; /* NULL or array of dofs */
    SCattiSynchronization *per_dof_synchronization;    /* NULL or array of dofs */

    /* Optional minimum trajectory duration (-1 = not set) */
    double minimum_duration;
    bool has_minimum_duration;

    /* ---- Pro features ---- */

    /* Intermediate waypoints: flat array of num_waypoints * dofs doubles.
     * Each waypoint is dofs consecutive doubles. NULL if no waypoints. */
    double *intermediate_positions;
    size_t num_intermediate_waypoints;

    /* Per-section kinematic constraints: flat arrays of (num_waypoints+1) * dofs.
     * Section i constraints at offset i*dofs. NULL = use global. */
    double *per_section_max_velocity;
    double *per_section_max_acceleration;
    double *per_section_max_jerk;
    double *per_section_min_velocity;
    double *per_section_min_acceleration;

    /* Per-section position limits: flat arrays of (num_waypoints+1) * dofs. */
    double *per_section_max_position;
    double *per_section_min_position;

    /* Global position limits during trajectory (NULL = no limits) */
    double *max_position;     /* NULL or array of dofs */
    double *min_position;     /* NULL or array of dofs */

    /* Per-section minimum duration: array of (num_waypoints+1). NULL = no constraint. */
    double *per_section_minimum_duration;

    /* Calculation interruption budget in microseconds. 0 = no interruption. */
    double interrupt_calculation_duration;
} SCattiInputParameter;

SCattiInputParameter* scatti_input_create(size_t dofs);
void scatti_input_destroy(SCattiInputParameter *inp);
bool scatti_input_validate(const SCattiInputParameter *inp,
                            bool check_current_within_limits,
                            bool check_target_within_limits);
bool scatti_input_is_equal(const SCattiInputParameter *a, const SCattiInputParameter *b);
void scatti_input_copy(SCattiInputParameter *dst, const SCattiInputParameter *src);

/* Set intermediate waypoints. Copies the data. positions is num_waypoints * dofs doubles. */
void scatti_input_set_intermediate_positions(SCattiInputParameter *inp,
                                               const double *positions,
                                               size_t num_waypoints);

#endif /* SCATTI_INPUT_PARAMETER_H */
