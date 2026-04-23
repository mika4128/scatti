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
