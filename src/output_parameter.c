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

#include <scatti/scatti_config.h>
#include <scatti/output_parameter.h>

#include <stdlib.h>
#include <string.h>

SCattiOutputParameter* scatti_output_create(size_t dofs) {
    SCattiOutputParameter *out = (SCattiOutputParameter*)SCATTI_CALLOC(1, sizeof(SCattiOutputParameter));
    if (!out) return NULL;

    out->degrees_of_freedom = dofs;

    out->trajectory = scatti_trajectory_create(dofs);
    if (!out->trajectory) {
        SCATTI_FREE(out);
        return NULL;
    }

    out->new_position     = (double*)SCATTI_CALLOC(dofs, sizeof(double));
    out->new_velocity     = (double*)SCATTI_CALLOC(dofs, sizeof(double));
    out->new_acceleration = (double*)SCATTI_CALLOC(dofs, sizeof(double));
    out->new_jerk         = (double*)SCATTI_CALLOC(dofs, sizeof(double));

    if (!out->new_position || !out->new_velocity ||
        !out->new_acceleration || !out->new_jerk) {
        scatti_output_destroy(out);
        return NULL;
    }

    out->time = 0.0;
    out->new_section = 0;
    out->did_section_change = false;
    out->new_calculation = false;
    out->was_calculation_interrupted = false;
    out->calculation_duration = 0.0;

    return out;
}

void scatti_output_destroy(SCattiOutputParameter *out) {
    if (!out) return;
    scatti_trajectory_destroy(out->trajectory);
    SCATTI_FREE(out->new_position);
    SCATTI_FREE(out->new_velocity);
    SCATTI_FREE(out->new_acceleration);
    SCATTI_FREE(out->new_jerk);
    SCATTI_FREE(out);
}

void scatti_output_pass_to_input(const SCattiOutputParameter *out, SCattiInputParameter *inp) {
    if (!out || !inp) return;

    const size_t dofs = out->degrees_of_freedom;
    const size_t dsz = dofs * sizeof(double);

    memcpy(inp->current_position, out->new_position, dsz);
    memcpy(inp->current_velocity, out->new_velocity, dsz);
    memcpy(inp->current_acceleration, out->new_acceleration, dsz);

    /* If section changed and we have intermediate waypoints, remove the first waypoint */
    if (out->did_section_change && inp->num_intermediate_waypoints > 0) {
        size_t remaining = inp->num_intermediate_waypoints - 1;
        if (remaining == 0) {
            SCATTI_FREE(inp->intermediate_positions);
            inp->intermediate_positions = NULL;
            inp->num_intermediate_waypoints = 0;
        } else {
            /* Shift waypoints forward by one */
            memmove(inp->intermediate_positions,
                    inp->intermediate_positions + dofs,
                    remaining * dofs * sizeof(double));
            inp->num_intermediate_waypoints = remaining;
        }

        /* Also shift per-section constraints if present */
        size_t old_nsec = remaining + 2; /* was num_waypoints+1 sections */
        size_t new_nsec = remaining + 1;

#define SHIFT_PER_SEC(field) \
        if (inp->field) { \
            memmove(inp->field, inp->field + dofs, new_nsec * dofs * sizeof(double)); \
        }

        SHIFT_PER_SEC(per_section_max_velocity)
        SHIFT_PER_SEC(per_section_max_acceleration)
        SHIFT_PER_SEC(per_section_max_jerk)
        SHIFT_PER_SEC(per_section_min_velocity)
        SHIFT_PER_SEC(per_section_min_acceleration)
        SHIFT_PER_SEC(per_section_max_position)
        SHIFT_PER_SEC(per_section_min_position)
#undef SHIFT_PER_SEC

        if (inp->per_section_minimum_duration) {
            memmove(inp->per_section_minimum_duration,
                    inp->per_section_minimum_duration + 1,
                    new_nsec * sizeof(double));
        }
        (void)old_nsec;
    }
}
