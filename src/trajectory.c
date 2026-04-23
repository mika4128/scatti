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
#include <scatti/trajectory.h>
#include <scatti/utils.h>

#include <math.h>
#include <stdlib.h>
#include <string.h>

SCattiTrajectory* scatti_trajectory_create(size_t dofs) {
    SCattiTrajectory *traj = (SCattiTrajectory*)SCATTI_CALLOC(1, sizeof(SCattiTrajectory));
    if (!traj) return NULL;

    traj->degrees_of_freedom = dofs;
    traj->num_sections = 1;
    traj->section_capacity = 1;
    traj->duration = 0.0;

    traj->profiles = (SCattiProfile*)SCATTI_CALLOC(dofs, sizeof(SCattiProfile));
    traj->cumulative_times = (double*)SCATTI_CALLOC(1, sizeof(double));
    traj->independent_min_durations = (double*)SCATTI_CALLOC(dofs, sizeof(double));
    traj->position_extrema = (SCattiBound*)SCATTI_CALLOC(dofs, sizeof(SCattiBound));

    if (!traj->profiles || !traj->cumulative_times ||
        !traj->independent_min_durations || !traj->position_extrema) {
        scatti_trajectory_destroy(traj);
        return NULL;
    }

    for (size_t dof = 0; dof < dofs; ++dof) {
        scatti_profile_init(&traj->profiles[dof]);
    }

    return traj;
}

void scatti_trajectory_destroy(SCattiTrajectory *traj) {
    if (!traj) return;
    SCATTI_FREE(traj->profiles);
    SCATTI_FREE(traj->cumulative_times);
    SCATTI_FREE(traj->independent_min_durations);
    SCATTI_FREE(traj->position_extrema);
    SCATTI_FREE(traj);
}

bool scatti_trajectory_resize(SCattiTrajectory *traj, size_t num_sections) {
    if (!traj || num_sections == 0) return false;

    const size_t dofs = traj->degrees_of_freedom;

    if (num_sections > traj->section_capacity) {
        SCattiProfile *new_profiles = (SCattiProfile*)SCATTI_REALLOC(
            traj->profiles, num_sections * dofs * sizeof(SCattiProfile));
        double *new_times = (double*)SCATTI_REALLOC(
            traj->cumulative_times, num_sections * sizeof(double));

        if (!new_profiles || !new_times) {
            /* Restore on failure */
            if (new_profiles) traj->profiles = new_profiles;
            if (new_times) traj->cumulative_times = new_times;
            return false;
        }

        traj->profiles = new_profiles;
        traj->cumulative_times = new_times;
        traj->section_capacity = num_sections;

        /* Initialize new profiles */
        for (size_t s = traj->num_sections; s < num_sections; ++s) {
            for (size_t d = 0; d < dofs; ++d) {
                scatti_profile_init(&traj->profiles[s * dofs + d]);
            }
            traj->cumulative_times[s] = 0.0;
        }
    }

    traj->num_sections = num_sections;
    return true;
}

/*
 * state_to_integrate_from: Determine the integration base state at a given time.
 * Supports multi-section trajectories via binary search on cumulative_times.
 */
static void state_to_integrate_from(const SCattiTrajectory *traj, double time,
                                    size_t *new_section,
                                    double *t_out, double *p_out, double *v_out,
                                    double *a_out, double *j_out)
{
    const size_t dofs = traj->degrees_of_freedom;
    const size_t nsec = traj->num_sections;

    if (time >= traj->duration) {
        /* Past the end of trajectory */
        *new_section = nsec;
        size_t last = nsec - 1;
        for (size_t dof = 0; dof < dofs; ++dof) {
            const SCattiProfile *prof = &traj->profiles[last * dofs + dof];
            double t_pre = prof->brake.duration;
            double t_diff = time - (traj->duration - (t_pre + prof->t_sum[6]) + t_pre + prof->t_sum[6]);
            /* Simplify: time past the end of last section's profile */
            double section_start = (last > 0) ? traj->cumulative_times[last - 1] : 0.0;
            t_diff = time - section_start - t_pre - prof->t_sum[6];
            t_out[dof] = t_diff;
            p_out[dof] = prof->p[7];
            v_out[dof] = prof->v[7];
            a_out[dof] = prof->a[7];
            j_out[dof] = 0.0;
        }
        return;
    }

    /* Binary search to find current section */
    size_t section = 0;
    if (nsec > 1) {
        size_t lo = 0, hi = nsec;
        while (lo < hi) {
            size_t mid = lo + (hi - lo) / 2;
            if (traj->cumulative_times[mid] <= time) {
                lo = mid + 1;
            } else {
                hi = mid;
            }
        }
        section = lo;
        if (section >= nsec) section = nsec - 1;
    }

    *new_section = section;

    /* Time offset within this section */
    double section_start = (section > 0) ? traj->cumulative_times[section - 1] : 0.0;
    double t_diff = time - section_start;

    for (size_t dof = 0; dof < dofs; ++dof) {
        const SCattiProfile *prof = &traj->profiles[section * dofs + dof];
        double t_diff_dof = t_diff;

        /* Brake pre-trajectory (only in first section, or in each section for waypoints) */
        if (prof->brake.duration > 0.0) {
            if (t_diff_dof < prof->brake.duration) {
                size_t index = (t_diff_dof < prof->brake.t[0]) ? 0 : 1;
                if (index > 0) {
                    t_diff_dof -= prof->brake.t[index - 1];
                }
                t_out[dof] = t_diff_dof;
                p_out[dof] = prof->brake.p[index];
                v_out[dof] = prof->brake.v[index];
                a_out[dof] = prof->brake.a[index];
                j_out[dof] = prof->brake.j[index];
                continue;
            } else {
                t_diff_dof -= prof->brake.duration;
            }
        }

        /* Non-time synchronization: past the end of this DOF's profile */
        if (t_diff_dof >= prof->t_sum[6]) {
            t_out[dof] = t_diff_dof - prof->t_sum[6];
            p_out[dof] = prof->p[7];
            v_out[dof] = prof->v[7];
            a_out[dof] = prof->a[7];
            j_out[dof] = 0.0;
            continue;
        }

        /* Binary search in t_sum[0..6] */
        size_t index_dof = 0;
        {
            size_t lo = 0, hi = 7;
            while (lo < hi) {
                size_t mid = lo + (hi - lo) / 2;
                if (prof->t_sum[mid] <= t_diff_dof) {
                    lo = mid + 1;
                } else {
                    hi = mid;
                }
            }
            index_dof = lo;
        }

        if (index_dof > 0) {
            t_diff_dof -= prof->t_sum[index_dof - 1];
        }

        t_out[dof] = t_diff_dof;
        p_out[dof] = prof->p[index_dof];
        v_out[dof] = prof->v[index_dof];
        a_out[dof] = prof->a[index_dof];
        j_out[dof] = prof->j[index_dof];
    }
}

SCATTI_HOT
void scatti_trajectory_at_time(const SCattiTrajectory *traj, double time,
                                double * SCATTI_RESTRICT new_position,
                                double * SCATTI_RESTRICT new_velocity,
                                double * SCATTI_RESTRICT new_acceleration,
                                double * SCATTI_RESTRICT new_jerk,
                                size_t *new_section)
{
    const size_t dofs = traj->degrees_of_freedom;

    /* Temporary arrays on stack for small DOFs, heap otherwise */
    double t_buf[16], p_buf[16], v_buf[16], a_buf[16], j_buf[16];
    double *t_arr = t_buf, *p_arr = p_buf, *v_arr = v_buf, *a_arr = a_buf, *j_arr = j_buf;
    bool heap = false;

    if (dofs > 16) {
        t_arr = (double*)SCATTI_MALLOC(dofs * sizeof(double));
        p_arr = (double*)SCATTI_MALLOC(dofs * sizeof(double));
        v_arr = (double*)SCATTI_MALLOC(dofs * sizeof(double));
        a_arr = (double*)SCATTI_MALLOC(dofs * sizeof(double));
        j_arr = (double*)SCATTI_MALLOC(dofs * sizeof(double));
        heap = true;
    }

    state_to_integrate_from(traj, time, new_section, t_arr, p_arr, v_arr, a_arr, j_arr);

    for (size_t dof = 0; dof < dofs; ++dof) {
        double p_out, v_out, a_out;
        scatti_integrate(t_arr[dof], p_arr[dof], v_arr[dof], a_arr[dof], j_arr[dof],
                          &p_out, &v_out, &a_out);
        new_position[dof] = p_out;
        new_velocity[dof] = v_out;
        new_acceleration[dof] = a_out;
        if (new_jerk) {
            new_jerk[dof] = j_arr[dof];
        }
    }

    if (heap) {
        SCATTI_FREE(t_arr);
        SCATTI_FREE(p_arr);
        SCATTI_FREE(v_arr);
        SCATTI_FREE(a_arr);
        SCATTI_FREE(j_arr);
    }
}

void scatti_trajectory_at_time_simple(const SCattiTrajectory *traj, double time,
                                       double *new_position, double *new_velocity,
                                       double *new_acceleration)
{
    size_t new_section;
    scatti_trajectory_at_time(traj, time, new_position, new_velocity,
                               new_acceleration, NULL, &new_section);
}

double scatti_trajectory_get_duration(const SCattiTrajectory *traj) {
    return traj->duration;
}

size_t scatti_trajectory_get_intermediate_durations(const SCattiTrajectory *traj,
                                                     double *out_durations)
{
    for (size_t s = 0; s < traj->num_sections; ++s) {
        out_durations[s] = traj->cumulative_times[s];
    }
    return traj->num_sections;
}

void scatti_trajectory_get_position_extrema(SCattiTrajectory *traj) {
    const size_t dofs = traj->degrees_of_freedom;
    for (size_t dof = 0; dof < dofs; ++dof) {
        /* Initialize from first section */
        SCattiBound bound = scatti_profile_get_position_extrema(&traj->profiles[dof]);

        /* Merge across all sections */
        for (size_t s = 1; s < traj->num_sections; ++s) {
            double section_start = traj->cumulative_times[s - 1];
            SCattiBound sb = scatti_profile_get_position_extrema(
                &traj->profiles[s * dofs + dof]);
            if (sb.min < bound.min) {
                bound.min = sb.min;
                bound.t_min = sb.t_min + section_start;
            }
            if (sb.max > bound.max) {
                bound.max = sb.max;
                bound.t_max = sb.t_max + section_start;
            }
        }

        traj->position_extrema[dof] = bound;
    }
}

bool scatti_trajectory_get_first_time_at_position(const SCattiTrajectory *traj,
                                                   size_t dof, double position,
                                                   double *time, double time_after)
{
    if (dof >= traj->degrees_of_freedom) return false;

    const size_t dofs = traj->degrees_of_freedom;

    /* Search through all sections */
    for (size_t s = 0; s < traj->num_sections; ++s) {
        double section_start = (s > 0) ? traj->cumulative_times[s - 1] : 0.0;
        double adjusted_time_after = time_after - section_start;
        if (adjusted_time_after < 0.0) adjusted_time_after = 0.0;

        if (scatti_profile_get_first_state_at_position(
                &traj->profiles[s * dofs + dof], position, time, adjusted_time_after)) {
            *time += section_start;
            return true;
        }
    }

    return false;
}

void scatti_trajectory_get_independent_min_durations(const SCattiTrajectory *traj,
                                                      double *out_durations)
{
    for (size_t dof = 0; dof < traj->degrees_of_freedom; ++dof) {
        out_durations[dof] = traj->independent_min_durations[dof];
    }
}

const SCattiProfile* scatti_trajectory_get_profile(const SCattiTrajectory *traj, size_t dof)
{
    if (dof >= traj->degrees_of_freedom) return NULL;
    return &traj->profiles[dof];
}

const SCattiProfile* scatti_trajectory_get_section_profile(const SCattiTrajectory *traj,
                                                              size_t section, size_t dof)
{
    if (section >= traj->num_sections || dof >= traj->degrees_of_freedom) return NULL;
    return &traj->profiles[section * traj->degrees_of_freedom + dof];
}
