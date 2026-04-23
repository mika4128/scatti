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

#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include <scatti/scatti.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

static size_t n_assertions = 0;
static size_t n_assertion_failures = 0;

/* Simple xorshift64 PRNG */
static uint64_t rng_state = 42;

static double rand_uniform(double lo, double hi) {
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 7;
    rng_state ^= rng_state << 17;
    double u = (double)(rng_state & 0xFFFFFFFFULL) / 4294967296.0;
    return lo + u * (hi - lo);
}

/* Box-Muller for normal distribution */
static double rand_normal(double mean, double stddev) {
    double u1 = rand_uniform(1e-10, 1.0);
    double u2 = rand_uniform(0.0, 1.0);
    double z = sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
    return mean + stddev * z;
}

static int rand_int(int lo, int hi) {
    return lo + (int)(rand_uniform(0, 1) * (hi - lo + 1));
}

static void fill_position(double *arr, size_t n) {
    for (size_t i = 0; i < n; i++) arr[i] = rand_normal(0.0, 4.0);
}

static void fill_dynamic_or_zero(double *arr, size_t n, double prob) {
    for (size_t i = 0; i < n; i++) {
        arr[i] = (rand_uniform(0, 1) < prob) ? rand_normal(0.0, 0.8) : 0.0;
    }
}

static void fill_limit(double *arr, size_t n) {
    for (size_t i = 0; i < n; i++) arr[i] = rand_uniform(0.08, 16.0);
}

static void fill_limit_high(double *arr, size_t n) {
    for (size_t i = 0; i < n; i++) arr[i] = rand_uniform(10.0, 1000000.0);
}

static void fill_limit_with_offset(double *arr, size_t n, const double *offset) {
    for (size_t i = 0; i < n; i++) {
        arr[i] = fabs(offset[i]) + rand_uniform(0.08, 16.0);
    }
}

static void fill_limit_high_with_offset(double *arr, size_t n, const double *offset) {
    for (size_t i = 0; i < n; i++) {
        arr[i] = fabs(offset[i]) + rand_uniform(10.0, 1000000.0);
    }
}

static void fill_discrete(double *arr, size_t n) {
    for (size_t i = 0; i < n; i++) arr[i] = (double)rand_int(-1, 1);
}

static void fill_discrete_limit(double *arr, size_t n) {
    for (size_t i = 0; i < n; i++) arr[i] = (double)rand_int(1, 2);
}

static void fill_discrete_limit_with_offset(double *arr, size_t n, const double *offset) {
    for (size_t i = 0; i < n; i++) {
        arr[i] = fabs(offset[i]) + (double)rand_int(1, 2);
    }
}

static void fill_min_limit(double *arr, size_t n) {
    for (size_t i = 0; i < n; i++) arr[i] = rand_uniform(-16.0, -0.08);
}

static void fill_min_limit_with_offset(double *arr, size_t n, const double *offset) {
    for (size_t i = 0; i < n; i++) {
        double val = rand_uniform(-16.0, -0.08);
        /* Ensure min is <= -|target| */
        if (val > -fabs(offset[i])) val = -fabs(offset[i]) - 0.08;
        arr[i] = val;
    }
}

static size_t check_calculation(SCatti *otg, SCattiInputParameter *inp,
                                SCattiOutputParameter *out) {
    SCattiResult result = scatti_update(otg, inp, out);
    if (result == SCattiErrorTrajectoryDuration) return 0;

    n_assertions++;
    if (result != SCattiWorking &&
        !(result == SCattiFinished && scatti_trajectory_get_duration(out->trajectory) < 0.005)) {
        n_assertion_failures++;
        return 1; /* error */
    }

    n_assertions++;
    if (scatti_trajectory_get_duration(out->trajectory) < -1e-10) {
        n_assertion_failures++;
        return 1;
    }

    for (size_t dof = 0; dof < otg->degrees_of_freedom; dof++) {
        n_assertions++;
        if (isnan(out->new_position[dof]) || isnan(out->new_velocity[dof]) || isnan(out->new_acceleration[dof])) {
            n_assertion_failures++;
            return 1;
        }
    }

    double duration = scatti_trajectory_get_duration(out->trajectory);
    size_t ndofs = otg->degrees_of_freedom;
    double pos[8], vel[8], acc[8];

    /* Initial state verification: sample at t=0 */
    scatti_trajectory_at_time_simple(out->trajectory, 0.0, pos, vel, acc);
    for (size_t dof = 0; dof < ndofs; dof++) {
        /* Position check only for Position control mode */
        if (inp->control_interface == SCattiPosition) {
            n_assertions++;
            if (fabs(pos[dof] - inp->current_position[dof]) > 1e-6) {
                n_assertion_failures++;
            }
        }
        n_assertions++;
        if (fabs(vel[dof] - inp->current_velocity[dof]) > 1e-6) {
            n_assertion_failures++;
        }
        /* Skip acceleration check for second-order (max_jerk=inf) profiles:
           second-order profiles immediately apply the chosen acceleration,
           so a(t=0) is the profile acceleration, not current_acceleration. */
        if (!isinf(inp->max_jerk[dof])) {
            n_assertions++;
            if (fabs(acc[dof] - inp->current_acceleration[dof]) > 1e-6) {
                n_assertion_failures++;
            }
        }
    }

    /* Final state verification: sample at t=duration */
    if (duration > 0.01) {
        scatti_trajectory_at_time_simple(out->trajectory, duration, pos, vel, acc);
        for (size_t dof = 0; dof < ndofs; dof++) {
            if (inp->control_interface == SCattiPosition) {
                n_assertions++;
                if (fabs(pos[dof] - inp->target_position[dof]) > 1e-6) {
                    n_assertion_failures++;
                }
            }
            n_assertions++;
            if (fabs(vel[dof] - inp->target_velocity[dof]) > 1e-6) {
                n_assertion_failures++;
            }
            n_assertions++;
            if (fabs(acc[dof] - inp->target_acceleration[dof]) > 1e-4) {
                n_assertion_failures++;
            }
        }
    }

    return 0;
}

/* Step-through: run update loop, recalculating with a second otg at each step */
static size_t step_through_check(SCatti *otg, SCattiInputParameter *inp,
                                 SCattiOutputParameter *out, size_t max_checks) {
    SCatti *otg2 = scatti_create(otg->degrees_of_freedom, 0.001);
    SCattiInputParameter *inp2 = scatti_input_create(otg->degrees_of_freedom);
    SCattiOutputParameter *out2 = scatti_output_create(otg->degrees_of_freedom);

    /* First check */
    size_t errs = check_calculation(otg, inp, out);
    size_t checks = 1;

    SCattiResult result;
    while ((result = scatti_update(otg, inp, out)) == SCattiWorking) {
        scatti_output_pass_to_input(out, inp);

        /* Re-check from current state with different timestep */
        scatti_input_copy(inp2, inp);
        scatti_reset(otg2);
        errs += check_calculation(otg2, inp2, out2);
        checks++;

        if (checks >= max_checks) break;
    }

    scatti_output_destroy(out2);
    scatti_input_destroy(inp2);
    scatti_destroy(otg2);
    return checks;
}

int main(int argc, char **argv) {
    size_t num_trajectories = 256 * 1024;
    if (argc > 1) {
        num_trajectories = (size_t)atoi(argv[1]);
    }

    /* Compute per-category counts matching C++ distribution */
    size_t position_random_1 = num_trajectories / 10;
    size_t position_second_random_3 = num_trajectories / 25;
    if (position_second_random_3 > 250000) position_second_random_3 = 250000;
    size_t step_through_3_count = 0; /* Same as C++ default */
    size_t random_direction_3 = num_trajectories / 50;
    if (random_direction_3 > 250000) random_direction_3 = 250000;
    size_t random_discrete_3 = num_trajectories / 10;
    if (random_discrete_3 > 250000) random_discrete_3 = 250000;
    size_t velocity_random_3 = num_trajectories / 10;
    size_t velocity_random_discrete_3 = num_trajectories / 10;
    if (velocity_random_discrete_3 > 250000) velocity_random_discrete_3 = 250000;
    size_t velocity_second_random_3 = num_trajectories / 25;
    if (velocity_second_random_3 > 250000) velocity_second_random_3 = 250000;

    size_t fixed_total = position_random_1 + step_through_3_count + random_direction_3 +
                         velocity_random_3 + random_discrete_3 + position_second_random_3 +
                         velocity_second_random_3 + velocity_random_discrete_3;
    size_t remainder = (num_trajectories > fixed_total) ? (num_trajectories - fixed_total) : 0;
    size_t position_random_3 = (size_t)(remainder * 95 / 100);
    size_t random_3_high = (size_t)(remainder * 5 / 100);

    printf("=== scatti Random Test Suite ===\n");
    printf("<number_trajectories>\n");
    printf("\tPosition (1 DoF): %zu\n", position_random_1);
    printf("\tPosition (3 DoF): %zu\n", position_random_3);
    printf("\tPosition Discrete (3 DoF): %zu\n", random_discrete_3);
    printf("\tPosition High Limits (3 DoF): %zu\n", random_3_high);
    printf("\tPosition Second Order (3 DoF): %zu\n", position_second_random_3);
    printf("\tPosition Step Through (3 DoF): %zu\n", step_through_3_count);
    printf("\tVelocity (3 DoF): %zu\n", velocity_random_3);
    printf("\tVelocity Discrete (3 DoF): %zu\n", velocity_random_discrete_3);
    printf("\tVelocity Second Order (3 DoF): %zu\n", velocity_second_random_3);
    printf("\tRandom Direction (3 DoF): %zu\n", random_direction_3);
    printf("Total: %zu\n\n", num_trajectories);

    size_t errors = 0;
    size_t tested = 0;
    const size_t dofs = 3;

    SCatti *otg = scatti_create(dofs, 0.005);
    SCattiInputParameter *inp = scatti_input_create(dofs);
    SCattiOutputParameter *out = scatti_output_create(dofs);

    struct timespec start, end_ts;
    clock_gettime(CLOCK_MONOTONIC, &start);

    /* ==================== Position random 3-DOF ==================== */
    printf("Position random 3-DOF (%zu)...\n", position_random_3);
    rng_state = 42;
    for (size_t i = 0; i < position_random_3; i++) {
        scatti_reset(otg);

        /* Alternate Phase/Time sync (matching C++) */
        if (i < position_random_3 / 2) {
            inp->synchronization = SCattiSyncPhase;
        } else {
            inp->synchronization = SCattiSyncTime;
        }

        /* First 5% use Discrete discretization */
        if (i < position_random_3 / 20) {
            inp->duration_discretization = SCattiDiscrete;
        } else {
            inp->duration_discretization = SCattiContinuous;
        }

        fill_position(inp->current_position, dofs);
        fill_dynamic_or_zero(inp->current_velocity, dofs, 0.9);
        fill_dynamic_or_zero(inp->current_acceleration, dofs, 0.8);
        fill_position(inp->target_position, dofs);
        fill_dynamic_or_zero(inp->target_velocity, dofs, 0.7);
        fill_dynamic_or_zero(inp->target_acceleration, dofs, 0.6);
        fill_limit_with_offset(inp->max_velocity, dofs, inp->target_velocity);
        fill_limit_with_offset(inp->max_acceleration, dofs, inp->target_acceleration);
        fill_limit(inp->max_jerk, dofs);

        if (!scatti_input_validate(inp, false, true)) continue;

        errors += check_calculation(otg, inp, out);
        tested++;
    }
    inp->synchronization = SCattiSyncTime;
    inp->duration_discretization = SCattiContinuous;

    /* ==================== Position random 1-DOF ==================== */
    SCatti *otg1 = scatti_create(1, 0.005);
    SCattiInputParameter *inp1 = scatti_input_create(1);
    SCattiOutputParameter *out1 = scatti_output_create(1);

    printf("Position random 1-DOF (%zu)...\n", position_random_1);
    for (size_t i = 0; i < position_random_1; i++) {
        scatti_reset(otg1);
        fill_position(inp1->current_position, 1);
        fill_dynamic_or_zero(inp1->current_velocity, 1, 0.9);
        fill_dynamic_or_zero(inp1->current_acceleration, 1, 0.8);
        fill_position(inp1->target_position, 1);
        fill_dynamic_or_zero(inp1->target_velocity, 1, 0.7);
        fill_dynamic_or_zero(inp1->target_acceleration, 1, 0.6);
        fill_limit_with_offset(inp1->max_velocity, 1, inp1->target_velocity);
        fill_limit_with_offset(inp1->max_acceleration, 1, inp1->target_acceleration);
        fill_limit(inp1->max_jerk, 1);

        if (!scatti_input_validate(inp1, false, true)) continue;

        errors += check_calculation(otg1, inp1, out1);
        tested++;
    }

    scatti_output_destroy(out1);
    scatti_input_destroy(inp1);
    scatti_destroy(otg1);

    /* ==================== Position discrete 3-DOF ==================== */
    printf("Position discrete 3-DOF (%zu)...\n", random_discrete_3);
    for (size_t i = 0; i < random_discrete_3; i++) {
        scatti_reset(otg);

        if (i < random_discrete_3 / 2) {
            inp->synchronization = SCattiSyncPhase;
        } else {
            inp->synchronization = SCattiSyncTime;
        }

        fill_discrete(inp->current_position, dofs);
        fill_discrete(inp->current_velocity, dofs);
        fill_discrete(inp->current_acceleration, dofs);
        fill_discrete(inp->target_position, dofs);
        fill_discrete(inp->target_velocity, dofs);
        fill_discrete(inp->target_acceleration, dofs);
        fill_discrete_limit_with_offset(inp->max_velocity, dofs, inp->target_velocity);
        fill_discrete_limit_with_offset(inp->max_acceleration, dofs, inp->target_acceleration);
        fill_discrete_limit(inp->max_jerk, dofs);

        if (!scatti_input_validate(inp, false, true)) {
            continue;
        }

        errors += check_calculation(otg, inp, out);
        tested++;
    }
    inp->synchronization = SCattiSyncTime;

    /* ==================== Position high limits 3-DOF ==================== */
    printf("Position high limits 3-DOF (%zu)...\n", random_3_high);
    for (size_t i = 0; i < random_3_high; i++) {
        scatti_reset(otg);

        fill_position(inp->current_position, dofs);
        fill_dynamic_or_zero(inp->current_velocity, dofs, 0.1);
        fill_dynamic_or_zero(inp->current_acceleration, dofs, 0.1);
        fill_position(inp->target_position, dofs);
        fill_dynamic_or_zero(inp->target_velocity, dofs, 0.1);
        fill_dynamic_or_zero(inp->target_acceleration, dofs, 0.1);
        fill_limit_high_with_offset(inp->max_velocity, dofs, inp->target_velocity);
        fill_limit_high_with_offset(inp->max_acceleration, dofs, inp->target_acceleration);
        fill_limit_high(inp->max_jerk, dofs);

        if (!scatti_input_validate(inp, false, true)) {
            continue;
        }

        errors += check_calculation(otg, inp, out);
        tested++;
    }

    /* ==================== Position second order 3-DOF ==================== */
    printf("Position second order 3-DOF (%zu)...\n", position_second_random_3);
    /* Reset max_jerk to INFINITY for second-order mode */
    for (size_t d = 0; d < dofs; d++) inp->max_jerk[d] = INFINITY;
    for (size_t i = 0; i < position_second_random_3; i++) {
        scatti_reset(otg);

        if (i < position_second_random_3 / 2) {
            inp->synchronization = SCattiSyncPhase;
        } else {
            inp->synchronization = SCattiSyncTime;
        }

        if (i < position_second_random_3 / 20) {
            inp->duration_discretization = SCattiDiscrete;
        } else {
            inp->duration_discretization = SCattiContinuous;
        }

        fill_position(inp->current_position, dofs);
        fill_dynamic_or_zero(inp->current_velocity, dofs, 0.9);
        fill_dynamic_or_zero(inp->current_acceleration, dofs, 0.8);
        fill_position(inp->target_position, dofs);
        fill_dynamic_or_zero(inp->target_velocity, dofs, 0.7);
        fill_dynamic_or_zero(inp->target_acceleration, dofs, 0.6);
        fill_limit_with_offset(inp->max_velocity, dofs, inp->target_velocity);
        fill_limit_with_offset(inp->max_acceleration, dofs, inp->target_acceleration);
        /* max_jerk stays INFINITY -> second order */

        if (!scatti_input_validate(inp, false, true)) {
            continue;
        }

        errors += check_calculation(otg, inp, out);
        tested++;
    }
    inp->synchronization = SCattiSyncTime;
    inp->duration_discretization = SCattiContinuous;
    /* Restore max_jerk for subsequent tests */
    fill_limit(inp->max_jerk, dofs);

    /* ==================== Step-through 3-DOF ==================== */
    if (step_through_3_count > 0) {
        SCatti *otg_st = scatti_create(dofs, 0.01);
        SCattiInputParameter *inp_st = scatti_input_create(dofs);
        SCattiOutputParameter *out_st = scatti_output_create(dofs);

        printf("Step-through 3-DOF (%zu)...\n", step_through_3_count);
        for (size_t i = 0; i < step_through_3_count; ) {
            scatti_reset(otg_st);

            fill_position(inp_st->current_position, dofs);
            fill_dynamic_or_zero(inp_st->current_velocity, dofs, 0.9);
            fill_dynamic_or_zero(inp_st->current_acceleration, dofs, 0.8);
            fill_position(inp_st->target_position, dofs);
            fill_dynamic_or_zero(inp_st->target_velocity, dofs, 0.7);
            fill_dynamic_or_zero(inp_st->target_acceleration, dofs, 0.6);
            fill_limit_with_offset(inp_st->max_velocity, dofs, inp_st->target_velocity);
            fill_limit_with_offset(inp_st->max_acceleration, dofs, inp_st->target_acceleration);
            fill_limit(inp_st->max_jerk, dofs);

            if (!scatti_input_validate(inp_st, false, true)) {
                continue;
            }

            i += step_through_check(otg_st, inp_st, out_st, 1000);
            tested++;
        }

        scatti_output_destroy(out_st);
        scatti_input_destroy(inp_st);
        scatti_destroy(otg_st);
    }

    /* ==================== Velocity random 3-DOF ==================== */
    printf("Velocity random 3-DOF (%zu)...\n", velocity_random_3);
    inp->control_interface = SCattiVelocity;
    inp->current_position[0] = 0.0; inp->current_position[1] = 0.0; inp->current_position[2] = 0.0;
    for (size_t i = 0; i < velocity_random_3; i++) {
        scatti_reset(otg);

        fill_dynamic_or_zero(inp->current_velocity, dofs, 0.9);
        fill_dynamic_or_zero(inp->current_acceleration, dofs, 0.8);
        fill_dynamic_or_zero(inp->target_velocity, dofs, 0.7);
        fill_dynamic_or_zero(inp->target_acceleration, dofs, 0.6);
        fill_limit_with_offset(inp->max_acceleration, dofs, inp->target_acceleration);
        fill_limit(inp->max_jerk, dofs);

        errors += check_calculation(otg, inp, out);
        tested++;
    }

    /* ==================== Velocity discrete 3-DOF ==================== */
    printf("Velocity discrete 3-DOF (%zu)...\n", velocity_random_discrete_3);
    inp->control_interface = SCattiVelocity;
    for (size_t i = 0; i < velocity_random_discrete_3; i++) {
        scatti_reset(otg);

        if (i < velocity_random_discrete_3 / 2) {
            inp->synchronization = SCattiSyncPhase;
        } else {
            inp->synchronization = SCattiSyncTime;
        }

        fill_discrete(inp->current_position, dofs);
        fill_discrete(inp->current_velocity, dofs);
        fill_discrete(inp->current_acceleration, dofs);
        fill_discrete(inp->target_position, dofs);
        fill_discrete(inp->target_velocity, dofs);
        fill_discrete(inp->target_acceleration, dofs);
        fill_discrete_limit_with_offset(inp->max_velocity, dofs, inp->target_velocity);
        fill_discrete_limit_with_offset(inp->max_acceleration, dofs, inp->target_acceleration);
        fill_discrete_limit(inp->max_jerk, dofs);

        if (!scatti_input_validate(inp, false, true)) {
            continue;
        }

        errors += check_calculation(otg, inp, out);
        tested++;
    }
    inp->synchronization = SCattiSyncTime;

    /* ==================== Velocity second order 3-DOF ==================== */
    printf("Velocity second order 3-DOF (%zu)...\n", velocity_second_random_3);
    inp->control_interface = SCattiVelocity;
    inp->current_position[0] = 0.0; inp->current_position[1] = 0.0; inp->current_position[2] = 0.0;
    /* Reset max_jerk to INFINITY for second-order mode */
    for (size_t d = 0; d < dofs; d++) inp->max_jerk[d] = INFINITY;
    for (size_t i = 0; i < velocity_second_random_3; i++) {
        scatti_reset(otg);

        fill_dynamic_or_zero(inp->current_velocity, dofs, 0.9);
        fill_dynamic_or_zero(inp->current_acceleration, dofs, 0.8);
        fill_dynamic_or_zero(inp->target_velocity, dofs, 0.7);
        fill_dynamic_or_zero(inp->target_acceleration, dofs, 0.6);
        fill_limit_with_offset(inp->max_acceleration, dofs, inp->target_acceleration);
        /* max_jerk stays INFINITY -> second order */

        errors += check_calculation(otg, inp, out);
        tested++;
    }

    /* ==================== Random direction 3-DOF (asymmetric limits) ==================== */
    printf("Random direction 3-DOF (%zu)...\n", random_direction_3);
    inp->control_interface = SCattiPosition;

    /* Allocate min_velocity and min_acceleration */
    inp->min_velocity = (double*)calloc(dofs, sizeof(double));
    inp->min_acceleration = (double*)calloc(dofs, sizeof(double));

    for (size_t i = 0; i < random_direction_3; i++) {
        scatti_reset(otg);

        fill_position(inp->current_position, dofs);
        fill_dynamic_or_zero(inp->current_velocity, dofs, 0.9);
        fill_dynamic_or_zero(inp->current_acceleration, dofs, 0.8);
        fill_position(inp->target_position, dofs);
        fill_dynamic_or_zero(inp->target_velocity, dofs, 0.7);
        fill_dynamic_or_zero(inp->target_acceleration, dofs, 0.6);
        fill_limit_with_offset(inp->max_velocity, dofs, inp->target_velocity);
        fill_limit_with_offset(inp->max_acceleration, dofs, inp->target_acceleration);
        fill_limit(inp->max_jerk, dofs);
        fill_min_limit_with_offset(inp->min_velocity, dofs, inp->target_velocity);
        fill_min_limit_with_offset(inp->min_acceleration, dofs, inp->target_acceleration);

        if (!scatti_input_validate(inp, false, true)) {
            continue;
        }

        errors += check_calculation(otg, inp, out);
        tested++;
    }

    /* Clean up min limits for further tests */
    free(inp->min_velocity);
    inp->min_velocity = NULL;
    free(inp->min_acceleration);
    inp->min_acceleration = NULL;

    clock_gettime(CLOCK_MONOTONIC, &end_ts);
    double elapsed = (end_ts.tv_sec - start.tv_sec) + (end_ts.tv_nsec - start.tv_nsec) / 1e9;

    printf("\n=== Results ===\n");
    printf("Tested: %zu trajectories\n", tested);
    printf("Errors: %zu (%.4f%%)\n", errors, tested > 0 ? 100.0 * errors / tested : 0.0);
    printf("Assertions: %zu (%zu failures)\n", n_assertions, n_assertion_failures);
    printf("Time:   %.3f seconds\n", elapsed);

    scatti_output_destroy(out);
    scatti_input_destroy(inp);
    scatti_destroy(otg);

    return errors > 0 ? 1 : 0;
}
