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

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <scatti/scatti.h>

static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define APPROX_EPS 1e-4

#define CHECK_APPROX(val, expected, msg) do { \
    tests_run++; \
    if (fabs((double)(val) - (double)(expected)) < APPROX_EPS) { \
        tests_passed++; \
    } else { \
        tests_failed++; \
        printf("  FAIL: %s: got %.10f, expected %.10f (line %d)\n", msg, (double)(val), (double)(expected), __LINE__); \
    } \
} while(0)

#define CHECK(cond, msg) do { \
    tests_run++; \
    if (cond) { \
        tests_passed++; \
    } else { \
        tests_failed++; \
        printf("  FAIL: %s (line %d)\n", msg, __LINE__); \
    } \
} while(0)

static void set_arr3(double *arr, double a, double b, double c) {
    arr[0] = a; arr[1] = b; arr[2] = c;
}

static void set_arr2(double *arr, double a, double b) {
    arr[0] = a; arr[1] = b;
}

static bool arrays_approx_equal7(const double *a, const double *b) {
    for (size_t i = 0; i < 7; i++) {
        if (fabs(a[i] - b[i]) >= APPROX_EPS) return false;
    }
    return true;
}

/* Helper: run update loop until finished, return duration */
static double run_to_finish(SCatti *otg, SCattiInputParameter *inp, SCattiOutputParameter *out) {
    SCattiResult result;
    int max_iter = 200000;
    while (max_iter-- > 0) {
        result = scatti_update(otg, inp, out);
        if (result != SCattiWorking) break;
        scatti_output_pass_to_input(out, inp);
    }
    return scatti_trajectory_get_duration(out->trajectory);
}

/* Helper: allocate and set per-DOF synchronization */
static void set_per_dof_sync3(SCattiInputParameter *inp, SCattiSynchronization s0, SCattiSynchronization s1, SCattiSynchronization s2) {
    if (!inp->per_dof_synchronization) {
        inp->per_dof_synchronization = (SCattiSynchronization*)malloc(3 * sizeof(SCattiSynchronization));
    }
    inp->per_dof_synchronization[0] = s0;
    inp->per_dof_synchronization[1] = s1;
    inp->per_dof_synchronization[2] = s2;
}

/* Helper: allocate and set per-DOF control interface */
static void set_per_dof_ci3(SCattiInputParameter *inp, SCattiControlInterface c0, SCattiControlInterface c1, SCattiControlInterface c2) {
    if (!inp->per_dof_control_interface) {
        inp->per_dof_control_interface = (SCattiControlInterface*)malloc(3 * sizeof(SCattiControlInterface));
    }
    inp->per_dof_control_interface[0] = c0;
    inp->per_dof_control_interface[1] = c1;
    inp->per_dof_control_interface[2] = c2;
}

static void set_enabled3(SCattiInputParameter *inp, bool e0, bool e1, bool e2) {
    inp->enabled[0] = e0;
    inp->enabled[1] = e1;
    inp->enabled[2] = e2;
}

/* ========================== Trajectory Tests ========================== */

static void test_trajectory_basic(void) {
    printf("Test: trajectory basic...\n");

    SCatti *otg = scatti_create(3, 0.005);
    SCattiInputParameter *inp = scatti_input_create(3);
    SCattiOutputParameter *out = scatti_output_create(3);

    set_arr3(inp->current_position, 0.0, -2.0, 0.0);
    set_arr3(inp->current_velocity, 0.0, 0.0, 0.0);
    set_arr3(inp->current_acceleration, 0.0, 0.0, 0.0);
    set_arr3(inp->target_position, 1.0, -3.0, 2.0);
    set_arr3(inp->target_velocity, 0.0, 0.3, 0.0);
    set_arr3(inp->target_acceleration, 0.0, 0.0, 0.0);
    set_arr3(inp->max_velocity, 1.0, 1.0, 1.0);
    set_arr3(inp->max_acceleration, 1.0, 1.0, 1.0);
    set_arr3(inp->max_jerk, 1.0, 1.0, 1.0);

    /* Calculate offline */
    SCattiTrajectory *traj = scatti_trajectory_create(3);
    SCattiResult result = scatti_calculate(otg, inp, traj);
    CHECK(result == SCattiWorking, "calculate returns Working");
    CHECK_APPROX(scatti_trajectory_get_duration(traj), 4.0, "duration == 4.0");

    /* Update online */
    result = scatti_update(otg, inp, out);
    CHECK(result == SCattiWorking, "update returns Working");
    CHECK_APPROX(scatti_trajectory_get_duration(out->trajectory), 4.0, "update duration == 4.0");

    /* at_time(0.0) == current state */
    double pos[3], vel[3], acc[3], jerk[3];
    size_t section;
    scatti_trajectory_at_time(out->trajectory, 0.0, pos, vel, acc, jerk, &section);
    CHECK_APPROX(pos[0], 0.0, "pos[0] at t=0");
    CHECK_APPROX(pos[1], -2.0, "pos[1] at t=0");
    CHECK_APPROX(pos[2], 0.0, "pos[2] at t=0");
    CHECK_APPROX(vel[0], 0.0, "vel[0] at t=0");
    CHECK_APPROX(vel[1], 0.0, "vel[1] at t=0");
    CHECK_APPROX(vel[2], 0.0, "vel[2] at t=0");

    /* at_time(duration) == target state */
    scatti_trajectory_at_time(out->trajectory, scatti_trajectory_get_duration(out->trajectory), pos, vel, acc, jerk, &section);
    CHECK_APPROX(pos[0], 1.0, "pos[0] at end");
    CHECK_APPROX(pos[1], -3.0, "pos[1] at end");
    CHECK_APPROX(pos[2], 2.0, "pos[2] at end");
    CHECK_APPROX(vel[0], 0.0, "vel[0] at end");
    CHECK_APPROX(vel[1], 0.3, "vel[1] at end");
    CHECK_APPROX(vel[2], 0.0, "vel[2] at end");

    /* at_time(2.0) specific values */
    scatti_trajectory_at_time(out->trajectory, 2.0, pos, vel, acc, jerk, &section);
    CHECK_APPROX(pos[0], 0.5, "pos[0] at t=2");
    CHECK_APPROX(pos[1], -2.6871268303, "pos[1] at t=2");
    CHECK_APPROX(pos[2], 1.0, "pos[2] at t=2");
    CHECK_APPROX(jerk[2], -1.0, "jerk[2] at t=2");
    CHECK(section == 0, "section == 0 at t=2");

    /* at_time(5.0) past duration: jerk should be zero, section=1 */
    scatti_trajectory_at_time(out->trajectory, 5.0, pos, vel, acc, jerk, &section);
    CHECK_APPROX(jerk[0], 0.0, "jerk[0] at t=5 (past duration)");
    CHECK_APPROX(jerk[1], 0.0, "jerk[1] at t=5 (past duration)");
    CHECK_APPROX(jerk[2], 0.0, "jerk[2] at t=5 (past duration)");
    CHECK(section == 1, "section == 1 at t=5 (past duration)");

    /* Independent min durations */
    double *min_dur = out->trajectory->independent_min_durations;
    CHECK_APPROX(min_dur[0], 3.1748021039, "min_dur[0]");
    CHECK_APPROX(min_dur[1], 3.6860977315, "min_dur[1]");
    CHECK_APPROX(min_dur[2], scatti_trajectory_get_duration(out->trajectory), "min_dur[2]");

    /* Position extrema - full check */
    scatti_trajectory_get_position_extrema(out->trajectory);
    SCattiBound *ext = out->trajectory->position_extrema;
    CHECK_APPROX(ext[0].max, 1.0, "ext[0].max");
    CHECK_APPROX(ext[0].t_max, 4.0, "ext[0].t_max");
    CHECK_APPROX(ext[0].min, 0.0, "ext[0].min");
    CHECK_APPROX(ext[0].t_min, 0.0, "ext[0].t_min");
    CHECK_APPROX(ext[1].max, -2.0, "ext[1].max");
    CHECK_APPROX(ext[1].t_max, 0.0, "ext[1].t_max");
    CHECK_APPROX(ext[1].t_min, 3.2254033308, "ext[1].t_min");
    CHECK_APPROX(ext[1].min, -3.1549193338, "ext[1].min");
    CHECK_APPROX(ext[2].t_max, 4.0, "ext[2].t_max");
    CHECK_APPROX(ext[2].max, 2.0, "ext[2].max");
    CHECK_APPROX(ext[2].t_min, 0.0, "ext[2].t_min");
    CHECK_APPROX(ext[2].min, 0.0, "ext[2].min");

    /* First time at position - all DOFs */
    double time_found;
    bool found;
    found = scatti_trajectory_get_first_time_at_position(out->trajectory, 0, 0.0, &time_found, 0.0);
    CHECK(found, "first_time pos[0]=0.0 found");
    CHECK_APPROX(time_found, 0.0, "first_time pos[0]=0.0");

    found = scatti_trajectory_get_first_time_at_position(out->trajectory, 0, 0.5, &time_found, 0.0);
    CHECK(found, "first_time pos[0]=0.5 found");
    CHECK_APPROX(time_found, 2.0, "first_time pos[0]=0.5");

    found = scatti_trajectory_get_first_time_at_position(out->trajectory, 0, 1.0, &time_found, 0.0);
    CHECK(found, "first_time pos[0]=1.0 found");
    CHECK_APPROX(time_found, 4.0, "first_time pos[0]=1.0");

    found = scatti_trajectory_get_first_time_at_position(out->trajectory, 1, -3.0, &time_found, 0.0);
    CHECK(found, "first_time pos[1]=-3.0 found");
    CHECK_APPROX(time_found, 2.6004877902, "first_time pos[1]=-3.0");

    found = scatti_trajectory_get_first_time_at_position(out->trajectory, 1, -3.1, &time_found, 0.0);
    CHECK(found, "first_time pos[1]=-3.1 found");
    CHECK_APPROX(time_found, 2.8644154489, "first_time pos[1]=-3.1");

    found = scatti_trajectory_get_first_time_at_position(out->trajectory, 2, 0.05, &time_found, 0.0);
    CHECK(found, "first_time pos[2]=0.05 found");
    CHECK_APPROX(time_found, 0.6694329501, "first_time pos[2]=0.05");

    found = scatti_trajectory_get_first_time_at_position(out->trajectory, 0, -1.0, &time_found, 0.0);
    CHECK(!found, "no first_time for pos[0]=-1.0");

    found = scatti_trajectory_get_first_time_at_position(out->trajectory, 1, -3.4, &time_found, 0.0);
    CHECK(!found, "no first_time for pos[1]=-3.4");

    found = scatti_trajectory_get_first_time_at_position(out->trajectory, 6, 0.0, &time_found, 0.0);
    CHECK(!found, "no first_time for out-of-range DOF 6");

    scatti_trajectory_destroy(traj);
    scatti_output_destroy(out);
    scatti_input_destroy(inp);
    scatti_destroy(otg);
}

static void test_single_dof(void) {
    printf("Test: single DOF...\n");

    SCatti *otg = scatti_create(1, 0.005);
    SCattiInputParameter *inp = scatti_input_create(1);

    inp->current_position[0] = 0.0;
    inp->target_position[0] = 1.0;
    inp->max_velocity[0] = 1.0;
    inp->max_acceleration[0] = 1.0;
    inp->max_jerk[0] = 1.0;

    SCattiTrajectory *traj = scatti_trajectory_create(1);
    SCattiResult result = scatti_calculate(otg, inp, traj);
    CHECK(result == SCattiWorking, "single DOF calculate");
    CHECK_APPROX(scatti_trajectory_get_duration(traj), 3.1748, "single DOF duration");

    /* at_time(0.0) should equal current_position */
    double pos, vel_val, acc_val;
    scatti_trajectory_at_time_simple(traj, 0.0, &pos, &vel_val, &acc_val);
    CHECK_APPROX(pos, 0.0, "single DOF pos at t=0");

    /* midpoint */
    scatti_trajectory_at_time_simple(traj, 3.1748 / 2.0, &pos, &vel_val, &acc_val);
    CHECK_APPROX(pos, 0.5, "single DOF midpoint");

    scatti_trajectory_destroy(traj);
    scatti_input_destroy(inp);
    scatti_destroy(otg);
}

static void test_velocity_interface(void) {
    printf("Test: velocity interface...\n");

    SCatti *otg = scatti_create(3, 0.005);
    SCattiInputParameter *inp = scatti_input_create(3);
    SCattiOutputParameter *out = scatti_output_create(3);

    inp->control_interface = SCattiVelocity;
    set_arr3(inp->current_position, 0.0, 0.0, 0.0);
    set_arr3(inp->current_velocity, 0.0, -1.0, 0.0);
    set_arr3(inp->current_acceleration, 0.0, 0.0, 0.0);
    set_arr3(inp->target_velocity, 1.0, 1.0, -0.5);
    set_arr3(inp->target_acceleration, 0.0, 0.0, 0.0);
    set_arr3(inp->max_acceleration, 1.0, 1.0, 1.0);
    set_arr3(inp->max_jerk, 1.0, 1.0, 1.0);

    SCattiResult result = scatti_update(otg, inp, out);
    CHECK(result == SCattiWorking, "velocity interface update");
    double dur = scatti_trajectory_get_duration(out->trajectory);
    CHECK(dur > 0.0, "velocity duration > 0");

    scatti_output_destroy(out);
    scatti_input_destroy(inp);
    scatti_destroy(otg);
}

static void test_minimum_duration(void) {
    printf("Test: minimum duration...\n");

    SCatti *otg = scatti_create(3, 0.005);
    SCattiInputParameter *inp = scatti_input_create(3);
    SCattiOutputParameter *out = scatti_output_create(3);

    set_arr3(inp->current_position, 0.0, -2.0, 0.0);
    set_arr3(inp->current_velocity, 0.0, 0.0, 0.0);
    set_arr3(inp->current_acceleration, 0.0, 0.0, 0.0);
    set_arr3(inp->target_position, 1.0, -3.0, 2.0);
    set_arr3(inp->target_velocity, 0.2, -0.3, 0.8);
    set_arr3(inp->target_acceleration, 0.0, 0.0, 0.0);
    set_arr3(inp->max_velocity, 1.0, 1.0, 1.0);
    set_arr3(inp->max_acceleration, 1.0, 1.0, 1.0);
    set_arr3(inp->max_jerk, 1.0, 1.0, 1.0);
    inp->has_minimum_duration = true;
    inp->minimum_duration = 12.0;

    SCattiResult result = scatti_update(otg, inp, out);
    CHECK(result == SCattiWorking, "min duration update");
    CHECK_APPROX(scatti_trajectory_get_duration(out->trajectory), 12.0, "min duration == 12.0");

    scatti_output_destroy(out);
    scatti_input_destroy(inp);
    scatti_destroy(otg);
}

static void test_high_speed(void) {
    printf("Test: high speed trajectory...\n");

    SCatti *otg = scatti_create(3, 0.005);
    SCattiInputParameter *inp = scatti_input_create(3);
    SCattiOutputParameter *out = scatti_output_create(3);

    set_arr3(inp->current_position, 1300.0, 0.0, 0.02);
    set_arr3(inp->current_velocity, 1200.0, 0.0, 0.0);
    set_arr3(inp->current_acceleration, 0.0, 0.0, 0.0);
    set_arr3(inp->target_position, 1400.0, 0.0, 0.02);
    set_arr3(inp->target_velocity, 0.0, 0.0, 0.0);
    set_arr3(inp->target_acceleration, 0.0, 0.0, 0.0);
    set_arr3(inp->max_velocity, 800.0, 1.0, 1.0);
    set_arr3(inp->max_acceleration, 40000.0, 1.0, 1.0);
    set_arr3(inp->max_jerk, 200000.0, 1.0, 1.0);

    SCattiResult result = scatti_update(otg, inp, out);
    CHECK(result == SCattiWorking, "high speed update");
    CHECK_APPROX(scatti_trajectory_get_duration(out->trajectory), 0.167347, "high speed duration");

    /* Independent min durations after high-speed */
    double *min_dur = out->trajectory->independent_min_durations;
    CHECK_APPROX(min_dur[0], scatti_trajectory_get_duration(out->trajectory), "high speed min_dur[0]");
    CHECK_APPROX(min_dur[1], 0.0, "high speed min_dur[1]");
    CHECK_APPROX(min_dur[2], 0.0, "high speed min_dur[2]");

    scatti_output_destroy(out);
    scatti_input_destroy(inp);
    scatti_destroy(otg);
}

static void test_step_through(void) {
    printf("Test: step through trajectory...\n");

    SCatti *otg = scatti_create(3, 0.005);
    SCattiInputParameter *inp = scatti_input_create(3);
    SCattiOutputParameter *out = scatti_output_create(3);

    set_arr3(inp->current_position, 0.0, -2.0, 0.0);
    set_arr3(inp->current_velocity, 0.0, 0.0, 0.0);
    set_arr3(inp->current_acceleration, 0.0, 0.0, 0.0);
    set_arr3(inp->target_position, 1.0, -3.0, 2.0);
    set_arr3(inp->target_velocity, 0.0, 0.3, 0.0);
    set_arr3(inp->target_acceleration, 0.0, 0.0, 0.0);
    set_arr3(inp->max_velocity, 1.0, 1.0, 1.0);
    set_arr3(inp->max_acceleration, 1.0, 1.0, 1.0);
    set_arr3(inp->max_jerk, 1.0, 1.0, 1.0);

    double dur = run_to_finish(otg, inp, out);
    CHECK_APPROX(dur, 4.0, "step-through duration == 4.0");

    CHECK(fabs(out->new_position[0] - 1.0) < 0.01, "final pos[0]");
    CHECK(fabs(out->new_position[1] - (-3.0)) < 0.01, "final pos[1]");
    CHECK(fabs(out->new_position[2] - 2.0) < 0.01, "final pos[2]");

    scatti_output_destroy(out);
    scatti_input_destroy(inp);
    scatti_destroy(otg);
}

static void test_second_order(void) {
    printf("Test: second order (infinite jerk)...\n");

    SCatti *otg = scatti_create(1, 0.005);
    SCattiInputParameter *inp = scatti_input_create(1);
    SCattiOutputParameter *out = scatti_output_create(1);

    inp->current_position[0] = 0.0;
    inp->target_position[0] = 1.0;
    inp->max_velocity[0] = 1.0;
    inp->max_acceleration[0] = 1.0;
    inp->max_jerk[0] = INFINITY;

    SCattiResult result = scatti_update(otg, inp, out);
    CHECK(result == SCattiWorking, "second order update");
    double dur = scatti_trajectory_get_duration(out->trajectory);
    CHECK(dur > 0.0, "second order duration > 0");

    scatti_output_destroy(out);
    scatti_input_destroy(inp);
    scatti_destroy(otg);
}

static void test_first_order(void) {
    printf("Test: first order (infinite jerk+accel)...\n");

    SCatti *otg = scatti_create(1, 0.005);
    SCattiInputParameter *inp = scatti_input_create(1);
    SCattiOutputParameter *out = scatti_output_create(1);

    inp->current_position[0] = 0.0;
    inp->target_position[0] = 1.0;
    inp->max_velocity[0] = 1.0;
    inp->max_acceleration[0] = INFINITY;
    inp->max_jerk[0] = INFINITY;

    SCattiResult result = scatti_update(otg, inp, out);
    CHECK(result == SCattiWorking, "first order update");
    double dur = scatti_trajectory_get_duration(out->trajectory);
    CHECK_APPROX(dur, 1.0, "first order duration == 1.0");

    scatti_output_destroy(out);
    scatti_input_destroy(inp);
    scatti_destroy(otg);
}

/* ========================== Error / new_calculation ========================== */

static void test_error_and_new_calculation(void) {
    printf("Test: error handling and new_calculation flag...\n");

    SCatti *otg = scatti_create(3, 0.005);
    SCattiInputParameter *inp = scatti_input_create(3);
    SCattiOutputParameter *out = scatti_output_create(3);

    set_arr3(inp->current_position, 0.0, -2.0, 0.0);
    set_arr3(inp->current_velocity, 0.0, 0.0, 0.0);
    set_arr3(inp->current_acceleration, 0.0, 0.0, 0.0);
    set_arr3(inp->target_position, 1.0, -3.0, 2.0);
    set_arr3(inp->target_velocity, 2.0, 0.3, 0.0);  /* target_velocity[0]=2.0 > max_velocity[0]=1.0 */
    set_arr3(inp->target_acceleration, 0.0, 0.0, 0.0);
    set_arr3(inp->max_velocity, 1.0, 1.0, 1.0);
    set_arr3(inp->max_acceleration, 1.0, 1.0, 1.0);
    set_arr3(inp->max_jerk, 1.0, 1.0, 1.0);

    SCattiResult result = scatti_update(otg, inp, out);
    CHECK(result == SCattiErrorInvalidInput, "target vel exceeds max -> error");
    CHECK(!out->new_calculation, "new_calculation false after error");

    /* Fix input and try again */
    set_arr3(inp->target_velocity, 0.2, -0.3, 0.8);
    result = scatti_update(otg, inp, out);
    CHECK(result == SCattiWorking, "valid input -> working");
    CHECK(out->new_calculation, "new_calculation true after valid update");

    scatti_output_destroy(out);
    scatti_input_destroy(inp);
    scatti_destroy(otg);
}

/* ========================== Input Validation ========================== */

static void test_input_validation(void) {
    printf("Test: input validation...\n");

    SCattiInputParameter *inp = scatti_input_create(2);
    set_arr2(inp->current_position, 0.0, -2.0);
    set_arr2(inp->current_velocity, 0.0, 0.0);
    set_arr2(inp->current_acceleration, 0.0, 0.0);
    set_arr2(inp->target_position, 1.0, -3.0);
    set_arr2(inp->target_velocity, 0.0, 0.3);
    set_arr2(inp->target_acceleration, 0.0, 0.0);
    set_arr2(inp->max_velocity, 1.0, 1.0);
    set_arr2(inp->max_acceleration, 1.0, 1.0);
    set_arr2(inp->max_jerk, 1.0, 1.0);

    CHECK(scatti_input_validate(inp, false, false), "valid input passes");
    CHECK(scatti_input_validate(inp, true, true), "valid input full check");

    /* NaN in max_jerk */
    double saved = inp->max_jerk[1];
    inp->max_jerk[1] = NAN;
    CHECK(!scatti_input_validate(inp, false, false), "NaN max_jerk fails");
    inp->max_jerk[1] = saved;

    /* NaN in current_position */
    saved = inp->current_position[1];
    inp->current_position[1] = NAN;
    CHECK(!scatti_input_validate(inp, false, false), "NaN current_position fails");
    inp->current_position[1] = saved;

    /* Negative max_acceleration */
    saved = inp->max_acceleration[1];
    inp->max_acceleration[1] = -1.0;
    CHECK(!scatti_input_validate(inp, false, false), "negative max_accel fails");
    inp->max_acceleration[1] = saved;

    /* Negative max_velocity */
    saved = inp->max_velocity[1];
    inp->max_velocity[1] = -1.0;
    CHECK(!scatti_input_validate(inp, false, false), "negative max_vel fails");
    inp->max_velocity[1] = saved;

    /* Target velocity exceeds max */
    inp->target_velocity[1] = 1.3;
    CHECK(scatti_input_validate(inp, false, false), "target_vel>max without check passes");
    CHECK(!scatti_input_validate(inp, false, true), "target_vel>max with target check fails");
    inp->target_velocity[1] = 0.3;

    /* Current velocity exceeds max */
    inp->current_velocity[0] = 2.0;
    CHECK(scatti_input_validate(inp, false, false), "curr_vel>max without check passes");
    CHECK(!scatti_input_validate(inp, true, false), "curr_vel>max with current check fails");
    CHECK(scatti_input_validate(inp, false, true), "curr_vel>max with only target check passes");
    inp->current_velocity[0] = 0.0;

    /* Current velocity + acceleration: decelerating should pass */
    inp->current_velocity[0] = 1.0;
    inp->current_acceleration[0] = -1.0;
    CHECK(scatti_input_validate(inp, true, true), "v0=1,a0=-1 passes (decelerating)");

    /* Current velocity + acceleration: accelerating beyond max should fail */
    inp->current_acceleration[0] = 1.0;
    CHECK(!scatti_input_validate(inp, true, true), "v0=1,a0=1 fails (will exceed)");

    /* Edge case: v0=0.72, a0=0.72 should pass */
    inp->current_velocity[0] = 0.72;
    inp->current_acceleration[0] = 0.72;
    CHECK(scatti_input_validate(inp, true, true), "v0=0.72,a0=0.72 passes");

    /* Target velocity + acceleration */
    inp->current_velocity[0] = 0.0;
    inp->current_acceleration[0] = 0.0;
    inp->target_velocity[1] = 0.72;
    inp->target_acceleration[1] = 0.72;
    CHECK(scatti_input_validate(inp, true, true), "target v=0.72,a=0.72 passes");

    inp->target_velocity[1] = 1.0;
    inp->target_acceleration[1] = -0.0001;
    CHECK(!scatti_input_validate(inp, true, true), "target v=1.0,a=-0.0001 fails (will exceed)");

    inp->target_velocity[1] = 0.3;
    inp->target_acceleration[1] = 0.0;

    scatti_input_destroy(inp);
}

/* ========================== Enabled DOF Flag ========================== */

static void test_enabled(void) {
    printf("Test: enabled DOF flag...\n");

    SCatti *otg = scatti_create(3, 0.005);
    SCattiInputParameter *inp = scatti_input_create(3);
    SCattiOutputParameter *out = scatti_output_create(3);

    set_enabled3(inp, true, false, false);
    set_arr3(inp->current_position, 0.0, -2.0, 0.0);
    set_arr3(inp->current_velocity, 0.0, 0.1, 0.0);
    set_arr3(inp->current_acceleration, 0.0, 0.0, -0.2);
    set_arr3(inp->target_position, 1.0, -3.0, 2.0);
    set_arr3(inp->target_velocity, 0.0, 0.0, 0.0);
    set_arr3(inp->target_acceleration, 0.0, 0.0, 0.0);
    set_arr3(inp->max_velocity, 1.0, 1.0, 1.0);
    set_arr3(inp->max_acceleration, 1.0, 1.0, 1.0);
    set_arr3(inp->max_jerk, 1.0, 1.0, 1.0);

    SCattiResult result = scatti_update(otg, inp, out);
    CHECK(result == SCattiWorking, "enabled update");
    CHECK_APPROX(scatti_trajectory_get_duration(out->trajectory), 3.1748021039, "enabled duration");

    double pos[3], vel[3], acc[3];
    scatti_trajectory_at_time_simple(out->trajectory, 0.0, pos, vel, acc);
    CHECK_APPROX(pos[0], 0.0, "enabled pos[0] at t=0");
    CHECK_APPROX(vel[1], 0.1, "enabled vel[1] at t=0 (disabled DOF coasts)");

    scatti_trajectory_at_time_simple(out->trajectory, scatti_trajectory_get_duration(out->trajectory), pos, vel, acc);
    CHECK_APPROX(pos[0], 1.0, "enabled DOF 0 reaches target");
    CHECK_APPROX(pos[1], -1.6825197896, "disabled DOF 1 coasts");
    CHECK_APPROX(pos[2], -1.0079368399, "disabled DOF 2 coasts");

    /* Make sure that disabled DoFs overwrite prior blocks */
    set_enabled3(inp, true, true, true);
    set_arr3(inp->current_position, 0.0, 0.0, 0.0);
    set_arr3(inp->target_position, 100.0, -3000.0, 2000.0);
    set_arr3(inp->target_velocity, 1.0, 1.0, 1.0);
    scatti_reset(otg);
    result = scatti_update(otg, inp, out);

    set_enabled3(inp, false, false, true);
    set_arr3(inp->current_position, 0.0, -2.0, 0.0);
    set_arr3(inp->current_velocity, 0.0, 0.2, 0.0);
    set_arr3(inp->current_acceleration, 0.0, 0.2, 0.0);
    set_arr3(inp->target_position, 1.0, -3.0, 2.0);
    set_arr3(inp->target_velocity, 0.0, 0.0, 0.2);
    set_arr3(inp->target_acceleration, 0.0, 0.0, -0.1);
    set_arr3(inp->max_velocity, 1.0, 1.0, 1.0);
    set_arr3(inp->max_acceleration, 1.0, 1.0, 1.0);
    set_arr3(inp->max_jerk, 1.0, 1.0, 1.0);

    scatti_reset(otg);
    result = scatti_update(otg, inp, out);
    CHECK(result == SCattiWorking, "enabled overwrite update");
    CHECK_APPROX(scatti_trajectory_get_duration(out->trajectory), 3.6578610221, "enabled overwrite duration");

    scatti_output_destroy(out);
    scatti_input_destroy(inp);
    scatti_destroy(otg);
}

/* ========================== Phase Synchronization ========================== */

static void test_phase_synchronization(void) {
    printf("Test: phase synchronization...\n");

    SCatti *otg = scatti_create(3, 0.005);
    SCattiInputParameter *inp = scatti_input_create(3);
    SCattiOutputParameter *out = scatti_output_create(3);
    SCattiTrajectory *traj = scatti_trajectory_create(3);

    set_arr3(inp->current_position, 0.0, -2.0, 0.0);
    set_arr3(inp->current_velocity, 0.0, 0.0, 0.0);
    set_arr3(inp->current_acceleration, 0.0, 0.0, 0.0);
    set_arr3(inp->target_position, 1.0, -3.0, 2.0);
    set_arr3(inp->target_velocity, 0.0, 0.0, 0.0);
    set_arr3(inp->target_acceleration, 0.0, 0.0, 0.0);
    set_arr3(inp->max_velocity, 1.0, 1.0, 1.0);
    set_arr3(inp->max_acceleration, 1.0, 1.0, 1.0);
    set_arr3(inp->max_jerk, 1.0, 1.0, 1.0);
    inp->synchronization = SCattiSyncPhase;

    /* Basic equal limits */
    SCattiResult result = scatti_calculate(otg, inp, traj);
    CHECK(result == SCattiWorking, "phase sync calculate");
    CHECK_APPROX(scatti_trajectory_get_duration(traj), 4.0, "phase sync duration");
    const SCattiProfile *p0 = scatti_trajectory_get_profile(traj, 0);
    const SCattiProfile *p1 = scatti_trajectory_get_profile(traj, 1);
    const SCattiProfile *p2 = scatti_trajectory_get_profile(traj, 2);
    CHECK(arrays_approx_equal7(p0->t, p1->t), "phase sync: p0.t == p1.t");
    CHECK(arrays_approx_equal7(p0->t, p2->t), "phase sync: p0.t == p2.t");

    /* Check position at t=1 */
    scatti_reset(otg);
    result = scatti_update(otg, inp, out);
    double pos[3], vel[3], acc[3];
    scatti_trajectory_at_time_simple(out->trajectory, 1.0, pos, vel, acc);
    CHECK_APPROX(pos[0], 0.0833333333, "phase sync pos[0] at t=1");
    CHECK_APPROX(pos[1], -2.0833333333, "phase sync pos[1] at t=1");
    CHECK_APPROX(pos[2], 0.1666666667, "phase sync pos[2] at t=1");

    /* Different max limits, same proportional result */
    set_arr3(inp->current_position, 0.0, -2.0, 0.0);
    set_arr3(inp->target_position, 10.0, -3.0, 2.0);
    set_arr3(inp->max_velocity, 10.0, 2.0, 1.0);
    set_arr3(inp->max_acceleration, 10.0, 2.0, 1.0);
    set_arr3(inp->max_jerk, 10.0, 2.0, 1.0);
    scatti_reset(otg);
    result = scatti_update(otg, inp, out);
    CHECK_APPROX(scatti_trajectory_get_duration(out->trajectory), 4.0, "phase sync diff limits dur");
    scatti_trajectory_at_time_simple(out->trajectory, 1.0, pos, vel, acc);
    CHECK_APPROX(pos[0], 0.8333333333, "phase sync diff limits pos[0]");
    p0 = scatti_trajectory_get_profile(out->trajectory, 0);
    p1 = scatti_trajectory_get_profile(out->trajectory, 1);
    p2 = scatti_trajectory_get_profile(out->trajectory, 2);
    CHECK(arrays_approx_equal7(p0->t, p1->t), "phase sync diff: p0.t == p1.t");
    CHECK(arrays_approx_equal7(p0->t, p2->t), "phase sync diff: p0.t == p2.t");

    /* Equal start and target -> finished */
    set_arr3(inp->current_position, 1.0, -2.0, 3.0);
    set_arr3(inp->target_position, 1.0, -2.0, 3.0);
    set_arr3(inp->max_velocity, 1.0, 1.0, 1.0);
    set_arr3(inp->max_acceleration, 1.0, 1.0, 1.0);
    set_arr3(inp->max_jerk, 1.0, 1.0, 1.0);
    scatti_reset(otg);
    result = scatti_update(otg, inp, out);
    CHECK(result == SCattiFinished, "phase sync equal -> finished");
    CHECK_APPROX(scatti_trajectory_get_duration(out->trajectory), 0.0, "phase sync equal dur=0");

    /* Target velocity only */
    set_arr3(inp->current_position, 0.0, 0.0, 0.0);
    set_arr3(inp->current_velocity, 0.0, 0.0, 0.0);
    set_arr3(inp->current_acceleration, 0.0, 0.0, 0.0);
    set_arr3(inp->target_position, 0.0, 0.0, 0.0);
    set_arr3(inp->target_velocity, 0.2, 0.3, 0.4);
    set_arr3(inp->target_acceleration, 0.0, 0.0, 0.0);
    result = scatti_calculate(otg, inp, traj);
    CHECK(result == SCattiWorking, "phase sync target_vel only");
    p0 = scatti_trajectory_get_profile(traj, 0);
    p1 = scatti_trajectory_get_profile(traj, 1);
    p2 = scatti_trajectory_get_profile(traj, 2);
    CHECK(arrays_approx_equal7(p0->t, p1->t), "phase sync tv: p0.t == p1.t");
    CHECK(arrays_approx_equal7(p0->t, p2->t), "phase sync tv: p0.t == p2.t");

    /* Slight target position difference breaks phase sync */
    set_arr3(inp->target_position, 0.0, 0.0, 0.01);
    result = scatti_calculate(otg, inp, traj);
    p0 = scatti_trajectory_get_profile(traj, 0);
    p1 = scatti_trajectory_get_profile(traj, 1);
    CHECK(!arrays_approx_equal7(p0->t, p1->t), "phase sync broken by pos diff");

    /* Proportional current vel/acc maintains phase sync */
    set_arr3(inp->current_velocity, 0.4, 0.15, 0.2);
    set_arr3(inp->current_acceleration, 0.8, 0.3, 0.4);
    set_arr3(inp->target_position, 0.0, 0.0, 0.0);
    set_arr3(inp->target_velocity, 0.0, 0.0, 0.0);
    set_arr3(inp->target_acceleration, 0.0, 0.0, 0.0);
    result = scatti_calculate(otg, inp, traj);
    p0 = scatti_trajectory_get_profile(traj, 0);
    p1 = scatti_trajectory_get_profile(traj, 1);
    p2 = scatti_trajectory_get_profile(traj, 2);
    CHECK(arrays_approx_equal7(p0->t, p1->t), "phase sync prop: p0.t == p1.t");
    CHECK(arrays_approx_equal7(p0->t, p2->t), "phase sync prop: p0.t == p2.t");

    /* Different max_velocity breaks phase sync */
    set_arr3(inp->max_velocity, 1.0, 0.2, 1.0);
    result = scatti_calculate(otg, inp, traj);
    p0 = scatti_trajectory_get_profile(traj, 0);
    p1 = scatti_trajectory_get_profile(traj, 1);
    CHECK(!arrays_approx_equal7(p0->t, p1->t), "phase sync broken by max_vel");

    /* Velocity control interface + phase sync */
    set_arr3(inp->current_position, 0.0, 0.02, 1.0);
    set_arr3(inp->current_velocity, -0.2, 0.15, 0.2);
    set_arr3(inp->current_acceleration, -0.4, 0.3, 0.4);
    set_arr3(inp->target_position, 0.03, 0.0, 0.0);
    set_arr3(inp->target_velocity, -0.02, 0.015, 0.02);
    set_arr3(inp->target_acceleration, 0.0, 0.0, 0.0);
    set_arr3(inp->max_velocity, 1.0, 1.0, 1.0);
    set_arr3(inp->max_acceleration, 1.0, 1.0, 1.0);
    set_arr3(inp->max_jerk, 1.0, 1.0, 1.0);
    inp->control_interface = SCattiVelocity;
    result = scatti_calculate(otg, inp, traj);
    p0 = scatti_trajectory_get_profile(traj, 0);
    p1 = scatti_trajectory_get_profile(traj, 1);
    p2 = scatti_trajectory_get_profile(traj, 2);
    CHECK(arrays_approx_equal7(p0->t, p1->t), "phase sync vel: p0.t == p1.t");
    CHECK(arrays_approx_equal7(p0->t, p2->t), "phase sync vel: p0.t == p2.t");

    /* Non-zero target_accel breaks phase sync in velocity mode */
    set_arr3(inp->target_acceleration, 0.01, 0.0, 0.0);
    result = scatti_calculate(otg, inp, traj);
    p0 = scatti_trajectory_get_profile(traj, 0);
    p1 = scatti_trajectory_get_profile(traj, 1);
    CHECK(!arrays_approx_equal7(p0->t, p1->t), "phase sync vel broken by target_acc");

    /* All-zero velocity mode */
    set_arr3(inp->current_position, 0.0, 0.0, 0.0);
    set_arr3(inp->current_velocity, 0.0, 0.0, 0.0);
    set_arr3(inp->current_acceleration, 0.0, 0.0, 0.0);
    set_arr3(inp->target_position, 0.0, 0.0, 0.0);
    set_arr3(inp->target_velocity, 0.0, 0.0, 0.0);
    set_arr3(inp->target_acceleration, 0.0, 0.0, 0.0);
    result = scatti_calculate(otg, inp, traj);
    CHECK(result == SCattiWorking, "phase sync vel all-zero");

    inp->control_interface = SCattiPosition;
    inp->synchronization = SCattiSyncTime;

    scatti_trajectory_destroy(traj);
    scatti_output_destroy(out);
    scatti_input_destroy(inp);
    scatti_destroy(otg);
}

/* ========================== Discretization ========================== */

static void test_discretization(void) {
    printf("Test: duration discretization...\n");

    SCatti *otg = scatti_create(3, 0.01);
    SCattiInputParameter *inp = scatti_input_create(3);
    SCattiOutputParameter *out = scatti_output_create(3);
    SCattiTrajectory *traj = scatti_trajectory_create(3);

    set_arr3(inp->current_position, 0.0, 0.0, 0.0);
    set_arr3(inp->current_velocity, 0.0, 0.0, 0.0);
    set_arr3(inp->current_acceleration, 0.0, 0.0, 0.0);
    set_arr3(inp->target_position, 1.0, -3.0, 2.0);
    set_arr3(inp->target_velocity, 0.2, 0.2, 0.2);
    set_arr3(inp->target_acceleration, 0.0, 0.0, 0.0);
    set_arr3(inp->max_velocity, 1.0, 1.0, 1.0);
    set_arr3(inp->max_acceleration, 2.0, 2.0, 2.0);
    set_arr3(inp->max_jerk, 1.8, 2.4, 2.0);
    inp->duration_discretization = SCattiDiscrete;

    SCattiResult result = scatti_calculate(otg, inp, traj);
    CHECK(result == SCattiWorking, "discrete calculate");
    CHECK_APPROX(scatti_trajectory_get_duration(traj), 4.5, "discrete duration == 4.5");

    result = scatti_update(otg, inp, out);
    double pos[3], vel[3], acc[3];
    scatti_trajectory_at_time_simple(out->trajectory, 4.5, pos, vel, acc);
    CHECK_APPROX(pos[0], 1.0, "discrete end pos[0]");
    CHECK_APPROX(pos[1], -3.0, "discrete end pos[1]");
    CHECK_APPROX(pos[2], 2.0, "discrete end pos[2]");

    scatti_trajectory_destroy(traj);
    scatti_output_destroy(out);
    scatti_input_destroy(inp);
    scatti_destroy(otg);
}

/* ========================== Per-DOF Settings ========================== */

static void test_per_dof_setting(void) {
    printf("Test: per-DOF settings...\n");

    SCatti *otg = scatti_create(3, 0.005);
    SCattiInputParameter *inp = scatti_input_create(3);
    SCattiTrajectory *traj = scatti_trajectory_create(3);
    double pos[3], vel[3], acc[3];

    set_arr3(inp->current_position, 0.0, -2.0, 0.0);
    set_arr3(inp->current_velocity, 0.0, 0.0, 0.0);
    set_arr3(inp->current_acceleration, 0.0, 0.0, 0.0);
    set_arr3(inp->target_position, 1.0, -3.0, 2.0);
    set_arr3(inp->target_velocity, 0.0, 0.3, 0.0);
    set_arr3(inp->target_acceleration, 0.0, 0.0, 0.0);
    set_arr3(inp->max_velocity, 1.0, 1.0, 1.0);
    set_arr3(inp->max_acceleration, 1.0, 1.0, 1.0);
    set_arr3(inp->max_jerk, 1.0, 1.0, 1.0);

    /* Baseline position control */
    SCattiResult result = scatti_calculate(otg, inp, traj);
    CHECK(result == SCattiWorking, "per-dof baseline");
    CHECK_APPROX(scatti_trajectory_get_duration(traj), 4.0, "per-dof baseline dur");
    scatti_trajectory_at_time_simple(traj, 2.0, pos, vel, acc);
    CHECK_APPROX(pos[0], 0.5, "per-dof baseline pos[0]");

    /* Global velocity control */
    inp->control_interface = SCattiVelocity;
    result = scatti_calculate(otg, inp, traj);
    CHECK(result == SCattiWorking, "per-dof global vel");
    CHECK_APPROX(scatti_trajectory_get_duration(traj), 1.095445115, "per-dof global vel dur");
    scatti_trajectory_at_time_simple(traj, 1.0, pos, vel, acc);
    CHECK_APPROX(pos[1], -1.8641718534, "per-dof global vel pos[1]");

    /* Per-DOF CI: Position/Velocity/Position */
    set_per_dof_ci3(inp, SCattiPosition, SCattiVelocity, SCattiPosition);
    result = scatti_calculate(otg, inp, traj);
    CHECK(result == SCattiWorking, "per-dof mixed CI");
    CHECK_APPROX(scatti_trajectory_get_duration(traj), 4.0, "per-dof mixed CI dur");
    scatti_trajectory_at_time_simple(traj, 2.0, pos, vel, acc);
    CHECK_APPROX(pos[0], 0.5, "per-dof mixed CI pos[0]");
    CHECK_APPROX(pos[1], -1.8528486838, "per-dof mixed CI pos[1]");
    CHECK_APPROX(pos[2], 1.0, "per-dof mixed CI pos[2]");

    /* Per-DOF sync: Time/None/Time */
    set_per_dof_sync3(inp, SCattiSyncTime, SCattiSyncNone, SCattiSyncTime);
    result = scatti_calculate(otg, inp, traj);
    CHECK(result == SCattiWorking, "per-dof T/N/T");
    CHECK_APPROX(scatti_trajectory_get_duration(traj), 4.0, "per-dof T/N/T dur");
    scatti_trajectory_at_time_simple(traj, 2.0, pos, vel, acc);
    CHECK_APPROX(pos[0], 0.5, "per-dof T/N/T pos[0]");
    CHECK_APPROX(pos[1], -1.5643167673, "per-dof T/N/T pos[1]");

    /* Position mode, sync None/Time/Time */
    inp->control_interface = SCattiPosition;
    free(inp->per_dof_control_interface);
    inp->per_dof_control_interface = NULL;
    set_per_dof_sync3(inp, SCattiSyncNone, SCattiSyncTime, SCattiSyncTime);
    result = scatti_calculate(otg, inp, traj);
    CHECK(result == SCattiWorking, "per-dof N/T/T");
    CHECK_APPROX(scatti_trajectory_get_duration(traj), 4.0, "per-dof N/T/T dur");
    scatti_trajectory_at_time_simple(traj, 2.0, pos, vel, acc);
    CHECK_APPROX(pos[0], 0.7482143874, "per-dof N/T/T pos[0]");
    CHECK_APPROX(pos[1], -2.6871268303, "per-dof N/T/T pos[1]");

    /* Independent min durations: DOF 0 (Sync::None) reaches target at its own time */
    double min_dur[3];
    scatti_trajectory_get_independent_min_durations(traj, min_dur);
    scatti_trajectory_at_time_simple(traj, min_dur[0], pos, vel, acc);
    CHECK_APPROX(pos[0], inp->target_position[0], "per-dof N: DOF0 at target");
    scatti_trajectory_at_time_simple(traj, min_dur[1], pos, vel, acc);
    CHECK_APPROX(pos[1], -3.0890156397, "per-dof N: DOF1 at min_dur");
    scatti_trajectory_at_time_simple(traj, min_dur[2], pos, vel, acc);
    CHECK_APPROX(pos[2], inp->target_position[2], "per-dof N: DOF2 at target");

    /* Time/Time/None with high velocities */
    set_arr3(inp->current_position, 0.0, 0.0, 0.0);
    set_arr3(inp->current_velocity, 0.0, 0.0, 0.0);
    set_arr3(inp->current_acceleration, 0.0, 0.0, 0.0);
    set_arr3(inp->target_position, 35.0, 35.0, 35.0);
    set_arr3(inp->target_velocity, 125.0, 125.0, 100.0);
    set_arr3(inp->target_acceleration, 0.0, 0.0, 0.0);
    set_arr3(inp->max_velocity, 125.0, 125.0, 100.0);
    set_arr3(inp->max_acceleration, 2000.0, 2000.0, 2000.0);
    set_arr3(inp->max_jerk, 20000.0, 20000.0, 20000.0);
    set_per_dof_sync3(inp, SCattiSyncTime, SCattiSyncTime, SCattiSyncNone);
    result = scatti_calculate(otg, inp, traj);
    CHECK(result == SCattiWorking, "per-dof T/T/N high vel");
    CHECK_APPROX(scatti_trajectory_get_duration(traj), 0.4207106781, "per-dof T/T/N dur");

    /* None/None/Time */
    set_arr3(inp->current_position, 0.0, -2.0, 0.0);
    set_arr3(inp->current_velocity, 0.0, 0.2, 0.0);
    set_arr3(inp->current_acceleration, 0.0, 0.2, 0.0);
    set_arr3(inp->target_position, 1.0, -3.0, 2.0);
    set_arr3(inp->target_velocity, 0.0, 0.0, 0.2);
    set_arr3(inp->target_acceleration, 0.0, 0.0, -0.1);
    set_arr3(inp->max_velocity, 1.0, 1.0, 1.0);
    set_arr3(inp->max_acceleration, 1.0, 1.0, 1.0);
    set_arr3(inp->max_jerk, 1.0, 1.0, 1.0);
    set_per_dof_sync3(inp, SCattiSyncNone, SCattiSyncNone, SCattiSyncTime);
    result = scatti_calculate(otg, inp, traj);
    CHECK(result == SCattiWorking, "per-dof N/N/T");
    CHECK_APPROX(scatti_trajectory_get_duration(traj), 3.7885667284, "per-dof N/N/T dur");

    /* None/Time/None */
    set_per_dof_sync3(inp, SCattiSyncNone, SCattiSyncTime, SCattiSyncNone);
    result = scatti_calculate(otg, inp, traj);
    CHECK(result == SCattiWorking, "per-dof N/T/N");
    CHECK_APPROX(scatti_trajectory_get_duration(traj), 3.7885667284, "per-dof N/T/N dur");

    /* Combined with enabled {true, false, true} */
    set_enabled3(inp, true, false, true);
    result = scatti_calculate(otg, inp, traj);
    CHECK(result == SCattiWorking, "per-dof with enabled");
    CHECK_APPROX(scatti_trajectory_get_duration(traj), 3.6578610221, "per-dof enabled dur");

    /* Phase/None/Phase */
    set_arr3(inp->current_position, 0.0, 0.0, 0.0);
    set_arr3(inp->current_velocity, 0.2, 0.0, -0.1);
    set_arr3(inp->current_acceleration, 0.0, 0.0, 0.0);
    set_arr3(inp->target_position, 1.0, -0.2, -0.5);
    set_arr3(inp->target_velocity, 0.0, 0.0, 0.0);
    set_arr3(inp->target_acceleration, 0.0, 0.0, 0.0);
    set_arr3(inp->max_velocity, 1.0, 1.0, 1.0);
    set_arr3(inp->max_acceleration, 1.0, 1.0, 1.0);
    set_arr3(inp->max_jerk, 1.0, 1.0, 1.0);
    set_per_dof_sync3(inp, SCattiSyncPhase, SCattiSyncNone, SCattiSyncPhase);
    set_enabled3(inp, true, true, true);
    result = scatti_calculate(otg, inp, traj);
    CHECK(result == SCattiWorking, "per-dof Phase/None/Phase");
    CHECK_APPROX(scatti_trajectory_get_duration(traj), 2.848387279, "Phase/None/Phase dur");
    const SCattiProfile *pp0 = scatti_trajectory_get_profile(traj, 0);
    const SCattiProfile *pp1 = scatti_trajectory_get_profile(traj, 1);
    const SCattiProfile *pp2 = scatti_trajectory_get_profile(traj, 2);
    CHECK(!arrays_approx_equal7(pp0->t, pp1->t), "Phase/None/Phase: p0 != p1");
    CHECK(arrays_approx_equal7(pp0->t, pp2->t), "Phase/None/Phase: p0 == p2");

    scatti_trajectory_destroy(traj);
    scatti_input_destroy(inp);
    scatti_destroy(otg);
}

/* ========================== Zero Limits ========================== */

static void test_zero_limits(void) {
    printf("Test: zero limits...\n");

    SCatti *otg = scatti_create(3, 0.005);
    SCattiInputParameter *inp = scatti_input_create(3);
    SCattiOutputParameter *out = scatti_output_create(3);

    /* Constant velocity coasting */
    set_arr3(inp->current_position, 0.0, -2.0, 0.0);
    set_arr3(inp->current_velocity, 0.2, 0.0, 0.0);
    set_arr3(inp->current_acceleration, 0.0, 0.0, 0.0);
    set_arr3(inp->target_position, 1.0, -3.0, 0.0);
    set_arr3(inp->target_velocity, 0.2, 0.0, 0.0);
    set_arr3(inp->target_acceleration, 0.0, 0.0, 0.0);
    set_arr3(inp->max_velocity, 1.0, 1.0, 1.0);
    set_arr3(inp->max_acceleration, 0.0, 1.0, 0.0);
    set_arr3(inp->max_jerk, 0.0, 1.0, 0.0);

    SCattiResult result = scatti_update(otg, inp, out);
    CHECK(result == SCattiWorking, "zero limits coasting");
    CHECK_APPROX(scatti_trajectory_get_duration(out->trajectory), 5.0, "zero limits dur=5.0");

    /* Zero jerk conflict in step 1 */
    set_arr3(inp->current_position, 0.0, -2.0, 0.0);
    set_arr3(inp->current_velocity, -0.2, 0.0, 0.0);
    set_arr3(inp->current_acceleration, 1.0, 0.0, 0.0);
    set_arr3(inp->target_position, 0.4, -3.0, 0.0);
    set_arr3(inp->target_velocity, 0.8, 0.0, 0.0);
    set_arr3(inp->target_acceleration, 1.0, 0.0, 0.0);
    set_arr3(inp->max_velocity, 1.0, 200.0, 1.0);
    set_arr3(inp->max_acceleration, 1.0, 200.0, 0.0);
    set_arr3(inp->max_jerk, 0.0, 200.0, 0.0);
    scatti_reset(otg);
    result = scatti_update(otg, inp, out);
    CHECK(result == SCattiErrorZeroLimits, "zero limits conflict step 1");

    /* Zero jerk conflict with other DOFs */
    set_arr3(inp->target_position, 0.3, -3.0, 0.0);
    set_arr3(inp->max_velocity, 1.0, 2.0, 1.0);
    set_arr3(inp->max_acceleration, 1.0, 2.0, 0.0);
    set_arr3(inp->max_jerk, 0.0, 2.0, 0.0);
    scatti_reset(otg);
    result = scatti_update(otg, inp, out);
    CHECK(result == SCattiErrorZeroLimits, "zero limits conflict other");

    /* Velocity interface: zero jerk conflict */
    inp->control_interface = SCattiVelocity;
    set_arr3(inp->current_position, 0.0, -2.0, 0.0);
    set_arr3(inp->current_velocity, -0.2, 0.0, 0.0);
    set_arr3(inp->current_acceleration, 1.0, 0.0, 0.2);
    set_arr3(inp->target_position, 0.4, -3.0, 0.0);
    set_arr3(inp->target_velocity, 0.9, 0.5, 0.4);
    set_arr3(inp->target_acceleration, 1.0, 0.0, 0.2);
    set_arr3(inp->max_velocity, 1.0, 2.0, 1.0);
    set_arr3(inp->max_acceleration, 1.0, 2.0, 6.0);
    set_arr3(inp->max_jerk, 0.0, 2.0, 0.0);
    scatti_reset(otg);
    result = scatti_update(otg, inp, out);
    CHECK(result == SCattiErrorZeroLimits, "zero limits vel conflict");

    /* Partial zero jerk: {1, 2, 0} */
    set_arr3(inp->max_jerk, 1.0, 2.0, 0.0);
    scatti_reset(otg);
    result = scatti_update(otg, inp, out);
    CHECK(result == SCattiWorking, "zero limits vel partial (1,2,0)");
    CHECK_APPROX(scatti_trajectory_get_duration(out->trajectory), 2.0, "zero limits vel dur=2.0");

    /* Different pattern: {0, 2, 20} */
    set_arr3(inp->max_jerk, 0.0, 2.0, 20.0);
    scatti_reset(otg);
    result = scatti_update(otg, inp, out);
    CHECK(result == SCattiWorking, "zero limits vel partial (0,2,20)");
    CHECK_APPROX(scatti_trajectory_get_duration(out->trajectory), 1.1, "zero limits vel dur=1.1");

    inp->control_interface = SCattiPosition;
    scatti_output_destroy(out);
    scatti_input_destroy(inp);
    scatti_destroy(otg);
}

/* ========================== Pro Feature Tests ========================== */

static void test_intermediate_waypoints(void) {
    printf("Test: intermediate waypoints...\n");
    const size_t dofs = 3;

    SCatti *otg = scatti_create_waypoints(dofs, 0.01, 10);
    SCattiInputParameter *inp = scatti_input_create(dofs);
    SCattiOutputParameter *out = scatti_output_create(dofs);

    /* Start at origin */
    for (size_t d = 0; d < dofs; ++d) {
        inp->current_position[d] = 0.0;
        inp->current_velocity[d] = 0.0;
        inp->current_acceleration[d] = 0.0;
        inp->max_velocity[d] = 3.0;
        inp->max_acceleration[d] = 3.0;
        inp->max_jerk[d] = 4.0;
    }

    /* Single intermediate waypoint at (1, 1, 1), final target (2, 2, 2) */
    double waypoints[] = {1.0, 1.0, 1.0};
    scatti_input_set_intermediate_positions(inp, waypoints, 1);

    inp->target_position[0] = 2.0;
    inp->target_position[1] = 2.0;
    inp->target_position[2] = 2.0;
    inp->target_velocity[0] = 0.0;
    inp->target_velocity[1] = 0.0;
    inp->target_velocity[2] = 0.0;
    inp->target_acceleration[0] = 0.0;
    inp->target_acceleration[1] = 0.0;
    inp->target_acceleration[2] = 0.0;

    /* Run update loop */
    SCattiResult result;
    int steps = 0;
    bool section_changed = false;
    while ((result = scatti_update(otg, inp, out)) == SCattiWorking) {
        steps++;
        if (out->did_section_change) section_changed = true;
        scatti_output_pass_to_input(out, inp);
        if (steps > 100000) break; /* safety */
    }

    CHECK(result == SCattiFinished, "waypoints: trajectory finished");
    CHECK(section_changed, "waypoints: section change detected");
    CHECK(out->trajectory->num_sections == 2, "waypoints: 2 sections");

    /* Sample trajectory at t=duration: verify final position matches target (2,2,2) */
    {
        double duration = scatti_trajectory_get_duration(out->trajectory);
        double pos[3], vel[3], acc[3];

        scatti_trajectory_at_time_simple(out->trajectory, duration, pos, vel, acc);
        CHECK_APPROX(pos[0], 2.0, "waypoints: final pos[0]==2.0");
        CHECK_APPROX(pos[1], 2.0, "waypoints: final pos[1]==2.0");
        CHECK_APPROX(pos[2], 2.0, "waypoints: final pos[2]==2.0");

        /* Sample trajectory at t=0: verify initial position matches (0,0,0) */
        scatti_trajectory_at_time_simple(out->trajectory, 0.0, pos, vel, acc);
        CHECK_APPROX(pos[0], 0.0, "waypoints: init pos[0]==0.0");
        CHECK_APPROX(pos[1], 0.0, "waypoints: init pos[1]==0.0");
        CHECK_APPROX(pos[2], 0.0, "waypoints: init pos[2]==0.0");

        /* Verify waypoint (1.0) is reached at some time > 0 for DOF 0 */
        double wp_time = 0.0;
        bool found = scatti_trajectory_get_first_time_at_position(out->trajectory, 0, 1.0, &wp_time, 0.0);
        CHECK(found, "waypoints: waypoint pos 1.0 found for DOF 0");
        CHECK(wp_time > 0.0, "waypoints: waypoint time > 0");
    }

    scatti_output_destroy(out);
    scatti_input_destroy(inp);
    scatti_destroy(otg);
}

static void test_multiple_waypoints(void) {
    printf("Test: multiple waypoints...\n");
    const size_t dofs = 1;

    SCatti *otg = scatti_create_waypoints(dofs, 0.01, 10);
    SCattiInputParameter *inp = scatti_input_create(dofs);
    SCattiOutputParameter *out = scatti_output_create(dofs);

    inp->current_position[0] = 0.0;
    inp->max_velocity[0] = 5.0;
    inp->max_acceleration[0] = 5.0;
    inp->max_jerk[0] = 10.0;

    /* 3 intermediate waypoints: 1.0, 3.0, 2.0, final target 5.0 */
    double waypoints[] = {1.0, 3.0, 2.0};
    scatti_input_set_intermediate_positions(inp, waypoints, 3);

    inp->target_position[0] = 5.0;

    SCattiResult result;
    int steps = 0;
    int section_changes = 0;
    size_t last_section = 0;
    while ((result = scatti_update(otg, inp, out)) == SCattiWorking) {
        steps++;
        if (out->new_section > last_section) {
            section_changes++;
            last_section = out->new_section;
        }
        scatti_output_pass_to_input(out, inp);
        if (steps > 100000) break;
    }

    CHECK(result == SCattiFinished, "multi-wp: finished");
    CHECK(out->trajectory->num_sections == 4, "multi-wp: 4 sections");
    CHECK(section_changes >= 3, "multi-wp: at least 3 section transitions");

    /* Sample trajectory at t=duration: verify final position is 5.0 */
    {
        double duration = scatti_trajectory_get_duration(out->trajectory);
        double pos[1], vel[1], acc[1];

        scatti_trajectory_at_time_simple(out->trajectory, duration, pos, vel, acc);
        CHECK_APPROX(pos[0], 5.0, "multi-wp: final pos==5.0");

        /* Sample trajectory at t=0: verify initial position is 0.0 */
        scatti_trajectory_at_time_simple(out->trajectory, 0.0, pos, vel, acc);
        CHECK_APPROX(pos[0], 0.0, "multi-wp: init pos==0.0");

        /* Verify each intermediate waypoint is reached */
        double wp_time = 0.0;
        bool found;

        found = scatti_trajectory_get_first_time_at_position(out->trajectory, 0, 1.0, &wp_time, 0.0);
        CHECK(found, "multi-wp: waypoint 1.0 reached");

        found = scatti_trajectory_get_first_time_at_position(out->trajectory, 0, 3.0, &wp_time, 0.0);
        CHECK(found, "multi-wp: waypoint 3.0 reached");

        found = scatti_trajectory_get_first_time_at_position(out->trajectory, 0, 2.0, &wp_time, 0.0);
        CHECK(found, "multi-wp: waypoint 2.0 reached");
    }

    scatti_output_destroy(out);
    scatti_input_destroy(inp);
    scatti_destroy(otg);
}

static void test_per_section_constraints(void) {
    printf("Test: per-section constraints...\n");
    const size_t dofs = 1;

    SCatti *otg = scatti_create_waypoints(dofs, 0.01, 10);
    SCattiInputParameter *inp = scatti_input_create(dofs);
    SCattiOutputParameter *out = scatti_output_create(dofs);

    inp->current_position[0] = 0.0;
    inp->max_velocity[0] = 5.0;
    inp->max_acceleration[0] = 5.0;
    inp->max_jerk[0] = 10.0;

    /* One waypoint at 2.0, target at 5.0 */
    double wp[] = {2.0};
    scatti_input_set_intermediate_positions(inp, wp, 1);
    inp->target_position[0] = 5.0;

    /* Per-section velocity limits: section 0 slow, section 1 fast */
    inp->per_section_max_velocity = (double*)malloc(2 * sizeof(double));
    inp->per_section_max_velocity[0] = 1.0; /* slow first segment */
    inp->per_section_max_velocity[1] = 5.0; /* fast second segment */

    SCattiResult result;
    int steps = 0;
    while ((result = scatti_update(otg, inp, out)) == SCattiWorking) {
        steps++;
        scatti_output_pass_to_input(out, inp);
        if (steps > 100000) break;
    }

    CHECK(result == SCattiFinished, "per-section: finished");
    CHECK(out->trajectory->num_sections == 2, "per-section: 2 sections");

    /* The first section should take longer due to lower velocity limit */
    double sec1_dur = out->trajectory->cumulative_times[0];
    double sec2_dur = out->trajectory->cumulative_times[1] - sec1_dur;
    /* Section 1 covers 2.0 units at max 1.0 m/s, section 2 covers 3.0 units at max 5.0 m/s */
    CHECK(sec1_dur > sec2_dur, "per-section: slow section takes longer");

    /* Verify initial and final positions */
    {
        double pos[1], vel[1], acc[1];
        double duration = scatti_trajectory_get_duration(out->trajectory);

        scatti_trajectory_at_time_simple(out->trajectory, 0.0, pos, vel, acc);
        CHECK_APPROX(pos[0], 0.0, "per-section: init pos==0.0");

        scatti_trajectory_at_time_simple(out->trajectory, duration, pos, vel, acc);
        CHECK_APPROX(pos[0], 5.0, "per-section: final pos==5.0");

        /* Verify velocity at midpoint of section 1 respects per-section constraint (max_vel=1.0) */
        scatti_trajectory_at_time_simple(out->trajectory, sec1_dur / 2.0, pos, vel, acc);
        CHECK(fabs(vel[0]) <= 1.0 + 0.01, "per-section: sec1 mid vel <= 1.0 + tol");

        /* Verify velocity at midpoint of section 2 respects per-section constraint (max_vel=5.0) */
        scatti_trajectory_at_time_simple(out->trajectory, sec1_dur + sec2_dur / 2.0, pos, vel, acc);
        CHECK(fabs(vel[0]) <= 5.0 + 0.01, "per-section: sec2 mid vel <= 5.0 + tol");
    }

    scatti_output_destroy(out);
    scatti_input_destroy(inp);
    scatti_destroy(otg);
}

static void test_position_limits(void) {
    printf("Test: position limits...\n");
    const size_t dofs = 1;

    SCatti *otg = scatti_create_waypoints(dofs, 0.01, 10);
    SCattiInputParameter *inp = scatti_input_create(dofs);
    SCattiOutputParameter *out = scatti_output_create(dofs);

    inp->current_position[0] = 0.0;
    inp->max_velocity[0] = 5.0;
    inp->max_acceleration[0] = 5.0;
    inp->max_jerk[0] = 10.0;
    inp->target_position[0] = 10.0;

    /* Set position limits that the trajectory must respect */
    inp->max_position = (double*)malloc(sizeof(double));
    inp->min_position = (double*)malloc(sizeof(double));
    inp->max_position[0] = 10.5;
    inp->min_position[0] = -0.5;

    /* This should succeed - trajectory stays within bounds */
    SCattiResult result;
    int steps = 0;
    while ((result = scatti_update(otg, inp, out)) == SCattiWorking) {
        steps++;
        scatti_output_pass_to_input(out, inp);
        if (steps > 100000) break;
    }
    CHECK(result == SCattiFinished, "pos-limits: valid trajectory finishes");

    /* Sample trajectory at 100 evenly-spaced points and verify position bounds */
    {
        double duration = scatti_trajectory_get_duration(out->trajectory);
        double pos[1], vel[1], acc[1];
        bool all_within_bounds = true;
        for (int i = 0; i <= 100; i++) {
            double t = duration * (double)i / 100.0;
            scatti_trajectory_at_time_simple(out->trajectory, t, pos, vel, acc);
            if (pos[0] < -0.5 - 0.01 || pos[0] > 10.5 + 0.01) {
                all_within_bounds = false;
                break;
            }
        }
        CHECK(all_within_bounds, "pos-limits: all sampled positions within bounds");

        /* Verify final position */
        scatti_trajectory_at_time_simple(out->trajectory, duration, pos, vel, acc);
        CHECK_APPROX(pos[0], 10.0, "pos-limits: final pos==10.0");

        /* Verify position extrema are within bounds */
        scatti_trajectory_get_position_extrema(out->trajectory);
        CHECK(out->trajectory->position_extrema[0].min >= -0.5 - 0.01,
              "pos-limits: extrema min >= -0.5 - tol");
        CHECK(out->trajectory->position_extrema[0].max <= 10.5 + 0.01,
              "pos-limits: extrema max <= 10.5 + tol");
    }

    scatti_output_destroy(out);
    scatti_input_destroy(inp);
    scatti_destroy(otg);
}

static void test_calculate_offline_waypoints(void) {
    printf("Test: offline waypoint calculation...\n");
    const size_t dofs = 2;

    SCatti *otg = scatti_create_waypoints(dofs, 0.01, 10);
    SCattiInputParameter *inp = scatti_input_create(dofs);

    for (size_t d = 0; d < dofs; ++d) {
        inp->current_position[d] = 0.0;
        inp->max_velocity[d] = 3.0;
        inp->max_acceleration[d] = 3.0;
        inp->max_jerk[d] = 4.0;
    }

    /* Two waypoints */
    double wps[] = {1.0, 1.0, 2.0, 2.0};
    scatti_input_set_intermediate_positions(inp, wps, 2);

    inp->target_position[0] = 3.0;
    inp->target_position[1] = 3.0;

    SCattiTrajectory *traj = scatti_trajectory_create(dofs);
    scatti_trajectory_resize(traj, 3);

    SCattiResult result = scatti_calculate(otg, inp, traj);
    CHECK(result == SCattiWorking, "offline-wp: calculation succeeds");
    CHECK(traj->num_sections == 3, "offline-wp: 3 sections");
    CHECK(traj->duration > 0.0, "offline-wp: positive duration");

    /* Sample at midpoint */
    double pos[2], vel[2], acc[2];
    scatti_trajectory_at_time_simple(traj, traj->duration / 2.0, pos, vel, acc);
    CHECK(pos[0] > 0.0 && pos[0] < 3.5, "offline-wp: midpoint position reasonable");

    /* Verify initial position matches (0,0) */
    scatti_trajectory_at_time_simple(traj, 0.0, pos, vel, acc);
    CHECK_APPROX(pos[0], 0.0, "offline-wp: init pos[0]==0.0");
    CHECK_APPROX(pos[1], 0.0, "offline-wp: init pos[1]==0.0");

    /* Verify final position matches (3,3) */
    scatti_trajectory_at_time_simple(traj, traj->duration, pos, vel, acc);
    CHECK_APPROX(pos[0], 3.0, "offline-wp: final pos[0]==3.0");
    CHECK_APPROX(pos[1], 3.0, "offline-wp: final pos[1]==3.0");

    /* Verify each section has positive duration (cumulative_times strictly increasing) */
    CHECK(traj->cumulative_times[0] > 0.0, "offline-wp: section 0 has positive duration");
    for (size_t i = 1; i < traj->num_sections; i++) {
        CHECK(traj->cumulative_times[i] > traj->cumulative_times[i - 1],
              "offline-wp: cumulative times strictly increasing");
    }

    scatti_trajectory_destroy(traj);
    scatti_input_destroy(inp);
    scatti_destroy(otg);
}

static void test_backward_compat_no_waypoints(void) {
    printf("Test: backward compat (no waypoints with waypoint-enabled instance)...\n");
    const size_t dofs = 1;

    /* Create with waypoint support but don't use waypoints */
    SCatti *otg = scatti_create_waypoints(dofs, 0.01, 10);
    SCattiInputParameter *inp = scatti_input_create(dofs);
    SCattiOutputParameter *out = scatti_output_create(dofs);

    inp->current_position[0] = 0.0;
    inp->target_position[0] = 5.0;
    inp->max_velocity[0] = 3.0;
    inp->max_acceleration[0] = 3.0;
    inp->max_jerk[0] = 4.0;

    SCattiResult result;
    int steps = 0;
    while ((result = scatti_update(otg, inp, out)) == SCattiWorking) {
        steps++;
        scatti_output_pass_to_input(out, inp);
        if (steps > 100000) break;
    }

    CHECK(result == SCattiFinished, "compat: finished without waypoints");
    CHECK(out->trajectory->num_sections == 1, "compat: single section");

    /* Verify final position and positive duration */
    {
        double duration = scatti_trajectory_get_duration(out->trajectory);
        CHECK(duration > 0.0, "compat: trajectory duration > 0");

        double pos[1], vel[1], acc[1];
        scatti_trajectory_at_time_simple(out->trajectory, duration, pos, vel, acc);
        CHECK_APPROX(pos[0], 5.0, "compat: final pos==5.0");
    }

    scatti_output_destroy(out);
    scatti_input_destroy(inp);
    scatti_destroy(otg);
}

/* ========================== Main ========================== */

int main(void) {
    printf("=== scatti Test Suite ===\n\n");

    test_trajectory_basic();
    test_single_dof();
    test_velocity_interface();
    test_minimum_duration();
    test_high_speed();
    test_step_through();
    test_second_order();
    test_first_order();
    test_error_and_new_calculation();
    test_input_validation();
    test_enabled();
    test_phase_synchronization();
    test_discretization();
    test_per_dof_setting();
    test_zero_limits();

    /* Pro feature tests */
    test_intermediate_waypoints();
    test_multiple_waypoints();
    test_per_section_constraints();
    test_position_limits();
    test_calculate_offline_waypoints();
    test_backward_compat_no_waypoints();

    printf("\n=== Results: %d/%d passed, %d failed ===\n",
           tests_passed, tests_run, tests_failed);

    return tests_failed > 0 ? 1 : 0;
}
