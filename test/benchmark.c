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

/* Simple xorshift64 PRNG (same seed as C++ benchmark for comparability) */
static uint64_t rng_states[3] = {42, 43, 44};

static double rand_uniform_s(int s, double lo, double hi) {
    rng_states[s] ^= rng_states[s] << 13;
    rng_states[s] ^= rng_states[s] >> 7;
    rng_states[s] ^= rng_states[s] << 17;
    double u = (double)(rng_states[s] & 0xFFFFFFFFULL) / 4294967296.0;
    return lo + u * (hi - lo);
}

static double rand_normal_s(int s, double mean, double stddev) {
    double u1 = rand_uniform_s(s, 1e-10, 1.0);
    double u2 = rand_uniform_s(s, 0.0, 1.0);
    return mean + stddev * sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

static void fill_position(double *arr, size_t n) {
    for (size_t i = 0; i < n; i++) arr[i] = rand_normal_s(0, 0.0, 4.0);
}
static void fill_dynamic_or_zero(double *arr, size_t n, double prob) {
    for (size_t i = 0; i < n; i++)
        arr[i] = (rand_uniform_s(1, 0, 1) < prob) ? rand_normal_s(1, 0.0, 0.8) : 0.0;
}
static void fill_limit(double *arr, size_t n) {
    for (size_t i = 0; i < n; i++) arr[i] = rand_uniform_s(2, 0.1, 12.0);
}
static void fill_limit_offset(double *arr, size_t n, const double *offset) {
    for (size_t i = 0; i < n; i++) arr[i] = fabs(offset[i]) + rand_uniform_s(2, 0.1, 12.0);
}

static double get_time_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1e6 + ts.tv_nsec / 1000.0;
}

int main(int argc, char **argv) {
    size_t dofs = 7;
    size_t n_iters = 20;
    size_t n_traj = 32768;

    if (argc > 1) dofs = (size_t)atoi(argv[1]);
    if (argc > 2) n_traj = (size_t)atoi(argv[2]);
    if (argc > 3) n_iters = (size_t)atoi(argv[3]);

    printf("=== scatti Benchmark ===\n");
    printf("DOFs: %zu, Trajectories per iteration: %zu, Iterations: %zu\n\n", dofs, n_traj, n_iters);

    CRuckig *otg = scatti_create(dofs, 0.005);
    CRuckigInputParameter *inp = scatti_input_create(dofs);
    CRuckigTrajectory *traj = scatti_trajectory_create(dofs);

    double *avg_arr = calloc(n_iters, sizeof(double));
    double *worst_arr = calloc(n_iters, sizeof(double));
    double *global_arr = calloc(n_iters, sizeof(double));

    for (size_t iter = 0; iter < n_iters; iter++) {
        /* Reset PRNG per iteration for consistency */
        rng_states[0] = 42 + iter * 1000;
        rng_states[1] = 43 + iter * 1000;
        rng_states[2] = 44 + iter * 1000;

        double avg = 0.0;
        double worst = 0.0;
        size_t count = 0;

        double global_start = get_time_us();

        for (size_t i = 0; i < n_traj; i++) {
            fill_position(inp->current_position, dofs);
            fill_dynamic_or_zero(inp->current_velocity, dofs, 0.9);
            fill_dynamic_or_zero(inp->current_acceleration, dofs, 0.8);
            fill_position(inp->target_position, dofs);
            fill_dynamic_or_zero(inp->target_velocity, dofs, 0.7);
            fill_dynamic_or_zero(inp->target_acceleration, dofs, 0.6);
            fill_limit_offset(inp->max_velocity, dofs, inp->target_velocity);
            fill_limit_offset(inp->max_acceleration, dofs, inp->target_acceleration);
            fill_limit(inp->max_jerk, dofs);

            if (!scatti_input_validate(inp, false, true)) continue;

            double t0 = get_time_us();
            scatti_calculate(otg, inp, traj);
            double t1 = get_time_us();

            double elapsed = t1 - t0;
            count++;
            avg += (elapsed - avg) / count;
            if (elapsed > worst) worst = elapsed;
        }

        double global_end = get_time_us();
        double global_per = (global_end - global_start) / n_traj;

        avg_arr[iter] = avg;
        worst_arr[iter] = worst;
        global_arr[iter] = global_per;
    }

    /* Compute mean and std */
    double avg_mean = 0, worst_mean = 0, global_mean = 0;
    for (size_t i = 0; i < n_iters; i++) {
        avg_mean += avg_arr[i];
        worst_mean += worst_arr[i];
        global_mean += global_arr[i];
    }
    avg_mean /= n_iters;
    worst_mean /= n_iters;
    global_mean /= n_iters;

    double avg_std = 0, worst_std = 0, global_std = 0;
    for (size_t i = 0; i < n_iters; i++) {
        avg_std += (avg_arr[i] - avg_mean) * (avg_arr[i] - avg_mean);
        worst_std += (worst_arr[i] - worst_mean) * (worst_arr[i] - worst_mean);
        global_std += (global_arr[i] - global_mean) * (global_arr[i] - global_mean);
    }
    avg_std = sqrt(avg_std / n_iters);
    worst_std = sqrt(worst_std / n_iters);
    global_std = sqrt(global_std / n_iters);

    printf("--- scatti (C) Results ---\n");
    printf("Average Calculation Duration  %.3f +/- %.3f [us]\n", avg_mean, avg_std);
    printf("Worst Calculation Duration    %.3f +/- %.3f [us]\n", worst_mean, worst_std);
    printf("End-to-end Duration           %.3f +/- %.3f [us]\n", global_mean, global_std);

    free(avg_arr);
    free(worst_arr);
    free(global_arr);
    scatti_trajectory_destroy(traj);
    scatti_input_destroy(inp);
    scatti_destroy(otg);

    return 0;
}
