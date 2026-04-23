/*
 * Cross-validation test: compare scatti (C) against Ruckig (C++)
 *
 * Generates random inputs, computes trajectories with both libraries,
 * and verifies that durations and final states match within tolerance.
 *
 * Build:
 *   g++ -O2 -std=c++17 \
 *       -I/path/to/ruckig/include -I/path/to/scatti/include \
 *       test_comparison.cpp \
 *       -L/path/to/ruckig/build -lruckig \
 *       -L/path/to/scatti/build -lscatti \
 *       -lm -o test_comparison
 */

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <algorithm>

#include <ruckig/ruckig.hpp>

extern "C" {
#include <scatti/scatti.h>
}

using namespace ruckig;

/* Tolerances */
static constexpr double DURATION_TOL = 1e-6;   /* Duration must match within 1µs */
static constexpr double POSITION_TOL = 1e-6;
static constexpr double VELOCITY_TOL = 1e-6;
static constexpr double ACCEL_TOL    = 1e-4;

struct Stats {
    size_t total = 0;
    size_t matched = 0;
    size_t duration_mismatch = 0;
    size_t final_state_mismatch = 0;
    size_t result_mismatch = 0;
    size_t skipped = 0;
    double max_duration_diff = 0.0;
    double max_position_diff = 0.0;
    double max_velocity_diff = 0.0;
};

static void compare_one(
    Ruckig<3>& ruckig_cpp,
    CRuckig* ruckig_c,
    InputParameter<3>& inp_cpp,
    CRuckigInputParameter* inp_c,
    Stats& stats)
{
    stats.total++;

    /* Sync C input from C++ input */
    for (int d = 0; d < 3; d++) {
        inp_c->current_position[d]     = inp_cpp.current_position[d];
        inp_c->current_velocity[d]     = inp_cpp.current_velocity[d];
        inp_c->current_acceleration[d] = inp_cpp.current_acceleration[d];
        inp_c->target_position[d]      = inp_cpp.target_position[d];
        inp_c->target_velocity[d]      = inp_cpp.target_velocity[d];
        inp_c->target_acceleration[d]  = inp_cpp.target_acceleration[d];
        inp_c->max_velocity[d]         = inp_cpp.max_velocity[d];
        inp_c->max_acceleration[d]     = inp_cpp.max_acceleration[d];
        inp_c->max_jerk[d]             = inp_cpp.max_jerk[d];
    }

    /* Compute C++ trajectory */
    OutputParameter<3> out_cpp;
    auto result_cpp = ruckig_cpp.update(inp_cpp, out_cpp);
    if (result_cpp == Result::ErrorTrajectoryDuration ||
        result_cpp == Result::ErrorInvalidInput) {
        stats.skipped++;
        return;
    }

    /* Compute C trajectory */
    CRuckigOutputParameter* out_c = scatti_output_create(3);
    scatti_reset(ruckig_c);
    CRuckigResult result_c = scatti_update(ruckig_c, inp_c, out_c);

    /* Compare results */
    int cpp_ok = (result_cpp == Result::Working ||
                  (result_cpp == Result::Finished && out_cpp.trajectory.get_duration() < 0.005));
    int c_ok = (result_c == CRuckigWorking ||
                (result_c == CRuckigFinished && scatti_trajectory_get_duration(out_c->trajectory) < 0.005));

    if (cpp_ok != c_ok) {
        stats.result_mismatch++;
        scatti_output_destroy(out_c);
        return;
    }

    if (!cpp_ok) {
        stats.skipped++;
        scatti_output_destroy(out_c);
        return;
    }

    /* Compare durations */
    double dur_cpp = out_cpp.trajectory.get_duration();
    double dur_c   = scatti_trajectory_get_duration(out_c->trajectory);
    double dur_diff = std::abs(dur_cpp - dur_c);

    stats.max_duration_diff = std::max(stats.max_duration_diff, dur_diff);

    if (dur_diff > DURATION_TOL) {
        stats.duration_mismatch++;
        if (stats.duration_mismatch <= 5) {
            printf("  Duration mismatch: C++=%.10f  C=%.10f  diff=%.2e\n", dur_cpp, dur_c, dur_diff);
            printf("    input: p0=[%.4f,%.4f,%.4f] pf=[%.4f,%.4f,%.4f]\n",
                   inp_cpp.current_position[0], inp_cpp.current_position[1], inp_cpp.current_position[2],
                   inp_cpp.target_position[0], inp_cpp.target_position[1], inp_cpp.target_position[2]);
        }
        scatti_output_destroy(out_c);
        return;
    }

    /* Compare final states */
    std::array<double, 3> pos_cpp, vel_cpp, acc_cpp;
    out_cpp.trajectory.at_time(dur_cpp, pos_cpp, vel_cpp, acc_cpp);

    double pos_c[3], vel_c[3], acc_c[3];
    scatti_trajectory_at_time_simple(out_c->trajectory, dur_c, pos_c, vel_c, acc_c);

    bool state_ok = true;
    for (int d = 0; d < 3; d++) {
        double pd = std::abs(pos_cpp[d] - pos_c[d]);
        double vd = std::abs(vel_cpp[d] - vel_c[d]);
        stats.max_position_diff = std::max(stats.max_position_diff, pd);
        stats.max_velocity_diff = std::max(stats.max_velocity_diff, vd);

        if (pd > POSITION_TOL || vd > VELOCITY_TOL) {
            state_ok = false;
        }
    }

    if (!state_ok) {
        stats.final_state_mismatch++;
        if (stats.final_state_mismatch <= 5) {
            printf("  Final state mismatch at duration=%.6f:\n", dur_cpp);
            for (int d = 0; d < 3; d++) {
                printf("    dof%d: C++ pos=%.10f  C pos=%.10f  diff=%.2e\n",
                       d, pos_cpp[d], pos_c[d], std::abs(pos_cpp[d] - pos_c[d]));
            }
        }
        scatti_output_destroy(out_c);
        return;
    }

    stats.matched++;
    scatti_output_destroy(out_c);
}

int main(int argc, char** argv) {
    size_t num_trajectories = 100000;
    int seed = 42;

    if (argc > 1) num_trajectories = std::atol(argv[1]);
    if (argc > 2) seed = std::atoi(argv[2]);

    printf("=== scatti vs Ruckig Cross-Validation ===\n");
    printf("Trajectories: %zu, Seed: %d\n\n", num_trajectories, seed);

    Ruckig<3> ruckig_cpp{0.005};
    InputParameter<3> inp_cpp;

    CRuckig* ruckig_c = scatti_create(3, 0.005);
    CRuckigInputParameter* inp_c = scatti_input_create(3);

    std::default_random_engine gen(seed);
    std::normal_distribution<double> position_dist(0.0, 4.0);
    std::normal_distribution<double> dynamic_dist(0.0, 0.8);
    std::uniform_real_distribution<double> limit_dist(0.08, 16.0);
    std::uniform_real_distribution<double> zero_one(0.0, 1.0);

    Stats stats;

    for (size_t i = 0; i < num_trajectories; i++) {
        /* Generate random input (same algorithm as both test suites) */
        for (int d = 0; d < 3; d++) {
            inp_cpp.current_position[d]     = position_dist(gen);
            inp_cpp.current_velocity[d]     = (zero_one(gen) < 0.9) ? dynamic_dist(gen) : 0.0;
            inp_cpp.current_acceleration[d] = (zero_one(gen) < 0.8) ? dynamic_dist(gen) : 0.0;
            inp_cpp.target_position[d]      = position_dist(gen);
            inp_cpp.target_velocity[d]      = (zero_one(gen) < 0.7) ? dynamic_dist(gen) : 0.0;
            inp_cpp.target_acceleration[d]  = (zero_one(gen) < 0.6) ? dynamic_dist(gen) : 0.0;
            inp_cpp.max_velocity[d]         = limit_dist(gen) + std::abs(inp_cpp.target_velocity[d]);
            inp_cpp.max_acceleration[d]     = limit_dist(gen) + std::abs(inp_cpp.target_acceleration[d]);
            inp_cpp.max_jerk[d]             = limit_dist(gen);
        }

        /* Skip invalid inputs */
        if (!ruckig_cpp.validate_input<false>(inp_cpp)) {
            continue;
        }

        compare_one(ruckig_cpp, ruckig_c, inp_cpp, inp_c, stats);
    }

    printf("=== Results ===\n");
    printf("Total:              %zu\n", stats.total);
    printf("Matched:            %zu (%.2f%%)\n", stats.matched,
           stats.total > 0 ? 100.0 * stats.matched / stats.total : 0.0);
    printf("Skipped:            %zu\n", stats.skipped);
    printf("Result mismatch:    %zu\n", stats.result_mismatch);
    printf("Duration mismatch:  %zu\n", stats.duration_mismatch);
    printf("Final state mismatch: %zu\n", stats.final_state_mismatch);
    printf("\nMax differences:\n");
    printf("  Duration: %.2e\n", stats.max_duration_diff);
    printf("  Position: %.2e\n", stats.max_position_diff);
    printf("  Velocity: %.2e\n", stats.max_velocity_diff);

    scatti_input_destroy(inp_c);
    scatti_destroy(ruckig_c);

    return (stats.result_mismatch + stats.duration_mismatch + stats.final_state_mismatch) > 0 ? 1 : 0;
}
