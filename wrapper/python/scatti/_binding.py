"""Low-level cffi binding for scatti shared library."""

import cffi
import ctypes.util
import os
import sys

ffi = cffi.FFI()

# ---- C declarations (ABI mode — parsed at import time) ----
ffi.cdef("""
/* ---- enums ---- */
typedef enum {
    SCattiWorking = 0,
    SCattiFinished = 1,
    SCattiError = -1,
    SCattiErrorInvalidInput = -100,
    SCattiErrorTrajectoryDuration = -101,
    SCattiErrorPositionalLimits = -102,
    SCattiErrorZeroLimits = -104,
    SCattiErrorExecutionTimeCalculation = -110,
    SCattiErrorSynchronizationCalculation = -111
} SCattiResult;

typedef enum {
    SCattiPosition = 0,
    SCattiVelocity = 1
} SCattiControlInterface;

typedef enum {
    SCattiSyncTime = 0,
    SCattiSyncTimeIfNecessary = 1,
    SCattiSyncPhase = 2,
    SCattiSyncNone = 3
} SCattiSynchronization;

typedef enum {
    SCattiContinuous = 0,
    SCattiDiscrete = 1
} SCattiDurationDiscretization;

/* ---- opaque forward declarations ---- */
typedef struct SCatti SCatti;
typedef struct SCattiTrajectory SCattiTrajectory;
typedef struct SCattiProfile SCattiProfile;

/* ---- structs ---- */
typedef struct {
    double min;
    double max;
    double t_min;
    double t_max;
} SCattiBound;

typedef struct {
    size_t degrees_of_freedom;

    SCattiControlInterface control_interface;
    SCattiSynchronization synchronization;
    SCattiDurationDiscretization duration_discretization;

    double *current_position;
    double *current_velocity;
    double *current_acceleration;
    double *target_position;
    double *target_velocity;
    double *target_acceleration;
    double *max_velocity;
    double *max_acceleration;
    double *max_jerk;
    double *min_velocity;
    double *min_acceleration;
    bool   *enabled;
    SCattiControlInterface *per_dof_control_interface;
    SCattiSynchronization  *per_dof_synchronization;
    double minimum_duration;
    bool   has_minimum_duration;

    /* Pro features */
    double *intermediate_positions;
    size_t num_intermediate_waypoints;
    double *per_section_max_velocity;
    double *per_section_max_acceleration;
    double *per_section_max_jerk;
    double *per_section_min_velocity;
    double *per_section_min_acceleration;
    double *per_section_max_position;
    double *per_section_min_position;
    double *max_position;
    double *min_position;
    double *per_section_minimum_duration;
    double interrupt_calculation_duration;
} SCattiInputParameter;

typedef struct {
    size_t degrees_of_freedom;
    SCattiTrajectory *trajectory;
    double *new_position;
    double *new_velocity;
    double *new_acceleration;
    double *new_jerk;
    double time;
    size_t new_section;
    bool   did_section_change;
    bool   new_calculation;
    bool   was_calculation_interrupted;
    double calculation_duration;
} SCattiOutputParameter;

/* ---- main API ---- */
SCatti* scatti_create(size_t dofs, double delta_time);
SCatti* scatti_create_waypoints(size_t dofs, double delta_time, size_t max_waypoints);
void     scatti_destroy(SCatti *r);
void     scatti_reset(SCatti *r);
bool     scatti_validate_input(const SCatti *r,
                                const SCattiInputParameter *input,
                                bool check_current_within_limits,
                                bool check_target_within_limits);
SCattiResult scatti_calculate(SCatti *r,
                                const SCattiInputParameter *input,
                                SCattiTrajectory *trajectory);
SCattiResult scatti_update(SCatti *r,
                             const SCattiInputParameter *input,
                             SCattiOutputParameter *output);

/* ---- input ---- */
SCattiInputParameter* scatti_input_create(size_t dofs);
void scatti_input_destroy(SCattiInputParameter *inp);
bool scatti_input_validate(const SCattiInputParameter *inp,
                            bool check_current_within_limits,
                            bool check_target_within_limits);
void scatti_input_copy(SCattiInputParameter *dst,
                        const SCattiInputParameter *src);
void scatti_input_set_intermediate_positions(SCattiInputParameter *inp,
                                               const double *positions,
                                               size_t num_waypoints);

/* ---- output ---- */
SCattiOutputParameter* scatti_output_create(size_t dofs);
void scatti_output_destroy(SCattiOutputParameter *out);
void scatti_output_pass_to_input(const SCattiOutputParameter *out,
                                  SCattiInputParameter *inp);

/* ---- trajectory ---- */
SCattiTrajectory* scatti_trajectory_create(size_t dofs);
void scatti_trajectory_destroy(SCattiTrajectory *traj);
bool scatti_trajectory_resize(SCattiTrajectory *traj, size_t num_sections);
double scatti_trajectory_get_duration(const SCattiTrajectory *traj);
size_t scatti_trajectory_get_intermediate_durations(const SCattiTrajectory *traj,
                                                      double *out_durations);
void scatti_trajectory_at_time(const SCattiTrajectory *traj, double time,
                                double *new_position, double *new_velocity,
                                double *new_acceleration, double *new_jerk,
                                size_t *new_section);
void scatti_trajectory_get_position_extrema(SCattiTrajectory *traj);
bool scatti_trajectory_get_first_time_at_position(
        const SCattiTrajectory *traj, size_t dof,
        double position, double *time, double time_after);
void scatti_trajectory_get_independent_min_durations(
        const SCattiTrajectory *traj, double *out_durations);
""")


def _find_library():
    """Locate libscatti.so using several strategies."""
    # 1. Explicit environment variable
    path = os.environ.get("SCATTI_LIB")
    if path and os.path.isfile(path):
        return path

    # 2. Relative to this file (../../build/libscatti.so)
    here = os.path.dirname(os.path.abspath(__file__))
    for relpath in [
        os.path.join(here, "..", "..", "..", "build", "libscatti.so"),
        os.path.join(here, "..", "..", "..", "build", "libscatti.dylib"),
    ]:
        candidate = os.path.normpath(relpath)
        if os.path.isfile(candidate):
            return candidate

    # 3. System library path
    found = ctypes.util.find_library("scatti")
    if found:
        return found

    raise OSError(
        "Cannot find libscatti shared library. "
        "Set SCATTI_LIB=/path/to/libscatti.so or install the library system-wide."
    )


lib = ffi.dlopen(_find_library())
