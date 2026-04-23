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
    CRuckigWorking = 0,
    CRuckigFinished = 1,
    CRuckigError = -1,
    CRuckigErrorInvalidInput = -100,
    CRuckigErrorTrajectoryDuration = -101,
    CRuckigErrorPositionalLimits = -102,
    CRuckigErrorZeroLimits = -104,
    CRuckigErrorExecutionTimeCalculation = -110,
    CRuckigErrorSynchronizationCalculation = -111
} CRuckigResult;

typedef enum {
    CRuckigPosition = 0,
    CRuckigVelocity = 1
} CRuckigControlInterface;

typedef enum {
    CRuckigSyncTime = 0,
    CRuckigSyncTimeIfNecessary = 1,
    CRuckigSyncPhase = 2,
    CRuckigSyncNone = 3
} CRuckigSynchronization;

typedef enum {
    CRuckigContinuous = 0,
    CRuckigDiscrete = 1
} CRuckigDurationDiscretization;

/* ---- opaque forward declarations ---- */
typedef struct CRuckig CRuckig;
typedef struct CRuckigTrajectory CRuckigTrajectory;
typedef struct CRuckigProfile CRuckigProfile;

/* ---- structs ---- */
typedef struct {
    double min;
    double max;
    double t_min;
    double t_max;
} CRuckigBound;

typedef struct {
    size_t degrees_of_freedom;

    CRuckigControlInterface control_interface;
    CRuckigSynchronization synchronization;
    CRuckigDurationDiscretization duration_discretization;

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
    CRuckigControlInterface *per_dof_control_interface;
    CRuckigSynchronization  *per_dof_synchronization;
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
} CRuckigInputParameter;

typedef struct {
    size_t degrees_of_freedom;
    CRuckigTrajectory *trajectory;
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
} CRuckigOutputParameter;

/* ---- main API ---- */
CRuckig* scatti_create(size_t dofs, double delta_time);
CRuckig* scatti_create_waypoints(size_t dofs, double delta_time, size_t max_waypoints);
void     scatti_destroy(CRuckig *r);
void     scatti_reset(CRuckig *r);
bool     scatti_validate_input(const CRuckig *r,
                                const CRuckigInputParameter *input,
                                bool check_current_within_limits,
                                bool check_target_within_limits);
CRuckigResult scatti_calculate(CRuckig *r,
                                const CRuckigInputParameter *input,
                                CRuckigTrajectory *trajectory);
CRuckigResult scatti_update(CRuckig *r,
                             const CRuckigInputParameter *input,
                             CRuckigOutputParameter *output);

/* ---- input ---- */
CRuckigInputParameter* scatti_input_create(size_t dofs);
void scatti_input_destroy(CRuckigInputParameter *inp);
bool scatti_input_validate(const CRuckigInputParameter *inp,
                            bool check_current_within_limits,
                            bool check_target_within_limits);
void scatti_input_copy(CRuckigInputParameter *dst,
                        const CRuckigInputParameter *src);
void scatti_input_set_intermediate_positions(CRuckigInputParameter *inp,
                                               const double *positions,
                                               size_t num_waypoints);

/* ---- output ---- */
CRuckigOutputParameter* scatti_output_create(size_t dofs);
void scatti_output_destroy(CRuckigOutputParameter *out);
void scatti_output_pass_to_input(const CRuckigOutputParameter *out,
                                  CRuckigInputParameter *inp);

/* ---- trajectory ---- */
CRuckigTrajectory* scatti_trajectory_create(size_t dofs);
void scatti_trajectory_destroy(CRuckigTrajectory *traj);
bool scatti_trajectory_resize(CRuckigTrajectory *traj, size_t num_sections);
double scatti_trajectory_get_duration(const CRuckigTrajectory *traj);
size_t scatti_trajectory_get_intermediate_durations(const CRuckigTrajectory *traj,
                                                      double *out_durations);
void scatti_trajectory_at_time(const CRuckigTrajectory *traj, double time,
                                double *new_position, double *new_velocity,
                                double *new_acceleration, double *new_jerk,
                                size_t *new_section);
void scatti_trajectory_get_position_extrema(CRuckigTrajectory *traj);
bool scatti_trajectory_get_first_time_at_position(
        const CRuckigTrajectory *traj, size_t dof,
        double position, double *time, double time_after);
void scatti_trajectory_get_independent_min_durations(
        const CRuckigTrajectory *traj, double *out_durations);
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
