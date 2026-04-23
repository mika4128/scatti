#ifndef SCATTI_TRAJECTORY_H
#define SCATTI_TRAJECTORY_H

#include <stdbool.h>
#include <stddef.h>
#include <scatti/profile.h>

typedef struct {
    size_t degrees_of_freedom;

    /* Multi-section support: profiles[section * dofs + dof] */
    CRuckigProfile *profiles;       /* Array of num_sections * dofs profiles */
    size_t num_sections;             /* Number of sections (1 for state-to-state) */
    size_t section_capacity;         /* Allocated capacity for sections */
    double duration;
    double *cumulative_times;        /* Array of num_sections cumulative durations */

    double *independent_min_durations; /* Array of dofs */
    CRuckigBound *position_extrema;    /* Array of dofs */
} CRuckigTrajectory;

/* Create trajectory for single-section (backward compatible) */
CRuckigTrajectory* scatti_trajectory_create(size_t dofs);
void scatti_trajectory_destroy(CRuckigTrajectory *traj);

/* Resize trajectory for multi-section (num_sections = max_waypoints + 1) */
bool scatti_trajectory_resize(CRuckigTrajectory *traj, size_t num_sections);

/* Query trajectory state at time */
void scatti_trajectory_at_time(const CRuckigTrajectory *traj, double time,
                                double *new_position, double *new_velocity,
                                double *new_acceleration, double *new_jerk,
                                size_t *new_section);

/* Simplified version without jerk/section */
void scatti_trajectory_at_time_simple(const CRuckigTrajectory *traj, double time,
                                       double *new_position, double *new_velocity,
                                       double *new_acceleration);

double scatti_trajectory_get_duration(const CRuckigTrajectory *traj);

/* Get intermediate durations (cumulative times array). Returns num_sections. */
size_t scatti_trajectory_get_intermediate_durations(const CRuckigTrajectory *traj,
                                                     double *out_durations);

/* Get position extrema for all DOFs */
void scatti_trajectory_get_position_extrema(CRuckigTrajectory *traj);

/* Get first time at position for a DOF. Returns true if found. */
bool scatti_trajectory_get_first_time_at_position(const CRuckigTrajectory *traj,
                                                   size_t dof, double position,
                                                   double *time, double time_after);

/* Get independent minimum durations (one per DOF). Caller provides array of dofs. */
void scatti_trajectory_get_independent_min_durations(const CRuckigTrajectory *traj,
                                                      double *out_durations);

/* Get the underlying profile for a specific DOF in a section (read-only). */
const CRuckigProfile* scatti_trajectory_get_profile(const CRuckigTrajectory *traj, size_t dof);

/* Get profile for specific section and DOF. */
const CRuckigProfile* scatti_trajectory_get_section_profile(const CRuckigTrajectory *traj,
                                                              size_t section, size_t dof);

#endif /* SCATTI_TRAJECTORY_H */
