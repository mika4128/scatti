#*******************************************************************
# Permission is hereby granted, free of charge, to any person obtaining
# a copy of this software and associated documentation files (the
# "Software"), to deal in the Software without restriction, including
# without limitation the rights to use, copy, modify, merge, publish,
# distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject
# to the following conditions:
# 
# The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
# CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
# TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
# 
# Description: scatti - pure C99 jerk-limited real-time trajectory
#              generation. Port of Ruckig with embed-friendly
#              memory and math hooks.
# Author:      杨阳 (Yang Yang) <mika-net@outlook.com>
# Origin:      Based on Ruckig Community Edition by Lars Berscheid
#              (https://github.com/pantor/ruckig)
# License:     MIT (SPDX-License-Identifier: MIT)
# Copyright (c) 2026 杨阳 (Yang Yang)
# *******************************************************************

"""scatti — Python bindings for the scatti trajectory generation library.

Usage::

    from scatti import Ruckig, InputParameter, OutputParameter, Result

    ruckig = Ruckig(3, 0.01)
    inp = InputParameter(3)
    out = OutputParameter(3)

    inp.current_position = [0.0, 0.0, 0.5]
    inp.target_position = [5.0, -2.0, -3.5]
    inp.max_velocity = [3.0, 1.0, 3.0]
    inp.max_acceleration = [3.0, 2.0, 1.0]
    inp.max_jerk = [4.0, 3.0, 2.0]

    while True:
        res = ruckig.update(inp, out)
        print(out.time, list(out.new_position))
        out.pass_to_input(inp)
        if res == Result.Finished:
            break
        if res != Result.Working:
            raise RuntimeError(res)
"""

from __future__ import annotations

import enum
from copy import copy as _stdlib_copy

from ._binding import ffi, lib

__all__ = [
    "Ruckig",
    "InputParameter",
    "OutputParameter",
    "Trajectory",
    "Result",
    "ControlInterface",
    "Synchronization",
    "DurationDiscretization",
]

# ---------------------------------------------------------------------------
# Enums
# ---------------------------------------------------------------------------

class Result(enum.IntEnum):
    Working = 0
    Finished = 1
    Error = -1
    ErrorInvalidInput = -100
    ErrorTrajectoryDuration = -101
    ErrorPositionalLimits = -102
    ErrorZeroLimits = -104
    ErrorExecutionTimeCalculation = -110
    ErrorSynchronizationCalculation = -111


class ControlInterface(enum.IntEnum):
    Position = 0
    Velocity = 1


class Synchronization(enum.IntEnum):
    Time = 0
    TimeIfNecessary = 1
    Phase = 2
    No = 3


class DurationDiscretization(enum.IntEnum):
    Continuous = 0
    Discrete = 1


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _arr_to_list(ptr, n):
    """Convert a C double* to a Python list."""
    return [ptr[i] for i in range(n)]


def _list_to_arr(ptr, values, n):
    """Copy a Python sequence into a C double*."""
    if len(values) != n:
        raise ValueError(f"Expected {n} values, got {len(values)}")
    for i in range(n):
        ptr[i] = values[i]


def _make_array_property(c_field, doc=None):
    """Create a property that reads/writes a C double* as a Python list."""
    def fget(self):
        return _arr_to_list(getattr(self._ptr, c_field), self._dofs)

    def fset(self, values):
        _list_to_arr(getattr(self._ptr, c_field), values, self._dofs)

    return property(fget, fset, doc=doc)


# ---------------------------------------------------------------------------
# Trajectory
# ---------------------------------------------------------------------------

class Trajectory:
    """Read-only wrapper around a SCattiTrajectory."""

    def __init__(self, ptr, dofs):
        self._ptr = ptr
        self._dofs = dofs

    @property
    def duration(self):
        return lib.scatti_trajectory_get_duration(self._ptr)

    def at_time(self, time):
        """Return (position, velocity, acceleration, jerk) at *time*."""
        pos = ffi.new("double[]", self._dofs)
        vel = ffi.new("double[]", self._dofs)
        acc = ffi.new("double[]", self._dofs)
        jrk = ffi.new("double[]", self._dofs)
        sec = ffi.new("size_t *")
        lib.scatti_trajectory_at_time(self._ptr, time, pos, vel, acc, jrk, sec)
        return (
            _arr_to_list(pos, self._dofs),
            _arr_to_list(vel, self._dofs),
            _arr_to_list(acc, self._dofs),
            _arr_to_list(jrk, self._dofs),
        )

    def get_position_extrema(self):
        """Compute and return position extrema as list of (min, max, t_min, t_max)."""
        lib.scatti_trajectory_get_position_extrema(self._ptr)
        bounds = self._ptr.position_extrema
        result = []
        for i in range(self._dofs):
            b = bounds[i]
            result.append((b.min, b.max, b.t_min, b.t_max))
        return result

    def get_first_time_at_position(self, dof, position, time_after=0.0):
        """Return the first time the trajectory reaches *position* for *dof*, or None."""
        t = ffi.new("double *")
        found = lib.scatti_trajectory_get_first_time_at_position(
            self._ptr, dof, position, t, time_after
        )
        return t[0] if found else None

    def get_independent_min_durations(self):
        """Return per-DOF minimum durations."""
        out = ffi.new("double[]", self._dofs)
        lib.scatti_trajectory_get_independent_min_durations(self._ptr, out)
        return _arr_to_list(out, self._dofs)


# ---------------------------------------------------------------------------
# InputParameter
# ---------------------------------------------------------------------------

class InputParameter:
    """Wraps SCattiInputParameter with Pythonic access."""

    def __init__(self, dofs):
        self._dofs = dofs
        self._ptr = lib.scatti_input_create(dofs)
        if self._ptr == ffi.NULL:
            raise MemoryError("Failed to create SCattiInputParameter")
        self._owned = {}  # field_name -> cdata ref (prevent GC of ffi.new buffers)

    def __del__(self):
        if hasattr(self, "_ptr") and self._ptr != ffi.NULL:
            # NULL out Python-allocated pointers so C's free() doesn't touch them
            for field in getattr(self, "_owned", {}):
                setattr(self._ptr, field, ffi.NULL)
            lib.scatti_input_destroy(self._ptr)
            self._ptr = ffi.NULL

    @property
    def degrees_of_freedom(self):
        return self._dofs

    # --- array properties ---
    current_position = _make_array_property("current_position")
    current_velocity = _make_array_property("current_velocity")
    current_acceleration = _make_array_property("current_acceleration")
    target_position = _make_array_property("target_position")
    target_velocity = _make_array_property("target_velocity")
    target_acceleration = _make_array_property("target_acceleration")
    max_velocity = _make_array_property("max_velocity")
    max_acceleration = _make_array_property("max_acceleration")
    max_jerk = _make_array_property("max_jerk")

    # --- optional array properties ---
    @property
    def min_velocity(self):
        if self._ptr.min_velocity == ffi.NULL:
            return None
        return _arr_to_list(self._ptr.min_velocity, self._dofs)

    @min_velocity.setter
    def min_velocity(self, values):
        if values is None:
            return
        if self._ptr.min_velocity == ffi.NULL:
            buf = ffi.new("double[]", self._dofs)
            self._ptr.min_velocity = buf
            self._owned['min_velocity'] = buf
        _list_to_arr(self._ptr.min_velocity, values, self._dofs)

    @property
    def min_acceleration(self):
        if self._ptr.min_acceleration == ffi.NULL:
            return None
        return _arr_to_list(self._ptr.min_acceleration, self._dofs)

    @min_acceleration.setter
    def min_acceleration(self, values):
        if values is None:
            return
        if self._ptr.min_acceleration == ffi.NULL:
            buf = ffi.new("double[]", self._dofs)
            self._ptr.min_acceleration = buf
            self._owned['min_acceleration'] = buf
        _list_to_arr(self._ptr.min_acceleration, values, self._dofs)

    @property
    def enabled(self):
        if self._ptr.enabled == ffi.NULL:
            return None
        return [bool(self._ptr.enabled[i]) for i in range(self._dofs)]

    @enabled.setter
    def enabled(self, values):
        if values is None:
            return
        if self._ptr.enabled == ffi.NULL:
            buf = ffi.new("bool[]", self._dofs)
            self._ptr.enabled = buf
            self._owned['enabled'] = buf
        if len(values) != self._dofs:
            raise ValueError(f"Expected {self._dofs} values, got {len(values)}")
        for i in range(self._dofs):
            self._ptr.enabled[i] = values[i]

    # --- scalar/enum properties ---
    @property
    def control_interface(self):
        return ControlInterface(self._ptr.control_interface)

    @control_interface.setter
    def control_interface(self, value):
        self._ptr.control_interface = int(value)

    @property
    def synchronization(self):
        return Synchronization(self._ptr.synchronization)

    @synchronization.setter
    def synchronization(self, value):
        self._ptr.synchronization = int(value)

    @property
    def duration_discretization(self):
        return DurationDiscretization(self._ptr.duration_discretization)

    @duration_discretization.setter
    def duration_discretization(self, value):
        self._ptr.duration_discretization = int(value)

    @property
    def minimum_duration(self):
        if not self._ptr.has_minimum_duration:
            return None
        return self._ptr.minimum_duration

    @minimum_duration.setter
    def minimum_duration(self, value):
        if value is None:
            self._ptr.has_minimum_duration = False
        else:
            self._ptr.minimum_duration = value
            self._ptr.has_minimum_duration = True

    # --- Pro features ---
    @property
    def intermediate_positions(self):
        n = self._ptr.num_intermediate_waypoints
        if n == 0 or self._ptr.intermediate_positions == ffi.NULL:
            return []
        total = n * self._dofs
        flat = [self._ptr.intermediate_positions[i] for i in range(total)]
        return [flat[i * self._dofs:(i + 1) * self._dofs] for i in range(n)]

    @intermediate_positions.setter
    def intermediate_positions(self, waypoints):
        if not waypoints:
            lib.scatti_input_set_intermediate_positions(self._ptr, ffi.NULL, 0)
            return
        n = len(waypoints)
        flat = ffi.new("double[]", n * self._dofs)
        for i, wp in enumerate(waypoints):
            if len(wp) != self._dofs:
                raise ValueError(f"Waypoint {i}: expected {self._dofs} values, got {len(wp)}")
            for d in range(self._dofs):
                flat[i * self._dofs + d] = wp[d]
        lib.scatti_input_set_intermediate_positions(self._ptr, flat, n)

    @property
    def max_position(self):
        if self._ptr.max_position == ffi.NULL:
            return None
        return _arr_to_list(self._ptr.max_position, self._dofs)

    @max_position.setter
    def max_position(self, values):
        if values is None:
            return
        if self._ptr.max_position == ffi.NULL:
            buf = ffi.new("double[]", self._dofs)
            self._ptr.max_position = buf
            self._owned['max_position'] = buf
        _list_to_arr(self._ptr.max_position, values, self._dofs)

    @property
    def min_position(self):
        if self._ptr.min_position == ffi.NULL:
            return None
        return _arr_to_list(self._ptr.min_position, self._dofs)

    @min_position.setter
    def min_position(self, values):
        if values is None:
            return
        if self._ptr.min_position == ffi.NULL:
            buf = ffi.new("double[]", self._dofs)
            self._ptr.min_position = buf
            self._owned['min_position'] = buf
        _list_to_arr(self._ptr.min_position, values, self._dofs)

    @property
    def interrupt_calculation_duration(self):
        return self._ptr.interrupt_calculation_duration

    @interrupt_calculation_duration.setter
    def interrupt_calculation_duration(self, value):
        self._ptr.interrupt_calculation_duration = value

    def validate(self, check_current_within_limits=False, check_target_within_limits=True):
        return lib.scatti_input_validate(
            self._ptr, check_current_within_limits, check_target_within_limits
        )


# ---------------------------------------------------------------------------
# OutputParameter
# ---------------------------------------------------------------------------

class OutputParameter:
    """Wraps SCattiOutputParameter with Pythonic access."""

    def __init__(self, dofs):
        self._dofs = dofs
        self._ptr = lib.scatti_output_create(dofs)
        if self._ptr == ffi.NULL:
            raise MemoryError("Failed to create SCattiOutputParameter")
        self._trajectory = Trajectory(self._ptr.trajectory, dofs)

    def __del__(self):
        if hasattr(self, "_ptr") and self._ptr != ffi.NULL:
            lib.scatti_output_destroy(self._ptr)
            self._ptr = ffi.NULL

    def __copy__(self):
        new = OutputParameter.__new__(OutputParameter)
        new._dofs = self._dofs
        new._ptr = lib.scatti_output_create(self._dofs)
        # Copy scalar fields
        new._ptr.time = self._ptr.time
        new._ptr.new_section = self._ptr.new_section
        new._ptr.did_section_change = self._ptr.did_section_change
        new._ptr.new_calculation = self._ptr.new_calculation
        new._ptr.was_calculation_interrupted = self._ptr.was_calculation_interrupted
        new._ptr.calculation_duration = self._ptr.calculation_duration
        # Copy arrays
        for i in range(self._dofs):
            new._ptr.new_position[i] = self._ptr.new_position[i]
            new._ptr.new_velocity[i] = self._ptr.new_velocity[i]
            new._ptr.new_acceleration[i] = self._ptr.new_acceleration[i]
            new._ptr.new_jerk[i] = self._ptr.new_jerk[i]
        new._trajectory = Trajectory(new._ptr.trajectory, self._dofs)
        return new

    @property
    def degrees_of_freedom(self):
        return self._dofs

    @property
    def new_position(self):
        return _arr_to_list(self._ptr.new_position, self._dofs)

    @property
    def new_velocity(self):
        return _arr_to_list(self._ptr.new_velocity, self._dofs)

    @property
    def new_acceleration(self):
        return _arr_to_list(self._ptr.new_acceleration, self._dofs)

    @property
    def new_jerk(self):
        return _arr_to_list(self._ptr.new_jerk, self._dofs)

    @property
    def time(self):
        return self._ptr.time

    @property
    def new_section(self):
        return self._ptr.new_section

    @property
    def did_section_change(self):
        return bool(self._ptr.did_section_change)

    @property
    def new_calculation(self):
        return bool(self._ptr.new_calculation)

    @property
    def was_calculation_interrupted(self):
        return bool(self._ptr.was_calculation_interrupted)

    @property
    def calculation_duration(self):
        return self._ptr.calculation_duration

    @property
    def trajectory(self):
        return self._trajectory

    def pass_to_input(self, inp):
        """Copy output state back to input for the next control cycle."""
        lib.scatti_output_pass_to_input(self._ptr, inp._ptr)


# ---------------------------------------------------------------------------
# Ruckig
# ---------------------------------------------------------------------------

class Ruckig:
    """Main trajectory generator — wraps SCatti."""

    def __init__(self, dofs, delta_time, max_waypoints=0):
        self._dofs = dofs
        if max_waypoints > 0:
            self._ptr = lib.scatti_create_waypoints(dofs, delta_time, max_waypoints)
        else:
            self._ptr = lib.scatti_create(dofs, delta_time)
        if self._ptr == ffi.NULL:
            raise MemoryError("Failed to create SCatti")

    def __del__(self):
        if hasattr(self, "_ptr") and self._ptr != ffi.NULL:
            lib.scatti_destroy(self._ptr)
            self._ptr = ffi.NULL

    @property
    def degrees_of_freedom(self):
        return self._dofs

    def reset(self):
        lib.scatti_reset(self._ptr)

    def validate_input(self, inp, check_current_within_limits=False,
                       check_target_within_limits=True):
        return lib.scatti_validate_input(
            self._ptr, inp._ptr,
            check_current_within_limits, check_target_within_limits
        )

    def update(self, inp, out):
        """Advance one control cycle. Returns a :class:`Result`."""
        return Result(lib.scatti_update(self._ptr, inp._ptr, out._ptr))

    def calculate(self, inp, traj):
        """One-shot calculation. Returns a :class:`Result`."""
        return Result(lib.scatti_calculate(self._ptr, inp._ptr, traj._ptr))
