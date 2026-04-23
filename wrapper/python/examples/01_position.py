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

"""Position control example — mirrors the C++ / Ruckig example."""

from copy import copy

from scatti import InputParameter, OutputParameter, Result, Ruckig


if __name__ == "__main__":
    # Create instances: 3 DOFs, 0.01 s control cycle
    ruckig = Ruckig(3, 0.01)
    inp = InputParameter(3)
    out = OutputParameter(3)

    # Set input parameters
    inp.current_position = [0.0, 0.0, 0.5]
    inp.current_velocity = [0.0, -2.2, -0.5]
    inp.current_acceleration = [0.0, 2.5, -0.5]

    inp.target_position = [5.0, -2.0, -3.5]
    inp.target_velocity = [0.0, -0.5, -2.0]
    inp.target_acceleration = [0.0, 0.0, 0.5]

    inp.max_velocity = [3.0, 1.0, 3.0]
    inp.max_acceleration = [3.0, 2.0, 1.0]
    inp.max_jerk = [4.0, 3.0, 2.0]

    print("\t".join(["t"] + [str(i) for i in range(ruckig.degrees_of_freedom)]))

    # Generate the trajectory within the control loop
    first_output, out_list = None, []
    res = Result.Working
    while res == Result.Working:
        res = ruckig.update(inp, out)

        print("\t".join([f"{out.time:0.3f}"] + [f"{p:0.3f}" for p in out.new_position]))
        out_list.append(copy(out))

        out.pass_to_input(inp)

        if not first_output:
            first_output = copy(out)

    print(f"Calculation duration: {first_output.calculation_duration:0.1f} [µs]")
    print(f"Trajectory duration: {first_output.trajectory.duration:0.4f} [s]")
