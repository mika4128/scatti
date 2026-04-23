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

"""Velocity control example."""

from scatti import (
    ControlInterface,
    InputParameter,
    OutputParameter,
    Result,
    Ruckig,
)


if __name__ == "__main__":
    ruckig = Ruckig(3, 0.01)
    inp = InputParameter(3)
    out = OutputParameter(3)

    inp.control_interface = ControlInterface.Velocity

    inp.current_position = [0.0, 0.0, 0.5]
    inp.current_velocity = [3.0, -2.2, -0.5]
    inp.current_acceleration = [0.0, 2.5, -0.5]

    inp.target_velocity = [0.0, -0.5, -1.5]
    inp.target_acceleration = [0.0, 0.0, 0.5]

    inp.max_acceleration = [3.0, 2.0, 1.0]
    inp.max_jerk = [6.0, 6.0, 4.0]

    print("t | position")
    while ruckig.update(inp, out) == Result.Working:
        pos = ", ".join(f"{p:.4f}" for p in out.new_position)
        print(f"{out.time:.4f} | [{pos}]")
        out.pass_to_input(inp)

    print(f"Trajectory duration: {out.trajectory.duration:.4f} [s]")
