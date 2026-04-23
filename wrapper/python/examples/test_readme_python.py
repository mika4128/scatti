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

#!/usr/bin/env python3
"""README Python 示例路径冒烟测试。

优先使用环境变量 SCATTI_LIB；未设置时尝试仓库根目录下 ``build/libscatti.so``。
运行前需将 ``wrapper/python`` 加入 PYTHONPATH（或由已安装的 scatti 包提供模块）。
"""

from __future__ import annotations

import os
import sys
from typing import Optional


def near(a: float, b: float, eps: float) -> bool:
    return abs(a - b) <= eps


def _guess_scatti_lib() -> Optional[str]:
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.normpath(os.path.join(here, "..", "..", ".."))
    for name in ("libscatti.so", "libscatti.dylib"):
        p = os.path.join(root, "build", name)
        if os.path.isfile(p):
            return p
    return None


def main() -> int:
    lib = os.environ.get("SCATTI_LIB") or _guess_scatti_lib()
    if not lib or not os.path.isfile(lib):
        print(
            "error: set SCATTI_LIB to the built libscatti.so "
            "(e.g. build/libscatti.so) or build the shared library under <repo>/build/",
            file=sys.stderr,
        )
        return 1
    os.environ["SCATTI_LIB"] = lib

    try:
        from scatti import InputParameter, OutputParameter, Result, Ruckig
    except ImportError as e:
        print(f"error: cannot import scatti ({e}); pip install cffi", file=sys.stderr)
        return 1

    r = Ruckig(3, 0.01)
    inp = InputParameter(3)
    out = OutputParameter(3)

    inp.current_position = [0.0, 0.0, 0.5]
    inp.target_position = [5.0, -2.0, -3.5]
    inp.max_velocity = [3.0, 1.0, 0.6]
    inp.max_acceleration = [3.0, 2.0, 1.0]
    inp.max_jerk = [4.0, 3.0, 2.0]

    steps = 0
    while True:
        res = r.update(inp, out)
        out.pass_to_input(inp)
        steps += 1
        if res == Result.Finished:
            break
        if res != Result.Working:
            print(f"error: unexpected Result {res} at step {steps}", file=sys.stderr)
            return 1

    eps = 1e-5
    tgt = inp.target_position
    pos = out.new_position
    for i in range(3):
        if not near(pos[i], tgt[i], eps):
            print(f"final position mismatch axis {i}: {pos[i]} vs {tgt[i]}", file=sys.stderr)
            return 2

    print(f"readme python: ok, {steps} steps, t={out.time:.3f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
