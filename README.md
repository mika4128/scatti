# scatti

**English** | [简体中文](README.zh.md)

<p align="center">
  <a href="https://github.com/mika4128/cruckig/actions/workflows/build-deb.yml">
    <img src="https://github.com/mika4128/cruckig/actions/workflows/build-deb.yml/badge.svg" alt="Build DEB workflow">
  </a>
  <a href="https://github.com/mika4128/cruckig/issues">
    <img src="https://img.shields.io/github/issues/mika4128/cruckig.svg" alt="Issues">
  </a>
  <a href="https://github.com/mika4128/cruckig/releases">
    <img src="https://img.shields.io/github/v/release/mika4128/cruckig.svg?include_prereleases&sort=semver" alt="Releases">
  </a>
  <a href="https://github.com/mika4128/cruckig/blob/main/LICENSE">
    <img src="https://img.shields.io/badge/license-MIT-green.svg" alt="MIT">
  </a>
</p>

> **Pronunciation**: Italian /ˈskat.ti/ — `scatti` means "bursts / jolts" (rapid movements), fitting a jerk-limited motion generator.

**Pure C99 port of Ruckig — real-time jerk-limited trajectory generation.**

## Features

- Pure C99, zero third-party dependencies, portable to bare-metal / RT kernels (LinuxCNC RTAPI).
- Arbitrary DOF, position / velocity control, three synchronization modes (time / phase / independent).
- 3rd / 2nd / 1st order models, asymmetric velocity and acceleration limits.
- Full Ruckig Pro feature set: **intermediate waypoints**, **per-section constraints**, **position bounds**, **computation interrupts** (microsecond budgets).
- 3-level CMake optimization switch (`-DSCATTI_OPT_LEVEL={0,1,2}`).
- Optional musl cube root + overridable memory macros for libm-free / RTAPI embedded environments.
- Out-of-the-box Python (cffi) + Rust (bindgen) bindings.

<p align="center">
  <img src="assets/hero.png" alt="scatti smooth position with bounded jerk" width="720">
</p>

<p align="center"><sub>Smooth position + bounded acceleration + piecewise-constant jerk — 3-DOF, 500 Hz real-time generation.</sub></p>

## Performance

7-DOF mean / end-to-end (µs):

| Version | Mean | End-to-end |
|---------|:---:|:---:|
| Ruckig C++ `-O3`                              | 6.65 | 7.55 |
| Ruckig C++ `-O3 -march=native -flto` (fair)   | 6.46 | 7.34 |
| **scatti L2 (libm cbrt)**                     | **6.19** | **7.08** |
| **scatti L2 + fast cbrt** (musl replaces libm) | **6.03** | **6.93** |

scatti L2 is **7%** faster on average than fairly-compiled Ruckig, **~9%** faster than the default Release build.
The built-in musl cbrt adds another 2-3% and removes the libm dependency — suitable for RT kernels.

DOF scaling (fast cbrt mode, same machine):

| DOFs | Mean (µs) | End-to-end (µs) |
|:---:|:---:|:---:|
|  3 | 2.24  | 2.69  |
|  7 | 6.04  | 6.94  |
| 14 | 12.76 | 14.15 |

### Numerical consistency

Cross-checked against C++ Ruckig over 99,344 random trajectories; both libm and fast-cbrt configurations are **bit-identical**:

| Metric | Result |
|--------|--------|
| Match rate | **100.00%** |
| Max duration diff | 4.55e-13 s |
| Max position diff | 5.90e-12 |
| Max velocity diff | 9.24e-14 |

## Quick start

```bash
cmake -B build -DSCATTI_OPT_LEVEL=1
cmake --build build -j
sudo cmake --install build      # or: cd build && cpack   to build a .deb
```

Minimal usage:

```c
#include <scatti/scatti.h>

CRuckig *otg = scatti_create(3, 0.01);       /* 3 DOF, 10 ms cycle */
CRuckigInputParameter *in  = scatti_input_create(3);
CRuckigOutputParameter *out = scatti_output_create(3);

in->current_position[0] = 0.0;  in->target_position[0] = 5.0;
in->max_velocity[0]     = 3.0;
in->max_acceleration[0] = 3.0;
in->max_jerk[0]         = 4.0;
/* fill the remaining DOFs the same way */

while (scatti_update(otg, in, out) == CRuckigWorking) {
    /* drive the actuator: out->new_position / new_velocity / new_acceleration */
    scatti_output_pass_to_input(out, in);
}

scatti_output_destroy(out);
scatti_input_destroy(in);
scatti_destroy(otg);
```

Headers live in `include/scatti/`; the full API and Pro fields are documented in the corresponding header comments.

CMake integration:

```cmake
add_subdirectory(scatti)
target_link_libraries(your_target PRIVATE scatti m)
```

## Acknowledgements

Based on [Ruckig](https://github.com/pantor/ruckig) by Lars Berscheid (original C++ implementation, Community Edition).

Paper: Berscheid, L., & Kroeger, T. (2021). *Jerk-limited Real-time Trajectory Generation with Arbitrary Target States.* Robotics: Science and Systems (RSS). [arXiv:2105.04830](https://arxiv.org/abs/2105.04830)

## License

MIT (same as Ruckig Community Edition).
