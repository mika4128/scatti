# cruckig

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

**Ruckig 的纯 C 移植版 — 机器人与机械的实时运动轨迹生成**

cruckig 是 [Ruckig](https://github.com/pantor/ruckig) C++ 运动规划库的完整纯 C 移植。生成**时间最优、加加速度（jerk）限制**的轨迹，适用于机器人实时控制。

## 特性

- 纯 C99 实现，零第三方依赖
- 支持任意自由度（DOF）
- 位置控制与速度控制接口
- 时间同步 / 相位同步 / 独立模式
- 非对称速度与加速度限制
- 三阶 / 二阶 / 一阶运动学模型
- 轨迹位置极值查询
- **中间路点（Waypoints）** — 多段轨迹规划，逐段顺序求解
- **逐段运动约束** — 每段独立设置速度/加速度/加加速度上下限
- **位置限制** — 全局或逐段位置上下限，确保轨迹不越界
- **计算中断支持** — 设置微秒级计算预算，超时可恢复继续
- 3 级编译器优化可通过 CMake 宏切换
- 支持 `make install` 和 `.deb` 包构建
- Python 绑定（cffi，零编译，加载 `.so`）
- Rust 绑定（bindgen，自动生成 FFI）

## 使用说明（纯 C API）

cruckig 是**独立的 C99 库**：通过头文件 `cruckig/cruckig.h` 暴露函数与结构体，**不能**当作 C++ Ruckig 的头文件或符号的直接替代品。算法背景与 jerk 限制轨迹问题可参考原论文 [Jerk-limited Real-time Trajectory Generation with Arbitrary Target States](https://arxiv.org/abs/2105.04830)；**具体能调哪些函数、参数怎么摆、内存谁拥有**，以 `include/cruckig/` 下头文件及注释为准。

### 对象生命周期与实时约束

- 用 `cruckig_create` / `cruckig_input_create` / `cruckig_output_create`（及路点场景的 `cruckig_create_waypoints`）在堆上创建对象，用对应的 `*_destroy` 成对释放。
- `CRuckigInputParameter` / `CRuckigOutputParameter` 内部的 `double*` 等缓冲区由库在 `*_create` 里分配好，长度等于构造时传入的 `dofs`；你**不要**在控制周期内反复 `create`/`destroy`，应在任务或线程启动时建好、退出时拆掉。
- 路点数据通过 `cruckig_input_set_intermediate_positions` **拷贝**进库；调用方仍可自由管理自己的源数组。

### 在线控制循环

每个控制周期：用当前 `input` 调用 `cruckig_update`，读 `output` 里的 `new_*` 驱动执行器，再调用 `cruckig_output_pass_to_input`，把输出状态写回 `input` 的「当前」状态，供下一周期使用。若外部测量与规划不一致，下一周期只需把真实状态写进 `current_*`，库会重新规划。

```c
const size_t dofs = 3;
const double delta_time = 0.01;
CRuckig *otg = cruckig_create(dofs, delta_time);
CRuckigInputParameter *in = cruckig_input_create(dofs);
CRuckigOutputParameter *out = cruckig_output_create(dofs);
/* 生产代码中应检查 otg / in / out 是否为 NULL（失败时对非空指针分别 destroy） */
/* 每个使能轴都要设置 max_velocity > 0（创建后默认为 0）；max_acceleration/max_jerk 默认为无穷大 */
/* 填写 in->current_*、target_*、max_* … */
while (1) {
    CRuckigResult r = cruckig_update(otg, in, out);
    /* 使用 out->new_position 等 */
    cruckig_output_pass_to_input(out, in);
    if (r == CRuckigFinished)
        break;
    if (r != CRuckigWorking)
        break; /* 负值为错误码 */
}
cruckig_output_destroy(out);
cruckig_input_destroy(in);
cruckig_destroy(otg);
```

### 输入里要设置什么

在 `in->degrees_of_freedom` 与创建时一致的前提下，为每个轴填写当前/目标运动学量和限制；可选地设置 `control_interface`、`synchronization`、`duration_discretization`，以及 `min_velocity` / `min_acceleration` 等指针（`NULL` 表示使用文档中的默认语义）。扩展能力（中间路点、逐段限制、位置上下限、计算时间预算等）见 `include/cruckig/input_parameter.h` 等头文件中的 Pro 字段说明；各结构体成员与枚举以同目录头文件为准。

### 校验与错误

计算前可调用 `cruckig_validate_input`，参数含义见 `cruckig.h` 与相关头文件注释。`cruckig_update` / `cruckig_calculate` 返回 `CRuckigResult`：`CRuckigWorking` 表示仍在运动，`CRuckigFinished` 表示到达目标，负值为各类错误码（完整枚举在同一文档）。

### 离线求一整条轨迹

先用 `cruckig_trajectory_create(dofs)` 得到 `CRuckigTrajectory *`，再在已有 `CRuckig` 与填好的 `CRuckigInputParameter` 上调用 `cruckig_calculate`，随后用 `cruckig_trajectory_*` 按时间取样或查极值。纯离线时 `delta_time` 常取 `0`，行为以 `cruckig_calculate` 及 `cruckig_trajectory_*` 在头文件中的说明为准。

### 数值与测试

尽量用与物理意义匹配的单位（例如米、秒），避免在「毫米 + 极大 jerk」这类组合上把浮点推到不舒服的区间。本仓库自带的对比与随机测试结果见下文「正确性验证」与「运行测试」。

## 性能对比 / Benchmark

测试环境：7-DOF, 32768 条随机轨迹, 20 轮迭代

| 版本 | 平均计算 (µs) | 最差情况 (µs) | 端到端 (µs) |
|------|:---:|:---:|:---:|
| **cruckig Level 2** | **6.56** | **38.8** | **7.50** |
| C++ Ruckig (-O3) | 7.00 | 33.8 | 7.95 |
| cruckig Level 1 | 7.77 | 40.0 | 8.68 |
| cruckig Level 0 | 8.39 | 39.5 | 9.28 |

Level 2 下 C 版本比 C++ 版本**平均快 6.3%，端到端快 5.7%**。

### 正确性验证

与原版 C++ Ruckig 交叉对比 99,344 条随机轨迹：

| 指标 | 结果 |
|------|------|
| 匹配率 | **100.00%** |
| 最大时长差 | 4.55e-13 s（接近机器精度） |
| 最大位置差 | 5.90e-12 |
| 最大速度差 | 9.24e-14 |

### 优化级别说明

| 级别 | 编译选项 | 适用场景 |
|------|----------|----------|
| 0 (默认) | `-O2` | 可移植，适合交叉编译 |
| 1 | `-O3 -flto` | 推荐，跨文件内联 |
| 2 | `-O3 -flto -march=native` + 数学优化 | 最高性能，仅限本机运行 |

```bash
cmake .. -DCRUCKIG_OPT_LEVEL=2   # 选择优化级别
```

## 交叉编译

### STM32 / Cortex-M 注意

- **堆**：`cruckig_*_create` 等在堆上分配；链接脚本里需预留足够 **heap**，且勿在控制周期内反复 `create`/`destroy`（见上文「对象生命周期与实时约束」）。
- **`double`**：接口与内部计算均为 **双精度浮点**。许多 Cortex-M（例如带 FPU 的 M4F）硬件仅加速 **单精度**，`double` 往往走软件实现，**指令数与代码体积**会明显增加；上板前请实测 `cruckig_update` 是否满足周期预算。
- **勿用 `-march=native`**：嵌入式与交叉编译请使用 **`CRUCKIG_OPT_LEVEL=0` 或 `1`**；Level 2 含本机指令调优，仅适用于在编译所用 CPU 上运行的主机程序。

## 快速开始

### 构建

```bash
cd cruckig
mkdir build && cd build
cmake .. -DCRUCKIG_OPT_LEVEL=1
make -j$(nproc)
```

### 安装

```bash
sudo make install
```

或构建 `.deb` 包（自动分为运行时包 + 开发包）：

```bash
cpack
# 生成两个包:
#   libcruckig_0.1.0_amd64.deb      — 运行时 (libcruckig.so)
#   libcruckig-dev_0.1.0_amd64.deb  — 开发包 (头文件, libcruckig.a, cmake config)

# 安装运行时
sudo dpkg -i libcruckig_*.deb

# 安装开发包 (依赖运行时包)
sudo dpkg -i libcruckig-dev_*.deb
```

### 使用示例

```c
#include <stdio.h>
#include <cruckig/cruckig.h>

int main(void) {
    CRuckig *otg = cruckig_create(3, 0.01);
    CRuckigInputParameter *inp = cruckig_input_create(3);
    CRuckigOutputParameter *out = cruckig_output_create(3);

    inp->current_position[0] = 0.0;
    inp->current_position[1] = 0.0;
    inp->current_position[2] = 0.5;
    inp->target_position[0] = 5.0;
    inp->target_position[1] = -2.0;
    inp->target_position[2] = -3.5;
    inp->max_velocity[0] = 3.0;
    inp->max_velocity[1] = 1.0;
    inp->max_velocity[2] = 0.6;
    inp->max_acceleration[0] = 3.0;
    inp->max_acceleration[1] = 2.0;
    inp->max_acceleration[2] = 1.0;
    inp->max_jerk[0] = 4.0;
    inp->max_jerk[1] = 3.0;
    inp->max_jerk[2] = 2.0;

    CRuckigResult result;
    while (1) {
        result = cruckig_update(otg, inp, out);
        printf("t=%.3f pos=[%.4f, %.4f, %.4f]\n",
               out->time,
               out->new_position[0], out->new_position[1], out->new_position[2]);
        cruckig_output_pass_to_input(out, inp);
        if (result == CRuckigFinished)
            break;
        if (result != CRuckigWorking)
            return 1;
    }

    cruckig_output_destroy(out);
    cruckig_input_destroy(inp);
    cruckig_destroy(otg);
    return 0;
}
```

### 在 CMake 项目中使用

```cmake
add_subdirectory(cruckig)
target_link_libraries(your_target PRIVATE cruckig m)
```

### Python

```python
from cruckig import Ruckig, InputParameter, OutputParameter, Result

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
    print(f"t={out.time:.3f} pos={out.new_position}")
    out.pass_to_input(inp)
    if res == Result.Finished:
        break
    if res != Result.Working:
        raise SystemExit(f"error: {res}")
```

安装：先构建 C 库，然后 `pip install wrapper/python/`。或设置 `CRUCKIG_LIB` 环境变量指向 `libcruckig.so`。

**示例与自检**：`wrapper/python/examples/` 下有可运行脚本，说明见该目录 [README](wrapper/python/examples/README.md)。其中 **`test_readme_python.py`** 与上文代码路径一致，用于验证绑定是否可用；在仓库根目录可先 `cmake` 编出 `build/libcruckig.so`，再执行：

```bash
PYTHONPATH=wrapper/python python3 wrapper/python/examples/test_readme_python.py
```

（脚本在未设置 `CRUCKIG_LIB` 时会尝试使用 `build/libcruckig.so`。）

### Rust

```rust
use cruckig::{Ruckig, InputParameter, OutputParameter, Result};

let mut ruckig = Ruckig::new(3, 0.01);
let mut input = InputParameter::new(3);
let mut output = OutputParameter::new(3);

input.current_position_mut().copy_from_slice(&[0.0, 0.0, 0.5]);
input.target_position_mut().copy_from_slice(&[5.0, -2.0, -3.5]);
input.max_velocity_mut().copy_from_slice(&[3.0, 1.0, 3.0]);
input.max_acceleration_mut().copy_from_slice(&[3.0, 2.0, 1.0]);
input.max_jerk_mut().copy_from_slice(&[4.0, 3.0, 2.0]);

loop {
    let res = ruckig.update(&input, &mut output);
    println!("t={:.3} pos={:?}", output.time(), output.new_position());
    output.pass_to_input(&mut input);
    match res {
        Result::Finished => break,
        Result::Working => {}
        _ => std::process::exit(1),
    }
}
```

在 `Cargo.toml` 中添加依赖：

```toml
[dependencies]
cruckig = { path = "path/to/cruckig/wrapper/rust" }
```

**自检**：`wrapper/rust/tests/readme_example.rs` 与 README 示例同一路径，运行 `cargo test --test readme_example`（在 `wrapper/rust` 目录下）可确认 crate 与内嵌 C 源码编译、链接正常；CMake 的 `ctest` 在启用包装器测试时也会执行该用例（测试名 **`readme_rust`**）。

## 运行测试

在 `build` 目录下（`cmake` 且 `BUILD_TESTS=ON` 后）：

```bash
./cruckig_test_readme       # README「使用示例」同一路径的冒烟测试（终点与目标位置一致）
./cruckig_test              # 223 个确定性测试 (含 Pro 功能验证)
./cruckig_test_random       # 随机轨迹测试 (10 类, 26 万条轨迹, 520 万断言)
./cruckig_benchmark         # 性能基准
ctest                       # 含 C readme_example；默认另含 Python/Rust 冒烟（需 python3+cffi、cargo）
ctest -L wrapper            # 仅 Python + Rust README 测试
```

关闭包装器 ctest：`cmake .. -DBUILD_WRAPPER_TESTS=OFF`。

**单独运行（不经过 ctest）**

```bash
# Python：需已构建共享库 + 安装 cffi；详见 wrapper/python/examples/README.md
cd /path/to/cruckig
export PYTHONPATH=wrapper/python
# 可选：export CRUCKIG_LIB=/path/to/cruckig/build/libcruckig.so
python3 wrapper/python/examples/test_readme_python.py

# Rust：crate 内编译 C 源，不依赖 libcruckig.so
cd /path/to/cruckig/wrapper/rust && cargo test --test readme_example
```

### 与 C++ Ruckig 交叉对比（可选）

```bash
cmake .. -DCRUCKIG_OPT_LEVEL=1 -DRUCKIG_DIR=/path/to/ruckig
make -j$(nproc)
./cruckig_test_comparison   # 10 万条随机轨迹逐条对比
```

## 更多说明

- [Python 示例目录说明](wrapper/python/examples/README.md) — 示例脚本、`test_readme_python` 冒烟与环境变量

## 目录结构

```
cruckig/
├── include/cruckig/       # 公共头文件 (14 个 .h)
├── src/                   # 库源文件 (19 个 .c)
├── test/                  # 测试和基准测试
├── examples/              # C 使用示例
├── wrapper/
│   ├── python/            # Python 绑定 (cffi)
│   │   ├── cruckig/       # Python 包
│   │   └── examples/      # Python 示例与 test_readme_python 冒烟脚本（见 README）
│   └── rust/              # Rust 绑定 (bindgen)
│       ├── src/           # Rust crate
│       ├── tests/         # readme_example 集成测试（对齐 README）
│       └── examples/      # Rust 示例
├── doc/                   # 技术文档（中文文件名）
└── CMakeLists.txt         # 构建脚本 (含 install/CPack/deb)
```

## 许可证

与 Ruckig Community Edition 相同，MIT 许可证。

## 致谢

基于 [Ruckig](https://github.com/pantor/ruckig) by Lars Berscheid。

论文: Berscheid, L., & Kroeger, T. (2021). *Jerk-limited Real-time Trajectory Generation with Arbitrary Target States.* Robotics: Science and Systems (RSS).
