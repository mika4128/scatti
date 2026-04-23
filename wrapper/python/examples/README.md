# Python 示例说明

运行前需已构建 **`libscatti.so`**（或 macOS 上的 `libscatti.dylib`），并安装 **cffi**（`pip install cffi`）。

在 **`wrapper/python`** 目录下将当前目录加入模块搜索路径：

```bash
cd wrapper/python
PYTHONPATH=. python3 examples/01_position.py
```

或在仓库根目录：

```bash
export PYTHONPATH=wrapper/python
python3 wrapper/python/examples/01_position.py
```

## 脚本一览

| 文件 | 说明 |
|------|------|
| `01_position.py` | 多轴位置控制在线循环示例 |
| `05_velocity.py` | 速度控制接口示例 |
| `test_readme_python.py` | 与根目录 README「Python」代码路径一致的**冒烟测试**：跑完一条轨迹并校验终点与目标位置一致（容差 `1e-5`）。未设置 `SCATTI_LIB` 时，会尝试使用仓库下 `build/libscatti.so`（或 `.dylib`）。 |

CMake 开启 `BUILD_TESTS` 且 `BUILD_WRAPPER_TESTS=ON` 时，`ctest` 中的 **`readme_python`** 即调用本目录下的 `test_readme_python.py`，并通过环境变量注入 `SCATTI_LIB` 与 `PYTHONPATH`。

## 环境变量

- **`SCATTI_LIB`**：指向共享库的绝对路径（可选；不设置时由 `test_readme_python.py` 尝试自动推断，或由 `scatti._binding` 按 [构建说明](../../doc/构建说明.md) 中「库查找顺序」一节所述顺序查找）。
- **`PYTHONPATH`**：需包含 `wrapper/python`（使能 `import scatti`）。
