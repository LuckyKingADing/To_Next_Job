# TCMSF 初始化评估工具

本目录包含 TCMSF 系统初始化成功率/耗时测试和重初始化检测的工具。

## 目录结构

```
init_evaluator/
├── include/                    # 头文件
│   └ init_state_recorder.h
├── src/                        # 源文件
│   └ init_state_recorder.cpp
├── tcmsf_init_evaluator_main.cc       # C++ 主程序
├── tcmsf_init_evaluator_dispatcher.py # Python 调度器（推荐使用）
├── tcmsf_reinit_detector.py           # 重初始化检测工具
└── README.md
```

## 构建

```bash
cd /apollo
bazel build modules/localization/src/TCMSF/tcmsf/util:tcmsf_init_evaluator --config=release_build
```

构建产物路径：
```
/apollo/.cache/bazel/base/execroot/apollo/bazel-out/k8-opt/bin/modules/localization/src/TCMSF/tcmsf/util/tcmsf_init_evaluator
```

---

## 工具1: 初始化成功率/耗时测试

### 功能

- 通过固定间隔切片方法，测试不同起始时间点的初始化成功率
- 精确统计初始化耗时（从 TCMSF 内部日志提取）
- 支持自动模式（运行到数据结束）
- 自动检测 record 数据的起始时间戳

### 关键时间节点

| 状态节点 | 日志格式 | 含义 |
|---------|---------|------|
| 切片开始 | 程序开始读取数据 | 从 `slice_start_timestamp` 开始灌入数据 |
| Dynamic Ready | `[startup] Dynamic ready (gnss)! imu time: XXX` | 检测到车辆运动，满足初始化条件 |
| Start Init | `[startup] TCMSF Start Init! imu time: XXX` | TCMSF 内部开始初始化计算 |
| TCMSF Ready | `[startup] TCMSF Ready! imu time: XXX` | 初始化完成 |

**总初始化耗时** = TCMSF Ready 时间 - 切片开始时间

包含：数据读取时间 + 等待 Dynamic Ready + TCMSF 内部初始化

### 使用方法

```bash
python3 tcmsf_init_evaluator_dispatcher.py \
    record_path \
    output_path \
    start_timestamp \
    slice_interval \
    slice_count \
    init_timeout \
    [imu_config_path]
```

#### 参数说明

| 参数 | 说明 | 示例 |
|------|------|------|
| `record_path` | record 文件路径（单个文件或目录） | `/apollo/data/record` |
| `output_path` | 输出结果路径 | `/apollo/data/output` |
| `start_timestamp` | 起始时间戳，`0` 表示自动检测 | `0` 或 `1774324300` |
| `slice_interval` | 切片间隔（秒） | `40` |
| `slice_count` | 切片总数，`-1` 表示运行到数据结束 | `20` 或 `-1` |    -1 not understand
| `init_timeout` | 单个切片初始化超时时间（秒） | `300` |
| `imu_config_path` | IMU 配置文件路径（可选） | |

#### 示例

**自动模式（推荐）**：从头开始，运行到数据结束，超时 300 秒
```bash
python3 tcmsf_init_evaluator_dispatcher.py \
    /mnt/d/dockers/rt/rtk_pvt/2026/0324/2026-03-24_19-30-11 \
    /apollo/data/init_eval_result \
    0 40 -1 300
```

**固定切片模式**：测试 20 个切片，超时 120 秒
```bash
python3 tcmsf_init_evaluator_dispatcher.py \
    /mnt/d/dockers/rt/rtk_pvt/2026/0324/2026-03-24_11-52-51 \
    data/init_eval_result \
    1774324300 40 20 120
```

### 输出文件

| 文件 | 内容 |
|------|------|
| `init_eval_results.csv` | 每个切片的详细结果（CSV格式） |
| `init_eval_summary.txt` | 汇总统计（成功率、平均耗时、时间范围等） |
| `init_eval_log.txt` | 详细日志 |

### 结果示例

```
Total slices tested: 98
Successful initializations: 98
Success rate: 100.0%

Average init time: 2.42 seconds
Min init time: 0.9995 seconds
Max init time: 113.71 seconds
Median init time: 1.01 seconds
```

---

## 工具2: 重初始化检测

### 功能

- 运行完整的 record 数据（不切片）
- 检测 "reinit due to large pos error" 等重初始化日志
- 统计重初始化次数和时间戳
- 分组输出初始化日志

### 使用方法

```bash
python3 tcmsf_reinit_detector.py \
    record_path \
    output_path \
    [start_timestamp] \
    [imu_config_path]
```

#### 参数说明

| 参数 | 说明 |
|------|------|
| `record_path` | record 文件路径 |
| `output_path` | 输出结果路径 |
| `start_timestamp` | 起始时间戳（可选，默认从头开始） |
| `imu_config_path` | IMU 配置文件路径（可选） |

#### 示例

```bash
python3 tcmsf_reinit_detector.py \
    /apollo/data/2026/data_0410-145857 \
    /apollo/data/reinit_result
```

### 输出文件

| 文件 | 内容 |
|------|------|
| `reinit_report.txt` | 重初始化报告（分组显示初始化日志） |
| `full_record_output.log` | 完整运行日志 |

### 结果示例

```
Total initialization count: 3
Reinitialization events detected: 2

# First Initialization (count=1)
[startup] Dynamic ready (gnss)! imu time: 1775804340.63
[startup] TCMSF Start Init! imu time: 1775804341.13
[startup] TCMSF Ready! imu time: 1775804342.14
...

# Reinitialization #1 (count=2)
reinit due to large pos error
...
```

---

## 技术说明

### 为什么使用子进程？

TCMSF 内部使用 `static auto xxx_call_once_ = [&...]()()` 模式，导致关键日志只在首次运行时输出。通过在每个切片启动独立子进程，可以绕过这个限制，确保每个切片都能捕获完整的初始化日志。

### 超时机制

- 默认超时：300秒（5分钟）
- 使用 wall clock 时间判断，而非数据时间戳
- 超时后标记为 FAILED

### 无效切片处理

自动模式下，如果连续 3 个切片没有数据（gnss_count=0, imu_count=0），则判定数据结束并停止测试。无效切片不计入统计。

---

## 注意事项

1. **运行环境**：需要在 Apollo 容器内运行，依赖 `CYBER_PATH` 和 `LD_LIBRARY_PATH` 环境变量
2. **数据要求**：record 数据需包含 GNSS、IMU 等必要通道
3. **时间戳精度**：所有时间戳输出为 4 位小数（毫秒级精度）
4. **路径映射**：宿主机路径 `/home/dingxingyu/E2E_PingCe/ota_data` 映射到容器 `/apollo/data`