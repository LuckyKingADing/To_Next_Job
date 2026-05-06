# SLAM 方向开源项目汇总

> 来源：招聘岗位技能要求中涉及的开源项目  
> 整理时间：2026.05  
> 适用方向：定位算法 / SLAM / 多传感器融合

---

## 一、视觉 SLAM

### 1. ORB-SLAM2 / ORB-SLAM3

**定位：** 视觉 SLAM 领域的标杆项目，工业界和学术界影响力最大

**GitHub**
- ORB-SLAM2：`https://github.com/UZ-SLAMLab/ORB_SLAM2`
- ORB-SLAM3：`https://github.com/UZ-SLAMLab/ORB_SLAM3`

**核心模块**

```
ORB-SLAM2：
├── Tracking（跟踪）：ORB 特征提取、对极几何、PnP
├── Local Mapping（局部建图）：局部 BA 优化
├── Loop Closing（回环检测）：词袋模型 DBoW2
└── Bundle Adjustment：光束法平差

ORB-SLAM3：
├── 增加了 IMU 融合（Visual-Inertial）
├── 支持单目、双目、RGB-D
├── 多地图系统
└── 召回率更高的回环检测
```

**学习重点**

| 模块 | 面试常考点 |
|---|---|
| ORB 特征提取 | 旋转不变性、快速检索 |
| 词袋模型 DBoW2 | 回环检测的核心，tf-idf 权重 |
| 对极几何 | 基础矩阵、本质矩阵的求解方法 |
| PnP | EPnP、UPnP 原理 |
| BA 优化 | g2o / Ceres 求解，稀疏性利用 |
| 共视图（Covisibility Graph）| 局部建图和回环的图结构 |

**需要掌握到什么程度**
- 能跑通 EuRoC 数据集
- 能看懂 Tracking 线程的代码流程
- 能解释词袋模型如何加速回环检测
- 能手写一个简化版的特征点 SLAM

---

### 2. VINS-Mono / VINS-Fusion

**定位：** 视觉惯性里程计（VIO）工业落地最佳选择，代码质量高、文档好

**GitHub**
- VINS-Mono：`https://github.com/HKUST-Aerial-Robotics/VINS-Mono`
- VINS-Fusion：`https://github.com/HKUST-Aerial-Robotics/VINS-Fusion`

**核心模块**

```
VINS-Mono：
├── feature_tracker/     # 特征跟踪、光流法
├── estimator/          # 核心融合估计
│   ├── initial_ex_rotation.cpp   # 相机-IMU 外参初始化
│   ├── initial_ex_rotation.h
│   ├── initial_aligment.cpp      # 视觉结构与 IMU 对齐
│   ├── estimator.cpp              # 滑动窗口优化主逻辑
│   ├── estimator_node.cpp        # ROS 接口
│   └── factor/                   # 各类残差定义
│       ├── imu_factor.h          # IMU 预积分残差
│       ├── projection_factor.h   # 视觉重投影残差
│       └── marginalization_factor.h  # 边缘化残差
└── camera_models/      # 相机模型、畸变

VINS-Fusion：多相机版本 + GPS 插件
```

**学习重点**

| 模块 | 面试常考点 |
|---|---|
| IMU 预积分 | 预积分量定义、残差推导、bias 雅可比 |
| 初始化 | 纯视觉结构恢复 → IMU 对齐 → 尺度估计 |
| 滑动窗口 | 窗口大小选择、边缘化策略、先验残差 |
| 紧耦合 | IMU 残差 + 视觉残差联合优化 |
| 外参标定 | 相机-IMU 旋转外参的初始化估计 |

**需要掌握到什么程度**
- 能跑通 EuRoC 数据集，精度达标
- 能精读 `estimator.cpp` 和 `imu_factor.h` 的代码
- 能用自己的话讲解 VINS 的初始化流程
- 知道与 MSCKF 的区别（优化 vs 滤波）

**数据集配合**
- **EuRoC MAV**：最经典的 VIO 数据集，包含 Visual + IMU 真值
- MH_01 ~ MH_05（机器大厅）、V1_01 ~ V1_03（飞行器）、V2_01 ~ V2_03（飞行器）

---

### 3. OKVIS

**定位：** 多目视觉惯性 SLAM，滤波器方法的代表

**GitHub**：`https://github.com/ethz-asl/okvis_ros`

**核心特点**
- 多目相机支持（针孔模型 + 畸变）
- 基于非线性优化的 IMU 和视觉联合估计
- 可视化工具完善

**学习价值**
- 对比 VINS（优化方法）：理解不同方法的 trade-off
- 多目相机融合的工程实现

---

### 4. ROVIO

**定位：** 直接法（Direct）VIO，基于滤波

**GitHub**：`https://github.com/ethz-asl/rovio`

**核心特点**
- 直接使用图像灰度信息（不提取特征点）
- 基于滤波的紧耦合视觉惯性估计
- 论文：《ROVIO: Robust Visual Inertial Odometry》

**学习价值**
- 理解直接法和特征点法的区别
- 滤波方法在 VIO 中的实现

---

### 5. SVO / SVO2

**定位：** 半直接法（Semi-Direct）视觉里程计，速度极快

**GitHub**
- SVO：`https://github.com/uzh-rpg/rpg_svo`
- SVO2：`https://github.com/uzh-rpg/rpg_svo_pro`

**核心特点**
- 使用稀疏直接法（半直接）
- 速度极快，适合嵌入式平台
- 与 IMU 融合的版本存在

**学习价值**
- 理解半直接法的思路
- 快速 VO 系统的设计理念

---

### 6. DSO / LSD-SLAM

**定位：** 直接法 SLAM 的经典

**GitHub**
- DSO：`https://github.com/JakobEngel/dso`
- LSD-SLAM：`https://github.com/tum-vision/lsd_slam`

**核心特点**
- DSO：直接法稀疏里程计，基于优化，精度极高
- LSD-SLAM：大范围直接法 SLAM，基于深度滤波器

**学习价值**
- 理解光度误差 vs 几何误差
- 直接法的优势（弱纹理场景）和劣势

---

## 二、激光 SLAM

### 7. LOAM / LeGO-LOAM / A-LOAM

**定位：** 激光 SLAM 奠基性框架

**GitHub**
- A-LOAM：`https://github.com/HKSAerialRobotics/A-LOAM`（推荐学习版本，简化版 LOAM）
- LeGO-LOAM：`https://github.com/RobustFieldAutonomyLab/LeGO-LOAM`
- LOAM 原始版（非开源，但有社区复现）

**核心模块**

```
LeGO-LOAM：
├── imageProjection/      # 点云分割（地面分割、障碍物分割）
├── featureAssociation/  # 特征提取（边缘点、平面点）
│   ├── groundProjection
│   └── laserOdometry
├── mapOptmization/      # 位姿图优化
└── transformFusion/     # 坐标变换融合
```

**学习重点**

| 模块 | 面试常考点 |
|---|---|
| 特征提取 | 曲率计算、边缘点 vs 平面点的判定 |
| Scan Registration | 点云配准，ICP 变体 |
| 地面分割 | 基于地面法向量的分割方法 |
| 位姿图优化 | 因子图在 LiDAR SLAM 中的应用 |

**需要掌握到什么程度**
- 能跑通 KITTI 数据集
- 能理解 LeGO-LOAM 的分割思想
- 能对比 LOAM vs LeGO-LOAM 的改进点

---

### 8. LIO-SAM

**定位：** 因子图 + 多传感器激光 SLAM，工业落地最常用

**GitHub**：`https://github.com/TixiaoShan/LIO-SAM`

**核心模块**

```
LIO-SAM：
├── src/
│   ├── imageProjection.cpp      # 点云处理、畸变校正
│   ├── featureExtraction.cpp    # 特征提取（平滑度）
│   ├── imageProjection.cpp
│   ├── gtsamTypes.h             # GTSAM 自定义因子
│   └──
├── config/
│   └── params.yaml               # 参数配置
└── notes/
    └── LIO-SAM_notes.pdf        # 作者的算法笔记
```

**因子图结构（LIO-SAM 最核心）**

```
因子图中的因子：
├── IMU Preintegration Factor      # IMU 预积分因子
├── GPS Factor                     # GPS 位置因子（可选）
├── Loop Closure Factor             # 回环检测因子
├── Visual Loop Factor              # 视觉回环（可选）
└── LiDAR Odometry Factor           # 激光里程计因子
```

**需要掌握到什么程度**
- 能跑通 KITTI 或你自有数据集
- 能画出 LIO-SAM 的因子图结构
- 能理解 IMU/GPS/LiDAR 各因子的作用
- 能给 LIO-SAM 添加自定义因子（面试加分项）

---

### 9. FAST-LIO / FAST-LIO2

**定位：** 紧耦合 LiDAR-IMU 里程计，速度极快，精度高

**GitHub**
- FAST-LIO2：`https://github.com/hku-mars/FAST_LIO`

**核心特点**

```
FAST-LIO2 亮点：
├── 增量式 kd-tree（iKD-Tree）：增量更新，无需全局重建
├── 紧耦合 IKDF-SAM：LiDAR 残差 + IMU 预积分残差联合优化
├── 反向传播：补偿运动畸变
├── 运行频率：> 20Hz（LOAM 级别速度）
└── 支持多种激光雷达：Velodyne、Ouster、速腾、禾赛
```

**需要掌握到什么程度**
- 能跑通室内和室外数据集
- 能理解增量 kd-tree 的优势
- 能对比 FAST-LIO2 vs LIO-SAM 的差异

---

### 10. LIO-Mapping / LINS

**GitHub**
- LIO-Mapping：`https://github.com/RobustFieldAutonomyLab/lio-mapping`
- LINS：`https://github.com/ChengGao97/LINS`

**核心特点**
- LIO-Mapping：紧耦合 LiDAR-IMU 建图
- LINS：LIO-Mapping 的实时版本，用 EKF 加速

**学习价值**
- 理解 LiDAR-IMU 紧耦合的不同实现路径

---

## 三、多传感器融合

### 11. Apollo 定位模块

**定位：** 自动驾驶量产方向最直接的参考项目

**核心源码目录**

```
apollo/modules/localization/
├── msf/                          # 多传感器融合定位
│   ├── integrated_navigation_system/
│   │   ├── gnss_ins_integrator.cpp       # GNSS + INS 融合
│   │   ├── gnss_imu_integrator.cpp        # GNSS/IMU 融合核心
│   │   ├── gnss_robust_integrator.cpp    # 抗差融合
│   │   └── gnss_position_integrator.cpp   # 纯 GNSS 定位
│   ├── koopa/                          # KFO/紧组合相关
│   └── params/                          # 参数配置
├── init_service/                 # 定位初始化
├── dag/                          # CyberRT DAG 配置
└── conf/                         # 配置文件
```

**需要掌握到什么程度**
- 能画 Apollo 定位模块的完整数据流
- 能解释 RTK、视觉定位、融合定位三种模式
- 能读懂 `gnss_imu_integrator.cpp` 的融合逻辑
- 能讲解 Apollo 的降级策略

---

### 12. LIO-SAM-GPS（扩展）

**GitHub**：`https://github.com/TixiaoShan/LIO-SAM`

在 LIO-SAM 基础上加入 GPS 因子的扩展版本，是你做 GNSS-IMU-ODO 融合的很好参考。

---

## 四、工具库

### 13. GTSAM

**定位：** 因子图优化库，定位融合方向必备

**GitHub**：`https://github.com/borglab/gtsam`

**核心概念**

```
GTSAM 核心要素：
├── Keys：变量的唯一标识（Symbol / Index）
├── Factors：因子节点（UnaryFactor / BinaryFactor / N元因子）
│   ├── PriorFactor：先验因子
│   ├── BetweenFactor：位姿间约束（odometry）
│   ├── GenericNumericFactor：自定义残差
│   └── 自定义因子（继承 DerivedFactor）
├── Values：变量值集合
├── NonlinearFactorGraph：非线性因子图
└── LevenbergMarquardtOptimizer / GaussNewtonOptimizer：优化器
```

**学习路径**

```
Step 1：安装 + 跑通官方 examples
Step 2：实现一个 2D SLAM 问题（自主数据集）
Step 3：实现 IMU + GPS 融合 demo
Step 4：自定义因子开发
```

---

### 14. g2o

**定位：** 图优化通用库，SLAM 圈最常用

**GitHub**：`https://github.com/RainerKuemmerle/g2o`

**核心概念**

```
g2o 核心要素：
├── BaseVertex：自定义顶点的基类
├── BaseEdge：自定义边的基类
├── OptimizationAlgorithm：优化算法（GN / LM / Dogleg）
└── SparseBlockMatrix：稀疏结构加速
```

**学习路径**
- 对比 g2o 和 GTSAM 的使用体验
- 用 g2o 实现 ORB-SLAM2 中的 BA 优化部分

---

### 15. Eigen

**定位：** C++ 矩阵运算库，所有定位算法的底层依赖

**GitHub**：`https://github.com/ggventurini/eigen`

**必须掌握的内容**
- 矩阵乘法、转置、求逆、分解（SVD、Cholesky）
- 四元数与旋转矩阵互转：`Eigen::Quaterniond` / `Eigen::AngleAxisd`
- 块操作：`matrix.block()`, `matrix.col()`, `matrix.row()`
- 几何模块：`Eigen::Geometry`

---

### 16. Ceres Solver

**定位：** 非线性最小二乘求解器（Google）

**GitHub**：`https://github.com/ceres-solver/ceres-solver`

**核心特点**
- 自动求导（Jet）：不需要手动算雅可比
- 鲁棒性更强（Huber Loss、Cauchy Loss 等）
- ORB-SLAM3 使用 Ceres

**学习价值**
- 对比 Ceres vs GTSAM vs g2o
- 理解自动求导的原理

---

### 17. Kalibr

**定位：** 传感器标定工具箱

**GitHub**：`https://github.com/ethz-asl/kalibr`

**核心功能**
- Camera-IMU 外参标定
- Camera-Camera 外参标定（多目）
- 相机内参标定（畸变模型）

**学习路径**
- 用 Kalibr 标定 Camera-IMU 外参
- 理解手眼标定（Hand-Eye Calibration）的数学原理

---

### 18. RTKLIB

**定位：** GNSS 定位开源库，RTK/PPP 全套工具

**GitHub**：`https://github.com/tomojitakasu/RTKLIB`

**核心功能**
- 单点定位（SPP）
- RTK 定位（浮点解 + 固定解）
- PPP / PPP-RTK
- 支持 RINEX 格式读取

**学习路径**
- 用 RTKLIB 处理一组 RINEX 数据
- 理解 RTK 解算的参数配置
- 对比固定解和浮点解的精度

---

### 19. IMU-TK

**定位：** IMU 内参标定工具

**GitHub**：`https://github.com/Kyle-edwards/IMU-TK`

**核心功能**
- 加速度计内参：零偏、比例因子、非正交性
- 陀螺仪内参：零偏、比例因子、温度漂移

**学习价值**
- 理解 IMU 标定的流程
- 实际动手标定一款 IMU

---

### 20. Cartographer

**定位：** Google 开源的 2D/3D SLAM 系统

**GitHub**：`https://github.com/cartographer-project/cartographer`

**核心特点**
- 2D/3D 同步定位与建图
- 实时闭环检测（Submap + CSM）
- 支持多种传感器：激光雷达、IMU、里程计
- Google 在自动驾驶服务中实际使用

**学习价值**
- 理解 submap 机制和闭环检测
- 理解分支定界（Branch and Bound）加速

---

### 21. RTAB-Map

**定位：** 实时外观定位与建图

**GitHub**：`https://github.com/introlab/rtabmap`

**核心特点**
- 实时回环检测（词袋模型）
- 支持 RGB-D、双目、单目 + IMU
- 适合室内机器人定位

---

## 五、开源项目学习优先级（按方向）

### GNSS/IMU/ODO 融合方向（你的方向）

| 项目 | 优先级 | 原因 |
|---|---|---|
| **Apollo 定位模块** | ⭐⭐⭐⭐⭐ | 直接相关，面试最可能被问到 |
| **GTSAM** | ⭐⭐⭐⭐⭐ | 因子图实践，必备工具 |
| **VINS-Mono** | ⭐⭐⭐⭐ | IMU 预积分学习的最佳参考 |
| **RTKLIB** | ⭐⭐⭐⭐ | GNSS 实战工具 |
| **LIO-SAM** | ⭐⭐⭐ | 因子图融合的参考 |
| **g2o** | ⭐⭐⭐ | 图优化工具对比 |
| **Kalibr** | ⭐⭐⭐ | 传感器标定工具 |

### SLAM / VIO 方向

| 项目 | 优先级 | 原因 |
|---|---|---|
| **ORB-SLAM2/3** | ⭐⭐⭐⭐⭐ | 视觉 SLAM 标杆 |
| **VINS-Mono** | ⭐⭐⭐⭐⭐ | VIO 标杆 |
| **LIO-SAM** | ⭐⭐⭐⭐ | LiDAR-IMU 融合标杆 |
| **FAST-LIO2** | ⭐⭐⭐⭐ | 高速 LiDAR-IMU |
| **LeGO-LOAM** | ⭐⭐⭐⭐ | 轻量 LiDAR SLAM |
| **Cartographer** | ⭐⭐⭐ | 工程化参考 |
| **ROVIO** | ⭐⭐ | 直接法滤波 VIO |
| **OKVIS** | ⭐⭐ | 多目 VIO |
| **SVO** | ⭐⭐ | 高速 VO |

### 通用工具（必学）

| 项目 | 优先级 | 原因 |
|---|---|---|
| **Eigen** | ⭐⭐⭐⭐⭐ | 矩阵运算基础 |
| **Ceres** | ⭐⭐⭐⭐ | 优化求解器 |
| **GTSAM** | ⭐⭐⭐⭐⭐ | 因子图工具 |
| **g2o** | ⭐⭐⭐⭐ | 图优化工具 |
| **PCL** | ⭐⭐⭐⭐ | 点云处理 |

---

## 六、学习路线建议

### 视觉 SLAM 学习路线

```
第 1 步：视觉 SLAM 十四讲过一遍
  → 搭建一个简单的特征点 SLAM 系统

第 2 步：ORB-SLAM2
  → 跑通 + 精读 Tracking 线程
  → 理解词袋回环检测

第 3 步：VINS-Mono
  → 跑通 EuRoC 数据集
  → 精读 IMU 预积分和滑窗部分
  → 对比 ORB-SLAM2（优化 vs 滤波）
```

### 激光 SLAM 学习路线

```
第 1 步：LOAM / A-LOAM 原理
  → 理解点云配准、特征提取

第 2 步：LeGO-LOAM
  → 跑通 KITTI，理解地面分割

第 3 步：FAST-LIO2
  → 跑通室内/室外数据，理解紧耦合

第 4 步：LIO-SAM
  → 因子图框架，理解多传感器融合
```

### 工具库学习路线

```
第 1 步：Eigen（1周）
  → 矩阵运算、四元数、几何变换

第 2 步：GTSAM（2周）
  → 因子图构建、优化求解、自定义因子

第 3 步：RTKLIB（1周）
  → GNSS 数据处理、RTK 定位
```

---

## 六、深度学习 SLAM

> 基于深度学习的端到端 SLAM，是当前学术前沿，也是未来趋势。面试中常被问到"深度学习在 SLAM 中的应用"，了解这些项目能展示你对前沿方向的关注。

### 22. DeepFactors / CodeSLAM / SceneCode

**定位：** 深度学习 + SLAM 的先驱探索，端到端深度估计

**核心项目**

| 项目 | GitHub | 特点 |
|---|---|---|
| **CodeSLAM** | `https://github.com/jmhandy/CodeSLAM` | 深度学习提取稠密几何先验 |
| **DeepFactors** | `https://github.com/jmhandy/DeepFactors` | CodeSLAM 的实际实现 |
| **SceneCode** | `https://github.com/jianzhili/SceneCode` | 语义感知的场景编码 |
| **DeepV2D** | `https://github.com/princeton-vl/DeepV2D` | 深度学习 + SfM，端到端 SfM |

**核心思路**

```
传统 SLAM：
输入 → 特征提取 → 匹配 → 几何估计 → 稠密重建
        ↑ 人工设计          ↑ 几何优化

深度学习 SLAM：
输入 → CNN 特征提取 + 深度估计 → 端到端优化
              ↑ 学习得到        ↑ 可微分
```

**学习价值**
- 理解深度学习如何辅助传统几何 SLAM
- 理解可微分几何优化的概念

---

### 23. Neural Tangent SLAM / DeepRKSLAM

**定位：** 端到端可学习的 SLAM 系统

| 项目 | GitHub | 说明 |
|---|---|---|
| **Neural Tangent SLAM** | 学术论文 | 用 Neural Tangent Kernel 分析 SLAM 系统 |
| **DeepRKSLAM** | `https://github.com/alextobias/deeprkslam` | 深度学习 + 光束法平差的结合 |
| **KgSLAM** | `https://github.com/dzy03/KgSLAM` | 知识图谱 + SLAM |

**核心思路**
- 用神经网络替代传统 SLAM 的部分模块
- 特征提取 → CNN；位姿回归 → MLP；BA → 可微分 BA 层

---

### 24. BAF-VIO / DeepVIO / VINet / Selective-SLAM

**定位：** 深度学习辅助的 VIO

| 项目 | GitHub | 特点 |
|---|---|---|
| **VINet** | `https://github.com/HealthVR/VINet` | 深度学习 + IMU 融合的开创性工作 |
| **BAF-VIO** | `https://github.com/Andrew-Qibin/BAF-VIO` | 深度学习辅助的 VIO Bundle Adjustment |
| **DeepVIO** | 学术实现 | 深度学习 + 视觉惯性里程计 |
| **Selective-SLAM** | `https://github.com/mit-prostrided/ProStrided` | 选择性感知的视觉 SLAM |
| **Kalman Filters Meet Neural Network** | 学术实现 | 神经网络辅助的滤波 |
| **UnDeepVO** | `https://github.com/tlldano/UnDeepVO` | 深度学习单目视觉里程计 |
| **SfMLearner** | `https://github.com/tinghuiz/SfMLearner` | 深度学习单目深度 + 里程计联合学习 |

**核心特点**

```
深度学习 VIO 的几种方式：
1. 深度学习替代特征点提取：SuperPoint + SuperGlue
2. 深度学习替代整个 VO：端到端里程计
3. 深度学习辅助 BA：深度估计提供稠密先验
4. 深度学习辅助 IMU 积分：学习漂移补偿
```

**面试常考点**
- SuperPoint + SuperGlue 的组合（当前最热的特征匹配方案）
- 深度学习 SLAM vs 传统 SLAM 的优缺点
- 深度学习 SLAM 的泛化性问题

---

### 25. SuperPoint / SuperGlue

**定位：** 深度学习特征点检测与匹配，是深度学习 SLAM 的重要组件

**GitHub**
- SuperPoint：`https://github.com/magicleap/SuperPoint`
- SuperGlue：`https://github.com/magicleap/SuperGlue`

**核心特点**

```
SuperPoint：自监督学习的特征点检测网络
  → 替代 ORB / SIFT 的特征检测
  → 在 Magic Leap 数据集上训练
  → 精度和召回率优于传统方法

SuperGlue：图神经网络特征匹配
  → 输入：两组特征点的位置 + 描述子
  → 输出：两组特征点之间的匹配关系
  → 在室内外数据集上均优于传统匹配（RANSAC + NN）
```

**学习价值**
- 这两个项目组合已经替代了传统 ORB 特征点成为很多新 SLAM 系统的选择
- 面试中常被问到：SuperGlue vs RANSAC + FLANN 的区别

---

### 26. ORB-SLAM3 + 深度学习特征

**GitHub**：`https://github.com/UZ-SLAMLab/ORB_SLAM3`

现代 SLAM 系统中，深度学习特征点正在被广泛探索：
- **ORB-SLAM3 + SuperPoint**：用 SuperPoint 替代 ORB 特征点
- **SuperPoint SLAM**：学术项目，结合 SuperPoint 和 ORB-SLAM3
- **D2Net**：`https://github.com/mihaidusmanu/d2-net` 稠密特征匹配

---

### 27. 端到端视觉里程计（End-to-End VO）

**核心项目**

| 项目 | GitHub | 说明 |
|---|---|---|
| **PoseNet** | `https://github.com/alextobias/PoseNet` | 深度学习直接回归位姿的开创性工作 |
| **GeoNet** | `https://github.com/tinghuiz/GeoNet` | 深度估计 + 光流 + 位姿联合学习 |
| **Struct2Depth** | `https://github.com/tensorflow/models/tree/master/research/struct2depth` | 结构感知深度估计 |
| **Monodepth2** | `https://github.com/nianticlabs/monodepth2` | 自监督深度估计，工业界最常用 |
| **MotionBA** | 学术论文 | 深度学习辅助的 Bundle Adjustment |

**核心特点**

```
端到端 VO 的代表架构（CNN + RNN）：
图像序列 → CNN 特征提取 → LSTM 时序建模 → 位姿输出

代表：PoseNet 及其后续改进（VidLoc, LSTM-Pose, GeoNet）
优势：不需要特征工程，端到端
劣势：泛化性差，无法保证几何一致性
```

---

## 七、语义 SLAM

> 语义 SLAM 在传统 SLAM 框架上引入语义理解，实现"知道这是什么"而不仅仅是"知道在哪里"。在自动驾驶和机器人领域有重要应用。

### 28. SemanticFusion / Semantic Segmentation

**定位：** 语义标签与 SLAM 建图融合的先驱

**核心项目**

| 项目 | GitHub | 说明 |
|---|---|---|
| **SemanticFusion** | `https://github.com/ placard/ElasticFusion` | 语义标签融合到 3D 建图 |
| **SemanticFusion (Macadamia)** | `https://github.com/Andy3g/SemanticFusion` | 语义融合到 ElasticFusion |
| **SegMap** | `https://github.com/demelt/segmap` | 语义建图，地图级别的语义表示 |
| **Semantic Visual Localization** | 学术论文 | 语义辅助的视觉定位 |

---

### 29. Semantic SLAM 经典项目

**GitHub**
- `https://github.com/mit-acl/semantic_3d_mapping`
- `https://github.com/qianyizhang/Semantic-SLAM`
- `https://github.com/raulmur/Orb-SLAM2_Semantic_Segmentation`

**主流融合方案**

```
语义 SLAM 的几种实现方式：

方案 1：语义分割 + 传统 SLAM
  → 每一帧做语义分割（Mask R-CNN / DeepLab）
  → 将语义标签融入建图过程
  → 优势：利用现有成熟分割模型
  → 代表：ORB-SLAM2 + Mask R-CNN

方案 2：动态 SLAM（动态物体处理）
  → 检测并过滤动态物体（行人、车辆）
  → 提高动态场景下的鲁棒性
  → 代表：DS-SLAM（ICRA 2018）

方案 3：物体级 SLAM
  → 以物体为核心建立地图
  → 地图更紧凑、可复用
  → 代表：QuadricSLAM、CubeSLAM
```

---

### 30. CubeSLAM / QuadricSLAM

**定位：** 物体级语义 SLAM，用椭圆/超二次曲面表示物体

**GitHub**
- CubeSLAM：`https://github.com/shi9/8/CubeSLAM`
- QuadricSLAM：`https://github.com/ydyth/QuadricSLAM`

**核心特点**

```
CubeSLAM：
- 单目 SLAM + 3D 物体检测
- 用立方体表示物体（9DoF）
- 物体检测（YOLO）+ SLAM 联合优化
- 无需先验 3D 模型

QuadricSLAM：
- 用超二次曲面（双四元数曲面）表示物体
- 更通用的形状表示
- 基于 GTSAM 实现因子图
```

**面试常考点**
- 为什么用椭圆/立方体表示物体？
- 物体级 SLAM 的优势（地图可复用、语义定位）
- 与特征点 SLAM 的区别

---

### 31. DS-SLAM / RDS-SLAM（动态场景）

**定位：** 处理动态场景的语义 SLAM

**GitHub**
- DS-SLAM：`https://github.com/raulmur/Orb-SLAM2_Semantic_Segmentation`
- RDS-SLAM：`https://github.com/s有的/RDS-SLAM`
- Dynamic-SLAM：`https://github.com/Andrew-Qibin/DynamicSegmentation`

**核心特点**

```
动态场景 SLAM 的核心问题：
- 动态物体（行人、车辆）导致匹配错误
- 传统 SLAM 假设场景静态

DS-SLAM 的解决思路：
1. 语义分割网络（SegNet）：识别静态/动态区域
2. 过滤动态特征点：只保留静态点用于匹配
3. 传统 SLAM：ORB-SLAM2 处理剩余静态点
4. 语义建图：3D 点云 + 语义标签

RDS-SLAM：
- 在 DS-SLAM 基础上增加实时性
- 更高效的动态点过滤
```

**面试常考点**
- 动态场景下 SLAM 失效的原因
- 如何用语义分割辅助过滤动态点
- 语义 SLAM vs 传统 SLAM 的区别

---

### 32. Point-LIO / Point2Lidar-SLAM（语义 + LiDAR）

**GitHub**
- Point-LIO：`https://github.com/hku-mars/Point-LIO`
- `https://github.com/YijiaMian/Point2LIDAR-SLAM`

**核心特点**
- 将语义信息融入 LiDAR SLAM
- 语义辅助的地面/物体分割
- 提升 LiDAR SLAM 在复杂场景的鲁棒性

---

### 33. Kimera / Kimera-VIO / Kimera-Semantics

**定位：** MIT 开源的度量语义 SLAM 框架，最完整的语义 SLAM 系统之一

**GitHub**
- Kimera：`https://github.com/MIT-SPARK/Kimera`
- Kimera-VIO：`https://github.com/MIT-SPARK/Kimera-VIO`
- Kimera-Semantics：`https://github.com/MIT-SPARK/Kimera-Semantics`

**核心模块**

```
Kimera 整体框架：
├── Kimera-RPGO          # 鲁棒位姿图优化
├── Kimera-Semantics     # 度量语义 3D 重建
│   ├── 语义分割：PyTorch 语义网络
│   ├── 3D 建图：体素网格（Voxel Hashing）
│   └── 语义融合：3D 网格 + 语义标签
├── Kimera-VIO          # VIO 前端（超像素 + 光流）
└── Kimera-Multi       # 多机器人版本

Kimera-VIO 特点：
- 超像素（SuperPixel）提取特征
- 惯性测量预积分
- 紧耦合优化
- 完整 ROS 接口
```

**需要掌握到什么程度**
- 能跑通 Kimera-VIO（EuRoC 数据集）
- 理解语义建图的流程
- 理解 Voxel Hashing 的 3D 重建方法

---

### 34. SceneCode / Neural Scene Representation

**定位：** 神经场景表示 + SLAM 的前沿探索

**GitHub**
- SceneCode：`https://github.com/jianzhili/SceneCode`
- NeRF-SLAM：`https://github.com/ToniRV/NeRF-SLAM` — 深度学习 + NeRF + SLAM
- NICE-SLAM：`https://github.com/cvg/NICE-SLAM` — 神经隐式编码 SLAM
- iMAP：`https://github.com/ymfan/IMAP`

**核心特点**

```
NeRF + SLAM 是当前最热门的方向：
- NeRF（Neural Radiance Fields）：神经辐射场
- 用神经网络隐式表示 3D 场景
- 优化：使渲染图像与观测图像一致

NeRF-SLAM 的流程：
1. SLAM 系统提供粗糙位姿
2. NeRF 优化场景几何和外观
3. NeRF 提供稠密深度先验
4. 两者联合优化

NICE-SLAM：
- 用分层特征网格（Hierarchical Feature Grid）替代 NeRF
- 可实时运行（10 FPS）
- 稠密建图 + 定位
```

**面试常考点**
- NeRF 的基本原理：体素渲染、神经辐射场
- NeRF 与传统 SLAM 的区别和联系
- 语义 SLAM 的发展趋势

---

## 八、深度学习 SLAM & 语义 SLAM 学习路线

### 学习路线（2周）

```
第 1 步：深度学习特征点（1周）
  → SuperPoint 论文泛读 + 代码跑通
  → SuperGlue 论文泛读 + 代码跑通
  → 对比：ORB vs SuperPoint 的性能差异

第 2 步：深度学习 VIO（1周）
  → VINet 论文泛读
  → BAF-VIO 论文泛读
  → 理解：深度学习如何辅助 VIO

第 3 步：语义 SLAM（选做，有余力学）
  → Kimera-Semantics 跑通
  → CubeSLAM 跑通 + 理解物体级 SLAM

第 4 步：NeRF + SLAM 前沿（选做）
  → NICE-SLAM 论文泛读
  → 理解神经隐式表示在 SLAM 中的应用
```

---

## 九、项目汇总优先级总表（完整版）

### GNSS/IMU/ODO 融合方向（你的方向）

| 项目 | 优先级 | 原因 |
|---|---|---|
| **Apollo 定位模块** | ⭐⭐⭐⭐⭐ | 直接相关，面试最可能被问到 |
| **GTSAM** | ⭐⭐⭐⭐⭐ | 因子图实践，必备工具 |
| **VINS-Mono** | ⭐⭐⭐⭐ | IMU 预积分学习的最佳参考 |
| **RTKLIB** | ⭐⭐⭐⭐ | GNSS 实战工具 |
| **LIO-SAM** | ⭐⭐⭐ | 因子图融合的参考 |
| **Kalibr** | ⭐⭐⭐ | 传感器标定工具 |
| **SuperPoint + SuperGlue** | ⭐⭐ | 了解深度学习 SLAM 前沿 |
| **Kimera** | ⭐⭐ | 有余力可了解 |

### SLAM / VIO 算法方向

| 项目 | 优先级 | 原因 |
|---|---|---|
| **ORB-SLAM2/3** | ⭐⭐⭐⭐⭐ | 视觉 SLAM 标杆 |
| **VINS-Mono** | ⭐⭐⭐⭐⭐ | VIO 标杆 |
| **LIO-SAM** | ⭐⭐⭐⭐ | LiDAR-IMU 融合标杆 |
| **FAST-LIO2** | ⭐⭐⭐⭐ | 高速 LiDAR-IMU |
| **LeGO-LOAM** | ⭐⭐⭐⭐ | 轻量 LiDAR SLAM |
| **Kimera-VIO/Semantics** | ⭐⭐⭐ | 度量语义 SLAM |
| **CubeSLAM** | ⭐⭐⭐ | 物体级语义 SLAM |
| **SuperPoint + SuperGlue** | ⭐⭐⭐ | 深度学习特征匹配 |
| **NICE-SLAM / NeRF-SLAM** | ⭐⭐ | 前沿探索，了解趋势 |
| **Cartographer** | ⭐⭐⭐ | 工程化参考 |

### 通用工具（必学）

| 项目 | 优先级 | 原因 |
|---|---|---|
| **Eigen** | ⭐⭐⭐⭐⭐ | 矩阵运算基础 |
| **Ceres** | ⭐⭐⭐⭐ | 优化求解器 |
| **GTSAM** | ⭐⭐⭐⭐⭐ | 因子图工具 |
| **g2o** | ⭐⭐⭐⭐ | 图优化工具 |
| **PCL** | ⭐⭐⭐⭐ | 点云处理 |
