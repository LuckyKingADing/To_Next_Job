# 3DGS-SLAM 系列深度解析报告

> 本文档包含三个基于 3D Gaussian Splatting 或相关技术的 SLAM 系统的深度分析：
> 1. MASt3R-SLAM（CVPR 2025）
> 2. SplaTAM（CVPR 2024）
> 3. Photo-SLAM（CVPR 2024）

---

# 一、MASt3R-SLAM 深度解析报告

> 项目地址：https://github.com/rmurai0610/MASt3R-SLAM
> 论文：CVPR 2025 — MASt3R-SLAM: Real-Time Dense SLAM with 3D Reconstruction Priors
> 代码仓库位置：`/Users/ading/Code_Projects/Temp_Projects/MASt3R-SLAM`
> 项目定位：基于 MASt3R 视觉基础模型的稠密 SLAM

---

## 1.1 代码架构与流程分析

### 1.1.1 项目整体架构

MASt3R-SLAM 是 2025 年 CVPR 的最新工作，使用 **MASt3R**（Matched 3D Representations）作为视觉前端，替代传统特征点法。MASt3R 是 DUSt3R 的增强版本，结合了匹配和重建能力。

```
┌──────────────────────────────────────────────────────────────┐
│                    main.py (335行)                          │
│                   主进程：数据加载 + 线程调度                  │
├────────────────┬──────────────────┬──────────────────────┤
│  前端推理线程    │  跟踪线程          │   后端优化线程        │
│  (PyTorch)     │  (FrameTracker)   │  (FactorGraph)      │
├────────────────┼──────────────────┼──────────────────────┤
│  MASt3R 模型   │  GA-Net (GN 求解器)│  lietorch Sim3     │
│  ASMK 检索      │  C++ GN 后端      │  滑动窗口 BA        │
├────────────────┴──────────────────┴──────────────────────┤
│  in3d 可视化 (OpenGL)                               │
└─────────────────────────────────────────────────────┘
```

### 1.1.2 核心算法流程

#### 主入口：`main.py`

```python
# main.py 第1-25行
def main():
    # 加载 MASt3R 模型 (ViT-Large)
    model = load_mast3r(device=device)
    # 加载 ASMK 检索数据库
    retriever = load_retriever(model)
    # 加载数据集
    dataset = load_dataset(cfg, args)
    # 创建共享状态
    states = SharedStates()
    keyframes = SharedKeyframes()
    # 启动后端优化线程
    backend_proc = mp.Process(target=run_backend, args=(cfg, model, states, keyframes, K))
    backend_proc.start()
    # 主循环：数据加载 + MASt3R 推理 + 跟踪
    for frame_data in dataset:
        frame = create_frame(frame_data, model)  # MASt3R 推理
        FrameTracker.track(frame)
        keyframes.append_if_kf(frame)
```

#### MASt3R 前端推理：`mast3r_utils.py`

MASt3R 一次性输出稠密的 3D 点、置信度和描述子：

```python
# mast3r_utils.py 第55-79行
@torch.inference_mode
def mast3r_symmetric_inference(model, frame_i, frame_j):
    # 编码：ViT 特征 + 位置编码
    frame_i.feat, frame_i.pos, _ = model._encode_image(frame_i.img, frame_i.img_true_shape)
    frame_j.feat, frame_j.pos, _ = model._encode_image(frame_j.img, frame_j.img_true_shape)

    # 解码：对称推理 (i→j 和 j→i 双向)
    feat1, feat2 = frame_i.feat, frame_j.feat
    res11, res21 = decoder(model, feat1, feat2, pos1, pos2, shape1, shape2)  # i→j
    res22, res12 = decoder(model, feat2, feat1, pos2, pos1, shape2, shape1)  # j→i

    # 提取稠密输出: X(3D点), C(conf), D(desc), Q(desc_conf)
    X, C, D, Q = [r["pts3d"][0], r["conf"][0], r["desc"][0], r["desc_conf"][0] for r in res]
    return X, C, D, Q  # [4, H, W, 3] 等多种尺寸
```

#### 跟踪器：`tracker.py`

`FrameTracker` 实现帧到关键帧的跟踪：

```python
# tracker.py 第28-100行
def track(self, frame: Frame):
    keyframe = self.keyframes.last_keyframe()

    # MASt3R 匹配：获取 3D-3D 对应关系 + 置信度
    idx_f2k, valid_match_k, Xff, Cff, Qff, Xkf, Ckf, Qkf = mast3r_match_asymmetric(
        model, frame, keyframe, idx_i2j_init=self.idx_f2k
    )

    # 几何一致性过滤
    valid_Cf = Cf > C_conf   # 3D 坐标置信度
    valid_Ck = Ck > C_conf
    valid_Q = Qk > Q_conf     # 描述子置信度
    valid_opt = valid_match_k & valid_Cf & valid_Ck & valid_Q

    # Sim3 相机位姿优化 (GN)
    if not use_calib:
        T_WCf, T_CkCf = self.opt_pose_ray_dist_sim3(Xf, Xk, T_WCf, T_WCk, Qk, valid_opt)
    else:
        T_WCf, T_CkCf = self.opt_pose_calib_sim3(...)
```

#### 后端因子图优化：`global_opt.py`

`FactorGraph` 使用 **Gauss-Newton** 求解器（`mast3r_slam_backends`，C++ 扩展）：

```python
# global_opt.py 第12-100行
class FactorGraph:
    def add_factors(self, ii, jj, min_match_frac, is_reloc=False):
        # MASt3R 稠密匹配生成因子
        idx_i2j, idx_j2i, valid_j, valid_i, Qii, Qjj, Qji, Qij = mast3r_match_symmetric(
            self.model, feat_i, pos_i, feat_j, pos_j, shape_i, shape_j
        )
        # 双向置信度过滤
        Qj = sqrt(Qii[batch_inds, idx_i2j] * Qji)
        Qi = sqrt(Qjj[batch_inds, idx_j2i] * Qij)
        # C++ GN 求解
        self.solve_GN_rays()  # 或 solve_GN_calib()
```

### 1.1.3 核心数据结构设计

| 数据结构 | 文件 | 用途 |
|---------|------|------|
| `Frame` (dataclass) | `frame.py:17-32` | 单帧数据容器（图像、3D点、置信度、Sim3位姿） |
| `SharedKeyframes` | `frame.py` | 共享关键帧列表（多进程安全） |
| `SharedStates` | `frame.py` | 线程间状态同步（INIT/TRACKING/RELOC/TERMINATED） |
| `FactorGraph` | `global_opt.py` | 稠密因子图管理 |
| `RetrievalDatabase` | `retrieval_database.py` | ASMK 图像检索数据库（回环检测） |

### 1.1.4 多进程架构

```
主进程 (main.py)
 ├─ 数据加载循环
 ├─ MASt3R 推理
 ├─ FrameTracker.track()
 └─ keyframes 共享状态
     │
     └── Backend 进程 (run_backend)
         ├─ FactorGraph (C++ GN 求解器)
         ├─ 回环检测 (ASMK 检索)
         └─ 全局 BA
```

---

## 1.2 技术亮点与创新点

### 1.2.1 算法创新

1. **MASt3R 替代 ORB 特征**：MASt3R（Matched 3D）一次性输出稠密 3D 点、置信度和描述子，无需传统的特征提取+匹配+三角化流程。ViT-Large 编码器提供极强的语义理解能力。

2. **Sim(3) 统一优化**：使用 Sim3（而非 SE3）允许尺度漂移校正，支持无标定（`use_calib=False`）和在线标定（`use_calib=True`）两种模式。

3. **多进程解耦**：前端推理（GPU）与后端优化（GPU）分进程运行，避免 PyTorch CUDA 内存碎片。

4. **ASMK 图像检索**：使用 Aggregated Selective Match Kernels 实现高效的地点识别，支持回环检测和重定位。

### 1.2.2 与 ORB-SLAM 类系统的本质区别

| 维度 | ORB-SLAM 类 | MASt3R-SLAM |
|------|------------|-------------|
| 特征 | 稀疏 ORB 角点（~1000个） | 稠密像素级 3D（~H×W 个） |
| 匹配 | BoW + 几何验证 | MASt3R 端到端 3D-3D 匹配 |
| 优化变量 | 相机位姿 + 3D点 | 相机位姿（Sim3） |
| 先验 | 无 | MASt3R 深度/位姿先验 |
| 建图 | 稀疏点云 | 稠密 3D 重建（MASt3R 直接输出） |

---

## 1.3 面试问题整理

### 基础概念类

1. **MASt3R 与 DUSt3R 的区别是什么？**
   - 参考答案：DUSt3R 是单目深度估计+位姿的网络，MASt3R 在此基础上增加了匹配能力（双向 3D-3D 对应），支持 SLAM 中的帧间对应关系建立。MASt3R 还引入了置信度和描述子，支持检索任务。

2. **什么是 ASMK 图像检索？它在 SLAM 中有什么用？**
   - 参考答案：ASMK（Aggregated Selective Match Kernels）是一种高效的视觉地点识别方法。通过聚合局部描述子的加权直方图，实现对光照/视角变化的鲁棒匹配。在 SLAM 中用于回环检测和重定位。

### 工程实践类

1. **稠密 SLAM 的计算瓶颈在哪里？MASt3R-SLAM 如何优化？**
   - 参考答案：瓶颈是稠密匹配的内存占用（$HW$ 级别的对应关系）和 GN 求解器的计算复杂度。MASt3R-SLAM 通过多进程分离推理和优化、`img_downsample` 参数降采样、以及 C++ GN 后端（`mast3r_slam_backends`）来优化。

---

# 二、SplaTAM 深度解析报告

> 项目地址：https://github.com/spla-tam/splatam
> 论文：CVPR 2024 — Splat, Track & Map: Real-Time 3D Gaussian Splatting for Dense SLAM
> 代码仓库位置：`/Users/ading/Code_Projects/Temp_Projects/SplaTAM`
> 项目定位：基于 3D Gaussian Splatting 的稠密 RGB-D SLAM

---

## 2.1 代码架构与流程分析

### 2.1.1 项目整体架构

SplaTAM 将 3D Gaussian Splatting（3DGS）引入 SLAM 跟踪与建图流程：

```
┌─────────────────────────────────────────────────────────────┐
│                    scripts/splatam.py (1013行)               │
│                    主程序：数据加载 + SLAM + 3DGS 训练      │
├──────────────────────────────────────────────────────────┤
│  数据集加载 (gradslam_datasets)                              │
│  ├── ReplicaDataset / ScannetDataset / TUMDataset         │
│  ├── NeRFCaptureDataset (iPhone)                         │
│  └── ICLDataset / AzureKinectDataset / RealSenseDataset   │
├──────────────────────────────────────────────────────────┤
│  SLAM 跟踪 + 建图 (splatam.py: 核心循环)                  │
│  ├── 跟踪: 特征匹配 + PnP / ICP                         │
│  ├── 关键帧选择: keyframe_selection_overlap()             │
│  └── 3DGS 建图: GaussianRasterization + 反向传播          │
├──────────────────────────────────────────────────────────┤
│  3D Gaussian Splatting (diff_gaussian_rasterization)     │
│  ├── 前向渲染: GaussianRasterization.cuda_rasterizer     │
│  └── 梯度计算: 自动微分                                  │
└─────────────────────────────────────────────────────────┘
```

### 2.1.2 核心算法流程

#### 主程序入口：`scripts/splatam.py`

```python
# splatam.py 第150-450行（核心循环）
for iteration in tqdm(range(num_images)):
    # Step 1: 加载 RGB-D 帧
    color, depth, intrinsics, w2c = dataset[iteration]
    color = color.cuda(); depth = depth.cuda()

    # Step 2: 跟踪（如果有关键帧）
    if len(gaussians.means3D) > 0:
        # 将高斯投影到当前帧，计算渲染颜色
        rendered_image = rasterize(gaussians, cam_view)
        # 计算光度和深度损失，反向传播更新相机位姿
        loss = l1_loss_v1(rendered_image, color)
        # 更新相机位姿 (可微)
        camera_opt.step()
        camera_scheduler.step()

    # Step 3: 关键帧判定
    if should_add_keyframe():
        # 从当前帧初始化新的 Gaussian
        new_pts, mean3_sq_dist = get_pointcloud(color, depth, intrinsics, w2c)
        # 增加 Gaussian 到场景
        gaussians.increasePcd(new_pts, ...)
        # 关键帧跟踪优化
        for _ in range(tracking_iters):
            optimize_keyframe_tracking(...)

    # Step 4: 地图优化（Mapping）
    if len(gaussians.means3D) > init_kf_count:
        for _ in range(mapping_iters):
            # 渲染 + 损失 + 反向传播
            rendered = rasterize(gaussians, ...)
            loss = l1_loss_v1(rendered, keyframe_image)
            loss.backward()
            # densify + prune
            densify(gaussians)
            prune_gaussians(gaussians)
```

#### Gaussian 初始化：`get_pointcloud()` 函数

```python
# splatam.py 第67-118行
def get_pointcloud(color, depth, intrinsics, w2c, ...):
    # 反投影: pixel → camera coords → world coords
    x_grid, y_grid = meshgrid(...)
    pts_cam = torch.stack((
        (x_grid - CX)/FX * depth_z,  # Xc
        (y_grid - CY)/FY * depth_z,  # Yc
        depth_z                        # Zc
    ), dim=-1)
    # 相机→世界坐标变换
    pts = (w2c^-1 @ pts4)[:, :3]
    # 颜色拼接
    point_cld = torch.cat((pts, colors), -1)
    # 均方距离计算（用于 Gaussian 尺度初始化）
    mean3_sq_dist = depth_z**2 / ((FX+FY)/2)**2
    return point_cld, mean3_sq_dist
```

#### Gaussian 参数初始化：`initialize_params()`

```python
# splatam.py 第120-165行
def initialize_params(init_pt_cld, num_frames, mean3_sq_dist):
    # 3D 位置
    means3D = init_pt_cld[:, :3]
    # 旋转（四元数，默认各向同性）
    unnorm_rots = np.tile([1,0,0,0], (num_pts, 1))
    # 不透明度（可微）
    logit_opacities = torch.zeros((num_pts, 1), dtype=torch.float, device="cuda")
    # 尺度（从深度导出的方差）
    log_scales = torch.log(torch.sqrt(mean3_sq_dist))
    # 球谐系数（颜色，DC + 高频）
    rgb_colors = init_pt_cld[:, 3:6]
    # 相机位姿（作为可优化参数）
    cam_rots = np.tile([1,0,0,0], (1, 1, num_frames))
    params = {means3D, rgb_colors, unnorm_rots, logit_opacities, log_scales, cam_rots, cam_trans}
```

### 2.1.3 核心数据结构设计

| 数据结构 | 文件 | 用途 |
|---------|------|------|
| Gaussian 参数 (dict) | `splatam.py:131-143` | means3D, features, opacity, scaling, rotation |
| Camera (struct) | `utils/slam_helpers.py` | 相机内参+外参，可微优化 |
| GaussianRasterizer | `diff_gaussian_rasterization` | 前向渲染器（CUDA） |
| Keyframe | 内存中的 tensor | 跟踪+建图时的参考帧 |

### 2.1.4 跟踪与建图的核心函数调用链

```
splatam.py 主循环
 ├─ get_pointcloud() → 从 RGB-D 生成点云 + 均方距离
 ├─ initialize_params() → 初始化 Gaussian 参数
 ├─ rasterize() → Gaussian Splatting 前向渲染
 ├─ l1_loss_v1() → 光度损失
 ├─ camera_opt.step() → 相机位姿优化（反向传播）
 ├─ increasePcd() → 添加新 Gaussian
 ├─ densify() → 克隆/分裂高斯
 └─ prune_gaussians() → 移除不透明度过低的 Gaussians
```

---

## 2.2 技术亮点与创新点

### 2.2.1 算法创新

1. **3DGS 替代 NeRF 作为场景表示**：3DGS 使用各向异性高斯椭球替代 NeRF 的体素密度场，渲染速度提升 100 倍以上（analytical rasterization vs. volumetric rendering）。

2. **端到端可微 SLAM**：跟踪和建图都通过反向传播优化，无需显式的 BA 或因子图。相机位姿作为可微参数参与渲染损失的反向传播。

3. **多数据集支持**：TUM-RGBD、Replica、ScanNet、ScanNet++、iPhone NeRFCapture，覆盖室内重建主流数据集。

### 2.2.2 关键参数配置

| 参数 | SplaTAM 配置 | 含义 |
|------|------------|------|
| `tracking_iters` | 40-200 | 跟踪迭代次数 |
| `mapping_iters` | 30-60 | 建图迭代次数 |
| `gaussian_distribution` | isotropic/anisotropic | 高斯各向同性/各向异性 |
| `mean_sq_dist_method` | projective | 从深度推导初始尺度 |

---

## 2.3 面试问题整理

### 基础概念类

1. **3D Gaussian Splatting 的前向渲染是如何工作的？**
   - 参考答案：将 3D 高斯椭球投影到 2D 图像平面，根据像素与高斯中心的距离计算每个像素的颜色贡献（高斯权重 × SH 系数 × 不透明度），对所有高斯求和得到最终像素颜色。使用 tile-based rasterization 加速。

2. **SplaTAM 中 Gaussian 的尺度是如何初始化的？**
   - 参考答案：使用 projective 方法：$\text{scale} = \frac{\text{depth}}{f}$，即深度越大（距离越远），初始尺度越大。这基于透视投影的几何直觉：远处物体在图像上的像素覆盖面积更大。

### 工程实践类

1. **SplaTAM 如何处理动态物体？**
   - 参考答案：当前版本没有显式的动态物体处理机制。Gaussian Splatting 对动态物体会自然建模为场景的一部分，跟踪时可能产生较大误差。可参考 Dynamic 3D Gaussians 论文的方法处理动态场景。

---

# 三、Photo-SLAM 深度解析报告

> 项目地址：https://github.com/HuajianUP/Photo-SLAM
> 论文：CVPR 2024
> 代码仓库位置：`/Users/ading/Code_Projects/Temp_Projects/Photo-SLAM`
> 项目定位：ORB-SLAM3 + Gaussian Splatting 光度级重建

---

## 3.1 代码架构与流程分析

### 3.1.1 项目整体架构

Photo-SLAM 是**架构最为复杂**的 3DGS-SLAM 项目，采用了 C++/CUDA/PyTorch 混合架构，将 ORB-SLAM3 定位能力与 Gaussian Splatting 光度重建深度结合：

```
┌──────────────────────────────────────────────────────────────┐
│                    C++ 主程序 (examples/*.cpp)                   │
│         Replica RGB-D / TUM / EuRoC / RealSense 等入口      │
├──────────────────────────────────────────────────────────────┤
│  ORB-SLAM3 (C++ + Sophus + g2o + DBoW2)                   │
│  负责: Tracking / LocalMapping / LoopClosing               │
│  输出: 相机位姿 + 稀疏特征点地图                             │
├──────────────────────────────────────────────────────────────┤
│  Gaussian Mapper (C++ + PyTorch LibTorch)                   │
│  ├── GaussianModel (C++ 封装 PyTorch)                      │
│  │   └── gaussian_model.cpp (1130行)                      │
│  ├── GaussianScene                                        │
│  ├── GaussianTrainer (训练循环)                           │
│  └── GaussianRenderer (CUDA 光栅化)                      │
│      └── cuda_rasterizer/ (7 个 .h/.cu 文件)              │
├──────────────────────────────────────────────────────────────┤
│  IMGUI Viewer (OpenGL)                                    │
│  └── imgui_viewer.cpp (711行) + imgui 库                  │
└──────────────────────────────────────────────────────────────┘
```

### 3.1.2 核心算法流程

#### C++ 主程序入口：`examples/replica_rgbd.cpp`

```cpp
// examples/replica_rgbd.cpp
int main(int argc, char** argv) {
    // 1. 加载 ORB-SLAM3 词汇表 + 配置
    ORB_SLAM3::System SLAM(vocFile, settingsFile, ORB_SLAM3::System::RGBD, true);

    // 2. 初始化 Gaussian Mapper
    GaussianMapper mapper(SLAM_ptr, gaussian_config, output_dir, seed, device_type);

    // 3. 主循环：读取 RGB-D 图像
    for (;;) {
        cv::Mat imRGB, imDepth, Tcw;
        // 读取图像...

        // 4. ORB-SLAM3 跟踪（获取位姿 + 关键帧）
        Tcw = SLAM.TrackRGBD(imRGB, imDepth, timestamp);

        // 5. 发送到 Gaussian Mapper
        mapper.InsertKeyFrame(kf_ptr);

        // 6. Mapper 后台线程处理（训练+渲染）
        mapper.Run();  // 后台独立线程
    }
}
```

#### Gaussian Mapper：`src/gaussian_mapper.cpp`

```cpp
// gaussian_mapper.cpp 第100-300行
// 后台线程：处理每个新关键帧
void GaussianMapper::InsertKeyFrame(KeyFrame* kf) {
    std::unique_lock<std::mutex> lock(keyframeMutex);
    keyframes.push_back(kf);

    // 为新关键帧生成 Gaussian
    GaussianKeyFrame new_gkf;
    new_gkf.createFromKeyFrame(kf, model_params_);
    gaussians_->increasePcd(new_gkf.points, new_gkf.colors, iteration_);

    keyframeUpdated.notify_one();  // 通知训练线程
}

// 后台训练线程
void GaussianMapper::Run() {
    while (!stopped_) {
        // 等待新关键帧
        std::unique_lock<std::mutex> lock(keyframeMutex);
        keyframeUpdated.wait(lock);

        // 渲染 + 损失计算 + 反向传播
        auto [render, image, depth] = scene_->render(gaussians_, view, training_args_);
        float loss_rgb = loss_utils::l1_loss(render, image);
        float loss_depth = loss_utils::l1_loss(render_depth, depth);
        loss = loss_rgb + lambda_depth * loss_depth;
        loss.backward();

        // densify + prune
        gaussians_->densify(...);
        gaussians_->prune(...);
    }
}
```

### 3.1.3 CUDA 光栅化器

```
cuda_rasterizer/
├── auxiliary.h        — 辅助函数（颜色变换、激活函数）
├── operate_points.h   — 点操作（投影、变换）
├── forward.h / backward.h — 前向/反向传播 kernel
├── rasterizer_impl.h  — 渲染器实现
└── rasterizer.h       — 对外接口
```

### 3.1.4 关键文件一览

| 文件 | 行数 | 核心功能 |
|------|------|---------|
| `src/gaussian_mapper.cpp` | 2055 | Gaussian Mapper 主类，训练循环 |
| `src/gaussian_model.cpp` | 1130 | Gaussian 参数管理 |
| `examples/realsense_rgbd.cpp` | 453 | RealSense 实时入口 |
| `cuda_rasterizer/auxiliary.h` | 165 | CUDA 辅助函数 |
| `viewer/imgui_viewer.cpp` | 711 | OpenGL 可视化 |
| `src/gaussian_trainer.cpp` | 155 | 训练器 |

---

## 3.2 技术亮点与创新点

### 3.2.1 算法创新

1. **ORB-SLAM3 + 3DGS 深度耦合**：ORB-SLAM3 提供高精度的相机位姿作为 3DGS 训练的监督信号，而非让 3DGS 从头学习位姿。这解决了纯 3DGS SLAM 方案中位姿学习不稳定的根本问题。

2. **Stereo SGM 深度补全**：对于单目模式，使用 OpenCV CUDA 的 Stereo SGM（Semi-Global Matching）从双视图生成稠密深度图，弥补单目缺乏深度的问题。

3. **C++/LibTorch 深度集成**：所有 3DGS 代码用 C++ 编写，通过 LibTorch 调用 PyTorch 算子，避免了 Python GIL 限制，实现了真正的实时性能。

4. **闭环感知的建图**：闭环检测触发后重新优化 Gaussian 场景，实现全局一致的光度重建。

### 3.2.2 三类 3DGS-SLAM 对比

| 特性 | SplaTAM | Photo-SLAM | MASt3R-SLAM |
|------|---------|-----------|-------------|
| 定位方式 | 可微渲染损失反向传播 | ORB-SLAM3（传统特征点） | MASt3R + GN 求解器 |
| 建图 | 3DGS（在线优化） | 3DGS（ORB 位姿监督） | MASt3R 3D 先验 |
| 稠密度 | 全像素级 | 依赖关键帧密度 | 全像素级 |
| 多传感器 | RGB-D | RGB-D/单目/双目 | RGB-D |
| 语言 | Python | C++ + LibTorch | Python + C++ GN |
| 建图质量 | 高（端到端） | 高（位姿精确） | 高（基础模型先验） |

---

## 3.3 面试问题整理

### 基础概念类

1. **Gaussian Splatting 中的 tile-based rasterization 是如何工作的？**
   - 参考答案：将图像分成多个 tile（如 16×16 像素），每个 tile 分配一个线程块。遍历每个 Gaussian，将其投影到图像平面后确定覆盖的 tile，对每个 tile 收集所有覆盖的 Gaussian。最后在每个 tile 内对 Gaussian 按深度排序，进行 alpha blending 渲染。

2. **Photo-SLAM 为什么选择 ORB-SLAM3 而非学习式跟踪？**
   - 参考答案：ORB-SLAM3 提供亚像素精度的相机位姿估计和全局闭环修正，是目前最成熟的传统视觉 SLAM 系统。将其作为位姿监督可以避免纯学习方法的位姿漂移问题，同时 3DGS 专注于提升建图质量。

### 工程实践类

1. **如何在 C++ 中集成 PyTorch (LibTorch) 进行 3DGS 训练？**
   - 参考答案：使用 `torch::nn::Module` 封装 Gaussian 参数为可微张量，`torch::autograd` 管理梯度，`torch::nn::functional` 实现损失计算。CUDA kernel 处理光栅化。Photo-SLAM 的 `gaussian_model.cpp` 展示了完整的工程实践。

2. **SplaTAM 和 Photo-SLAM 在 Gaussian 参数管理上的区别是什么？**
   - 参考答案：SplaTAM 使用纯 Python 的 PyTorch tensor 管理 Gaussian 参数，简单直观但速度受 Python GIL 限制。Photo-SLAM 使用 C++ 类 (`GaussianModel`) 封装 PyTorch tensor，通过 LibTorch C++ API 操作，在 C++ 主程序中实现高效实时。

---

# 四、扩展知识图谱（3DGS-SLAM 方向）

### 4.1 关联项目总览

| 项目 | 核心方法 | 发表 |
|------|---------|------|
| **3D Gaussian Splatting** | 各向异性高斯 + tile rasterization | SIGGRAPH 2023 |
| **DynaGS** | 动态场景高斯分裂/合并 | CVPR 2024 |
| **Gaussian-SLAM** | 在线增量高斯建图 | ICRA 2024 |
| **MonGS** | 单目 Gaussian SLAM | IROS 2024 |
| **GsLAM** | 高斯辅助 SLAM | 综述 |

### 4.2 延伸方向

1. **实时性优化**：当前 3DGS-SLAM 的主要瓶颈是渲染和优化速度。可探索：① Tensor Parallelism 多卡并行；② 降采样训练；③ 小型化高斯表示。

2. **动态场景处理**：参考 DynaGS 等工作，增加 Gaussian 的动态/静态分类处理。

3. **大规模场景**：当前方案适合单房间尺度，大规模城市场景需要分层/分块管理高斯。

4. **与 NeRF 的结合**：Photo-SLAM 的 ORB-SLAM3 + GS 范式可推广到 NeRF 作为建图表示（如 Instant-NGP），实现更精细的光度重建。
