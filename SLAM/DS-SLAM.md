# DS-SLAM 深度解析报告

> 项目地址：https://github.com/ivipsourcecode/DS-SLAM
> 论文：DS-SLAM: A Semantic Visual SLAM towards Dynamic Environments (IROS 2018)
> 代码仓库位置：`/Users/ading/Code_Projects/Temp_Projects/DS-SLAM`
> 项目定位：语义动态SLAM（基于SegNet语义分割 + 几何光流一致性验证）

---

## 一、代码架构与流程分析

### 1. 项目整体架构

DS-SLAM 在 ORB-SLAM2 基础上新增了**两个独立线程**：语义分割线程（Segment）和稠密建图线程（PointCloudMapping），形成**五线程架构**：

```
┌──────────────────────────────────────────────────────────────┐
│                    System (src/System.cc)                       │
│         构造时初始化所有线程：                                 │
│         - mpTracker (Tracking*)                               │
│         - mpLocalMapper (LocalMapping*)                       │
│         - mpLoopCloser (LoopClosing*)                        │
│         - mpSegment (Segment*)     ← 新增                     │
│         - mpPointCloudMapping (PointCloudMapping*) ← 新增   │
├─────────────────┬──────────────────┬────────────────────────┤
│    Tracking      │   LocalMapping    │      LoopClosing       │
│    (追踪线程)     │   (局部建图)      │       (闭环检测)       │
├─────────────────┴──────────────────┴────────────────────────┤
│   ┌───────────────┐        ┌──────────────────────────┐     │
│   │  Segment      │        │  PointCloudMapping       │     │
│   │  (语义分割)    │        │  (稠密语义建图)          │     │
│   │  — SegNet/Caffe│       │  — PCL 点云             │     │
│   │  — 跳过1帧推理 │        │  — VoxelGrid 降采样    │     │
│   │  — 32类分割   │        │  — ROS 发布 (5Hz)      │     │
│   └───────────────┘        └──────────────────────────┘     │
├──────────────────────────────────────────────────────────────┤
│  Tracking ↔ Segment: RGB图像推送 + 分割图拉取 (mutex/cv)   │
│  Tracking → PointCloudMapping: 关键帧+语义图推送 (mutex)      │
│  System 中初始化线程并注入依赖: mpSegment->SetTracker()     │
│  System 中初始化线程并注入依赖: mpSegment->SetTracker()     │
└──────────────────────────────────────────────────────────────┘
```

### 2. 核心算法流程

#### System 初始化与线程启动

`System.cc` 构造函数负责初始化并启动全部五个线程，并将 Segment 线程注入到 Tracking 中：

```cpp
// src/System.cc:156-167
// 构造时启动所有线程
mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

// 新增：语义分割线程
mpSegment = new Segment(pascal_prototxt, pascal_caffemodel, pascal_png);
mptSegment = new thread(&ORB_SLAM2::Segment::Run, mpSegment);
mpSegment->SetTracker(mpTracker);  // Segment 持有 Tracker 指针

// 注入依赖
mpTracker->SetLocalMapper(mpLocalMapper);
mpTracker->SetLoopClosing(mpLoopCloser);
mpTracker->SetSegment(mpSegment);  // ← 关键：Tracking 可访问分割结果

// TrackRGBD 统一入口
cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp) {
    // 内部调用 mpTracker->GrabImageStereo()，包含完整的动态剔除流程
    return mpTracker->GrabImageStereo(im, im, timestamp);
}
```

`Segment::Run()` 是独立运行的分割线程，使用 **SegNet**（基于Caffe）在 GPU 上执行像素级语义分割：

```cpp
// src/Segment.cc 第73-112行
void Segment::Run() {
    classifier = new Classifier(model_file, trained_file);  // 加载 SegNet Caffe 模型
    while (1) {
        usleep(1);
        if (!isNewImgArrived()) continue;  // 等待新图像

        if (mSkipIndex == SKIP_NUMBER) {  // SKIP_NUMBER=1，每隔1帧推理一次
            mImgSegment = classifier->Predict(mImg, label_colours);  // SegNet 前向推理
            // 推理耗时打印
            mSkipIndex = 0;
        }
        mSkipIndex++;
        ProduceImgSegment();  // 交换缓冲区，通知 Tracking
        if (CheckFinish()) break;
    }
}
```

SegNet 推理使用 Caffe 后端（`libsegmentation/libsegmentation.cpp:52-91`）：

```cpp
// libsegmentation.cpp 第67-91行
cv::Mat Classifier::Predict(const cv::Mat& img, cv::Mat LUT_image) {
    // 输入预处理
    Preprocess(img, &input_channels);
    // GPU 推理
    net_->Forward();
    // 取出输出层 → [H, W, C] → argmax → 类别标签图
    cv::Point maxId;
    cv::minMaxLoc(class_each_row.row(i), 0, &maxValue, 0, &maxId);
    prediction_map.at<uchar>(i) = maxId.x;  // 类别ID (0-19, 15=person)
    return prediction_map;  // 像素级类别标签图
}
```

#### 线程二：Tracking 与动态物体剔除（核心）

**Tracking 接收分割结果**（`src/Tracking.cc:175-254`）：

```cpp
// GrabImageStereo 是 RGB-D 相机的入口
cv::Mat Tracking::GrabImageStereo(const cv::Mat& imRectLeft, ...) {
    // 等待分割图像就绪
    while (!isNewSegmentImgArrived()) { usleep(1); }  // 阻塞等待

    // 创建 Frame（Frame构造函数中进行几何光流检测）
    mCurrentFrame = Frame(mImGray, mImDepth, timestamp, ...);

    // 语义分割 + 几何验证 → 剔除动态点
    mCurrentFrame.CalculEverything(mImRGB, mImGray, mImDepth,
                                  mpSegment->mImgSegmentLatest);
    // mImgSegmentLatest: 当前分割标签图 (CV_8UC1, 像素值为类别ID)
    // mImgSegment_color_final: 伪彩色分割可视化图

    Track();  // 进入标准 ORB-SLAM2 跟踪流程
}
```

**动态物体剔除的两层策略**：

**第一层：几何一致性验证**（`Frame::ProcessMovingObject`，`src/Frame.cc:339-416`）—— 帧间光流 + 极线约束：

```cpp
// src/Frame.cc:338-416 完整流程
void Frame::ProcessMovingObject(const cv::Mat &imgray) {
    T_M.clear();  // 重置动态点列表

    // Step 1: Shi-Tomasi 角点检测（最多1000个）
    cv::goodFeaturesToTrack(imGrayPre, prepoint, 1000, 0.01, 8, cv::Mat(), 3, true, 0.04);
    // 亚像素精化（搜索窗口 10x10，终止条件 20次迭代/0.03）
    cv::cornerSubPix(imGrayPre, prepoint, cv::Size(10, 10),
                     cv::Size(-1,-1), cv::TermCriteria(CV_TERMCRIT_ITER|20, CV_TERMCRIT_EPS, 0.03));

    // Step 2: 金字塔 LK 光流追踪（窗口 22x22，5层金字塔）
    cv::calcOpticalFlowPyrLK(imGrayPre, imgray, prepoint, nextpoint,
                             state, err, cv::Size(22, 22), 5,
                             cv::TermCriteria(CV_TERMCRIT_ITER|20, CV_TERMCRIT_EPS, 0.01));

    // Step 3: 边界过滤 + 光度一致性验证
    for each光流匹配点:
        if 光流点越界 → state[i]=0
        if 9邻域光度差异和 > 2120 → state[i]=0  (limit_of_check=2120)
        if state[i] 存活 → 保留到 F_prepoint / F_nextpoint

    // Step 4: RANSAC 估计基础矩阵 F（核心！）
    cv::Mat mask = cv::Mat(cv::Size(1, 300), CV_8UC1);  // 最多300个内点
    cv::Mat F = cv::findFundamentalMat(
        F_prepoint, F_nextpoint,     // 输入对应点
        mask,                       // 内点掩码
        cv::FM_RANSAC,             // RANSAC 方法
        0.1,                       // 阈值：像素到极线的最大距离(px)
        0.99                       // 置信度
    );
    // F 为 3x3 矩阵，描述两帧间的对极几何关系

    // Step 5: 极线约束验证（双向过滤）
    // 对 RANSAC 保留的内点，再次做精确极线检查
    for each RANSAC 内点:
        // 计算第二帧中的点 x2 到对应极线 l2 = F · x1 的距离
        A = F[0,0]*x1 + F[0,1]*y1 + F[0,2]
        B = F[1,0]*x1 + F[1,1]*y1 + F[1,2]
        C = F[2,0]*x1 + F[2,1]*y1 + F[2,2]
        d = |A*x2 + B*y2 + C| / sqrt(A²+B²)
        if d <= 0.1 (px): 保留为正常静态点 → F2_prepoint/F2_nextpoint
        else: 违反极线约束 → 可能是动态物体 → 加入 T_M

    // T_M 最终存储：违反极线约束的光流点（动态候选）
    for each光流匹配点（原始列表，不含RANSAC滤除的）:
        if 未通过极线约束 (d > limit_dis_epi=1):
            T_M.push_back(nextpoint[i])
}
```

**极线约束的数学原理**：如果 $x_1$ 和 $x_2$ 是同一个 3D 点 $X$ 在两帧中的投影，则满足 $x_2^T F x_1 = 0$（对极约束），即 $x_2$ 必然落在 $x_1$ 对应的极线 $l_2 = Fx_1$ 上。动态物体上的像素由于独立运动，不满足此约束，因此被检测出来。

**第二层：语义分割验证**（`ORBextractor::CheckMovingKeyPoints`，`src/ORBextractor.cc:1062-...`）—— 双重检查动态点是否落在"人"区域：

```cpp
// Frame.cc 第226-253
void Frame::CalculEverything(..., const cv::Mat &imS) {  // imS = mImgSegmentLatest
    // 检查分割图中是否有"人"类别
    for (m, n) in imS:
        if imS[m,n] == PEOPLE_LABLE:  // PEOPLE_LABLE = 15
            flagprocess = 1; break;

    // 如果有动态物体 + 几何检测到了异常点
    if (!T_M.empty() && flagprocess) {
        flag_mov = mpORBextractorLeft->CheckMovingKeyPoints(
            imGray, imS, mvKeysTemp, T_M);
    }
}

// ORBextractor.cc:1062-...
int CheckMovingKeyPoints(const cv::Mat &imGray, const cv::Mat &imS,
                        std::vector<std::vector<cv::KeyPoint>>& mvKeysT,
                        std::vector<cv::Point2f> T) {  // T = T_M
    // 对每个几何异常点 T[i]，在其邻域(±15px)内检查语义标签
    for (int i = 0; i < T.size(); i++) {
        for (int m = -15; m < 15; m++) {
            for (int n = -15; n < 15; n++) {
                if imS[my, mx] == PEOPLE_LABLE:  // 落在人区域
                    flag_orb_mov = 1;  // 标记为动态ORB点
                    // 删除该ORB特征点
                    DeleteOneRowOfMat(mvKeysT[0], i);
                    break;
            }
        }
    }
}
```

#### 线程三：稠密语义建图（PointCloudMapping）

`PointCloudMapping` 在收到关键帧后生成带语义标签的点云：

```cpp
// src/pointcloudmapping.cc 第88-168行
PointCloud::Ptr generatePointCloud(KeyFrame* kf, cv::Mat& semantic_color,
                                  cv::Mat& semantic, cv::Mat& color, cv::Mat& depth) {
    for (m, n) in depth:
        d = depth[m,n]
        if d < 0.01 || d > 8: continue  // 过滤无效深度

        // 语义检查：邻域搜索 (3步采样, ±20px范围)
        // 如果该点邻域内有 PEOPLE_LABLE → 跳过（不加入点云）
        if semantic[m+i, n+j] == PEOPLE_LABLE: continue

        // 物理坐标反投影
        p.x = (n - Camera::cx) * d / Camera::fx;
        p.y = (m - Camera::cy) * d / Camera::fy;
        p.z = d;

        // 颜色：静态区域用RGB，语义区域用标签伪彩色
        if semantic[m,n] == 0:
            p.rgb = color[m,n];  // 静态背景用真实颜色
        else:
            p.rgb = semantic_color[m,n];  // 动态物体用语义颜色
}
```

### 3. 核心数据结构设计

| 数据结构 | 文件 | 用途 |
|---------|------|------|
| `mImgSegmentLatest` (Mat, CV_8UC1) | `Segment.h:72` | 当前分割标签图（像素值=类别ID） |
| `mImgSegment_color_final` (Mat, CV_8UC3) | `Segment.h:70` | 分割结果伪彩色可视化 |
| `T_M` (vector<Point2f>) | `Frame.cc:346` | 违反极线约束的光流点（几何异常） |
| `mvKeysTemp` (vector<KeyPoint>) | `Frame.cc:323` | ORB特征点（待过滤） |
| `PEOPLE_LABLE = 15` | `ORBextractor.h` | Pascal VOC 类别中"人"的ID |
| `flag_mov` | `Frame.h` | 动态物体是否存在标志 |
| `mbNewSegImgFlag` | `Tracking.h` | 分割图就绪标志 |

### 4. 多线程同步机制

```
Tracking 线程                    Segment 线程
     │                              │
     │──── GetImg(RGB) ─────────────→│  (Tracking 主动推送 RGB 图像)
     │                              │
     │                              │  SegNet Caffe 推理 (~100ms)
     │                              │
     │←─── isNewSegmentImgArrived()──│  (semaphore/flag)
     │                              │
     │──── GrabImageStereo ──────────│
     │   └── Frame.CalculEverything  │
     │       ├── ProcessMovingObject │  (光流+极线，几何)
     │       └── CheckMovingKeyPts   │  (语义分割验证)
     │                              │
     ↓                              ↓
  Tracking 流程                    下一帧推理
```

关键同步手段：
- `mutex + condition_variable` 控制分割图像的消费者-生产者模式
- `mbNewSegImgFlag` 布尔标志
- `mSkipIndex` 每隔1帧推理一次（控制推理频率）
- 关键帧同时分发到 LocalMapping 和 PointCloudMapping 两个队列

### 5. 核心函数调用关系全流程

```
ROS 节点 / 主程序
 └─ System.TrackStereo(frame_left, frame_right, timestamp)
     └─ Tracking.GrabImageStereo()
         ├─ Tracking.GetImg(RGB) → Segment.mImg = RGB
         │
         ├─ 等待: while(!isNewSegmentImgArrived())
         │
         ├─ Frame(imGray, imDepth, ...) [Geometry阶段]
         │   ├─ ExtractORBKeyPoints() → mvKeysTemp
         │   └─ ProcessMovingObject() [几何异常检测]
         │       ├─ goodFeaturesToTrack() → Shi-Tomasi 角点
         │       ├─ calcOpticalFlowPyrLK() → 光流追踪
         │       ├─ 光度一致性验证 → 过滤误匹配
         │       └─ 极线约束验证 → T_M (动态候选)
         │
         ├─ Frame.CalculEverything() [语义验证阶段]
         │   ├─ 检查 imS 中是否有 PEOPLE_LABLE
         │   └─ CheckMovingKeyPoints() [几何+语义双重验证]
         │       ├─ T_M 邻域内检查 → 删除落在人区域的ORB点
         │       └─ DeleteOneRowOfMat() → 从 mvKeysTemp 删除
         │
         ├─ ExtractORBDesp() → 生成描述子
         ├─ ComputeStereoFromRGBD() → 生成初始 MapPoint
         ├─ AssignFeaturesToGrid()
         │
        └─ Tracking.Track()
            ├─ TrackReferenceKeyFrame()
            ├─ TrackLocalMap()
            └─ NeedNewKeyFrame() → InsertKeyFrame()
                ├─ LocalMapping.InsertKeyFrame(pKF)
                └─ PointCloudMapping.InsertKeyFrame(pKF, mImS_C, mImS, mImRGB, mImDepth)
                    └─ generatePointCloud() [语义点云建图, PCL发布到ROS]
```

---

## 二、技术亮点与创新点

### 1. 算法创新

#### (1) 语义+几何双层动态物体检测

DS-SLAM 的核心创新在于**两级过滤机制**：

| 层级 | 方法 | 作用 |
|------|------|------|
| 第一层（几何） | 光流LK + 极线约束 | 检测帧间显著运动的区域，不依赖语义 |
| 第二层（语义） | SegNet 分割 "person" | 区分真正的动态物体（人）vs 场景变化（光照、阴影） |

只有两层的判断**重合**时，才判定为动态物体并删除对应特征点。这种设计兼顾了：
- 几何层不怕漏检（任何显著运动都能检测）
- 语义层不怕误检（只有"人"才被删除，静态物体不会被误伤）

#### (2) 稠密语义点云地图

区别于传统 SLAM 只输出稀疏特征点地图，DS-SLAM 额外构建了**稠密点云地图**（基于 PCL）。在收到关键帧后，`PointCloudMapping` 线程执行以下操作：

1. **稠密反投影**：遍历 RGB-D 深度图每个像素，通过相机内参反投影到 3D 世界坐标
2. **语义过滤**：邻域搜索（3步采样 + ±20px）检查是否为 `PEOPLE_LABLE`，若是则跳过不加入地图
3. **VoxelGrid 降采样**：`voxel.setLeafSize(resolution, resolution, resolution)` 将点云体素化
4. **统计离群点过滤**：`sor.setMeanK(50)` + `sor.setStddevMulThresh(1.0)` 移除噪声点
5. **坐标变换**：将点云从相机坐标系变换到世界坐标系后发布

```cpp
// pointcloudmapping.cc:67-76
PointCloudMapping::PointCloudMapping(double resolution_) {
    this->resolution = resolution_;
    voxel.setLeafSize(resolution, resolution, resolution);   // 体素降采样
    this->sor.setMeanK(50);                                     // 统计离群点参数
    this->sor.setStddevMulThresh(1.0);
    globalMap = boost::make_shared<PointCloud>();
    viewerThread = boost::make_shared<thread>(...);  // 后台发布线程
}
```

ROS 入口（`ros_tum_realtime.cc`）通过 `SLAM.mpTracker` 和 `SLAM.mpSegment` 直接访问内部线程的数据成员来采集各环节耗时：

```cpp
// ros_tum_realtime.cc:143-155
while (ros::ok() && ni < nImages) {
    imRGB = cv::imread(...);
    imD = cv::imread(...);
    double tframe = vTimestamps[ni];

    // ORB-SLAM2 统一入口，同时包含 SegNet 分割 + 几何检测 + 跟踪
    Camera_Pose = SLAM.TrackRGBD(imRGB, imD, tframe);

    // 采集各阶段耗时
    orbTimeTmp      = SLAM.mpTracker->orbExtractTime;    // ORB 提取 ~30ms
    movingTimeTmp   = SLAM.mpTracker->movingDetectTime; // 几何检测 ~10ms
    segmentationTime = SLAM.mpSegment->mSegmentTime;    // SegNet 推理 ~100ms
    // 最终输出: 相机位姿(Camera_Pose) + 稠密点云(RVIZ)
}
// 统计平均耗时
cout << "mean segmentation time  = " << segmentationTime/nImages << " ms" << endl;
cout << "mean moving detection    = " << movingTotalTime/nImages   << " ms" << endl;
cout << "mean orb extract time   = " << orbTotalTime/nImages     << " ms" << endl;
```

### 2. 工程实践亮点

1. **SegNet Caffe GPU推理**：使用 Caffe 后端在 GPU 上执行语义分割，每帧分割耗时约 100ms
2. **推理跳帧**：每处理1帧，跳过1帧做分割（`SKIP_NUMBER=1`），节省计算资源
3. **邻域采样降计算量**：点云建图时用3步采样 + ±20px范围搜索检查语义标签
4. **双缓冲交换**：`ProduceImgSegment()` 使用 swap 技术避免内存拷贝

### 3. 与 YOLOv8-ORB-SLAM3 的对比

| 对比维度 | DS-SLAM | YOLOv8-ORB-SLAM3 |
|---------|---------|------------------|
| 分割网络 | SegNet (Caffe) | YOLOv8-Seg (OpenCV DNN / ONNX) |
| 动态检测 | 几何(光流+极线) + 语义(SegNet) | 仅语义(YOLOv8 mask) |
| 精度 | 类别级分割 | 实例级分割掩码 |
| 速度 | ~100ms/帧 | ~30-50ms/帧 |
| 框架基础 | ORB-SLAM2 | ORB-SLAM3 |
| 建图 | OctoMap稠密 | 无 |
| 动态物体 | 仅person | 仅person |
| 代码质量 | 工程代码 | 原型代码 |

---

## 三、面试问题整理

### 基础概念类（校招/初级）

1. **什么是极线约束（Epipolar Constraint）？如何在代码中使用它检测动态物体？**
   - 参考答案：极线约束指如果点 $x_1$ 在第一帧成像，对应的三维点 $X$ 在第二帧必然落在极线 $F \cdot x_1$ 上。实际代码（`Frame.cc:388-396`）：对光流追踪到的对应点 $x_2$，计算其到极线的距离 $d = |F \cdot [x_2,y_2,1]| / \sqrt{A^2+B^2}$，如果距离大于阈值(0.1)说明违反极线约束，该点可能是动态物体。

2. **SegNet 和 YOLOv8 在语义分割上有什么区别？**
   - 参考答案：SegNet 是编码器-解码器结构（基于VGG16），通过 max-pooling 索引进行上采样，保持了空间精度，但参数量大（~120MB）。YOLOv8 是 Anchor-free 单阶段检测+分割，端到端更快（3-5倍），但分割精度略低。

3. **光流法和特征点法SLAM在动态场景中各有什么优劣？**
   - 参考答案：光流法逐像素跟踪，可检测更细粒度的运动但计算量大；特征点法只处理显著角点，计算高效但可能漏检小或慢速运动物体。DS-SLAM 组合两者：光流用于几何验证（动态），特征点用于SLAM跟踪（精度）。

### 工程实践类（社招/中级）

1. **DS-SLAM 中的"邻域检查"策略（±15px、±20px）有什么意义？**
   - 参考答案：ORB特征点定位在角点中心，语义分割的mask边界有误差。如果只在精确坐标检查容易漏检（在mask边缘的特征点）或误检（特征点接近但不在mask内）。使用邻域检查提高了鲁棒性，±15px 对应约 0.5-1° 的角点定位容差。

2. **如何优化 DS-SLAM 的实时性瓶颈（SegNet 推理 ~100ms）？**
   - 参考答案：① 用更轻量的分割网络（如MobileNet-DeepLabV3+）；② 异步流水线（分割和跟踪并行）；③ 感兴趣区域（ROI）分割——只对SLAM跟踪质量差的区域做分割；④ 知识蒸馏压缩大模型。

3. **为什么要在几何验证后再做语义验证，而不是直接用语义结果？**
   - 参考答案：纯语义分割有局限性：① 漏检（遮挡、分裂）；② 类别限制（只处理person）；③ 分割延迟。光流几何验证可以**兜底**——即使语义分割失败，任何显著帧间运动的点都会产生 T_M，后续局部地图优化时可降低其权重。语义层则负责**去假**——过滤掉非动态的运动（如相机快速旋转导致的全局光流）。

---

## 四、扩展知识图谱

### 前置知识

```
必备基础
├── ORB-SLAM2 架构：Tracking / LocalMapping / LoopClosing 三线程
├── 光流法：Lucas-Kanade (LK)、金字塔光流
├── 对极几何：本质矩阵F、极线约束、三角化
├── 语义分割：SegNet 编码器-解码器、空洞卷积
└── 八叉树地图：OctoMap、占据概率

深度学习基础
├── CNN 分割网络：SegNet、FCN、DeepLabV3+
├── 目标检测：YOLO、R-CNN
└── 实例分割：Mask R-CNN、YOLOv8-Seg

工程基础
├── Caffe 框架：Protobuf 模型加载、GPU 前向推理
├── ROS 多线程：nodehandle、publisher/subscriber、service
└── PCL 点云库：VoxelGrid 滤波、transformPointCloud
```

### 关联项目

| 项目 | 特点 | 与本项目关系 |
|------|------|-------------|
| **ORB-SLAM2** | 基础SLAM框架 | 直接继承 |
| **YOLOv8-ORB-SLAM3** | 实例分割+SLAM | 同方向，新一代方法 |
| **SemanticFusion** | CNN+OctoMap融合 | 稠密语义建图参考 |
| **MaskFusion** | 实例分割+动态SLAM | 更精细的动态处理 |
| **Detectron2-SLAM** | Detectron2实例分割 | 高精度但更慢 |

### 延伸方向

1. **实时语义分割**：替换 SegNet 为轻量级网络（ICNet、ESPNet、MobileNetV3+DeepLabV3+）
2. **多类别动态物体**：扩展到车辆、动物等而非仅 person
3. **动态物体跟踪**：不仅删除动态点，还跟踪动态物体轨迹（用于避障）
4. **动态地图更新**：动态物体消失后，原区域应重新激活特征点生成
