# MiniNav — Indoor Mobile Robot Localization & Navigation System

> 一个以 C++ 为核心、面向室内移动机器人的定位与导航系统。
> 从仿真出发,逐步扩展到 odom、EKF 多传感器融合、路径规划、路径跟踪控制,
> 并最终在 Raspberry Pi 5 + 4WD 小车平台上完成室内自主移动验证。

---

## 1. 项目目标

MiniNav 要回答移动机器人导航领域最核心的三个问题:

| 问题         | 描述   | 核心技术                          |
|------------|------|-------------------------------|
| **我在哪?**   | 定位问题 | Wheel odometry、IMU、EKF 多传感器融合 |
| **我要去哪?**  | 规划问题 | Occupancy grid map、A\* 全局路径规划 |
| **我怎么过去?** | 控制问题 | Pure Pursuit 路径跟踪、闭环控制        |

项目不做单点算法演示,而是构建一个**简化但完整**的导航系统,
覆盖从运动学建模 → 状态估计 → 规划 → 控制 → 实车部署的完整链路。

---

## 2. 技术栈

### 2.1 核心语言与标准

- **C++23**,启用 **C++ Modules**(`FILE_SET CXX_MODULES`)
    - 项目的模块化设计是长期投资:更清晰的依赖管理、更快的增量编译、更强的封装。
    - 这也是对 C++ 现代特性的学习实践。

### 2.2 构建与工具链

| 类别   | 工具                                         |
|------|--------------------------------------------|
| 编译器  | Clang 18                                   |
| 构建系统 | CMake 3.28 + Ninja                         |
| 配置   | `CMakePresets.json`(Debug/Release presets) |
| 链接器  | `lld`(`-fuse-ld=lld`)                      |
| IDE  | CLion on WSL(Ubuntu)                       |

### 2.3 核心库

| 库                         | 用途                               | 引入阶段     |
|---------------------------|----------------------------------|----------|
| **Eigen3**                | 矩阵运算、向量、雅可比、协方差矩阵                | V0 重构时引入 |
| **GoogleTest**            | 单元测试框架                           | V0 阶段接入  |
| **Rerun SDK (C++)**       | 实时 3D/2D 可视化,机器人状态与轨迹流式展示        | V0 阶段接入  |
| **ROS2 (Humble / Jazzy)** | 节点化、topic 通信、RViz2 可视化、launch 系统 | V4 阶段引入  |

### 2.4 辅助工具

- **Python**:实验后处理、误差分析(RMSE)、批量实验脚本、参数扫描、论文级图表
- **Git / GitHub**:版本控制、CI(后期)、发布演示
- **Linux / WSL**:开发环境;树莓派端为原生 Ubuntu 22.04/24.04

---

## 3. 系统架构:五层能力模型

MiniNav 的系统架构自底向上分为五层,每一层对应一项可独立验证的机器人核心能力。

```
┌─────────────────────────────────────────────┐
│ Layer 5: Real Robot Deployment              │  Raspberry Pi 5 + 4WD car
├─────────────────────────────────────────────┤
│ Layer 4: Motion Control                     │  Pure Pursuit / PID
├─────────────────────────────────────────────┤
│ Layer 3: Global Planning                    │  Occupancy grid + A*
├─────────────────────────────────────────────┤
│ Layer 2: Localization & State Estimation    │  Odom + IMU + EKF
├─────────────────────────────────────────────┤
│ Layer 1: Kinematic Simulation               │  Differential-drive model
└─────────────────────────────────────────────┘
```

### Layer 1 — 运动学仿真

2D 差分驱动机器人模型。

- **状态**:位姿 `Pose2D = (x, y, yaw)`
- **控制输入**:Twist `(v, w)`,线速度与角速度
- **积分器**:欧拉积分(V0)→ 可扩展至 Runge-Kutta
- **角度规范化**:`wrap_angle` 保证 `yaw ∈ (-π, π]`

### Layer 2 — 定位与状态估计(项目核心)

机器人不再直接读取真实位姿,而是通过带噪声的传感器估计。

- **传感器建模**:
    - Wheel odometry:控制噪声 + 编码器离散化噪声
    - IMU:角速度测量噪声 + 偏置
    - (扩展)landmark 观测或 UWB 位置观测
- **状态估计器**:扩展卡尔曼滤波器 **EKF**
    - 状态向量 `x = [px, py, θ]ᵀ`(V2 基础版)
    - 可扩展至 `[px, py, θ, v, ω, b_ω]ᵀ`(含速度和 IMU bias)
    - 预测:基于运动学模型线性化,构造雅可比 `F`
    - 更新:根据观测类型构造 `H`,执行 Kalman gain 计算与协方差更新

### Layer 3 — 全局路径规划

- **地图表示**:二维 occupancy grid,支持从 PGM/PNG 加载
- **算法**:A\* 搜索,支持曼哈顿与欧几里得启发式
- **膨胀层**:障碍物膨胀保证路径安全距离
- **输出**:一条离散 waypoint 序列,交给控制层

### Layer 4 — 路径跟踪控制

- **控制器**:Pure Pursuit(主)+ PID(备选横向误差控制)
- **输入**:当前 EKF 估计位姿 + 全局路径
- **输出**:`Twist (v, w)` 指令,送回机器人模型(仿真)或底盘驱动(实车)
- **调参点**:look-ahead 距离、最大线速度、最大角速度

### Layer 5 — 实车系统

- **硬件平台**:Raspberry Pi 5 (4GB) + Adeept 4WD Smart Car Kit
- **软件框架**:ROS2(Humble 或 Jazzy)
- **传感器扩展**:根据原装小车硬件能力,考虑补充 IMU(BNO055/MPU6050)与轮式编码器
- **通信拓扑**:PC 端 RViz2 远程监控,Pi 端运行核心节点

---

## 4. 目录结构

```
mininav/
├── CMakeLists.txt
├── CMakePresets.json
├── README.md
├── cmake/
│   └── warnings.cmake              # 编译警告/-Werror 策略
├── config/                         # YAML/JSON 参数配置
├── data/                           # 仿真输出(轨迹 CSV 等)
├── docs/
│   ├── architecture.md             # 架构总览
│   ├── math/
│   │   ├── kinematics.md           # 运动学模型推导
│   │   ├── odom_noise.md           # 噪声建模
│   │   └── ekf.md                  # EKF 完整数学推导
│   ├── experiments/
│   │   ├── v0.md
│   │   ├── v1_odom_drift.md
│   │   └── v2_ekf_vs_odom.md
│   └── design/
│       └── module_graph.md         # C++ 模块依赖图
├── scripts/                        # Python 后处理
│   ├── plot_trajectory.py
│   ├── compute_rmse.py
│   └── run_experiments.py
├── results/                        # 实验产出:图、GIF、报告
├── src/
│   ├── core/                       # 运动学、类型、日志、IO
│   │   ├── types.ixx
│   │   ├── math.ixx
│   │   ├── robot_model.ixx / .cpp
│   │   ├── command_source.ixx / staged_command_source.cpp
│   │   ├── trajectory.ixx
│   │   ├── csv_writer.ixx / .cpp
│   │   └── logger.ixx
│   ├── localization/               # odom、IMU 模型、EKF
│   │   ├── odometry.ixx / .cpp
│   │   ├── imu_model.ixx / .cpp
│   │   └── ekf.ixx / .cpp
│   ├── planning/                   # grid map、A*
│   │   ├── occupancy_grid.ixx / .cpp
│   │   └── astar.ixx / .cpp
│   ├── control/                    # Pure Pursuit / PID
│   │   └── pure_pursuit.ixx / .cpp
│   ├── viz/                        # Rerun 封装
│   │   └── rerun_sink.ixx / .cpp
│   └── apps/                       # 可执行入口
│       ├── sim_v0_main.cpp
│       ├── sim_v1_odom_main.cpp
│       ├── sim_v2_ekf_main.cpp
│       └── sim_full_nav_main.cpp
├── tests/                          # GoogleTest 单元测试
│   ├── robot_model_tests.cpp
│   ├── math_tests.cpp
│   ├── ekf_tests.cpp
│   ├── astar_tests.cpp
│   └── pure_pursuit_tests.cpp
└── ros2_ws/                        # V4 之后:ROS2 工作空间
    └── src/
        ├── mininav_msgs/
        ├── mininav_sim/
        ├── mininav_localization/
        ├── mininav_planning/
        └── mininav_control/
```

---

## 5. 版本路线图

每个版本都是一次完整迭代,而不是推倒重来。版本之间向后兼容,早期代码持续被新版本复用。

### V0 — 理想运动仿真

- 目标:实现差分驱动运动学模型,输入 `(v, w)` 命令序列,输出完整轨迹。
- 关键模块:`RobotModel`、`CommandSource`、`Trajectory`、`CsvWriter`、`Logger`
- 产出:rerun 录屏、Python 对比图、`docs/v0.md` 实验笔记

### V1 — 带噪声的里程计

- 目标:引入控制噪声与编码器噪声,生成带漂移的 odom 轨迹。
- 对比 ground truth 与 odom,量化漂移量与时间的关系。
- 产出:`docs/math/odom_noise.md`、漂移曲线、`docs/experiments/v1_odom_drift.md`

### V2 — EKF 多传感器融合

- 目标:实现扩展卡尔曼滤波器,融合 odom + IMU + 位置观测。
- 手推预测方程、更新方程、雅可比矩阵,完整写入 `docs/math/ekf.md`。
- 三轨迹对比:ground truth / odom / EKF,配 RMSE 表格。
- 参数实验:扫描 Q、R 矩阵,记录效果对比。
- 产出:EKF 技术文档、Rerun 可视化 GIF、参数实验报告

### V3 — 全局路径规划

- 目标:2D occupancy grid 加载、A\* 全局规划、障碍物膨胀。
- 支持从 PGM 图像加载地图;支持手画地图。
- 产出:规划路径可视化、`docs/experiments/v3_planning.md`

### V4 — 路径跟踪控制 + 完整仿真闭环

- 目标:Pure Pursuit 控制器;把 EKF + A\* + 控制串起来。
- 给定目标点,机器人从起点规划、跟踪、走到终点,全流程自动。
- 产出:**完整 sim demo 视频/GIF**(简历首屏素材)

### V5 — ROS2 节点化

- 目标:将核心算法包装为 ROS2 节点。
- 节点拓扑:`simulator_node` → `odom_node` → `ekf_node` → `planner_node` → `controller_node`
- 使用 topic/message 通信,使用 launch 文件统一启动,RViz2 可视化。
- 产出:RViz2 截图、launch 文件、ROS2 版 README

### V6 — 实车部署

- 目标:Pi 5 上运行整套 ROS2 系统,驱动 Adeept 4WD 小车。
- 真实 odom(编码器)、IMU、实车 EKF 调参。
- **Sim-to-real gap 分析**:记录仿真参数在实车上失效的具体表现与解决过程。
- 产出:**实车导航 demo 视频**

### V7 — SLAM 与室内自主导航(Nice-to-have)

- 目标:加入 2D 激光雷达,利用 `slam_toolbox` 或 Cartographer 建图。
- 把建好的图交给 MiniNav 的规划与控制层。
- 产出:真实室内环境建图 + 自主导航 demo

---

## 6. 可视化策略(Rerun.io)

Rerun 是一款开源的多模态数据可视化工具,专为机器人/CV/ML 场景设计。
它的 C++ SDK 允许你在仿真或实车循环中**流式地 log 出**位姿、点云、图像、
坐标系、路径等数据,Rerun Viewer 会实时渲染出一个**专业级的 3D/2D 视图**。

### 为什么选 Rerun?

- **一行代码出图**:不用写 matplotlib、不用自己管坐标系。
- **时间轴原生支持**:可以暂停、回放、倒带,看任意时刻的状态。
- **跨语言**:C++ 和 Python SDK 数据兼容,调试和出图都方便。
- **专业的视觉呈现**:截图或录屏直接可以放进简历和 README,质感远超 matplotlib。

### 在 MiniNav 中的用途

| 阶段  | Rerun 可视化内容                                     |
|-----|-------------------------------------------------|
| V0  | 机器人位姿、轨迹、控制输入时间序列                               |
| V1  | ground truth vs odom 双轨迹对比                      |
| V2  | ground truth / odom / EKF 三轨迹 + 协方差椭圆 + IMU 数据流 |
| V3  | occupancy grid 地图 + A\* 搜索展开过程 + 规划路径           |
| V4  | 完整导航过程:地图 + 路径 + 机器人轨迹 + look-ahead 点           |
| V5+ | 直接接入 ROS2 topic,实车实时可视化                         |

### 接入方式

- 在 `src/viz/rerun_sink.ixx` 中封装一个 `RerunSink` 模块
- 主循环调用 `sink.log_pose(t, pose)`、`sink.log_path(...)` 等接口
- Rerun Viewer 独立进程运行,通过 TCP/内存通道接收数据

> 具体 Rerun C++ SDK 的安装、CMake 集成、基础 API 使用,将在 V0 扩展阶段
> 作为独立的学习任务单独展开。

---

## 7. 测试策略

### 7.1 单元测试(GoogleTest)

- **框架**:GoogleTest,通过 CMake `FetchContent` 引入,避免污染系统
- **目标覆盖范围**:
    - `core`:运动学 step 的直线/原地转/圆周运动正确性、`wrap_angle` 边界
    - `localization`:EKF 预测一致性、雅可比数值验证(有限差分对比)、退化情况
    - `planning`:A\* 在已知地图上的最短路正确性、不可达检测
    - `control`:Pure Pursuit 在直线/圆弧路径上的控制输出合理性
- **组织方式**:每个模块对应一个 test 文件,CMake 使用 `enable_testing()` + `gtest_discover_tests`

### 7.2 集成测试

- 用 Python 脚本驱动可执行程序,比对输出 CSV 与预期轨迹(数值容差)
- 回归测试:锁定若干典型场景的 RMSE 基线,防止重构时退化

### 7.3 持续集成

- GitHub Actions:配置 Clang 18 + CMake + Ninja 的 Ubuntu runner
- 每次 push 自动构建 + 跑单元测试

---

## 8. 语言分工

| 语言         | 模块                                      | 原因           |
|------------|-----------------------------------------|--------------|
| **C++**    | 所有核心算法:运动学、EKF、A\*、Pure Pursuit、ROS2 节点 | 性能、工业实践、面试价值 |
| **Python** | 实验脚本、误差分析、RMSE 计算、matplotlib 报告图、参数扫描   | 后处理灵活、画图方便   |

C++ 负责**系统跑起来**,Python 负责**实验讲清楚**。

---

## 9. 关键产出物

项目完成后,仓库应该包含以下可对外展示的资产:

### 9.1 代码资产

- 完整、模块化、带测试的 C++ 代码库
- ROS2 工作空间(V5 之后)
- Python 实验脚本与 Jupyter notebook(可选)

### 9.2 文档资产

- **README.md**:首屏放 demo GIF,含快速开始、架构图、结果摘要
- **数学推导文档**:`docs/math/ekf.md`、`docs/math/kinematics.md` 等,面试时可以直接拿出来讲
- **实验报告**:每个版本一篇,含图、数据、结论
- **架构文档**:`docs/architecture.md`,系统模块关系图

### 9.3 视觉资产

- V0-V2:Rerun 截图/录屏,含轨迹对比
- V2:EKF 效果 GIF + RMSE 表格
- V4:完整仿真导航 demo GIF
- V6:**实车导航视频**(简历王牌)

### 9.4 叙事资产

每个版本结束时写一篇技术博客(`docs/experiments/vN_*.md`),说清楚:
- 这一版要解决什么问题
- 设计了哪些方案,为什么选这个
- 遇到了什么坑,怎么解决
- 结果怎么样,还能怎么改

**这些博客是面试时最有用的素材。** 面试官问"你这个项目里 EKF 怎么调的",
你可以直接打开 blog 讲,而不是现场硬想。

---

## 10. 项目核心卖点

| 卖点       | 说明                                                        |
|----------|-----------------------------------------------------------|
| **完整**   | 不是单点算法,而是从仿真到实车的完整导航系统                                    |
| **可解释**  | 每一层都有数学推导、设计文档与实验验证                                       |
| **可量化**  | 有图、有 RMSE、有参数扫描、有 sim-to-real 对比                          |
| **可扩展**  | 从 V0 到 V7 的每一步都是向前兼容的迭代,而非重写                              |
| **工程味重** | 现代 C++(modules)、CMakePresets、GoogleTest、CI、ROS2、Rerun 全家桶 |
| **有实车**  | Raspberry Pi 5 + 4WD 小车真实部署,含 sim-to-real 叙事              |

---

## 一句话总结

> MiniNav 是一个以 **现代 C++ 为核心**、面向室内移动机器人的定位与导航系统,
> 从运动学仿真出发,逐步扩展到 odom、EKF 多传感器融合、A\* 规划、Pure Pursuit
> 控制、ROS2 节点化,并最终在 Raspberry Pi 5 小车上完成室内自主导航验证。