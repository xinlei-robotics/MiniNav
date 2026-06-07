# MiniNav — Indoor Mobile Robot Localization & Navigation System

> 一个以现代 C++ 为核心、面向室内移动机器人的定位与导航系统。
> 从差分驱动运动学仿真出发,逐步叠加噪声建模、EKF 多传感器融合、
> 路径规划、跟踪控制、ROS 2 节点化,并最终在 Raspberry Pi 5 + 4WD
> 小车平台上完成室内自主移动的实车闭环。

> **当前进度(截至 2026-06):V0 / V1 / V2 已完成**,V3(全局路径规划)
> 为下一个里程碑。本文档既是项目总愿景,也是版本路线图——已完成版本
> (✅)的描述对齐仓库真实状态,未完成版本(V3–V7)是**前瞻规划**,
> 其模块名、目录、量化指标均为设计意图,可能随实现调整。

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
每一个版本都是一次完整迭代,而不是推倒重来——版本之间向后兼容,
早期代码持续被新版本复用。

---

## 2. 技术栈

### 2.1 核心语言与标准

- **C++23**,启用 **C++ Modules**(`FILE_SET CXX_MODULES`)
  - 模块化设计带来更清晰的依赖管理、更快的增量编译、更强的封装。
  - 项目以"封闭代码库"形态运行(没有外部库 import 它),
    工具链风险可控,适合作为现代 C++ 编译模型的实践场。

### 2.2 构建与工具链

| 类别   | 工具                                         |
|------|--------------------------------------------|
| 编译器  | Clang 18                                   |
| 构建系统 | CMake 3.28 + Ninja                         |
| 配置   | `CMakePresets.json`(Debug/Release presets) |
| 链接器  | `lld`(`-fuse-ld=lld`)                      |
| 测试   | GoogleTest + CTest(`gtest_discover_tests`) |
| CI   | GitHub Actions(ubuntu-24.04 + clang-18)    |
| IDE  | CLion on WSL(Ubuntu 24.04)                 |

### 2.3 核心库

| 库                         | 用途                               | 引入阶段                  |
|---------------------------|----------------------------------|-----------------------|
| **Eigen3**                | 矩阵运算、向量、雅可比、协方差矩阵                | V0(系统级安装)             |
| **GoogleTest**            | 单元测试框架                           | V0(FetchContent)      |
| **Rerun SDK (C++)**       | 实时 3D/2D 可视化,机器人状态与轨迹流式展示        | V0(FetchContent 混合模式) |
| **CLI11**                 | 命令行参数解析(`--seed` / `--preset` 等) | V1(FetchContent)      |
| **yaml-cpp**              | 地图与控制器参数的外部配置文件                  | V3 引入                 |
| **spdlog**                | 替换 V0 内置 logger,结构化日志            | V3 引入                 |
| **gmock**                 | viz 等需要 mock 第三方依赖的模块            | V3 引入                 |
| **ROS 2 (Jazzy Jalisco)** | 节点化、topic 通信、RViz2 可视化、launch 系统 | V4 引入                 |

> **当前已集成**:Eigen3、GoogleTest、Rerun SDK、CLI11(V0–V2)。
> yaml-cpp / spdlog / gmock / ROS 2 是 V3+ 的**规划项,尚未引入**。
>
> **依赖管理策略**:Eigen 用系统包(header-only 共享高效)、GoogleTest
> 用纯 FetchContent(ABI 风险)、Rerun 与 CLI11 用 `FetchContent` +
> `FIND_PACKAGE_ARGS` 混合(本机有装走 find、否则 fetch)。不同模式对应
> 不同第三方库的工程形态,没有"银弹策略"。

### 2.4 辅助工具

- **Python**:实验后处理、误差分析(RMSE)、批量实验脚本、参数扫描、论文级静态图表
- **Git / GitHub**:版本控制、CI、发布演示
- **Linux / WSL**:开发环境;树莓派端为原生 Ubuntu 24.04

---

## 3. 工程实践与设计原则

MiniNav 的代码组织遵循一组贯穿所有版本的设计原则,它们决定了
"新增功能时不需要重写旧代码"这一关键性质。

### 3.1 版本化的状态结构 + ADL 自由函数

每一个版本都有自己的快照结构 `SimStateV0` / `SimStateV1` / ...,
不追求"一个万能 SimState"。新版本通过**新文件中的重载**扩展功能,
而不是修改已有版本的代码:

- 数据结构:`SimStateV0`(t, pose, twist)→ `SimStateV1`(+truth_pose、
  odom_pose、encoder_ticks)→ `SimStateV2`(+imu_omega、6 维 ekf_mean、
  6×6 ekf_cov、nis_encoder、nis_imu)
- 序列化:`csv_row(SimStateV0)` / `csv_row(SimStateV1)` / ... ,
  通过 Argument-Dependent Lookup(实参依赖查找)在编译期解析
- 可视化:`log_to_rerun(SimStateV0, ...)` / `log_to_rerun(SimStateV1, ...)`
  同样通过 ADL 扩展

容器层用 `Trajectory<T>` 模板复用——"行为相同、类型不同"用模板,
不用继承。

### 3.2 依赖倒置:估计器只认接口、不认来源

`WheelOdometry::update(EncoderTicks, dt)` 这个接口签名**不知道**
ticks 来自仿真还是 GPIO 中断。仿真编码器产生的 `EncoderTicks` 与
V6 实车 Pi 5 GPIO 中断累加产生的 `EncoderTicks` 是**同一个 struct**,
估计器代码在 sim→real 切换时**一行都不用改**。

CMake 层面把这条原则物理化:`mininav_sensors` 与
`mininav_localization` 是两个互不依赖的静态库,只通过 `core` 中
的 plain struct 间接耦合。

### 3.3 PIMPL 隔离重型第三方依赖

`viz` 静态库的头文件**不** `#include <rerun.hpp>`,只通过
`std::unique_ptr<Impl>` 前向声明持有实现。Rerun 的具体类型只出现
在 `.cpp` 里。下游 target(`sim_v0` / `sim_v1` / 未来的
`mininav_node`)的编译时间不受 Rerun 头文件大小影响,符号也不被
污染。这同时保证了"在树莓派上跑无可视化版本"时,核心算法库
不需要 `#ifdef` 大改。

### 3.4 双轨产出:CSV(确定性)与 Rerun(交互式)共存

`.rrd` 二进制文件无法在 git 中 diff,不能作为回归基线。CSV 文本格式
是回归测试的天然载体,同时可被 Python pandas 直接读取做误差分析。
两者共存而不替代,各承担不同的"读者-用途"组合——这一点在第 7 节
可视化策略里展开。

### 3.5 严格警告 + 第三方库 SYSTEM 隔离

Debug 模式开 `-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion
-Werror`。这种强度在工业代码里能抓出 `int → size_t` 等隐式转换的
潜在 bug。对项目自己的代码严格、对 Rerun / Eigen 等第三方库用
`SYSTEM include` 屏蔽——是"对自己严格、对他人宽容"的工业标准实践。

### 3.6 可复现实验框架

`RngFactory` 通过 FNV-1a 64-bit 字符串哈希派生每个噪声源的独立
子种子。每个噪声源(actuator、encoder slip 左、encoder slip 右、
未来的 IMU 噪声)各自拥有独立 RNG。`--seed` 控制全局复现性,
新增噪声源不会扰动已有序列。CSV 文件头嵌入 metadata
(seed / preset / dt / duration / generated_at),任何一次仿真都可
精确回放。

---

## 4. 系统架构:五层能力模型

MiniNav 的系统架构自底向上分为五层,每一层对应一项可独立验证的
机器人核心能力。

```
┌─────────────────────────────────────────────┐
│ Layer 5: Real Robot Deployment              │  Raspberry Pi 5 + 4WD car
├─────────────────────────────────────────────┤
│ Layer 4: Motion Control                     │  Pure Pursuit / PID
├─────────────────────────────────────────────┤
│ Layer 3: Global Planning                    │  Occupancy grid + A*
├─────────────────────────────────────────────┤
│ Layer 2: Localization & State Estimation    │  Sensors + Odom + IMU + EKF
├─────────────────────────────────────────────┤
│ Layer 1: Kinematic Simulation               │  Differential-drive model
└─────────────────────────────────────────────┘
```

### Layer 1 — 运动学仿真

2D 差分驱动机器人模型。

- **状态**:位姿 `Pose2D` = `Eigen::Vector2d position` + `double yaw`
  (`yaw` 与 `(x, y)` 在数学上不是同质的,不强行统一为 `Vector3d`,
  通过 `to_vector() / from_vector()` 桥接 EKF 状态向量)
- **控制输入**:Twist2D `(v, w)`,线速度与角速度
- **积分器**:V0 用一阶欧拉;V2 在 `core/integrators` 引入 `rk4_step`,
  `differential_drive_step`(真值通道)现已委托给它。EKF 过程模型另有
  一个可选的 Euler/RK4 开关(归因实验用,默认 RK4),与真值共用同一
  `rk4_step` 内核(`ekf_integrator_consistency_tests` 守护一致性)。
- **角度规范化**:`wrap_angle` 保证 `yaw ∈ (-π, π]`

### Layer 2 — 定位与状态估计(项目核心)

Layer 2 在 CMake 层面物化为**两个互不依赖**的静态库
`mininav_sensors` 与 `mininav_localization`,通过 plain struct
(`EncoderTicks` + 标量陀螺读数)在 app 主循环里间接耦合。

**`sensors` 子层**:
- `ActuatorModel`:Thrun *Probabilistic Robotics* §5.3 Velocity
  Motion Model,把命令 twist 映射为带噪声的真实速度。方差与
  $(v^2, \omega^2)$ 关联,**静止命令下方差归零**(无静止漂移)。
- `WheelEncoderModel`:"先打滑(乘性高斯)后量化(整数 tick)" 的
  物理因果链;通过累计弧长 + 差分输出正确处理低速量化欠采样。
- `ImuModel`(V2 引入):角速度测量 + 白噪声 + 慢漂 bias。

**`localization` 子层**:
- `WheelOdometry`:`update(EncoderTicks, dt) → Pose2D`。
  完全依赖倒置,不知道 ticks 来自哪里。
- `Ekf`(V2 引入):6 维状态 `x = [px, py, θ, v, ω, b_ω]ᵀ`,
  扩展状态包含陀螺零偏 `b_ω`。每步 `predict → update_encoder →
  update_imu`;编码器与陀螺都作为**对隐状态的观测**(而非控制输入),
  Joseph-form 协方差更新,解析 Jacobian 经有限差分校验。

### Layer 3 — 全局路径规划

- **地图表示**:二维 occupancy grid,支持从 PGM/PNG 加载
- **算法**:A\* 搜索,曼哈顿与欧几里得启发式
- **膨胀层**:障碍物膨胀保证路径安全距离
- **接口风格**:`GlobalPlanner` 类的签名提前对齐
  `nav2_core::GlobalPlanner` 的形态(不依赖 ROS 消息类型,使用项目
  自己的 `Path` 类型),V4 引入 ROS 2 时只需要薄薄一层适配器
- **配置**:V3 同步引入 yaml-cpp,地图路径、膨胀半径、启发式选择
  等参数从 YAML 加载

### Layer 4 — 路径跟踪控制

- **控制器**:Pure Pursuit(主)+ PID(备选横向误差控制)
- **接口风格**:`Controller` 类签名对齐 `nav2_core::Controller`
- **输入**:当前 EKF 估计位姿 + 全局路径
- **输出**:`Twist2D (v, w)` 指令,送回机器人模型(仿真)或底盘
  驱动(实车)
- **调参点**:look-ahead 距离、最大线速度、最大角速度
  (V3 引入的 yaml-cpp 同样承载这些参数)

### Layer 5 — 实车系统

- **基础平台**:Raspberry Pi 5 (4GB) + Adeept 4WD Smart Car Kit
  的机械底盘与外壳
- **传感器加装**:
  - **IMU**:**BNO055**(I2C,板载传感器融合,V6 sim-to-real
    调参时省力)
  - **编码器**:原车 N20 电机**没有编码器**,需替换为带编码器的
    电机(光电或霍尔均可,以分辨率与价格权衡)
- **软件框架**:ROS 2 Jazzy Jalisco
- **通信拓扑**:PC 端 RViz2 远程监控,Pi 端运行核心节点

---

## 5. 目录结构

下面是**当前真实结构**(截至 V2)。标注 `(规划)` 的条目尚不存在,
是 V3+ 的设计意图。

```
mininav/
├── CMakeLists.txt
├── CMakePresets.json
├── README.md  CHANGELOG.md  CONTRIBUTING.md  AGENTS.md  CLAUDE.md
├── compile_commands.json
├── requirements.txt                # Python 依赖(rerun-sdk 等)
├── cmake/
│   ├── warnings.cmake              # 严格警告策略(INTERFACE lib)
│   ├── google_test.cmake           # GoogleTest 引入
│   ├── rerun.cmake                 # Rerun SDK 混合模式引入
│   └── cli11.cmake                 # CLI11 引入
├── .github/workflows/ci.yml        # GitHub Actions 工作流
├── config/                         # 空占位,YAML 参数配置 (规划, V3 起)
├── data/                           # 仿真输出:traj.csv / traj_v1.csv / traj_v2*.csv
├── docs/
│   ├── project-overview.md         # 本文档:项目总愿景与版本路线
│   ├── project-management.md       # issue / 看板 / milestone 约定
│   ├── v0_summary.md               # 各版本阶段性总结
│   ├── v1_summary.md
│   ├── v2_summary.md
│   ├── math/
│   │   ├── odom_noise.md           # Velocity Motion Model + 编码器物理/量化 (V1)
│   │   ├── EKF_Foundations.md      # EKF 预测/更新、Jacobian、Joseph form (V2)
│   │   └── runge_kutta_integration.md  # RK4 过程积分及其解析 Jacobian (V2)
│   └── experiments/
│       └── v2_ekf_fusion.md        # 20-seed EKF-vs-odom 定量报告 (V2)
├── scripts/                        # Python 后处理(按版本组织)
│   ├── plot_trajectory.py          # V0 出图
│   ├── v1/analyze_drift.py         # V1 漂移分析
│   └── v2/                         # V2 EKF 分析
│       ├── analyze_ekf.py          #   三轨迹 / RMSE / NIS / 3σ / bias
│       ├── analyze_covariance.py   #   协方差椭圆演化 + 动画
│       ├── analyze_integrator.py   #   RK4-vs-Euler 单 seed 对
│       └── sweep_integrator.py     #   RK4-vs-Euler 多 seed 平均
├── results/                        # 实验产出:results/v{0,1,2}/ 下的 PNG / GIF
├── src/
│   ├── core/                       # 运动学、类型、Trajectory、CSV、随机数、积分器
│   │   ├── types.{ixx,cpp}         # Pose2D / Twist2D / EncoderTicks / SimStateV{0,1,2}
│   │   ├── math.ixx                # wrap_angle, kPi
│   │   ├── kinematics.{ixx,cpp}    # differential_drive_step + inverse/forward
│   │   ├── integrators.{ixx,cpp}   # rk4_step(V2 引入,真值与 EKF 共用)
│   │   ├── robot_model.{ixx,cpp}
│   │   ├── command_source.ixx + staged_command_source.cpp
│   │   ├── trajectory.ixx          # Trajectory<T> 模板
│   │   ├── csv_format.{ixx,cpp}    # csv_header / csv_row 重载
│   │   ├── csv_writer.ixx          # write_csv<T> 模板
│   │   ├── random.ixx              # RngFactory + FNV-1a tag 派生
│   │   └── logger.ixx              # (规划) V3 替换为 spdlog 封装
│   ├── sensors/                    # 独立静态库:执行 + 观测噪声模型
│   │   ├── actuator_model.{ixx,cpp}
│   │   ├── wheel_encoder.{ixx,cpp}
│   │   └── imu_model.{ixx,cpp}     # V2 引入:gyro 白噪声 + 可漂移 bias
│   ├── localization/               # 独立静态库:估计器
│   │   ├── wheel_odometry.{ixx,cpp}
│   │   ├── ekf_state.ixx           # V2:Vec6/Mat6、StateIdx、EkfState6
│   │   ├── ekf.{ixx,cpp}           # V2:6D EKF(predict + encoder/imu update)
│   │   └── encoder_observation.{ixx,cpp}  # V2:解码 z 与推导 R
│   ├── viz/                        # PIMPL 隔离 Rerun
│   │   ├── rerun_sink.{ixx,cpp}
│   │   └── sim_state_log.{ixx,cpp} # log_to_rerun(SimStateV{0,1,2}, ...)
│   ├── apps/                       # 可执行入口(各版本并存,作为回归基线)
│   │   ├── sim_v0_main.cpp
│   │   ├── sim_v1_main.cpp
│   │   └── sim_v2_main.cpp
│   ├── planning/                   # (规划) V3:occupancy_grid + astar
│   └── control/                    # (规划) V4:pure_pursuit
├── tests/                          # GoogleTest,按子库组织
│   ├── core/                       # math / kinematics / robot_model / trajectory / types / random
│   ├── sensors/                    # actuator / wheel_encoder / imu_model
│   ├── localization/               # wheel_odometry + 8 个 EKF 测试 + encoder_observation
│   ├── planning/                   # (规划) V3
│   └── control/                    # (规划) V4
└── ros2_ws/                        # (规划) V4:ROS 2 工作空间
    └── src/  ├ mininav_msgs ├ mininav_sim ├ mininav_localization
              ├ mininav_planning └ mininav_control
```

---

## 6. 版本路线图

每个版本都是一次完整迭代,而不是推倒重来。版本之间向后兼容,
早期版本的可执行档作为回归基线持续保留。

### V0 — 理想运动仿真 ✅

- **目标**:实现差分驱动运动学模型,输入 `(v, w)` 命令序列,
  输出完整轨迹。建立项目的可演化、可测试、可可视化骨架。
- **关键模块**:`RobotModel`、`CommandSource`、`Trajectory<T>`、
  `csv_format`、`viz/RerunSink`、严格警告策略
- **量化指标**:核心运动学单元测试 100% 通过,CSV 在 IDE / CLI / CI
  三种启动方式下字节一致(可 diff 回归)
- **交付**:`sim_v0` 可执行档、`traj.csv`、`docs/v0_summary.md`

### V1 — 噪声与里程计漂移 ✅

- **目标**:在 V0 的理想骨架上引入两条**独立的**不完美链路
  (执行通道的 Velocity Motion Model + 观测通道的"打滑 + 量化"),
  让 odom 估计与真值之间产生**可量化的**漂移。
- **关键模块**:`ActuatorModel`、`WheelEncoderModel`、`WheelOdometry`、
  `RngFactory`(per-tag seed 派生)、CLI11 集成、三档 noise preset
  (`low-noise` / `default` / `high-noise`)、CSV metadata header
- **量化指标**:`default` preset、20 秒仿真下位置漂移 0.2-0.6 m,
  `--seed N` 两次运行 CSV 完全一致(可复现性回归)
- **交付**:`sim_v1` 可执行档、三轨迹 Rerun 可视化、
  `scripts/v1/analyze_drift.py`、`docs/v1_summary.md`、
  `docs/math/odom_noise.md`

### V2 — EKF 多传感器融合 ✅

- **目标**:用扩展卡尔曼滤波器对抗 V1 留下的 odom 漂移。引入陀螺
  `ImuModel`(在 V1 推迟以避免过早抽象的兑现),把编码器与陀螺作为
  对隐状态的观测融合,得到带不确定性的 `ekf` 估计,并在线估计陀螺零偏。
- **关键模块**:`ImuModel`(角速度 + 白噪声 + 可漂移 bias)、`Ekf`
  (6 维状态 `[px, py, θ, v, ω, b_ω]`,`predict → update_encoder →
  update_imu` 三阶段,Joseph form,可选 Euler/RK4 过程积分器)、
  `encoder_observation`(由物理参数推导 R)、`SimStateV2`、NIS 一致性诊断、
  协方差椭圆可视化
- **方法**:解析 Jacobian `G = ∂g/∂x` 对 **Euler 和 RK4 两条路径**都用
  中心差分逐列验证;Q 由 V1 actuator 的 α 推导、R 由 encoder 物理参数推导,
  `--q-scale`/`--r-scale` 只缩放滤波器信任度做敏感性分析
- **量化指标**(20-seed 聚合,见实验报告):融合增益**档位相关**——
  `low-noise` 下 position RMSE 相对 odom **−48.9%**,`default` 下 **−8.3%**;
  **关键发现**:在线 bias 估计有工作域,`high-noise` 下会失稳(20 seed
  里 19 个发散)——更具表达力的模型只在新增状态足够可观测时才更好
- **交付**:`sim_v2` 可执行档、三轨迹对比(truth / odom / ekf)、
  `docs/experiments/v2_ekf_fusion.md`(20-seed 定量报告)、
  `docs/math/EKF_Foundations.md` + `docs/math/runge_kutta_integration.md`、
  `docs/v2_summary.md`

### V3 — 全局路径规划

- **目标**:从 PGM/PNG 加载二维占据栅格地图,实现 A\* 全局规划与
  障碍物膨胀。引入 yaml-cpp 承载地图与规划器配置。
- **关键模块**:`OccupancyGrid`(PGM/PNG IO + 膨胀)、`AStar`
  (曼哈顿/欧几里得启发式)、`GlobalPlanner` 接口(签名提前对齐
  `nav2_core::GlobalPlanner`)、YAML 配置加载
- **工程升级**:同期引入 spdlog 替换 V0 内置 logger;引入 gmock,
  补 viz 模块的测试覆盖
- **量化指标**:典型 200×200 地图规划时间 ≤ 50 ms;在手画测试
  地图集合上,A\* 输出路径长度与已知最短路偏差 ≤ 1 个 cell
- **交付**:`sim_v3` 可执行档、规划路径 Rerun 可视化、手画/真实
  地图实验集、`docs/experiments/v3_planning.md`

### V4 — 控制 + ROS 2 节点化

- **目标**:实现 Pure Pursuit 跟踪控制器,把 V0-V3 所有模块**重新
  打包**成 ROS 2 节点。引入 ROS 2 Jazzy Jalisco。
- **关键模块**:`PurePursuit`(接口对齐 `nav2_core::Controller`)、
  ROS 2 packages(`mininav_msgs` / `mininav_sim` /
  `mininav_localization` / `mininav_planning` / `mininav_control`)、
  launch 文件、RViz2 配置
- **量化指标**:在 V3 规划路径集合上,Pure Pursuit 跟踪误差均值
  ≤ 10 cm、峰值 ≤ 30 cm(default 噪声下);ROS 2 节点 1Hz 心跳无
  消息丢失
- **交付**:ROS 2 工作空间、launch 文件、RViz2 截图、
  `docs/experiments/v4_control.md`

### V5 — 完整仿真闭环

- **目标**:在 ROS 2 内串联 EKF + A\* + Pure Pursuit,完成
  "给定 goal pose → 规划 → 跟踪 → 到达"的端到端 demo。
- **量化指标**:在 5 个测试场景下,机器人成功到达 goal(误差
  ≤ 20 cm)的成功率 ≥ 80%;端到端时延(goal 下发 → 第一条 cmd 输出)
  ≤ 100 ms
- **交付**:完整 sim demo MP4 + GIF(README 首屏与 LinkedIn
  分享素材)、`docs/experiments/v5_full_loop.md`

### V6 — 实车部署

- **目标**:在 Raspberry Pi 5 上跑整套 ROS 2 系统,驱动加装了
  BNO055 IMU 与编码器电机的 Adeept 4WD 小车,完成室内自主导航。
- **硬件准备**:
  - 加装 BNO055 IMU(I2C)
  - 替换原车 N20 电机为**带编码器**的电机
  - 必要时补充降压模块、电源隔离
- **关键工作**:实车 odom 标定、IMU 标定与温漂补偿、EKF 噪声
  参数从 V2 仿真值迁移到实车标定值
- **量化指标**:**sim-to-real gap 表格**——每个 EKF / 控制器
  参数的仿真值 vs 实车标定值并列;同一条命令序列在仿真与实车上
  的轨迹 Hausdorff 距离 ≤ X m
- **交付**:**实车室内导航视频**(简历首屏王牌)、
  `docs/experiments/v6_sim_to_real_gap.md`(项目最有故事的一篇)

### V7 — SLAM 集成(Stretch Goal)

- **定位**:**非必做**。仅在 V6 完成且时间允许时启动。
- **方法**:**不**自写 SLAM,采用成熟的 **slam_toolbox** 在
  实车上建图,把生成的 occupancy grid 输入到 MiniNav 的 V3 规划器。
  这样 V7 的工作是"SLAM 集成 + 真实环境导航",而非"SLAM 算法实现",
  风险可控,收益是"真实室内环境从零建图到自主导航"的完整视频。
- **交付**(如完成):真实室内建图 + 自主导航视频

---

## 7. 可视化策略

MiniNav 的可视化分为三层,每一层有不同的**读者**和**职责**:

| 层 | 工具                 | 职责               | 读者               |
|---|--------------------|------------------|------------------|
| 1 | **Rerun Viewer**   | 开发期最高频迭代手段,交互式回放 | 开发者本人(每天)        |
| 2 | **Python PNG/PDF** | 论文级静态图,精度无损,文档嵌入 | 文档/报告读者(`docs/`) |
| 3 | **MP4 / GIF**      | 首屏展示,LinkedIn/社交 | README 首屏、外部观看者  |

### 7.1 Rerun 的角色

Rerun 是项目的**开发期日常工作面**。机器人主循环每一步通过 ADL
自由函数 `log_to_rerun(SimStateVN, ...)` 把状态推到 Rerun 后端,
Viewer 实时渲染 3D/2D 视图与时间序列,支持暂停、回放、倒带。

各版本 Rerun 视图内容:

| 阶段 | Rerun 可视化内容                                                                                                                              |
|----|------------------------------------------------------------------------------------------------------------------------------------------|
| V0 | 机器人位姿、轨迹、控制输入时间序列                                                                                                                        |
| V1 | **三轨迹**:cmd_traj(完美执行)/ truth(actuator 噪声后)/ odom(编码器全链路)<br/>诊断时序:cmd_v/w、true_velocity_v/w、encoder_dticks_l/r、error/position、error/yaw |
| V2 | 在三轨迹上叠加 `ekf` 估计轨迹;`bias_omega` 学习曲线(估计 vs 真值)实时收敛演示;协方差椭圆演化由 Python 脚本离线出图                                                              |
| V3 | occupancy grid 地图 + A\* 搜索展开过程 + 规划路径                                                                                                    |
| V4 | 完整导航过程:地图 + 路径 + 机器人轨迹 + look-ahead 点                                                                                                    |
| V5 | ROS 2 topic 直接接入 Rerun(或并行接 RViz2)                                                                                                       |
| V6 | 实车实时可视化(Pi 端流到 PC 端 Rerun)                                                                                                               |

### 7.2 Python 静态图的角色

Python 脚本从 CSV 出 PNG/PDF/GIF。`.rrd` 是二进制格式不可 diff、
不可嵌入文档,Rerun 截图清晰度也有限。脚本按版本组织在 `scripts/v{N}/`,
产出落在 `results/v{N}/`:V1 的 `scripts/v1/analyze_drift.py` 出
`trajectory.png` 与 `drift_over_time.png`;V2 的 `scripts/v2/` 出三轨迹、
累积 RMSE、NIS 一致性、3σ 状态误差、bias 学习曲线与协方差椭圆演化(含
`covariance_evolution.gif`)。

### 7.3 MP4 / GIF 的角色

V5 完整闭环 demo 与 V6 实车视频是 README 首屏与外部分享素材。
Rerun 录屏 + ffmpeg 转 GIF 是标准生成路径。简历 PDF 无法嵌入
GIF,但 GitHub README 与 LinkedIn 帖子可以。

### 7.4 viz 库的工程隔离

`viz` 是独立的 STATIC 库,与 `core` 平级。通过 PIMPL 让
`rerun_sink.ixx` 不暴露 `<rerun.hpp>`——下游 target 看不到 Rerun
符号,编译时间不受 Rerun 头文件大小影响。这样"在树莓派上跑无
可视化版本"时,核心算法库不需要任何 `#ifdef` 改动。

---

## 8. 测试策略

测试存在的意义是给未来的重构提供**安全网**。MiniNav 的测试策略
按"被重构概率"分配投入,优先覆盖核心算法、可复现性约定、跨层
接口。

### 8.1 单元测试(GoogleTest)

每个静态库对应一个测试可执行档(`core_tests` / `sensors_tests` /
`localization_tests` / `planning_tests` / `control_tests`),
通过 `gtest_discover_tests` 自动注册到 CTest,支持 `ctest -R`
精细化筛选。

| 库              | 覆盖重点                                                       |
|----------------|------------------------------------------------------------|
| `core`         | 运动学积分、`wrap_angle` 边界、`Trajectory` 容器、RngFactory tag 派生稳定性 |
| `sensors`      | ActuatorModel σ=0 时跳过 RNG、WheelEncoder 累计-差分语义、低速量化欠采样     |
| `localization` | WheelOdometry 纯函数性、EKF predict/update、**雅可比有限差分数值验证**(V2)  |
| `planning`     | A\* 最短路正确性、不可达检测、膨胀正确性(V3)                                 |
| `control`      | Pure Pursuit 在直线/圆弧上的输出合理性(V4)                             |

### 8.2 CSV 回归 diff

每次重构核心模块后强制跑一次 `--no-viz` 模式生成新 CSV,
与 baseline diff,空 diff 即证明无数值回归。这条 baseline diff
**同时检验**多个不变量:RngFactory 稳定性、σ=0 跳过 RNG 约定、
估计器纯函数性、主循环无隐式状态——任何一处违反都会让 diff 非空。

### 8.3 可复现性回归(V1 起)

同一 `--seed N` 两次运行,跳过 `# generated_at` 注释行后 diff
应为空。这是 V1 引入的核心约定,贯穿后续所有版本。

### 8.4 集成测试(V4 起)

V4 引入 ROS 2 后,通过 `colcon test` 跑节点级集成测试:
节点能否正常启动、topic 能否正确收发、launch 文件是否有效。

### 8.5 持续集成

GitHub Actions 已激活,矩阵为 `ubuntu-24.04 + clang-18`,每次
push 自动构建 + 跑所有单元测试 + CSV 回归 diff。V4 之后加入
ROS 2 节点的 colcon test。

---

## 9. 语言分工

| 语言         | 模块                                       | 原因          |
|------------|------------------------------------------|-------------|
| **C++**    | 所有核心算法:运动学、EKF、A\*、Pure Pursuit、ROS 2 节点 | 性能、工业实践、跨平台 |
| **Python** | 实验脚本、误差分析、RMSE 计算、matplotlib 静态图、参数扫描    | 后处理灵活、画图方便  |

C++ 负责**系统跑起来**,Python 负责**实验讲清楚**。

---

## 10. 关键产出物

项目完成后,仓库包含以下可对外展示的资产。

### 10.1 代码资产

- 模块化、带测试、CI 持续运行的 C++ 代码库
- ROS 2 工作空间(V4 之后)
- Python 实验脚本

### 10.2 文档资产

**数学推导**(`docs/math/`)——项目算法的"白皮书":

| 文档                           | 内容                                         | 版本 |
|------------------------------|--------------------------------------------|----|
| `odom_noise.md`              | Velocity Motion Model 四参数推导、编码器物理模型、量化误差分析 | V1 |
| `EKF_Foundations.md`         | EKF 预测/更新方程、雅可比手推与有限差分验证、Joseph form       | V2 |
| `runge_kutta_integration.md` | RK4 过程积分及其解析 Jacobian                      | V2 |

> V0 运动学暂无独立数学文档(推导见 `v0_summary.md`);V3+ 的规划 /
> 控制推导待对应版本补齐。

**实验报告**(`docs/experiments/`):每个版本结束时一篇,说清楚"问题→
方案→坑→结果",含图、数据、结论。已有 `v2_ekf_fusion.md`(V2)。

**版本总结**(`docs/vN_summary.md`):每个版本一篇阶段性总结,记录架构、
关键设计决策、踩坑实录、与下一个版本的衔接。已有 `v0`/`v1`/`v2`。

**项目管理**(`docs/project-management.md`):issue 模板、看板列、
milestone 与发布约定。

### 10.3 视觉资产

- V0-V2:Rerun 录屏 + Python PNG(三轨迹对比、漂移曲线、协方差椭圆)
- V2:EKF RMSE 表格
- V5:**完整仿真导航 MP4 + GIF**(README 首屏)
- V6:**实车导航视频**+ **sim-to-real gap 表格**

### 10.4 量化指标汇总

每个版本都有显式的量化交付,贯穿整个项目:

| 版本 | 关键量化指标                                                              |
|----|---------------------------------------------------------------------|
| V0 | 单元测试 100% 通过,CSV 跨启动方式字节一致                                          |
| V1 | `default` preset 20s 位置漂移 0.2-0.6 m,seed 复现性 byte-exact             |
| V2 | 融合增益档位相关(low-noise position RMSE −48.9%),雅可比有限差分双路径校验;bias 估计工作域已量化 |
| V3 | 200×200 地图 A\* 规划 ≤ 50 ms,路径长度偏差 ≤ 1 cell                           |
| V4 | Pure Pursuit 跟踪误差均值 ≤ 10 cm、峰值 ≤ 30 cm                              |
| V5 | 5 个测试场景到达成功率 ≥ 80%,端到端时延 ≤ 100 ms                                   |
| V6 | sim-to-real gap 表(每个参数仿真 vs 实测),轨迹 Hausdorff 距离量化                   |

---

## 11. 项目核心卖点

| 卖点       | 说明                                                                       |
|----------|--------------------------------------------------------------------------|
| **完整**   | 不是单点算法,而是从仿真到实车的完整导航系统                                                   |
| **可解释**  | 每一层都有数学推导、设计文档与实验验证                                                      |
| **可量化**  | 每个版本都有显式量化指标,有图、有 RMSE、有参数扫描、有 sim-to-real gap                           |
| **可扩展**  | 从 V0 到 V6 的每一步都是向前兼容的迭代,而非重写;早期可执行档作为回归基线持续保留                            |
| **工程味重** | 现代 C++(modules、ADL 扩展点、PIMPL)、CMakePresets、GoogleTest、CI、ROS 2、Rerun 全家桶 |
| **有实车**  | Raspberry Pi 5 + 4WD(替换带编码器电机 + BNO055 IMU)真实部署,含 sim-to-real 叙事         |

---

## 一句话总结

> MiniNav 是一个以**现代 C++ 为核心**、面向室内移动机器人的定位
> 与导航系统,从运动学仿真出发,逐步叠加噪声建模、EKF 多传感器
> 融合、A\* 路径规划、Pure Pursuit 跟踪控制、ROS 2 节点化,并
> 最终在 Raspberry Pi 5 小车上完成室内自主导航的实车闭环验证。