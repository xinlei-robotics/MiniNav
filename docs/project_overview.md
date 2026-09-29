# MiniNav — Indoor Mobile Robot Localization & Navigation System

> 一个以现代 C++ 为核心、面向室内移动机器人的定位与导航系统。
> 从差分驱动运动学仿真出发,逐步叠加噪声建模、EKF 多传感器融合、
> 路径规划、闭环跟踪控制、ROS 2 + Nav2 集成,并最终在 Raspberry Pi 5 + 4WD
> 小车平台上完成室内自主移动的实车闭环。

> **当前进度(截至 2026-09):V0 / V1 / V2 / V3 已完成**,V4(闭环路径跟踪)
> 为下一个里程碑。2026-09-28 调整了 V4 / V5 的划分:V4 只在纯 C++ 仿真里完成
> 闭环跟踪,ROS 2 + Nav2 集成整体移到 V5(理由见 §6 V4)。本文档既是项目总
> 愿景,也是版本路线图——已完成版本(✅)的描述对齐仓库真实状态,未完成版本
> (V4–V7)是**前瞻规划**,其模块名、目录、量化指标均为设计意图,可能随实现调整。

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
| **yaml-cpp**              | `planner.yaml` / `map.yaml` 等外部配置文件   | V3(FetchContent 混合模式) |
| **spdlog**                | 替换 V0 内置 logger,分级、带时间戳的日志        | V3(FetchContent 混合模式) |
| **gmock**                 | `VizSink` 等接口的 mock,viz 层单测         | V3(随 GoogleTest)      |
| **ROS 2 (Jazzy Jalisco)** | 节点化、标准消息与 TF、RViz2 可视化、launch 系统    | V5 引入(规划)             |
| **Nav2**                  | 行为树导航编排;A\* 规划器与 Pure Pursuit 控制器以插件接入 | V5 引入(规划)             |

> **当前已集成**:Eigen3、GoogleTest / gmock、Rerun SDK、CLI11、yaml-cpp、
> spdlog(V0–V3)。ROS 2 与 Nav2 是 V5 的**规划项,尚未引入**。
>
> **依赖管理策略**:Eigen 用系统包(header-only 共享高效)、GoogleTest
> 用纯 FetchContent(ABI 风险)、Rerun / CLI11 / yaml-cpp / spdlog 用
> `FetchContent` + `FIND_PACKAGE_ARGS` 混合(本机有装走 find、否则 fetch)。
> spdlog 以 PRIVATE 方式链接进 `core`,不向下游暴露任何类型。不同模式对应
> 不同第三方库的工程形态,没有"银弹策略"。

### 2.4 辅助工具

- **Python**:实验后处理、误差分析(RMSE)、批量实验脚本、参数扫描、论文级静态图表
- **Git / GitHub**:版本控制、CI、发布演示
- **Linux / WSL**:开发环境;树莓派端为原生 Ubuntu 24.04

---

## 3. 工程实践与设计原则

MiniNav 的代码组织遵循一组贯穿所有版本的设计原则,它们决定了
"新增功能时不需要重写旧代码"这一关键性质。

### 3.1 Plain-data 状态结构 + ADL 自由函数

仿真状态是一个 plain-data 结构 `SimState`(无继承),序列化与可视化
不写成成员函数,而是**自由函数**,通过 Argument-Dependent Lookup
(实参依赖查找)在编译期解析:

- 序列化:`csv_header(SimState)` / `csv_row(SimState)`
- 可视化:`log_to_rerun(sink, SimState, ...)`

要支持一个新的记录类型,只需为它新增一组重载,而不必改动容器或可视化
后端。容器层用 `Trajectory<T>` 模板复用——"行为相同、类型不同"用模板,
不用继承。

V0–V2 期间曾经每个版本各有一个快照结构(`SimStateV0` / `V1` / `V2`)并存;
V2 收尾后改为 tag-based 版本策略,trunk 只保留当前的 `SimState`,历史版本由
git tag 保存(见 §6)。V3 的规划产出是一次性的,不套 per-step 结构,而是
独立的 `PlanResult` + `path.csv`。

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
在 `.cpp` 里。下游 target(`sim` / 未来的 ROS 2 节点)的编译时间不受
Rerun 头文件大小影响,符号也不被污染。这同时保证了"在树莓派上跑无
可视化版本"时,核心算法库不需要 `#ifdef` 大改。

V3 在 PIMPL 之上又加了一层**接口隔离**:可视化逻辑只依赖抽象的
`VizSink`,`RerunSink` 是它唯一的 Rerun 实现。测试里用 gmock 的
`MockVizSink` 断言实体路径与调用契约,不起 Viewer——"能 mock"本身就
证明了依赖被隔离干净。

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
│ Layer 4: Motion Control                     │  Regulated Pure Pursuit
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

### Layer 3 — 全局路径规划(V3 引入)

Layer 3 在 CMake 层面物化为静态库 `planning`,只依赖 `core` 与 yaml-cpp,
与 `sensors` / `localization` 没有编译期依赖:`OccupancyGrid` 进、`Path` 出。

- **地图表示**:二维 occupancy grid(free / occupied / unknown 三值),从 ROS
  `map_server` 风格的 PGM(P2/P5)+ `map.yaml` 加载;origin 取左下角,越界视为占据
- **膨胀层**:多源欧氏距离变换 + 按半径膨胀,把机器人缩成配置空间里的一个点;
  同一张距离场还驱动可选的代价梯度(让路径远离障碍)
- **算法**:A\* 搜索,4/8 连通,Manhattan / Euclidean / Octile 启发式,8 连通防穿角;
  启发式与连通度的组合必须 admissible(Manhattan + 8 连通会被拒绝)
- **接口风格**:`GlobalPlanner` 类的签名提前对齐
  `nav2_core::GlobalPlanner` 的形态(不依赖 ROS 消息类型,使用项目
  自己的 `Path` 类型),V5 接入 Nav2 时只需要薄薄一层插件适配器
- **配置**:yaml-cpp 读 `config/planner.yaml`(膨胀半径、启发式、连通度等),
  CLI 可逐项覆盖;规划入口是 `sim plan`

### Layer 4 — 路径跟踪控制(V4 规划)

- **控制器**:Regulated Pure Pursuit 的核心子集——速度自适应 look-ahead、
  曲率限速、接近目标减速、大角度先原地转向、加速度限幅。不另做 PID 备选;
  对照组是"定 look-ahead、不限速"的经典 Pure Pursuit(同版本消融)
- **接口风格**:`Controller` / `GoalChecker` / `ProgressChecker` 对齐
  `nav2_core` 的同名接口(不依赖 ROS 类型),V5 直接包成 Nav2 插件
- **输入**:EKF 估计位姿与速度 + 全局路径(A\* 结果经首尾替换与视线捷径平滑)
- **输出**:`Twist2D (v, w)` 指令,经执行器模型(饱和 + 一阶滞后)送回仿真,
  或送到实车底盘驱动
- **参数**:look-ahead 时间、期望速度、曲率限速半径、加速度上限放在
  `config/nav.yaml`;轮径、轮距、底盘外形、执行器限幅与时间常数放在
  `config/robot.yaml`(单一来源,膨胀半径由外形推导)。默认值由线性化稳定性
  分析(look-ahead 时间必须大于执行器滞后)与安全裕度预算推出
- **考核口径**:真值误差 ≤ 控制误差 + 定位误差。控制器只对"EKF 估计位姿到
  路径"的误差负责;定位漂移单独报告,不混进控制指标

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

下面是**当前真实结构**(截至 V3)。标注 `(规划)` 的条目尚不存在,
是 V4+ 的设计意图。

```
mininav/
├── CMakeLists.txt
├── CMakePresets.json
├── README.md  CHANGELOG.md  CONTRIBUTING.md
├── requirements.txt                # Python 依赖(rerun-sdk 等)
├── cmake/
│   ├── warnings.cmake              # 严格警告策略(INTERFACE lib)
│   ├── google_test.cmake           # GoogleTest(含 gmock)引入
│   ├── rerun.cmake                 # Rerun SDK 混合模式引入
│   ├── cli11.cmake                 # CLI11 引入
│   ├── yaml_cpp.cmake              # yaml-cpp 混合模式引入(V3)
│   └── spdlog.cmake                # spdlog 混合模式引入(V3)
├── .github/workflows/ci.yml        # GitHub Actions 工作流
├── config/
│   ├── planner.yaml                # A* 规划器配置(V3)
│   ├── robot.yaml                  # 机器人描述(几何 / 外形 / 执行器限幅与滞后)(V4)
│   └── nav.yaml                    # (规划) V4:导航参数(规划 / 控制 / 到达判定)
├── maps/                           # PGM + map.yaml:corridor / room / maze / office / office500(V3)
├── data/                           # 运行产出(不入库):traj.csv / path.csv
├── docs/
│   ├── project_overview.md         # 本文档:项目总愿景与版本路线
│   ├── project_management.md       # issue / 看板 / milestone 约定
│   ├── v0_summary.md               # 各版本阶段性总结
│   ├── v1_summary.md
│   ├── v2_summary.md
│   ├── v3_summary.md
│   ├── math/
│   │   ├── odom_noise.md           # Velocity Motion Model + 编码器物理/量化 (V1)
│   │   ├── EKF_Foundations.md      # EKF 预测/更新、Jacobian、Joseph form (V2)
│   │   ├── runge_kutta_integration.md  # RK4 过程积分及其解析 Jacobian (V2)
│   │   └── astar_planning.md       # 栅格、膨胀、A* 最优性、启发式可采纳性 (V3)
│   └── experiments/
│       ├── v2_ekf_fusion.md        # 20-seed EKF-vs-odom 定量报告 (V2)
│       └── v3_planning.md          # 规划耗时 / 最优性 / 确定性报告 (V3)
├── scripts/                        # Python 后处理(按版本组织)
│   ├── plot_trajectory.py          # V0 出图
│   ├── v1/analyze_drift.py         # V1 漂移分析
│   ├── v2/                         # V2 EKF 分析(analyze_ekf / covariance / integrator / sweep)
│   └── v3/                         # V3 规划:plot_plan / benchmark_planner / optimality_check /
│                                   #   animate_search / gen_office500 / _mapio
├── results/                        # 实验产出:results/v{0,1,2,3}/ 下的 PNG / GIF
├── src/
│   ├── core/                       # 运动学、类型、路径几何、机器人描述、Trajectory、CSV、随机数、积分器、日志
│   │   ├── types.{ixx,cpp}         # Pose2D / Twist2D / EncoderTicks / SimState
│   │   ├── path.{ixx,cpp}          # Path + 折线投影 / look-ahead 求交(V4 从 planning 迁入)
│   │   ├── robot_description.{ixx,cpp}  # RobotDescription + robot.yaml 严格解析(V4)
│   │   ├── math.ixx                # wrap_angle, kPi
│   │   ├── kinematics.{ixx,cpp}    # differential_drive_step + inverse/forward
│   │   ├── integrators.{ixx,cpp}   # rk4_step(V2 引入,真值与 EKF 共用)
│   │   ├── command_source.ixx + staged_command_source.cpp
│   │   ├── trajectory.ixx          # Trajectory<T> 模板
│   │   ├── csv_format.{ixx,cpp}    # csv_header / csv_row 重载
│   │   ├── csv_writer.ixx          # write_csv<T> 模板
│   │   ├── random.ixx              # RngFactory + FNV-1a tag 派生
│   │   └── logger.{ixx,cpp}        # 日志接口;V3 起后端为 spdlog(PRIVATE)
│   ├── sensors/                    # 独立静态库:执行 + 观测噪声模型
│   │   ├── actuator_model.{ixx,cpp}
│   │   ├── wheel_encoder.{ixx,cpp}
│   │   └── imu_model.{ixx,cpp}     # V2 引入:gyro 白噪声 + 可漂移 bias
│   ├── simulation/                 # 独立静态库(V4):被控对象,依赖 core + sensors
│   │   ├── noise_presets.ixx       # 三档噪声标定(low-noise / default / high-noise)
│   │   └── plant.{ixx,cpp}         # Plant:执行噪声 → encoder / IMU → 真值积分
│   ├── localization/               # 独立静态库:估计器
│   │   ├── wheel_odometry.{ixx,cpp}
│   │   ├── ekf_state.ixx           # V2:Vec6/Mat6、StateIdx、EkfState6
│   │   ├── ekf.{ixx,cpp}           # V2:6D EKF(predict + encoder/imu update)
│   │   ├── encoder_observation.{ixx,cpp}  # V2:解码 z 与推导 R
│   │   └── ekf_pipeline.{ixx,cpp}  # V4:ticks + gyro → predict / update 编排(V5 EKF 节点边界)
│   ├── planning/                   # 独立静态库(V3):只依赖 core + yaml-cpp
│   │   ├── grid_types.ixx          # GridCoord / PlannerConfig / is_admissible
│   │   ├── planner_config.{ixx,cpp}  # planner.yaml 解析 / 序列化
│   │   ├── occupancy_grid.{ixx,cpp}  # OccupancyGrid + world/grid 变换
│   │   ├── map_io.{ixx,cpp}        # PGM(P2/P5)+ map.yaml 加载
│   │   ├── inflation.{ixx,cpp}     # 欧氏距离变换 + 膨胀
│   │   └── astar.{ixx,cpp}         # GlobalPlanner / AStarPlanner / PlanResult
│   ├── viz/                        # 接口 + PIMPL 隔离 Rerun
│   │   ├── viz_sink.{ixx,cpp}      # VizSink 抽象接口(V3)
│   │   ├── rerun_sink.{ixx,cpp}    # RerunSink : VizSink
│   │   ├── sim_state_log.{ixx,cpp} # log_to_rerun(SimState, ...)
│   │   └── plan_log.{ixx,cpp}      # PlanScene + log_plan(V3)
│   ├── apps/sim/                   # 单一 sim,CLI11 子命令(模块 mininav.apps.sim)
│   │   ├── main.cpp                # 子命令解析与分派
│   │   ├── sim.ixx + common.cpp    # 选项结构、入口声明、各模式共享工具
│   │   ├── ekf_mode.cpp            # sim ekf:V2 定位仿真
│   │   └── plan_mode.cpp           # sim plan:V3 一次性规划;(规划) V4:nav_mode.cpp
│   └── control/                    # (规划) V4:Controller 接口 + Pure Pursuit + 速度平滑
├── tests/                          # GoogleTest,按子库组织
│   ├── core/                       # math / kinematics / trajectory / types / random / path / robot_description
│   ├── sensors/                    # actuator / wheel_encoder / imu_model
│   ├── simulation/                 # plant(与直接组合传感器模型逐位相同)
│   ├── localization/               # wheel_odometry + 8 个 EKF 测试 + encoder_observation + ekf_pipeline
│   ├── planning/                   # grid_types / occupancy_grid / map_io / inflation / astar / planner_config
│   ├── viz/                        # gmock:viz_sink_log_tests
│   ├── control/                    # (规划) V4
│   ├── tools/                      # csv_compare:golden 比较工具 + 单测
│   └── golden/                     # golden CSV 回归基线(标签 regression,见 golden/README.md)
└── ros2_ws/                        # (规划) V5:colcon 包——节点(仿真 / EKF)、
                                    #   Nav2 插件(A* / Pure Pursuit)、bringup;只用标准消息
```

---

## 6. 版本路线图

每个版本都是一次完整迭代,而不是推倒重来。V0–V2 期间,各版本的可执行档
曾经并存,作为回归基线;V2 收尾后改为 **tag-based 版本策略**:`main` 只保留
当前最佳设计(单一 `sim`),每个完成的里程碑由 git tag + GitHub Release +
`docs/` 回顾文档保存(`v0.1.0`=V0 … `v0.4.0`=V3),回归保护靠测试与确定性输出。
下文各版本的"交付"一栏记录的是**当时**的产物。

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

### V3 — 全局路径规划 ✅

- **目标**:从 PGM + `map.yaml` 加载二维占据栅格地图,实现障碍物膨胀与 A\*
  全局规划。引入 yaml-cpp 承载规划器配置。
- **关键模块**:新增静态库 `planning`——`OccupancyGrid`(PGM P2/P5 加载)、
  欧氏距离变换膨胀、`AStarPlanner`(4/8 连通,Manhattan / Euclidean / Octile
  启发式,防穿角,可选代价梯度)、`GlobalPlanner` 接口(签名对齐
  `nav2_core::GlobalPlanner`)、`planner.yaml` 加载;`viz` 新增 `PlanScene` /
  `log_plan`
- **工程升级**:spdlog 替换 V0 内置 logger(接口不变);抽出 `VizSink` 接口,
  用 gmock 补上 viz 模块的测试覆盖
- **量化指标**:200×200 地图单次规划 p95 **3.94 ms**(目标 ≤ 50 ms);手画测试
  地图上与 Dijkstra 最短路偏差 **0 cell**(目标 ≤ 1 cell);同输入 `path.csv`
  逐字节一致。**关键发现**:最优性保证依赖启发式与连通度的搭配——Manhattan
  在 8 连通下高估对角步,悄无声息地超标 1.13 cell,因此被配置层直接拒绝
- **交付**:`sim --map` 规划模式(演进单一 `sim`,不再另起 `sim_v3`)、Rerun
  规划视图、`path.csv`、手画 + 程序生成的地图集、`scripts/v3/` 与搜索过程动画、
  `docs/experiments/v3_planning.md`、`docs/math/astar_planning.md`、
  `docs/v3_summary.md`

### V4 — 闭环路径跟踪

> **2026-09-28 调整**:原 V4 是"Pure Pursuit + 把 V0–V3 全部重新打包成 ROS 2
> 节点",原 V5 是"在 ROS 2 内完成端到端闭环"。重新划分的理由:
>
> - 控制(新算法)与 ROS 化(新框架)是互不依赖的两类风险,绑在一起约 10 个
>   PR,而第一次真正闭环开车要等到 V5;
> - 在异步、按墙钟运行的 ROS 2 里调控制器,实验不可复现;确定性的 C++ 仿真
>   (同 seed 逐字节一致、进 CI)才是开发与验证算法的地方;
> - 原跟踪误差指标没有区分控制误差与定位漂移,按真值量,测到的主要是 EKF 漂移。
>
> 因此 V4 只在纯 C++ 仿真里完成闭环,ROS 2 + Nav2 集成整体移到 V5。

- **目标**:让机器人沿 A\* 路径真正开到终点——规划 → 路径后处理 → 跟踪
  (控制器输入 EKF 估计位姿)→ 到达,全程同 seed 逐字节确定。
- **关键模块**:
  - `control` 库:`Controller` / `GoalChecker` / `ProgressChecker`(对齐
    `nav2_core`)、Regulated Pure Pursuit 子集、速度平滑器
  - `simulation` 库:被控对象 `Plant`(执行器饱和 + 一阶滞后 → 执行噪声 →
    真值积分 → 传感器);`localization` 新增 `EkfPipeline`。二者正是 V5 仿真
    节点与 EKF 节点的边界
  - 路径后处理:首尾替换为真实起止点、视线捷径平滑
  - 机器人描述 `config/robot.yaml`(由外形推导膨胀半径)、导航参数
    `config/nav.yaml`;一张真实尺度的演示地图
  - `sim` 拆成 `ekf / plan / nav` 子命令;golden CSV 回归护栏进 CI
- **量化指标**:
  - 控制误差(EKF 估计位姿到路径的横向距离,default 噪声,5 场景 × 10 seed):
    均值 ≤ 10 cm、峰值 ≤ 30 cm
  - 线性化理论吻合:直线小偏置下,反向超调与调节距离相对解析值
    (e^−π ≈ 4.3%、约 4.26 倍 look-ahead 距离)偏差 ≤ 10%
  - 无噪声与 oracle(真值进控制器)运行:零碰撞、100% 到达
  - 真值到达误差与碰撞率 vs 路程:只报告、不设门槛,用来决定 V5 场景设计与
    绝对定位的引入时机
- **交付**:`sim nav` 闭环模式、`nav.csv`、Rerun 闭环视图与 README GIF、
  `docs/experiments/v4_control.md`、`docs/math/pure_pursuit.md`、
  `docs/v4_summary.md`

### V5 — ROS 2 + Nav2 集成

- **目标**:把 V4 已验证的闭环搬进 ROS 2 Jazzy,由 Nav2 编排:在 RViz2 里
  点目标 → 规划 → 跟踪 → 到达。
- **关键模块**:
  - 仿真节点(包 `Plant`):订阅 `/cmd_vel`,发布 `/joint_states`(轮子位置)、
    `/imu`、`/clock`
  - EKF 节点(包 `EkfPipeline`):按时间戳异步融合,发布 `/odom` 与 TF
    `odom→base_link`(REP-105;`map→odom` 暂为静态,留给绝对定位)
  - Nav2 插件:A\* 全局规划器与 Pure Pursuit 控制器(接口在 V3 / V4 已按
    `nav2_core` 形态设计,插件只是薄适配);导航流程交给 Nav2 的
    bt_navigator,不自写状态机
  - 只用标准消息(`JointState` / `Imu` / `Odometry` / TF / `Path` /
    `OccupancyGrid`),不建自定义消息包。这些正是 V6 硬件驱动要发的话题,
    EKF 节点在仿真与实车之间一行不改
  - launch、参数文件、RViz2 配置;launch_testing 端到端测试进 CI
- **量化指标**:5 个场景目标到达率 ≥ 80%(真值误差 ≤ 20 cm;场景路程上限按
  V4 测得的漂移曲线确定);端到端时延(goal 下发 → 第一条 cmd_vel)≤ 100 ms
- **交付**:colcon 工作空间、RViz2 导航 demo(MP4 + GIF,README 首屏与
  LinkedIn 分享素材)、`docs/experiments/v5_full_loop.md`

### V6 — 实车部署

- **目标**:在 Raspberry Pi 5 上跑整套 ROS 2 系统,驱动加装了
  BNO055 IMU 与编码器电机的 Adeept 4WD 小车,完成室内自主导航。
- **硬件准备**(提前采购,与 V4 / V5 软件并行):
  - 加装 BNO055 IMU(I2C)
  - 替换原车 N20 电机为**带编码器**的电机
  - 必要时补充降压模块、电源隔离
- **关键工作**:实车 odom 标定、IMU 标定与温漂补偿、EKF 噪声
  参数从 V2 仿真值迁移到实车标定值;`config/robot.yaml` 换成实测值(外形、
  限幅、执行器时间常数、滑移转向的有效轮距)
- **绝对定位**:4WD 是滑移转向,打滑远大于差速仿真,真机的到达精度几乎一定
  离不开绝对定位。方案(ArUco + Pi 摄像头,或 2D LiDAR + AMCL / slam_toolbox)
  依据 V4 的"到达误差 vs 路程"数据在 V6 开工前确定
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
自由函数 `log_to_rerun(sink, SimState, ...)` 把状态推到可视化后端,
Viewer 实时渲染 3D/2D 视图与时间序列,支持暂停、回放、倒带;规划模式则把
整个场景一次性以 static 数据推送(`log_plan`)。

各版本 Rerun 视图内容:

| 阶段 | Rerun 可视化内容                                                                                                                              |
|----|------------------------------------------------------------------------------------------------------------------------------------------|
| V0 | 机器人位姿、轨迹、控制输入时间序列                                                                                                                        |
| V1 | **三轨迹**:cmd_traj(完美执行)/ truth(actuator 噪声后)/ odom(编码器全链路)<br/>诊断时序:cmd_v/w、true_velocity_v/w、encoder_dticks_l/r、error/position、error/yaw |
| V2 | 在三轨迹上叠加 `ekf` 估计轨迹;`bias_omega` 学习曲线(估计 vs 真值)实时收敛演示;协方差椭圆演化由 Python 脚本离线出图                                                              |
| V3 | 占据栅格 + 膨胀安全裕度 + A\* 规划路径 + 起止位姿(static);A\* 搜索展开过程由 `scripts/v3/animate_search.py` 离线渲染成动画                                         |
| V4 | 闭环导航:地图 + 原始 / 平滑路径 + 真值与 EKF 轨迹 + EKF 3σ 椭圆 + look-ahead 点与追踪圆弧 + 控制 / 定位 / 真值三种误差时序 |
| V5 | RViz2 为主(Nav2 标准面板:地图、代价地图、路径、机器人位姿);需要时把 ROS 2 topic 桥接到 Rerun |
| V6 | 实车实时可视化(Pi 端流到 PC 端 Rerun)                                                                                                               |

### 7.2 Python 静态图的角色

Python 脚本从 CSV 出 PNG/PDF/GIF。`.rrd` 是二进制格式不可 diff、
不可嵌入文档,Rerun 截图清晰度也有限。脚本按版本组织在 `scripts/v{N}/`,
产出落在 `results/v{N}/`:V1 的 `scripts/v1/analyze_drift.py` 出
`trajectory.png` 与 `drift_over_time.png`;V2 的 `scripts/v2/` 出三轨迹、
累积 RMSE、NIS 一致性、3σ 状态误差、bias 学习曲线与协方差椭圆演化(含
`covariance_evolution.gif`);V3 的 `scripts/v3/` 出规划总览图
`plan_<map>.png`、耗时基准、最优性对比与 A\* 搜索动画 `search_<map>.gif`
(文件名随地图,多张地图的产出互不覆盖)。

### 7.3 MP4 / GIF 的角色

V3 的 A\* 搜索动画(`search_office500.gif`)目前是 README 首屏;V4 的闭环
导航 GIF、V5 的 RViz2 / Nav2 导航 demo 与 V6 实车视频将依次成为之后的首屏
与外部分享素材。
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
`localization_tests` / `planning_tests` / `viz_tests`,V4 起加
`control_tests` / `simulation_tests`),通过 `gtest_discover_tests` 自动注册到 CTest,并按库打
标签(`ctest -L planning`),支持 `ctest -R` 精细化筛选。

| 库              | 覆盖重点                                                                     |
|----------------|--------------------------------------------------------------------------|
| `core`         | 运动学积分、`wrap_angle` 边界、`Trajectory` 容器、RngFactory tag 派生稳定性               |
| `sensors`      | ActuatorModel σ=0 时跳过 RNG、WheelEncoder 累计-差分语义、低速量化欠采样                   |
| `localization` | WheelOdometry 纯函数性、EKF predict/update、**雅可比有限差分数值验证**(V2)                |
| `planning`     | 坐标往返、PGM 加载与 y 翻转、欧氏膨胀、A\* 精确最优长度、不可达检测、启发式可采纳性拒绝、200×200 压力图与耗时(V3) |
| `viz`          | gmock 断言可视化下沉的实体路径与调用契约,不起 Viewer(V3)                                   |
| `control`      | 解析用例:直线小偏置的超调与调节距离对照线性化解、圆弧稳态误差为零、切角与 look-ahead 成正比;限速、原地转向、到达停车(V4) |

### 8.2 CSV 回归 diff

每次重构核心模块后强制跑一次 `--no-viz` 模式生成新 CSV,
与 baseline diff,空 diff 即证明无数值回归。这条 baseline diff
**同时检验**多个不变量:RngFactory 稳定性、σ=0 跳过 RNG 约定、
估计器纯函数性、主循环无隐式状态——任何一处违反都会让 diff 非空。
V4 计划把这条 diff 固化为 CTest 的 golden 回归:EKF / 规划 / 导航三种模式的
基线 CSV 入库(`tests/golden/`)并进 CI;浮点列按 1e-9 相对容差比较,以容忍
不同 CPU 上 libm 实现的末位差异。

### 8.3 可复现性回归(V1 起)

同一 `--seed N` 两次运行,跳过 `# generated_at` 注释行后 diff
应为空。这是 V1 引入的核心约定,贯穿后续所有版本。

### 8.4 集成测试(V4 起)

V4:`sim nav` 闭环集成测试进 CTest——无噪声下到达且零碰撞、oracle(真值进
控制器)下到达误差在容差内、同 seed 两次运行 `nav.csv` 逐字节一致。

V5:引入 ROS 2 后,通过 `colcon test` + launch_testing 跑节点级与端到端集成
测试——节点能否正常启动、topic 能否正确收发、发出目标后能否到达。

### 8.5 持续集成

GitHub Actions 已激活,环境为 `ubuntu-24.04 + clang-18`,每个 PR 与
push 到 `main` 时以 Debug preset 构建并跑全部单元测试。CSV 回归 diff 目前
在本地手动执行,尚未进 CI;只在 Release 下有意义的断言(如 A\* 的 50 ms
耗时预算)在 CI 中显示为 Skipped。V4 计划把 golden 回归与闭环集成测试纳入
CI;V5 再加入 ROS 2 的 colcon test。

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
- ROS 2 工作空间与 Nav2 插件(V5 之后)
- Python 实验脚本

### 10.2 文档资产

**数学推导**(`docs/math/`)——项目算法的"白皮书":

| 文档                           | 内容                                         | 版本 |
|------------------------------|--------------------------------------------|----|
| `odom_noise.md`              | Velocity Motion Model 四参数推导、编码器物理模型、量化误差分析 | V1 |
| `EKF_Foundations.md`         | EKF 预测/更新方程、雅可比手推与有限差分验证、Joseph form       | V2 |
| `runge_kutta_integration.md` | RK4 过程积分及其解析 Jacobian                      | V2 |
| `astar_planning.md`          | 占据栅格与配置空间膨胀、A\* 最优性证明、启发式在 4/8 连通下的可采纳性与一致性 | V3 |

> V0 运动学暂无独立数学文档(推导见 `v0_summary.md`)。V4 计划新增
> `pure_pursuit.md`:几何、直线附近线性化与滞后稳定性、切角尺度律、安全裕度预算。

**实验报告**(`docs/experiments/`):每个版本结束时一篇,说清楚"问题→
方案→坑→结果",含图、数据、结论。已有 `v2_ekf_fusion.md`(V2)、
`v3_planning.md`(V3)。

**版本总结**(`docs/vN_summary.md`):每个版本一篇阶段性总结,记录架构、
关键设计决策、踩坑实录、与下一个版本的衔接。已有 `v0`/`v1`/`v2`/`v3`。

**项目管理**(`docs/project_management.md`):issue 模板、看板列、
milestone 与发布约定。

### 10.3 视觉资产

- V0-V2:Rerun 录屏 + Python PNG(三轨迹对比、漂移曲线、协方差椭圆)
- V2:EKF RMSE 表格
- V3:A\* 搜索过程动画(500×500 楼宇平面)、规划总览图、耗时与最优性图表
- V4:**闭环导航 GIF**(仿真,README 首屏候选)
- V5:**RViz2 / Nav2 导航 MP4 + GIF**(README 首屏与分享素材)
- V6:**实车导航视频**+ **sim-to-real gap 表格**

### 10.4 量化指标汇总

每个版本都有显式的量化交付,贯穿整个项目:

| 版本 | 关键量化指标                                                              |
|----|---------------------------------------------------------------------|
| V0 | 单元测试 100% 通过,CSV 跨启动方式字节一致                                          |
| V1 | `default` preset 20s 位置漂移 0.2-0.6 m,seed 复现性 byte-exact             |
| V2 | 融合增益档位相关(low-noise position RMSE −48.9%),雅可比有限差分双路径校验;bias 估计工作域已量化 |
| V3 | 200×200 地图 A\* 规划 ≤ 50 ms(实测 p95 3.94 ms),路径长度偏差 ≤ 1 cell(实测 0 cell) |
| V4 | 控制误差(估计位姿到路径)均值 ≤ 10 cm、峰值 ≤ 30 cm;线性化理论吻合 ≤ 10%;无噪声 / oracle 零碰撞 |
| V5 | Nav2 闭环:5 个场景到达率 ≥ 80%(场景路程按 V4 漂移数据确定),端到端时延 ≤ 100 ms |
| V6 | sim-to-real gap 表(每个参数仿真 vs 实测),轨迹 Hausdorff 距离量化                   |

---

## 11. 项目核心卖点

| 卖点       | 说明                                                                       |
|----------|--------------------------------------------------------------------------|
| **完整**   | 不是单点算法,而是从仿真到实车的完整导航系统                                                   |
| **可解释**  | 每一层都有数学推导、设计文档与实验验证                                                      |
| **可量化**  | 每个版本都有显式量化指标,有图、有 RMSE、有参数扫描、有 sim-to-real gap                           |
| **可扩展**  | 从 V0 到 V6 的每一步都是向前兼容的迭代,而非重写;每个里程碑由 git tag 完整保存,trunk 只保留当前最佳设计             |
| **工程味重** | 现代 C++(modules、ADL 扩展点、PIMPL)、CMakePresets、GoogleTest、CI、ROS 2 / Nav2、Rerun 全家桶 |
| **有实车**  | Raspberry Pi 5 + 4WD(替换带编码器电机 + BNO055 IMU)真实部署,含 sim-to-real 叙事         |

---

## 一句话总结

> MiniNav 是一个以**现代 C++ 为核心**、面向室内移动机器人的定位
> 与导航系统,从运动学仿真出发,逐步叠加噪声建模、EKF 多传感器
> 融合、A\* 路径规划、Pure Pursuit 闭环跟踪、ROS 2 + Nav2 集成,并
> 最终在 Raspberry Pi 5 小车上完成室内自主导航的实车闭环验证。