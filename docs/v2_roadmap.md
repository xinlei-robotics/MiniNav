# MiniNav V2 执行路线图

> 本文档是 V2 阶段开工前的**前瞻性计划**,记录 V2 要解决的问题、
> 设计决策、PR 拆分、数学骨架与待定项。V2 完成后会有对应的
> `v2_summary.md` 作为回顾性总结(类似 `v0_summary.md` / `v1_summary.md`)。

---

## 0. V2 要解决的问题

### 0.1 V1 留下的核心问题

V1 把"odom 漂移"从一个理论问题变成了项目里一条可测、可复现的曲线:

- 20s `default` preset 下,position error 累计到 ~2m,yaw error 累计到 ~20°
- 这个漂移**不是 bug**,是 wheel odometry 的根本局限——只用一种传感器、
  开环积分,任何小误差都会随时间累计

V2 的核心任务是引入**第二种传感器(IMU)** + 一个**概率融合框架(EKF)**,
让多传感器互相约束,把估计 pose 拉回到接近真值。

### 0.2 V2 的"看得见的成果"

V2 完成时,Rerun Viewer 里能同时看到**四条轨迹**:

| Trail                              | 含义                          |
|------------------------------------|-----------------------------|
| `/world/robot/cmd_traj/trail`      | V0 留下的理想执行轨迹                |
| `/world/robot/truth/trail`         | V1 引入的真值轨迹(带 actuator 噪声)   |
| `/world/robot/odom/trail`          | V1 引入的纯 odom 估计(带漂移)        |
| `/world/robot/ekf/trail`           | **V2 新增的 EKF 估计(显著接近真值)**   |

同时 Python 分析脚本输出三组 RMSE:`RMSE(truth, odom)` /
`RMSE(truth, ekf)` / `RMSE(truth, ekf_with_bias)`,**至少 50% 的
position RMSE 降低**作为 V2 的"verifiable interview soundbite"。

### 0.3 V2 解决什么、不解决什么

**V2 解决**:

- 引入 IMU 传感器仿真模型
- 引入 EKF 框架(`predict` + `update`)
- 实现 encoder + IMU 的双传感器观测融合
- 通过 state augmentation 学习并修正 gyro bias
- 把欧拉积分升级为 RK4
- 量化 RMSE 减少幅度

**V2 不解决**:

- **不引入闭环控制**:命令仍然是 V0 留下的开环 `StagedCommandSource`。
  Pure Pursuit 等控制器在 V4。
- **不引入地图与全局规划**:占据栅格 + A\* 在 V3。
- **不引入 ROS 2**:V4 才打包成 ROS 节点。EKF 在 V2 阶段是裸 C++ 库。
- **不做 SLAM**:V2 是 localization with **known** robot starting pose,
  不解决"机器人不知道自己在哪"的问题。
- **不上 IMU 的 accelerometer**:只用 gyro。2D 差分驱动场景下,
  加速度计对 yaw 估计没增量贡献,徒增工程复杂度。V6 实车阶段如果发现
  加速度计有用再加。

---

## 1. 关键设计决策

### 1.1 State vector:5D `(px, py, θ, v, ω)`

V2 的状态向量定义为:

$$
\mathbf{x}_k = \begin{bmatrix} p_x \\ p_y \\ \theta \\ v \\ \omega \end{bmatrix}_k
$$

其中 `v` 是机器人**体坐标系下的前向速度**(差分驱动机器人无侧向速度),
`ω` 是 yaw rate。

**为什么选 5D 而不是 3D**:

最直觉的方案是 3D 状态 `(px, py, θ)` + odom 作为 control input(Thrun
"odometry motion model" §5.4)。这种 EKF 中 encoder 噪声通过 G·M·G^T
传播为过程噪声 Q。

这种方案的问题在 IMU 的引入:**IMU 观测的是 ω,而 ω 在 3D state 里不存在**。
要让 IMU 作观测有几条路:

1. **把 ω 临时塞进 state(本质上就是回到 4D 或 5D)**
2. **IMU 用作 control input 的"修正"**(complementary filter,
   不是 Bayesian EKF)
3. **dead reckon yaw separately**(把 yaw 单拎出来,encoder 算 (px, py),
   IMU 算 θ,然后 cross-covariance 融合,工程复杂度高)

5D 状态把 (v, ω) 提升为隐藏状态,两个传感器都成为对它们的观测:

- Encoder 观测 `(v, ω)`
- IMU 观测 `ω`

这是教科书意义上的"多传感器对同一隐状态的融合",数学最干净,代码最直白,
面试讲故事最顺畅。

**Constant velocity model 的物理意义**:

过程模型假设"如果没有新信息,机器人保持当前速度"。这不是说现实中机器人
不变速,而是说"我们不预知它怎么变速,把变速作为不确定性建模进 Q"。
v 和 ω 在预测步是 random walk,在 update 步被传感器测量约束回来。

### 1.2 Encoder 和 IMU 都作观测,不用 odom 作为 control

V1 的 `WheelOdometry` 仍然保留——它不是 EKF 的输入,而是**EKF 的对照基线**。
PR2 完成后,你能跑三种估计并排:

- `odom` (V1 的纯积分)
- `ekf-encoder-only` (EKF 但只用 encoder 观测)
- `ekf-encoder+imu` (PR3 之后的完整融合)

第二种应该和第一种**数值几乎一致**——这是 EKF 实现正确性的强检验:
当只有一个传感器、且 R 设得合理时,EKF 退化为该传感器的开环积分。
**任何显著不同都暴露 EKF 实现 bug**——这是非常有价值的回归测试。

### 1.3 Bias 单独通过 state augmentation 在 PR4 引入

PR1-3 用 5D state,gyro 测量模型是 `ω_imu = ω + N(0, σ²)`,没有 bias 项。
PR4 把 state 扩到 6D:

$$
\mathbf{x}_k = \begin{bmatrix} p_x & p_y & \theta & v & \omega & b_\omega \end{bmatrix}^T_k
$$

测量模型相应变为 `ω_imu = ω + b_ω + N(0, σ²)`。

**为什么 PR4 单独做、不一开始就上**:

PR3 跑完时,EKF 估计已经比纯 odom 好,但**仍然有系统偏差**——因为 IMU 有
未建模的 bias,filter 把它误以为是真实的 ω 信号。这时候 PR4 引入
bias 维度,你在 Rerun Time Series 视图里能直接看到:

- `b_ω` 从初始值 0 开始
- filter 在前几秒"学到" bias 是 ~0.02 rad/s
- 估计的 yaw error 随之下降一个台阶

这是 state augmentation 最有说服力的演示方式——**"看,filter 学到了
我没告诉它的东西"**。

### 1.4 RK4 推到 PR5,与 EKF 实现解耦

V1 summary §7.2 承诺"V2 引入 EKF 时切换到 RK2/RK4"。我们**部分兑现**:
RK4 在 PR5 实现,不和 EKF 同时做。

理由是工程纪律:EKF 实现和数值积分方法是**正交的**两件事,绑在一起做
会让"性能改进归因"变模糊。分开做:

- PR1-4 用 Euler:能看到 EKF 把漂移降了多少
- PR5 换 RK4:能再额外看到 RK4 比 Euler 又降了多少

每一步都有独立的"看得见的进步",debug 时也能精确定位是哪一步引入了
回归。

### 1.5 Process noise Q 与 measurement noise R 的设计哲学

V2 引入两个 EKF 调参旋钮:Q(过程噪声协方差)和 R(观测噪声协方差)。

**Q 来自 V1 留下的 actuator 噪声参数**——它**不是凭空调出来的**。
V1 的 Velocity Motion Model 已经定义了 (α₁, α₂, α₃, α₄),这些参数
精确描述了真实速度相对命令速度的协方差结构。V2 的 Q 直接由 (v, ω) 的
方差推导:

$$
Q_{vv} = \alpha_1 v^2 + \alpha_2 \omega^2, \quad
Q_{\omega\omega} = \alpha_3 v^2 + \alpha_4 \omega^2
$$

**R 来自 V1 留下的 sensor 噪声参数**:

- `R_encoder = diag(σ_v_enc², σ_ω_enc²)` 由 V1 的 `σ_slip` 推导
- `R_imu = σ_imu²` 由 V2 新引入的 IMU 噪声参数

这样 Q 和 R 都不是"凭手感调出来的参数",而是从更底层的物理参数推导
而来。**任何对 Q 或 R 的修改都对应一个可解释的物理改动**——这是
"工业级 Bayesian 估计"的核心 discipline。

PR5 阶段会留一组 `--q-scale` `--r-scale` CLI 参数允许手动微调,但缺省
应该是从物理参数推出来的。

---

## 2. PR 拆分与里程碑

V2 拆为 6 个 PR,每个 PR 都有清晰的"merged when X works"标准。

```
PR1  EKF math foundation + predict-only
PR2  Encoder observation update (first Kalman gain)
PR3  IMU model + fusion (the "magic moment")
PR4  Gyro bias state augmentation
PR5  RK4 + tuning + NIS + analysis scripts
PR6  Experiment docs + v2_summary + README + CHANGELOG
```

> **拆分说明(执行期调整)**:原计划把实验文档与 README 一并放进 PR5,
> 但代码/分析(RK4、NIS、Q/R 调参、分析脚本)与对外文档(实验报告、
> summary、README、CHANGELOG)是两类性质不同的工作,绑在一个 PR 里会让
> review 责任不清。故 PR5 收窄为"代码 + 分析脚本",文档统一拆出为 PR6。

每个 PR 一节,下面展开。

---

### PR1: EKF 数学基础 + 5D state + predict-only

#### 目标

实现一个**只有 predict、没有 update**的 5D EKF。这是一个故意"不完整"
的版本——它的价值不在于产生好的估计,而在于把 EKF 的数学骨架立起来:
状态向量、过程模型、Jacobian、协方差传播、`SimStateV2` 的扩展。

完成后,EKF 估计的 mean 是一条随时间偏离真值的轨迹(因为 (v, ω) 是
random walk,没有 measurement 约束),协方差 ellipse 在 Rerun 2D 视图里
**无界增长**——非常清晰地展示"为什么必须有 observation"。

#### 数学骨架

5D 状态在时间步 dt 下的过程模型:

$$
f(\mathbf{x}_k) = \begin{bmatrix}
p_x + v \cos\theta \cdot dt \\
p_y + v \sin\theta \cdot dt \\
\theta + \omega \cdot dt \\
v \\
\omega
\end{bmatrix}
$$

Jacobian F = ∂f/∂x:

$$
F = \begin{bmatrix}
1 & 0 & -v \sin\theta \cdot dt & \cos\theta \cdot dt & 0 \\
0 & 1 &  v \cos\theta \cdot dt & \sin\theta \cdot dt & 0 \\
0 & 0 & 1 & 0 & dt \\
0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 1
\end{bmatrix}
$$

协方差预测:

$$
P_{k+1}^- = F P_k F^T + Q
$$

其中 Q 是 5×5 过程噪声矩阵,(v, v)、(ω, ω) 项由 V1 actuator 模型推导,
其他维度的 Q 通过单步弧长引入的位置/角度不确定性建模。

#### 新增 / 修改文件

```
src/
├── core/
│   ├── types.{ixx,cpp}            # + SimStateV2 (内含 EkfState5)
│   └── csv_format.{ixx,cpp}       # + csv_header/csv_row(SimStateV2)
├── localization/                  # 在 V1 的 WheelOdometry 旁边新增
│   ├── ekf_state.{ixx,cpp}        # ★ EkfState5 (mean + covariance)
│   └── ekf.{ixx,cpp}              # ★ Ekf::predict (无 update)
└── apps/
    └── sim_v2_main.cpp            # ★ V2 主循环,sim_v0/v1 保留

tests/
└── localization/
    ├── ekf_predict_tests.cpp                  # ★ 单步 predict 行为
    └── ekf_jacobian_finite_diff_tests.cpp     # ★ 有限差分验证 F
```

#### 关键测试

**`JacobianMatchesFiniteDifference`**——这是 PR1 的灵魂测试。手推的 F
可能写错(很容易把符号搞反或把 dt 漏掉一项),有限差分是机械的"反复跑
`f` 并差分"的过程,不会出错。两者不一致就直接报错。

```cpp
TEST(EkfJacobian, MatchesFiniteDifference) {
    EkfState5 x = ...;  // 随机一个非零状态
    Mat5 F_analytic = compute_F(x, dt);
    Mat5 F_numeric = compute_F_finite_diff(x, dt, 1e-6);
    EXPECT_TRUE(F_analytic.isApprox(F_numeric, 1e-4));
}
```

如果你想在面试里展示一手数学功底,**这条测试是最值得拿出来讲的**——
"我手推 Jacobian 并用有限差分独立校验"是 robotics 工程师识别同行的
信号。

#### Deliverable / "Merged when"

- [ ] `sim_v2 --no-viz --seed 42 --preset default` 跑得通
- [ ] 输出 `traj_v2.csv` 包含 5 个新列:`ekf_x, ekf_y, ekf_yaw, ekf_v, ekf_omega`
- [ ] Rerun 中能看到 EKF mean 轨迹(预期偏离真值,这是正常的)
- [ ] `JacobianMatchesFiniteDifference` 测试通过
- [ ] 协方差正定性检验:每步后 `P` 仍是 symmetric positive definite

---

### PR2: Encoder 作为第一个观测——完成 Kalman update

#### 目标

PR1 的 EKF 只有 predict,covariance 无界增长。PR2 把 encoder 接入作
**第一个观测**,完成 Kalman update 步骤。完成后:

- EKF 估计的 mean 和 V1 的 wheel odometry **数值上几乎重合**
  (encoder 是唯一信息源时,EKF 退化为该传感器的开环积分)
- 协方差**有界**——稳态下 trace(P) 不再无限增长

这个"和 wheel odom 重合"的现象是 EKF 实现正确性的强 sanity check。

#### 数学骨架

Encoder 观测模型:

$$
\mathbf{z}_{enc} = h_{enc}(\mathbf{x}) + \mathbf{v}_{enc}, \quad
h_{enc}(\mathbf{x}) = \begin{bmatrix} v \\ \omega \end{bmatrix}, \quad
H_{enc} = \begin{bmatrix} 0 & 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 0 & 1 \end{bmatrix}
$$

注意 z_enc 不是直接读 EncoderTicks——而是先经过 `WheelOdometry::forward_kinematics`
解码为 `(v̂, ω̂)`。这一步复用 V1 的代码,不重写。

R_enc 从 V1 的 `σ_slip` 推导(轮速到 v/ω 的传递关系):

$$
R_{enc} = \begin{bmatrix}
\sigma_v^2 & 0 \\
0 & \sigma_\omega^2
\end{bmatrix}, \quad
\sigma_v^2 = \frac{\sigma_{slip}^2 \cdot r^2}{2}, \quad
\sigma_\omega^2 = \frac{\sigma_{slip}^2 \cdot r^2}{2 W^2}
$$

(r = 轮半径, W = 轮距)

Kalman update:

$$
\begin{aligned}
\mathbf{y}_k &= \mathbf{z}_k - h(\mathbf{x}_k^-) \quad \text{(innovation)} \\
S_k &= H P_k^- H^T + R \\
K_k &= P_k^- H^T S_k^{-1} \quad \text{(Kalman gain)} \\
\mathbf{x}_k^+ &= \mathbf{x}_k^- + K_k \mathbf{y}_k \\
P_k^+ &= (I - K_k H) P_k^- \quad \text{(Joseph form 推荐)}
\end{aligned}
$$

**Joseph form 协方差更新**:
$$
P_k^+ = (I - K_k H) P_k^- (I - K_k H)^T + K_k R K_k^T
$$

数学上等价于简化形式,但**对数值精度更鲁棒**——它保证 P 永远是 PSD
(positive semi-definite)。简化形式在浮点累计后可能轻微负定,导致
filter 发散。**Joseph form 是工业级 EKF 的标配**——一定要用。

#### 新增 / 修改文件

```
src/localization/
└── ekf.{ixx,cpp}                  # + Ekf::update_encoder (Joseph form)

tests/localization/
├── ekf_update_encoder_tests.cpp        # ★ 单步 update 数值正确性
└── ekf_degenerates_to_odom_tests.cpp   # ★ EKF-encoder-only ≈ WheelOdometry
```

#### 关键测试

**`EkfEncoderOnlyDegeneratesToWheelOdometry`**:

```cpp
// 跑 20s,EKF 只用 encoder 观测;同时跑 V1 的 WheelOdometry
// 两条轨迹应在数值精度内重合
TEST(EkfDegradation, EncoderOnlyMatchesWheelOdom) {
    auto [ekf_traj, odom_traj] = run_both_estimators(seed=42, preset="default");
    for (size_t i = 0; i < ekf_traj.size(); ++i) {
        EXPECT_NEAR(ekf_traj[i].pose.x(), odom_traj[i].pose.x(), 1e-6);
        EXPECT_NEAR(ekf_traj[i].pose.y(), odom_traj[i].pose.y(), 1e-6);
        EXPECT_NEAR(ekf_traj[i].pose.yaw(), odom_traj[i].pose.yaw(), 1e-6);
    }
}
```

这条测试是非常严苛的——它把"EKF 实现 bug-free"和"WheelOdometry
实现 bug-free"绑在一起。两者数值不一致就立刻暴露问题。

#### Deliverable / "Merged when"

- [ ] EKF predict + update 两步都跑得通
- [ ] PR1 中无界增长的协方差现在 bounded(trace(P) 进入稳态)
- [ ] `EkfEncoderOnlyDegeneratesToWheelOdometry` 通过
- [ ] Rerun 中能看到 covariance ellipse 大小稳定,不再增长

---

### PR3: IMU 仿真模型 + 第二个观测 — fusion 时刻

#### 目标

引入 IMU(只用 gyro)作为第二个观测源。完成后:

- EKF 估计的 yaw 显著优于 wheel odom 的 yaw
- position error 因 yaw 改善而间接改善(因为 (px, py) 通过过程模型耦合
  到 θ)
- Rerun 里 `ekf trail` 与 `truth trail` 明显比 `odom trail` 更贴合

这是 V2 的"WOW moment"——第一次能在视觉上看到融合的力量。

#### 数学骨架

IMU 测量模型(PR3 阶段,无 bias):

$$
\mathbf{z}_{imu} = h_{imu}(\mathbf{x}) + \mathbf{v}_{imu}, \quad
h_{imu}(\mathbf{x}) = \omega, \quad
H_{imu} = \begin{bmatrix} 0 & 0 & 0 & 0 & 1 \end{bmatrix}
$$

R_imu = σ_imu² 是 IMU 噪声参数(预设值,V6 实车阶段标定)。

EKF 主循环每步执行**三个阶段**:

```
1. predict       (run with dt)
2. update_encoder (with z = forward_kinematics(EncoderTicks))
3. update_imu    (with z = imu.measure(true_omega))
```

观测的先后顺序在数学上**不影响最终估计**(对线性测量, EKF update 顺序
无关),但在数值上有微小差异。我们固定先 encoder 后 IMU 的顺序,
保证可复现。

**IMU 仿真模型(PR3)**:

```cpp
struct ImuParams {
    double sigma_omega = 0.005;  // rad/s, gyro 白噪声标准差
};

class ImuModel {
public:
    ImuModel(ImuParams, RngFactory&);
    double measure(double true_omega);  // 返回带噪 ω_imu
};
```

仿真器在每个时间步:`ω_imu = ω_true + N(0, σ²)`。`ω_true` 来自
`ActuatorModel` 的输出(与 V1 一致)。

IMU 模型放在 `src/sensors/imu_model.{ixx,cpp}`,**与 encoder 平级**。
不复用 actuator 的 RNG——用 `RngFactory::make_engine("imu_gyro_noise")`
拿到独立子种子(PR3 自动保证 V1 已有的随机序列不被扰动)。

#### 新增 / 修改文件

```
src/
├── sensors/
│   └── imu_model.{ixx,cpp}        # ★ ImuModel + sigma_omega
├── localization/
│   └── ekf.{ixx,cpp}              # + Ekf::update_imu
└── apps/
    └── sim_v2_main.cpp            # 主循环加 IMU + 第二次 update

tests/sensors/
└── imu_model_tests.cpp            # ★ IMU 噪声统计性质 + RNG 独立性

tests/localization/
└── ekf_fusion_tests.cpp           # ★ 双传感器 RMSE < 单传感器 RMSE
```

#### 关键测试

**`FusionImprovesRmseOverSingleSensor`**:

```cpp
// 三种估计器跑同一条 truth,比较 RMSE
auto rmse_odom = compute_rmse(truth, odom_only);
auto rmse_ekf_enc = compute_rmse(truth, ekf_encoder_only);
auto rmse_ekf_fused = compute_rmse(truth, ekf_encoder_imu);

EXPECT_LT(rmse_ekf_fused.yaw, rmse_ekf_enc.yaw * 0.7);     // yaw 至少改善 30%
EXPECT_LT(rmse_ekf_fused.position, rmse_ekf_enc.position * 0.8);
```

具体比例阈值会根据 PR2 完成后实测调整。这条测试**用代码定义了
"什么叫融合成功"**——不是肉眼看图,是数值阈值。

#### Deliverable / "Merged when"

- [ ] IMU sim 模型与 RNG 独立性测试通过
- [ ] EKF update_imu 数值正确
- [ ] FusionImprovesRmseOverSingleSensor 通过
- [ ] Rerun 中肉眼可见 ekf trail 比 odom trail 更贴 truth trail
- [ ] CSV 输出新增 `imu_omega` 列

---

### PR4: Gyro bias 通过 state augmentation 学习

#### 目标

PR3 完成后,EKF 已经比 odom 好,但仍有**系统残差**——因为真实 IMU
有 bias,filter 把它当成真实的 ω 信号。PR4 把 bias 加入状态向量,
让 filter 在线学习并修正它。

完成后:

- 状态向量扩展到 6D
- Rerun Time Series 视图能看到 `b_ω` 从 0 开始,几秒内"学到"真值附近
- yaw error 在 bias 学好后下降一个明显台阶

#### 数学骨架

扩展状态:

$$
\mathbf{x}_k = \begin{bmatrix} p_x & p_y & \theta & v & \omega & b_\omega \end{bmatrix}^T_k
$$

过程模型新增 bias 的 random walk:

$$
b_{\omega,k+1} = b_{\omega,k} + N(0, q_b)
$$

q_b 应该非常小(bias 慢漂),典型值 q_b ≈ (1e-4)² rad/s · √dt。

修改后的 F(只展示 bias 新增的行列):

$$
F = \begin{bmatrix}
\ddots & \\
& 1
\end{bmatrix}_{6 \times 6}
$$

(bias 的对角项,且 bias 不耦合到任何其他状态)

IMU 观测模型变更:

$$
h_{imu}(\mathbf{x}) = \omega + b_\omega, \quad
H_{imu} = \begin{bmatrix} 0 & 0 & 0 & 0 & 1 & 1 \end{bmatrix}
$$

注意 H_imu 的 b_ω 那一列是 **1**——这是关键:它告诉 filter "IMU 测量
中包含 bias 的贡献"。

Encoder 观测保持不变(encoder 不感知 IMU bias)。

**仿真层面也要更新**:IMU 模型新增 bias 字段:

```cpp
struct ImuParams {
    double sigma_omega = 0.005;
    double bias_omega_init = 0.02;      // ★ 新增:初始 bias 值(rad/s)
    double bias_random_walk = 1e-4;     // ★ 新增:bias 慢漂强度
};
```

仿真中 bias 实际值也按 random walk 演化(给 filter 学一个移动靶子)。
PR4 阶段把 bias_random_walk 设为 0(常数 bias)先看 filter 能否收敛;
确认后再开 random walk,看 filter 能否跟踪。

#### Observability 分析

bias 是否可观测(filter 能否学到它)取决于状态空间结构:

- **仅 IMU 观测时**:bias 与真实 ω **不可分离**——`z_imu = ω + b`,
  filter 无法区分大 ω 小 bias 和小 ω 大 bias
- **加上 encoder 观测**:encoder 单独提供 ω 的信息(不含 bias),
  现在 `b = z_imu - ω_enc`(在噪声范围内),bias **可观测**

这是 sensor fusion 在 SLAM/state estimation 中最经典的 observability
论点之一。**PR4 的代码看起来只是加了一个状态维度,但能讲出"为什么
bias 在双传感器场景下可观测,在单传感器下不可观测"是大幅加分项**。

#### 新增 / 修改文件

```
src/
├── sensors/imu_model.{ixx,cpp}    # + bias_omega + bias_random_walk
├── localization/
│   ├── ekf_state.{ixx,cpp}        # 升级到 6D state
│   └── ekf.{ixx,cpp}              # 升级 F, H_imu
└── apps/sim_v2_main.cpp           # CSV 输出 b_omega

tests/localization/
└── ekf_bias_observability_tests.cpp   # ★ bias 能被 filter 学到
```

#### 关键测试

**`GyroBiasIsLearnedWithinFiveSeconds`**:

```cpp
TEST(EkfBias, LearnedWithin5Seconds) {
    constexpr double TRUE_BIAS = 0.02;
    auto traj = run_ekf_with_bias(true_bias=TRUE_BIAS, duration=20s);

    // t=5s 之后,estimated bias 应该收敛到真值附近
    for (auto& state : traj.slice(after=5.0)) {
        EXPECT_NEAR(state.ekf_bias_omega, TRUE_BIAS, 0.005);
    }
}
```

这条测试要求 filter 在 5 秒内学到 ±25% 容差内的 bias。如果失败,
通常是 q_b 设太小(filter 不肯调整 bias)或 R_imu 设太大(filter
不信任 IMU)。

#### Deliverable / "Merged when"

- [ ] State 扩到 6D,所有现有测试仍然通过
- [ ] `GyroBiasIsLearnedWithinFiveSeconds` 通过
- [ ] Rerun Time Series 能看到 b_omega 学习曲线
- [ ] V1 summary §7.4 提到的"bias 是 V2 状态扩展"承诺兑现


---

### PR5: RK4 积分 + 调参 + NIS + 分析脚本

#### 目标

V2 的"engineering polish PR"——补齐数值积分升级、参数调优、filter
consistency 诊断,以及把这些都暴露出来的可视化分析脚本。对外文档
(实验报告 / summary / README / CHANGELOG)拆到 PR6,见下。完成后,
V2 在技术深度与工程严谨度上 production-ready,只差成文对外。

#### 内容拆分

**(a) RK4 积分(兑现 V1 §7.2 的承诺)**:

把 `differential_drive_step` 升级到 4 阶 Runge-Kutta。新建
`src/core/integrators.{ixx,cpp}` 提供 `rk4_step` 自由函数,接口与
`differential_drive_step` 一致——`sim_v0`/`sim_v1`/`sim_v2` 都能切换
集成器。

测试:`CircularMotionReturnsNearStart` 在 RK4 下容差从 ~10cm 收紧到
~1cm 量级。

**(b) Q / R 参数调优 + CLI 旋钮**:

CLI 加入 `--q-scale FLOAT` 和 `--r-scale FLOAT`,允许临时缩放 Q 和 R
做敏感性分析。配合 NIS 检验(下面)做 filter consistency。

**(c) NIS (Normalized Innovation Squared) consistency check**:

NIS 是 filter 自我诊断的工业级工具。每步算:

$$
\epsilon_k = \mathbf{y}_k^T S_k^{-1} \mathbf{y}_k
$$

理论上 ε_k 应服从 χ² 分布(自由度 = 测量维度)。统计 20 秒内 NIS 落在
95% χ² 置信区间内的比例,接近 95% 说明 filter "consistent"(Q 和 R
匹配真实噪声水平),否则需调参。

加入 CSV 输出 `nis_encoder` `nis_imu` 列,分析脚本可视化。这一项在
面试中**有相当杀伤力**——很多人会做 EKF,但不会做 consistency 验证。

**(d) 分析脚本扩展**(按版本组织到 `scripts/v2/`):

- `analyze_ekf.py`:三轨迹(truth / odom / ekf)2D 对比、三 RMSE 时间
  曲线(含 `--no-bias` 的三路 RMSE)、6 个状态维度的 estimation error +
  3σ 包络曲线、NIS 95% 区间检验图、bias 学习曲线
- `analyze_integrator.py` + `sweep_integrator.py`:RK4-vs-Euler 的精度
  归因实验(多 seed 平均)
- `analyze_covariance.py`:位置协方差椭圆随时间演化(静态叠加 + 几何量
  时间序列 + 动画),作为 observability 的直观证据

#### Deliverable / "Merged when"

- [ ] RK4 集成器实现 + 测试容差收紧
- [ ] Q/R CLI 旋钮(`--q-scale` / `--r-scale`)
- [ ] NIS 输出(CSV `nis_encoder` / `nis_imu` 列 + 数值正确性测试)
- [ ] `scripts/v2/` 分析脚本跑通,产出全部图件
- [ ] `results/` 与 `scripts/` 按版本(v0/v1/v2)组织

---

### PR6: 实验文档 + v2_summary + README + CHANGELOG

#### 目标

V2 的"对外成文 PR"——把 PR1–5 的代码、数学、实验结果固化成可供他人
(尤其招聘者)阅读的文档。本 PR 不改任何 `src/` 代码,只产出文档与
对图件的引用,基于 PR5 已生成的 `results/v2/` 图件成文。

#### 内容拆分

**(a) 实验文档** `docs/experiments/v2_ekf_fusion.md`,3000 字左右:

```
1. 问题陈述(V1 的漂移问题)
2. 数学建模(5D + 6D state, Jacobian, observation models)
3. 实验设计(三档 preset × 三种估计器 × 多 seed)
4. 实验结果(RMSE 表 + NIS consistency 表 + 协方差椭圆 + RK4 归因)
5. 结论(EKF 改善幅度 + bias 学习能力 + 系统限制)
```

这份文档是 V2 阶段的**核心面试素材**。

**(b) V2 summary** `docs/v2_summary.md`:回顾性总结,体例对齐
`v0_summary.md` / `v1_summary.md`,记录设计决策的事后复盘与 V3 的接口。

**(c) 更新 README**:

- 首屏图换成 V2 的"truth / odom / ekf 三轨迹"对比
- Latest milestone 节切到 V2
- Roadmap 表格 V2 标 ✅,V3 标 → next
- Engineering foundations 节加入"manual Jacobian + finite-diff
  validation"和"Joseph form covariance update"两条

**(d) CHANGELOG**:新增 V2 条目,汇总 PR1–6 的对外可见改动。

#### Deliverable / "Merged when"

- [ ] `docs/experiments/v2_ekf_fusion.md` 写完
- [ ] `docs/v2_summary.md` 写完(回顾性总结)
- [ ] README 更新到 V2
- [ ] CHANGELOG 新增 V2 条目

---

## 3. 不在 V2 范围内的事

V2 是 portfolio 项目的一个版本,**不是一篇博士论文**。下面这些有价值
但不属于 V2 的事项:

| 项                                     | 为什么不做                                 | 留到哪个版本    |
|---------------------------------------|---------------------------------------|-----------|
| **UKF (Unscented KF)**                | EKF 在 V2 的非线性度足够轻,UKF 收益有限            | 不做        |
| **Particle Filter 对比**                | 需要重新实现一遍 + 对比,工程 ROI 低                | 不做        |
| **完整 IMU(含 accelerometer)**           | 2D 差分驱动场景下加速度计对 yaw 估计无增量贡献           | V6 实车若有需要 |
| **EKF SLAM**                          | 需要引入 landmark + 数据关联,跨度太大             | 完全不在路线图   |
| **Loosely vs tightly coupled fusion** | 我们做的是 loosely coupled, 对比 tightly 价值低 | 不做        |

把 V2 限定在"EKF + 双传感器 + bias + RK4"是已经相当饱满的工作量,
不要扩张范围。

---

## 4. 待你决定的开放问题

下面这些问题不影响 V2 整体方向,但影响实现细节。最好在 PR1 启动前
拍板,以免中途返工。

### 4.1 IMU bias 初始值如何处理

PR4 中 EKF 启动时 b_ω 初始估计:

b_ω 初值 = 0,covariance 初值 = (0.1)² (大)

### 4.2 是否在 PR3 加 outlier rejection

真实 IMU 偶尔会有 spike(测量异常值)。EKF 标准做法是 chi-square
test:innovation 超过 3σ 就拒绝该测量。

**选择**:**PR3 不做**,等 V6 实车阶段如果 IMU 真出现 spike 再加。
仿真器的 IMU 是干净高斯,不会有 outlier,加了反而引入未用代码。

### 4.3 CSV 列爆炸怎么办

V2 后 `traj_v2.csv` 大约会有 ~30 列(V1 已经 13 列, +6 EKF state, +6
P 对角, +1 IMU obs, +2 NIS, +bias 等)。

**PR1-4 接受 30 列,CSV 文件略胖但仍 parseable,PR5 评估是否切到 Parquet**。CSV 在 V2
仍然是 single source of truth,简单可靠;Parquet 是 V3+ 大数据量
的备选。

### 4.4 什么时候做 NIS

NIS consistency check**统一在 PR5**。PR1-4 应该聚焦"功能正确",NIS 是
"诊断工具"——两者混在一起做容易让 PR 责任不清。

### 4.5 sim_v0 / sim_v1 是否保留

V2 引入 sim_v2 后,前两个 simulator 依然保留为可执行档,但不再维护。它们的存在对调试很有帮助，且CMake编译开销很小。

---

**节奏建议**:

- 每个 PR 在合并前**自己 review 一次**——把 diff 翻一遍,问"如果是
  招聘者读这个 PR,我能讲清楚每一行的设计动机吗?"
- PR3 和 PR4 之间留一个**思考缓冲期(3-5 天)**——在 PR3 完成后,
  不立刻开 PR4,先和我讨论 observability 的细节、试着自己推一遍
  bias 的可观性证明。bias 这块的数学比其他 PR 更微妙。
- PR6 的实验文档**和 PR1–5 的代码并行写**,不要"代码全做完再补文档"
  ——会漏掉很多关键细节。

---

## 6.面试中能讲深入的话题

- **Jacobian 推导 + 有限差分验证**(数学功底信号)
- **Joseph form covariance update**(工业实现细节)
- **EKF 退化为单传感器积分的 sanity check**(工程纪律信号)
- **Bias 在双传感器场景下的可观测性**(estimation theory 信号)
- **NIS consistency check**(诊断能力信号)
- **Q 和 R 由物理参数推导**(系统级思维信号)


---
