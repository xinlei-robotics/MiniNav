# V1 里程计噪声模型 (Odometry Noise Model)

> **适用版本**：MiniNav V1  
> **依赖文档**：`docs/math/v0_kinematics.md`(差分驱动正向积分)  
> **参考文献**：Thrun, Burgard, Fox. *Probabilistic Robotics*, Ch. 5.3 — 5.4

---

## 1. 概述

本文档定义 V1 仿真引入的两类噪声及其数学模型：

1. **执行噪声 (Actuator Noise)** ： 由电机响应的不确定性引入，作用于命令 $(v, \omega)$ 与真实运动之间。
2. **传感器噪声 (Sensor Noise)** ： 编码器的量化误差与打滑误差，作用于真实轮速与编码器读数之间。

V1 在 V0 的基础上保留同一套差分驱动运动学积分 (`differential_drive_step`)，但通过在两个不同位置注入噪声，使"真值"与"里程计估计"产生可观测的发散。这种发散是后续 EKF 等状态估计模块存在的根本原因。

---

## 2. 符号约定

| 符号                                              | 含义                                  | 单位                     |
|-------------------------------------------------|-------------------------------------|------------------------|
| $v, \omega$                                     | 命令线速度、角速度                           | m/s, rad/s             |
| $\hat v, \hat \omega$                           | 带执行噪声的真实线速度、角速度                     | m/s, rad/s             |
| $v_l, v_r$                                      | 左/右轮真实线速度                           | m/s                    |
| $v_{l,\text{meas}}, v_{r,\text{meas}}$          | 经打滑后实际进入编码器的左/右轮线速度                 | m/s                    |
| $\hat v_{\text{enc}}, \hat \omega_{\text{enc}}$ | 由编码器读数解码得到的估计速度                     | m/s, rad/s             |
| $L$                                             | 轴距 (wheel base, 左右轮中心距)             | m                      |
| $r_w$                                           | 轮半径                                 | m                      |
| $N_{\text{tpr}}$                                | 编码器单圈 tick 数 (ticks per revolution) | —                      |
| $\Delta s_{\text{tick}}$                        | 单 tick 对应的弧长                        | m                      |
| $\Delta t$                                      | 仿真步长                                | s                      |
| $\alpha_1, \alpha_2, \alpha_3, \alpha_4$        | 执行噪声参数                              | s/m, s/rad, s/m, s/rad |
| $\sigma_{\text{slip}}$                          | 打滑噪声标准差                             | 无量纲                    |

---

## 3. 三种位姿的语义

V1 引入噪声后，仿真中存在三条语义不同的位姿轨迹：

| 位姿           | 来源                                                         | 含义                            |
|--------------|------------------------------------------------------------|-------------------------------|
| `cmd_pose`   | 命令 $(v, \omega)$ 直接积分                                      | "理想执行"轨迹，对应 V0 的 `truth_pose` |
| `truth_pose` | 真实速度 $(\hat v, \hat \omega)$ 积分                            | 带执行噪声、但完全可观测的真实运动             |
| `odom_pose`  | 编码器估计 $(\hat v_{\text{enc}}, \hat \omega_{\text{enc}})$ 积分 | 仅基于轮速读数的位姿估计                  |

V1 引入噪声之后 `truth_pose` 不再等价于"命令的完美执行"。这与 V0 的语义不同，调用代码必须明确这一区分。

---

## 4. 执行噪声：Velocity Motion Model

### 4.1 模型形式

参考 *Probabilistic Robotics*，差分驱动机器人的执行噪声采用基于车体速度 $(v, \omega)$ 的模型：

$$
\hat v = v + \mathcal N\!\bigl(0,\; \alpha_1 v^2 + \alpha_2 \omega^2\bigr)
$$

$$
\hat \omega = \omega + \mathcal N\!\bigl(0,\; \alpha_3 v^2 + \alpha_4 \omega^2\bigr)
$$

### 4.2 参数物理含义

| 参数         | 含义            | 物理来源                  |
|------------|---------------|-----------------------|
| $\alpha_1$ | 线速度自身导致的线速度方差 | 电机响应非线性、PWM 死区、负载补偿不足 |
| $\alpha_2$ | 角速度诱发的线速度方差   | 转弯时左右轮响应不对称导致的纵向漂移    |
| $\alpha_3$ | 线速度诱发的角速度方差   | 直行时轮子安装偏差、地面不平产生的偏航   |
| $\alpha_4$ | 角速度自身导致的角速度方差 | 转弯角速度跟踪误差             |

### 4.3 设计依据

**为什么方差是 $v^2, \omega^2$ 的线性组合：**

1. **零速度边界条件**：当 $v = 0, \omega = 0$ 时，方差严格为零。物理意义为车体静止时不产生执行误差，纯加性高斯噪声违反这一约束。
2. **标准差与速度成正比**：$\sigma = \sqrt{\alpha v^2} = \sqrt{\alpha} \cdot |v|$，符合"速度越高、误差幅度越大"的工程经验。

**交叉项的意义：**

交叉项捕捉了线运动与角运动之间的耦合：转弯不仅会偏离目标 $\omega$，还会污染 $v$；直行不仅会偏离目标 $v$，还会污染 $\omega$。这种耦合是真实差分驱动平台的固有特性。

### 4.4 参数辨识

工业实践中，$\alpha_i$ 由实测轨迹 vs ground truth 拟合得到。该版本目的是在完美模型中加入噪声来探究噪声的影响，因此仿真将其作为可调超参，配置在 `config/v1_default.yaml` 中。详见 §8 默认值。

### 4.5 输出语义

执行噪声的输出 $(\hat v, \hat \omega)$ 直接馈入 `differential_drive_step` 推进 `truth_pose`：

```
(v, ω) ──[执行噪声]──> (v̂, ω̂) ──[differential_drive_step]──> truth_pose
```

---

## 5. 差分驱动运动学

### 5.1 逆运动学 (车体速度 → 轮速)

$$
v_l = \hat v - \frac{L \hat \omega}{2}, \qquad
v_r = \hat v + \frac{L \hat \omega}{2}
$$

### 5.2 正运动学 (轮速 → 车体速度)

$$
\hat v_{\text{enc}} = \frac{v_{l,\text{meas}} + v_{r,\text{meas}}}{2}, \qquad
\hat \omega_{\text{enc}} = \frac{v_{r,\text{meas}} - v_{l,\text{meas}}}{L}
$$

### 5.3 几何直觉

- **线速度** = 两轮线速度的**平均值**(车体平移即两轮共同前进)。
- **角速度** = 两轮线速度的**差** ÷ **轴距**(差分越大则转弯越急；轴距越大、同样差分则对应的角速度越小)。

### 5.4 模块组织

`inverse_kinematics()` 与 `forward_kinematics()` 实现为自由函数(非成员)，与 `differential_drive_step` 同位于 `mininav.core.kinematics` 模块，体现三者属于同一物理模型的不同投影：

| 函数                        | 输入空间               | 输出空间               |
|---------------------------|--------------------|--------------------|
| `inverse_kinematics`      | 车体速度 $(v, \omega)$ | 轮速 $(v_l, v_r)$    |
| `forward_kinematics`      | 轮速 $(v_l, v_r)$    | 车体速度 $(v, \omega)$ |
| `differential_drive_step` | 车体速度 + 当前位姿        | 下一步位姿              |

---

## 6. 编码器模型

编码器仿真包含两个独立的物理过程：**打滑**(随机性，物理层)与**量化**(确定性，传感器层)。

### 6.1 打滑：Multiplicative Noise

$$
v_{l,\text{meas}} = v_l \cdot \bigl(1 + \mathcal N(0, \sigma_{\text{slip}}^2)\bigr)
$$

$$
v_{r,\text{meas}} = v_r \cdot \bigl(1 + \mathcal N(0, \sigma_{\text{slip}}^2)\bigr)
$$

**注**：左右轮的打滑噪声独立采样。两边的接触条件、磨损、地面粗糙度互相独立，若使用同一个噪声样本会丢失"打滑导致航向漂移"的关键统计特性(此时打滑只改变 $v$，不改变 $\omega$)。

**为什么使用乘性噪声？**

| 噪声形式                                                           | 静止时 ($v_l = 0$) 行为 | 物理一致性            |
|----------------------------------------------------------------|--------------------|------------------|
| 加性 $v_{\text{meas}} = v_l + \mathcal N(0, \sigma^2)$           | 编码器报告非零速度          | ✗ 违反物理(无牵引力则无打滑) |
| 乘性 $v_{\text{meas}} = v_l \cdot (1 + \mathcal N(0, \sigma^2))$ | 编码器报告零速度           | ✓ 与轮地接触物理一致      |

乘性形式额外保证打滑幅度随驱动力(以速度为代理)线性增长，符合接触力学经验。

### 6.2 量化：Tick 离散化

每个编码器 tick 对应固定弧长：

$$
\Delta s_{\text{tick}} = \frac{2 \pi r_w}{N_{\text{tpr}}}
$$

仿真步长内左轮累计 tick 数：

$$
\Delta \text{ticks}_l = \operatorname{round}\!\left( \frac{v_{l,\text{meas}} \cdot \Delta t}{\Delta s_{\text{tick}}} \right)
$$

(右轮同理。)

**量化误差性质**：`round` 引入的误差最大为 $\pm 0.5$ tick，对应 $\pm \Delta s_{\text{tick}} / 2$ 的距离误差。这是**确定性**误差(同样的输入产生同样的输出)，而非随机噪声。

### 6.3 量化误差的相对量级

绝对量化误差不变，但**相对**误差反比于速度。以 $r_w = 32.5$ mm、$N_{\text{tpr}} = 1024$ 为例：

| 速度       | $\Delta t$ 内距离 | 对应 ticks | 相对误差    |
|----------|----------------|----------|---------|
| 1.0 m/s  | 100 mm         | 500      | 0.1%    |
| 0.1 m/s  | 10 mm          | 50       | 1.0%    |
| 0.01 m/s | 1 mm           | 5        | **10%** |

(假设 $\Delta t = 0.1$ s，每 tick ≈ 0.2 mm。)

**工程结论**：低速时编码器读数相对不可靠，启动阶段尤其明显。这是真实里程计的固有特性，也是为什么实际系统通常会在低速时降低编码器观测的权重(或与 IMU 等其他源融合)。

### 6.4 噪声注入顺序

打滑与量化的注入顺序按物理因果链确定：

```
v_l (真实命令对应轮速)
   │
   ├─[打滑：物理层，轮地接触]──> v_l,meas
   │
   └─[量化：传感器层，编码器电路]──> Δticks_l
```

**先打滑、后量化**对应真实物理过程：滑移发生在轮子和地面之间(机械层)，编码器随后对实际转动的车轮编码计数(电路层)。反向顺序在物理上无对应过程，统计特性也不正确。

---

## 7. 里程计积分

### 7.1 双通道结构

V1 中存在两条独立的位姿积分通道：

| 通道  | 输入                                                | 积分函数                      | 输出           |
|-----|---------------------------------------------------|---------------------------|--------------|
| 真值  | $(\hat v, \hat \omega)$                           | `differential_drive_step` | `truth_pose` |
| 里程计 | $(\hat v_{\text{enc}}, \hat \omega_{\text{enc}})$ | `differential_drive_step` | `odom_pose`  |

里程计通道的输入由编码器读数经正运动学解码得到：

$$
\hat v_{\text{enc}} = \frac{v_{l,\text{meas}} + v_{r,\text{meas}}}{2}, \qquad
\hat \omega_{\text{enc}} = \frac{v_{r,\text{meas}} - v_{l,\text{meas}}}{L}
$$

其中 $v_{\cdot,\text{meas}}$ 由 `Δticks` 反解：$v_{\cdot,\text{meas}} \approx \Delta\text{ticks}_\cdot \cdot \Delta s_{\text{tick}} / \Delta t$。

### 7.2 设计准则：模型与输入解耦

两条通道**复用同一段** `differential_drive_step` 实现。这是 V0 阶段定下来的架构原则，V1 严格保持。其后果是：

> 真值与里程计估计的发散，**完全来自输入差异**(执行端 + 传感端注入的噪声)，与积分模型本身无关。

这种"控制变量"式的设计使得：

- 关闭执行噪声 ($\alpha_i = 0$) 时，差异完全来自编码器；
- 关闭编码器噪声 ($\sigma_{\text{slip}} = 0$, $N_{\text{tpr}} \to \infty$) 时，差异完全来自执行噪声；
- 全部关闭时，两条通道严格重合(回退到 V0 行为)。

这一性质既便于调试与单元测试，也使每一种噪声源对最终漂移的贡献可独立量化。

---

## 8. 默认参数值

配置文件：`config/v1_default.yaml`。

下表为初始建议值，可以实车标定或仿真扫描进一步调整。

| 参数                     | 初始值      | 单位    | 备注                   |
|------------------------|----------|-------|----------------------|
| $\alpha_1$             | $0.01$   | s/m   | 直行噪声主项               |
| $\alpha_2$             | $0.001$  | s/rad | 转弯对线速度的污染较弱          |
| $\alpha_3$             | $0.001$  | s/m   | 直行对角速度的污染较弱          |
| $\alpha_4$             | $0.01$   | s/rad | 转弯噪声主项               |
| $\sigma_{\text{slip}}$ | $0.02$   | —     | 对应约 2% 的打滑率          |
| $r_w$                  | $0.0325$ | m     | Adeept 4WD 标称值，需实测校准 |
| $N_{\text{tpr}}$       | $1024$   | —     | 典型增量编码器值             |
| $L$                    | $0.135$  | m     | Adeept 4WD 标称轴距，需实测  |

---

## 9. 实现要点速查

| 数学对象                  | 实现位置                                              |
|-----------------------|---------------------------------------------------|
| Velocity Motion Model | `mininav.core.noise` 模块的 `apply_actuator_noise()` |
| 逆/正运动学                | `mininav.core.kinematics` 模块的自由函数                 |
| 编码器仿真                 | `mininav.sensors.wheel_encoder` 模块                |
| 里程计积分                 | `mininav.localization.wheel_odometry` 模块          |
| 数值积分核                 | `differential_drive_step`(V0 既有，V1 复用)            |

---

## 10. 参考文献

1. Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*, Ch. 5.3 (Velocity Motion Model), Ch. 5.4 (Odometry Motion Model). MIT Press.
2. Siegwart, R., Nourbakhsh, I. R., & Scaramuzza, D. (2011). *Introduction to Autonomous Mobile Robots* (2nd ed.), Ch. 3 (Mobile Robot Kinematics), Ch. 4 (Perception). MIT Press.
