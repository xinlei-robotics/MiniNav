# Pure Pursuit 路径跟踪:几何、线性化、滞后稳定性与安全裕度

> 本文是 MiniNav V4 闭环路径跟踪的数学说明:Pure Pursuit 的转向几何、直线附近的
> 线性化与"响应只取决于走过的距离"、执行器滞后与零阶保持下的稳定性判据、
> 圆弧与折线转角上的误差、安全裕度预算,以及"控制误差 vs 定位误差"的分解。
> 每条结论最后都落到 `config/nav.yaml` 的某个默认参数上(§10)。对应代码在
> `src/control/`(`PurePursuitController`、`VelocitySmoother`)与 `src/apps/sim/nav_mode.cpp`;
> 定量验证见 [`docs/experiments/v4_control.md`](../experiments/v4_control.md)(E1–E4)。

## 目录

1. [问题与符号](#1-问题与符号)
2. [Pure Pursuit 几何](#2-pure-pursuit-几何)
3. [直线附近的线性化](#3-直线附近的线性化)
4. [执行器滞后与零阶保持](#4-执行器滞后与零阶保持)
5. [圆弧上的零稳态误差](#5-圆弧上的零稳态误差)
6. [折线转角:尺度律与小角度理论](#6-折线转角尺度律与小角度理论)
7. [安全裕度预算](#7-安全裕度预算)
8. [误差分解与定位漂移](#8-误差分解与定位漂移)
9. [噪声增益与 look-ahead 的折中](#9-噪声增益与-look-ahead-的折中)
10. [与 Nav2 RPP 参数和实现的对应](#10-与-nav2-rpp-参数和实现的对应)

---

## 1. 问题与符号

差速底盘的运动学(V0 起的 `differential_drive_step`):

$$
\dot x = v\cos\varphi,\qquad \dot y = v\sin\varphi,\qquad \dot\varphi = \omega .
$$

路径跟踪的任务:给定一条折线路径 $P$(规划器输出、经 V4 后处理),在每个控制节拍
由当前位姿 $(x, y, \varphi)$ 和速度算出指令 $(v, \omega)$,使车沿 $P$ 走到终点。
Pure Pursuit 把它写成**曲率**问题:先定线速度 $v$,再给出曲率 $\kappa$,
$\omega = v\kappa$。

| 符号 | 含义 | MiniNav 默认值 |
|---|---|---|
| $L$ | look-ahead 距离(车到"追踪点"$G$ 的距离) | $T_L\cdot v$,夹在 $[0.10, 0.60]$ m |
| $T_L$ | look-ahead 时间 $L/v$ | 1.0 s |
| $e$ | 横向误差(左正) | — |
| $\psi$ | 航向误差(车头相对路径切向) | — |
| $s$ | 沿路径走过的距离;$\sigma = s/L$ 为归一化距离 | — |
| $\tau$ | 执行器一阶滞后时间常数 | 0.10 s(`robot.yaml`,占位) |
| $T_c$ | 控制周期(零阶保持) | 0.05 s(20 Hz) |
| $r$ | 滞后比 $\tau_\text{eff}/T_L$ | 0.125 |

---

## 2. Pure Pursuit 几何

在车体系(原点在车,$x$ 朝前,$y$ 朝左)里取路径上距车 $d$ 的点
$G = (x_g, y_g)$,$x_g^2 + y_g^2 = d^2$(正常跟踪时 $d = L$;剩余路程不足 $L$
时 $G$ 取终点,$d < L$)。找一段圆弧:过原点、在原点与 $x$ 轴相切、并过 $G$。
与 $x$ 轴在原点相切的圆,圆心必在 $y$ 轴上,记为 $(0, R)$;过 $G$ 的条件

$$
x_g^2 + (y_g - R)^2 = R^2
\;\Longrightarrow\;
x_g^2 + y_g^2 = 2 y_g R
\;\Longrightarrow\;
R = \frac{d^2}{2 y_g},
$$

$$
\boxed{\;\kappa = \frac{1}{R} = \frac{2\,y_g}{d^2} = \frac{2\sin\alpha}{d}\;}
$$

其中 $\alpha$ 是 $G$ 相对车头的方位角($y_g = d\sin\alpha$)。$\kappa$ 的符号由
$y_g$ 决定:追踪点在左则左转。指令 $\omega = v\kappa$。

这是一个**纯几何**的控制律:没有增益要调,唯一的参数是 $L$。下面所有的动态性质
(阻尼、滞后裕度、切角、噪声放大)都由 $L$ 与速度、滞后的关系决定。

实现(`pure_pursuit.cpp`)里两处与教科书不同:分母用实际的 $d^2$ 而不是 $L^2$
(接近终点时 $G$ 钉在终点上,$d$ 变短);$G$ 在世界系里由"投影点向前、与车相距
$L$ 的第一个出圆点"求出(`lookahead_point`,圆-线段求交,取二次方程较大根),不对
路径重采样,稠密与稀疏路径都适用。

---

## 3. 直线附近的线性化

### 3.1 闭环方程

取路径为 $x$ 轴。车在 $(x, e)$、航向 $\psi$(此时航向误差就是航向)。追踪点在
路径上、与车相距 $L$:$G = (x + \sqrt{L^2 - e^2},\, 0)$。世界系下车到 $G$ 的向量
为 $(\sqrt{L^2 - e^2},\, -e)$,转到车体系:

$$
y_g = -\sin\psi\,\sqrt{L^2 - e^2} - e\cos\psi \;\approx\; -(e + L\psi)
\qquad (|e| \ll L,\ |\psi| \ll 1).
$$

于是 $\kappa \approx -2(e + L\psi)/L^2$。运动学 $\dot e = v\sin\psi \approx v\psi$,
$\dot\psi = \omega = v\kappa$,消去 $\psi$($\dot e = v\psi \Rightarrow \ddot e = v\dot\psi$):

$$
\ddot e + \frac{2v}{L}\,\dot e + \frac{2v^2}{L^2}\,e = 0 .
$$

与标准二阶系统 $\ddot e + 2\zeta\omega_n\dot e + \omega_n^2 e = 0$ 对照:

$$
\omega_n = \frac{\sqrt2\,v}{L},\qquad
\zeta = \frac{2v/L}{2\omega_n} = \frac{1}{\sqrt2} \approx 0.707 .
$$

**阻尼比与 $v$、$L$ 都无关。** 这就是 Pure Pursuit 不用调增益的原因:几何
本身给出了一个阻尼比固定为 $1/\sqrt2$ 的二阶系统——超调小、又不过分迟缓。

### 3.2 距离域:速度消失

把自变量换成走过的距离 $s = vt$($\mathrm{d}/\mathrm{d}t = v\,\mathrm{d}/\mathrm{d}s$):

$$
e'' + \frac{2}{L}\,e' + \frac{2}{L^2}\,e = 0 .
$$

**速度完全消失。** 纯运动学模型下,响应是一条只取决于 $s/L$ 的空间曲线——同一个
$L$,0.1 m/s 与 0.5 m/s 走出同一条轨迹,只是用时不同。特征根
$\lambda = (-1 \pm i)/L$,通解 $e = e^{-\sigma}(A\cos\sigma + B\sin\sigma)$,
$\sigma = s/L$。初值 $e(0) = e_0$、$e'(0) = \sin\psi_0 \approx \psi_0$ 给出
$A = e_0$、$B = e_0 + L\psi_0$。

### 3.3 两个解析基准

**初始偏置、与路径平行出发**($\psi_0 = 0$):

$$
e(s) = e_0\, e^{-\sigma}\,(\cos\sigma + \sin\sigma),\qquad
e'(s) = -\frac{2e_0}{L}\, e^{-\sigma}\sin\sigma .
$$

- 极值在 $\sin\sigma = 0$,第一次穿过路径之后的反向极值在 $\sigma = \pi$:
  $e(\pi L) = -e_0 e^{-\pi}$,**反向超调 $e^{-\pi} \approx 4.32\%$,出现在 $s = \pi L$**。
- $\cos\sigma + \sin\sigma = \sqrt2\sin(\sigma + \pi/4)$,包络 $\sqrt2\, e_0 e^{-\sigma}$
  落进 2% 带的条件 $\sqrt2 e^{-\sigma} \le 0.02$,即
  $\sigma \ge \ln(50\sqrt2) \approx 4.26$。包络是上界;曲线本身最后一次离开 2% 带在
  $\sigma \approx 4.22$(数值求得)。

**无偏置、带航向误差出发**($e_0 = 0$,$\psi_0 \ne 0$):

$$
e(s) = \psi_0 L\, e^{-\sigma}\sin\sigma,
$$

峰值在 $\tan\sigma = 1$,即 $\sigma = \pi/4$:$|e|_\max = \psi_0 L\,e^{-\pi/4}/\sqrt2
\approx 0.322\,\psi_0 L$。这正是出弯瞬态的尺度(§7.2):车带着航向误差离开转角,
偏移与当时的 $L$ 成正比。

**验证**(E1a,无噪声、$\tau = 0$、100 Hz、$e_0 = 5$ cm,
$L \in \{0.2, 0.3, 0.5\}$ m × $v \in \{0.1, 0.3, 0.5\}$ m/s 共 9 次):反向超调
4.33–4.35%(偏差 ≤ 0.7%)、极值位置 $3.08\text{–}3.14\,L$(≤ 1.8%)、2% 调节距离
$4.13\text{–}4.21\,L$(≤ 1.9%);9 条曲线在 $s/L$ 坐标下重合。$e_0 = 30$ cm 时
超调 4.5–5.3%(+4% 到 +24%):线性化要求 $|e| \ll L$,$e_0 > L$ 时追踪点一开始就
取在投影点上,已是另一种几何。

---

## 4. 执行器滞后与零阶保持

### 4.1 三阶特征方程与 Routh–Hurwitz

实际电机不会瞬间达到指令角速度。给角速度加一阶滞后 $\tau\dot\omega_a = \omega_c - \omega_a$
($\omega_c = v\kappa$ 是指令,$\omega_a$ 是实际值),状态 $(e, \psi, \omega_a)$:

$$
\dot e = v\psi,\qquad \dot\psi = \omega_a,\qquad
\dot\omega_a = \frac{1}{\tau}\left(-\frac{2v}{L^2}(e + L\psi) - \omega_a\right).
$$

系统矩阵的特征多项式(乘以 $\tau$):

$$
\tau s^3 + s^2 + \frac{2v}{L}\,s + \frac{2v^2}{L^2} = 0 .
$$

三阶多项式 $a_3 s^3 + a_2 s^2 + a_1 s + a_0$($a_i > 0$)的 Routh–Hurwitz 判据是
$a_2 a_1 > a_3 a_0$:

$$
1\cdot\frac{2v}{L} > \tau\cdot\frac{2v^2}{L^2}
\quad\Longleftrightarrow\quad
\boxed{\;\tau < \frac{L}{v} \equiv T_L\;}
$$

**look-ahead 时间必须大于执行器滞后。** 在边界 $\tau = T_L$ 上,特征方程有一对
纯虚根(持续振荡)。

### 4.2 归一化:只剩一个参数

令 $t = T_L\,\hat t$($T_L = L/v$),特征方程两边乘 $T_L^2$:

$$
r\,p^3 + p^2 + 2p + 2 = 0,\qquad r = \frac{\tau}{T_L}.
$$

闭环行为只由**滞后比** $r$ 决定。由于 $s/L = vt/L = t/T_L$,归一化时间也就是
归一化距离 $\sigma$。对应的状态方程(`scripts/v4/step_response.py` 的 `lag_model`
直接用它):

$$
\tilde e' = \tilde\psi,\qquad \tilde\psi' = \tilde w,\qquad
r\,\tilde w' = -2\tilde e - 2\tilde\psi - \tilde w,
$$

$\tilde e = e/L$,$\tilde w = \omega_a L/v$。$r = 0$ 时退化为 §3.2 的二阶方程。
特征分解求得的阶跃(初始偏置)响应:

| $r = \tau/T_L$ | 主导极点阻尼 $\zeta$ | 反向超调 | 2% 调节距离 |
|---|---|---|---|
| 0 | 0.707 | 4.32% | 4.22 $L$ |
| 0.1 | 0.699 | 4.50% | 3.85 $L$ |
| 0.2 | 0.605 | 6.08% | 3.45 $L$ |
| 0.3 | 0.411 | 10.97% | 4.61 $L$ |
| 0.5 | 0.200 | 24.62% | 9.95 $L$ |
| 0.7 | 0.093 | 37.61% | 22.8 $L$ |
| 0.9 | 0.026 | 48.98% | > 60 $L$ |
| 1.0 | 0 | 等幅振荡($p = \pm i\sqrt2$) | — |

$r \lesssim 0.2$ 时滞后几乎无害(超调 ≤ 6%,调节距离甚至略短);越过 0.3 后
阻尼迅速消失。**设计规则:$r \le 0.2$,即 $T_L \ge 5\,\tau_\text{eff}$。**

### 4.3 零阶保持 ≈ 半个周期的纯延迟

控制器每 $T_c$ 算一次指令,之间保持不变。零阶保持的传递函数

$$
H_\text{ZOH}(s) = \frac{1 - e^{-sT_c}}{sT_c}
= e^{-sT_c/2}\,\frac{\sinh(sT_c/2)}{sT_c/2}
\approx e^{-sT_c/2}\qquad(|sT_c| \ll 1),
$$

即约 $T_c/2$ 的纯延迟。低频下 $e^{-sT} \approx 1/(1 + sT)$,与执行器滞后串联:
$\frac{1}{(1 + \tau s)(1 + Ts)} \approx \frac{1}{1 + (\tau + T)s}$,故

$$
\tau_\text{eff} \approx \tau + \frac{T_c}{2}.
$$

默认 $\tau = 0.10$ s、20 Hz:$\tau_\text{eff} = 0.125$ s,$T_L \ge 0.625$ s。
默认 $T_L = 1.0$ s,$r = 0.125$。

**验证**(E1b,$L = 0.3$ m、$v = 0.3$ m/s,横向台阶):100 Hz 下 $\tau \in \{0, …, 0.7\}$ s
的超调与模型相差 ≤ 0.3 个百分点;20 Hz 的点按 $r_\text{eff} = (\tau + 0.025)/T_L$ 落回
同一条模型曲线($r \le 0.5$ 时差 ≤ 0.5 个百分点),说明 $T_c/2$ 折算成立。

### 4.4 速度自适应 look-ahead 的第一性原理依据

固定 $L$ 时 $r = \tau_\text{eff}\,v/L$ 随速度线性增大:同一控制器在低速稳、高速振。
令 $L = T_L\cdot v$,则

$$
r = \frac{\tau_\text{eff}}{T_L}\quad\text{与速度无关。}
$$

这就是 Nav2 RPP `use_velocity_scaled_lookahead_dist` 的依据:它不是"高速看远点"
的经验,而是让闭环在所有速度下保持同一个滞后比。低速时 $L$ 被 $L_\min$ 钳住,
等效 $T_L = L_\min/v$ 反而更大,稳定性只会更好。E1c($\tau = 0.2$ s):固定 $L = 0.3$ m 时
$v = 0.15 / 0.30 / 0.45$ m/s 的超调为 4.5 / 6.1 / 11.0%;$L = T_L v$ 时为 6.2 / 6.1 / 6.1%。

---

## 5. 圆弧上的零稳态误差

设路径是半径 $R$ 的圆弧,车在圆上且车头与圆相切。"过车、在车处与车头相切"的圆
构成一族(由曲率参数化);Pure Pursuit 从中挑出过 $G$ 的那一个。路径圆本身过车、
与车头相切,且 $G$ 就在路径圆上——所以挑出的就是路径圆,$\kappa_c = 1/R$,车留在
圆上。条件是追踪点存在:弦长 $L \le 2R$。

结论:**恒曲率路径上稳态误差为零,误差只出现在曲率变化处。** A\* 的 8 连通台阶
路径每个转角都是曲率冲激,是 Pure Pursuit 最差的输入;路径后处理(首尾替换 +
视线捷径)把每个转角都变成"少而大"的折角,§6 给出它们的代价。
(`PurePursuitAnalytic.ConstantCurvaturePathHasZeroSteadyStateError`:R = 1 m 的
720 段折线圆上误差 < 1e-4 m。)

---

## 6. 折线转角:尺度律与小角度理论

### 6.1 尺度律

纯运动学闭环在缩放 $x \to cx$、$L \to cL$、路径 $\to c\cdot$路径下不变:车体系的
$y_g \to c\,y_g$,$\kappa = 2y_g/L^2 \to \kappa/c$;以距离为自变量
$\mathrm{d}\varphi/\mathrm{d}s = \kappa$,两边同乘 $c$ 后形式不变。所以闭环轨迹也
整体缩放 $c$ 倍。

单个折线转角(两条射线,转角 $\theta$)**没有内禀长度**,唯一的长度是 $L$。所以
任何偏差都只能是

$$
\delta_\text{in} = f(\theta)\cdot L\quad\text{(转角内侧,切角)},\qquad
\delta_\text{out} = g(\theta)\cdot L\quad\text{(出弯后越到外侧)} .
$$

切角发生在转弯内侧——也就是**朝障碍物(门框、墙角)的那一侧**。

### 6.2 小角度理论:追踪点的"预览"

小斜率下曲率 $\approx y''$,航向误差 $\approx y'$。追踪点在车前方约 $L$ 处,
$y_g \approx y_\text{ref}(x + L) - y - L y'$,于是

$$
y'' + \frac{2}{L}\,y' + \frac{2}{L^2}\,y = \frac{2}{L^2}\,y_\text{ref}(x + L) .
$$

右端是**提前 $L$ 的参考**:Pure Pursuit 在转角前 $L$ 处就开始转弯。取左转
$\theta$(小角度)的折线 $y_\text{ref}(x) = \theta\max(x, 0)$,车沿 $x$ 轴来,
$x < -L$ 时一切为零。令 $\sigma = (x + L)/L$,输入是从 $\sigma = 0$ 开始的斜坡
$\theta(x + L)$;特解 $y_p = \theta x$(恰好就是出弯后的路径),齐次部分由
$y(-L) = y'(-L) = 0$ 定出:

$$
y = \theta x + \theta L\, e^{-\sigma}\cos\sigma .
$$

相对路径的偏差(左正):

- 转角前($0 \le \sigma \le 1$,参考为 $y = 0$):$e = \theta L(\sigma - 1 + e^{-\sigma}\cos\sigma)$,
  单调增($e' = \theta L\,(1 - e^{-\sigma}(\cos\sigma + \sin\sigma)) \ge 0$);
- 转角后($\sigma \ge 1$,参考为 $y = \theta x$):$e = \theta L\, e^{-\sigma}\cos\sigma$。

内侧最大偏差在车到达拐点的横坐标时($\sigma = 1$),外侧最大偏差在 $\sigma = 3\pi/4$:

$$
\boxed{\;f(\theta) \approx e^{-1}\cos(1)\,\theta \approx 0.199\,\theta,\qquad
g(\theta) \approx \frac{e^{-3\pi/4}}{\sqrt2}\,\theta \approx 0.067\,\theta\;}
\qquad(\theta\ \text{取弧度}).
$$

**验证**(E2a,经典 PP、$\tau = 0$、$L \in \{0.1, 0.2, 0.4\}$ m):$f$ 与 $L$ 无关
(三个 $L$ 相差 ≤ 4%,$L = 0.1$ m 时 100 Hz 的离散化略降低切角);小角度理论在
22.5° 时 0.078 vs 实测 0.074–0.077,45° 时 0.156 vs 0.144–0.150。大角度时饱和:

| $\theta$ | 22.5° | 45° | 67.5° | 90° | 112.5° | 135° |
|---|---|---|---|---|---|---|
| $f(\theta)$ 实测($L = 0.4$ m) | 0.077 | 0.150 | 0.215 | 0.269 | 0.290 | 0.235 |
| $0.199\,\theta$ | 0.078 | 0.156 | 0.234 | 0.312 | 0.390 | 0.468 |
| $g(\theta)$ 实测 | 0.026 | 0.054 | 0.083 | 0.116 | 0.165 | 0.303 |
| $0.067\,\theta$ | 0.026 | 0.053 | 0.079 | 0.105 | 0.132 | 0.158 |

锐角转弯(135°)时出弯外侧偏移反超切角:车在拐点附近来不及转完,冲到外侧。
(规划阶段估的"$f(135°) = 0.303$"其实是两侧偏差的最大值,即这里的 $g$。)

---

## 7. 安全裕度预算

### 7.1 离散化给出的最坏裕度

规划只保证"路径落在膨胀栅格的 free cell 里"(视线捷径的每一段经
`segment_is_free` 检查,所以线段上每个点也在 free cell 里)。free cell 的格心到
任一障碍 cell 格心的距离 $> r_\text{infl}$;路径点可偏离格心 $\frac{\sqrt2}{2}\,\text{res}$,
障碍 cell 是边长 res 的方块(边界偏离其格心至多 $\frac{\sqrt2}{2}\,\text{res}$)。
所以路径到障碍边界的净空最坏为 $r_\text{infl} - \sqrt2\,\text{res}$。车体用外接圆
$r_\text{robot}$ 近似,有效裕度

$$
m_\text{eff} = r_\text{infl} - r_\text{robot} - \sqrt2\,\text{res}
= 0.25 - 0.136 - 0.071 \approx 0.043\ \text{m}.
$$

这是**最坏情况**。E2c 量到 apartment 与 office500 上平滑路径的实际最小净空
约 10.7–11.6 cm:捷径把路径拉到可行走带的边缘,但很少同时碰上离散化的两个最坏项。

### 7.2 预算

真值车体不碰撞的充分条件(各项最坏情况相加,车偏向障碍一侧):

$$
\max\big(f(\theta)\,L_\text{corner},\; 0.322\,\psi_\text{exit}\,L_\text{cruise}\big)
+ e_\text{est} \;\le\; m_\text{eff} .
$$

第二项是 V4 PR2 发现的**出弯瞬态**(§3.3 的航向误差响应):曲率限速只看前方
$L_\kappa$ 处的几何,一过转角就解除,车带着航向误差 $\psi_\text{exit}$ 加速回巡航,
$L$ 随之变长,偏移 $\approx 0.322\,\psi_\text{exit}\,L$。

推论:

- **转角限速**:$L_\text{corner} = T_L\,v_\text{corner}$,
  $v_\text{corner} \le m_\text{eff}/(f(\theta)\,T_L)$;90° 转角、$T_L = 1$ s 时
  $\le 0.043/0.271 \approx 0.16$ m/s(巡航 0.30 m/s 必须降速)。
- **曲率限速的参数**:调节曲率取固定距离 $L_\kappa = 0.40$ m 处的点,90° 转角处
  $\sin\alpha \in [0.71, 1]$,$R = L_\kappa/(2\sin\alpha) \in [0.20, 0.28]$ m;
  $v = v_\text{des}\,R/R_\min$ 取 $R_\min = 0.60$ m 给出 0.10–0.14 m/s ≤ 0.16 m/s。
  固定 $L_\kappa$(而不是用随速度变化的 $L$)切断了"降速 → $L$ 变短 → 曲率变大 →
  再降速"的正反馈。
- **$L_\min \le m_\text{eff}/f(90°) \approx 0.16$ m**,取 0.10 m。

**验证**(E2b,完整 RPP、$\tau = 0.1$ s、20 Hz、L 形路径、无噪声):默认参数下
内侧切角 2.05 cm、出弯外侧偏移 2.82 cm、转角速度 0.094 m/s,都在 $m_\text{eff}$ 内。
$T_L$ 从 1.0 降到 0.7 s 时出弯偏移降到 1.05 cm($\propto L$),但 $T_L = 0.5$ s
($r = 0.25$)时出弯后进入"跟踪 ↔ 原地转向"的航向极限环,用时翻倍;$R_\min = 0.3$ m
时 0.6 s 就已出现。

### 7.3 第三项才是瓶颈

控制项($f\,L$、出弯瞬态)在默认参数下只吃掉 2–3 cm;$e_\text{est}$ 随路程无界
增长(§8.3)。E4 里 office500 的 34 m 路线 20 个 seed 中 19 个在 7–29 m 处碰撞,
oracle(真值进控制器)10/10 到达——这条不等式最终由定位决定。

---

## 8. 误差分解与定位漂移

### 8.1 三种误差

记路径折线为 $P$,$\operatorname{dist}(p, P) = \min_{q \in P}\|p - q\|$。

- $e_\text{true} = \operatorname{dist}(p_\text{true}, P)$:真值横向误差;
- $e_\text{ctrl} = \operatorname{dist}(\hat p, P)$:控制误差——控制器看到、也只能对它负责;
- $e_\text{est} = \|p_\text{true} - \hat p\|$:估计误差。

### 8.2 1-Lipschitz 界

对任意 $p_1, p_2$,设 $q_2^\star \in P$ 是 $p_2$ 的最近点:

$$
\operatorname{dist}(p_1, P) \le \|p_1 - q_2^\star\| \le \|p_1 - p_2\| + \|p_2 - q_2^\star\|
= \|p_1 - p_2\| + \operatorname{dist}(p_2, P).
$$

交换 $p_1, p_2$ 得 $|\operatorname{dist}(p_1, P) - \operatorname{dist}(p_2, P)| \le \|p_1 - p_2\|$,即

$$
|e_\text{true} - e_\text{ctrl}| \le e_\text{est},\qquad
e_\text{true} \le e_\text{ctrl} + e_\text{est} .
$$

同理,到达时($\|\hat p - g\| \le \text{tol}_{xy}$)真值到达误差
$\|p_\text{true} - g\| \le \text{tol}_{xy} + e_\text{est}$。`--controller-input truth`
(oracle)令 $e_\text{est} \equiv 0$,把两项用实验拆开。注意 $\operatorname{dist}$ 必须是
**全局**最近距离:控制器内部的投影只在窗口里找(防回折跳段),不满足这个不等式,
所以 `nav.csv` 的 $e_\text{ctrl}$ / $e_\text{true}$ 用全局投影另算(`distance_to_path`)。
E4 在 50 次 EKF 运行的 137 676 个样本上逐点检查:最大违反量 5e-8 m(CSV 舍入)。

### 8.3 漂移增长律:$s^{3/2}$

只有编码器与陀螺时,位置不可观测([`v2_summary.md`](../v2_summary.md) §6.4),误差随路程无界增长。主导项是**航向
随机游走**:编码器两侧滑移之差与陀螺噪声让航向误差 $\delta\theta(s)$ 像布朗运动一样
扩散,$\operatorname{Var}[\delta\theta(s)] = q\,s$;横向位置误差是航向误差的积分
$\delta y(s) = \int_0^s \delta\theta(u)\,\mathrm{d}u$,

$$
\operatorname{Var}[\delta y(s)] = \int_0^s\!\!\int_0^s q\min(u, w)\,\mathrm{d}u\,\mathrm{d}w
= \frac{q\,s^3}{3},\qquad \sigma_y \propto s^{3/2}.
$$

(若航向误差是常值偏差 $b$,则 $\delta y = b s^2/2 \propto s^2$;陀螺零偏被在线估计,
所以剩下的是随机游走项。)E4 在 office500 的 34 m 路线上(20 个 seed,不带地图跟随
同一条路径):$e_\text{est}$ 中位数 5 m 处 3.8 cm、10 m 处 10.2 cm、34 m 处 65.7 cm,
双对数斜率 1.49–1.52。

---

## 9. 噪声增益与 look-ahead 的折中

线性化控制律 $\omega = v\kappa \approx -\frac{2v}{L^2}\,e - \frac{2v}{L}\,\psi$。代入
$L = T_L v$:

$$
\frac{\partial\omega}{\partial e} = -\frac{2}{T_L^2\,v},\qquad
\frac{\partial\omega}{\partial\psi} = -\frac{2}{T_L}.
$$

估计噪声(横向、航向)进入角速度指令时被放大 $\propto 1/T_L^2$ 与 $\propto 1/T_L$。
$T_L$ 因此处在三方拉扯之中:

| 机制 | 随 $T_L$ | 来源 |
|---|---|---|
| 滞后比 $r = \tau_\text{eff}/T_L$ | 越小越稳 | §4 |
| 切角 $f(\theta)L$ 与出弯瞬态 $0.322\,\psi L$ | $\propto T_L$ | §6、§7 |
| 估计噪声 → $\omega$ 抖动 | $\propto 1/T_L^2$、$1/T_L$ | 本节 |

E3(5 场景 × 5 seed,default 噪声,EKF 进控制器,$v = 0.3$ m/s):$\omega$ 抖动
(减去 1 s 滑动平均后的 RMS)从 $T_L = 0.6$ s 的 0.139 rad/s 降到 1.0 s 的 0.061、
1.4 s 的 0.045;峰值 $e_\text{ctrl}$ 在 $T_L = 1.0$ s 最小(6.6 cm)。默认值 $T_L = 1.0$ s
保留——推导与实验给出的是同一个折中点。

---

## 10. 与 Nav2 RPP 参数和实现的对应

### 10.1 参数由哪条推导决定

| Nav2 RPP / velocity_smoother 参数 | `config/nav.yaml`(`controller` 段) | 默认 | 依据 |
|---|---|---|---|
| `controller_frequency` | `control_frequency` | 20 Hz | §4.3:$\tau_\text{eff} = \tau + T_c/2$ |
| `use_velocity_scaled_lookahead_dist` + `lookahead_time` | `use_velocity_scaled_lookahead`、`lookahead_time` | on、1.0 s | §4.2–§4.4:$r = 0.125 \le 0.2$,与速度无关;§9 噪声折中 |
| `min_lookahead_dist` / `max_lookahead_dist` | `min_lookahead` / `max_lookahead` | 0.10 / 0.60 m | §7.2:$L_\min \le m_\text{eff}/f(90°)$ |
| `lookahead_dist` | `lookahead_dist` | 0.30 m | 仅经典 PP 对照(速度自适应关闭时) |
| `use_regulated_linear_velocity_scaling` + `regulated_linear_scaling_min_radius` | `use_curvature_regulation`、`regulated_min_radius` | on、0.60 m | §7.2:转角限速 |
| `use_fixed_curvature_lookahead` + `curvature_lookahead_dist` | (总是开启)`curvature_lookahead_dist` | 0.40 m | §7.2:切断降速正反馈 |
| `approach_velocity_scaling_dist` / `min_approach_linear_velocity` | `approach_dist` / `min_approach_vel` | 0.40 m / 0.05 m/s | 终点平滑停车 |
| `use_rotate_to_heading` + `rotate_to_heading_min_angle` / `_angular_vel` | `use_rotate_to_heading`、`rotate_to_heading_angle`、`rotate_vel` | on、0.785 rad、1.0 rad/s | $\alpha$ 大时线性化失效,先原地转 |
| `max_robot_pose_search_dist` | `max_projection_search_dist` | 1.0 m | 进度投影窗口,回折路径不跳段 |
| velocity_smoother `max_accel` | `max_accel` / `max_angular_accel` | 0.5 m/s² / 3.0 rad/s² | 同时进入 EKF 的 $q_{dv} = (a_\max\,dt)^2$、$q_{d\omega}$(恒速模型看不到的加减速) |
| `use_cost_regulated_linear_velocity_scaling`、`use_collision_detection` | — | 未实现 | V4 改为测量真值净空(§7) |

### 10.2 代码与测试

| 概念 | 代码 | 守护它的测试 |
|---|---|---|
| 曲率几何 $\kappa = 2y_g/d^2$(车体系) | `pure_pursuit.cpp`:`compute_velocity_commands` | `PurePursuit.CurvatureMatchesHandCalculation`、`PurePursuit.CurvatureIsComputedInRobotFrame` |
| 追踪点(圆-线段求交)、单调进度 | `path.cpp`:`project_onto` / `lookahead_point` | `PathLookahead.ResultIsExactlyLFromRobot`、`PathProjection.WindowKeepsProgressOnHairpinOutboundLeg`、`PurePursuit.ProgressStaysOnOutboundLegOfHairpin` |
| $e^{-\pi}$、$\ln(50\sqrt2)$、距离域与速度无关 | 闭环(解析用例) | `PurePursuitAnalytic.ReverseOvershootIsExpMinusPiAtPiL`、`…SettlingDistanceIsLn50Sqrt2TimesL`、`…DistanceDomainResponseIsIndependentOfSpeed` |
| 速度自适应 $L = \operatorname{clamp}(T_L v)$ | `lookahead_distance` | `PurePursuit.LookaheadScalesWithSpeedWithinBounds` |
| 一阶滞后(精确离散化)、饱和 | `plant.cpp`:`apply_dynamics` | `PlantDynamics.FirstOrderLagReaches63PercentAtTau`、`PlantDynamics.ExactDiscretizationNeverOvershoots` |
| 圆弧零稳态误差、切角尺度律、转角预算 | 闭环(解析用例) | `PurePursuitAnalytic.ConstantCurvaturePathHasZeroSteadyStateError`、`…CornerCuttingScalesWithLookahead`、`…RegulatedPursuitTakesCornerWithinInsideMarginBudget` |
| 曲率限速、接近减速、$\omega$ 上限 | `regulated_velocity` | `PurePursuit.CurvatureRegulationSlowsDownBeforeCorner`、`…ApproachSlowsDownNearGoalWithFloor`、`…AngularVelocityLimitReducesLinearSpeedKeepingCurvature` |
| 平滑器(两分量同比例缩放,保曲率) | `velocity_smoother.cpp` | `VelocitySmoother.ScalesBothComponentsTogetherPreservingCurvatureFromRest` |
| 误差分解(全局投影)、碰撞(原始地图)、确定性 | `nav_mode.cpp`:`distance_to_path` / `footprint_clearance` | `nav.*` 集成测试、golden `nav_apartment_s2_seed42.csv` |

---

## 参考文献

1. R. C. Coulter. *Implementation of the Pure Pursuit Path Tracking Algorithm.*
   CMU-RI-TR-92-01, Carnegie Mellon University, 1992.
2. S. Macenski, S. Singh, F. Martín, J. Ginés. *Regulated Pure Pursuit for Robot Path
   Tracking.* Autonomous Robots 47, 2023.(Nav2 RPP 的设计与参数)
3. J. M. Snider. *Automatic Steering Methods for Autonomous Automobile Path Tracking.*
   CMU-RI-TR-09-08, 2009.(Pure Pursuit 的线性化与稳定性)
4. K. J. Åström, R. M. Murray. *Feedback Systems: An Introduction for Scientists and
   Engineers*, 2nd ed., Princeton University Press, 2021.(二阶系统、Routh–Hurwitz、延迟近似)
5. G. F. Franklin, J. D. Powell, M. Workman. *Digital Control of Dynamic Systems*,
   3rd ed., 1998.(零阶保持的等效延迟)
6. Y. Bar-Shalom, X. R. Li, T. Kirubarajan. *Estimation with Applications to Tracking
   and Navigation*, Wiley, 2001.(NIS / NEES 一致性检验)
7. S. Thrun, W. Burgard, D. Fox. *Probabilistic Robotics*, MIT Press, 2005.(航位推算的
   误差增长)
8. ROS 2 Navigation:`nav2_regulated_pure_pursuit_controller`、`nav2_velocity_smoother`、
   `nav2_core::Controller` 接口。
