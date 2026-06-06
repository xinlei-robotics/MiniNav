# 差分驱动运动的 Runge-Kutta 积分

## 1. 问题陈述

本项目中用如下状态表示二维平面上的机器人位姿:

$$
\mathbf{x} =
\begin{bmatrix}
x \\
y \\
\theta
\end{bmatrix},
$$

控制输入是车体坐标系下的 twist:

$$
\mathbf{u} =
\begin{bmatrix}
v \\
\omega
\end{bmatrix}.
$$

对无侧向速度的差分驱动机器人,连续时间运动学模型为:

$$
\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u})
=
\begin{bmatrix}
v \cos\theta \\
v \sin\theta \\
\omega
\end{bmatrix}.
$$

仿真器需要在每个离散时间步内计算:

$$
\mathbf{x}_{k+1} \approx \mathbf{x}(t_k + \Delta t),
$$

其中输入是当前状态、一个在该时间步内近似保持不变的控制量,以及时间步长
`dt`。

早期 MiniNav 版本使用显式欧拉积分:

$$
\begin{aligned}
\mathbf{x}_{k+1}
&= \mathbf{x}_k + \Delta t f(\mathbf{x}_k, \mathbf{u}_k).
\end{aligned}
$$

欧拉积分的优点是简单,但它只在时间区间起点评估一次速度场。圆周运动时,
机器人的 heading 在一个时间步内部持续变化,所以"起点 heading"并不是该时间步内
平均运动方向的好近似。很多小误差累积后,一圈圆周运动结束时就会看到明显的闭合误差。

## 2. Runge-Kutta 的基本思想

常微分方程的精确解可以写成积分形式:

$$\begin{aligned}\mathbf{x}(t_k + \Delta t)&= \mathbf{x}(t_k)+ \int_{t_k}^{t_k+\Delta t} f(\mathbf{x}(t), \mathbf{u}_k) \, dt. \end{aligned}$$

Runge-Kutta 方法的核心思想是:不要只用区间起点的斜率近似整个积分,而是在
区间内部采样多个斜率,再用加权平均近似这个积分。

经典四阶 Runge-Kutta,也就是 RK4,使用四个斜率:

$$
\begin{aligned}
\mathbf{k}_1 &= f(\mathbf{x}_k, \mathbf{u}_k), \\
\mathbf{k}_2 &= f(\mathbf{x}_k + \frac{\Delta t}{2}\mathbf{k}_1, \mathbf{u}_k), \\
\mathbf{k}_3 &= f(\mathbf{x}_k + \frac{\Delta t}{2}\mathbf{k}_2, \mathbf{u}_k), \\
\mathbf{k}_4 &= f(\mathbf{x}_k + \Delta t\mathbf{k}_3, \mathbf{u}_k).
\end{aligned}
$$

然后用如下加权平均更新状态:

$$\mathbf{x}_{k+1}=\mathbf{x}_k+\frac{\Delta t}{6}\left(\mathbf{k}_1 + 2\mathbf{k}_2 + 2\mathbf{k}_3 + \mathbf{k}_4 \right).$$

这里的权重 `1, 2, 2, 1` 不是经验公式。它来自 Taylor 展开匹配:RK4 的数值
更新在 $dt^4$ 阶以内与真实 ODE 解一致。因此:

- 欧拉积分的局部截断误差是 $O(dt^2)$,全局误差是 $O(dt)$。
- RK4 的局部截断误差是 $O(dt^5)$,全局误差是 $O(dt^4)$。

这就是 RK4 可以在不减小仿真步长的情况下显著降低长期积分误差的原因。

## 3. 应用到差分驱动运动学

对 MiniNav 的差分驱动模型,每个 RK4 斜率都是一个 pose derivative:

$$
\mathbf{k}_i =
\begin{bmatrix}
v \cos\theta_i \\
v \sin\theta_i \\
\omega
\end{bmatrix}.
$$

在一个仿真时间步内,控制量近似为常数,所以 `v` 和 `omega` 在 `k1`、`k2`、
`k3`、`k4` 中不变。真正变化的是中间状态里的 heading。

例如:

$$
\theta_2 = \theta_k + \frac{\Delta t}{2} k_{1,\theta}.
$$

因此 `k2` 会在时间区间中点附近评估平移速度方向,而不是继续使用起点方向。
这正是 RK4 在曲线运动中比欧拉积分更准确的关键。

最终 yaw 在 RK4 更新完成后做一次 wrap:

$$
\theta_{k+1} \leftarrow \mathrm{wrapToPi}(\theta_{k+1}).
$$

中间 heading 不需要 wrap,因为 `sin` 和 `cos` 本身具有周期性。只 wrap 最终
存储的 pose,实现更简单,也能保持 MiniNav 一贯的 yaw 表示约定。

## 4. 为什么不用闭式圆弧公式

当 twist 在一个时间步内严格为常数时,差分驱动运动确实存在闭式解:

$$
\begin{aligned}
x_{k+1} &= x_k + \frac{v}{\omega}
\left(\sin(\theta_k + \omega\Delta t) - \sin\theta_k\right), \\
y_{k+1} &= y_k - \frac{v}{\omega}
\left(\cos(\theta_k + \omega\Delta t) - \cos\theta_k\right), \\
\theta_{k+1} &= \theta_k + \omega\Delta t.
\end{aligned}
$$

这个公式在 `omega` 非零且常数时是精确的。但 MiniNav 选择 RK4 作为积分器,
原因是 RK4 更适合作为通用 ODE 积分接口:

- 未来加入闭环控制器后,控制可能在更细粒度上变化。
- 未来过程模型可能不再只是简单的差分驱动运动学。
- RK4 不需要在 `omega = 0` 附近写特殊分支;闭式圆弧公式需要单独处理直线极限。

因此,闭式公式更像是"当前简单模型下的特解",而 RK4 更适合作为项目长期演进的
数值积分基础设施。

## 5. 预期效果

最直观的回归测试是圆周运动。

在欧拉积分下,机器人实际走的是圆的多边形近似,完整转一圈后可能离起点还有数厘米
误差。切换到 RK4 后,在当前 MiniNav 时间步长下,纯积分误差已经小于 encoder tick
量化误差。

这并不意味着 odometry drift 被消除了。RK4 解决的是确定性的数值积分误差;
执行器噪声、encoder 量化、slip、IMU 噪声和 gyro bias 仍然需要 EKF 这样的概率
状态估计方法处理。
